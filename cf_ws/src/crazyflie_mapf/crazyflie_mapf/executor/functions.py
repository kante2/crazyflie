"""ROS-free validation, interpolation and stopping for the fleet executor.

Trajectory times are relative to the shared batch ``issued_at``. Every interval
uses constant acceleration, the same double-integrator model as the planner.
An expired or rejected batch stops the entire fleet with bounded deceleration;
the stop reference is integrated once rather than reset at every timer tick.
"""

from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
import math


def scalar(value, name="number"):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("a three-dimensional vector is required")
    return tuple(scalar(component, "vector component") for component in value)


def norm(value):
    return math.sqrt(sum(component * component for component in value))


@dataclass(frozen=True)
class FullStateSetpoint:
    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: float = 0.0
    omega: tuple = (0.0, 0.0, 0.0)
    mode: str = "TRAJECTORY"


@dataclass(frozen=True)
class Trajectory:
    goal_id: str
    times: tuple
    positions: tuple
    velocities: tuple
    accelerations: tuple
    yaw: float

    @classmethod
    def parse(cls, data, speed, acceleration, braking=False):
        if not isinstance(data, dict) or not isinstance(data.get("goal_id"), str):
            raise ValueError("trajectory requires its goal identity")
        times = tuple(scalar(t, "trajectory time") for t in data["times"])
        if len(times) < 2 or abs(times[0]) > 1e-9 or any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("trajectory times must increase from zero")
        positions = tuple(vector(p) for p in data["positions"])
        velocities = tuple(vector(v) for v in data["velocities"])
        accelerations = tuple(vector(a) for a in data["accelerations"])
        if any(len(values) != len(times) for values in (positions, velocities, accelerations)):
            raise ValueError("all trajectory knots must have matching lengths")
        # Speed and acceleration are Euclidean limits. A braking rollout may
        # begin above the limit after a measured disturbance, but never increase
        # that initial speed. Constant acceleration has a convex velocity norm,
        # so checking interval endpoints also bounds the complete interval.
        speeds = tuple(norm(v) for v in velocities)
        speed_limit = max(speed, speeds[0]) if braking else speed
        if any(value > speed_limit + 1e-3 for value in speeds):
            raise ValueError("trajectory exceeds the configured speed bound")
        if braking and any(second > first + 1e-3 for first, second in zip(speeds, speeds[1:])):
            raise ValueError("braking trajectory must not increase its measured initial speed")
        if any(norm(a) > acceleration + 1e-3 for a in accelerations):
            raise ValueError("trajectory exceeds the configured acceleration bound")
        for k, (start, end) in enumerate(zip(times, times[1:])):
            dt = end - start
            expected_p = tuple(p + v * dt + 0.5 * a * dt * dt for p, v, a in zip(positions[k], velocities[k], accelerations[k]))
            expected_v = tuple(v + a * dt for v, a in zip(velocities[k], accelerations[k]))
            if max(abs(a - b) for a, b in zip(expected_p, positions[k + 1])) > 2e-3 or max(abs(a - b) for a, b in zip(expected_v, velocities[k + 1])) > 2e-3:
                raise ValueError("trajectory knots violate double-integrator dynamics")
        return cls(data["goal_id"], times, positions, velocities, accelerations, scalar(data.get("yaw", 0), "yaw"))

    def sample(self, elapsed):
        if elapsed < -1e-6 or elapsed > self.times[-1] + 1e-6:
            raise ValueError("trajectory sample is outside its time window")
        if elapsed >= self.times[-1]:
            return FullStateSetpoint(self.positions[-1], self.velocities[-1], self.accelerations[-1], self.yaw)
        k = max(0, bisect_right(self.times, max(0.0, elapsed)) - 1)
        dt = max(0.0, elapsed - self.times[k])
        position = tuple(p + v * dt + 0.5 * a * dt * dt for p, v, a in zip(self.positions[k], self.velocities[k], self.accelerations[k]))
        velocity = tuple(v + a * dt for v, a in zip(self.velocities[k], self.accelerations[k]))
        return FullStateSetpoint(position, velocity, self.accelerations[k], self.yaw)


class StateEstimator:
    """Estimate world velocity from strictly increasing stamped position samples."""

    def __init__(self, names, frame="world", window=0.08):
        self.names = tuple(names)
        self.frame = frame
        self.window = window
        self.history = {name: deque() for name in self.names}
        self.states = {}

    def update(self, name, position, stamp, received_at, received_wall=None):
        if name not in self.history:
            return False
        p, stamp, received_at = vector(position), scalar(stamp), scalar(received_at)
        history = self.history[name]
        if history and stamp <= history[-1][0]:
            return False
        # Large gaps should not turn an old displacement into a current velocity.
        if history and stamp - history[-1][0] > 0.5:
            history.clear()
        history.append((stamp, p))
        while len(history) > 2 and stamp - history[1][0] >= self.window:
            history.popleft()
        if len(history) < 2:
            v = (0.0, 0.0, 0.0)
        else:
            elapsed = stamp - history[0][0]
            v = tuple((b - a) / elapsed for a, b in zip(history[0][1], p))
        state = dict(position=p, velocity=v, stamp=stamp, received_at=received_at, frame=self.frame)
        if received_wall is not None:
            state["received_wall"] = scalar(received_wall)
        self.states[name] = state
        return True


class FleetExecutor:
    """Accept a complete fleet batch and sample all robots at one shared time."""

    def __init__(self, names, settings):
        self.names = tuple(sorted(names))
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("a nonempty set of robot names is required")
        self.settings = settings
        self.frame = settings.get("frame", "world")
        self.speed = float(settings.get("speed", 2.0))
        self.acceleration = float(settings.get("acceleration", 0.6))
        self.deceleration = float(settings.get("deceleration", 0.8))
        self.ttl = float(settings.get("command_ttl", 0.4))
        self.state_timeout = float(settings.get("state_timeout", 0.5))
        if any(not math.isfinite(value) or value <= 0 for value in (self.speed, self.acceleration, self.deceleration, self.ttl, self.state_timeout)):
            raise ValueError("executor limits and timeouts must be positive and finite")
        self.goal_revision = -1
        self.goals = {}
        self.sequence = -1
        self.issued_at = None
        self.valid_until = None
        self.trajectories = None
        self.last_error = None
        self.last_setpoints = {}
        self._stop = None
        self._stop_started = None
        self._last_now = None

    def set_goals(self, data, now):
        self._envelope(data)
        revision = data.get("revision")
        if type(revision) is not int or revision < 0:
            raise ValueError("goal revision must be a nonnegative integer")
        if revision <= self.goal_revision:
            return False
        stamp = scalar(data["stamp"], "goal stamp")
        if stamp > now + 0.05:
            raise ValueError("goal stamp is in the future")
        robots = data["robots"]
        if not isinstance(robots, dict) or set(robots) != set(self.names):
            raise ValueError("goals must name the complete enabled fleet")
        goals = {}
        for name in self.names:
            entry = robots[name]
            if not isinstance(entry, dict) or not isinstance(entry.get("goal_id"), str) or not entry["goal_id"]:
                raise ValueError("goal identity must be a nonempty string")
            goals[name] = dict(goal_id=entry["goal_id"], position=vector(entry["position"]))
        self.goals = goals
        self.goal_revision = revision
        self.invalidate("GOAL_CHANGED")
        return True

    def _envelope(self, data):
        if not isinstance(data, dict) or type(data.get("version")) is not int or data.get("version") != 1 or data.get("frame") != self.frame:
            raise ValueError("message version and world frame must match")

    def invalidate(self, reason):
        self.trajectories = None
        self.last_error = str(reason)

    def accept(self, data, now):
        """Validate before atomically replacing the active fleet trajectory.

        Delayed duplicates cannot replace a newer batch. Other malformed batches
        invalidate the active batch so a partial fleet can never keep moving.
        """
        try:
            self._envelope(data)
            sequence = data.get("sequence")
            if type(sequence) is not int or sequence < 0:
                raise ValueError("trajectory sequence must be a nonnegative integer")
            if sequence <= self.sequence:
                return False
            if data.get("status") not in ("SOLVED", "BRAKING"):
                raise ValueError("planner did not publish a validated trajectory")
            issued = scalar(data["issued_at"], "issued_at")
            valid_until = scalar(data["valid_until"], "valid_until")
            if issued > now + 0.02 or not issued <= now < valid_until or valid_until - issued > self.ttl + 1e-6:
                raise ValueError("trajectory has an invalid validity window")
            if self.issued_at is not None and issued < self.issued_at:
                raise ValueError("trajectory time moved backwards")
            robots = data["trajectories"]
            if not isinstance(robots, dict) or set(robots) != set(self.names) or set(self.goals) != set(self.names):
                raise ValueError("trajectory must include the complete assigned fleet")
            braking = data["status"] == "BRAKING"
            limit = self.deceleration if braking else self.acceleration
            trajectories = {name: Trajectory.parse(robots[name], self.speed, limit, braking=braking) for name in self.names}
            shared_times = trajectories[self.names[0]].times
            bounds = self.settings.get("bounds")
            for name, trajectory in trajectories.items():
                if trajectory.goal_id != self.goals[name]["goal_id"]:
                    raise ValueError("trajectory belongs to an old goal")
                if trajectory.times != shared_times:
                    raise ValueError("fleet trajectories must share the same time knots")
                if valid_until - issued > trajectory.times[-1] + 1e-6:
                    raise ValueError("trajectory does not cover its validity window")
                if bounds is not None:
                    low, high = map(vector, bounds)
                    if any(any(value < minimum - 1e-3 or value > maximum + 1e-3 for value, minimum, maximum in zip(position, low, high)) for position in trajectory.positions):
                        raise ValueError("trajectory leaves the configured flight bounds")
                    for k, (start, end) in enumerate(zip(trajectory.times, trajectory.times[1:])):
                        for axis, acceleration in enumerate(trajectory.accelerations[k]):
                            if abs(acceleration) < 1e-9:
                                continue
                            extremum = -trajectory.velocities[k][axis] / acceleration
                            if 0 < extremum < end - start:
                                value = trajectory.positions[k][axis] + trajectory.velocities[k][axis] * extremum + 0.5 * acceleration * extremum**2
                                if value < low[axis] - 1e-3 or value > high[axis] + 1e-3:
                                    raise ValueError("trajectory crosses flight bounds between time knots")
            self.sequence, self.issued_at, self.valid_until = sequence, issued, valid_until
            self.trajectories = trajectories
            self._stop = self._stop_started = None
            self.last_error = None
            return True
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.invalidate(exc)
            return False

    def _fresh(self, state, now):
        try:
            return state.get("frame", self.frame) == self.frame and 0 <= now - scalar(state["stamp"]) <= self.state_timeout
        except (TypeError, KeyError, ValueError):
            return False

    def sample(self, now, states):
        now = scalar(now, "sample time")
        if self._last_now is not None and now < self._last_now:
            self.invalidate("CLOCK_REWIND")
            self._stop = self._stop_started = None
        self._last_now = now
        if self.trajectories is not None:
            if not all(name in states and self._fresh(states[name], now) for name in self.names):
                self.invalidate("STALE_STATE")
            elif not self.issued_at <= now < self.valid_until:
                self.invalidate("STALE_TRAJECTORY")
            else:
                result = {name: trajectory.sample(now - self.issued_at) for name, trajectory in self.trajectories.items()}
                self.last_setpoints = result
                return result
        if self._stop is None:
            initial = {}
            for name in self.names:
                state = states.get(name)
                if state is not None and self._fresh(state, now):
                    p, v = vector(state["position"]), vector(state["velocity"])
                    previous = self.last_setpoints.get(name)
                    yaw = previous.yaw if previous else 0.0
                elif name in self.last_setpoints:
                    previous = self.last_setpoints[name]
                    p, v, yaw = previous.position, previous.velocity, previous.yaw
                elif state is not None:
                    p, v, yaw = vector(state["position"]), vector(state["velocity"]), 0.0
                else:
                    raise ValueError(f"no measured state or reference for {name}")
                initial[name] = (p, v, yaw)
            self._stop, self._stop_started = initial, now
        elapsed = max(0.0, now - self._stop_started)
        result = {}
        for name, (p, v, yaw) in self._stop.items():
            speed = norm(v)
            stop_time = speed / self.deceleration
            braking_time = min(elapsed, stop_time)
            a = tuple(-self.deceleration * value / speed for value in v) if speed > 1e-9 else (0.0, 0.0, 0.0)
            position = tuple(x + speed_component * braking_time + 0.5 * acceleration * braking_time * braking_time for x, speed_component, acceleration in zip(p, v, a))
            velocity = tuple(value + acceleration * braking_time for value, acceleration in zip(v, a))
            stopped = elapsed >= stop_time
            result[name] = FullStateSetpoint(position, (0.0, 0.0, 0.0) if stopped else velocity, (0.0, 0.0, 0.0) if stopped else a, yaw, mode="HOVER" if stopped else "BRAKING")
        self.last_setpoints = result
        return result

    def arrived(self, name, state):
        goal = self.goals.get(name)
        if goal is None:
            return False
        distance = norm(tuple(a - b for a, b in zip(vector(state["position"]), goal["position"])))
        return distance <= self.settings.get("arrival_radius", 0.1) and norm(vector(state["velocity"])) <= self.settings.get("arrival_speed", 0.15)
