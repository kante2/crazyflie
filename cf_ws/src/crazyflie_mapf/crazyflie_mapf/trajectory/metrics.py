"""Measured fleet metrics independent of the planner's safety validator.

Tracking uses the last issued executable batch at the measurement stamp. The
batch supplied to observe is installed *after* measurements are evaluated, so a
fresh plan cannot hide the tracking error that triggered it. Pair minima here
are measured-state metrics; the planner's continuous forecast validation is a
separate diagnostic.
"""

import bisect
import math
from itertools import combinations


def _vector(value):
    result = tuple(float(component) for component in value)
    if len(result) != 3 or not all(math.isfinite(component) for component in result):
        raise ValueError("a finite 3D vector is required")
    return result


class FleetMetrics:
    def __init__(self, config):
        self.config = config
        self.horizontal = float(config["horizontal_separation"])
        self.vertical = float(config["vertical_separation"])
        self.timeout = float(config.get("state_timeout", .5))
        if not all(math.isfinite(value) and value > 0 for value in
                   (self.horizontal, self.vertical, self.timeout)):
            raise ValueError("positive finite separation and state timeout are required")
        self.frame = config.get("frame", "world")
        self.clock_resets = 0
        self._clear()

    def _clear(self):
        self.last_now = None
        self.min_distance = self.min_scaled_distance = None
        self.position_square_sum = self.velocity_square_sum = 0.0
        self.tracking_count = 0
        self.tracking_max = self.velocity_max = None
        self.last_state_stamp = {}
        self.last_tracking_stamp = {}
        self.reference = {}
        self.reference_stamp = None
        self.last_batch_key = None
        self.solve_times = []
        self.solve_sum = 0.0
        self.solve_stats = dict(latest=None, mean=None, p95=None, max=None, count=0)

    def reset(self):
        """Start a new measurement epoch, e.g. after a simulation clock reset."""
        self.clock_resets += 1
        self._clear()

    def _states(self, states, now):
        valid, ignored = {}, []
        for robot, state in states.items():
            try:
                position = _vector(state["position"])
                velocity = _vector(state["velocity"])
                stamp = float(state["stamp"])
                radius = float(state.get("radius", 0.0))
                if (not math.isfinite(stamp) or not math.isfinite(radius) or radius < 0
                        or stamp > now + .05 or now - stamp > self.timeout
                        or stamp < self.last_state_stamp.get(robot, -math.inf)
                        or state.get("frame", self.frame) != self.frame):
                    raise ValueError("invalid or stale state")
            except (KeyError, TypeError, ValueError, OverflowError):
                ignored.append(robot)
                continue
            valid[robot] = dict(position=position, velocity=velocity, stamp=stamp,
                                radius=radius, goal_id=state.get("goal_id"))
            self.last_state_stamp[robot] = stamp
        return valid, ignored

    def _sample_reference(self, robot, state):
        trajectory = self.reference.get(robot)
        if trajectory is None or state["stamp"] < self.reference_stamp:
            return None
        if (state["goal_id"] is not None and trajectory["goal_id"] is not None
                and state["goal_id"] != trajectory["goal_id"]):
            return None
        elapsed = state["stamp"] - self.reference_stamp
        times = trajectory["times"]
        index = min(len(times) - 1, max(0, bisect.bisect_right(times, elapsed) - 1))
        duration = min(elapsed - times[index], times[index + 1] - times[index]) if index + 1 < len(times) else 0.0
        position, velocity, acceleration = (trajectory[key][index] for key in
                                            ("positions", "velocities", "accelerations"))
        return (
            tuple(p + duration * v + .5 * duration * duration * a
                  for p, v, a in zip(position, velocity, acceleration)),
            tuple(v + duration * a for v, a in zip(velocity, acceleration)),
        )

    def _record_batch(self, batch):
        if not isinstance(batch, dict):
            return
        if batch.get("status") == "WAITING":
            return
        try:
            stamp = float(batch["stamp"])
            if not math.isfinite(stamp):
                return
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        key = (stamp, batch.get("sequence"))
        if key == self.last_batch_key:
            return
        if self.last_batch_key is not None and stamp < self.last_batch_key[0]:
            return
        self.last_batch_key = key
        solve_ms = batch.get("diagnostics", {}).get("solve_ms")
        if isinstance(solve_ms, (int, float)) and math.isfinite(solve_ms) and solve_ms >= 0:
            solve_ms = float(solve_ms)
            bisect.insort(self.solve_times, solve_ms)
            self.solve_sum += solve_ms
            quantile_index = .95 * (len(self.solve_times) - 1)
            lower = math.floor(quantile_index)
            upper = math.ceil(quantile_index)
            p95 = self.solve_times[lower] + (quantile_index - lower) * (self.solve_times[upper] - self.solve_times[lower])
            self.solve_stats = dict(latest=solve_ms, mean=self.solve_sum / len(self.solve_times),
                                    p95=p95, max=self.solve_times[-1], count=len(self.solve_times))
        if batch.get("status") not in ("SOLVED", "BRAKING") or batch.get("frame", self.frame) != self.frame:
            return
        references = {}
        for robot, trajectory in batch.get("trajectories", {}).items():
            try:
                times = tuple(float(time) for time in trajectory["times"])
                if (not times or times[0] != 0.0 or not all(math.isfinite(time) for time in times)
                        or any(second <= first for first, second in zip(times, times[1:]))):
                    raise ValueError("invalid trajectory times")
                parsed = {key: tuple(_vector(vector) for vector in trajectory[key])
                          for key in ("positions", "velocities", "accelerations")}
                if any(len(vectors) != len(times) for vectors in parsed.values()):
                    raise ValueError("trajectory vectors and times must have matching lengths")
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            references[robot] = dict(parsed, times=times, goal_id=trajectory.get("goal_id"))
        if references:
            self.reference, self.reference_stamp = references, stamp

    def observe(self, states, batch=None, now=None):
        """Return a JSON-safe measured snapshot, accepting a new batch afterwards."""
        if now is None:
            stamps = [state.get("stamp") for state in states.values()
                      if isinstance(state, dict) and isinstance(state.get("stamp"), (int, float))
                      and math.isfinite(state["stamp"])]
            now = max(stamps) if stamps else self.last_now if self.last_now is not None else 0.0
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("a finite observation stamp is required")
        if self.last_now is not None and now < self.last_now - 1e-9:
            self.reset()
        self.last_now = now
        valid, ignored = self._states(states, now)
        distances, scaled = [], []
        for first, second in combinations(valid.values(), 2):
            relative = tuple(a - b for a, b in zip(first["position"], second["position"]))
            distances.append(math.sqrt(sum(value * value for value in relative)))
            horizontal = max(self.horizontal, first["radius"] + second["radius"])
            scaled.append(math.sqrt(sum((value / axis) ** 2 for value, axis in
                                        zip(relative, (horizontal, horizontal, self.vertical)))))
        current_distance = min(distances) if distances else None
        current_scaled = min(scaled) if scaled else None
        if current_distance is not None:
            self.min_distance = current_distance if self.min_distance is None else min(self.min_distance, current_distance)
            self.min_scaled_distance = current_scaled if self.min_scaled_distance is None else min(self.min_scaled_distance, current_scaled)
        position_errors, velocity_errors = {}, {}
        for robot, state in valid.items():
            reference = self._sample_reference(robot, state)
            if reference is None:
                continue
            position_error = math.dist(state["position"], reference[0])
            velocity_error = math.dist(state["velocity"], reference[1])
            position_errors[robot], velocity_errors[robot] = position_error, velocity_error
            if state["stamp"] <= self.last_tracking_stamp.get(robot, -math.inf):
                continue
            self.last_tracking_stamp[robot] = state["stamp"]
            self.tracking_count += 1
            self.position_square_sum += position_error ** 2
            self.velocity_square_sum += velocity_error ** 2
            self.tracking_max = position_error if self.tracking_max is None else max(self.tracking_max, position_error)
            self.velocity_max = velocity_error if self.velocity_max is None else max(self.velocity_max, velocity_error)
        self._record_batch(batch)
        return dict(
            stamp=now, measured_robots=len(valid), ignored_states=ignored, clock_resets=self.clock_resets,
            current_min_distance_m=current_distance, current_min_scaled_distance=current_scaled,
            min_distance_m=self.min_distance, min_scaled_distance=self.min_scaled_distance,
            tracking_error_m=position_errors, tracking_velocity_error_mps=velocity_errors,
            tracking_max_m=self.tracking_max,
            tracking_rms_m=math.sqrt(self.position_square_sum / self.tracking_count) if self.tracking_count else None,
            tracking_velocity_max_mps=self.velocity_max,
            tracking_velocity_rms_mps=math.sqrt(self.velocity_square_sum / self.tracking_count) if self.tracking_count else None,
            tracking_samples=self.tracking_count, solve_ms=dict(self.solve_stats),
        )
