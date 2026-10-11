"""Fleet ROS adapter: one timer streams all Crazyflies in the same tick."""

import json
import time

from .functions import FleetExecutor, StateEstimator, norm


class DroneExecutorNode:
    """Use the existing Crazyswarm node instead of creating one node per robot."""

    def __init__(self, stack, homes, settings):
        from geometry_msgs.msg import PoseStamped
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.stack = stack
        self.node = stack.allcfs
        self.homes = homes
        self.settings = settings
        self.names = tuple(sorted(homes))
        if set(stack.names()) != set(self.names):
            raise ValueError("enabled configuration and connected Crazyflies differ")
        self.handles = {name: stack.get(name) for name in self.names}
        self.executor = FleetExecutor(self.names, settings)
        self.estimator = StateEstimator(self.names, settings["frame"])
        self.phase = "WAITING_POSE"
        self.phase_started = self.now()
        self.stable_since = None
        self.land_requested = False
        self.last_state_publish = None
        self._last_warning = 0.0
        self._String = String
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.node.create_publisher(String, "/fleet/states", qos_profile_sensor_data)
        self.subscriptions = [
            self.node.create_subscription(TFMessage, "/tf", self._tf, qos_profile_sensor_data),
            self.node.create_subscription(String, "/fleet/goals", self._goals, latched),
            self.node.create_subscription(String, "/fleet/trajectories", self._trajectories, 1),
            self.node.create_subscription(String, "/fleet/control", self._control, 10),
        ]
        for name in self.names:
            self.subscriptions.append(self.node.create_subscription(PoseStamped, f"/{name}/pose", lambda msg, robot=name: self._pose(robot, msg), qos_profile_sensor_data))
        self.timer = self.node.create_timer(1.0 / settings["stream_rate"], self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.node.get_logger().info(f"fleet executor: {', '.join(self.names)}, {settings['stream_rate']:g} Hz common loop")

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def _record(self, name, position, stamp):
        try:
            now = self.now()
            if stamp > now + 0.05:
                return
            self.estimator.update(name, position, stamp, now, time.monotonic())
        except (TypeError, ValueError):
            return

    def _tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id.strip("/") != self.settings["frame"]:
                continue
            name = transform.child_frame_id.strip("/")
            p = transform.transform.translation
            self._record(name, (p.x, p.y, p.z), self._stamp(transform.header.stamp))

    def _pose(self, name, message):
        if message.header.frame_id.strip("/") != self.settings["frame"]:
            return
        p = message.pose.position
        self._record(name, (p.x, p.y, p.z), self._stamp(message.header.stamp))

    def _warning(self, message):
        now = time.monotonic()
        if now - self._last_warning >= 2.0:
            self.node.get_logger().warning(message)
            self._last_warning = now

    def _goals(self, message):
        try:
            self.executor.set_goals(json.loads(message.data), self.now())
        except (ValueError, KeyError, TypeError) as exc:
            self._warning(f"ignored fleet goal batch: {exc}")

    def _trajectories(self, message):
        if self.land_requested:
            self.executor.invalidate("LAND_REQUESTED")
            return
        if self.phase != "AIRBORNE":
            return
        try:
            data = json.loads(message.data)
            accepted = self.executor.accept(data, self.now())
            if not accepted and self.executor.last_error:
                self._warning(f"fleet braking: {self.executor.last_error}")
        except (ValueError, TypeError) as exc:
            self.executor.invalidate(f"MALFORMED_JSON: {exc}")
            self._warning(f"fleet braking: invalid trajectory JSON: {exc}")

    def _control(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict) and data.get("op") == "land":
                self.land_requested = True
        except (ValueError, TypeError):
            pass

    def _phase(self, phase, now):
        self.phase, self.phase_started = phase, now
        self.node.get_logger().info(f"fleet executor phase: {phase}")

    def _fresh(self, now):
        states = self.estimator.states
        wall = time.monotonic()
        return all(name in states and self.executor._fresh(states[name], now) and wall - states[name]["received_wall"] <= self.settings["wall_timeout"] for name in self.names)

    def _land(self, now):
        self.stack.land(height=self.settings["land_height"], duration=self.settings["land_duration"], wait=False)
        self._phase("LANDING", now)

    def _publish_states(self, now, fresh):
        if set(self.estimator.states) != set(self.names):
            return
        if self.last_state_publish is not None and 0 <= now - self.last_state_publish < 1.0 / self.settings["state_rate"]:
            return
        robots = {}
        for name in self.names:
            state = self.estimator.states[name]
            goal = self.executor.goals.get(name)
            last = self.executor.last_setpoints.get(name)
            robots[name] = dict(position=state["position"], velocity=state["velocity"], stamp=state["stamp"], received_at=state["received_at"], phase=self.phase, ready=self.phase == "AIRBORNE" and fresh, goal_id=goal["goal_id"] if goal else None, arrived=fresh and self.phase == "AIRBORNE" and self.executor.arrived(name, state), execution_mode=last.mode if last else "HIGH_LEVEL")
        data = dict(version=1, frame=self.settings["frame"], stamp=now, robots=robots, executor=dict(phase=self.phase, sequence=self.executor.sequence, error=self.executor.last_error))
        self.state_pub.publish(self._String(data=json.dumps(data, allow_nan=False)))
        self.last_state_publish = now

    def tick(self):
        now = self.now()
        fresh = self._fresh(now)
        states = self.estimator.states
        if self.phase == "WAITING_POSE":
            if self.land_requested:
                self._phase("LANDED", now)
            elif fresh and all(norm(states[name]["velocity"]) < 0.1 for name in self.names):
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= 0.2:
                    self.stack.takeoff(height=self.settings["takeoff_height"], duration=self.settings["takeoff_duration"], wait=False)
                    self._phase("TAKEOFF", now)
            else:
                self.stable_since = None
        elif self.phase == "TAKEOFF":
            if self.land_requested:
                self._land(now)
            elif fresh and now - self.phase_started >= self.settings["takeoff_duration"] and all(abs(states[name]["position"][2] - self.settings["takeoff_height"]) <= self.settings["arrival_radius"] and norm(states[name]["velocity"]) <= self.settings["arrival_speed"] for name in self.names):
                self._phase("AIRBORNE", now)
            elif now - self.phase_started > self.settings["takeoff_timeout"]:
                self._phase("TAKEOFF_FAILED", now)
        elif self.phase == "TAKEOFF_FAILED" and self.land_requested:
            self._land(now)
        elif self.phase == "AIRBORNE" and self.land_requested:
            self.executor.invalidate("LAND_REQUESTED")
            self._phase("BRAKING_FOR_LAND", now)
        elif self.phase == "LAND_HANDOFF" and now - self.phase_started >= 0.3:
            self._land(now)
        elif self.phase == "LANDING":
            if fresh and now - self.phase_started >= self.settings["land_duration"] and all(states[name]["position"][2] <= self.settings["land_height"] + self.settings["arrival_radius"] for name in self.names):
                self._phase("LANDED", now)
        if self.phase in ("AIRBORNE", "BRAKING_FOR_LAND"):
            if not fresh:
                self.executor.invalidate("STALE_STATE")
            # No sleep or per-drone execution loop: every handle receives the
            # same timestamp's reference before control returns to ROS.
            setpoints = self.executor.sample(now, states)
            for name in self.names:
                command = setpoints[name]
                self.handles[name].cmdFullState(command.position, command.velocity, command.acceleration, command.yaw, command.omega)
            if self.phase == "BRAKING_FOR_LAND" and all(command.mode == "HOVER" for command in setpoints.values()) and fresh and all(norm(states[name]["velocity"]) <= self.settings["arrival_speed"] for name in self.names):
                for handle in self.handles.values():
                    handle.notifySetpointsStop()
                self._phase("LAND_HANDOFF", now)
        self._publish_states(now, fresh)


def options(args=None):
    from ..common.trajectory import trajectory_options
    return trajectory_options(args)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from .stack import DroneStack
    from ..common.trajectory import load_dmpc_settings, load_robot_positions

    opt = options(args)
    settings, homes = load_dmpc_settings(opt.config), load_robot_positions(opt.robots)
    stack = DroneStack()
    try:
        DroneExecutorNode(stack, homes, settings)
        rclpy.spin(stack.allcfs)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stack.allcfs.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
