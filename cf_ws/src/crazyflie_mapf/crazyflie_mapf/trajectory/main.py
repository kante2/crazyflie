"""ROS adapter for centralized calculation and exchanged per-drone QPs."""

import time

from ..common.config import decode, encode
from ..common.fleet import goal_batch, state_batch
from ..common.ros import qos, ros_time, spin, timer
from ..common.trajectory import load_experiment, trajectory_options
from .functions import MultiDroneTrajectoryPlanner, sample_trajectory
from .metrics import FleetMetrics
from .visualization import TrajectoryVisualization


class TrajectoryPlannerNode:
    def __init__(self, node, homes, settings, preview=False):
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data

        self.node, self.homes, self.config = node, homes, settings
        self.preview = preview
        self.planner = MultiDroneTrajectoryPlanner(settings)
        self.metrics = FleetMetrics(settings)
        self.visualization = TrajectoryVisualization(node, homes, settings)
        self.states, self.goals, self.revision = {}, {}, -1
        self.received, self.state_stamp, self.last_now = None, None, None
        self.last_batch, self.last_status = None, None
        self._String = String
        self.trajectory_pub = node.create_publisher(String, "/fleet/trajectories", 1)
        self.status_pub = node.create_publisher(String, "/trajectory/status", qos())
        self.subscriptions = [node.create_subscription(String, "/fleet/goals", self.receive_goals, qos())]
        if preview:
            self.state_pub = node.create_publisher(String, "/fleet/states", qos_profile_sensor_data)
        else:
            self.subscriptions.append(node.create_subscription(String, "/fleet/states", self.receive_states, qos_profile_sensor_data))
        self.timer = timer(node, 1.0 / settings["plan_rate"], self.tick)
        node.get_logger().info(f"per-agent DMPC on central PC: {len(homes)} robots, {settings['horizon']:g}s horizon, {settings['plan_rate']:g}Hz replanning" + (" (ideal tracking preview)" if preview else ""))

    def warning(self, text):
        self.node.get_logger().warning(text, throttle_duration_sec=2.0)

    def receive_goals(self, message):
        try:
            data = decode(message)
            revision, goals = goal_batch(data, self.homes, self.config["frame"])
            if revision < self.revision:
                return
            if revision == self.revision and any(goals[r]["goal_id"] != self.goals[r]["goal_id"] or goals[r]["position"] != self.goals[r]["position"] for r in self.homes):
                raise ValueError("the same goal revision cannot change destinations or IDs")
            self.revision, self.goals = revision, goals
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            self.warning(f"fleet goals ignored: {exc}")

    def receive_states(self, message):
        try:
            stamp, states = state_batch(decode(message), self.homes, self.config["frame"])
            if self.state_stamp is not None and stamp < self.state_stamp:
                return
            for state in states.values():
                state["radius"] = self.config["robot_radius"]
            self.state_stamp, self.states, self.received = stamp, states, time.monotonic()
            self.visualization.record_states(states)
            self.metrics.observe(states, now=ros_time(self.node))
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            self.warning(f"fleet states ignored: {exc}")

    def preview_states(self, now):
        for robot, home in self.homes.items():
            state = self.states.get(robot)
            if self.last_batch and robot in self.last_batch["trajectories"] and self.last_batch["status"] in ("SOLVED", "BRAKING"):
                sample = sample_trajectory(self.last_batch["trajectories"][robot], now - self.last_batch["stamp"])
                p, v = sample["position"], sample["velocity"]
            elif state:
                p, v = state["position"], state["velocity"]
            else:
                p, v = (home[0], home[1], self.config["takeoff_height"]), (0., 0., 0.)
            self.states[robot] = dict(position=p, velocity=v, stamp=now, received_at=now, phase="AIRBORNE", ready=True, frame=self.config["frame"], radius=self.config["robot_radius"], goal_id=self.goals.get(robot, {}).get("goal_id"))
        self.received = time.monotonic()
        self.metrics.observe(self.states, now=now)
        self.state_pub.publish(self._String(data=encode(dict(version=1, frame=self.config["frame"], stamp=now, robots=self.states))))

    def tick(self):
        now = ros_time(self.node)
        if self.last_now is not None and now < self.last_now:
            self.planner = MultiDroneTrajectoryPlanner(self.config)
            self.states, self.last_batch, self.state_stamp = {}, None, None
        self.last_now = now
        if self.preview:
            self.preview_states(now)
        fresh = self.received is not None and time.monotonic() - self.received <= self.config["wall_timeout"] and set(self.states) == set(self.homes) and all(s["ready"] and -.05 <= now - s["stamp"] <= self.config["state_timeout"] for s in self.states.values())
        if fresh and set(self.goals) == set(self.homes):
            batch = self.planner.plan(self.states, self.goals, now)
        else:
            batch = self.planner.waiting(now, "waiting for fresh airborne states and complete goals")
        # JSON and all reference timing use the planner's shared simulation clock.
        self.last_batch = batch
        self.trajectory_pub.publish(self._String(data=encode(batch)))
        metrics = self.metrics.observe(self.states, batch, now)
        self.status_pub.publish(self._String(data=encode(dict(version=1, frame=self.config["frame"], stamp=now, preview=self.preview, status=batch["status"], goal_revision=self.revision, sequence=batch["sequence"], dt=self.config["dt"], horizon=self.config["horizon"], diagnostics=batch["diagnostics"], metrics=metrics, over_budget=batch["diagnostics"]["solve_ms"] > 1000 / self.config["plan_rate"]))))
        self.visualization.publish(batch, self.states, self.goals, metrics)
        if batch["status"] != self.last_status:
            self.node.get_logger().info(f"trajectory planner: {batch['status']}")
            self.last_status = batch["status"]


def main(args=None):
    import rclpy
    opt = trajectory_options(args)
    _, settings, homes, _ = load_experiment(opt.robots, opt.config, opt.goals, opt.obstacles)
    rclpy.init()
    node = rclpy.create_node("multi_drone_trajectory_planner")
    TrajectoryPlannerNode(node, homes, settings, opt.preview)
    spin(node)


if __name__ == "__main__":
    main()
