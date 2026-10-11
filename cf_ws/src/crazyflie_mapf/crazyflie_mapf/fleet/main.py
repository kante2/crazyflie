"""Fleet mission adapter: destinations, queues and measured completion only."""

import math
import time

from ..common.config import decode, encode
from ..common.fleet import envelope, state_batch
from ..common.ros import qos, ros_time, spin, timer
from ..common.trajectory import load_experiment, trajectory_options
from .functions import FleetManager


class FleetManagerNode:
    def __init__(self, node, homes, settings, goals):
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data
        from visualization_msgs.msg import MarkerArray

        self.node, self.config, self.homes = node, settings, homes
        self.manager = FleetManager(homes, settings)
        self.initial_goals = self.manager.validate_goals(goals)
        self.states, self.state_stamp, self.received = {}, None, None
        self.rejection = None
        self.last_goal_draw = None
        self.visible_goal_markers = set()
        self._String, self._MarkerArray = String, MarkerArray
        self.goals_pub = node.create_publisher(String, "/fleet/goals", qos())
        self.status_pub = node.create_publisher(String, "/fleet/status", qos())
        self.markers_pub = node.create_publisher(MarkerArray, "/trajectory/goals", qos())
        self.subscriptions = [
            node.create_subscription(String, "/fleet/states", self.receive_states, qos_profile_sensor_data),
            node.create_subscription(String, "/fleet/goal_request", self.request_goal, 10),
        ]
        self.timer = timer(node, 1.0 / settings["state_rate"], self.tick)
        self.last_finished = False
        node.get_logger().info(f"FleetManager configured goals: waiting for {len(homes)} measured airborne states")

    def warning(self, text):
        self.node.get_logger().warning(text, throttle_duration_sec=2.0)

    def receive_states(self, message):
        try:
            stamp, states = state_batch(decode(message), self.homes, self.config["frame"])
            if self.state_stamp is not None and stamp < self.state_stamp:
                return
            self.states, self.state_stamp, self.received = states, stamp, time.monotonic()
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            self.warning(f"fleet states ignored: {exc}")

    def request_goal(self, message):
        try:
            data = decode(message)
            envelope(data, self.config["frame"])
            if type(data.get("enqueue", False)) is not bool:
                raise ValueError("enqueue는 bool이어야 합니다.")
            self.manager.request_goal(data["robot"], data["position"], ros_time(self.node), data.get("enqueue", False))
            self.rejection = None
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            self.rejection = str(exc)
            self.warning(f"goal request rejected: {exc}")

    def publish_goals(self, now):
        self.goals_pub.publish(self._String(data=encode(dict(version=1, frame=self.config["frame"], stamp=now, revision=self.manager.revision, robots=self.manager.goals))))
        if self.last_goal_draw is not None and 0 <= now - self.last_goal_draw < 1 / self.config["plan_rate"]:
            return
        self.last_goal_draw = now
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker
        from ..common.markers import COLORS, marker
        markers = []
        for i, (robot, goal) in enumerate(self.manager.goals.items()):
            p = goal["position"]
            ring = marker(self.node, self.config["frame"], "goal_ring", i, Marker.LINE_STRIP, p, COLORS[i % len(COLORS)], (.012, 0, 0))
            ring.points = [Point(x=.14 * math.cos(k * math.tau / 32), y=.14 * math.sin(k * math.tau / 32), z=0.) for k in range(33)]
            ring.color.a = .75
            markers.append(ring)
            if goal["status"] == "COMPLETED":
                check = marker(self.node, self.config["frame"], "goal_reached", i, Marker.LINE_STRIP, p, COLORS[i % len(COLORS)], (.017, 0, 0))
                check.points = [Point(x=-.065, y=0., z=.008), Point(x=-.015, y=-.045, z=.008), Point(x=.075, y=.055, z=.008)]
                markers.append(check)
            else:
                state = self.states.get(robot)
                if state is None or math.dist(state["position"], p) >= .4:
                    offset_x, offset_y = (.28 if p[0] >= 0 else -.28), (.28 if p[1] >= 0 else -.28)
                    label = marker(self.node, self.config["frame"], "goal_label", i, Marker.TEXT_VIEW_FACING, (p[0] + offset_x, p[1] + offset_y, p[2] + .04), COLORS[i % len(COLORS)], (0, 0, .10))
                    label.text = f"{robot}:goal"
                    markers.append(label)
        keys = {(m.ns, m.id) for m in markers}
        removed = [Marker(ns=ns, id=index, action=Marker.DELETE) for ns, index in self.visible_goal_markers - keys]
        self.markers_pub.publish(self._MarkerArray(markers=removed + markers))
        self.visible_goal_markers = keys

    def tick(self):
        now = ros_time(self.node)
        fresh = self.received is not None and time.monotonic() - self.received <= self.config["wall_timeout"] and all(-.05 <= now - s["stamp"] <= self.config["state_timeout"] for s in self.states.values())
        if not self.manager.goals and fresh and self.states and all(s["ready"] for s in self.states.values()):
            self.manager.assign_goals(self.initial_goals, now)
            self.node.get_logger().info("configured destinations assigned")
        if self.manager.goals:
            if fresh:
                self.manager.update(self.states, now)
            self.publish_goals(now)
        snapshot = self.manager.snapshot(now)
        self.status_pub.publish(self._String(data=encode(dict(version=1, frame=self.config["frame"], stamp=now, state_fresh=fresh, rejection=self.rejection, **snapshot))))
        if snapshot["finished"] and not self.last_finished:
            self.node.get_logger().info(f"all destinations reached: {snapshot['mission_time']:.2f} simulation seconds")
        self.last_finished = snapshot["finished"]


def main(args=None):
    import rclpy
    opt = trajectory_options(args)
    _, settings, homes, goals = load_experiment(opt.robots, opt.config, opt.goals, opt.obstacles)
    rclpy.init()
    node = rclpy.create_node("fleet_manager")
    FleetManagerNode(node, homes, settings, goals)
    spin(node)


if __name__ == "__main__":
    main()
