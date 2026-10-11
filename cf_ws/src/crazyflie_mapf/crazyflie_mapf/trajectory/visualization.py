"""Separate measured tracks, timed forecasts and body-sized occupancy in RViz."""

import math
from collections import deque

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..common.obstacles import geometry, motion_state
from ..common.markers import COLORS, marker
from ..common.ros import qos
from .display_geometry import curve_samples, swept_tube, time_annotations


def _points(positions):
    return [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in positions]


class TrajectoryVisualization:
    def __init__(self, node, robots, settings):
        self.node, self.robots, self.config = node, tuple(robots), settings
        self.publishers = {
            layer: node.create_publisher(MarkerArray, f"/trajectory/{layer}", qos())
            for layer in ("paths", "occupancy", "trails", "markers")
        }
        self.visible = {layer: set() for layer in self.publishers}
        self.history = {robot: deque(maxlen=600) for robot in self.robots}
        self.last_stamp = {}

    def _marker(self, namespace, index, kind, xyz, color, scale, alpha=1.):
        m = marker(self.node, self.config["frame"], namespace, index, kind, xyz, color, scale)
        m.color.a = alpha
        return m

    def record_states(self, states):
        """Retain real positions; stationary repeats never obscure flown paths."""
        for robot, state in states.items():
            if robot not in self.history:
                continue
            stamp = state["stamp"]
            previous = self.last_stamp.get(robot)
            if previous is not None and stamp < previous:
                self.history[robot].clear()
            elif previous is not None and stamp == previous:
                continue
            self.last_stamp[robot] = stamp
            position = tuple(state["position"])
            history = self.history[robot]
            if previous is not None and stamp - previous > self.config["state_timeout"]:
                # A telemetry gap must not become a fictitious flown segment.
                history.append((stamp, None))
            last_position = history[-1][1] if history else None
            if last_position is None or math.dist(last_position, position) >= .02:
                history.append((stamp, position))

    def _publish(self, layer, markers, lifetime=None):
        keys = {(m.ns, m.id) for m in markers}
        removed = [Marker(ns=ns, id=index, action=Marker.DELETE)
                   for ns, index in self.visible[layer] - keys]
        if lifetime is None:
            lifetime = max(1.0, 4 / self.config["plan_rate"])
        for m in markers:
            m.lifetime = Duration(sec=int(lifetime), nanosec=int((lifetime % 1) * 1e9))
        self.publishers[layer].publish(MarkerArray(markers=removed + markers))
        self.visible[layer] = keys

    def _trail(self, robot, color):
        segments, current = [], []
        for _, position in self.history[robot]:
            if position is None:
                if len(current) > 1:
                    segments.append(current)
                current = []
            else:
                current.append(position)
        if len(current) > 1:
            segments.append(current)
        lines = []
        for part, positions in enumerate(segments):
            line = self._marker(f"flown/{robot}", part, Marker.LINE_STRIP, (0, 0, 0), color, (.012, 0, 0), .45)
            line.points = _points(positions)
            lines.append(line)
        return lines

    def publish(self, batch, states, goals=None, metrics=None):
        self.record_states(states)
        paths, occupancy, trails, overlays = [], [], [], []
        now = self.node.get_clock().now().nanoseconds * 1e-9
        for index, obstacle in enumerate(self.config.get('obstacles', [])):
            overlays.append(self._marker('obstacles', index, Marker.CUBE, motion_state(obstacle, now)[0],
                                         (.8, .35, .12), obstacle['size'], .9))
            center, scale = geometry(obstacle, self.config['robot_radius'], self.config['collision_margin'], stamp=now)
            overlays.append(self._marker('obstacle_clearance', index, Marker.SPHERE, center,
                                         (.9, .5, .2), 2/scale, .10))
        goals = goals or {}
        remaining = batch["valid_until"] - now
        valid = batch["status"] in ("SOLVED", "BRAKING") and remaining > 0
        for index, robot in enumerate(self.robots):
            color = COLORS[index % len(COLORS)]
            trails.extend(self._trail(robot, color))
            state = states.get(robot)
            if not state:
                continue
            p = state["position"]
            speed = math.sqrt(sum(v * v for v in state["velocity"]))
            fresh = -.05 <= now - state["stamp"] <= self.config["state_timeout"]
            arrived = goals.get(robot, {}).get("status") == "COMPLETED"
            overlays.append(self._marker("measured_drone", index, Marker.SPHERE, p, color, (.11,) * 3, 1. if fresh else .35))
            label = self._marker("drone_label", index, Marker.TEXT_VIEW_FACING, (p[0], p[1], p[2] + .25), color, (0, 0, .105))
            if not fresh:
                mode = "stale"
            elif arrived:
                mode = "arrived"
            elif state.get("phase") == "TAKEOFF":
                mode = "takeoff"
            elif not state.get("ready"):
                mode = "waiting"
            elif batch["status"] == "BRAKING":
                mode = "braking"
            else:
                mode = "moving" if speed > .05 else "holding"
            # Some Ogre font builds stretch whitespace. Compact lines also
            # keep the label beside the drone instead of across its trajectory.
            speed_text = f"{speed:.2f}m/s" if fresh else "--"
            label.text = f"{robot}:{speed_text}\n{mode}"
            overlays.append(label)
            trajectory = batch["trajectories"].get(robot) if valid else None
            if not trajectory:
                continue
            samples = curve_samples(trajectory)
            centers = [point for _, point in samples]
            horizon = trajectory["times"][-1]
            moving = any(math.dist(centers[0], point) > .025 for point in centers)
            if moving:
                curve = self._marker("future_path", index, Marker.LINE_STRIP, (0, 0, 0), color, (.028, 0, 0))
                curve.points = _points(centers)
                curve.colors = [ColorRGBA(
                    r=float(color[0] * (1 - .45 * t / horizon)),
                    g=float(color[1] * (1 - .45 * t / horizon)),
                    b=float(color[2] * (1 - .45 * t / horizon)), a=1.)
                    for t, _ in samples]
                paths.append(curve)
                mesh = self._marker("body_tube", index, Marker.TRIANGLE_LIST, (0, 0, 0), color, (1, 1, 1), .10)
                mesh.points = _points(swept_tube(centers, self.config["robot_radius"]))
                occupancy.append(mesh)
                for tick, (text, position) in enumerate(time_annotations(trajectory)):
                    paths.append(self._marker(f"forecast_time/{robot}", tick, Marker.SPHERE, position, color, (.055,) * 3))
                    if math.dist(position, p) < .3:
                        # Near a settled goal the drone label already identifies
                        # this small forecast; avoid stacking time text on it.
                        continue
                    text_marker = self._marker(f"forecast_label/{robot}", tick, Marker.TEXT_VIEW_FACING, (position[0], position[1], position[2] + .12), color, (0, 0, .085))
                    text_marker.text = text
                    paths.append(text_marker)
                velocity = trajectory["velocities"][0]
                length = math.sqrt(sum(v * v for v in velocity))
                if length > .05:
                    arrow = self._marker("travel_direction", index, Marker.ARROW, (0, 0, 0), color, (.018, .06, .09), .85)
                    arrow.points = _points([centers[0], tuple(a + .28 * v / length for a, v in zip(centers[0], velocity))])
                    paths.append(arrow)
            else:
                occupancy.append(self._marker("stationary_occupancy", index, Marker.SPHERE, centers[0], color, (2 * self.config["robot_radius"],) * 3, .08))

        reached = sum(goal.get("status") == "COMPLETED" for goal in goals.values())
        if batch["status"] in ("BRAKING", "UNSAFE"):
            status = "STOPPING"
        elif batch["status"] == "WAITING":
            status = "WAITING"
        else:
            status = "ARRIVED" if goals and reached == len(self.robots) else "FLYING"
        hud = self._marker("fleet_overview", 0, Marker.TEXT_VIEW_FACING, (-2.6, .6, 1.4), (.82, .87, .94), (0, 0, .13))
        hud.text = f"{status}:{reached}/{len(self.robots)}\nPreview:{self.config['horizon']:g}s|r={self.config['robot_radius']:g}m\nTrail:measured"
        gap = (metrics or {}).get("min_distance_m")
        if gap is not None:
            hud.text += f"\nMin-gap:{gap:.2f}m"
        overlays.append(hud)
        for layer, markers in (("paths", paths), ("occupancy", occupancy), ("trails", trails), ("markers", overlays)):
            lifetime = max(1e-9, remaining) if layer in ("paths", "occupancy") else None
            self._publish(layer, markers, lifetime)
