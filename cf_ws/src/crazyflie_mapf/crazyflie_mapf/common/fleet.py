"""Atomic, timestamped fleet messages, independent of ROS transports."""

import math

from .config import point


def envelope(data, frame):
    if not isinstance(data, dict) or type(data.get("version")) is not int or data["version"] != 1 or data.get("frame") != frame:
        raise ValueError("version=1과 공통 좌표계가 필요합니다.")


def finite_stamp(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError("유한한 상태 시각이 필요합니다.")
    return float(value)


def state_batch(data, robots, frame):
    envelope(data, frame)
    stamp = finite_stamp(data["stamp"])
    entries = data["robots"]
    if not isinstance(entries, dict) or set(entries) != set(robots):
        raise ValueError("모든 활성 기체의 상태가 필요합니다.")
    result = {}
    for robot, state in entries.items():
        result[robot] = dict(state, position=point(state["position"]), velocity=point(state["velocity"]), stamp=finite_stamp(state["stamp"]), frame=frame)
        if type(state.get("ready")) is not bool:
            raise ValueError("기체 ready 상태는 bool이어야 합니다.")
        if state["stamp"] > stamp + 0.05:
            raise ValueError("수신 배치보다 미래인 기체 시각입니다.")
    return stamp, result


def goal_batch(data, robots, frame):
    envelope(data, frame)
    revision = data["revision"]
    finite_stamp(data["stamp"])
    if type(revision) is not int or revision < 0 or set(data["robots"]) != set(robots):
        raise ValueError("편대 목적지 revision/기체 목록이 잘못되었습니다.")
    goals = {}
    for robot, goal in data["robots"].items():
        if not isinstance(goal["goal_id"], str) or not goal["goal_id"]:
            raise ValueError("목적지 ID가 필요합니다.")
        goals[robot] = dict(goal, position=point(goal["position"]))
    return revision, goals
