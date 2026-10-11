"""궤적 계산과 비행 명령을 소유하지 않는 편대 임무 상태기."""

from ..common.obstacles import validate_position

import math
from collections import deque

from ..common.config import point


class FleetManager:
    def __init__(self, robots, settings):
        self.robots, self.config = tuple(robots), settings
        self.revision = 0
        self.goals = {}
        self.queues = {r: deque() for r in self.robots}
        self.completed = []
        self.failed = {}
        self.started_at = None
        self.completed_at = None

    def validate_goals(self, goals):
        if set(goals) != set(self.robots):
            raise ValueError("모든 활성 기체의 목적지가 필요합니다.")
        lo, hi = self.config["bounds"]
        validated = {r: point(p) for r, p in goals.items()}
        for p in validated.values():
            validate_position(p, self.config)
            if not all(a <= v <= b for a, v, b in zip(lo, p, hi)):
                raise ValueError("목적지가 계획 공간 밖에 있습니다.")
        axes = (self.config["horizontal_separation"],) * 2 + (self.config["vertical_separation"],)
        names = list(validated)
        for i, left in enumerate(names):
            for right in names[i + 1 :]:
                if sum(((a - b) / axis) ** 2 for a, b, axis in zip(validated[left], validated[right], axes)) < 1.0:
                    raise ValueError("두 목적지의 안전 영역이 겹칩니다.")
        return validated

    def assign_goals(self, goals, now):
        validated = self.validate_goals(goals)
        if self.goals and all(tuple(self.goals[r]["position"]) == p for r, p in validated.items()):
            return self.goals
        self.revision += 1
        for robot, p in validated.items():
            previous = self.goals.get(robot)
            if previous is None or tuple(previous["position"]) != p:
                self.goals[robot] = dict(goal_id=f"{robot}:{self.revision}", position=p, status="ACTIVE")
                self.failed.pop(robot, None)
        self.started_at = now if self.started_at is None else self.started_at
        self.completed_at = None
        return self.goals

    def request_goal(self, robot, position, now, enqueue=False):
        if robot not in self.robots:
            raise ValueError("알 수 없는 기체입니다.")
        target = point(position)
        validate_position(target, self.config)
        if enqueue and not (self.goals.get(robot, {}).get("status") == "COMPLETED" and not self.queues[robot]):
            lo, hi = self.config["bounds"]
            if not all(a <= v <= b for a, v, b in zip(lo, target, hi)):
                raise ValueError("목적지가 계획 공간 밖에 있습니다.")
            self.queues[robot].append(target)
            return
        if set(self.goals) != set(self.robots):
            raise ValueError("초기 편대 목적지가 아직 준비되지 않았습니다.")
        goals = {r: g["position"] for r, g in self.goals.items()}
        goals[robot] = target
        self.assign_goals(goals, now)

    def update(self, states, now):
        for robot, goal in list(self.goals.items()):
            state = states.get(robot)
            if not state:
                continue
            if "FAILED" in state.get("phase", ""):
                goal["status"] = "FAILED"
                self.failed[robot] = state.get("reason", "executor failed")
            if goal["status"] != "ACTIVE" or state.get("goal_id") != goal["goal_id"] or not state.get("ready"):
                continue
            if not -0.05 <= now - state["stamp"] <= self.config["state_timeout"]:
                continue
            speed = math.sqrt(sum(v * v for v in state["velocity"]))
            if math.dist(state["position"], goal["position"]) <= self.config["arrival_radius"] and speed <= self.config["arrival_speed"]:
                goal["status"] = "COMPLETED"
                self.completed.append(dict(robot=robot, goal_id=goal["goal_id"], stamp=now))
                if self.queues[robot]:
                    target = self.queues[robot][0]
                    try:
                        self.request_goal(robot, target, now)
                    except ValueError as exc:
                        self.failed[robot] = str(exc)
                        goal["status"] = "FAILED"
                    else:
                        self.queues[robot].popleft()
        if self.goals and not any(self.queues.values()) and all(g["status"] == "COMPLETED" for g in self.goals.values()):
            if self.completed_at is None:
                self.completed_at = now

    def snapshot(self, now):
        return dict(
            revision=self.revision, robots=self.goals,
            queues={r: list(q) for r, q in self.queues.items()},
            completed=list(self.completed), failed=dict(self.failed),
            finished=self.completed_at is not None,
            reached=sum(g["status"] == "COMPLETED" for g in self.goals.values()),
            arrival_rate=sum(g["status"] == "COMPLETED" for g in self.goals.values()) / len(self.robots),
            mission_time=(self.completed_at if self.completed_at is not None else now) - self.started_at
            if self.started_at is not None else 0.0,
        )
