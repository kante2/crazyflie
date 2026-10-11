from pathlib import Path
"""Mission completion follows fresh measured arrival, never predicted arrival."""

from copy import deepcopy

import pytest

from crazyflie_mapf.common.trajectory import load_dmpc_settings
from crazyflie_mapf.fleet.functions import FleetManager


@pytest.fixture
def manager():
    settings = load_dmpc_settings(Path(__file__).resolve().parents[1] / "config/planner.yaml")
    fleet = FleetManager(("cf1", "cf2"), settings)
    fleet.assign_goals({"cf1": [-2, 0, 1.2], "cf2": [2, 0, 1.2]}, 0.0)
    return fleet


def state(manager, robot, now=1.0, **updates):
    goal = manager.goals[robot]
    result = dict(position=goal["position"], velocity=[0, 0, 0], stamp=now,
                  goal_id=goal["goal_id"], ready=True, phase="TRACKING")
    return dict(result, **updates)


@pytest.mark.parametrize("goals", [
    {"cf1": [-2, 0, 1.2]},
    {"cf1": [-2, 0, 1.2], "cf2": [8, 0, 1.2]},
    {"cf1": [-2, 0, 1.2], "cf2": [float("nan"), 0, 1.2]},
    {"cf1": [0, 0, 1.2], "cf2": [.2, 0, 1.2]},
    {"cf1": [0, 0, 1.2], "cf2": [0, 0, 2.0]},
])
def test_invalid_fleet_assignment_preserves_existing_mission(manager, goals):
    original = deepcopy(manager.goals)
    revision = manager.revision
    with pytest.raises(ValueError):
        manager.assign_goals(goals, 2.0)
    assert manager.goals == original
    assert manager.revision == revision


@pytest.mark.parametrize("updates", [
    {"ready": False},
    {"goal_id": "old-goal"},
    {"stamp": 0.0},
    {"stamp": 2.0},
    {"position": [-1.5, 0, 1.2]},
    {"velocity": [1, 0, 0]},
])
def test_completion_requires_matching_ready_fresh_stopped_arrival(manager, updates):
    manager.update({"cf1": state(manager, "cf1", **updates)}, 1.0)
    assert manager.goals["cf1"]["status"] == "ACTIVE"
    assert manager.completed == []


def test_measured_completion_is_idempotent_and_freezes_mission_time(manager):
    measured = {robot: state(manager, robot, now=3.0) for robot in manager.robots}
    manager.update(measured, 3.0)
    manager.update(measured, 3.1)
    assert len(manager.completed) == 2
    snapshot = manager.snapshot(20.0)
    assert snapshot["finished"]
    assert snapshot["mission_time"] == 3.0


def test_changing_one_goal_preserves_completed_other_robot(manager):
    manager.update({"cf1": state(manager, "cf1")}, 1.0)
    completed_goal = deepcopy(manager.goals["cf1"])
    previous_id = manager.goals["cf2"]["goal_id"]
    manager.request_goal("cf2", [2, 1, 1.2], 2.0)
    assert manager.goals["cf1"] == completed_goal
    assert manager.goals["cf2"]["goal_id"] != previous_id
    assert manager.goals["cf2"]["status"] == "ACTIVE"
    assert not manager.snapshot(2.0)["finished"]


def test_enqueued_goal_waits_for_measured_arrival_and_preserves_peer(manager):
    manager.update({"cf2": state(manager, "cf2")}, 1.0)
    completed_peer = deepcopy(manager.goals["cf2"])
    initial_id = manager.goals["cf1"]["goal_id"]
    manager.request_goal("cf1", [-2, 1, 1.2], 1.0, enqueue=True)
    assert manager.goals["cf1"]["goal_id"] == initial_id
    manager.update({"cf1": state(manager, "cf1", now=2.0)}, 2.0)
    assert manager.goals["cf1"]["goal_id"] != initial_id
    assert manager.goals["cf1"]["position"] == (-2.0, 1.0, 1.2)
    assert manager.goals["cf1"]["status"] == "ACTIVE"
    assert manager.goals["cf2"] == completed_peer
    assert not manager.queues["cf1"]
    assert len(manager.completed) == 2
    assert not manager.snapshot(2.0)["finished"]


def test_blocked_queued_goal_is_reported_and_never_marks_fleet_finished(manager):
    manager.request_goal("cf1", [2, 0, 1.2], 0.0, enqueue=True)
    manager.update({robot: state(manager, robot, now=1.0) for robot in manager.robots}, 1.0)
    assert manager.failed["cf1"]
    assert len(manager.queues["cf1"]) == 1
    assert not manager.snapshot(1.0)["finished"]


def test_executor_failure_is_distinct_from_arrival(manager):
    manager.update({"cf1": state(manager, "cf1", phase="FAILED", reason="lost tracking")}, 1.0)
    assert manager.goals["cf1"]["status"] == "FAILED"
    assert manager.failed["cf1"] == "lost tracking"
    assert manager.completed == []
    assert not manager.snapshot(1.0)["finished"]


def test_invalid_queued_goal_and_unknown_robot_are_rejected(manager):
    with pytest.raises(ValueError):
        manager.request_goal("cf1", [9, 0, 1.2], 1.0, enqueue=True)
    with pytest.raises(ValueError):
        manager.request_goal("cf9", [1, 0, 1.2], 1.0)
    assert not manager.queues["cf1"]


def test_enqueue_after_fleet_completion_immediately_starts_available_robot(manager):
    manager.update({robot: state(manager, robot, now=1.0) for robot in manager.robots}, 1.0)
    completed_peer = deepcopy(manager.goals["cf2"])
    old_id = manager.goals["cf1"]["goal_id"]
    assert manager.snapshot(1.0)["finished"]
    manager.request_goal("cf1", [-2, 1, 1.2], 2.0, enqueue=True)
    assert manager.goals["cf1"]["status"] == "ACTIVE"
    assert manager.goals["cf1"]["goal_id"] != old_id
    assert manager.goals["cf1"]["position"] == (-2.0, 1.0, 1.2)
    assert manager.goals["cf2"] == completed_peer
    assert not manager.queues["cf1"]
    assert not manager.snapshot(2.0)["finished"]
    manager.update({robot: state(manager, robot, now=3.0) for robot in manager.robots}, 3.0)
    assert manager.snapshot(3.0)["finished"]
    assert manager.snapshot(3.0)["reached"] == 2
    assert len(manager.completed) == 3


def test_unchanged_completed_goal_request_preserves_revision_and_completion_time(manager):
    manager.update({robot: state(manager, robot, now=1.0) for robot in manager.robots}, 1.0)
    previous = deepcopy(manager.snapshot(1.0))
    completed_at = manager.completed_at
    manager.request_goal("cf1", manager.goals["cf1"]["position"], 2.0)
    assert manager.snapshot(2.0) == previous
    assert manager.completed_at == completed_at
