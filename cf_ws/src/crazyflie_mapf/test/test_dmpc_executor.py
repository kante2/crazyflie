"""Fleet batch boundaries and physical stopping, independent of ROS/hardware."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from crazyflie_mapf.executor.functions import FleetExecutor, StateEstimator
from crazyflie_mapf.executor.main import DroneExecutorNode


@pytest.fixture
def executor():
    result = FleetExecutor(("cf1", "cf2"), dict(frame="world", speed=2.0, acceleration=0.6, deceleration=0.8, command_ttl=0.4, state_timeout=0.5))
    result.set_goals(dict(version=1, frame="world", revision=0, stamp=1.0, robots={name: dict(goal_id=f"goal-{name}", position=(3.0, index, 1.2)) for index, name in enumerate(result.names)}), 1.0)
    return result


def batch(executor, sequence=0, issued=1.0):
    robots = {}
    for index, name in enumerate(executor.names):
        robots[name] = dict(goal_id=executor.goals[name]["goal_id"], times=[0.0, 0.2, 0.4], positions=[(0.0, index, 1.2), (0.012, index, 1.2), (0.048, index, 1.2)], velocities=[(0.0, 0.0, 0.0), (0.12, 0.0, 0.0), (0.24, 0.0, 0.0)], accelerations=[(0.6, 0.0, 0.0), (0.6, 0.0, 0.0), (0.0, 0.0, 0.0)])
    return dict(version=1, frame="world", sequence=sequence, issued_at=issued, valid_until=issued + 0.4, status="SOLVED", trajectories=robots)


def states(executor, stamp, speed=0.0):
    return {name: dict(frame="world", position=(0.0, index, 1.2), velocity=(speed, 0.0, 0.0), stamp=stamp) for index, name in enumerate(executor.names)}


def test_shared_tick_interpolates_position_velocity_and_acceleration(executor):
    assert executor.accept(batch(executor), 1.0)
    commands = executor.sample(1.1, states(executor, 1.1))
    assert set(commands) == {"cf1", "cf2"}
    for index, name in enumerate(executor.names):
        assert commands[name].position == pytest.approx((0.003, index, 1.2))
        assert commands[name].velocity == pytest.approx((0.06, 0.0, 0.0))
        assert commands[name].acceleration == pytest.approx((0.6, 0.0, 0.0))
    just_before = executor.sample(1.2 - 1e-8, states(executor, 1.2))["cf1"]
    just_after = executor.sample(1.2 + 1e-8, states(executor, 1.2))["cf1"]
    assert just_before.position == pytest.approx(just_after.position, abs=1e-7)
    assert just_before.velocity == pytest.approx(just_after.velocity, abs=1e-7)


@pytest.mark.parametrize("mutation", [
    lambda data: data.update(frame="robot-local"),
    lambda data: data.update(status="UNSAFE"),
    lambda data: data.update(status="STALE"),
    lambda data: data.update(issued_at=2.0),
    lambda data: data.update(valid_until=0.9),
    lambda data: data.update(valid_until=2.0),
    lambda data: data["trajectories"].pop("cf2"),
    lambda data: data["trajectories"]["cf1"].update(goal_id="old-goal"),
    lambda data: data["trajectories"]["cf1"]["positions"].__setitem__(1, (2.0, 0.0, 1.2)),
    lambda data: data["trajectories"]["cf1"]["positions"].__setitem__(1, (float("nan"), 0.0, 1.2)),
    lambda data: data["trajectories"]["cf1"]["times"].__setitem__(1, 0.0),
])
def test_bad_batch_invalidates_every_robot_atomically(executor, mutation):
    assert executor.accept(batch(executor), 1.0)
    bad = batch(executor, sequence=1)
    mutation(bad)
    assert not executor.accept(bad, 1.1)
    assert executor.trajectories is None
    commands = executor.sample(1.1, states(executor, 1.1, speed=1.0))
    assert all(command.mode == "BRAKING" for command in commands.values())
    assert all(command.velocity == (1.0, 0.0, 0.0) for command in commands.values())


def test_delayed_duplicate_does_not_replace_newer_batch(executor):
    assert executor.accept(batch(executor, sequence=2), 1.0)
    assert not executor.accept(batch(executor, sequence=1), 1.1)
    assert executor.sequence == 2
    assert executor.trajectories is not None


def test_dynamic_but_out_of_bounds_trajectory_is_rejected(executor):
    executor.settings["bounds"] = [[-4.5, -4.5, 0.2], [4.5, 4.5, 3.5]]
    data = batch(executor)
    data["trajectories"]["cf1"]["positions"] = [(p[0] + 10.0, p[1], p[2]) for p in data["trajectories"]["cf1"]["positions"]]
    assert not executor.accept(data, 1.0)
    assert "bounds" in executor.last_error


def test_diagonal_velocity_obeys_euclidean_speed_limit(executor):
    data = batch(executor)
    row = data["trajectories"]["cf1"]
    row["velocities"] = [(1.6, 1.6, 0.0)] * 3
    row["accelerations"] = [(0.0, 0.0, 0.0)] * 3
    row["positions"] = [(1.6 * t, 1.6 * t, 1.2) for t in row["times"]]
    assert not executor.accept(data, 1.0)
    assert "speed bound" in executor.last_error


def test_diagonal_acceleration_obeys_euclidean_acceleration_limit(executor):
    data = batch(executor)
    row = data["trajectories"]["cf1"]
    row["accelerations"] = [(0.5, 0.5, 0.0)] * 2 + [(0.0, 0.0, 0.0)]
    row["velocities"] = [(0.5 * t, 0.5 * t, 0.0) for t in row["times"]]
    row["positions"] = [(0.25 * t * t, 0.25 * t * t, 1.2) for t in row["times"]]
    assert not executor.accept(data, 1.0)
    assert "acceleration bound" in executor.last_error


def test_braking_rollout_can_stop_measured_initial_overspeed(executor):
    data = batch(executor)
    data["status"] = "BRAKING"
    for index, name in enumerate(executor.names):
        row = data["trajectories"][name]
        row["positions"] = [(2.5 * t - 0.4 * t * t, index, 1.2) for t in row["times"]]
        row["velocities"] = [(2.5 - 0.8 * t, 0.0, 0.0) for t in row["times"]]
        row["accelerations"] = [(-0.8, 0.0, 0.0)] * 2 + [(0.0, 0.0, 0.0)]
    assert executor.accept(data, 1.0)
    assert executor.sample(1.1, states(executor, 1.1, speed=2.5))["cf1"].velocity[0] == pytest.approx(2.42)


def test_braking_status_cannot_introduce_an_increasing_speed(executor):
    data = batch(executor)
    data["status"] = "BRAKING"
    assert not executor.accept(data, 1.0)
    assert "increase" in executor.last_error


def test_landing_request_cannot_be_replaced_by_an_incoming_moving_batch(executor):
    adapter = DroneExecutorNode.__new__(DroneExecutorNode)
    adapter.land_requested = True
    adapter.executor = executor
    adapter._trajectories(SimpleNamespace(data="{ignored until landing finishes}"))
    assert executor.trajectories is None
    assert executor.last_error == "LAND_REQUESTED"


def test_expiration_brakes_from_measured_state_without_reference_reset(executor):
    assert executor.accept(batch(executor), 1.0)
    moving = states(executor, 1.4, speed=2.0)
    start = executor.sample(1.4, moving)["cf1"]
    assert start.position == (0.0, 0.0, 1.2)
    assert start.velocity == (2.0, 0.0, 0.0)
    assert start.acceleration == (-0.8, -0.0, -0.0)
    # A stationary measurement after the fallback starts must not repeatedly
    # restart the stopping reference; integrate the original 2 m/s state.
    halfway = executor.sample(2.4, states(executor, 2.4))["cf1"]
    assert halfway.position[0] == pytest.approx(1.6)
    assert halfway.velocity[0] == pytest.approx(1.2)
    stopped = executor.sample(4.4, states(executor, 4.4))["cf1"]
    assert stopped.position[0] == pytest.approx(2.5)
    assert stopped.velocity == (0.0, 0.0, 0.0)
    assert stopped.acceleration == (0.0, 0.0, 0.0)
    assert stopped.mode == "HOVER"


def test_stale_measurement_uses_last_reference_for_bounded_stop(executor):
    assert executor.accept(batch(executor), 1.0)
    previous = executor.sample(1.2, states(executor, 1.2))["cf1"]
    stopped = executor.sample(1.8, states(executor, 1.0))["cf1"]
    assert stopped.position == previous.position
    assert stopped.velocity == previous.velocity
    assert stopped.mode == "BRAKING"
    assert executor.last_error == "STALE_STATE"


def test_goal_change_rejects_old_plan_and_requires_complete_new_identity(executor):
    assert executor.accept(batch(executor), 1.0)
    updated = dict(version=1, frame="world", revision=1, stamp=1.1, robots=deepcopy(executor.goals))
    updated["robots"]["cf1"].update(goal_id="replacement", position=(1.0, 2.0, 1.2))
    assert executor.set_goals(updated, 1.1)
    assert executor.trajectories is None
    old = batch(executor, sequence=1, issued=1.1)
    old["trajectories"]["cf1"]["goal_id"] = "goal-cf1"
    assert not executor.accept(old, 1.1)
    assert executor.accept(batch(executor, sequence=2, issued=1.1), 1.1)


def test_state_estimator_uses_measurement_time_not_receipt_time():
    estimator = StateEstimator(("cf1",))
    assert estimator.update("cf1", (0.0, 0.0, 1.2), 1.0, 5.0)
    assert estimator.update("cf1", (0.2, 0.0, 1.2), 1.1, 8.0)
    assert estimator.states["cf1"]["velocity"] == pytest.approx((2.0, 0.0, 0.0))
    assert not estimator.update("cf1", (100.0, 0.0, 1.2), 1.05, 9.0)
    assert estimator.states["cf1"]["position"] == (0.2, 0.0, 1.2)
    assert estimator.update("cf1", (100.0, 0.0, 1.2), 3.0, 10.0)
    assert estimator.states["cf1"]["velocity"] == (0.0, 0.0, 0.0)


def test_fleet_node_tick_sends_every_robot_without_a_blocking_execution_loop(executor):
    assert executor.accept(batch(executor), 1.0)
    adapter = DroneExecutorNode.__new__(DroneExecutorNode)
    adapter.now = lambda: 1.1
    adapter._fresh = lambda _: True
    adapter.settings = executor.settings
    adapter.phase = "AIRBORNE"
    adapter.land_requested = False
    adapter.names = executor.names
    adapter.executor = executor
    adapter.estimator = SimpleNamespace(states=states(executor, 1.1))
    adapter.handles = {name: Mock() for name in executor.names}
    adapter.stack = Mock()
    adapter._publish_states = Mock()
    adapter.tick()
    for handle in adapter.handles.values():
        handle.cmdFullState.assert_called_once()
        assert handle.cmdFullState.call_args.args[1] == pytest.approx((0.06, 0.0, 0.0))
    adapter.stack.sleep.assert_not_called()
    adapter.stack.stream_full_state.assert_not_called()
