"""Independent measured separation and prior-reference tracking statistics."""

import json
import math

import pytest

from crazyflie_mapf.trajectory.metrics import FleetMetrics


def settings():
    return {"frame": "world", "horizontal_separation": .52,
            "vertical_separation": 1.1, "state_timeout": .5}


def state(position=(0, 0, 1.2), velocity=(0, 0, 0), stamp=0.0, **extra):
    return dict(position=position, velocity=velocity, stamp=stamp, frame="world", **extra)


def batch(stamp=0.0, sequence=1, solve_ms=10.0, offset=0.0, status="SOLVED"):
    # Analytic p(t) = offset + t + t² / 2, v(t) = 1 + t.
    return dict(stamp=stamp, sequence=sequence, status=status, frame="world",
                diagnostics={"solve_ms": solve_ms}, trajectories={"cf1": {
                    "times": [0.0, 1.0, 2.0],
                    "positions": [[offset, 0, 1.2], [offset + 1.5, 0, 1.2], [offset + 3.5, 0, 1.2]],
                    "velocities": [[1, 0, 0], [2, 0, 0], [2, 0, 0]],
                    "accelerations": [[1, 0, 0], [0, 0, 0], [0, 0, 0]],
                }})


def test_measured_horizontal_vertical_and_running_minima():
    metrics = FleetMetrics(settings())
    first = metrics.observe({"cf1": state(), "cf2": state((.52, 0, 1.2))})
    assert first["min_distance_m"] == pytest.approx(.52)
    assert first["min_scaled_distance"] == pytest.approx(1.0)
    second = metrics.observe({"cf1": state(stamp=1), "cf2": state((0, 0, 2), stamp=1)})
    assert second["current_min_distance_m"] == pytest.approx(.8)
    assert second["current_min_scaled_distance"] == pytest.approx(.8 / 1.1)
    assert second["min_distance_m"] == pytest.approx(.52)
    assert second["min_scaled_distance"] == pytest.approx(.8 / 1.1)


def test_scaled_measurement_encloses_larger_robot_radii():
    snapshot = FleetMetrics(settings()).observe({
        "cf1": state(radius=.4), "cf2": state((.8, 0, 1.2), radius=.4)})
    assert snapshot["current_min_scaled_distance"] == pytest.approx(1.0)


def test_tracking_samples_previous_plan_at_measurement_stamp_before_new_plan():
    metrics = FleetMetrics(settings())
    first = metrics.observe({"cf1": state(velocity=(1, 0, 0))}, batch())
    assert first["tracking_samples"] == 0
    observed = state((.725, 0, 1.2), (1.6, 0, 0), stamp=.5)
    # New prediction begins exactly at this measured position. It must not hide
    # the 0.1 m / 0.1 m/s mismatch with the previously executed prediction.
    result = metrics.observe({"cf1": observed}, batch(.5, 2, offset=.725), now=.6)
    assert result["tracking_error_m"]["cf1"] == pytest.approx(.1)
    assert result["tracking_velocity_error_mps"]["cf1"] == pytest.approx(.1)
    assert result["tracking_max_m"] == pytest.approx(.1)
    assert result["tracking_rms_m"] == pytest.approx(.1)


def test_tracking_rms_counts_each_robot_measurement_once():
    metrics = FleetMetrics(settings())
    metrics.observe({}, batch())
    metrics.observe({"cf1": state((.725, 0, 1.2), (1.5, 0, 0), stamp=.5)})
    repeated = metrics.observe({"cf1": state((.725, 0, 1.2), (1.5, 0, 0), stamp=.5)})
    assert repeated["tracking_samples"] == 1
    result = metrics.observe({"cf1": state((1.8, 0, 1.2), (2, 0, 0), stamp=1)})
    assert result["tracking_samples"] == 2
    assert result["tracking_max_m"] == pytest.approx(.3)
    assert result["tracking_rms_m"] == pytest.approx(math.sqrt((.1 ** 2 + .3 ** 2) / 2))


def test_solve_statistics_ignore_duplicate_batch_and_use_interpolated_p95():
    metrics = FleetMetrics(settings())
    metrics.observe({}, batch(solve_ms=10))
    duplicate = metrics.observe({}, batch(solve_ms=10))
    assert duplicate["solve_ms"]["count"] == 1
    second = metrics.observe({}, batch(.2, 2, solve_ms=30), now=.2)
    assert second["solve_ms"] == pytest.approx(dict(latest=30, mean=20, p95=29, max=30, count=2))


def test_backward_clock_starts_new_measurement_epoch():
    metrics = FleetMetrics(settings())
    metrics.observe({"cf1": state(stamp=10), "cf2": state((.6, 0, 1.2), stamp=10)}, batch(10), now=10)
    reset = metrics.observe({"cf1": state(stamp=0)}, batch(0), now=0)
    assert reset["clock_resets"] == 1
    assert reset["min_distance_m"] is None
    assert reset["tracking_samples"] == 0
    assert reset["solve_ms"]["count"] == 1


def test_invalid_stale_frame_and_out_of_order_states_do_not_poison_metrics():
    metrics = FleetMetrics(settings())
    metrics.observe({"cf1": state(stamp=1)}, now=1)
    invalid = {
        "cf1": state(stamp=.9),
        "cf2": state((float("nan"), 0, 1.2), stamp=1),
        "cf3": dict(state(stamp=1), frame="map"),
        "cf4": state(stamp=0),
    }
    result = metrics.observe(invalid, now=1)
    assert result["measured_robots"] == 0
    assert set(result["ignored_states"]) == set(invalid)
    assert result["current_min_distance_m"] is None
    json.dumps(result, allow_nan=False)


def test_future_states_and_invalid_compute_times_are_ignored():
    result = FleetMetrics(settings()).observe({"cf1": state(stamp=1)}, batch(solve_ms=float("nan")), now=0)
    assert result["measured_robots"] == 0
    assert result["solve_ms"]["count"] == 0
    json.dumps(result, allow_nan=False)


def test_failed_plan_does_not_replace_executable_tracking_reference():
    metrics = FleetMetrics(settings())
    metrics.observe({}, batch())
    metrics.observe({}, batch(.2, 2, offset=99, status="UNSAFE"), now=.2)
    result = metrics.observe({"cf1": state((.625, 0, 1.2), (1.5, 0, 0), stamp=.5)})
    assert result["tracking_error_m"]["cf1"] == pytest.approx(0)
    assert result["solve_ms"]["count"] == 2


def test_delayed_older_batch_does_not_rewind_reference_or_compute_statistics():
    metrics = FleetMetrics(settings())
    metrics.observe({}, batch(.2, 2, offset=.25), now=.2)
    metrics.observe({}, batch(0, 1, offset=99, solve_ms=999), now=.3)
    result = metrics.observe({"cf1": state((.595, 0, 1.2), (1.3, 0, 0), stamp=.5)})
    assert result["tracking_error_m"]["cf1"] == pytest.approx(0)
    assert result["solve_ms"]["count"] == 1


def test_waiting_for_takeoff_is_excluded_from_planning_time_statistics():
    result = FleetMetrics(settings()).observe({}, batch(status="WAITING", solve_ms=0))
    assert result["solve_ms"]["count"] == 0
