"""Numerical closed-loop DMPC checks, with independent dense swept checks."""

from itertools import combinations

import numpy as np
import pytest

from crazyflie_mapf.trajectory.functions import (
    MultiDroneTrajectoryPlanner, sample_trajectory,
)


def states(positions, now=0.0, velocities=None):
    return {name: {'position': list(position), 'velocity': list((velocities or {}).get(name, (0,0,0))),
                   'stamp': now, 'frame': 'world'} for name, position in positions.items()}


def independent_swept_check(batch, horizontal=.52, vertical=1.1):
    """Sample the continuous quadratic directly, independently of planner code."""
    trajectories = batch['trajectories']
    scale = np.asarray([1/horizontal, 1/horizontal, 1/vertical])
    minimum = np.inf
    for first, second in combinations(trajectories, 2):
        a, b = trajectories[first], trajectories[second]
        for k, start in enumerate(a['times'][:-1]):
            duration = a['times'][k+1]-start
            t = np.linspace(0, duration, 81)[:, None]
            pa = np.asarray(a['positions'][k])+t*np.asarray(a['velocities'][k])+.5*t*t*np.asarray(a['accelerations'][k])
            pb = np.asarray(b['positions'][k])+t*np.asarray(b['velocities'][k])+.5*t*t*np.asarray(b['accelerations'][k])
            minimum = min(minimum, np.min(np.linalg.norm((pa-pb)*scale, axis=1)))
    assert minimum >= 1-2e-4
    return minimum


def assert_dynamics(batch, speed=2, acceleration=.6):
    for trajectory in batch['trajectories'].values():
        p, v, a = (np.asarray(trajectory[key]) for key in ('positions', 'velocities', 'accelerations'))
        dt = np.diff(trajectory['times'])[:, None]
        np.testing.assert_allclose(p[1:], p[:-1]+dt*v[:-1]+.5*dt*dt*a[:-1], atol=2e-7)
        np.testing.assert_allclose(v[1:], v[:-1]+dt*a[:-1], atol=2e-7)
        assert np.max(np.linalg.norm(v, axis=1)) <= speed+2e-4
        assert np.max(np.linalg.norm(a, axis=1)) <= acceleration+2e-4
        assert np.linalg.norm(v[-1]) < 2e-4


def simulate(positions, goals, change=None, perturb=None):
    planner = MultiDroneTrajectoryPlanner()
    measured = states(positions)
    counts, resets, constraints = {}, set(), 0
    for tick in range(150):
        now = tick*.2
        for state in measured.values():
            state['stamp'] = now
        if change and tick == change[0]:
            goals = dict(goals, **change[1])
        if perturb and tick == perturb[0]:
            for name, offset in perturb[1].items():
                measured[name]['position'] = (np.asarray(measured[name]['position'])+offset).tolist()
        batch = planner.plan(measured, goals, now)
        assert batch['status'] in ('SOLVED', 'BRAKING'), batch['diagnostics']
        assert set(batch['trajectories']) == set(measured)
        assert batch['issued_at'] == now
        assert batch['valid_until'] > now
        independent_swept_check(batch)
        assert_dynamics(batch, acceleration=.8 if batch['status'] == 'BRAKING' else .6)
        counts[batch['status']] = counts.get(batch['status'], 0)+1
        resets.update(batch['diagnostics']['event_resets'])
        constraints += batch['diagnostics']['collision_constraints']
        for name, trajectory in batch['trajectories'].items():
            # All crossing experiments remain at the requested common altitude.
            np.testing.assert_allclose(np.asarray(trajectory['positions'])[:,2], 1.2, atol=1e-5)
            measured[name].update(sample_trajectory(trajectory, .2))
        if (max(np.linalg.norm(np.asarray(measured[n]['position'])-goals[n]) for n in measured) < .1
            and max(np.linalg.norm(s['velocity']) for s in measured.values()) < .15):
            return tick+1, counts, resets, constraints
    pytest.fail(f'did not complete: {measured}; {counts}')


@pytest.mark.parametrize('positions', [
    {'cf1': (-2,0,1.2), 'cf2': (2,0,1.2)},
    {'cf1': (-2,0,1.2), 'cf2': (0,-2,1.2)},
    {'cf1': (-2,0,1.2), 'cf2': (0,-2,1.2), 'cf3': (2,0,1.2), 'cf4': (0,2,1.2)},
    {'cf1': (-1.5,-1.5,1.2), 'cf2': (1.5,-1.5,1.2), 'cf3': (1.5,1.5,1.2), 'cf4': (-1.5,1.5,1.2)},
])
def test_two_and_four_drone_closed_loop_crossings(positions):
    goals = {name: (-x,-y,z) for name, (x,y,z) in positions.items()}
    ticks, counts, _, constraints = simulate(positions, goals)
    assert ticks < 100
    assert counts['SOLVED'] >= 1
    # The hardest symmetric four-agent cross must exercise collision QPs,
    # rather than only relying on the planar guide.
    if len(positions) == 4 and positions['cf1'][1] == 0:
        assert constraints > 0


def test_online_goal_change_and_tracking_disturbance():
    positions = {'cf1':(-2,0,1.2), 'cf2':(0,-2,1.2), 'cf3':(2,0,1.2), 'cf4':(0,2,1.2)}
    goals = {name:(-x,-y,z) for name,(x,y,z) in positions.items()}
    ticks, _, resets, _ = simulate(positions, goals,
        change=(6, {'cf1':(2,1,1.2)}), perturb=(4, {'cf2':[-.25,0,0]}))
    assert ticks < 100
    assert {'cf1','cf2'} <= resets


def test_prediction_uses_measured_velocity_and_synchronizes_timestamps():
    planner = MultiDroneTrajectoryPlanner()
    measured = states({'cf1':(-2,0,1.2)}, now=.9, velocities={'cf1':(.5,0,0)})
    batch = planner.plan(measured, {'cf1':(2,0,1.2)}, now=1.0)
    assert batch['status'] == 'SOLVED'
    trajectory = batch['trajectories']['cf1']
    np.testing.assert_allclose(trajectory['positions'][0], (-1.95,0,1.2))
    np.testing.assert_allclose(trajectory['velocities'][0], (.5,0,0))
    sample = sample_trajectory(trajectory, .1)
    acceleration = np.asarray(trajectory['accelerations'][0])
    np.testing.assert_allclose(sample['position'], [-1.95,0,1.2]+np.asarray([.05,0,0])+.005*acceleration)


def test_braking_fallback_preserves_velocity_and_stopping_distance():
    planner = MultiDroneTrajectoryPlanner({'exchange_iterations':1})
    measured = states({'cf1':(-2,0,1.2)}, velocities={'cf1':(2.4,0,0)})
    batch = planner.plan(measured, {'cf1':(2,0,1.2)}, 0)
    assert batch['status'] == 'BRAKING'
    trajectory = batch['trajectories']['cf1']
    assert trajectory['velocities'][0][0] == 2.4
    assert trajectory['positions'][1][0] > -2
    assert trajectory['accelerations'][0][0] == pytest.approx(-.8)
    assert trajectory['positions'][-1][0] == pytest.approx(-2+2.4**2/(2*.8))
    assert np.linalg.norm(trajectory['velocities'][-1]) < 1e-10


def test_unavoidable_crossing_never_publishes_fake_safe_hold():
    planner = MultiDroneTrajectoryPlanner()
    measured = states({'cf1':(-.35,0,1.2), 'cf2':(.35,0,1.2)},
                      velocities={'cf1':(2,0,0),'cf2':(-2,0,0)})
    batch = planner.plan(measured, {'cf1':(2,0,1.2),'cf2':(-2,0,1.2)}, 0)
    assert batch['status'] == 'UNSAFE'
    assert batch['diagnostics']['min_scaled_distance'] < 1
    assert batch['trajectories']['cf1']['velocities'][0][0] == 2


@pytest.mark.parametrize('mutate', [
    lambda s: s['cf1'].update(stamp=-1),
    lambda s: s['cf1'].update(stamp=1),
    lambda s: s['cf1'].update(velocity=[float('nan'),0,0]),
    lambda s: s['cf1'].update(frame='map'),
    lambda s: s['cf1'].update(radius=-.1),
])
def test_stale_or_invalid_states_fail_closed(mutate):
    planner = MultiDroneTrajectoryPlanner()
    measured = states({'cf1':(-2,0,1.2), 'cf2':(2,0,1.2)})
    mutate(measured)
    batch = planner.plan(measured, {'cf1':(2,0,1.2),'cf2':(-2,0,1.2)}, 0)
    assert batch['status'] == 'STALE'
    assert batch['trajectories'] == {}


def test_downwash_vertical_clearance_and_passage():
    planner = MultiDroneTrajectoryPlanner()
    # Same XY with 0.8m vertical separation is geometrically invalid despite
    # exceeding the horizontal 0.52m spacing rule.
    measured = states({'cf1':(0,0,1.2),'cf2':(0,0,2.0)})
    assert planner.plan(measured, {n:s['position'] for n,s in measured.items()}, 0)['status'] == 'UNSAFE'
    # Overpasses with 1.4m height difference satisfy the anisotropic volume.
    measured = states({'cf1':(-2,0,1),'cf2':(0,-2,2.4)})
    batch = planner.plan(measured, {'cf1':(2,0,1),'cf2':(0,2,2.4)}, 0)
    assert batch['status'] == 'SOLVED'
    independent_swept_check(batch)


def test_goal_ids_are_preserved_in_atomic_batch():
    planner = MultiDroneTrajectoryPlanner()
    measured = states({'cf1':(-2,0,1.2)})
    goals = {'cf1':{'position':(2,0,1.2),'goal_id':'change-7'}}
    batch = planner.plan(measured, goals, 0)
    assert batch['trajectories']['cf1']['goal_id'] == 'change-7'
    assert batch['sequence'] == 1
    assert planner.plan(measured, goals, .1)['sequence'] == 2


def test_braking_fallback_uses_the_shared_clock_including_measurement_latency():
    planner = MultiDroneTrajectoryPlanner()
    measured = states({'cf1':(-2,0,1.2)}, now=.9, velocities={'cf1':(2.4,0,0)})
    batch = planner.plan(measured, {'cf1':(2,0,1.2)}, 1.0)
    assert batch['status'] == 'BRAKING'
    np.testing.assert_allclose(batch['trajectories']['cf1']['positions'][0], (-1.76,0,1.2))
    np.testing.assert_allclose(batch['trajectories']['cf1']['velocities'][0], (2.4,0,0))


def test_clock_extrapolation_uncertainty_inflates_pair_safety_volume():
    planner = MultiDroneTrajectoryPlanner()
    positions = {'cf1':(-.325,0,1.2), 'cf2':(.325,0,1.2)}
    fresh = planner.plan(states(positions), positions, 0)
    assert fresh['status'] == 'SOLVED'
    # Both positions are old, but have not reached the stale timeout. Their
    # unobserved acceleration can have moved them into each other's volume.
    aged = planner.plan(states(positions, now=-.49), positions, 0)
    assert aged['status'] == 'UNSAFE'
    assert aged['diagnostics']['min_scaled_distance'] < 1


def test_normal_tracking_bias_preserves_nominal_reference_until_event_reset():
    planner = MultiDroneTrajectoryPlanner()
    goals = {'cf1':(2,0,1.2)}
    first = planner.plan(states({'cf1':(-2,0,1.2)}), goals, 0)
    previous = first
    for tick in range(1, 8):
        now = tick*.2
        nominal = sample_trajectory(previous['trajectories']['cf1'], .2)
        # A fixed low altitude tracking bias must not become a new commanded
        # altitude every cycle. Normal tracking keeps the accepted input alive.
        measured = states({'cf1':np.asarray(nominal['position'])+[0,0,-.05]}, now,
                          {'cf1':nominal['velocity']})
        batch = planner.plan(measured, goals, now)
        assert batch['status'] == 'SOLVED'
        assert batch['diagnostics']['initialization']['cf1'] == 'nominal'
        np.testing.assert_allclose(batch['trajectories']['cf1']['positions'][0], nominal['position'], atol=1e-8)
        np.testing.assert_allclose(np.asarray(batch['trajectories']['cf1']['positions'])[:,2], 1.2, atol=1e-6)
        previous = batch
    disturbed = states({'cf1':np.asarray(nominal['position'])+[0,0,-.2]}, now+.2,
                       {'cf1':nominal['velocity']})
    reset = planner.plan(disturbed, goals, now+.2)
    assert reset['diagnostics']['initialization']['cf1'] == 'measured'
    assert 'cf1' in reset['diagnostics']['event_resets']
    np.testing.assert_allclose(reset['trajectories']['cf1']['positions'][0], disturbed['cf1']['position'])


def test_waiting_envelope_advances_sequence_without_unsafe_commands():
    planner = MultiDroneTrajectoryPlanner()
    waiting = planner.waiting(0, 'not all robots airborne')
    assert waiting['status'] == 'WAITING'
    assert waiting['trajectories'] == {}
    assert waiting['sequence'] == 1
    assert waiting['diagnostics']['reason'] == 'not all robots airborne'
    assert planner.waiting(.2)['sequence'] == 2
