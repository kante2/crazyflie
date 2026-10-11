"""Shared-clock motion and continuous checks through direction reversals."""

from pathlib import Path
import runpy

import numpy as np
import pytest
import yaml

from crazyflie_mapf.common.obstacles import load_obstacles, motion_state, motion_breaks
from crazyflie_mapf.common.trajectory import config_directory, load_experiment
from crazyflie_mapf.trajectory.functions import MultiDroneTrajectoryPlanner, sample_trajectory


def moving_box():
    return dict(name='moving', position=[-2., 0., 1.2], size=[.2, .2, .2],
                motion=dict(type='ping_pong', end_position=[2., 0., 1.2], period=4., start_time=1.))


@pytest.mark.parametrize('stamp,x,v', [(0,-2,0), (1,-2,2), (2,0,2), (3,2,-2), (4,0,-2), (5,-2,2), (9,-2,2)])
def test_motion_uses_absolute_clock_and_reverses(stamp, x, v):
    p, velocity = motion_state(moving_box(), stamp)
    np.testing.assert_allclose(p, [x, 0, 1.2])
    np.testing.assert_allclose(velocity, [v, 0, 0])


def test_motion_breaks_include_start_and_each_turn():
    assert motion_breaks(moving_box(), .5, 7.5) == [.5, 1., 3., 5., 7., 7.5]
    assert motion_breaks(moving_box(), 1., 3.) == [1., 3.]


@pytest.mark.parametrize('field,value', [('period', 0), ('period', float('inf')), ('start_time', -1),
                                         ('start_time', True), ('type', 'unknown'), ('end_position', [0,0,-1])])
def test_invalid_motion(tmp_path, field, value):
    box = moving_box(); box['type'] = 'box'
    box['motion'][field] = value
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(dict(frame='world', obstacles=[box])))
    with pytest.raises(ValueError):
        load_obstacles(path, 'world')


def test_reversal_collision_between_knots_with_safe_endpoints():
    box = moving_box()
    box['motion'].update(period=.2, start_time=0.)
    planner = MultiDroneTrajectoryPlanner(dict(obstacles=[box]))
    forecast = dict(positions=np.tile([0.,0.,1.2], (planner.steps+1,1)),
                    velocities=np.zeros((planner.steps+1,3)), accelerations=np.zeros((planner.steps+1,3)))
    minimum, hits = planner._obstacle_collisions({'cf1': forecast}, {'cf1': {}})
    assert minimum < 1e-8
    assert hits and 0 < hits[0][2] < .2
    # It starts and ends clear; checking only knots would miss both passes.
    np.testing.assert_allclose(motion_state(box, 0)[0], motion_state(box, .2)[0])


def test_unsafe_braking_when_moving_obstacle_sweeps_stationary_drone():
    box = moving_box()
    box['motion'].update(period=.2, start_time=0.)
    planner = MultiDroneTrajectoryPlanner(dict(obstacles=[box]))
    batch = planner.plan({'cf1': dict(position=[0,0,1.2], velocity=[0,0,0], stamp=0.)},
                         {'cf1': [4.4, 0, 1.2]}, 0.)
    assert batch['status'] == 'UNSAFE'
    assert batch['diagnostics']['min_obstacle_scaled_distance'] < 1


@pytest.mark.parametrize('mode,count,moving', [('none',0,False), ('static',1,False), ('dynamic',1,True)])
def test_mode_configs_and_entrypoints(mode, count, moving):
    cfg = config_directory()
    boxes = load_obstacles(cfg / f'obstacles_{mode}.yaml', 'world')
    assert len(boxes) == count
    assert any('motion' in box for box in boxes) == moving
    name = 'no' if mode == 'none' else mode
    namespace = runpy.run_path(str(cfg.parent / 'launch' / f'{name}_obstacles.launch.py'))
    generate = namespace['generate_launch_description']
    generate.__globals__['create_launch'] = lambda selected: selected
    assert generate() == mode


@pytest.mark.parametrize('initial_time', [3.3, 9.7])
def test_seven_drones_avoid_moving_box_and_arrive(initial_time):
    cfg = config_directory()
    _, settings, homes, goals = load_experiment(*(cfg / name for name in (
        'robots.yaml', 'planner.yaml', 'goals.yaml', 'obstacles_dynamic.yaml')))
    planner = MultiDroneTrajectoryPlanner(settings)
    states = {n: dict(position=[*p[:2], 1.2], velocity=[0,0,0], stamp=initial_time) for n,p in homes.items()}
    box = settings["obstacles"][0]
    motion = box["motion"]
    for tick in range(100):
        now = initial_time+tick*.2
        for state in states.values():
            state['stamp'] = now
        batch = planner.plan(states, goals, now)
        assert batch['status'] in ('SOLVED', 'BRAKING'), batch['diagnostics']
        for name, trajectory in batch['trajectories'].items():
            # Independent formula for the example's triangular motion, sampled
            # throughout the entire accepted trajectory including braking.
            for k in range(planner.steps):
                t = np.linspace(0, .2, 15)
                absolute = now+k*.2+t
                phase = np.maximum(absolute-motion['start_time'], 0.) % motion['period']
                half_period = motion['period']/2
                fraction = np.where(phase < half_period, phase/half_period, 2-phase/half_period)
                centers = (np.asarray(box['position']) + fraction[:,None]
                           * (np.asarray(motion['end_position'])-box['position']))
                xyz = (trajectory['positions'][k] + t[:,None]*np.asarray(trajectory['velocities'][k])
                       + .5*t[:,None]**2*np.asarray(trajectory['accelerations'][k]))
                half = np.asarray(box['size'])/2+settings['robot_radius']+settings['collision_margin']
                assert not np.any(np.all(np.abs(xyz-centers) < half-1e-4, axis=1))
            states[name].update(sample_trajectory(trajectory, .2))
        if (max(np.linalg.norm(np.array(states[n]['position'])-goals[n]) for n in states) < .1
                and max(np.linalg.norm(s['velocity']) for s in states.values()) < .15):
            return
    pytest.fail('moving-obstacle experiment did not finish')
