"""Static obstacle input, continuous collision gating and closed-loop detours."""

from itertools import product
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from crazyflie_mapf.common.obstacles import (
    box_sdf, geometry, load_obstacles, validate_takeoff,
)
from crazyflie_mapf.common.trajectory import config_directory, load_experiment
from crazyflie_mapf.fleet.functions import FleetManager
from crazyflie_mapf.trajectory.functions import MultiDroneTrajectoryPlanner, sample_trajectory


def experiment():
    p = config_directory()
    return load_experiment(*(p / f for f in ('robots.yaml', 'planner.yaml', 'goals.yaml', 'obstacles_static.yaml')))


@pytest.mark.parametrize('change', [
    lambda d: d.update(frame='map'),
    lambda d: d['obstacles'][0].update(size=[0, 1, 1]),
    lambda d: d['obstacles'][0].update(position=[0, 0, float('nan')]),
    lambda d: d['obstacles'][0].update(position=[0, 0, -.1]),
    lambda d: d['obstacles'][0].update(enabled='false'),
    lambda d: d['obstacles'][0].update(type='sphere'),
    lambda d: d['obstacles'][0].update(yaw=1),
    lambda d: d['obstacles'].append(dict(d['obstacles'][0])),
])
def test_invalid_obstacle_input(tmp_path, change):
    data = yaml.safe_load((config_directory() / 'obstacles_static.yaml').read_text())
    change(data)
    path = tmp_path / 'obstacles_static.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_obstacles(path, 'world')


def test_disabled_and_empty_obstacles(tmp_path):
    data = yaml.safe_load((config_directory() / 'obstacles_static.yaml').read_text())
    data['obstacles'][0]['enabled'] = False
    path = tmp_path / 'obstacles_static.yaml'
    path.write_text(yaml.safe_dump(data))
    assert load_obstacles(path, 'world') == []
    data['obstacles'] = []
    path.write_text(yaml.safe_dump(data))
    assert load_obstacles(path, 'world') == []


def test_box_corners_inside_safety_ellipsoid_and_sdf_matches_config():
    _, cfg, _, _ = experiment()
    box = cfg['obstacles'][0]
    center, scale = geometry(box, cfg['robot_radius'], cfg['collision_margin'])
    for signs in product((-1, 1), repeat=3):
        inflated_corner = np.asarray(signs) * (np.asarray(box['size'])/2 + .24)
        assert np.linalg.norm(inflated_corner*scale) <= 1+1e-12
    model = ET.fromstring(box_sdf(box)).find('model')
    assert model.findtext('static') == 'true'
    assert list(map(float, model.findtext('pose').split()[:3])) == list(center)
    for kind in ('visual', 'collision'):
        assert list(map(float, model.findtext(f'link/{kind}/geometry/box/size').split())) == list(box['size'])


def test_takeoff_and_online_goals_reject_obstacles():
    _, cfg, homes, goals = experiment()
    with pytest.raises(ValueError, match='이륙'):
        validate_takeoff((0, 0, 0), cfg)
    manager = FleetManager(homes, cfg)
    manager.assign_goals(goals, 0)
    for enqueue in (False, True):
        with pytest.raises(ValueError, match='장애물'):
            manager.request_goal('cf1', (0, 0, 1.2), 1, enqueue=enqueue)


def test_continuous_obstacle_check_detects_between_knot_crossing():
    _, cfg, _, _ = experiment()
    planner = MultiDroneTrajectoryPlanner(cfg)
    forecast = {'positions': np.tile([2., 0., 1.2], (planner.steps+1, 1)),
                'velocities': np.zeros((planner.steps+1, 3)),
                'accelerations': np.zeros((planner.steps+1, 3))}
    forecast['positions'][0] = [-2, 0, 1.2]
    forecast['velocities'][0] = [20, 0, 0]
    minimum, hits = planner._obstacle_collisions({'cf1': forecast}, {'cf1': {}})
    assert minimum < 1 and hits
    assert 0 < hits[0][2] < cfg['dt']


def test_unavoidable_obstacle_braking_is_unsafe():
    _, cfg, _, _ = experiment()
    state = dict(position=[-1., 0, 1.2], velocity=[2., 0, 0], stamp=0.)
    batch = MultiDroneTrajectoryPlanner(cfg).plan({'cf1': state}, {'cf1': [2.5, 0, 1.2]}, 0)
    assert batch['status'] == 'UNSAFE'
    assert batch['diagnostics']['min_obstacle_scaled_distance'] < 1


@pytest.mark.parametrize('count', [1, 7])
def test_default_box_closed_loop_arrival_and_independent_clearance(count):
    _, cfg, homes, goals = experiment()
    homes = dict(list(homes.items())[:count])
    goals = {n: goals[n] for n in homes}
    planner = MultiDroneTrajectoryPlanner(cfg)
    states = {n: dict(position=[*p[:2], cfg['takeoff_height']], velocity=[0, 0, 0], stamp=0.)
              for n, p in homes.items()}
    for tick in range(120):
        for state in states.values():
            state['stamp'] = tick*.2
        batch = planner.plan(states, goals, tick*.2)
        assert batch['status'] in ('SOLVED', 'BRAKING'), batch['diagnostics']
        for name, trajectory in batch['trajectories'].items():
            # Independently sample the physical inflated AABB, not planner geometry.
            for k in range(planner.steps):
                t = np.linspace(0, cfg['dt'], 21)[:, None]
                points = (trajectory['positions'][k] + t*np.asarray(trajectory['velocities'][k])
                          + .5*t*t*np.asarray(trajectory['accelerations'][k]))
                for box in cfg['obstacles']:
                    delta = np.abs(points - box['position'])
                    half = np.asarray(box['size'])/2 + cfg['robot_radius'] + cfg['collision_margin']
                    assert not np.any(np.all(delta < half-1e-4, axis=1))
            states[name].update(sample_trajectory(trajectory, .2))
        if (max(np.linalg.norm(np.asarray(states[n]['position'])-goals[n]) for n in states) < .1
                and max(np.linalg.norm(s['velocity']) for s in states.values()) < .15):
            return
    pytest.fail('obstacle detour did not reach all goals')
