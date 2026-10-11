"""Experiment configuration must drive all enabled agents consistently."""

from pathlib import Path
import shutil

import pytest
import yaml

from crazyflie_mapf.common.trajectory import load_experiment


@pytest.fixture
def config(tmp_path):
    source = Path(__file__).resolve().parents[1] / 'config'
    for name in ('robots.yaml', 'goals.yaml', 'planner.yaml'):
        shutil.copy2(source / name, tmp_path / name)
    return tmp_path


def edit(directory, filename, change):
    path = directory / filename
    data = yaml.safe_load(path.read_text())
    change(data)
    path.write_text(yaml.safe_dump(data))


def load(directory):
    return load_experiment(directory / 'robots.yaml', directory / 'planner.yaml', directory / 'goals.yaml')


def test_default_seven_drone_crossing(config):
    _, settings, homes, goals = load(config)
    assert set(homes) == set(goals) == {f'cf{i}' for i in range(1, 8)}
    assert goals['cf1'] == (2.5, 0.0, 1.2)
    assert settings['speed'] == 2.5


def test_explicit_goal_is_not_replaced_by_mirrored_start(config):
    edit(config, 'goals.yaml', lambda d: d['goals'].update(cf1=[2.4, 1.0, 1.2]))
    _, _, _, goals = load(config)
    assert goals['cf1'] == (2.4, 1.0, 1.2)


def test_disabled_drone_is_removed_from_homes_and_goals(config):
    edit(config, 'robots.yaml', lambda d: d['robots']['cf4'].update(enabled=False))
    _, _, homes, goals = load(config)
    assert len(homes) == 6
    assert 'cf4' not in homes and 'cf4' not in goals


@pytest.mark.parametrize('filename,change', [
    ('goals.yaml', lambda d: d['goals'].pop('cf1')),
    ('goals.yaml', lambda d: d['goals'].update(cf9=[0, 0, 1.2])),
    ('goals.yaml', lambda d: d.update(frame='wrong_frame')),
    ('goals.yaml', lambda d: d['goals'].update(cf1=[99, 0, 1.2])),
    ('goals.yaml', lambda d: d['goals'].update(cf1=d['goals']['cf2'])),
    ('robots.yaml', lambda d: d['robots']['cf1'].update(enabled='false')),
    ('robots.yaml', lambda d: d['robots']['cf1'].update(uri=d['robots']['cf2']['uri'])),
    ('robots.yaml', lambda d: d['robots']['cf1'].update(initial_position=d['robots']['cf2']['initial_position'])),
    ('robots.yaml', lambda d: d['robots']['cf1'].update(initial_position=[float('nan'), 0, 0])),
    ('planner.yaml', lambda d: d.update(command_ttl=0.01)),
])
def test_invalid_configuration_is_rejected_before_launch(config, filename, change):
    edit(config, filename, change)
    with pytest.raises(ValueError):
        load(config)
