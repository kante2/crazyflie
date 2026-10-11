"""Configured static/moving boxes and conservative enclosing collision ellipsoids."""

import math
import re
from pathlib import Path

import numpy as np
import yaml

from .config import point


def load_obstacles(path, frame):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {'frame', 'obstacles'} or data['frame'] != frame:
        raise ValueError(f'{path}: frame과 obstacles를 확인하세요.')
    if not isinstance(data['obstacles'], list):
        raise ValueError('obstacles는 목록이어야 합니다.')
    names, result = set(), []
    for item in data['obstacles']:
        if not isinstance(item, dict) or set(item) - {'name', 'enabled', 'type', 'position', 'size', 'motion'}:
            raise ValueError('장애물 설정에 지원하지 않는 항목이 있습니다.')
        name = item.get('name')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) or name in names:
            raise ValueError('장애물 이름은 중복 없는 영문/숫자/밑줄이어야 합니다.')
        names.add(name)
        if type(item.get('enabled', True)) is not bool or item.get('type') != 'box':
            raise ValueError('장애물은 type: box, enabled: true/false를 사용하세요.')
        position, size = point(item.get('position')), point(item.get('size'))
        if min(size) <= 0 or position[2] - size[2]/2 < -1e-8:
            raise ValueError('장애물 크기는 양수이며 바닥은 지면 이상이어야 합니다.')
        obstacle = dict(name=name, position=position, size=size)
        if 'motion' in item:
            motion = item['motion']
            if (not isinstance(motion, dict) or set(motion) != {'type', 'end_position', 'period', 'start_time'}
                    or motion['type'] != 'ping_pong'):
                raise ValueError('motion: type, end_position, period, start_time이 필요합니다.')
            end = point(motion['end_position'])
            for key in ('period', 'start_time'):
                value = motion[key]
                if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                    raise ValueError(f'motion.{key}: 유한한 숫자가 필요합니다.')
            if motion['period'] < .1 or motion['start_time'] < 0 or end[2]-size[2]/2 < -1e-8:
                raise ValueError('period >= 0.1, start_time >= 0, 지면 이상의 이동 경로가 필요합니다.')
            if end == position:
                raise ValueError('동적 장애물의 시작과 끝 위치가 같습니다.')
            obstacle['motion'] = dict(motion, end_position=end)
        if item.get('enabled', True):
            result.append(obstacle)
    return result


def motion_state(obstacle, stamp):
    """Exact position and right-hand velocity on a periodic linear ping-pong path.

    stamp is shared ROS time (simulation seconds in SIL); the obstacle waits at
    position until start_time. Direction changes are instantaneous by design.
    """
    start = np.asarray(obstacle['position'], dtype=float)
    motion = obstacle.get('motion')
    if not motion or stamp < motion['start_time']:
        return start, np.zeros(3)
    half = motion['period']/2
    phase = (stamp-motion['start_time']) % motion['period']
    delta = np.asarray(motion['end_position'])-start
    if phase < half:
        return start+delta*phase/half, delta/half
    return start+delta*(2-phase/half), -delta/half


def motion_breaks(obstacle, start, end):
    """Partition an interval at every start/reversal; each part is linear."""
    motion = obstacle.get('motion')
    cuts = [start, end]
    if motion:
        epoch, half = motion['start_time'], motion['period']/2
        first = max(0, math.floor((start-epoch)/half)+1)
        last = math.ceil((end-epoch)/half)
        cuts.extend(epoch+k*half for k in range(first, last) if start < epoch+k*half < end)
    return sorted(cuts)


def swept_envelope(obstacle):
    """Conservative whole-motion box for permanent goals and takeoff columns."""
    if 'motion' not in obstacle:
        return obstacle
    start, end = np.asarray(obstacle['position']), np.asarray(obstacle['motion']['end_position'])
    return dict(position=(start+end)/2, size=np.asarray(obstacle['size'])+np.abs(end-start))


def geometry(obstacle, radius, margin, uncertainty=0., stamp=0.):
    # The ellipsoid encloses the ENTIRE inflated box, including its corners.
    # sqrt(3) is essential: simply using half extents would inscribe it.
    axes = np.sqrt(3) * (np.asarray(obstacle['size'])/2 + radius + margin + uncertainty)
    return motion_state(obstacle, stamp)[0], 1/axes


def validate_position(position, settings):
    for obstacle in settings.get('obstacles', []):
        center, scale = geometry(swept_envelope(obstacle), settings.get('robot_radius', .18), settings['collision_margin'])
        if np.linalg.norm((np.asarray(position)-center)*scale) <= 1:
            raise ValueError(f"목적지가 장애물 {obstacle['name']}의 안전 영역과 겹칩니다.")


def validate_takeoff(home, settings):
    for obstacle in settings.get('obstacles', []):
        center, scale = geometry(swept_envelope(obstacle), settings.get('robot_radius', .18), settings['collision_margin'])
        closest = np.asarray([home[0], home[1], np.clip(center[2], home[2], settings['takeoff_height'])])
        if np.linalg.norm((closest-center)*scale) <= 1:
            raise ValueError(f"이륙 경로가 장애물 {obstacle['name']}의 안전 영역과 겹칩니다.")


def box_sdf(obstacle):
    size = ' '.join(map(str, obstacle['size']))
    pose = ' '.join(map(str, obstacle['position'])) + ' 0 0 0'
    shape = f'<geometry><box><size>{size}</size></box></geometry>'
    return (f'<sdf version="1.9"><model name="obstacle_{obstacle["name"]}"><static>true</static>'
            f'<pose>{pose}</pose><link name="body"><collision name="box">{shape}</collision>'
            f'<visual name="box">{shape}<material><ambient>0.8 0.35 0.12 1</ambient>'
            '<diffuse>0.8 0.35 0.12 1</diffuse></material></visual></link></model></sdf>')
