"""공통 설정과 ROS 없이 검증할 수 있는 전달 데이터 검증."""

import json
import math
from pathlib import Path

import yaml


def point(value):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 3
        or any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            for v in value
        )
    ):
        raise ValueError("유한한 3D 좌표가 필요합니다.")
    return tuple(float(v) for v in value)


def encode(data):
    return json.dumps(data, ensure_ascii=False, allow_nan=False)


def decode(message):
    data = json.loads(message.data)
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError("version=1인 JSON 객체가 필요합니다.")
    return data


