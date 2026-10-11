"""온라인 궤적 모드의 공유 설정, 좌표와 실행 인자."""

import argparse
import math
import re
import sys
from pathlib import Path

import yaml

from .config import point


def config_directory():
    local = Path(__file__).resolve().parents[2] / "config"
    if local.is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("crazyflie_mapf")) / "config"


def read_mapping(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f"YAML 매핑이 필요합니다: {path}")
    return data


def load_dmpc_settings(path):
    defaults_path = config_directory() / "planner.yaml"
    defaults = yaml.safe_load(defaults_path.read_text())
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) - set(defaults):
        raise ValueError("DMPC 설정은 YAML 매핑이어야 합니다.")
    settings = dict(defaults, **data)
    for name, default in defaults.items():
        if isinstance(default, (int, float)):
            value = settings[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or (value < 0 if name == "collision_margin" else value <= 0):
                raise ValueError(f"{name}: 유한한 양수가 필요합니다.")
    if type(settings["exchange_iterations"]) is not int:
        raise ValueError("exchange_iterations는 양의 정수입니다.")
    if not isinstance(settings["frame"], str) or not settings["frame"].strip("/"):
        raise ValueError("공통 frame이 필요합니다.")
    lo, hi = map(point, settings["bounds"])
    if any(a >= b for a, b in zip(lo, hi)):
        raise ValueError("잘못된 DMPC 지도 범위입니다.")
    if settings["dt"] > settings["horizon"] or abs(settings["horizon"] / settings["dt"] - round(settings["horizon"] / settings["dt"])) > 1e-8:
        raise ValueError("horizon은 dt의 정수 배수여야 합니다.")
    if not lo[2] <= settings["takeoff_height"] <= hi[2]:
        raise ValueError("이륙 높이는 계획 공간 안에 있어야 합니다.")
    if settings["command_ttl"] <= 1 / settings["plan_rate"]:
        raise ValueError("명령 유효 기간은 계획 주기보다 길어야 합니다.")
    return settings


def load_robot_config(path):
    data = read_mapping(path)
    robots, types = data.get("robots"), data.get("robot_types")
    if not isinstance(robots, dict) or not robots or not isinstance(types, dict):
        raise ValueError("robots와 robot_types 매핑이 필요합니다.")
    addresses = set()
    active = 0
    for name, robot in robots.items():
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
            raise ValueError("기체 이름은 영문자로 시작하는 영문/숫자/밑줄이어야 합니다.")
        if not isinstance(robot, dict) or type(robot.get("enabled")) is not bool:
            raise ValueError(f"{name}: enabled는 bool이어야 합니다.")
        point(robot.get("initial_position"))
        if robot.get("type") not in types:
            raise ValueError(f"{name}: robot_types에 없는 기체 유형입니다.")
        uri = robot.get("uri")
        if not isinstance(uri, str) or not uri.strip():
            raise ValueError(f"{name}: uri가 필요합니다.")
        if robot["enabled"]:
            if uri in addresses:
                raise ValueError("활성 기체의 uri가 중복됩니다.")
            addresses.add(uri)
            active += 1
    if not active:
        raise ValueError("활성 기체가 필요합니다.")
    return data


def load_robot_positions(path):
    data = load_robot_config(path)
    return {name: point(robot["initial_position"])
            for name, robot in data["robots"].items() if robot["enabled"]}


def load_experiment(robots_path, planner_path, goals_path, obstacles_path=None):
    """Validate the complete experiment before launching any simulator processes."""
    from ..fleet.functions import FleetManager

    robots = load_robot_config(robots_path)
    settings = load_dmpc_settings(planner_path)
    from .obstacles import load_obstacles, validate_takeoff
    if obstacles_path is not None:
        settings["obstacles"] = load_obstacles(obstacles_path, settings["frame"])
    homes = {name: point(r["initial_position"])
             for name, r in robots["robots"].items() if r["enabled"]}
    frame = settings["frame"]
    if robots.get("all", {}).get("reference_frame") != frame:
        raise ValueError("robots/planner의 reference_frame과 frame이 같아야 합니다.")
    for name in homes:
        robot = robots["robots"][name]
        kind = robots["robot_types"][robot["type"]]
        if kind.get("connection", "crazyflie") != "crazyflie":
            raise ValueError("활성 기체는 crazyflie connection이어야 합니다.")
        if robot.get("reference_frame", kind.get("reference_frame", frame)) != frame:
            raise ValueError("모든 활성 기체는 공통 frame을 사용해야 합니다.")
        low, high = settings["bounds"]
        if not all(low[i] <= homes[name][i] <= high[i] for i in (0, 1)) or not 0 <= homes[name][2] <= settings["takeoff_height"]:
            raise ValueError(f"{name}: 초기 위치는 XY 경계 안, 지면~이륙 높이 사이여야 합니다.")
    data = read_mapping(goals_path)
    if data.get("frame") != frame or not isinstance(data.get("goals"), dict):
        raise ValueError("goals의 frame과 목적지 매핑을 확인하세요.")
    if set(data["goals"]) - set(robots["robots"]):
        raise ValueError("goals에 등록되지 않은 기체가 있습니다.")
    if set(homes) - set(data["goals"]):
        raise ValueError("모든 활성 기체의 목적지가 필요합니다.")
    goals = FleetManager(homes, settings).validate_goals({r: data["goals"][r] for r in homes})
    # Takeoff is vertical: reject starting columns that already violate separation.
    airborne = {r: (*p[:2], settings["takeoff_height"]) for r, p in homes.items()}
    FleetManager(homes, settings).validate_goals(airborne)
    for home in homes.values():
        validate_takeoff(home, settings)
    return robots, settings, homes, goals


def trajectory_options(args=None):
    from rclpy.utilities import remove_ros_args
    config = config_directory()
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(config / "planner.yaml"))
    parser.add_argument("--robots", default=str(config / "robots.yaml"))
    parser.add_argument("--goals", default=str(config / "goals.yaml"))
    parser.add_argument("--obstacles", default=str(config / "obstacles_static.yaml"))
    parser.add_argument("--preview", action="store_true")
    return parser.parse_args(remove_ros_args(sys.argv)[1:] if args is None else args)
