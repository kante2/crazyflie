"""static obstacle simulation; shared implementation lives in common/."""

from crazyflie_mapf.common.simulation_launch import generate_launch_description as create_launch


def generate_launch_description():
    return create_launch('static')
