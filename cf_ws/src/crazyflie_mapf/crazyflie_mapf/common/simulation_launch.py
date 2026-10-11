"""Launch only this package's experiment and the shared simulation backends."""

import os
from functools import partial
import ipaddress
from pathlib import Path
import shlex
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from crazyflie_mapf.common.obstacles import box_sdf
from crazyflie_mapf.common.trajectory import load_experiment, read_mapping


def start(context, mode):
    directory = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    paths = {key: directory / filename for key, filename in (
        ('robots', 'robots.yaml'), ('planner', 'planner.yaml'), ('goals', 'goals.yaml'), ('obstacles', f'obstacles_{mode}.yaml'))}
    robots, settings, homes, _ = load_experiment(paths['robots'], paths['planner'], paths['goals'], paths['obstacles'])
    if mode == 'none' and settings['obstacles']:
        raise ValueError('장애물 없는 모드에서는 obstacles: []를 사용하세요.')
    if mode == 'static' and any('motion' in o for o in settings['obstacles']):
        raise ValueError('정적 모드에서는 motion을 사용할 수 없습니다. dynamic_obstacles.launch.py를 사용하세요.')
    simulation = read_mapping(directory / 'simulation.yaml')
    operation = LaunchConfiguration('operation').perform(context) or simulation['operation']
    if operation not in ('run', 'preview'):
        raise ValueError('operation은 run 또는 preview입니다.')

    def flag(name):
        override = LaunchConfiguration(name).perform(context)
        if override:
            if override.lower() not in ('true', 'false'):
                raise ValueError(f'{name}은 true/false입니다.')
            return override.lower() == 'true'
        if type(simulation[name]) is not bool:
            raise ValueError(f'{name}은 YAML bool이어야 합니다.')
        return simulation[name]

    gazebo, gazebo_gui, rviz = flag('gazebo'), flag('gazebo_gui'), flag('rviz')
    actions = []
    if gazebo and operation == 'run':
        # All Gazebo processes in this local SIL experiment share one interface.
        # Auto-detection can select docker0 instead of the active local interface.
        gazebo_ip = str(simulation.get('gazebo_ip', '127.0.0.1'))
        ipaddress.IPv4Address(gazebo_ip)
        actions.append(SetEnvironmentVariable('GZ_IP', gazebo_ip))
    if simulation['software_rendering']:
        actions += [SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
                    SetEnvironmentVariable('LIBGL_DRI3_DISABLE', '1')]
    else:
        actions.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'))
    actions.append(SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'))
    bindings = simulation.get('firmware_bindings', '')
    if bindings:
        bindings_path = (directory / bindings).resolve()
        if not bindings_path.is_dir():
            raise ValueError(f'firmware_bindings 디렉터리가 없습니다: {bindings_path}')
        actions.append(SetEnvironmentVariable('PYTHONPATH', str(bindings_path) + os.pathsep + os.environ.get('PYTHONPATH', '')))

    def add(node, stop_on_exit=True):
        if stop_on_exit:
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='MAPF process exited'))])))
        actions.append(node)

    common = ['--robots', str(paths['robots']), '--config', str(paths['planner']), '--goals', str(paths['goals']), '--obstacles', str(paths['obstacles'])]
    if operation == 'preview':
        common.append('--preview')
    else:
        description = Path(get_package_share_directory('crazyflie_description')) / 'urdf/crazyflie_description.urdf'
        server = {'sim': simulation['sim'], 'robot_description': description.read_text()}
        if not simulation['sim']['visualizations']['rviz']['enabled']:
            raise ValueError('SIL 위치 피드백을 위해 sim.visualizations.rviz.enabled=true가 필요합니다.')
        add(Node(package='crazyflie_sim', executable='crazyflie_server', name='crazyflie_server',
                 output='screen', parameters=[robots, server]))

    for executable in ['fleet_manager', 'trajectory_planner'] + (['drone_executor'] if operation == 'run' else []):
        add(Node(package='crazyflie_mapf', executable=executable, name=executable,
                 arguments=common, output='screen', parameters=[{'use_sim_time': operation == 'run'}]))

    # preview is an ideal model visualization and has no SIL/TF-driven Gazebo models.
    if gazebo and operation == 'run':
        world = (directory / simulation['world_file']).resolve()
        world_name = simulation['world_name']
        if ET.parse(world).getroot().find('world').get('name') != world_name:
            raise ValueError('simulation.yaml의 world_name과 SDF world 이름이 달라요.')
        engine = simulation['render_engine']
        if engine not in ('ogre', 'ogre2'):
            raise ValueError('render_engine은 ogre 또는 ogre2입니다.')
        gz_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_launch)),
            launch_arguments={'gz_args': f'-r -v 2 {"" if gazebo_gui else "-s "}--render-engine-gui {engine} {shlex.quote(str(world))}',
                              'on_exit_shutdown': 'True'}.items()))
        add(Node(package='ros_gz_bridge', executable='parameter_bridge', name='mapf_set_pose_bridge',
                 arguments=[f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose'], output='screen'))
        mesh = Path(get_package_share_directory('crazyflie_description')) / 'urdf/cf2_assembly_with_props.dae'
        for name, position in homes.items():
            sdf = (f'<sdf version="1.9"><model name="{name}"><static>true</static><link name="body">'
                   f'<visual name="drone"><geometry><mesh><uri>{escape(mesh.as_uri())}</uri></mesh></geometry>'
                   '</visual></link></model></sdf>')
            spawn = Node(package='ros_gz_sim', executable='create', name=f'spawn_{name}', output='screen',
                         arguments=['-world', world_name, '-name', name, '-string', sdf,
                                    '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2])])
            actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn,
                on_exit=lambda event, context: [EmitEvent(event=Shutdown(reason='Gazebo spawn failed'))] if event.returncode else [])))
            add(spawn, False)
        for obstacle in settings['obstacles']:
            name = 'obstacle_' + obstacle['name']
            spawn = Node(package='ros_gz_sim', executable='create', name='spawn_' + name,
                         arguments=['-world', world_name, '-name', name, '-string', box_sdf(obstacle)], output='screen')
            actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn,
                on_exit=lambda event, context: [EmitEvent(event=Shutdown(reason='Gazebo obstacle spawn failed'))] if event.returncode else [])))
            add(spawn, False)
        add(Node(package='crazyflie_mapf', executable='gazebo_pose_bridge', name='gazebo_pose_bridge',
                 parameters=[{'robot_names': list(homes), 'reference_frame': settings['frame'], 'world_name': world_name,
                              'obstacles_file': str(paths['obstacles']), 'use_sim_time': True}], output='screen'))
    if rviz:
        rviz_file = (directory / simulation['rviz_config']).resolve()
        if not rviz_file.is_file():
            raise ValueError(f'RViz 설정 파일이 없습니다: {rviz_file}')
        add(Node(package='rviz2', executable='rviz2', name='mapf_rviz', output='screen',
                 arguments=['-d', str(rviz_file)], parameters=[{'use_sim_time': operation == 'run'}]))
    return actions


def generate_launch_description(mode="static"):
    if mode not in ("none", "static", "dynamic"):
        raise ValueError("Unknown obstacle mode")
    config = str(Path(get_package_share_directory('crazyflie_mapf')) / 'config')
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=config, description='Folder containing this experiment configuration'),
        DeclareLaunchArgument('operation', default_value='', description='run / preview; empty uses simulation.yaml'),
        DeclareLaunchArgument('gazebo', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('gazebo_gui', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('rviz', default_value='', description='true / false; empty uses simulation.yaml'),
        OpaqueFunction(function=partial(start, mode=mode)),
    ])
