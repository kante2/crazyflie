from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_mapf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML', 'osqp>=1.0,<2'],
    zip_safe=True,
    description='Independent multi-drone DMPC simulation and PBS priority-search library.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'priority_demo = crazyflie_mapf.demo:main',
        'fleet_manager = crazyflie_mapf.fleet.main:main',
        'trajectory_planner = crazyflie_mapf.trajectory.main:main',
        'drone_executor = crazyflie_mapf.executor.main:main',
        'gazebo_pose_bridge = crazyflie_mapf.executor.gazebo_pose_bridge:main',
    ]},
)
