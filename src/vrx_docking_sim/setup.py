import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vrx_docking_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yvan Eustache',
    maintainer_email='yvan.eustache@gmail.com',
    description='Complete docking simulator: USV/dock ROS 2 nodes, UDP dock link, ArduPilot SITL, Mission Planner, RC joystick, APF vectors',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dock_pose_source_node = vrx_docking_sim.dock_pose_source_node:main',
            'dock_udp_broadcaster_node = vrx_docking_sim.dock_udp_broadcaster_node:main',
            'dock_udp_receiver_node = vrx_docking_sim.dock_udp_receiver_node:main',
            'boat_pose_node = vrx_docking_sim.boat_pose_node:main',
            'apf_controller_node = vrx_docking_sim.apf_controller_node:main',
            'mavros_setpoint_bridge_node = vrx_docking_sim.mavros_setpoint_bridge_node:main',
            'joystick_rc_node = vrx_docking_sim.joystick_rc_node:main',
            'vector_marker_node = vrx_docking_sim.vector_marker_node:main',
            'apf_field_marker_node = vrx_docking_sim.apf_field_marker_node:main',
        ],
    },
)
