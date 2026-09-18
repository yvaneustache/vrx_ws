"""Spawns a single WAM-V flown by ArduPilot Rover SITL instead of
vrx_apf_dock's own control stack.

Only handles the Gazebo side: world + WAM-V spawn with ArduPilotPlugin
attached (see ardupilot_enabled/thruster_config:=ArduPilot in
wamv_gazebo.urdf.xacro), plus a ground-truth odometry bridge for
visualization/debugging. ArduPilot SITL and MAVROS are separate processes,
started independently (see the package README/dev notes):

  cd ~/vrx_ws/ardupilot
  Tools/autotest/sim_vehicle.py -v Rover -f gazebo-rover --console --map

  ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14555
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vrx_gz.bridge import Bridge, BridgeDirection


def launch(context, *args, **kwargs):
    bridges = [
        Bridge(
            gz_topic='/model/wamv/odometry',
            ros_topic='/vrx_ardupilot/wamv/odometry',
            gz_type='gz.msgs.Odometry',
            ros_type='nav_msgs/msg/Odometry',
            direction=BridgeDirection.GZ_TO_ROS),
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[b.argument() for b in bridges],
        remappings=[b.remapping() for b in bridges],
    )

    return [bridge_node]


def generate_launch_description():
    ardupilot_wamv_config = os.path.join(
        get_package_share_directory('vrx_ardupilot'), 'config', 'ardupilot_wamv.yaml')

    vrx_competition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vrx_gz'), 'launch',
                         'competition.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'config_file': ardupilot_wamv_config,
            'sim_mode': 'full',
            'extra_gz_args': '-v 1',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                               description='Name of world'),
        vrx_competition_launch,
        OpaqueFunction(function=launch),
    ])
