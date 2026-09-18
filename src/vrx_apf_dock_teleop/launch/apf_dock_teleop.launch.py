"""Spawns the two-WAM-V scenario under joystick control, with a one-button
handoff to the APF docking controller.

Same world/spawn/bridge setup as vrx_apf_dock's apf_dock.launch.py, but runs
both apf_controller_node (publishing to /vrx_apf_dock/wamv/setpoint_apf,
*not* the final setpoint topic) and joy_node + joystick_setpoint_node, which
acts as the sole arbitrator on /vrx_apf_dock/wamv/setpoint: LT toggles
joystick_setpoint_node between forwarding manual stick/button control and
forwarding apf_controller_node's docking output -- see joystick_setpoint_node
for the toggle logic.
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
    # Same ground-truth odometry bridges as apf_dock.launch.py -- see that
    # file for why /model/<name>/pose and dynamic_pose/info aren't usable.
    bridges = [
        Bridge(
            gz_topic='/model/wamv/odometry',
            ros_topic='/vrx_apf_dock/wamv/raw_odom',
            gz_type='gz.msgs.Odometry',
            ros_type='nav_msgs/msg/Odometry',
            direction=BridgeDirection.GZ_TO_ROS),
        Bridge(
            gz_topic='/model/wamv2/odometry',
            ros_topic='/vrx_apf_dock/dock/raw_odom',
            gz_type='gz.msgs.Odometry',
            ros_type='nav_msgs/msg/Odometry',
            direction=BridgeDirection.GZ_TO_ROS),
    ]

    extra_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[b.argument() for b in bridges],
        remappings=[b.remapping() for b in bridges],
    )

    nodes = [
        Node(package='vrx_apf_dock', executable='state_estimator_node', output='screen'),
        # Unremapped: publishes to setpoint_apf, consumed (or not) by
        # joystick_setpoint_node's LT toggle -- see module docstring.
        Node(package='vrx_apf_dock', executable='apf_controller_node', output='screen'),
        Node(package='vrx_apf_dock', executable='thrust_mixer_node', output='screen'),
        Node(package='vrx_apf_dock', executable='vector_marker_node', output='screen'),
        Node(package='vrx_apf_dock', executable='apf_field_marker_node', output='screen'),
        Node(package='joy', executable='joy_node', output='screen'),
        Node(
            package='vrx_apf_dock_teleop', executable='joystick_setpoint_node',
            output='screen',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'max_heading_rate': LaunchConfiguration('max_heading_rate'),
            }]),
    ]

    return [extra_bridge_node, *nodes]


def generate_launch_description():
    two_wamv_config = os.path.join(
        get_package_share_directory('vrx_gz'), 'config', 'two_wamv.yaml')

    vrx_competition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vrx_gz'), 'launch',
                         'competition.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'config_file': two_wamv_config,
            'sim_mode': 'full',
            'extra_gz_args': '-v 1',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                               description='Name of world'),
        DeclareLaunchArgument('max_speed', default_value='2.0',
                               description='Speed (m/s) at full stick deflection'),
        DeclareLaunchArgument('max_heading_rate', default_value='1.0',
                               description='Heading rudder rate (rad/s) at full stick deflection'),
        vrx_competition_launch,
        OpaqueFunction(function=launch),
    ])
