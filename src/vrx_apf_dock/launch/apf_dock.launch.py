"""Spawns the two-WAM-V docking scenario and starts the APF docking controller.

Includes vrx_gz's competition.launch.py (world + two_wamv.yaml spawn + default
per-model bridges: pose, joint_states, ...), then adds the extra bridges the
default spawn does not provide for USVs (IMU, thrusters) plus our 3 control
nodes.
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
    # World-frame ground truth: the per-model /model/<name>/pose bridge (set
    # up by default) only carries static sensor-to-link offsets, not the
    # model's pose in the world, and the generic Pose_V -> TFMessage bridge
    # on /world/<world>/dynamic_pose/info drops entity names entirely, so
    # neither is usable to recover which entry is which robot. Instead this
    # relies on VRX's OdometryPublisher plugin (ground_truth_enabled, forced
    # on for every spawned model in vrx_gz/model.py::xacro_cmd()), which
    # publishes gz.msgs.Odometry on /model/<name>/odometry -- a clean
    # per-model bridge to nav_msgs/Odometry.
    # IMU and thruster topics (/wamv/sensors/imu/imu/data, /wamv/thrusters/*)
    # are already bridged natively by VRX's own spawn pipeline (payload
    # bridges), no need to add them here.
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

    control_nodes = [
        Node(package='vrx_apf_dock', executable='state_estimator_node', output='screen'),
        # No teleop arbitrator in this launch (see vrx_apf_dock_teleop for
        # that), so remap the APF setpoint straight onto the topic
        # thrust_mixer_node actually listens to.
        Node(package='vrx_apf_dock', executable='apf_controller_node', output='screen',
             remappings=[('/vrx_apf_dock/wamv/setpoint_apf', '/vrx_apf_dock/wamv/setpoint')]),
        Node(package='vrx_apf_dock', executable='thrust_mixer_node', output='screen'),
        Node(package='vrx_apf_dock', executable='vector_marker_node', output='screen'),
        Node(package='vrx_apf_dock', executable='apf_field_marker_node', output='screen'),
    ]

    return [extra_bridge_node, *control_nodes]


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
            # vrx_gz/launch.py hardcodes '-v 4' (debug) before appending this;
            # gz sim's CLI keeps the last -v it sees, so this downgrades to
            # errors-only and silences the harmless but noisy per-frame
            # DetachableJoint "Child Link dummy_upper could not be found"
            # warning (there's no "platform" model in two_wamv.yaml).
            'extra_gz_args': '-v 1',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                               description='Name of world'),
        vrx_competition_launch,
        OpaqueFunction(function=launch),
    ])
