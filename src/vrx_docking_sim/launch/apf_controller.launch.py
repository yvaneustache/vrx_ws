# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 3 -- boat_pose_node + apf_controller_node (Kalman-filtered pose
+ APF docking law), testable independently of MAVROS/SITL/joystick: no
motor is driven yet. Assumes docking_sim_spawn.launch.py (livrable 1) and
dock_udp_chain.launch.py (livrable 2) are already running. Validate with:

  ros2 topic echo /vrx_docking_sim/wamv/setpoint_apf

boat_pose_node needs use_sim_time:=true: it re-broadcasts the "world" ->
"wamv"/"dock" TF transforms on a fast fixed-rate timer, stamped with
get_clock().now() (see that node's docstring for why) -- without sim time
that timer would stamp with the wall clock while RViz2/other TF consumers
are on sim time, reintroducing the flicker this timer was added to fix.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_docking_sim',
            executable='boat_pose_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='vrx_docking_sim',
            executable='apf_controller_node',
            output='screen',
        ),
    ])
