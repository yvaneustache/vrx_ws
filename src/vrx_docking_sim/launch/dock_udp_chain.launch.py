# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 2 -- the dock -> USV UDP chain alone, testable independently of
SITL/MAVROS/joystick: dock_pose_source_node reads the dock's ground-truth
Gazebo pose (from docking_sim_spawn.launch.py's odometry bridge, must
already be running) and turns it into lat/lon+rpy;
dock_udp_broadcaster_node sends it over UDP in the guerledocking wire
format; dock_udp_receiver_node listens on the same port and republishes it
on the USV side. Validate with:

  ros2 topic echo /vrx_docking_sim/dock/udp_pose

dock_udp_receiver_node needs use_sim_time:=true: the UDP wire format it
reads carries no timestamp (see dock_udp_broadcaster_node's docstring), so
it stamps each received PoseStamped from its own node clock on arrival --
without use_sim_time that is the wall clock (~1.79e9 s, Unix epoch) while
/clock (Gazebo's sim time) is only a few hundred seconds. That stamp is
what boat_pose_node forwards onto the "world" -> "dock" TF it broadcasts
(livrable 7's dock-centred RViz2 view), so without this the transform is
silently unresolvable once RViz2 itself is on sim time -- same class of
bug as the marker nodes' stamps (see vectors.launch.py), discovered live
via Fixed Frame "dock" showing no markers at all.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_docking_sim',
            executable='dock_pose_source_node',
            output='screen',
        ),
        Node(
            package='vrx_docking_sim',
            executable='dock_udp_broadcaster_node',
            output='screen',
        ),
        Node(
            package='vrx_docking_sim',
            executable='dock_udp_receiver_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
