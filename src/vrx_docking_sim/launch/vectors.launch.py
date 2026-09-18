# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 7 -- vector_marker_node + apf_field_marker_node, publishing both
to the Gazebo GUI (gz-transport, no-op if headless:=true) and to RViz2
(visualization_msgs/MarkerArray on /vrx_docking_sim/markers). Assumes
livrables 1-4 are already running. Validate with Gazebo's GUI (headless
off) and/or:

  rviz2 --ros-args -p use_sim_time:=true
  # Fixed Frame "world", add a MarkerArray display on /vrx_docking_sim/markers

use_sim_time:=true on both nodes: without it, header.stamp comes from the
wall clock (~1.79e9 s, Unix epoch) while /clock (Gazebo's sim time) is only
at a few hundred seconds -- RViz2's tf2 message filter silently drops
every marker as "too far in the future" once it (or anything else in the
TF graph) is on sim time, no error shown, message count still increasing
(discovered live while validating livrable 7: markers rendered once right
after a node restart, then stopped -- consistent with the wall/sim clock
gap growing past whatever the filter tolerates). RViz2 itself also needs
use_sim_time:=true (see the command above) to interpret those stamps
correctly.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        Node(
            package='vrx_docking_sim',
            executable='vector_marker_node',
            output='screen',
            parameters=[sim_time],
        ),
        Node(
            package='vrx_docking_sim',
            executable='apf_field_marker_node',
            output='screen',
            parameters=[sim_time],
        ),
    ])
