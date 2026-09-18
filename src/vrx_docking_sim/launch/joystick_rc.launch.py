# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 5 -- RC-transmitter-style joystick: standard `joy` package's
joy_node (reads the physical controller) + joystick_rc_node (emulates the
transmitter logic, see that file's docstring). Assumes livrables 1-4 are
already running (Gazebo, dock UDP chain, APF controller, MAVROS bridge,
ArduPilot SITL + MAVROS as separate processes).

Without physical joystick hardware, exercise joystick_rc_node directly by
publishing synthetic /joy messages instead of running joy_node, e.g.:

  ros2 topic pub /joy sensor_msgs/msg/Joy \
    "{axes: [0,1.0,0], buttons: [0,0,0,0,0]}" -r 20
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        Node(
            package='vrx_docking_sim',
            executable='joystick_rc_node',
            output='screen',
        ),
    ])
