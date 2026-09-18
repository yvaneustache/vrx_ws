# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 4 -- mavros_setpoint_bridge_node alone. Assumes livrables 1-3
are already running, plus ArduPilot Rover SITL and MAVROS as separate
processes (not started here, same convention as vrx_ardupilot):

  cd ~/vrx_ws/vrx_ardupilot_sitl_instance1
  ../ardupilot/build/sitl/bin/ardurover --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I1

  ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5770

Validate by forcing GUIDED by hand once armed:

  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_docking_sim',
            executable='mavros_setpoint_bridge_node',
            output='screen',
        ),
    ])
