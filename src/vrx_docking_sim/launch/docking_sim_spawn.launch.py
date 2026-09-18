# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 1 — Gazebo side only: spawns the USV (flown by ArduPilot Rover
SITL, see vrx_ardupilot/launch/ardupilot_wamv.launch.py for the pattern this
mirrors) and the dock (a second, plain-physics WAM-V, see
vrx/vrx_gz/config/two_wamv.yaml) in the same world, plus ground-truth
odometry bridges for both, for visualization/debugging while later
livrables replace the dock's ground-truth pose with its real UDP broadcast.

Does not modify vrx_ardupilot or vrx_apf_dock: this is a new, third spawn
config, started by a new launch file.

Spawns the two models directly via vrx_gz.launch.simulation()/spawn() and
vrx_gz.model.Model (unmodified, shared vrx_gz code -- same functions
competition.launch.py itself calls) instead of going through
competition.launch.py's config_file:=docking_sim.yaml path used in earlier
livrables. Reason: the USV needs a custom URDF (see urdf/wamv_gazebo_ned_fix
.urdf.xacro's docstring -- ArduPilotPlugin's default <gazeboXYZToNED> is off
by a 90 degree yaw term, confirmed live by comparing Gazebo ground truth to
/mavros/global_position/global: position AND heading reported to
MAVROS/Mission Planner were both wrong), and competition.launch.py's
config_file:=... multi-model path (Model.FromConfig) has no per-model way to
set that -- only its single-default-model fallback path honours a urdf:=
launch argument. Building the Model objects here instead, with
set_urdf() only on the USV, gets the same effect without touching
vrx_gz/wamv_gazebo.urdf.xacro (shared with vrx_ardupilot, protected by the
non-regression principle, see PROPOSITION_SIMULATEUR_COMPLET.md §8).

Also publishes a static "world" TF frame (identity, world -> map): nothing
in this stack otherwise publishes a frame by that name (robot_state_publisher
only publishes each WAM-V's own internal link tree, e.g. wamv/wamv/base_link,
with no common parent), so without this RViz2 has no "world" option in
Fixed Frame at all -- discovered live while validating livrable 7's
RViz2 markers (all of which use frame_id="world", matching plain Gazebo
world-frame coordinates already, no other transform needed).

ArduPilot SITL and MAVROS are separate processes, started independently:

  cd ~/vrx_ws/vrx_ardupilot_sitl_instance1
  ../ardupilot/build/sitl/bin/ardurover --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I1 --home <lat>,<lon>,0,0

  ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5770

(--model JSON, not sim_vehicle.py -f gazebo-rover — see ARCHITECTURE.md
§7.5 for why the latter does not work. --home: ArduPilot's own default
[CMAC/Canberra] otherwise, unrelated to wherever Gazebo actually places the
vehicle -- see PROPOSITION_SIMULATEUR_COMPLET.md §9, livrable 6 note, for
the coordinates and how they're derived.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import vrx_gz.launch
from vrx_gz.bridge import Bridge, BridgeDirection
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    usv_urdf = os.path.join(
        get_package_share_directory('vrx_docking_sim'), 'urdf',
        'wamv_gazebo_ned_fix.urdf.xacro')

    # model_type must be 'wam-v' (hyphenated, matching vrx_gz.model.USVS),
    # not 'wamv' -- 'wamv' is only the spawned entity's name (model_name,
    # first arg). Using 'wamv' for model_type too silently dropped
    # scale:=0.5 from the xacro invocation (Model.xacro_cmd() only appends
    # it when self.model_type in USVS), so the USV spawned at the default
    # scale (1.0) instead of 0.5 -- caught live by checking the actual
    # xacro command line vrx_gz printed at spawn.
    usv = Model('wamv', 'wam-v', [-532, 162, 0, 0, 0, 0])
    usv.set_scale(0.5)
    usv.set_ardupilot(True)
    usv.set_urdf(usv_urdf)

    dock = Model('wamv2', 'wam-v', [-517, 182, 0, 0, 0, 0])

    spawn_processes = vrx_gz.launch.simulation(
        world_name, headless=headless, extra_gz_args='-v 1')
    spawn_processes += vrx_gz.launch.spawn('full', world_name, [usv, dock])
    # competition_bridges() (unmodified, shared vrx_gz code) bridges /clock
    # among other topics -- competition.launch.py always called this
    # separately from spawn(), dropped by mistake in an earlier version of
    # this rewrite. Its absence didn't break the docking control loop itself
    # (nothing here subscribes to /clock directly), but every node started
    # with use_sim_time:=true and never having received a /clock message
    # reports time zero from get_clock().now() -- silently breaking anything
    # that stamps messages with it, discovered live via vector_marker_node/
    # apf_field_marker_node's RViz2 markers (header.stamp stuck at 0,
    # silently dropped by RViz2's tf2 message filter, no visible error).
    spawn_processes += vrx_gz.launch.competition_bridges(world_name)

    bridges = [
        Bridge(
            gz_topic='/model/wamv/odometry',
            ros_topic='/vrx_docking_sim/wamv/odometry',
            gz_type='gz.msgs.Odometry',
            ros_type='nav_msgs/msg/Odometry',
            direction=BridgeDirection.GZ_TO_ROS),
        Bridge(
            gz_topic='/model/wamv2/odometry',
            ros_topic='/vrx_docking_sim/dock/odometry',
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

    world_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
    )

    return spawn_processes + [bridge_node, world_frame_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta',
                               description='Name of world'),
        DeclareLaunchArgument('headless', default_value='false',
                               description='True to run Gazebo without its GUI'),
        OpaqueFunction(function=launch),
    ])
