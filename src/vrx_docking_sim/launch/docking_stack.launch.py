# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""Livrable 8 -- integration finale : un seul `ros2 launch` pour tout ce que
les livrables 2-3-4-7 lancaient jusqu'ici en 4 terminaux separes
(dock_udp_chain.launch.py, apf_controller.launch.py, mavros_bridge.launch.py,
vectors.launch.py). Reutilise ces quatre fichiers tels quels via
IncludeLaunchDescription plutot que de redupliquer les definitions de noeuds
-- si l'un d'eux est corrige/modifie plus tard, docking_stack.launch.py reste
a jour automatiquement, aucune double maintenance.

Ne regroupe PAS tout : conformement au decoupage deja documente en §7.2 de
PROPOSITION_SIMULATEUR_COMPLET.md, restent des processus separes (memes
raisons que vrx_ardupilot -- non-regression, §8) :
  - Gazebo + spawn (docking_sim_spawn.launch.py) : doit deja tourner, ce
    fichier bridge les topics d'odometrie que dock_pose_source_node/
    boat_pose_node consomment.
  - ArduPilot Rover SITL (`ardurover --model JSON ...`) et `mavlink-routerd` :
    binaires externes, pas des noeuds ROS, demarres en dehors de launch.
  - MAVROS (`ros2 launch mavros apm.launch fcu_url:=...`) : package tiers,
    son propre launch file, pas duplique ici.
  - joystick_rc.launch.py : terminal separe expres (manette physique
    optionnelle, `/dev/input/js0` peut etre absent -- voir ce fichier).

Assume que docking_sim_spawn.launch.py (livrable 1), ArduPilot SITL,
mavlink-routerd et MAVROS tournent deja (voir §7.1/§7.2). Une fois lance,
armer + passer en GUIDED (bouton joystick ou `ros2 service call
/mavros/set_mode ...`) declenche le docking automatique -- memes commandes
de suivi qu'en §7.1 :

  ros2 topic echo /vrx_docking_sim/wamv/setpoint_apf
  ros2 topic echo /mavros/state
  ros2 topic echo /vrx_docking_sim/wamv/docking_reached
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('vrx_docking_sim')

    def include(launch_file_name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', launch_file_name)))

    return LaunchDescription([
        include('dock_udp_chain.launch.py'),
        include('apf_controller.launch.py'),
        include('mavros_bridge.launch.py'),
        include('vectors.launch.py'),
    ])
