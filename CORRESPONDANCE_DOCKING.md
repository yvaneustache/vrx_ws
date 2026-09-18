# Correspondance avec `docking/` (guerleboat / guerledocking)

Auteur : Claude (assistant) · Document généré le 2026-09-08

> Table de correspondance entre les fichiers de `docking/catkin_ws/src/guerleboat` et `docking/catkin_ws/src/guerledocking` (ROS 1 Melodic, projet ENSTA) et les nouveaux fichiers du simulateur de docking dans `vrx_ws` (ROS 2 Jazzy), décrit dans [`PROPOSITION_SIMULATEUR_COMPLET.md`](PROPOSITION_SIMULATEUR_COMPLET.md). Chaque fichier créé dans `vrx_ws` pour ce projet porte, dans son en-tête, une ligne renvoyant à cette table — voir §3 pour le format exact.

Chemins `docking/` abrégés depuis `docking/catkin_ws/src/` pour lisibilité.

> **Non-régression** : tous les fichiers listés en §1/§2 sont **nouveaux, dans le package `vrx_docking_sim`**. Aucun fichier de `vrx_apf_dock`/`vrx_apf_dock_teleop` n'est modifié — Mode A continue de fonctionner tel quel avec ses propres `apf_controller_node.py`, `vector_marker_node.py`, `apf_field_marker_node.py`, `joystick_setpoint_node.py` inchangés. Seuls `apf_lib.py` et `geometry_utils.py` (de `vrx_apf_dock`) sont réutilisés, par **import**, jamais par copie (voir `PROPOSITION_SIMULATEUR_COMPLET.md` §3.1/§8).

---

## 1. Nœud dock — package `vrx_docking_sim`

| Fichier `vrx_docking_sim` (nouveau) | Fichier(s) `docking/` d'origine |
|---|---|
| `dock_pose_source_node.py` | *(aucun — nouveau)* : simule les capteurs physiques du dock (GPS+SBG), absents en simulation. |
| `dock_udp_broadcaster_node.py` | `guerledocking/scripts/node_data_broadcaster.py` |
| `dock_mavlink_emitter_node.py` | **Différé, hors périmètre pour l'instant** — Mission Planner n'affiche que l'USV dans cette première version (voir `PROPOSITION_SIMULATEUR_COMPLET.md` §4). Gardé ici pour mémoire si le besoin d'afficher le dock dans Mission Planner revient. |

## 2. Nœuds USV — package `vrx_docking_sim` (sauf mention contraire)

| Fichier `vrx_docking_sim` (nouveau, sauf mention) | Fichier(s) `docking/` d'origine |
|---|---|
| `dock_udp_receiver_node.py` | `guerleboat/scripts/node_udp_converter.py` (+ `node_udp_simulator.py` comme référence pour un éventuel mode de test sans UDP réel) |
| `boat_pose_node.py` | `guerleboat/scripts/node_boat.py` |
| `apf_controller_node.py` (nouveau — **distinct** de `vrx_apf_dock/apf_controller_node.py`, qui reste inchangé pour le Mode A) | `guerleboat/scripts/node_control.py` (orchestration + filtre de Kalman) |
| `vrx_apf_dock/apf_lib.py` (**réutilisé par import, package `vrx_apf_dock`, non copié/modifié**) | `guerleboat/scripts/classBoat.py` (classe `Boat`, méthode `controller`) — déjà porté, gardé tel quel |
| `mavros_setpoint_bridge_node.py` (nouveau) | `guerleboat/scripts/node_control.py` (publication `/cmd_vel` + gating `armed`/`guided`) + `vrx_apf_dock/thrust_mixer_node.py` (PID de cap, relocalisé — l'original reste inchangé et continue de servir au Mode A) |
| `joystick_rc_node.py` (nouveau — **distinct** de `vrx_apf_dock_teleop/joystick_setpoint_node.py`, qui reste inchangé pour le Mode A) | `guerlerover/launch/launch_rover.launch` (référence de mapping `joy`/`teleop_twist_joy` — `guerleboat` n'avait pas de joystick propre) |
| `vector_marker_node.py`, `apf_field_marker_node.py` (nouveau — **distincts** des fichiers homonymes de `vrx_apf_dock`, qui restent inchangés pour le Mode A) | `guerleboat/scripts/node_visualization.py` (logique de tracé du champ de potentiel et des flèches consigne/actuel — reprise en marqueurs Gazebo/RViz2 au lieu de matplotlib) |
| `vrx_apf_dock/geometry_utils.py` (**réutilisé par import, package `vrx_apf_dock`, non copié/modifié**) | Fonctions équivalentes dispersées dans `classBoat.py`/`node_control.py`/`roblib.py` (`sawtooth`, conversions angle) |

`scripts/fake_mission_planner.py` et `scripts/fake_mp_arm_auto.py` (nouveaux, outils de dev hors run-time, comme `vrx_apf_dock/scripts/plot_apf_field.py`) : clients `pymavlink` indépendants de ROS/MAVROS, utilisés pour valider le chemin Mission Planner réel (livrable 6) — upload de mission et armement/AUTO via le port dédié `mavlink-router` (UDP 14550), à exécuter sur l'hôte.

## 3. Fichiers non repris (référence uniquement)

| Fichier `docking/` | Statut |
|---|---|
| `guerleboat/scripts/node_polar_visualization.py` | Script orphelin dans `docking/` (non installé, non lancé, contient un bug de portée de variable) — non porté. |
| `guerleboat/scripts/utils.py` | Téléchargement de fond de carte OSM pour matplotlib — sans objet en Gazebo/RViz2, non porté. |
| `guerleboat/scripts/roblib.py`, `classBoat.py` (racine `vrx_ws`), `controller.py` | Déjà présents à la racine de `vrx_ws` comme référence académique (Luc Jaulin/ENSTA Bretagne) — non exécutés, cf. `ARCHITECTURE.md` §6. |
| `guerleboat/launch/launch_boat.launch`, `guerledocking/launch/launch_dock.launch` | Inspirent la structure de `docking_stack.launch.py` (nouveau, ROS 2) — non copiés tels quels (catkin/roslaunch XML → colcon/launch Python). |

## 4. Format de l'en-tête de correspondance

Chaque fichier Python nouveau ou adapté listé en §1/§2 porte, juste après le shebang (`#!/usr/bin/env python3`) et avant le reste du code, une ligne unique :

```python
#!/usr/bin/env python3
# Portage de docking/catkin_ws/src/guerleboat/scripts/node_boat.py (ROS1 Melodic -> ROS2 Jazzy)
```

Pour un composant sans équivalent dans `docking/` :

```python
#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
```

Une seule ligne, pas de docstring multi-lignes — juste de quoi retrouver la source sans ouvrir ce document.
