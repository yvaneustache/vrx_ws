# Docking WAM-V par champs de potentiel — Document de développement

## Contexte
- Simulateur : Gazebo (gz-sim) + ROS 2, projet VRX (`src/vrx`).
- 2 WAM-V définis dans `vrx_gz/config/two_wamv.yaml` :
  - `wamv` : le drone à piloter.
  - `wamv2` : sert de repère de "dock" (position + cap cible), pas de vraie structure de dock.
- Loi de commande fournie dans `controller.py` (+ `roblib.py` pour les utilitaires), équations des champs de potentiel dans `TabAPF_robmooc.png`.

## Logique de commande (rappel de controller.py)
- État robot `x = (px, py, v, theta)`, cible dock `phat = (px, py)`, cap dock `theta`.
- Point d'approche `phat0 = phat + 1.5 * unit(theta)` (devant le dock, sur l'axe du cap).
- Cas 1 — devant le dock (`unit^T (x - phat) < value`) : attraction combinée
  - potentiel attractif conique vers `phat` (`c21`)
  - potentiel uniforme le long du cap du dock (`c22`)
- Cas 2 — derrière/dans le dock : répulsion + attraction plane
  - potentiel plan/ligne le long de la normale au cap (`c11`) → repousse vers l'axe
  - potentiel uniforme dans la direction opposée au cap (`c12`) → pousse à l'intérieur
- Sortie : `u = (vbar, thetabar)` = vitesse et cap désirés, plus variables d'état `value`/`start` à conserver d'un appel à l'autre (machine à états à 2 phases).

## Constat technique (topics VRX)
- Un WAM-V n'a **pas** de topic `cmd_vel` exploitable (ce bridge n'existe que pour les UAV dans `vrx_gz/model.py`). Le contrôle se fait par thruster :
  - `/model/<name>/pose` (TFMessage, vérité terrain position+orientation)
  - `<namespace>/thrusters/<name>/thrust` (poussée, Float64)
  - `<namespace>/thrusters/<name>/pos` (angle azimutal, si layout avec azimut)
  - Layout par défaut (`thruster_config`) = `H` (probablement 2 propulseurs arrière fixes, sans azimut → pilotage par différentiel de poussée).
- Il faut donc un étage supplémentaire entre la sortie du contrôleur APF (`vbar`, `thetabar`) et les commandes moteur : un asservissement de cap (erreur `thetabar - theta`) + un mixage différentiel gauche/droite, plus un asservissement/mapping de vitesse `vbar → thrust`.

## Plan de développement proposé
1. **Package ROS 2** dédié (ex. `vrx_apf_dock`), noeud Python unique au départ.
2. **Entrées** : pose du drone (`/model/wamv/pose`) et pose du dock (`/model/wamv2/pose`), extraction (x, y, yaw, v estimée) depuis TF/quaternion.
3. **Bloc APF** : portage direct de `controller.py` (import `roblib` ou réécriture des quelques fonctions utilisées pour éviter la dépendance matplotlib en environnement embarqué).
4. **Bloc bas niveau** : cap → différentiel de poussée gauche/droite (+ saturation), vitesse → poussée totale.
5. **Bridges Gazebo↔ROS2** additionnels à déclarer dans le launch (thrust gauche/droite pour `wamv`, poses des deux modèles).
6. **Launch file** dédié qui : lance `two_wamv.yaml` (ou un monde dédié), lance les bridges nécessaires, lance le noeud de contrôle.
7. **Tests / réglage** : lancer en visuel, vérifier la manœuvre d'approche puis d'entrée dans le dock, régler `c11, c12, c21, c22`, la distance `1.5` du point d'approche, et le seuil `value`.
8. **Documentation** : tenir ce fichier à jour à chaque étape/décision.

## Décisions validées avec l'utilisateur
1. **Propulsion** : layout `H` (2 thrusters fixes "left"/"right", pas d'azimut) — de toute façon `thruster_config` est codé en dur à `H` dans `model.py::xacro_cmd()`, donc pas le choix. Cap piloté par différentiel de poussée gauche/droite.
2. **Dock (`wamv2`)** : libre, jamais commandé (pas de `locked`). Note : l'arg xacro `locked` existe mais n'est référencé nulle part dans `wamv_gazebo.urdf.xacro` → actuellement mort, aucun effet de toute façon.
3. **Monde de test** : `sydney_regatta`.
4. **Estimation de vitesse** : IMU + intégration (accélération linéaire → vitesse). L'IMU (`imu_wamv`) est en fait déjà présente par défaut sur chaque WAM-V car `model.py::xacro_cmd()` force `vrx_sensors_enabled:=true` (donc IMU, GPS, caméras, lidar tous actifs par défaut, indépendamment des flags individuels dans le yaml).
5. **Portage de `controller.py`** : nettoyage de la machine à états en portant (suppression de `sum` mort, clarification `value`/`start`), plutôt que copie fidèle.
6. **Architecture** : 3 noeuds séparés — `state_estimator`, `apf_controller`, `thrust_mixer`.
7. **Nom du package** : `vrx_apf_dock`.

## Faits techniques confirmés (lecture du code VRX)
- Bridge de pose (`/model/<name>/pose` → `tf2_msgs/TFMessage`, topic ROS `pose`) déjà actif par défaut pour **tous** les modèles spawnés (UAV et USV) via `Model.bridges()` — rien à ajouter pour la pose de `wamv` et `wamv2`.
- Pas de bridge `cmd_vel` pour les USV (réservé aux UAV) → confirmé, contrôle uniquement par thrusters.
- Thrusters (layout H, fichier `wamv_aft_thrusters.xacro`) : topics gz `wamv/thrusters/left/thrust` et `wamv/thrusters/right/thrust` (type `gz.msgs.Double` / ROS `std_msgs/Float64`), à bridger nous-mêmes (`BridgeDirection.ROS_TO_GZ`). Le joint `pos` (azimut) existe aussi mais reste à 0 par défaut, pas besoin de le piloter en layout H.
- IMU : capteur `imu_wamv_sensor` sur le lien `<namespace>/imu_wamv_link`. Topic gz attendu : `/world/<world_name>/model/<model_name>/link/imu_wamv_link/sensor/imu_wamv_sensor/imu` (`gz.msgs.IMU` / ROS `sensor_msgs/Imu`), à bridger nous-mêmes (`BridgeDirection.GZ_TO_ROS`).
- `gz-sim-detachable-joint-system` présent inconditionnellement dans le xacro (cherche un modèle enfant nommé `platform`) mais `suppress_child_warning: true` → sans modèle `platform` dans le monde (cas de `two_wamv.yaml` + `sydney_regatta`), ce plugin ne fait rien. Pas un blocage.
- `vrx_gz/launch/competition.launch.py` accepte déjà `world:=sydney_regatta config_file:=<chemin two_wamv.yaml>` pour spawner directement plusieurs robots depuis un yaml — on l'inclut plutôt que de dupliquer la logique de spawn.
- Classes réutilisables : `vrx_gz.bridge.Bridge` / `BridgeDirection` + le pattern `Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[b.argument() ...], remappings=[b.remapping() ...])` vu dans `vrx_gz/launch.py`.

## Architecture retenue (noeuds & topics)
```
[Gazebo]
  pose(wamv)  --(bridge par défaut, TF)-->        ┐
  pose(wamv2) --(bridge par défaut, TF)-->        ├─> state_estimator
  imu(wamv)   --(bridge à ajouter, Imu)-->         ┘
                                                       │
                                    /vrx_apf_dock/wamv/odom (nav_msgs/Odometry: x,y,yaw,twist.linear.x=v)
                                    /vrx_apf_dock/dock/pose (geometry_msgs/PoseStamped: x,y,yaw du dock)
                                                       │
                                                       ▼
                                                apf_controller (port nettoyé de controller.py, apf_lib.py)
                                                       │
                                    /vrx_apf_dock/wamv/setpoint (geometry_msgs/Twist détourné :
                                                                  linear.x = vbar, angular.z = thetabar ABSOLU,
                                                                  pas une vitesse angulaire — documenté en commentaire)
                                                       │
                                                       ▼
                                                thrust_mixer (PID de cap + mixage différentiel + saturation)
                                                       │
                        wamv/thrusters/left/thrust, wamv/thrusters/right/thrust (std_msgs/Float64, bridge à ajouter)
```

## Environnement d'exécution : Docker, pas l'hôte
VRX (version gz-sim, pas Gazebo Classic) ne peut tourner que dans le conteneur Docker `vrx-devel-custom` (image buildée depuis `src/vrx/docker/`), qui a ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim 8.10) installés — l'hôte n'a que Gazebo Classic 11 et ROS Humble. Deux conteneurs tournaient déjà (`upbeat_wescoff`, `romantic_wu`), montant tout `/home/eustache` en bind rw, donc `vrx_ws` est visible aux mêmes chemins dans le conteneur.
**Toujours builder/lancer depuis le conteneur** :
```
docker exec -it upbeat_wescoff bash
source /opt/ros/jazzy/setup.bash
source /home/eustache/vrx_ws/install/setup.bash
cd /home/eustache/vrx_ws
colcon build --packages-select vrx_apf_dock --merge-install --symlink-install
ros2 launch vrx_apf_dock apf_dock.launch.py headless:=True   # ou sans headless pour la GUI
```
(Le workspace utilise le layout colcon `--merge-install` : ne pas omettre ce flag.)

## Dépendances système ajoutées (sur l'hôte, pas dans le conteneur qui les avait déjà)
- `sudo apt install python3-sdformat14 libsdformat14` (requis par `vrx_gz/model.py`, `import sdformat14`, pour parser le SDF généré et détecter les payloads). Sans rapport avec notre package, c'est une dépendance de la pipeline de spawn VRX elle-même.

## Modification du dépôt vendored `src/vrx`
- `vrx_gz/src/vrx_gz/model.py::xacro_cmd()` : ajout de `ground_truth_enabled:=true` (à côté des 3 autres args déjà forcés : `locked`, `vrx_sensors_enabled`, `thruster_config`). Nécessaire pour activer le plugin `gz-sim-odometry-publisher-system` (`wamv_p3d.xacro`) sur chaque WAM-V spawné — voir section suivante.

## Coup dur évité : la pose "monde" n'est PAS où on l'attendait
Deux pistes explorées puis abandonnées avant de trouver la bonne source :
1. **`/model/<name>/pose` (bridge par défaut, TFMessage)** : ne contient QUE les offsets statiques capteur→lien parent (translation quasi nulle, rotation identité pour l'IMU) — PAS la pose du robot dans le monde. Piège classique : le plugin `PosePublisher` de VRX est configuré avec `publish_sensor_pose=true` mais `publish_model_pose=false`/`publish_link_pose=false`.
2. **`/world/<world>/dynamic_pose/info`** (topic natif gz-sim, vérité-terrain de toutes les entités) : contient bien une entrée par entité avec le bon nom (`wamv`, `wamv2`, ...) côté gz-transport natif (vérifié via `gz topic -e`), MAIS le bridge générique `ros_gz_bridge` `Pose_V -> tf2_msgs/TFMessage` ne préserve PAS le champ `name` pour ce topic (frame_id/child_frame_id restent vides côté ROS) — probablement parce que ce Pose_V n'a pas de header par-pose contrairement à celui de `/model/.../pose`. Piste abandonnée : dépendre de l'ordre des poses dans le message aurait été trop fragile.

**Solution retenue** : activer le plugin `OdometryPublisher` de VRX (`ground_truth_enabled`, cf. `wamv_p3d.xacro`), qui publie nativement `gz.msgs.Odometry` sur `/model/<name>/odometry` — un bridge standard vers `nav_msgs/Odometry` qui, lui, fonctionne parfaitement (testé et confirmé). `state_estimator_node` consomme désormais `/vrx_apf_dock/{wamv,dock}/raw_odom` (bridgés depuis `/model/wamv/odometry` et `/model/wamv2/odometry`) plutôt que `dynamic_pose/info`.

## Test bout-en-bout (2026-07-13, dans le conteneur, `sim_mode=full headless=true`)
Chaîne complète validée en conditions réelles (topics inspectés via `ros2 topic echo`, avec des timeouts généreux ~15s car la découverte DDS est lente dans ce conteneur chargé) :
- `/vrx_apf_dock/wamv/odom` et `/vrx_apf_dock/dock/pose` : positions cohérentes avec les points de spawn (`wamv` ≈ (-532,162), `wamv2`/dock ≈ (-532,172)).
- `/vrx_apf_dock/wamv/setpoint` : vbar≈4.9 m/s, thetabar≈0.96 rad (proche du cap du dock ≈1.0 rad) → cas 1 (attraction) correctement engagé.
- `/wamv/thrusters/{left,right}/thrust` : poussées cohérentes (≈56-57 chacune, léger différentiel pour la correction de cap).
- Position du wamv suivie sur ~10s : passe de (-531.5, 162.8) à (-528.2, 168.6), donc se rapproche bien du dock (-532, 172) → comportement d'approche conforme à l'attendu.
- Pas encore observé : la transition vers le cas 2 (répulsion/re-centrage une fois "derrière" le dock) ni l'arrêt final — à valider visuellement (sans `headless`) pour le réglage fin des gains (`c11,c12,c21,c22`, gains PID du `thrust_mixer`).

## Architecture finale (mise à jour, remplace le schéma précédent)
```
[Gazebo, ground_truth_enabled:=true sur chaque WAM-V]
  /model/wamv/odometry   --(bridge ajouté)-->  /vrx_apf_dock/wamv/raw_odom  ┐
  /model/wamv2/odometry  --(bridge ajouté)-->  /vrx_apf_dock/dock/raw_odom ├─> state_estimator_node
  /wamv/sensors/imu/imu/data (natif, déjà bridgé par VRX)                  ┘
                                                       │
                                    /vrx_apf_dock/wamv/odom (nav_msgs/Odometry : pose monde + twist.linear.x = v intégrée IMU)
                                    /vrx_apf_dock/dock/pose (geometry_msgs/PoseStamped : pose monde du dock)
                                                       │
                                                       ▼
                                                apf_controller_node (apf_lib.py, port nettoyé de controller.py)
                                                       │
                                    /vrx_apf_dock/wamv/setpoint (geometry_msgs/Twist détourné :
                                                                  linear.x=vbar, angular.z=thetabar ABSOLU)
                                                       │
                                                       ▼
                                                thrust_mixer_node (PID cap + PI vitesse + mixage différentiel)
                                                       │
                        /wamv/thrusters/{left,right}/thrust (std_msgs/Float64, natif, déjà bridgé par VRX)
```

## Logs de debug ajoutés
`apf_controller_node` et `thrust_mixer_node` publient désormais un `get_logger().info(..., throttle_duration_sec=1.0)` à chaque itération : position/cap wamv+dock, setpoint (v,theta), erreur de cap, poussées gauche/droite. Utile pour tout réglage de gains — regarder la sortie console du `ros2 launch` (pas besoin de `ros2 topic echo`).

## Bug identifié et corrigé : cap qui ne convergeait pas
Symptôme rapporté par l'utilisateur : le wamv se dirige bien vers le dock mais ne s'aligne pas sur le cap voulu. Diagnostic via les logs ajoutés :
- Le couple de rotation disponible est proportionnel au *différentiel* gauche/droite (`diff_thrust`, piloté par `KP_HEADING * heading_error`), alors que la poussée qui lance le bateau tout droit est la poussée *totale* (`total_thrust`, piloté par `KP_SPEED`/`KI_SPEED`).
- Avec `KP_HEADING=100` (valeur de test de l'utilisateur), `diff_thrust` restait de l'ordre de 100 N face à un `total_thrust` de 500-600 N : le bateau accélérait (jusqu'à ~2 m/s) bien plus vite qu'il ne tournait, et l'amortissement hydrodynamique en lacet (plus fort à vitesse d'avance élevée) figeait `heading_error` autour de -0.9/-1.0 rad en continu.
- **Fix validé en simulation** : `KP_HEADING: 100 → 3000` dans `thrust_mixer_node.py`. Résultat observé : le cap converge en ~4s au lieu de stagner indéfiniment, et suit correctement les changements de consigne de l'APF au fil de l'approche.

## Côté d'entrée du dock (avant/arrière)
Le point d'approche (`p_dock_approach = phat + 1.5*unit`) et la bascule cas1/cas2 sont définis relativement au cap (`yaw`) du dock — c'est donc **`two_wamv.yaml`** qui décide de quel côté le drone entre, pas `apf_lib.py`. Constat : avec `wamv2` à `rpy: [0,0,0]`, le drone entrait par l'avant (côté est, +x). Demande : entrer par l'arrière → cap du dock tourné de 180° dans `two_wamv.yaml` (`rpy: [0,0,3.14159265]`). Confirmé en simulation : le drone approche maintenant par l'ouest (x descend jusqu'à ~-551 avant de recourber vers le dock à x=-532), donc par le côté opposé — comportement voulu obtenu sans toucher au code.

**Effet secondaire observé (pas encore traité)** : avec le réglage actuel `c21=5, c22=100` (fort ratio uniforme/attractif), le drone suit une ligne quasi droite dans la direction du cap du dock et ne recourbe que très tard, très près de la cible → gros dépassement en distance avant de rentrer (overshoot ~19m dans le test). Pour un rentrée plus "propre" (moins de dépassement), augmenter `c21` relativement à `c22` dans `apf_lib.py`, ou plafonner la vitesse d'approche. Pas encore fait, à la demande de l'utilisateur (règle les gains lui-même).

## Visualisation des vecteurs dans Gazebo
Nouveau nœud `vector_marker_node.py` : dessine 2 flèches directement dans la vue Gazebo (pas RViz — les marqueurs RViz ne s'affichent pas dans le rendu 3D natif de Gazebo), avec pour origine la position du wamv :
- rouge = cap actuel du drone
- verte = cap visé (setpoint APF, `thetabar`)

Implémentation : `gz.transport13`/`gz.msgs10.marker_pb2.Marker` en direct (pas de bridge ROS — les marqueurs Gazebo n'ont pas d'équivalent ROS standard).

**Deux bugs trouvés et corrigés avant que ça fonctionne** :
1. `LINE_LIST` (segments fins) rend en ~1px dans le moteur de rendu Ogre2 de Gazebo → quasi invisible sur un monde qui fait des centaines de mètres. Remplacé par des primitives 3D pleines : `CYLINDER` (hampe, rayon 0.15m) + `CONE` (pointe, rayon 0.5m), orientées par quaternion (rotation de l'axe local +Z du cylindre/cône vers la direction 2D voulue — toujours une rotation de 90° puisque Z est perpendiculaire à tout vecteur du plan XY).
2. **Cause principale** : `/marker` est en réalité un **service** Gazebo (`gz.msgs.Marker` → `gz.msgs.Empty`), pas un topic pub/sub classique ! Vérifié via `gz service -i -s /marker` (fournisseur bien présent une fois `MarkerManager` chargé côté GUI) vs `gz topic -i -t /marker` (`No subscribers` en permanence, même avec le plugin chargé). Le code publiait donc dans le vide. Corrigé : `gz_node.request('/marker', marker, Marker, Empty, timeout_ms)` au lieu de `advertise()`/`publish()`.
3. **Piège n°2** : en passant à `/marker_array` (pour batcher les 4 primitives en un seul appel, cf. section délai ci-dessous), le type de réponse n'est **pas** `gz.msgs.Empty` comme pour `/marker` mais `gz.msgs.Boolean` — vérifié via `gz service -i -s /marker_array`. Un mismatch de type sur `response_type` fait échouer silencieusement `request()` (retourne juste `False`, aucune exception). Corrigé, testé manuellement : `result=True, response=data:true`.

**Délai observé entre les marqueurs et le drone** : deux causes cumulées, corrigées.
1. Le nœud utilisait un `create_timer(0.1s)` découplé de l'arrivée des données d'odométrie → jusqu'à 100ms de retard de phase en plus de la latence de transport.
2. 4 appels de service bloquants séquentiels (`/marker` par primitive) par cycle → latence cumulée.
Corrigé : déclenchement direct depuis le callback d'odométrie (`_odom_cb` appelle `_publish_markers()` immédiatement, plus de timer séparé) + les 4 primitives regroupées en un seul message `Marker_V` envoyé en un seul appel au service `/marker_array` (au lieu de 4 appels à `/marker`).

Notes utiles pour la suite :
- `MarkerManager` est un plugin **GUI only** (`gz-gui-8/plugins/libMarkerManager.so`), jamais actif en `headless:=True` — donc `gz topic/service -i` sur `/marker` ne montrera jamais de fournisseur en headless, c'est normal, pas un bug.
- Attention à ne jamais lancer une session de test (même headless) en parallèle d'une session GUI existante : les noms de nœuds/topics ROS2 sont identiques d'un lancement à l'autre (pas de namespacing par instance), donc deux lancements simultanés se marchent dessus (deux `state_estimator_node`, bridges dupliqués sur les mêmes topics). Toujours vérifier `pgrep -af 'gz sim|ros2 launch vrx_apf_dock'` avant de relancer, et ne tuer que les PID de sa propre session si une autre tourne déjà.

## Réglage des gains PID (thrust_mixer_node.py)
Valeurs retenues après test en simulation (headless, logs) :
```
KP_HEADING = 400.0, KI_HEADING = 20.0, KD_HEADING = 50.0, HEADING_INTEGRAL_LIMIT = 1.0
KP_SPEED = 100.0, KI_SPEED = 10.0, KD_SPEED = 5.0, SPEED_INTEGRAL_LIMIT = 2.0
```
Observation : convergence de cap rapide et propre en phase de croisière (loin du dock) — `heading_error` décroît de 1.48 à 0.02 rad en ~8s sans oscillation. Près du point cible (`phat`), le setpoint lui-même tourne très vite (terme `c21/r^3` qui explose près de la singularité `r→0`) et le suivi de cap devient logiquement moins net à ce moment précis (le robot ralentit fortement, `v` proche de 0) — ce n'est pas un défaut de régulation mais un effet inhérent à la loi APF près de la cible.

## Corrections apportées à apf_lib.py (comparaison avec classBoat.py)
`classBoat.py` (fichier de référence trouvé dans `vrx_ws/`) est une version plus aboutie de la même loi APF. Points 1 à 6 identifiés et corrigés dans `apf_lib.py`, architecture conservée (toujours 3 nœuds, setpoint absolu `(v, theta)`) :
1. **Amortissement de vitesse près de la cible** : `v_cmd = min(norm(v_vec), k*norm(p_dock_approach-p)/10)` — le `/10` supplémentaire réduit fortement le dépassement en approche finale.
2. **Saturation explicite `VMAX=5.0`** sur `v_cmd` (absente avant).
3. **Distance de transition proportionnelle** : `TRANSITION_DIST = 10*L` (utilise enfin la constante `L` déjà déclarée mais inutilisée) au lieu du `3.0` codé en dur.
4. **Ré-armement de l'attraction** : `_switch_value` ne revient plus à 0 en sortant de la branche d'attraction (comportement de `classBoat.py`, différent de `controller.py` original) — reste à `TRANSITION_DIST` en permanence après le premier déclenchement, élargissant la bande où l'attraction est le régime "par défaut". `_active` peut se ré-armer si le robot dérive derrière le dock au-delà de `-_switch_value/3`, utile après un premier accostage (vagues, dérive).
5. **Marge configurable** : `MARGIN = 1.5` (constante nommée au lieu du littéral).
6. **Gains de cap** : architecture volontairement conservée (le cap est régulé séparément par un PID complet dans `thrust_mixer_node.py`, plus complet que le PI de `classBoat.py`) — pas de changement de code, juste confirmation que les gains de référence de `classBoat.py` (kd=5, ki=0.02) ne sont pas directement transposables vu l'unité de sortie différente (poussée différentielle vs vitesse de lacet directe).

**Validé** : simulation Python autonome (sans Gazebo) montre un dépassement réduit à ~10m au lieu de ~19m, et une décroissance de vitesse monotone sans oscillation. Confirmé aussi en simulation complète (headless) : trajectoire courbe fluide, `setpoint v` décroissant progressivement de 3.8 à 2.5 m/s au fil de l'approche, pas d'oscillation.

## `self._reached` : arrêt moteur au docking, et bug de faux-positif corrigé
L'utilisateur a ajouté `self._reached` (latch : une fois `r < REACHED_RADIUS=5`, `compute()` retourne toujours `(0,0)`). Deux corrections apportées :
1. **Bug du cap absolu à l'arrêt** : `theta_cmd=0.0` renvoyé par `compute()` une fois `reached` est un cap ABSOLU (vers l'est), pas "arrête de tourner" — le laisser tel quel aurait fait le bateau pivoter vers l'est au lieu de s'arrêter. Corrigé dans `apf_controller_node.py` : quand `reached`, on substitue `theta_cmd = self._wamv_yaw` (cap courant), ce qui annule l'erreur de cap et laisse le PID de `thrust_mixer_node.py` converger naturellement vers une poussée nulle.
2. **Faux-positif "reached" côté arrière** : `r < REACHED_RADIUS` seul ne vérifie pas de quel côté du dock on est. Test (script Python isolé, dock face -x, drone parti du mauvais côté à (-15,2)) : `reached` se déclenchait à r=4.77 alors que `switch` valait encore `0.0` (jamais passé en case 1/attraction) — le bateau se figeait derrière le dock, jamais entré. Corrigé : `reached` n'est mis à `True` que si `r < REACHED_RADIUS` **et** qu'on est dans la branche avant/attraction (`front_branch`).

## Test du scénario "mauvais côté" : collision physique réelle confirmée
Après le correctif de gating, retest en Python pur : le bateau traverse bien vers le bon côté avant de s'arrêter, mais `min_r` observé pendant la traversée = **0.44m** — extrêmement proche du point du dock. Test en simulation réelle (spawn temporaire du wamv à (-547,202), à l'ouest du dock à (-532,200,yaw=π), donc côté "derrière" pour cette orientation) : **collision physique confirmée** via `/vrx/contacts` — contact soutenu entre `wamv/base_link` et `wamv2/base_link` (forces de contact réelles ~23-26N), suffisamment fort pour pousser et faire dériver le dock (wamv2) lui-même, qui n'est plus resté sur place. `reached` n'est jamais passé à `True` dans ce run (les deux bateaux restent embourbés en contact permanent).

**Cause profonde** : les équations de l'énoncé (et `classBoat.py`) n'incluent aucun terme de répulsion basé sur la distance réelle à la coque du dock — seul le terme `n·nᵀ` recentre sur l'AXE (la ligne passant par le point du dock), ce qui peut nécessiter de passer très près du point du dock pour corriger un désaxage important, indépendamment de la taille réelle des coques. C'est une limite inhérente à la loi donnée, pas un bug introduit par le portage.

Position de spawn du wamv restaurée à l'original `(-532,162), rpy=[0,0,1]` après ce test diagnostic (c'était temporaire, pas un changement de config demandé).

**Non résolu, à décider avec l'utilisateur** : faut-il ajouter un terme de répulsion réel basé sur la distance à la coque du dock (au-delà de ce que donnent les équations de l'énoncé), ou accepter cette limite si les scénarios de test/évaluation ne placent jamais le drone aussi loin du côté opposé au cap du dock ?

## Vérification de `back_branch` : logique correcte, spawn en cause
La logique de `back_branch` (`unit@diff < switch_value`, formule `c21,c22`) est correcte et conforme à l'intention voulue par l'utilisateur : côté "mauvais" (derrière le dock) → doit parcourir loin (jusqu'à `TRANSITION_DIST`) avant d'être repoussé, ce n'est pas un bug. Confirmé par test Python isolé sur les deux côtés séparément.

**Cause réelle du blocage observé** : le spawn actuel du wamv (`-532,162`, même x que le dock à `-532,200`) est un cas limite dégénéré — exactement perpendiculaire à l'axe du dock. Comme `two_wamv.yaml` utilise une valeur approchée de π (`3.14159265` au lieu de la pleine précision double), `sin(theta_dock)` n'est pas exactement 0 mais infime (~3.6e-9) ; multiplié par le grand offset y (-38), ça suffit à faire basculer `unit@diff` légèrement négatif par pur bruit numérique, plaçant le drone en `back_branch` dès le départ. Résultat : oscillation permanente à r≈10 (rebond entre x=-542.2 et x=-541.8), jamais d'entrée. Ce n'est pas un bug de code — c'est un problème de géométrie de spawn.

**Fix appliqué** : nouveau spawn `xyz: [-545, 199, 0], rpy: [0,0,0]` — clairement du bon côté (x très inférieur à celui du dock), proche de l'axe d'entrée pour limiter le risque de franchir la frontière en cours de route. Testé en Python isolé : convergence propre et monotone, `reached=True` à r=0.999, jamais de bascule en `back_branch`.

## Nouveau problème découvert en simulation réelle : le dock dérive trop vite
Test complet en Gazebo avec le nouveau spawn : approche initiale propre (v décroît de 5.0 à ~1.8), MAIS le dock (`wamv2`, volontairement non-commandé/libre, décision prise plus tôt dans le projet) dérive de façon significative et continue sous l'effet du vent/courant du monde VRX — environ 0.46 m/s de dérive constante observée. Résultat : le drone se stabilise en régime de "poursuite" (v_cmd≈1.79-1.80, cap ≈ direction de dérive du dock) sans jamais réduire l'écart en dessous de r≈5, donc `reached` ne se déclenche jamais dans ce test (25+ secondes, log de 18k lignes, 0 occurrence de `reached=True`).

**Non résolu, à décider avec l'utilisateur** : le dock doit-il être ancré/verrouillé (contredit la décision initiale "dock libre" mais rendrait le test déterministe), ou faut-il augmenter `VMAX`/les gains pour que le drone puisse rattraper la dérive du dock ?

## Terme de répulsion dédié pour éviter le dock en back_branch
Confirmé (avec la config actuelle du yaml, dock yaw=0) : le drone en `back_branch` fonçait droit sur le dock (`c22*unit` pousse directement vers lui avec ce cap) et ne s'écartait qu'à r≈1.06 — bien trop tard, collision réelle confirmée via `/vrx/contacts` (contact soutenu `left_float`/`left_float`).

**Fix** : nouveau terme de répulsion de type Khatib (rayon d'influence fini, nul à `SAFETY_RADIUS` et croissant en douceur en s'en approchant), ajouté uniquement dans `back_branch` (n'affecte pas l'approche normale dans l'autre branche) :
```python
SAFETY_RADIUS = 15.0
SAFETY_GAIN = 300000.0
if r < SAFETY_RADIUS:
    v_vec += SAFETY_GAIN * (1/r - 1/SAFETY_RADIUS) / r**2 * (diff/r)
```
Itéré plusieurs fois (4000→15000→40000→100000→300000 avec RADIUS 8→15) en testant le r_min à chaque fois (Python isolé) : distance centre-à-centre minimale passée de 1.06m à 7.9m. Confirmé sans régression sur l'approche côté correct (toujours `reached=True` à r=0.99).

**Résultat en simulation réelle (avec physique/collision réelle)** : nette amélioration, mais **un contact bref a encore été détecté** (`left_front_float_collision`) au moment du passage le plus proche — les coques ont une largeur réelle (les flotteurs dépassent du centre), donc une marge centre-à-centre, même généreuse, ne garantit pas zéro contact si l'angle de passage est defavorable. Le message de contact capturé pourrait aussi être un message figé (topic capturé alors que les bateaux étaient déjà loin l'un de l'autre) plutôt qu'un contact réellement en cours — pas encore élucidé avec certitude.

**Non résolu / à décider** : pousser encore la marge de sécurité (SAFETY_RADIUS/GAIN plus grands), ou accepter ce niveau (contact bref, sans la collision soutenue d'avant) comme suffisant ?

## Suppression du warning DetachableJoint
`[Wrn] [DetachableJoint.cc:348] Child Link dummy_upper could not be found.` vient d'un plugin de `wamv_gazebo.urdf.xacro` (vendored, cherche un modèle "platform" absent de `two_wamv.yaml`). Le xacro a bien `<suppress_child_warning>true</suppress_child_warning>` mais ça ne supprime pas ce warning précis (probablement émis à une étape différente du plugin, indépendante de ce flag). Plutôt que modifier le xacro vendored, fix non-invasif dans `apf_dock.launch.py` : `extra_gz_args: '-v 1'` passé à `competition.launch.py`, qui l'ajoute après le `-v 4` codé en dur dans `vrx_gz/launch.py` — `gz sim` garde le dernier `-v` vu sur la ligne de commande. Confirmé : 0 occurrence du warning, log ~100 lignes au lieu de plusieurs milliers, aucune erreur introduite. (Réduit aussi tous les messages Dbg/Wrn en général, pas seulement celui-ci — à remonter à `-v 2` ou plus si besoin de voir d'autres warnings à l'avenir.)

## Fix : le drone continuait d'avancer après `reached=True`
Symptôme : `speed_cmd=0.00` mais `total_thrust` restait bloqué autour de 200-250 en continu, le bateau continuait de dériver vers l'avant à vitesse quasi constante après l'arrêt. Cause : windup classique de l'intégrale de vitesse — `KI_SPEED=100 × SPEED_INTEGRAL_LIMIT=2.0 = 200`, accumulée pendant la longue croisière, qui ne se résorbe que très lentement une fois `speed_cmd` à 0 (l'erreur devient petite donc l'intégrale ne redescend quasiment plus).

**Fix** (`thrust_mixer_node.py`) : reset immédiat de `_speed_error_integral` à chaque fois que `speed_cmd == 0.0` (déclenché exactement par le retour `(0.0, 0.0)` de `compute()` une fois `reached`), au lieu de laisser l'intégrale se dissiper lentement.

**Résultat mesuré (1ère itération)** : `total_thrust` passe de 526 à **-5.3** (freinage actif) dès l'instant où `reached=True`, puis se stabilise à des valeurs bien plus faibles (~50-110 au lieu de ~200-250 en continu). Mais les moteurs continuaient à tourner un peu (~100 de poussée) : le reset de l'intégrale ne suffisait pas, le terme **proportionnel** (`KP_SPEED × speed_error`) prenait le relais, car `_current_v` (vitesse estimée par intégration fuyante de l'IMU) reste bruitée/pas fiable à basse vitesse (l'IMU du WAM-V est connue pour être mal orientée, cf. le TODO dans `classBoat.py`) — le régulateur passait son temps à "corriger" une vitesse qui n'était peut-être pas réellement là.

**Fix définitif** : une fois `reached` (`speed_cmd==0.0`), on coupe entièrement la régulation PID (vitesse ET cap) et on publie directement `left=0.0, right=0.0`, sans passer par les gains. Confirmé via `ros2 topic echo /wamv/thrusters/{left,right}/thrust` : `data: 0.0` des deux côtés. Le bateau continue de décélérer par inertie/frottement de l'eau après la coupure (dérive résiduelle qui diminue progressivement, cohérent avec un arrêt moteur réel).

## Prochaines étapes
1. Lancer sans `headless` (GUI) pour observer visuellement la manœuvre complète (approche + entrée dans le dock + comportement cas 2).
2. Poursuivre le réglage fin des gains : `c11,c12,c21,c22` dans `apf_lib.py`, `KP_SPEED/KI_SPEED/MAX_THRUST` dans `thrust_mixer_node.py` (KP_HEADING=3000 validé).
3. Vérifier le comportement de la transition cas 1 → cas 2 (hysteresis `switch_value` 0/3) et le "docking" final (`start`/`_active` latch à 0.1 m).
4. Nettoyer les bridges du launch file si des doublons apparaissent avec ceux déjà auto-générés par VRX (`payload_bridges_impl`).

## Perf : real-time factor bas en simulation — cause identifiée
Symptôme rapporté (2026-08-17) : la simulation tourne lentement (RTF < 1).

**Cause** : `vrx_gz/model.py::xacro_cmd()` force `vrx_sensors_enabled:=true` sans condition (ligne 239), ce qui active dans `wamv_gazebo.urdf.xacro` (bloc `vrx_sensors_enabled`, lignes 156-184) 3 caméras (stéréo + latérale) + un lidar 16 faisceaux + un pinger sur **chaque** WAM-V — soit 6 caméras et 2 lidars rendus à chaque pas de physique pour les 2 WAM-V du scénario (`wamv`+`wamv2`), alors que le pipeline `vrx_apf_dock` ne consomme que l'odométrie (`ground_truth_enabled`) et l'IMU. Le rendu caméra/lidar est de loin la partie la plus coûteuse de gz-sim, cause quasi certaine du RTF dégradé.

**Fix appliqué** (`vrx_gz/src/vrx_gz/model.py`, dépôt vendored, symlink-installé donc effectif sans rebuild) : `vrx_sensors_enabled:=false` (au lieu de `true`) + ajout explicite de `imu_enabled:=true` (l'IMU n'est ajoutée que via `imu_enabled` seul ou dans le bloc `vrx_sensors_enabled` — sans ce flag séparé, désactiver `vrx_sensors_enabled` aurait aussi supprimé l'IMU dont dépend `state_estimator_node`). `ground_truth_enabled` reste forcé à `true` (odométrie toujours nécessaire).

**Non encore validé en simulation réelle** : à confirmer via `gz stats` / `WorldStats` (GUI) ou les logs `ros2 launch` que le RTF remonte proche de 1.0 après ce changement.

## Réduction de taille du wamv piloté (facteur d'échelle 0.3)
Demande (2026-08-17) : réduire la taille du `wamv` (le robot piloté), pas du `wamv2` (dock). D'abord exploré l'idée d'un `model_type: usv` distinct pour le pilote — abandonné : aucun modèle USV alternatif n'existe dans ce workspace (ni package ROS, ni cache Fuel local), et `model_type` n'est qu'un label de catégorie dans `vrx_gz` — `Model.generate()` retombe toujours sur le même xacro `wamv_gazebo.urdf.xacro` tant que `self.urdf` n'est pas explicitement pointé ailleurs. Retenu : garder `wam-v` pour les deux robots, mais rendre le xacro paramétrable par un facteur d'échelle uniforme.

**Mécanisme ajouté** : nouvel arg xacro `scale` (défaut `1.0`), propriété globale propagée à tous les fichiers inclus (même pattern que `namespace` déjà existant). Passé depuis `model.py::xacro_cmd()` uniquement pour les `model_type` USV (`if self.model_type in USVS`), lu depuis un nouveau champ optionnel `scale:` dans la config yaml (`Model._FromConfigDict`, suit le pattern déjà utilisé pour `payload`/`flight_time`). Dans `two_wamv.yaml`, `scale: 0.3` ajouté **uniquement** sur l'entrée `wamv` — `wamv2` (dock) reste à l'échelle 1 (taille réelle).

**Fichiers vendored modifiés** (tous dans `src/vrx`, non commités) :
- `vrx_gz/src/vrx_gz/model.py` : plumbing `scale` (attribut, `set_scale()`, arg xacro conditionnel, parsing yaml).
- `vrx_gz/config/two_wamv.yaml` : `scale: 0.3` sur `wamv`.
- `vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro` : arg/propriété globale `scale`, position des batteries et offset IMU multipliés par `scale`.
- `vrx_urdf/wamv_description/urdf/wamv_base.urdf.xacro` : mesh (`<scale>`), toutes les origines/géométries de collision, masse (`×scale³`) et inertie (`×scale⁵`) du hull — **a aussi son propre arg/propriété `scale` local** (comme `namespace` déjà présent), nécessaire car ce fichier est aussi xacro-généré **au moment du build CMake** (`xacro_add_files` dans `wamv_description/CMakeLists.txt`), indépendamment de l'inclusion depuis `wamv_gazebo.urdf.xacro`.
- `vrx_urdf/wamv_description/urdf/battery.xacro` : mesh, collision, masse/inertie des 2 batteries scalées pareil (sinon des batteries de 23.5kg sur une coque de 4.86kg à l'échelle 0.3 auraient été absurdes).
- `vrx_urdf/wamv_gazebo/urdf/dynamics/wamv_gazebo_dynamics_plugin.xacro` : `hull_length`/`hull_radius`/`points` (×scale) du plugin `vrx::Surface` (flottaison) ; coefficients de traînée linéaire/quadratique du plugin `vrx::SimpleHydrodynamics` (×scale², approximation proportionnelle à la surface frontale — **pas** un scaling de Froude rigoureux, ce sont des coefficients empiriques).
- `vrx_urdf/wamv_gazebo/urdf/thruster_layouts/wamv_aft_thrusters.xacro` : position des 2 moteurs (×scale).
- `vrx_urdf/wamv_description/urdf/thrusters/engine.xacro` : mesh, collisions, masse/inertie moteur+hélice, offset du joint hélice, tous scalés pareil.
- `vrx_urdf/wamv_gazebo/urdf/thruster_layouts/wamv_gazebo_thruster_config.xacro` : `propeller_diameter` (×scale) ; `x_u`/`x_uu` (×scale², mêmes coefficients de traînée utilisés dans la formule `max_thrust_cmd` — cohérent avec le scaling du plugin hydrodynamique) → `max_thrust_cmd` passe automatiquement de 2353N à ~212N par moteur.

**Piège rencontré** : après les premières modifs, la simulation continuait à spawn le wamv à taille réelle malgré `scale:=0.3` passé en argument xacro — `xacro` résolvait `$(find wamv_gazebo)`/`$(find wamv_description)` vers `install/share/...`, qui sont des **copies statiques non symlinkées** (contrairement à `vrx_gz`, package `ament_python` qui lui est bien symlink-installé). Il a fallu `colcon build --packages-select wamv_gazebo wamv_description --merge-install --symlink-install` pour que les éditions du xacro source prennent effet. Après ce rebuild ponctuel, les fichiers `urdf/` sont bien symlinkés (`install(DIRECTORY urdf/ ...)` supporte le symlink install), donc les futures éditions de ces `.xacro` seront à nouveau effectives sans rebuild — seul ce premier rebuild était nécessaire pour "activer" le symlink.

**Résultat vérifié** (génération xacro + `gz sdf -p` via `Model.generate()`, en dehors de Gazebo) : `wamv` (scale 0.3) → masse lumpée `base_link` ≈ 6.13 kg (hull 4.86 + 2×0.6345 batteries), `hull_length` 1.47m, `max_thrust_cmd` ≈ 212N/moteur. `wamv2` (scale 1.0, dock) → masse ≈ 227 kg (hull 180 + 2×23.5 batteries), `hull_length` 4.9m, `max_thrust_cmd` ≈ 2353N/moteur — identique à l'original, non affecté.

**Non encore fait / à re-régler après ce changement** : tous les gains (`c11,c12,c21,c22` dans `apf_lib.py`, PID `KP_HEADING/KI_HEADING/KD_HEADING/KP_SPEED/KI_SPEED/KD_SPEED` dans `thrust_mixer_node.py`, `VMAX`, `SAFETY_RADIUS`/`SAFETY_GAIN`, `REACHED_RADIUS`) ont été calibrés pour le wamv à taille réelle — avec un hull 3.3× plus petit et ~30× plus léger, la dynamique (accélération, inertie de lacet, temps de réponse) change significativement. Passage à vide en simulation réelle (Gazebo) pas encore fait à ce stade — seule la génération SDF a été vérifiée hors-Gazebo.

## Nouveau package `vrx_apf_dock_teleop` : réglage des PID au joystick (2026-08-18)
Besoin : pouvoir piloter `thrust_mixer_node` directement au joystick (vitesse + cap) pour régler ses gains PID sans dépendre de la loi APF (`apf_controller_node`) pendant le réglage.

**Design** : nouveau package `vrx_apf_dock_teleop`, un seul nœud `joystick_setpoint_node.py`, qui publie sur le **même** topic hijacké que `apf_controller_node` (`/vrx_apf_dock/wamv/setpoint`, `Twist` : `linear.x=vbar`, `angular.z=thetabar ABSOLU`) — donc `thrust_mixer_node` n'a besoin d'aucune modification et ne sait pas d'où vient la consigne. **Ne jamais lancer en même temps que `apf_controller_node`** (même topic, publications concurrentes) → nouveau launch file dédié `apf_dock_teleop.launch.py` qui reprend le monde/spawn/bridges de `apf_dock.launch.py` mais remplace `apf_controller_node` par `joy_node` (package ROS `joy`) + `joystick_setpoint_node`.

**Mapping** (aligné sur la convention déjà utilisée pour le téléop direct-thrust dans `vrx_gz/config/wamv.yaml`) :
- Stick gauche vertical (axe 1) → vitesse (`linear.x`), échelle `max_speed` (param ROS, défaut 2.0 m/s).
- Stick droit horizontal (axe 2) → vitesse de rotation façon palonnier, intégrée dans le temps en une consigne de cap **absolue** (`thrust_mixer_node` attend un cap absolu, pas une vitesse angulaire) — échelle `max_heading_rate` (param ROS, défaut 1.0 rad/s).
- Bouton mort (deadman) 4 (L1, même convention que `wamv.yaml`) : relâché → coupure nette (`speed_cmd=0`, même convention que le latch `reached` de `thrust_mixer_node`) et cap **gelé** (pas remis à 0) pour ne pas faire virer le bateau vers un cap obsolète en ré-appuyant.

**Pas encore fait** : build/test réel (aucun conteneur Docker `vrx-devel-custom` actif au moment de l'implémentation — build/lancement à faire via le workflow habituel, cf. section "Environnement d'exécution" plus haut). Seule la syntaxe Python a été vérifiée (`py_compile`).

## Réglage des gains PID en direct via rqt_reconfigure (2026-08-18)
Besoin : sliders pour ajuster les gains PID de `thrust_mixer_node.py` pendant que la simulation tourne. Deux options possibles : vrai plugin GUI natif Gazebo (QML/C++, lourd à écrire et recompiler) vs paramètres ROS 2 + `rqt_reconfigure` (outil standard, sliders, pas besoin de code custom) — la seconde a été retenue.

**Changement** : les anciennes constantes module (`KP_HEADING`, `KI_HEADING`, `KD_HEADING`, `HEADING_INTEGRAL_LIMIT`, `KP_SPEED`, `KI_SPEED`, `KD_SPEED`, `SPEED_INTEGRAL_LIMIT`, `MAX_THRUST`) sont devenues des paramètres ROS déclarés (`declare_parameters`) dans `ThrustMixerNode.__init__`, avec les mêmes valeurs par défaut. Un callback (`add_on_set_parameters_callback`) met à jour un dict `self._gains` à chaque changement, lu dans `_mix_and_publish` à chaque cycle — donc réglable en direct sans relancer le nœud.

**Usage** : lancer normalement (`ros2 launch vrx_apf_dock apf_dock.launch.py` ou le nouveau `apf_dock_teleop.launch.py`), puis dans un autre terminal du conteneur : `ros2 run rqt_reconfigure rqt_reconfigure` (paquet `ros-jazzy-rqt-reconfigure`, à installer si absent), sélectionner `thrust_mixer_node` dans la liste, et les 9 gains apparaissent en sliders. Alternative en ligne de commande : `ros2 param set /thrust_mixer_node kp_heading 900.0`.

**Non fait / hors scope** : les paramètres de la loi APF (`c11,c12,c21,c22`, `VMAX`, etc. dans `apf_lib.py`) ne sont pas concernés — ce ne sont pas des gains PID à proprement parler, et la demande portait spécifiquement sur "les PID". À faire pareil si besoin un jour (nécessiterait de les exposer comme paramètres sur `apf_controller_node` et de les répercuter dans `ApfDockController`).

## Consignes préréglées au joystick : vconf/phiconf (2026-08-18)
Ajout dans `joystick_setpoint_node.py` (package `vrx_apf_dock_teleop`) de deux nouveaux paramètres ROS, réglables en direct via `rqt_reconfigure` (même mécanisme dict+callback que `thrust_mixer_node.py`) :
- `vconf` (m/s, défaut 1.0) : tant que RB/R1 (bouton 5) est maintenu, la vitesse commandée est fixée à `vconf` au lieu de suivre le stick gauche — entrée en échelon précise et répétable pour observer la réponse du PID de vitesse.
- `phiconf` (deg, défaut 15.0) : chaque appui sur RT/R2 (bouton 7, détection de front montant, pas maintenu) décale la consigne de cap absolue de `+phiconf` degrés — entrée en échelon pour la réponse du PID de cap.

Les deux actions sont gated sur le deadman (bouton 4) : sans deadman, RB/RT restent inertes (cohérent avec le hard-stop existant). `phiconf` utilise une détection de front (`_heading_step_prev_held`) plutôt qu'un maintien, pour n'appliquer le saut qu'une fois par appui.

## Bascule manuel/docking au bouton LT (2026-08-18)
Besoin : depuis `vrx_apf_dock_teleop`, pouvoir basculer en direct entre pilotage manuel (cap/vitesse au joystick) et pilotage automatique (loi APF), au bouton LT/L2.

**Problème initial** : `apf_controller_node` et `joystick_setpoint_node` publiaient tous deux directement sur `/vrx_apf_dock/wamv/setpoint` (celui lu par `thrust_mixer_node`) — les faire tourner en même temps aurait fait s'affronter leurs publications.

**Solution retenue** : `apf_controller_node` publie maintenant sur `/vrx_apf_dock/wamv/setpoint_apf` (nouveau nom, toujours calculé en continu, sans savoir si quelqu'un l'utilise). `joystick_setpoint_node` devient l'arbitre unique du topic final `/vrx_apf_dock/wamv/setpoint` : selon l'état `_docking_active` (basculé par LT/L2, bouton 6, détection de front), il republie soit son propre calcul manuel, soit tel quel le dernier message reçu sur `setpoint_apf`.
- `apf_dock.launch.py` (docking seul, sans teleop) : `apf_controller_node` lancé avec un `remapping` `setpoint_apf -> setpoint` pour publier directement dessus (pas d'arbitre nécessaire dans ce launch).
- `apf_dock_teleop.launch.py` : lance maintenant `apf_controller_node` (non-remappé) **en plus** de `joy_node`/`joystick_setpoint_node`, pour permettre la bascule.

**Détails de comportement** :
- Le deadman (bouton 4/L1) ne conditionne que le mode manuel — en mode docking, LT est le seul contrôle d'engagement/désengagement (comme un vrai bouton d'autopilote), pas besoin de le maintenir.
- Au retour en manuel (2ᵉ appui LT), le cap de consigne du palonnier virtuel est re-semé depuis le yaw réel courant du bateau (`_current_yaw`, suivi en continu désormais, séparé de `_heading_cmd`) — sinon `_heading_cmd` serait resté figé à sa valeur d'avant l'engagement du docking, provoquant un virage brusque vers un cap obsolète.
- RT (saut de cap `phiconf`) est désactivé pendant le mode docking (n'a de sens qu'en pilotage manuel).

**Limite résolue — reset au ré-engagement** : `apf_controller_node` écoute maintenant `/vrx_apf_dock/apf/reset` (`std_msgs/Empty`, générique, sans référence au teleop) et appelle `self._apf.reset()` à réception. `joystick_setpoint_node` publie dessus exactement à la transition manuel→docking (pas à chaque appui, seulement quand `_docking_active` passe à `True`), et vide aussi son propre `_apf_setpoint` mis en cache à ce moment-là (pour que `_publish_setpoint` fasse un hard-stop d'un tick le temps qu'un setpoint post-reset arrive, plutôt que de republier un ancien setpoint potentiellement déjà "reached").

## Gains APF (c11/c12/c21/c22/...) exposés dans rqt (2026-08-18)
Même mécanisme que pour `thrust_mixer_node.py` : les anciennes constantes module de `apf_lib.py` (`c11, c12, c21, c22, VMAX, MARGIN, TRANSITION_DIST, REACHED_RADIUS, SAFETY_RADIUS, SAFETY_GAIN`) sont devenues un dict `DEFAULT_GAINS` (clés en snake_case : `c11, c12, c21, c22, vmax, margin, transition_dist, reached_radius, safety_radius, safety_gain`).

**Changement d'API** : `ApfDockController.__init__(self, gains=None)` — chaque instance porte désormais son propre `self.gains` (dict, copie de `DEFAULT_GAINS` par défaut), lu à chaque appel de `compute()` au lieu des anciennes constantes globales. `apf_controller_node.py` déclare ces 10 valeurs comme paramètres ROS (`declare_parameters` + `add_on_set_parameters_callback`, identique au pattern de `thrust_mixer_node.py`), et le callback écrit directement dans `self._apf.gains[...]` — donc réglable en direct dans `rqt_reconfigure` sous `apf_controller_node`, à côté de `thrust_mixer_node` pour les PID.

Testé hors-ROS (`ApfDockController().compute(...)` + modif de `gains['vmax']` en direct) : la sortie change bien immédiatement, pas besoin de recréer l'instance.

## Visualisation du champ de vecteurs APF (2026-08-18)
Nouveau script standalone `vrx_apf_dock/scripts/plot_apf_field.py` (sans ROS/Gazebo, matplotlib + numpy, même esprit que les tests Python isolés déjà mentionnés dans ce journal) : échantillonne `ApfDockController.compute()` sur une grille de 1m sur un carré 10×10m centré sur le dock, **une instance fraîche du contrôleur par point** (sinon l'hystérésis `_switch_value`/`_active` d'un point contaminerait le suivant selon l'ordre d'échantillonnage — non représentatif d'un vrai champ statique). Flèche = direction `theta_cmd`, longueur/couleur = `v_cmd` (donc déjà clampé/amorti, pas le vecteur potentiel brut). CLI : `--size`, `--step`, `--dock-heading-deg`, `--out`, plus un override par gain (`--c11`, `--vmax`, etc.). Sortie : `apf_field.png` dans le même dossier.

**Découverte via la visualisation** : sur toute la droite x=0 (perpendiculaire au cap du dock, passant par le dock, pour `theta_dock=0`), `v_cmd` tombe exactement à 0 — visible comme une colonne de flèches quasi invisibles au centre du champ. Cause : dans la branche avant (`front_branch`), le facteur `k = -sign(unit @ (p_dock - p_robot))` vaut `sign(0) = 0` exactement sur cette droite (ambiguïté avant/arrière), ce qui annule `v_cmd = min(|v_vec|, k * ...)` quel que soit `v_vec`. C'est un cas dégénéré de la loi elle-même (droite de mesure nulle, différent du cas de spawn quasi-x=0 déjà documenté plus haut qui lui était dû au bruit flottant sur `back_branch`) — pas un bug du script de visualisation. Non corrigé, à la demande de l'utilisateur si besoin un jour (probablement sans conséquence pratique : probabilité nulle de tomber exactement sur cette droite en simulation réelle).

## Sliders interactifs pour plot_apf_field.py (2026-08-18)
`plot_apf_field.py` devient interactif par défaut (`matplotlib.widgets.Slider`, backend `TkAgg`/`Qt5Agg` disponibles sur l'hôte) : 4 sliders pour `c11` (0-200), `c12` (0-500), `c21` (0-100), `c22` (0-500), redessinant le champ en direct (`quiv.set_UVC(...)` + re-titre, pas de refonte de figure). Taille par défaut passée à 15×15m (`--size 15`). Le mode statique précédent (PNG, sans display) reste disponible via `--static --out fichier.png`, utile en conteneur/headless. Code factorisé (`compute_field`, `setup_axes`, `title_for`) pour éviter la duplication entre les deux modes.

## Bug corrigé : `plt` non défini dans `setup_axes` (2026-08-18)
En testant les sliders headlessement (backend `Agg`, `slider.set_val(...)` simulé sans vraie fenêtre), `NameError: name 'plt' is not defined` dans `setup_axes` (utilisait `plt.Circle`). Cause : `plt` n'était importé que localement dans `run_static`/`run_interactive` via un `global plt; import matplotlib.pyplot as plt` fragile, mais `setup_axes` (fonction séparée) en dépendait aussi sans garantie d'ordre.

**Fix** : `Circle` importé directement depuis `matplotlib.patches` en haut du fichier (indépendant du backend, pas besoin de `plt` du tout) ; suppression du hack `global plt` dans les deux fonctions `run_*`, chacune garde son propre import local de `pyplot` après avoir choisi son backend.

**Vérifié** : simulation headless (`matplotlib.use('Agg')` + `slider.set_val(...)` pour déclencher `on_changed` sans display réel) confirme que `quiv.U`/titre changent bien à chaque coefficient modifié (`c22`, puis `c11`, testés indépendamment).

## Champ de vecteurs APF affiché en direct dans Gazebo (2026-08-18)
Équivalent en direct, dans le rendu Gazebo, de `scripts/plot_apf_field.py` (matplotlib, hors-ligne) : nouveau nœud `apf_field_marker_node.py`, dessine une grille de petites flèches (via `/marker_array`, GUI only) autour de la position réelle (et possiblement dérivante) du dock, une par point de grille, chaque flèche calculée avec une **instance fraîche** de `ApfDockController` (même raison que le script matplotlib : éviter que l'hystérésis d'un point contamine le suivant).

**Refactor** : extraction des helpers de dessin de flèches (`_direction_quaternion`, `_set_color`, `_make_segment_marker`, `_make_arrow_markers`, l'appel `/marker_array`) depuis `vector_marker_node.py` vers un nouveau module partagé `gz_marker_utils.py` (fonctions génériques prenant `ns` et `shaft_length` explicites au lieu de constantes de module figées), pour être réutilisées par les deux nœuds sans duplication. `vector_marker_node.py` garde sa propre logique de `_shaft_length(speed)` (vitesse → longueur, spécifique à ses 2 grosses flèches de navigation) et l'utilise via le module partagé.

**Synchronisation des gains** : plutôt qu'une copie indépendante des gains APF (comme le fait `plot_apf_field.py`), `apf_field_marker_node` interroge périodiquement (`update_period`, défaut 2s) le service `/apf_controller_node/get_parameters` pour récupérer les 10 gains **réellement utilisés** par le contrôleur en cours d'exécution — donc le champ affiché dans Gazebo suit en direct les réglages faits via `rqt_reconfigure` sur `apf_controller_node`. Dégrade proprement (garde les derniers gains connus / les défauts de `apf_lib.DEFAULT_GAINS`) si ce nœud n'est pas encore up.

**Visuel** : flèches petites (rayon fût 0.03m, tête 0.15m) pour tenir dans une maille de `grid_step=1m` par défaut, longueur plafonnée par `max_shaft_length` (0.6m) et proportionnelle à `|v_cmd|/vmax`, couleur interpolée bleu (lent) → jaune (rapide). Paramètres ROS : `grid_size` (15m), `grid_step` (1m), `update_period` (2s), `max_shaft_length` (0.6m).

**Ajouté aux deux launch files** (`apf_dock.launch.py` et `apf_dock_teleop.launch.py`), à côté de `vector_marker_node`. Même limite déjà documentée pour les marqueurs Gazebo : invisible en `headless:=True` (plugin `MarkerManager` GUI-only), pas un bug.

Vérifié dans le conteneur (imports réels `gz.transport13`/`gz.msgs10`, construction de marqueurs, interpolation de couleur) — pas encore observé en simulation réelle (GUI) à ce stade.

## Pilotage direct du dock au joystick, bouton A (2026-08-18)
Besoin : déplacer manuellement le dock (`wamv2`, normalement libre/jamais commandé) au joystick, en maintenant la touche A.

**Choix de conception** : contrôle en boucle ouverte (pas de PID), contrairement au wamv. Raison : `state_estimator_node` ne produit une estimation de vitesse (par intégration IMU) que pour le `wamv` — le dock n'a qu'une pose brute (`/vrx_apf_dock/dock/pose`, sans vitesse), donc pas de boucle fermée possible sans dupliquer toute la chaîne IMU→vitesse pour lui aussi. Vu que le besoin est juste de pouvoir repositionner le dock à la main (pas de manœuvre autonome du dock lui-même), le mapping direct stick→poussée suffit et évite cette duplication.

**Implémentation** (`joystick_setpoint_node.py`) : bouton A (bouton 1) maintenu redirige les *mêmes* deux sticks (gauche vertical = poussée totale, droit horizontal = différentiel/virage) vers `/wamv2/thrusters/{left,right}/thrust` au lieu du setpoint du wamv — deux nouveaux paramètres ROS `dock_max_thrust` (300N) et `dock_max_diff_thrust` (150N) bornent l'échelle. Un second timer dédié (20Hz, comme le timer de setpoint existant) publie ces commandes en continu.

**Point d'attention** : contrairement au setpoint hijacké du wamv (où l'absence de deadman fait tomber `speed_cmd` à 0 puis coupe proprement via `thrust_mixer_node`), ici on écrit *directement* les topics de poussée bas niveau — `gz-sim-thruster-system` retient la dernière commande reçue indéfiniment, sans timeout. Donc relâcher A doit *activement* publier du zéro (pas juste arrêter de publier), sinon le dock resterait poussé au dernier niveau commandé au lieu de redevenir libre. Fait : `_publish_dock_thrust` publie explicitement `0.0/0.0` tant que A n'est pas maintenu.

## Journal
- 2026-07-13 : Exploration du repo (`controller.py`, `roblib.py`, `two_wamv.yaml`, bridges VRX, xacro WAM-V, model.py). Constat clé : pas de `cmd_vel` pour WAM-V, thruster_config forcé à `H`, IMU déjà active par défaut (`vrx_sensors_enabled:=true` codé en dur), `locked` mort, plugin d'attache no-op sans modèle `platform`. Décisions validées avec l'utilisateur (propulsion, dock libre, monde, estimation vitesse, portage nettoyé, 3 noeuds, nom de package). Implémentation complète du package `vrx_apf_dock` (apf_lib, 3 noeuds, launch). Découverte que l'exécution doit se faire dans le conteneur Docker `vrx-devel-custom` (ROS Jazzy + Gazebo Harmonic), pas sur l'hôte (Humble + Gazebo Classic). Installation de `python3-sdformat14`/`libsdformat14` sur l'hôte (dépendance de `vrx_gz`, sans rapport avec l'exécution en conteneur mais nécessaire pour les imports Python). Découverte que `/model/<name>/pose` et `dynamic_pose/info` ne donnent pas la pose monde exploitable ; activation de `ground_truth_enabled` dans `vrx_gz/model.py` (modif du repo vendored) pour utiliser `OdometryPublisher` → `/model/<name>/odometry`. Test bout-en-bout headless réussi : le wamv se dirige vers le dock avec des commandes de poussée cohérentes.
