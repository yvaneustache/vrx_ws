# Intégration VRX + ArduPilot + MAVROS — Journal de développement

## Contexte
Projet séparé du docking APF (`vrx_apf_dock`/`vrx_apf_dock_teleop`) : faire piloter le WAM-V de VRX par ArduPilot Rover SITL (via le plugin `ArduPilotPlugin` d'`ardupilot_gazebo`) au lieu de la pile de contrôle custom (`thrust_mixer_node`/`apf_controller_node`), puis exposer ça à ROS2 via MAVROS.

**État trouvé au démarrage** (2026-08-19) : `ardupilot` (dépôt officiel) et `ardupilot_gazebo` déjà clonés et compilés dans `vrx_ws/`, mais seulement testés avec les modèles de démo du dépôt (`iris`, `zephyr`), jamais intégrés au WAM-V/VRX. Sessions SITL déjà lancées par ailleurs (logs Mission Planner, `mav.tlog`). Seul `ros-jazzy-mavros-msgs` (les messages) est installé, pas `mavros` (le nœud) lui-même.

## Nouveau package `vrx_ardupilot`
Package séparé (`src/vrx_ardupilot`, `ament_python`) plutôt que de toucher `vrx_apf_dock` : config de spawn (`config/ardupilot_wamv.yaml`, un seul WAM-V) + launch (`launch/ardupilot_wamv.launch.py`, monde + spawn + bridge d'odométrie minimal, sans les nœuds de contrôle APF).

## Modifications au dépôt vendored `src/vrx` (partagées, additives, opt-in)
Choix : réutiliser le modèle physique du WAM-V existant (coque, hydrodynamique) plutôt que d'en dupliquer un, via un nouveau flag xacro `ardupilot_enabled` (défaut `false` — comportement de `vrx_apf_dock` totalement inchangé, vérifié par régression).

- **`vrx_gz/src/vrx_gz/model.py`** : nouveau champ `ardupilot` (pattern identique à `scale` ajouté précédemment) — `set_ardupilot()`, parsing du champ yaml `ardupilot: true`, et dans `xacro_cmd()` : force `thruster_config:=ArduPilot` (au lieu de `H`) + `ardupilot_enabled:=true` quand actif.
- **Nouveau `wamv_gazebo/urdf/thruster_layouts/wamv_aft_thrusters_ardupilot.xacro`** : mêmes moteurs/hélices physiques que le layout `H`, mais **sans** le plugin `gz-sim-thruster-system` — ce plugin commande activement la vitesse du joint hélice à chaque pas (0 par défaut si rien ne publie dessus), ce qui se serait battu avec `ArduPilotPlugin` pilotant directement le même joint.
- **`wamv_gazebo.urdf.xacro`** : nouvelle branche `thruster_config == 'ArduPilot'` dans la chaîne if/elif existante (H/T/X) ; nouvel arg `ardupilot_enabled` ; bloc `<gazebo><plugin name="ArduPilotPlugin">` conditionnel en fin de fichier, avec `channel="0"` → `left_engine_propeller_joint` (SERVO1/ThrottleLeft) et `channel="2"` → `right_engine_propeller_joint` (SERVO3/ThrottleRight), conforme à la config skid-steer bateau d'ArduPilot Rover (`FRAME_CLASS=2` Boat, `SERVO1_FUNCTION=73`, `SERVO3_FUNCTION=74` — trouvés dans `ardupilot/Tools/autotest/default_params/rover-skid.parm`, référencés par le frame SITL `gazebo-rover`). `multiplier`/`cmd_max` sont des valeurs non calibrées, à régler une fois le bateau réellement piloté.

**Piège rencontré** : les commentaires XML interdisent `--` n'importe où dans le corps (seulement en début/fin `<!--`/`-->`) — plusieurs commentaires écrits avec des `--` façon prose ont fait planter `xacro` (`XML_ERROR_EMPTY_DOCUMENT` en aval, même symptôme trompeur que le bug `imu_enabled`+`vrx_sensors_enabled` dupliqué documenté dans `260713_development.md`). Corrigé en remplaçant par `;`/`,`.

**Piège rencontré (build)** : `libArduPilotPlugin.so` n'était compilé que dans `src/ardupilot_gazebo/build/` (build manuel, jamais passé par `colcon build`), donc absent de `install/lib` où `GZ_SIM_SYSTEM_PLUGIN_PATH` pointe — Gazebo ne le trouvait pas. `colcon build --packages-select ardupilot_gazebo --merge-install --symlink-install` corrige ça (le hook d'environnement du package ajoute `install/lib/ardupilot_gazebo/` au path automatiquement après re-source).

## Test de spawn réel (headless, `ros2 launch vrx_gz competition.launch.py ... config_file:=ardupilot_wamv.yaml`)
- ✅ SDF généré proprement, `gz sdf -p` OK.
- ✅ `ArduPilotPlugin` chargé (`Loaded system [ArduPilotPlugin] for entity [143]`).
- ✅ Aucune régression sur `two_wamv.yaml` (le spawn du projet docking génère toujours un SDF correct, sans `ArduPilotPlugin`, avec `gz-sim-thruster-system` intact).
- ❌ **Bloquant actuel** : `[wamv] imu_sensor [wamv::wamv/imu_wamv_link::imu_wamv_sensor] not found, abort ArduPilot plugin.`

## Diagnostic du blocage IMU (lecture du source `ArduPilotPlugin.cc`)
Le nom scoped utilisé (`wamv::wamv/imu_wamv_link::imu_wamv_sensor`) correspond exactement aux noms littéraux du SDF généré (vérifié) — **pas un bug de nommage**. La vraie cause : ce lookup a lieu dans `PreUpdate()`, gardé par `if (!imuInitialized)` avec `imuInitialized` mis à `true` **inconditionnellement avant** la recherche (commentaire du code : *"we're only going to try this once"*) — donc si l'entité capteur IMU n'existe pas encore dans l'ECS au tout premier `PreUpdate()` du plugin, l'échec est **définitif** (pas de nouvelle tentative).

Hypothèse forte : c'est un problème d'ordonnancement propre au **spawn dynamique** (le WAM-V est créé en cours de simulation via le service `ros_gz_sim create`, après que le monde tourne déjà), alors que les exemples qui marchent dans `ardupilot_gazebo` (`iris_with_ardupilot`, etc.) sont inclus **statiquement dans le fichier monde**, chargés en même temps que tout le reste au démarrage — le système `Sensors` a alors le temps de créer l'entité IMU avant que `ArduPilotPlugin::PreUpdate()` ne s'exécute pour la première fois. Pas encore confirmé avec certitude, juste la piste la plus probable au vu du code.

## Fix retenu : patch `ardupilot_gazebo` (option la plus simple)
Entre patcher `ardupilot_gazebo` (une fonction, quelques lignes, un rebuild du plugin) et basculer vers une inclusion statique dans un monde dédié (change l'architecture de lancement, perd l'abstraction de spawn `vrx_gz`), la première option est nettement la plus simple — retenue.

**Modification** (`src/ardupilot_gazebo/src/ArduPilotPlugin.cc`, fonction `PreUpdate()`) : `imuInitialized` n'est plus mis à `true` inconditionnellement *avant* la recherche du capteur IMU (commentaire d'origine : *"we're only going to try this once"*), mais seulement *après* une recherche réussie (juste avant `node.Subscribe(imuTopicName, ...)`). En cas d'échec (`entities.empty()`), on `return` simplement sans lever le flag — le prochain `PreUpdate()` retente. Supprime le message d'erreur `"imu_sensor [...] not found, abort ArduPilot plugin."` (qui n'était de toute façon qu'un abandon de la seule partie IMU du plugin, pas du plugin entier — le reste de `PreUpdate()` continuait déjà). Le message équivalent pour l'anémomètre (`anemometerName`, plus haut dans le fichier) n'a pas été touché — non concerné par ce projet.

**Rebuild** : `colcon build --packages-select ardupilot_gazebo --merge-install --symlink-install` (recompile bien le C++, ~5s, confirmé par le timestamp du `.so` et par la disparition du message d'erreur IMU dans les `strings` du binaire — seul le message anémomètre, textuellement proche, reste).

**Retest headless confirmé** : plus aucune erreur ; log confirme `[ImuSensor.cc:150] IMU data for [wamv::wamv/imu_wamv_link::imu_wamv_sensor] advertised on [.../imu]` — le capteur est bien créé et trouvé (sur un tick ultérieur au premier, exactement l'hypothèse de timing de spawn dynamique). `ArduPilotPlugin` chargé sans erreur, prêt à accepter une connexion FDM.

## MAVROS installé (2026-08-26)
`sudo apt-get install -y ros-jazzy-mavros ros-jazzy-mavros-extras` + `sudo bash /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh` (datasets EGM96 geoid/gravity/magnetic requis par mavros, installés dans `/usr/share/GeographicLib/`). Les deux confirmés présents.

## Tentative de connexion SITL ↔ Gazebo (2026-08-26) : bloquée, cause identifiée avec précision

**Piège n°1 (résolu)** : `sim_vehicle.py` doit être lancé avec le python du venv existant (`my-venv/bin/python3`, qui a `pexpect`/`pymavlink`) — le python système n'a pas `pexpect`.

**Piège n°2 (résolu)** : `-f gazebo-rover` seul ne suffit pas. Dans `vehicleinfo.json`, l'entrée `gazebo-rover` n'a pas de clé `"model"` explicite ; `vehicleinfo.py::options_for_frame()` (ligne ~50) fait alors `ret["model"] = frame`, donc `--model gazebo-rover` est passé tel quel au binaire `ardurover`. Côté ArduPilot, le préfixe `"gazebo"` du nom de modèle sélectionne l'ANCIEN backend legacy `SITL::Gazebo` (`libraries/SITL/SIM_Gazebo.cpp`, protocole simple `float motor_speed[16]`, 64 octets, port de retour fixe 9003) — **incompatible** avec le protocole JSON moderne (magic number + frame_rate + frame_count + pwm[]) qu'attend `ardupilot_gazebo`'s `ArduPilotPlugin.cc`. C'est ce protocole JSON qu'utilise la session ArduCopter/iris déjà fonctionnelle de l'utilisateur (`--model JSON` explicite dans sa commande). **Fix** : passer `--model JSON` explicitement (`options_for_frame()` : `if opts.model is not None: ret["model"] = opts.model`, l'override fonctionne bien).

**Diagnostic réseau (tcpdump)** avec `--model JSON` :
- ArduPilot envoie bien des paquets UDP de 40 octets vers `127.0.0.1:9002` (`Starting SITL: JSON`, `JSON control interface set to 127.0.0.1:9002`, confirmé par capture) — taille et magic number (18458 pour 16 canaux) vérifiés identiques des deux côtés du code (`SIM_JSON.h` vs `ArduPilotPlugin.cc`).
- **Mais Gazebo ne répond jamais** : aucun paquet retour `9002 -> ...` capturé, et aucun `gzwarn`/`gzerr` correspondant (ni "Incorrect protocol magic", ni "Duplicate input frame") dans le log Gazebo pendant ces essais — signe que `getServoPacket()` (le `recv()` côté plugin) ne "voit" tout simplement pas ces paquets pourtant confirmés arrivés au niveau OS/loopback par tcpdump, alors que le port 9002 est bien celui sur lequel Gazebo est bindé (`netstat` confirme `4136/gz sim` dessus).
- Côté ArduPilot (`Rover.log`) : `"No JSON sensor message received, resending servos"` toutes les ~1.1s — cohérent avec un pattern lockstep "envoie puis attend une réponse, retente après timeout faute de réponse", donc bien un problème côté réception Gazebo, pas d'émission ArduPilot.

**Non résolu** : pourquoi le `recv()` du plugin ne récupère pas des paquets pourtant confirmés présents sur le socket au niveau réseau. Champ de recherche restreint (format/magic vérifiés identiques, adresses/ports vérifiés cohérents) mais la cause exacte nécessiterait un débogage plus profond (ex. breakpoint gdb dans `ArduPilotPlugin::getServoPacket`/`Socket_native::recv`, ou comparer avec le comportement exact de la session iris/ArduCopter déjà fonctionnelle de l'utilisateur pour repérer une différence de configuration).

**État laissé en place** : le monde Gazebo (WAM-V + ArduPilotPlugin) tourne toujours en arrière-plan dans le conteneur pour permettre de reprendre le débogage sans tout relancer. Tous les processus SITL/tcpdump de test ont été nettoyés.

## Session iris relancée par l'utilisateur pour comparaison (2026-08-26, suite)

**Découverte n°1 (réelle, corrigée) : collision de port UDP entre les deux simulations.** `netstat` a montré `127.0.0.1:9002` bindé **simultanément** par mon monde WAM-V (`sydney_regatta.sdf`) et par la session iris de l'utilisateur (`iris_runway.sdf`) — les deux `gz sim` tournent sur le même hôte sans isolation de domaine de découverte gz-transport (pas de `GZ_PARTITION` distinct). Conséquence encore pire que prévu : mon spawn (`ros_gz_sim create`) a été **servi par le mauvais monde** — le `wamv` s'est retrouvé réellement créé dans `iris_runway` au lieu de `sydney_regatta` (confirmé via `gz topic -l` : topics `/world/iris_runway/model/wamv/...`). Entité stray supprimée proprement (`gz service ... /world/iris_runway/remove`), session iris de l'utilisateur non affectée.

**Fix** : lancer mon monde avec `GZ_PARTITION=vrx_ardupilot_test` (isolation complète du domaine de découverte gz-transport) + `fdm_port_in` changé de 9002 à **9012** dans le xacro (correspond à l'instance SITL 1 : `simulator_port_in/out += instance*10`, formule trouvée dans `AP_HAL_SITL/SITL_cmdline.cpp`). Après ce fix, spawn confirmé propre : `wamv` bien dans `sydney_regatta`, port 9012 bindé par mon propre process, aucune collision. Lancer ArduPilot avec `--model JSON -I1` en conséquence.

**Découverte n°2 (réelle, en cours) : le trafic réseau arrive mais n'est jamais traité.** Une fois le port isolé, `tcpdump` confirme qu'ArduPilot envoie bien des paquets JSON (40 octets, bon magic number) vers `127.0.0.1:9012` toutes les ~1.1s (motif de retry `"No JSON sensor message received, resending servos"`). Capture large (`gz sim` port 9002, session iris) confirme qu'un échange bidirectionnel sain existe ailleurs sur cette même machine à ~1kHz — donc le protocole/format n'est **pas** en cause. Testé avec un délai de 90s (pas juste un problème de timing/probabilité) : toujours aucun heartbeat.

**Diagnostic par `strace`** (tracing du process `gz sim` en direct) : **zéro appel `select()`/`pselect6`** de la fonction `SocketUDP::pollin()` (utilisée par `ArduPilotPlugin::getServoPacket()`) pendant toute la fenêtre observée — la chaîne `PreUpdate() → ReceiveServoPacket() → getServoPacket() → sock.recv() → pollin() → select()` n'est simplement jamais atteinte.

**Cause trouvée en relisant `PreUpdate()`** (`ArduPilotPlugin.cc` ligne ~1104) : ce code n'est atteint QUE dans la branche `else` d'un `if (!imuInitialized) { ... } else { ReceiveServoPacket(); ... }` — donc tant que la recherche de l'IMU (`entitiesFromScopedName`) échoue, on ne sort jamais du bloc IMU et on n'atteint jamais la logique FDM. Recherche du message `"Found IMU sensor with name [...]"` (la confirmation propre au plugin, PAS le message `ImuSensor.cc` qui confirme juste que Gazebo a créé le capteur — deux choses différentes que j'avais confondues) : **absent du log, dans toute la durée du test**. Mon fix précédent (retry au lieu d'abandonner après un seul essai) a supprimé le message d'erreur associé, donnant l'illusion à tort que "plus d'erreur = ça marche" alors que ça échouait silencieusement en boucle.

**Piste explorée : nom scoped `<imuName>` mal formé.** Le lien du WAM-V s'appelle littéralement `wamv/imu_wamv_link` (le `/` fait partie du nom, conséquence de la conversion URDF→SDF de VRX) — hypothèse que `entitiesFromScopedName` traite ce `/` comme un séparateur d'imbrication supplémentaire en plus de `::`, cassant la résolution du chemin scoped `wamv::wamv/imu_wamv_link::imu_wamv_sensor`. **Changé en nom non-scopé** (`<imuName>imu_wamv_sensor</imuName>`, laissant le plugin utiliser son fallback `EntitiesFromUnscopedName`) — rebuild + retest : **toujours aucun `"Found IMU sensor"` dans le log**, donc cette hypothèse précise n'était pas (ou pas seule) la bonne cause.

**État actuel** : la vraie cause de l'échec de `entitiesFromScopedName`/`EntitiesFromUnscopedName` pour cette entité IMU reste non identifiée avec certitude. L'hypothèse la plus probable, qui était en fait ma toute première intuition avant la découverte de la collision de port (cf. plus haut dans ce journal, section "Diagnostic du blocage IMU") : les modèles inclus **statiquement dans le fichier monde** (comme `iris_with_ardupilot`, `zephyr`, tous les exemples qui marchent dans `ardupilot_gazebo`) sont visibles par toutes les queries ECS dès le premier `PreUpdate()`, alors qu'un modèle **spawné dynamiquement en cours de simulation** (via le service `ros_gz_sim create`, ce que fait tout le pipeline `vrx_gz`) pourrait exposer ses entités capteur à un rythme/ordre différent, pas forcément visible de la même façon aux autres systèmes au moment où `ArduPilotPlugin::PreUpdate()` les cherche — malgré le fait que le capteur IMU lui-même se crée et s'annonce correctement (`ImuSensor.cc`) quelques dizaines de lignes de log plus tôt.

## Prochaines étapes
1. **Confirmer l'hypothèse spawn dynamique vs statique** : le test le plus décisif serait d'inclure le WAM-V (avec `ardupilot_enabled:=true`) **statiquement** dans un fichier monde dédié (comme le fait `iris_with_ardupilot` dans `iris_runway.sdf`) plutôt que via le pipeline de spawn dynamique de `vrx_gz`, et vérifier si `"Found IMU sensor"` apparaît alors dès le début. Si oui, cause confirmée — mais implique de changer l'architecture de lancement (un monde figé plutôt que la config yaml dynamique de `vrx_gz`), donc perte de la flexibilité position/scale/multi-robot actuelle pour ce véhicule.
2. Alternative plus contenue : ajouter des logs de debug temporaires (`gzerr`/`gzdbg`) directement dans `entitiesFromScopedName`/`EntitiesFromUnscopedName` ou juste avant l'appel, pour voir exactement ce qui est recherché vs ce qui existe réellement dans l'ECM à ce moment précis — plus rapide à tester qu'un monde statique complet, et donnerait une réponse définitive sans changer l'architecture.
3. Une fois la connexion FDM établie : calibrer `multiplier`/`cmd_max` des deux canaux de contrôle (`wamv_gazebo.urdf.xacro`, bloc `ardupilot_enabled`) — valeurs actuelles non testées.
4. Lancer MAVROS : `ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14555` (à ajuster selon le port de sortie MAVProxy réellement utilisé).

## ✅ Connexion FDM établie (2026-08-26, résolu)

**Logs de debug temporaires ajoutés** dans `ArduPilotPlugin::PreUpdate()` (juste avant `entitiesFromScopedName`/`EntitiesFromUnscopedName`) : dump du nom recherché, de l'entité modèle, et de **tous les descendants** de l'entité `wamv` avec leur nom et si c'est un capteur IMU. Rebuild + relance.

**Résultat du dump** : l'entité `imu_wamv_sensor` (id 179) est bien présente, bien un descendant du modèle, `isImuSensor=1`, nom exact `imu_wamv_sensor` — **et la ligne finale du debug confirmait déjà `entities found after scoped+unscoped=1`**, c'est-à-dire un succès de la recherche IMU dès ce test !

**Fausse piste dans mon propre diagnostic** : je n'avais pas vu le message natif `"Found IMU sensor with name [...]"` (`gzmsg`, donc niveau `[Msg]`) juste après dans le fichier de log, et j'en avais conclu à tort que la recherche échouait encore. En réalité, `gzmsg`/`gzdbg` passent par un stdout **bufferisé par blocs** dès qu'il est redirigé vers un fichier (`> fichier.log`), contrairement à mes propres logs de debug en `gzerr` (stderr, non bufferisé) qui apparaissaient instantanément — le message de succès existait déjà en mémoire, juste pas encore flush sur disque au moment où je grepais le fichier. **Test direct de la connexion** (au lieu de se fier au log) : heartbeat MAVLink reçu immédiatement (`autopilot: ArduPilot`, `type: rover`) !

**Cause racine confirmée a posteriori** : le fix précédent (`<imuName>imu_wamv_sensor</imuName>`, nom non-scopé au lieu de `wamv::wamv/imu_wamv_link::imu_wamv_sensor`) **était le bon fix** — mon échec à le confirmer plus tôt était une erreur de méthode de diagnostic (buffering de log), pas un échec du fix lui-même. L'hypothèse initiale sur le `/` cassant la résolution du scoped-name reste plausible mais n'a jamais été confirmée avec certitude puisque le contournement (nom non-scopé) a suffi à débloquer la situation.

**Logs de debug retirés**, rebuild propre, **connexion re-testée et confirmée deux fois de suite** (deux heartbeats consécutifs reçus) avec le build final sans instrumentation.

**Chaîne complète maintenant validée** : Gazebo (WAM-V + `ArduPilotPlugin`, monde isolé via `GZ_PARTITION`, port FDM 9012) ↔ ArduPilot Rover SITL (`--model JSON -I1`) ↔ MAVLink (heartbeat confirmé via `pymavlink` sur `tcp:127.0.0.1:5770`).

## Commandes pour reproduire la connexion qui marche
```bash
# Terminal 1 : monde Gazebo (WAM-V + ArduPilotPlugin), isolé de toute autre session gz sim sur la machine
export GZ_PARTITION=vrx_ardupilot_test
source /opt/ros/jazzy/setup.bash && source /home/eustache/vrx_ws/install/setup.bash
cd /home/eustache/vrx_ws
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta headless:=true \
  config_file:=/home/eustache/vrx_ws/src/vrx_ardupilot/config/ardupilot_wamv.yaml sim_mode:=full

# Terminal 2 : SITL Rover, instance 1 (ports décalés de 10 -> FDM 9012/9013, MAVLink 5770)
source /home/eustache/vrx_ws/my-venv/bin/activate
cd /home/eustache/vrx_ws
ardupilot/build/sitl/bin/ardurover --model JSON --speedup 1 --slave 0 --sim-address=127.0.0.1 -I1
# (ou via sim_vehicle.py : Tools/autotest/sim_vehicle.py -v Rover --model JSON -I 1)
```
**Important** : ne PAS utiliser `-f gazebo-rover` seul (sélectionne le mauvais backend legacy) ; ne PAS utiliser l'instance 0 par défaut si une autre session ArduPilot+Gazebo tourne déjà sur la machine (collision de port confirmée) ; toujours définir `GZ_PARTITION` à une valeur unique par simulation Gazebo simultanée sur le même hôte.

## ✅ MAVROS connecté (2026-08-26, résolu)

**Tentative directe** : `ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5770` (connexion directe au port SITL, sans passer par MAVProxy — suffisant pour un seul client). Échec immédiat : `symbol lookup error ... diagnostic_updater7UpdaterC1E...` — `ros-jazzy-mavros` (build du 2026-06-15) attend une ABI de `ros-jazzy-diagnostic-updater` plus récente que celle installée (build du 2025-10-25).

**Cascade de mismatches ABI** : upgrade ciblé de `diagnostic-updater`/`diagnostic-msgs`/`mavros-msgs` → nouvelle erreur, cette fois dans `libdiagnostic_msgs__rosidl_typesupport_fastrtps_cpp.so` (symbole `fastcdr::Cdr::serialize`). Le mismatch remontait donc jusqu'à toute la pile Fast-DDS/rmw (`fastcdr`, `fastrtps`, `rmw-fastrtps-*`, `rosidl-typesupport-fastrtps-*`) — 318 paquets `ros-jazzy-*` avaient une mise à jour disponible au total, signe que **toute l'installation ROS de ce conteneur datait d'avant juin 2026**, pas seulement les paquets liés à mavros.

**Fix** (confirmé avec l'utilisateur avant de lancer, gros changement — 318 paquets) : `sudo apt-get install -y --only-upgrade $(apt list --upgradable | grep '^ros-jazzy' | cut -d/ -f1)`. Aucune régression : toutes les simulations déjà en cours (mon monde WAM-V, la session iris de l'utilisateur) ont continué de tourner sans interruption pendant et après l'upgrade.

**Résultat** : MAVROS démarre, se connecte, et confirme :
```
CON: Got HEARTBEAT, connected. FCU: ArduPilot
FCU: ArduRover V4.8.0-dev (27b35986)
```
`ros2 topic echo /mavros/state --once` :
```
connected: true
armed: false
guided: false
manual_input: true
mode: MANUAL
system_status: 4
```

**Chaîne complète maintenant opérationnelle de bout en bout** : Gazebo (WAM-V + `ArduPilotPlugin`) ↔ ArduPilot Rover SITL ↔ MAVLink ↔ **MAVROS** ↔ topics ROS2 (`/mavros/state`, `/mavros/imu/data`, etc.).

## ✅ Poussée réelle obtenue (2026-08-26, résolu)

**Premier test (armé + `/mavros/rc/override` RC1=1500/RC3=1700, 10s)** : déplacement quasi nul (~0.06m). Deux causes cumulées trouvées et corrigées :

**Cause n°1 : paramètres skid-steer jamais appliqués.** `FRAME_CLASS`/`SERVO1_FUNCTION`/`SERVO3_FUNCTION` valaient encore les défauts Rover classiques (1/26/70 : direction+un seul throttle) au lieu de la config bateau (2/73/74). En cause : l'instance lancée directement (`bin/ardurover ... -I1`, sans passer par `sim_vehicle.py -f gazebo-rover` qui charge `rover-skid.parm`) n'avait jamais reçu ces valeurs. Pire : **`eeprom.bin` était partagé avec la session ArduCopter de l'utilisateur** (même `cwd=/home/eustache/vrx_ws` pour les deux lancements) — donc `ros2 param set` sur ces params persistait dans un fichier utilisé par un autre véhicule. Fix : dossier dédié `vrx_ws/vrx_ardupilot_sitl_instance1/` comme cwd pour cette instance, `eeprom.bin` isolé, params re-réglés et confirmés persistants après redémarrage du firmware (nécessaire pour que `SERVO*_FUNCTION` prenne effet).

**Cause n°2, plus profonde : piloter le joint hélice ne produit aucune poussée.** Même avec la config skid-steer correcte, le déplacement restait négligeable. En cause : `type=VELOCITY` fait tourner le joint de l'hélice directement, mais **Gazebo ne convertit pas la rotation d'un joint en force de propulsion sur la coque** sans un plugin dédié — exactement le rôle de `gz-sim-thruster-system`, que j'avais retiré plus tôt (`thruster_config:=ArduPilot`) pour éviter qu'il ne se batte avec `ArduPilotPlugin` sur le même joint. Résultat : le joint tournait dans le vide, sans aucun couplage au mouvement du bateau.

**Fix** : `ArduPilotPlugin` supporte un mode `<control type="COMMAND">` — au lieu d'actuer un joint, il publie une simple valeur `Double` sur un topic gz-transport de son choix (`<cmd_topic>`). Reconfiguré pour publier directement sur `wamv/thrusters/{left,right}/thrust` — **le topic natif que `gz-sim-thruster-system` écoute déjà** (le même que `thrust_mixer_node` du projet docking utilise normalement). ArduPilot pilote maintenant la physique de poussée déjà calibrée de VRX, sans la réinventer. Formule : `cmd = multiplier*(raw_cmd + offset)` avec `raw_cmd ∈ [0,1]` sur la plage PWM ; `offset=-0.5` centre le neutre ArduPilot (PWM 1500) sur poussée nulle, `multiplier=1200` donne ±600N à pleine déflexion. Retour à `thruster_config:=H` standard (plus besoin du layout de contournement `wamv_aft_thrusters_ardupilot.xacro`, laissé dans le dépôt mais plus utilisé par défaut).

**Résultat confirmé** : même test (armé, RC3=1700 avant, 10s) → déplacement ≈0.78m (vs 0.06m avant), soit un facteur ~13×, confirmant une vraie poussée physique cette fois.

## Prochaines étapes
1. Affiner `multiplier`/`offset` si besoin (±600N est un point de départ raisonnable, pas calibré finement) et régler les gains de cap/vitesse d'ArduPilot Rover (`ATC_STR_RAT_FF`, `CRUISE_THROTTLE`, etc., déjà à leurs valeurs par défaut de `rover-skid.parm`).
2. Tester le comportement en virage (différentiel gauche/droite) et en mode GUIDED (setpoints de vitesse/cap via MAVROS plutôt que RC override).
3. Si usage prolongé prévu : relancer MAVROS via `sim_vehicle.py`/MAVProxy plutôt qu'une connexion TCP directe, pour permettre plusieurs clients simultanés (MAVROS + Mission Planner/QGroundControl + `pymavlink` de debug).
4. Décider si `ardupilot: true` doit rester une option manuelle dans un yaml de spawn dédié (`vrx_ardupilot`) ou s'intégrer davantage avec le reste du projet (docking `vrx_apf_dock` reste indépendant, non affecté par ce travail).
5. **Toujours lancer les instances SITL depuis un dossier de travail dédié** (jamais `vrx_ws/` directement) pour éviter de partager `eeprom.bin` avec d'autres sessions ArduPilot (Copter, autres tests Rover, etc.).

## État laissé dans le conteneur (fin de session)
- Mon monde WAM-V (`sydney_regatta.sdf`, `GZ_PARTITION=vrx_ardupilot_test`, port FDM 9012) tourne toujours en arrière-plan, connexion FDM validée avec un SITL Rover instance 1 déjà testé (heartbeat confirmé).
- La session iris/ArduCopter de l'utilisateur (`iris_runway.sdf`, port 9002) tourne aussi, intacte, non affectée par mes tests.
- Tous mes process de test SITL (`ardurover` instance 1) ont été nettoyés après la dernière vérification ; à relancer avec la commande ci-dessus pour continuer.
- `strace` et `tcpdump` installés dans le conteneur (utiles pour la suite si besoin).
- Code de debug temporaire dans `ArduPilotPlugin.cc` retiré ; le fix conservé est uniquement `<imuName>imu_wamv_sensor</imuName>` (nom non-scopé) dans `wamv_gazebo.urdf.xacro`.
