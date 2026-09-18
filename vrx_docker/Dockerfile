# Image de developpement pour vrx_ws -- etend l'image officielle publiee par
# VRX (construite a partir de src/vrx/docker/Dockerfile.base + Dockerfile.devel,
# non modifies ici -- fichiers partages, voir PROPOSITION_SIMULATEUR_COMPLET.md
# §8) avec les outils installes manuellement au fil des sessions precedentes
# directement dans ~/vrx_ws (donc perdus a chaque `git clone` frais, puisque
# ce ne sont pas des fichiers source versionnes -- binaires compiles/
# applications tierces, voir .gitignore) : mono (pour MissionPlanner.exe),
# le binaire mavlink-router, et les paquets Python d'ArduPilot (pymavlink,
# MAVProxy, empy...).
#
# Construire :
#   docker build -t vrx-devel-custom -f vrx_docker/Dockerfile vrx_docker/
#
# Puis lancer exactement comme avant (aucun changement a cette commande) :
#   docker run -it --rm --privileged \
#     --volume /dev/input:/dev/input --volume /run/udev:/run/udev:ro \
#     --volume /home/eustache:/home/eustache --env DISPLAY=$DISPLAY \
#     --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --gpus all \
#     --user 1000:1000 --workdir /home/eustache \
#     vrx-devel-custom /bin/bash

FROM ghcr.io/osrf/vrx-devel:latest

USER root

# GStreamer (headers dev) -- requis par le CMakeLists.txt de
# src/ardupilot_gazebo (pkg_check_modules REQUIRED gstreamer-1.0
# gstreamer-app-1.0, plugin camera/streaming video), absent de l'image de
# base VRX -- sans ca, `colcon build` echoue directement sur ce paquet et
# fait avorter toute la chaine de dependants (vrx_ros, wamv_description,
# ardupilot_msgs...).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-gl \
        gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

# MAVROS -- pont MAVLink/ROS2 (ros2 launch mavros apm.launch...) -- et joy
# (joy_node, telecommande physique/synthetique, ros2 launch vrx_docking_sim
# joystick_rc.launch.py). L'image de base VRX installe seulement
# ros-jazzy-mavros-msgs et ros-jazzy-joy-linux (definitions de messages/
# variante alternative), pas les paquets eux-memes -- decouvert en direct
# via "Package 'mavros'/'joy' not found" sur un clone frais. Recherche
# systematique du reste des dependances manquantes via `rosdep install
# --from-paths src --ignore-src` (voir patches/apply_patches.sh pour le
# contexte des sous-modules vendored) : plus rien d'autre ne manquait.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-mavros \
        ros-${ROS_DISTRO}-mavros-extras \
        ros-${ROS_DISTRO}-joy \
    # mavros_node plante au demarrage avec "undefined symbol:
    # ...UpdaterC1..." sans cet upgrade explicite : la version de
    # ros-jazzy-diagnostic-updater deja installee par l'image de base
    # (4.2.6) est ABI-incompatible avec celle contre laquelle mavros a ete
    # compile (le constructeur Updater a change de signature) -- `apt
    # install ros-jazzy-mavros` seul ne force pas la mise a jour d'une
    # dependance deja "satisfaite" au sens du .deb (aucune contrainte de
    # version stricte declaree), il faut l'upgrade explicitement.
    && apt-get install -y --only-upgrade ros-${ROS_DISTRO}-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# mono (execute MissionPlanner.exe, application .NET Framework via mono) --
# depot officiel mono-project.com, "stable-focal" reste la ligne recommandee
# meme sur les Ubuntu plus recents (pas de republication par nom de code a
# chaque version, verifie sur la doc officielle mono-project.com).
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates gnupg \
    && gpg --homedir /tmp --no-default-keyring \
         --keyring gnupg-ring:/usr/share/keyrings/mono-official-archive-keyring.gpg \
         --keyserver hkp://keyserver.ubuntu.com:80 \
         --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF \
    && chmod +r /usr/share/keyrings/mono-official-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/mono-official-archive-keyring.gpg] https://download.mono-project.com/repo/ubuntu stable-focal main" \
         > /etc/apt/sources.list.d/mono-official-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends mono-complete \
    && rm -rf /var/lib/apt/lists/*

# Micro-XRCE-DDS-Gen -- generateur de code requis par la configuration waf
# du wrapper ROS 2 d'ArduPilot (ardupilot/Tools/ros2/ardupilot_sitl, colcon
# build echoue sinon avec "Could not find the program ['microxrceddsgen']").
# Procedure officielle : https://ardupilot.org/dev/docs/ros2-install.html
# openjdk-17 (pas default-jre=21 sur Ubuntu 24.04) : le Gradle 7.6 embarque
# dans ce depot ne supporte pas les class files Java 21 ("Unsupported class
# file major version 65"), constate en direct.
RUN apt-get update && apt-get install -y --no-install-recommends openjdk-17-jre-headless git \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --recurse-submodules \
         https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
         /opt/Micro-XRCE-DDS-Gen \
    && cd /opt/Micro-XRCE-DDS-Gen && ./gradlew assemble
ENV PATH="/opt/Micro-XRCE-DDS-Gen/scripts:${PATH}"

# mavlink-router -- relais MAVLink multi-client (MAVROS + Mission Planner en
# parallele, voir PROPOSITION_SIMULATEUR_COMPLET.md §9). Binaire officiel,
# release v4 de mavlink-router/mavlink-router sur GitHub.
RUN curl -L -s -o /usr/local/bin/mavlink-routerd \
      https://github.com/mavlink-router/mavlink-router/releases/download/v4/mavlink-routerd-glibc-x86_64 \
    && chmod +x /usr/local/bin/mavlink-routerd

# Paquets Python d'ArduPilot (build waf + MAVProxy + scripts de diagnostic
# pymavlink) dans un venv dedie ajoute au PATH -- evite les soucis
# "externally-managed-environment" (PEP 668) d'un `pip install` systeme sur
# Ubuntu 24.04, et rend mavproxy.py/pymavlink disponibles dans tous les
# shells du conteneur sans `source .../bin/activate`. empy epingle a 3.3.4 :
# les versions plus recentes cassent le build waf d'ArduPilot (piege connu).
RUN python3 -m venv /opt/ardupilot-venv --system-site-packages \
    && /opt/ardupilot-venv/bin/pip install --no-cache-dir \
         empy==3.3.4 pymavlink MAVProxy pexpect future lxml pyserial pyyaml numpy pillow
ENV PATH="/opt/ardupilot-venv/bin:${PATH}"

# Remplace l'utilisateur "ubuntu" (uid 1000) de l'image de base par
# "eustache" -- docker run utilise --user 1000:1000 (uid brut, sans nom) +
# --volume /home/eustache:/home/eustache, donc le nom resolu pour ce uid
# doit correspondre a l'utilisateur hote reel, sinon le prompt affiche
# "ubuntu@..." au lieu de "eustache@...". Reproduit ce que faisait
# `rocker --user` auparavant sur l'ancienne image (vu dans son historique
# de build : userdel de l'utilisateur existant a cet uid, puis useradd).
RUN userdel -r ubuntu 2>/dev/null; \
    groupadd -g 1000 eustache 2>/dev/null; \
    useradd --no-log-init --uid 1000 --gid 1000 \
        -G sudo,video,plugdev,dialout,input,render \
        -s /bin/bash -d /home/eustache -m eustache \
    && echo "eustache ALL=NOPASSWD: ALL" > /etc/sudoers.d/eustache

# GID 110 = groupe "render" sur CETTE machine hote (verifie avec
# `getent group 110`) -- --gpus all (nvidia-container-runtime) l'injecte
# automatiquement comme groupe secondaire du conteneur pour l'acces au
# device GPU, mais aucune entree /etc/group du conteneur n'a ce GID (le
# groupe "render" interne a l'image a un GID different) : `groups`/le
# prompt affichaient "cannot find name for group ID 110". Un nom
# quelconque a ce GID suffit a lever l'avertissement -- specifique a cette
# machine, a ajuster (`getent group $(id -g video)` cote hote) si
# reconstruit ailleurs.
RUN groupadd -g 110 hostrender 2>/dev/null || true

USER eustache
