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
