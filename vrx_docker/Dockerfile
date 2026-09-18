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

USER ubuntu
