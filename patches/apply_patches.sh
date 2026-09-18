#!/bin/bash
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
#
# Applique les correctifs locaux aux sous-modules vendored (src/vrx,
# src/ardupilot_gazebo) qui avaient ete edites directement sur disque dans
# des sessions precedentes, sans jamais etre commites dans l'historique du
# sous-module ni captures par le lien git (gitlink) du sur-projet -- donc
# perdus a chaque `git clone --recurse-submodules` frais. Voir §8 de
# PROPOSITION_SIMULATEUR_COMPLET.md pour le detail de ce que fait chaque
# patch et pourquoi. Idempotent : peut etre relance sans effet si les
# patches sont deja appliques.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

apply_if_needed() {
    # Chemin absolu obligatoire : `git -C <submodule_dir> apply` resout
    # aussi le chemin du fichier patch relativement a <submodule_dir>, pas
    # au repertoire courant -- un chemin relatif ici echoue silencieusement
    # en "fichier introuvable", constate en direct.
    local patch_file="$(pwd)/$1" submodule_dir="$2"
    if git -C "$submodule_dir" apply --check "$patch_file" 2>/dev/null; then
        echo "Application de $patch_file sur $submodule_dir..."
        git -C "$submodule_dir" apply "$patch_file"
    elif git -C "$submodule_dir" apply --reverse --check "$patch_file" 2>/dev/null; then
        echo "$patch_file deja applique sur $submodule_dir, rien a faire."
    else
        echo "ERREUR : $patch_file ne s'applique pas proprement sur $submodule_dir" \
             "(ni dans un sens ni dans l'autre) -- le sous-module a peut-etre change" \
             "de commit depuis la creation du patch. Verifier a la main." >&2
        exit 1
    fi
}

apply_if_needed patches/vrx.patch src/vrx
apply_if_needed patches/ardupilot_gazebo.patch src/ardupilot_gazebo

echo "OK -- penser a 'colcon build --merge-install --symlink-install" \
     "--packages-select ardupilot_gazebo' pour recompiler le plugin C++ modifie."
