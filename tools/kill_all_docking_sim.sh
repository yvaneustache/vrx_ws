#!/bin/bash
# Arrête tout ce qui tourne pour le simulateur de docking : Gazebo, ArduPilot
# SITL, MAVROS, mavlink-router, tous les nœuds vrx_docking_sim, joystick,
# RViz2. À exécuter EN FICHIER (bash kill_all_docking_sim.sh), jamais en
# ligne inline (bash -c "..."), sinon le motif de recherche apparaît dans la
# commande qui lance le script lui-même et peut se tuer en plein milieu
# (bug rencontré en direct plus tôt dans le projet).

PATTERNS=(
  "gz sim"
  "ardurover"
  "mavros_node"
  "ros2 launch mavros"
  "mavlink-routerd"
  "install/lib/vrx_docking_sim/"
  "install/lib/ardupilot_sitl/"
  "joy_node"
  "rviz2"
  "ros_gz_bridge"
  "robot_state_publisher"
  "static_transform_publisher"
)

echo "=== Recherche des processus à arrêter ==="
ALL_PIDS=()
for pattern in "${PATTERNS[@]}"; do
  pids=$(ps aux | grep -F -- "$pattern" | grep -v grep | awk '{print $2}')
  if [ -n "$pids" ]; then
    echo "--- motif: $pattern ---"
    ps aux | grep -F -- "$pattern" | grep -v grep
    ALL_PIDS+=($pids)
  fi
done

if [ ${#ALL_PIDS[@]} -eq 0 ]; then
  echo "Rien à arrêter."
  exit 0
fi

echo ""
echo "=== Arrêt de ${#ALL_PIDS[@]} process ==="
for pid in "${ALL_PIDS[@]}"; do
  kill -9 "$pid" 2>/dev/null
done

sleep 1

echo ""
echo "=== Vérification ==="
remaining=0
for pattern in "${PATTERNS[@]}"; do
  pids=$(ps aux | grep -F -- "$pattern" | grep -v grep)
  if [ -n "$pids" ]; then
    echo "ENCORE VIVANT ($pattern):"
    echo "$pids"
    remaining=1
  fi
done

if [ "$remaining" -eq 0 ]; then
  echo "Tout est arrêté."
else
  echo "Certains process résistent (relance-le, ou vérifie s'ils redémarrent seuls)."
fi
