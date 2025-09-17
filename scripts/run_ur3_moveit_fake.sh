#!/usr/bin/env bash
# Run UR3 + MoveIt 2 (fake hardware) con RViz
set -e

UR_TYPE="${1:-ur3}"               # ur3, ur3e, ...
ROBOT_IP="${ROBOT_IP:-127.0.0.1}" # IP fittizio in fake hw
SOFT_RENDER="${SOFT_RENDER:-0}"   # 1 = forza rendering software

# --- funzione per fare source in modo "safe" anche se nounset è attivo ---
source_safe () {
  local had_nounset=0
  case $- in *u*) had_nounset=1; set +u;; esac
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  # shellcheck disable=SC1090
  source "$1"
  if [ "$had_nounset" -eq 1 ]; then set -u; fi
}

# Ambiente
source_safe /opt/ros/humble/setup.bash
if [ -f "$HOME/ur_ros2_ws/install/setup.bash" ]; then
  source_safe "$HOME/ur_ros2_ws/install/setup.bash"
fi

# Verifica pacchetti minimi
ros2 pkg prefix ur_robot_driver     >/dev/null
ros2 pkg prefix ur_moveit_config    >/dev/null

# Chiudi eventuali istanze precedenti
pkill -f "ur_control.launch.py"     || true
pkill -f "ur_moveit.launch.py"      || true
pkill -f "rviz2 -d"                 || true

# Impostazioni grafica (Wayland/GNOME più stabile con xcb)
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
if [ "$SOFT_RENDER" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE || true
fi

# Avvia il driver (fake hardware) in background
echo "[*] Avvio ur_robot_driver (fake hardware)..."
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} use_fake_hardware:=true \
  launch_rviz:=false > /tmp/ur_driver_fake.log 2>&1 &
DRIVER_PID=$!
sleep 3

# Avvia MoveIt 2 (apre RViz con MotionPlanning)
echo "[*] Avvio MoveIt 2 (RViz si aprirà tra poco)..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=${UR_TYPE} launch_rviz:=true

# Quando chiudi RViz/MoveIt, spegni il driver
kill "$DRIVER_PID" 2>/dev/null || true
echo "Chiuso. Log driver: /tmp/ur_driver_fake.log"

