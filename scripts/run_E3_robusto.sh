#!/usr/bin/env bash
# E3 robusto: URDF -> robot_state_publisher -> RViz -> pub /joint_states -> misura /tf & CPU
set -e

RATES=("5" "10" "20" "60")
BASE="$HOME/tesi_ur3"
CSV="$BASE/results/E3_joint_states.csv"
URDF="/tmp/ur3_e3.urdf"
RVIZCFG="/tmp/ur3_e3_min.rviz"
LOGDIR="/tmp"
DUR_PUB=14     # s: durata pubblicazione joint_states
DUR_MEAS=10    # s: durata misura tf/cpu

# ---------- helpers ----------
source_safe () {
  # fa "source" anche se alcune variabili non sono definite
  case $- in *u*) set +u; RESTORE_U=1;; *) RESTORE_U=0;; esac
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
  # shellcheck disable=SC1090
  source "$1"
  [ "$RESTORE_U" = 1 ] && set -u || true
}
stop_all() {
  pkill -f "ros2 topic pub .* /joint_states" >/dev/null 2>&1 || true
  pkill -f "^rviz2(\s|$)" >/dev/null 2>&1 || true
  pkill -f "robot_state_publisher" >/dev/null 2>&1 || true
}
wait_node() { # $1 = node name contains
  for _ in {1..30}; do ros2 node list | grep -q "$1" && return 0; sleep 0.3; done; return 1;
}
have_param() { # $1 node, $2 param
  ros2 param get "$1" "$2" >/dev/null 2>&1
}
last_avg_from_hz_log() { awk '/average rate:/ {v=$3} END{print v}' "$1"; }

# ---------- ambiente ----------
set -u
source_safe /opt/ros/humble/setup.bash
[ -f "$HOME/ur_ros2_ws/install/setup.bash" ] && source_safe "$HOME/ur_ros2_ws/install/setup.bash"

mkdir -p "$BASE/results" "$BASE/docs/plots"
command -v pidstat >/dev/null || sudo apt -y install sysstat >/dev/null 2>&1 || true

# Render stabile su GNOME/Wayland
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
unset LIBGL_ALWAYS_SOFTWARE || true

# ---------- prepara RViz minimale ----------
cat > "$RVIZCFG" <<'RV'
Panels:
- Class: rviz_common/Displays
  Name: Displays
Visualization Manager:
  Displays:
  - Class: rviz_default_plugins/Grid
    Name: Grid
  - Class: rviz_default_plugins/RobotModel
    Name: RobotModel
  Global Options:
    Fixed Frame: base_link
RV

# ---------- prepara URDF ----------
XACRO="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
if [ ! -f "$XACRO" ]; then
  echo "[ERRORE] Manca ur_description / xacro: $XACRO"; exit 1
fi
xacro "$XACRO" ur_type:=ur3 name:=ur3 prefix:="" > "$URDF"

# ---------- header CSV ----------
[ -s "$CSV" ] || echo "pub_rate_hz,tf_hz,cpu_percent,fluidita_note" > "$CSV"

# ---------- funzione di misura ----------
measure_rate() {
  local RATE="$1"
  echo "=== E3 @ ${RATE} Hz ==="
  stop_all

  # 1) avvia robot_state_publisher con robot_description dall’URDF
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat "$URDF")" \
    > "$LOGDIR/rsp_e3_${RATE}.log" 2>&1 &
  sleep 0.5
  if ! wait_node "robot_state_publisher"; then
    echo "  [!] robot_state_publisher non avviato"; echo "${RATE},,," >> "$CSV"; return
  fi
  # verifica che robot_description sia caricato
  if ! have_param "/robot_state_publisher" "robot_description"; then
    echo "  [!] robot_description non presente"; echo "${RATE},,," >> "$CSV"; return
  fi

  # 2) avvia RViz
  rviz2 -d "$RVIZCFG" > "$LOGDIR/rviz_e3_${RATE}.log" 2>&1 &
  # attendi partenza rviz
  for _ in {1..20}; do pgrep -n rviz2 >/dev/null && break; sleep 0.3; done
  sleep 1

  # 3) pubblica /joint_states a RATE Hz (in YAML completo)
  timeout ${DUR_PUB}s ros2 topic pub -r "$RATE" /joint_states sensor_msgs/msg/JointState \
"header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
position: [0.0, -1.2, 1.2, -1.2, 1.57, 0.0]
velocity: []
effort: []" >/dev/null &
  sleep 2

  # 4) misura /tf e CPU
  TFLOG="$LOGDIR/tf_${RATE}.txt"; : > "$TFLOG"
  ros2 topic hz -w 100 /tf > "$TFLOG" 2>&1 &
  HZ_PID=$!

  RVIZ_PID=$(pgrep -n rviz2 || true)
  if [ -n "${RVIZ_PID:-}" ]; then
    CPU=$(pidstat -p "$RVIZ_PID" 1 "$DUR_MEAS" | awk 'END{print $8}')
  else
    CPU=""
  fi

  sleep "$DUR_MEAS"
  kill "$HZ_PID" >/dev/null 2>&1 || true
  sleep 0.5
  TFHZ=$(last_avg_from_hz_log "$TFLOG")

  echo "${RATE},${TFHZ:-},${CPU:-}," >> "$CSV"
  echo "  → tf_hz=${TFHZ:-NA}  cpu=${CPU:-NA}% (rate=${RATE} Hz)"

  stop_all
  sleep 0.5
}

for r in "${RATES[@]}"; do
  measure_rate "$r"
done

# grafici se disponibile
python3 "$BASE/scripts/analyze_results.py" >/dev/null 2>&1 || true
echo "Fatto. CSV: $CSV"
