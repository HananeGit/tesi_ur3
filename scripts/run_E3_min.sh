#!/usr/bin/env bash
# E3 minimale e robusto: /tf vs PUB_RATE senza dipendere da altri script
set -euo pipefail

RATES=("5" "10" "20" "60")
BASE="$HOME/tesi_ur3"
CSV="$BASE/results/E3_joint_states.csv"
PLOT="$BASE/docs/plots"
LOG="/tmp/e3_min.log"
URDF="/tmp/ur3_e3.urdf"
RVIZCFG="/tmp/ur3_e3_min.rviz"

# --- source "safe" (evita errori unbound var durante i source) ---
source_safe () {
  local had_nounset=0
  case $- in *u*) had_nounset=1; set +u;; esac
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
  # shellcheck disable=SC1090
  source "$1"
  if [ "$had_nounset" -eq 1 ]; then set -u; fi
}

source_safe /opt/ros/humble/setup.bash
if [ -f "$HOME/ur_ros2_ws/install/setup.bash" ]; then
  source_safe "$HOME/ur_ros2_ws/install/setup.bash"
fi

mkdir -p "$BASE/results" "$PLOT"
command -v pidstat >/dev/null || sudo apt -y install sysstat >/dev/null 2>&1 || true

# RViz più stabile su GNOME
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
unset LIBGL_ALWAYS_SOFTWARE || true

# 1) URDF dall’xacro ufficiale
XACRO="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
xacro "$XACRO" ur_type:=ur3 name:=ur3 prefix:="" > "$URDF"

# 2) RViz config minimale (Grid + RobotModel, Fixed Frame = base_link)
cat > "$RVIZCFG" <<'RV'
Panels:
- Class: rviz_common/Displays
  Name: Displays
Visualization Manager:
  Class: ""
  Displays:
  - Class: rviz_default_plugins/Grid
    Name: Grid
  - Class: rviz_default_plugins/RobotModel
    Name: RobotModel
  Global Options:
    Fixed Frame: base_link
  Tools:
  - Class: rviz_default_plugins/Interact
  - Class: rviz_default_plugins/MoveCamera
RV

# 3) Header CSV
[ -s "$CSV" ] || echo "pub_rate_hz,tf_hz,cpu_percent,fluidita_note" > "$CSV"

# 4) killer pulito
stop_all() {
  pkill -f "robot_state_publisher" || true
  pkill -f "^rviz2(\s|$)" || true
  pkill -f "ros2 topic pub .* /joint_states" || true
}

measure_one() {
  local RATE="$1"
  echo "=== E3 @ ${RATE} Hz ===" | tee -a "$LOG"
  stop_all

  # 5) avvia robot_state_publisher (pubblica robot_description)
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat "$URDF")" \
    > /tmp/rsp_e3.log 2>&1 &

  # 6) avvia RViz con la config minimale
  rviz2 -d "$RVIZCFG" > /tmp/rviz_e3.log 2>&1 &
  # attendi RViz
  for _ in {1..20}; do pgrep -n rviz2 >/dev/null && break; sleep 0.3; done
  sleep 1

  # 7) pubblica joint_states alla frequenza richiesta (12s)
  timeout 12s ros2 topic pub -r "$RATE" /joint_states sensor_msgs/msg/JointState \
"header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
position: [0.0, -1.2, 1.2, -1.2, 1.57, 0.0]
velocity: []
effort: []" >/dev/null &
  sleep 2

  # 8) misura /tf (10s) e CPU RViz (10s)
  TFHZ=$(timeout 10s bash -lc 'ros2 topic hz -w 100 /tf 2>/dev/null' | awk '/average rate:/ {v=$3} END{print v}')
  PID=$(pgrep -n rviz2 || true)
  if [ -n "${PID:-}" ]; then
    CPU=$(pidstat -p "$PID" 1 10 | awk 'END{print $8}')
  else
    CPU=""
  fi

  echo "${RATE},${TFHZ:-},${CPU:-}," >> "$CSV"
  echo "→ rate=${RATE}Hz  tf_hz=${TFHZ:-NA}  cpu=${CPU:-NA}%" | tee -a "$LOG"

  stop_all
  sleep 1
}

for r in "${RATES[@]}"; do
  measure_one "$r"
done

# 9) grafici (se presente analyzer)
python3 "$BASE/scripts/analyze_results.py" || true
echo "Fatto. CSV: $CSV"
