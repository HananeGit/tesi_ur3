#!/usr/bin/env bash
# E3: misura relazione /tf vs PUB_RATE (5/10/20/60 Hz)
set -euo pipefail

RATES=("5" "10" "20" "60")
BASE="$HOME/tesi_ur3"
CSV="$BASE/results/E3_joint_states.csv"
LOGDIR="/tmp"
DUR_PUB=14    # sec: pubblicazione joint_states
DUR_MEAS=10   # sec: misura tf/cpu

# --- source "safe" (disattiva -u temporaneamente) ---
source_safe () {
  local had_nounset=0
  case $- in *u*) had_nounset=1; set +u;; esac
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
  # shellcheck disable=SC1090
  source "$1"
  if [ "$had_nounset" -eq 1 ]; then set -u; fi
}

# --- Ambiente ROS ---
source_safe /opt/ros/humble/setup.bash
if [ -f "$HOME/ur_ros2_ws/install/setup.bash" ]; then
  source_safe "$HOME/ur_ros2_ws/install/setup.bash"
fi

# --- Requisiti ---
mkdir -p "$BASE/results" "$BASE/docs/plots"
command -v pidstat >/dev/null || sudo apt -y install sysstat >/dev/null 2>&1 || true

# --- Header CSV ---
[ -s "$CSV" ] || echo "pub_rate_hz,tf_hz,cpu_percent,fluidita_note" > "$CSV"

# --- Stop pulito ---
stop_all() { "$BASE/scripts/stop_ur3.sh" 2>/dev/null || true; }

# --- Attesa RViz ---
wait_for_rviz() {
  for _ in {1..20}; do
    if pgrep -n rviz2 >/dev/null; then return 0; fi
    sleep 0.5
  done
  return 1
}

# --- Misura singola frequenza ---
measure_rate() {
  local RATE="$1"
  echo "=== E3 @ ${RATE} Hz ==="
  stop_all

  export QT_QPA_PLATFORM=xcb
  unset LIBGL_ALWAYS_SOFTWARE || true

  # Avvia RViz + robot_state_publisher SENZA GUI
  USE_GUI=0 "$BASE/scripts/run_ur3_rviz.sh" >"$LOGDIR/e3_${RATE}.log" 2>&1 &
  sleep 2
  if ! wait_for_rviz; then
    echo "${RATE},,," >> "$CSV"
    echo "  [!] RViz non avviato"
    stop_all; sleep 1; return
  fi

  # Pubblica joint_states alla frequenza desiderata
  timeout ${DUR_PUB}s ros2 topic pub -r "$RATE" /joint_states sensor_msgs/msg/JointState "{
    name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
    position: [0.0, -1.2, 1.2, -1.2, 1.57, 0.0]
  }" >/dev/null &
  sleep 2

  # Misura /tf (scrivo log su file e poi estraggo l'average)
  TFLOG="$LOGDIR/tf_${RATE}.txt"; : > "$TFLOG"
  ros2 topic hz -w 100 /tf >"$TFLOG" 2>&1 &
  HZ_PID=$!

  # CPU media RViz per DUR_MEAS secondi
  RVIZ_PID=$(pgrep -n rviz2 || true)
  if [ -n "${RVIZ_PID:-}" ]; then
    CPU=$(pidstat -p "$RVIZ_PID" 1 "$DUR_MEAS" | awk 'END{print $8}')
  else
    CPU=""
  fi

  # attendo, poi fermo misura hz
  sleep "$DUR_MEAS"
  kill "$HZ_PID" 2>/dev/null || true
  sleep 1

  TFHZ=$(awk '/average rate:/ {v=$3} END{print v}' "$TFLOG")

  echo "${RATE},${TFHZ:-},${CPU:-}," >> "$CSV"
  echo "  → rate=${RATE}Hz  tf_hz=${TFHZ:-NA}  cpu=${CPU:-NA}%"

  stop_all
  sleep 1
}

# --- Loop ---
for r in "${RATES[@]}"; do
  measure_rate "$r"
done

# Grafici (se presente lo script)
python3 "$BASE/scripts/analyze_results.py" || true
echo "Fatto. CSV: $CSV"

