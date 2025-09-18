#!/usr/bin/env bash
set -e

BASE="$HOME/tesi_ur3"
CSV="$BASE/results/E1_tempo_avvio.csv"
mkdir -p "$BASE/results"

# ---- source "safe" ----
source /opt/ros/humble/setup.bash
test -f "$HOME/ur_ros2_ws/install/setup.bash" && source "$HOME/ur_ros2_ws/install/setup.bash"

# ---- RViz config minimale (base_link + RobotModel) ----
RVIZCFG="/tmp/e1_base.rviz"
cat > "$RVIZCFG" <<'RV'
Panels:
- Class: rviz_common/Displays
  Name: Displays
Visualization Manager:
  Display Groups:
    - Class: ""
  Displays:
  - Class: rviz_default_plugins/Grid
    Name: Grid
  - Class: rviz_default_plugins/RobotModel
    Name: RobotModel
  Global Options:
    Fixed Frame: base_link
RV

# ---- funzioni utili ----
stop_all() {
  pkill -f "^rviz2(\s|$)" 2>/dev/null || true
  pkill -f robot_state_publisher 2>/dev/null || true
  pkill -f "ur_control.launch.py" 2>/dev/null || true
  pkill -f "moveit.launch.py" 2>/dev/null || true
}

gen_urdf() {
  XACRO="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
  URDF="/tmp/ur3_e1.urdf"
  xacro "$XACRO" ur_type:=ur3 name:=ur3 prefix:="" > "$URDF"
}

wait_tf_static() {
  # attende un messaggio su /tf_static (timeout 20s)
  timeout 20s bash -lc 'ros2 topic echo /tf_static --once >/dev/null'
}

measure_once() {
  local scenario="$1" note="${2:-}"
  gen_urdf
  stop_all

  # avvia RSP (pubblica robot_description + TF)
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat "$URDF")" >/tmp/e1_rsp.log 2>&1 &

  # scenario specifico
  if [ "$scenario" = "rviz_solo" ]; then
    rviz2 -d "$RVIZCFG" >/tmp/e1_rviz.log 2>&1 &
  elif [ "$scenario" = "moveit_fake" ]; then
    # serve ur_robot_driver + moveit_config
    if ! ros2 pkg prefix ur_moveit_config >/dev/null 2>&1; then
      echo "[E1] moveit_fake non disponibile: ur_moveit_config mancante. Salto."
      return 0
    fi
    ros2 launch ur_robot_driver ur_control.launch.py \
      ur_type:=ur3 use_fake_hardware:=true robot_ip:=127.0.0.1 launch_rviz:=false \
      >/tmp/e1_driver.log 2>&1 &
    sleep 2
    ros2 launch ur_moveit_config moveit.launch.py \
      ur_type:=ur3 use_fake_hardware:=true \
      >/tmp/e1_moveit.log 2>&1 &
  else
    echo "Scenario non valido: $scenario"; return 1
  fi

  # misura time-to-first-tf_static
  t0=$(date +%s.%3N)
  wait_tf_static || true
  t1=$(date +%s.%3N)
  dt=$(python3 - <<PY
t0=float("$t0"); t1=float("$t1")
print(f"{t1-t0:.3f}")
PY
)
  [ -s "$CSV" ] || echo "scenario,run,tempo_s,note" > "$CSV"
  printf "%s,%d,%.3f,%s\n" "$scenario" "$(date +%s)" "$dt" "$note" >> "$CSV"
  echo "[E1] $scenario -> ${dt}s"

  stop_all
}

# ---- se chiamato senza args, fa 1 misura per ogni scenario ----
if [ $# -eq 0 ]; then
  measure_once rviz_solo
  measure_once moveit_fake
else
  measure_once "$@"
fi
echo "OK -> $CSV"
