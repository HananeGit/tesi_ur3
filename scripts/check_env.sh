#!/usr/bin/env bash
# Diagnostica rapida UR3 + ROS 2 Humble
set -e

# -- source "safe" (evita errori AMENT_TRACE_SETUP_FILES)
source_safe () {
  local had_nounset=0; case $- in *u*) had_nounset=1; set +u;; esac
  export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
  source "$1"
  [ $had_nounset -eq 1 ] && set -u || true
}

echo "==[ 1/7 ] Ambiente ROS =="
source_safe /opt/ros/humble/setup.bash
[ -f "$HOME/ur_ros2_ws/install/setup.bash" ] && source_safe "$HOME/ur_ros2_ws/install/setup.bash" || true
echo "ROS_DISTRO: ${ROS_DISTRO:-<vuoto>}"
command -v ros2 >/dev/null || { echo "❌ ros2 non trovato"; exit 1; }
echo "✅ ros2 OK"

echo "==[ 2/7 ] Pacchetti chiave =="
for p in ur_description robot_state_publisher rviz2; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "✅ $p"; else echo "❌ manca $p"; fi
done
command -v xacro >/dev/null && echo "✅ xacro" || echo "❌ manca xacro"

echo "==[ 3/7 ] Generazione URDF da xacro =="
XACRO="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
URDF="/tmp/ur3_check.urdf"
if [ -f "$XACRO" ]; then
  if xacro "$XACRO" ur_type:=ur3 name:=ur3 prefix:="" > "$URDF"; then
    echo "✅ URDF generato: $URDF"
  else
    echo "❌ xacro fallito (argomenti mancanti?)"; exit 1
  fi
else
  echo "❌ Non trovo $XACRO"; exit 1
fi

echo "==[ 4/7 ] Nodo robot_state_publisher =="
if ros2 node list | grep -q /robot_state_publisher; then
  echo "✅ robot_state_publisher in esecuzione"
else
  echo "ℹ️ Avvio temporaneo robot_state_publisher per il test…"
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat "$URDF")" \
    --log-level error >/tmp/rsp_check.log 2>&1 &
  RSP_PID=$!
  sleep 1
fi
ros2 param get /robot_state_publisher robot_description >/dev/null 2>&1 && \
  echo "✅ robot_description presente" || echo "❌ robot_description assente"

echo "==[ 5/7 ] joint_states =="
if ros2 topic list | grep -q '^/joint_states$'; then
  if ros2 topic echo /joint_states --once >/tmp/js_once.txt 2>&1; then
    echo "✅ /joint_states ricevuto almeno 1 msg"
  else
    echo "⚠️ /joint_states esiste ma non arrivano messaggi (apri GUI slider o publ. da terminale)"
  fi
else
  echo "❌ topic /joint_states assente (serve GUI slider o publisher)"
fi

echo "==[ 6/7 ] TF =="
if ros2 topic list | grep -q '^/tf$'; then
  echo "  Frequenza TF (2s):"
  ros2 topic hz -w 2 /tf || true
else
  echo "❌ topic /tf assente (robot_state_publisher non attivo o senza joint_states)"
fi

echo "==[ 7/7 ] RViz pronto? =="
echo "Suggerimento: rviz2 --ros-args -p robot_description:=\"\$(cat $URDF)\""
echo "Fixed Frame: base_link | Display: RobotModel (File: $URDF)"

# cleanup eventuale
[ -n "${RSP_PID:-}" ] && kill "$RSP_PID" >/dev/null 2>&1 || true
echo "== Diagnostica completata =="
