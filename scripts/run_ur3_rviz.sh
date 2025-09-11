
#!/usr/bin/env bash
# UR3 in RViz2 (ROS 2 Humble) – visualizzazione "one-click"
set -e

UR_TYPE="${1:-ur3}"             # ur3, ur3e, ur5e...
USE_GUI="${USE_GUI:-1}"         # 1 = joint_state_publisher_gui, 0 = publisher /joint_states
PUB_RATE="${PUB_RATE:-10}"      # Hz se USE_GUI=0
SOFT_RENDER="${SOFT_RENDER:-0}" # 1 = forza rendering software

# Ambiente
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ur_ros2_ws/install/setup.bash" ]; then
  source "$HOME/ur_ros2_ws/install/setup.bash"
fi

# Verifiche
ros2 pkg prefix ur_description >/dev/null
command -v xacro >/dev/null  || { echo "manca xacro (sudo apt install ros-humble-xacro)"; exit 1; }
command -v rviz2 >/dev/null  || { echo "manca rviz2 (sudo apt install ros-humble-rviz2)"; exit 1; }

# Chiudi eventuali istanze
pkill -f rviz2                     || true
pkill -f joint_state_publisher_gui || true
pkill -f robot_state_publisher     || true

# Genera URDF da xacro
UR_XACRO="$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro"
UR_URDF="/tmp/${UR_TYPE}.urdf"
xacro "$UR_XACRO" ur_type:=${UR_TYPE} name:=${UR_TYPE} prefix:="" > "$UR_URDF"
[ -s "$UR_URDF" ] || { echo "ERRORE: URDF non generato"; exit 1; }

# Avvia robot_state_publisher (background)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat "$UR_URDF")" \
  --log-level warn > /tmp/rsp.log 2>&1 &
RSP_PID=$!

# Sorgente joint states
if [ "$USE_GUI" = "1" ]; then
  ros2 run joint_state_publisher_gui joint_state_publisher_gui > /tmp/jsp.log 2>&1 &
  PUB_PID=$!
else
  ros2 topic pub -r "$PUB_RATE" /joint_states sensor_msgs/msg/JointState \
  "{name: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
    position: [0.0, -1.2, 1.2, -1.2, 1.57, 0.0]}" > /tmp/joint_pub.log 2>&1 &
  PUB_PID=$!
fi

# Config minima RViz (RobotModel da File)
RVIZ_CFG="/tmp/${UR_TYPE}.rviz"
cat > "$RVIZ_CFG" <<RV
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Source: File
      Description File: $UR_URDF
  Global Options:
    Fixed Frame: base_link
RV

# Avvia RViz
[ "$SOFT_RENDER" = "1" ] && export LIBGL_ALWAYS_SOFTWARE=1
rviz2 -d "$RVIZ_CFG"

# Cleanup quando chiudi RViz
kill "$PUB_PID" 2>/dev/null || true
kill "$RSP_PID" 2>/dev/null || true
echo "Chiuso. Log: /tmp/rsp.log /tmp/jsp.log /tmp/joint_pub.log"

