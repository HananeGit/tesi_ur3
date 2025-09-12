# Tesi UR3 su ROS 2 – Pipeline riproducibile (RViz, fake driver, MoveIt 2)

## Obiettivo
Pipeline **one-click** per vedere e muovere **UR3** in **RViz2** e pianificare con **MoveIt 2** in **fake hardware**, senza robot reale. Inclusi script, config, troubleshooting ed esperimenti misurabili.

## Requisiti
- Ubuntu 22.04 + ROS 2 Humble (con `ur_description`, `ur_robot_driver`, `ur_moveit_config` presenti nel tuo workspace).
- Pacchetti: `ros-humble-rviz2`, `ros-humble-joint-state-publisher-gui`, `ros-humble-xacro`.

## Quick start
```bash
# Demo A – Visualizzazione & slider
~/tesi_ur3/scripts/run_ur3_rviz.sh
# Varianti: senza slider / joint_states continuo
USE_GUI=0 PUB_RATE=20 ~/tesi_ur3/scripts/run_ur3_rviz.sh
# Se RViz ha problemi GPU:
SOFT_RENDER=1 ~/tesi_ur3/scripts/run_ur3_rviz.sh

# Demo B – MoveIt 2 (fake hardware)
~/tesi_ur3/scripts/run_ur3_moveit_fake.sh
# Variante GPU
SOFT_RENDER=1 ~/tesi_ur3/scripts/run_ur3_moveit_fake.sh

# Stop di emergenza
~/tesi_ur3/scripts/stop_ur3.sh

