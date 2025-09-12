#!/usr/bin/env bash
pkill -f "ros2 topic pub .* /joint_states" || true
pkill -f joint_state_publisher_gui || true
pkill -f move_group || true
pkill -f ur_control.launch.py || true
pkill -f ur_robot_driver || true
pkill -f robot_state_publisher || true
echo "Processi UR3/MoveIt fermati."
