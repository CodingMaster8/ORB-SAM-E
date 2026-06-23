#!/usr/bin/env bash
# Teleop con joystick conectado AL ORIN (USB o Bluetooth).
# joy_linux lee /dev/input/js0 -> /joy ; teleop_twist_joy -> /cmd_vel -> chassis (Nano).
# LB = hombre muerto (mantener presionado), RB = turbo.
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
if [ ! -e /dev/input/js0 ]; then
  echo "ERROR: no hay joystick en /dev/input/js0. Conecta el control (USB) o emparejalo por Bluetooth."
  exit 1
fi
ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.08 -p autorepeat_rate:=20.0 &
JOY_PID=$!
exec ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/teleop.yaml -r __node:=teleop_twist_joy_node
