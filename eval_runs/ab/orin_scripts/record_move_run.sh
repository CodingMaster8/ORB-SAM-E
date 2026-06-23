#!/usr/bin/env bash
# Graba bag para un run con movimiento de ORB-SAM-E.
# Camara cruda (re-ejecutable offline para A/B baseline vs filtrado) + pose SLAM + odom + tf.
NAME="${1:-move_$(date +%Y%m%d_%H%M%S)}"
OUT="$HOME/runs/bags/$NAME"
mkdir -p "$HOME/runs/bags"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
exec ros2 bag record -o "$OUT" \
  /cam_1/image /cam_1/camera_info \
  /orbslam3/pose /odom /odom_raw /tf /tf_static
