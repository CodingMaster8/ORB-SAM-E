#!/usr/bin/env bash
# Replay offline BASELINE (sin filtro EfficientSAM3) sobre un bag.
# Camara remapeada a /replay/image; el driver reenvia al topic del nodo C++.
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
  use_filter:=false \
  camera_topic:=/replay/image \
  trajectory_output:=${1:-$HOME/runs/replay_baseline_traj.txt} \
  map_points_output:=${2:-$HOME/runs/replay_baseline_map.ply}
