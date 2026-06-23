#!/usr/bin/env bash
# Replay offline del pipeline ORB-SAM-E sobre un bag (camara remapeada a /replay/image
# para no chocar con la camara viva del robot).
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
  use_filter:=true \
  camera_topic:=/replay/image \
  trajectory_output:=${1:-$HOME/runs/replay_traj.txt} \
  filter_metrics_output:=${2:-$HOME/runs/replay_metrics.json} \
  map_points_output:=${3:-$HOME/runs/replay_map.ply}
