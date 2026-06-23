#!/usr/bin/env bash
# Full ORB-SAM-E pipeline: camera -> EfficientSAM3 filter -> ORB-SLAM3 (headless).
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
exec ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
  use_filter:=true \
  trajectory_output:=${1:-$HOME/orbslam_run_filtered.txt} map_points_output:=${3:-$HOME/orbslam_map.ply} \
  filter_metrics_output:=${2:-$HOME/filter_metrics.json}
