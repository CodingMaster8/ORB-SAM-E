#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
  settings_name:=JetAuto \
  camera_topic:=/cam_1/image \
  trajectory_output:=$HOME/live_traj.txt > ~/live_run.log 2>&1
