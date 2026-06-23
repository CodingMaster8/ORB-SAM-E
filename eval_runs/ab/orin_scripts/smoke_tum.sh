#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rm -f ~/smoke_mono.log ~/smoke_driver.log ~/smoke_traj.txt ~/smoke_posehz.log
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
  -p node_name_arg:=mono_slam_cpp \
  -p use_viewer:=false \
  -p trajectory_output:="$HOME/smoke_traj.txt" > ~/smoke_mono.log 2>&1 &
MONO_PID=$!
sleep 8
ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \
  -p settings_name:=TUM1 \
  -p dataset_path:="$HOME/tum_data/rgbd_dataset_freiburg1_xyz" \
  -p frame_rate:=30.0 > ~/smoke_driver.log 2>&1 &
DRV_PID=$!
sleep 25
timeout 15 ros2 topic hz /orbslam3/pose > ~/smoke_posehz.log 2>&1
sleep 5
kill -INT $MONO_PID 2>/dev/null
kill -INT $DRV_PID 2>/dev/null
sleep 6
kill -9 $MONO_PID $DRV_PID 2>/dev/null
echo SMOKE_DONE
