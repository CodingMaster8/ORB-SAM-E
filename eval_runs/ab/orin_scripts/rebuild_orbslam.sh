#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
export CMAKE_BUILD_PARALLEL_LEVEL=3
export MAKEFLAGS="-j3"
colcon build --packages-select ros2_orb_slam3 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -40
echo "COLCON_RC=${PIPESTATUS[0]}"
echo "REBUILD_DONE"
