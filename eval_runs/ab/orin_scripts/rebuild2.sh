#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
export CMAKE_BUILD_PARALLEL_LEVEL=3
export MAKEFLAGS="-j3"
colcon build --packages-select ros2_orb_slam3 --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ > ~/rebuild2_full.log 2>&1
echo "COLCON_RC=$?" | tee -a ~/rebuild2_full.log
echo "REBUILD2_DONE" | tee -a ~/rebuild2_full.log
