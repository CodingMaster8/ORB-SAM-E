#!/usr/bin/env bash
set -e
PW=jetson
WS=~/ros2_ws
PKG=$WS/src/ORB-SAM-E/ros2_orb_slam3
echo "===[deps]==="
echo "$PW" | sudo -S apt-get install -y libboost-dev libboost-serialization-dev libssl-dev libomp-dev 2>&1 | tail -5
echo "===[DBoW2]==="
cd "$PKG/orb_slam3/Thirdparty/DBoW2"
rm -rf build && cmake -B build -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build build -j4 2>&1 | tail -5
ls -la lib/ 2>&1
echo "===[g2o]==="
cd "$PKG/orb_slam3/Thirdparty/g2o"
rm -rf build && cmake -B build -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build build -j4 2>&1 | tail -5
ls -la lib/ 2>&1
echo "===[colcon ros2_orb_slam3]==="
source /opt/ros/humble/setup.bash
cd "$WS"
export CMAKE_BUILD_PARALLEL_LEVEL=3
export MAKEFLAGS="-j3"
colcon build --packages-select ros2_orb_slam3 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -50
echo "COLCON_RC=${PIPESTATUS[0]}"
echo "ORBSLAM_BUILD_DONE"
