#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export PYTHONPATH=~/ros2_ws/src/ORB-SAM-E/efficientsam3_ros2:$PYTHONPATH
~/venvs/esam3/bin/python3 -m efficientsam3_ros2.bag_detection_overlays \
  --bag ~/runs/bags/dyn1 --out ~/runs/dyn1_overlays --every 90
~/venvs/esam3/bin/python3 -m efficientsam3_ros2.bag_detection_overlays \
  --bag ~/runs/bags/dyn2 --out ~/runs/dyn2_overlays --every 90
echo OVERLAYS_DONE
