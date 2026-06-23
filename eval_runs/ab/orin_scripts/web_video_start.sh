#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/jetson/jetauto_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
exec ros2 run web_video_server web_video_server --ros-args -p port:=8080 -p address:=0.0.0.0
