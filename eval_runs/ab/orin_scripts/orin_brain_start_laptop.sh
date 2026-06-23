#!/bin/bash
# Como orin_brain_start.sh pero con cyclone_both.xml: expone el DDS al hotspot
# para que la laptop vea /map, /scan, /tf (RViz) y pueda enviar /cmd_vel (teleop).
pkill -9 -f 'ros2 launch mi_proyecto_sim' 2>/dev/null
pkill -9 -f slam_toolbox  2>/dev/null
pkill -9 -f nav2_amcl     2>/dev/null
pkill -9 -f rf2o_laser_odometry 2>/dev/null
pkill -9 -f control_diferencial 2>/dev/null
pkill -9 -f planificador_rrt    2>/dev/null
pkill -9 -f nav_goal_bridge     2>/dev/null
pkill -9 -f map_server_planner  2>/dev/null
pkill -9 -f map_server          2>/dev/null
pkill -9 -f filtro_lidar        2>/dev/null
pkill -9 -f wait_for_tf         2>/dev/null
sleep 1

export HOME=/home/jetson
source /opt/ros/humble/setup.bash
source /home/jetson/jetauto_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclone_both.xml

exec ros2 launch mi_proyecto_sim "$@"
