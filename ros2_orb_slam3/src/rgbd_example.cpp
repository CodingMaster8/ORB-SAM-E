/*
* RGB-D node entrypoint (issue #16), mirrors src/mono_example.cpp.
* Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
* Compatible with ROS2 Humble
*/

//* Import all necessary modules
#include "ros2_orb_slam3/common_rgbd.hpp"

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node

    //* Declare a node object
    auto node = std::make_shared<RgbdMode>();

    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------
