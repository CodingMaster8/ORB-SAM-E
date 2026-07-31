// Include file for the RGB-D node (issue #16)
#ifndef COMMON_RGBD_HPP // Header guard to prevent multiple inclusions
#define COMMON_RGBD_HPP

// C++ includes
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <cstdlib> // to find home directory
#include <memory>

#include <cstring>
#include <sstream>

//* ROS2 includes
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" //* Camera pose output (for eval/downstream)

//* message_filters: pair the RGB and depth streams by header timestamp.
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::placeholders::_1;
using std::placeholders::_2;

// Include Eigen
#include <Eigen/Dense>

// Include cv-bridge
// NOTE: ROS 2 Humble ships only cv_bridge/cv_bridge.h (the .hpp variant exists
// from Iron/Jazzy onward). Use .h so this builds on the robot's Humble install.
#include <cv_bridge/cv_bridge.h>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

#ifndef pass
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation
#endif

//* Node specific definitions
//* RGB-D counterpart of MonocularMode (common.hpp). The two streams are paired
//* with a message_filters ApproximateTime synchronizer and fed to
//* ORB_SLAM3::System::TrackRGBD. The TUM dataset driver stamps the RGB and the
//* associated depth frame with the SAME header timestamp, so pairs match with
//* ~0 slop; the max interval only guards against mispairing on frame drops.
class RgbdMode : public rclcpp::Node
{
    public:
    std::string experimentConfig = ""; // String to receive settings sent by the python driver
    std::string receivedConfig = "";

    //* Class constructor
    RgbdMode(); // Constructor

    ~RgbdMode(); // Destructor

    private:

        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/"; //! Change to match path to your workspace
        std::string nodeName = "";         // Name of this node
        std::string vocFilePath = "";      // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file (orb_slam3/config/RGB-D/)
        bool bSettingsFromPython = false;  // Flag set once when experiment setting from python node is received

        std::string subexperimentconfigName = ""; // Subscription topic name
        std::string pubconfigackName = "";        // Publisher topic name
        std::string subRgbMsgName = "";           // Topic carrying the RGB images
        std::string subDepthMsgName = "";         // Topic carrying the registered depth images
        std::string trajectoryOutputPath = "";    // If set, full frame trajectory (TUM format) is saved here on shutdown
        std::string keyframeTrajectoryOutputPath = ""; // If set, keyframe trajectory (TUM format) is saved here on shutdown
        std::string mapPointsOutputPath = "";     // If set, map points of the active map are saved here (ASCII PLY) on shutdown

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr saveTrigger_subscription_; // on-demand save of trajectory + map

        //* Synchronized RGB + depth subscribers (ApproximateTime with a tight
        //* max interval; the dataset driver stamps both images identically).
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgbSub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depthSub_;
        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate_;

        //* Camera pose (Twc) output, published when tracking is OK. Frames are
        //* ORB-SLAM3-local (poseMapFrame/poseCamFrame), kept separate from the robot's
        //* map/odom so this never interferes with the EKF/Nav2 TF tree.
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
        std::string poseTopicName = "";
        std::string poseMapFrame = "";
        std::string poseCamFrame = "";

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent = nullptr; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false;   // Shows OpenCV window output

        //* ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg); // Callback to process settings sent over by Python node
        void RGBD_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,
                           const sensor_msgs::msg::Image::ConstSharedPtr& msgD); // Synchronized RGB + depth pair
        void SaveTrigger_callback(const std_msgs::msg::String& msg); // On-demand dump of trajectory + map points
        void SaveOutputs(); // Writes trajectory_output (TUM), keyframe_trajectory_output (TUM) and map_points_output (PLY) if configured

        //* Helper functions
        void initializeVSLAM(std::string& configString); //* Method to bind an initialized VSLAM framework to this node
};

#endif
