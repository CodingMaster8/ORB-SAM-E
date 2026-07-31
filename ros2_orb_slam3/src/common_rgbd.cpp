/*

RGB-D mode node for ORB-SLAM3 (issue #16).

Structured to mirror src/common.cpp (MonocularMode) as closely as possible:
same python-driver handshake, same save-outputs behavior, same pose output.
The differences are:
  * subscribes to an RGB topic AND a depth topic, paired with a
    message_filters ApproximateTime synchronizer (queue 10, max interval
    configurable, default 20 ms = the classic TUM associate.py tolerance);
  * calls ORB_SLAM3::System::TrackRGBD instead of TrackMonocular;
  * settings are loaded from orb_slam3/config/RGB-D/<name>.yaml, which must
    contain RGBD.DepthMapFactor (5000 for TUM PNG depth), Stereo.ThDepth and
    Stereo.b;
  * in RGB-D mode SaveTrajectoryTUM (full frame trajectory) is valid, which is
    what published dynamic-SLAM baselines (DynaSLAM/DS-SLAM) evaluate ATE on.

REQUIREMENTS
* Make sure to set path to your workspace in common_rgbd.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common_rgbd.hpp"
#include "rclcpp/qos.hpp"

//* Constructor
RgbdMode::RgbdMode() : Node("rgbd_node_cpp")
{
    //* Find path to home directory
    homeDir = getenv("HOME");

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 RGB-D NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given");           // Name of this agent
    this->declare_parameter("voc_file_arg", "file_not_set");         // Needs to be overriden with appropriate name
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file

    // Parameters for dynamic object filtering pipeline integration.
    // Only the RGB stream is ever filtered: the EfficientSAM3 gray-out would
    // corrupt depth values, so depth always comes raw from the driver.
    this->declare_parameter("use_filtered_images", false); // Set true to take RGB from the EfficientSAM3 output topic
    this->declare_parameter("rgb_topic", "/rgbd_py_driver/rgb_msg");     // Topic for RGB input
    this->declare_parameter("depth_topic", "/rgbd_py_driver/depth_msg"); // Topic for depth input (always raw)

    // If set, the estimated FRAME trajectory (TUM format) is written here on
    // shutdown. Unlike monocular, RGB-D has metric scale and per-frame poses,
    // so SaveTrajectoryTUM is valid and is what the literature reports ATE on.
    this->declare_parameter("trajectory_output", "");
    // Optional: also dump the keyframe-only trajectory (same format the mono
    // node produces) for apples-to-apples comparison with mono runs.
    this->declare_parameter("keyframe_trajectory_output", "");
    // If set, the 3D map points of the ACTIVE map are dumped as an ASCII PLY on shutdown.
    this->declare_parameter("map_points_output", "");

    // Headless control: identical to the mono node, viewer OFF by default.
    this->declare_parameter("use_viewer", false);

    // Pose output: publish the camera pose (Twc) on this topic whenever tracking is healthy.
    this->declare_parameter("pose_topic", "/orbslam3/pose");
    this->declare_parameter("map_frame", "orb_map");
    this->declare_parameter("camera_frame", "orb_cam");

    // Synchronizer tuning. 20 ms is the classic TUM associate.py tolerance;
    // the dataset driver stamps rgb+depth identically so pairs normally match
    // with ~0 slop and this only guards against mispairing on frame drops.
    this->declare_parameter("sync_queue_size", 10);
    this->declare_parameter("sync_max_interval_ms", 20.0);

    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    nodeName = this->get_parameter("node_name_arg").as_string();
    vocFilePath = this->get_parameter("voc_file_arg").as_string();
    settingsFilePath = this->get_parameter("settings_file_path_arg").as_string();

    // Get pipeline integration parameters
    bool useFilteredImages = this->get_parameter("use_filtered_images").as_bool();
    std::string rgbTopicParam = this->get_parameter("rgb_topic").as_string();
    std::string depthTopicParam = this->get_parameter("depth_topic").as_string();

    trajectoryOutputPath = this->get_parameter("trajectory_output").as_string();
    keyframeTrajectoryOutputPath = this->get_parameter("keyframe_trajectory_output").as_string();
    mapPointsOutputPath = this->get_parameter("map_points_output").as_string();

    //* Pose output parameters
    poseTopicName = this->get_parameter("pose_topic").as_string();
    poseMapFrame = this->get_parameter("map_frame").as_string();
    poseCamFrame = this->get_parameter("camera_frame").as_string();

    int syncQueueSize = static_cast<int>(this->get_parameter("sync_queue_size").as_int());
    double syncMaxIntervalMs = this->get_parameter("sync_max_interval_ms").as_double();

    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGB-D/";
    }

    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subexperimentconfigName = "/rgbd_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp node
    pubconfigackName = "/rgbd_py_driver/exp_settings_ack";           // send an acknowledgement to the python node

    // Use parameterized topics - allows switching between direct feed and filtered RGB
    if (useFilteredImages) {
        // RGB comes from the EfficientSAM3 dynamic object filter; depth stays raw.
        subRgbMsgName = "/camera/image_filtered";
        subDepthMsgName = depthTopicParam;
        RCLCPP_INFO(this->get_logger(), "*** USING FILTERED RGB IMAGES (Dynamic Object Filtering ENABLED) ***");
        RCLCPP_INFO(this->get_logger(), "*** Depth is NOT filtered (raw from driver) ***");
    } else {
        // Direct feed (default behavior)
        subRgbMsgName = rgbTopicParam;
        subDepthMsgName = depthTopicParam;
    }

    RCLCPP_INFO(this->get_logger(), "RGB topic: %s", subRgbMsgName.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth topic: %s", subDepthMsgName.c_str());

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&RgbdMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* QoS: BEST_EFFORT to match the python driver's publishers, but with a
    //* deeper queue than the mono node (depth 1 would starve the synchronizer).
    rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
    image_qos.depth = static_cast<size_t>(syncQueueSize);

    //* Synchronized RGB + depth subscribers
    rgbSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, subRgbMsgName, image_qos);
    depthSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, subDepthMsgName, image_qos);

    syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(syncQueueSize), *rgbSub_, *depthSub_);
    syncApproximate_->setMaxIntervalDuration(
        rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(syncMaxIntervalMs * 1e6)));
    syncApproximate_->registerCallback(std::bind(&RgbdMode::RGBD_callback, this, _1, _2));

    //* publisher for the estimated camera pose (Twc), only emitted when tracking is OK
    posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(poseTopicName, 10);
    RCLCPP_INFO(this->get_logger(), "Pose topic: %s (frame %s)", poseTopicName.c_str(), poseMapFrame.c_str());

    //* On-demand save trigger (same contract as the mono node): publishing any
    //* std_msgs/String on /orbslam3/save_outputs dumps trajectory + map points.
    saveTrigger_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/orbslam3/save_outputs", 1,
        std::bind(&RgbdMode::SaveTrigger_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

//* Destructor
RgbdMode::~RgbdMode()
{
    // Stop all threads
    // Release resources and cleanly shutdown
    if (pAgent != nullptr)
    {
        pAgent->Shutdown();
        SaveOutputs();
    }
    pass;
}

//* Save trajectory + map points NOW (no shutdown needed). Triggered on demand
//* via /orbslam3/save_outputs and also from the destructor as a fallback.
void RgbdMode::SaveTrigger_callback(const std_msgs::msg::String& msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Save trigger received, dumping outputs ...");
    SaveOutputs();
}

void RgbdMode::SaveOutputs()
{
    if (pAgent == nullptr) return;

    // Full frame trajectory (TUM format). Valid in RGB-D mode (metric scale,
    // per-frame poses) and the quantity the dynamic-SLAM literature reports
    // ATE RMSE on (DynaSLAM / DS-SLAM tables).
    if (!trajectoryOutputPath.empty())
    {
        try
        {
            pAgent->SaveTrajectoryTUM(trajectoryOutputPath);
            std::cout << "Saved frame trajectory to: "
                      << trajectoryOutputPath << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to save trajectory: " << e.what() << std::endl;
        }
    }

    // Optional keyframe-only trajectory, same output the mono node produces.
    if (!keyframeTrajectoryOutputPath.empty())
    {
        try
        {
            pAgent->SaveKeyFrameTrajectoryTUM(keyframeTrajectoryOutputPath);
            std::cout << "Saved keyframe trajectory to: "
                      << keyframeTrajectoryOutputPath << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to save keyframe trajectory: " << e.what() << std::endl;
        }
    }

    // Sparse 3D reconstruction (map points of the active map) as ASCII PLY.
    if (!mapPointsOutputPath.empty())
    {
        try
        {
            std::vector<ORB_SLAM3::MapPoint*> mapPoints = pAgent->GetAllMapPoints();
            std::vector<Eigen::Vector3f> goodPoints;
            goodPoints.reserve(mapPoints.size());
            for (ORB_SLAM3::MapPoint* mp : mapPoints)
            {
                if (mp != nullptr && !mp->isBad())
                    goodPoints.push_back(mp->GetWorldPos());
            }

            std::ofstream plyFile(mapPointsOutputPath);
            plyFile << "ply\nformat ascii 1.0\n"
                    << "element vertex " << goodPoints.size() << "\n"
                    << "property float x\nproperty float y\nproperty float z\n"
                    << "end_header\n";
            for (const Eigen::Vector3f& p : goodPoints)
                plyFile << p.x() << " " << p.y() << " " << p.z() << "\n";

            std::cout << "Saved " << goodPoints.size()
                      << " map points to: " << mapPointsOutputPath << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to save map points: " << e.what() << std::endl;
        }
    }
    std::cout.flush();
}

//* Callback which accepts experiment parameters from the Python node
void RgbdMode::experimentSetting_callback(const std_msgs::msg::String& msg){

    // Guard against double initialization - only process the first config message
    if (bSettingsFromPython) {
        RCLCPP_WARN(this->get_logger(), "Already initialized, ignoring duplicate config message");
        // Still send ACK so driver doesn't hang
        auto message = std_msgs::msg::String();
        message.data = "ACK";
        configAck_publisher_->publish(message);
        return;
    }

    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();

    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", experimentConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";

    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);
}

//* Method to bind an initialized VSLAM framework to this node
void RgbdMode::initializeVSLAM(std::string& configString){

    // Watchdog, if the paths to vocabulary and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    //* Build .yaml`s file path
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example: .../orb_slam3/config/RGB-D/TUM3.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::RGBD;
    enablePangolinWindow = this->get_parameter("use_viewer").as_bool(); // Headless by default (no display on Orin)
    enableOpenCVWindow = false;

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "RgbdMode node initialized" << std::endl;
}

//* Callback fired by the synchronizer with a matched (RGB, depth) pair
void RgbdMode::RGBD_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,
                             const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
{
    // Frames can arrive before the python handshake finished initializing SLAM.
    if (pAgent == nullptr) return;

    //* Convert ROS images to OpenCV images
    cv_bridge::CvImagePtr cv_ptrRGB;
    cv_bridge::CvImagePtr cv_ptrD;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msgRGB); // passthrough (bgr8 from drivers/filter)
        // Depth passthrough: TUM PNGs arrive as 16UC1 (millimeter*5 units).
        // Do NOT scale here: Tracking::GrabImageRGBD applies
        // RGBD.DepthMapFactor from the settings yaml (it converts any non-CV_32F
        // input with factor 1/DepthMapFactor), so pre-converting to meters would
        // double-scale. A camera that already publishes 32FC1 meters must use a
        // yaml with RGBD.DepthMapFactor: 1.0.
        cv_ptrD = cv_bridge::toCvCopy(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading RGB-D pair: %s", e.what());
        return;
    }

    //* Frame timestamp comes from the RGB header. The dataset driver stamps
    //* headers with the ORIGINAL TUM timestamps (required both for the
    //* synchronizer and so the saved trajectory lines up with groundtruth.txt).
    const double frameTimestamp = rclcpp::Time(msgRGB->header.stamp).seconds();

    //* Perform all ORB-SLAM3 operations in RGB-D mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, frameTimestamp);

    //* Publish the camera pose Twc only while tracking is healthy (same
    //* behavior/frames as the mono node).
    if (pAgent->GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf qwc = Twc.unit_quaternion();

        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp = msgRGB->header.stamp; // keep the source image timestamp
        poseMsg.header.frame_id = poseMapFrame;
        poseMsg.pose.position.x = twc.x();
        poseMsg.pose.position.y = twc.y();
        poseMsg.pose.position.z = twc.z();
        poseMsg.pose.orientation.x = qwc.x();
        poseMsg.pose.orientation.y = qwc.y();
        poseMsg.pose.orientation.z = qwc.z();
        poseMsg.pose.orientation.w = qwc.w();
        posePublisher_->publish(poseMsg);
    }
}
