/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"
#include "rclcpp/qos.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file
    
    // Parameters for dynamic object filtering pipeline integration
    this->declare_parameter("use_filtered_images", false); // Set true to use filtered images from EfficientSAM3
    this->declare_parameter("image_topic", "/mono_py_driver/img_msg"); // Topic for image input
    this->declare_parameter("timestep_topic", "/mono_py_driver/timestep_msg"); // Topic for timestep input  
    // If set, the estimated keyframe trajectory (TUM format) is written here on shutdown,
    // enabling offline ATE/RPE evaluation against the dataset groundtruth.
    this->declare_parameter("trajectory_output", "");
    // If set, the 3D map points of the ACTIVE map are dumped as an ASCII PLY on
    // shutdown (sparse reconstruction; note a tracking reset discards prior maps).
    this->declare_parameter("map_points_output", "");

    // Headless control: the robot/Orin has no display, so the Pangolin viewer is
    // OFF by default. Set use_viewer:=true (e.g. when debugging from a laptop with
    // X forwarding) to bring the visualization back.
    this->declare_parameter("use_viewer", false);

    // Pose output: publish the camera pose (Twc) on this topic whenever tracking is
    // healthy. Frames are ORB-SLAM3-local and intentionally separate from map/odom.
    this->declare_parameter("pose_topic", "/orbslam3/pose");
    this->declare_parameter("map_frame", "orb_map");
    this->declare_parameter("camera_frame", "orb_cam");
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // Get pipeline integration parameters
    rclcpp::Parameter param_use_filtered = this->get_parameter("use_filtered_images");
    bool useFilteredImages = param_use_filtered.as_bool();
    
    rclcpp::Parameter param_img_topic = this->get_parameter("image_topic");
    std::string imgTopicParam = param_img_topic.as_string();
    
    rclcpp::Parameter param_ts_topic = this->get_parameter("timestep_topic");
    std::string tsTopicParam = param_ts_topic.as_string();

    rclcpp::Parameter param_traj_out = this->get_parameter("trajectory_output");
    trajectoryOutputPath = param_traj_out.as_string();
    mapPointsOutputPath = this->get_parameter("map_points_output").as_string();

    //* Pose output parameters
    poseTopicName = this->get_parameter("pose_topic").as_string();
    poseMapFrame = this->get_parameter("map_frame").as_string();
    poseCamFrame = this->get_parameter("camera_frame").as_string();
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp node
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    
    // Use parameterized topics - allows switching between direct camera feed and filtered images
    if (useFilteredImages) {
        // Use filtered images from EfficientSAM3 dynamic object filter
        subImgMsgName = "/camera/image_filtered";
        subTimestepMsgName = "/mono_py_driver/timestep_msg"; // Timestep still from driver
        RCLCPP_INFO(this->get_logger(), "*** USING FILTERED IMAGES (Dynamic Object Filtering ENABLED) ***");
    } else {
        // Use direct image feed (default behavior)
        subImgMsgName = imgTopicParam;
        subTimestepMsgName = tsTopicParam;
    }
    
    RCLCPP_INFO(this->get_logger(), "Image topic: %s", subImgMsgName.c_str());
    RCLCPP_INFO(this->get_logger(), "Timestep topic: %s", subTimestepMsgName.c_str());

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* QoS profile matching the Python driver (BEST_EFFORT reliability)
    rclcpp::QoS image_qos(1);
    image_qos.best_effort();
    image_qos.keep_last(1);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, image_qos, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* publisher for the estimated camera pose (Twc), only emitted when tracking is OK
    posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(poseTopicName, 10);
    RCLCPP_INFO(this->get_logger(), "Pose topic: %s (frame %s)", poseTopicName.c_str(), poseMapFrame.c_str());

    //* On-demand save trigger: publishing any std_msgs/String here dumps the
    //* trajectory + map points immediately. This makes saving robust to messy
    //* teardown (SIGKILL, launch not forwarding SIGINT, power loss imminent...).
    saveTrigger_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/orbslam3/save_outputs", 1,
        std::bind(&MonocularMode::SaveTrigger_callback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
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
void MonocularMode::SaveTrigger_callback(const std_msgs::msg::String& msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Save trigger received, dumping outputs ...");
    SaveOutputs();
}

void MonocularMode::SaveOutputs()
{
    if (pAgent == nullptr) return;

    // Keyframe trajectory (TUM format) for offline ATE/RPE evaluation.
    // Monocular mode must use the keyframe trajectory (SaveTrajectoryTUM is
    // not valid for monocular).
    if (!trajectoryOutputPath.empty())
    {
        try
        {
            pAgent->SaveKeyFrameTrajectoryTUM(trajectoryOutputPath);
            std::cout << "Saved keyframe trajectory to: "
                      << trajectoryOutputPath << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to save trajectory: " << e.what() << std::endl;
        }
    }

    // Sparse 3D reconstruction (map points of the active map) as ASCII PLY
    // for offline visualization (Open3D/CloudCompare/matplotlib).
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
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
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
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = this->get_parameter("use_viewer").as_bool(); // Headless by default (no display on Orin)
    enableOpenCVWindow = false; // OpenCV debug window unused (imshow is commented in Img_callback)
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Frame timestamp: prefer the image header stamp when present (live camera and
    //* the EfficientSAM3 filter preserve it), so pipeline latency does not desync the
    //* frame from the latched timestep. Dataset drivers leave the header at 0 and
    //* fall back to the timestep topic (legacy behavior).
    double frameTimestamp = timeStep;
    const double headerStamp = rclcpp::Time(msg.header.stamp).seconds();
    if (headerStamp > 0.0) frameTimestamp = headerStamp;

    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, frameTimestamp); 

    //* Publish the camera pose Twc (camera w.r.t. ORB-SLAM3 map) only while tracking
    //* is healthy. Skipping the not-initialized/lost states avoids emitting garbage
    //* identity poses. Frames are ORB-SLAM3-local; downstream eval aligns with Sim3.
    if (pAgent->GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf qwc = Twc.unit_quaternion();

        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.header.stamp = msg.header.stamp; // keep the source image timestamp
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


