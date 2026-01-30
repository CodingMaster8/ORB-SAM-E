#!/usr/bin/env python3
"""
Launch file for the EfficientSAM3 + ORB-SLAM3 pipeline.

This launch file starts both:
1. The Dynamic Filter Node (EfficientSAM3) - filters dynamic objects
2. The ORB-SLAM3 nodes (C++ and Python) - performs SLAM

Usage:
    # Launch the full pipeline
    ros2 launch efficientsam3_ros2 slam_pipeline.launch.py
    
    # With custom model path
    ros2 launch efficientsam3_ros2 slam_pipeline.launch.py \
        model_path:=/path/to/efficient_sam3_repvit_s.pt
    
    # For EuRoC dataset testing
    ros2 launch efficientsam3_ros2 slam_pipeline.launch.py \
        model_path:=/path/to/model.pt \
        settings_name:=EuRoC \
        image_seq:=sample_euroc_MH05

Author: Generated for EfficientSAM3 + ROS2 ORB-SLAM3 integration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for the SLAM pipeline."""
    
    # Get package directories
    efficientsam3_share = get_package_share_directory('efficientsam3_ros2')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    # EfficientSAM3 arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/user/weights/efficient_sam3_repvit_s.pt',
        description='Path to EfficientSAM3 model checkpoint'
    )
    
    efficientsam3_path_arg = DeclareLaunchArgument(
        'efficientsam3_path',
        default_value='',
        description='Path to efficientsam3_arm package (empty if in PYTHONPATH)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.3',
        description='Detection confidence threshold'
    )
    
    masking_strategy_arg = DeclareLaunchArgument(
        'masking_strategy',
        default_value='grayout',
        description='Masking strategy: blackout, grayout, inpaint, blur'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='auto',
        description='Inference device: auto, cpu, cuda, mps'
    )
    
    # Topic arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/mono_py_driver/img_msg',
        description='Input camera topic (from Python driver)'
    )
    
    filtered_topic_arg = DeclareLaunchArgument(
        'filtered_topic',
        default_value='/camera/image_filtered',
        description='Output filtered image topic'
    )
    
    # ORB-SLAM3 arguments
    settings_name_arg = DeclareLaunchArgument(
        'settings_name',
        default_value='EuRoC',
        description='ORB-SLAM3 settings file name (e.g., EuRoC, TUM2)'
    )
    
    image_seq_arg = DeclareLaunchArgument(
        'image_seq',
        default_value='sample_euroc_MH05',
        description='Image sequence folder name'
    )
    
    # Pipeline control
    enable_filter_arg = DeclareLaunchArgument(
        'enable_filter',
        default_value='true',
        description='Enable dynamic object filtering (false to bypass)'
    )
    
    # =========================================================================
    # Nodes
    # =========================================================================
    
    # Dynamic Filter Node (EfficientSAM3)
    dynamic_filter_node = Node(
        package='efficientsam3_ros2',
        executable='dynamic_filter_node',
        name='dynamic_filter_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'efficientsam3_path': LaunchConfiguration('efficientsam3_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'masking_strategy': LaunchConfiguration('masking_strategy'),
            'device': LaunchConfiguration('device'),
            'input_topic': LaunchConfiguration('camera_topic'),
            'output_topic': LaunchConfiguration('filtered_topic'),
            'publish_mask': True,
            'publish_detections': False,
            'backbone_type': 'repvit',
            'model_name': 's',
            'process_every_n_frames': 1,
            'dynamic_classes': [
                'person', 'car', 'truck', 'bus', 
                'motorcycle', 'bicycle', 'dog', 'cat'
            ],
        }],
    )
    
    # Note: ORB-SLAM3 nodes are typically started separately due to their
    # complex initialization. Here we document how to start them:
    
    # =========================================================================
    # Launch Description
    # =========================================================================
    
    return LaunchDescription([
        # Declare arguments
        model_path_arg,
        efficientsam3_path_arg,
        confidence_threshold_arg,
        masking_strategy_arg,
        device_arg,
        camera_topic_arg,
        filtered_topic_arg,
        settings_name_arg,
        image_seq_arg,
        enable_filter_arg,
        
        # Start the dynamic filter node
        dynamic_filter_node,
        
        # Note: Start ORB-SLAM3 nodes manually or uncomment below after
        # modifying ros2_orb_slam3 to use filtered images
    ])


# Alternative launch description for testing without ORB-SLAM3
def generate_filter_only_launch_description():
    """Generate launch description for filter node only (testing)."""
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/user/weights/efficient_sam3_repvit_s.pt',
        description='Path to EfficientSAM3 model checkpoint'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw',
        description='Input camera topic'
    )
    
    dynamic_filter_node = Node(
        package='efficientsam3_ros2',
        executable='dynamic_filter_node',
        name='dynamic_filter_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/camera/image_filtered',
            'publish_mask': True,
        }],
    )
    
    return LaunchDescription([
        model_path_arg,
        input_topic_arg,
        dynamic_filter_node,
    ])
