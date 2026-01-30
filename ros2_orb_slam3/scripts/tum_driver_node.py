#!/usr/bin/env python3
"""
Python node for ORB-SLAM3 with TUM RGB-D Dataset support.

TUM RGB-D dataset format:
    sequence_name/
    ├── rgb/
    │   ├── 1305031102.175304.png
    │   └── ...
    ├── depth/
    │   └── ...
    ├── rgb.txt          # timestamp filename pairs
    ├── depth.txt        # timestamp filename pairs
    └── groundtruth.txt  # ground truth poses

Author: Based on mono_driver_node.py by Azmyin Md. Kamal
Date: 2024

Command line arguments:
-- settings_name: TUM1, TUM2, TUM3 (camera calibration config)
-- dataset_path: Full path to TUM dataset sequence folder
-- use_filter: Whether to use EfficientSAM3 dynamic filtering
"""

import sys
import os
import time
from pathlib import Path
import numpy as np
import cv2

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message templates
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class TUMDatasetDriver(Node):
    """
    ROS2 node that reads TUM RGB-D format datasets and publishes images.
    
    Supports both direct mode (to ORB-SLAM3) and filtered mode (through EfficientSAM3).
    """
    
    def __init__(self, node_name="tum_driver_node"):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("settings_name", "TUM1")  # TUM1, TUM2, TUM3
        self.declare_parameter("dataset_path", "")  # Full path to dataset
        self.declare_parameter("use_filter", False)  # Use EfficientSAM3 filter
        self.declare_parameter("frame_rate", 30.0)  # TUM datasets are typically 30fps
        self.declare_parameter("start_frame", 0)
        self.declare_parameter("end_frame", -1)  # -1 means all frames
        self.declare_parameter("use_depth", False)  # For future RGB-D support

        # Parse parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.dataset_path = str(self.get_parameter('dataset_path').value)
        self.use_filter = bool(self.get_parameter('use_filter').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.start_frame = int(self.get_parameter('start_frame').value)
        self.end_frame = int(self.get_parameter('end_frame').value)
        self.use_depth = bool(self.get_parameter('use_depth').value)

        # Validate dataset path
        if not self.dataset_path:
            # Default path
            self.dataset_path = str(Path.home()) + "/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum"
        
        # Print configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("TUM Dataset Driver Configuration")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Settings: {self.settings_name}")
        self.get_logger().info(f"Dataset path: {self.dataset_path}")
        self.get_logger().info(f"Use filter: {self.use_filter}")
        self.get_logger().info(f"Frame rate: {self.frame_rate}")
        self.get_logger().info("=" * 60)

        # CvBridge for image conversion
        self.br = CvBridge()

        # Load dataset
        self.rgb_timestamps, self.rgb_files = self.load_tum_dataset(self.dataset_path)
        
        if len(self.rgb_files) == 0:
            self.get_logger().error(f"No images found in {self.dataset_path}")
            self.get_logger().error("Expected TUM format with rgb/ folder and rgb.txt file")
            raise RuntimeError("Dataset not found or empty")
        
        self.get_logger().info(f"Loaded {len(self.rgb_files)} images from dataset")

        # ROS2 topic names
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        
        if self.use_filter:
            self.pub_img_name = "/camera/image_raw"  # Filter node subscribes here
            self.get_logger().info(f"FILTER MODE: Publishing to {self.pub_img_name}")
        else:
            self.pub_img_name = "/mono_py_driver/img_msg"  # Direct to ORB-SLAM3
            self.get_logger().info(f"DIRECT MODE: Publishing to {self.pub_img_name}")
        
        self.pub_timestep_name = "/mono_py_driver/timestep_msg"

        # State
        self.send_config = True
        self.frame_id = 0

        # QoS profile
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers and Subscribers
        self.pub_config = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.sub_ack = self.create_subscription(String, self.sub_exp_ack_name, self.ack_callback, 10)
        self.pub_img = self.create_publisher(Image, self.pub_img_name, image_qos)
        self.pub_timestep = self.create_publisher(Float64, self.pub_timestep_name, 1)

        self.get_logger().info("TUM Driver initialized, attempting handshake...")

    def load_tum_dataset(self, dataset_path):
        """
        Load TUM RGB-D dataset format.
        
        TUM rgb.txt format:
            # timestamp filename
            1305031102.175304 rgb/1305031102.175304.png
            
        Returns:
            timestamps: List of float timestamps
            filenames: List of full paths to RGB images
        """
        timestamps = []
        filenames = []
        
        rgb_txt_path = os.path.join(dataset_path, "rgb.txt")
        rgb_folder = os.path.join(dataset_path, "rgb")
        
        # Method 1: Read from rgb.txt (preferred)
        if os.path.exists(rgb_txt_path):
            self.get_logger().info(f"Reading image list from {rgb_txt_path}")
            with open(rgb_txt_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    # Skip comments and empty lines
                    if line.startswith('#') or len(line) == 0:
                        continue
                    
                    parts = line.split()
                    if len(parts) >= 2:
                        timestamp = float(parts[0])
                        filename = parts[1]
                        
                        # Build full path
                        full_path = os.path.join(dataset_path, filename)
                        
                        if os.path.exists(full_path):
                            timestamps.append(timestamp)
                            filenames.append(full_path)
                        else:
                            self.get_logger().warning(f"Image not found: {full_path}")
        
        # Method 2: Fallback - scan rgb/ folder directly
        elif os.path.exists(rgb_folder):
            self.get_logger().info(f"No rgb.txt found, scanning {rgb_folder} directly")
            
            # Get all png/jpg files
            image_files = []
            for ext in ['*.png', '*.jpg', '*.jpeg']:
                import glob
                image_files.extend(glob.glob(os.path.join(rgb_folder, ext)))
            
            # Sort by filename (which is the timestamp in TUM format)
            image_files = sorted(image_files)
            
            for img_path in image_files:
                basename = os.path.basename(img_path)
                # Extract timestamp from filename (e.g., "1305031102.175304.png")
                try:
                    timestamp = float(basename.rsplit('.', 1)[0])
                except ValueError:
                    # If filename is not a timestamp, use index
                    timestamp = float(len(timestamps))
                
                timestamps.append(timestamp)
                filenames.append(img_path)
        
        return timestamps, filenames

    def ack_callback(self, msg):
        """Handle acknowledgement from ORB-SLAM3 node."""
        self.get_logger().info(f"Received ACK: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake(self):
        """Send configuration to ORB-SLAM3 node."""
        if self.send_config:
            msg = String()
            msg.data = self.settings_name  # e.g., "TUM1", "TUM2", "TUM3"
            self.pub_config.publish(msg)
            time.sleep(0.01)

    def publish_frame(self, idx):
        """Publish a single frame."""
        if idx >= len(self.rgb_files):
            return False
        
        img_path = self.rgb_files[idx]
        timestamp = self.rgb_timestamps[idx]
        
        # Read image
        img = cv2.imread(img_path)
        if img is None:
            self.get_logger().warning(f"Failed to read image: {img_path}")
            return True  # Continue to next frame
        
        # Convert to ROS message
        try:
            img_msg = self.br.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = f"camera_frame_{self.frame_id}"
            
            timestep_msg = Float64()
            timestep_msg.data = timestamp
            
            # Publish timestep first, then image
            self.pub_timestep.publish(timestep_msg)
            self.pub_img.publish(img_msg)
            
            self.frame_id += 1
            
            if self.frame_id % 100 == 0:
                self.get_logger().info(f"Published frame {self.frame_id}/{len(self.rgb_files)}")
                
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TUMDatasetDriver("tum_driver_node")
    except RuntimeError as e:
        print(f"Error: {e}")
        rclpy.shutdown()
        return
    
    rate = node.create_rate(node.frame_rate)
    
    # Handshake with ORB-SLAM3
    while node.send_config:
        node.handshake()
        rclpy.spin_once(node)
        if not node.send_config:
            break
    
    node.get_logger().info("Handshake complete!")
    
    if node.use_filter:
        node.get_logger().info("=" * 60)
        node.get_logger().info("IMPORTANT: Make sure EfficientSAM3 filter node is running!")
        node.get_logger().info("  ros2 run efficientsam3_ros2 dynamic_filter_node")
        node.get_logger().info("=" * 60)
        time.sleep(1)
    
    # Determine frame range
    start = node.start_frame
    end = node.end_frame if node.end_frame > 0 else len(node.rgb_files)
    end = min(end, len(node.rgb_files))
    
    node.get_logger().info(f"Playing frames {start} to {end}")
    
    # Main loop - publish frames
    for idx in range(start, end):
        try:
            rclpy.spin_once(node)
            if not node.publish_frame(idx):
                break
            rate.sleep()
        except KeyboardInterrupt:
            break
    
    node.get_logger().info("Dataset playback complete")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
