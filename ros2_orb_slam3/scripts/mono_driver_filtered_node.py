#!/usr/bin/env python3
"""
Python node for the MonocularMode cpp node WITH Dynamic Object Filtering.

This is an alternative driver that publishes images to the EfficientSAM3 filter
node instead of directly to ORB-SLAM3. The filtered images are then forwarded
to ORB-SLAM3 automatically.

Pipeline:
    mono_driver_filtered_node.py --> EfficientSAM3 Filter --> ORB-SLAM3

Author: Modified from original by Azmyin Md. Kamal
Date: 01/01/2024

Requirements:
* Dataset must be configured in EuRoC MAV format
* EfficientSAM3 filter node must be running
* ORB-SLAM3 node must be started with use_filtered_images:=true

Command line arguments:
-- settings_name: EuRoC, TUM2, KITTI etc
-- image_seq: MH01, V102, etc
"""

# Imports
import sys
import os
import glob
import time
import copy
import shutil
from pathlib import Path
import argparse
import natsort
import yaml
import numpy as np
import cv2

# ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Import ROS2 message templates
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError


class MonoDriverFiltered(Node):
    """
    Modified mono driver that publishes to the dynamic filter pipeline.
    
    The key difference from the original driver is that images are published
    to /camera/image_raw which the filter node subscribes to, rather than
    directly to the ORB-SLAM3 node.
    """
    
    def __init__(self, node_name="mono_py_driver_filtered"):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("image_seq", "NULL")
        self.declare_parameter("use_filter", True)  # New parameter
        self.declare_parameter("frame_rate", 20.0)  # Configurable frame rate

        # Parse parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.image_seq = str(self.get_parameter('image_seq').value)
        self.use_filter = bool(self.get_parameter('use_filter').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)

        # Debug
        print(f"-------------- Received parameters --------------------------")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print(f"self.use_filter: {self.use_filter}")
        print(f"self.frame_rate: {self.frame_rate}")
        print()

        # Global path definitions
        self.home_dir = str(Path.home()) + "/ros2_test/src/ros2_orb_slam3"
        self.parent_dir = "TEST_DATASET"
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq

        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")

        # Global variables
        self.node_name = "mono_py_driver_filtered"
        self.image_seq_dir = ""
        self.imgz_seqz = []
        self.time_seqz = []

        # Define a CvBridge object
        self.br = CvBridge()

        # Read images from the chosen dataset
        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(
            self.image_sequence_dir, "mav0"
        )

        print(f"Image directory: {self.image_seq_dir}")
        print(f"Number of images: {len(self.imgz_seqz)}")

        # ROS2 topic names - KEY DIFFERENCE: publish to filter input topic
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        
        if self.use_filter:
            # Publish to filter node input (filter will forward to SLAM)
            self.pub_img_to_agent_name = "/camera/image_raw"
            print(f"*** FILTER MODE: Publishing to {self.pub_img_to_agent_name} ***")
        else:
            # Direct mode (bypass filter)
            self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
            print(f"*** DIRECT MODE: Publishing to {self.pub_img_to_agent_name} ***")
        
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        
        self.send_config = True

        # QoS profile for image topics
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(
            String, self.pub_exp_config_name, 1
        )

        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")

        self.subscribe_exp_ack_ = self.create_subscription(
            String,
            self.sub_exp_ack_name,
            self.ack_callback,
            10
        )

        # Publisher to send RGB image (with QoS for real-time)
        self.publish_img_msg_ = self.create_publisher(
            Image, self.pub_img_to_agent_name, image_qos
        )

        self.publish_timestep_msg_ = self.create_publisher(
            Float64, self.pub_timestep_to_agent_name, 1
        )

        # Initialize work variables
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.show_imgz = False
        self.frame_id = 0
        self.frame_count = 0
        self.inference_time = []

        print()
        print(f"MonoDriverFiltered initialized, attempting handshake with CPP node")

    def get_image_dataset_asl(self, exp_dir, agent_name="mav0"):
        """Returns images and list of timesteps in ascending order from ASL formatted dataset."""
        imgz_file_list = []
        time_list = []

        agent_cam0_fld = exp_dir + "/" + agent_name + "/" + "cam0"
        imgz_file_dir = agent_cam0_fld + "/" + "data" + "/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir), reverse=False)

        for iox in imgz_file_list:
            time_step = iox.split(".")[0]
            time_list.append(time_step)

        return imgz_file_dir, imgz_file_list, time_list

    def ack_callback(self, msg):
        """Callback function for handshake acknowledgement."""
        print(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake_with_cpp_node(self):
        """Send and receive acknowledge of sent configuration settings."""
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def run_py_node(self, idx, imgz_name):
        """Master function that sends the RGB image message."""
        img_msg = None

        img_look_up_path = self.imgz_seqz_dir + imgz_name
        timestep = float(imgz_name.split(".")[0])
        self.frame_id = self.frame_id + 1

        # Convert to ROS Image message
        img_msg = self.br.cv2_to_imgmsg(
            cv2.imread(img_look_up_path), encoding="passthrough"
        )
        
        # Add header with timestamp for synchronization
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = f"camera_frame_{self.frame_id}"
        
        timestep_msg = Float64()
        timestep_msg.data = timestep

        try:
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    n = MonoDriverFiltered("mono_py_driver_filtered")
    rate = n.create_rate(n.frame_rate)

    # Blocking loop to initialize handshake
    while n.send_config:
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        if not n.send_config:
            break

    print(f"Handshake complete")
    
    if n.use_filter:
        print("=" * 60)
        print("IMPORTANT: Make sure the EfficientSAM3 filter node is running!")
        print("  ros2 run efficientsam3_ros2 dynamic_filter_node")
        print("=" * 60)
        time.sleep(1)  # Give time for filter node to be ready

    # Blocking loop to send RGB image and timestep message
    for idx, imgz_name in enumerate(n.imgz_seqz[n.start_frame:n.end_frame]):
        try:
            rclpy.spin_once(n)
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            if n.frame_id > n.frame_stop and n.frame_stop != -1:
                print(f"BREAK!")
                break

        except KeyboardInterrupt:
            break

    # Cleanup
    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
