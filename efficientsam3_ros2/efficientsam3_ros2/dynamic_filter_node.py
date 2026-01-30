#!/usr/bin/env python3
"""
ROS2 Node for Dynamic Object Filtering using EfficientSAM3.

This node subscribes to camera images, filters out dynamic objects using
EfficientSAM3, and publishes the filtered images for downstream SLAM processing.

Subscribed Topics:
    /camera/image_raw (sensor_msgs/Image): Raw camera images

Published Topics:
    /camera/image_filtered (sensor_msgs/Image): Filtered images with dynamic objects masked
    /dynamic_filter/mask (sensor_msgs/Image): Binary mask visualization (optional)
    /dynamic_filter/detections (std_msgs/String): JSON detection info (optional)

Parameters:
    model_path (str): Path to EfficientSAM3 checkpoint
    efficientsam3_path (str): Path to efficientsam3_arm package
    dynamic_classes (list): List of object classes to filter
    confidence_threshold (float): Detection confidence threshold
    masking_strategy (str): How to handle masked regions
    input_topic (str): Input image topic name
    output_topic (str): Output filtered image topic name
    publish_mask (bool): Whether to publish mask visualization
    publish_detections (bool): Whether to publish detection info

Usage:
    ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
        -p model_path:=/path/to/checkpoint.pt \
        -p input_topic:=/camera/image_raw

Author: Generated for EfficientSAM3 + ROS2 ORB-SLAM3 integration
"""

import json
from typing import Optional

import numpy as np
import cv2

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# Import our core filter logic (ROS2-independent)
from .filter_core import DynamicObjectFilter, MaskingStrategy, Detection


class DynamicFilterNode(Node):
    """
    ROS2 Node that filters dynamic objects from camera images.
    
    This node acts as a preprocessing step for SLAM systems, removing
    dynamic objects (people, vehicles, etc.) from the image stream to
    improve SLAM accuracy in dynamic environments.
    """
    
    def __init__(self):
        super().__init__('dynamic_filter_node')
        
        # Declare parameters with defaults
        self._declare_parameters()
        
        # Get parameter values
        self.model_path = self.get_parameter('model_path').value
        self.efficientsam3_path = self.get_parameter('efficientsam3_path').value
        self.dynamic_classes = self.get_parameter('dynamic_classes').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.masking_strategy_str = self.get_parameter('masking_strategy').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_mask = self.get_parameter('publish_mask').value
        self.publish_detections = self.get_parameter('publish_detections').value
        self.device = self.get_parameter('device').value
        self.backbone_type = self.get_parameter('backbone_type').value
        self.model_name = self.get_parameter('model_name').value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').value
        
        # Validate model path
        if not self.model_path:
            self.get_logger().error("model_path parameter is required!")
            raise ValueError("model_path parameter must be set")
        
        # Parse masking strategy
        try:
            masking_strategy = MaskingStrategy(self.masking_strategy_str)
        except ValueError:
            self.get_logger().warn(
                f"Invalid masking_strategy '{self.masking_strategy_str}', using 'grayout'"
            )
            masking_strategy = MaskingStrategy.GRAYOUT
        
        # Parse dynamic classes
        if isinstance(self.dynamic_classes, str):
            # If passed as comma-separated string
            self.dynamic_classes = [c.strip() for c in self.dynamic_classes.split(',')]
        
        # Log configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("Dynamic Filter Node Configuration:")
        self.get_logger().info(f"  Model path: {self.model_path}")
        self.get_logger().info(f"  Device: {self.device}")
        self.get_logger().info(f"  Dynamic classes: {self.dynamic_classes}")
        self.get_logger().info(f"  Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"  Masking strategy: {masking_strategy.value}")
        self.get_logger().info(f"  Input topic: {self.input_topic}")
        self.get_logger().info(f"  Output topic: {self.output_topic}")
        self.get_logger().info(f"  Process every N frames: {self.process_every_n_frames}")
        self.get_logger().info("=" * 60)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize the filter (lazy loading of model)
        self.get_logger().info("Initializing DynamicObjectFilter...")
        self.filter = DynamicObjectFilter(
            model_path=self.model_path,
            efficientsam3_path=self.efficientsam3_path if self.efficientsam3_path else None,
            dynamic_classes=self.dynamic_classes,
            confidence_threshold=self.confidence_threshold,
            masking_strategy=masking_strategy,
            device=self.device,
            backbone_type=self.backbone_type,
            model_name=self.model_name,
        )
        
        # Frame counter for skip logic
        self.frame_count = 0
        self.last_mask: Optional[np.ndarray] = None
        
        # Set up QoS profile for image topics
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber for input images
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            image_qos
        )
        
        # Create publisher for filtered images
        self.image_pub = self.create_publisher(
            Image,
            self.output_topic,
            image_qos
        )
        
        # Optional publishers
        if self.publish_mask:
            self.mask_pub = self.create_publisher(
                Image,
                '/dynamic_filter/mask',
                image_qos
            )
        else:
            self.mask_pub = None
        
        if self.publish_detections:
            self.detections_pub = self.create_publisher(
                String,
                '/dynamic_filter/detections',
                10
            )
        else:
            self.detections_pub = None
        
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.log_stats)
        
        self.get_logger().info("Dynamic Filter Node initialized successfully!")
        self.get_logger().info(f"Waiting for images on {self.input_topic}...")
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values."""
        self.declare_parameter('model_path', '')
        self.declare_parameter('efficientsam3_path', '')
        self.declare_parameter('dynamic_classes', [
            'person', 'car', 'truck', 'bus', 'motorcycle', 'bicycle', 'dog', 'cat'
        ])
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('masking_strategy', 'grayout')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_filtered')
        self.declare_parameter('publish_mask', True)
        self.declare_parameter('publish_detections', False)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('backbone_type', 'repvit')
        self.declare_parameter('model_name', 's')
        self.declare_parameter('process_every_n_frames', 1)
    
    def image_callback(self, msg: Image):
        """
        Callback for incoming camera images.
        
        Args:
            msg: ROS2 Image message
        """
        self.frame_count += 1
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        
        # Check if we should process this frame or reuse last mask
        should_process = (self.frame_count % self.process_every_n_frames) == 0
        
        if should_process:
            # Run full inference
            filtered_image, mask, detections = self.filter.process_frame(cv_image)
            self.last_mask = mask
            
            # Publish detections info if enabled
            if self.detections_pub is not None and detections:
                det_msg = String()
                det_msg.data = json.dumps([
                    {
                        'confidence': d.confidence,
                        'bbox': list(d.bbox),
                    }
                    for d in detections
                ])
                self.detections_pub.publish(det_msg)
        else:
            # Reuse last mask for faster processing
            if self.last_mask is not None and self.last_mask.any():
                filtered_image = self.filter._apply_mask(cv_image, self.last_mask)
                mask = self.last_mask
            else:
                filtered_image = cv_image
                mask = None
        
        # Publish filtered image
        try:
            filtered_msg = self.bridge.cv2_to_imgmsg(filtered_image, encoding='bgr8')
            filtered_msg.header = msg.header  # Preserve timestamp
            self.image_pub.publish(filtered_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error publishing filtered image: {e}")
        
        # Publish mask visualization if enabled
        if self.mask_pub is not None and mask is not None:
            try:
                # Convert binary mask to viewable image
                mask_vis = (mask * 255).astype(np.uint8)
                mask_msg = self.bridge.cv2_to_imgmsg(mask_vis, encoding='mono8')
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Error publishing mask: {e}")
    
    def log_stats(self):
        """Periodically log processing statistics."""
        stats = self.filter.get_stats()
        self.get_logger().info(
            f"Stats: frames={stats['total_frames_processed']}, "
            f"detections={stats['total_detections']}, "
            f"avg_det/frame={stats['avg_detections_per_frame']:.2f}"
        )


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    try:
        node = DynamicFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
