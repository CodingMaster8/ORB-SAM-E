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
    mask_dilation_px (int): Dilate the cached mask by this many pixels (0 disables)
    mask_dilation_px_per_sec (float): Extra dilation per second of mask age (0 disables)
    mask_max_age_sec (float): Stop applying masks older than this (<=0 disables)

Usage:
    ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
        -p model_path:=/path/to/checkpoint.pt \
        -p input_topic:=/camera/image_raw

Author: Generated for EfficientSAM3 + ROS2 ORB-SLAM3 integration
"""

import json
import threading
import time
from typing import Optional, Tuple

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
        self.dynamic_prompts = self.get_parameter('dynamic_prompts').value
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
        self.num_threads = self.get_parameter('num_threads').value
        self.metrics_output = self.get_parameter('metrics_output').value
        self.use_fp16 = self.get_parameter('use_fp16').value
        self.mask_dilation_px = int(self.get_parameter('mask_dilation_px').value)
        self.mask_dilation_px_per_sec = float(
            self.get_parameter('mask_dilation_px_per_sec').value
        )
        self.mask_max_age_sec = float(self.get_parameter('mask_max_age_sec').value)
        
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
        self.get_logger().info(f"  Dynamic prompts: {self.dynamic_prompts}")
        self.get_logger().info(f"  Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"  Masking strategy: {masking_strategy.value}")
        self.get_logger().info(f"  Input topic: {self.input_topic}")
        self.get_logger().info(f"  Output topic: {self.output_topic}")
        self.get_logger().info(f"  Process every N frames: {self.process_every_n_frames}")
        self.get_logger().info(f"  fp16 autocast: {self.use_fp16}")
        self.get_logger().info(
            f"  Mask dilation: {self.mask_dilation_px} px "
            f"(+{self.mask_dilation_px_per_sec} px/s of staleness)"
        )
        self.get_logger().info(f"  Mask max age: {self.mask_max_age_sec} s")
        self.get_logger().info(f"  Num prompts: {len(self.dynamic_prompts)}")
        self.get_logger().info(f"  CPU threads: {self.num_threads if self.num_threads > 0 else 'auto'}")
        self.get_logger().info("=" * 60)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize the filter
        self.get_logger().info("Initializing DynamicObjectFilter...")
        self.filter = DynamicObjectFilter(
            model_path=self.model_path,
            efficientsam3_path=self.efficientsam3_path if self.efficientsam3_path else None,
            dynamic_classes=self.dynamic_classes,
            dynamic_prompts=self.dynamic_prompts,
            confidence_threshold=self.confidence_threshold,
            masking_strategy=masking_strategy,
            device=self.device,
            backbone_type=self.backbone_type,
            model_name=self.model_name,
            num_threads=self.num_threads,
            use_fp16=self.use_fp16,
        )
        
        # Eagerly load the model NOW so the first frame isn't blocked by cold start
        self.get_logger().info("Loading EfficientSAM3 model (this may take 20-30s on CPU)...")
        self.filter.ensure_model_loaded()
        self.get_logger().info("Model ready!")
        
        # Frame counter + async inference state.
        # The image callback never blocks on the GPU: it applies the latest
        # available mask and republishes at full camera rate, while a worker
        # thread keeps refreshing the mask on the newest frame.
        self.frame_count = 0
        # Latest successful inference result: (mask-or-None, monotonic timestamp).
        # Swapped as a single tuple reference (atomic under GIL) so the mask and
        # its age are always read consistently in the image callback.
        self._mask_state: Optional[Tuple[Optional[np.ndarray], float]] = None
        self.skipped_frames = 0   # passthrough frames published with no mask available
        self.reused_frames = 0    # frames published with a (possibly stale) mask
        self.expired_frames = 0   # passthrough frames because the cached mask was too old
        # Cached elliptical dilation kernel (rebuilt when the radius changes).
        self._dilate_kernel: Optional[np.ndarray] = None
        self._dilate_kernel_px = -1
        self._latest_frame = None  # newest (cv_image, header) awaiting inference
        self._frame_lock = threading.Lock()
        self._new_frame_evt = threading.Event()
        self._stop_evt = threading.Event()
        self._last_inferred_frame = 0
        self._infer_thread = threading.Thread(
            target=self._inference_loop, name="esam3_inference", daemon=True
        )
        self._infer_thread.start()
        
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
        self.declare_parameter('dynamic_classes', ['person'])
        # Multi-prompt body-part prompts that work well with MobileCLIP-S1
        self.declare_parameter('dynamic_prompts', [
            'human leg and hands', 'human shirt', 'human face',
            'human hands', 'humans pants', 'human head'
        ])
        self.declare_parameter('confidence_threshold', 0.03)
        self.declare_parameter('masking_strategy', 'grayout')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_filtered')
        self.declare_parameter('publish_mask', True)
        self.declare_parameter('publish_detections', False)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('backbone_type', 'repvit')
        self.declare_parameter('model_name', 's')
        self.declare_parameter('process_every_n_frames', 1)
        # If set, filtering/timing stats are written here (JSON) on shutdown.
        self.declare_parameter('metrics_output', '')
        self.declare_parameter('num_threads', 0)  # 0 = PyTorch default
        self.declare_parameter('use_fp16', False)  # fp16 autocast on CUDA (~1.9x on Orin)
        # Dilate the cached mask (elliptical kernel, radius in pixels) before
        # applying it, to cover object motion between slow inference updates.
        # 0 disables dilation.
        self.declare_parameter('mask_dilation_px', 20)
        # Extra dilation radius per second of mask staleness (pixels/second).
        # 0.0 disables age-scaled dilation.
        self.declare_parameter('mask_dilation_px_per_sec', 0.0)
        # If the last successful inference is older than this (seconds), stop
        # applying the mask and pass frames through unfiltered. <= 0 disables.
        self.declare_parameter('mask_max_age_sec', 2.0)
    
    def image_callback(self, msg: Image):
        """
        Fast path: every camera frame is republished immediately with the latest
        available mask applied. GPU inference never runs here (see _inference_loop).
        
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
        
        # Hand the newest frame to the inference worker (it always takes the latest).
        with self._frame_lock:
            self._latest_frame = (cv_image, msg.header)
        self._new_frame_evt.set()
        
        # Apply the most recent mask (possibly a few frames stale - dynamic objects
        # move little in ~0.6s at robot speeds) and publish at full camera rate.
        # The mask is dilated to cover object motion since it was computed, and
        # dropped entirely (passthrough) once it exceeds mask_max_age_sec.
        mask_state = self._mask_state
        if mask_state is not None:
            mask, mask_stamp = mask_state
            mask_age = time.monotonic() - mask_stamp
        else:
            mask, mask_age = None, None

        if (
            mask is not None
            and self.mask_max_age_sec > 0.0
            and mask_age > self.mask_max_age_sec
        ):
            self.get_logger().warning(
                f"Cached dynamic-object mask is stale ({mask_age:.1f}s > "
                f"{self.mask_max_age_sec:.1f}s limit); publishing frames "
                f"unfiltered until inference recovers",
                throttle_duration_sec=5.0,
            )
            mask = None
            self.expired_frames += 1

        if mask is not None and mask.any():
            mask = self._dilate_mask(mask, mask_age)
            filtered_image = self.filter._apply_mask(cv_image, mask)
            self.reused_frames += 1
        else:
            filtered_image = cv_image
            mask = None
            self.skipped_frames += 1
        
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
    
    def _dilate_mask(self, mask: np.ndarray, age_sec: Optional[float]) -> np.ndarray:
        """
        Dilate the cached mask with an elliptical kernel so it still covers a
        moving object that has shifted since the mask was computed.

        Radius = mask_dilation_px + mask_dilation_px_per_sec * mask age.
        Returns the mask unchanged if the effective radius is <= 0.
        """
        px = self.mask_dilation_px
        if self.mask_dilation_px_per_sec > 0.0 and age_sec is not None:
            px += int(self.mask_dilation_px_per_sec * age_sec)
        if px <= 0:
            return mask
        # Cap the kernel radius so a pathological age can't blank the frame
        # (and to bound the cost of cv2.dilate).
        px = min(px, 100)
        if px != self._dilate_kernel_px:
            self._dilate_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * px + 1, 2 * px + 1)
            )
            self._dilate_kernel_px = px
        return cv2.dilate(mask.astype(np.uint8), self._dilate_kernel)

    def _inference_loop(self):
        """Worker thread: run EfficientSAM3 on the newest frame, refresh the mask.
        
        Runs as fast as the GPU allows (~0.6s/frame on the Orin with fp16);
        process_every_n_frames additionally enforces a minimum number of camera
        frames between inferences (useful to cap GPU load alongside SLAM).
        """
        while not self._stop_evt.is_set():
            if not self._new_frame_evt.wait(timeout=0.2):
                continue
            self._new_frame_evt.clear()
            
            # Throttle: require N new camera frames since the last inference.
            if (self.frame_count - self._last_inferred_frame) < self.process_every_n_frames:
                continue
            
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is None:
                continue
            
            cv_image, header = frame
            self._last_inferred_frame = self.frame_count
            try:
                _, mask, detections = self.filter.process_frame(cv_image)
            except Exception as e:
                # Note: the timestamp in _mask_state is NOT refreshed on failure,
                # so a crashing/stuck worker lets the cached mask age out and the
                # callback falls back to passthrough after mask_max_age_sec.
                self.get_logger().error(f"Inference error: {e}")
                continue

            # Atomic reference swap (mask + freshness timestamp together);
            # the callback picks it up on the next frame.
            self._mask_state = (mask, time.monotonic())
            
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
    
    def log_stats(self):
        """Periodically log processing statistics."""
        stats = self.filter.get_stats()
        self.get_logger().info(
            f"Stats: received={self.frame_count}, "
            f"inferred={stats['total_frames_processed']} (async), "
            f"published_with_mask={self.reused_frames}, "
            f"published_no_mask={self.skipped_frames} "
            f"(expired={self.expired_frames}), "
            f"detections={stats['total_detections']}, "
            f"avg_det/frame={stats['avg_detections_per_frame']:.2f}, "
            f"inference={stats['inference_fps']:.1f} FPS "
            f"({stats['inference_mean_ms']:.1f} ms), "
            f"device={stats['device']}"
        )

    def write_metrics(self):
        """Write the final filtering/timing stats to JSON (if metrics_output set)."""
        if not self.metrics_output:
            return
        try:
            import json
            stats = self.filter.get_stats()
            with open(self.metrics_output, 'w') as f:
                json.dump(stats, f, indent=2)
            self.get_logger().info(f"Wrote filter metrics to {self.metrics_output}")
        except Exception as e:
            self.get_logger().error(f"Failed to write metrics: {e}")


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    node = None
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
        if node is not None:
            node._stop_evt.set()
            node._infer_thread.join(timeout=5.0)
            node.write_metrics()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
