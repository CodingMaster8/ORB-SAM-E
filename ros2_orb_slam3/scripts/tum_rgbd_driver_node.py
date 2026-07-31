#!/usr/bin/env python3
"""
Python driver node for the RgbdMode cpp node with TUM RGB-D dataset support (issue #16).

Reads a TUM RGB-D sequence, associates rgb.txt and depth.txt by nearest
timestamp (classic associate.py logic, max diff 0.02 s) and publishes the
paired frames. Depth is ALWAYS published raw (16UC1); in filter mode only the
RGB stream is routed through the EfficientSAM3 dynamic-object filter.

TUM RGB-D dataset format:
    sequence_name/
    |-- rgb/
    |-- depth/
    |-- rgb.txt          # timestamp filename pairs
    |-- depth.txt        # timestamp filename pairs
    `-- groundtruth.txt  # ground truth poses

Pipelines:
    direct:   tum_rgbd_driver -> (rgb + depth) -> rgbd_node_cpp
    filtered: tum_rgbd_driver -> rgb -> EfficientSAM3 filter -> rgbd_node_cpp
                             `-> depth ----------------------^

Both the RGB message and its associated depth message are stamped with the
ORIGINAL TUM rgb timestamp: the cpp node pairs the streams by header stamp
(message_filters) and uses that stamp as the SLAM frame timestamp, so the
saved trajectory lines up with groundtruth.txt for ATE/RPE evaluation.

Author: Based on tum_driver_node.py
Date: 2026
"""

import os
import time
from pathlib import Path

import cv2

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message templates
from sensor_msgs.msg import Image
from std_msgs.msg import String
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge, CvBridgeError


def read_tum_file_list(list_path):
    """Parse a TUM rgb.txt / depth.txt file.

    Returns:
        List of (timestamp: float, relative_filename: str) tuples.
    """
    entries = []
    with open(list_path, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#") or len(line) == 0:
                continue
            parts = line.split()
            if len(parts) >= 2:
                entries.append((float(parts[0]), parts[1]))
    return entries


def associate(rgb_entries, depth_entries, max_diff=0.02):
    """Associate rgb and depth entries by nearest timestamp.

    Classic TUM associate.py logic: enumerate all candidate pairs within
    max_diff, sort by |dt| and greedily accept pairs whose rgb and depth
    entries are both still unmatched.

    Returns:
        List of ((t_rgb, rgb_file), (t_depth, depth_file)) sorted by t_rgb.
    """
    potential = []
    for i, (t_rgb, _) in enumerate(rgb_entries):
        for j, (t_depth, _) in enumerate(depth_entries):
            diff = abs(t_rgb - t_depth)
            if diff < max_diff:
                potential.append((diff, i, j))
    potential.sort()

    used_rgb = set()
    used_depth = set()
    matches = []
    for _, i, j in potential:
        if i not in used_rgb and j not in used_depth:
            used_rgb.add(i)
            used_depth.add(j)
            matches.append((rgb_entries[i], depth_entries[j]))
    matches.sort(key=lambda m: m[0][0])
    return matches


def stamp_from_float(ts):
    """Convert a float unix timestamp to a builtin_interfaces/Time message."""
    msg = TimeMsg()
    msg.sec = int(ts)
    msg.nanosec = int(round((ts - msg.sec) * 1e9))
    if msg.nanosec >= 1_000_000_000:  # guard against rounding overflow
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


class TUMRgbdDatasetDriver(Node):
    """ROS2 node that reads TUM RGB-D sequences and publishes rgb + depth pairs.

    Supports direct mode (straight to rgbd_node_cpp) and filtered mode (RGB is
    routed through the EfficientSAM3 filter; depth always goes raw).
    """

    def __init__(self, node_name="tum_rgbd_driver_node"):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("settings_name", "TUM3")  # yaml in orb_slam3/config/RGB-D/
        self.declare_parameter("dataset_path", "")       # Full path to sequence folder
        self.declare_parameter("use_filter", False)      # Route RGB through EfficientSAM3
        self.declare_parameter("frame_rate", 30.0)       # Fallback pacing when timestamps are unusable
        self.declare_parameter("playback_speed", 1.0)    # 1.0 = real time, 2.0 = twice as fast
        self.declare_parameter("start_frame", 0)
        self.declare_parameter("end_frame", -1)          # -1 means all frames
        self.declare_parameter("max_time_diff", 0.02)    # associate.py tolerance (seconds)

        # Parse parameters
        self.settings_name = str(self.get_parameter("settings_name").value)
        self.dataset_path = str(self.get_parameter("dataset_path").value)
        self.use_filter = bool(self.get_parameter("use_filter").value)
        self.frame_rate = float(self.get_parameter("frame_rate").value)
        self.playback_speed = float(self.get_parameter("playback_speed").value)
        self.start_frame = int(self.get_parameter("start_frame").value)
        self.end_frame = int(self.get_parameter("end_frame").value)
        self.max_time_diff = float(self.get_parameter("max_time_diff").value)

        # Validate dataset path
        if not self.dataset_path:
            self.dataset_path = str(Path.home()) + "/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/tum"

        # Print configuration
        self.get_logger().info("=" * 60)
        self.get_logger().info("TUM RGB-D Dataset Driver Configuration")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Settings: {self.settings_name}")
        self.get_logger().info(f"Dataset path: {self.dataset_path}")
        self.get_logger().info(f"Use filter: {self.use_filter}")
        self.get_logger().info(f"Playback speed: {self.playback_speed}")
        self.get_logger().info(f"Association max diff: {self.max_time_diff}s")
        self.get_logger().info("=" * 60)

        # CvBridge for image conversion
        self.br = CvBridge()

        # Load and associate the dataset
        self.pairs = self.load_tum_rgbd_dataset(self.dataset_path)
        if len(self.pairs) == 0:
            self.get_logger().error(f"No associated rgb/depth pairs found in {self.dataset_path}")
            self.get_logger().error("Expected TUM format with rgb.txt, depth.txt, rgb/ and depth/ folders")
            raise RuntimeError("Dataset not found or empty")
        self.get_logger().info(f"Associated {len(self.pairs)} rgb/depth pairs")

        # ROS2 topic names (handshake mirrors the mono drivers, rgbd namespace)
        self.pub_exp_config_name = "/rgbd_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/rgbd_py_driver/exp_settings_ack"

        if self.use_filter:
            # RGB goes through the EfficientSAM3 filter; the cpp node subscribes
            # to the filter output (/camera/image_filtered).
            self.pub_rgb_name = "/camera/image_raw"
            self.get_logger().info(f"FILTER MODE: RGB -> {self.pub_rgb_name} (filter input)")
        else:
            self.pub_rgb_name = "/rgbd_py_driver/rgb_msg"
            self.get_logger().info(f"DIRECT MODE: RGB -> {self.pub_rgb_name}")

        # Depth is NEVER filtered: gray-out would corrupt the depth values.
        self.pub_depth_name = "/rgbd_py_driver/depth_msg"
        self.get_logger().info(f"Depth -> {self.pub_depth_name} (always raw)")

        # State
        self.send_config = True
        self.frame_id = 0

        # QoS profile: BEST_EFFORT like the mono drivers, but a deeper queue so
        # the cpp synchronizer has room to pair the two streams.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers and Subscribers
        self.pub_config = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.sub_ack = self.create_subscription(String, self.sub_exp_ack_name, self.ack_callback, 10)
        self.pub_rgb = self.create_publisher(Image, self.pub_rgb_name, image_qos)
        self.pub_depth = self.create_publisher(Image, self.pub_depth_name, image_qos)

        self.get_logger().info("TUM RGB-D driver initialized, attempting handshake...")

    def load_tum_rgbd_dataset(self, dataset_path):
        """Read rgb.txt/depth.txt and associate them by nearest timestamp.

        Returns:
            List of (timestamp, rgb_full_path, depth_full_path) sorted by time.
        """
        rgb_txt = os.path.join(dataset_path, "rgb.txt")
        depth_txt = os.path.join(dataset_path, "depth.txt")

        if not os.path.exists(rgb_txt) or not os.path.exists(depth_txt):
            self.get_logger().error(f"Missing rgb.txt or depth.txt in {dataset_path}")
            return []

        rgb_entries = read_tum_file_list(rgb_txt)
        depth_entries = read_tum_file_list(depth_txt)
        self.get_logger().info(
            f"Read {len(rgb_entries)} rgb and {len(depth_entries)} depth entries")

        matches = associate(rgb_entries, depth_entries, self.max_time_diff)

        pairs = []
        missing = 0
        for (t_rgb, rgb_file), (_t_depth, depth_file) in matches:
            rgb_path = os.path.join(dataset_path, rgb_file)
            depth_path = os.path.join(dataset_path, depth_file)
            if os.path.exists(rgb_path) and os.path.exists(depth_path):
                pairs.append((t_rgb, rgb_path, depth_path))
            else:
                missing += 1
        if missing:
            self.get_logger().warning(f"{missing} associated pairs had missing image files")
        return pairs

    def ack_callback(self, msg):
        """Handle acknowledgement from the ORB-SLAM3 RGB-D node."""
        self.get_logger().info(f"Received ACK: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake(self):
        """Send configuration (settings yaml name) to the ORB-SLAM3 RGB-D node."""
        if self.send_config:
            msg = String()
            msg.data = self.settings_name  # e.g. "TUM1", "TUM2", "TUM3"
            self.pub_config.publish(msg)
            time.sleep(0.01)

    def publish_pair(self, idx):
        """Publish one associated rgb/depth pair. Returns False past the end."""
        if idx >= len(self.pairs):
            return False

        timestamp, rgb_path, depth_path = self.pairs[idx]

        rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)  # BGR uint8
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # uint16, TUM units
        if rgb is None or depth is None:
            self.get_logger().warning(f"Failed to read pair: {rgb_path} / {depth_path}")
            return True  # Continue to next frame

        try:
            stamp = stamp_from_float(timestamp)

            rgb_msg = self.br.cv2_to_imgmsg(rgb, encoding="bgr8")
            # The ORIGINAL TUM timestamp: required for stream pairing in the cpp
            # node and for groundtruth alignment of the saved trajectory.
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = f"camera_frame_{self.frame_id}"

            # Depth stays in native 16UC1 TUM units; ORB-SLAM3 applies
            # RGBD.DepthMapFactor from the settings yaml. The depth message is
            # stamped with the SAME timestamp as its associated rgb frame so
            # the synchronizer pairs them exactly.
            depth_msg = self.br.cv2_to_imgmsg(depth, encoding="16UC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = rgb_msg.header.frame_id

            self.pub_depth.publish(depth_msg)
            self.pub_rgb.publish(rgb_msg)

            self.frame_id += 1
            if self.frame_id % 100 == 0:
                self.get_logger().info(f"Published pair {self.frame_id}/{len(self.pairs)}")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")

        return True


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TUMRgbdDatasetDriver("tum_rgbd_driver_node")
    except RuntimeError as e:
        print(f"Error: {e}")
        rclpy.shutdown()
        return

    # Handshake with the ORB-SLAM3 RGB-D node
    while node.send_config:
        node.handshake()
        rclpy.spin_once(node)
        if not node.send_config:
            break

    node.get_logger().info("Handshake complete!")

    if node.use_filter:
        node.get_logger().info("=" * 60)
        node.get_logger().info("IMPORTANT: Make sure the EfficientSAM3 filter node is running!")
        node.get_logger().info("  (only the RGB stream is filtered; depth is published raw)")
        node.get_logger().info("=" * 60)
        time.sleep(1)

    # Determine frame range
    start = node.start_frame
    end = node.end_frame if node.end_frame > 0 else len(node.pairs)
    end = min(end, len(node.pairs))

    node.get_logger().info(f"Playing pairs {start} to {end}")

    # Main loop: publish pairs paced by the ORIGINAL dataset timestamps
    # (divided by playback_speed). Falls back to frame_rate pacing when the
    # timestamp delta is degenerate.
    fallback_dt = 1.0 / node.frame_rate if node.frame_rate > 0 else 0.033
    for idx in range(start, end):
        try:
            t0 = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            if not node.publish_pair(idx):
                break

            if idx + 1 < end:
                dt = node.pairs[idx + 1][0] - node.pairs[idx][0]
                if not (0.0 < dt < 1.0):  # gaps / bad stamps -> fixed rate
                    dt = fallback_dt
                dt /= max(node.playback_speed, 1e-6)
                remaining = dt - (time.monotonic() - t0)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            break

    node.get_logger().info("Dataset playback complete")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
