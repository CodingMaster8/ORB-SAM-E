#!/usr/bin/env python3
"""
Live camera driver for the MonocularMode C++ node.

Replaces the dataset (EuRoC/TUM) drivers for ON-ROBOT operation: instead of
reading images from disk, it subscribes to a live camera topic (the JetAuto's
Orbbec RGB stream on /cam_1/image) and feeds the existing ORB-SLAM3 C++ node
through the same topics it already expects:

    handshake : publish <settings_name> on /mono_py_driver/experiment_settings,
                wait for "ACK" so the C++ node loads <settings_name>.yaml and
                initializes the VSLAM system.
    per frame : publish the frame timestamp on /mono_py_driver/timestep_msg and
                forward the Image on /mono_py_driver/img_msg.

Key differences vs. the dataset drivers:
  * the timestamp comes from the camera's header.stamp (real wall/sensor time),
    not a synthetic dataset counter;
  * BEST_EFFORT QoS on the camera subscription to match typical sensor drivers
    (and the C++ node's image subscription).

Params:
  -- settings_name : name of the .yaml in orb_slam3/config/Monocular/ (e.g. JetAuto)
  -- camera_topic  : input live image topic (default /cam_1/image)
  -- use_filter    : filtered-pipeline mode. Images flow camera -> dynamic_filter_node
                     -> C++ node directly, so this driver only performs the handshake
                     and does NOT republish images (the C++ node reads the timestamp
                     from the filtered image's header.stamp).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64


class LiveCameraDriver(Node):
    def __init__(self, node_name="live_camera_driver"):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter("settings_name", "JetAuto")
        self.declare_parameter("camera_topic", "/cam_1/image")
        self.declare_parameter("use_filter", False)

        self.settings_name = str(self.get_parameter("settings_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.use_filter = bool(self.get_parameter("use_filter").value)

        # Topic names expected by the C++ node (kept identical to the dataset drivers)
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"

        self.send_config = True
        self.frame_id = 0

        # Handshake publisher/subscriber
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10
        )
        self.exp_config_msg = self.settings_name

        # Real-time image QoS (match sensor drivers and the C++ node)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Output publishers (to the C++ ORB-SLAM3 node)
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, image_qos)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)

        # Live camera subscription (created after handshake completes)
        self.camera_sub_ = None

        self.get_logger().info(
            f"LiveCameraDriver: settings='{self.settings_name}', camera_topic='{self.camera_topic}'"
        )
        self.get_logger().info("Attempting handshake with C++ node ...")

    # ---- handshake ----------------------------------------------------------
    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.get_logger().info("Got ACK from C++ node, handshake complete")
            self.send_config = False

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def start_camera_subscription(self):
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.camera_sub_ = self.create_subscription(
            Image, self.camera_topic, self.camera_callback, image_qos
        )
        self.get_logger().info(f"Subscribed to live camera: {self.camera_topic}")

    # ---- per-frame forwarding ----------------------------------------------
    def camera_callback(self, img_msg):
        # Derive a monotonic-ish timestamp in seconds from the image header.
        # Fall back to the node clock if the source stamp is empty (some drivers).
        stamp = img_msg.header.stamp
        timestep = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if timestep <= 0.0:
            now = self.get_clock().now().to_msg()
            timestep = float(now.sec) + float(now.nanosec) * 1e-9
            img_msg.header.stamp = now

        self.frame_id += 1

        ts_msg = Float64()
        ts_msg.data = timestep

        # Order matters: timestep first, then image (the C++ node latches the last
        # timestep and uses it when the image callback fires).
        self.publish_timestep_msg_.publish(ts_msg)
        self.publish_img_msg_.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiveCameraDriver("live_camera_driver")

    # Blocking handshake loop
    while rclpy.ok() and node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node, timeout_sec=0.1)

    if not rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
        return

    # Filtered mode: images flow camera -> filter -> C++ node directly and the C++
    # node takes the timestamp from the image header, so after the handshake this
    # driver has nothing left to forward. Keep spinning only to stay alive for
    # launch-file lifecycle symmetry.
    if node.use_filter:
        node.get_logger().info(
            "use_filter=true: handshake done; images go camera -> filter -> SLAM "
            "directly (no republishing from this driver)"
        )
    else:
        node.start_camera_subscription()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
