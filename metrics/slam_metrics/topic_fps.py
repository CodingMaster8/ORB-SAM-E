#!/usr/bin/env python3
"""
topic_fps — measure ON-ROBOT pipeline rates over a fixed window.

Samples the camera input and the SLAM pose output simultaneously and reports
the effective tracking FPS plus the input->output drop ratio. With the
EfficientSAM3 filter in the loop, also samples the filtered image topic, so a
single run yields the three paper numbers (camera FPS, filter FPS, SLAM FPS).

Usage (Orin, with the bridge DDS env):
    python3 -m slam_metrics.topic_fps --duration 30 --out fps.json
    # add --filtered-topic /camera/image_filtered when the filter runs
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


class TopicFps(Node):
    def __init__(self, args):
        super().__init__("topic_fps")
        self.counts = {"camera": 0, "pose": 0, "filtered": 0}
        self.stamps = {"camera": [], "pose": [], "filtered": []}

        self.create_subscription(Image, args.camera_topic,
                                 lambda m: self._tick("camera"), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, args.pose_topic,
                                 lambda m: self._tick("pose"), 10)
        if args.filtered_topic:
            self.create_subscription(Image, args.filtered_topic,
                                     lambda m: self._tick("filtered"), qos_profile_sensor_data)
        self.filtered_enabled = bool(args.filtered_topic)

    def _tick(self, key: str):
        self.counts[key] += 1
        self.stamps[key].append(time.monotonic())

    def stats(self, duration: float) -> dict:
        def rate_stats(key: str) -> dict:
            ts = self.stamps[key]
            out = {"count": self.counts[key], "fps": self.counts[key] / duration}
            if len(ts) >= 3:
                deltas = [b - a for a, b in zip(ts, ts[1:])]
                deltas.sort()
                out["period_ms_p50"] = deltas[len(deltas) // 2] * 1e3
                out["period_ms_p95"] = deltas[int(len(deltas) * 0.95)] * 1e3
            return out

        res = {
            "duration_s": duration,
            "camera": rate_stats("camera"),
            "slam_pose": rate_stats("pose"),
        }
        if self.filtered_enabled:
            res["filtered"] = rate_stats("filtered")
        if self.counts["camera"]:
            res["slam_drop_ratio"] = 1.0 - self.counts["pose"] / self.counts["camera"]
        return res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=30.0, help="seconds to sample")
    ap.add_argument("--camera-topic", default="/cam_1/image")
    ap.add_argument("--pose-topic", default="/orbslam3/pose")
    ap.add_argument("--filtered-topic", default="",
                    help="set to /camera/image_filtered when the filter is running")
    ap.add_argument("--out", default="", help="optional JSON output path")
    args = ap.parse_args()

    rclpy.init()
    node = TopicFps(args)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    duration = time.monotonic() - t0

    res = node.stats(duration)
    print(json.dumps(res, indent=2))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, indent=2)
        print(f"saved -> {args.out}")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
