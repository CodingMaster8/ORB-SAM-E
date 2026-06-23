#!/usr/bin/env python3
"""
live_eval_logger — capture ON-ROBOT trajectories in TUM format for ATE/RPE.

Logs, with a SINGLE clock (this node's arrival time, so association across
machines is clean — same pattern as jetauto_bridge/tools/evo/evo_logger.py):

  est.tum    estimated pose  = /orbslam3/pose   (geometry_msgs/PoseStamped)
  gt.tum     ground truth    = /optitrack/rigid_body (PoseStamped, sensor QoS)
  odom.tum   EKF odometry    = /odom            (nav_msgs/Odometry, comparison)
  summary.txt  message counts + ready-to-run evo / slam_metrics commands

Run it on the Orin (or laptop) WITH the bridge DDS env:
    export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 -m slam_metrics.live_eval_logger --out-dir ~/eval_runs/run1

Stop with Ctrl-C. Then evaluate (monocular -> Sim3 alignment with scale):
    python3 -m slam_metrics.trajectory_eval est.tum gt.tum --rpe-delta 1.0
    # or with evo (laptop):  evo_ape tum gt.tum est.tum -as
"""

from __future__ import annotations

import argparse
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def _tum_line(t: float, p, q) -> str:
    return (f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")


class LiveEvalLogger(Node):
    def __init__(self, args):
        super().__init__("live_eval_logger")
        self.out_dir = os.path.expanduser(args.out_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        self._files = {
            "est": open(os.path.join(self.out_dir, "est.tum"), "w"),
            "gt": open(os.path.join(self.out_dir, "gt.tum"), "w"),
            "odom": open(os.path.join(self.out_dir, "odom.tum"), "w"),
        }
        self._counts = {k: 0 for k in self._files}

        self.create_subscription(PoseStamped, args.est_topic, self._est_cb, 10)
        self.create_subscription(PoseStamped, args.gt_topic, self._gt_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 20)

        self._report_timer = self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"Logging est={args.est_topic} gt={args.gt_topic} odom={args.odom_topic} "
            f"-> {self.out_dir}")

    def _now(self) -> float:
        # Single clock for all streams (association-friendly across machines).
        return self.get_clock().now().nanoseconds * 1e-9

    def _write(self, key: str, pose) -> None:
        self._files[key].write(_tum_line(self._now(), pose.position, pose.orientation))
        self._counts[key] += 1

    def _est_cb(self, msg: PoseStamped):
        self._write("est", msg.pose)

    def _gt_cb(self, msg: PoseStamped):
        self._write("gt", msg.pose)

    def _odom_cb(self, msg: Odometry):
        self._write("odom", msg.pose.pose)

    def _report(self):
        c = self._counts
        self.get_logger().info(f"est={c['est']} gt={c['gt']} odom={c['odom']}")

    def close(self):
        for f in self._files.values():
            f.flush()
            f.close()
        with open(os.path.join(self.out_dir, "summary.txt"), "w") as s:
            s.write("live_eval_logger summary\n")
            for k, v in self._counts.items():
                s.write(f"{k}: {v} samples\n")
            s.write(
                "\nEvaluate (monocular -> scale-corrected alignment):\n"
                "  python3 -m slam_metrics.trajectory_eval est.tum gt.tum --rpe-delta 1.0\n"
                "  evo_ape tum gt.tum est.tum -as   # laptop\n")
        print(f"\nSaved TUM trajectories to {self.out_dir}")
        for k, v in self._counts.items():
            print(f"  {k}: {v} samples")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default="~/eval_runs/run")
    ap.add_argument("--est-topic", default="/orbslam3/pose")
    ap.add_argument("--gt-topic", default="/optitrack/rigid_body")
    ap.add_argument("--odom-topic", default="/odom")
    args = ap.parse_args()

    rclpy.init()
    node = LiveEvalLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
