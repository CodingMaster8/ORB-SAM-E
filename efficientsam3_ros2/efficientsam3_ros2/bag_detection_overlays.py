#!/usr/bin/env python3
"""
Extract frames from a rosbag2 and run the EfficientSAM3 dynamic filter on them,
saving side-by-side overlays (original | mask | filtered) as PNGs.

Qualitative evidence for the paper: shows whether walking persons in a recorded
robot run are actually detected/masked by the filter.

Run inside the esam3 venv on the Orin:
  ~/venvs/esam3/bin/python3 -m efficientsam3_ros2.bag_detection_overlays \
      --bag ~/runs/bags/move2 --out ~/runs/move2_overlays \
      --start 80 --end 200 --every 60
"""
import argparse
import os
import sys

import numpy as np
import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to rosbag2 directory")
    ap.add_argument("--topic", default="/cam_1/image")
    ap.add_argument("--out", required=True, help="Output directory for PNGs")
    ap.add_argument("--start", type=float, default=0.0, help="Seconds from bag start")
    ap.add_argument("--end", type=float, default=1e12, help="Seconds from bag start")
    ap.add_argument("--every", type=int, default=60, help="Process every Nth frame in window")
    ap.add_argument("--model-path", default=os.path.expanduser(
        "~/weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt"))
    ap.add_argument("--esam3-path", default=os.path.expanduser(
        "~/ros2_ws/src/ORB-SAM-E/efficientsam3_arm"))
    ap.add_argument("--prompts", default="person")
    ap.add_argument("--threshold", type=float, default=0.3)
    args = ap.parse_args()

    sys.path.insert(0, args.esam3_path)
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from efficientsam3_ros2.filter_core import DynamicObjectFilter

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    filt = DynamicObjectFilter(
        model_path=args.model_path,
        efficientsam3_path=args.esam3_path,
        dynamic_prompts=[p.strip() for p in args.prompts.split(",")],
        confidence_threshold=args.threshold,
        device="cuda",
        use_fp16=True,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))

    os.makedirs(args.out, exist_ok=True)
    t0 = None
    idx = 0
    saved = 0
    summary = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != args.topic:
            continue
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel < args.start or rel > args.end:
            continue
        idx += 1
        if (idx - 1) % args.every != 0:
            continue

        msg = deserialize_message(data, Image)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("rgb8",):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = np.ascontiguousarray(img[:, :, :3])

        filtered, mask, detections = filt.process_frame(img)
        n = len(detections)
        confs = [f"{d.confidence:.2f}" for d in detections]
        summary.append((rel, n, confs))

        overlay = img.copy()
        overlay[mask > 0] = (0.4 * overlay[mask > 0] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
        for d in detections:
            x1, y1, x2, y2 = map(int, d.bbox)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(overlay, f"{d.confidence:.2f}", (x1, max(15, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        panel = np.hstack([img, overlay, filtered])
        cv2.putText(panel, f"t={rel:.1f}s  dets={n}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        out_path = os.path.join(args.out, f"frame_t{rel:06.1f}s_d{n}.png")
        cv2.imwrite(out_path, panel)
        saved += 1
        print(f"t={rel:6.1f}s  detections={n}  {confs}")

    n_frames = len(summary)
    n_with = sum(1 for _, n, _ in summary if n > 0)
    print(f"\nProcessed {n_frames} frames, {n_with} with detections "
          f"({100.0 * n_with / max(1, n_frames):.0f}%), saved {saved} panels to {args.out}")


if __name__ == "__main__":
    main()
