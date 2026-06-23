#!/usr/bin/env python3
"""
Render a rosbag2 camera stream through the EfficientSAM3 filter into a
3-panel MP4: [ original | detection overlay (red mask + boxes) | grayout ].

The right panel is exactly what the SLAM receives. To keep runtime sane on
the Orin, inference runs every --infer-every rendered frames and the latest
mask is reused in between (same behavior as the live worker-thread node).

Run inside the esam3 venv on the Orin:
  ~/venvs/esam3/bin/python3 -m efficientsam3_ros2.bag_filter_video \
      --bag ~/runs/bags/dyn1 --out ~/runs/dyn1_filter.mp4
"""
import argparse
import os
import sys
import time

import numpy as np
import cv2


def overlay_mask(bgr, mask, alpha=0.5):
    out = bgr.copy()
    if mask is None:
        return out
    mb = mask.astype(bool)
    if not mb.any():
        return out
    red = np.zeros_like(bgr)
    red[:] = (0, 0, 255)
    out[mb] = ((1.0 - alpha) * bgr[mb] + alpha * red[mb]).astype(np.uint8)
    return out


def draw_boxes(bgr, dets):
    out = bgr
    for det in dets:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.putText(out, f"{det.confidence:.2f}", (x1, max(0, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
    return out


def apply_grayout(bgr, mask):
    if mask is None:
        return bgr.copy()
    mb = mask.astype(bool)
    if not mb.any():
        return bgr.copy()
    out = bgr.copy()
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    out[mb] = np.stack([gray, gray, gray], axis=-1)[mb]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/cam_1/image")
    ap.add_argument("--out", required=True)
    ap.add_argument("--render-stride", type=int, default=2,
                    help="Render every Nth bag frame")
    ap.add_argument("--infer-every", type=int, default=10,
                    help="Run inference every Nth rendered frame")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--model-path", default=os.path.expanduser(
        "~/weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt"))
    ap.add_argument("--esam3-path", default=os.path.expanduser(
        "~/ros2_ws/src/ORB-SAM-E/efficientsam3_arm"))
    ap.add_argument("--prompts", default="person")
    ap.add_argument("--threshold", type=float, default=0.3)
    args = ap.parse_args()

    sys.path.insert(0, args.esam3_path)
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from efficientsam3_ros2.filter_core import DynamicObjectFilter, MaskingStrategy

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    print("Loading model...")
    filt = DynamicObjectFilter(
        model_path=args.model_path,
        efficientsam3_path=args.esam3_path,
        dynamic_prompts=[p.strip() for p in args.prompts.split(",")],
        confidence_threshold=args.threshold,
        masking_strategy=MaskingStrategy.GRAYOUT,
        device="cuda",
        use_fp16=True,
    )
    filt.ensure_model_loaded()
    print("Model ready")

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))

    writer = None
    mask, dets = None, []
    t0 = None
    bag_idx = 0
    rendered = 0
    n_infer = 0
    start = time.perf_counter()

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != args.topic:
            continue
        if t0 is None:
            t0 = t_ns * 1e-9
        bag_idx += 1
        if (bag_idx - 1) % args.render_stride != 0:
            continue

        msg = deserialize_message(data, Image)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            img = img.copy()

        if rendered % args.infer_every == 0:
            _, mask, dets = filt.process_frame(img)
            n_infer += 1

        det_panel = draw_boxes(overlay_mask(img, mask), dets)
        gray_panel = apply_grayout(img, mask)
        panel = np.hstack([img, det_panel, gray_panel])

        h, w = img.shape[:2]
        header_h = 30
        if writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(args.out, fourcc, args.fps,
                                     (w * 3, h + header_h))
            if not writer.isOpened():
                raise SystemExit(f"Could not open VideoWriter for {args.out}")

        rel = t_ns * 1e-9 - t0
        masked_pct = (100.0 * np.count_nonzero(mask) / mask.size) if mask is not None else 0.0
        bar = np.zeros((header_h, w * 3, 3), dtype=np.uint8)
        cv2.putText(bar, f"t={rel:6.1f}s  dets={len(dets)}  masked={masked_pct:4.1f}%"
                    f"   [ original | deteccion | grayout (entrada al SLAM) ]",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(np.vstack([bar, panel]))
        rendered += 1

        if rendered % 200 == 0:
            el = time.perf_counter() - start
            print(f"  {rendered} frames rendered, {n_infer} inferences, {el:.0f}s")

    if writer is not None:
        writer.release()
    print(f"VIDEO_DONE {args.out}: {rendered} frames, {n_infer} inferences, "
          f"{time.perf_counter() - start:.0f}s wall")


if __name__ == "__main__":
    main()
