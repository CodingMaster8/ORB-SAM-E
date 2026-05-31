"""
Render a dataset of RGB frames through the EfficientSAM3 DynamicObjectFilter
and write an annotated MP4 so the dynamic-object masks can be shared as a video.

Each output frame is a side-by-side panel:
    [ original | original + red mask overlay + detection boxes ]
with a header showing the frame index, detection count, and masked-pixel %.

Example (on the pod):
    cd $WORK/ros2_ws/src/ORB-SAM-E/efficientsam3_ros2/efficientsam3_ros2
    python3 make_filter_video.py \
        --frames $WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz/rgb \
        --model  $HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
        --efficientsam3-path $WORK/ros2_ws/src/ORB-SAM-E/efficientsam3_arm \
        --output $HOME/walking_xyz_masks.mp4 \
        --device cuda --fps 10

Then copy it back to your Mac:
    scp -P <port> -i ~/dev/personal/ros_pod root@<host>:$HOME/walking_xyz_masks.mp4 .
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

from filter_core import DynamicObjectFilter, MaskingStrategy


def _list_frames(frames_dir: str) -> list:
    exts = ("*.png", "*.jpg", "*.jpeg")
    files = []
    for ext in exts:
        files.extend(glob.glob(os.path.join(frames_dir, ext)))
    # TUM filenames are timestamps, so lexicographic sort == temporal order.
    return sorted(files)


def _overlay_mask(bgr: np.ndarray, mask: np.ndarray, alpha: float = 0.5) -> np.ndarray:
    """Blend a red overlay onto the regions where mask is set."""
    out = bgr.copy()
    if mask is None:
        return out
    mask_bool = mask.astype(bool)
    if not mask_bool.any():
        return out
    red = np.zeros_like(bgr)
    red[:] = (0, 0, 255)  # BGR red
    out[mask_bool] = (
        (1.0 - alpha) * bgr[mask_bool] + alpha * red[mask_bool]
    ).astype(np.uint8)
    return out


def _draw_boxes(bgr: np.ndarray, detections) -> np.ndarray:
    out = bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.putText(
            out,
            f"{det.confidence:.2f}",
            (x1, max(0, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    return out


def _header(width: int, text: str, height: int = 28) -> np.ndarray:
    bar = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        bar,
        text,
        (8, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return bar


def main() -> None:
    parser = argparse.ArgumentParser(description="Render EfficientSAM3 masks to a video")
    parser.add_argument("--frames", required=True, help="Directory of RGB frames")
    parser.add_argument("--model", required=True, help="Path to EfficientSAM3 checkpoint")
    parser.add_argument("--efficientsam3-path", default=None, help="Path to efficientsam3_arm package")
    parser.add_argument("--output", default="filter_masks.mp4", help="Output MP4 path")
    parser.add_argument("--device", default="cuda", help="cpu | cuda | mps | auto")
    parser.add_argument("--fps", type=float, default=10.0, help="Playback FPS of the output video")
    parser.add_argument("--confidence", type=float, default=0.03, help="Detection confidence threshold")
    parser.add_argument("--alpha", type=float, default=0.5, help="Mask overlay opacity (0-1)")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit number of frames (0 = all)")
    parser.add_argument("--stride", type=int, default=1, help="Process every Nth frame")
    parser.add_argument("--skip-empty", action="store_true", help="Only keep frames that have detections")
    parser.add_argument(
        "--prompts",
        default=None,
        help="Comma-separated prompts (default: filter's body-part prompts)",
    )
    args = parser.parse_args()

    frames = _list_frames(args.frames)
    if args.stride > 1:
        frames = frames[:: args.stride]
    if args.max_frames > 0:
        frames = frames[: args.max_frames]
    if not frames:
        raise SystemExit(f"No frames found in {args.frames}")

    dynamic_prompts = None
    if args.prompts:
        dynamic_prompts = [p.strip() for p in args.prompts.split(",") if p.strip()]

    print(f"Found {len(frames)} frames. Loading model on {args.device}...")
    filt = DynamicObjectFilter(
        model_path=args.model,
        efficientsam3_path=args.efficientsam3_path,
        dynamic_prompts=dynamic_prompts,
        confidence_threshold=args.confidence,
        masking_strategy=MaskingStrategy.GRAYOUT,
        device=args.device,
    )
    filt.ensure_model_loaded()

    # Probe the first frame to fix the output size.
    sample = cv2.imread(frames[0])
    if sample is None:
        raise SystemExit(f"Failed to read {frames[0]}")
    h, w = sample.shape[:2]
    header_h = 28
    out_w, out_h = w * 2, h + header_h

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(args.output, fourcc, args.fps, (out_w, out_h))
    if not writer.isOpened():
        raise SystemExit(f"Could not open VideoWriter for {args.output}")

    written = 0
    t0 = time.perf_counter()
    for i, fp in enumerate(frames):
        img = cv2.imread(fp)
        if img is None:
            print(f"  [skip] unreadable: {fp}")
            continue
        if img.shape[:2] != (h, w):
            img = cv2.resize(img, (w, h))

        _, mask, dets = filt.process_frame(img)

        if args.skip_empty and not dets:
            continue

        masked_pct = (
            float(np.count_nonzero(mask)) / mask.size * 100.0 if mask is not None else 0.0
        )
        right = _overlay_mask(img, mask, alpha=args.alpha)
        right = _draw_boxes(right, dets)
        panel = np.hstack([img, right])

        header = _header(
            out_w,
            f"frame {i + 1}/{len(frames)}   dets={len(dets)}   masked={masked_pct:4.1f}%   "
            f"[left: original | right: dynamic mask]",
            header_h,
        )
        frame_out = np.vstack([header, panel])
        writer.write(frame_out)
        written += 1

        if (i + 1) % 25 == 0:
            elapsed = time.perf_counter() - t0
            print(f"  processed {i + 1}/{len(frames)} frames ({elapsed:.1f}s)")

    writer.release()
    stats = filt.get_stats()
    dt = time.perf_counter() - t0
    print(
        f"\nDone. Wrote {written} frames to {args.output}\n"
        f"  detection_rate={stats['detection_rate'] * 100:.1f}%  "
        f"avg_masked={stats['avg_masked_pixel_ratio'] * 100:.1f}%  "
        f"inference_fps={stats['inference_fps']:.2f}  wall={dt:.1f}s"
    )


if __name__ == "__main__":
    main()
