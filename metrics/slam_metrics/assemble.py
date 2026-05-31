"""
Assemble the pieces of one experiment run into a single results.json.

Combines (all optional):
    --config       JSON file or inline key=value pairs describing the run
    --filter-stats filter node metrics JSON (from metrics_output param)
    --resources    resources.json (from monitor_cli)
    --estimate     estimated trajectory (TUM) + --groundtruth -> ATE/RPE
    environment is captured automatically (env_info.collect()).

Produces a results.json that report.py can render.

Example:
    python -m slam_metrics.assemble \
        --name fr3_walking_xyz_filtered \
        --config use_filter=true model=repvit_s sequence=fr3/walking_xyz device=cuda \
        --filter-stats filter_metrics.json \
        --resources resources.json \
        --estimate KeyFrameTrajectory.txt --groundtruth groundtruth.txt \
        --mono \
        --out results.json
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Any, Dict

from . import env_info
from . import trajectory_eval as te
from .report import ExperimentRecord


def _parse_config(items) -> Dict[str, Any]:
    cfg: Dict[str, Any] = {}
    if not items:
        return cfg
    # Single JSON file?
    if len(items) == 1 and os.path.isfile(items[0]):
        with open(items[0]) as f:
            return json.load(f)
    for it in items:
        if "=" not in it:
            continue
        k, v = it.split("=", 1)
        low = v.lower()
        if low in ("true", "false"):
            cfg[k] = (low == "true")
        else:
            try:
                cfg[k] = int(v)
            except ValueError:
                try:
                    cfg[k] = float(v)
                except ValueError:
                    cfg[k] = v
    return cfg


def _load_json(path):
    if not path:
        return None
    with open(path) as f:
        return json.load(f)


def main() -> None:
    p = argparse.ArgumentParser(description="Assemble one experiment results.json")
    p.add_argument("--name", required=True)
    p.add_argument("--config", nargs="*", help="JSON file OR key=value pairs")
    p.add_argument("--filter-stats", help="Filter node metrics JSON")
    p.add_argument("--resources", help="resources.json from monitor_cli")
    p.add_argument("--estimate", help="Estimated trajectory (TUM)")
    p.add_argument("--groundtruth", help="Groundtruth trajectory (TUM)")
    p.add_argument("--mono", action="store_true",
                   help="Monocular: align with scale (Sim3). Omit for metric trajectories.")
    p.add_argument("--max-diff", type=float, default=0.02)
    p.add_argument("--rpe-delta", type=float, default=1.0)
    p.add_argument("--out", default="results.json")
    args = p.parse_args()

    config = _parse_config(args.config)

    filter_stats = _load_json(args.filter_stats) or {}
    # Map the filter node's flat timing into the registry-style timing table.
    timing: Dict[str, Any] = {}
    if filter_stats:
        timing["efficientsam3_inference"] = {
            "name": "efficientsam3_inference",
            "count": filter_stats.get("inference_count", 0),
            "fps": filter_stats.get("inference_fps", 0.0),
            "mean_ms": filter_stats.get("inference_mean_ms", 0.0),
            "median_ms": filter_stats.get("inference_median_ms", 0.0),
            "p95_ms": filter_stats.get("inference_p95_ms", 0.0),
            "min_ms": filter_stats.get("inference_min_ms", 0.0),
            "max_ms": filter_stats.get("inference_max_ms", 0.0),
        }

    filtering = {}
    for k in ("total_frames_processed", "total_detections",
              "avg_detections_per_frame", "frames_with_detections",
              "detection_rate", "avg_masked_pixel_ratio", "device"):
        if k in filter_stats:
            filtering[k] = filter_stats[k]

    trajectory = None
    if args.estimate and args.groundtruth:
        if not os.path.isfile(args.estimate):
            print(f"WARNING: estimate not found: {args.estimate}")
        elif not os.path.isfile(args.groundtruth):
            print(f"WARNING: groundtruth not found: {args.groundtruth}")
        else:
            res = te.evaluate(
                est_path=args.estimate,
                gt_path=args.groundtruth,
                align_with_scale=args.mono,
                max_difference=args.max_diff,
                rpe_delta=args.rpe_delta,
            )
            trajectory = res.to_dict()

    rec = ExperimentRecord(
        name=args.name,
        config=config,
        env=env_info.collect(),
        timing=timing,
        resources=_load_json(args.resources) or {},
        trajectory=trajectory,
        filtering=filtering,
    )
    rec.save_json(args.out)
    print(f"Wrote {args.out}")
    if trajectory and trajectory.get("ate"):
        print(f"  ATE RMSE: {trajectory['ate']['rmse']:.4f} m "
              f"(scale={trajectory['scale']:.4f}, "
              f"assoc={trajectory['num_associations']})")


if __name__ == "__main__":
    main()
