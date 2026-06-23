# ORB-SAM-E Metrics Toolkit

Reproducible, paper-ready metric collection for ORB-SAM-E experiments
(ORB-SLAM3 + EfficientSAM3 dynamic-object filtering).

It captures the numbers a SLAM paper needs to compare a **baseline** (no
filtering) against the **filtered** pipeline on dynamic sequences (e.g. TUM
`fr3/walking_*`), plus the runtime/resource numbers that justify edge
deployment (e.g. Jetson Orin Nano).

## What it measures

| Category | Metrics | Where it comes from |
|---|---|---|
| **Accuracy** (headline) | ATE (RMSE/mean/median/std), RPE (trans + rot) | estimated trajectory vs `groundtruth.txt`, Umeyama alignment |
| **Runtime** | EfficientSAM3 inference latency (ms) + FPS, percentiles | timing hooks in `filter_core.py` |
| **Resources** | GPU util %, GPU mem, power, temp, CPU %, RAM | `system_monitor` (NVML/`nvidia-smi` + `psutil`) |
| **Filtering** | detections/frame, % frames with detections, masked-pixel ratio | `filter_core.py` |
| **Reproducibility** | GPU model/arch, CUDA, driver, torch, OS, CPU arch, git commit | `env_info` |

> Monocular note: ORB-SLAM3 monocular trajectories are only known **up to
> scale**, so ATE/RPE use a **Sim3 (scale-corrected) alignment** (`--mono`).
> For stereo/RGB-D (metric) trajectories, drop `--mono` to use SE3.

## Install

These are offline analysis tools, separate from the colcon build:

```bash
pip3 install -r metrics/requirements.txt
```

All deps degrade gracefully: without `pynvml`/`nvidia-smi`, GPU fields are
null; without `psutil`, CPU/RAM fields are null. Only `numpy` is required.

## Quick start (full benchmark on the cloud pod)

From the `metrics/` directory, run the pipeline once per condition. The script
launches the filter + SLAM + TUM driver, samples resources, saves the
trajectory, and evaluates ATE/RPE automatically:

```bash
SEQ=$HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz

# Baseline (no dynamic filtering)
./run_benchmark.sh fr3_walk_baseline TUM3 "$SEQ" false results/fr3_walk_baseline

# Filtered (EfficientSAM3 on GPU)
./run_benchmark.sh fr3_walk_filtered TUM3 "$SEQ" true  results/fr3_walk_filtered

# Combine into one comparison report
python3 -m slam_metrics.report \
    results/fr3_walk_baseline/results.json \
    results/fr3_walk_filtered/results.json \
    -o results/comparison.md
```

`results/comparison.md` contains a baseline-vs-filtered summary table (ATE,
RPE, FPS, GPU memory) plus per-run detail — ready to drop into a paper.

Environment overrides: `MODEL`, `ESAM3_PATH`, `DEVICE` (default `cuda`).

## Using the pieces individually

Trajectory accuracy only (after a run saved `KeyFrameTrajectory.txt`):

```bash
python3 -m slam_metrics.trajectory_eval \
    KeyFrameTrajectory.txt /path/to/sequence/groundtruth.txt \
    --rpe-delta 1.0            # add --no-scale for stereo/RGB-D
```

Resource monitor around any command:

```bash
python3 -m slam_metrics.system_monitor          # 5 s self-test
# or run for a session and stop with SIGINT:
python3 -m slam_metrics.monitor_cli --out-json resources.json --out-csv resources.csv
```

Environment snapshot:

```bash
python3 -m slam_metrics.env_info
```

## On-robot (live) capture

The benchmark above replays a dataset; on the real robot (Jetson Orin) use the
live tools instead. They record what's needed to compute ATE/RPE against ground
truth **and** to re-run the same sequence offline (baseline vs filtered).

```bash
# One-shot orchestrator: rosbag (camera+poses+GT+tf) + TUM logger + resource monitor.
# Run while the live SLAM pipeline is already up (orbsame_live.launch.py).
./record_eval_run.sh run1                 # records until Ctrl-C (or pass duration_s)

# Or the pieces directly (need the robot's CycloneDDS env exported first):
python3 -m slam_metrics.live_eval_logger --out-dir ~/eval_runs/run1   # est/gt/odom -> TUM
python3 -m slam_metrics.topic_fps --duration 30 --out fps.json \
    --filtered-topic /camera/image_filtered                            # camera/filter/SLAM FPS
```

`live_eval_logger` writes `est.tum` / `gt.tum` / `odom.tum` on a single clock,
ready for `trajectory_eval` (Sim3 for monocular) or `evo`. See
[`../docs/ROBOT_INTEGRATION_TODO.md`](../docs/ROBOT_INTEGRATION_TODO.md) for the
full robot benchmark protocol and [`../eval_runs/`](../eval_runs/) for recorded
outputs and the offline A/B analysis.

## How metric collection is wired into the pipeline

- **`filter_core.py`** times each EfficientSAM3 inference (with CUDA sync) and
  tracks detections + masked-pixel ratio; exposed via `get_stats()`.
- **`dynamic_filter_node.py`** adds a `metrics_output` ROS param — set it to a
  path and the node dumps its final stats as JSON on shutdown.
- **`ros2_orb_slam3` `mono_node_cpp`** adds a `trajectory_output` ROS param —
  set it and the node saves the keyframe trajectory (TUM format) on shutdown,
  which is what ATE/RPE are computed against.

## Modules

| Module | Purpose |
|---|---|
| `slam_metrics/trajectory_eval.py` | ATE/RPE + Umeyama alignment + TUM association (numpy only) |
| `slam_metrics/perf_timer.py` | per-stage latency/FPS accumulator |
| `slam_metrics/system_monitor.py` | background GPU/CPU/RAM sampler |
| `slam_metrics/monitor_cli.py` | run the sampler for a session → resources.json/csv |
| `slam_metrics/env_info.py` | hardware/software environment capture |
| `slam_metrics/assemble.py` | combine all parts → one `results.json` |
| `slam_metrics/report.py` | render `results.json` files → Markdown comparison |
| `slam_metrics/live_eval_logger.py` | **on-robot**: log live est/gt/odom → TUM on a single clock (for live ATE/RPE vs ground truth) |
| `slam_metrics/topic_fps.py` | **on-robot**: measure camera vs filter vs SLAM-pose FPS over a window → JSON (the three paper FPS numbers) |

## Suggested experiment matrix for the paper

Run baseline vs filtered across the dynamic and a static control sequence:

| Sequence | Why |
|---|---|
| `fr3/walking_xyz` | high dynamics — biggest expected ATE improvement from filtering |
| `fr3/walking_rpy` | high dynamics, rotation |
| `fr3/walking_halfsphere` | high dynamics |
| `fr1/desk` or `fr3/long_office_household` | (near-)static control — filtering should **not** hurt accuracy |

Report ATE RMSE (baseline vs filtered, + % improvement), inference FPS, and
GPU memory/power per device.
