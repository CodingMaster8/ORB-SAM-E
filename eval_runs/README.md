# eval_runs/

Outputs and analysis for ORB-SAM-E evaluation runs — the **paired
baseline-vs-filtered** experiments plus the offline scripts that turn recorded
robot data into paper numbers and figures.

For the metrics *toolkit* (ATE/RPE, FPS, resources) see
[`../metrics/README.md`](../metrics/README.md). For the integration log that
produced these runs see
[`../docs/ROBOT_INTEGRATION_TODO.md`](../docs/ROBOT_INTEGRATION_TODO.md).

## What is tracked vs. ignored

To keep the repo small, only **scripts and small text results** are committed.
Heavy binaries are gitignored (see the repo `.gitignore`):

| Tracked | Ignored (local only) |
|---|---|
| Analysis scripts (`*.py`), operational scripts (`orin_scripts/*.sh`) | Raw camera rosbags (`bags/`, ~hundreds of MB) |
| Trajectories (`*.tum`, `*.txt`) and small SLAM maps (`*.ply`) | Gaussian-splat / COLMAP reconstruction (`**/gsplat/`) |
| Metrics and results (`*.json`), small pose bags (`*_posebag/`) | Videos (`*.mp4`), images/overlays (`*.png`), launch logs (`*.log`) |

If you clone fresh, the heavy inputs (bags, gsplat) will be missing; the small
tracked artifacts are enough to inspect/plot results, and the scripts let you
regenerate everything from a bag.

## Layout

```
eval_runs/
├── extract_posebag.py     # parse a /orbslam3/pose rosbag (CDR, no ROS needed) -> TUM
├── ab_analysis.py         # baseline-vs-filtered A/B analysis -> ab_results.json
├── ab/                    # the A/B experiment (bags dyn1, dyn2)
│   ├── <bag>_baseline_pose.tum / _filtered_pose.tum   # per-frame pose streams
│   ├── <bag>_odom.tum                                 # wheel odometry (reference)
│   ├── <bag>_{baseline,filtered}_traj.txt / _map.ply  # keyframe traj + SLAM map
│   ├── <bag>_{baseline,filtered}_posebag/             # raw recorded pose bag (small)
│   ├── <bag>_filtered_metrics.json                    # filter stats (latency, dets)
│   ├── ab_results.json / ab_canvas_data.json          # aggregated A/B results
│   ├── overlays/  (gitignored)                        # detection-overlay PNGs
│   └── orin_scripts/                                  # operational scripts (see below)
├── move2/                 # first live moving run + its replay + a gsplat experiment
│   ├── run_move2_{slam,odom}.tum                       # live SLAM vs odom trajectories
│   ├── replay_move2_{filtered.txt,map.ply,metrics.json}
│   └── gsplat/  (gitignored)                           # Gaussian-splat reconstruction
└── bags/  (gitignored)    # raw recorded camera rosbags (the source data)
```

### Naming convention

Within a run directory, files are prefixed by the bag/run name and the
condition: `dyn1_baseline_*` vs `dyn1_filtered_*` are the paired baseline (no
filter) and filtered (EfficientSAM3) replays of the **same** recorded bag, so
their trajectories are directly comparable against the shared `_odom.tum`.

## A/B analysis workflow

The monocular SLAM resets on heavy dynamics, so analysis works on the longest
continuously-tracked segment, aligned to odometry with Sim(3):

```bash
# 1. Pose bag -> TUM (runs on the Mac; pure-Python CDR parser, no ROS)
python3 extract_posebag.py ab/dyn1_filtered_posebag -o ab/dyn1_filtered_pose.tum

# 2. Paired A/B analysis for a bag (reads <bag>_{baseline,filtered}_pose.tum + _odom.tum)
python3 ab_analysis.py --dir ab --bags dyn1 dyn2 -o ab/ab_results.json
```

(See each script's `--help` / module docstring for exact arguments.)

## orin_scripts/

`ab/orin_scripts/` is a **snapshot of the operational shell scripts that live on
the robot/laptop** (mostly under `~` on the Orin), kept here for reference and
reproducibility. They are not AB-specific. By purpose:

- **Build:** `build_orbslam.sh`, `build_pangolin.sh`, `rebuild_orbslam.sh`,
  `rebuild2.sh`, `setup_esam3.sh`, `test_esam3.sh`.
- **Live run:** `orin_slam_start.sh`, `orin_compute_start.sh`,
  `orin_brain_start.sh`, `orin_brain_start_laptop.sh`, `live_test.sh`,
  `live_filtered.sh`, `teleop_orin.sh`, `stop_live.sh`, `smoke_tum.sh`.
- **Record / replay / A/B:** `record_move_run.sh`, `orin_record_static.sh`,
  `replay_baseline.sh`, `replay_filtered.sh`, `run_one_replay.sh`,
  `run_ab_replay.sh`, `run_ab_all.sh`, `serializar_mapa.sh`.
- **Figures / evidence:** `gen_overlays.sh`, `gen_videos.sh`, `evo_capture.sh`,
  `web_video_start.sh`.
