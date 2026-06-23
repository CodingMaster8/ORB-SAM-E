# ORB-SAM-E

**Dynamic-object–aware monocular SLAM for the edge.** ORB-SAM-E combines
[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with
[EfficientSAM3](https://github.com/SimonZeng7108/efficientsam3) so that moving
objects (people, vehicles, …) are segmented out of the camera stream *before*
they reach the SLAM tracker, improving pose accuracy in dynamic scenes. The
target deployment is a **Jetson Orin Nano** on a mobile robot (JetAuto).

```
┌─────────────────┐   /cam_1/image    ┌──────────────────────────┐   /camera/image_filtered   ┌──────────────────────────┐
│  Camera driver  │ ─────────────────▶│  EfficientSAM3 filter    │ ──────────────────────────▶│   ORB-SLAM3 node (C++)   │
│   (live / bag)  │  sensor_msgs/Image│   node (Python, GPU)     │      sensor_msgs/Image      │  pose + map (orb_map →   │
└─────────────────┘                   │  masks dynamic objects   │                             │  orb_cam frames)         │
                                      └──────────────────────────┘                             └──────────────────────────┘
```

The ORB-SLAM3 node runs unfiltered images (baseline) or filtered images
(`use_filtered_images:=true`), which makes paired **baseline vs. filtered**
A/B evaluation the core experiment of the project.

> **Status (Jun 2026):** the full pipeline runs live on a Jetson Orin Nano
> (camera → filter → SLAM) and a first round of paired A/B runs is complete.
> See [`docs/ROBOT_INTEGRATION_TODO.md`](docs/ROBOT_INTEGRATION_TODO.md) for the
> live integration log and open items, and
> [`docs/PAPER_DL_FINDINGS.md`](docs/PAPER_DL_FINDINGS.md) for the measured
> deep-learning results.

---

## Repository layout

| Path | What it is |
|---|---|
| [`ros2_orb_slam3/`](ros2_orb_slam3/) | ROS 2 wrapper around ORB-SLAM3 (C++ `mono_node_cpp`), thirdparty (DBoW2, g2o, Sophus), camera/dataset drivers, configs and launch files. |
| [`efficientsam3_ros2/`](efficientsam3_ros2/) | ROS 2 package with the EfficientSAM3 **dynamic-object filter node** and offline bag tools. See its [README](efficientsam3_ros2/README.md). |
| [`efficientsam3_arm/`](efficientsam3_arm/) | The EfficientSAM3 model code (ARM/edge port) the filter imports. Injected via `sys.path` (`efficientsam3_path`), not pip-installed on the robot. |
| [`metrics/`](metrics/) | Offline, paper-ready metrics toolkit: ATE/RPE accuracy, FPS/latency, GPU/CPU/RAM, environment capture. See its [README](metrics/README.md). |
| [`eval_runs/`](eval_runs/) | Experiment outputs (trajectories, maps, metrics) and the offline A/B analysis scripts. Heavy binaries are gitignored — see its [README](eval_runs/README.md). |
| [`media/`](media/) | Shareable figures/videos/PDF for talks and the paper. Binaries gitignored — see its [README](media/README.md). |
| [`docs/`](docs/) | All long-form guides and research notes (see the index below). |
| `paper/` | LaTeX sources for the validation paper. **Gitignored** (built locally). |

### Key components

- **`ros2_orb_slam3/scripts/live_camera_driver_node.py`** — feeds the live robot
  camera (`/cam_1/image`) into ORB-SLAM3, performing the settings handshake.
- **`ros2_orb_slam3/launch/orbsame_live.launch.py`** — on-robot launch for the
  full pipeline (SLAM + camera driver + optional filter). Headless by default.
- **`ros2_orb_slam3/orb_slam3/config/Monocular/JetAuto.yaml`** — camera
  intrinsics for the JetAuto robot (Orbbec Astra, factory calibration).
- **`efficientsam3_ros2/efficientsam3_ros2/dynamic_filter_node.py`** — the ROS
  filter node (worker-thread inference, fp16, configurable prompts/threshold).
- **`efficientsam3_ros2/efficientsam3_ros2/bag_detection_overlays.py`** /
  **`bag_filter_video.py`** — offline tools to render detection overlays / a
  3-panel comparison video from a recorded rosbag (qualitative evidence).
- **`resume_setup.sh`** — rebuilds a RunPod GPU pod's environment after a Stop
  wipes the container disk (see [`docs/CLOUD_GPU_TESTING.md`](docs/CLOUD_GPU_TESTING.md)).

---

## Documentation index

All guides live in [`docs/`](docs/). Start with setup, then deployment, then
the research notes. Some research notes are written in Spanish; their titles
below indicate the language.

### Setup & build
- [`docs/UBUNTU_SETUP.md`](docs/UBUNTU_SETUP.md) — full Ubuntu + ROS 2 setup,
  ORB-SLAM3 build, weights, and first run (the canonical install guide).
- [`efficientsam3_ros2/VM_SETUP.md`](efficientsam3_ros2/VM_SETUP.md) — quicker
  setup focused on the filter package.

### Running on GPU hardware
- [`docs/CLOUD_GPU_TESTING.md`](docs/CLOUD_GPU_TESTING.md) — which cloud GPU to
  rent (and why) to validate the CUDA pipeline; includes a hard-won "gotchas"
  section and a results template.
- [`docs/POD_RUNBOOK.md`](docs/POD_RUNBOOK.md) — copy-paste runbook from a fresh
  RunPod GPU pod to recorded benchmark numbers.

### Robot deployment & research findings
- [`docs/ROBOT_INTEGRATION_TODO.md`](docs/ROBOT_INTEGRATION_TODO.md) *(ES)* —
  the live Jetson/JetAuto integration log: phases done, A/B results, open items.
- [`docs/PAPER_DL_FINDINGS.md`](docs/PAPER_DL_FINDINGS.md) *(ES)* — deep-learning
  findings measured on the real robot (checkpoint pitfalls, latency anatomy,
  the measured performance ladder, recommended config).
- [`docs/FPS_IMPROVEMENT_PLAN.md`](docs/FPS_IMPROVEMENT_PLAN.md) — the filter
  throughput cost model and the prioritized list of speedups.

### Evaluation
- [`metrics/README.md`](metrics/README.md) — the metrics toolkit (ATE/RPE, FPS,
  resources) and the suggested experiment matrix for the paper.
- [`eval_runs/README.md`](eval_runs/README.md) — layout of recorded runs and the
  offline A/B analysis scripts.

---

## Quick start

The detailed, step-by-step path is [`docs/UBUNTU_SETUP.md`](docs/UBUNTU_SETUP.md).
In short, on an Ubuntu machine with a CUDA GPU:

```bash
# 1. Build the ROS 2 workspace (after installing ROS 2 + ORB-SLAM3 deps; see UBUNTU_SETUP.md)
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 2. Run the full pipeline (baseline or filtered) — on the robot, use the live launch:
ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
    settings_name:=JetAuto camera_topic:=/cam_1/image \
    trajectory_output:=$HOME/orbslam_run.txt

# Filtered (full ORB-SAM-E), with the EfficientSAM3.1 config validated on the Orin:
ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
    use_filter:=true \
    model_path:=$HOME/weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt \
    trajectory_output:=$HOME/orbslam_run_filtered.txt
```

To test only the filter logic (no ROS needed), see the `filter_core.py` usage in
[`efficientsam3_ros2/README.md`](efficientsam3_ros2/README.md).

### Hardware targets

| Target | Use |
|---|---|
| **Jetson Orin Nano 8 GB** (aarch64) | The real deployment; final numbers come from here. |
| Cloud GPU (RunPod T4/A4000, …) | Validate the CUDA pipeline and get reproducible edge-class numbers ([`docs/CLOUD_GPU_TESTING.md`](docs/CLOUD_GPU_TESTING.md)). |
| Ubuntu VM (CPU) | Functional testing only — the filter is too slow for real-time. |

---

## Contributing

This is a research codebase; the goal of this structure is to make it easy for
others to find their way around and reproduce results.

- **Where things go:**
  - SLAM/C++ and ROS launch/config/drivers → `ros2_orb_slam3/`.
  - Filter node and bag tools → `efficientsam3_ros2/`.
  - Model code → `efficientsam3_arm/` (treat as a vendored upstream port; keep
    changes minimal and backward-compatible — see the change table in
    `docs/PAPER_DL_FINDINGS.md`).
  - Evaluation/metrics code → `metrics/`; run outputs → `eval_runs/`.
  - New long-form guides or research notes → `docs/`, and add a line to the
    index above.
- **Do not commit large binaries.** Datasets, model weights, rosbags (`.db3`),
  point clouds (`.ply`), videos, and images are gitignored on purpose. Keep
  weights/datasets out of git (see `.gitignore`); store small text artifacts
  (trajectories, metrics JSON, analysis scripts) under `eval_runs/`.
- **Reproducibility first.** When reporting numbers, measure the real code path
  (`filter_core` / the ROS nodes), not re-implementations, and capture the
  environment with the `metrics/` toolkit. The "gotchas" in
  `docs/CLOUD_GPU_TESTING.md` and `docs/PAPER_DL_FINDINGS.md` exist so nobody
  re-debugs them.
- **Language.** Setup/deployment guides are in English; some research/findings
  notes are in Spanish (marked *(ES)* in the index).

---

## Acknowledgments & license

Built on top of:
- [EfficientSAM3](https://github.com/SimonZeng7108/efficientsam3) — Chengxi Simon Zeng et al.
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) — Carlos Campos et al.
- [ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) — Azmyin Md. Kamal.

See each subproject for its respective license (the ROS filter package is
Apache-2.0).
