# ORB-SAM-E: Cloud Pod Runbook (step by step)

A copy-paste runbook to go from a **fresh RunPod GPU pod** to **recorded
benchmark results** (ATE/RPE accuracy + FPS + GPU usage) for the paper.

- **Target pod:** RunPod, **RTX A4000**, template
  `runpod/pytorch:...-torch280-ubuntu2404` (Ubuntu 24.04 → **ROS 2 Jazzy**).
- **You already have:** code working on an Ubuntu VM (CPU). This reproduces it
  on a CUDA GPU and records metrics with the `metrics/` toolkit.
- **Time:** ~1–2 h the first time (mostly the ORB-SLAM3 build).

> Everything below runs **on the pod** unless a step says "(on your Mac)".

---

## Step 0 — Connect and sanity-check the GPU

SSH in (use the command from the pod's **Connect** page), then:

```bash
nvidia-smi                                   # GPU + driver visible?
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# Expect: 2.8.x  True
lsb_release -a                               # Expect: Ubuntu 24.04
```

If `torch.cuda.is_available()` is **False**, stop — the pod/template is wrong.

---

## Step 1 — Persistent storage (do this so you don't redo setup)

Put everything under the persistent volume (usually `/workspace`) so a pod
restart keeps your build, weights, and datasets:

```bash
df -h | grep -E "workspace|/$"               # confirm /workspace exists
export WORK=/workspace                        # fallback: export WORK=$HOME
echo "export WORK=$WORK" >> ~/.bashrc
```

Everything below uses `$WORK`. (If there is no volume, use `$HOME` — but then
**terminating** the pod deletes your work.)

---

## Step 2 — System update + base tools

```bash
apt-get update && apt-get upgrade -y
apt-get install -y curl wget git vim build-essential cmake pkg-config \
    ninja-build software-properties-common locales sudo
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## Step 3 — Install ROS 2 Jazzy

```bash
add-apt-repository universe -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update
apt-get install -y ros-jazzy-desktop ros-dev-tools \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-vision-opencv \
    ros-jazzy-rqt-image-view ros-jazzy-sensor-msgs ros-jazzy-std-msgs

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
ros2 --version
```

---

## Step 4 — System dependencies (OpenCV, Eigen, Boost, etc.)

```bash
apt-get install -y \
    libeigen3-dev libopencv-dev python3-opencv \
    libboost-all-dev libssl-dev \
    libglew-dev libglfw3-dev libgtk-3-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    python3-pip python3-dev python3-numpy python3-venv
```

---

## Step 5 — Install Pangolin (ORB-SLAM3 visualization dependency)

```bash
cd $WORK
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
yes | ./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build -j"$(nproc)"
cmake --install build
ldconfig
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Step 6 — Get the ORB-SAM-E code onto the pod

**Option A — clone from git (if you've pushed the repo):**

```bash
mkdir -p $WORK/ros2_ws/src
cd $WORK/ros2_ws/src
git clone <YOUR_REPO_URL> ORB-SAM-E
```

**Option B — copy from your Mac (run this block ON YOUR MAC):**

```bash
# Replace HOST/PORT with the pod's SSH details from the Connect page.
POD="root@<POD_IP>"; PORT=<PORT>
ssh -p $PORT $POD "mkdir -p /workspace/ros2_ws/src"
rsync -avz -e "ssh -p $PORT" --exclude '.git' --exclude 'build' --exclude 'install' \
    /Users/pablovargas/dev/personal/ORB-SAM-E \
    $POD:/workspace/ros2_ws/src/
```

After this you should have `$WORK/ros2_ws/src/ORB-SAM-E/` containing
`ros2_orb_slam3`, `efficientsam3_ros2`, `efficientsam3_arm`, and `metrics`.

---

## Step 7 — Python packages

PyTorch is **already installed** on this image — do **not** reinstall it.

```bash
cd $WORK/ros2_ws/src/ORB-SAM-E
pip3 install -e efficientsam3_arm
pip3 install pillow natsort matplotlib scipy
pip3 install -r metrics/requirements.txt        # metrics toolkit deps
```

---

## Step 8 — Build ORB-SLAM3 third-party libs + the workspace

```bash
cd $WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2
mkdir -p build && cd build && cmake .. && make -j"$(nproc)"

cd $WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/g2o
mkdir -p build && cd build && cmake .. && make -j"$(nproc)"

# Build the ROS 2 workspace (includes the trajectory-saving C++ change)
cd $WORK/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep init 2>/dev/null; rosdep update
rosdep install -r --from-paths src --ignore-src -y --rosdistro jazzy || true
colcon build --symlink-install

echo "source $WORK/ros2_ws/install/setup.bash" >> ~/.bashrc
source $WORK/ros2_ws/install/setup.bash
ros2 pkg list | grep -E "efficientsam3_ros2|ros2_orb_slam3"
```

> If `mono_node_cpp` can't find the ORB vocabulary at runtime, see
> **Troubleshooting → Vocabulary path** below.

---

## Step 9 — Download model weights + a TUM dataset

```bash
mkdir -p $HOME/weights
wget -O $HOME/weights/efficient_sam3_repvit_s.pt \
  "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_repvit_s.pt"

# Dynamic sequence (people walking) — the key one for the paper
mkdir -p $WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/tum
cd $WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/tum
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg3/tgz/rgbd_dataset_freiburg3_walking_xyz.tgz
tar -xzf rgbd_dataset_freiburg3_walking_xyz.tgz && rm rgbd_dataset_freiburg3_walking_xyz.tgz

# Optional static control sequence
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/tgz/rgbd_dataset_freiburg1_desk.tgz
tar -xzf rgbd_dataset_freiburg1_desk.tgz && rm rgbd_dataset_freiburg1_desk.tgz
```

---

## Step 10 — Headless display (the pod has no monitor)

ORB-SLAM3 opens a Pangolin window, which needs an X display. Use a virtual one:

```bash
apt-get install -y xvfb mesa-utils
# Test that a virtual GL context works:
xvfb-run -a glxinfo | grep "OpenGL renderer" || echo "GL via xvfb not available"
```

You'll prefix the benchmark with `xvfb-run -a` (Step 12).

---

## Step 11 — Quick functional test (verify GPU inference works)

```bash
cd $WORK/ros2_ws/src/ORB-SAM-E/efficientsam3_ros2/efficientsam3_ros2
python3 filter_core.py \
    --image $WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data/1403636579763555584.png \
    --model $HOME/weights/efficient_sam3_repvit_s.pt \
    --efficientsam3-path $WORK/ros2_ws/src/ORB-SAM-E/efficientsam3_arm \
    --no-show
```

Expected: model loads, prints detections, and `Stats:` shows `device: cuda`
with an `inference_fps` value.

---

## Step 12 — Run the automated benchmarks (this records the metrics)

```bash
source /opt/ros/jazzy/setup.bash
source $WORK/ros2_ws/install/setup.bash
cd $WORK/ros2_ws/src/ORB-SAM-E/metrics

SEQ=$WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz

# Baseline: no dynamic filtering
xvfb-run -a ./run_benchmark.sh fr3_walk_baseline TUM3 "$SEQ" false results/fr3_walk_baseline

# Filtered: EfficientSAM3 on GPU
xvfb-run -a ./run_benchmark.sh fr3_walk_filtered TUM3 "$SEQ" true  results/fr3_walk_filtered

# Combine into one comparison report (ATE/RPE, FPS, GPU usage)
python3 -m slam_metrics.report results/*/results.json -o results/comparison.md
cat results/comparison.md
```

Each run prints `ATE RMSE: ... m` when it finishes. The headline result is the
**ATE improvement of `fr3_walk_filtered` vs `fr3_walk_baseline`**.

Repeat for other sequences by changing `SEQ`, the `TUMx` setting, and the run
name (see `metrics/README.md` for the suggested sequence matrix).

---

## Step 13 — Pull results back to your Mac

Run **on your Mac**:

```bash
POD="root@<POD_IP>"; PORT=<PORT>
rsync -avz -e "ssh -p $PORT" \
    $POD:/workspace/ros2_ws/src/ORB-SAM-E/metrics/results \
    /Users/pablovargas/dev/personal/ORB-SAM-E/metrics/
```

You now have `results/comparison.md`, per-run `results.json`, `resources.csv`,
and `KeyFrameTrajectory.txt` locally.

---

## Step 14 — Stop billing

When finished, **Terminate** the pod from the RunPod dashboard (Stop only if
you have a persistent volume and intend to return soon).

---

## Troubleshooting

**Vocabulary / settings path not found (`mono_node_cpp` exits immediately).**
`ros2_orb_slam3` hardcodes a workspace path in
`ros2_orb_slam3/include/ros2_orb_slam3/common.hpp` (`packagePath`). Make it
match your layout, then rebuild:

```bash
# Edit common.hpp: set packagePath to match this workspace, e.g.
#   std::string packagePath = "ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/";
cd $WORK/ros2_ws && colcon build --symlink-install
```

**`xvfb-run: glxinfo` shows no renderer / Pangolin still fails.** Install
software GL and force it:

```bash
apt-get install -y libgl1-mesa-dri
export LIBGL_ALWAYS_SOFTWARE=1
```

**ATE shows "Too few timestamp associations".** The estimated and groundtruth
timestamps don't overlap. The TUM driver stamps frames with ROS clock time, not
the dataset timestamp — if associations are ~0, widen the tolerance:

```bash
python3 -m slam_metrics.trajectory_eval \
    results/fr3_walk_filtered/KeyFrameTrajectory.txt "$SEQ/groundtruth.txt" \
    --max-diff 0.1
```

`trajectory_eval` aligns **with scale** by default (correct for monocular);
pass `--no-scale` only for metric stereo/RGB-D trajectories. (In the
`assemble`/`run_benchmark.sh` path the equivalent flag is `--mono`.)

**Low GPU utilization / want a more Jetson-like number.** Remember the A4000 is
far stronger than an Orin Nano, so FPS is an upper bound. Use `repvit_s` and
note this in the paper (see `CLOUD_GPU_TESTING.md`).

**Module not found: efficientsam3_arm.**

```bash
cd $WORK/ros2_ws/src/ORB-SAM-E/efficientsam3_arm && pip3 install -e .
```

---

## One-glance command summary

```bash
# after Steps 2–10 are done, every session just needs:
source /opt/ros/jazzy/setup.bash
source $WORK/ros2_ws/install/setup.bash
cd $WORK/ros2_ws/src/ORB-SAM-E/metrics
SEQ=$WORK/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz
xvfb-run -a ./run_benchmark.sh fr3_walk_baseline TUM3 "$SEQ" false results/fr3_walk_baseline
xvfb-run -a ./run_benchmark.sh fr3_walk_filtered TUM3 "$SEQ" true  results/fr3_walk_filtered
python3 -m slam_metrics.report results/*/results.json -o results/comparison.md
```
