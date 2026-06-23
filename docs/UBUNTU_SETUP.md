# ORB-SAM-E: Ubuntu Setup Guide

Complete guide for setting up ORB-SAM-E (ORB-SLAM3 + EfficientSAM3) on Ubuntu with ROS 2 Jazzy Jalisco.

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Step 1: Install Ubuntu 24.04](#step-1-install-ubuntu-2404)
- [Step 2: Install ROS 2 Jazzy Jalisco](#step-2-install-ros-2-jazzy-jalisco)
- [Step 3: Install System Dependencies](#step-3-install-system-dependencies)
- [Step 4: Install Pangolin](#step-4-install-pangolin)
- [Step 5: Build ORB-SLAM3 Third-Party Libraries](#step-5-build-orb-slam3-third-party-libraries)
- [Step 6: Install Python Dependencies](#step-6-install-python-dependencies)
- [Step 7: Download Model Weights](#step-7-download-model-weights)
- [Step 8: Setup ROS 2 Workspace](#step-8-setup-ros-2-workspace)
- [Step 9: Build the Workspace](#step-9-build-the-workspace)
- [Using TUM RGB-D Dataset](#using-tum-rgb-d-dataset)
- [Step 10: Test Installation](#step-10-test-installation)
- [Step 11: Run the Full Pipeline](#step-11-run-the-full-pipeline)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)
- [Performance Optimization](#performance-optimization)

---

## Overview

ORB-SAM-E combines:
- **ORB-SLAM3**: Visual SLAM system for camera pose estimation and mapping
- **EfficientSAM3**: Lightweight semantic segmentation for filtering dynamic objects
- **ROS 2 Jazzy**: Robotics middleware for inter-process communication

The pipeline filters out dynamic objects (people, vehicles, animals) from camera frames before SLAM processing, improving localization accuracy in dynamic environments.

### Architecture

```
┌─────────────────┐    /camera/image_raw     ┌──────────────────────────┐
│  Camera/Driver  │ ─────────────────────────▶│  EfficientSAM3 Filter    │
│  (Image Source) │    sensor_msgs/Image      │       Node (Python)      │
└─────────────────┘                           └────────────┬─────────────┘
                                                           │
                                                           │ /camera/image_filtered
                                                           ▼
                                              ┌──────────────────────────┐
                                              │   ORB-SLAM3 Node (C++)   │
                                              │  - Camera Tracking       │
                                              │  - Pose Estimation       │
                                              │  - Map Building          │
                                              └──────────────────────────┘
```

---

## System Requirements

### Hardware (Minimum)
- **CPU**: x86_64 (Intel/AMD) or ARM aarch64 (e.g., NVIDIA Jetson, Raspberry Pi 5, Apple Silicon via VM)
- **RAM**: 8 GB (16 GB recommended)
- **Storage**: 20 GB free space
- **GPU**: Optional (CUDA-capable NVIDIA GPU for acceleration)

### Hardware (Recommended)
- **CPU**: Intel i5/i7, AMD Ryzen 5/7, or ARM Cortex-A76+
- **RAM**: 16-32 GB
- **GPU**: NVIDIA GPU with CUDA support (RTX 20xx or newer)

### Software
- Ubuntu 24.04 LTS (Noble Numbat) - **Required for ROS 2 Jazzy**
- Or Ubuntu 22.04 LTS (Jammy Jellyfish) with ROS 2 Humble

---

## Step 1: Install Ubuntu 24.04

If you don't have Ubuntu installed:

1. Download Ubuntu 24.04 LTS from [ubuntu.com/download](https://ubuntu.com/download/desktop)
2. Create a bootable USB drive using [Balena Etcher](https://www.balena.io/etcher/)
3. Boot from USB and follow the installation wizard

After installation, update the system:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git vim software-properties-common
```

---

## Step 2: Install ROS 2 Jazzy Jalisco

### 2.1 Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### 2.2 Setup Sources

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.3 Install ROS 2 Packages

```bash
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-jazzy-desktop

# Install development tools
sudo apt install -y ros-dev-tools

# Install additional ROS 2 packages needed for this project
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-opencv \
    ros-jazzy-rqt-image-view \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs
```

### 2.4 Environment Setup

Add to `~/.bashrc`:

```bash
echo "# ROS 2 Jazzy" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.5 Verify Installation

```bash
ros2 --version
# Expected output: ros2 version X.X.X
```

---

## Step 3: Install System Dependencies

### 3.1 Build Essentials

```bash
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    ninja-build
```

### 3.2 Eigen3

```bash
sudo apt install -y libeigen3-dev

# Verify installation
pkg-config --modversion eigen3
# Expected: 3.4.x or higher
```

### 3.3 OpenCV

Ubuntu 24.04 comes with OpenCV 4.6+:

```bash
sudo apt install -y \
    libopencv-dev \
    python3-opencv

# Verify installation
python3 -c "import cv2; print(cv2.__version__)"
# Expected: 4.6.x or higher
```

### 3.4 Boost Libraries

```bash
sudo apt install -y \
    libboost-all-dev \
    libboost-serialization-dev \
    libboost-system-dev
```

### 3.5 Other Dependencies

```bash
sudo apt install -y \
    libssl-dev \
    libglew-dev \
    libglfw3-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran
```

---

## Step 4: Install Pangolin

Pangolin is required for ORB-SLAM3's visualization.

### 4.1 Clone and Build

```bash
cd ~/Documents
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# Install prerequisites
./scripts/install_prerequisites.sh recommended

# Build
cmake -B build
cmake --build build -j$(nproc)

# Install system-wide
sudo cmake --install build
```

### 4.2 Configure Dynamic Library Path

```bash
# Check if /usr/local/lib is in the library path
echo $LD_LIBRARY_PATH

# Add to library path (add to ~/.bashrc)
echo '' >> ~/.bashrc
echo '# Pangolin library path' >> ~/.bashrc
echo 'if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then' >> ~/.bashrc
echo '    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'fi' >> ~/.bashrc

source ~/.bashrc
sudo ldconfig
```

### 4.3 Verify Installation

```bash
ls /usr/local/lib | grep -i pangolin
# Should show libpangolin.so files
```

---

## Step 5: Build ORB-SLAM3 Third-Party Libraries

The ORB-SLAM3 package includes DBoW2 and g2o libraries that may need rebuilding.

> **ARM users:** The pre-built libraries in the repository were compiled for x86_64. You **must** rebuild them on ARM (aarch64) — the steps below handle this automatically. If you already have x86_64 `.so` files in the `lib/` directories, the clean rebuild steps below will replace them with native ARM binaries.

### 5.1 Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 5.2 Copy Project Files

**Option A: From this repository (if cloned)**
```bash
# Assuming you've cloned ORB-SAM-E somewhere
cp -r /path/to/ORB-SAM-E/efficientsam3_arm ~/ros2_ws/src/
cp -r /path/to/ORB-SAM-E/efficientsam3_ros2 ~/ros2_ws/src/
cp -r /path/to/ORB-SAM-E/ros2_orb_slam3 ~/ros2_ws/src/
```

**Option B: Transfer from MacOS via SCP**
```bash
# Run on MacOS:
scp -r /Users/pablovargas/dev/personal/ORB-SAM-E/efficientsam3_arm user@ubuntu-ip:~/ros2_ws/src/
scp -r /Users/pablovargas/dev/personal/ORB-SAM-E/efficientsam3_ros2 user@ubuntu-ip:~/ros2_ws/src/
scp -r /Users/pablovargas/dev/personal/ORB-SAM-E/ros2_orb_slam3 user@ubuntu-ip:~/ros2_ws/src/
```

### 5.3 Build DBoW2

```bash
cd ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2

# Clean any previous build (important on ARM if pre-built x86_64 binaries exist)
rm -rf build

mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 5.4 Build g2o

```bash
cd ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/g2o

# Clean any previous build (important on ARM if pre-built x86_64 binaries exist)
rm -rf build

mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

> **Note:** You may see deprecation warnings about `Eigen::AlignedBit` during the g2o build. These are harmless and do not affect the resulting library.

### 5.5 Verify Libraries Built

```bash
ls ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2/lib/
# Should show: libDBoW2.so

ls ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/lib/
# Should show: libg2o.so
```

**On ARM, verify the libraries match your architecture:**
```bash
file ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
# Should show: ELF 64-bit LSB shared object, ARM aarch64

file ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/g2o/lib/libg2o.so
# Should show: ELF 64-bit LSB shared object, ARM aarch64
```

---

## Step 6: Install Python Dependencies

### 6.1 Python Base Packages

```bash
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-venv
```

> **Ubuntu 24.04 (PEP 668):** Ubuntu 24.04 marks the system Python as "externally managed," so `pip3 install` will fail by default. Since ROS 2 nodes need system-wide access to these packages, add `--break-system-packages` to all `pip3 install` commands below. Alternatively, you can configure this globally:
> ```bash
> mkdir -p ~/.config/pip
> echo -e "[global]\nbreak-system-packages = true" > ~/.config/pip/pip.conf
> ```

### 6.2 Install PyTorch

**For CPU only (default, works on x86_64 and ARM aarch64):**
```bash
pip3 install --break-system-packages torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

**For CUDA (if you have NVIDIA GPU):**
```bash
# Check your CUDA version first
nvidia-smi

# Install matching PyTorch (example for CUDA 12.1)
pip3 install --break-system-packages torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### 6.3 Install EfficientSAM3 ARM Package

```bash
cd ~/ros2_ws/src/ORB-SAM-E/efficientsam3_arm
pip3 install --break-system-packages -e .
```

### 6.4 Install Additional Python Packages

```bash
pip3 install --break-system-packages \
    pillow \
    natsort \
    matplotlib \
    scipy \
    einops \
    pycocotools
```

> **Note:** `einops` and `pycocotools` are required by the EfficientSAM3 ARM model but are not listed in its core dependencies. They must be installed explicitly.

### 6.5 Verify PyTorch Installation

```bash
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"
```

Verify EfficientSAM3 ARM imports work:
```bash
python3 -c "from efficientsam3_arm import build_efficientsam3_image_model; print('efficientsam3_arm: OK')"
```

---

## Step 7: Download Model Weights

### 7.1 Create Weights Directory

```bash
mkdir -p ~/weights
```

### 7.2 Download EfficientSAM3 Checkpoint

**Option A: RepViT-M0.9 + MobileClip-S1 (Recommended)**
```bash
wget -O ~/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_repvit-m0_9_mobileclip_s1.pth"
```

**Option B: RepViT-S (Smallest, Fastest)**
```bash
wget -O ~/weights/efficient_sam3_repvit_s.pt \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_repvit_s.pt"
```

**Option C: TinyViT-M (Balanced)**
```bash
wget -O ~/weights/efficient_sam3_tinyvit_m.pt \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_tinyvit_m.pt"
```

**Option D: TinyViT-L (Larger, More Accurate)**
```bash
wget -O ~/weights/efficient_sam3_tinyvit_l.pt \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_tinyvit_l.pt"
```

### 7.3 Verify Download

```bash
ls -lh ~/weights/
# Should show the downloaded .pth/.pt file
```

---

## Step 8: Setup ROS 2 Workspace

### 8.1 Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 8.2 Install ROS Dependencies

```bash
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src -y --rosdistro jazzy
```

---

## Step 9: Build the Workspace

### 9.1 Build All Packages

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash

# Build with symlink install (allows editing Python files without rebuild)
colcon build --symlink-install
```

### 9.2 Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash

# Add to ~/.bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 9.3 Verify Build

```bash
ros2 pkg list | grep -E "(efficientsam3|orb_slam3)"
# Should show:
# efficientsam3_ros2
# ros2_orb_slam3
```

---

## Using TUM RGB-D Dataset

The TUM RGB-D benchmark is widely used for evaluating visual SLAM systems. This section covers downloading and running ORB-SAM-E with TUM datasets.

### TUM Dataset Structure

TUM datasets have this format:
```
rgbd_dataset_freiburgX_sequence/
├── rgb/                    # RGB images
│   ├── 1305031102.175304.png
│   └── ...
├── depth/                  # Depth images (optional for monocular)
│   └── ...
├── rgb.txt                 # Timestamp-filename pairs
├── depth.txt               # Depth timestamp-filename pairs
├── groundtruth.txt         # Ground truth poses
└── accelerometer.txt       # IMU data (optional)
```

### Download TUM Dataset

We provide a download script for common TUM sequences:

```bash
# Make script executable
chmod +x ~/ros2_ws/src/ros2_orb_slam3/scripts/download_tum_dataset.sh

# See available sequences
~/ros2_ws/src/ros2_orb_slam3/scripts/download_tum_dataset.sh --help

# Download recommended test sequence (fr1/desk)
~/ros2_ws/src/ros2_orb_slam3/scripts/download_tum_dataset.sh fr1/desk

# Download sequence with walking people (good for testing dynamic filtering!)
~/ros2_ws/src/ros2_orb_slam3/scripts/download_tum_dataset.sh fr3/walking_xyz
```

### Available TUM Sequences

| Sequence | Config | Description | Good for |
|----------|--------|-------------|----------|
| `fr1/xyz` | TUM1 | Simple camera motion | Calibration |
| `fr1/desk` | TUM1 | Office desk scene | Basic testing |
| `fr1/room` | TUM1 | Full room | SLAM evaluation |
| `fr2/desk` | TUM2 | Office desk | Different camera |
| `fr2/pioneer_slam` | TUM2 | Robot SLAM sequence | Robot navigation |
| `fr3/long_office_household` | TUM3 | Long office sequence | Extended testing |
| `fr3/walking_xyz` | TUM3 | **Has walking people** | **Dynamic filtering!** |
| `fr3/walking_rpy` | TUM3 | **Has walking people** | **Dynamic filtering!** |

### Manual Download

If you prefer manual download:

```bash
# Create directory
mkdir -p ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum
cd ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum

# Download fr1/desk sequence
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz
tar -xzf rgbd_dataset_freiburg1_desk.tgz
rm rgbd_dataset_freiburg1_desk.tgz

# Download fr3/walking_xyz (has dynamic objects - people walking)
wget https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_xyz.tgz
tar -xzf rgbd_dataset_freiburg3_walking_xyz.tgz
rm rgbd_dataset_freiburg3_walking_xyz.tgz
```

### Run ORB-SLAM3 with TUM Dataset

#### Without Dynamic Filtering

```bash
# Terminal 1 - Start ORB-SLAM3 C++ node
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp

# Terminal 2 - Start TUM driver
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \
    -p settings_name:=TUM1 \
    -p dataset_path:=$HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg1_desk
```

#### With EfficientSAM3 Dynamic Filtering

This is especially useful for sequences with people (fr3/walking_*):

```bash
# Terminal 1 - Start EfficientSAM3 filter node
source ~/ros2_ws/install/setup.bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered \
    -p confidence_threshold:=0.3

# Terminal 2 - Start ORB-SLAM3 (subscribing to filtered images)
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=true

# Terminal 3 - Start TUM driver with filter mode
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \
    -p settings_name:=TUM3 \
    -p dataset_path:=$HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz \
    -p use_filter:=true
```

### TUM Driver Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `settings_name` | string | "TUM1" | Config file: TUM1, TUM2, or TUM3 |
| `dataset_path` | string | required | Full path to TUM sequence folder |
| `use_filter` | bool | false | Enable EfficientSAM3 filtering |
| `frame_rate` | float | 30.0 | Playback frame rate (Hz) |
| `start_frame` | int | 0 | Start from this frame |
| `end_frame` | int | -1 | End at this frame (-1 = all) |

### Choosing the Right Config File

| Dataset Series | Camera | Config File |
|----------------|--------|-------------|
| freiburg1 (fr1/*) | Kinect RGB | **TUM1.yaml** |
| freiburg2 (fr2/*) | Kinect RGB | **TUM2.yaml** |
| freiburg3 (fr3/*) | Asus Xtion | **TUM3.yaml** |

### Quick Start Script for TUM

Create a convenience script:

```bash
cat > ~/run_tum_slam.sh << 'EOF'
#!/bin/bash
# Run ORB-SAM-E with TUM dataset

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Configuration
SETTINGS=${1:-TUM1}
DATASET=${2:-$HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg1_desk}
USE_FILTER=${3:-false}

echo "Settings: $SETTINGS"
echo "Dataset: $DATASET"
echo "Use Filter: $USE_FILTER"

if [ "$USE_FILTER" = "true" ]; then
    # Start filter in background
    ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
        -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth &
    FILTER_PID=$!
    sleep 5
    
    # Start SLAM with filtered images
    ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
        -p node_name_arg:=mono_slam_cpp \
        -p use_filtered_images:=true &
else
    # Start SLAM directly
    ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
        -p node_name_arg:=mono_slam_cpp &
fi

SLAM_PID=$!
sleep 3

# Start TUM driver
ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \
    -p settings_name:=$SETTINGS \
    -p dataset_path:=$DATASET \
    -p use_filter:=$USE_FILTER

# Cleanup
kill $SLAM_PID 2>/dev/null
[ -n "$FILTER_PID" ] && kill $FILTER_PID 2>/dev/null
EOF

chmod +x ~/run_tum_slam.sh
```

Usage:
```bash
# Without filter (fr1/desk)
~/run_tum_slam.sh TUM1 ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg1_desk false

# With filter (fr3/walking - has people!)
~/run_tum_slam.sh TUM3 ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz true
```

---

## Step 10: Test Installation

### 10.1 Test EfficientSAM3 (No ROS 2 Required)

```bash
cd ~/ros2_ws/src/efficientsam3_ros2/efficientsam3_ros2

# Test with sample image from ORB-SLAM3 dataset
python3 filter_core.py \
    --image ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data/1403636579763555584.png \
    --model ~/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    --efficientsam3-path ~/ros2_ws/src/efficientsam3_arm \
    --no-show
```

Expected output:
```
Loading EfficientSAM3 model...
Model loaded successfully
Processing image...
Detected X objects
```

### 10.2 Test ROS 2 Filter Node

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    -p efficientsam3_path:=$HOME/ros2_ws/src/efficientsam3_arm

# Terminal 2
ros2 node list
# Should show: /dynamic_filter_node
```

### 10.3 Test ORB-SLAM3 Standalone

```bash
# Terminal 1 - Start ORB-SLAM3 C++ node
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp

# Terminal 2 - Start driver node
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05
```

If working correctly, you should see the Pangolin visualization window with camera trajectory and map points.

---

## Step 11: Run the Full Pipeline

### Option 1: Using Launch File (Recommended)

```bash
# Terminal 1 - Launch filter and view nodes
ros2 launch efficientsam3_ros2 slam_pipeline.launch.py \
    model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth

# Terminal 2 - Start ORB-SLAM3 C++ node with filtered images
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=true

# Terminal 3 - Start image driver
ros2 run ros2_orb_slam3 mono_driver_filtered_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05 \
    -p use_filter:=true
```

### Option 2: Manual Node Startup

```bash
# Terminal 1 - Start EfficientSAM3 filter node
source ~/ros2_ws/install/setup.bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered \
    -p confidence_threshold:=0.3

# Terminal 2 - Start ORB-SLAM3 C++ node
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=true

# Terminal 3 - Start image driver
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_filtered_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05 \
    -p use_filter:=true
```

### Option 3: Quick Start Script

Create a script for convenience:

```bash
cat > ~/run_orb_sam_e.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start filter in background
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth &
FILTER_PID=$!
sleep 5

# Start SLAM in background
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=true &
SLAM_PID=$!
sleep 3

# Start driver (foreground)
ros2 run ros2_orb_slam3 mono_driver_filtered_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05 \
    -p use_filter:=true

# Cleanup
kill $FILTER_PID $SLAM_PID 2>/dev/null
EOF

chmod +x ~/run_orb_sam_e.sh
```

Run with:
```bash
~/run_orb_sam_e.sh
```

### Monitor the Pipeline

```bash
# View available topics
ros2 topic list

# Check image flow
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/image_filtered

# View images (requires display)
ros2 run rqt_image_view rqt_image_view

# View filter mask
ros2 run rqt_image_view rqt_image_view /dynamic_filter/mask
```

---

## Configuration Reference

### Filter Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | **required** | Path to EfficientSAM3 checkpoint |
| `efficientsam3_path` | string | "" | Path to efficientsam3_arm package |
| `dynamic_classes` | list | [person, car, ...] | Object classes to filter |
| `confidence_threshold` | float | 0.3 | Detection confidence threshold |
| `masking_strategy` | string | "grayout" | How to handle masked regions |
| `input_topic` | string | /camera/image_raw | Input image topic |
| `output_topic` | string | /camera/image_filtered | Output filtered image topic |
| `publish_mask` | bool | true | Publish mask visualization |
| `device` | string | "auto" | Inference device (auto/cpu/cuda) |
| `process_every_n_frames` | int | 1 | Frame skip: reuse last mask for N-1 frames |
| `num_threads` | int | 0 | PyTorch CPU threads (0 = auto) |
| `dynamic_prompts` | list | [6 body-part prompts] | Text prompts for detection (fewer = faster) |

### Performance Tuning — Filter Node Commands

The filter node has three speed profiles. Choose based on your hardware:

**Default (GPU / Apple Silicon)** — 6 prompts, no frame skip:

```bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered
```

**Fast (CPU with multiple cores)** — 3 prompts, skip every 5 frames, use all cores:

```bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered \
    -p process_every_n_frames:=5 \
    -p num_threads:=4 \
    -p dynamic_prompts:="['human head and face', 'human shirt and body', 'human legs and pants']"
```

**Ultra-fast (VM / low-resource CPU)** — 1 combined prompt, skip every 5 frames, use all cores:

```bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered \
    -p process_every_n_frames:=5 \
    -p num_threads:=4 \
    -p dynamic_prompts:="['human head and face shirt and body legs and pants']"
```

> **Tip:** The model loads eagerly at startup (~20-30s on CPU). Wait for the
> `Model ready!` log before starting the TUM driver. The first frame is always
> processed immediately; intermediate frames reuse the last mask at near-zero cost.

### Masking Strategies

| Strategy | Description | Performance |
|----------|-------------|-------------|
| `blackout` | Set masked pixels to black | Fastest |
| `grayout` | Set masked pixels to gray | Fast (recommended) |
| `inpaint` | Fill with surrounding texture | Slow |
| `blur` | Apply Gaussian blur | Medium |

### Edit Configuration File

```bash
nano ~/ros2_ws/src/efficientsam3_ros2/config/dynamic_filter.yaml
```

```yaml
dynamic_filter_node:
  ros__parameters:
    model_path: "/home/user/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth"
    dynamic_prompts:
      - "human head and face"
      - "human shirt and body"
      - "human legs and pants"
    confidence_threshold: 0.03
    masking_strategy: "grayout"
    device: "auto"
    process_every_n_frames: 5
    num_threads: 4
```

---

## Troubleshooting

### Issue: Third-party libraries fail on ARM (aarch64)

If you see errors like `cannot execute binary file: Exec format error` or linker errors when building ORB-SLAM3, the pre-built `.so` files are x86_64 binaries. You must rebuild them natively:

```bash
# Verify the problem
file ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
# If it shows "x86-64" instead of "ARM aarch64", rebuild:

# Rebuild DBoW2
cd ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)

# Rebuild g2o
cd ~/ros2_ws/src/ORB-SAM-E/ros2_orb_slam3/orb_slam3/Thirdparty/g2o
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)
```

### Issue: "Module not found: efficientsam3_arm"

```bash
# Add to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$HOME/ros2_ws/src/efficientsam3_arm

# Or reinstall
cd ~/ros2_ws/src/efficientsam3_arm
pip3 install -e .
```

### Issue: "Cannot load model" / "Model file not found"

```bash
# Check file exists
ls -la ~/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth

# Check PyTorch can load it
python3 -c "import torch; torch.load('$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth', map_location='cpu')"
```

### Issue: Pangolin errors / "cannot find -lpangolin"

```bash
# Rebuild Pangolin
cd ~/Documents/Pangolin/build
sudo cmake --install .
sudo ldconfig
```

### Issue: ORB-SLAM3 build errors

```bash
# Rebuild third-party libraries
cd ~/ros2_ws/src/ros2_orb_slam3/orb_slam3/Thirdparty/DBoW2
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)

cd ~/ros2_ws/src/ros2_orb_slam3/orb_slam3/Thirdparty/g2o
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)

# Rebuild workspace
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install

MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 --executor sequential
```

### Issue: "libDBoW2.so not found" at runtime

```bash
# Check if library is installed
ls ~/ros2_ws/install/ros2_orb_slam3/lib/

# Add to library path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/install/ros2_orb_slam3/lib
```

### Issue: Low FPS / Slow inference

```bash
# Use frame skipping
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p process_every_n_frames:=2 \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth

# Use CUDA if available
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p device:=cuda \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth
```

### Issue: No detections from filter

```bash
# Lower confidence threshold
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p confidence_threshold:=0.15 \
    -p model_path:=$HOME/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth
```

---

## Performance Optimization

### 1. Use Smaller Model

RepViT-S is the fastest option:
```bash
wget -O ~/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_repvit-m0_9_mobileclip_s1.pth"
```

### 2. Enable Frame Skipping

Process every Nth frame:
```bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p process_every_n_frames:=2
```

### 3. Use CUDA (if available)

```bash
# Check CUDA availability
python3 -c "import torch; print(torch.cuda.is_available())"

# Use CUDA device
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p device:=cuda
```

### 4. Reduce Dynamic Classes

Only filter essential objects:
```yaml
dynamic_classes:
  - "person"
  - "car"
```

### 5. Use Faster Masking Strategy

```bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p masking_strategy:=blackout
```

---

## Quick Reference Commands

```bash
# Source environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# List ROS 2 topics
ros2 topic list

# Check topic frequency
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/image_filtered

# View nodes
ros2 node list

# View node parameters
ros2 param list /dynamic_filter_node

# Visualize images
ros2 run rqt_image_view rqt_image_view

# Record rosbag
ros2 bag record -o slam_session /camera/image_raw /camera/image_filtered

# Play rosbag
ros2 bag play slam_session
```

---

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [EfficientSAM3 Repository](https://github.com/SimonZeng7108/efficientsam3)
- [ROS 2 ORB-SLAM3 Original](https://github.com/Mechazo11/ros2_orb_slam3)

---

## License

- EfficientSAM3: Apache 2.0
- ORB-SLAM3: GPLv3
- This integration: Apache 2.0
