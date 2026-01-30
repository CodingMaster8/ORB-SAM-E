# VM Setup Guide: EfficientSAM3 + ORB-SLAM3 Pipeline

This guide walks you through setting up the dynamic object filtering SLAM pipeline on your Ubuntu VM with ROS2 Jazzy.

## Prerequisites

Your VM should have:
- Ubuntu 22.04 or 24.04
- ROS2 Jazzy Jalisco installed
- At least 8GB RAM (16GB recommended)
- Python 3.10+

## Step 1: Transfer Code from MacOS

### Option A: Using the Deploy Script

On your MacOS machine:
```bash
cd /Users/pablovargas/dev/personal/efficientsam3_ros2/scripts
chmod +x deploy_to_vm.sh
./deploy_to_vm.sh <vm_username> <vm_ip_address>
```

Example:
```bash
./deploy_to_vm.sh ubuntu 192.168.64.2
```

### Option B: Manual Copy

```bash
# On MacOS
scp -r /Users/pablovargas/dev/personal/efficientsam3_ros2 user@vm:/home/user/ros2_ws/src/
scp -r /Users/pablovargas/dev/personal/efficientsam3_arm user@vm:/home/user/ros2_ws/src/
scp -r /Users/pablovargas/dev/personal/ros2_orb_slam3 user@vm:/home/user/ros2_ws/src/
```

## Step 2: Install Dependencies on VM

SSH into your VM:
```bash
ssh user@vm_ip
```

### System Dependencies
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build tools
sudo apt install -y build-essential cmake git

# Install Python dependencies
sudo apt install -y python3-pip python3-dev python3-numpy python3-opencv

# Install ROS2 dependencies (if not already installed)
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport
```

### Install PyTorch
```bash
# CPU only (recommended for VM)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Or with CUDA if your VM has GPU passthrough
# pip install torch torchvision
```

### Install EfficientSAM3 ARM
```bash
cd ~/ros2_ws/src/efficientsam3_arm
pip install -e .
```

### Install Additional Python Packages
```bash
pip install pillow natsort
```

## Step 3: Download Model Weights

```bash
mkdir -p ~/weights

# Download RepViT-S (smallest, fastest)
wget -O ~/weights/efficient_sam3_repvit_s.pt \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_repvit_s.pt"
```

Alternative models (larger, more accurate):
```bash
# TinyViT-M
wget -O ~/weights/efficient_sam3_tinyvit_m.pt \
    "https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_tinyvit_m.pt"
```

## Step 4: Build ROS2 Workspace

```bash
cd ~/ros2_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

If build fails for ros2_orb_slam3, check ORB-SLAM3 dependencies:
```bash
# Install Pangolin
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build -j4
sudo cmake --install build

# Configure library path
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

## Step 5: Test the Installation

### Test 1: Verify EfficientSAM3 (No ROS2)
```bash
cd ~/ros2_ws/src/efficientsam3_ros2/efficientsam3_ros2

# Test with a sample image
python filter_core.py \
    --image ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data/1403636579763555584.png \
    --model ~/weights/efficient_sam3_repvit_s.pt \
    --efficientsam3-path ~/ros2_ws/src/efficientsam3_arm \
    --no-show
```

### Test 2: Run Filter Node
```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit_s.pt \
    -p efficientsam3_path:=$HOME/ros2_ws/src/efficientsam3_arm
```

### Test 3: Full Pipeline
```bash
# Terminal 1: Filter Node
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit_s.pt \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered

# Terminal 2: ORB-SLAM3 C++ Node
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=true

# Terminal 3: Python Driver
ros2 run ros2_orb_slam3 mono_driver_filtered_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05 \
    -p use_filter:=true
```

## Step 6: Monitor the Pipeline

```bash
# View available topics
ros2 topic list

# Check image flow
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/image_filtered

# View images (requires display)
ros2 run rqt_image_view rqt_image_view
```

## Troubleshooting

### "Module not found: efficientsam3_arm"
```bash
# Add to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$HOME/ros2_ws/src/efficientsam3_arm
# Or reinstall
cd ~/ros2_ws/src/efficientsam3_arm && pip install -e .
```

### "Cannot load model"
```bash
# Check file exists
ls -la ~/weights/efficient_sam3_repvit_s.pt

# Check PyTorch can load it
python3 -c "import torch; torch.load('$HOME/weights/efficient_sam3_repvit_s.pt', map_location='cpu')"
```

### Low FPS
```bash
# Use frame skipping
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p process_every_n_frames:=2

# Use smaller model
# Download efficient_sam3_efficientvit_s.pt instead
```

### ORB-SLAM3 Build Errors
See the original ros2_orb_slam3 README for detailed build instructions.

## Quick Start Commands

Save this as `run_pipeline.sh`:
```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start filter in background
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=$HOME/weights/efficient_sam3_repvit_s.pt &

sleep 5

# Start SLAM in background
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p use_filtered_images:=true &

sleep 3

# Start driver (foreground)
ros2 run ros2_orb_slam3 mono_driver_filtered_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05
```

Make it executable and run:
```bash
chmod +x run_pipeline.sh
./run_pipeline.sh
```
