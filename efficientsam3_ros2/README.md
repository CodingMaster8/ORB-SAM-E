# EfficientSAM3 ROS2 - Dynamic Object Filtering for SLAM

This ROS2 package provides dynamic object filtering for visual SLAM using [EfficientSAM3](https://github.com/SimonZeng7108/efficientsam3). It filters out dynamic objects (people, vehicles, animals, etc.) from camera frames before they are processed by SLAM systems like ORB-SLAM3, improving localization accuracy in dynamic environments.

## Architecture

```
┌─────────────────┐    /camera/image_raw     ┌──────────────────────────┐
│  Camera/Driver  │ ─────────────────────────▶│  EfficientSAM3 Filter    │
│  (Image Source) │    sensor_msgs/Image      │       Node (Python)      │
└─────────────────┘                           │                          │
                                              │  - Detects: person, car, │
                                              │    bike, dog, etc.       │
                                              │  - Creates binary mask   │
                                              │  - Applies mask to image │
                                              └────────────┬─────────────┘
                                                           │
                                                           │ /camera/image_filtered
                                                           │ sensor_msgs/Image
                                                           ▼
                                              ┌──────────────────────────┐
                                              │   ORB-SLAM3 Node (C++)   │
                                              │                          │
                                              │  - Extracts ORB features │
                                              │  - Runs SLAM pipeline    │
                                              │  - Outputs pose + map    │
                                              └──────────────────────────┘
```

## Requirements

### System
- Ubuntu 22.04/24.04
- ROS2 Jazzy Jalisco (or Humble)
- Python 3.10+
- OpenCV 4.2+

### Python Dependencies
```bash
pip install torch torchvision numpy opencv-python pillow
```

### Model Weights
Download EfficientSAM3 checkpoint from [HuggingFace](https://huggingface.co/Simon7108528/EfficientSAM3):
```bash
mkdir -p ~/weights
wget -O ~/weights/efficient_sam3_repvit_s.pt \
    https://huggingface.co/Simon7108528/EfficientSAM3/resolve/main/stage1_all_converted/efficient_sam3_repvit_s.pt
```

## Installation

### 1. Clone and Setup Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone packages (or copy from deployment)
# - efficientsam3_ros2/
# - efficientsam3_arm/
# - ros2_orb_slam3/
```

### 2. Install EfficientSAM3 ARM

```bash
cd ~/ros2_ws/src/efficientsam3_arm
pip install -e .
```

### 3. Build ROS2 Packages

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Option 1: Full Pipeline with Launch File

```bash
# Terminal 1: Launch the filter node
ros2 launch efficientsam3_ros2 slam_pipeline.launch.py \
    model_path:=/home/user/weights/efficient_sam3_repvit_s.pt

# Terminal 2: Start ORB-SLAM3 C++ node (with filtered images)
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=true

# Terminal 3: Start Python driver
ros2 run ros2_orb_slam3 mono_driver_filtered_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05 \
    -p use_filter:=true
```

### Option 2: Manual Node Startup

```bash
# Terminal 1: Start filter node
ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
    -p model_path:=/home/user/weights/efficient_sam3_repvit_s.pt \
    -p input_topic:=/camera/image_raw \
    -p output_topic:=/camera/image_filtered

# Terminal 2: Start ORB-SLAM3 (same as above)
# Terminal 3: Start driver (same as above)
```

### Option 3: Bypass Filter (Direct Mode)

To run without filtering (original behavior):

```bash
# ORB-SLAM3 without filter
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:=false

# Original driver
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args \
    -p settings_name:=EuRoC \
    -p image_seq:=sample_euroc_MH05
```

## Configuration

### Filter Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | required | Path to EfficientSAM3 checkpoint |
| `efficientsam3_path` | string | "" | Path to efficientsam3_arm package |
| `dynamic_classes` | list | [person, car, ...] | Object classes to filter |
| `confidence_threshold` | float | 0.3 | Detection confidence threshold |
| `masking_strategy` | string | "grayout" | How to handle masked regions |
| `input_topic` | string | /camera/image_raw | Input image topic |
| `output_topic` | string | /camera/image_filtered | Output filtered image topic |
| `publish_mask` | bool | true | Publish mask visualization |
| `device` | string | "auto" | Inference device (auto/cpu/cuda) |
| `process_every_n_frames` | int | 1 | Frame skip for performance |

### Masking Strategies

| Strategy | Description | Performance |
|----------|-------------|-------------|
| `blackout` | Set masked pixels to black | Fastest |
| `grayout` | Set masked pixels to gray | Fast (recommended) |
| `inpaint` | Fill with surrounding texture | Slow |
| `blur` | Apply Gaussian blur | Medium |

### Configuration File

Edit `config/dynamic_filter.yaml` for custom settings:

```yaml
dynamic_filter_node:
  ros__parameters:
    model_path: "/home/user/weights/efficient_sam3_repvit_s.pt"
    dynamic_classes:
      - "person"
      - "car"
      - "bicycle"
    confidence_threshold: 0.3
    masking_strategy: "grayout"
```

## Topics

### Subscribed
- `/camera/image_raw` (sensor_msgs/Image) - Raw camera images

### Published
- `/camera/image_filtered` (sensor_msgs/Image) - Filtered images
- `/dynamic_filter/mask` (sensor_msgs/Image) - Binary mask visualization
- `/dynamic_filter/detections` (std_msgs/String) - Detection info (JSON)

## Testing

### Test Filter Core (No ROS2 Required)

You can test the filtering logic locally on MacOS:

```bash
cd efficientsam3_ros2/efficientsam3_ros2
python filter_core.py \
    --image /path/to/test_image.jpg \
    --model /path/to/efficient_sam3_repvit_s.pt \
    --efficientsam3-path /path/to/efficientsam3_arm
```

### Test with ROS2 (VM)

```bash
# Check topics
ros2 topic list

# View filtered images
ros2 run rqt_image_view rqt_image_view /camera/image_filtered

# View mask
ros2 run rqt_image_view rqt_image_view /dynamic_filter/mask
```

## Performance Tips

1. **Use smaller model**: RepViT-S (4.72M params) is fastest
2. **Skip frames**: Set `process_every_n_frames: 2` to process every other frame
3. **Lower resolution**: Reduce input image size if possible
4. **Use GPU**: Set `device: cuda` if available
5. **Reduce classes**: Filter only essential dynamic objects

## Troubleshooting

### Model not loading
- Check model path is correct
- Ensure efficientsam3_arm is installed: `pip install -e .`
- Check PYTHONPATH includes efficientsam3_arm

### No detections
- Lower `confidence_threshold` (e.g., 0.2)
- Verify dynamic_classes include relevant objects
- Check input images are being received

### Low FPS
- Increase `process_every_n_frames`
- Use GPU if available
- Use smaller model (RepViT-S)

## License

Apache 2.0

## Acknowledgments

- [EfficientSAM3](https://github.com/SimonZeng7108/efficientsam3) by Chengxi Simon Zeng et al.
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) by Carlos Campos et al.
- [ROS2 ORB-SLAM3](https://github.com/Mechazo11/ros2_orb_slam3) by Azmyin Md. Kamal
