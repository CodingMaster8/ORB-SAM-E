#!/usr/bin/env bash
# Import + single-frame inference test for EfficientSAM3 (run inside esam3 venv).
set -e
source ~/venvs/esam3/bin/activate
ESAM_PATH=~/ros2_ws/src/ORB-SAM-E/efficientsam3_arm
echo "=== import test ==="
PYTHONPATH=$ESAM_PATH python - <<PY
import torch
from efficientsam3_arm import build_efficientsam3_image_model
from efficientsam3_arm.model.sam3_image_processor import Sam3Processor
print("imports OK | torch", torch.__version__, "| cuda", torch.cuda.is_available())
PY
echo "=== single-frame inference (filter_core CLI) ==="
python ~/ros2_ws/src/ORB-SAM-E/efficientsam3_ros2/efficientsam3_ros2/filter_core.py \
  --image ~/frame.jpg \
  --model ~/weights/efficient_sam3_repvit-m0_9_mobileclip_s1.pth \
  --efficientsam3-path $ESAM_PATH \
  --output ~/frame_filtered.jpg --no-show
echo "TEST_ESAM3_DONE"
