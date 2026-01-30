#!/bin/bash
#
# Deploy EfficientSAM3 + ORB-SLAM3 Pipeline to Ubuntu VM
#
# This script syncs the local development files to the VM for testing.
#
# Usage:
#   ./deploy_to_vm.sh <vm_user> <vm_ip> [vm_workspace_path]
#
# Example:
#   ./deploy_to_vm.sh ubuntu 192.168.1.100
#   ./deploy_to_vm.sh ubuntu 192.168.1.100 /home/ubuntu/ros2_ws/src
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check arguments
if [ $# -lt 2 ]; then
    echo -e "${RED}Error: Missing arguments${NC}"
    echo "Usage: $0 <vm_user> <vm_ip> [vm_workspace_path]"
    echo "Example: $0 ubuntu 192.168.1.100"
    exit 1
fi

VM_USER="$1"
VM_IP="$2"
VM_WS="${3:-/home/$VM_USER/ros2_ws/src}"

# Local paths (adjust if your structure is different)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_BASE="$(dirname "$(dirname "$SCRIPT_DIR")")"

EFFICIENTSAM3_ROS2_LOCAL="$LOCAL_BASE/efficientsam3_ros2"
EFFICIENTSAM3_ARM_LOCAL="$LOCAL_BASE/efficientsam3_arm"
ROS2_ORB_SLAM3_LOCAL="$LOCAL_BASE/ros2_orb_slam3"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  EfficientSAM3 + ORB-SLAM3 Deployment  ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "VM Target: ${VM_USER}@${VM_IP}:${VM_WS}"
echo ""

# Check if local directories exist
for dir in "$EFFICIENTSAM3_ROS2_LOCAL" "$EFFICIENTSAM3_ARM_LOCAL" "$ROS2_ORB_SLAM3_LOCAL"; do
    if [ ! -d "$dir" ]; then
        echo -e "${RED}Error: Directory not found: $dir${NC}"
        exit 1
    fi
done

# Create remote directory structure
echo -e "${YELLOW}Creating remote directory structure...${NC}"
ssh "${VM_USER}@${VM_IP}" "mkdir -p ${VM_WS}"

# Sync efficientsam3_ros2 package
echo -e "${YELLOW}Syncing efficientsam3_ros2...${NC}"
rsync -avz --progress \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    --exclude '*.egg-info' \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    "$EFFICIENTSAM3_ROS2_LOCAL/" \
    "${VM_USER}@${VM_IP}:${VM_WS}/efficientsam3_ros2/"

# Sync efficientsam3_arm library
echo -e "${YELLOW}Syncing efficientsam3_arm...${NC}"
rsync -avz --progress \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    --exclude '*.egg-info' \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    --exclude 'sam3' \
    --exclude 'stage1' \
    --exclude 'data' \
    --exclude '*.pt' \
    --exclude '*.pth' \
    "$EFFICIENTSAM3_ARM_LOCAL/" \
    "${VM_USER}@${VM_IP}:${VM_WS}/efficientsam3_arm/"

# Sync ros2_orb_slam3 package
echo -e "${YELLOW}Syncing ros2_orb_slam3...${NC}"
rsync -avz --progress \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    "$ROS2_ORB_SLAM3_LOCAL/" \
    "${VM_USER}@${VM_IP}:${VM_WS}/ros2_orb_slam3/"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Deployment Complete!                  ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Next steps on VM:"
echo ""
echo "1. SSH to VM:"
echo "   ssh ${VM_USER}@${VM_IP}"
echo ""
echo "2. Install Python dependencies:"
echo "   cd ${VM_WS}/efficientsam3_arm"
echo "   pip install -e ."
echo ""
echo "3. Download model weights:"
echo "   mkdir -p ~/weights"
echo "   # Download from HuggingFace: https://huggingface.co/Simon7108528/EfficientSAM3"
echo ""
echo "4. Build ROS2 workspace:"
echo "   cd ~/ros2_ws"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   colcon build --symlink-install"
echo ""
echo "5. Test the pipeline (see README for details)"
echo ""
