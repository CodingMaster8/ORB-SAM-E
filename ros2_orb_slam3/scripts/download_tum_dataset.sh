#!/bin/bash
#
# Download TUM RGB-D Dataset sequences
# 
# Usage:
#   ./download_tum_dataset.sh [sequence_name] [output_dir]
#
# Available sequences (freiburg1 - fr1, freiburg2 - fr2, freiburg3 - fr3):
#   fr1/desk, fr1/desk2, fr1/room, fr1/xyz, fr1/rpy, fr1/plant, fr1/teddy
#   fr2/desk, fr2/xyz, fr2/rpy, fr2/pioneer_360, fr2/pioneer_slam
#   fr3/long_office_household, fr3/nostructure_notexture_far
#
# Examples:
#   ./download_tum_dataset.sh fr1/desk ~/datasets/tum
#   ./download_tum_dataset.sh fr3/long_office_household ~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum

set -e

# Default values
DEFAULT_OUTPUT_DIR="$HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum"
TUM_BASE_URL="https://cvg.cit.tum.de/rgbd/dataset"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_usage() {
    echo "TUM RGB-D Dataset Downloader"
    echo ""
    echo "Usage: $0 [sequence_name] [output_dir]"
    echo ""
    echo "Available sequences:"
    echo ""
    echo "  Freiburg 1 (fr1) - Use with TUM1.yaml config:"
    echo "    fr1/xyz           - Simple camera motion for calibration"
    echo "    fr1/rpy           - Rotational motion"
    echo "    fr1/desk          - Office desk scene (recommended for testing)"
    echo "    fr1/desk2         - Office desk, different trajectory"
    echo "    fr1/room          - Full room sequence"
    echo "    fr1/plant         - Plant scene"
    echo "    fr1/teddy         - Teddy bear scene"
    echo ""
    echo "  Freiburg 2 (fr2) - Use with TUM2.yaml config:"
    echo "    fr2/xyz           - Simple motion"
    echo "    fr2/rpy           - Rotational motion"
    echo "    fr2/desk          - Office desk"
    echo "    fr2/pioneer_360   - Pioneer robot, 360 turn"
    echo "    fr2/pioneer_slam  - Pioneer robot, SLAM sequence"
    echo ""
    echo "  Freiburg 3 (fr3) - Use with TUM3.yaml config:"
    echo "    fr3/long_office_household  - Long office sequence (recommended)"
    echo "    fr3/nostructure_notexture_far"
    echo "    fr3/sitting_xyz"
    echo "    fr3/walking_xyz   - Has walking people (good for dynamic filtering!)"
    echo "    fr3/walking_rpy   - Has walking people"
    echo "    fr3/sitting_halfsphere"
    echo ""
    echo "Examples:"
    echo "  $0 fr1/desk"
    echo "  $0 fr3/walking_xyz ~/datasets/tum"
    echo ""
    echo "Output directory defaults to: $DEFAULT_OUTPUT_DIR"
}

# Map sequence name to TUM download filename
get_tum_filename() {
    local seq=$1
    case $seq in
        # Freiburg 1
        fr1/xyz) echo "rgbd_dataset_freiburg1_xyz" ;;
        fr1/rpy) echo "rgbd_dataset_freiburg1_rpy" ;;
        fr1/desk) echo "rgbd_dataset_freiburg1_desk" ;;
        fr1/desk2) echo "rgbd_dataset_freiburg1_desk2" ;;
        fr1/room) echo "rgbd_dataset_freiburg1_room" ;;
        fr1/plant) echo "rgbd_dataset_freiburg1_plant" ;;
        fr1/teddy) echo "rgbd_dataset_freiburg1_teddy" ;;
        
        # Freiburg 2
        fr2/xyz) echo "rgbd_dataset_freiburg2_xyz" ;;
        fr2/rpy) echo "rgbd_dataset_freiburg2_rpy" ;;
        fr2/desk) echo "rgbd_dataset_freiburg2_desk" ;;
        fr2/pioneer_360) echo "rgbd_dataset_freiburg2_pioneer_360" ;;
        fr2/pioneer_slam) echo "rgbd_dataset_freiburg2_pioneer_slam" ;;
        fr2/pioneer_slam2) echo "rgbd_dataset_freiburg2_pioneer_slam2" ;;
        fr2/pioneer_slam3) echo "rgbd_dataset_freiburg2_pioneer_slam3" ;;
        
        # Freiburg 3
        fr3/long_office_household) echo "rgbd_dataset_freiburg3_long_office_household" ;;
        fr3/nostructure_notexture_far) echo "rgbd_dataset_freiburg3_nostructure_notexture_far" ;;
        fr3/sitting_xyz) echo "rgbd_dataset_freiburg3_sitting_xyz" ;;
        fr3/sitting_halfsphere) echo "rgbd_dataset_freiburg3_sitting_halfsphere" ;;
        fr3/walking_xyz) echo "rgbd_dataset_freiburg3_walking_xyz" ;;
        fr3/walking_rpy) echo "rgbd_dataset_freiburg3_walking_rpy" ;;
        fr3/walking_halfsphere) echo "rgbd_dataset_freiburg3_walking_halfsphere" ;;
        
        *) echo "" ;;
    esac
}

# Get the settings file to use for a sequence
get_settings_file() {
    local seq=$1
    case $seq in
        fr1/*) echo "TUM1" ;;
        fr2/*) echo "TUM2" ;;
        fr3/*) echo "TUM3" ;;
        *) echo "TUM1" ;;
    esac
}

# Main script
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    show_usage
    exit 0
fi

if [ -z "$1" ]; then
    print_info "No sequence specified. Downloading recommended test sequence: fr1/desk"
    SEQUENCE="fr1/desk"
else
    SEQUENCE="$1"
fi

OUTPUT_DIR="${2:-$DEFAULT_OUTPUT_DIR}"

# Get TUM filename
TUM_FILENAME=$(get_tum_filename "$SEQUENCE")
if [ -z "$TUM_FILENAME" ]; then
    print_error "Unknown sequence: $SEQUENCE"
    echo ""
    show_usage
    exit 1
fi

SETTINGS_FILE=$(get_settings_file "$SEQUENCE")

print_info "Sequence: $SEQUENCE"
print_info "TUM filename: $TUM_FILENAME"
print_info "Use with settings: $SETTINGS_FILE.yaml"
print_info "Output directory: $OUTPUT_DIR"

# Create output directory
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

# Check if already downloaded
FINAL_DIR="$OUTPUT_DIR/$TUM_FILENAME"
if [ -d "$FINAL_DIR" ] && [ -f "$FINAL_DIR/rgb.txt" ]; then
    print_warn "Dataset already exists at $FINAL_DIR"
    print_info "To re-download, delete the directory first"
    exit 0
fi

# Download URL - extract freiburg number (1, 2, or 3) from sequence name
FREIBURG_NUM=$(echo "$SEQUENCE" | sed 's/fr\([0-9]\).*/\1/')
DOWNLOAD_URL="$TUM_BASE_URL/freiburg${FREIBURG_NUM}/${TUM_FILENAME}.tgz"

print_info "Downloading from: $DOWNLOAD_URL"
print_info "This may take a few minutes..."

# Download
TGZ_FILE="${TUM_FILENAME}.tgz"
if command -v wget &> /dev/null; then
    wget -O "$TGZ_FILE" "$DOWNLOAD_URL"
elif command -v curl &> /dev/null; then
    curl -L -o "$TGZ_FILE" "$DOWNLOAD_URL"
else
    print_error "Neither wget nor curl found. Please install one of them."
    exit 1
fi

# Extract
print_info "Extracting..."
tar -xzf "$TGZ_FILE"
rm "$TGZ_FILE"

# Verify
if [ -d "$FINAL_DIR" ] && [ -f "$FINAL_DIR/rgb.txt" ]; then
    print_info "Download complete!"
    echo ""
    echo "Dataset location: $FINAL_DIR"
    echo ""
    echo "To run ORB-SLAM3 with this dataset:"
    echo ""
    echo "  # Terminal 1 - ORB-SLAM3 C++ node"
    echo "  ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp"
    echo ""
    echo "  # Terminal 2 - TUM driver"
    echo "  ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \\"
    echo "      -p settings_name:=$SETTINGS_FILE \\"
    echo "      -p dataset_path:=$FINAL_DIR"
    echo ""
    echo "To run WITH EfficientSAM3 dynamic filtering:"
    echo ""
    echo "  # Terminal 1 - Filter node"
    echo "  ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \\"
    echo "      -p model_path:=\$HOME/weights/efficient_sam3_repvit_s.pt"
    echo ""
    echo "  # Terminal 2 - ORB-SLAM3 (using filtered images)"
    echo "  ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \\"
    echo "      -p node_name_arg:=mono_slam_cpp \\"
    echo "      -p use_filtered_images:=true"
    echo ""
    echo "  # Terminal 3 - TUM driver with filter"
    echo "  ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \\"
    echo "      -p settings_name:=$SETTINGS_FILE \\"
    echo "      -p dataset_path:=$FINAL_DIR \\"
    echo "      -p use_filter:=true"
    echo ""
else
    print_error "Download or extraction failed"
    exit 1
fi
