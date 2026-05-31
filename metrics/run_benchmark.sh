#!/bin/bash
#
# End-to-end benchmark for one ORB-SAM-E run on a TUM sequence.
#
# Captures: EfficientSAM3 timing/filtering stats, GPU/CPU/RAM usage, and the
# estimated trajectory, then evaluates ATE/RPE vs groundtruth and writes a
# results.json (+ resources.csv) into an output directory.
#
# Run it once with USE_FILTER=false (baseline) and once with USE_FILTER=true,
# then combine the two results.json with:  python -m slam_metrics.report ...
#
# Usage:
#   ./run_benchmark.sh <NAME> <SETTINGS> <DATASET_PATH> <USE_FILTER> [OUTDIR]
#
# Example (run from the metrics/ directory):
#   ./run_benchmark.sh fr3_walk_baseline TUM3 \
#       $HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz \
#       false results/fr3_walk_baseline
#
#   ./run_benchmark.sh fr3_walk_filtered TUM3 \
#       $HOME/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/tum/rgbd_dataset_freiburg3_walking_xyz \
#       true results/fr3_walk_filtered
#
set -euo pipefail

NAME=${1:?need a run NAME}
SETTINGS=${2:?need SETTINGS (TUM1/TUM2/TUM3)}
DATASET=${3:?need DATASET_PATH}
USE_FILTER=${4:-false}
OUTDIR=${5:-results/$NAME}

MODEL=${MODEL:-$HOME/weights/efficient_sam3_repvit_s.pt}
ESAM3_PATH=${ESAM3_PATH:-$HOME/ros2_ws/src/efficientsam3_arm}
DEVICE=${DEVICE:-cuda}
# Directory of this script (so `python -m slam_metrics...` works from anywhere).
METRICS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$OUTDIR"
OUTDIR="$(cd "$OUTDIR" && pwd)"
TRAJ="$OUTDIR/KeyFrameTrajectory.txt"
FILTER_STATS="$OUTDIR/filter_metrics.json"
RES_JSON="$OUTDIR/resources.json"
RES_CSV="$OUTDIR/resources.csv"
GROUNDTRUTH="$DATASET/groundtruth.txt"

echo "=== ORB-SAM-E benchmark: $NAME (filter=$USE_FILTER, device=$DEVICE) ==="
echo "Output: $OUTDIR"

pids=()
cleanup() {
    for p in "${pids[@]:-}"; do kill -INT "$p" 2>/dev/null || true; done
    sleep 2
    for p in "${pids[@]:-}"; do kill -9 "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

# 1) Resource monitor (background)
( cd "$METRICS_DIR" && python3 -m slam_metrics.monitor_cli \
    --out-json "$RES_JSON" --out-csv "$RES_CSV" --interval 0.5 ) &
MON_PID=$!; pids+=("$MON_PID")
sleep 1

# 2) Filter node (only when filtering), writes its stats on shutdown
if [ "$USE_FILTER" = "true" ]; then
    ros2 run efficientsam3_ros2 dynamic_filter_node --ros-args \
        -p model_path:="$MODEL" \
        -p efficientsam3_path:="$ESAM3_PATH" \
        -p device:="$DEVICE" \
        -p metrics_output:="$FILTER_STATS" &
    FILTER_PID=$!; pids+=("$FILTER_PID")
    sleep 5
fi

# 3) ORB-SLAM3 node, saves keyframe trajectory on shutdown
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args \
    -p node_name_arg:=mono_slam_cpp \
    -p use_filtered_images:="$USE_FILTER" \
    -p trajectory_output:="$TRAJ" &
SLAM_PID=$!; pids+=("$SLAM_PID")
sleep 3

# 4) TUM driver (foreground; blocks until the sequence finishes)
ros2 run ros2_orb_slam3 tum_driver_node.py --ros-args \
    -p settings_name:="$SETTINGS" \
    -p dataset_path:="$DATASET" \
    -p use_filter:="$USE_FILTER"

echo "=== Playback done; shutting nodes down so trajectory/metrics flush ==="
# SIGINT the SLAM (and filter) nodes so their shutdown writes files.
kill -INT "$SLAM_PID" 2>/dev/null || true
[ "$USE_FILTER" = "true" ] && kill -INT "${FILTER_PID:-}" 2>/dev/null || true
sleep 4
kill -INT "$MON_PID" 2>/dev/null || true
sleep 2

# 5) Assemble results.json (+ ATE/RPE if groundtruth present)
EXTRA=()
[ "$USE_FILTER" = "true" ] && EXTRA+=(--filter-stats "$FILTER_STATS")
[ -f "$GROUNDTRUTH" ] && EXTRA+=(--estimate "$TRAJ" --groundtruth "$GROUNDTRUTH" --mono)

( cd "$METRICS_DIR" && python3 -m slam_metrics.assemble \
    --name "$NAME" \
    --config use_filter="$USE_FILTER" settings="$SETTINGS" \
             device="$DEVICE" model="$(basename "$MODEL")" \
             dataset="$(basename "$DATASET")" \
    --resources "$RES_JSON" \
    "${EXTRA[@]}" \
    --out "$OUTDIR/results.json" )

echo "=== Done. Results in $OUTDIR/results.json ==="
