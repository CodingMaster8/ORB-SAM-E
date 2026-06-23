#!/usr/bin/env bash
#
# record_eval_run.sh — ON-ROBOT (Orin) evaluation capture for ORB-SAM-E.
#
# Records everything needed to (a) compute live ATE/RPE vs ground truth and
# (b) re-run the SAME sequence offline through baseline / filtered pipelines:
#   * rosbag2: /cam_1/image /orbslam3/pose /optitrack/rigid_body /odom /tf /tf_static
#   * TUM trajectories (est/gt/odom) via slam_metrics.live_eval_logger
#   * resource samples (GPU/CPU/RAM) via slam_metrics.monitor_cli
#
# Usage (Orin):
#   ./record_eval_run.sh <run_name> [duration_s]
#   # duration omitted -> record until Ctrl-C
#
# Prereq: the live SLAM pipeline is already running (live_test.sh / orbsame_live.launch.py).
set -u

RUN_NAME="${1:?usage: record_eval_run.sh <run_name> [duration_s]}"
DURATION="${2:-0}"
OUT_DIR="$HOME/eval_runs/$RUN_NAME"
METRICS_DIR="$(cd "$(dirname "$0")" && pwd)"

# Bridge DDS env (required for ANY ROS command on the Orin)
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///home/jetson/cyclonedds-orin.xml}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="$METRICS_DIR:${PYTHONPATH:-}"

mkdir -p "$OUT_DIR"
echo "[record_eval_run] output: $OUT_DIR"

ros2 bag record -o "$OUT_DIR/bag" \
  /cam_1/image /orbslam3/pose /optitrack/rigid_body /odom /tf /tf_static \
  > "$OUT_DIR/bag_record.log" 2>&1 &
BAG_PID=$!

python3 -m slam_metrics.live_eval_logger --out-dir "$OUT_DIR" \
  > "$OUT_DIR/logger.log" 2>&1 &
LOGGER_PID=$!

python3 -m slam_metrics.monitor_cli \
  --out-json "$OUT_DIR/resources.json" --out-csv "$OUT_DIR/resources.csv" \
  > "$OUT_DIR/monitor.log" 2>&1 &
MONITOR_PID=$!

cleanup() {
  echo "[record_eval_run] stopping..."
  kill -INT "$BAG_PID" "$LOGGER_PID" "$MONITOR_PID" 2>/dev/null
  # Give rosbag2 and the logger time to flush/close cleanly.
  sleep 8
  kill -9 "$BAG_PID" "$LOGGER_PID" "$MONITOR_PID" 2>/dev/null
  echo "[record_eval_run] done -> $OUT_DIR"
  echo "Evaluate:  python3 -m slam_metrics.trajectory_eval $OUT_DIR/est.tum $OUT_DIR/gt.tum --rpe-delta 1.0"
}
trap cleanup INT TERM

if [ "$DURATION" -gt 0 ] 2>/dev/null; then
  echo "[record_eval_run] recording for ${DURATION}s (Ctrl-C to stop early)"
  sleep "$DURATION"
  cleanup
else
  echo "[record_eval_run] recording until Ctrl-C"
  wait "$BAG_PID"
fi
