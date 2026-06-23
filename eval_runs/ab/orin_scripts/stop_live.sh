#!/usr/bin/env bash
# Stop the live ORB-SLAM3 pipeline GRACEFULLY so trajectory_output gets written.
# (SIGINT -> ORB-SLAM3 Shutdown() runs final BA + SaveKeyFrameTrajectoryTUM; needs >6s.)
PIDS=$(pgrep -f "ros2 launch ros2_orb_slam3 orbsame_live")
[ -z "$PIDS" ] && PIDS=$(pgrep -f "mono_node_cpp|live_camera_driver_node")
if [ -z "$PIDS" ]; then echo "live pipeline not running"; exit 0; fi
echo "SIGINT -> $PIDS"
kill -INT $PIDS 2>/dev/null
for i in $(seq 1 120); do
  pgrep -f "mono_node_cpp" >/dev/null || { echo "stopped cleanly after ${i}s"; ls -la ~/live_traj.txt 2>/dev/null; exit 0; }
  sleep 1
done
echo "still alive after 120s, force killing"
pkill -9 -f "mono_node_cpp|live_camera_driver_node|orbsame_live" 2>/dev/null
