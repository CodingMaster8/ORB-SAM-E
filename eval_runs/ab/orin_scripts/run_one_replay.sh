#!/usr/bin/env bash
# run_one_replay.sh <bag> <baseline|filtered> <out_prefix>
# Graba el stream de /orbslam3/pose a un bag (robusto a resets del mono SLAM)
# ademas de la trayectoria KF y el mapa via save trigger.
BAG=$1; MODE=$2; PREFIX=$3
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

pkill -9 -f "orbsame_liv[e]" 2>/dev/null; pkill -9 -f "mono_node_cp[p]" 2>/dev/null
pkill -9 -f "dynamic_filter_nod[e]" 2>/dev/null; sleep 2

if [ "$MODE" = "filtered" ]; then
  setsid nohup ~/replay_filtered.sh ${PREFIX}_traj.txt ${PREFIX}_metrics.json ${PREFIX}_map.ply > ${PREFIX}_launch.log 2>&1 < /dev/null &
  for i in $(seq 1 40); do grep -aq "Model ready" ${PREFIX}_launch.log && break; sleep 3; done
  grep -aq "Model ready" ${PREFIX}_launch.log || { echo "ERROR_MODEL ${PREFIX}"; exit 1; }
  sleep 3
else
  setsid nohup ~/replay_baseline.sh ${PREFIX}_traj.txt ${PREFIX}_map.ply > ${PREFIX}_launch.log 2>&1 < /dev/null &
  sleep 15
fi

# grabar el stream de poses durante el replay
rm -rf ${PREFIX}_posebag
setsid nohup ros2 bag record -o ${PREFIX}_posebag /orbslam3/pose > ${PREFIX}_posebag.log 2>&1 < /dev/null &
sleep 3

ros2 bag play "$BAG" --topics /cam_1/image --remap /cam_1/image:=/replay/image --rate 1.0 > ${PREFIX}_play.log 2>&1
sleep 3
ros2 topic pub --once /orbslam3/save_outputs std_msgs/msg/String "{data: save}" > /dev/null 2>&1
sleep 15

pkill -INT -f "bag recor[d]" 2>/dev/null; sleep 3
pkill -INT -f "dynamic_filter_nod[e]" 2>/dev/null; sleep 3
pkill -9 -f "orbsame_liv[e]" 2>/dev/null; pkill -9 -f "mono_node_cp[p]" 2>/dev/null
pkill -9 -f "live_camera_drive[r]" 2>/dev/null; pkill -9 -f "dynamic_filter_nod[e]" 2>/dev/null
sleep 2

KF=$(wc -l < ${PREFIX}_traj.txt 2>/dev/null || echo 0)
RESETS=$(grep -ac "New Map created" ${PREFIX}_launch.log)
echo "REPLAY_DONE $PREFIX kf=$KF resets=$RESETS"
