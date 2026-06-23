#!/usr/bin/env bash
# Un replay A/B completo: lanzar pipeline -> reproducir bag (rate 0.5) -> guardar -> limpiar.
# Uso: run_ab_replay.sh <bag_dir> <baseline|filtered> <out_prefix>
BAG=$1; MODE=$2; PREFIX=$3
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash

if [ "$MODE" = "filtered" ]; then
  setsid nohup ~/replay_filtered.sh ${PREFIX}_traj.txt ${PREFIX}_metrics.json ${PREFIX}_map.ply > ${PREFIX}_launch.log 2>&1 < /dev/null &
  for i in $(seq 1 40); do grep -aq "Model ready" ${PREFIX}_launch.log && break; sleep 3; done
  sleep 3
else
  setsid nohup ~/replay_baseline.sh ${PREFIX}_traj.txt ${PREFIX}_map.ply > ${PREFIX}_launch.log 2>&1 < /dev/null &
  sleep 15
fi

ros2 bag play "$BAG" --topics /cam_1/image --remap /cam_1/image:=/replay/image --rate 1.0 > ${PREFIX}_play.log 2>&1
sleep 3
ros2 topic pub --once /orbslam3/save_outputs std_msgs/msg/String "{data: save}" > /dev/null 2>&1
sleep 12

pkill -INT -f "dynamic_filter_nod" 2>/dev/null
sleep 3
pkill -9 -f "orbsame_liv" 2>/dev/null
pkill -9 -f "mono_node_cp" 2>/dev/null
pkill -9 -f "live_camera_drive" 2>/dev/null
pkill -9 -f "dynamic_filter_nod" 2>/dev/null
sleep 2
echo "REPLAY_DONE $PREFIX"
ls -la ${PREFIX}_traj.txt 2>/dev/null || echo "WARN: sin trayectoria"
