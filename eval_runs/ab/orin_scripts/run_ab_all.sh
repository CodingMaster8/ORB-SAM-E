#!/usr/bin/env bash
~/run_one_replay.sh ~/runs/bags/dyn1 baseline ~/runs/ab/dyn1_baseline
~/run_one_replay.sh ~/runs/bags/dyn1 filtered ~/runs/ab/dyn1_filtered
~/run_one_replay.sh ~/runs/bags/dyn2 baseline ~/runs/ab/dyn2_baseline
~/run_one_replay.sh ~/runs/bags/dyn2 filtered ~/runs/ab/dyn2_filtered
echo AB_ALL_DONE
