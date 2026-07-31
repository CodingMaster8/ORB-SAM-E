#!/usr/bin/env python3
"""
TUM RGB-D dataset evaluation launch (issue #16).

Starts:
  * rgbd_node_cpp          : the ORB-SLAM3 C++ RGB-D node (headless; pairs the
                             rgb + depth streams by header stamp and calls
                             TrackRGBD; publishes /orbslam3/pose).
  * tum_rgbd_driver_node   : reads a TUM RGB-D sequence, associates
                             rgb.txt/depth.txt (max diff 0.02 s), performs the
                             settings handshake and publishes the paired frames
                             paced by the original timestamps.
  * dynamic_filter_node    : (only with use_filter:=true) EfficientSAM3
                             dynamic-object filter in its torch venv. ONLY the
                             RGB stream goes through the filter; depth is
                             published raw by the driver.

RGB-D mode has metric scale and per-frame poses, so trajectory_output receives
the FULL frame trajectory (SaveTrajectoryTUM) - the quantity published
dynamic-SLAM baselines (DynaSLAM/DS-SLAM) report ATE RMSE on.

Baseline (no filter):
  ros2 launch ros2_orb_slam3 tum_rgbd_eval.launch.py \
      settings_name:=TUM3 \
      dataset_path:=$HOME/datasets/rgbd_dataset_freiburg3_walking_xyz \
      trajectory_output:=$HOME/rgbd_run.txt

Full ORB-SAM-E pipeline:
  ros2 launch ros2_orb_slam3 tum_rgbd_eval.launch.py \
      use_filter:=true \
      settings_name:=TUM3 \
      dataset_path:=$HOME/datasets/rgbd_dataset_freiburg3_walking_xyz \
      model_path:=$HOME/weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt \
      trajectory_output:=$HOME/rgbd_run_filtered.txt
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser("~")


def generate_launch_description():
    settings_name = LaunchConfiguration("settings_name")
    dataset_path = LaunchConfiguration("dataset_path")
    playback_speed = LaunchConfiguration("playback_speed")
    use_viewer = LaunchConfiguration("use_viewer")
    pose_topic = LaunchConfiguration("pose_topic")
    trajectory_output = LaunchConfiguration("trajectory_output")
    keyframe_trajectory_output = LaunchConfiguration("keyframe_trajectory_output")
    map_points_output = LaunchConfiguration("map_points_output")
    use_filter = LaunchConfiguration("use_filter")
    filter_python = LaunchConfiguration("filter_python")
    filter_pkg_root = LaunchConfiguration("filter_pkg_root")
    esam3_path = LaunchConfiguration("esam3_path")
    model_path = LaunchConfiguration("model_path")
    filtered_topic = LaunchConfiguration("filtered_topic")
    filter_prompts = LaunchConfiguration("filter_prompts")
    filter_threshold = LaunchConfiguration("filter_threshold")
    filter_fp16 = LaunchConfiguration("filter_fp16")
    filter_every_n = LaunchConfiguration("filter_every_n")
    filter_metrics_output = LaunchConfiguration("filter_metrics_output")

    return LaunchDescription([
        DeclareLaunchArgument("settings_name", default_value="TUM3",
                              description="Name of the .yaml in orb_slam3/config/RGB-D/ (TUM1/TUM2/TUM3)"),
        DeclareLaunchArgument("dataset_path", default_value="",
                              description="Full path to the TUM RGB-D sequence folder"),
        DeclareLaunchArgument("playback_speed", default_value="1.0",
                              description="Dataset playback speed multiplier (1.0 = real time)"),
        DeclareLaunchArgument("use_viewer", default_value="false",
                              description="Pangolin viewer (needs a display); keep false on the pod"),
        DeclareLaunchArgument("pose_topic", default_value="/orbslam3/pose"),
        DeclareLaunchArgument("trajectory_output", default_value="",
                              description="If set, FULL frame trajectory (TUM format) saved here on shutdown"),
        DeclareLaunchArgument("keyframe_trajectory_output", default_value="",
                              description="If set, keyframe-only trajectory (TUM format) saved here on shutdown"),
        DeclareLaunchArgument("map_points_output", default_value="",
                              description="If set, active-map 3D points saved here (ASCII PLY) on shutdown"),

        # ---- EfficientSAM3 filter pipeline (RGB stream only) ----
        DeclareLaunchArgument("use_filter", default_value="false",
                              description="Enable the EfficientSAM3 dynamic-object filter on the RGB stream"),
        DeclareLaunchArgument("filter_python",
                              default_value=os.path.join(HOME, "venvs/esam3/bin/python3"),
                              description="Python interpreter of the torch venv for the filter node"),
        DeclareLaunchArgument("filter_pkg_root",
                              default_value=os.path.join(HOME, "ros2_ws/src/ORB-SAM-E/efficientsam3_ros2"),
                              description="Root dir containing the efficientsam3_ros2 python package"),
        DeclareLaunchArgument("esam3_path",
                              default_value=os.path.join(HOME, "ros2_ws/src/ORB-SAM-E/efficientsam3_arm"),
                              description="Path to the efficientsam3_arm package (added to sys.path)"),
        DeclareLaunchArgument("model_path",
                              default_value=os.path.join(
                                  HOME, "weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt"),
                              description="EfficientSAM3 checkpoint"),
        DeclareLaunchArgument("filtered_topic", default_value="/camera/image_filtered"),
        DeclareLaunchArgument("filter_prompts", default_value="person",
                              description="Comma-separated text prompts"),
        DeclareLaunchArgument("filter_threshold", default_value="0.3"),
        DeclareLaunchArgument("filter_fp16", default_value="true"),
        DeclareLaunchArgument("filter_every_n", default_value="2",
                              description="Run inference every N frames (mask reused in between)"),
        DeclareLaunchArgument("filter_metrics_output", default_value="",
                              description="If set, filter timing/detection stats JSON saved here on shutdown"),

        # ORB-SLAM3 C++ RGB-D node (headless)
        Node(
            package="ros2_orb_slam3",
            executable="rgbd_node_cpp",
            name="rgbd_slam_cpp",
            output="screen",
            parameters=[{
                "node_name_arg": "rgbd_slam_cpp",
                "use_viewer": use_viewer,
                "pose_topic": pose_topic,
                "trajectory_output": trajectory_output,
                "keyframe_trajectory_output": keyframe_trajectory_output,
                "map_points_output": map_points_output,
                "use_filtered_images": use_filter,
            }],
        ),

        # TUM RGB-D dataset driver (handshake + paced rgb/depth playback)
        Node(
            package="ros2_orb_slam3",
            executable="tum_rgbd_driver_node.py",
            name="tum_rgbd_driver",
            output="screen",
            parameters=[{
                "settings_name": settings_name,
                "dataset_path": dataset_path,
                "use_filter": use_filter,
                "playback_speed": playback_speed,
            }],
        ),

        # EfficientSAM3 dynamic filter, executed with the venv python (torch
        # lives there, not in the system interpreter ROS launch would use).
        # Subscribes to the driver's RGB output, republishes filtered frames
        # with the header (dataset timestamp) preserved.
        ExecuteProcess(
            condition=IfCondition(use_filter),
            cmd=[
                filter_python, "-m", "efficientsam3_ros2.dynamic_filter_node",
                "--ros-args",
                "-p", ["model_path:=", model_path],
                "-p", ["efficientsam3_path:=", esam3_path],
                "-p", "input_topic:=/camera/image_raw",
                "-p", ["output_topic:=", filtered_topic],
                "-p", ["dynamic_prompts:=[", filter_prompts, "]"],
                "-p", ["confidence_threshold:=", filter_threshold],
                "-p", ["use_fp16:=", filter_fp16],
                "-p", ["process_every_n_frames:=", filter_every_n],
                "-p", ["metrics_output:=", filter_metrics_output],
                "-p", "device:=cuda",
            ],
            # Prepend (not replace): rclpy etc. still come from the ROS PYTHONPATH.
            additional_env={
                "PYTHONPATH": [
                    filter_pkg_root, ":",
                    EnvironmentVariable("PYTHONPATH", default_value=""),
                ]
            },
            output="screen",
            name="dynamic_filter_node",
        ),
    ])
