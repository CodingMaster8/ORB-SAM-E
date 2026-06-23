#!/usr/bin/env python3
"""
Live ON-ROBOT launch for ORB-SLAM3 monocular on the JetAuto (Orin, headless).

Starts:
  * mono_node_cpp        : the ORB-SLAM3 C++ node (headless; publishes /orbslam3/pose)
  * live_camera_driver   : forwards the live camera (/cam_1/image) into the node and
                           performs the settings handshake.
  * dynamic_filter_node  : (only with use_filter:=true) EfficientSAM3 dynamic-object
                           filter running inside its torch venv; images then flow
                           camera -> filter -> SLAM and the driver only handshakes.

This is the real-camera counterpart of the dataset (TUM/EuRoC) workflow. It does NOT
touch the robot's map/odom TF or Nav2: ORB-SLAM3 publishes its pose on its own topic
in its own frames (orb_map/orb_cam) for recording/evaluation.

Baseline (no filter):
  ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
      settings_name:=JetAuto camera_topic:=/cam_1/image \
      trajectory_output:=$HOME/orbslam_run.txt

Full ORB-SAM-E pipeline (EfficientSAM3.1 config validated on the Orin):
  ros2 launch ros2_orb_slam3 orbsame_live.launch.py \
      use_filter:=true \
      model_path:=$HOME/weights/efficient_sam3p1_repvit_s_mobileclip_s0_ctx16.pt \
      trajectory_output:=$HOME/orbslam_run_filtered.txt

Set use_viewer:=true only when running with a display (e.g. laptop + X forwarding).
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
    camera_topic = LaunchConfiguration("camera_topic")
    use_viewer = LaunchConfiguration("use_viewer")
    pose_topic = LaunchConfiguration("pose_topic")
    trajectory_output = LaunchConfiguration("trajectory_output")
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
        DeclareLaunchArgument("settings_name", default_value="JetAuto",
                              description="Name of the .yaml in orb_slam3/config/Monocular/"),
        DeclareLaunchArgument("camera_topic", default_value="/cam_1/image",
                              description="Live camera image topic"),
        DeclareLaunchArgument("use_viewer", default_value="false",
                              description="Pangolin viewer (needs a display); keep false on Orin"),
        DeclareLaunchArgument("pose_topic", default_value="/orbslam3/pose"),
        DeclareLaunchArgument("trajectory_output", default_value="",
                              description="If set, TUM keyframe trajectory saved here on shutdown"),
        DeclareLaunchArgument("map_points_output", default_value="",
                              description="If set, active-map 3D points saved here (ASCII PLY) on shutdown"),

        # ---- EfficientSAM3 filter pipeline (Fase 7) ----
        DeclareLaunchArgument("use_filter", default_value="false",
                              description="Enable the EfficientSAM3 dynamic-object filter pipeline"),
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
        # Validated on the Orin with the sam3p1 checkpoint: 'person' alone, thr 0.3, fp16.
        DeclareLaunchArgument("filter_prompts", default_value="person",
                              description="Comma-separated text prompts"),
        DeclareLaunchArgument("filter_threshold", default_value="0.3"),
        DeclareLaunchArgument("filter_fp16", default_value="true"),
        DeclareLaunchArgument("filter_every_n", default_value="2",
                              description="Run inference every N frames (mask reused in between)"),
        DeclareLaunchArgument("filter_metrics_output", default_value="",
                              description="If set, filter timing/detection stats JSON saved here on shutdown"),

        # ORB-SLAM3 C++ node (headless)
        Node(
            package="ros2_orb_slam3",
            executable="mono_node_cpp",
            name="mono_slam_cpp",
            output="screen",
            parameters=[{
                "node_name_arg": "mono_slam_cpp",
                "use_viewer": use_viewer,
                "pose_topic": pose_topic,
                "trajectory_output": trajectory_output,
                "map_points_output": map_points_output,
                "use_filtered_images": use_filter,
            }],
        ),

        # Live camera driver (handshake; forwards frames only when the filter is off)
        Node(
            package="ros2_orb_slam3",
            executable="live_camera_driver_node.py",
            name="live_camera_driver",
            output="screen",
            parameters=[{
                "settings_name": settings_name,
                "camera_topic": camera_topic,
                "use_filter": use_filter,
            }],
        ),

        # EfficientSAM3 dynamic filter, executed with the venv python (torch lives
        # there, not in the system interpreter ROS launch would normally use).
        ExecuteProcess(
            condition=IfCondition(use_filter),
            cmd=[
                filter_python, "-m", "efficientsam3_ros2.dynamic_filter_node",
                "--ros-args",
                "-p", ["model_path:=", model_path],
                "-p", ["efficientsam3_path:=", esam3_path],
                "-p", ["input_topic:=", camera_topic],
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
