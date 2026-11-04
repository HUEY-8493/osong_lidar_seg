#!/usr/bin/env python3
"""
================================================================================
LiDAR Crane Bringup 전용 런치 파일 (lidar_crane_bringup.launch.py)
================================================================================
- 크레인 모드로 lidar_bringup.launch.py를 자동 실행합니다.
- TF: config/lidar_tf_crane.yaml 자동 적용
- Perception: cx128s2_bev_perception_crane.yaml 자동 적용
- Bridge: guardian_lidar_bridge (mode:=crane) 자동 적용
- RViz2: rviz/lidar_crane.rviz (훅/빔/TF 시각화) 자동 적용
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory("lidar_bringup")
    main_launch = os.path.join(pkg_bringup, "launch", "lidar_bringup.launch.py")

    pkg_perc = get_package_share_directory("cx128s2_bev_perception")
    default_perception_params = os.path.join(pkg_perc, "config", "cx128s2_bev_perception_crane.yaml")
    default_tf_config = "config/lidar_tf_crane.yaml"
    default_rviz_config = "rviz/lidar_crane_work.rviz"
    default_overlay = os.path.join(pkg_perc, "config", "crane_boundary_clustering_experimental.yaml")

    declared_arguments = [
        DeclareLaunchArgument("use_lidar_driver", default_value="true", description="Launch external LiDAR driver ('true'/'false')"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz2 for crane visualization ('true'/'false')"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time ('true'/'false')"),
        DeclareLaunchArgument("publish_tf", default_value="true", description="Publish static crane TF ('true'/'false')"),
        DeclareLaunchArgument("tf_config", default_value=default_tf_config, description="YAML TF configuration file"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config, description="RViz configuration file"),
        DeclareLaunchArgument("perception_params_file", default_value=default_perception_params, description="Perception params file"),
        DeclareLaunchArgument("perception_params_overlay_file", default_value=default_overlay, description="Optional experimental YAML overlay"),
        DeclareLaunchArgument("perception_input_topic", default_value="", description="Optional perception input topic override"),
        DeclareLaunchArgument("perception_target_frame", default_value="", description="Optional perception target frame override"),
        DeclareLaunchArgument("guardian_bridge_enabled", default_value="true", description="Start Guardian bridge ('true'/'false')"),
        DeclareLaunchArgument("guardian_bridge_mode", default_value="crane", description="Guardian bridge mode ('crane')"),
        DeclareLaunchArgument("guardian_bridge_manager_url", default_value="http://127.0.0.1:8765/api/internal/bridge", description="Bridge endpoint"),
        DeclareLaunchArgument("guardian_recorder_enabled", default_value="false", description="Start PCD recorder ('true'/'false')"),
    ]

    forward_args = [
        "use_lidar_driver",
        "use_rviz",
        "use_sim_time",
        "publish_tf",
        "tf_config",
        "rviz_config",
        "perception_params_file",
        "perception_params_overlay_file",
        "perception_input_topic",
        "perception_target_frame",
        "guardian_bridge_enabled",
        "guardian_bridge_mode",
        "guardian_bridge_manager_url",
        "guardian_recorder_enabled",
    ]

    launch_args_dict = {"system_mode": "crane"}
    for arg in forward_args:
        launch_args_dict[arg] = LaunchConfiguration(arg)

    include_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch),
        launch_arguments=launch_args_dict.items(),
    )

    return LaunchDescription(declared_arguments + [include_main])
