#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    runtime_overrides = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    input_topic = LaunchConfiguration("input_topic").perform(context)
    target_frame = LaunchConfiguration("target_frame").perform(context)
    if input_topic:
        runtime_overrides["input_topic"] = input_topic
    if target_frame:
        runtime_overrides["target_frame"] = target_frame

    parameters = [LaunchConfiguration("params_file")]
    overlay = LaunchConfiguration("params_overlay_file").perform(context).strip()
    engine = LaunchConfiguration("perception_engine").perform(context).strip()
    if engine not in ("auto", "v2", "general_v2", "legacy"):
        raise ValueError("perception_engine must be auto, v2, general_v2 or legacy")
    if engine == "auto":
        mode = "default"
        for path in [LaunchConfiguration("params_file").perform(context), overlay]:
            if not path:
                continue
            with open(path, encoding="utf-8") as stream:
                content = yaml.safe_load(stream) or {}
            for key in ("/**", "cx128s2_bev_perception", "/cx128s2_bev_perception"):
                mode = content.get(key, {}).get("ros__parameters", {}).get("perception_mode", mode)
        engine = "v2" if mode == "crane_beam_work" else "legacy"
    if engine == "v2":
        parameters.append(LaunchConfiguration("v2_params_file"))
    if engine == "general_v2":
        parameters.append(LaunchConfiguration("general_v2_params_file"))
    if overlay:
        parameters.append(overlay)
    parameters.append(runtime_overrides)

    return [
        LogInfo(msg=f"[perception] engine={engine}; " +
                ("Crane V2 R15+C4 (legacy detector thresholds are ignored)" if engine == "v2" else
                 "General V2 independent ground/detection/tracking engine" if engine == "general_v2" else "legacy/general")),
        Node(
            package="cx128s2_bev_perception",
            executable=("crane_v2_experimental_node" if engine == "v2" else
                        "general_v2_node" if engine == "general_v2" else "cx128s2_bev_perception_node"),
            name="cx128s2_bev_perception",
            parameters=parameters,
            respawn=LaunchConfiguration("respawn"),
            respawn_delay=2.0,
            output="screen",
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("cx128s2_bev_perception")
    default_param_file = os.path.join(pkg_share, "config", "cx128s2_bev_perception.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("perception_engine", default_value="auto",
                              description="auto: Crane YAML selects Crane V2, General YAML selects original General; explicit general_v2 is experimental"),
        DeclareLaunchArgument("general_v2_params_file", default_value=os.path.join(pkg_share, "config", "general_v2.yaml"),
                              description="Independent General V2 settings"),
        DeclareLaunchArgument("v2_params_file", default_value=os.path.join(pkg_share, "config", "crane_v2.yaml"),
                              description="Crane V2 defaults; legacy detection parameters do not configure V2"),
        DeclareLaunchArgument("params_file", default_value=default_param_file),
        DeclareLaunchArgument("params_overlay_file", default_value="",
                              description="Optional YAML overlay after params_file"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("respawn", default_value="false",
                              description="Restart perception after unexpected exit"),
        DeclareLaunchArgument(
            "input_topic",
            default_value="",
            description="Optional override. Empty means use params_file value.",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="",
            description="Optional override. Empty means use params_file value.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
