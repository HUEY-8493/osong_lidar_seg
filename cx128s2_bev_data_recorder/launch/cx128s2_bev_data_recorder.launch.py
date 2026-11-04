#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("cx128s2_bev_data_recorder")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            share, "config", "cx128s2_bev_data_recorder.yaml")),
        Node(package="cx128s2_bev_data_recorder",
             executable="cx128s2_bev_data_recorder_node",
             name="cx128s2_bev_data_recorder",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
    ])
