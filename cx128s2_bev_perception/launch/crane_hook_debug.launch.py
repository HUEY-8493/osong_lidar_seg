# Legacy standalone replay experiment, not the production crane launch.
# Uses the Python scratch detector; use lidar_crane_bringup for C++ perception.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_path = '/home/sjjin/construct_ws/src/cx128s2_bev_perception/config/crane_hook_debug.rviz'
    bag_path = '/home/sjjin/construct_ws/src/lidar_ground_tools/measurements/crane_2026_09_22_032118/crane_hook_bag'

    # 1. Static TF: world -> lidar
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_lidar_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '6.63827667',
            '--roll', '-1.56719728',
            '--pitch', '-0.02748935',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'lidar'
        ]
    )

    # 2. Crane Hook Detector Node (Python)
    detector_proc = ExecuteProcess(
        cmd=['python3', '/home/sjjin/construct_ws/src/cx128s2_bev_perception/scripts/crane_hook_detector_node.py'],
        output='screen'
    )

    # 3. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 4. rosbag play (loop mode)
    bag_play_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '-l', '--rate', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        static_tf_node,
        detector_proc,
        rviz_node,
        bag_play_proc
    ])
