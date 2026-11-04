import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('redis_comm')
    default_config = os.path.join(pkg_share, 'config', 'redis_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the Redis configuration YAML file'
    )

    redis_node = Node(
        package='redis_comm',
        executable='redis_node',
        name='redis_comm_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        redis_node,
    ])

