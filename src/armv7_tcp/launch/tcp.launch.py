# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = str(get_package_share_path('armv7_tcp') / 'config' / 'tcp.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'tcp_config', default_value=default_cfg,
            description='YAML with TCP offset + payload defaults.'),
        Node(
            package='armv7_tcp',
            executable='tcp_publisher_node',
            name='tcp_publisher_node',
            output='screen',
            parameters=[LaunchConfiguration('tcp_config')],
        ),
    ])
