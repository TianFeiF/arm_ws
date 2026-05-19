# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Bring up joint_diagnostics_node."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        get_package_share_path('armv7_diagnostics') / 'config' / 'joint_diagnostics.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'diag_config', default_value=default_config,
            description='Joint diagnostics thresholds yaml.'),
        Node(
            package='armv7_diagnostics',
            executable='joint_diagnostics_node',
            name='joint_diagnostics_node',
            output='screen',
            parameters=[LaunchConfiguration('diag_config')],
        ),
    ])
