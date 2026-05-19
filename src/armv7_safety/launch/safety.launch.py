# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Bring up the workspace bbox checker + E-Stop node together."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_bbox_yaml = str(
        get_package_share_path('armv7_safety') / 'config' / 'workspace_bbox.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bbox_config', default_value=default_bbox_yaml,
            description='Path to workspace bounding-box YAML.'),
        DeclareLaunchArgument(
            'controller_name', default_value='plan_group_controller',
            description='Controller deactivated when E-Stop triggers.'),

        Node(
            package='armv7_safety',
            executable='workspace_bbox_node',
            name='workspace_bbox_node',
            output='screen',
            parameters=[LaunchConfiguration('bbox_config')],
        ),
        Node(
            package='armv7_safety',
            executable='estop_node',
            name='estop_node',
            output='screen',
            parameters=[{
                'controller_name': LaunchConfiguration('controller_name'),
            }],
        ),
    ])
