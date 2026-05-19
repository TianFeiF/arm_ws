# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Visualisation-only launch for armv7_description.

Brings up robot_state_publisher, joint_state_publisher_gui and RViz2 with the
view config. Does NOT load any ros2_control hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('armv7_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'armv7.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
            output='screen',
        ),
    ])
