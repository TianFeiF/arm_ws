# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Run the static-pose gravity-data collector.

Assumes the arm is already up (real or fake) with plan_group_controller active:

    ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true   # terminal A
    ros2 launch armv7_dyn_ident collect.launch.py                     # terminal B
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = f'{get_package_share_directory("armv7_dyn_ident")}/config/excitation.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('output_csv', default_value='/tmp/armv7_gravity.csv'),
        Node(
            package='armv7_dyn_ident',
            executable='collect',
            name='armv7_dyn_collect',
            output='screen',
            parameters=[config, {'output_csv': LaunchConfiguration('output_csv')}],
        ),
    ])
