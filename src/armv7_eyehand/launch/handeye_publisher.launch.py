# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Publish the hand-eye calibration result as a static TF.

This is a thin wrapper around `tf2_ros static_transform_publisher` that reads
the calibration result from `handeye_calibration.yaml`. Replace the YAML
values with `easy_handeye2`'s output after running calibration.
"""
import yaml

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_static_tf(context):
    cfg_path = LaunchConfiguration('handeye_config').perform(context)
    with open(cfg_path) as fh:
        cfg = yaml.safe_load(fh)['handeye_publisher_node']['ros__parameters']

    t = [str(v) for v in cfg['translation']]
    q = [str(v) for v in cfg['rotation']]
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_static_tf',
        output='screen',
        arguments=[
            '--x', t[0], '--y', t[1], '--z', t[2],
            '--qx', q[0], '--qy', q[1], '--qz', q[2], '--qw', q[3],
            '--frame-id', cfg['parent_frame'],
            '--child-frame-id', cfg['camera_frame'],
        ],
    )]


def generate_launch_description():
    default_cfg = str(get_package_share_path('armv7_eyehand')
                      / 'config' / 'handeye_calibration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('handeye_config', default_value=default_cfg),
        OpaqueFunction(function=_make_static_tf),
    ])
