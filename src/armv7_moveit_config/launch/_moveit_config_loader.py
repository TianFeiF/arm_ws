# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Shared MoveItConfigsBuilder factory.

All sub-launches in this package use this helper so the URDF xacro source
lives in `armv7_description` (single source of truth) while the SRDF + planner
configs live here. Pass `ros2_control_xacro` as an absolute path to the
hardware-specific *.ros2_control.xacro from a bringup package; leave empty
for description-only (no controllers).
"""
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(ros2_control_xacro: str = ""):
    description_pkg = get_package_share_path("armv7_description")
    urdf_xacro = str(description_pkg / "urdf" / "armv7.urdf.xacro")

    return (
        MoveItConfigsBuilder("armv7", package_name="armv7_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            mappings={"ros2_control_xacro": ros2_control_xacro},
        )
        .to_moveit_configs()
    )
