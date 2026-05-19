# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Smoke tests for armv7_py.

These do NOT require rclpy.init() or any running nodes — they only verify
that the module imports cleanly, public names exist, and a few pure-Python
helpers work.
"""


def test_module_imports():
    import armv7_py
    assert hasattr(armv7_py, 'Armv7Client')
    assert hasattr(armv7_py, 'JOINT_NAMES')


def test_joint_names_seven():
    from armv7_py import JOINT_NAMES
    assert len(JOINT_NAMES) == 7
    assert JOINT_NAMES == [f'joint{i}' for i in range(1, 8)]


def test_pose_helpers():
    from armv7_py.client import home_pose, small_demo_pose
    home = home_pose()
    assert home == [0.0] * 7
    demo = small_demo_pose()
    assert len(demo) == 7
    assert all(-1.0 < q < 1.0 for q in demo)


def test_to_duration_msg():
    from armv7_py.client import _to_duration_msg
    d = _to_duration_msg(2.5)
    assert d.sec == 2
    assert d.nanosec == 500_000_000
