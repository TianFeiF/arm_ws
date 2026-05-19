# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Sanity checks on the armv7 URDF + xacro.

Runs in `colcon test`. Does NOT require ROS to be running.
"""
from __future__ import annotations

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory


EXPECTED_JOINTS = [f'joint{i}' for i in range(1, 8)]
EXPECTED_LINKS = ['base_link'] + [f'link{i}' for i in range(1, 8)]


@pytest.fixture(scope='module')
def share() -> str:
    return get_package_share_directory('armv7_description')


def test_urdf_file_exists(share):
    assert os.path.isfile(os.path.join(share, 'urdf', 'armv7.urdf'))


def test_urdf_xacro_file_exists(share):
    assert os.path.isfile(os.path.join(share, 'urdf', 'armv7.urdf.xacro'))


def test_urdf_links_and_joints(share):
    tree = ET.parse(os.path.join(share, 'urdf', 'armv7.urdf'))
    root = tree.getroot()
    link_names = [link.get('name') for link in root.findall('link')]
    joint_names = [j.get('name') for j in root.findall('joint')]
    for l in EXPECTED_LINKS:
        assert l in link_names, f"missing link: {l}"
    for j in EXPECTED_JOINTS:
        assert j in joint_names, f"missing joint: {j}"


def test_urdf_no_legacy_package_refs(share):
    """package:// references must point at armv7_description after the rename."""
    with open(os.path.join(share, 'urdf', 'armv7.urdf')) as fh:
        text = fh.read()
    assert 'package://armv7_description/' in text, "no package:// refs?"
    assert 'package://armv7/' not in text, "stale package://armv7/ left over"


def _strip_comments(xml: str) -> str:
    """Drop all <!-- ... --> blocks so substring tests don't match comment text."""
    out = []
    i = 0
    while i < len(xml):
        if xml.startswith('<!--', i):
            end = xml.find('-->', i + 4)
            if end == -1:
                break
            i = end + 3
        else:
            out.append(xml[i])
            i += 1
    return ''.join(out)


def test_xacro_processes_description_only(share):
    """Top-level xacro must succeed with no ros2_control_xacro arg."""
    xacro_path = os.path.join(share, 'urdf', 'armv7.urdf.xacro')
    result = subprocess.run(
        ['xacro', xacro_path],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f"xacro failed:\n{result.stderr}"
    body = _strip_comments(result.stdout)
    assert '<robot' in body
    assert '<ros2_control' not in body, \
        "description-only xacro should NOT contain a <ros2_control> element"
    for j in EXPECTED_JOINTS:
        assert f'name="{j}"' in body


def test_xacro_processes_with_fake_hardware():
    """Plus an injected fake ros2_control xacro should yield mock_components."""
    desc_share = get_package_share_directory('armv7_description')
    bringup_share = get_package_share_directory('armv7_bringup')
    xacro_path = os.path.join(desc_share, 'urdf', 'armv7.urdf.xacro')
    fake = os.path.join(bringup_share, 'urdf', 'armv7_fake.ros2_control.xacro')

    result = subprocess.run(
        ['xacro', xacro_path, f'ros2_control_xacro:={fake}'],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, f"xacro failed:\n{result.stderr}"
    body = _strip_comments(result.stdout)
    assert '<ros2_control' in body
    assert 'mock_components/GenericSystem' in body
