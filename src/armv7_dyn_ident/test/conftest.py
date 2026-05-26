# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
import os

import pytest


def _urdf_path():
    here = os.path.dirname(__file__)
    candidate = os.path.normpath(
        os.path.join(here, '..', '..', 'armv7_description', 'urdf', 'armv7.urdf'))
    if os.path.exists(candidate):
        return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        return f'{get_package_share_directory("armv7_description")}/urdf/armv7.urdf'
    except Exception:
        return candidate


@pytest.fixture(scope='session')
def gm():
    from armv7_dyn_ident.gravity_model import GravityModel
    from armv7_dyn_ident.urdf_model import parse_urdf_file, serial_chain
    links, joints = parse_urdf_file(_urdf_path())
    return GravityModel(serial_chain(links, joints))
