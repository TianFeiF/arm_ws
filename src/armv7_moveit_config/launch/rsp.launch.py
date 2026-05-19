# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from _moveit_config_loader import build_moveit_config  # noqa: E402

from moveit_configs_utils.launches import generate_rsp_launch  # noqa: E402


def generate_launch_description():
    return generate_rsp_launch(build_moveit_config())
