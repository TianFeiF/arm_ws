# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""pose_grid — sweep joint1 + joint2 through a small grid of angles.

Useful for:
- Visually verifying the kinematic chain
- Smoke-testing trajectory execution after any bringup change
- Capturing a /joint_states baseline for `armv7_dyn_ident` (Phase 4)

Run:
    ros2 run armv7_examples pose_grid
    ros2 run armv7_examples pose_grid --steps 5 --range 0.5
"""
from __future__ import annotations

import argparse
import sys

import rclpy

from armv7_py import Armv7Client


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--steps', type=int,   default=4,
                   help='Grid resolution per axis (default: 4 → 16 poses).')
    p.add_argument('--range', type=float, default=0.6,
                   help='Half-range in rad for both joints (default: 0.6 → ±0.6 rad).')
    p.add_argument('--dt',    type=float, default=1.5,
                   help='Seconds between waypoints (default: 1.5).')
    return p.parse_args()


def main(argv=None) -> int:
    args = _parse()
    if args.steps < 2:
        print("--steps must be ≥ 2", file=sys.stderr)
        return 1

    rclpy.init(args=argv)
    arm = Armv7Client()
    try:
        if not arm.wait_for_joint_state(timeout_sec=5.0):
            print("ERR: no /joint_states within 5 s", file=sys.stderr)
            return 1

        # build the grid
        step = (2 * args.range) / (args.steps - 1)
        waypoints = []
        for i in range(args.steps):
            for j in range(args.steps):
                q = [0.0] * 7
                q[0] = -args.range + step * i        # joint1
                q[1] = -args.range + step * j        # joint2
                waypoints.append(q)
        print(f"sweeping {len(waypoints)} poses (joint1/joint2, ±{args.range:.2f} rad)")

        handle = arm.move_through_joints(waypoints, dt_sec=args.dt)
        if not handle.wait(timeout_sec=args.dt * len(waypoints) + 10.0):
            print("ERR: grid sweep failed", file=sys.stderr)
            return 2

        # return home
        arm.move_to_joint([0.0] * 7, duration_sec=3.0).wait(timeout_sec=15.0)
        print("done.")
        return 0
    finally:
        arm.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
