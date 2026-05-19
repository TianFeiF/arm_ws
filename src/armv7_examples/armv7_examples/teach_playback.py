# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""teach_playback — record 5 joint poses interactively, then replay the loop.

Workflow:
1. Move the arm by hand (or via RViz) to a pose, then press Enter in this
   terminal to capture it. Repeat for 5 poses.
2. The arm replays the captured poses, looping `--repeats` times.

Run:
    ros2 run armv7_examples teach_playback
    ros2 run armv7_examples teach_playback --poses 7 --repeats 3
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy

from armv7_py import Armv7Client


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses',   type=int,   default=5,
                   help='Number of poses to record (default: 5).')
    p.add_argument('--repeats', type=int,   default=2,
                   help='How many replay passes through the sequence (default: 2).')
    p.add_argument('--dt',      type=float, default=2.5,
                   help='Seconds between waypoints when replaying (default: 2.5).')
    return p.parse_args()


def main(argv=None) -> int:
    args = _parse()
    rclpy.init(args=argv)
    arm = Armv7Client()
    try:
        if not arm.wait_for_joint_state(timeout_sec=5.0):
            print("ERR: no /joint_states within 5 s", file=sys.stderr)
            return 1

        recorded = []
        print("\n=== TEACH PHASE ===")
        print(f"Move the arm by hand or via RViz to each pose, then press Enter.\n"
              f"({args.poses} poses to record)\n")
        for i in range(args.poses):
            input(f"Pose {i + 1}/{args.poses} — press Enter to capture: ")
            q = arm.get_joint_state()
            if q is None:
                print("ERR: lost /joint_states", file=sys.stderr)
                return 1
            recorded.append(q)
            print(f"  captured: {['%.3f' % v for v in q]}")

        print("\n=== PLAYBACK PHASE ===")
        for pass_no in range(args.repeats):
            print(f"\npass {pass_no + 1}/{args.repeats}")
            handle = arm.move_through_joints(recorded, dt_sec=args.dt)
            if not handle.wait(timeout_sec=args.dt * len(recorded) + 5.0):
                print("ERR: playback failed", file=sys.stderr)
                return 2
            time.sleep(0.5)

        print("done.")
        return 0
    finally:
        arm.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
