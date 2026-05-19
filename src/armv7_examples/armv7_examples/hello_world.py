# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""hello_world — move the arm to home pose (all zeros), then to a 30 deg pose.

Run:
    ros2 run armv7_examples hello_world
"""
from __future__ import annotations

import sys

import rclpy

from armv7_py import Armv7Client
from armv7_py.client import home_pose, small_demo_pose


def main(argv=None) -> int:
    rclpy.init(args=argv)
    arm = Armv7Client()
    try:
        if not arm.wait_for_joint_state(timeout_sec=5.0):
            print("ERR: no /joint_states within 5 s. "
                  "Is `ros2 launch armv7_bringup arm.launch.py` running?",
                  file=sys.stderr)
            return 1

        print(f"current joints: "
              f"{['%.3f' % q for q in arm.get_joint_state()]}")
        tcp = arm.get_tcp_pose()
        if tcp:
            print(f"current TCP   : "
                  f"({tcp.x:.3f}, {tcp.y:.3f}, {tcp.z:.3f})")

        print("→ moving to home pose...")
        if not arm.move_to_joint(home_pose(), duration_sec=3.0).wait(timeout_sec=15.0):
            print("ERR: move-to-home failed", file=sys.stderr)
            return 2

        print("→ moving to 30° pose...")
        if not arm.move_to_joint(small_demo_pose(), duration_sec=3.0).wait(timeout_sec=15.0):
            print("ERR: move-to-demo failed", file=sys.stderr)
            return 2

        print("→ returning home...")
        arm.move_to_joint(home_pose(), duration_sec=3.0).wait(timeout_sec=15.0)
        print("done.")
        return 0
    finally:
        arm.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
