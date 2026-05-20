# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""pick_and_place — minimal pick & place using the dummy gripper.

Requires launch with `ee=dummy_gripper` (which loads the gripper xacro and
spawns ee_gripper_controller). Without that, the gripper service call will
just warn and continue — the arm motion still runs so you can validate the
trajectory side without the EE.

Flow:
    1. open gripper
    2. move to "approach A"   — 10 cm above the pick pose
    3. move to "pick A"
    4. close gripper
    5. move back to "approach A"
    6. move to "approach B"   — 10 cm above the place pose
    7. move to "place B"
    8. open gripper
    9. retreat to "approach B"
   10. return home

Run:
    ros2 run armv7_examples pick_and_place
    ros2 run armv7_examples pick_and_place --pick 0.40 -0.20 0.20 --place 0.40 0.20 0.20
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from armv7_py import Armv7Client
from armv7_py.client import home_pose


GRIPPER_TOPIC = '/ee_gripper_controller/commands'
GRIPPER_OPEN = 0.035       # m
GRIPPER_CLOSED = 0.010     # m


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pick',  nargs=7, type=float,
                   metavar=('Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'Q7'),
                   default=[0.5, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0],
                   help='Pick joint pose (7 floats, rad). Default is a safe demo pose.')
    p.add_argument('--place', nargs=7, type=float,
                   metavar=('Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'Q7'),
                   default=[-0.5, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0],
                   help='Place joint pose (7 floats, rad).')
    p.add_argument('--approach-offset', type=float, default=0.3,
                   help='Joint-2 angle offset added to the pick/place pose for the '
                        'approach pose, simulating "10 cm above" without IK. Default 0.3 rad.')
    p.add_argument('--dt', type=float, default=2.5,
                   help='Seconds per arm move (default 2.5).')
    return p.parse_args()


class GripperClient(Node):
    """Wraps `/ee_gripper_controller/commands` (Float64MultiArray).

    The dummy gripper uses position_controllers/JointGroupPositionController
    which takes a Float64MultiArray of joint positions on its `commands` topic.
    """

    def __init__(self):
        super().__init__('pick_and_place_gripper_client')
        self._pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._announced_missing = False

    def _send(self, position: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self._pub.publish(msg)

    def open(self) -> None:
        if self._pub.get_subscription_count() == 0:
            if not self._announced_missing:
                self.get_logger().warn(
                    f"no subscriber on {GRIPPER_TOPIC} — "
                    "is the dummy gripper loaded? Continuing without gripper.")
                self._announced_missing = True
            return
        print("  → gripper open")
        self._send(GRIPPER_OPEN)
        time.sleep(0.8)

    def close(self) -> None:
        if self._pub.get_subscription_count() == 0:
            return
        print("  → gripper close")
        self._send(GRIPPER_CLOSED)
        time.sleep(0.8)


def main(argv=None) -> int:
    args = _parse()
    rclpy.init(args=argv)
    arm = Armv7Client()
    gripper = GripperClient()
    try:
        if not arm.wait_for_joint_state(timeout_sec=5.0):
            print("ERR: no /joint_states within 5 s.", file=sys.stderr)
            return 1

        # Build approach poses by offsetting joint2.
        def approach(q):
            a = list(q)
            a[1] += args.approach_offset
            return a

        steps = [
            ("HOME",       home_pose(),         args.dt),
            ("OPEN",       None,                0.0),
            ("APPR A",     approach(args.pick), args.dt),
            ("PICK A",     args.pick,           args.dt),
            ("CLOSE",      None,                0.0),
            ("RETREAT A",  approach(args.pick), args.dt),
            ("APPR B",     approach(args.place), args.dt),
            ("PLACE B",    args.place,          args.dt),
            ("OPEN",       None,                0.0),
            ("RETREAT B",  approach(args.place), args.dt),
            ("HOME",       home_pose(),         args.dt),
        ]

        for i, (label, q, dur) in enumerate(steps, 1):
            print(f"[{i:2d}/{len(steps)}] {label}")
            if q is None:
                # gripper step
                if label == "OPEN":
                    gripper.open()
                elif label == "CLOSE":
                    gripper.close()
                continue
            ok = arm.move_to_joint(q, duration_sec=dur).wait(timeout_sec=dur + 5.0)
            if not ok:
                print(f"ERR: step {label} failed/timed out", file=sys.stderr)
                return 2

        print("done.")
        return 0
    finally:
        gripper.destroy_node()
        arm.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
