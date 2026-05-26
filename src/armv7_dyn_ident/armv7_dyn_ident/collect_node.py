# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Static-pose data collector for gravity identification.

Drives the arm to a sequence of static configurations using the EXISTING
position-mode trajectory controller (FollowJointTrajectory action), lets each
pose settle, then averages a window of /joint_states to record (q, tau). The
arm never leaves position mode, so this is safe to run before any torque /
free-drive work exists.

Output is a CSV with header  q1..qN,tau1..tauN  consumed by `identify`.

    ros2 run armv7_dyn_ident collect --ros-args \
        -p output_csv:=/tmp/armv7_gravity.csv \
        -p n_poses:=60

WARNING: the auto-generated random poses are NOT collision-checked. Inspect
them (or pass your own via the `poses` param, flattened row-major) and keep a
hand on the E-Stop the first time.
"""
from __future__ import annotations

import csv
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory

from .excitation import static_poses
from .urdf_model import parse_urdf_file, serial_chain

DEFAULT_JOINTS = [f'joint{i}' for i in range(1, 8)]


class CollectNode(Node):

    def __init__(self):
        super().__init__('armv7_dyn_collect')
        self.declare_parameter('joints', DEFAULT_JOINTS)
        self.declare_parameter('action_name',
                               '/plan_group_controller/follow_joint_trajectory')
        self.declare_parameter('output_csv', '/tmp/armv7_gravity.csv')
        self.declare_parameter('n_poses', 60)
        self.declare_parameter('margin', 0.12)
        self.declare_parameter('seed', 0)
        self.declare_parameter('move_time', 4.0)        # s to reach each pose
        self.declare_parameter('settle_time', 1.5)      # s to wait after arrival
        self.declare_parameter('samples_per_pose', 40)
        self.declare_parameter('urdf', '')              # override; else from description pkg
        self.declare_parameter('poses', [])             # flattened row-major override

        self.joints: List[str] = list(
            self.get_parameter('joints').get_parameter_value().string_array_value
        ) or DEFAULT_JOINTS
        self.n = len(self.joints)
        self.action_name = self.get_parameter('action_name').value
        self.output_csv = self.get_parameter('output_csv').value
        self.move_time = float(self.get_parameter('move_time').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.samples_per_pose = int(self.get_parameter('samples_per_pose').value)

        self._latest: Optional[JointState] = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self._client = ActionClient(self, FollowJointTrajectory, self.action_name)

        self.poses = self._resolve_poses()

    # -- setup helpers ----------------------------------------------------
    def _resolve_poses(self) -> List[List[float]]:
        explicit = list(self.get_parameter('poses').get_parameter_value().double_array_value)
        if explicit:
            if len(explicit) % self.n != 0:
                raise ValueError(f'poses length {len(explicit)} not a multiple of {self.n}')
            return [explicit[i:i + self.n] for i in range(0, len(explicit), self.n)]
        lower, upper = self._joint_limits()
        return static_poses(lower, upper,
                            count=int(self.get_parameter('n_poses').value),
                            margin=float(self.get_parameter('margin').value),
                            seed=int(self.get_parameter('seed').value))

    def _joint_limits(self):
        urdf = self.get_parameter('urdf').value
        if not urdf:
            urdf = f'{get_package_share_directory("armv7_description")}/urdf/armv7.urdf'
        links, joints = parse_urdf_file(urdf)
        jmap = {j.name: j for j in joints.values()}
        lower = [jmap[name].lower for name in self.joints]
        upper = [jmap[name].upper for name in self.joints]
        return lower, upper

    # -- runtime ----------------------------------------------------------
    def _on_js(self, msg: JointState):
        self._latest = msg

    def _reorder(self, msg: JointState, field: str):
        idx = {name: i for i, name in enumerate(msg.name)}
        src = getattr(msg, field)
        if any(j not in idx for j in self.joints) or len(src) < len(msg.name):
            return None
        return [src[idx[j]] for j in self.joints]

    def _send_pose(self, q: List[float]) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.velocities = [0.0] * self.n
        secs = int(self.move_time)
        pt.time_from_start = Duration(sec=secs, nanosec=int((self.move_time - secs) * 1e9))
        goal.trajectory.points = [pt]

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('goal rejected')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result() is not None

    def _collect_sample(self):
        q_acc = [0.0] * self.n
        tau_acc = [0.0] * self.n
        got = 0
        deadline = time.time() + max(2.0, self.samples_per_pose * 0.05)
        last_id = None
        while got < self.samples_per_pose and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self._latest
            if msg is None or msg is last_id:
                continue
            last_id = msg
            q = self._reorder(msg, 'position')
            tau = self._reorder(msg, 'effort')
            if q is None or tau is None:
                continue
            for k in range(self.n):
                q_acc[k] += q[k]
                tau_acc[k] += tau[k]
            got += 1
        if got == 0:
            return None
        return [v / got for v in q_acc], [v / got for v in tau_acc]

    def run(self):
        self.get_logger().info(f'waiting for action server {self.action_name} ...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('action server not available')
            return
        rows = []
        total = len(self.poses)
        for i, pose in enumerate(self.poses):
            self.get_logger().info(f'[{i + 1}/{total}] moving to '
                                   + ' '.join(f'{v:+.2f}' for v in pose))
            if not self._send_pose(pose):
                self.get_logger().warn('  move failed, skipping pose')
                continue
            time.sleep(self.settle_time)
            sample = self._collect_sample()
            if sample is None:
                self.get_logger().warn('  no joint_states with effort, skipping')
                continue
            q, tau = sample
            rows.append(list(q) + list(tau))
            self.get_logger().info('  tau = ' + ' '.join(f'{v:+.2f}' for v in tau))

        if not rows:
            self.get_logger().error('collected 0 samples; nothing written')
            return
        self._write_csv(rows)
        self.get_logger().info(f'wrote {len(rows)} samples to {self.output_csv}')

    def _write_csv(self, rows):
        header = [f'q{i + 1}' for i in range(self.n)] + [f'tau{i + 1}' for i in range(self.n)]
        with open(self.output_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)


def main(args=None):
    rclpy.init(args=args)
    node = CollectNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
