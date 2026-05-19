# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Aggregate /joint_states into /diagnostics for rqt_runtime_monitor.

For each joint, publishes a DiagnosticStatus with:
  level   OK | WARN | ERROR | STALE
  message human-readable
  values  [position, velocity, effort, max_position, max_velocity, max_effort]

Thresholds come from a per-joint yaml (see config/joint_diagnostics.yaml). Joint
limits come from MoveIt's joint_limits.yaml (single source of truth).

Phase-2 scope: position/velocity/effort grading from /joint_states only.
Phase-4 scope (TODO): per-slave drive temperature + error_code from EtherCAT.
"""
from __future__ import annotations

import os
from typing import Optional

import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def _resolve_path(path: str) -> str:
    """Resolve $(find pkg)/... and ~/... in a path string."""
    if path.startswith('$(find '):
        pkg, rest = path[len('$(find '):].split(')', 1)
        return os.path.join(get_package_share_directory(pkg.strip()), rest.lstrip('/'))
    return os.path.expanduser(path)


class JointDiagnosticsNode(Node):

    def __init__(self) -> None:
        super().__init__('joint_diagnostics_node')

        self.declare_parameter('limits_yaml',
                               '$(find armv7_moveit_config)/config/joint_limits.yaml')
        self.declare_parameter('stale_after_sec', 1.0)
        self.declare_parameter('position_warn_frac', 0.95)
        self.declare_parameter('velocity_warn_frac', 0.85)
        self.declare_parameter('effort_warn_frac', 0.85)
        self.declare_parameter('publish_rate', 2.0)

        self._stale_sec = float(self.get_parameter('stale_after_sec').value)
        self._pf = float(self.get_parameter('position_warn_frac').value)
        self._vf = float(self.get_parameter('velocity_warn_frac').value)
        self._ef = float(self.get_parameter('effort_warn_frac').value)

        self._limits = self._load_limits(
            _resolve_path(self.get_parameter('limits_yaml').value)
        )
        self.get_logger().info(
            f"loaded limits for {len(self._limits)} joints: {list(self._limits.keys())}"
        )

        self._last_msg: Optional[JointState] = None
        self._last_stamp_ns: int = 0

        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_state, 10)
        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self._timer = self.create_timer(1.0 / rate, self._publish)

    # ---------------------------------------------------------------- helpers

    @staticmethod
    def _load_limits(path: str) -> dict:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"joint_limits.yaml not found: {path}")
        with open(path) as fh:
            blob = yaml.safe_load(fh)
        return blob.get('joint_limits', {})

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_msg = msg
        self._last_stamp_ns = self.get_clock().now().nanoseconds

    # ---------------------------------------------------------------- core

    def _grade(self, name: str, pos: float, vel: float, eff: float) -> tuple[int, str]:
        lim = self._limits.get(name)
        if lim is None:
            return DiagnosticStatus.WARN, f"no limits configured for {name}"

        # position
        mn = lim.get('min_position', -1e9)
        mx = lim.get('max_position', 1e9)
        half = max(abs(mn), abs(mx))
        if not (mn <= pos <= mx):
            return DiagnosticStatus.ERROR, f"position {pos:.3f} outside [{mn},{mx}]"
        if half > 0 and abs(pos) > self._pf * half:
            return DiagnosticStatus.WARN, f"position {pos:.3f} near limit ±{half:.2f}"

        # velocity
        mv = lim.get('max_velocity', 1e9)
        if abs(vel) > mv:
            return DiagnosticStatus.ERROR, f"velocity {vel:.2f} exceeds {mv:.2f}"
        if mv > 0 and abs(vel) > self._vf * mv:
            return DiagnosticStatus.WARN, f"velocity {vel:.2f} near limit {mv:.2f}"

        # effort
        me = lim.get('max_effort', 1e9)
        if abs(eff) > me:
            return DiagnosticStatus.ERROR, f"effort {eff:.2f} exceeds {me:.2f}"
        if me > 0 and abs(eff) > self._ef * me:
            return DiagnosticStatus.WARN, f"effort {eff:.2f} near limit {me:.2f}"

        return DiagnosticStatus.OK, 'nominal'

    def _publish(self) -> None:
        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()

        if self._last_msg is None:
            stale = DiagnosticStatus(
                level=DiagnosticStatus.STALE,
                name='armv7/joint_states',
                message='no /joint_states received yet',
                hardware_id='armv7',
            )
            out.status.append(stale)
            self._pub.publish(out)
            return

        # global stale check
        age_sec = (self.get_clock().now().nanoseconds - self._last_stamp_ns) / 1e9
        if age_sec > self._stale_sec:
            stale = DiagnosticStatus(
                level=DiagnosticStatus.STALE,
                name='armv7/joint_states',
                message=f'no /joint_states for {age_sec:.2f}s',
                hardware_id='armv7',
            )
            out.status.append(stale)
            self._pub.publish(out)
            return

        msg = self._last_msg
        n = len(msg.name)
        pos = list(msg.position) if msg.position else [0.0] * n
        vel = list(msg.velocity) if msg.velocity else [0.0] * n
        eff = list(msg.effort) if msg.effort else [0.0] * n

        for i, name in enumerate(msg.name):
            p, v, e = pos[i], vel[i], eff[i]
            level, why = self._grade(name, p, v, e)
            status = DiagnosticStatus(
                level=level,
                name=f'armv7/{name}',
                message=why,
                hardware_id='armv7',
            )
            status.values = [
                KeyValue(key='position', value=f'{p:.4f}'),
                KeyValue(key='velocity', value=f'{v:.4f}'),
                KeyValue(key='effort', value=f'{e:.4f}'),
            ]
            lim = self._limits.get(name, {})
            for k in ('min_position', 'max_position', 'max_velocity', 'max_effort'):
                if k in lim:
                    status.values.append(KeyValue(key=k, value=f"{lim[k]:.4f}"))
            out.status.append(status)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
