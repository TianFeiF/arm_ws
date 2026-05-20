# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Hot-reloadable TCP offset + payload publisher.

Publishes a static-style TF (every tick, like a regular publisher rather than
tf2_static — that way late subscribers ALWAYS get a fresh value even if it
just changed) from `parent_frame` to `tcp_frame`. Pose is taken from params
`tcp_offset_xyz` and `tcp_offset_rpy`; both are dynamic — changing them via
`ros2 param set` takes effect on the next tick.

Also publishes the current payload (mass, COM, inertia) as a JSON-encoded
String on `/armv7/payload`. Phase-4 dynamics work consumes this — for v0.1
it's just a parking spot so external code has a place to look.
"""
from __future__ import annotations

import json
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

import tf2_ros


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def _rpy_to_quat(r: float, p: float, y: float) -> tuple:
    """Roll-pitch-yaw (XYZ extrinsic) → (qx, qy, qz, qw). No external dep."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class TcpPublisherNode(Node):

    def __init__(self) -> None:
        super().__init__('tcp_publisher_node')

        self.declare_parameter('parent_frame', 'link7')
        self.declare_parameter('tcp_frame', 'tcp')
        self.declare_parameter('tcp_offset_xyz', [0.0, 0.0, 0.15])
        self.declare_parameter('tcp_offset_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_rate', 50.0)

        self.declare_parameter('payload_mass', 0.0)
        self.declare_parameter('payload_com', [0.0, 0.0, 0.0])
        self.declare_parameter('payload_inertia',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self._br = tf2_ros.TransformBroadcaster(self)
        self._payload_pub = self.create_publisher(
            String, '/armv7/payload', _latched_qos())

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self._tcp_timer = self.create_timer(1.0 / rate, self._publish_tcp)
        self._payload_timer = self.create_timer(1.0, self._publish_payload)

        # Re-evaluate params each tick so live changes take effect.
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"tcp_publisher_node up: {self.get_parameter('parent_frame').value} → "
            f"{self.get_parameter('tcp_frame').value} @ {rate:.1f} Hz"
        )

    # ─────────────────────────────────────────────────────────────── params

    def _on_set_params(self, params):
        # Accept anything. The next timer tick will use the new values.
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in ('tcp_offset_xyz', 'tcp_offset_rpy', 'payload_com') \
                    and len(p.value) != 3:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} must be length 3")
            if p.name == 'payload_inertia' and len(p.value) != 6:
                return SetParametersResult(
                    successful=False,
                    reason="payload_inertia must be length 6 [ixx,iyy,izz,ixy,ixz,iyz]")
        self.get_logger().info(
            "tcp/payload params updated: "
            + ", ".join(p.name for p in params))
        return SetParametersResult(successful=True)

    # ─────────────────────────────────────────────────────────────── tcp

    def _publish_tcp(self) -> None:
        parent = self.get_parameter('parent_frame').value
        child = self.get_parameter('tcp_frame').value
        xyz: List[float] = list(self.get_parameter('tcp_offset_xyz').value)
        rpy: List[float] = list(self.get_parameter('tcp_offset_rpy').value)

        if len(xyz) != 3 or len(rpy) != 3:
            self.get_logger().warn(
                "tcp_offset_xyz / _rpy must each be length 3",
                throttle_duration_sec=5.0)
            return

        qx, qy, qz, qw = _rpy_to_quat(*rpy)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._br.sendTransform(msg)

    # ─────────────────────────────────────────────────────────────── payload

    def _publish_payload(self) -> None:
        msg = String()
        msg.data = json.dumps({
            'mass':    float(self.get_parameter('payload_mass').value),
            'com':     list(self.get_parameter('payload_com').value),
            'inertia': list(self.get_parameter('payload_inertia').value),
        })
        self._payload_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TcpPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
