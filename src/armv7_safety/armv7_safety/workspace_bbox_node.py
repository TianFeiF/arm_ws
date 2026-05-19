# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Cartesian workspace bounding-box safety check.

Watches TF for the TCP frame, publishes:
- /safety/in_bounds        std_msgs/Bool       — true while TCP is inside the box
- /safety/bbox_state       std_msgs/String     — "ok" | "warning" | "out_of_bounds"

If `auto_estop_on_exit:=true`, the node calls /estop_trigger when the TCP exits
the box. Default is off — operators usually want a warning first.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException


def _latched_qos() -> QoSProfile:
    """TRANSIENT_LOCAL so a late subscriber gets the last value immediately."""
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


class WorkspaceBboxNode(Node):

    def __init__(self) -> None:
        super().__init__('workspace_bbox_node')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('tcp_frame_id', 'link7')
        self.declare_parameter('min_x', -0.6)
        self.declare_parameter('max_x', 0.6)
        self.declare_parameter('min_y', -0.6)
        self.declare_parameter('max_y', 0.6)
        self.declare_parameter('min_z', 0.0)
        self.declare_parameter('max_z', 1.2)
        self.declare_parameter('check_rate', 50.0)
        self.declare_parameter('margin', 0.05)
        self.declare_parameter('auto_estop_on_exit', False)

        self._frame = self.get_parameter('frame_id').value
        self._tcp = self.get_parameter('tcp_frame_id').value
        self._bounds = (
            self.get_parameter('min_x').value, self.get_parameter('max_x').value,
            self.get_parameter('min_y').value, self.get_parameter('max_y').value,
            self.get_parameter('min_z').value, self.get_parameter('max_z').value,
        )
        self._margin = float(self.get_parameter('margin').value)
        self._auto_estop = bool(self.get_parameter('auto_estop_on_exit').value)
        rate = max(1.0, float(self.get_parameter('check_rate').value))

        self._pub_in_bounds = self.create_publisher(Bool, '/safety/in_bounds', 10)
        # bbox_state is latched so subscribers connecting after the last
        # transition still see the current state.
        self._pub_state = self.create_publisher(
            String, '/safety/bbox_state', _latched_qos())

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        self._estop_cli = self.create_client(Trigger, '/safety/estop_trigger')
        self._last_state = ''
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"workspace_bbox watching {self._tcp} in {self._frame}, "
            f"box=({self._bounds[0]:.2f},{self._bounds[1]:.2f}) "
            f"({self._bounds[2]:.2f},{self._bounds[3]:.2f}) "
            f"({self._bounds[4]:.2f},{self._bounds[5]:.2f}) "
            f"margin={self._margin:.3f}, auto_estop={self._auto_estop}"
        )

    def _classify(self, x: float, y: float, z: float) -> str:
        mn_x, mx_x, mn_y, mx_y, mn_z, mx_z = self._bounds
        if not (mn_x <= x <= mx_x and mn_y <= y <= mx_y and mn_z <= z <= mx_z):
            return 'out_of_bounds'
        # inside the box; near a face?
        m = self._margin
        near_face = (
            x - mn_x < m or mx_x - x < m
            or y - mn_y < m or mx_y - y < m
            or z - mn_z < m or mx_z - z < m
        )
        return 'warning' if near_face else 'ok'

    def _tick(self) -> None:
        try:
            tf = self._tf_buf.lookup_transform(
                self._frame, self._tcp, rclpy.time.Time())
        except TransformException as ex:
            # First few ticks before TF buffer fills are normal; throttle log
            self.get_logger().warn(
                f"TF {self._frame}->{self._tcp} unavailable: {ex}",
                throttle_duration_sec=5.0,
            )
            return

        t = tf.transform.translation
        state = self._classify(t.x, t.y, t.z)

        self._pub_in_bounds.publish(Bool(data=(state != 'out_of_bounds')))
        # Republish state on every tick. The latched QoS means new subscribers
        # still get the current value immediately, AND continuous publishing
        # makes external watchdogs (e.g. `ros2 topic hz /safety/bbox_state`)
        # work as expected.
        self._pub_state.publish(String(data=state))

        if state != self._last_state:
            level = self.get_logger().info if state == 'ok' else self.get_logger().warn
            level(f"bbox state {self._last_state or '(init)'} -> {state} "
                  f"(TCP={t.x:.3f},{t.y:.3f},{t.z:.3f})")
            self._last_state = state

            if state == 'out_of_bounds' and self._auto_estop:
                if self._estop_cli.wait_for_service(timeout_sec=0.1):
                    self._estop_cli.call_async(Trigger.Request())
                    self.get_logger().error("auto-estop triggered: TCP out of bbox")


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceBboxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
