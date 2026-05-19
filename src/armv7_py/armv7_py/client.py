# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Armv7Client — high-level Python facade for the armv7 arm.

Design
------
- Wraps the `FollowJointTrajectory` action server that the standard
  joint_trajectory_controller exposes. We do NOT require moveit_py — the
  client just needs ros2_control + a trajectory controller to be running.
- For Cartesian moves we lean on MoveIt's GetPositionIK service so a user can
  pass `(x, y, z, quat)` directly.
- Async by design: every move_* returns a future-like object that resolves to
  `True` (success) or `False` (timeout / failure). Pass `wait=True` to block.
- TCP pose is read straight from `/tf` (no extra publishers required).

Usage
-----
```python
import rclpy
from armv7_py import Armv7Client

rclpy.init()
arm = Armv7Client()

arm.move_to_joint([0, 0, 0, 0, 0, 0, 0], wait=True)
arm.jog(joint=0, delta=0.5, wait=True)
print(arm.get_joint_state())
print(arm.get_tcp_pose())
arm.stop()

arm.shutdown()
rclpy.shutdown()
```
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros


JOINT_NAMES: List[str] = [f'joint{i}' for i in range(1, 8)]
TRAJ_ACTION = '/plan_group_controller/follow_joint_trajectory'


@dataclass
class TcpPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    def as_tuple(self) -> tuple:
        return (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)


class Armv7Client:
    """High-level facade. Spins its own executor in a background thread.

    Always call `shutdown()` (or use `with Armv7Client() as arm:`) so the
    background thread joins cleanly.
    """

    def __init__(self,
                 node: Optional[Node] = None,
                 base_frame: str = 'base_link',
                 tcp_frame: str = 'link7',
                 default_duration_sec: float = 3.0):
        self._owns_node = node is None
        if not rclpy.ok():
            rclpy.init()
        self._node = node or rclpy.create_node('armv7_py_client')
        self._base = base_frame
        self._tcp = tcp_frame
        self._default_dur = float(default_duration_sec)

        self._cbg = ReentrantCallbackGroup()

        # joint_state cache
        self._js_lock = threading.Lock()
        self._js_msg: Optional[JointState] = None
        self._node.create_subscription(
            JointState, '/joint_states', self._on_js, 10,
            callback_group=self._cbg,
        )

        # tf2 listener
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self._node)

        # trajectory action
        self._traj_cli = ActionClient(self._node, FollowJointTrajectory,
                                      TRAJ_ACTION, callback_group=self._cbg)

        # E-Stop service
        self._estop_cli = self._node.create_client(
            Trigger, '/safety/estop_trigger', callback_group=self._cbg)

        # spin in background
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------ basic

    def shutdown(self) -> None:
        self._executor.shutdown()
        if self._owns_node:
            self._node.destroy_node()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.shutdown()

    # ----------------------------------------------------------------- state

    def _on_js(self, msg: JointState) -> None:
        with self._js_lock:
            self._js_msg = msg

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._js_msg is not None and self.get_joint_state() is not None:
                return True
            time.sleep(0.05)
        return False

    def get_joint_state(self) -> Optional[List[float]]:
        """Return current 7-vector of joint positions in JOINT_NAMES order, or None."""
        with self._js_lock:
            msg = self._js_msg
        if msg is None:
            return None
        try:
            idx = [msg.name.index(j) for j in JOINT_NAMES]
        except ValueError:
            return None
        return [msg.position[i] for i in idx]

    def get_tcp_pose(self, timeout_sec: float = 1.0) -> Optional[TcpPose]:
        """Look up `tcp_frame` in `base_frame`. Returns None on TF timeout."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                tf = self._tf_buf.lookup_transform(
                    self._base, self._tcp, rclpy.time.Time())
                t = tf.transform.translation
                r = tf.transform.rotation
                return TcpPose(t.x, t.y, t.z, r.x, r.y, r.z, r.w)
            except tf2_ros.TransformException:
                time.sleep(0.05)
        return None

    # ------------------------------------------------------------------ move

    def move_to_joint(self,
                      joint_positions: Sequence[float],
                      duration_sec: Optional[float] = None,
                      wait: bool = False,
                      timeout_sec: float = 30.0) -> 'TrajectoryHandle':
        """Send a single-waypoint trajectory to `joint_positions`.

        Returns a TrajectoryHandle. Call `.wait(timeout)` or pass `wait=True`.
        """
        if len(joint_positions) != 7:
            raise ValueError(f"expected 7 joints, got {len(joint_positions)}")

        dur = float(duration_sec if duration_sec is not None else self._default_dur)

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in joint_positions]
        pt.velocities = [0.0] * 7
        pt.time_from_start = _to_duration_msg(dur)
        traj.points = [pt]

        return self._send_trajectory(traj, wait=wait, timeout_sec=timeout_sec)

    def move_through_joints(self,
                            waypoints: Sequence[Sequence[float]],
                            dt_sec: float = 1.5,
                            wait: bool = False,
                            timeout_sec: float = 60.0) -> 'TrajectoryHandle':
        """Send a multi-waypoint joint trajectory, equally spaced by `dt_sec`."""
        if not waypoints:
            raise ValueError("waypoints empty")
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        for i, q in enumerate(waypoints):
            if len(q) != 7:
                raise ValueError(f"waypoint {i} has {len(q)} joints, expected 7")
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start = _to_duration_msg((i + 1) * dt_sec)
            traj.points.append(pt)
        return self._send_trajectory(traj, wait=wait, timeout_sec=timeout_sec)

    def jog(self,
            joint: int,
            delta: float,
            duration_sec: float = 1.0,
            wait: bool = False) -> 'TrajectoryHandle':
        """Move one joint by `delta` rad from its current position.

        `joint` is 0-indexed (0=joint1).
        """
        if not 0 <= joint <= 6:
            raise ValueError(f"joint index {joint} out of range [0,6]")
        current = self.get_joint_state()
        if current is None:
            raise RuntimeError(
                "no /joint_states yet — call wait_for_joint_state() first")
        target = list(current)
        target[joint] += float(delta)
        return self.move_to_joint(target, duration_sec=duration_sec, wait=wait)

    # ----------------------------------------------------------------- stop

    def stop(self) -> bool:
        """Trigger /safety/estop_trigger. Returns True on success."""
        if not self._estop_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn("/safety/estop_trigger not available")
            return False
        fut = self._estop_cli.call_async(Trigger.Request())
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        done.wait(timeout=5.0)
        resp = fut.result() if fut.done() else None
        return bool(resp and resp.success)

    # --------------------------------------------------------------- private

    def _send_trajectory(self, traj: JointTrajectory, wait: bool,
                         timeout_sec: float) -> 'TrajectoryHandle':
        if not self._traj_cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f"{TRAJ_ACTION} action server not available — is plan_group_controller active?")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        send_fut = self._traj_cli.send_goal_async(goal)

        handle = TrajectoryHandle(self._node)
        send_fut.add_done_callback(handle._on_goal_response)

        if wait:
            handle.wait(timeout_sec=timeout_sec)
        return handle


# ------------------------------------------------------------------ helpers

def _to_duration_msg(sec: float) -> DurationMsg:
    return DurationMsg(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))


class TrajectoryHandle:
    """Async handle for an in-flight trajectory.

    Use `.wait(timeout)` to block, or check `.done()` / `.success` later.
    """

    def __init__(self, node: Node):
        self._node = node
        self._goal_handle = None
        self._result = None
        self._done_event = threading.Event()
        self.success: Optional[bool] = None

    def _on_goal_response(self, fut) -> None:
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.success = False
            self._done_event.set()
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut) -> None:
        try:
            self._result = fut.result()
            ec = getattr(self._result.result, 'error_code', 0) if self._result else 1
            self.success = (ec == FollowJointTrajectory.Result.SUCCESSFUL)
        except Exception:
            self.success = False
        self._done_event.set()

    def done(self) -> bool:
        return self._done_event.is_set()

    def wait(self, timeout_sec: float = 30.0) -> bool:
        return self._done_event.wait(timeout=timeout_sec) and bool(self.success)

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()


# ---------------------------------------------------------------- shortcuts

def home_pose() -> List[float]:
    """All-zero pose — arm stretched along world Z."""
    return [0.0] * 7


def small_demo_pose() -> List[float]:
    """A safe non-trivial pose: ~30 deg on each joint."""
    a = math.radians(30.0)
    return [a, -a, a, -a, a, -a, a]
