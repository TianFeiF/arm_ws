# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Software E-Stop for the armv7 arm.

Provides
- /safety/estop                  std_msgs/Bool        (latched, true => stopped)
- /safety/estop_trigger          std_srvs/Trigger
- /safety/estop_clear            std_srvs/Trigger

Trigger semantics:
    1. Publish estop=true latched.
    2. Call controller_manager `switch_controller`:
         deactivate: [plan_group_controller]
         strictness: STRICT
       The trajectory controller is the only one that can command motion;
       deactivating it makes ros2_control hold the last commanded position.
    3. Service returns success / message.

Clear semantics: re-activates plan_group_controller. The arm will resume taking
trajectory goals.

NOT certified as a functional-safety E-Stop. Use a hardware E-Stop on top of this
in any real deployment.
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from controller_manager_msgs.srv import ListControllers, SwitchController


CONTROLLER = 'plan_group_controller'


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


class EstopNode(Node):

    def __init__(self) -> None:
        super().__init__('estop_node')

        self.declare_parameter('controller_name', CONTROLLER)
        self._controller = self.get_parameter('controller_name').value

        self._lock = threading.Lock()
        self._stopped = False

        # ReentrantCallbackGroup is required so that service handlers can
        # synchronously wait on async service clients without deadlocking
        # under MultiThreadedExecutor.
        self._cbg = ReentrantCallbackGroup()

        self._pub = self.create_publisher(Bool, '/safety/estop', _latched_qos())
        self._pub.publish(Bool(data=False))            # initial state

        self.create_service(Trigger, '/safety/estop_trigger',
                            self._on_trigger, callback_group=self._cbg)
        self.create_service(Trigger, '/safety/estop_clear',
                            self._on_clear, callback_group=self._cbg)

        self._switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=self._cbg)
        self._list_cli = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._cbg)
        self.get_logger().info(
            f"estop_node up; controller={self._controller}, "
            "topics=/safety/estop, srvs=/safety/estop_trigger /safety/estop_clear")

    @staticmethod
    def _await_future(future, timeout_sec: float):
        """Block until `future` completes or `timeout_sec` elapses.

        Safe to call from a service handler when the client is in a
        ReentrantCallbackGroup AND the executor is multi-threaded — another
        executor thread will drive the future to completion.
        """
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        done_event.wait(timeout=timeout_sec)
        return future.result() if future.done() else None

    def _controller_is_active(self) -> bool:
        if not self._list_cli.wait_for_service(timeout_sec=0.5):
            return False
        resp = self._await_future(
            self._list_cli.call_async(ListControllers.Request()), timeout_sec=2.0)
        if resp is None:
            return False
        for c in resp.controller:
            if c.name == self._controller:
                return c.state == 'active'
        return False

    def _switch(self, activate: bool) -> tuple[bool, str]:
        if not self._switch_cli.wait_for_service(timeout_sec=2.0):
            return False, '/controller_manager/switch_controller unavailable'

        req = SwitchController.Request()
        req.strictness = SwitchController.Request.STRICT
        if activate:
            req.activate_controllers = [self._controller]
        else:
            req.deactivate_controllers = [self._controller]

        resp = self._await_future(self._switch_cli.call_async(req), timeout_sec=5.0)
        # Even if the RPC timed out, the controller state may have changed.
        actual_active = self._controller_is_active()
        if resp is not None and resp.ok:
            return True, f"{'activated' if activate else 'deactivated'} {self._controller}"
        if actual_active is activate:
            return True, (f"{'activated' if activate else 'deactivated'} "
                          f"{self._controller} (verified post-RPC)")
        return False, ('switch_controller RPC failed and controller is '
                       f"{'inactive' if not actual_active else 'active'}")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._stopped:
                response.success = True
                response.message = 'already stopped'
                return response
            ok, msg = self._switch(activate=False)
            if ok:
                self._stopped = True
                self._pub.publish(Bool(data=True))
                self.get_logger().error(f"E-STOP TRIGGERED: {msg}")
            else:
                self.get_logger().error(f"E-STOP TRIGGER FAILED: {msg}")
            response.success = ok
            response.message = msg
            return response

    def _on_clear(self, request, response):
        with self._lock:
            if not self._stopped:
                response.success = True
                response.message = 'not stopped'
                return response
            ok, msg = self._switch(activate=True)
            if ok:
                self._stopped = False
                self._pub.publish(Bool(data=False))
                self.get_logger().info(f"E-STOP CLEARED: {msg}")
            else:
                self.get_logger().error(f"E-STOP CLEAR FAILED: {msg}")
            response.success = ok
            response.message = msg
            return response


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
