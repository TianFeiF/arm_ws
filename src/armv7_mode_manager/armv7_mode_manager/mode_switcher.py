#!/usr/bin/env python3
# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""
Runtime POSITION <-> FORCE mode coordinator.

Encapsulates what a "mode" means so clients (the rqt panel, scripts) just ask
for one:

  * POSITION = MoveIt / JointTrajectoryController active, impedance inactive.
              On real EtherCAT hardware the drives are in CiA-402 CSP (mode 8).
  * FORCE    = Cartesian impedance active, JTC inactive.
              On real hardware the drives are in CST (mode 10).

Exposes:
  ~/to_position   std_srvs/Trigger
  ~/to_force      std_srvs/Trigger
  ~/current_mode  std_msgs/String   (latched)

Switching is one atomic controller_manager/switch_controller call (deactivate
the old set, activate the new). On real hardware a drive-mode handover is
sequenced around the controller switch (Step 2 -- gated by `manage_drive_mode`,
off by default so fake / mock bringup works unchanged).

DANGER (real hardware, force mode): the impedance controller starts DISABLED; it
holds 0 torque until you enable it. Switching INTO force mode therefore leaves
the arm un-actuated for a moment -- keep it supported / braked. Switching OUT of
force mode hands back to JTC which holds position from the current pose.
"""
from __future__ import annotations

import threading
import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger


class ModeSwitcher(Node):

    def __init__(self):
        super().__init__('mode_switcher')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('position_controllers', ['plan_group_controller'])
        self.declare_parameter('force_controllers', ['cartesian_impedance_controller'])
        self.declare_parameter('switch_timeout', 5.0)
        # Step 2 (real hardware): also write the CiA-402 drive mode. Off for
        # fake / mock bringup, where there is no mode interface to drive.
        self.declare_parameter('manage_drive_mode', False)
        self.declare_parameter('mode_command_topic', '/mode_controller/commands')
        self.declare_parameter('n_joints', 7)
        self.declare_parameter('drive_mode_position', 8.0)   # CiA-402 CSP
        self.declare_parameter('drive_mode_force', 10.0)     # CiA-402 CST
        self.declare_parameter('mode_switch_delay', 0.3)     # s between mode write and switch
        # Initial mode reported on ~/current_mode (the launch spawns the matching
        # controller active); no switch is performed, this just labels the state.
        self.declare_parameter('start_mode', 'force')
        # Engage gravity comp WITH the force switch so the arm never sees a
        # 0-torque window (= drop). to_force enables the impedance controller and,
        # on real hardware, only flips the drive to CST AFTER gravity comp is
        # holding. to_position disables it again so the next to_force re-engages
        # cleanly (reseed + tare).
        self.declare_parameter('auto_enable_force', True)
        self.declare_parameter('impedance_enable_service',
                               '/cartesian_impedance_controller/enable')
        # Seconds to let the impedance ramp-in complete (while CSP still holds)
        # before flipping the drive to CST. Match the controller's ramp_in_time
        # plus a margin.
        self.declare_parameter('force_engage_delay', 2.5)
        # Force mode engages in ZERO-GRAVITY (free-drive): gravity + friction
        # comp only, NO Cartesian stiffness. A K=0 spring can't fling the arm on
        # a tiny tracking error at the switch instant -> no jerk. The Cartesian
        # spring is opt-in afterwards (panel "阻抗弹簧" toggle sets K > 0).
        self.declare_parameter('impedance_param_service',
                               '/cartesian_impedance_controller/set_parameters')
        self.declare_parameter('force_engage_stiffness',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.cm = self.get_parameter('controller_manager').value
        self.pos_ctrls = list(
            self.get_parameter('position_controllers').get_parameter_value().string_array_value)
        self.force_ctrls = list(
            self.get_parameter('force_controllers').get_parameter_value().string_array_value)
        self.switch_timeout = float(self.get_parameter('switch_timeout').value)
        self.manage_drive_mode = bool(self.get_parameter('manage_drive_mode').value)
        self.n_joints = int(self.get_parameter('n_joints').value)
        self.drive_mode_position = float(self.get_parameter('drive_mode_position').value)
        self.drive_mode_force = float(self.get_parameter('drive_mode_force').value)
        self.mode_switch_delay = float(self.get_parameter('mode_switch_delay').value)
        self.auto_enable_force = bool(self.get_parameter('auto_enable_force').value)
        self.force_engage_delay = float(self.get_parameter('force_engage_delay').value)
        self.force_engage_stiffness = list(
            self.get_parameter('force_engage_stiffness').get_parameter_value()
            .double_array_value)

        self._cb = ReentrantCallbackGroup()
        self._switch_cli = self.create_client(
            SwitchController, f'{self.cm}/switch_controller', callback_group=self._cb)
        self._enable_cli = self.create_client(
            SetBool, self.get_parameter('impedance_enable_service').value,
            callback_group=self._cb)
        self._setparam_cli = self.create_client(
            SetParameters, self.get_parameter('impedance_param_service').value,
            callback_group=self._cb)
        self._mode_cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('mode_command_topic').value, 10)

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_pub = self.create_publisher(String, '~/current_mode', latched)
        self._mode = str(self.get_parameter('start_mode').value)
        self._publish_mode()

        self.create_service(Trigger, '~/to_position', self._on_to_position,
                            callback_group=self._cb)
        self.create_service(Trigger, '~/to_force', self._on_to_force,
                            callback_group=self._cb)

        self.get_logger().info(
            f'mode_switcher up. position={self.pos_ctrls} force={self.force_ctrls} '
            f'manage_drive_mode={self.manage_drive_mode}')

    def _publish_mode(self):
        self._mode_pub.publish(String(data=self._mode))

    def _switch(self, activate, deactivate):
        if not self._switch_cli.wait_for_service(timeout_sec=self.switch_timeout):
            return False, 'switch_controller service not available'
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = rclpy.duration.Duration(seconds=self.switch_timeout).to_msg()
        fut = self._switch_cli.call_async(req)
        # We are spun by a MultiThreadedExecutor; block this callback's thread.
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=self.switch_timeout + 1.0):
            return False, 'switch_controller call timed out'
        res = fut.result()
        if res is None:
            return False, 'switch_controller returned no result'
        return res.ok, ('ok' if res.ok else 'switch_controller reported failure')

    def _write_drive_mode(self, mode_value):
        msg = Float64MultiArray()
        msg.data = [float(mode_value)] * self.n_joints
        self._mode_cmd_pub.publish(msg)

    def _set_stiffness(self, values):
        """Set the impedance controller's Cartesian stiffness (6 entries)."""
        if not self._setparam_cli.wait_for_service(timeout_sec=self.switch_timeout):
            return False, 'set_parameters service not available'
        p = ParamMsg()
        p.name = 'stiffness'
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in values])
        fut = self._setparam_cli.call_async(SetParameters.Request(parameters=[p]))
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=self.switch_timeout + 1.0):
            return False, 'set stiffness timed out'
        res = fut.result()
        if res is None or not res.results or not res.results[0].successful:
            return False, 'set stiffness rejected'
        return True, 'ok'

    def _set_impedance_enabled(self, value):
        """Call the impedance controller's ~/enable; returns (ok, message)."""
        if not self._enable_cli.wait_for_service(timeout_sec=self.switch_timeout):
            return False, 'impedance enable service not available'
        req = SetBool.Request()
        req.data = bool(value)
        fut = self._enable_cli.call_async(req)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout=self.switch_timeout + 1.0):
            return False, 'impedance enable call timed out'
        res = fut.result()
        if res is None:
            return False, 'impedance enable returned no result'
        return res.success, res.message

    def _on_to_position(self, _req, res):
        # force -> position WITHOUT a jerk. While in force/CST the drive's
        # position command (0x607a) is STALE (frozen at the last position-mode
        # pose), but the arm has actually moved under free-drive. Flipping
        # straight to CSP would snap it back to that stale target. So:
        #   1. Co-activate JTC ALONGSIDE the impedance (different interfaces:
        #      JTC claims position+velocity, impedance claims effort -> no
        #      conflict). JTC reads the current state and writes 0x607a = current
        #      pose. The drive is still CST so 0x607a is ignored; the arm is held
        #      by the impedance's effort. 0x607a is now correctly seeded.
        #   2. Flip the drive to CSP -> it obeys 0x607a (= current pose) -> holds
        #      in place, NO jerk. The impedance's effort is now ignored.
        #   3. Deactivate + disable the impedance (JTC keeps holding). The disable
        #      makes the next to_force a clean disabled->enabled transition.
        ok, msg = self._switch(activate=self.pos_ctrls, deactivate=[])
        if not ok:
            res.success = False
            res.message = f'-> position: activating JTC failed: {msg}'
            self.get_logger().error(res.message)
            return res
        time.sleep(self.mode_switch_delay)        # let JTC write 0x607a = current
        if self.manage_drive_mode:
            self._write_drive_mode(self.drive_mode_position)
            time.sleep(self.mode_switch_delay)
        self._switch(activate=[], deactivate=self.force_ctrls)
        if self.auto_enable_force:
            self._set_impedance_enabled(False)
        self._mode = 'position'
        self._publish_mode()
        res.success = True
        res.message = f'-> position (JTC seeded current pose first): {msg}'
        self.get_logger().info(res.message)
        return res

    def _on_to_force(self, _req, res):
        # position -> force, WITHOUT a drop and WITHOUT a spring jerk. Engages
        # ZERO-GRAVITY (free-drive): gravity + friction comp only, K = 0.
        #   1. switch controllers (impedance active; JTC stops -> drive holds via
        #      CSP, which still ignores the impedance's effort).
        #   2. set stiffness = 0 (zero-gravity). A K=0 spring cannot fling the arm
        #      on the small tracking error present at the switch -> no jerk.
        #   3. ENABLE the impedance -> gravity comp computes/ramps. Drive still CSP
        #      so no mechanical effect yet (holds).
        #   4. wait force_engage_delay for the ramp (CSP holding).
        #   5. flip the drive to CST -> obeys FULLY-ramped gravity comp -> seamless
        #      weightless hold, no drop. The Cartesian spring is opt-in afterwards.
        ok, msg = self._switch(activate=self.force_ctrls, deactivate=self.pos_ctrls)
        if ok:
            self._set_stiffness(self.force_engage_stiffness)
        enable_msg = ''
        if ok and self.auto_enable_force:
            en_ok, enable_msg = self._set_impedance_enabled(True)
            if not en_ok:
                self.get_logger().warn(f'impedance enable failed: {enable_msg}')
        if ok and self.manage_drive_mode:
            time.sleep(self.force_engage_delay if self.auto_enable_force
                       else self.mode_switch_delay)
            self._write_drive_mode(self.drive_mode_force)
        if ok:
            self._mode = 'force'
            self._publish_mode()
        res.success = ok
        if self.auto_enable_force:
            res.message = (f'-> force / zero-gravity (gravity comp on, K=0): {msg} '
                           f'{enable_msg}').strip()
        else:
            res.message = (f'-> force: {msg}. DANGER: impedance NOT auto-enabled '
                           '(auto_enable_force=false) -- enable it / support the arm.')
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
