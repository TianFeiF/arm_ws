#!/usr/bin/env python3
# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""
Tkinter dashboard for live tuning + targeting of the Cartesian impedance controller.

Three workspaces in one window:

  1. ENABLE / DISABLE big colour-coded button + status indicator.
  2. Target offset sliders (X/Y/Z relative to a captured seed pose) — slider
     change publishes a new ~/target_pose immediately, so you can drag a slider
     and watch the arm move (in fake mode, watch the spring force in real time).
     "Re-seed" snapshots the current TCP via TF as the new zero-offset target.
  3. Live stiffness / damping_ratio / wrench_limit sliders. Slider change calls
     `ros2 param set /cartesian_impedance_controller ...` via rclpy AsyncParameter
     client. The controller's on_set_parameters_callback validates and applies
     the new values WITHOUT requiring a restart.

  Plus a single "Screw preset" button that pushes K = [100,100,2000,50,50,5]
  and matching damping in one shot — the canonical XY-soft / Z-stiff /
  Rz-free configuration.

A side panel shows live pose_error / commanded_wrench / commanded_torque at
~10 Hz, so you do not need an extra `ros2 topic echo` terminal while tuning.

Run:
    ros2 run armv7_impedance_moveit tuning_dashboard

Requires: python3-tk (apt: `sudo apt install python3-tk`).
"""
from __future__ import annotations

import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

import tf2_ros
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterValue, ParameterType


CONTROLLER = '/cartesian_impedance_controller'
BASE_FRAME = 'base_link'
TIP_FRAME = 'link7'


# ----------- slider configuration (label, axis, min, max, default) -----------
TARGET_AXES = [('X', 0, -0.10, 0.10), ('Y', 1, -0.10, 0.10), ('Z', 2, -0.10, 0.10)]

STIFFNESS_AXES = [
    ('K_x',  0,    0.0, 3000.0,  200.0),
    ('K_y',  1,    0.0, 3000.0,  200.0),
    ('K_z',  2,    0.0, 3000.0,  200.0),
    ('K_rx', 3,    0.0,  200.0,   20.0),
    ('K_ry', 4,    0.0,  200.0,   20.0),
    ('K_rz', 5,    0.0,  200.0,   20.0),
]

DAMPING_AXES = [
    ('Z_x',  0, 0.0, 1.5, 0.7),
    ('Z_y',  1, 0.0, 1.5, 0.7),
    ('Z_z',  2, 0.0, 1.5, 0.7),
    ('Z_rx', 3, 0.0, 1.5, 0.7),
    ('Z_ry', 4, 0.0, 1.5, 0.7),
    ('Z_rz', 5, 0.0, 1.5, 0.7),
]


class RosClient(Node):
    """rclpy node that runs in a side thread and exposes thread-safe handles."""

    def __init__(self):
        super().__init__('impedance_tuning_dashboard')
        self.pose_pub = self.create_publisher(PoseStamped, f'{CONTROLLER}/target_pose', 10)
        self.enable_cli = self.create_client(SetBool, f'{CONTROLLER}/enable')
        self.set_param_cli = self.create_client(SetParameters, f'{CONTROLLER}/set_parameters')
        self.tare_cli = self.create_client(Trigger, f'{CONTROLLER}/tare_external_wrench')

        self.pose_error = [0.0] * 6
        self.cmd_wrench = [0.0] * 6
        self.cmd_torque = [0.0] * 7
        self.ext_wrench = [0.0] * 6
        self.last_seen = 0.0

        self.create_subscription(
            Float64MultiArray, f'{CONTROLLER}/pose_error',
            self._on_err, 10)
        self.create_subscription(
            Float64MultiArray, f'{CONTROLLER}/commanded_wrench',
            self._on_wrench, 10)
        self.create_subscription(
            Float64MultiArray, f'{CONTROLLER}/commanded_torque',
            self._on_torque, 10)
        self.create_subscription(
            WrenchStamped, f'{CONTROLLER}/external_wrench',
            self._on_ext_wrench, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_err(self, msg):
        if len(msg.data) >= 6:
            self.pose_error = list(msg.data[:6])
            self.last_seen = time.time()

    def _on_wrench(self, msg):
        if len(msg.data) >= 6:
            self.cmd_wrench = list(msg.data[:6])

    def _on_torque(self, msg):
        if len(msg.data) >= 7:
            self.cmd_torque = list(msg.data[:7])

    def _on_ext_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        self.ext_wrench = [f.x, f.y, f.z, t.x, t.y, t.z]

    # ---------- service / publish helpers --------------------------------
    def call_enable(self, value: bool, timeout: float = 1.0) -> str:
        if not self.enable_cli.wait_for_service(timeout_sec=timeout):
            return 'enable service not ready'
        fut = self.enable_cli.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.done() and fut.result() is not None:
            return fut.result().message
        return 'enable call timed out'

    def call_tare(self, timeout: float = 1.0) -> str:
        if not self.tare_cli.wait_for_service(timeout_sec=timeout):
            return 'tare service not ready'
        fut = self.tare_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.done() and fut.result() is not None:
            return fut.result().message
        return 'tare call timed out'

    def get_current_tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TIP_FRAME,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
            return t
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def publish_target(self, seed_tf, dx: float, dy: float, dz: float):
        """seed_tf: TransformStamped of base->tip captured earlier."""
        if seed_tf is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x = seed_tf.transform.translation.x + dx
        msg.pose.position.y = seed_tf.transform.translation.y + dy
        msg.pose.position.z = seed_tf.transform.translation.z + dz
        msg.pose.orientation = seed_tf.transform.rotation
        self.pose_pub.publish(msg)

    def set_double(self, name: str, value: float, timeout: float = 1.0) -> str:
        if not self.set_param_cli.wait_for_service(timeout_sec=timeout):
            return 'set_parameters service not ready'
        p = ParamMsg()
        p.name = name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req = SetParameters.Request(parameters=[p])
        fut = self.set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return 'set_parameters timed out'
        res = fut.result().results[0]
        return 'ok' if res.successful else f'rejected: {res.reason}'

    def set_double_array(self, name: str, values, timeout: float = 1.0) -> str:
        if not self.set_param_cli.wait_for_service(timeout_sec=timeout):
            return 'set_parameters service not ready'
        p = ParamMsg()
        p.name = name
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in values])
        req = SetParameters.Request(parameters=[p])
        fut = self.set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return 'set_parameters timed out'
        res = fut.result().results[0]
        return 'ok' if res.successful else f'rejected: {res.reason}'


def _spin(node):
    rclpy.spin(node)


class Dashboard:
    def __init__(self, root: tk.Tk, ros: RosClient):
        self.root = root
        self.ros = ros
        self.seed_tf = None
        self.enabled = False

        root.title('armv7 Cartesian 阻抗 — 调试面板')

        # -------- top: enable + status --------
        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, columnspan=2, sticky='ew')
        self.enable_btn = tk.Button(
            top, text='已停用  (点击启用)', bg='#a33', fg='white',
            font=('TkDefaultFont', 14, 'bold'), padx=12, pady=8,
            command=self._toggle_enable)
        self.enable_btn.pack(side='left', fill='x', expand=True)
        self.status_label = ttk.Label(top, text='—', anchor='e')
        self.status_label.pack(side='right', padx=8)

        # -------- left: target offsets --------
        target = ttk.LabelFrame(
            root, text='目标位姿偏移  (相对 seed,base_link 系)', padding=8)
        target.grid(row=1, column=0, sticky='nsew', padx=6, pady=4)
        self.target_vars = {}
        for label, axis, lo, hi in TARGET_AXES:
            v = tk.DoubleVar(value=0.0)
            self.target_vars[axis] = v
            self._make_slider_row(
                target, f'd{label}', v, lo, hi,
                self._publish_target, fmt='{:+.4f}', suffix='m')
        btn_row = ttk.Frame(target)
        btn_row.pack(fill='x', pady=4)
        ttk.Button(btn_row, text='重新锚定到当前 TCP',
                   command=self._reseed).pack(side='left', fill='x', expand=True)
        ttk.Button(btn_row, text='偏移归零',
                   command=self._reset_offsets).pack(side='right', fill='x', expand=True)

        # -------- right: stiffness + damping --------
        stiff = ttk.LabelFrame(root, text='刚度  (实时,不需重启)', padding=8)
        stiff.grid(row=1, column=1, sticky='nsew', padx=6, pady=4)
        self.stiff_vars = {}
        for label, axis, lo, hi, default in STIFFNESS_AXES:
            v = tk.DoubleVar(value=default)
            self.stiff_vars[axis] = v
            self._make_slider_row(
                stiff, label, v, lo, hi,
                self._apply_stiffness, fmt='{:.1f}')
        preset = ttk.Frame(stiff)
        preset.pack(fill='x', pady=4)
        ttk.Button(preset, text='保守起点  (K=100/10, ζ=1, dz=0.10, scale=0.5)',
                   command=self._preset_conservative
                   ).pack(side='left', fill='x', expand=True, padx=1)
        ttk.Button(preset, text='打螺丝预设',
                   command=self._preset_screw
                   ).pack(side='left', fill='x', expand=True, padx=1)
        ttk.Button(preset, text='默认',
                   command=self._preset_default
                   ).pack(side='left', fill='x', expand=True, padx=1)

        damp = ttk.LabelFrame(root, text='阻尼比  (0=无阻尼,1=临界)',
                              padding=8)
        damp.grid(row=2, column=1, sticky='nsew', padx=6, pady=4)
        self.damp_vars = {}
        for label, axis, lo, hi, default in DAMPING_AXES:
            v = tk.DoubleVar(value=default)
            self.damp_vars[axis] = v
            self._make_slider_row(
                damp, label, v, lo, hi,
                self._apply_damping, fmt='{:.2f}')

        # -------- right-bottom: friction-comp tuning ----------
        # The two knobs here are the most common cause of "shaking" in
        # impedance mode -- see README. velocity_deadband > 0 kills the
        # positive feedback that coulomb*tanh(qd/eps) introduces at qd~0;
        # friction_scale lets you live-dial all comp from 0 to 1.5x.
        fric = ttk.LabelFrame(root, text='摩擦补偿  (阻抗稳定性,抖动时往左拨 scale)',
                              padding=8)
        fric.grid(row=3, column=1, sticky='nsew', padx=6, pady=4)

        self.deadband_var = tk.DoubleVar(value=0.05)
        self._make_slider_row(
            fric, 'dz', self.deadband_var, 0.0, 0.3,
            self._apply_deadband, fmt='{:.3f}', suffix='rad/s')

        self.friction_scale_var = tk.DoubleVar(value=1.0)
        self._make_slider_row(
            fric, 'scale', self.friction_scale_var, 0.0, 1.5,
            self._apply_friction_scale, fmt='{:.2f}', suffix='×')

        preset_fric = ttk.Frame(fric)
        preset_fric.pack(fill='x', pady=4)
        ttk.Button(preset_fric, text='关闭摩擦补偿  (一键诊断抖动)',
                   command=self._fric_off).pack(side='left', fill='x', expand=True)
        ttk.Button(preset_fric, text='恢复默认  (dz=0.05, scale=1.0)',
                   command=self._fric_default).pack(side='right', fill='x', expand=True)

        # -------- bottom: live readouts --------
        live = ttk.LabelFrame(root, text='实时读数  (~10 Hz)', padding=8)
        live.grid(row=2, column=0, sticky='nsew', padx=6, pady=4)
        self.err_lbl = ttk.Label(live, text='位姿误差: ...',
                                 font=('TkFixedFont', 10))
        self.err_lbl.pack(anchor='w')
        self.wr_lbl = ttk.Label(live, text='力旋量: ...',
                                font=('TkFixedFont', 10))
        self.wr_lbl.pack(anchor='w')
        self.tau_lbl = ttk.Label(live, text='关节力矩: ...',
                                 font=('TkFixedFont', 10))
        self.tau_lbl.pack(anchor='w')
        self.ext_lbl = ttk.Label(live, text='外力估计: ...',
                                 font=('TkFixedFont', 10), foreground='#0066cc')
        self.ext_lbl.pack(anchor='w')
        ext_btn_row = ttk.Frame(live)
        ext_btn_row.pack(fill='x', pady=(2, 0))
        ttk.Button(ext_btn_row, text='外力置零 (tare,机械臂静止时按)',
                   command=self._tare_wrench).pack(side='left', fill='x', expand=True)
        self.fresh_lbl = ttk.Label(live, text='—', anchor='e')
        self.fresh_lbl.pack(anchor='e')

        # -------- bottom-left: pose-error rolling plot ----------
        plot_box = ttk.LabelFrame(
            root,
            text='位姿误差曲线  (近 10 秒,翻译上半 / 旋转下半,目标线虚线)',
            padding=4)
        plot_box.grid(row=3, column=0, sticky='nsew', padx=6, pady=4)
        # 10 Hz refresh × 10 s history
        self.PLOT_N = 100
        self.plot_buffer = [deque(maxlen=self.PLOT_N) for _ in range(6)]
        self.plot_canvas = tk.Canvas(plot_box, height=200, bg='white',
                                     highlightthickness=1,
                                     highlightbackground='#888')
        self.plot_canvas.pack(fill='both', expand=True)
        self.plot_canvas.bind('<Configure>', lambda _: self._redraw_plot())
        ctrl_row = ttk.Frame(plot_box)
        ctrl_row.pack(fill='x', pady=(4, 0))
        ttk.Button(ctrl_row, text='清空曲线  (改参后用这个看新收敛)',
                   command=self._clear_plot
                   ).pack(side='left', fill='x', expand=True)

        # -------- bottom: external-wrench rolling plot (full width) ----------
        wr_box = ttk.LabelFrame(
            root,
            text='外力估计曲线  (近 10 秒,力上半 N / 力矩下半 Nm,残差法,'
                 '低速最准)',
            padding=4)
        wr_box.grid(row=4, column=0, columnspan=2, sticky='nsew', padx=6, pady=4)
        self.wr_buffer = [deque(maxlen=self.PLOT_N) for _ in range(6)]
        self.wr_canvas = tk.Canvas(wr_box, height=160, bg='white',
                                   highlightthickness=1, highlightbackground='#888')
        self.wr_canvas.pack(fill='both', expand=True)
        self.wr_canvas.bind('<Configure>', lambda _: self._redraw_wrench_plot())

        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(1, weight=1)
        root.rowconfigure(2, weight=0)
        root.rowconfigure(3, weight=1)
        root.rowconfigure(4, weight=1)

        # auto-seed on startup once TF is available
        root.after(500, self._reseed)
        root.after(100, self._refresh)

    # ---------------- helpers -----------------------------------------
    def _make_slider_row(self, parent, label_text, var, lo, hi, apply_fn,
                         fmt='{:.2f}', suffix=''):
        """
        Build one row: [label] [slider expand] [editable Entry] [suffix].

        - Drag the slider OR type a value in the Entry: both update the same
          DoubleVar and call apply_fn so the controller sees the change.
        - The Entry is a separate StringVar so per-keystroke updates don't
          spam set_parameters; commit happens on Return / FocusOut only.
        """
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=2)
        ttk.Label(row, text=label_text, width=5).pack(side='left')

        if suffix:
            ttk.Label(row, text=suffix, width=len(suffix) + 1,
                      anchor='w').pack(side='right')

        text_var = tk.StringVar(value=fmt.format(var.get()))

        def _on_var_change(*_, _v=var, _t=text_var, _f=fmt):
            s = _f.format(_v.get())
            if _t.get() != s:
                _t.set(s)

        def _on_commit(_evt=None, _v=var, _t=text_var, _f=fmt, _fn=apply_fn,
                       _lo=lo, _hi=hi):
            try:
                value = float(_t.get())
            except (ValueError, tk.TclError):
                _t.set(_f.format(_v.get()))
                return
            value = max(_lo, min(_hi, value))  # clamp typed input to slider range
            _v.set(value)
            _fn()

        entry = ttk.Entry(row, textvariable=text_var, width=8, justify='right')
        entry.pack(side='right', padx=2)
        entry.bind('<Return>', _on_commit)
        entry.bind('<FocusOut>', _on_commit)

        scale = ttk.Scale(row, from_=lo, to=hi, orient='horizontal', variable=var,
                          command=lambda *_, _fn=apply_fn: _fn())
        scale.pack(side='left', fill='x', expand=True)

        var.trace_add('write', _on_var_change)
        return row

    # ---------------- handlers ----------------------------------------
    def _toggle_enable(self):
        self.enabled = not self.enabled
        msg = self.ros.call_enable(self.enabled)
        if self.enabled:
            self.enable_btn.configure(text='已启用  (点击停用)',
                                      bg='#2a7', fg='white')
        else:
            self.enable_btn.configure(text='已停用  (点击启用)',
                                      bg='#a33', fg='white')
        self.status_label.configure(text=msg)

    def _reseed(self):
        t = self.ros.get_current_tcp()
        if t is None:
            self.status_label.configure(text='TF 尚未就绪,重试中...')
            self.root.after(500, self._reseed)
            return
        self.seed_tf = t
        for v in self.target_vars.values():
            v.set(0.0)
        self._publish_target()
        self.status_label.configure(
            text=f'seed: ({t.transform.translation.x:+.3f}, '
                 f'{t.transform.translation.y:+.3f}, '
                 f'{t.transform.translation.z:+.3f})')

    def _reset_offsets(self):
        for v in self.target_vars.values():
            v.set(0.0)
        self._publish_target()

    def _publish_target(self):
        if self.seed_tf is None:
            return
        dx = self.target_vars[0].get()
        dy = self.target_vars[1].get()
        dz = self.target_vars[2].get()
        self.ros.publish_target(self.seed_tf, dx, dy, dz)

    def _apply_stiffness(self):
        vals = [self.stiff_vars[i].get() for i in range(6)]
        msg = self.ros.set_double_array('stiffness', vals)
        if msg != 'ok':
            self.status_label.configure(text=f'刚度: {msg}')

    def _apply_damping(self):
        vals = [self.damp_vars[i].get() for i in range(6)]
        msg = self.ros.set_double_array('damping_ratio', vals)
        if msg != 'ok':
            self.status_label.configure(text=f'阻尼比: {msg}')

    def _preset_screw(self):
        for i, k in enumerate([100.0, 100.0, 2000.0, 50.0, 50.0, 5.0]):
            self.stiff_vars[i].set(k)
        for i, d in enumerate([0.7, 0.7, 1.0, 0.7, 0.7, 0.5]):
            self.damp_vars[i].set(d)
        self._apply_stiffness()
        self._apply_damping()

    def _apply_deadband(self):
        msg = self.ros.set_double('velocity_deadband', self.deadband_var.get())
        if msg != 'ok':
            self.status_label.configure(text=f'死区: {msg}')

    def _apply_friction_scale(self):
        msg = self.ros.set_double(
            'friction_compensation_scale', self.friction_scale_var.get())
        if msg != 'ok':
            self.status_label.configure(text=f'摩擦补偿系数: {msg}')

    def _fric_off(self):
        self.friction_scale_var.set(0.0)
        self.deadband_var.set(0.3)
        self._apply_friction_scale()
        self._apply_deadband()

    def _fric_default(self):
        self.friction_scale_var.set(1.0)
        self.deadband_var.set(0.05)
        self._apply_friction_scale()
        self._apply_deadband()

    def _preset_default(self):
        defaults_k = [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
        defaults_d = [0.7] * 6
        for i, k in enumerate(defaults_k):
            self.stiff_vars[i].set(k)
        for i, d in enumerate(defaults_d):
            self.damp_vars[i].set(d)
        self._apply_stiffness()
        self._apply_damping()

    def _preset_conservative(self):
        """
        Half-stiffness + critical damping + wider deadband + half friction.

        Recommended starting point when tuning impedance on real hardware --
        very unlikely to oscillate, leaves headroom to crank up. From here,
        raise stiffness and lower deadband one step at a time per the recipe
        in docs/testing_phase5.md.
        """
        for i, k in enumerate([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]):
            self.stiff_vars[i].set(k)
        for i, d in enumerate([1.0] * 6):
            self.damp_vars[i].set(d)
        self.deadband_var.set(0.10)
        self.friction_scale_var.set(0.5)
        self._apply_stiffness()
        self._apply_damping()
        self._apply_deadband()
        self._apply_friction_scale()

    def _refresh(self):
        e = self.ros.pose_error
        w = self.ros.cmd_wrench
        t = self.ros.cmd_torque
        self.err_lbl.configure(
            text=f'位姿误差  trans [{e[0]:+.4f}, {e[1]:+.4f}, {e[2]:+.4f}] m  '
                 f'rot [{e[3]:+.4f}, {e[4]:+.4f}, {e[5]:+.4f}] rad')
        self.wr_lbl.configure(
            text=f'力旋量    F   [{w[0]:+7.2f}, {w[1]:+7.2f}, {w[2]:+7.2f}] N  '
                 f'M   [{w[3]:+7.2f}, {w[4]:+7.2f}, {w[5]:+7.2f}] Nm')
        self.tau_lbl.configure(
            text='关节力矩  j=  '
                 + ' '.join(f'{x:+6.2f}' for x in t) + ' Nm')
        x = self.ros.ext_wrench
        self.ext_lbl.configure(
            text=f'外力估计  F   [{x[0]:+6.2f}, {x[1]:+6.2f}, {x[2]:+6.2f}] N  '
                 f'M   [{x[3]:+6.2f}, {x[4]:+6.2f}, {x[5]:+6.2f}] Nm')
        age = time.time() - self.ros.last_seen
        fresh = ('过期' if age > 1.0 else '实时')
        self.fresh_lbl.configure(text=f'数据: {fresh}  年龄 {age:.1f}s')
        for i in range(6):
            self.plot_buffer[i].append(e[i])
            self.wr_buffer[i].append(x[i])
        self._redraw_plot()
        self._redraw_wrench_plot()
        self.root.after(100, self._refresh)

    def _clear_plot(self):
        for buf in self.plot_buffer:
            buf.clear()
        self.plot_canvas.delete('plot')

    def _draw_split_plot(self, canvas, buffer, colors, labels, units, floors, fmt):
        """Two-row time plot: channels 0-2 on top, 3-5 on bottom, shared helper."""
        c = canvas
        c.delete('plot')
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 120 or h < 80 or len(buffer[0]) < 2:
            return
        LABEL_W = 110
        plot_w = w - LABEL_W
        half_h = h // 2
        for half_idx in (0, 1):
            y_top = half_idx * half_h
            plot_h = half_h
            y_center = y_top + plot_h // 2
            ch_indices = (0, 1, 2) if half_idx == 0 else (3, 4, 5)
            max_abs = max((abs(v) for ch in ch_indices for v in buffer[ch]), default=0.0)
            max_abs = max(max_abs, floors[half_idx])
            c.create_line(0, y_center, plot_w, y_center,
                          fill='#888', dash=(2, 3), tags='plot')
            for ch in ch_indices:
                if len(buffer[ch]) < 2:
                    continue
                pts = []
                for k, v in enumerate(buffer[ch]):
                    px = (k / (self.PLOT_N - 1)) * plot_w
                    py = y_center - (v / max_abs) * (plot_h * 0.45)
                    pts.extend([px, py])
                c.create_line(*pts, fill=colors[ch], width=1.5, tags='plot')
            c.create_text(4, y_top + 2, anchor='nw',
                          text=f'±{max_abs:{fmt}} {units[half_idx]}',
                          fill='#666', font=('TkFixedFont', 9), tags='plot')
            for j, ch in enumerate(ch_indices):
                last_v = buffer[ch][-1] if buffer[ch] else 0.0
                c.create_text(plot_w + 4, y_top + 4 + j * 18, anchor='nw',
                              text=f'{labels[ch]:>4}: {last_v:+{fmt}}',
                              fill=colors[ch], font=('TkFixedFont', 9), tags='plot')
        c.create_line(0, half_h, plot_w, half_h, fill='#aaa', tags='plot')
        c.create_text(2, h - 2, anchor='sw', text='← 10 s',
                      fill='#aaa', font=('TkFixedFont', 8), tags='plot')

    def _redraw_plot(self):
        self._draw_split_plot(
            self.plot_canvas, self.plot_buffer,
            colors=['#d22', '#2a8', '#26d', '#a44', '#256', '#704'],
            labels=['eX', 'eY', 'eZ', 'eRx', 'eRy', 'eRz'],
            units=['m', 'rad'], floors=[0.005, 0.01], fmt='.4f')

    def _redraw_wrench_plot(self):
        self._draw_split_plot(
            self.wr_canvas, self.wr_buffer,
            colors=['#d22', '#2a8', '#26d', '#a44', '#256', '#704'],
            labels=['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'],
            units=['N', 'Nm'], floors=[2.0, 0.5], fmt='.2f')

    def _tare_wrench(self):
        msg = self.ros.call_tare()
        self.status_label.configure(text=f'外力置零: {msg}')


def main():
    rclpy.init()
    ros = RosClient()
    spin_thread = threading.Thread(target=_spin, args=(ros,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    Dashboard(root, ros)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
