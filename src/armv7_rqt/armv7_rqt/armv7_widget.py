# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""
Unified armv7 control widget (Qt) for the rqt panel.

Combines, against a single ROS node (rqt's executor spins it):
  * Mode switch     -> armv7_mode_manager /mode_switcher/{to_position,to_force}
  * Impedance       -> cartesian_impedance_controller enable / tare / live params
  * Monitoring      -> pose_error, commanded_wrench, external_wrench readouts

All ROS subscriptions cache the latest values; a Qt QTimer in the GUI thread
reads the caches and updates the labels, so no ROS callback ever touches a Qt
widget directly (thread-safe). Service calls are fire-and-forget (call_async)
with the result stashed for the timer to surface in the status line.
"""
import math

from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
import tf2_ros

BASE_FRAME = 'base_link'
TIP_FRAME = 'link7'

CONTROLLER = '/cartesian_impedance_controller'
MODE_NS = '/mode_switcher'

# (label, param_name, index, min, max, step, decimals)
STIFFNESS = [
    ('K_x', 0, 0.0, 3000.0, 25.0, 0), ('K_y', 1, 0.0, 3000.0, 25.0, 0),
    ('K_z', 2, 0.0, 3000.0, 25.0, 0), ('K_rx', 3, 0.0, 200.0, 5.0, 0),
    ('K_ry', 4, 0.0, 200.0, 5.0, 0), ('K_rz', 5, 0.0, 200.0, 5.0, 0),
]
DAMPING = [('Z_%s' % a, i, 0.0, 1.5, 0.05, 2)
           for i, a in enumerate(['x', 'y', 'z', 'rx', 'ry', 'rz'])]


# ---- minimal quaternion helpers (x, y, z, w), dependency-free ----
def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_rotate(q, v):
    """Rotate 3-vector v by quaternion q (x,y,z,w)."""
    qx, qy, qz, qw = q
    t = 2.0 * np.cross([qx, qy, qz], v)
    return np.asarray(v) + qw * t + np.cross([qx, qy, qz], t)


class Armv7Widget(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setObjectName('Armv7Widget')
        self.setWindowTitle('armv7 控制面板')

        # ---- ROS comms ----
        self._pose_err = [0.0] * 6
        self._cmd_wrench = [0.0] * 6
        self._ext_wrench = [0.0] * 6
        self._mode = 'unknown'
        self._status = ''

        self._enable_cli = node.create_client(SetBool, f'{CONTROLLER}/enable')
        self._tare_cli = node.create_client(Trigger, f'{CONTROLLER}/tare_external_wrench')
        self._setparam_cli = node.create_client(SetParameters, f'{CONTROLLER}/set_parameters')
        self._to_pos_cli = node.create_client(Trigger, f'{MODE_NS}/to_position')
        self._to_force_cli = node.create_client(Trigger, f'{MODE_NS}/to_force')
        self._target_pub = node.create_publisher(
            PoseStamped, f'{CONTROLLER}/target_pose', 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        self._seed_tf = None   # captured base->tip transform for target offsets

        node.create_subscription(Float64MultiArray, f'{CONTROLLER}/pose_error',
                                 self._on_err, 10)
        node.create_subscription(Float64MultiArray, f'{CONTROLLER}/commanded_wrench',
                                 self._on_wr, 10)
        node.create_subscription(WrenchStamped, f'{CONTROLLER}/external_wrench',
                                 self._on_ext, 10)
        node.create_subscription(String, f'{MODE_NS}/current_mode', self._on_mode, 10)

        self._build_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)

    # ---------------- ROS callbacks (executor thread) -----------------
    def _on_err(self, m):
        if len(m.data) >= 6:
            self._pose_err = list(m.data[:6])

    def _on_wr(self, m):
        if len(m.data) >= 6:
            self._cmd_wrench = list(m.data[:6])

    def _on_ext(self, m):
        f, t = m.wrench.force, m.wrench.torque
        self._ext_wrench = [f.x, f.y, f.z, t.x, t.y, t.z]

    def _on_mode(self, m):
        self._mode = m.data

    # ---------------- UI ----------------------------------------------
    def _build_ui(self):
        root = QVBoxLayout(self)

        # mode row
        mode_box = QGroupBox('控制模式  (运行时切换,无需重启)')
        ml = QHBoxLayout(mode_box)
        self.mode_lbl = QLabel('当前: unknown')
        self.mode_lbl.setStyleSheet('font-weight: bold;')
        self.btn_pos = QPushButton('切到位控 (MoveIt)')
        self.btn_force = QPushButton('切到力控 (阻抗,自动启用重力补偿)')
        self.btn_pos.clicked.connect(self._switch_to_position)
        self.btn_force.clicked.connect(self._switch_to_force)
        ml.addWidget(self.mode_lbl)
        ml.addWidget(self.btn_pos)
        ml.addWidget(self.btn_force)
        root.addWidget(mode_box)

        # impedance enable/tare + spring toggle row
        imp_box = QGroupBox('力控:重力补偿(0重力)/ 阻抗弹簧')
        il = QHBoxLayout(imp_box)
        self.btn_enable = QPushButton('启用 / 停用 (重力补偿)')
        self.btn_enable.setCheckable(True)
        self.btn_enable.toggled.connect(self._on_enable_toggle)
        # Cartesian stiffness on/off. OFF = 0-gravity free-drive (K=0); ON =
        # apply the K spinbox values (with a re-seed so the spring starts at rest).
        self.spring_check = QCheckBox('阻抗弹簧 (Cartesian 刚度 K>0)')
        self.spring_check.toggled.connect(self._on_spring_toggle)
        self.btn_tare = QPushButton('外力置零 (tare)')
        self.btn_tare.clicked.connect(lambda: self._call(self._tare_cli, '外力置零'))
        il.addWidget(self.btn_enable)
        il.addWidget(self.spring_check)
        il.addWidget(self.btn_tare)
        root.addWidget(imp_box)

        # target pose offset (only effective when the spring K>0 is on)
        tgt_box = QGroupBox('目标位姿偏移 (相对 seed;仅阻抗弹簧开时有效)')
        tgl = QVBoxLayout(tgt_box)
        # row 1: frame selector + translation
        r1 = QHBoxLayout()
        r1.addWidget(QLabel('坐标系'))
        self.frame_combo = QComboBox()
        self.frame_combo.addItems(['base_link (世界系)', '末端 tool (工具系)'])
        self.frame_combo.currentIndexChanged.connect(lambda *_: self._publish_target())
        r1.addWidget(self.frame_combo)
        self.offset_spins = {}
        for i, label in enumerate(['dX', 'dY', 'dZ']):
            r1.addWidget(QLabel(label))
            s = self._mk_spin(-0.15, 0.15, 0.005, 4, 0.0)
            s.valueChanged.connect(lambda *_: self._publish_target())
            self.offset_spins[i] = s
            r1.addWidget(s)
        tgl.addLayout(r1)
        # row 2: rotation (rpy, rad) + re-seed
        r2 = QHBoxLayout()
        for i, label in enumerate(['dRoll', 'dPitch', 'dYaw']):
            r2.addWidget(QLabel(label))
            s = self._mk_spin(-0.8, 0.8, 0.02, 3, 0.0)
            s.valueChanged.connect(lambda *_: self._publish_target())
            self.offset_spins[3 + i] = s
            r2.addWidget(s)
        btn_reseed = QPushButton('重新锚定到当前 TCP')
        btn_reseed.clicked.connect(self._reseed_target)
        r2.addWidget(btn_reseed)
        tgl.addLayout(r2)
        root.addWidget(tgt_box)

        # stiffness + damping grids
        grids = QHBoxLayout()
        grids.addWidget(self._spin_group('刚度 K (弹簧开时生效)', STIFFNESS,
                                         self._apply_stiffness, 'stiffness'))
        grids.addWidget(self._spin_group('阻尼比 ζ (实时)', DAMPING,
                                         self._apply_damping, 'damping_ratio'))
        root.addLayout(grids)

        # friction comp
        fric_box = QGroupBox('摩擦补偿 (抖动时调小 scale)')
        fl = QHBoxLayout(fric_box)
        self.dz_spin = self._mk_spin(0.0, 0.3, 0.01, 3, 0.05)
        self.scale_spin = self._mk_spin(0.0, 1.5, 0.05, 2, 1.0)
        self.dz_spin.valueChanged.connect(
            lambda v: self._set_double('velocity_deadband', v))
        self.scale_spin.valueChanged.connect(
            lambda v: self._set_double('friction_compensation_scale', v))
        fl.addWidget(QLabel('死区 dz'))
        fl.addWidget(self.dz_spin)
        fl.addWidget(QLabel('scale'))
        fl.addWidget(self.scale_spin)
        btn_fric_off = QPushButton('关闭摩擦补偿')
        btn_fric_off.clicked.connect(self._fric_off)
        fl.addWidget(btn_fric_off)
        root.addWidget(fric_box)

        # null-space control (7-DoF redundancy: stops elbow drift -> no joint
        # jump across mode switches)
        ns_box = QGroupBox('零空间控制 (7自由度冗余,防关节漂移/突变)')
        nl = QHBoxLayout(ns_box)
        self.ns_damp_spin = self._mk_spin(0.0, 50.0, 0.5, 2, 1.0)
        self.ns_stiff_spin = self._mk_spin(0.0, 200.0, 1.0, 1, 0.0)
        self.ns_damp_spin.valueChanged.connect(
            lambda v: self._set_double('nullspace_damping', v))
        self.ns_stiff_spin.valueChanged.connect(
            lambda v: self._set_double('nullspace_stiffness', v))
        nl.addWidget(QLabel('阻尼 (防漂移)'))
        nl.addWidget(self.ns_damp_spin)
        nl.addWidget(QLabel('刚度 (锁姿态)'))
        nl.addWidget(self.ns_stiff_spin)
        root.addWidget(ns_box)

        # presets
        preset_box = QGroupBox('刚度预设')
        pl = QHBoxLayout(preset_box)
        for label, ks, ds in [
            ('保守起点', [100, 100, 100, 10, 10, 10], [1.0] * 6),
            ('稳定基线', [500, 500, 500, 30, 30, 30], [0.4, 0.4, 0.4, 0.2, 0.2, 0.2]),
            ('打螺丝', [100, 100, 2000, 50, 50, 5], [0.7, 0.7, 1.0, 0.7, 0.7, 0.5]),
        ]:
            b = QPushButton(label)
            b.clicked.connect(lambda _c, k=ks, d=ds: self._apply_preset(k, d))
            pl.addWidget(b)
        root.addWidget(preset_box)

        # readouts
        mon_box = QGroupBox('实时监控')
        mol = QVBoxLayout(mon_box)
        self.err_lbl = QLabel('位姿误差: ...')
        self.wr_lbl = QLabel('力旋量: ...')
        self.ext_lbl = QLabel('外力估计: ...')
        self.ext_lbl.setStyleSheet('color: #0066cc;')
        for w in (self.err_lbl, self.wr_lbl, self.ext_lbl):
            w.setFrameShape(QFrame.NoFrame)
            mol.addWidget(w)
        root.addWidget(mon_box)

        self.status_lbl = QLabel('')
        self.status_lbl.setStyleSheet('color: #555;')
        root.addWidget(self.status_lbl)
        root.addStretch(1)

    def _mk_spin(self, lo, hi, step, decimals, val):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        s.setValue(val)
        s.setKeyboardTracking(False)   # only emit on commit, not per keystroke
        return s

    def _spin_group(self, title, spec, apply_fn, _pname):
        box = QGroupBox(title)
        grid = QGridLayout(box)
        spins = {}
        for r, (label, idx, lo, hi, step, dec) in enumerate(spec):
            grid.addWidget(QLabel(label), r, 0)
            default = 500.0 if title.startswith('刚度') and idx < 3 else (
                30.0 if title.startswith('刚度') else (0.4 if idx < 3 else 0.2))
            s = self._mk_spin(lo, hi, step, dec, default)
            s.valueChanged.connect(lambda *_: apply_fn())
            grid.addWidget(s, r, 1)
            spins[idx] = s
        if title.startswith('刚度'):
            self.stiff_spins = spins
        else:
            self.damp_spins = spins
        return box

    # ---------------- handlers ----------------------------------------
    def _set_enable_button(self, checked):
        """Reflect impedance enabled state without re-firing the enable service."""
        self.btn_enable.blockSignals(True)
        self.btn_enable.setChecked(checked)
        self.btn_enable.blockSignals(False)

    def _switch_to_force(self):
        # to_force engages ZERO-GRAVITY (gravity comp on, K=0), so reflect:
        # enabled = on, spring = off.
        self._call(self._to_force_cli, '力控')
        self._set_enable_button(True)
        self._set_spring_check(False)

    def _switch_to_position(self):
        self._call(self._to_pos_cli, '位控')
        self._set_enable_button(False)
        self._set_spring_check(False)

    def _set_spring_check(self, checked):
        self.spring_check.blockSignals(True)
        self.spring_check.setChecked(checked)
        self.spring_check.blockSignals(False)

    def _on_spring_toggle(self, checked):
        if checked:
            # spring ON: re-seed target to current pose (so the spring starts at
            # rest, no jerk), then apply the K spinbox values.
            self._reseed_target()
            self._set_double_array('stiffness', [self.stiff_spins[i].value() for i in range(6)])
            self._status = '阻抗弹簧已开 (K>0)'
        else:
            self._set_double_array('stiffness', [0.0] * 6)
            self._status = '阻抗弹簧已关 (0重力,K=0)'

    def _reseed_target(self):
        try:
            t = self._tf_buffer.lookup_transform(
                BASE_FRAME, TIP_FRAME, Time(), timeout=Duration(seconds=0.2))
            self._seed_tf = t
            for s in self.offset_spins.values():
                s.blockSignals(True)
                s.setValue(0.0)
                s.blockSignals(False)
            self._publish_target()
            self._status = ('seed: (%.3f, %.3f, %.3f)' % (
                t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z))
        except Exception as e:  # noqa: BLE001
            self._status = f'重新锚定失败 (TF): {e}'

    def _publish_target(self):
        if self._seed_tf is None:
            return
        tr = self._seed_tf.transform
        p0 = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        q0 = np.array([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
        d_trans = np.array([self.offset_spins[i].value() for i in range(3)])
        d_rpy = [self.offset_spins[3 + i].value() for i in range(3)]
        dq = quat_from_rpy(*d_rpy)

        tool_frame = self.frame_combo.currentIndex() == 1
        if tool_frame:
            # translation along the tool axes; rotation in the tool frame
            p = p0 + quat_rotate(q0, d_trans)
            q = quat_mul(q0, dq)
        else:
            # translation in base (world) axes; rotation in the base frame
            p = p0 + d_trans
            q = quat_mul(dq, q0)
        q = q / np.linalg.norm(q)

        msg = PoseStamped()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        self._target_pub.publish(msg)

    def _on_enable_toggle(self, checked):
        req = SetBool.Request()
        req.data = bool(checked)
        self._call_setbool(self._enable_cli, req,
                           '阻抗启用' if checked else '阻抗停用')

    def _apply_stiffness(self):
        # Only push K to the controller when the spring is ON. With it OFF we are
        # in 0-gravity mode and the spinboxes just hold the to-be-applied values.
        if self.spring_check.isChecked():
            self._set_double_array(
                'stiffness', [self.stiff_spins[i].value() for i in range(6)])

    def _apply_damping(self):
        self._set_double_array('damping_ratio', [self.damp_spins[i].value() for i in range(6)])

    def _apply_preset(self, ks, ds):
        for i in range(6):
            self.stiff_spins[i].blockSignals(True)
            self.stiff_spins[i].setValue(float(ks[i]))
            self.stiff_spins[i].blockSignals(False)
            self.damp_spins[i].blockSignals(True)
            self.damp_spins[i].setValue(float(ds[i]))
            self.damp_spins[i].blockSignals(False)
        # Choosing a stiffness preset implies turning the spring on (re-seeds +
        # applies K). Damping is applied regardless.
        self._apply_damping()
        if self.spring_check.isChecked():
            self._set_double_array('stiffness', list(map(float, ks)))
        else:
            self._set_spring_check(True)
            self._on_spring_toggle(True)

    def _fric_off(self):
        self.scale_spin.setValue(0.0)
        self.dz_spin.setValue(0.3)

    # ---------------- ROS helpers -------------------------------------
    def _call(self, cli, label):
        if not cli.service_is_ready():
            self._status = f'{label}: 服务未就绪'
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self._stash(label, f))

    def _call_setbool(self, cli, req, label):
        if not cli.service_is_ready():
            self._status = f'{label}: 服务未就绪'
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: self._stash(label, f))

    def _stash(self, label, fut):
        try:
            res = fut.result()
            self._status = f'{label}: {getattr(res, "message", "ok")}'
        except Exception as e:  # noqa: BLE001
            self._status = f'{label}: {e}'

    def _set_double_array(self, name, values):
        self._send_param(name, ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in values]), name)

    def _set_double(self, name, value):
        self._send_param(name, ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)), name)

    def _send_param(self, name, pv, label):
        if not self._setparam_cli.service_is_ready():
            self._status = f'{label}: set_parameters 未就绪'
            return
        p = ParamMsg()
        p.name = name
        p.value = pv
        req = SetParameters.Request(parameters=[p])
        fut = self._setparam_cli.call_async(req)

        def _done(f, _label=label):
            try:
                r = f.result().results[0]
                if not r.successful:
                    self._status = f'{_label}: 拒绝 {r.reason}'
            except Exception as e:  # noqa: BLE001
                self._status = f'{_label}: {e}'
        fut.add_done_callback(_done)

    # ---------------- GUI-thread refresh ------------------------------
    def _refresh(self):
        m = self._mode
        self.mode_lbl.setText(f'当前: {m}')
        if m == 'position':
            self.mode_lbl.setStyleSheet('font-weight: bold; color: #1565c0;')
        elif m == 'force':
            self.mode_lbl.setStyleSheet('font-weight: bold; color: #c62828;')
        else:
            self.mode_lbl.setStyleSheet('font-weight: bold; color: #777;')
        e = self._pose_err
        w = self._cmd_wrench
        x = self._ext_wrench
        self.err_lbl.setText(
            '位姿误差  trans [%+.4f %+.4f %+.4f] m  rot [%+.4f %+.4f %+.4f] rad'
            % tuple(e))
        self.wr_lbl.setText(
            '力旋量    F [%+7.2f %+7.2f %+7.2f] N  M [%+6.2f %+6.2f %+6.2f] Nm'
            % tuple(w))
        self.ext_lbl.setText(
            '外力估计  F [%+7.2f %+7.2f %+7.2f] N  M [%+6.2f %+6.2f %+6.2f] Nm'
            % tuple(x))
        self.status_lbl.setText(self._status)

    def shutdown(self):
        self._timer.stop()
