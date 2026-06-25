# armv7_impedance_moveit

**Status:** v0.2 **alpha** — Cartesian impedance is working in fake mode; first
hardware bring-up is the next step. Use the v0.1
[`armv7_zero_force_controller`](../armv7_zero_force_controller/README.md) for
free-drive teaching while this loop is still being characterised on the arm.

The end-target is a **screw-driving primitive**: low XY stiffness for
self-alignment, high Z stiffness against the workpiece, controlled rotation
about the tool axis with a torque threshold to detect seating. This package
ships the underlying Cartesian impedance controller that primitive will sit on.

---

## 1. What `CartesianImpedanceController` does

Each ros2_control cycle:

```
e_p  = p_target − p_current                            (translation error)
e_R  = axis * angle of (R_target · R_currentᵀ)         (rotation error vector)
v    = J(q) · q̇                                        (Cartesian twist)
W    = diag(K) · e − diag(D) · v                       (spring-damper)
τ    = JᵀW
       + gravity_scale · G(q)                          ← reused from v0.1
       + coulomb_friction · tanh(q̇ / eps)              ← reused from v0.1
       + viscous_friction · q̇
       − damping · q̇
```

Each step has a hard safety cap: `max_pose_error` clips `e` before the spring;
`wrench_limit` clips `W` before `Jᵀ`; `max_torque` clips `τ`; `velocity_limit`
zeroes a joint that runs away. The whole vector is scaled by an activation
ramp and gated by an atomic enable flag (same patterns as
`armv7_zero_force_controller`, so safety story carries over).

## 2. ROS interface

**Subscribers**
- `~/target_pose` — `geometry_msgs/PoseStamped` (in `root_link` frame).
  On activate the target is seeded to the current TCP pose, so commanded
  wrench is 0 until you publish a new target.

**Service**
- `~/enable` — `std_srvs/SetBool`. Controller starts **disabled**. Every
  `disabled → enabled` transition **automatically re-seeds the target pose to
  the current TCP**, so the arm holds wherever it is at the moment of enable —
  not where it was minutes earlier when the controller activated. (This avoids
  a yank back if the arm sagged or was moved manually while disabled.)

**Debug / data topics** (published every cycle)
- `~/pose_error` — `Float64MultiArray`, 6 entries `[ex, ey, ez, eRx, eRy, eRz]` (m / rad).
- `~/commanded_wrench` — `Float64MultiArray`, 6 entries `[Fx, Fy, Fz, Mx, My, Mz]` (N / N·m).
- `~/commanded_torque` — `Float64MultiArray`, N entries, effort written to each joint.
- `~/external_wrench` — `geometry_msgs/WrenchStamped`, estimated external wrench
  on the TCP (residual method, see §5.7).

**Tare service**
- `~/tare_external_wrench` — `std_srvs/Trigger`, zeroes the external-wrench
  estimate at the current (settled) reading.

## 3. Parameters (`config/impedance.yaml`)

| Group | Param | Default | Notes |
|---|---|---|---|
| Cartesian | `stiffness` | `[200, 200, 200, 20, 20, 20]` | N/m, N·m/rad. Diagonal, in `root_link`. |
| | `damping_ratio` | `[0.7, …]` | 0–1. Critical damping ratio per axis. |
| | `wrench_limit` | `[60, 60, 60, 10, 10, 10]` | Hard cap before Jᵀ. |
| | `max_pose_error` | `[0.10, 0.10, 0.10, 0.50, 0.50, 0.50]` | Clip *before* multiplying by K — keeps a bad target safe. |
| | `root_link` / `tip_link` | `base_link` / `link7` | KDL chain. |
| Joint baseline | `gravity_scale` / `gravity_vector` | `0.9` / `[0,0,-g]` | Same as v0.1. |
| | `max_torque` / `velocity_limit` | per-joint Nm / 2.0 rad/s | Hard safety. |
| | `damping` | `[0.5, …]` | Joint-space viscous. |
| | `coulomb_friction` / `viscous_friction` / `coulomb_velocity_eps` | from `armv7_dyn_ident` | Reuse v0.1 calibration. |
| | `ramp_in_time` | `2.0 s` | Activation ramp. |
| | `enable_at_start` | `false` | Start disabled. |
| Optional | `identified_params_file` | (empty → URDF) | `armv7_dyn_ident` output. |

## 4. Usage

```bash
# real hardware (drives in CST / mode 10):
ros2 launch armv7_impedance_moveit impedance.launch.py

# fake hardware (no physics; verifies the loop loads + math):
ros2 launch armv7_impedance_moveit impedance.launch.py \
    use_fake_hardware:=true use_rt:=false

# enable (only after operator is clear of the arm):
ros2 service call /cartesian_impedance_controller/enable \
    std_srvs/srv/SetBool "{data: true}"

# send a target pose (BASE frame):
ros2 topic pub --once /cartesian_impedance_controller/target_pose \
    geometry_msgs/msg/PoseStamped \
    "{header:{frame_id: base_link}, pose:{position:{x: 0.4, y: 0.0, z: 0.5},
      orientation:{w: 1.0}}}"

# watch the spring force in real time:
ros2 topic echo /cartesian_impedance_controller/commanded_wrench
```

> Same mutual-exclusion as free-drive: impedance loads in CST and cannot
> coexist with MoveIt position-mode tracking on the same drives. Use
> `arm.launch.py` when you need OMPL planning back.

## 4.5. Tuning dashboard (`tuning_dashboard`)

A Tkinter GUI for interactive tuning. Three workspaces in one window:

1. **Big ENABLE / DISABLE toggle**, colour-coded (red = off, green = on). Wraps
   the `~/enable` service.
2. **Target offset sliders** (X / Y / Z, ±10 cm relative to a captured seed
   pose). Slider movement publishes `~/target_pose` immediately. "Re-seed to
   current TCP" snapshots the current `link7` via TF as the new zero-offset
   pose. "Reset offsets" zeros all sliders (and re-publishes the seed pose).
3. **Live stiffness + damping_ratio sliders.** Slider release calls
   `~/set_parameters` directly; the controller's `on_set_parameters` callback
   validates and swaps the live params via a `realtime_tools::RealtimeBuffer`
   **without restarting the controller**. Includes a *Screw preset* button
   that one-shot applies `K = [100, 100, 2000, 50, 50, 5]` and matching
   damping — XY soft, Z stiff, Rz free.
4. **Live readouts** of `pose_error`, `commanded_wrench`, and
   `commanded_torque` refresh at ~10 Hz.

```bash
ros2 run armv7_impedance_moveit tuning_dashboard
```

> Note: **`ros2 param set /cartesian_impedance_controller stiffness ...` shows
> "Node not found"** in some ROS 2 Humble versions because the CLI daemon
> doesn't surface controller lifecycle nodes for param ops. The underlying
> service `/cartesian_impedance_controller/set_parameters` works regardless,
> and the dashboard talks to it directly.

Validation, fake mode: K=200 → wrench[0] ≈ 10 N at 5 cm X offset; live set
K_x=400 → wrench[0] ≈ 20 N **in the next control cycle**, no restart.
Negative K is rejected with the controller's existing wrench atomic.

## 5. Tuning playbook

1. **Start conservative.** Defaults are deliberately soft (`K_trans=200, K_rot=20`).
   At those gains a 5 cm Z error gives 10 N wrench — comparable to the friction
   you already calibrated.
2. **Bring up in fake mode first.** Confirm `commanded_wrench` matches `K·e`
   for hand-picked targets. The math is straightforward; if it doesn't match,
   it's a frame mismatch (`root_link` / target frame_id) not the controller.
3. **First real-hardware run**: target = current pose, enable, observe the
   arm sits still. If it twitches, lower `gravity_scale` or `coulomb_friction`
   per joint until it does (re-run the steady-state check from `testing_phase4`).
4. **Push manually.** With low K, the operator can feel the spring; verify
   restoring force is reasonable (and direction matches expectation).
5. **Crank stiffness up incrementally** — double `K_trans` until the arm holds
   ~0.5 mm at the seed pose under normal handling. Beyond ~5000 N/m the
   discrete-time loop at 200 Hz needs a closer look at damping; bump up
   `damping_ratio` toward 1.0 before pushing K higher.
6. **Anisotropic gains** for the screw use case: `K = [100, 100, 2000, 50, 50, 5]`
   makes XY very compliant (1 cm error → 1 N), Z stiff against the screw head,
   and rotation about Z (the driving axis) free.

## 5.5. Why the arm shakes in impedance mode (and the fix)

The free-drive friction-comp recipe `f = coulomb · tanh(q̇/eps)` works great
for hand-guidance, but **becomes a positive-feedback loop the instant a
Cartesian spring closes around it**:

```
At |q̇| << eps,  tanh(q̇/eps) ≈ q̇/eps,  so  f_coulomb ≈ (coulomb/eps) · q̇
```

For the calibrated `coulomb_friction[joint1] = 2.2 Nm` with `eps = 0.05`, that
is **44 Nm·s/rad of equivalent positive damping** — vastly exceeding our
`damping = 0.5 Nm·s/rad`. Free-drive doesn't excite this loop (there is no
spring closing it); impedance does, and the arm diverges into a limit-cycle
shake whose amplitude grows until `max_torque` clips it.

**Fix shipped in this controller:** every cycle the velocity is shifted by
`velocity_deadband` before going into `tanh`, so

```
q̇_eff = sign(q̇) · max(0, |q̇| − velocity_deadband)
f_coulomb = friction_compensation_scale · coulomb · tanh(q̇_eff / eps)
```

At `|q̇| < velocity_deadband` the comp is **exactly 0** — no positive feedback
near rest. Above the deadband the comp engages normally for intentional pushes.

The default `velocity_deadband: 0.05 rad/s` is roughly the velocity at which
spring oscillation would otherwise visibly start; if you still see shake, dial
**`friction_compensation_scale`** down (it scales both Coulomb AND viscous
comp from 1.0 to 0.0, no restart). Both are live-tunable from the dashboard's
"Friction comp" pane:

- **dz** slider — `velocity_deadband` 0…0.3 rad/s.
- **scale** slider — `friction_compensation_scale` 0…1.5×.
- **Disable friction comp** button — `scale → 0`, `dz → 0.3` in one shot
  (proves the diagnosis: if the shake stops, friction comp was the cause).
- **Defaults** button — back to `dz=0.05`, `scale=1.0`.

Tuning recipe on real hardware:
1. Start with default `dz=0.05`, `scale=1.0`. If stable, you're done.
2. If shaking: press "Disable friction comp". Shake gone → confirmed.
3. Gradually raise scale back toward 1.0 with `dz` at 0.05; stop where the
   first signs of oscillation appear, back off 20%.
4. Larger `dz` helps stability but makes free-drive feel heavier at start of a
   push. 0.05–0.1 is the usable range.

## 5.7. External-wrench estimation (no F/T sensor)

The controller estimates the external wrench on the TCP from the joint-torque
residual — **no force/torque sensor required**:

```
tau_ext = tau_measured − gravity_scale · G(q)          (gravity-only residual)
F_ext   = (J Jᵀ + λI)⁻¹ J · tau_ext                    (damped least squares)
F_ext   = lowpass(F_ext) − tare
```

Published on `~/external_wrench` (`geometry_msgs/WrenchStamped`, in
`external_wrench_frame`, default `base_link`). The dashboard's "外力估计" line
and the bottom wrench plot show it live.

**Tare** ("外力置零" button / `~/tare_external_wrench` service): captures the
current reading as the new zero, so the standing load (payload, cable tension,
gravity-model error) reads 0 and only *changes* are reported. Tare automatically
on every enable, after the ramp-in completes, and any time you press the button
while the arm is at rest.

Parameters (`config/impedance.yaml`):
- `external_wrench_lambda` (default 0.01) — damped pseudo-inverse regularisation.
  Smaller = higher fidelity away from singularities, noisier near them.
- `external_wrench_filter_cutoff` (5.0 Hz) — output low-pass.
- `external_wrench_frame` (base_link) — published frame.

**Caveats (v1):**
- **Gravity-only residual.** Friction is NOT subtracted (velocity-dependent and
  sign-fragile). The estimate is most accurate at **low velocity** — exactly the
  screw-seating regime (press and hold). Fast manual pushes read noisier. v2:
  add the friction model + identified inertia tensor.
- **Singular poses attenuate it.** At/near a singular configuration (e.g. the
  q=0 upright pose) the damped pseudo-inverse correctly degrades the estimate
  toward 0 in the degenerate directions — you genuinely can't observe wrench the
  Jacobian can't map. Work at a non-singular pose.
- **Fake hardware = plumbing only.** mock_components mirrors the commanded
  torque and doesn't simulate dynamics, so fake mode validates the pipeline
  (residual → pinv → filter → tare → publish), not real external-force physics.
  Real validation: push the TCP on hardware and watch `F_ext`.

## 6. Known limits (v0.2 alpha)

- **Stiffness frame = `root_link`.** Tool-frame stiffness (anisotropy aligned
  with the bit, even when the arm is tilted) is a v0.2.1 follow-up.
- **No null-space damping policy.** With 7 DoF and a 6 DoF task there is a
  free internal motion; the v0.1 joint-space `damping` keeps it quiet but
  this is not a proper redundancy resolution.
- **No external-wrench input yet.** The screw primitive will need a wrench
  source — either F/T sensor (`/wrench` topic) or joint-torque residual
  estimator. Both planned next; see [TODO.md](TODO.md).
- **No admittance sibling.** Plan called for both modes; impedance is the
  primary path for screw-driving so it shipped first.
- **First-hardware data not yet collected.** Tuning numbers above are
  starting points from the v0.1 calibration, not measurements at high
  stiffness.

## 7. Roadmap to screw-driving

| Step | What's needed | Status |
|---|---|---|
| 1 | Working Cartesian impedance | ✅ (this package) |
| 2 | External-wrench sensing | ⏳ joint-torque residual estimator OR F/T sensor |
| 3 | Tool-frame anisotropy | ⏳ v0.2.1 |
| 4 | Screw-drive primitive node | ⏳ schedules stiffness change + joint7 rotation + termination on `F_z` / `τ_z` / depth |
| 5 | Hardware tuning + safety cell | ⏳ first attempt with a test screw + dummy workpiece |

See [TODO.md](TODO.md) for the per-step checklist with package references.
