# armv7_impedance_moveit — v0.2 implementation checklist

Status legend: `[ ]` open  `[~]` partial / draft  `[x]` done.

## Controller core

- [x] **Cartesian impedance controller plugin** (`CartesianImpedanceController`).
      Implemented in `src/cartesian_impedance_controller.cpp`. Reuses gravity +
      friction compensation from `armv7_zero_force_controller` for the joint
      baseline; adds spring-damper + Jᵀ for Cartesian tracking. Validated in
      fake mode: 5 cm Z target offset → 20 N Z wrench (= K_z · max_pose_error).
- [ ] **Stiffness in tool frame**. Currently in `root_link`. Once a tool frame
      is hot-loadable via `armv7_tcp`, support `stiffness_frame: tcp` to align
      the anisotropy with the bit no matter the arm orientation. Pattern:
      transform the per-axis K diagonal into base via `R_tcp` before the
      multiply. Watch out: damping ratio also rotates.
- [ ] **Cartesian admittance controller plugin** as a sibling — same chain,
      same param layout, opposite direction (`external_wrench → desired velocity`).
      Skip if the screw primitive is fine with pure impedance. Defer.

## External wrench

The screw primitive needs to know `F_ext` (downward force on the bit, torque
about the bit axis) to detect seating. Pick a path:

- [ ] **Path (a) — F/T sensor.** Easiest if a sensor is in the EE chain. Wire
      it via [`docs/integration/ft_sensor.md`](../../docs/integration/ft_sensor.md);
      the impedance controller subscribes to `/wrench` (sensor frame) and
      transforms to `root_link`. Subtracts a tare snapshot to remove the
      cable+EE static load. Defer the actual subscription until hardware sensor
      is on the arm; signature is already in `impedance_interface.hpp`.
- [x] **Path (b) — joint-torque residual estimator.** SHIPPED (v1, gravity-only).
      `F_ext = (J Jᵀ + λI)⁻¹ J (τ_measured − gravity_scale·G(q))`, low-passed and
      tared, published on `~/external_wrench`. Tare on enable (after ramp) and via
      `~/tare_external_wrench`. Dashboard shows readout + curve. **v2 TODO:**
      subtract the friction model from the residual (currently gravity-only, so
      the estimate is accurate only at low velocity); add identified inertia
      tensor so the `M q̈ + C q̇` terms can be subtracted during motion.

## Screw-driving primitive (separate node)

Schedules the impedance controller. Not a controller_interface plugin — it's a
regular `rclpy` / `rclcpp` node sequencing service / topic calls.

- [ ] **`screw_primitive` node** (new package or under `armv7_examples`).
      State machine:
  1. APPROACH    — high stiffness all axes, MoveIt or direct pose to
                   `screw_head_pose + Δz_clear`.
  2. SOFT-XY      — push `stiffness = [k_lo, k_lo, k_hi, k_lo, k_lo, k_free]`
                    via `~/set_impedance` service.
  3. ENGAGE       — target descends `Δz_clear` along Z slowly.
  4. DRIVE        — joint7 cyclic command; monitor `external_wrench`.
                    Stop condition: `F_z > F_seat_threshold` OR
                                    `τ_z > τ_seat_threshold` OR
                                    travel exceeds `Δz_max`.
  5. RETRACT      — restore stiffness; pose back to APPROACH.
  Abort condition: `~/safety/estop` flips true → stop joint7, command zero
  effort via `~/enable false`, raise.
- [x] **Live stiffness / damping_ratio / wrench_limit / max_pose_error /
      gravity_scale updates.** Implemented via `on_set_parameters_callback` +
      `realtime_tools::RealtimeBuffer` swap. The `tuning_dashboard` GUI hits
      `~/set_parameters` directly. Atomic-or-nothing: any invalid entry
      rejects the whole batch and the controller never sees a half-updated
      set. Per-joint safety knobs (`max_torque`, `velocity_limit`, friction,
      `damping`) intentionally NOT in the live set — those still require
      deactivate / reactivate.
- [ ] **`SetImpedance` service msg** under `armv7_impedance_moveit/srv/`.
      The live `~/set_parameters` path above already covers what the screw
      primitive needs at v0.2 alpha; a dedicated msg is mostly an API-clarity
      nicety and can be deferred to v0.2.1.

## Safety

- [ ] **`armv7_safety` integration**. Subscribe to `/safety/estop` (latched)
      in the impedance controller; when true, set `enabled_=false` and zero
      effort the same cycle.
- [ ] **First-activation guard**. If the seed pose differs from `target_pose_`
      by more than `0.5 * max_pose_error` on any axis, refuse activation and
      log why. Catches "operator forgot to re-publish target after moving the
      arm" surprises.
- [ ] **Wrench-rate clamp**. Today's `max_pose_error` caps wrench *magnitude*
      but a target step still produces an instantaneous wrench. Add a
      slew-rate limit on `target_pose_` (smoothed setpoint) for sharper
      operator-supplied targets.

## Calibration / model improvements (needed downstream)

- [ ] Extend `armv7_dyn_ident` with a **dynamic-parameter identifier** that
      reuses the Fourier excitation generator already shipped
      (`excitation.py`). Needed for residual-based external-wrench estimate to
      be quiet under motion.
- [ ] **Friction map**. Coulomb is a scalar per joint today; cogging from
      harmonic drives is position-dependent. Identify a lookup table per joint
      and let the controller interpolate. The `est. Coulomb |f_c|` data from
      bidirectional sampling is already per-pose — just don't aggregate to a
      single median.

## Testing

- [ ] Hardware acceptance gate (modelled on
      [`docs/testing_phase4.md`](../../docs/testing_phase4.md) §4.2.2):
  - Operator can lock 5 of 6 DOFs at `K = 2000 N/m` and slide the TCP along
    the compliant axis with < 5 N of effort.
  - Steady-state Cartesian error < 2 mm holding a 5 kg payload disturbance.
  - E-Stop → controller disabled within 100 ms.
- [ ] `docs/testing_phase5.md` — once the controller has been tuned on real
      hardware (any phase-4-style step-by-step acceptance harness).
- [ ] Synthetic unit tests for the pose-error helper (axis-angle extraction
      around the `±π` boundary; near-identity numerical stability).

## Documentation

- [x] Top README rewritten for v0.2 alpha.
- [ ] Per-axis stiffness tuning notebook (Jupyter or
      `armv7_examples/cartesian_tuning.py`) — push the arm, plot wrench
      response, fit `K_eff`. Useful for first-real-hardware bring-up.
- [ ] "Why screw-driving uses impedance not pure position" 1-pager — geometric
      misalignment tolerance, force-on-seat detection, why high task-space K
      is the wrong knob for it.
- [ ] Friction-vs-Cartesian-compliance interaction note. High Coulomb
      compensation + low Cartesian K interact unintuitively (the controller
      may push the arm along the operator-applied direction).

---

Pick a box, open a PR. The v0.2 interface contract in
[`include/armv7_impedance_moveit/impedance_interface.hpp`](include/armv7_impedance_moveit/impedance_interface.hpp)
is the API consumers depend on — extend it but try not to break it.
