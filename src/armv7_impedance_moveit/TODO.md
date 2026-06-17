# armv7_impedance_moveit — v0.2 implementation checklist

Status legend: `[ ]` open  `[~]` partial / draft  `[x]` done.
Every item points at a v0.1 package whose pattern to mirror.

## Controller core

- [ ] **Cartesian impedance controller plugin** (`controller_interface::ControllerInterface`).
      Mirror `armv7_zero_force_controller/src/gravity_compensation_controller.cpp`
      for parameter handling, KDL chain build, enable service, ramp-in, torque
      clamping. Add a `~/target_pose` subscriber and the `pose_error → wrench`
      spring-damper.
- [ ] **Cartesian admittance controller plugin** as a sibling — same chain, same
      param layout, different direction: `external_wrench → desired velocity`.
      Skipping in v0.2 is acceptable; keep impedance as the single controller.
- [ ] **External wrench estimator**. Two paths, pick one to ship:
  - (a) Read from `/wrench` topic (F/T sensor exists). Trivial; the limit is
        sensor availability and frame transforms.
  - (b) Estimate from joint torque residual:
        `F_ext = (Jᵀ)⁺ (τ_measured - G(q) - τ_friction)`.
        Needs the friction model from `armv7_zero_force_controller` AND a
        tighter version of `armv7_dyn_ident` that identifies the inertia
        tensor (not just first moments). `armv7_dyn_ident` currently
        identifies gravity-only parameters; an extension that adds a
        Fourier-trajectory regressor over `qdd` is the natural next step.
- [ ] **`SetImpedance` service msg** under `armv7_impedance_moveit/srv/`. Carries
      `CartesianImpedanceParams`. ROS msg files require regenerating once when
      the service is defined; the existing `armv7_safety` package can be the
      template (it ships an action / service definition pattern).

## MoveIt integration

- [ ] **`MoveItControllerHandle` adapter** so MoveIt can stream waypoints to
      this controller exactly as it streams to `plan_group_controller`. The
      `armv7_moveit_config/config/moveit_controllers.yaml` already declares
      one such handle — clone that block with the impedance controller's
      action name.
- [ ] **Constrained-Cartesian planning** wrapper. OMPL plans in joint space;
      add a thin MoveItCpp wrapper that takes a desired Cartesian path + an
      anisotropic compliance and lets the impedance loop "stay near" the path.

## Calibration / model improvements

- [ ] Extend `armv7_dyn_ident` with a **dynamic-parameter identifier** that
      reuses the existing Fourier excitation generator (`excitation.py`
      already has `fourier_trajectory` and `random_fourier_coeffs`) and adds
      the inertia-tensor regressor. Output schema lives next to gravity
      params in `identified_params.yaml`.
- [ ] **Friction map** — current `coulomb_friction` is a scalar per joint;
      a real harmonic drive's friction depends on joint position (cogging)
      and load. Identify a lookup table per joint and let the controller
      interpolate.

## Safety

- [ ] **`armv7_safety` integration**. The workspace-bbox check should mute
      the impedance loop the moment the TCP leaves bounds (today it only
      publishes `/safety/in_bounds` — add a service hook the impedance
      controller subscribes to).
- [ ] **E-Stop response** must drop torque to 0 within 100 ms of the
      `/safety/estop` topic going `true`. Today's gravity controller already
      reads `enabled_` atomically; the impedance controller needs the same.
- [ ] **First-activation warning** dialog (RViz panel or just a printed
      warning) listing the active stiffness on each axis. Big stiffness in
      a poorly-tuned dimension is the #1 way to oscillate the arm.

## Testing

- [ ] Unit tests with mock interfaces, modeled on `armv7_dyn_ident/test/`.
- [ ] Fake-hardware regression: load the controller in `use_fake_hardware:=true`
      bringup, assert non-NaN wrench output for a hand-crafted pose error.
- [ ] Hardware acceptance test sheet, modeled on
      [`docs/testing_phase4.md`](../../docs/testing_phase4.md) — both the
      "lock-5-DoF / leave-1-compliant" demo and the steady-state Cartesian
      error under payload.

## Documentation

- [ ] Author a `docs/testing_phase5.md` once the controller exists.
- [ ] Per-axis stiffness tuning table (sane defaults for "soft assembly" vs
      "rigid jig" use cases).
- [ ] Friction-vs-Cartesian-compliance tradeoff note — high task-space
      compliance and high Coulomb friction comp interact unintuitively.

---

Pick a box, open a PR, link it back to issue `[impedance starter]`. The
v0.1 interface in [`include/armv7_impedance_moveit/impedance_interface.hpp`](include/armv7_impedance_moveit/impedance_interface.hpp)
is the contract — extend it but try not to break it for downstream consumers.
