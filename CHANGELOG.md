# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.1.0] — 2026-06-17

First public release, after a 4-week MVP sprint covering bringup, safety,
end-effector / sensor templates, and a hardware-validated gravity-compensation
free-drive mode. Buildable from a clean Ubuntu 22.04 + ROS 2 Humble install
via `scripts/install_deps.sh`.

### Added — Phase 1 (W1, bringup & docs)

- `armv7_description` — moved from the legacy `armv7` package, URDF + meshes,
  cleaned `package://` references, `armv7.urdf.xacro` top-level with optional
  args `ros2_control_xacro`, `ee_xacro`, `ee_ros2_control_xacro`, `ee_parent`,
  `initial_positions_file`.
- `armv7_moveit_config` — split out of the legacy `armv7_moveit`. Fully
  populated `joint_limits.yaml` (position / velocity / acceleration / effort
  per joint). Sub-launches share `_moveit_config_loader.py`.
- `armv7_bringup` — single entrypoint `arm.launch.py` with
  `use_fake_hardware`, `use_rt`, `use_rviz`, `use_safety`, `use_diagnostics`,
  `use_tcp`, `ee`. EtherCAT (`armv7_ethercat.ros2_control.xacro`) and mock
  (`armv7_fake.ros2_control.xacro`) variants. Controller spawn race fixed via
  `OnProcessExit` chaining (joint_state_broadcaster → plan_group_controller →
  ee_gripper_controller).
- `scripts/install_deps.sh`, `docker/Dockerfile.dev` + `docker-compose.yml`,
  `.github/workflows/ci.yml` (colcon build + lint), root `LICENSE`
  (Apache-2.0).
- `docs/installation.md`, `docs/quickstart.md`, `docs/troubleshooting.md`
  (22 entries), `PORTING_NOTES.md`, `GITHUB_UPLOAD_GUIDE.md`, `SCRIPT_USAGE.md`.

### Added — Phase 2 (W2, safety & API)

- `armv7_safety` — `workspace_bbox_node` publishes `/safety/in_bounds` and
  `/safety/bbox_state` (latched + every tick). `estop_node` provides
  `/safety/estop` (latched), `/safety/estop_trigger`, `/safety/estop_clear`;
  toggles `plan_group_controller` via `controller_manager`'s `switch_controller`.
- `armv7_diagnostics` — `joint_diagnostics_node` subscribes `/joint_states`,
  publishes a `diagnostic_msgs/DiagnosticArray` graded against `joint_limits.yaml`.
- `armv7_py` — `Armv7Client` Python facade: `move_to_joint`, `move_through_joints`,
  `jog`, `get_joint_state`, `get_tcp_pose`, `stop`. Built on
  `FollowJointTrajectory`.
- `armv7_cpp_api` — same 5 methods in C++, header `client.hpp` + source.
- `armv7_examples` — `hello_world.py`, `teach_playback.py`, `pose_grid.py`.
- `docs/testing.md` (3-tier A/B/C testing harness).
- Unit tests across the new packages, `colcon test` clean (~176 tests at
  Phase 2 close).

### Added — Phase 3 (W3, end-effector & sensors)

- `armv7_ee_dummy_gripper` — two-finger gripper xacro + `mock_components`
  `ros2_control` template + `gripper_controller.yaml`. Macro names
  (`armv7_ee`, `armv7_ee_ros2_control`) are the integration contract any
  custom EE plugs into.
- `armv7_tcp` — `tcp_publisher_node` publishes `link7→tcp` TF every cycle
  and `/armv7/payload` latched JSON. `tcp_offset_xyz/rpy`,
  `payload_mass/com/inertia` are hot-reloadable via `ros2 param set`.
- `armv7_eyehand` — RealSense D435 mount xacro + `easy_handeye2`-output
  static TF launcher. Customers paste their calibration into
  `handeye_calibration.yaml`.
- `armv7_examples/pick_and_place.py` — full 11-step demo using the dummy gripper.
- `docs/integration/ft_sensor.md` — EtherCAT (ATI Axia80-EC) and network
  (`WrenchStamped` topic) F/T-sensor integration paths.
- `docs/testing_phase3.md` — Phase 3 acceptance harness.

### Added — Phase 4 (W4, dynamics & free-drive)

- `armv7_dyn_ident` — gravity-parameter identification:
  - `gravity_model.py` — symbolic regressor `G(q) = Y(q)·φ` built from the
    URDF via SymPy + `cse` lambdify. Validated to machine epsilon against the
    energy-gradient finite-difference and against the C++ KDL controller.
  - `excitation.py` — `static_poses()` random pose grid + `fourier_trajectory()`
    generator (reserved for v0.2 inertia-tensor identification).
  - `collect_node.py` — drives the arm through static poses via
    `FollowJointTrajectory` in the existing position mode, samples averaged
    `(q, τ)` per pose. **Bidirectional sampling** (default) approaches each
    pose from both directions to cancel Coulomb friction in the pair-mean.
  - `identify.py` — regularised LS toward URDF priors; detects bidirectional
    pairs and additionally reports the friction-cancelled gravity residual
    and a per-joint Coulomb friction estimate (median half-diff), with a
    suggested `coulomb_friction` line to paste into the controller.
  - Output `identified_params.yaml` (per-link mass + CoM + meta).
- `armv7_zero_force_controller` — `controller_interface::ControllerInterface`
  C++ plugin commanding `effort`. Per cycle:
  ```
  τ = gravity_scale · G(q)
    + coulomb_friction · tanh(q̇ / eps)
    + viscous_friction · q̇
    − damping · q̇
  ```
  with `max_torque` clamp, `ramp_in_time` activation ramp, `velocity_limit`
  cutoff, atomic `~/enable` service, and a debug topic
  `~/gravity_torque`. KDL chain built from `robot_description` (provided by
  controller_manager); per-link mass + CoM optionally overridden from
  `identified_params_file`.
- `armv7_bringup/urdf/armv7_cst.ros2_control.xacro` + three
  `EUPH{17,14,11}_cst_config.yaml` — torque-mode (CiA-402 CST, mode 10) PDO
  configs with `effort` command interface and `1/state_factor` torque
  scaling per drive size.
- `armv7_zero_force_controller/launch/free_drive.launch.py` — entrypoint for
  free-drive (real CST or `use_fake_hardware:=true`). Writes a temporary
  override yaml when `identified_params:=` is given so the override reliably
  reaches the controller.
- `armv7_impedance_moveit` — **starter package only**, interface contract +
  `README` + `TODO.md`. No control loop in v0.1; mounts the API v0.2 will
  fill in.
- `docs/testing_phase4.md` — Phase 4 acceptance harness. Real-hardware
  procedure for CST free-drive, with safety sequence and `gravity_scale`
  / `coulomb_friction` tuning recipes.

### Changed

- All inertia plus mesh assets renamed `armv7` → `armv7_description`.
- MoveIt `.setup_assistant` fixed: `package: armv7_description`,
  `relative_path: urdf/armv7.urdf.xacro`.
- Realtime path: kept `chrt -f 99` available via `use_rt:=true` argument,
  but no longer required for fake-hardware bringup.
- README rewritten in Chinese, English version pending.

### Fixed

- IgH EtherCAT master location is now resolved via `pkg-config`
  (`libethercat`), no more hardcoded `/usr/local/etherlab` in the inline
  `ethercat_driver_ros2` copy.
- CiA-402 watchdog `AL status 0x001B` fix documented: DC sync
  (`assign_activate: 0x0300`) must be active and the control loop must run
  under realtime scheduling. Slave configs ship with it enabled.
- `controller_manager` ≥ 2.54 spawn race resolved by chaining spawners via
  `OnProcessExit`.
- `mock_components/GenericSystem` not mirroring command → state when the
  command interface carries `<param name="min/max">`. Fix is now the
  documented EE template pattern: enforce limits in `<joint><limit>` instead.
- FastDDS `/dev/shm/fastrtps_*` leak after `kill -9` documented; cleanup is
  part of the standard test harness preamble.
- `armv7_zero_force_controller/launch/free_drive.launch.py`: passing
  `identified_params` as a dotted-key dict did not propagate through CM;
  now writes a temp override yaml.
- `armv7_dyn_ident/identify.py`: output path now expands `~` and creates
  the parent directory if needed.

### Known limitations

- **Gravity-only dynamics.** v0.1 identifies first moments (mass + CoM)
  only. Full inertia tensor identification is v0.2.
- **No online friction adaptation.** Coulomb friction is a static per-joint
  scalar. Real harmonic drives have position-dependent cogging — v0.2 will
  add a friction map.
- **No Cartesian impedance loop yet.** `armv7_impedance_moveit` is interface
  only; use `armv7_zero_force_controller` for free-drive in this release.
- **EE / payload effect on gravity.** Attaching a gripper requires
  re-identifying (or extending the URDF chain) for accurate compensation.
  The `~/payload` topic is published but not yet consumed by the
  zero-force controller; v0.2.
- **Real-hardware CST procedure is operator-critical.** Detailed safety
  sequence in `docs/testing_phase4.md` §4.2.2 — read before the first run.

### Stats

- 14 packages (was 9 at start of sprint).
- 22 troubleshooting entries.
- 4 phase-specific testing harnesses.

[0.1.0]: https://github.com/TianFeiF/arm_ws/releases/tag/v0.1.0
