# armv7 v0.1.0 — Release Notes

**Release date:** 2026-06-17
**ROS distro:** Humble
**Tested on:** Ubuntu 22.04 + EUPH-series 7-DoF arm + IgH EtherCAT master

---

## What this release is

A complete-enough ROS 2 stack for the EUPH 7-DoF EtherCAT manipulator so an
external partner can:

1. `git clone` → run `scripts/install_deps.sh` → all dependencies installed.
2. `ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true` → MoveIt
   RViz comes up with the arm planning.
3. Hook up real hardware, drop `use_fake_hardware:=true` → the same launch
   drives the real EtherCAT slaves under SCHED_FIFO 99.
4. `ros2 run armv7_examples hello_world` → arm moves.
5. **Push the arm by hand** in free-drive (gravity + friction compensated)
   via `armv7_zero_force_controller`.

All of the above in under an hour from a clean Ubuntu image.

## What's new at a glance

| Feature | Package | Status |
|---|---|---|
| Unified bringup (sim ↔ real) | `armv7_bringup` | ✅ Stable |
| Software E-Stop + workspace bbox | `armv7_safety` | ✅ Stable |
| `/diagnostics` health aggregator | `armv7_diagnostics` | ✅ Stable |
| Python / C++ API (`Armv7Client`) | `armv7_py`, `armv7_cpp_api` | ✅ Stable |
| Pick-and-place + 2 more demos | `armv7_examples` | ✅ Stable |
| Modular EE (dummy gripper) | `armv7_ee_dummy_gripper` | ✅ Template |
| Hot-reload TCP + payload | `armv7_tcp` | ✅ Stable |
| Hand-eye camera template | `armv7_eyehand` | ✅ Template |
| F/T sensor integration | `docs/integration/ft_sensor.md` | ✅ Documented |
| Gravity-parameter identification | `armv7_dyn_ident` | ✅ Hardware-validated |
| Gravity + friction free-drive | `armv7_zero_force_controller` | ✅ Hardware-validated |
| Cartesian impedance | `armv7_impedance_moveit` | ⚠️ Interface only — v0.2 |

## Hardware-validated highlights

These two are the headline items because they were calibrated and tuned on the
real arm in this release cycle:

### Free-drive that actually feels light

`armv7_zero_force_controller` runs gravity compensation + Coulomb / viscous
friction compensation on the EtherCAT drives in CST (torque) mode.
Step-by-step:

```bash
# 1. one-shot data collection (~10 min, position mode, no torque commands)
ros2 launch armv7_dyn_ident collect.launch.py

# 2. offline identification — prints suggested coulomb_friction values
ros2 run armv7_dyn_ident identify --ros-args \
    -p csv:=/tmp/armv7_gravity.csv \
    -p out:=~/arm_ws/src/armv7_dyn_ident/config/identified_params.yaml

# 3. paste the suggested coulomb_friction line into
#    src/armv7_zero_force_controller/config/gravity_compensation.yaml,
#    then start free-drive:
ros2 launch armv7_zero_force_controller free_drive.launch.py

# 4. enable when clear of the arm:
ros2 service call /gravity_compensation_controller/enable \
    std_srvs/srv/SetBool "{data: true}"
```

On the EUPH reference arm the per-joint Coulomb friction estimates came out at
roughly 3.7, 3.7, 0.8, 0.7, 0.2, 0.3, 0.2 Nm. Pasting 60 % of those into
`coulomb_friction` gives a free-drive mode where horizontal (XY) pushes feel
about as light as vertical ones — the dominant resistance on a static arm at
zero velocity is friction, not gravity, and this release calibrates both.

### Identification with friction cancellation

`armv7_dyn_ident` now drives each pose **bidirectionally** (above + below) and
the pair-mean cancels Coulomb friction in the data fed to least squares. The
fitted gravity model on the EUPH arm has a per-joint pair-mean residual under
1 Nm everywhere — the symbolic regressor matches the C++ KDL gravity
implementation to machine precision (4 × 10⁻¹⁶).

## Documentation

- [README.md](README.md) — entry point.
- [docs/installation.md](docs/installation.md) — bare-metal install.
- [docs/quickstart.md](docs/quickstart.md) — first motion.
- [docs/testing.md](docs/testing.md) — A/B/C testing harness for the core stack.
- [docs/testing_phase3.md](docs/testing_phase3.md) — EE / sensor / TCP tests.
- [docs/testing_phase4.md](docs/testing_phase4.md) — identification + free-drive.
- [docs/troubleshooting.md](docs/troubleshooting.md) — 22 entries covering EtherCAT,
  realtime, DDS, MoveIt overlay, controller_manager spawn race, mock-component
  oddities, free-drive sag / drift / stiffness.
- [docs/integration/ft_sensor.md](docs/integration/ft_sensor.md) — F/T sensor wiring.
- [CHANGELOG.md](CHANGELOG.md) — full per-package change list for this release.
- [plan.md](plan.md) — 4-week MVP plan + v0.2 roadmap.

## Known limitations

- **Gravity-only dynamic model.** Inertia tensor identification is v0.2.
  Free-drive is fine; high-acceleration trajectory tracking would benefit
  from the missing terms.
- **No Cartesian impedance loop.** `armv7_impedance_moveit` is interface-only.
  Use `armv7_zero_force_controller` for any "compliance" you need today.
- **Real-arm torque mode is operator-critical.** Drives in CST hold zero
  torque whenever the controller is disabled — the arm falls. The procedure
  in [`docs/testing_phase4.md` §4.2.2](docs/testing_phase4.md) MUST be
  followed; brake or physically support the arm before disabling.
- **Friction is per-joint scalar.** Position-dependent cogging is not yet
  modeled. The current parameters work for slow / static interactions.
- **EE payload changes invalidate the gravity model.** Re-identify (or update
  the URDF chain) when mounting a tool of meaningful mass. The `~/payload`
  topic is published but not yet consumed by the gravity controller — v0.2.

## Roadmap

The shape of v0.2 (in priority order):

1. Cartesian impedance controller (fills the `armv7_impedance_moveit`
   skeleton).
2. Inertia tensor identification using the `armv7_dyn_ident` Fourier
   excitation generator already shipped.
3. Position-dependent friction lookup.
4. EE payload hot-merge into the gravity model.
5. Online safety: integrate `armv7_safety` E-Stop with the zero-force
   controller's torque-out path.
6. RViz teaching panel (waypoint capture / playback).

Full v0.2 backlog in [plan.md §7](plan.md) and
[`src/armv7_impedance_moveit/TODO.md`](src/armv7_impedance_moveit/TODO.md).

## Upgrade notes

This is the first release — there is no upgrade path. If you have been
running off `main` during the sprint:

- Pull `v0.1.0`, **`rm -rf build install log`**, rebuild with
  `colcon build --symlink-install`. Some packages were renamed
  (`armv7` → `armv7_description`, `armv7_moveit` → `armv7_moveit_config` /
  `armv7_bringup`); a clean build avoids stale Python module shadows.
- If you customised `gravity_compensation.yaml`, the file now has three new
  parameters (`coulomb_friction`, `viscous_friction`, `coulomb_velocity_eps`).
  Defaults are `0`, so the legacy gravity-only behavior is preserved — opt
  in to friction compensation by following the procedure above.

## Thanks

To the operator who spent an afternoon on the lab arm tuning friction values
and reporting back "手感已经很棒了" — that confirmation is the difference
between this release and a roadmap.
