# armv7_impedance_moveit (v0.1 starter)

**Status:** interface contract only — **no controller implementation in v0.1**.
This package exists so that downstream code can be written against a stable
6-DoF Cartesian impedance / admittance API while the actual control loop is
built in v0.2.

If you want a working free-drive controller TODAY, use
[`armv7_zero_force_controller`](../armv7_zero_force_controller/README.md) —
its gravity + friction compensation already gives a usable "manual guidance"
mode without needing impedance.

---

## 1. What "Cartesian impedance" means here

Two complementary modes on the same task-space spring-damper:

| Mode | Input | Output | Use case |
|---|---|---|---|
| **Impedance** | target pose | wrench (driving joint torque via Jᵀ) | "soft" position tracking — push the arm, it springs back |
| **Admittance** | measured external wrench | velocity (driving joint vel) | force-driven motion — hold and the arm doesn't move; push and it yields |

In both modes the operator chooses three things per task DOF:
1. **stiffness** k (N/m or N·m/rad)
2. **damping ratio** ζ (dimensionless; controller derives absolute damping)
3. **wrench limit** (safety clamp)

Stiffness = 0 in some axes makes that direction *infinitely compliant* — useful
for "trace this surface" or "constrain to plane" demonstrations.

## 2. Why this isn't in v0.1

The hard parts are:

| Block | Status in v0.1 | Why deferred |
|---|---|---|
| External-wrench estimate | not built | needs either an F/T sensor (per [docs/integration/ft_sensor.md](../../docs/integration/ft_sensor.md)) **OR** a calibrated joint-torque residual `τ_measured − G(q) − τ_friction`; the latter requires the friction model to be tight (this release just calibrated it on hardware — good enough for free-drive, not yet for sub-Nm wrench estimates) |
| Jacobian + null-space | not built | KDL `ChainJntToJacSolver` is the easy half; choosing a null-space damping policy that doesn't fight the operator is harder |
| Real-time RNEA | partial | `armv7_zero_force_controller` does gravity via KDL each cycle; full RNEA inverse dynamics for impedance also needs Coriolis terms, which need the identified inertia tensor (only mass + CoM are identified in v0.1) |
| Integration with MoveIt | not built | OMPL still plans in joint space; the impedance loop just tracks the planned waypoints with task-space compliance instead of strict joint-space PD |

v0.2 (see [plan.md §7](../../plan.md)) takes the last ~4 weeks of hardware
calibration and friction identification as inputs and builds the loop on top.

## 3. Interface contract (what the v0.2 controller will look like)

**ROS topics** (subscribed):
- `~/target_pose` — `geometry_msgs/PoseStamped` — impedance setpoint
- `~/target_wrench` — `geometry_msgs/WrenchStamped` — feed-forward wrench
- `~/external_wrench` — `geometry_msgs/WrenchStamped` — measured / estimated external load

**ROS topics** (published):
- `~/pose_error` — Cartesian tracking error in `reference_frame`
- `~/commanded_wrench` — wrench actually applied this cycle
- `~/commanded_torque` — joint torque written to the hardware (matches `armv7_zero_force_controller`'s debug topic for consistency)

**ROS services**:
- `~/enable` — `std_srvs/SetBool` — same semantics as `armv7_zero_force_controller`
- `~/set_impedance` — push new stiffness / damping ratio / wrench limit at runtime

**ROS parameters** (read at on_configure):
- `joints` — same list the zero-force controller takes
- `robot_description` — provided by controller_manager
- `root_link`, `tip_link` — task-space frames
- `stiffness`, `damping_ratio`, `wrench_limit` — 6 entries each, axes
  `[trans_x, trans_y, trans_z, rot_x, rot_y, rot_z]`
- All safety knobs from `armv7_zero_force_controller`
  (`max_torque`, `velocity_limit`, `ramp_in_time`, `enable_at_start`)

The C++ structs in [`impedance_interface.hpp`](include/armv7_impedance_moveit/impedance_interface.hpp)
match this list. Downstream code that already speaks to those structs will
compile against v0.2 without changes.

## 4. How to know v0.2 is ready

Acceptance gate (mirrors `armv7_zero_force_controller`'s):
- Operator can lock 5 of 6 DOFs at `k = 2000 N/m`, leave one DOF compliant
  (`k = 0`), and slide the TCP along that axis with < 5 N of effort.
- Servo can hold a `(target_pose, target_wrench)` setpoint with steady-state
  Cartesian error < 2 mm under a 5 kg payload disturbance.
- E-Stop service (`armv7_safety`) deactivates the controller within 100 ms
  without commanding a torque step.

## 5. Contributing

See [TODO.md](TODO.md) for the v0.2 implementation checklist. Pick one of the
top-level boxes and open a PR — every item has a hint as to which existing
package's pattern to mirror.

## 6. Range of applicability

The v0.2 controller is intended for low-speed teaching, surface following, and
constrained insertion. It is **not** a hard real-time servo for high-speed
trajectory tracking — for that, use `arm.launch.py` with MoveIt + position
mode. Cartesian impedance has fundamentally different stability margins from
joint-PD, and pushing it hard in the wrong direction makes the arm oscillate
faster than you can react. v0.2 will ship with conservative default gains and
a warning at first activation.
