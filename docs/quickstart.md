# Quickstart

Assumes [installation.md](installation.md) is done and you have a fresh shell with the workspace sourced.

---

## 1. Sim-only (no arm connected)

The fastest way to see something move.

```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
```

This brings up:
1. `robot_state_publisher` — publishes TF.
2. `move_group` — MoveIt planning server.
3. RViz with the **MoveIt Motion Planning** panel.
4. `ros2_control_node` with `mock_components/GenericSystem` — pretends to be 7 perfect joints.
5. `joint_state_broadcaster` + `plan_group_controller`.

### Use it

In RViz:
1. Set the **Fixed Frame** to `base_link` (top-left).
2. In the MoveIt panel, **Planning** tab → choose a goal:
   - Drag the orange interactive marker, OR
   - Click "Select Goal State" → pick a named pose from the dropdown.
3. Click **Plan** → ghost trajectory appears.
4. Click **Execute** → mock arm moves over ~2 s.

### Common arguments

`ros2 launch armv7_bringup arm.launch.py --show-args` for the full list. Most useful:

| Arg | Default | When to change |
|---|---|---|
| `use_fake_hardware` | `false` | `true` for sim |
| `use_rviz` | `true` | `false` for headless servers / CI |
| `use_rt` | `true` | `false` if no realtime group set up yet |
| `db` | `false` | `true` to enable MoveIt warehouse DB (for saving poses) |
| `use_safety` | `true` | `false` to skip `armv7_safety` (workspace bbox + E-Stop) |
| `use_diagnostics` | `true` | `false` to skip `armv7_diagnostics` aggregator |

---

## 2. Real hardware

### 2.1 Pre-launch checklist (mandatory)

```bash
systemctl is-active ethercat          # active
ethercat slaves                        # all 7 visible
ls -l /dev/EtherCAT0                   # crw-rw-r-- root ethercat
id | grep -oE 'realtime|ethercat'      # both appear
ulimit -r                              # 99
echo $LD_LIBRARY_PATH | grep ethercat  # has install/ethercat_interface/lib
which move_group                       # /opt/ros/humble/lib/...
```

If any line fails, fix it before launching. Most issues are covered in [troubleshooting.md](troubleshooting.md).

### 2.2 Launch

```bash
ros2 launch armv7_bringup arm.launch.py
```

Wait for these log lines (in this order):
```
EthercatDriver: Activated EcMaster!
EthercatDriver: System Successfully started!
controller_manager: update rate is 200 Hz
controller_manager: Successful set up FIFO RT scheduling policy with priority 50.
Configured and activated joint_state_broadcaster
Configured and activated plan_group_controller
```

Total cold-start time: ~10 s.

### 2.3 Verify in another shell

Source the workspace in a second terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash
```

Then check:
```bash
# Hardware component
ros2 control list_hardware_components
# armv7_ethercat                         system  configured  active

# Controllers
ros2 control list_controllers
# joint_state_broadcaster                joint_state_broadcaster/...  active
# plan_group_controller                  joint_trajectory_controller/...  active

# Current joint angles
ros2 topic echo /joint_states --once

# Watch EtherCAT slave states live
watch -n 1 'ethercat slaves'    # all 7 should stay OP +

# Safety + diagnostics layer (Phase 2)
ros2 topic echo /safety/estop --once          # latched, false = ok
ros2 topic echo /safety/in_bounds --once      # true = TCP inside bbox
ros2 topic echo /safety/bbox_state --once     # 'ok' | 'warning' | 'out_of_bounds'
ros2 topic echo /diagnostics --once           # per-joint health

# Visualise diagnostics
ros2 run rqt_runtime_monitor rqt_runtime_monitor   # GUI tree of joint states
```

### 2.4 Drive it from RViz

Same workflow as sim (see § 1). If you hit `Execute` and nothing happens, see [troubleshooting.md § execute-no-motion](troubleshooting.md#execute-no-motion).

---

## 3. Stop / restart

### Clean stop
Ctrl-C in the launch terminal. Wait ~5 s for graceful shutdown. The arm holds its current pose (servos remain energised).

### Emergency stop
**Hardware E-Stop button is still the only safety-certified stop.** A software E-Stop is now available for non-safety-critical cases:
```bash
# Stop motion immediately (deactivates plan_group_controller)
ros2 service call /safety/estop_trigger std_srvs/srv/Trigger

# Resume taking trajectories
ros2 service call /safety/estop_clear std_srvs/srv/Trigger

# Watch the state latched (will fire once on subscribe)
ros2 topic echo /safety/estop
```
The arm holds position via the drive's CSP mode after E-Stop. NOT functionally safe (no certification), so always use a hardware E-Stop button on top of this for any deployment near humans.

### After a fault
If an EtherCAT slave faults (e.g. over-current):
```bash
# In armv7_bringup, the EthercatDriver has a `reset_fault` command interface.
# Re-launch arm.launch.py — it resets faults on activate.
# Or, manually clear via SDO:
ros2 service call ... (not yet exposed; on the Phase 2 list)
```

---

## 4. Save / replay trajectories

Phase 2 deliverable — see `armv7_examples/teach_playback.py` in the roadmap. For now, use the MoveIt warehouse DB:
```bash
ros2 launch armv7_bringup arm.launch.py db:=true
```
In RViz → **Stored Scenes** panel.

---

## 5. Where next

- Working code? → [plan.md](../plan.md) for what's coming in v0.1 / v0.2.
- Need a custom EE / sensor? → Phase 3 of [plan.md](../plan.md), interfaces TBD.
- Broke something? → [troubleshooting.md](troubleshooting.md).
