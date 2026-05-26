# Troubleshooting

Issues encountered during porting / first-deployment, each with diagnosis and fix. Skim the symptom list, jump to the matching section.

| # | Symptom (first error you see) | Section |
|---|---|---|
| 1 | `chrt: 设置 pid 0 的策略失败: 不允许的操作` | [realtime-permission](#realtime-permission) |
| 2 | `Failed to open /dev/EtherCAT0: Permission denied` | [ethercat-device](#ethercat-device) |
| 3 | `cannot open shared object file: libethercat_interface.so` | [ld-library-path](#ld-library-path) |
| 4 | `AL status message 0x001B: "Sync manager watchdog"` | [watchdog](#watchdog) |
| 5 | `Slave did not respond to state change request!` (stuck in SAFEOP) | [dc-sync](#dc-sync) |
| 6 | `move_group ... Segmentation fault` in `libmoveit_ompl_interface.so.2.5.9` | [moveit-overlay](#moveit-overlay) |
| 7 | CMake: `include directory '/usr/local/etherlab/include' which doesn't exist` | [ethercat-cmake-path](#ethercat-cmake-path) |
| 8 | `package 'armv7' not found` after package rename | [package-rename](#package-rename) |
| 9 | `E: 无法定位软件包 ros-humble-rosparam-shortcuts` | [rosparam-shortcuts](#rosparam-shortcuts) |
| 10 | Plan succeeds in RViz but arm doesn't move on Execute | [execute-no-motion](#execute-no-motion) |
| 11 | `Didn't receive robot state (joint angles) ... 1.000000 seconds` | [no-joint-states](#no-joint-states) |
| 12 | `setup.bash` keeps pulling in `~/ws_moveit` after edit | [stale-setup-chain](#stale-setup-chain) |
| 13 | `Could not configure controller ... no controller with this name exists` (spawner) | [controller-spawn-race](#controller-spawn-race) |
| 14 | E-Stop trigger returns `success=True message='already stopped'` after fresh launch | [stale-estop-state](#stale-estop-state) |
| 15 | `/safety/bbox_state` topic empty but `workspace_bbox_node` is running | [bbox-tf-unavailable](#bbox-tf-unavailable) |
| 16 | `hello_world` / `teach_playback` exits with `follow_joint_trajectory action server not available` | [trajectory-action-missing](#trajectory-action-missing) |
| 17 | `RTPS_TRANSPORT_SHM Error: Failed init_port ... open_and_lock_file failed` flooding the log | [dds-shm-leak](#dds-shm-leak) |
| 18 | 真机:电机一进入 OP 就突然冲向 0 点,而不是保持当前位置 | [joints-jump-to-zero](#joints-jump-to-zero) |
| 18 | dummy gripper command accepted but finger never moves in sim | [gripper-mock-no-mirror](#gripper-mock-no-mirror) |

---

## realtime-permission

**Full symptom:**
```
chrt: 设置 pid 0 的策略失败: 不允许的操作
[ERROR] [taskset-5]: process has died ...
```

**Cause:** `chrt -f 99` requests `SCHED_FIFO` priority 99, which requires either `CAP_SYS_NICE` on the binary or the user being in a group with `rtprio 99` in `limits.conf`. By default Ubuntu has neither.

**Fix:**
```bash
sudo groupadd -f realtime
sudo usermod -aG realtime $USER
sudo tee /etc/security/limits.d/realtime.conf >/dev/null <<'EOF'
@realtime - rtprio 99
@realtime - memlock unlimited
EOF
```
**Log out of the desktop session and log back in** (not just open a new terminal). Verify:
```bash
id | grep -o realtime    # realtime
ulimit -r                # 99
chrt -f 99 echo ok       # prints "ok"
```

The bootstrap script `scripts/install_deps.sh` does all the above for you.

---

## ethercat-device

**Full symptom:**
```
[ros2_control_node-5] Failed to open /dev/EtherCAT0: Permission denied
[ros2_control_node-5] ... ecrt_master_slave_config ...
Segmentation fault
```
(The segfault is a downstream bug in `ethercat_driver` that doesn't handle open() failure — the root cause is permission.)

**Cause:** `/dev/EtherCAT0` is owned `root:ethercat` mode `0664`. Your user is not in the `ethercat` group.

**Diagnose:**
```bash
ls -l /dev/EtherCAT0      # crw-rw-r-- root ethercat ...
id | grep ethercat        # empty → confirmed
```

**Fix:**
```bash
sudo usermod -aG ethercat $USER
# Log out and back in
```
For a single-shell workaround without re-login: `newgrp ethercat`, but you'll lose your ROS environment in that new shell.

---

## ld-library-path

**Full symptom:**
```
Failed to load library /home/tian/arm_ws/install/ethercat_driver/lib/libethercat_driver.so.
... dlopen error: libethercat_interface.so: cannot open shared object file
```

**Cause:** You sourced `/opt/ros/humble/setup.bash` but not the workspace's `install/setup.bash`. Or you ran `newgrp` which started a fresh shell that lost the previously sourced env.

**Fix:**
```bash
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep ethercat_interface   # must appear
```

To make permanent, add the two `source` lines to `~/.bashrc`.

---

## watchdog

**Full symptom (in `dmesg -wT | grep EtherCAT`):**
```
EtherCAT ERROR 0-X: Failed to set OP state, slave refused state change (SAFEOP + ERROR).
EtherCAT ERROR 0-X: AL status message 0x001B: "Sync manager watchdog".
```
And `ethercat slaves` shows random slaves stuck in `SAFEOP E`.

**Cause:** `ros2_control_node` is missing PDO cycles → slave's SM2 watchdog times out. Two common reasons:
1. **Not running under SCHED_FIFO** — see [realtime-permission](#realtime-permission).
2. **DC sync register not configured** — see [dc-sync](#dc-sync).

Without RT, even at 100 Hz cycle the watchdog can trigger because Linux scheduler jitter exceeds the ~8 ms SM watchdog window.

**Verify RT is active:**
```bash
ps -eLo pid,tid,cls,rtprio,comm | grep ros2_control_node
# CLS must be FF (FIFO), rtprio should be 50 (controller_manager) or 99
```

If `CLS = TS` (SCHED_OTHER), the `chrt -f 99` prefix didn't take effect. Re-check `ulimit -r` (must be 99) and `launch` log for "Successful set up FIFO RT scheduling policy".

---

## dc-sync

**Full symptom:** `ethercat slaves` consistently leaves the **same** slaves in `SAFEOP E`, even with RT scheduling.

**Cause:** [EUPH11_config.yaml](../src/armv7_bringup/config/EUPH11_config.yaml), [EUPH14_config.yaml](../src/armv7_bringup/config/EUPH14_config.yaml), [EUPH17_config.yaml](../src/armv7_bringup/config/EUPH17_config.yaml) line 4 has `assign_activate: 0x0300` commented out. EYOU drives require DC sync mode active to enter OP — without it, some slaves fail the cycle check.

**Fix:** In all three EUPH yamls, uncomment line 4:
```yaml
# assign_activate: 0x0300   ← remove the leading #
```

All three files must agree (either all enabled or all disabled). Mixed config = sync mismatch = the same SAFEOP E failure mode.

---

## moveit-overlay

**Full symptom:**
```
[move_group-3] Stack trace ...
#0 ... "/home/tian/ws_moveit/install/moveit_planners_ompl/lib/libmoveit_ompl_interface.so.2.5.9",
       in typeinfo name for ompl_interface::JointModelStateSpaceFactory
Segmentation fault (Invalid permissions for mapped object)
```

**Cause:** `~/.bashrc` sources both `/opt/ros/humble/setup.bash` (system MoveIt 2.x) and `~/ws_moveit/install/setup.bash` (a from-source MoveIt 2.5.9 built earlier). The two libraries' ABIs disagree, and `dlopen` returns memory that fails type checks.

**Fix — migrate to apt MoveIt (one-time):**
```bash
sudo apt install -y ros-humble-moveit ros-humble-moveit-planners \
  ros-humble-moveit-configs-utils ros-humble-moveit-visual-tools \
  ros-humble-moveit-servo ros-humble-moveit-setup-assistant \
  ros-humble-moveit-task-constructor-core ros-humble-srdfdom \
  ros-humble-launch-param-builder
```

Then comment out the overlay line in `~/.bashrc`:
```bash
# source ~/ws_moveit/install/setup.bash
source ~/arm_ws/install/setup.bash
```

If the overlay was active when you last built `arm_ws`, its `install/setup.bash` chains it in via `_colcon_prefix_chain_bash_source_script`. See [stale-setup-chain](#stale-setup-chain) below.

---

## ethercat-cmake-path

**Full symptom (during `colcon build`):**
```
CMake Warning: ament_export_include_directories() package 'ethercat_interface' exports
  the include directory '/usr/local/etherlab/include' which doesn't exist
CMake Error: Imported target "ethercat_generic_slave::ethercat_generic_slave" includes
  non-existent path "/usr/local/etherlab/include"
```

**Cause:** The original `ethercat_driver_ros2` fork hard-codes IgH master at `/usr/local/etherlab/`. The apt-installed IgH lives in `/usr` instead.

**Fix:** Already patched in this repo — [src/ethercat_driver_ros2/ethercat_interface/CMakeLists.txt](../src/ethercat_driver_ros2/ethercat_interface/CMakeLists.txt) and [src/ethercat_driver_ros2/ethercat_manager/CMakeLists.txt](../src/ethercat_driver_ros2/ethercat_manager/CMakeLists.txt) now use `pkg-config`:
```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(ETHERCAT REQUIRED libethercat)
```

If you still see the warning after `git pull`, your old `install/ethercat_*` is leaking stale paths. Clean rebuild:
```bash
rm -rf build/ethercat_* install/ethercat_*
colcon build --packages-up-to ethercat_driver_ros2
```

---

## package-rename

**Full symptom:**
```
"package 'armv7' not found, searching: ['.../install/armv7_bringup', .../install/armv7_description', ...]"
```

**Cause:** You upgraded to v0.1.0 where `armv7` was renamed to `armv7_description` and `armv7_moveit` was split into `armv7_moveit_config` + `armv7_bringup`. Your old `install/` directory still has the old packages, OR some config file points at the old name.

**Fix:**
```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

If the error persists, grep for stale refs:
```bash
grep -rn "armv7\b" src/ --include="*.yaml" --include="*.xacro" --include="*.py" \
    | grep -v "armv7_description\|armv7_moveit_config\|armv7_bringup\|armv7_ethercat\|armv7_fake"
```
The likely offender is `src/armv7_moveit_config/.setup_assistant`.

---

## rosparam-shortcuts

**Full symptom:**
```
E: 无法定位软件包 ros-humble-rosparam-shortcuts
```

**Cause:** `rosparam_shortcuts` is not in the Humble apt repo.

**Fix:** **Skip it.** This project does not use `rosparam_shortcuts` (grep confirms no source references). The old `~/ws_moveit` overlay carried it as a transitive dep of `moveit2_tutorials`. The bootstrap script and [installation.md](installation.md) no longer list it.

If you do need it later (for an external project), build from source:
```bash
cd ~/arm_ws/src && git clone -b ros2 https://github.com/PickNikRobotics/rosparam_shortcuts.git
cd .. && colcon build --packages-select rosparam_shortcuts
```

---

## execute-no-motion

**Full symptom:** RViz Plan succeeds, ghost trajectory shows, but **Execute** does nothing (or aborts with no obvious error).

**Cause:** Most often, the `plan_group_controller` is not active. Check:
```bash
ros2 control list_controllers
# plan_group_controller   joint_trajectory_controller/...   active   ← required
```

If `inactive`, activate it:
```bash
ros2 control switch_controllers --activate plan_group_controller
```

Other causes:
- The planned trajectory goes through a joint-limit violation. Look at `move_group` log for `Trajectory execution aborted`.
- For real hardware: an EtherCAT slave is in fault state. `ethercat slaves` should show `OP +` for all; a `+` next to anything other than `OP` means error.

---

## no-joint-states

**Full symptom:**
```
[move_group-3] Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds.
```

**Cause:** `joint_state_broadcaster` is not running OR is not configured. This is usually a downstream symptom of `ros2_control_node` having died (see causes in [realtime-permission](#realtime-permission) and [ethercat-device](#ethercat-device)).

**Diagnose:**
```bash
ros2 control list_controllers          # joint_state_broadcaster should be active
ros2 topic hz /joint_states            # should print 10+ Hz
```

If `joint_state_broadcaster` is missing entirely, `ros2_control_node` never finished startup. Scroll up in the launch terminal — look for the FIRST error, not the joint-state warning.

---

## stale-setup-chain

**Full symptom:** You commented out `source ~/ws_moveit/install/setup.bash` in `~/.bashrc`, but `echo $AMENT_PREFIX_PATH` still shows ws_moveit paths after `exec bash`.

**Cause:** `colcon build` writes a chain of parent prefixes into `arm_ws/install/setup.bash` based on `AMENT_PREFIX_PATH` **at build time**. Even though you removed the overlay from `~/.bashrc`, the install script still re-loads it.

Look in [arm_ws/install/setup.bash](../install/setup.bash) for `COLCON_CURRENT_PREFIX="/home/.../ws_moveit/install"` — if it's there, your install is stale.

**Fix:**
```bash
# 1) Confirm ~/.bashrc no longer sources ws_moveit
grep ws_moveit ~/.bashrc

# 2) Wipe arm_ws install
cd ~/arm_ws
rm -rf install build log

# 3) Open a fresh shell — sourcing the missing arm_ws install fails harmlessly,
#    so the env is just /opt/ros/humble
exec bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'    # only /opt/ros/humble

# 4) Build in the clean env
colcon build --symlink-install
source install/setup.bash

# 5) Verify chain
grep COLCON_CURRENT_PREFIX install/setup.bash    # only /opt/ros/humble + self
```

---

## controller-spawn-race

**Full symptom (in launch log):**
```
[spawner-5] Controller already loaded, skipping load_controller
[ros2_control_node-4] Could not configure controller with name 'joint_state_broadcaster' because no controller with this name exists
[spawner-5] Failed to configure controller
```
`ros2 control list_controllers` shows the controller as `unconfigured` or missing entirely.

**Cause:** `ros-humble-controller-manager` ≥ 2.54.0 auto-discovers controllers from `controller_manager.ros__parameters.<name>.type` in the YAML, but the spawner can hit a window where `list_controllers` already shows the controller (so it skips `load_controller`) while `configure_controller` still rejects it. Two parallel spawner Nodes also race the same way.

The launch file [arm.launch.py](../src/armv7_bringup/launch/arm.launch.py) uses a single spawner Node that lists both controllers, which serialises load+configure inside one process and works on most runs. If you still hit the race occasionally:

**Workaround 1 — extend the spawner timeout** (already 30 s by default; raise to 60):
```bash
ros2 run controller_manager spawner joint_state_broadcaster plan_group_controller \
     --controller-manager /controller_manager \
     --controller-manager-timeout 60
```

**Workaround 2 — manual load/configure/activate** when the spawner fails:
```bash
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster configure
ros2 control set_controller_state joint_state_broadcaster activate
ros2 control load_controller plan_group_controller
ros2 control set_controller_state plan_group_controller configure
ros2 control set_controller_state plan_group_controller activate
```

**Workaround 3 — kill stale processes.** This is the most common cause when running on a dev machine where you've launched many times. Stale `ros2_control_node` / `move_group` / etc. instances on the same `ROS_DOMAIN_ID` reply to `list_controllers` with stale state:
```bash
pgrep -af "ros2_control_node\|move_group\|ros2 launch armv7\|workspace_bbox\|estop_node\|joint_diagnostics"
# kill -9 each PID
```
Then re-launch.

Upstream fix expected in controller_manager 2.55+; track at https://github.com/ros-controls/ros2_control.

---

## stale-estop-state

**Full symptom:** Fresh `ros2 launch armv7_bringup arm.launch.py`. First call to `/safety/estop_trigger` immediately returns:
```
success=True, message='already stopped'
```
without the trajectory controller being deactivated.

**Cause:** A previous `estop_node` instance from an earlier launch is still alive in the background. Both nodes claim the same service name and topic. The earlier (stale) one had `_stopped=True` from its previous trigger.

**Diagnose:**
```bash
pgrep -af estop_node           # should show exactly one PID per launch
```
If you see more than one, that's the bug.

**Fix:** Kill all stale ROS processes before re-launching:
```bash
for p in $(pgrep -f "estop_node\|workspace_bbox\|joint_diagnostics\|ros2_control_node\|move_group\|robot_state_publisher\|spawner\|rviz2"); do
    kill -9 "$p" 2>/dev/null
done
```
Then re-launch. Bash-launched `ros2 launch` jobs in the background do **not** propagate Ctrl-C to their child processes — always check `pgrep` after killing the job.

---

## bbox-tf-unavailable

**Full symptom:** `workspace_bbox_node` starts, prints its config, but `/safety/bbox_state` topic doesn't receive any messages.

**Cause:** The TF frame named by `tcp_frame_id` (default `link7`) doesn't exist yet because `robot_state_publisher` hasn't published the kinematic tree. This is normal for the first ~1 s after launch.

**Diagnose:**
```bash
ros2 run tf2_ros tf2_echo base_link link7 --once     # should print a transform
```
If `tf2_echo` hangs:
- Check `robot_state_publisher` is running: `ros2 node list | grep robot_state_publisher`
- Check `/joint_states` is publishing: `ros2 topic hz /joint_states` — needs `joint_state_broadcaster` active.

**Fix:** Wait until the kinematic chain is fully published. The `workspace_bbox_node` keeps retrying every 20 ms and will publish as soon as TF works. If TF never appears, fix [joint_state_broadcaster](#controller-spawn-race) first.

You can also override `tcp_frame_id` to a different frame at launch:
```bash
ros2 launch armv7_safety safety.launch.py bbox_config:=my_overrides.yaml
```
where your override yaml sets `tcp_frame_id: link6` (or similar) for testing.

---

## trajectory-action-missing

**Full symptom:** any `armv7_py` / `armv7_cpp_api` consumer (including `armv7_examples`) raises:
```
RuntimeError: /plan_group_controller/follow_joint_trajectory action server not available
              — is plan_group_controller active?
```

**Cause:** `plan_group_controller` is not in the `active` state. The trajectory controller serves the `follow_joint_trajectory` action; without it, no joint trajectory can be executed.

**Diagnose:**
```bash
ros2 control list_controllers
# plan_group_controller   joint_trajectory_controller/...   active   ← required
ros2 action list | grep follow_joint_trajectory
# should print /plan_group_controller/follow_joint_trajectory
```

**Fix:**
1. If `plan_group_controller` shows `inactive`:
   ```bash
   ros2 control switch_controllers --activate plan_group_controller
   ```
2. If `plan_group_controller` is missing entirely → spawner failed, see [controller-spawn-race](#controller-spawn-race).
3. If you're running multiple `arm.launch.py` instances (look at `pgrep -af ros2_control_node`),
   stale ones may have grabbed the action name. Kill them all and re-launch.

---

## dds-shm-leak

**Full symptom:** every spawn / service call times out with no obvious cause, and the log contains many lines like:
```
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7467: open_and_lock_file failed
```
`ros2_control_node` may even log `Loading controller 'plan_group_controller'` successfully, while the `spawner` declares `FATAL Failed loading controller plan_group_controller` and dies — the service responded but its reply was lost in transport.

**Cause:** FastDDS (the default rmw for Humble) writes shared-memory lock files under `/dev/shm/fastrtps_*`. A process killed with `SIGKILL` (`kill -9`) does NOT get to clean those up. A subsequent process trying to grab the same port name fails to lock it.

**Diagnose:**
```bash
ls /dev/shm/ | grep fastrtps          # if there are entries when no ROS nodes are running → leaked
pgrep -af "ros2_control_node|move_group|robot_state_publisher|workspace_bbox|estop_node"
# should be empty when nothing should be running
```

**Fix:** Multi-pass kill (some launchers re-fork their children if killed singly) plus shm cleanup:
```bash
for i in 1 2 3 4 5; do
    for p in $(pgrep -f "ros2 launch armv7|ros2_control_node|move_group|workspace_bbox|estop_node|joint_diagnostics|robot_state_publisher|static_transform_publisher|spawner|rviz2"); do
        kill -KILL "$p" 2>/dev/null
    done
    sleep 1
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ros2 daemon stop && sleep 1 && ros2 daemon start
```
Then relaunch normally.

**Prevention:** when developing,
- stop launches with Ctrl-C in the launch terminal (lets SIGINT propagate)
- avoid running `&` in background then `kill $!` — the launcher dies but its children keep the shm
- prefer `setsid ros2 launch ... &` plus `kill -- -$PGID` to kill the whole process group

---

## gripper-mock-no-mirror

**Full symptom:** with `ee:=dummy_gripper`, `ee_gripper_controller` is `active`, the
command interface shows `[claimed]`, publishing to `/ee_gripper_controller/commands`
returns no error — but `ee_finger_left_joint` in `/joint_states` never changes from
its initial value (0.035).

**Cause:** `mock_components/GenericSystem` does NOT mirror a position command to the
position state interface when that command interface carries `<param name="min">` /
`<param name="max">` limit params. It treats a limited interface differently and
skips the mirror. (Real gripper drivers are unaffected — they implement their own
read/write.)

**Fix:** drop the min/max params from the command interface in the EE's
`*.ros2_control.xacro`:
```xml
<!-- WRONG (mock won't mirror) -->
<command_interface name="position">
  <param name="min">0.005</param>
  <param name="max">0.035</param>
</command_interface>

<!-- RIGHT -->
<command_interface name="position" />
```
Enforce the stroke limits in the URDF `<joint><limit ...>` instead — that's where
they belong, and MoveIt / collision checking reads them there.

This is already fixed in
[src/armv7_ee_dummy_gripper/urdf/dummy_gripper.ros2_control.xacro](../src/armv7_ee_dummy_gripper/urdf/dummy_gripper.ros2_control.xacro).

---

## free-drive-arm-falls

**Full symptom:** in `free_drive.launch.py` (CST / torque mode) the arm sags or drops —
either right after the drives enable, or the moment you toggle the controller off.

**Cause:** this is expected. In torque mode the drives provide **zero** holding torque
unless `gravity_compensation_controller` is *active AND enabled*. The controller starts
**disabled** on purpose (`enable_at_start: false`), and disabling it commands 0 torque.

**Fix / procedure:** physically support the arm (or use a brake) before launching, and
before disabling. Enable only once you are clear of the arm:
```bash
ros2 service call /gravity_compensation_controller/enable std_srvs/srv/SetBool "{data: true}"
```
The `ramp_in_time` (default 2 s) eases torque in so it does not jump. See
[docs/testing_phase4.md § 4.2.2](testing_phase4.md) for the full safety sequence.

---

## free-drive-drift-direction

**Full symptom:** with gravity compensation enabled the arm slowly drifts in one
direction (sinks, or floats upward) instead of staying put.

**Cause / fix:**
- Sinks (gravity under-compensated): the model mass/CoM is too light. Run
  `armv7_dyn_ident` and pass `identified_params:=...`, or nudge `gravity_scale` up
  toward 1.0.
- Floats up (over-compensated): lower `gravity_scale` (start at 0.8 — slightly heavy is
  always safer than floating).
- One joint pushes the *wrong* way: the drive's torque sign is opposite the URDF axis on
  that joint. Re-run `identify` with `joint_sign` flipped for it (e.g.
  `-p joint_sign:="[1,1,-1,1,1,1,1]"`); the controller commands in the same effort units
  it was identified in, so the fix carries through.
- Residual stiction drift within ~10°/s is normal for v0.1 (Coulomb friction is not
  compensated). Add a little `damping` to make it feel smoother.

---

## free-drive-no-robot-description

**Full symptom:** `gravity_compensation_controller` fails to configure with
`'robot_description' is empty; the controller_manager must provide it`.

**Cause:** the controller builds its KDL model from the `robot_description` the
controller_manager forwards. If the `ros2_control_node` was started without a
`robot_description` parameter (e.g. a hand-rolled launch), the controller has nothing to
parse.

**Fix:** start free-drive via `armv7_zero_force_controller free_drive.launch.py`, which
passes `robot_description` to the `ros2_control_node`. If you wrote your own launch, make
sure the `ros2_control_node` `parameters=[...]` includes the rendered
`{'robot_description': <urdf>}` dict, exactly as `arm.launch.py` does.

---

## joints-jump-to-zero

**Full symptom (REAL hardware only):** the instant the EtherCAT slaves reach OP
state during `arm.launch.py` startup, the joints snap to the 0 position at full
speed instead of holding where they were. Dangerous — the arm can swing hard.

**Cause:** CSP mode (`mode_of_operation: 8`) makes the drive follow target
position `0x607a` every cycle. Before any controller command arrives, the
target-position command interface is `NaN`, so `ethercat_generic_cia402_drive`
is supposed to write the *current actual position* as the default. The upstream
code only did that once the drive reported a non-zero `0x6061` (mode-of-operation
display). EYOU servos report `0x6061 = 0` until they reach OPERATION_ENABLED, so
in that window nothing was written and `0x607a` stayed at its power-on value of
**0** → the joint drove to zero.

**Fix (already applied in this repo)** — two coordinated changes in
[generic_ec_cia402_drive.cpp](../src/ethercat_driver_ros2/ethercat_generic_plugins/ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp):

1. **Seed earlier.** The target-position default is taken from the current
   feedback as soon as a valid position has been read (gated on
   `!std::isnan(last_position_)` instead of on the mode display).

2. **Hold disabled until seeded.** The CiA402 auto state machine is NOT allowed
   to advance out of SWITCH_ON_DISABLED until `last_position_` is valid. This
   closes the one-cycle window where the drive could reach OPERATION_ENABLED on
   the same cycle that `0x607a` was still 0 — the cause of the residual
   "twitch toward zero" that remained after fix #1 alone.

Together: the drive stays disabled (and therefore ignores `0x607a`) for the one
cycle where the seed lags, then enables only after the target has tracked the
actual pose for several cycles.

Rebuild after pulling:
```bash
colcon build --packages-select ethercat_generic_cia402_drive
```

**Verify on hardware (keep your hand on the hardware E-Stop):**
1. Power-cycle the drives (cold start) so they begin in SWITCH_ON_DISABLED.
2. `ros2 launch armv7_bringup arm.launch.py`
3. Watch the arm at OP entry — it must stay put, not move to zero.
4. `ros2 topic echo /joint_states --once` right after startup — positions should
   match the physical pose, not all-zeros.

**Residual caveat — warm restart:** if you Ctrl-C the launch and immediately
relaunch WITHOUT power-cycling, the drives may still be in OPERATION_ENABLED.
There can be a single ~10 ms cycle where `0x607a` is still 0 before feedback
propagates. For safety, **power-cycle (cold start) the drives between runs**, or
ensure a hardware E-Stop is within reach. A full clean-shutdown path that
disables the CiA402 state machine on `on_deactivate` is a v0.2 item.

---

## Still stuck?

1. Search `dmesg` for the **first** EtherCAT error after `systemctl restart ethercat`. AL status codes are documented in the IgH manual; common ones are in [PORTING_NOTES.md § 3](../PORTING_NOTES.md#3-EtherCAT-运行前环境准备).
2. Run the bootstrap script in check mode: `bash scripts/install_deps.sh --check` — it prints which prerequisites are missing.
3. File an issue. Include: Ubuntu version, kernel (`uname -a`), output of `ros2 doctor`, full `ros2 launch` log up to the first error.
