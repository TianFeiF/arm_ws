# armv7 — 7-DoF EtherCAT Manipulator (ROS 2 Humble)

ROS 2 stack for the **armv7** 7-axis arm with EYOU CiA-402 servo drives over EtherCAT (IgH master).
Built on `ros2_control` + MoveIt 2 + `ethercat_driver_ros2`.

## 中文摘要

7 自由度 EtherCAT 机械臂 ROS 2 软件栈,在 Humble + ros2_control + MoveIt 2 上提供 URDF、MoveIt 配置、硬件 bringup 与示例。当前为 **v0.1.0 开发中**,适合外部合作伙伴评估、二次开发。

---

## Status

Branch `main` is the active development line. Targeting first cut **v0.1.0** per [`plan.md`](plan.md).
Tested platform: Ubuntu 22.04 + ROS 2 Humble + Linux 6.8 + IgH EtherCAT master 1.6.9.

## Packages

| Package | Purpose |
|---|---|
| [`armv7_description`](src/armv7_description) | URDF / meshes / display + Gazebo launch |
| [`armv7_moveit_config`](src/armv7_moveit_config) | SRDF, planner configs, MoveIt 2 sub-launches |
| [`armv7_bringup`](src/armv7_bringup) | ros2_control xacros (fake & EtherCAT), EUPH slave configs, unified `arm.launch.py` |
| [`ethercat_driver_ros2/`](src/ethercat_driver_ros2) | Vendored ICube driver, patched for portable IgH path discovery (`pkg-config`) |

## Quickstart

### 1. System prerequisites (once per machine)
```bash
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-moveit-planners \
  ros-humble-moveit-configs-utils ros-humble-moveit-resources \
  ros-humble-moveit-visual-tools ros-humble-moveit-servo \
  ros-humble-moveit-setup-assistant ros-humble-moveit-task-constructor-core \
  ros-humble-srdfdom ros-humble-launch-param-builder \
  ethercat-master libethercat-dev pkg-config

# realtime group (lets chrt -f 99 succeed without sudo) — log out / log in after
sudo groupadd -f realtime
sudo usermod -aG realtime $USER
sudo tee /etc/security/limits.d/realtime.conf >/dev/null <<'EOF'
@realtime - rtprio 99
@realtime - memlock unlimited
EOF

# EtherCAT device perms
sudo usermod -aG ethercat $USER
sudo systemctl enable --now ethercat
```

### 2. Build
```bash
cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Run

```bash
# Mock hardware (no arm connected) — fastest way to see RViz + MoveIt
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false

# Real EtherCAT hardware (default)
ros2 launch armv7_bringup arm.launch.py
```

Either form opens RViz with the MoveIt Motion Planning panel. Drag the interactive marker, click **Plan & Execute**.

## Launch arguments

`ros2 launch armv7_bringup arm.launch.py --show-args`

| Arg | Default | Effect |
|---|---|---|
| `use_fake_hardware` | `false` | If `true`, use `mock_components/GenericSystem` instead of EtherCAT. No real hardware needed. |
| `use_rviz` | `true` | Start RViz with MoveIt motion-planning panel. |
| `db` | `false` | Start MoveIt warehouse database. |
| `use_rt` | `true` | Run `ros2_control_node` under `SCHED_FIFO 99`. Needs realtime-group setup above. |

## Documentation

- [`plan.md`](plan.md) — 4-week development roadmap to v0.1.0.
- [`PORTING_NOTES.md`](PORTING_NOTES.md) — Issues hit while porting to new hardware, with remedies. **Required reading** for first deployment.
- [`GITHUB_UPLOAD_GUIDE.md`](GITHUB_UPLOAD_GUIDE.md) — Step-by-step for uploading this repo to GitHub.
- [`SCRIPT_USAGE.md`](SCRIPT_USAGE.md) — Tutorial for the `init_and_push_to_github.sh` helper script.

## Repository layout

```
arm_ws/
├── src/
│   ├── armv7_description/        # URDF + meshes (visual only)
│   ├── armv7_moveit_config/      # MoveIt configs + sub-launches
│   ├── armv7_bringup/            # Hardware bringup + arm.launch.py
│   └── ethercat_driver_ros2/     # Patched ICube EtherCAT driver
├── LICENSE                       # Apache-2.0
├── README.md
├── plan.md
├── PORTING_NOTES.md
├── GITHUB_UPLOAD_GUIDE.md
├── SCRIPT_USAGE.md
└── init_and_push_to_github.sh
```

## Hardware

- **Arm:** Custom 7-DoF, exported via SolidWorks → URDF
- **Drives:** EYOU CiA-402 servo modules (`vendor_id 0x00001097`, `product_id 0x00002406`)
- **Bus:** EtherCAT (IgH master, Linux kernel driver `ec_master`)
- **NIC:** Any Intel NIC supported by IgH; bypass NetworkManager on that interface

## Known limitations (v0.1)

- Update rate clamped at 100 Hz — higher rates need PREEMPT_RT kernel
- No end-effector / gripper / sensor integration yet (Phase 3 in [`plan.md`](plan.md))
- No Python / C++ user API yet (Phase 2 in [`plan.md`](plan.md))
- Impedance / zero-force / dynamics identification packages **lost in porting** — re-implementation scheduled for Phase 4 (skeleton only) and v0.2 (full)

## License

Apache-2.0 — see [`LICENSE`](LICENSE).

## Contact

Maintainer: TianFeiF &lt;chunyvtian@gmail.com&gt;.
Issues and pull requests welcome once the repository is published.
