# Installation

Get the armv7 stack running on a fresh machine. Total time **≈ 45 min** (most is `apt install`).

> Target platform: **Ubuntu 22.04 + ROS 2 Humble + Linux 6.8+**.
> Tested with IgH EtherCAT master 1.6.9 and a 22-core desktop.

---

## 1. Prerequisites

### 1.1 Operating system
Ubuntu 22.04 LTS Desktop or Server. Other 22.04 derivatives may work but are untested. Do NOT use 24.04 — Humble is not packaged there.

### 1.2 ROS 2 Humble
Follow https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html. Install `ros-humble-desktop`, not the minimal flavour.

Verify:
```bash
source /opt/ros/humble/setup.bash
ros2 --version          # should print "ros2 cli version: humble"
```

### 1.3 EtherCAT master kernel module
The IgH EtherCAT master is delivered as a kernel module + user-space tool. Two install paths:

| Path | When |
|---|---|
| `sudo apt install ethercat-master ethercat-dkms` | **Recommended.** Debian package; works on stock kernels. |
| Build from `etherlab.org` source | Only if you need a custom feature (e.g. RTDM patch). |

After install, configure `/etc/ethercat.conf`:
```ini
MASTER0_DEVICE="aa:bb:cc:dd:ee:ff"   # MAC of the NIC dedicated to EtherCAT
DEVICE_MODULES="generic"
```

The MAC is from `ip link show <iface>`. **The NIC must NOT be managed by NetworkManager** — make it `unmanaged=true` in your NM config, or use a dedicated USB-NIC.

### 1.4 Hardware
- 7-DoF armv7 mechanical assembly
- 7× EYOU CiA-402 servo modules
- One Ethernet NIC for EtherCAT (plus your normal NIC for SSH/internet)
- Optional: a kernel with PREEMPT_RT for cycle times faster than 10 ms

---

## 2. Clone & bootstrap

```bash
mkdir -p ~/arm_ws && cd ~/arm_ws
git clone <repo-url> .

# One-shot system setup (apt, realtime group, ethercat group, udev rules)
bash scripts/install_deps.sh
```

The script prompts for sudo and prints a summary. It is **idempotent** — running it twice is safe.

> **Important:** if the script reports that you were just added to `realtime` or `ethercat` groups, **log out of your desktop session and log back in** before proceeding. Group membership only takes effect for new login sessions. Verify:
> ```bash
> id | grep -oE 'realtime|ethercat'   # both should appear
> ulimit -r                           # should print 99
> ```

---

## 3. Build

```bash
cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit config/yaml files without re-building.

Expected build time on a modern desktop: **3–5 min** for the first build, **< 30 s** for incremental rebuilds.

---

## 4. Verify

### 4.1 Mock-hardware smoke test (no arm required)
```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
```

You should see RViz pop up with the armv7 model. Drag the interactive marker, click **Plan & Execute** in the MoveIt panel — the model arm should move.

### 4.2 EtherCAT readiness (arm connected)
```bash
# IgH master service
systemctl is-active ethercat                       # active
ls -l /dev/EtherCAT0                               # crw-rw-r-- root ethercat

# All 7 slaves visible
ethercat slaves                                    # 7 lines, all PREOP

# Realtime privileges available
ulimit -r                                          # 99
chrt -f 99 echo "rt ok"                            # prints "rt ok"
```

Each of those lines must pass before you launch on real hardware.

### 4.3 Full bringup on real hardware
```bash
ros2 launch armv7_bringup arm.launch.py
```

Look for these in the log:
- `EthercatDriver: System Successfully started!`
- `controller_manager: Successful set up FIFO RT scheduling policy with priority 50.`
- `joint_state_broadcaster: Configured and activated`
- `plan_group_controller: Configured and activated`

Then plan & execute in RViz. The arm should move.

---

## 5. Add `~/.bashrc` shortcuts (optional)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/arm_ws/install/setup.bash" >> ~/.bashrc
```

⚠️ **Do not also source legacy overlays** (e.g. `~/ws_moveit/install/setup.bash`). They cause ABI mismatches with system MoveIt 2 — see [troubleshooting.md](troubleshooting.md#move_group-segfault).

---

## 6. What if something failed?

| Symptom | Go to |
|---|---|
| `chrt: ... 不允许的操作` | [troubleshooting.md § realtime-permission](troubleshooting.md#realtime-permission) |
| `Failed to open /dev/EtherCAT0` | [troubleshooting.md § ethercat-device](troubleshooting.md#ethercat-device) |
| `AL status message 0x001B` | [troubleshooting.md § watchdog](troubleshooting.md#watchdog) |
| `move_group` segfault | [troubleshooting.md § moveit-overlay](troubleshooting.md#moveit-overlay) |
| CMake error `/usr/local/etherlab/include` | [PORTING_NOTES.md § 2](../PORTING_NOTES.md#2-ethercat-库位置不可硬编码) |
| `cannot open libethercat_interface.so` | [troubleshooting.md § ld-library-path](troubleshooting.md#ld-library-path) |

---

Continue with [quickstart.md](quickstart.md) to drive the arm.
