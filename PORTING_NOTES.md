# 移植注意事项

记录本次将 `armv7_moveit` + `ethercat_driver_ros2` 工程从原机器移植到当前机器(Ubuntu 22.04 / ROS 2 Humble)时遇到的问题与修复方法。后续在新机器上部署时按本文档检查即可。

---

## 1. 实时调度 — 不可省略

### 现象
两种相关报错都见过:

**A. 完全没有 RT 时:**
```
chrt: 设置 pid 0 的策略失败: 不允许的操作
[ERROR] [taskset-5]: process has died ... ros2_control_node ...
```
随后 `joint_state_broadcaster`/`plan_group_controller` 永远 waiting。

**B. 没有 RT 但启用了 watchdog 时:**
```
EtherCAT ERROR 0-X: Failed to set OP state, slave refused state change (SAFEOP + ERROR).
EtherCAT ERROR 0-X: AL status message 0x001B: "Sync manager watchdog".
```
普通调度(`SCHED_OTHER`)无法保证 cycle period,master 错过几次 RxPDO,slave 的 SM2 watchdog 触发回退 SAFEOP。**哪些 slave 卡是随机的,因为这是抖动问题不是配置问题**。

### 修复 — 一次性配置 + 恢复 `chrt`

**一次性 sudo 配置(每台新机器只做一次):**
```bash
sudo groupadd -f realtime
sudo usermod -aG realtime $USER
sudo tee /etc/security/limits.d/realtime.conf >/dev/null <<'EOF'
@realtime   -   rtprio       99
@realtime   -   memlock      unlimited
@realtime   -   nice         -20
EOF
```
**必须注销当前桌面会话再登录**,否则 `id` 看不到 `realtime` 组,`ulimit -r` 还是 0。

**`launch` 里恢复前缀:**
```python
ros2_control_node = Node(
    ...
    prefix=['chrt -f 99']
)
```
不带 `taskset` — 除非内核启动参数加了 `isolcpus=N,...`,否则单纯钉核没有意义。

### 验证
```bash
ulimit -r          # 99
chrt -f 99 echo "rt ok"
ros2 launch armv7_bringup arm.launch.py
# launch 日志应该出现:
#   [chrt-5] Successful set up FIFO RT scheduling policy with priority 50.
#   [chrt-5] update rate is 200 Hz
```

### CPU 多核 + 真正需要硬实时时
- `/etc/default/grub` 加 `isolcpus=20,21 nohz_full=20,21 rcu_nocbs=20,21`(以 22 核机器为例)。
- launch 改 `prefix=['taskset -c 20,21 chrt -f 99']`。
- 内核改 PREEMPT_RT(`linux-image-rt-amd64` 或自编)。
普通台式机用前面那一步 `chrt -f 99` 已经足够稳。

---

## 2. EtherCAT 库位置不可硬编码

### 现象
`colcon build` 时报:
```
CMake Warning: include directory '/usr/local/etherlab/include' which doesn't exist
CMake Error: Imported target "ethercat_generic_slave::ethercat_generic_slave" includes
  non-existent path "/usr/local/etherlab/include"
```
`ethercat_generic_cia402_drive` 直接编译失败。

### 原因
原工程在两个 CMakeLists 里硬编码了 IgH EtherCAT master 的安装前缀:
- `src/ethercat_driver_ros2/ethercat_interface/CMakeLists.txt`
- `src/ethercat_driver_ros2/ethercat_manager/CMakeLists.txt`

```cmake
set(ETHERLAB_DIR /usr/local/etherlab)
```

不同机器上 IgH master 的安装位置可能不同:
| 安装方式 | 头文件 | 库 |
|---|---|---|
| 源码 `./configure --prefix=/usr/local/etherlab` | `/usr/local/etherlab/include` | `/usr/local/etherlab/lib` |
| Debian 包 `ethercat-master` / `libethercat-dev` | `/usr/include` | `/usr/lib/x86_64-linux-gnu` |
| 自定义 `--prefix=/opt/etherlab` | `/opt/etherlab/include` | `/opt/etherlab/lib` |

### 修复
改为 `pkg-config` 自动定位。修改后片段:
```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(ETHERCAT REQUIRED libethercat)
pkg_get_variable(ETHERCAT_INCLUDEDIR libethercat includedir)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERCAT_LIBRARY_DIRS})

ament_export_include_directories(
  include
  ${ETHERCAT_INCLUDEDIR}
)
```

`pkg-config` 会读取 `libethercat.pc`,该文件随任何正常打包/安装的 IgH master 提供。验证:
```bash
pkg-config --variable=includedir libethercat   # /usr/include
pkg-config --variable=libdir libethercat       # /usr/lib/x86_64-linux-gnu
pkg-config --libs libethercat                  # -lethercat
```

### 找不到 libethercat.pc 时
说明 IgH master 没装,或装在了非标准位置:
```bash
# 检查文件是否存在
find / -name "libethercat.pc" 2>/dev/null

# 临时方案:加入 PKG_CONFIG_PATH
export PKG_CONFIG_PATH=/opt/etherlab/lib/pkgconfig:$PKG_CONFIG_PATH
```
长期方案是把对应路径加进 `~/.bashrc`。

### 重新构建时必须清理旧产物
旧的 `install/ethercat_interface/share/.../ethercat_interfaceConfig.cmake` 仍会向下游导出 `/usr/local/etherlab/include`,即便您改了源码 CMakeLists,只 `colcon build` 不会刷新已 install 的 cmake 文件。必须:
```bash
cd ~/arm_ws
rm -rf build/ethercat_* install/ethercat_*
colcon build --packages-up-to ethercat_driver_ros2
```

---

## 3. EtherCAT 运行前环境准备

### IgH master 服务
```bash
sudo systemctl start ethercat
sudo systemctl status ethercat   # 看到 active (running)
```
如果服务不存在,确认 `/etc/ethercat.conf` 配置了:
```
MASTER0_DEVICE="aa:bb:cc:dd:ee:ff"   # 网卡 MAC
DEVICE_MODULES="generic"
```

### 用户权限
打开 `/dev/EtherCAT0` 需要属于 `ethercat` 组:
```bash
sudo usermod -aG ethercat $USER
newgrp ethercat
ls -l /dev/EtherCAT0    # crw-rw---- root ethercat
```

### 网卡占用
EtherCAT 接管的网卡不能同时被 NetworkManager 管理,否则会产生中断,导致丢帧。在 NetworkManager 配置里把对应接口 `unmanaged=true`,或直接把网线接在专用网卡。

### DC Sync 必须启用(关键)
[EUPH11_config.yaml](src/armv7_bringup/config/EUPH11_config.yaml)、[EUPH14_config.yaml](src/armv7_bringup/config/EUPH14_config.yaml)、[EUPH17_config.yaml](src/armv7_bringup/config/EUPH17_config.yaml) 三份从站配置文件的第 4 行:
```yaml
assign_activate: 0x0300   # DC Synch register —— 必须取消注释
```
**实测必须启用**:注释掉时只有部分 slave 进 OP,启用后 7 个全 OP。原因:
- EYOU 伺服在 OP 状态要求所有从站时钟同步,`0x0300` 让 master 给从站写 DC Sync 信号配置寄存器。
- 部分 slave 启用 DC 部分不启用会导致 master 周期与 slave 周期不一致,触发 sync 错误。
- 同型号同链路上 **三份配置文件务必保持一致** —— 要么都开,要么都关。

---

## 4. MoveIt overlay 与系统 MoveIt 冲突

### 现象 — 一旦尝试 plan & execute 就 move_group 段错误
```
Object "/home/tian/ws_moveit/install/moveit_planners_ompl/lib/libmoveit_ompl_interface.so.2.5.9"
typeinfo name for ompl_interface::JointModelStateSpaceFactory
Segmentation fault (Invalid permissions for mapped object)
```

### 原因
`~/.bashrc` 中 source 了 `~/ws_moveit/install/setup.bash` — 那是 2025 年从源码编译的 MoveIt 2.5.9 overlay。`armv7_moveit` 启动时 move_group 来自 overlay,但 `ros2_control`/`controller_manager` 等却来自当前系统 Humble。两边 ABI 不一致,加载 OMPL planner 时 typeinfo 错位 → SEGV。

### 永久修复:迁到 apt 系统 MoveIt

apt 已经有完整 MoveIt 2,无需保留 overlay:
```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-planners \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-resources \
  ros-humble-moveit-visual-tools \
  ros-humble-moveit-servo \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-task-constructor-core \
  ros-humble-srdfdom \
  ros-humble-launch-param-builder
```

> 注:`rosparam_shortcuts` 在 humble 的 apt 源里不存在,但 `armv7_moveit` 不用它,跳过即可。如果未来移植到需要 `rosparam_shortcuts` 的工程,自己单独 colcon clone+build `PickNikRobotics/rosparam_shortcuts`。

然后注释掉 `~/.bashrc` 里 source overlay 的那一行:
```bash
# source ~/ws_moveit/install/setup.bash
source ~/arm_ws/install/setup.bash
```

新开一个 shell,验证:
```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep ws_moveit  # 没输出 → ✓
which move_group                                         # /opt/ros/humble/lib/...
```

### 保留 overlay src 的理由
`~/ws_moveit/src/moveit2/moveit_ros/moveit_servo/` 下有三个 `rml_63_*` 文件是手工添加的 RML-63 servo demo 配置,与 armv7_moveit 项目无关 — 不要删 `~/ws_moveit/src`,以后该 demo 重启时还能用。但 `build/`、`install/`、`log/` 可以删:
```bash
rm -rf ~/ws_moveit/{build,install,log}
```

### 仅注释 .bashrc 不够 — 必须在干净环境下重 build arm_ws
`colcon build` 会把当时 `AMENT_PREFIX_PATH` 里所有 workspace prefix **写死**进 `arm_ws/install/setup.bash` 的 `_colcon_prefix_chain_bash_source_script` 链。哪怕事后注释掉 `~/.bashrc` 里 source overlay 的那行,只要 source 老 `arm_ws/install/setup.bash`,链里旧记录还是会把 overlay 拉回来。

正确顺序:
```bash
# 1) 注释 ~/.bashrc 里所有非系统 source 行,只留 /opt/ros/humble
# 2) 删除被污染的 arm_ws install/build
rm -rf ~/arm_ws/install ~/arm_ws/build ~/arm_ws/log

# 3) 开新 shell —— 此时 source 老 install 失败(因已删),环境只剩 /opt/ros/humble
exec bash

# 4) 验证
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'   # 只能有 /opt/ros/humble

# 5) 干净环境下重 build
cd ~/arm_ws
colcon build --symlink-install
source install/setup.bash

# 6) 校验新 setup.bash 不再 chain 任何 overlay
grep COLCON_CURRENT_PREFIX install/setup.bash
# 应当只看到 /opt/ros/humble 和 install 自身两行
```

---

## 5. 构建依赖检查清单

在新机器拉下工程后,先确认以下依赖已装:

```bash
# ROS 2 Humble + ros2_control
ros2 pkg list | grep -E 'ros2_control|hardware_interface' | head

# IgH EtherCAT master
dpkg -l | grep -E 'ethercat-master|libethercat-dev'
pkg-config --exists libethercat && echo "libethercat.pc OK"

# pkg-config 工具
which pkg-config

# rosdep
cd ~/arm_ws && rosdep install --from-paths src --ignore-src -r -y
```

全部通过后:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch armv7_bringup arm.launch.py
```

---

## 6. 已知不该 commit 的目录

提交到 Git 时务必忽略:
- `build/` `install/` `log/` — colcon 产物
- `.vscode/` `.idea/` — IDE 配置(可视团队约定保留 `.vscode/launch.json` 之类)
- `*.swp` `*~` — 编辑器临时文件
- `*.AppImage` `*.zip` — 大体积二进制
- `outputs_*` `terminal_logs.txt` `return.txt` — 运行时日志

参考根目录 `.gitignore`。
