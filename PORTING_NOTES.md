# 移植注意事项

记录本次将 `armv7_moveit` + `ethercat_driver_ros2` 工程从原机器移植到当前机器(Ubuntu 22.04 / ROS 2 Humble)时遇到的问题与修复方法。后续在新机器上部署时按本文档检查即可。

---

## 1. 移除实时调度与 CPU 绑定

### 现象
启动 `ros2 launch armv7_moveit ecdemo.launch.py` 后:
```
chrt: 设置 pid 0 的策略失败: 不允许的操作
[ERROR] [taskset-5]: process has died ... ros2_control_node ...
```
随后 `joint_state_broadcaster`、`plan_group_controller` 永远 waiting for service `/controller_manager/list_controllers`。

### 原因
`ecdemo.launch.py` 中对 `ros2_control_node` 使用了:
```python
prefix=['taskset -c 5-7 chrt -f 99']
```
- `chrt -f 99` 切换到 `SCHED_FIFO` 实时策略,需要 `CAP_SYS_NICE` 或 root。
- `taskset -c 5-7` 把进程绑定到 5/6/7 号 CPU,要求机器至少有 8 个核心,并最好配合 `isolcpus` 启动参数。

普通开发机不具备这些权限/配置,会直接退出。

### 修复
已删除该 `prefix` 字段,见 `src/armv7_moveit/launch/ecdemo.launch.py`。

### 如果新机器确实需要实时性能
两种方式二选一:

**A. 给可执行文件加 capability(推荐)**
```bash
sudo setcap cap_sys_nice+ep /opt/ros/humble/lib/controller_manager/ros2_control_node
sudo groupadd -f realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99
@realtime - memlock unlimited" | sudo tee /etc/security/limits.d/realtime.conf
# 重新登录
```

**B. 改用 PREEMPT_RT 内核 + sudo 启动**(对周期性能要求极高时再考虑)

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

---

## 4. MoveIt overlay 与系统 MoveIt 冲突(独立问题,记录备查)

### 现象
```
Object "/home/tian/ws_moveit/install/moveit_planners_ompl/lib/libmoveit_ompl_interface.so.2.5.9"
typeinfo name for ompl_interface::JointModelStateSpaceFactory
Segmentation fault
```

### 原因
`~/.bashrc` 里同时 source 了 `/opt/ros/humble/setup.bash` 和 `~/ws_moveit/install/setup.bash`,但 overlay 是较早版本编译的,与当前系统 MoveIt/OMPL ABI 不一致。

### 修复
任选其一:
- 在 `~/.bashrc` 里去掉 `source ~/ws_moveit/install/setup.bash`(只用系统 MoveIt)。
- 完全重建 overlay:
  ```bash
  sudo apt update && sudo apt upgrade ros-humble-moveit*
  rm -rf ~/ws_moveit/{build,install,log}
  cd ~/ws_moveit && colcon build
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
ros2 launch armv7_moveit ecdemo.launch.py
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
