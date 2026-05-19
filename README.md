# armv7 — 七自由度 EtherCAT 机械臂(ROS 2 Humble)

基于 ROS 2 Humble + ros2_control + MoveIt 2 + `ethercat_driver_ros2` 的 7 轴机械臂软件栈。下层用 IgH EtherCAT master 与 EYOU CiA-402 伺服驱动通信,上层用 MoveIt 2 做运动规划。

> English version coming when this project is upstreamed publicly. 中间过渡期请优先阅读这份中文文档。

---

## 当前状态

- **分支**:`main`,正在朝 v0.1.0 推进。
- **测试平台**:Ubuntu 22.04 + ROS 2 Humble + Linux 6.8 + IgH EtherCAT master 1.6.9。
- **里程碑**:Phase 1(W1.x)+ Phase 2 前 4 项(W2.1–W2.4)已完成,详见 [plan.md](plan.md)。

## 包结构

| 包 | 作用 |
|---|---|
| [`armv7_description`](src/armv7_description) | URDF、网格模型、可视化与 Gazebo 启动 |
| [`armv7_moveit_config`](src/armv7_moveit_config) | SRDF、运动规划器配置、MoveIt 2 子启动文件 |
| [`armv7_bringup`](src/armv7_bringup) | ros2_control 硬件 xacro(fake / EtherCAT 两种)、EtherCAT 从站配置、统一入口 `arm.launch.py` |
| [`armv7_safety`](src/armv7_safety) | 笛卡尔工作空间安全检测 + 软件 E-Stop |
| [`armv7_diagnostics`](src/armv7_diagnostics) | 关节健康度聚合,发布到 `/diagnostics` |
| [`armv7_py`](src/armv7_py) | Python 用户 API(`Armv7Client` — move_to_joint / jog / get_tcp_pose / stop) |
| [`armv7_cpp_api`](src/armv7_cpp_api) | C++ 用户 API(同 5 个方法,基于 rclcpp_action) |
| [`armv7_examples`](src/armv7_examples) | 示例脚本:`hello_world` / `teach_playback` / `pose_grid` |
| [`ethercat_driver_ros2/`](src/ethercat_driver_ros2) | ICube ethercat_driver_ros2 的内嵌副本,改用 `pkg-config` 自动定位 IgH master |

## 快速开始

### 1. 系统依赖(每台机器一次)
```bash
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-moveit-planners \
  ros-humble-moveit-configs-utils ros-humble-moveit-resources \
  ros-humble-moveit-visual-tools ros-humble-moveit-servo \
  ros-humble-moveit-setup-assistant ros-humble-moveit-task-constructor-core \
  ros-humble-srdfdom ros-humble-launch-param-builder \
  ethercat-master libethercat-dev pkg-config

# 实时组(让 chrt -f 99 不再需要 sudo) —— 配完务必注销重登
sudo groupadd -f realtime
sudo usermod -aG realtime $USER
sudo tee /etc/security/limits.d/realtime.conf >/dev/null <<'EOF'
@realtime - rtprio 99
@realtime - memlock unlimited
EOF

# EtherCAT 设备权限
sudo usermod -aG ethercat $USER
sudo systemctl enable --now ethercat
```

> 或者一行解决:`bash scripts/install_deps.sh`,自动完成上述所有步骤并自检。

### 2. 构建
```bash
cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动

```bash
# 干跑(不接真硬件,最快看到 MoveIt + RViz 的方式)
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false

# 真 EtherCAT 硬件(默认)
ros2 launch armv7_bringup arm.launch.py
```

无论哪种方式,RViz 会自动打开并加载 MoveIt 运动规划面板。拖动交互式 marker,点 **Plan & Execute** 即可看到运动。

## 启动参数

执行 `ros2 launch armv7_bringup arm.launch.py --show-args` 看完整列表。最常用的:

| 参数 | 默认 | 效果 |
|---|---|---|
| `use_fake_hardware` | `false` | 设为 `true` 使用 `mock_components/GenericSystem` 代替 EtherCAT,不需要真硬件 |
| `use_rviz` | `true` | 启动带 MoveIt 运动规划面板的 RViz |
| `db` | `false` | 启动 MoveIt warehouse 数据库(用于保存示教点) |
| `use_rt` | `true` | `ros2_control_node` 以 `SCHED_FIFO 99` 运行(需要先完成上面的实时组配置) |
| `use_safety` | `true` | 启动 `armv7_safety`(工作空间 bbox + E-Stop) |
| `use_diagnostics` | `true` | 启动 `armv7_diagnostics`(`/diagnostics` 聚合器) |

## 安全 + 诊断接口(Phase 2 新增)

启动后即刻可用,不需要额外配置:

```bash
# 软件急停 —— 立即停止运动
ros2 service call /safety/estop_trigger std_srvs/srv/Trigger
# 解除急停
ros2 service call /safety/estop_clear   std_srvs/srv/Trigger
# 急停状态(latched topic,订阅时立即收到当前值)
ros2 topic echo /safety/estop

# 工作空间边界
ros2 topic echo /safety/in_bounds     # TCP 是否在盒内 (bool)
ros2 topic echo /safety/bbox_state    # 'ok' | 'warning' | 'out_of_bounds'

# 关节诊断
ros2 topic echo /diagnostics --once
ros2 run rqt_robot_monitor rqt_robot_monitor       # GUI 树状展示(需先 apt install ros-humble-rqt-robot-monitor)
```

> ⚠️ 软件 E-Stop **不是**功能安全级别(无 SIL 认证)。任何与人协作的场景必须叠加硬件 E-Stop 按钮。

## 编程接口

### Python — `armv7_py`
```python
import rclpy
from armv7_py import Armv7Client

rclpy.init()
with Armv7Client() as arm:
    arm.wait_for_joint_state()
    arm.move_to_joint([0]*7, duration_sec=3.0).wait()        # 回零
    arm.jog(joint=0, delta=0.3).wait()                       # joint1 + 0.3 rad
    pose = arm.get_tcp_pose()                                # TcpPose(x, y, z, qx,...)
    print(pose)
    arm.stop()                                               # 触发软件 E-Stop
rclpy.shutdown()
```

### C++ — `armv7_cpp_api`
```cpp
#include "armv7_cpp_api/client.hpp"

rclcpp::init(0, nullptr);
{
  armv7_cpp_api::Armv7Client arm;
  arm.wait_for_joint_state();
  armv7_cpp_api::JointVector home{};
  arm.move_to_joint(home, 3.0)->wait(std::chrono::seconds(15));
}
rclcpp::shutdown();
```
两个 API 提供相同的 5 个方法:`move_to_joint` / `move_through_joints` / `jog` / `get_joint_state` / `get_tcp_pose` / `stop`。所有 move_* 是异步的,返回一个 handle,你可以选择 `.wait()` 阻塞或后续轮询 `.done()`。

### 运行示例
```bash
# 起干跑环境
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false &

# 新终端
ros2 run armv7_examples hello_world         # 回零 → 30° → 回零
ros2 run armv7_examples teach_playback      # 手动示教 5 个点位然后循环回放
ros2 run armv7_examples pose_grid           # joint1×joint2 的 4×4=16 点位栅格扫描
```

## 文档

- [plan.md](plan.md) — 4 周开发路线图,v0.1.0 → v0.2 计划。
- [PORTING_NOTES.md](PORTING_NOTES.md) — 移植到新机器时踩过的坑与对应修复。**首次部署必读**。
- [docs/installation.md](docs/installation.md) — 从空机到完成构建的完整步骤。
- [docs/quickstart.md](docs/quickstart.md) — 启动后如何让机械臂动起来。
- [docs/testing.md](docs/testing.md) — 标准功能测试手册(A/B/C 三档)。
- [docs/troubleshooting.md](docs/troubleshooting.md) — 17 个常见报错的诊断与修复。
- [GITHUB_UPLOAD_GUIDE.md](GITHUB_UPLOAD_GUIDE.md) — 把仓库推到 GitHub 的步骤。
- [SCRIPT_USAGE.md](SCRIPT_USAGE.md) — `init_and_push_to_github.sh` 使用教程。

## 仓库布局

```
arm_ws/
├── src/
│   ├── armv7_description/        # URDF + 网格(仅可视)
│   ├── armv7_moveit_config/      # MoveIt 配置 + 子启动
│   ├── armv7_bringup/            # 硬件 bringup + arm.launch.py
│   ├── armv7_safety/             # 工作空间 bbox + E-Stop
│   ├── armv7_diagnostics/        # /diagnostics 聚合
│   └── ethercat_driver_ros2/     # 已打 pkg-config 补丁的 ICube 驱动
├── scripts/install_deps.sh
├── docker/Dockerfile.dev + docker-compose.yml
├── .github/workflows/ci.yml
├── docs/
├── LICENSE                       # Apache-2.0
├── README.md  (本文件)
└── plan.md  PORTING_NOTES.md  GITHUB_UPLOAD_GUIDE.md  SCRIPT_USAGE.md
```

## 硬件信息

- **机械臂**:自研 7 自由度,SolidWorks 导出 URDF
- **驱动**:EYOU CiA-402 伺服模块(`vendor_id 0x00001097`,`product_id 0x00002406`)
- **总线**:EtherCAT(IgH master,内核驱动 `ec_master`)
- **网卡**:任意 IgH 支持的 Intel 网卡;该网卡需要从 NetworkManager 移除(`unmanaged=true`)

## 已知限制(v0.1)

- 控制周期上限 100 Hz,更高频率需要 PREEMPT_RT 内核。
- 末端执行器 / 夹爪 / 传感器还未集成(Phase 3,见 [plan.md](plan.md))。
- 阻抗控制 / 零力拖动 / 动力学辨识三个包在移植中遗失,只能从零重做 —— Phase 4 给骨架,v0.2 才会完整可用。
- `controller_manager` ≥ 2.54(Humble apt)启动时偶发并行 spawn 竞争 —— 见 [troubleshooting § controller-spawn-race](docs/troubleshooting.md#controller-spawn-race)。

## 协议

Apache-2.0,详见 [LICENSE](LICENSE)。

## 联系方式

维护者:TianFeiF <chunyvtian@gmail.com>。仓库正式公开后,欢迎 Issue 与 Pull Request。
