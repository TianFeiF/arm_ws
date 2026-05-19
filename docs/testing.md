# 功能测试文档

armv7 v0.1.0 标准功能测试手册。三档由浅入深:

| 档位 | 用时 | 需硬件 | 检查范围 |
|---|---|---|---|
| **A. 自动化测试** | 1 min | 否 | 单元测试 + 静态检查 |
| **B. 干跑功能测试** | 15 min | 否 | 所有 ROS 接口在 mock 上行为正确 |
| **C. 真硬件测试** | 30 min | 是 | EtherCAT + 物理运动 |

**任何一档失败都不要进下一档**,先按对应章节排查。

---

## 0. 测试前置(每次必做)

```bash
# 0.1 杀干净所有遗留 ROS 进程(关键 — 否则 controller 抢通信、bbox 抢 topic)
for i in 1 2 3 4 5; do
    for p in $(pgrep -f "ros2 launch armv7\|ros2_control_node\|move_group\|workspace_bbox\|estop_node\|joint_diagnostics\|robot_state_publisher\|static_transform_publisher\|spawner\|rviz2" 2>/dev/null); do
        kill -KILL "$p" 2>/dev/null
    done
    sleep 1
done

# 0.2 清 FastDDS 共享内存残留(kill -9 不会释放)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*

# 0.3 重启 ros2 daemon
ros2 daemon stop && sleep 1 && ros2 daemon start

# 0.4 验证清干净
pgrep -af "ros2 launch armv7|ros2_control_node|move_group|workspace_bbox|estop_node|joint_diagnostics" | head    # 应该没输出
ls /dev/shm/ | grep -c fastrtps                                                                                  # 应该是 0

# 0.5 进 workspace,build,source
cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 跳过 0.1–0.3 是 80% 测试失败的原因。**别跳**。

---

## 档位 A — 自动化测试(1 分钟)

### A.1 colcon test

```bash
cd ~/arm_ws
colcon test
colcon test-result
```

**通过标准**:
```
Summary: 176 tests, 0 errors, 0 failures, 25 skipped
```

覆盖范围:
- `armv7_description` — URDF 结构(7 link + 7 joint)、`package://` 引用正确、xacro 三种变体都能展开。
- `armv7_py` — 模块导入、`JOINT_NAMES`、helper 函数。
- `armv7_safety` — bbox 分类器边界判定逻辑。
- 全部包 — flake8 / pep257 / xmllint / cmake_lint。

### A.2 静态检查

```bash
# launch 文件能解析
ros2 launch armv7_bringup arm.launch.py --show-args | head -20
# 应该列出 6 个参数:use_fake_hardware / use_rviz / db / use_rt / use_safety / use_diagnostics

# 包都能找到
for p in armv7_description armv7_moveit_config armv7_bringup armv7_safety armv7_diagnostics armv7_py armv7_cpp_api armv7_examples; do
    echo "$p -> $(ros2 pkg prefix $p)"
done
```

**通过标准**:6 个参数齐全;8 个包都解析到 `~/arm_ws/install/<pkg>`。

---

## 档位 B — 干跑功能测试(15 分钟,无需真硬件)

### B.1 启动

**终端 A**(launch,保持开着,**不要 Ctrl-C 直到所有 B.x 都完成**):
```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
```

**通过标准** — 日志里必须出现以下所有行:
```
EthercatDriver: System Successfully started!         (mock 也叫这个,不要紧)
controller_manager: update rate is 200 Hz
Loading controller 'joint_state_broadcaster'
Configured and activated joint_state_broadcaster
Loading controller 'plan_group_controller'
Configured and activated plan_group_controller
estop_node up; controller=plan_group_controller, ...
loaded limits for 7 joints: ['joint1', 'joint2', ..., 'joint7']
workspace_bbox watching link7 in base_link, box=(-0.60,0.60) (-0.60,0.60) (0.00,1.20) ...
bbox state (init) -> ok (TCP=...)
```

RViz 应该自动打开,左侧 Motion Planning 面板看到 armv7 模型。

> **失败时**:看到 `Failed loading controller plan_group_controller` 或 `Could not configure controller ... no controller with this name exists` → 退回 [troubleshooting § controller-spawn-race](troubleshooting.md#controller-spawn-race),最大概率是没执行 0.1–0.3 步。

---

### B.2 — 关节限位(W2.1)

**终端 B**:
```bash
# 看 MoveIt 的运动学限位是否生效 — 应该能列出 7 个 joint 的限位
ros2 topic echo /joint_states --once | head -20
```

在 RViz **Motion Planning** 面板:
1. **Planning** → **Goal State** 下拉里选 `<custom>`
2. 拖 joint2 滑块到 ±2.0(超过 ±1.57 软限位)
3. **应该卡在 ±1.57 处**,不会让你拖更多

**通过标准**:joint1/3/5/7 卡在 ±3.14;joint2/4/6 卡在 ±1.57。

---

### B.3 — 工作空间盒(W2.2)

**终端 B**:
```bash
# 当前 TCP 应该在盒内
ros2 topic echo /safety/in_bounds --once
# 期望: data: true

ros2 topic echo /safety/bbox_state --once
# 期望: data: 'ok'

# 持续刷新频率
ros2 topic hz /safety/bbox_state           # 期望: ~50 Hz
# Ctrl-C 退出

# 把盒子的上盖压到 0.5 m(TCP 默认在 0.9 m,会越界)
ros2 param set /workspace_bbox_node max_z 0.5

# 立即应该收到状态变化
ros2 topic echo /safety/bbox_state --once
# 期望: data: 'out_of_bounds'

ros2 topic echo /safety/in_bounds --once
# 期望: data: false

# 恢复
ros2 param set /workspace_bbox_node max_z 1.2
ros2 topic echo /safety/bbox_state --once
# 期望: data: 'ok'
```

**通过标准**:四次 `echo` 结果按上面顺序正确。

---

### B.4 — 软件 E-Stop(W2.3)

**终端 B**:
```bash
# 启动状态
ros2 control list_controllers | grep plan_group_controller
# 期望: ... active
ros2 topic echo /safety/estop --once
# 期望: data: false

# 触发
ros2 service call /safety/estop_trigger std_srvs/srv/Trigger
# 期望: success=True, message 包含 'deactivated plan_group_controller'

sleep 1
ros2 control list_controllers | grep plan_group_controller
# 期望: ... inactive
ros2 topic echo /safety/estop --once
# 期望: data: true

# 试图在 RViz 里 Plan & Execute → 执行应该失败(controller inactive)

# 解除
ros2 service call /safety/estop_clear std_srvs/srv/Trigger
# 期望: success=True, message 包含 'activated plan_group_controller'

sleep 1
ros2 control list_controllers | grep plan_group_controller
# 期望: ... active
ros2 topic echo /safety/estop --once
# 期望: data: false
```

**通过标准**:6 个 echo 全部符合期望,RViz 在 estop 期间无法 execute。

---

### B.5 — 关节诊断(W2.4)

**终端 B**:
```bash
ros2 topic echo /diagnostics --once

# 应该看到 7 个 status 块,每个长这样:
#   level: 0                     ← 0=OK, 1=WARN, 2=ERROR, 3=STALE
#   name: armv7/joint1
#   message: nominal
#   hardware_id: armv7
#   values:
#     - {key: position, value: '...'}
#     - {key: velocity, value: '...'}
#     - {key: effort,   value: '...'}
#     - {key: max_position / max_velocity / max_effort, ...}

# 频率
ros2 topic hz /diagnostics                  # 期望: ~2 Hz

# 触发 WARN — 把警告阈值调到很严
ros2 param set /joint_diagnostics_node position_warn_frac 0.001
sleep 1
ros2 topic echo /diagnostics --once | grep -c "level: 1"
# 期望: 7 (所有 7 个 joint 都是 WARN)

# 恢复
ros2 param set /joint_diagnostics_node position_warn_frac 0.95
```

**GUI 验证(可选)**:
```bash
sudo apt install -y ros-humble-rqt-robot-monitor    # 首次需要装
ros2 run rqt_robot_monitor rqt_robot_monitor
```
左侧 Top-level 看到 `armv7`,展开后 7 个 `armv7/jointN` 绿点。

**通过标准**:7 个 status 都打印,频率 ~2 Hz,WARN 模式下全部 level=1。

---

### B.6 — Python API(W2.5)+ 示例(W2.7)

**终端 B**:
```bash
# 示例 1 — hello_world(自动)
ros2 run armv7_examples hello_world
```
**期望输出**:
```
current joints: ['0.000', '0.000', '0.000', '0.000', '0.000', '0.000', '0.000']
current TCP   : (-0.006, -0.002, 0.912)
→ moving to home pose...
→ moving to 30° pose...
→ returning home...
done.
```
**视觉验证**:RViz 中机械臂依次:回零 → 30° 倾斜 → 回零。

```bash
# 示例 2 — pose_grid(自动,16 点位扫描,约 25 秒)
ros2 run armv7_examples pose_grid --steps 4
# 期望: "sweeping 16 poses ..." 然后机械臂在 RViz 中扫过 16 个点

# 示例 3 — teach_playback(交互式)
ros2 run armv7_examples teach_playback --poses 3 --repeats 1
# 提示 "Pose 1/3 — press Enter to capture:"
# 在 RViz 拖 arm 到 3 个不同姿态,每次拖完回到终端按 Enter
# 然后机械臂在 RViz 里把 3 个姿态依次回放一次
```

**通过标准**:三个示例都正常退出(`return 0`),RViz 中机械臂动作可见。

---

### B.7 — 自己写一段 Python 验证 API

新建 `/tmp/test_arm_api.py`:
```python
import rclpy
from armv7_py import Armv7Client
from armv7_py.client import home_pose, small_demo_pose

rclpy.init()
with Armv7Client() as arm:
    assert arm.wait_for_joint_state(), "no /joint_states!"
    print("当前关节:", arm.get_joint_state())
    print("当前 TCP :", arm.get_tcp_pose())

    print("→ 移到 demo pose")
    ok = arm.move_to_joint(small_demo_pose(), duration_sec=2.5).wait(timeout_sec=10)
    print("成功" if ok else "失败")

    print("→ 单关节 jog: joint1 +0.5 rad")
    arm.jog(0, 0.5, duration_sec=1.5).wait()
    print("当前关节:", arm.get_joint_state())

    print("→ 触发软件急停")
    arm.stop()

    print("→ 回零")
    arm.move_to_joint(home_pose(), duration_sec=3.0).wait(timeout_sec=10)
rclpy.shutdown()
```
```bash
python3 /tmp/test_arm_api.py
```

**通过标准**:所有 print 都有输出,"成功" 出现一次,RViz 中可见运动。

---

### B.8 — C++ API(W2.6)

```bash
ros2 run armv7_cpp_api hello_world_cpp
```
**期望输出**:
```
current joints: 0.000 0.000 0.000 0.000 0.000 0.000 0.000
home reached.
```

如果机械臂已经在零位,看不到运动,可以先调一下:
```bash
ros2 run armv7_examples pose_grid --steps 2     # 把它推离零位
ros2 run armv7_cpp_api hello_world_cpp          # 应该把它拉回零位
```

**通过标准**:程序正常退出,RViz 中机械臂动了。

---

### B.9 — 关闭

回终端 A,Ctrl-C。等 5 秒看到所有进程 `process has finished cleanly`。

**通过标准**:没有 `process has died [exit code -11/-6]` 之类的崩溃日志。

如果有残留进程:
```bash
pgrep -af "ros2 launch armv7|ros2_control_node|move_group|workspace_bbox|estop_node|joint_diagnostics" | wc -l
# 应该是 0
```
非 0 就回 0.1–0.3。

---

## 档位 C — 真硬件测试(30 分钟)

**做这一档前 B 档全部通过 是硬性前提**。

### C.1 物理预检

```bash
# IgH master 服务
systemctl is-active ethercat                      # active
# 7 个 slave 在线
ethercat slaves | wc -l                            # 7
# 设备节点权限
ls -l /dev/EtherCAT0                              # crw-rw-r-- root ethercat
# 实时权限
ulimit -r                                          # 99
chrt -f 99 echo ok                                # 输出 ok
```

**通过标准**:5 个全 OK。任何一个失败回 [installation.md](installation.md)。

### C.2 启动真硬件

```bash
ros2 launch armv7_bringup arm.launch.py
# (use_fake_hardware 默认 false,use_rt 默认 true)
```

**通过标准** — 必须看到:
```
EthercatDriver: Activated EcMaster!
controller_manager: update rate is 200 Hz
controller_manager: Successful set up FIFO RT scheduling policy with priority 50.
Configured and activated joint_state_broadcaster
Configured and activated plan_group_controller
```
另一个终端 `watch -n 1 'ethercat slaves'` 应该 7 个 slave 全部 `OP +`,不变。

### C.3 安全第一 — 先测 E-Stop

```bash
# 触发软件急停
ros2 service call /safety/estop_trigger std_srvs/srv/Trigger
# 立即尝试在 RViz Plan & Execute → 机械臂应该不动
# 解除
ros2 service call /safety/estop_clear std_srvs/srv/Trigger
# 再 Plan & Execute → 机械臂正常运动
```

**通过标准**:急停期间 plan & execute 失败,解除后恢复。

### C.4 重复 B 档第 6–8 节(小幅运动版本)

把所有示例的运动幅度先调小,确认机械臂不会乱动:

```bash
# 示例 1 — 不变,zero 和 30° 是安全的
ros2 run armv7_examples hello_world

# 示例 2 — 默认 range=0.6 rad(±34°),先用 0.15 rad(±8°)试
ros2 run armv7_examples pose_grid --steps 3 --range 0.15

# 示例 3 — 不变
ros2 run armv7_examples teach_playback --poses 3 --repeats 1

# C++ — 不变
ros2 run armv7_cpp_api hello_world_cpp
```

**通过标准**:每个示例正常退出;真臂跟着 RViz 同步动;没有 EtherCAT 错误(`watch ethercat slaves` 一直保持 `OP +`)。

### C.5 长时运行测试(可选,30 分钟)

```bash
# 起一个循环:连续 30 分钟在 pose_grid 之间来回
while true; do
    ros2 run armv7_examples pose_grid --steps 2 --range 0.2
    sleep 2
done
```
同时观察:
- `dmesg -wT | grep -i ethercat` — 不应有新增 ERROR
- `ros2 topic hz /joint_states` — 稳定 ~50 Hz
- `ros2 topic echo /diagnostics --once | grep "level:"` — 全是 level: 0

**通过标准**:30 分钟内无 EtherCAT 报错,关节诊断全 OK,机械臂运动平滑。

---

## 测试报告模板

跑完一档后填一份(粘到 issue / 邮件给客户):

```
日期:        2026-05-19
测试人:      <你的名字>
平台:        Ubuntu 22.04 / kernel <uname -r>
ROS 2:       Humble apt
arm_ws git:  <git rev-parse --short HEAD>

档位 A 自动化:        [ ✓ / ✗ ]  通过 / 失败说明
档位 B 干跑:
  B.1 启动              [ ✓ / ✗ ]
  B.2 关节限位          [ ✓ / ✗ ]
  B.3 工作空间盒        [ ✓ / ✗ ]
  B.4 E-Stop            [ ✓ / ✗ ]
  B.5 诊断              [ ✓ / ✗ ]
  B.6 Python 示例       [ ✓ / ✗ ]
  B.7 自定义 Python     [ ✓ / ✗ ]
  B.8 C++ API           [ ✓ / ✗ ]
  B.9 干净关闭          [ ✓ / ✗ ]
档位 C 真硬件:
  C.1 物理预检          [ ✓ / ✗ ]
  C.2 启动              [ ✓ / ✗ ]
  C.3 E-Stop            [ ✓ / ✗ ]
  C.4 示例运动          [ ✓ / ✗ ]
  C.5 长时运行          [ ✓ / ✗ / 跳过 ]

失败项详情:
  - <子档位>: <错误现象> + <对照 troubleshooting.md 哪一条>

附件:
  - launch 日志:      <文件名或路径>
  - dmesg 截取:       <文件名或路径>
```

---

## 各档位预计耗时

| 档位 | 第一次 | 熟练后 |
|---|---|---|
| 0. 前置 | 3 min | 1 min |
| A     | 1 min | 1 min |
| B     | 25 min | 12 min |
| C     | 60 min | 25 min |

---

## 跑过这套测试之后

- 全 ✓ → 项目处于 v0.0.2-beta(W2 完成)的预期状态,可以推 GitHub 给外部协作者。
- 有 ✗ → 先按 troubleshooting.md 对应章节排查;实在不行整理出最小复现步骤再求助。
