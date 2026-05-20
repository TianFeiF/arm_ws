# Phase 3 功能测试文档 — 末端执行器与传感器

针对 Phase 3(W3.1–W3.6)交付的功能测试手册。**前提:[docs/testing.md](testing.md) 的档位 A/B 已通过**(核心 arm + safety + diagnostics 工作正常)。

本文档只测 Phase 3 新增的部分:

| 节 | 对应 | 内容 |
|---|---|---|
| 3.1 | W3.1 | 模块化 EE xacro(`ee_xacro` arg) |
| 3.2 | W3.2 | dummy 二指夹爪(几何 + 控制器 + 开合) |
| 3.3 | W3.5 | TCP 偏移热加载 + payload topic |
| 3.4 | W3.4 | 手眼相机 mount xacro + handeye 静态 TF |
| 3.5 | W3.3 | F/T 传感器(仅文档/xacro 校验,硬件部分需实物) |
| 3.6 | W3.6 | pick_and_place 示例 |

所有测试在 **干跑模式**(`use_fake_hardware:=true`)下进行,不需要真硬件。

---

## 0. 前置清理(每次必做)

与 [testing.md § 0](testing.md) 完全相同。简版:
```bash
for i in 1 2 3 4 5; do
  for p in $(pgrep -f "ros2 launch armv7|ros2_control_node|move_group|workspace_bbox|estop_node|joint_diagnostics|robot_state_publisher|static_transform_publisher|spawner|rviz2|tcp_publisher|topic pub"); do
    kill -KILL "$p" 2>/dev/null
  done; sleep 1
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ros2 daemon stop && sleep 1 && ros2 daemon start

cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 3.0 自动化测试

```bash
colcon test --packages-select armv7_description
colcon test-result
```
**通过标准**:`6 tests, 0 failures`(含 `test_xacro_processes_description_only` / `test_xacro_processes_with_fake_hardware`,验证 xacro 三种变体都能展开)。

静态 xacro 校验(无需启动):
```bash
# 渲染 arm + fake hw + dummy gripper geometry + gripper ros2_control
xacro $(ros2 pkg prefix armv7_description)/share/armv7_description/urdf/armv7.urdf.xacro \
  ros2_control_xacro:=$(ros2 pkg prefix armv7_bringup)/share/armv7_bringup/urdf/armv7_fake.ros2_control.xacro \
  ee_xacro:=$(ros2 pkg prefix armv7_ee_dummy_gripper)/share/armv7_ee_dummy_gripper/urdf/dummy_gripper.urdf.xacro \
  ee_ros2_control_xacro:=$(ros2 pkg prefix armv7_ee_dummy_gripper)/share/armv7_ee_dummy_gripper/urdf/dummy_gripper.ros2_control.xacro \
  | grep -E "ros2_control name=|ee_finger_left_joint|ee_tcp"
```
**通过标准**:输出含 `<ros2_control name="armv7">`、`<ros2_control name="ee_gripper">`、`ee_finger_left_joint`、`ee_tcp`。

---

## 3.1 — 模块化 EE xacro(W3.1)

### 启动(带夹爪)

**终端 A**(保持开着):
```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false ee:=dummy_gripper
```

**通过标准** — 日志里依次出现:
```
got segment ee_base
got segment ee_finger_left
got segment ee_finger_right
got segment ee_tcp
Loading hardware 'ee_gripper'
Successful 'activate' of hardware 'ee_gripper'
Configured and activated joint_state_broadcaster
Configured and activated plan_group_controller
Loaded ee_gripper_controller
configure successful   (ee_gripper_controller)
```
RViz 中末端 link7 之后多出夹爪几何(灰色方块 base + 两个手指)。

### 验证 TF 链
**终端 B**:
```bash
ros2 run tf2_ros tf2_echo link7 ee_tcp --once
# 期望: Translation: [0.000, 0.000, 0.120]  (ee_base 0.02 + ee_tcp 0.10)
```

### 不带夹爪对照
```bash
# 退出终端 A,重启不加 ee
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
# RViz 中末端就是裸 link7,没有夹爪几何;ros2 control list_controllers 只有 2 个控制器
```

**通过标准**:`ee:=dummy_gripper` 时有夹爪 + 3 个控制器;不带时无夹爪 + 2 个控制器。

---

## 3.2 — Dummy 夹爪开合(W3.2)

**终端 A** 保持 `ee:=dummy_gripper` 启动状态。**终端 B**:

```bash
# 控制器状态
ros2 control list_controllers | grep ee_gripper_controller
# 期望: ee_gripper_controller ... position_controllers/JointGroupPositionController ... active

# 命令接口已 claim
ros2 control list_hardware_interfaces | grep "ee_finger_left_joint/position"
# 期望: ee_finger_left_joint/position [available] [claimed]

# 命令 topic 存在
ros2 topic list | grep ee_gripper_controller/commands
# 期望: /ee_gripper_controller/commands
```

### 开合动作(关键)
夹爪命令需要**持续发布**几秒(`--once` 可能在控制器订阅前就断开):
```bash
# 闭合到 0.010 m
timeout 2 ros2 topic pub -r 20 /ee_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.010]}"

# 读 finger 当前位置
ros2 topic echo /joint_states --once --field position
# 在 name 数组里找 ee_finger_left_joint 对应的位置,期望 ≈ 0.010

# 张开到 0.035 m
timeout 2 ros2 topic pub -r 20 /ee_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.035]}"
```
RViz 中两个手指应该对称张合(右指通过 URDF mimic 跟随左指)。

**通过标准**:命令 0.010 后 `ee_finger_left_joint` ≈ 0.010;命令 0.035 后 ≈ 0.035;RViz 手指可见运动。

> ⚠️ **已知坑**:夹爪 command_interface **不能**带 `<param name="min/max">` —— mock_components 看到限位 param 后就不再镜像 command→state,sim 里手指会一直停在初始值。真实夹爪驱动不受此影响。详见 [troubleshooting § gripper-mock-no-mirror](troubleshooting.md#gripper-mock-no-mirror)。

---

## 3.3 — TCP 偏移热加载 + payload(W3.5)

**终端 A** 任意方式启动(`use_tcp:=true` 默认开)。**终端 B**:

```bash
# tcp frame 已发布
ros2 run tf2_ros tf2_echo link7 tcp --once
# 期望: Translation: [0.000, 0.000, 0.150]  (默认 tcp_offset_xyz)

# payload latched topic
ros2 topic echo /armv7/payload --once
# 期望: data: '{"mass": 0.0, "com": [0.0, 0.0, 0.0], "inertia": [0.0, ...]}'

# 热改 TCP 偏移(立即生效,不重启)
ros2 param set /tcp_publisher_node tcp_offset_xyz "[0.0, 0.0, 0.25]"
ros2 run tf2_ros tf2_echo link7 tcp --once
# 期望: Translation: [0.000, 0.000, 0.250]

# 热改 payload
ros2 param set /tcp_publisher_node payload_mass 0.5
ros2 topic echo /armv7/payload --once
# 期望: data 里 "mass": 0.5

# 非法输入应被拒
ros2 param set /tcp_publisher_node tcp_offset_xyz "[0.0, 0.0]"
# 期望: 设置失败,提示 "tcp_offset_xyz must be length 3"
```

**通过标准**:5 条命令结果都符合期望,TCP TF 在 param 改后立即变。

---

## 3.4 — 手眼相机模板(W3.4)

无需真相机,验证 xacro + 静态 TF launch。

```bash
# 1) 相机 mount xacro 能渲染
xacro $(ros2 pkg prefix armv7_eyehand)/share/armv7_eyehand/urdf/realsense_d435.urdf.xacro 2>&1 | head -3
# 期望: 不报错(它只是宏定义,单独渲染输出空 robot 也正常)

# 2) handeye 静态 TF 发布
ros2 launch armv7_eyehand handeye_publisher.launch.py &
sleep 3
ros2 run tf2_ros tf2_echo link7 d435_link --once
# 期望: Translation: [0.05, 0.0, 0.02]  (config/handeye_calibration.yaml 的占位值)
# 收尾
pkill -f handeye_static_tf
```

**通过标准**:xacro 渲染无报错;handeye 静态 TF 按 yaml 里的占位标定值发布。

> 实际用相机时:跑 RealSense 自己的 `realsense2_camera` launch,把本包的 mount xacro 加进 arm 的 ee_xacro 链,再用 `easy_handeye2` 标定后把结果填进 `handeye_calibration.yaml`。

---

## 3.5 — F/T 传感器(W3.3,文档/模板校验)

没有实物传感器,只能验证文档里的配置片段语法正确。

```bash
# 文档存在且可读
ls docs/integration/ft_sensor.md

# 文档里的 force_torque_sensor_broadcaster 包是否装了(EtherCAT 路径需要)
ros2 pkg prefix force_torque_sensor_broadcaster 2>&1 || \
  echo "未装 — 接 EtherCAT F/T 前需 sudo apt install ros-humble-ros2-controllers"
```

**通过标准**:文档可读;清楚知道接传感器前要补哪些包。真实硬件测试在拿到传感器后按 [ft_sensor.md](integration/ft_sensor.md) 的 A.6 / B.5 验证步骤做。

---

## 3.6 — pick_and_place 示例(W3.6)

**终端 A** 必须用 `ee:=dummy_gripper` 启动(否则夹爪命令无人接收,但手臂运动仍会跑)。**终端 B**:

```bash
ros2 run armv7_examples pick_and_place
```

**通过标准** — 输出 11 步全部走完:
```
[ 1/11] HOME
[ 2/11] OPEN
  → gripper open
[ 3/11] APPR A
[ 4/11] PICK A
[ 5/11] CLOSE
  → gripper close
[ 6/11] RETREAT A
[ 7/11] APPR B
[ 8/11] PLACE B
[ 9/11] OPEN
  → gripper open
[10/11] RETREAT B
[11/11] HOME
done.
```
RViz 中机械臂在两个位姿间搬运,夹爪在 PICK 处闭合、PLACE 处张开。

**自定义点位**:
```bash
ros2 run armv7_examples pick_and_place \
    --pick  0.5 -0.5 0.0 -1.0 0.0 0.5 0.0 \
    --place -0.5 -0.5 0.0 -1.0 0.0 0.5 0.0
```

> 如果不带 `ee:=dummy_gripper` 启动,会看到 `no subscriber on /ee_gripper_controller/commands — Continuing without gripper.`,手臂照常运动,只是夹爪不动。这是预期行为。

---

## 测试报告模板

```
日期:        2026-05-20
测试人:      <名字>
arm_ws git:  <git rev-parse --short HEAD>

3.0 自动化 + xacro 校验   [ ✓ / ✗ ]
3.1 模块化 EE xacro        [ ✓ / ✗ ]
3.2 夹爪开合               [ ✓ / ✗ ]
3.3 TCP 热加载 + payload   [ ✓ / ✗ ]
3.4 手眼相机模板           [ ✓ / ✗ ]
3.5 F/T 文档校验           [ ✓ / ✗ ]
3.6 pick_and_place         [ ✓ / ✗ ]

失败项:
  - <节>: <现象> → <troubleshooting.md 哪条>
```

---

## 全 ✓ 之后

Phase 3 处于 v0.0.3-rc1 的预期状态。末端执行器框架就绪,客户可以:
- 把 `dummy_gripper` 换成自己的夹爪(照 [armv7_ee_dummy_gripper](../src/armv7_ee_dummy_gripper) 的两个 xacro 改)。
- 按 [ft_sensor.md](integration/ft_sensor.md) 接力传感器。
- 用 [armv7_eyehand](../src/armv7_eyehand) 接相机做视觉。

下一步是 Phase 4(动力学辨识 / 零力拖动骨架 / v0.1.0 发布)。
