# armv7_ee_dummy_gripper

armv7 的**示范末端执行器**:用方块基元画的二指平行夹爪。它本身没什么用,作用是给客户一个**接入自己夹爪的模板**。

## 作用

- 演示末端执行器如何挂到 armv7 的 link7 上(模块化 xacro 约定)。
- 提供一个 ros2_control mock 接口,让 MoveIt 的 open/close 指令有东西可对接。
- 作为接入 RobotIQ 2F-85 / OnRobot RG-2 等真实夹爪的起点。

## 功能 / 内容

| 路径 | 说明 |
|---|---|
| `urdf/dummy_gripper.urdf.xacro` | 定义 `xacro:armv7_ee parent=...` 宏:加 `ee_base` + 左右指 + 3 个关节,TCP 帧 `ee_tcp` |
| `urdf/dummy_gripper.ros2_control.xacro` | 夹爪的 ros2_control 标签(mock 接口) |
| `config/gripper_controller.yaml` | `ee_gripper_controller`(`JointGroupPositionController`,控制 `ee_finger_left_joint`) |

## 接入约定

顶层 `armv7.urdf.xacro` 通过 `ee_xacro` arg 注入本包:

```xml
<xacro:include filename="$(find armv7_ee_dummy_gripper)/urdf/dummy_gripper.urdf.xacro" />
<xacro:armv7_ee parent="link7" />
```

你的真实夹爪只要也定义一个 `armv7_ee parent=...` 宏(加 link/joint),换 `ee_xacro` 路径即可,**不用动 armv7_description**。

## 使用方法

```bash
# 带夹爪启动(fake)
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true ee:=dummy_gripper

# 控制夹爪开合(position controller)
ros2 topic pub --once /ee_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.035]}"   # 开
ros2 topic pub --once /ee_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.005]}"   # 合

# 配合抓放示例
ros2 run armv7_examples pick_and_place
```

## 换成你的真实夹爪

1. 复制本包改名(如 `armv7_ee_robotiq_2f85`)。
2. 把方块换成真实 STL/dae,更新关节限位、link 质量。
3. ros2_control xacro 改成你夹爪的真实硬件接口(EtherCAT / 串口 / 等)。
4. 启动时 `ee:=robotiq_2f85`。

## 依赖

`xacro`、`ros2_control`、`position_controllers`(运行时)。

## 相关

末端帧 / TCP 的运行时偏移见 [armv7_tcp](../armv7_tcp/README.md);相机挂载见 [armv7_eyehand](../armv7_eyehand/README.md)。
