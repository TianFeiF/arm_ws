# armv7_description

armv7 七自由度机械臂的**描述层**包:URDF、网格模型、可视化启动。是整个工作区的几何与运动学单一真源(single source of truth),其他包都依赖它。

## 作用

- 提供机械臂的纯几何/运动学模型(无硬件、无控制)。
- 提供顶层 `armv7.urdf.xacro`,可选地注入 ros2_control 硬件标签和末端执行器,供 `armv7_bringup` / `armv7_moveit_config` 复用。
- 提供一个独立的 RViz 可视化启动,用于快速查看模型。

## 功能 / 内容

| 路径 | 说明 |
|---|---|
| `urdf/armv7.urdf` | SolidWorks 导出的纯 URDF(7 link + 7 joint),`package://armv7_description/...` 引用网格 |
| `urdf/armv7.urdf.xacro` | 顶层 xacro,带 3 个可选 arg:`ros2_control_xacro` / `ee_xacro` / `initial_positions_file` |
| `urdf/armv7.csv` | URDF Exporter 原始参数表(质量、质心、惯量),仅供人查阅 |
| `meshes/*.STL` | base_link + link1~7 的视觉/碰撞网格 |
| `config/joint_names.yaml` | 关节名列表 |
| `config/view.rviz` | display 用的 RViz 配置 |
| `launch/display.launch.py` | 仅可视化(robot_state_publisher + joint_state_publisher_gui + RViz) |
| `launch/gazebo.launch.py` | Gazebo (gz) 启动,Phase 3+ 完善中 |

## 顶层 xacro 的三种用法

```bash
# 1. 纯描述(无 ros2_control)
xacro $(ros2 pkg prefix armv7_description)/share/armv7_description/urdf/armv7.urdf.xacro

# 2. 注入 fake 硬件
xacro .../armv7.urdf.xacro \
    ros2_control_xacro:=$(ros2 pkg prefix armv7_bringup)/share/armv7_bringup/urdf/armv7_fake.ros2_control.xacro

# 3. fake 硬件 + dummy 夹爪
xacro .../armv7.urdf.xacro \
    ros2_control_xacro:=.../armv7_fake.ros2_control.xacro \
    ee_xacro:=$(ros2 pkg prefix armv7_ee_dummy_gripper)/share/armv7_ee_dummy_gripper/urdf/dummy_gripper.urdf.xacro
```

## 使用方法

```bash
# 只看模型(不需要任何硬件 / 控制器)
ros2 launch armv7_description display.launch.py
```
RViz 打开后,用 joint_state_publisher_gui 的滑块拖动各关节。

## 依赖关系

```
armv7_bringup ─┬─> armv7_moveit_config ─> armv7_description
               └────────────────────────> armv7_description
```
本包不依赖任何其他 armv7 包,处于依赖链最底层。

## 注意

- 改了网格或运动学,记得同步 `armv7_moveit_config/.setup_assistant` 里引用的路径。
- 关节限位的"权威值"在 URDF 的 `<limit>`;MoveIt 用 `armv7_moveit_config/config/joint_limits.yaml` 覆盖/增补加速度。两处需保持一致。
