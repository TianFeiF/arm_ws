# armv7_moveit_config

armv7 的 **MoveIt 2 配置**包:SRDF、运动规划器配置、关节限位、MoveIt 子启动文件。

## 作用

- 定义规划组(`plan_group`)、关节限位、运动学求解器、规划管线(OMPL / CHOMP / Pilz)。
- 提供 MoveIt 标准子启动文件(move_group / rviz / rsp / spawn_controllers 等),被 `armv7_bringup/arm.launch.py` 复用。
- URDF 不在本包,而是引用 `armv7_description`,保持描述层单一真源。

## 功能 / 内容

| 路径 | 说明 |
|---|---|
| `config/armv7.srdf` | 规划组、虚拟关节、自碰撞矩阵 |
| `config/kinematics.yaml` | IK 求解器配置 |
| `config/joint_limits.yaml` | position/velocity/**acceleration**/effort 全字段(加速度只能在这里定义,URDF 无此字段) |
| `config/pilz_cartesian_limits.yaml` | Pilz 笛卡尔速度/加速度限制 |
| `config/moveit_controllers.yaml` | MoveIt 侧控制器接口(FollowJointTrajectory) |
| `config/moveit.rviz` | MoveIt 运动规划面板的 RViz 配置 |
| `config/initial_positions.yaml` | fake 硬件 mock 的初始关节值 |
| `.setup_assistant` | MoveIt Setup Assistant 元数据(URDF 包名 = `armv7_description`) |
| `launch/_moveit_config_loader.py` | 共享的 `build_moveit_config()` 工厂,所有子启动复用 |
| `launch/move_group.launch.py` 等 | MoveIt 标准子启动 |

## 设计要点

所有子启动通过 `_moveit_config_loader.build_moveit_config(ros2_control_xacro="")` 构建 MoveItConfig,URDF 源指向 `armv7_description/urdf/armv7.urdf.xacro`。这样:
- SRDF + 规划器配置在本包
- URDF 几何在 `armv7_description`
- 硬件 ros2_control 标签在 `armv7_bringup`

三者依赖单向无环。

## 使用方法

通常**不单独启动**,而是由 `armv7_bringup/arm.launch.py` 统一拉起。如需单独调试 MoveIt(无硬件):

```bash
# 单独起 move_group(需要外部已提供 robot_description / 控制器)
ros2 launch armv7_moveit_config move_group.launch.py

# 编辑配置(改规划组、碰撞矩阵等)
ros2 launch armv7_moveit_config setup_assistant.launch.py
```

日常使用请走:
```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true
```

## 依赖

`armv7_description`(URDF)、`moveit_*`、`pilz_industrial_motion_planner`。

## 注意

- 改关节限位时,`joint_limits.yaml`(MoveIt)与 `armv7_description/urdf/armv7.urdf` 的 `<limit>`(ros2_control 读取)要一致。
- `.setup_assistant` 里 `urdf.package` 必须是 `armv7_description`、`relative_path` 必须是 `urdf/armv7.urdf.xacro`,否则 MoveItConfigsBuilder 找不到模型。
