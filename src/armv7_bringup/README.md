# armv7_bringup

armv7 的**硬件 bringup**包:ros2_control 硬件 xacro(fake / EtherCAT 两种)、EtherCAT 从站配置、控制器配置,以及统一入口 `arm.launch.py`。这是日常启动机械臂的包。

## 作用

- 把 `armv7_description`(几何)+ `armv7_moveit_config`(规划)+ ros2_control 硬件 + 安全/诊断层组装成一个可运行系统。
- 通过 `use_fake_hardware` 参数在 mock 仿真和真 EtherCAT 硬件之间切换。

## 功能 / 内容

| 路径 | 说明 |
|---|---|
| `launch/arm.launch.py` | **统一入口**,见下方参数表 |
| `urdf/armv7_fake.ros2_control.xacro` | mock 硬件(`mock_components/GenericSystem`) |
| `urdf/armv7_ethercat.ros2_control.xacro` | 真硬件,EtherCAT + EYOU CiA-402 驱动,7 个 `ec_module` |
| `config/ros2_controllers.yaml` | controller_manager + plan_group_controller + joint_state_broadcaster |
| `config/EUPH11_config.yaml` 等 | 三种 EYOU 从站的 PDO/SDO 映射(joint1/2→EUPH17,3/4→EUPH14,5/6/7→EUPH11) |

## 启动参数

`ros2 launch armv7_bringup arm.launch.py --show-args`

| 参数 | 默认 | 作用 |
|---|---|---|
| `use_fake_hardware` | `false` | `true` 用 mock,不需要真硬件 |
| `use_rviz` | `true` | 启动 MoveIt RViz 面板 |
| `db` | `false` | 启动 MoveIt warehouse 数据库 |
| `use_rt` | `true` | `ros2_control_node` 以 `SCHED_FIFO 99` 运行(需 realtime 组) |
| `use_safety` | `true` | 启动 `armv7_safety`(工作空间 bbox + E-Stop) |
| `use_diagnostics` | `true` | 启动 `armv7_diagnostics` |

## 使用方法

```bash
# 干跑(无硬件)
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false

# 真 EtherCAT 硬件(默认)
ros2 launch armv7_bringup arm.launch.py

# 无头(CI / 服务器)
ros2 launch armv7_bringup arm.launch.py use_rviz:=false
```

## 启动时序(arm.launch.py 内部)

1. `t=0` robot_state_publisher + move_group + RViz
2. `t=5s` `ros2_control_node`(等 MoveIt 稳定)
3. `t=7s` spawner #1 加载 `joint_state_broadcaster`
4. **`joint_state_broadcaster` spawner 进程退出后**(`OnProcessExit`)才启动 spawner #2 加载 `plan_group_controller`
5. `t=8s` 安全层 + 诊断层

> 第 3-4 步用 `OnProcessExit` 链式 spawner,是为了规避 `controller_manager ≥ 2.54` 并行 spawn 的竞争(见 [docs/troubleshooting.md § controller-spawn-race](../../docs/troubleshooting.md#controller-spawn-race))。

## 依赖

`armv7_description`、`armv7_moveit_config`、`armv7_safety`、`armv7_diagnostics`、`ethercat_driver`、`ethercat_generic_cia402_drive`、`ros2_control*`。

## 注意

- 真硬件启动前确认 IgH master 服务、`/dev/EtherCAT0` 权限、realtime 组都就绪(见 [docs/installation.md](../../docs/installation.md))。
- EUPH 配置第 4 行 `assign_activate: 0x0300`(DC Sync)必须启用,否则部分从站进不了 OP。
- EUPH 配置 + cia402 驱动已打补丁防止电机进 OP 冲零点,见 [docs/troubleshooting.md § joints-jump-to-zero](../../docs/troubleshooting.md#joints-jump-to-zero)。
