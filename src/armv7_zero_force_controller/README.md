# armv7_zero_force_controller

armv7 的**重力补偿(零力 / 自由拖动)控制器**:一个 ros2_control 控制器,在力矩接口上实时输出 G(q),抵消重力,让机械臂"失重",可徒手拖动示教。

> ⚠️ **危险**:力矩模式下机械臂只靠模型把自己撑住。模型不准会下坠或上飘。**全程手放硬件急停上**。控制器默认 **DISABLED**,需手动 enable。

## 作用

- 提供 free-drive / 零力示教能力:抵消重力后,操作者可徒手摆动机械臂。
- 是 [armv7_dyn_ident](../armv7_dyn_ident/README.md) 辨识结果的下游消费者(用辨识的质心提升补偿精度)。

## 功能

### `GravityCompensationController`(pluginlib 控制器)
每周期用 KDL 从 `robot_description` 构建模型,算重力力矩 `G(q)`,命令到 effort 接口。带:
- **ramp-in**:激活后力矩在 `ramp_in_time` 秒内线性升到满,避免突跳。
- **velocity_limit**:某关节超速则该周期力矩清零(失控保护)。
- **max_torque**:每关节力矩硬上限(安全天花板)。
- **damping**:粘性阻尼 `−d·q̇`,让拖动顺滑、抑制振荡。
- **enable service**:`/gravity_compensation_controller/enable`(`std_srvs/SetBool`)。

要求驱动器在**力矩模式(CiA-402 CST,mode 10)**,暴露 `effort` 命令接口。

## 配置 `config/gravity_compensation.yaml`

| 参数 | 默认 | 说明 |
|---|---|---|
| `gravity_scale` | 0.8 | 抵消多少比例重力。1.0=失重;调小让臂略"重"不上飘 |
| `ramp_in_time` | 2.0 | 力矩升满时间(s) |
| `velocity_limit` | 2.0 | 关节超速保护(rad/s) |
| `max_torque` | [30,30,12,12,5.5,5.5,5.5] | 每关节力矩上限(Nm),≤ URDF effort 限位 |
| `damping` | [0.5,...] | 粘性阻尼 |
| `enable_at_start` | false | 启动即上力矩?默认否,需手动 enable |
| `identified_params_file` | (空) | 填 armv7_dyn_ident 产出可提升精度;空则用 URDF 惯性 |

## 使用方法

```bash
# 真机(驱动器切到 CST / mode 10):
ros2 launch armv7_zero_force_controller free_drive.launch.py

# 干跑(mock,只验证控制器加载,无物理):
ros2 launch armv7_zero_force_controller free_drive.launch.py use_fake_hardware:=true use_rt:=false

# 确认清场、手在急停上后,使能力矩:
ros2 service call /gravity_compensation_controller/enable std_srvs/srv/SetBool "{data: true}"

# 关闭力矩:
ros2 service call /gravity_compensation_controller/enable std_srvs/srv/SetBool "{data: false}"
```

> free-drive 不加载 MoveIt —— 驱动器不能同时在位置模式和力矩模式。要回正常运动规划,Ctrl-C 后改用 `arm.launch.py`。

## 调参建议

1. 先 `gravity_scale: 0.8` + `enable_at_start: false`,enable 后观察是否缓慢下坠(正常,略重)。
2. 逐步把 `gravity_scale` 往 1.0 调,直到推动省力又不上飘。
3. 拖动发飘/振荡 → 加 `damping`。
4. 补偿明显不准(某些姿态特别重/轻)→ 用 `armv7_dyn_ident` 辨识后填 `identified_params_file`。

## 范围说明

- **v0.1(本包)**:基础重力补偿骨架,用 URDF 惯性即可跑,精度有限。
- **v0.2**:完整阻抗/导纳控制、用辨识结果 + 摩擦模型。

## 依赖

`controller_interface`、`hardware_interface`、`pluginlib`、`kdl_parser`/`orocos_kdl`、`realtime_tools`、`std_srvs`。
