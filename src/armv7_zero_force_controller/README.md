# armv7_zero_force_controller

armv7 的**重力补偿(零力 / 自由拖动)控制器**:一个 ros2_control 控制器,在力矩接口上实时输出 G(q),抵消重力,让机械臂"失重",可徒手拖动示教。

> ⚠️ **危险**:力矩模式下机械臂只靠模型把自己撑住。模型不准会下坠或上飘。**全程手放硬件急停上**。控制器默认 **DISABLED**,需手动 enable。

## 作用

- 提供 free-drive / 零力示教能力:抵消重力后,操作者可徒手摆动机械臂。
- 是 [armv7_dyn_ident](../armv7_dyn_ident/README.md) 辨识结果的下游消费者(用辨识的质心提升补偿精度)。

## 功能

### `GravityCompensationController`(pluginlib 控制器)
每周期用 KDL 从 `robot_description` 构建模型,算重力力矩 `G(q)`,命令到 effort 接口。最终输出力矩:

```
τ = gravity_scale · G(q)
  + coulomb_friction · tanh(q̇ / eps)     # 摩擦补偿:推哪个方向就在哪边出力
  + viscous_friction · q̇                  # 粘滞摩擦补偿
  − damping · q̇                           # 稳定性阻尼
```
然后按 `max_torque` 钳位、按 ramp 缩放、按 enable 门控写出。

附带的安全/可用机制:
- **ramp-in**:激活后力矩在 `ramp_in_time` 秒内线性升到满,避免突跳。
- **velocity_limit**:某关节超速则该周期力矩清零(失控保护)。
- **max_torque**:每关节力矩硬上限(安全天花板)。
- **damping**:粘性阻尼 `−d·q̇`,让拖动顺滑、抑制振荡。
- **enable service**:`/gravity_compensation_controller/enable`(`std_srvs/SetBool`)。
- **debug topic**:`/gravity_compensation_controller/gravity_torque`(`Float64MultiArray`)发布每周期实际写出去的总力矩,便于在线核对。

要求驱动器在**力矩模式(CiA-402 CST,mode 10)**,暴露 `effort` 命令接口。

## 配置 `config/gravity_compensation.yaml`

| 参数 | 默认 | 说明 |
|---|---|---|
| `gravity_scale` | 0.8 | 抵消多少比例重力。1.0=失重;调小让臂略"重"不上飘 |
| `ramp_in_time` | 2.0 | 力矩升满时间(s) |
| `velocity_limit` | 2.0 | 关节超速保护(rad/s) |
| `max_torque` | [30,30,12,12,5.5,5.5,5.5] | 每关节力矩上限(Nm),≤ URDF effort 限位 |
| `damping` | [0.5,...] | 粘性阻尼,稳定性用 |
| `coulomb_friction` | [0,...] | **每关节库仑摩擦补偿**(Nm),让拖动不"硬";从 armv7_dyn_ident 的输出抄过来,见下 |
| `viscous_friction` | [0,...] | 粘滞摩擦补偿;**必须 ≤ 同关节 `damping`** 否则净反馈正 → 振荡 |
| `coulomb_velocity_eps` | 0.05 | `tanh` 平滑阈值(rad/s),零速度附近抗抖 |
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

### 1. 第一步:重力本身

1. `gravity_scale: 0.8` + `enable_at_start: false`,enable 后臂略沉(正常)。
2. 逐步把 `gravity_scale` 往 1.0 调,推动省力又不上飘。
3. 拖动发飘/振荡 → 加 `damping`。
4. 某些姿态特别重/轻 → 用 [armv7_dyn_ident](../armv7_dyn_ident/README.md) 辨识后填 `identified_params_file`。

### 2. 第二步:摩擦补偿(关键)

只补重力的话,**任何方向**(包括 XY 水平推)都还要克服关节摩擦。Z 方向"感觉"轻只是因为高度变化让重力项跟着变 —— 不是摩擦更小。

`armv7_dyn_ident` 用**双向采样**采完一轮数据后,`identify` 会自动打印类似:
```
per-joint Coulomb friction |f_c| estimate (median half-diff, Nm):
  joint1:    3.665
  joint2:    3.681
  joint3:    0.801
  joint4:    0.684
  joint5:    0.190
  joint6:    0.267
  joint7:    0.233
suggested starting coulomb_friction (60% of estimate):
  [2.2, 2.21, 0.48, 0.41, 0.11, 0.16, 0.14]
```
**直接把 `suggested` 那一行抄进 `coulomb_friction`**,启动 free-drive → enable → 推一下,XY 方向手感会显著轻。

进一步调:
- 某关节还推不动 → 把对应 `coulomb_friction` 往 70–80% 调(不要直接到 100%,易抖)。
- 零速度附近抖 → 加大 `coulomb_velocity_eps`(0.08–0.1)。
- 想试粘滞摩擦补偿 → 加 `viscous_friction`,但**不要超过同关节的 `damping`**。

## 范围说明

- **v0.1(本包)**:重力补偿 + 库仑/粘滞摩擦补偿。URDF 惯性即可跑;接 armv7_dyn_ident 辨识结果后:重力精度 + 自动产出每关节摩擦估计。
- **v0.2**:完整任务空间阻抗 / 导纳控制(用 F/T 传感器或关节级力矩残差估计外力)。

## 依赖

`controller_interface`、`hardware_interface`、`pluginlib`、`kdl_parser`/`orocos_kdl`、`realtime_tools`、`std_srvs`。
