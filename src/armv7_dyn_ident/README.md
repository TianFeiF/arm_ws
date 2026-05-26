# armv7_dyn_ident

armv7 的**动力学(重力)参数辨识**包:静态位姿采集 + 离线最小二乘拟合,产出每个 link 的质心/一阶矩,供 [armv7_zero_force_controller](../armv7_zero_force_controller/README.md) 做精确重力补偿。

## 作用

- 在**现有位置模式**下安全地把机械臂驱动到一系列静止位姿,记录 (q, tau)。
- 离线用 URDF 重力回归子拟合 link 惯性参数(默认只辨识一阶矩 m·c,质量固定为 URDF 值)。

## 功能 / 节点 + 脚本

| 命令 | 说明 |
|---|---|
| `ros2 run armv7_dyn_ident collect` | 静态位姿数据采集,输出 CSV(`q1..qN,tau1..tauN`) |
| `ros2 run armv7_dyn_ident identify` | 读 CSV,拟合参数,输出 `identified_params.yaml` |

| 模块 | 说明 |
|---|---|
| `collect_node.py` | 用 FollowJointTrajectory 驱动到随机静止位姿,沉降后平均 `/joint_states` 采样 |
| `identify.py` | 正则化最小二乘:`min ‖Y(q)φ − τ‖² + ‖Γ(φ − φ_urdf)‖²` |
| `gravity_model.py` / `urdf_model.py` | 从 URDF 构建重力回归子 |
| `excitation.py` | 随机位姿生成 |

## 采集配置 `config/excitation.yaml`

```yaml
armv7_dyn_collect:
  ros__parameters:
    output_csv:       /tmp/armv7_gravity.csv
    n_poses:          60      # 位姿数,~60 足够辨识重力
    margin:           0.12    # 离每个关节限位留的余量
    move_time:        4.0     # 驱动到每个位姿的时间(慢=安全)
    settle_time:      1.5     # 到位后沉降时间
    samples_per_pose: 40      # 每个位姿平均的 /joint_states 条数
```

## 使用方法

```bash
# 1. 起机械臂(真机或 fake),plan_group_controller 必须 active
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true       # 终端 A

# 2. 采集(终端 B)
ros2 launch armv7_dyn_ident collect.launch.py
#    或直接:
ros2 run armv7_dyn_ident collect --ros-args -p output_csv:=/tmp/armv7_gravity.csv -p n_poses:=60

# 3. 离线辨识
ros2 run armv7_dyn_ident identify --ros-args -p csv:=/tmp/armv7_gravity.csv
#    默认固定质量、只辨识一阶矩;--free-masses 可辨识全部 4n 参数
```

产出的 `identified_params.yaml` 填进 `armv7_zero_force_controller` 的 `identified_params_file` 即可提升重力补偿精度。

## ⚠️ 安全警告

- 自动生成的随机位姿**没有碰撞检查**。第一次运行前先检查(或用 `poses` 参数传你自己的位姿,行优先展平),**手放硬件急停上**。
- 全程在位置模式,不进力矩模式,所以比 free-drive 安全;但机械臂会自己运动到各位姿。

## 范围说明

- **v0.1(本包)**:仅静态重力辨识(数据采集 + 离线拟合)。
- **Phase 4 / v0.2**:激励轨迹(傅立叶级数)+ 完整动力学(含摩擦、惯量)+ 在线辨识。

## 依赖

`rclpy`、`control_msgs`、`sensor_msgs`、`numpy`、`scipy`、`PyYAML`(辨识)。
