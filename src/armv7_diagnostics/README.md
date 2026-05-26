# armv7_diagnostics

armv7 的**关节健康度聚合**包:把 `/joint_states` 按关节限位分级,发布到标准 `/diagnostics`,供 rqt 可视化或外部监控订阅。

## 作用

- 给每个关节实时打健康等级(OK / WARN / ERROR / STALE)。
- 输出 ROS 2 标准 `diagnostic_msgs/DiagnosticArray`,可直接接 `rqt_robot_monitor` 或客户的监控看板。

## 功能 / 节点

### `joint_diagnostics_node`
订阅 `/joint_states`,读取 `armv7_moveit_config/config/joint_limits.yaml` 的限位,对每个关节发布一条 `DiagnosticStatus`:

| 字段 | 内容 |
|---|---|
| `level` | 0=OK / 1=WARN / 2=ERROR / 3=STALE |
| `name` | `armv7/jointN` |
| `values` | position / velocity / effort + max_position / max_velocity / max_effort |

发布到 `/diagnostics`,默认 ~2 Hz。

## 分级规则(`config/joint_diagnostics.yaml`)

| 条件 | 等级 |
|---|---|
| 位置 ≥ `position_warn_frac`(默认 0.95)× 限位 | WARN |
| 速度 ≥ `velocity_warn_frac`(默认 0.85)× 限位 | WARN |
| 力矩 ≥ `effort_warn_frac`(默认 0.85)× 限位 | WARN |
| 任一项超出硬限位 | ERROR |
| 超过 `stale_after_sec`(默认 1.0s)没收到 `/joint_states` | STALE |

## 使用方法

随 `arm.launch.py` 默认启动(`use_diagnostics:=true`)。也可单独启动:
```bash
ros2 launch armv7_diagnostics diagnostics.launch.py
```

查看:
```bash
# 命令行快照
ros2 topic echo /diagnostics --once

# 图形化(需先 sudo apt install ros-humble-rqt-robot-monitor)
ros2 run rqt_robot_monitor rqt_robot_monitor

# 临时调严警告阈值看 WARN 触发
ros2 param set /joint_diagnostics_node position_warn_frac 0.001
```

## 范围说明

- **v0.1 范围**:仅从 `/joint_states` 做 position/velocity/effort 分级。
- **Phase 4 计划(TODO)**:接入 EtherCAT 每个从站的驱动器温度 + 错误码(`0x603f`),做更细的硬件级诊断。

## 依赖

`rclpy`、`diagnostic_msgs`、`sensor_msgs`、`armv7_moveit_config`(读限位)。
