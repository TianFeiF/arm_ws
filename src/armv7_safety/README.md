# armv7_safety

armv7 的**软件安全层**:笛卡尔工作空间边界检测 + 软件 E-Stop。

> ⚠️ 这不是功能安全级别(无 SIL 认证)。与人协作场景必须叠加**硬件 E-Stop 按钮**。本包用于程序化急停和工作区监控。

## 作用

- 监控 TCP 是否在允许的工作空间盒内,越界时发警告(可选自动急停)。
- 提供软件急停接口:任何脚本 / 界面 / 远程节点都能触发停机并恢复。

## 功能 / 节点

### `workspace_bbox_node`
监听 TF(`base_link → link7`),按配置的立方体盒判定 TCP 状态,发布:

| Topic | 类型 | 说明 |
|---|---|---|
| `/safety/in_bounds` | `std_msgs/Bool` | TCP 是否在盒内,每 tick 发 |
| `/safety/bbox_state` | `std_msgs/String`(latched) | `ok` / `warning`(距边界 < margin)/ `out_of_bounds` |

### `estop_node`
软件急停状态机,触发后调用 `controller_manager/switch_controller` 停用 `plan_group_controller`:

| 接口 | 类型 | 说明 |
|---|---|---|
| `/safety/estop` | `std_msgs/Bool`(latched) | 急停状态,`true`=已停 |
| `/safety/estop_trigger` | `std_srvs/Trigger` | 触发急停(停用轨迹控制器) |
| `/safety/estop_clear` | `std_srvs/Trigger` | 解除急停(重新激活轨迹控制器) |

## 配置 `config/workspace_bbox.yaml`

```yaml
workspace_bbox_node:
  ros__parameters:
    frame_id: base_link
    tcp_frame_id: link7
    min_x/max_x/min_y/max_y/min_z/max_z: ...   # 盒子边界(米)
    margin: 0.05            # 距边界多近算 warning
    auto_estop_on_exit: false   # 越界是否自动触发急停
```

## 使用方法

随 `arm.launch.py` 默认启动(`use_safety:=true`)。也可单独启动:
```bash
ros2 launch armv7_safety safety.launch.py
```

常用命令:
```bash
# 软件急停 / 恢复
ros2 service call /safety/estop_trigger std_srvs/srv/Trigger
ros2 service call /safety/estop_clear   std_srvs/srv/Trigger

# 看状态
ros2 topic echo /safety/estop          # latched,订阅即收到当前值
ros2 topic echo /safety/bbox_state     # latched
ros2 topic echo /safety/in_bounds

# 运行时改盒子大小
ros2 param set /workspace_bbox_node max_z 0.5
```

## 实现要点

- `estop_node` 用 `MultiThreadedExecutor` + `ReentrantCallbackGroup`,允许在 service 回调里同步等待 `switch_controller` 异步调用而不死锁;并在 RPC 超时后通过 `list_controllers` 复核实际控制器状态。
- `/safety/estop` 和 `/safety/bbox_state` 都是 latched(`TRANSIENT_LOCAL`),后连的订阅者立即拿到当前值。

## 依赖

`rclpy`、`std_msgs`、`std_srvs`、`tf2_ros`、`controller_manager_msgs`、`sensor_msgs`。
