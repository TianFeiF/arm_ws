# armv7_tcp

armv7 的**热加载 TCP 偏移 + 负载**包:发布一个从 link7(或夹爪基座)到可配置 `tcp` 帧的 TF,以及当前负载(质量/质心/惯量)。两者都可在运行时用 `ros2 param set` 改,换夹爪或抓取工件无需重启。

## 作用

- 给机械臂定义一个可调的工具中心点(TCP),作为规划/示教的参考帧。
- 提供一个负载信息的"挂载点",供后续动力学(Phase 4)消费。

## 功能 / 节点

### `tcp_publisher_node`

| 输出 | 类型 | 说明 |
|---|---|---|
| TF `parent_frame → tcp_frame` | `geometry_msgs/TransformStamped` | 每 tick 发(非 static,后连订阅者总能拿到最新值) |
| `/armv7/payload` | `std_msgs/String`(JSON,latched) | 当前负载 mass / com / inertia |

## 配置 `config/tcp.yaml`

```yaml
tcp_publisher_node:
  ros__parameters:
    parent_frame:   link7      # 裸臂用 link7;带夹爪指向 ee_base
    tcp_frame:      tcp
    tcp_offset_xyz: [0.0, 0.0, 0.15]   # TCP 在 parent 中的位置(m)
    tcp_offset_rpy: [0.0, 0.0, 0.0]
    publish_rate:   50.0
    payload_mass:   0.0
    payload_com:    [0.0, 0.0, 0.0]
    payload_inertia:[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # ixx,iyy,izz,ixy,ixz,iyz
```

## 使用方法

```bash
ros2 launch armv7_tcp tcp.launch.py

# 运行时改 TCP 偏移(立即生效)
ros2 param set /tcp_publisher_node tcp_offset_xyz "[0.0, 0.0, 0.20]"

# 抓到工件后更新负载
ros2 param set /tcp_publisher_node payload_mass 0.50

# 查看
ros2 run tf2_ros tf2_echo link7 tcp
ros2 topic echo /armv7/payload
```

## 范围说明

- **v0.1**:TCP TF 可用;`/armv7/payload` 只是发布出来占位。
- **Phase 4**:`armv7_dyn_ident` / `armv7_zero_force_controller` 消费 payload,把工件质量加进动力学模型。

## 依赖

`rclpy`、`geometry_msgs`、`tf2_ros`。
