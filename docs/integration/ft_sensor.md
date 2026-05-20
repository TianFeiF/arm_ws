# 力/扭矩传感器集成

armv7 默认不带 F/T 传感器。如果您要接入(常见于打磨、装配、力控示教场景),有两条标准路径:

| 路径 | 适合 | 推荐 |
|---|---|---|
| **A. EtherCAT 串接到现有链路** | ATI Axia80-EC、Bota SensONE、HBM K6D 等带 EC 从站的传感器 | ✓ 实时性最好,与机械臂同步采样 |
| **B. 网口 / 串口外挂,发 wrench topic** | RobotIQ FT-300、ATI Net F/T、海宇瑞星等带独立 IP 的传感器 | ✓ 改动最小,但延迟 1-5 ms |

下文都假定您把传感器装在末端 link7 与夹爪 base 之间。如果安装位置不同,请把所有 `link7` 替换成实际安装位的 link 名。

---

## 路径 A — EtherCAT 串接

### A.1 物理接线

把 F/T 传感器作为第 8 个 EtherCAT 从站,串在最末一个伺服后:
```
[Master NIC] → joint1 → joint2 → ... → joint7 → [F/T sensor] → (终结)
```

`ethercat slaves` 应能看到 8 行。EtherCAT 网线**只能串行**,不要走星型 switch。

### A.2 拿到从站的 ESI 描述

向供应商索要 `.xml` ESI 文件(ATI 在产品页能下载)。

注册到 IgH master(只做一次):
```bash
sudo cp axia80-ec.xml /etc/ethercat/eep/
sudo systemctl restart ethercat
```

### A.3 写从站 yaml

仿照 `src/armv7_bringup/config/EUPH11_config.yaml` 写一份 `axia80_config.yaml`。关键差异:

- **`vendor_id` / `product_id`**:从 ESI 文件第一行抄。
- **PDO 映射**:F/T 传感器是单向 input(TxPDO 给 master 6 个浮点 Fx/Fy/Fz/Mx/My/Mz),没有 RxPDO。
- **没有 CiA-402**:不需要 `0x6040 Control word` / `0x6041 Status word` 那一套。

模板:
```yaml
# axia80_config.yaml — ATI Axia80-EC
vendor_id:  0x00000732
product_id: 0x26483052

# 没有 RxPDO

tpdo:
  - index: 0x1A00
    channels:
      - { index: 0x6000, sub_index: 0x01, type: float, state_interface: force.x }
      - { index: 0x6000, sub_index: 0x02, type: float, state_interface: force.y }
      - { index: 0x6000, sub_index: 0x03, type: float, state_interface: force.z }
      - { index: 0x6000, sub_index: 0x04, type: float, state_interface: torque.x }
      - { index: 0x6000, sub_index: 0x05, type: float, state_interface: torque.y }
      - { index: 0x6000, sub_index: 0x06, type: float, state_interface: torque.z }
      - { index: 0x6000, sub_index: 0x07, type: uint32 }       # status (unused)
```

### A.4 把传感器加进 ros2_control xacro

在 [src/armv7_bringup/urdf/armv7_ethercat.ros2_control.xacro](../../src/armv7_bringup/urdf/armv7_ethercat.ros2_control.xacro) 的 `<ros2_control>` 块里追加(在 joint7 之后):
```xml
<sensor name="ee_ft">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
    <ec_module name="Axia80">
        <plugin>ethercat_generic_plugins/EcGenericEcSlave</plugin>
        <param name="alias">0</param>
        <param name="position">7</param>     <!-- 链上的第 8 个 -->
        <param name="slave_config">$(find armv7_bringup)/config/axia80_config.yaml</param>
    </ec_module>
</sensor>
```

### A.5 发布到标准 topic

加一个 `force_torque_sensor_broadcaster`,它把 state_interface 直接转成 `geometry_msgs/WrenchStamped`:

[src/armv7_bringup/config/ros2_controllers.yaml](../../src/armv7_bringup/config/ros2_controllers.yaml) 末尾追加:
```yaml
controller_manager:
  ros__parameters:
    ee_ft_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

ee_ft_broadcaster:
  ros__parameters:
    sensor_name: ee_ft
    frame_id:    ft_sensor_link    # 您 URDF 里的传感器 link 名
```

在 `arm.launch.py` 里把它加进 spawner(用 `OnProcessExit` 串接,见 [troubleshooting § controller-spawn-race](../troubleshooting.md#controller-spawn-race) 的模式)。

### A.6 验证
```bash
ros2 topic echo /ee_ft_broadcaster/wrench
# 期望: WrenchStamped 消息,~200 Hz
ros2 topic hz /ee_ft_broadcaster/wrench
```

---

## 路径 B — 网口 / 串口外挂

### B.1 接线 & 装驱动

按厂家说明接电源 + 通信线。常见 ROS 2 驱动:

| 厂家 | 包 | 安装 |
|---|---|---|
| RobotIQ FT-300 | https://github.com/PickNikRobotics/robotiq | colcon 源码 |
| ATI Net F/T | https://github.com/UTNuclearRoboticsPublic/netft_utils | colcon 源码 |
| 通用 modbus RTU | 自行写,模板见 `examples/dummy_ft_publisher.py`(待 v0.2) |

### B.2 启动驱动节点

每个驱动节点的具体启动命令看其 README。约定它们都发布到:
```
/ft_sensor/wrench    geometry_msgs/WrenchStamped
```
如果厂家发布到别的 topic 名,用 launch remap 改名:
```python
Node(package='vendor_pkg', executable='vendor_node',
     remappings=[('/their_topic', '/ft_sensor/wrench')])
```

### B.3 接到 ROS TF 树

传感器 wrench 默认在传感器**自身**坐标系。要在 `base_link` 下用,需要 TF。

URDF 里加一个 `ft_sensor_link`,装在 `link7` 与夹爪 base 之间(物理上传感器在那):
```xml
<!-- 在 ee xacro 之前插入 -->
<link name="ft_sensor_link"/>
<joint name="ft_sensor_joint" type="fixed">
    <parent link="link7"/>
    <child  link="ft_sensor_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>   <!-- 传感器厚度 -->
</joint>
```
然后让夹爪 xacro 的 `parent="ft_sensor_link"` 而不是 `link7`。或者在 `arm.launch.py` 里设 `ee_parent:=ft_sensor_link`。

### B.4 转换到机械臂基座下(可选)

```bash
ros2 run tf2_ros tf2_echo base_link ft_sensor_link
# 拿到旋转矩阵,把 wrench 转换为基座系
```

或写一个 Python 转换节点(在 `armv7_safety` 风格,带 ReentrantCallbackGroup):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_wrench

class FTBaseFrame(Node):
    def __init__(self):
        super().__init__('ft_to_base')
        self.tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(WrenchStamped, '/ft_sensor/wrench', self.cb, 10)
        self.pub = self.create_publisher(WrenchStamped, '/ft_sensor/wrench_base', 10)
    def cb(self, msg):
        try:
            tf = self.tf_buf.lookup_transform('base_link', msg.header.frame_id,
                                              rclpy.time.Time())
            self.pub.publish(do_transform_wrench(msg, tf))
        except Exception as e:
            self.get_logger().warn(f'tf: {e}', throttle_duration_sec=5.0)
```

### B.5 验证
```bash
ros2 topic echo /ft_sensor/wrench --once
ros2 topic hz /ft_sensor/wrench
```

---

## 阻抗 / 力控 路线图

F/T 传感器值就位后,Phase 4 的 `armv7_zero_force_controller` 就能消费它做拖动示教。Phase 4 之前,您可以:

- 在自己的应用层订阅 wrench → 用 [armv7_py](../../src/armv7_py) 触发 `arm.stop()` 作硬阈值急停。
- 写一个简单的 admittance 节点:wrench 超过阈值时,沿反方向 jog 一小步。
- 数据采集:把 wrench + joint_states 一起录 bag,后期用于动力学辨识。

---

## 安全建议

- 真硬件第一次接传感器,先把 `/safety/estop` 的反应路径走一遍 — 软件 E-Stop 在异常 wrench 时是您唯一的兜底。
- F/T 传感器对**温度漂移**敏感,首次上电后让它静置 30 分钟再做零点标定:
  ```bash
  # 大多数 EtherCAT 型的零点写在 SDO 0x1010,网口型一般在驱动节点提供 `tare` service
  ros2 service call /ft_sensor/tare std_srvs/srv/Trigger
  ```
- 力控运动要把 `update_rate` 提到至少 500 Hz,这就要求 PREEMPT_RT 内核(见 [PORTING_NOTES § 1](../../PORTING_NOTES.md#1-实时调度-不可省略))。
