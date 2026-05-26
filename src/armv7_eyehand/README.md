# armv7_eyehand

armv7 的**手眼相机模板**:把 RealSense(或通用 USB)相机挂到指定 link 上,并把手眼标定结果发布成 TF。

## 作用

- 提供相机的安装 xacro(eye-in-hand,相机固定在臂上)。
- 把 `easy_handeye2`(或其他标定工具)算出的外参,以 static TF 形式发布(`parent → camera`),让视觉点云/检测结果能正确变换到机械臂坐标系。

## 功能 / 内容

| 路径 | 说明 |
|---|---|
| `urdf/realsense_d435.urdf.xacro` | RealSense D435 的挂载 xacro(可改成别的相机) |
| `config/handeye_calibration.yaml` | 手眼标定结果(占位,需替换成你的实际标定) |
| `launch/handeye_publisher.launch.py` | 读 yaml,用 `tf2_ros static_transform_publisher` 发布 `parent_frame → camera_frame` |

## 配置 `config/handeye_calibration.yaml`

```yaml
handeye_publisher_node:
  ros__parameters:
    parent_frame:   link7        # 带夹爪时改成 ee_base
    camera_frame:   d435_link
    translation:   [0.05, 0.0, 0.02]      # x,y,z (m)  ← 占位,替换成标定结果
    rotation:      [0.0, 0.0, 0.0, 1.0]   # qx,qy,qz,qw
    publish_rate:  20.0
```

## 使用方法

```bash
# 1. (一次)用 easy_handeye2 标定,把结果填进 handeye_calibration.yaml

# 2. 发布手眼 TF
ros2 launch armv7_eyehand handeye_publisher.launch.py

# 3. 验证
ros2 run tf2_ros tf2_echo link7 d435_link
```

把相机挂进模型(在顶层 xacro 里用 `ee_xacro` 或自行 include):
```xml
<xacro:include filename="$(find armv7_eyehand)/urdf/realsense_d435.urdf.xacro" />
```

## 范围说明

- **v0.1**:相机挂载 xacro + 标定结果 TF 发布。标定本身用外部 `easy_handeye2`,本包只消费结果。
- 真实相机驱动(`realsense2_camera` 等)不在本包,需另装。

## 依赖

`tf2_ros`、`xacro`。标定流程需额外安装 `easy_handeye2`。

## 相关

末端帧定义见 [armv7_ee_dummy_gripper](../armv7_ee_dummy_gripper/README.md) 与 [armv7_tcp](../armv7_tcp/README.md)。
