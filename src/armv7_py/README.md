# armv7_py

armv7 的 **Python 用户 API**:`Armv7Client` 类,封装关节运动、jog、TCP 位姿读取、软件急停,供应用层快速调用。

## 作用

- 让外部开发者用几行 Python 驱动机械臂,不必直接和 action / service / tf 打交道。
- 仅依赖标准 `FollowJointTrajectory` action + tf2,**不需要 moveit_py**。

## 功能 / API

`from armv7_py import Armv7Client`

| 方法 | 说明 |
|---|---|
| `wait_for_joint_state(timeout=5.0)` | 阻塞直到收到首个 `/joint_states` |
| `get_joint_state()` | 返回当前 7 关节位置 `list[float]`(JOINT_NAMES 顺序) |
| `get_tcp_pose(timeout=1.0)` | 返回 `TcpPose(x,y,z,qx,qy,qz,qw)`,从 tf2 读 `base_link→link7` |
| `move_to_joint(q, duration_sec=3.0, wait=False)` | 单点关节轨迹 |
| `move_through_joints(waypoints, dt_sec=1.5, wait=False)` | 多点关节轨迹 |
| `jog(joint, delta, duration_sec=1.0, wait=False)` | 单关节相对运动(joint 0-indexed) |
| `stop()` | 触发 `/safety/estop_trigger` 软件急停 |
| `shutdown()` | 关闭后台 executor 线程 |

所有 `move_*` 返回 `TrajectoryHandle`,异步;`handle.wait(timeout)` 阻塞,或后续查 `handle.done()` / `handle.success`。

辅助:`armv7_py.client.home_pose()`(全 0)、`small_demo_pose()`(各关节 ~30°)。

## 使用方法

```python
import rclpy
from armv7_py import Armv7Client
from armv7_py.client import home_pose, small_demo_pose

rclpy.init()
with Armv7Client() as arm:               # 自动管理后台线程
    arm.wait_for_joint_state()
    print(arm.get_joint_state())
    print(arm.get_tcp_pose())

    arm.move_to_joint(home_pose(), wait=True)          # 阻塞
    h = arm.move_to_joint(small_demo_pose())           # 异步
    h.wait(timeout_sec=10)
    arm.jog(joint=0, delta=0.3, wait=True)
    arm.stop()                                          # 软件急停
rclpy.shutdown()
```

前提:`arm.launch.py` 已在运行且 `plan_group_controller` 处于 active。

## 设计要点

- 自带 `MultiThreadedExecutor` + 后台 spin 线程,`ReentrantCallbackGroup`,所以可以在主线程同步等待而不阻塞回调。
- TCP 位姿直接读 `/tf`,不需要额外发布者。

## 依赖

`rclpy`、`control_msgs`、`trajectory_msgs`、`sensor_msgs`、`geometry_msgs`、`std_srvs`、`tf2_ros`。

## 相关

- C++ 对等实现:[armv7_cpp_api](../armv7_cpp_api/README.md)
- 可运行示例:[armv7_examples](../armv7_examples/README.md)
