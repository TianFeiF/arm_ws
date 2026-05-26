# armv7_cpp_api

armv7 的 **C++ 用户 API**:`armv7_cpp_api::Armv7Client`,与 [armv7_py](../armv7_py/README.md) 同样的 5 类方法,基于 `rclcpp_action`。

## 作用

- 给 C++ 应用提供和 Python 一致的高层接口,封装 `FollowJointTrajectory` action + tf2 + 软件急停。
- 自带 `MultiThreadedExecutor` 后台线程,构造即可用。

## 功能 / API

`#include "armv7_cpp_api/client.hpp"`

| 方法 | 说明 |
|---|---|
| `wait_for_joint_state(timeout=5s)` | 等首个 `/joint_states` |
| `get_joint_state()` | `std::optional<JointVector>`(7 维) |
| `get_tcp_pose(timeout=1s)` | `std::optional<TcpPose>`,tf2 读 `base_link→link7` |
| `move_to_joint(target, duration_sec)` | 单点关节轨迹,返回 `shared_ptr<TrajectoryHandle>` |
| `move_through_joints(waypoints, dt_sec)` | 多点关节轨迹 |
| `jog(joint_index, delta_rad, duration_sec)` | 单关节相对运动 |
| `stop()` | 触发 `/safety/estop_trigger` |

`TrajectoryHandle`:`wait(timeout)` 阻塞、`done()` / `succeeded()` 轮询、`cancel()`。

构造可选项 `Armv7ClientOptions`:`base_frame` / `tcp_frame` / `action_name` / `estop_service` / `default_duration_sec`。

## 使用方法

```cpp
#include "armv7_cpp_api/client.hpp"

rclcpp::init(0, nullptr);
{
  armv7_cpp_api::Armv7Client arm;                 // 自带后台 executor
  arm.wait_for_joint_state(std::chrono::seconds(5));

  armv7_cpp_api::JointVector home{};              // 全 0
  arm.move_to_joint(home, 3.0)->wait(std::chrono::seconds(15));

  arm.jog(0, 0.3, 1.0)->wait();
}
rclcpp::shutdown();
```

自带示例可执行:
```bash
ros2 run armv7_cpp_api hello_world_cpp
```

## 在你自己的包里链接

```cmake
find_package(armv7_cpp_api REQUIRED)
ament_target_dependencies(your_target armv7_cpp_api)
```

## 依赖

`rclcpp`、`rclcpp_action`、`control_msgs`、`trajectory_msgs`、`sensor_msgs`、`geometry_msgs`、`std_srvs`、`tf2`/`tf2_ros`/`tf2_geometry_msgs`。

## 注意

- MVP 阶段关闭了 uncrustify / cpplint / copyright 三个风格 linter(见 CMakeLists 注释),v0.2 再开。
