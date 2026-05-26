# armv7_examples

基于 [armv7_py](../armv7_py/README.md) 的**可运行示例**,演示如何用高层 API 驱动机械臂。新开发者从这里入门。

## 作用

- 提供从简到繁的几个完整脚本,既是教程,也是 bringup 改动后的烟雾测试。

## 功能 / 示例

| 命令 | 说明 |
|---|---|
| `ros2 run armv7_examples hello_world` | 回零 → 30° 姿态 → 回零(全自动) |
| `ros2 run armv7_examples pose_grid` | joint1×joint2 栅格扫描(默认 4×4=16 点) |
| `ros2 run armv7_examples teach_playback` | 交互示教:手动摆 N 个姿态捕获,然后循环回放 |
| `ros2 run armv7_examples pick_and_place` | 用 dummy 夹爪做抓取-放置(需 `ee=dummy_gripper`) |

## 使用方法

先起干跑环境(或真硬件):
```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false
```

另一个终端:
```bash
# 1. hello_world —— 最简,验证 API 通
ros2 run armv7_examples hello_world

# 2. pose_grid —— 自定义分辨率/范围
ros2 run armv7_examples pose_grid --steps 3 --range 0.3

# 3. teach_playback —— 交互
ros2 run armv7_examples teach_playback --poses 5 --repeats 2
#    提示时在 RViz 拖动机械臂到目标姿态,回终端按 Enter 捕获

# 4. pick_and_place —— 需要夹爪
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true ee:=dummy_gripper   # 终端A
ros2 run armv7_examples pick_and_place                                               # 终端B
```

> `pick_and_place` 若未加载夹爪,夹爪 service 调用只会告警跳过,手臂运动部分仍会执行,方便单独验证轨迹。

## 真机注意

真硬件首次跑示例,把幅度调小:
```bash
ros2 run armv7_examples pose_grid --steps 3 --range 0.15   # ±8° 而非默认 ±34°
```
手放硬件急停上。

## 依赖

`rclpy`、`armv7_py`。

## 相关

完整测试流程见 [docs/testing.md](../../docs/testing.md) 的档位 B。
