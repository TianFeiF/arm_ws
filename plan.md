# armv7 7-DoF EtherCAT 机械臂 — 开发计划 (v0.1)

> 目标受众:**外部合作伙伴 / 客户**
> 总工期:**4 周(短期 MVP)**
> 当前状态:硬件链路已通,可通过 MoveIt RViz Plan & Execute。

---

## 1. 愿景与边界

### 1.1 4 周后应该交出什么
外部开发者拿到仓库后,**1 小时内**能完成:
1. `git clone` → 跑一键 setup 脚本 → 装齐依赖。
2. 接好硬件 → `ros2 launch armv7_bringup arm.launch.py` 看到 MoveIt RViz。
3. 跑 `ros2 run armv7_examples hello_world.py` 看到机械臂动起来。
4. 翻 `docs/troubleshooting.md` 自查常见报错。

### 1.2 4 周内**不做**的事(明确边界,避免摊大)
| 不做 | 原因 |
|---|---|
| 工业级安全认证(ISO 10218 / TS 15066) | 周期跨度数月,需要专门人力 |
| 多机协调 / 双臂 | 当前仅单臂 |
| Web UI / 远程遥操 | 外部客户优先要 API,不是 GUI |
| 自研运动规划器 | MoveIt OMPL 已足够 v0.1 |
| 完整阻抗控制实现 | 单纯阻抗循环 1 周写不完(需要 task-space 接口、力反馈、可靠 dyn ident),v0.2 再做 |

### 1.3 高级控制(impedance / dyn_ident / zero_force)的现实交付
源代码已遗失。4 周内**只能给出骨架 + 接口约定 + 数据采集脚本**,真正的算法实现留到 v0.2(下一期)。这样外部客户看得到方向,但不会被半成品坑。

---

## 2. 里程碑总览

```
W1  ████████░░░░░░░░░░░░░░░░  包装 + 文档 + 上手  → v0.0.1-alpha (内部可跑)
W2  ░░░░░░░░████████░░░░░░░░  安全 + API + 示例   → v0.0.2-beta
W3  ░░░░░░░░░░░░░░░░████████  末端 / 传感器       → v0.0.3-rc1
W4  ░░░░░░░░░░░░░░░░░░░░████  高级控制骨架 + 发布 → v0.1.0
```

每周末打一个 git tag,客户可以按需停在任何里程碑。

---

## 3. Phase 1 — Week 1:包装与上手 (P0)

> 目标:把"机械臂能跑"做成"任何人能 5 步跑起来"。

### 3.1 交付清单

| # | 任务 | 验收标准 | 估时 |
|---|---|---|---|
| 1.1 | **README.md 重写** | 项目介绍、依赖、快速开始(3 行命令)、链接到 docs。无 emoji,中英文双语段落。 | 0.5d |
| 1.2 | **LICENSE 选定 + COPYRIGHT 头** | 根目录有 LICENSE 文件;src 下 cpp/py 顶部统一加 SPDX 头。建议 Apache-2.0(对客户友好)。 | 0.5d |
| 1.3 | **重命名 `armv7_moveit` → `armv7_bringup` 拆分** | `armv7_bringup`(launch + sim/real 切换)、`armv7_moveit_config`(纯 MoveIt 配置)、`armv7_description`(URDF + meshes,改名自 `armv7`)。 | 1.5d |
| 1.4 | **launch 入口统一** | `ros2 launch armv7_bringup arm.launch.py` 支持 `use_fake_hardware:=true\|false`;现 `ecdemo.launch.py` 标记 deprecated。 | 1d |
| 1.5 | **一键 setup 脚本 `scripts/install_deps.sh`** | 装 apt 依赖、IgH master 包、设 udev、加 realtime 组。脚本结尾自检并打印 next-step。 | 1d |
| 1.6 | **`docs/installation.md` + `docs/quickstart.md`** | 从空机器到机械臂动作,含截图。覆盖 PORTING_NOTES 已记录的所有坑。 | 1d |
| 1.7 | **`docs/troubleshooting.md`** | 至少 10 条已知报错 + 解决步骤(已经积累了一半,直接整理)。 | 0.5d |
| 1.8 | **Docker:`docker/Dockerfile.dev` + `docker-compose.yml`** | `docker compose up dev` 进入开发容器;CI 用同一份。 | 1.5d |
| 1.9 | **GitHub Actions:build + lint** | `.github/workflows/ci.yml` 跑 `colcon build` + `ament_lint` + `ament_cppcheck`。 | 1d |

### 3.2 验收门
- 一台干净的 Ubuntu 22.04,跑 `scripts/install_deps.sh` 后直接 `ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true` 看到 RViz + MoveIt + 机械臂可规划。
- CI 在 PR 上跑,失败阻断合并。
- `git tag v0.0.1-alpha`。

---

## 4. Phase 2 — Week 2:安全 + API + 示例 (P0)

> 目标:客户能通过代码安全调用机械臂,出问题能停得住。

### 4.1 交付清单

| # | 任务 | 验收标准 | 估时 |
|---|---|---|---|
| 2.1 | **关节软限位** | `config/joint_limits.yaml` 全字段填好(position / velocity / acceleration / effort);MoveIt + ros2_control 都生效。 | 0.5d |
| 2.2 | **笛卡尔工作空间盒** | URDF 中加 `<workspace_bbox>` 自定义元素;一个 ROS 2 节点订阅 TF 并发布 `/safety/in_bounds` 状态。 | 1d |
| 2.3 | **E-Stop 服务 + topic** | `armv7_safety` 包提供:`/estop` topic、`/estop_trigger` service、`/estop_clear` service。触发后立即调用 `controller_manager` 的 `switch_controller` 切到 `emergency_stop_controller`(保持当前关节)。 | 1.5d |
| 2.4 | **诊断聚合** | `armv7_diagnostics` 包,订阅 EtherCAT 每个 slave 的 error_code/电流/温度,发布 `/diagnostics`(diagnostic_msgs/DiagnosticArray)。`rqt_runtime_monitor` 直接可视化。 | 1d |
| 2.5 | **Python API `armv7_py`** | 提供 `Armv7Client` 类:`move_to_joint(q)`、`move_to_pose(pose, frame)`、`jog(direction, speed, duration)`、`get_joint_state()`、`get_tcp_pose()`、`stop()`。基于 MoveItPy + tf2。 | 1.5d |
| 2.6 | **C++ API 头文件** | `armv7_cpp_api` 包,同样的 5 个方法,基于 moveit_cpp。 | 1d |
| 2.7 | **示例脚本 `armv7_examples`** | `hello_world.py`(移动到 home)、`teach_playback.py`(5 个示教点循环)、`pose_grid.py`(网格扫描)。每个都有详细注释 + README。 | 1d |
| 2.8 | **单元测试** | `armv7_description` URDF 自检(`check_urdf`)、`armv7_safety` mock 测试 E-Stop 切换、`armv7_py` 用 fake_hardware 跑 hello_world。 | 1d |

### 4.2 验收门
- 客户写 5 行 Python 能让机械臂动:`Armv7Client().move_to_joint([0]*7)`。
- 按下 E-Stop topic 后 100ms 内机械臂停;再 clear 后能继续。
- `ros2 topic echo /diagnostics` 看到 7 个 joint 的健康度。
- CI 增加 `colcon test` step。
- `git tag v0.0.2-beta`。

---

## 5. Phase 3 — Week 3:末端执行器 + 传感器 (P1)

> 目标:客户带着自己的夹爪 / 力传感器 / 相机来,文档清晰告诉他们怎么接入。

### 5.1 交付清单

| # | 任务 | 验收标准 | 估时 |
|---|---|---|---|
| 3.1 | **末端执行器模块化 xacro** | `armv7_description/urdf/armv7.urdf.xacro` 把 link7 之后做成可选 `xacro:include`,通过 launch arg `ee_xacro:=path` 切换。 | 1d |
| 3.2 | **示范 EE:dummy 二指夹爪 xacro** | `armv7_ee_dummy_gripper` 包,提供 mesh + xacro + 一个 mock command interface(position 控制)。客户照着改。 | 1d |
| 3.3 | **F/T 传感器接入文档 + 接口约定** | `docs/integration/ft_sensor.md`:支持两种路径 — (a) EtherCAT 型(ATI Axia80-EC 等)添加到现有 ros2_control,模板代码;(b) 网口型(RobotIQ FT-300、ATI Net F/T)用 `wrench_msgs/WrenchStamped` topic。 | 1d |
| 3.4 | **手眼相机模板** | `armv7_eyehand` 包:提供 RealSense / 通用 USB 相机的 mount xacro + launch + `easy_handeye2` 集成。客户填外参 yaml 即可。 | 1.5d |
| 3.5 | **TCP / payload 热加载** | `/armv7/tcp_offset` 与 `/armv7/payload` 参数,运行时改了立即影响 MoveIt 与 dynamics。 | 1d |
| 3.6 | **加 EE 后的示例** | `pick_and_place_demo.py`:从 A 点抓 → B 点放,使用 dummy gripper。配 `ee_xacro:=dummy_gripper` 即可运行。 | 1d |

### 5.2 验收门
- 用 dummy_gripper 跑 `pick_and_place_demo.py` 成功。
- 换 EE 只需要写一个 xacro + 改一个 launch arg,**不动 armv7_description**。
- `git tag v0.0.3-rc1`。

---

## 6. Phase 4 — Week 4:高级控制骨架 + 发布 (P1 + P2)

> 目标:为 v0.2 的高级控制铺路;同时打包 v0.1.0 正式发布。

### 6.1 交付清单(范围克制)

| # | 任务 | 范围说明 | 估时 |
|---|---|---|---|
| 4.1 | **`armv7_dyn_ident` 重建** | 仅交付数据采集 + 离线处理:(a) ROS 2 节点跑预设激励轨迹(7 次幂 + 傅立叶级数);(b) 录 `joint_states` 到 ROS bag;(c) `scripts/identify.py` 用 SymPy + scipy.least_squares 拟合 Newton-Euler base parameters。**不做在线**。 | 2d |
| 4.2 | **`armv7_zero_force_controller` 骨架** | ros2_controllers 自定义控制器(`controller_interface::ControllerInterface`),只实现重力补偿(用 URDF inertias 即可,精度差但能跑)。effort_interface 直接命令 0 + grav_comp。 | 2d |
| 4.3 | **`armv7_impedance_moveit` starter** | **不实现控制循环**,只交付:(a) MoveIt config + 接口定义;(b) 一份 README 解释 6-DoF Cartesian impedance 控制需要什么;(c) 留 TODO + issue 模板给社区。 | 0.5d |
| 4.4 | **CHANGELOG.md + RELEASE_NOTES.md** | semver 规则、按 phase 列出新增功能。 | 0.5d |
| 4.5 | **demo 视频 + README 截图** | 1~2 分钟视频显示 MoveIt planning、teach playback、zero-force 模式手动拖动。 | 1d |
| 4.6 | **v0.1.0 release** | GH release + tarball + `colcon build` 通过 CI;Docker 镜像推到 ghcr.io。 | 1d |

### 6.2 验收门
- `armv7_zero_force_controller` 在实机上手动拖动 7 个关节,基本可以拖动(精度允许 10°/s 漂移)。
- v0.1.0 git tag + GH release 页面包含完整 changelog、安装指南、视频。
- 整套仓库符合 [REP-2003](https://ros.org/reps/rep-2003.html) 包结构规范。

---

## 7. v0.2 及之后路线图(参考,不在 4 周内)

| 模块 | 目标版本 | 预估工期 |
|---|---|---|
| 完整 Cartesian impedance / admittance 控制 | v0.2 | 4 周 |
| 在线动力学辨识 + 自适应重力补偿 | v0.2 | 3 周 |
| RViz panel(示教、点位编辑、轨迹回放) | v0.2 | 2 周 |
| Gazebo / Ignition 完整仿真支持 | v0.3 | 2 周 |
| Isaac Sim 桥接(`~/isaac-sim/` 已装) | v0.3 | 2 周 |
| Web UI(Foxglove Studio 集成) | v0.3 | 2 周 |
| 视觉抓取 pipeline(MoveIt Task Constructor + GraspIt) | v0.4 | 4 周 |
| 多模态遥操(VR / SpaceMouse) | v0.4 | 3 周 |

---

## 8. 包结构(4 周后形态)

```
arm_ws/src/
├── armv7_description/        # URDF + meshes(改名自 armv7)
├── armv7_moveit_config/      # 纯 MoveIt 配置(从 armv7_moveit 拆出)
├── armv7_bringup/            # launch 入口、sim/real 切换
├── armv7_safety/             # E-Stop、软限位
├── armv7_diagnostics/        # 诊断聚合
├── armv7_py/                 # Python API
├── armv7_cpp_api/            # C++ API
├── armv7_safety/             # ✓ W2.2 + 2.3 — workspace bbox + E-Stop
├── armv7_diagnostics/        # ✓ W2.4 — joint diagnostics aggregator
├── armv7_py/                 # ✓ W2.5 — Python facade (Armv7Client)
├── armv7_cpp_api/            # ✓ W2.6 — C++ facade
├── armv7_examples/           # ✓ W2.7 + W3.6 — hello_world / teach_playback / pose_grid / pick_and_place
├── armv7_ee_dummy_gripper/   # ✓ W3.1 + W3.2 — dummy 二指夹爪 xacro + mock ros2_control
├── armv7_tcp/                # ✓ W3.5 — 热加载 TCP TF + /armv7/payload
├── armv7_eyehand/            # ✓ W3.4 — RealSense mount xacro + handeye 静态 TF
├── armv7_ee_dummy_gripper/   # 示范 EE
├── armv7_eyehand/            # 手眼相机模板
├── armv7_dyn_ident/          # 动力学辨识(数据采集 + 离线脚本)
├── armv7_zero_force_controller/  # 重力补偿 free-drive
├── armv7_impedance_moveit/   # starter(README + 接口约定)
└── ethercat_driver_ros2/     # 已存在,不动
```

---

## 9. 风险与取舍

### 9.1 高风险项
| 风险 | 影响 | 缓解 |
|---|---|---|
| dyn_ident 拟合精度不足以让 zero_force 好用 | 4.2 验收门可能滑窗 | 接受 v0.1 用 URDF inertias 即可;v0.2 再用真识别结果 |
| 客户硬件五花八门(夹爪 / FT 各种品牌) | 3.x 写出来的接口不通用 | 只交付 dummy + RobotIQ + ATI 各一个示例,文档明确说"按此模板自改" |
| 1 人 4 周做不完 | 必然 | 按 Phase 优先级保留,**P0(Phase 1+2)是底线**,P1(Phase 3+4)切片交付 |
| 系统 MoveIt 与 ros2_control 的 API 改动 | 跨版本破坏 | 在 CI 锁 humble + ros2_control 4.x 版本 |
| 实时性偶发抖动 | 6.2 验收门不稳 | 文档明确 PREEMPT_RT 是 v0.2 目标,v0.1 用普通内核 + chrt -f 99 |

### 9.2 单人 vs 团队
- **1 人**:Phase 1 + 2 是底线,Phase 3 选做夹爪一项,Phase 4 仅做 4.4/4.5/4.6 发布相关。**不做 4.1/4.2**,只在 RELEASE_NOTES 里写明 v0.2 计划。
- **2~3 人**:Phase 1 + 2 一人,Phase 3 一人,Phase 4 一人。可以全部按计划交付。

---

## 10. 工作清单(可勾选)

完整任务在各 phase 表格里。这里列出每周的"必须完成"红线:

- [x] W1: README + LICENSE + install_deps.sh + Docker + CI build
- [x] W2: E-Stop + Python API + 3 个示例 + colcon test 通过
  - [x] W2.1  joint_limits.yaml(position / velocity / acceleration / effort)
  - [x] W2.2  workspace_bbox 安全检测 + /safety/in_bounds /safety/bbox_state
  - [x] W2.3  E-Stop topic/service + 联动 controller_manager
  - [x] W2.4  joint diagnostics 聚合 -> /diagnostics
  - [x] W2.5  Python API (armv7_py — Armv7Client)
  - [x] W2.6  C++ API (armv7_cpp_api — same 5 methods)
  - [x] W2.7  示例脚本(hello_world / teach_playback / pose_grid)
  - [x] W2.8  单元测试 + colcon test(176 tests, 0 failures)
- [x] W3: 模块化 EE xacro + dummy gripper + pick_and_place 示例
  - [x] W3.1 armv7.urdf.xacro 加 ee_xacro / ee_parent arg
  - [x] W3.2 armv7_ee_dummy_gripper(xacro + mock ros2_control 模板)
  - [x] W3.3 docs/integration/ft_sensor.md(EtherCAT + 网口/topic 两条路径)
  - [x] W3.4 armv7_eyehand(RealSense D435 mount + handeye 静态 TF launch)
  - [x] W3.5 armv7_tcp(/armv7/payload + 热加载 TCP TF)
  - [x] W3.6 pick_and_place 示例(用 dummy gripper)
- [ ] W4: zero_force_controller 骨架 + v0.1.0 release

---

## 11. 沟通与跟踪

- 每周末在 `CHANGELOG.md` 记录本周 done / next。
- GitHub Issues 用 label:`P0` / `P1` / `P2` / `bug` / `docs` / `phase-1`...`phase-4`。
- PR 必须挂 phase label + 关联 issue。
- 任何破坏 API 的改动须在 `MIGRATION.md` 单独记录。

---

> 本计划基于截至 2026-05-18 的现状制定。每周末复盘一次,根据实际进度调整 phase 3、4 的范围。
> Phase 1+2(P0)交付出去后,即便 phase 3+4 滑窗,客户也能在自家环境跑起来。
