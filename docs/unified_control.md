# 统一控制:运行时 位控 ↔ 力控 + rqt 面板

把机械臂的高级功能集成到一起:一个 bringup 同时加载 **位控(MoveIt / JTC)** 和
**力控(Cartesian 阻抗)**,运行时**无需重启**切换;一个 **rqt 面板**统一做模式切换 +
阻抗调参 + 监控。

```
                ┌─────────────────── rqt 面板 (armv7_rqt) ───────────────────┐
                │  [切到位控] [切到力控]   [启用阻抗] [外力置零]              │
                │  刚度 K_*  阻尼 ζ_*  摩擦 dz/scale  预设                    │
                │  位姿误差 / 力旋量 / 外力估计 实时读数                       │
                └───────────┬───────────────────────────┬───────────────────┘
                            │ 服务/参数                  │ 订阅
            ┌───────────────▼──────────┐   ┌────────────▼─────────────┐
            │ mode_switcher            │   │ cartesian_impedance_     │
            │ (armv7_mode_manager)     │   │ controller (enable/tare/ │
            │ ~/to_position ~/to_force │   │ stiffness/外力估计...)   │
            └───────────────┬──────────┘   └──────────────────────────┘
                            │ switch_controller (+ 实机 drive mode)
            ┌───────────────▼───────────────────────────────────────────┐
            │ 一个 controller_manager / 一个 ethercat 硬件接口            │
            │  jsb + plan_group_controller(JTC) + cartesian_impedance    │
            │  + mode_controller(实机, 写 0x6060)                        │
            └────────────────────────────────────────────────────────────┘
```

---

## 1. 核心概念:不是"两个驱动器",是一个

底层只有**一个** `ethercat_driver` 硬件接口,启动时加载一次。位控和力控的区别是:
1. **哪个控制器 active**(JTC 占 position 接口 / 阻抗占 effort 接口)—— 运行时可切。
2. **驱动器 CiA-402 模式**(CSP 8 位置 / CST 10 力矩)—— 真正的拦路虎。

`unified.ros2_control.xacro` 让一个硬件接口**同时暴露** position + velocity + effort +
`mode_of_operation` 命令接口。打了补丁的 cia402 插件把 `mode_of_operation` 命令接口的值
路由到 0x6060,于是 CSP 和 CST 在同一硬件接口上共存、运行时切换。

| 层 | 谁负责 | 运行时可切? |
|---|---|---|
| 控制器 active/inactive | `mode_switcher` → `switch_controller` | ✅ |
| 驱动器模式 0x6060 | `mode_controller`(forward_command_controller)写 8/10 | ✅(实机) |

---

## 2. 启动

### 干跑(fake,mock 同时接受 position 和 effort,切换即可演示)
```bash
ros2 launch armv7_bringup unified.launch.py use_fake_hardware:=true use_rt:=false
```
启动后:`joint_state_broadcaster` + `cartesian_impedance_controller`(**active**,但
内部 disabled = 0 力矩)+ `plan_group_controller`(inactive)+ `mode_switcher`。
**默认 force(力控)模式**。

> ⚠️ **为什么默认 force 而不是 position**:JTC(位控)在激活那一刻命令缓冲区未初始化,
> 会把机械臂**快速拽回 0 点** —— 上电瞬间的运动隐患。阻抗控制器默认 disabled(0 力矩),
> 启动后机械臂**原地保持不动**,安全。要从位控启动:`start_mode:=position`(知道风险再用)。

### 实机(CSP↔CST 运行时切换)
```bash
ros2 launch armv7_bringup unified.launch.py
```
实机额外加载 `mode_controller`(always active,持有 mode_of_operation 接口),且
`mode_switcher` 的 `manage_drive_mode` 自动为 true。

> 实机需要打了补丁的 `ethercat_generic_cia402_drive`(支持
> `command_interface/mode_of_operation`)。已在本仓库内。

### rqt 面板
```bash
rqt   # Plugins -> Robot Tools -> armv7 Panel
# 或直接:
rqt --standalone armv7_rqt
```

---

## 3. 模式切换

### 命令行
```bash
ros2 service call /mode_switcher/to_force    std_srvs/srv/Trigger
ros2 service call /mode_switcher/to_position std_srvs/srv/Trigger
ros2 topic echo  /mode_switcher/current_mode    # latched: position / force
```

### 面板
点 `切到位控` / `切到力控`。顶部"当前: position/force"色块实时显示。

### 切换时序(干跑)
纯控制器切换,原子完成:JTC ↔ 阻抗,position/effort 接口干净交接。已验证。

### 切换时序 —— **不掉落**(切力控 = 同时启用重力补偿)

`to_force` 现在把"启用阻抗(重力补偿)"和"切 CST"**编排在一起**,机械臂**全程被托住,没有 0 力矩窗口**:

```
to_force(位→力):
  1. switch_controller(阻抗 active, JTC inactive)   ← 仍 CSP,驱动器保持位置
  2. 启用阻抗(~/enable true)                        ← 重力补偿开始计算/ramp;CSP 仍忽略 effort
  3. 等 force_engage_delay(默认 2.5s,> ramp_in)    ← ramp 跑完,CSP 一直在托
  4. 切驱动器 CST(10)                                ← 此刻 effort 已是"满幅重力补偿" → 无缝接管,不掉
```

```
to_position(力→位)—— 不掉、不抖(无回零冲击):
  1. 把 JTC 和阻抗"同时"激活        ← JTC 占 position 接口、阻抗占 effort,互不冲突
                                       JTC 读当前位姿、写 0x607a = 当前位姿;CST 下被忽略,
                                       阻抗 effort 仍托着臂
  2. 切驱动器 CSP(8)                ← 驱动器按 0x607a(= 刚写的当前位姿)保持 → 不抖
  3. 停用阻抗(JTC 继续持位)         ← 下次 to_force 是干净的 enable 转换(重新 seed + tare)
```

> ⚠️ **为什么力→位会"剧烈移动一下"(已修)**:力控/CST 下驱动器的位置命令 0x607a 是**陈旧的**
> (停在离开位控时的位姿),但自由拖动期间臂已经被推到别处。如果直接切 CSP,驱动器会**猛地拽回**
> 那个陈旧目标。修法:**先让 JTC 与阻抗同时激活**,JTC 把 0x607a 写成当前位姿,**再**切 CSP →
> 驱动器原地保持,不抖。

**关键**:to_force 第 4 步切 CST 时重力补偿力矩已满幅(在 CSP 保持下提前 ramp 好),进 CST 即被接住
**不跌落**;to_position 切 CSP 前 0x607a 已被 JTC 写成当前位姿,**不回零冲击**。两个方向均已 fake 验证。

参数:`auto_enable_force`(默认 true)、`force_engage_delay`(默认 2.5s,实机按 ramp + 握手稳定性调)。

> ⚠️ 实机首次仍建议手放急停。`force_engage_delay` 要 ≥ 阻抗的 `ramp_in_time`(默认 2s),
> 否则切 CST 时 ramp 还没满、托不住会下沉一点。

---

## 4. 力控的两个子状态:0重力 / 阻抗弹簧

**力控 = 0重力(自由拖动)是默认**,阻抗弹簧单独开:

| 子状态 | 刚度 K | 行为 |
|---|---|---|
| **0重力 / 自由拖动**(默认) | **0** | 只补重力 + 摩擦。机械臂"失重",随手推到哪停哪。**没有弹簧 → 切换时不会被弹簧冲一下。** |
| **阻抗弹簧** | > 0(配置值) | 重力补偿 + Cartesian 弹簧。推开后回弹到目标位姿。 |

切到力控(`to_force`)**自动设 K=0**(任何客户端调用都安全),所以来回切换不抖。要弹簧时在面板勾
**"阻抗弹簧"**(会先重新锚定目标到当前位姿,弹簧从静止开始,无冲击),取消勾选回到 0重力。

## 5. 面板各区

| 区 | 控件 | 作用 |
|---|---|---|
| 控制模式 | 切到位控 / 切到力控 + 当前模式 | 调 `mode_switcher`(切力控=0重力) |
| 力控:重力补偿/弹簧 | 启用/停用、**阻抗弹簧开关**、外力置零(tare) | `~/enable`、`stiffness` 0↔配置值、`~/tare_external_wrench` |
| 目标位姿偏移 | **坐标系下拉**(base_link 世界系 / 末端 tool 工具系)+ dX/dY/dZ + dRoll/dPitch/dYaw + 重新锚定 | 发布 `~/target_pose`(**仅阻抗弹簧开时有效**)。world 系:平移沿世界轴、旋转在世界系;tool 系:平移沿工具轴、旋转在工具系 |
| 刚度 K | 6 个 spinbox(弹簧开时生效) | 弹簧的刚度值 |
| 阻尼比 ζ | 6 个 spinbox(实时) | 改 `damping_ratio` |
| 摩擦补偿 | dz / scale / 关闭按钮 | 改 `velocity_deadband`/`friction_compensation_scale` |
| 零空间控制 | 阻尼(防漂移)/ 刚度(锁姿态) | 改 `nullspace_damping`/`nullspace_stiffness`,治"切换时关节突变" |
| 刚度预设 | 保守起点 / 稳定基线 / 打螺丝 | 一键套 K+ζ **并打开弹簧** |
| 实时监控 | 位姿误差 / 力旋量 / 外力估计 | 订阅控制器 debug topic,~10 Hz |

> Tkinter 版 `tuning_dashboard` 仍可用(带曲线图);rqt 面板是集成进 RViz/rqt 工作流的版本。

---

## 6. 安全须知(实机)

0. **启动默认 force 模式**:阻抗 active 但 disabled(0 力矩),驱动器起始 CSP(8)保持位置,
   机械臂原地不动。**不会**像位控那样上电拽回 0 点。需要 MoveIt 规划时再 `切到位控`。
1. **切力控 = 同时启用重力补偿,不掉落**:`to_force` 自动启用阻抗并在重力补偿满幅后才切 CST,
   全程托住(见 §3)。`auto_enable_force=false` 可关掉这个自动启用(则恢复"切力控后手动启用"的
   旧行为,有掉落风险)。
2. **从 q=0 直立姿态别测力控**:那是奇异点,阻抗会抖(见 [testing_phase5.md](testing_phase5.md))。
2.5. **切换时部分关节突变(已缓解)**:7 自由度冗余的零空间不受控时,力控期间关节会在 TCP 不动
   的情况下**慢慢漂移**重构;切回位控 JTC 锁住漂移后的构型 → 看着像关节突变。零空间**阻尼**
   (默认 `nullspace_damping=1.0`)抑制这种漂移。还漂就加大阻尼;想把肘部**锁死**在切力控那一刻的
   姿态,加 `nullspace_stiffness`(>0)。这一项只作用在 J 的零空间,**不影响末端笛卡尔任务**。
3. **手放急停**,实机首次切换务必有人在旁。
4. `mode_switch_delay`(默认 0.3 s)等驱动器模式生效再切控制器;实机按握手稳定性调。

---

## 7. 相关包

| 包 | 角色 |
|---|---|
| [armv7_bringup](../src/armv7_bringup) | `unified.launch.py`、`armv7_unified.ros2_control.xacro`、`EUPH##_unified_config.yaml` |
| [armv7_mode_manager](../src/armv7_mode_manager) | `mode_switcher` 协调器 |
| [armv7_impedance_moveit](../src/armv7_impedance_moveit) | 阻抗控制器 + Tkinter dashboard |
| [armv7_rqt](../src/armv7_rqt) | rqt 统一面板 |
| `ethercat_generic_cia402_drive` | 已打补丁:`command_interface/mode_of_operation` 运行时模式切换 |

---

## 8. 现状

| 能力 | 状态 |
|---|---|
| 干跑运行时位↔力切换 | ✅ 已验证 |
| rqt 面板模式切换 + 调参 + 监控 | ✅ 端到端已验证(fake) |
| cia402 插件 mode 命令接口 | ✅ 编译通过,逻辑就绪 |
| 实机 CSP↔CST 握手 | ⏳ **需实机联调**(时序/安全在硬件上定) |
