# Phase 5(v0.2 alpha)功能测试 — Cartesian 阻抗 + 打螺丝路线

针对 v0.2 第一个交付 `armv7_impedance_moveit/CartesianImpedanceController` 的测试。前提:[testing.md](testing.md) 档位 A/B 通过,[testing_phase4.md](testing_phase4.md) 的重力 + 摩擦校准已完成(`identified_params.yaml` 和 `coulomb_friction` 都填好)。

阻抗控制器复用 Phase 4 那套 KDL 重力 + 库仑/粘滞摩擦补偿,在它之上叠加 Cartesian 弹簧阻尼。所以**先有干净的 Phase 4 校准,再上 Phase 5**。

| 节 | 内容 |
|---|---|
| 5.1 | 干跑加载 + 弹簧律数学校验(fake) |
| 5.2 | 实机阻抗 first run + 调参 |
| 5.3 | 打螺丝路线图测试目标(占位,实施靠 TODO.md) |

---

## 5.0 前置清理

同 [testing.md § 0](testing.md)。

---

## 5.1 — 干跑数学校验

确认控制器加载、弹簧律 `W = K·e` 在干跑模式下数值正确。**没有物理引擎,但 wrench / torque 计算路径完整**。

### 启动 + claim 检查

```bash
ros2 launch armv7_impedance_moveit impedance.launch.py use_fake_hardware:=true use_rt:=false
```

**通过标准** — 日志依次出现:
```
applied identified mass+CoM to 7/7 links from ~/arm_ws/src/armv7_dyn_ident/config/identified_params.yaml
configured: 7 joints, chain base_link -> link7, K_trans=[200, 200, 200] K_rot=[20.0, 20.0, 20.0], disabled at start
activated; target seeded to current pose; ramp-in 2.0s
Configured and activated cartesian_impedance_controller
```

```bash
ros2 control list_controllers
# joint_state_broadcaster ... active
# cartesian_impedance_controller ... active

ros2 control list_hardware_interfaces | grep -E "effort.*claimed"
# joint1..7/effort [available] [claimed]
```

### 默认状态 — disabled 时全 0

```bash
ros2 topic echo /cartesian_impedance_controller/commanded_torque --once
# data: [0, 0, 0, 0, 0, 0, 0]
```

### Enable 后 target = current → wrench = 0

```bash
ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: true}"
sleep 3

ros2 topic echo /cartesian_impedance_controller/pose_error --once
# data: [0, 0, 0, 0, 0, 0]   ← target seeded to current pose

ros2 topic echo /cartesian_impedance_controller/commanded_wrench --once
# data: [0, 0, 0, 0, 0, 0]   ← spring force zero (no impedance contribution)

ros2 topic echo /cartesian_impedance_controller/commanded_torque --once
# data ≈ G(q=0) * 0.9         ← pure gravity comp; joint2 主要项
```

### 关键 — 弹簧律验证

```bash
# 推一个 z = 0.45 的目标到 base_link 系。link7 在 q=0 时大约在 z ≈ 0.55,
# 所以期望 e_z ≈ -0.10(被 max_pose_error[2] 截断)。
ros2 topic pub --once /cartesian_impedance_controller/target_pose \
    geometry_msgs/msg/PoseStamped \
    "{header:{frame_id: base_link}, pose:{position:{x: 0.0, y: 0.0, z: 0.45},
      orientation:{w: 1.0}}}"
sleep 1

ros2 topic echo /cartesian_impedance_controller/pose_error --once
# data: [小, 小, -0.10, ~0, ~0, ~0]   ← Z 误差被 clip 到 -0.10

ros2 topic echo /cartesian_impedance_controller/commanded_wrench --once
# data: [小, 小, -20.0, ~0, ~0, ~0]   ← K_z * e_z = 200 * (-0.10) = -20 N
```

**通过标准**:`commanded_wrench[z]` ≈ `stiffness[z] * pose_error[z]`(±0.5 N)。XY 分量小但非零(当前 TCP 不严格在 base 原点正上方),旋转分量 ≈ 1e-5 rad 级别(数值误差)。

> 这一步严格证明了:**FK → pose error → spring law → wrench → Jᵀ → joint torque** 这条计算链都对了。换实机也是同一份代码。

### Disable 后立即归零

```bash
ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: false}"
sleep 1
ros2 topic echo /cartesian_impedance_controller/commanded_torque --once
# data: [0, 0, 0, 0, 0, 0, 0]
```

### 外力估计管路校验(残差法)

```bash
# enable(ramp 完成后会自动 tare)
ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: true}"
sleep 4

# external_wrench topic 存在并发布
ros2 topic echo /cartesian_impedance_controller/external_wrench --once
# 期望:force/torque 各分量 ~0(q=0 是奇异点,阻尼伪逆把估计压到 ~0,正常)

# 手动 tare 服务
ros2 service call /cartesian_impedance_controller/tare_external_wrench std_srvs/srv/Trigger
# 期望:success=True;之后 external_wrench 精确归 0
```

**通过标准**:topic 发布;tare 后归零。

> ⚠️ **fake 模式只能验证管路**(残差→伪逆→滤波→tare→发布),**不能验证真实外力物理** —— mock 硬件不做动力学仿真,且 q=0 是奇异点会把估计阻尼掉。真实外力检测只能在实机、非奇异姿态下做(§5.2 之后)。

---

## 5.2 — 实机 first run + 调参

### Pre-flight 清单(从 fake 到实机的第一次必看)

**A. 硬件 / 状态确认**
```bash
ethercat slaves        # 7 slaves 全部 OP,任何 SAFEOP+E / INIT 都不能开
groups | grep -E "realtime|ethercat"   # 都在
```
- 物理急停按钮就位、清场、最好旁边有第二个人。

**B. 配置 sanity check**(打开 `src/armv7_impedance_moveit/config/impedance.yaml`)
- `gravity_scale: 0.9` —— **别动**(略沉总比上飘安全)。
- `coulomb_friction: [2.2, 2.21, 0.48, 0.41, 0.11, 0.16, 0.14]` —— 跟 free-drive 调好的一致。
- `stiffness: [200, 200, 200, 20, 20, 20]` —— **首跑必须保持**。
- `max_pose_error: [0.10, 0.10, 0.10, 0.50, 0.50, 0.50]` —— wrench 上限的第一道闸。
- `wrench_limit: [60, 60, 60, 10, 10, 10]` —— 硬天花板。
- `max_torque: [30, 30, 12, 12, 5.5, 5.5, 5.5]` —— ≤ URDF effort 限位。
- `enable_at_start: false` —— **必须 false**。

**C. 不要在 q=0 直立姿态下做阻抗测试**(关键!)

机械臂在 q=0 直立(joint1/3/5/7 滚转轴**全部沿世界 Z 共线**)就是 7-DoF 标准的奇异点 —— Jacobian 旋转子空间秩亏,弹簧把臂往那个姿态拉,关节在 null-space 里乱晃 → 严重抖动。**别从 q=0 直接测阻抗。**

正确做法:**先用位置模式把臂规划到一个非奇异姿态,再切阻抗模式**。
```bash
# 1) 起位置模式 + MoveIt
ros2 launch armv7_bringup arm.launch.py

# 2) RViz 里规划到一个"弯曲"姿态,推荐:
#    q = [0, 0.5, 0, 0.8, 0, 0.5, 0]   (rad)
#    肩 30° / 肘 45° / 腕 30°,joint2/4/6 全部非零,远离奇异

# 3) 关掉 arm.launch.py
# 4) 起阻抗模式 —— 控制器会自动把这个非奇异姿态当 target seed
ros2 launch armv7_impedance_moveit impedance.launch.py
```

**D. 第一次启动 → 观察 → enable**(严格按序)
```bash
# 终端 1
ros2 launch armv7_impedance_moveit impedance.launch.py
# 等到 "Configured and activated cartesian_impedance_controller" + 2 秒 ramp 完成

# 终端 2:盯三条 topic(disabled 应该全 0)
ros2 topic echo /cartesian_impedance_controller/commanded_torque
ros2 topic echo /cartesian_impedance_controller/pose_error
ros2 topic echo /cartesian_impedance_controller/commanded_wrench
```
**Disabled 状态通过标准**:三条全 0(就是 NumPy 极小数级噪声)。

```bash
# 终端 3:**不发任何 target!** 只 enable
ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: true}"
```
**关键**:控制器在每次 `disabled → enabled` 转换时,**会自动**把当前 TCP 抓拍为新 target —— 就算 disabled 期间臂下垂或被你手动挪过,enable 那一瞬间 target = 当前位置,弹簧力 ≈ 0,**不会有突跳**。

**Enabled 通过标准**:
- `commanded_wrench` ≈ 全 0(数值噪声)。
- `commanded_torque` ≈ 重力补偿值(joint2 大约 0.69 Nm,跟你 free-drive 在 q=0 时输出一致 —— 同一份参数)。
- 机械臂**保持不动**。任何漂移、抖动、突跳 → 立刻:
  ```bash
  ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: false}"
  ```
  然后物理急停 → 排查。

**E. 第一推力**(在 enable 状态下,手指尖**轻推** TCP 5 cm)
- 默认 K=200 N/m → wrench 反力 ≈ 10 N。能明显感到弹簧回弹。
- 松手 → 回到 seed(摩擦死区可能差几 mm,正常)。

**F. 不要做的事**
- enable 前**绝对不要**发任何 `target_pose`(一旦 enable 立刻产生 wrench)。
- 不要离开急停范围。
- 不要从默认 K=200 直跳到 1000。一档档:200 → 500 → 1000。

### Step A — Seed 测试(确认控制器不会"无中生有"出力)

```bash
ros2 launch armv7_impedance_moveit impedance.launch.py
# 等到 cartesian_impedance_controller active

# enable 之前先发一个 target = 当前 TCP 位置(防 default seed 漂移):
ros2 run tf2_ros tf2_echo base_link link7 --once   # 抄 translation/rotation

# 用上面读到的位姿组装 target_pose 发布(或直接接受 on_activate 的 seed)

ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: true}"
```

**通过标准**:
- 启动后 2 秒内 ramp 完成。
- 臂稳定不动。轻推某轴,能感觉到弹簧回复力(默认 K_trans=200 N/m,推 5mm 应感觉到约 1 N 回复)。
- `pose_error` 在松手时收敛回 0。

**不通过 → 立刻 disable**:
```bash
ros2 service call /cartesian_impedance_controller/enable std_srvs/srv/SetBool "{data: false}"
```
然后检查 `gravity_scale` / `coulomb_friction`(可能 Phase 4 校准的值不对)。

### Step B — Step response

发一个 X 方向 +5 cm 的 target:
```bash
ros2 topic pub --once /cartesian_impedance_controller/target_pose ...  # X += 0.05
```
**通过标准**:臂朝 X 方向移动 ≤ 50 mm(K_x = 200 N/m,wrench = 10 N,实际位移由弹簧 + 实机阻尼决定),无振荡。

> 这一步是从"控制器在跑"到"控制器能跟踪"的分界线。能到这一步,后面就是调参的事。

### Step B' — 外力估计实测(残差法,非奇异姿态)

**前提**:已在**弯曲(非奇异)姿态**(如 §C 推荐的 `[0, 0.5, 0, 0.8, 0, 0.5, 0]`),enable 完成。

```bash
# 开 dashboard,看"外力估计"那行 + 底部"外力估计曲线"
ros2 run armv7_impedance_moveit tuning_dashboard

# 1) 静止时:点"外力置零(tare)",F/M 各分量归 0
# 2) 用手指尖往 +Z(下压)推 TCP,观察 dashboard:
#    - "外力估计"行 F_z 出现一个负值(下压方向)
#    - 底部曲线 Fz 曲线随手的力度起伏
# 3) 松手 → F_z 回到 0 附近
# 4) 往 X / Y 推,对应 F_x / F_y 响应
```

**通过标准**:
- tare 后静止读数 ~0。
- 手推方向与 F_ext 符号一致(下压→F_z 负,或按你的 base 系约定核对一次)。
- 力度大小与读数单调相关(用力推数值更大)。
- 松手归零。

> 这是 v1 **gravity-only** 残差,**低速最准**。快速猛推时估计会噪、滞后(低通 5 Hz)。打螺丝的 seated 检测是"压住不动",正好是低速高精度区。
> 若 F_ext 完全不动:确认在**非奇异**姿态(q=0 会被阻尼伪逆压掉);确认 effort **状态**接口有数(`ros2 topic echo /joint_states` 的 effort 字段非空)。

> 📌 **顺带修复**:激活 ramp-in 现在从 **enable 那一刻**起算(以前错误地从控制器 spawn 起算,导致你 enable 时 ramp 早已结束、力矩 0→满瞬间阶跃)。现在 enable 后力矩 2 秒线性升满,更安全。

### Step C' — 抖动诊断(如果 enable 后越抖越剧烈)

**症状**:enable 后机械臂不是简单微抖,而是**振荡越来越剧烈**,直到力矩钳位才停在大幅极限环。

**原因**:摩擦补偿在 q̇ → 0 时是**正反馈**。`f_coulomb = coulomb · tanh(q̇/eps)` 在 |q̇| << eps 时近似为 `(coulomb/eps) · q̇`,对你的 joint1 等价于 +44 Nm·s/rad 的正阻尼,**远超**我们的 joint 阻尼 0.5。free-drive 不出问题是因为没有 Cartesian 弹簧;impedance 一上电就闭环 → 发散。

**修法已经做进控制器了**:`velocity_deadband`(默认 0.05 rad/s)让 |q̇| < dz 时 `f_coulomb` 严格 = 0,正反馈直接砍掉。

**实机诊断流程**:
1. enable 后如果抖,**先开 dashboard**,找最右下"Friction comp"面板。
2. 点 **"Disable friction comp"** —— 一键把 `scale → 0`、`dz → 0.3`。
   - **抖立刻停** → 确诊就是摩擦补偿正反馈,继续按下面调。
   - 还抖 → 不是这个问题,告诉我,我看下一步。
3. 把 `scale` 滑回 1.0(`dz` 保持 0.05 默认),应该已经稳了。
4. 推一下 TCP,感觉一下手感。如果"启动费力"(死区内 free-drive 手感会比之前略沉),把 `dz` 往下调到 0.03,感觉够了停。

### Tuning dashboard(可选,但首跑后强烈推荐)

```bash
ros2 run armv7_impedance_moveit tuning_dashboard
```
窗口分四块:
- **大色块 ENABLE / DISABLE 按钮**(红=off / 绿=on)。
- **Target offset 滑块**(X/Y/Z,±10 cm 相对 seed pose)。"Re-seed to current TCP" 重新捕获当前 TF。
- **Stiffness / damping_ratio 滑块** —— 改完立刻通过 `set_parameters` 服务下发到控制器。**不用重启**就生效(下一个控制周期就更新)。"Screw preset" 一键切到 `K=[100, 100, 2000, 50, 50, 5]` + 配套 damping。
- **Live readouts** pose_error / wrench / torque,~10 Hz 刷新。

**实测验证**(fake 模式):K=200 → wrench[0] = 11.13 N(5 cm 偏移);滑块切 K_x=400 → 下一周期 wrench[0] = 22.27 N,完全没重启,数学准确翻倍。负值刚度被原子拒绝(`successful=False, reason='stiffness entries must be >= 0'`),wrench 保持原值。

> 已知小坑:`ros2 param set /cartesian_impedance_controller stiffness ...` 在本 Humble 版本会报 "Node not found" —— 是 ros2 CLI 守护进程对 controller lifecycle node 的可见性问题,不是控制器的问题。Dashboard 直接调底层 `set_parameters` 服务,绕开这个问题。

### Step C — 调 stiffness

参考 `armv7_impedance_moveit/README.md` §5。先 200 → 500 → 1000 N/m 一档档加,每档发 step response 看超调和稳态误差。**`K_trans` 超过 5000 之前,如果 200 Hz 控制频率下开始抖**:把对应 `damping_ratio` 推到 1.0,再考虑硬件升级控制频率。

### Step D — 各向异性测试(打螺丝姿态)

```yaml
# 临时改 impedance.yaml,把 XY 设软、Z 设硬:
stiffness: [80.0, 80.0, 2000.0, 50.0, 50.0, 5.0]
```
重启 impedance,enable,手扶住 TCP:
- XY 方向应该明显"软",1 cm 偏移大约 0.8 N 阻力。
- Z 方向应该"硬",1 mm 偏移就是 2 N。
- 绕 Z 旋转应该几乎自由(K_rz = 5 Nm/rad)。

**这就是打螺丝的姿态**:工件未对正时 XY 让一让,Z 顶在螺丝头上,关节 7 自由转动驱动。

---

## 5.3 — 打螺丝路线图占位

完整 primitive 需要:

1. **外力感知** —— F/T 传感器接 `/wrench`,或关节力矩残差估计 `(Jᵀ)⁺(τ_meas − G − τ_friction)`(参数已在控制器里齐全,加一个估计器节点)。详见 [TODO.md](../src/armv7_impedance_moveit/TODO.md)。
2. **状态机节点** `screw_primitive`(`armv7_examples` 下加 .py 即可),分 APPROACH / SOFT-XY / ENGAGE / DRIVE / RETRACT 五步,DRIVE 状态监控 `F_z > F_seat_threshold` 或 `τ_z > τ_seat_threshold` 触发终止。
3. **stiffness 切换**:目前每次切换刚度需要重启控制器。`SetImpedance` service msg 写好 + 在线参数热更接到 controller 里之后,primitive 可以在不重启的情况下从 APPROACH 高刚度切换到 SOFT-XY 模式。

测试目标(`docs/testing_phase5.md` 后续会加):
- 干跑:状态机能从 APPROACH 一路走到 RETRACT,中间 stiffness 切换、joint7 旋转、F_z 阈值触发,日志清晰。
- 实机:真螺丝 + 螺孔 / 木板靶,M3-M5 各跑 10 次,成功率 ≥ 70%。

---

## 测试报告模板

```
日期:        2026-06-17
测试人:      <名字>
arm_ws git:  <git rev-parse --short HEAD>
硬件:        [ ] fake   [ ] CST 实机

5.1 干跑加载 + 弹簧律 + 外力管路   [ ✓ / ✗ ]
5.2.A Seed 测试                  [ ✓ / ✗ / N/A 无硬件 ]
5.2.B Step response              [ ✓ / ✗ / N/A ]
5.2.B' 外力估计实测              [ ✓ / ✗ / N/A ]
5.2.C 调 stiffness 上限          [ ✓ / ✗ / N/A ]  最高稳定 K_trans: ___ N/m
5.2.D 打螺丝姿态各向异性          [ ✓ / ✗ / N/A ]

失败项:
  - <节>: <现象> → <对应 troubleshooting.md 条目 or TODO.md 子任务>
```

---

## 全 ✓ 之后

v0.2 alpha 的 Cartesian 阻抗就绪。下一里程碑:
- 关节力矩残差外力估计 → 螺丝 seated 检测。
- `screw_primitive` 状态机节点。
- `docs/testing_phase5.md` 加入 primitive 的实测验收。

完整路线图见 [`src/armv7_impedance_moveit/TODO.md`](../src/armv7_impedance_moveit/TODO.md)。
