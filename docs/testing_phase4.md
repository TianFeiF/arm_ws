# Phase 4 功能测试文档 — 动力学辨识与重力补偿

针对 Phase 4(V4.1–V4.2)交付的功能测试手册。**前提:[docs/testing.md](testing.md) 的档位 A/B 已通过**(核心 arm + 控制器工作正常)。

本文档覆盖:

| 节 | 对应 | 内容 |
|---|---|---|
| 4.1 | V4.1 | `armv7_dyn_ident`:重力模型、静态位姿数据采集、离线参数辨识 |
| 4.2 | V4.2 | `armv7_zero_force_controller`:重力补偿 / 自由拖动控制器 |

**两条主线:**
- **辨识(V4.1)在现有的位置模式下进行**,只*读取* effort 状态(0x6077),不命令力矩 —— 安全,可在任何时候做。
- **重力补偿(V4.2)需要驱动器进入力矩模式(CiA-402 CST / mode 10)**。这是真实机械臂上**唯一会主动命令力矩**的部分,有跌落风险,务必按 §4.2.2 的安全流程操作。

干跑(`use_fake_hardware:=true`)能验证**代码链路**(控制器加载、接口 claim、重力力矩计算),但 mock 硬件**没有物理仿真**,力矩不会让模型臂运动、采集到的 effort 全为 0。真实的辨识与拖动效果**只能在实机上验证**。

---

## 0. 前置清理(每次必做)

与 [testing.md § 0](testing.md) 相同。简版:
```bash
for i in 1 2 3; do
  for p in $(pgrep -f "ros2 launch armv7|ros2_control_node|move_group|robot_state_publisher|spawner|rviz2|armv7_dyn|free_drive"); do
    kill -KILL "$p" 2>/dev/null
  done; sleep 1
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ros2 daemon stop && sleep 1 && ros2 daemon start

cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4.0 自动化测试

```bash
colcon test --packages-select armv7_dyn_ident
colcon test-result --verbose | grep armv7_dyn_ident
```
**通过标准**:`9 tests, 0 failures`。其中关键三类:
- `test_torque_matches_energy_gradient`:符号重力力矩 G(q)=∂U/∂q 与势能数值微分一致(误差 < 1e-5)。
- `test_regressor_is_linear_in_params`:回归量满足 G(q)=Y(q)·φ。
- `test_identify_improves_on_urdf_prior`:合成数据下,辨识把保留位姿的重力误差压到 URDF 先验的 30% 以下。

C++ 控制器编译:
```bash
colcon build --packages-select armv7_zero_force_controller
# 通过标准:Finished,无 error(realtime_publisher 的 deprecation note 已消除)
```

---

## 4.1 — 动力学辨识(V4.1)

### 4.1.1 重力模型自检(无需启动)

```bash
python3 - <<'PY'
import numpy as np
from armv7_dyn_ident.gravity_model import build_from_urdf_file
from ament_index_python.packages import get_package_share_directory
urdf = f'{get_package_share_directory("armv7_description")}/urdf/armv7.urdf'
gm = build_from_urdf_file(urdf)
print("joints:", gm.joint_names)
print("G(q=0) =", np.round(gm.torque(np.zeros(7), gm.urdf_params()), 6))
PY
```
**通过标准**:打印 7 个关节名;`G(q=0)` 中 joint2 ≈ 0.0489 Nm、joint4 ≈ -0.00057 Nm,其余 ≈ 0。(模型构建首次约 15–20 s,正常。)

### 4.1.2 静态位姿数据采集(`collect`)

> 采集**在位置模式下**进行,复用 `arm.launch.py` 的 `plan_group_controller`。

**终端 A** 起臂(实机去掉 `use_fake_hardware`):
```bash
ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true use_rt:=false use_rviz:=false
```

**终端 B** 跑采集(干跑用小批量快速验证):
```bash
ros2 run armv7_dyn_ident collect --ros-args \
  -p n_poses:=3 -p move_time:=2.0 -p settle_time:=0.5 \
  -p samples_per_pose:=10 -p output_csv:=/tmp/armv7_gravity.csv
```
**通过标准**(干跑):日志依次出现 `[1/3] moving to ...` → `tau = ...` → `wrote 3 samples`;`/tmp/armv7_gravity.csv` 有表头 `q1..q7,tau1..tau7` + 3 行数据。**fake 模式下 tau 全为 0(无物理),属预期**。

**实机正式采集**(`use_fake_hardware` 去掉,先确认所有伺服 OP):
```bash
ros2 launch armv7_dyn_ident collect.launch.py    # 默认 60 个位姿,~5 min
```
> ⚠️ 自动生成的随机位姿**未做碰撞检查**。第一次跑务必盯着、手放急停;或用 `-p poses:="[q1..q7, q1..q7, ...]"` 传入自己确认过的安全位姿(行优先展开)。

### 4.1.3 离线辨识(`identify`)

合成自检(不需要真数据,验证脚本本身):
```bash
python3 - <<'PY'
import numpy as np, csv
from armv7_dyn_ident.gravity_model import build_from_urdf_file
from armv7_dyn_ident import identify as I
from ament_index_python.packages import get_package_share_directory
urdf = f'{get_package_share_directory("armv7_description")}/urdf/armv7.urdf'
gm = build_from_urdf_file(urdf); n = gm.n
rng = np.random.default_rng(1); phi0 = gm.urdf_params(); phi = phi0.copy()
for i in range(n): phi[4*i+1:4*i+4] += rng.normal(0, 0.01, 3)
with open('/tmp/syn.csv','w',newline='') as f:
    w=csv.writer(f); w.writerow([f'q{i+1}' for i in range(n)]+[f'tau{i+1}' for i in range(n)])
    for _ in range(60):
        q=rng.uniform(-1.4,1.4,n); w.writerow(list(q)+list(gm.torque(q,phi)+rng.normal(0,0.02,n)))
print("synthetic CSV /tmp/syn.csv ready")
PY

ros2 run armv7_dyn_ident identify --ros-args -p csv:=/tmp/syn.csv -p out:=/tmp/identified_params.yaml
```
**通过标准**:打印每关节 `urdf -> identified` 的力矩残差,identified 明显更小;写出 `/tmp/identified_params.yaml`(含 `identified_dynamics.links` 的 mass+com、`meta` 的 rms 与样本数)。

实机辨识就是把 `csv:=` 换成 §4.1.2 采集到的真实 CSV。若某关节 effort 读数符号与模型相反,加 `-p joint_sign:="[1,1,-1,1,1,1,1]"` 翻转。

---

## 4.2 — 重力补偿 / 自由拖动(V4.2)

### 4.2.1 干跑加载验证(fake hardware)

验证控制器能加载、激活、claim effort、按使能开关输出**正确的**重力力矩。

**终端 A**:
```bash
ros2 launch armv7_zero_force_controller free_drive.launch.py use_fake_hardware:=true use_rt:=false
```

**终端 B**:
```bash
# 1) 两个控制器都 active
ros2 control list_controllers
#   joint_state_broadcaster         ... active
#   gravity_compensation_controller ... active

# 2) 7 个 effort 命令接口被 claim
ros2 control list_hardware_interfaces | grep "effort"
#   joint1/effort [available] [claimed]  ... (×7)

# 3) 默认 DISABLED -> 力矩为 0
ros2 topic echo /gravity_compensation_controller/gravity_torque --once
#   data: [0,0,0,0,0,0,0]

# 4) 使能
ros2 service call /gravity_compensation_controller/enable std_srvs/srv/SetBool "{data: true}"

# 5) ENABLED -> 输出 G(q=0)
ros2 topic echo /gravity_compensation_controller/gravity_torque --once
#   data: [~0, 0.04894, ~0, -0.00057, ~0, -1.6e-5, ~0]
```
**通过标准**:第 5 步的力矩与 §4.1.1 的 `G(q=0)` 一致。这条等式说明 **C++ 控制器(KDL)与 Python 辨识模型用的是同一套重力物理与符号约定**(实测两者在 q=0 处吻合到 4e-16,机器精度)。

> 用辨识结果跑:`free_drive.launch.py ... identified_params:=/abs/path/identified_params.yaml`,日志会打印 `applied identified mass+CoM to 7/7 links`。

### 4.2.2 实机自由拖动流程(危险,严格按序)

**硬件前提**
- 伺服支持 CiA-402 CST(mode 10);PDO 已映射 0x6071 Target torque(本仓库 EUPH##_cst_config.yaml 已配)。
- effort 命令系数 = 1 / effort 状态系数:EUPH17=26.32、EUPH14=71.43、EUPH11=147.06(即额定力矩的千分比换算,已写进 *_cst_config.yaml)。**换驱动/换电机要重新核对额定力矩**。

**操作流程**
1. 先用 §4.1 采集 + 辨识,拿到 `identified_params.yaml`(没有也能跑,退化为 URDF 惯量,精度差些)。
2. **物理托住机械臂**或确认有抱闸 —— CST 模式下控制器未使能时输出 0 力矩,**重力会让臂跌落**。
3. 起 free-drive(实机,默认进 CST):
   ```bash
   ros2 launch armv7_zero_force_controller free_drive.launch.py \
       identified_params:=/abs/path/identified_params.yaml
   ```
   控制器**默认 disabled**,且使能后有 `ramp_in_time`(默认 2 s)力矩缓升,不会突跳。
4. 人离开工作空间、手放急停,再使能:
   ```bash
   ros2 service call /gravity_compensation_controller/enable std_srvs/srv/SetBool "{data: true}"
   ```
5. 用手拖动各关节。**验收(对应 plan.md 6.2)**:7 个关节都能基本拖动,松手后漂移 < 10°/s。
6. 收工:先 `enable {data: false}`(力矩归 0,托住臂)再关 launch。

**调参**(`config/gravity_compensation.yaml`)
- `gravity_scale`:先设 0.8 试(臂略重、绝不上飘),手感对了再加到 1.0。
- `damping`:加大让拖动更"黏"、抑制抖动;0 = 纯重力补偿。
- `max_torque`:每关节力矩硬上限(默认 ≤ URDF effort 限);安全天花板,别调高。
- `velocity_limit`:超速保护,关节超过该速度即该周期力矩归 0。

---

## 测试报告模板

```
日期:        2026-05-20
测试人:      <名字>
arm_ws git:  <git rev-parse --short HEAD>
硬件:        [ ] 干跑 fake   [ ] 实机 CST

4.0 自动化(9 tests)        [ ✓ / ✗ ]
4.1.1 重力模型自检          [ ✓ / ✗ ]
4.1.2 数据采集(CSV 成形)  [ ✓ / ✗ ]
4.1.3 离线辨识(残差下降)  [ ✓ / ✗ ]
4.2.1 控制器干跑加载        [ ✓ / ✗ ]
4.2.2 实机自由拖动          [ ✓ / ✗ / N/A 无硬件 ]

失败项:
  - <节>: <现象> → <troubleshooting.md 哪条>
```

---

## 全 ✓ 之后

V4.1 + V4.2 达到 plan.md 的预期:辨识数据链路与重力补偿控制器就绪。
- 干跑下整条 辨识→控制器 链路一致(机器精度互验)。
- 实机重力补偿/自由拖动可用(URDF 惯量即可跑,辨识后更准)。

留待 v0.2:在线辨识、完整 Cartesian impedance/admittance(见 plan.md §7)。
