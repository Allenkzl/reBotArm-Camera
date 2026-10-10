# reBotArm-Camera 工程交接文档

更新时间：2026-09-17（Target 3.4 完成后）
当前分支：`feature/payload-identification`
本文档取代此前所有版本，是唯一完整的交接入口。

---

## 0. 一分钟看懂现状

机械臂 J7 夹爪已拆除、相机已装到末端、线缆确认不会被拉扯、J1–J6 通讯正常。

**已经确认走不通的路线**：全局百分比重力补偿（J4 会先过补甚至飞臂，J2/J3 却始终不够）。

**正在走的路线**：用 J1–J6 的位置反馈 + 主动控制时的 PD 跟踪残差，离线最小二乘辨识相机质量与质心。

**当前卡在哪**：目标 3.4（多姿态静态平衡点采样）的**采集链路已经完全打通、数据质量很好**（重复性 0.068 N·m），但**辨识结果还没收敛到物理上合理的相机参数**（拟合出 0.21 kg，期望 0.85 kg）。问题不在数据，而在**模型里还缺一项**。

**下一步最该做的事**：验证"电机实际 kp 就等于我们下发的 kp"这个假设。整个项目所有力矩估计都建立在它之上，而它从未被独立验证过（详见第 9.1 节）。

**机械臂当前是安全的**：已失能，静止在 `[-8.31, 0.09, 0.13, 1.08, 0.48, 32.85]°`（接近 Home，J2/J3 贴着下限位靠限位支撑）。

---

## 1. 快速上手（接手第一天按这个顺序做）

1. 读本文档第 2 节**安全边界**，这是硬约束。
2. 确认能连上远端（第 3 节），跑一次只读审计确认机械臂还活着：

   ```bash
   ssh -i ~/.ssh/id_ed25519_rebot_codex com@com-desktop.local \
     'cd /home/com/reBotArm-Camera && /home/com/reBotArm_control_py/.venv/bin/python \
      payload_identification/feedback_audit.py --duration 2 --rate 2'
   ```

   这条命令**不使能、不运动**，只读位置，随时可以跑。
3. 读第 6 节**核心物理**。这一节是本次工作最值钱的部分，不理解它就无法判断后面的数据对错。
4. 读第 8 节**踩过的坑**。里面每一条都是真金白银换来的。
5. 从第 9 节挑一个方向继续。

---

## 2. 安全边界（必须遵守）

- **J7 已拆除**。默认不注册、不扫描依赖、不命令 J7，不执行夹爪 open/close/set，不做 J7 回零。
- **所有主动运动只允许 J1–J6。**
- **不要修改远端持久化机器人配置。** 此前所有真机脚本都用 `/tmp` 下的临时 J1–J6-only YAML（见 `feedback_audit.make_j1j6_config()`），跑完即弃。
- **主动运动、使能、轨迹执行、重力补偿输出、Home/URDF/固件/校准/电机参数写入，都必须单独获得用户确认。**
- **机械臂挂着相机时，绝不能在悬空或未支撑姿态直接 disable**，否则会自由落体。
- **如果回安全姿态失败，保持使能，让现场人员处理**，不要盲目失能。
- 使用 `can0` 前**必须再次确认它仍是 PCAN-USB**，不能是 Jetson 内置 CAN。
- 日志必须同时写 JSONL **并且**在控制台打印关键事件。
- 速度 abort 不能用单样本有限差分，必须用**滤波 + 连续超限计数**。
- **不要**在仓库里写 SSH 密码，需要时向用户线下获取。

### 关于"安全姿态"的一个具体事实

Home 姿态 `[-6.11, 0.08, 0.97, -1.15, 0.21, 32.86]°` 之所以是安全停车点，是因为 **J2/J3 都已经贴在下限位上**，机械臂是折起来"坐"在限位上的。所以在 Home 淡出力矩、失能，机械臂不会掉，只会靠在限位上。这也是每次实验收尾都用它的原因。

---

## 3. 远端环境

```text
host      : com-desktop.local   （解析到 192.168.50.194）
user      : com
ssh key   : ~/.ssh/id_ed25519_rebot_codex   （免密登录）
平台      : Jetson aarch64, L4T R36.4.3, kernel 5.15.148-tegra
工作目录  : /home/com/reBotArm-Camera       （注意：不是 git 仓库）
机器人 SDK: /home/com/reBotArm_control_py
python    : /home/com/reBotArm_control_py/.venv/bin/python
CAN       : can0 = PCAN-USB (0c72:000c), 1 Mbps, ERROR-ACTIVE, USB 1-2.2:1.0
            can1 = Jetson 内置 mttcan（不是当前控制总线）
电机驱动  : motorbridge 0.5.0（send_mit 最终进 C 原生库）
```

注意：

- `/home/com/reBotArm-Camera` 只是脚本和日志的工作目录，**没有 git**。仓库在本机，用 `scp` 同步脚本过去。
- `com-desktop.local` 的 mDNS 和 SSH 曾经出现过**间歇性失败**（连不上或认证失败）。遇到时先 `ping` 和 `nc -z 22`，再退回 IP `192.168.50.194`，通常等一会儿就恢复。
- 机器人 SDK 目录 `/home/com/reBotArm_control_py` 是**另一个 git 仓库**，本项目的「运行时」URDF 和重力标定数据都在里面。

---

## 4. 硬件与负载已知信息

用户提供：

```text
相机质量     : 约 0.8 kg（含支架约 0.9 kg）
相机质心偏移 : 沿末端 local X 方向约 +0.06 m
```

**重要修正与澄清**：

- 用户最初说质心偏移在 Y 轴，随后纠正为 **X 轴**。
- 这个 `+0.06 m` 的**基准点是 `gripper_end` 坐标系原点（≈ 原夹爪的抓取平面/指尖平面），不是法兰面**。交接时务必向用户确认清楚。
- 方向已用 FK 独立验证是对的：在 `gripper_end` 系下，臂体在 −X 侧（link5 `x∈[−0.254,−0.166]`、link6 `x∈[−0.167,−0.157]`、`gripper_end` 本体 `x∈[−0.157,−0.073]`、手指 `x∈[−0.089,0]`），**外侧是 +X**，所以 `+0.06` 的符号没错。

### 两个至今没有答案、但现在很关键的问题

1. **J7 电机是跟着夹爪一起拆掉了吗？** 它的质量算不算在 URDF 那 0.80 kg 里？
2. **相机具体怎么装的**——它的实际质心相对原夹爪抓取平面，是更靠外还是更靠里？装了多长支架？

---

## 5. 项目文件地图

```text
AGENTS.md
  项目状态、约定、变更日志（每次有进展都要更新）。

agent.md
  本文档。完整交接入口。

docs/payload-identification-plan.md
  负载辨识总规划（Stage 0-4）。Target 3.4 的状态已追加在文末。

docs/gravity-console.md
  网页重力补偿调参工具的说明与实测复盘。

docs/checklists/
  target-1-*            只读扫描
  target-2-*            只读重力矩估算、20% 补偿、全局调参复盘、回 Home 失能
  target-3-feedback-audit / pd-hold-probe / slow-sweep-result

payload_identification/
  feedback_audit.py          只读反馈审计（不使能、不运动）；含 make_j1j6_config()
  pd_hold_probe.py           最小主动 PD hold 探针；含 KP/KD/minimum_jerk/
                             make_model_gravity/shutdown_robot 等公共工具
  collect_slow_sweep.py      单关节慢速 sweep（Target 3.3）
  design_poses.py            【新】离线姿态设计与可辨识性分析（纯计算，不碰 CAN）
  collect_static_poses.py    【新】多姿态静态平衡点采样（Target 3.4 主脚本）
  fit_arm_error.py           【新】从 2026-07-17 旧标定扫频数据拟合臂体模型误差
  fit_payload.py             【新】离线拟合相机 m / 质心，含模型对比与质量扫描
  README.md

gravity_console/
  server.py                 网页端重力补偿工具（含 payload torque 计算逻辑）
  static/                   前端

logs/payload-id/            负载辨识 JSONL 日志
logs/runs/, logs/target-2/  网页调参和目标二的日志
```

---

## 6. 核心物理与数学（本节最重要）

### 6.1 重力补偿的力矩链路

SDK 里一次 MIT 指令的力矩构成：

```text
tau_ff  = clip(tau_model(q), -FF_CLAMP, +FF_CLAMP)      前馈
tau_pd  = kp * (q_target - q) - kd * vel                电机内部 PD
tau_cmd = tau_ff + tau_pd                               电机实际出力
```

`tau_model(q) = compute_generalized_gravity(model, q)`，即 Pinocchio 用 URDF 算出的"托住机械臂所需要的关节力矩"。

**关键点：没有独立力矩传感器。** 所有力矩都是从 PD 跟踪误差反推的：

```text
tau_est ≈ kp * err - kd * vel       (kp 被假定为精确的 N·m/rad)
```

### 6.2 URDF 里夹爪还在 —— 这解释了一切早期的困惑

URDF 中仍然包含 J7 夹爪：

```text
gripper_end    0.65    kg
gripper_left   0.0752  kg
gripper_right  0.0752  kg
--------------------------
合计           0.8004  kg     质心在 gripper_end 系 x ≈ -0.10 m
```

所以 `tau_model` 里**已经含有夹爪的重量**。拆掉夹爪、装上相机之后，PD 需要补的**不是相机本身的重量**，而是**两者之差**：

```text
tau_pd = g_true - tau_model
       = (g_arm_true - g_arm_URDF) + tau_camera - tau_gripper
```

如果臂体质量是对的，就简化为：

```text
tau_pd = tau_camera - tau_gripper
```

**这就是为什么早期残差只有 0.1–0.3 N·m，而相机单独应该有 ~4.5 N·m。** 夹爪（0.80 kg）和相机（0.85 kg）质量几乎相同、力臂方向相反，大部分互相抵消。

> 早期文档把"残差太小"归因于方法不行，这个归因是**错的**。真正的原因是夹爪与相机互相抵消。

### 6.3 静态平衡方程与可测残差

在 dwell 稳态（速度≈0），`tau_pd = kp*err`，且有：

```text
tau_ff + kp*err = g_true(q)
```

若前馈没有触发钳制（`tau_ff == tau_model`），则：

```text
tau_pd = g_true(q) - tau_model(q)
```

若前馈**被钳制**了，必须离线补偿：

```text
y := tau_pd - (tau_model - clip(tau_model)) + tau_gripper_URDF
```

本项目最终采用的 FF_CLAMP 是 `[1.0, 9.0, 12.0, 5.0, 0.8, 0.8]`，经离线检查**完全覆盖**了所有候选姿态的 `|tau_model|`（实测最大 `[0, 7.9, 7.14, 2.01, 0, 0]`），所以没有钳制，这一项为 0。但**换其他姿态/参数时必须重新检查**。

### 6.4 负载回归量（Payload Regressor）

SDK 计算末端负载重力矩的方式是：

```text
j_point   = J_lin - skew(R @ r) @ J_ang         点雅可比
tau_pay_i = m * g * j_point[2, i]
```

展开（记 `r_world = R @ r`，`omega_i = J_ang[:, i]`）：

```text
tau_pay_i = m*g*( J_lin[2,i] - (r_world_x * omega_iy - r_world_y * omega_ix) )
```

代入分组参数 `v = m * r`（则 `m * r_world = R @ v`），得到**对 θ 完全线性**的形式：

```text
θ = [m, m*rx, m*ry, m*rz]

A[i, 0]   = g * J_lin[2, i]
A[i, 1+k] = -g * R[0,k] * omega_iy + g * R[1,k] * omega_ix      (k = 0,1,2)

tau_payload = A(q) @ θ
```

**自检**：`design_poses.py` 每次运行都会把 `A @ θ` 和 SDK 直接的算法对比，实测最大偏差 **4.441e-16 N·m**，即完全一致。这个自检在 `fit_payload.py` 里也有（用 `computeJointTorqueRegressor` 对比模型自身重力，偏差 8.9e-16）。**改动回归量后必须先看这两条自检通过。**

### 6.5 相机/夹爪的精确简并（重要陷阱）

URDF 夹爪的重力矩**恰好**等于：

```text
tau_gripper = A(q) @ [0.8004, -0.0824, 0, 0]
```

（`-0.0824 / 0.8004 = -0.103 m`，正是夹爪质心位置）

也就是说，"夹爪"在数学上就是一个 0.8004 kg 位于 x = −0.103 m 的点质量。于是下面三个模型**残差完全相同**，质量互相正好差 0.800 kg：

| 模型 | 方程 | 拟合出的 m |
|---|---|---|
| `cam_only` | `tau_pd = A θ` | **−0.7294 kg** |
| `grip_corrected` | `tau_pd + tau_grip = A θ` | **+0.0710 kg** |
| `sign_flipped` | `tau_pd = tau_grip − A θ` | **+1.5298 kg** |

**结论：只靠换装后的数据，相机和夹爪在数学上不可辨识。** 必须有换装前的基线。

### 6.6 臂体模型误差 E_old（缺失的那块拼图）

URDF 的臂体质量不是 CAD 值，是 2026-07-17 手工标定过的。标定文档明确说：

> "The URDF gravity model with the current masses is correct to ~5–11% on every joint that carries load, with no zero offsets."

标定是在**带夹爪的旧配置**下做的，而且**原始扫频数据还在**：

```text
/home/com/reBotArm_control_py/tools/gravity_calibration/data/pdsweep_*.json
/home/com/reBotArm_control_py/tools/gravity_calibration/data/autofloat_joint3_vendor_*.json
```

每个样本含：全 6 关节姿态 `q6`、主动关节、实测保持力矩 `tau_est`、运动方向 `dir`。

定义旧配置的模型误差函数：

```text
E_old(q) = tau_pd_old(q) = g_true_old(q) - tau_model(q)
```

因为**臂体本身没有变**，这个误差函数在相机配置下**同样成立**。于是得到最终的辨识方程：

```text
tau_pd_new(q) = E_old(q) + tau_camera(q) - tau_gripper_URDF(q)

        ↓

y := tau_pd_new + tau_gripper_URDF(q) - E_old(q) = A(q) @ θ_camera
```

实际使用的 `E_old` 参数化形式（见 `fit_arm_error.py`）：

```text
E_old(q) = Σ_j (k_j - 1) * tau_model_j(q)      j ∈ {J2, J3, J4}
```

### 6.7 为什么必须同时拟合"每关节常数偏置"

可测残差里有一个**很大的、与姿态无关的分量**（摩擦、线缆力、常数模型偏差）。如果不让拟合吸收它，它会被错误地摊到相机参数上。

实测效果（σ 分析）：

| 拟合方式 | σ_m | σ_rx（质心沿工具 X） |
|---|---|---|
| 只用 4 个相机参数 | 1.3 % | **90.4 mm** |
| 4 个相机参数 + 6 个关节偏置 | 2.7 % | **10.7 mm** |

**质心的可辨识性几乎完全依赖于那 6 个偏置项。** 这不是可选项。

---

## 7. 里程碑全记录

### Target 1 — 相机安装后只读扫描 ✅

- 远端登录成功，`can0` 是 PCAN-USB（1 Mbps，ERROR-ACTIVE），`can1` 是 mttcan。
- J1–J6 在 `can0` 上在线，角度反馈可读。
- 未命令 J7、未使能、未运动、未运行重力补偿。

记录：`docs/checklists/target-1-scan-result-2026-09-17.md`

### Target 2 — 只读重力矩估算与全局补偿尝试 ✅

只读估算（用 `m = 0.8 kg`、`offset = [0.06, 0, 0] m`），near-Home：

```text
负载力矩主要在 J2/J3/J4
J3 总估计保持力矩 ~= 11.29 N·m
J4 负载贡献       ~= +2.74 N·m
J2 负载贡献       ~= -2.68 N·m
```

20% 主动重力补偿：临时 J1–J6-only 配置，J7 未注册，能减轻负载但不能托住。

全局百分比网页调参（`gravity_console/`）：

- 修过一个关键 bug：旧版只把机械臂模型重力下发，没把 0.8 kg 相机的 payload 加进去。
- 修复后网页使用 `model + payload` 总重力矩。
- 实测 **40–50% 时 J1/J4 更容易托起；60% 左右 J1/J4 过补甚至有飞臂倾向；而 J2/J3 从 20% 到 60% 始终支撑不足。**

**结论：全局百分比不是正确路线。** J4 会先过补而 J2/J3 仍不够，说明问题不是"整体缺力矩"，而是**力矩分配不对**——这直接指向负载参数辨识。

记录：`docs/checklists/target-2-*.md`、`docs/gravity-console.md`

### Target 3.1 — 只读反馈审计 ✅

脚本：`payload_identification/feedback_audit.py`

发现：

- `robot.arm.get_positions()` 可用，位置走 RobStride `mechPos` 参数 `0x7019`。
- **disabled/read-only 模式下没有可靠的 torque/current/effort 反馈。**
- `robot.get_state()` 在该模式下可能返回零数组。
- `mechVel` (`0x701A`) 在这版固件上**不是 rad/s**（实测比 `dq/dt` 大 2.6–4.8 倍且符号不一致）。

**结论：不能只靠只读数据辨识负载，必须进入主动 PD。**

### Target 3.2 — 最小主动 PD hold 探针 ✅

脚本：`payload_identification/pd_hold_probe.py`
日志：`logs/payload-id/pd_hold_probe_20260917_163417.jsonl`（240 样本）

```text
事件链: hold_complete -> returned_to_start -> disabled -> closed
q0_deg    ~= [-15.23, 0.12, 0.90, -1.15, -0.07, 33.42]
q_end_deg ~= [-14.61, 1.10, 1.12, -0.64, -0.02, 32.94]
6 秒内最大漂移 ~= [0.62, 1.01, 0.26, 0.51, 0.07, 0.51] deg
avg_tau_ff_nm  ~= [0.00, 1.54, 6.76, 2.00, -0.00, -0.00]
avg_tau_pd_nm  ~= [-0.09, -0.33, -0.07, -0.07, -0.01, 0.05]
```

**结论**：方法可行、数据稳定，但单姿态残差太弱，不能辨识质心。

### Target 3.3 — J3 慢速 sweep（三轮）⚠️ 结论已被修正

脚本：`payload_identification/collect_slow_sweep.py`

| 轮次 | 目标 J3 范围 | 实际 J3 跨度 | 实际 J4 跨度 | 结论 |
|---|---|---|---|---|
| Run 1 ±3° | −2.19 ~ +3.81° | 0.35° | 0.18° | 太弱，J3 基本没跟上 |
| Run 2 ±6° | −5.31 ~ +6.69° | 3.36° | 5.58° | J3 动了，但 J4 漂移过大 |
| Run 3 ±6° | −5.02 ~ +6.98° | 4.64° | 2.26° | 最强的一次，但仍混有跟踪误差 |

Run 3 日志：`logs/payload-id/slow_sweep_20260917_170219.jsonl`

#### ⚠️ 对早期结论的重要修正

早期文档称 "Run 3 的 negative dwell 是最有价值的数据，重复性很好（`tau_pd_J3` −4.22 / −4.23）"。**这个结论是错的。**

真实情况：**那是限位饱和，不是重力样本。**

- J3 下限是 `0°`，Home 停在 `+0.83°`。
- Run 3 命令 J3 到 `−5.02°`，**低于可达范围**，960 个样本里 **546 个的 J3 目标 < 0**。
- 关节停在 `q3 ≡ +1.033°` 不动（最后 40 个样本 `std = 0.0000°`，`tau_pd std = 0.0000`），是**顶在硬限位上**，不是稳态平衡。
- 而 `dwell_positive`（`q3 = +5.46°`，误差 `1.51°`）**才是有效的重力样本**。

#### ⚠️ 另一个被修正的建议

早期文档建议"把 target 设成当前实际姿态"（`docs/payload-identification-plan.md` 文末也有）。**这条建议是自我否定的**——`err = q_target − q_actual`，令 target 等于 actual 就把 PD 信号归零了，什么都测不到。

**正确做法：保持 target 固定不动，等它稳定到稳态，再采样。**

同理，早期建议的"只接受跟踪误差 ≤1° 的样本"过滤器也是错的——它会丢掉有效的 `dwell_positive`（1.51°）。**正确的接受准则是：窗口内 `std(tau_pd) ≈ 0` + 目标在限位内 + 未饱和。误差大小本身是信号，不是噪声。**

**结论：连续 sweep 的数据只适合判断"能不能动"和"残差是否随姿态变化"，不能直接拟合。**

---

## 8. Target 3.4 — 多姿态静态平衡点采样（本次工作）

### 8.1 思路转变

从"拖动关节过轨迹"改成"**命令一个固定目标然后等**"。稳态时 `kp*err` 精确等于重力与负载之差，这才是干净的测量。

### 8.2 `design_poses.py` — 离线姿态设计（纯计算，不碰 CAN）

能力：

- 精确负载回归量（自检 4.4e-16）
- 夹爪剥离模型（把 URDF 三个夹爪 link 的惯量置零重建模型）
- 安全性预演（FK 算相机与各腕部坐标系的世界高度）
- **按"实际到达姿态"校验限位**：先预测下垂量 `q_settled = q_target − tau_pd/kp`，迭代 3 次收敛，再用 `q_settled` 做限位和安全检查
- 贪心缩放的 D-最优选点
- **可辨识性分析**：给定 σ_τ，算出 `σ_m` 和 `σ_r`（含/不含每关节偏置）

#### 激励策略对比（含 6 个关节偏置，σ_τ = 0.05 N·m）

| 方案 | 动了哪些关节 | σ_m | σ_rx |
|---|---|---|---|
| (a) | 只动 J2/J3/J4 | 2.2 % | 9.1 mm |
| (b) | + J5±15°, J6±30° | 1.8 % | 8.5 mm |
| (c) | + J5±20°, J6±40° | 1.5 % | 6.8 mm |

**选择 (a)**：J5/J6 会**绕工具轴扭转相机线缆**，而收益只有约 2 mm，不值得冒线缆风险。

#### 两个关键设计决定

1. **J1 和 J6 对负载重力矩完全没有信息量。** J1 是竖直轴（重力矩恒为 0），J6 绕工具轴自转。实测残差跨度：J1 = 0.000、J6 = 0.001 N·m。它们只对估计关节偏置有帮助。
2. **限位约束必须作用于"实际到达姿态"。** 加上这条后，**80 个候选里有 50 个被剔除**。这正是 Target 3.3 失败的根因——命令到限位以下，关节顶在硬限位上，测到的是限位值而不是重力。另外实测 **J3 的物理硬限位在 ≈ +1.03°**，而 URDF 写的是 0°。

最终安全阈值提到 `min_camera_z = 0.20 m`，精度几乎不损失（σ_m 2.6 %，σ_rx 10.2 mm）。

### 8.3 `collect_static_poses.py` — 采样脚本

特点：

- 目标限位钳制（默认 2° 余量）
- **FK 预检安全扫描**：对每个目标做 ±4° 盒子采样，检查相机和各腕部坐标系的世界高度，**不通过就拒绝运动**
- nearest-neighbor 排序，减少行程
- minimum-jerk 斜坡，速度上限 4°/s，最短 4 s
- dwell 6 s，控速 40 Hz
- abort：跟踪误差超限 / **滤波速度**连续超限（不是单样本）
- 结尾**重复第一个姿态**，直接测整条链路的重复性
- 回 Home → 淡出力矩 → disable → 记录 post-disable 姿态
- J7 全程不注册、不命令

参数：

```text
kp = [8, 35, 40, 25, 5, 5]        kd = [0.8, 2.2, 2.8, 2.0, 0.4, 0.4]
ff_clamp = [1.0, 9.0, 12.0, 5.0, 0.8, 0.8]
```

#### 预检真的救了一次

第一版设计的 10 个姿态（离地阈值 0.09 m）在 ±4° 盒子预检下**有 2 个不合格**：

```text
pose 1 (J2=30.08, J3=30.97, J4=-31.15)  相机最低 0.086 m
pose 8 (J2=20.08, J3=15.97, J4=-16.15)  相机最低 0.078 m
```

**这两个姿态在只检查命令姿态时是"安全"的**，加上边界盒才暴露。重新设计（离地 ≥0.20 m）后预检 PASS，全部姿态相机高度 ≥0.173 m。

### 8.4 实验记录

**验证跑**（2 个姿态、短 dwell）：
`logs/payload-id/static_poses_20260917_180322.jsonl`

这次跑首次暴露了**符号反了**的问题（见 8.5）。

**完整跑**（10 个姿态 + 结尾重复）：

```text
日志      : logs/payload-id/static_poses_20260917_180506.jsonl
样本      : 5654
姿态时长  : 141.91 s
总时长    : 160.28 s
abort     : 无
事件链    : all_poses_done -> returned_to_start -> fade_complete -> disabled -> closed
post-disable 姿态: [-8.31, 0.09, 0.13, 1.08, 0.48, 32.85] deg（稳定）
```

姿态表（target 是命令值，settled 是实际停住的值）：

| # | J2 | J3 | J4 | 相机高度 |
|---|---|---|---|---|
| 0 | 30.08 | 60.97 | 13.85 | 0.657 |
| 1 | 50.08 | 45.97 | −16.15 | 0.210 |
| 2 | 10.08 | 60.97 | 28.85 | 0.752 |
| 3 | 10.08 | 30.97 | −31.15 | 0.223 |
| 4 | 10.08 | 60.97 | 13.85 | 0.716 |
| 5 | 40.08 | 30.97 | −1.15 | 0.214 |
| 6 | 10.08 | 45.97 | 28.85 | 0.683 |
| 7 | 10.08 | 60.97 | −31.15 | 0.485 |
| 8 | 50.08 | 60.97 | −31.15 | 0.270 |
| 9 | 20.08 | 60.97 | 28.85 | 0.751 |

（J1 = −6.11、J5 = 0.21、J6 = 32.86 全程不动）

**稳态残差 `tau_pd`（N·m，2.5 s 窗口均值）**：

| label | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| pose_0 | +0.277 | +0.466 | −1.386 | −0.577 | −0.024 | +0.003 |
| pose_9 | +0.273 | −1.213 | −1.157 | −0.371 | −0.024 | +0.001 |
| pose_2 | +0.275 | −0.969 | −0.648 | −0.355 | −0.024 | +0.002 |
| pose_6 | +0.272 | −0.825 | −1.487 | −0.454 | −0.023 | +0.002 |
| pose_4 | +0.273 | −0.836 | −0.725 | −0.546 | −0.024 | +0.002 |
| pose_5 | +0.274 | +1.438 | −2.558 | −1.190 | −0.023 | +0.001 |
| pose_1 | +0.272 | +0.928 | −2.015 | −1.131 | −0.024 | +0.003 |
| pose_8 | +0.275 | +0.889 | −1.710 | −1.107 | −0.024 | +0.001 |
| pose_3 | +0.273 | −0.366 | −2.347 | −1.089 | −0.023 | +0.001 |
| pose_7 | +0.274 | −0.648 | −1.337 | −1.088 | −0.022 | +0.001 |
| pose_0_repeat | +0.273 | +0.494 | −1.454 | −0.580 | −0.023 | +0.001 |

**数据质量：非常好。**

```text
重复性 (pose_0 vs pose_0_repeat):
  dq_deg  = [0.0146, -0.0403, 0.0952, 0.011, -0.0, 0.022]
  dtau_pd = [-0.0033, 0.0283, -0.0681, -0.0035, 0.0001, -0.0021]
  最大 |dtau_pd| = 0.0681 N*m      ← 这是整套链路的噪声底

J1 残差恒定 +0.273 N*m  →  纯摩擦/偏置，符合预期（竖直轴无重力矩）
J5/J6 残差 ≈ 0           →  符合预期（无重力矩）
```

### 8.5 关键发现：符号是反的

第一个姿态就暴露了问题：

| | 预测（`tau_pd = tau_cam − tau_grip`） | 实测 |
|---|---|---|
| `tau_pd` (J2,J3,J4) | [−1.12, +1.22, +1.13] | **[+0.46, −1.37, −0.58]** |
| 下垂量 (J2,J3,J4) | [−1.83, +1.74, +2.60] | **[+0.75, −1.96, −1.31]** |

**符号完全相反，三个关节一致。**

物理含义：换装后末端负载的等效重力矩**变小了**——数据表现得像"卸掉了 0.73 kg 位于 −0.18 m 的质量"。而 URDF 预测相机（0.85 kg @ +0.06）应该比夹爪（0.80 kg @ −0.10）**更重**。

**这是整个问题的核心矛盾。**

### 8.6 `fit_arm_error.py` — 从旧标定数据拟合臂体模型误差

用第 6.6 节的方法。结果：

```text
数据: 4 个扫频文件，2818 条有效方程（qmin=0.20 rad，|vel_fd| <= 1.0）
参数: 11（3 个 k + 4 个 bias + 4 个 friction）
条件数: 23.2                        ← 良态
拟合 rms: 0.2422 N*m
```

| 关节 | 本次拟合 k | ±1σ | 文档标定 k |
|---|---|---|---|
| J2 | 0.9630 | 0.0020 | 0.948 |
| J3 | 0.9484 | 0.0019 | 0.952 |
| J4 | 0.9566 | 0.0285 | 0.885 |

摩擦项与文档吻合得极好，**说明数据链路可信**：

| 关节 | 本次 friction | 文档 | 本次 bias | 文档 |
|---|---|---|---|---|
| J2 | 0.532 | 0.53 | −0.155 | −0.70 |
| J3 | 0.505 | 0.49 | +0.017 | +0.12 |
| J4 | 0.289 | 0.30 | +0.007 | +0.49 |
| J5 | 0.209 | 0.21 | +0.025 | — |

> **注意**：`--model dphi`（32 个自由臂体参数）是**秩亏的**，条件数 1e14，拟合发散到 ±71 N·m。因为旧数据是"每次只动一个关节"的扫频，激励不足。**不要重复这个尝试。**

### 8.7 `fit_payload.py` — 相机参数拟合

模型对比（11 个 dwell × 6 关节）：

```text
cam_only                 m= -0.7294 kg   r=[-0.1785, -0.0035, +0.0028]   rms=0.2583
cam_only+offsets         m= -0.8129 kg   r=[-0.1167, +0.0243, -0.0391]   rms=0.1865
grip_corrected           m= +0.0710 kg   r=[+0.6739, +0.0374, -0.0286]   rms=0.2583
grip_corrected+offsets   m= -0.0125 kg   r=[-0.9975, +1.5804, -2.5454]   rms=0.1865
sign_flipped             m= +1.5298 kg   r=[-0.1390, -0.0016, +0.0013]   rms=0.2583
sign_flipped+offsets     m= +1.6133 kg   r=[-0.1099, +0.0123, -0.0197]   rms=0.1865
calibrated (k=本次)      m= +0.2138 kg   r=[+0.0441, +0.0068, -0.0005]   rms=0.2471
calibrated (k=文档)      m= +0.2118 kg   r=[+0.0652, -0.0016, +0.0125]   rms=0.2291
calibrated+offsets       m= +0.0941 kg   r=[-0.1047, -0.2179, +0.3507]   rms=0.1867
full_arm (28 参数)       m= +0.0216 kg   r=[-43.51, ...]                 rms=0.2108  cond=1.0e14
```

**没有任何一个模型给出物理上合理的答案**（期望 m ≈ 0.85 kg，rx ≈ 0.06 m）。

### 8.8 决定性诊断：质量扫描

固定 m、只解质心（3 个一次矩分量），看残差：

```text
=== calibrated（4 参数，无偏置）===
  m [kg]     rms N*m     r=[x,y,z] m
   0.000     0.2862     —                      ← 网格上最小
   0.200     0.2473     [+0.0716, +0.0061, +0.0012]
   0.400     0.2772     [-0.1415, ...]
   0.800     0.4665     [-0.2480, ...]
   0.850     0.4954     [-0.2543, ...]
   1.200     0.7100     [-0.2835, ...]

=== calibrated + 6 关节偏置 ===
   0.000     0.1885     —                      ← 最小
   0.200     0.1890     [-0.1832, -0.0767, +0.1250]
   0.850     0.2815     [-0.2365, ...]
   1.200     0.3603     [-0.2412, ...]
```

**残差随相机质量单调上升，在 m ≈ 0 附近最小。也就是说：现有模型无法解释一个 0.85 kg 相机的存在。** 这不是数据噪声问题（噪声底只有 0.068 N·m），而是**模型里缺一项**。

### 8.9 残差结构（`calibrated` 模型，逐姿态）

```text
label          joint   y(meas)      pred     resid
pose_0            J2     0.059    -0.350     0.408
pose_0            J3     0.964     0.781     0.182
pose_0            J4     0.439     0.468    -0.028
pose_9            J2    -0.800    -0.050    -0.750
pose_9            J3     0.474     0.509    -0.035
pose_9            J4     0.145     0.245    -0.099
...
pose_3            J2    -1.322    -0.587    -0.735
pose_3            J3     0.814     1.073    -0.260
pose_3            J4     0.459     0.697    -0.238
pose_7            J4     0.338     0.647    -0.310
```

可读出的结构：

- **J4 残差系统性全负**（−0.03 ~ −0.34 N·m），说明 J4 上有一个未被建模的负向偏置。
- **J2 残差散布很大**（+0.44 ~ −0.75），同样 J2=10° 的五个姿态残差从 −0.045 到 −0.735，说明它还依赖 J3/J4。
- pose_0 与 pose_0_repeat 的残差高度一致（+0.408 / +0.442），说明这**不是随机噪声，是可重复的系统误差**。

---

## 9. 当前技术判断

### 9.1 最可能的元凶：力矩标度这个假设从未被验证

标定文档自己写了：

> "Torque scale rides on the MIT kp being exact N·m/rad (no independent torque sensor), so a firmware kp error of a few percent shifts all k values equally."

**整个项目的所有力矩估计——包括 2026-07-17 那次标定和本次辨识——都建立在"电机实际 kp 就等于我们下发的 kp"之上。这个假设从未被独立验证。**

而且有一个具体疑点值得注意：`make_j1j6_config()` 写的配置增益是 J2/J3/J4 = **150/150/50**，而主动脚本下发的是 **35/40/25**（Target 3.3/3.4）或 **30/20/10/8**（旧标定扫频）。`send_mit` 的 Python 实现确实会把传入的 kp 透传下去：

```python
# reBotArm_control_py/actuator/rebotarm.py:382
if kp is None:
    kp = self._mit_kp
...
self._mm[jc.name].send_mit(float(pos[i]), float(vel[i]), float(kp[i]), float(kd[i]), float(tau[i]))
```

但再往下是 motorbridge 的 **C 原生库**，增益是否被缩放/钳制/量化后与名义值不符，**无法从 Python 侧读到**。

**如果实际 kp 与下发值有一个比例差，那么所有残差都会被整体缩放**，正好能解释"拟合质量偏小 + 系统性偏差 + 符号看起来反了"这一组症状。

### 9.2 其他候选原因（按可疑度排序）

1. **URDF 的夹爪模型可能不是当时实际装的那个夹爪。** 如果真实夹爪比模型更重或质心更靠外，`tau_gripper` 就是错的，而它恰好是简并方向上的关键量。
2. **J7 电机的质量归属。** URDF 的 `gripper_end`（0.65 kg）visual 用的是 `motor_7.STL`，似乎已含电机。但真实拆装情况需要向用户确认。
3. **相机实际安装位置与 +0.06 m 假设不符**，或支架让质心更靠近腕部。
4. **接触/线缆力。** 相机线缆在特定姿态可能产生不可忽略的附加力矩。
5. **摩擦的建模不足。** Coulomb 摩擦在 0.2–0.53 N·m，与我们要测的信号同量级；而且它依赖接近方向，dwell 之间的历史不同。

### 9.3 明确不要再做的事

- ❌ 不要再盲目提高全局重力补偿百分比（已验证会让 J4 先过载）。
- ❌ 不要再做更大振幅的连续 sweep（残差混跟踪误差，Target 3.3 已证明）。
- ❌ 不要再尝试 32 参数的 `full_arm` 自由臂体拟合（秩亏，cond 1e14）。
- ❌ 不要再用 `cam_only` / `grip_corrected` / `sign_flipped` 三个模型互相比残差（它们数学上精确简并，残差必然相同）。
- ❌ 不要在 dwell 稳态之前采样，也不要把 target 设成当前实际姿态。

---

## 10. 推荐下一步（按优先级）

### 10.1 【最高优先】验证有效 kp —— 便宜且决定性

**原理**：在某姿态下施加一个**已知的力矩偏置** `Δτ`，让 PD 去平衡，测量关节位置移动 `Δq`。则：

```text
kp_eff = Δτ / Δq
```

**做法草案**（单次几分钟，不需要动硬件）：

1. 使能，在某个安全姿态（确保离限位足够远）用 PD 稳住。
2. 记录稳态 `q_a`。
3. 把目标从 `q_target` 移动一个已知小量 `Δq_cmd`（例如 0.5°、1°、2°），记录新的稳态 `q_b`。
4. 稳态下 `kp_nominal * (q_target − q_b) = F(q_b)`，而移动目标相当于施加了 `kp_nominal * Δq_cmd` 的等效力矩。
5. 用**已知的外部力矩**去标定更好：例如在末端挂一个已知砝码（需用户同意），或者用前馈通道注入一个已知的 `tau` 偏置，观察需要多大的位置偏移才能平衡。
6. 用多个 `Δτ` 点做线性回归，得到 `kp_eff / kp_nominal` 的比值。

**判据**：如果比值 ≈ 1，说明力矩标度没问题，转去查 9.2 的其他原因。如果比值明显 ≠ 1（例如 2–4 倍），那么**之前所有残差数值都要整体重标定**，辨识链路需要按新标度重跑。

### 10.2 【高优先】用新配置重做一次 auto-float 验证

旧标定有一个 `auto_float_test.py`：把 PD 淡到零、只留重力前馈让关节自由漂浮，用漂移量衡量模型误差（旧配置下 12 s 漂移仅 −0.0002 rad）。

**在相机配置下重做一次**：

- 如果机械臂在 `tau_model` 前馈下**向上漂**，说明模型低估了实际负载（或 kp 标度偏低）。
- 如果**向下沉**，说明模型高估了负载。

这能直接、直观地看出误差的方向和量级，比看数字快得多。注意必须在有足够限位余量、且有现场人员保护的姿态下做。

### 10.3 【中优先】用手工测量收敛几何不确定性

请用户实测：

- 相机加支架的**准确质量**（是否就是 0.9 kg？）。
- 相机质心相对 `gripper_end` 原点（原抓取平面）的 **X/Y/Z 偏移**，用尺子量即可，精度 5 mm 就够。
- **原夹爪**是否还在？能否称重？如果还在，直接称出真实夹爪质量，就能校准 `tau_gripper` 这个简并方向上的关键量。

### 10.4 【中优先】固化解耦实验

如果 10.1 表明力矩标度正常，那么需要一条**独立于夹爪模型**的路径。可行想法：

- 在末端**临时加一个已知质量的配重**（例如 0.5 kg 标准砝码，用户同意后），测量残差变化量 `Δtau`。因为配重的参数完全已知，可以直接标定"残差 → 一次矩"的映射，同时消掉了夹爪模型的不确定性。
- 或者：把相机**换回夹爪**测一次（如果硬件还在），直接拿到旧配置基线。哪怕只测 2–3 个姿态，也足以把 `E_old` 锚定住。

### 10.5 长期路线

- 辨识收敛后，把结果接进 `gravity_console/server.py`，替换掉写死的 `PAYLOAD_KG = 0.8` / `PAYLOAD_OFFSET_LOCAL_M = [0.06, 0, 0]`。
- 再做按关节独立的倍率或完整的模型补偿，验证 J2/J3 支撑改善而 J4 不过补。
- URDF 相机化改造（见第 11 节）是另一条独立的工作线。

---

## 11. 尚未完成的旁支工作：URDF 相机化改造

这条线一直**没有动工**，因为缺用户确认。背景信息：

### 远端有 4 份 URDF，2 种不同内容

```text
/home/com/reBot-DevArm/Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf
    md5 d500e6c9, 18078 B, 772 行      上游 main 一致
/home/com/reBotArm_control_py/urdf/RS/urdf/ReBot_Arm_RS.urdf
    md5 d500e6c9, 18078 B              ★ 本项目的运行时 URDF
/home/com/ReBot_Arm_Mujoco-RS/rebotarm_ros2_RS/src/rebotarm_bringup/description/urdf/ReBot_Arm_RS.urdf
    md5 788e26bb, 19434 B
/home/com/rebotarm_ros2/src/rebotarm_bringup/description/urdf/00-arm-rs_asm-v3.urdf
    19438 B
```

上游 `Rebot_Arm_description/README.md` 明确说：`Rebot_Arm_description/` 是**发布副本，不是运行时源**；运行时源是 `rebotarm_ros2_RS/src/rebotarm_bringup/description/`；模型更新后两份都要同步。两份 RS 副本在 link2–link6 的质量上差 0.309 kg，**不能盲目覆盖**。

### 改法（如果要做）

- 上游 URDF 关键行号：`gripper_end` link 在 537–602（inertial 539–552，visual 用 `motor_7.STL` 在 583–592），`j_gripper_end` 在 603–615；**要删除 `gripper_left` + `gripper_joint1` + `gripper_right` + `gripper_joint2`，即 616–771 行**；相机 link 插在 772 行 `</robot>` 之前。
- 推荐**新建 `ReBot_Arm_RS_camera.urdf`**，或**在 git 分支上原地改**（可随时回滚）。新建文件的话需要同步改**两处** `urdf_path`：`config/rebotarm_rs.yaml` 和 `gravity_console/server.py:make_j1j6_config()`（第 49 行附近，硬编码了 `"urdf/RS/urdf/ReBot_Arm_RS.urdf"` 和 `end_effector_frame: "gripper_end"`）。
- 同步清单：上游 description 仓库、`reBotArm_control_py` 里的 vendored 副本、ROS2 运行时副本；另外把 `PAYLOAD_KG` 改成 `0.0`，并注释掉 `config/rebotarm_rs.yaml` 里的 `gripper` 组（`motor_id: 0x07`）。
- 注意：URDF 的惯量张量是 5 位小数，且与质量**不自洽**（base_link 质量 ×2.03 但 Ixx ×1.11、Iyy ×2.82）。做静态重力补偿无害（只用到质量和质心），但做动力学/MoveIt 就值得怀疑。

### 需要用户回答的问题

1. `gripper_end` 和 J7 电机是一起拆掉的，还是保留？
2. 相机是装在 `gripper_end` 上还是直接装在 `link6` 上？

---

## 12. 踩过的坑（完整清单）

### 控制与安全

- **挂着相机悬空 disable = 自由落体。** 必须先回 Home（J2/J3 靠限位支撑）再淡出再失能。
- **J3 的物理硬限位 ≈ +1.03°**，而 URDF 写 0°。命令到 0° 以下，关节会顶在硬限位上，测到的是限位反力不是重力。
- **限位检查必须针对"实际到达姿态"**，不是命令姿态。下垂可以有 3°以上。
- **限位检查必须加边界盒**（±4°），只查单点会漏掉贴桌面的姿态。
- **速度 abort 必须用滤波 + 连续超限计数**，RobStride 位置量化会产生单样本尖峰。
- **不要改远端持久化配置**，全部用 `/tmp` 临时 YAML。
- 用 `can0` 前确认它还是 PCAN。
- 预检安全扫描不是形式主义——本次它真的拦下了 2 个相机离地仅 7.8 cm 的姿态。

### 数据与方法

- **"negative dwell 重复性好"是假象**——那是限位饱和，std=0 正是顶死的特征。有效样本是那个误差 1.51° 的 positive dwell。
- **"把 target 设成当前实际姿态"是自我否定**，会把 PD 信号归零。
- **"只接受跟踪误差 ≤1°"的过滤器会丢有效数据**。正确准则是窗口内 `std(tau_pd) ≈ 0` + 目标在限位内 + 未饱和。
- **目标低于可达范围时，`tau_pd` 完全无意义**（由目标决定，不由重力决定）。
- **`print_every_s = 1.0` 会让 dwell 只有 1 Hz 的日志**（40 Hz 采样只在每秒写一条）。采样窗口只有几个点。要更细的噪声估计就调小它。
- **接触区域的数据会毒害拟合**。旧标定文档记录了惨痛教训：从静止姿态（夹爪顶在桌上）扫频，拟合出**假的 −0.40 rad 零点偏移和假的 ×1.66 腕部比例**，而且 rms 看起来还很漂亮。
- **`mechVel` (0x701A) 不是 rad/s**，要用 `mechPos` 有限差分。

### 建模与数学

- **URDF 里夹爪还在**，所以可测残差是 `tau_camera − tau_gripper`，不是 `tau_camera`。
- **相机与夹爪在数学上精确简并**（差 0.8004 kg @ −0.103 m），只靠换装后数据不可辨识。
- **32 参数自由臂体拟合秩亏**（cond 1e14），别再试。
- **Per-joint 常数偏置不是可选项**，它把质心不确定度从 90 mm 压到 10 mm。
- **Pinocchio 里关节名是 `joint1..joint6`，不是 `link1..link6`**。`model.getJointId("link1")` **返回 0 而不报错**，会静默取到错误的参数列。写这类代码一定要显式检查返回值。
- **fixed joint 会被合并**：`gripper_end` 是固定关节，它的惯量被并进 `link6` 的 body；`j_gripper_end` 不是关节，是 frame。所以 `Y_link6` 和 `Y_gripper_end` 其实是同一个回归量。
- **base_link 的惯量在 Pinocchio 里归到 universe body**，所以 `computeTotalMass`（4.8311）比 URDF 直接求和（6.0085）少一个 base_link（1.1774）。这不是 bug。
- **读关节限位要用 `m.joints[i].idx_q`**，直接用关节 id 索引会 `IndexError: index 8 out of bounds`（因为 nq=8 而关节数不同）。
- **J1 和 J6 对负载重力矩零信息量**（竖直轴 / 工具自转轴）。别指望它们帮忙辨识负载。
- **`+0.06 m` 的基准是 `gripper_end` 原点，不是法兰面。**
- **前馈钳制必须离线补偿**：`tau_pd` 测的是 `g_true − clip(tau_model)`。本项目用的 clamp 已覆盖全部姿态，所以这项为 0，但换参数就必须重查。

### 环境与工具

- `com-desktop.local` 的 mDNS/SSH 会间歇性失败，`ping` + 退回 IP 通常能解决。
- 远端 `/home/com/reBotArm-Camera` **不是 git 仓库**，脚本要用 `scp` 同步。
- 本机沙箱里执行 `ssh`/`scp` 需要 `danger-full-access` 权限。

---

## 13. 常用命令

只读审计（**不使能、不运动**，随时可跑）：

```bash
ssh -i ~/.ssh/id_ed25519_rebot_codex com@com-desktop.local \
  'cd /home/com/reBotArm-Camera && /home/com/reBotArm_control_py/.venv/bin/python \
   payload_identification/feedback_audit.py --duration 2 --rate 2'
```

部署脚本：

```bash
cd /Users/allen/Desktop/Solution/reBotArm/reBotArm-Camera
scp -i ~/.ssh/id_ed25519_rebot_codex \
  payload_identification/*.py \
  com@com-desktop.local:/home/com/reBotArm-Camera/payload_identification/
```

离线姿态设计（**纯计算，安全**）：

```bash
cd /home/com/reBotArm-Camera
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/design_poses.py \
  --d5=0 --d6=0 --select=10 --min-camera-z=0.20 --min-lowest-z=0.10
```

采样脚本的 dry-run（只做预检和规划，**不使能、不运动**）：

```bash
cd /home/com/reBotArm-Camera
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_static_poses.py --dry-run
```

小规模验证跑（2 个姿态、短 dwell）：

```bash
cd /home/com/reBotArm-Camera
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_static_poses.py \
  --pose-indices 0,2 --dwell-s 2.5 --min-ramp-s 3
```

完整采样跑（约 3 分钟，**需要用户在场并确认**）：

```bash
cd /home/com/reBotArm-Camera
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/collect_static_poses.py --dwell-s 6.0
```

离线拟合：

```bash
cd /home/com/reBotArm-Camera
# 1) 从旧标定数据拟合臂体模型误差
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/fit_arm_error.py --model scale

# 2) 拟合相机参数（含模型对比）
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/fit_payload.py \
  logs/payload-id/static_poses_20260917_180506.jsonl \
  --models calibrated,grip_corrected,cam_only \
  --arm-error logs/payload-id/arm_error_fit.json \
  --settle-s 2.5

# 3) 质量扫描诊断
/home/com/reBotArm_control_py/.venv/bin/python payload_identification/fit_payload.py \
  logs/payload-id/static_poses_20260917_180506.jsonl \
  --models calibrated --arm-error logs/payload-id/arm_error_fit.json \
  --mass-scan 0,0.2,0.4,0.6,0.8,0.85,0.9,1.0,1.2
```

---

## 14. 关键日志与数据索引

```text
Target 2
  logs/target-2/camera_gc_20pct_20260917-142840.jsonl
  logs/target-2/home_disable_20260917-143657.jsonl
  logs/runs/gc_console_20260917_15*.jsonl

Target 3.1  只读审计
  logs/payload-id/feedback_audit_20260917_16*.jsonl

Target 3.2  PD hold
  logs/payload-id/pd_hold_probe_20260917_163417.jsonl

Target 3.3  J3 sweep（结论已被修正，见 7.3）
  logs/payload-id/slow_sweep_20260917_165524.jsonl   (+/-3 deg, 太弱)
  logs/payload-id/slow_sweep_20260917_165908.jsonl   (+/-6 deg, J4 漂移大)
  logs/payload-id/slow_sweep_20260917_170219.jsonl   (+/-6 deg, 最强但混跟踪误差)

Target 3.4  静态多姿态
  logs/payload-id/static_poses_20260917_180322.jsonl   验证跑（2 姿态）
  logs/payload-id/static_poses_20260917_180506.jsonl   ★ 完整跑（10 姿态 + 重复）
  logs/payload-id/pose_design_a.json                   选点结果与不确定度
  logs/payload-id/arm_error_fit.json                   臂体模型误差拟合结果
  logs/payload-id/fit_20260917.json                    相机拟合结果

旧配置基线（在 SDK 仓库里，不是本项目）
  /home/com/reBotArm_control_py/tools/gravity_calibration/data/
    pdsweep_joint2_Lpose_1784316253.json    1061 样本
    pdsweep_joint3_1784312640.json          1327 样本
    pdsweep_joint4_clear_1784315996.json     661 样本
    pdsweep_joint5_clear_1784316109.json     661 样本
    autofloat_joint3_vendor_1784313276.json  auto-float 验证
    final_clean_fits.json                    当时的拟合结果
  /home/com/reBotArm_control_py/docs/gravity_calibration_rs_2026-07-17.md
```

---

## 15. 术语速查

```text
tau_model(q)      compute_generalized_gravity(model, q)：URDF 算出的保持力矩
                  （含 URDF 里的夹爪）
tau_ff            实际下发的前馈 = clip(tau_model, ±FF_CLAMP)
tau_pd            kp*(q_target-q) - kd*vel，电机内部 PD 出力
tau_cmd           tau_ff + tau_pd
q_target / q_actual / err = q_target - q_actual
FF_CLAMP          前馈钳制向量，本项目 = [1.0, 9.0, 12.0, 5.0, 0.8, 0.8]
θ                 待辨识分组参数 = [m, m*rx, m*ry, m*rz]
A(q)              负载回归量（6×4），θ 与 tau_camera 之间的线性映射
E_old(q)          旧配置（带夹爪）下的臂体模型误差函数
k_j               每关节模型比例系数（J2 0.948 / J3 0.952 / J4 0.885，文档值）
gripper_end       末端坐标系；URDF 里作为 end_effector_frame；原点 ≈ 原抓取平面
dwell             保持目标不动、等待稳态的采样窗口
settled pose      稳态实际姿态 = q_target - tau_pd/kp
```

---

## 16. 交接检查表

接手后建议依次确认：

- [ ] 能免密 SSH 到 `com-desktop.local`
- [ ] `can0` 仍是 PCAN-USB
- [ ] 只读审计能读到 J1–J6 角度
- [ ] 机械臂静止姿态与本文档记录一致（`[-8.31, 0.09, 0.13, 1.08, 0.48, 32.85]°` 附近）
- [ ] 已读第 2 节安全边界，并理解"悬空不许失能"
- [ ] 已读第 6 节，理解 `tau_pd = tau_camera - tau_gripper`
- [ ] 已读第 12 节踩坑清单
- [ ] 已向用户确认第 4 节的两个遗留问题（J7 电机、相机安装几何）
- [ ] 已和用户确认是否可以做 10.1 的 kp 验证实验