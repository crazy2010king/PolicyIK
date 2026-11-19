# 睿尔曼 RM75-B 真机验证阶梯（L0–L14）

> **适用硬件**：睿尔曼 RM75-B（7-DoF，仿人冗余构型），现场 IP `192.168.1.18`
> （厂商示教器网页 `http://192.168.1.18/` 可访问、可控）。
> **依据**：设计文档《睿尔曼RM75-B双臂Drake底层接口驱动_系统整体规划设计.md》
> （下称《设计文档》）§七 分阶段路线、§九 验证体系、§五 实时执行层与安全；
> 以及 M001-acm1jg 已实现的设备层 / 规划层 / 实时层能力。
> **落地物**：`src/rm75_drake/validation/`（阶梯定义 + 真机执行器）、
> `scripts/real_machine_ladder.py`（入口）、`devices/rm75_b_192_168_1_18.yaml`（现场部署配置）。
> **本次真机实测记录**：`docs/evidence/rm75-b-real-arm-ladder.md`。

---

## 〇、这份文档解决什么问题

《设计文档》§七 给的是**实施**路线（P0–P7），§九 给的是**验证体系**（分层手段 + 门槛），
但真机现场真正要回答的是三个问题：

1. **第一步做什么？** —— 不是「规划一条轨迹」，而是「确认链路通、状态回读正常、
   机械臂已使能、围栏参数与现场一致」；
2. **每一步的通过判据是什么？** —— 没有判据的现场操作会退化成「看着还行」；
3. **哪一步没过就不许做下一步？** —— §七 的核心纪律是「不达标不进下一阶段」，
   但必须落到**可执行的门禁**上，而不是靠人自觉。

⇒ 本文档把真机验证拆成 **15 级阶梯（L0–L14）**：从「只读、不动」到
「双臂搬运 + 长稳 + 回放回归」，每一级都有**判据 / 证据 / 门禁 / 失败处置**。

---

## 一、三条铁律

| # | 铁律 | 落地方式 |
|---|---|---|
| 1 | **先只读，后运动** | L0–L5、L10 全程不下发任何运动指令；L6 才开始第一次动 |
| 2 | **不达标不进下一阶段** | 每级声明 `depends_on`；依赖没 PASS ⇒ 本级一律 `blocked`（工具强制，不靠自觉） |
| 3 | **没测的必须写「未验证」** | 真机不可达 → `SKIP` + 逐条列出未验证指标；人工项 → `manual`，等证据回填，**不假装自动可测** |

⚠️ **软件急停不能替代物理急停**：任何会动真机的级别（L6/L7/L8/L9/L12/L13），
操作者必须**手持物理急停**、工作范围内无人。

---

## 二、阶梯总览

| 级别 | 标题 | 层级 | 会动 | 依赖 | 判据（摘要） |
|---|---|---|---|---|---|
| [L0](#l0--链路与网关可达只读零-sdk) | 链路与网关可达 | A | 否 | — | TCP 可连接；网页可达 |
| [L1](#l1--厂商-sdk-与控制器能力只读) | SDK 与控制器能力 | A | 否 | L0 | SDK 可导入；7 自由度；代次已知 |
| [L2](#l2--真机连接与只读状态快照只读) | 连接与只读状态快照 | A | 否 | L1 | 连接建立；状态查询返回 0；无错误码 |
| [L3](#l3--udp-主动上报接入与周期标定只读) | UDP 主动上报与周期标定 | A | 否 | L2 | ≥100 帧；周期与期望一致；无不完整帧 |
| [L4](#l4--使能标志与就绪判定只读) | 使能标志与就绪判定 | A | 否 | L3 | 收到使能标志；7 轴全使能；非急停/缓停态 |
| [L5](#l5--运动前静态安全自检围栏--软限位只读) | 运动前静态安全自检 | A | 否 | L2 | 关节在软限位内；围栏零违规 |
| [L6](#l6--单轴小幅点动与回零首次运动) | 单轴小幅点动与回零 | B | **是** | L4, L5 | 点动误差 ≤ 1°；回零误差 ≤ 1° |
| [L7](#l7--5-ms-透传保持位姿与周期抖动运动) | 5 ms 透传保持与抖动 | B | **是** | L6 | 抖动 P99.9 ≤ 0.5 ms；零违规；零传输错误 |
| [L8](#l8--规划轨迹执行与末端跟踪运动--外部测量) | 规划轨迹执行与末端跟踪 | C | **是** | L7 | 执行退出码 0；末端误差 ≤ 3 mm |
| [L9](#l9--夹爪供电--协议--开合与夹持反馈运动) | 夹爪开合与夹持反馈 | B | **是** | L4 | 夹爪在线；开/合回读误差 ≤ 20 |
| [L10](#l10--安全七闸逐闸验证离线注入--真机围栏拒绝) | 安全七闸逐闸验证 | C | 否 | L2 | 7 闸全部拦下；真机越界目标被拒 |
| [L11](#l11--故障注入与恢复五类人工编排) | 故障注入与恢复（五类） | D | 否 | L7 | 5 类故障注入并恢复 ≥ 5 |
| [L12](#l12--长稳100-hz--30-min--8-h运动) | 长稳 30 min → 8 h | D | **是** | L7 | ≥30 min；状态 completed；零违规 |
| [L13](#l13--双臂接入标定与相对位姿搬运) | 双臂接入与相对位姿搬运 | E | **是** | L12 | 第二台臂可达；共同点 ≤ 0.5 mm；成功率 ≥ 95% |
| [L14](#l14--录制回放与-golden-file-回归离线) | 录制回放与 golden 回归 | E | 否 | L8 | golden 比对退出码 0 |

**复杂度递进**：A（只读、零风险）→ B（单点小幅运动）→ C（连续轨迹 / 完整安全闸）
→ D（破坏性故障与长时）→ E（双臂与产品化回归）。

---

## 三、逐级详解

### L0 — 链路与网关可达（只读，零 SDK）

- **目标**：确认开发机与机械臂同网段、TCP 指令通道与厂商示教器网页可达。
- **前置**：机械臂已上电；网线已连接；急停手柄在手边。
- **命令**：
  ```bash
  ping -c 3 192.168.1.18
  nc -vz 192.168.1.18 8080
  curl -s -o /dev/null -w '%{http_code}\n' http://192.168.1.18/
  ```
- **判据**：`tcp_connected == true`；`http_status ∈ {200, 301, 302, 403}`。
- **证据**：TCP 连接耗时、网页 HTTP 状态码。
- **不通过时**：确认上电与网线；确认本机在 `192.168.1.0/24`；
  **确认没有第二个进程占用 `192.168.1.18:8080`**（同一 IP+port 只能一个进程占用）。

### L1 — 厂商 SDK 与控制器能力（只读）

- **目标**：SDK 可导入、版本与 `versions.lock` 一致、控制器代次与自由度符合预期。
- **前置**：L0 通过。
- **命令**：`python3 -c 'import Robotic_Arm; print(Robotic_Arm.__file__)'`
  （⚠️ **分发名 `robotic-arm`，导入名 `Robotic_Arm`（大写 R/A）**）。
- **判据**：`sdk_available == true`；`arm_dof == 7`；`controller_version` 已知。
- **不通过时**：`pip install robotic-arm==1.1.7`；离线环境从 `vendor/wheels/` 安装。

### L2 — 真机连接与只读状态快照（只读）

- **目标**：建立 SDK 连接，读回关节角 / TCP 位姿 / 逐轴电流 / 逐轴错误码。
- **前置**：L1 通过；机械臂未急停。
- **命令**：`python3 scripts/real_machine_ladder.py --stage L2 --json`
- **判据**：`connected == true`；`state_query_ok == true`；`joint_error_count == 0`。
- **证据**：关节角（deg/rad 双写）、TCP 位姿、逐轴电流（mA）、逐轴错误码。
- **不通过时**：连接失败查 IP/端口与占用；错误码非 0 走官方流程
  （放宽限位 → 清错 → 上使能 → 回位 → 恢复限位）。

### L3 — UDP 主动上报接入与周期标定（只读）

- **目标**：打通**唯一**状态回读通道，并**实测**上报周期（定死 `cycle` 读法）。
- **前置**：L2 通过；上报目标 IP 填成**本机**在 `192.168.1.x` 上的地址（不是机械臂 IP）。
- **命令**：`python3 scripts/real_machine_ladder.py --stage L3 --json`
- **判据**：`frames ≥ 100`；`period_match == true`（实测 P50 与期望周期偏差 ≤ 10%）；
  `incomplete_frames == 0`。
- **证据**：P50/P99/P99.9/max、`cycle` 读法结论、不完整帧计数。
- **不通过时**：收不到帧 → 确认三线程模式（`RM_TRIPLE_MODE_E = 2`）与上报目标 IP；
  周期差 5 倍 → `cycle` 单位是 **5 ms 的倍数**（实测结论，见证据文档）。

### L4 — 使能标志与就绪判定（只读）

- **目标**：确认 **7 轴全部已使能**，且机械臂**不在急停/缓停态**。
- **前置**：L3 通过（使能标志在 UDP 帧里）。
- **命令**：`python3 scripts/real_machine_ladder.py --stage L4 --json`
- **判据**：`en_flags_received == true`；`all_axes_enabled == true`；
  `joint_error_count == 0`；`arm_current_status ∉ {9, 10}`（9 = 急停，10 = 缓停）。
- **不通过时**：未全轴使能 → 用示教器逐轴上使能或 `rm_set_joint_en_state`；
  状态 = 9/10 → **先释放急停**，本级不过 ⇒ 所有运动级一律 `blocked`。

> ⚠️ **TCP 连上 ≠ 可以动**（《设计文档》§五.3）：本级就是这条纪律的执行点。

### L5 — 运动前静态安全自检（围栏 / 软限位，只读）

- **目标**：当前姿态落在软件软限位内、TCP 在桌面之上且在奇异圆柱之外。
- **前置**：L2 通过（需要 TCP 位姿）。
- **命令**：`python3 scripts/real_machine_ladder.py --stage L5 --json`
- **判据**：`joints_within_soft_limits == true`；`fence_violations == 0`。
- **证据**：TCP 高度 / 水平距离、围栏参数、**围栏余量**
  （`fence_height_margin_m` / `fence_radius_margin_m`）、越界项列表。
- **不通过时**：TCP 低于桌面 → 抬高机械臂或修正 `table_height_m`；
  TCP 靠近基座轴线 → 移到奇异圆柱外或修正 `singularity_radius_m`。
- **注意**：围栏参数是**占位值**，必须按现场工位标定（桌面高度、基座位置）。
  余量只有几毫米时**不要动**（G6 会直接停机）。

### L6 — 单轴小幅点动与回零（首次运动）

- **目标**：第一次让机械臂动：单轴 5° 点动 → 反馈校验 → 回原位。
- **前置**：L4、L5 通过；工作范围内无人；**物理急停手柄在手边**。
- **命令**：
  ```bash
  python3 scripts/real_machine_ladder.py --stage L6 --allow-motion --json
  ```
- **判据**：`jog_error_deg ≤ 1.0`；`return_error_deg ≤ 1.0`；`joint_error_count == 0`。
- **证据**：指令关节角、**UDP 实测反馈**关节角、点动误差、回零误差。
- **不通过时**：误差大 → 先确认反馈是 UDP 实测值（不是命令回读）；
  点动后不回落 → **立即拍物理急停**，查围栏与限位。

### L7 — 5 ms 透传保持位姿与周期抖动（运动）

- **目标**：验证 5 ms 透传节拍与抖动口径（《设计文档》§二.1 预算、§六 主张 3）。
- **前置**：L6 通过；**必须显式给出保持位姿**（脚本绝不自己编造下发目标）。
- **命令**：
  ```bash
  python3 scripts/real_machine_ladder.py --stage L7 --allow-motion \
      --joint-rad "0 -0.6 0 0.9 0 0.5 0" --jitter-duration-s 60 --json
  ```
- **判据**：`jitter_p999_ms ≤ 0.5`；`num_violations == 0`；`num_errors == 0`。
- **证据**：抖动 P50/P99/P99.9/max、命令→执行延迟分位、安全违规数。
- **不通过时**：抖动超标 → 绑核 + `SCHED_FIFO`、把日志挪出热路径；
  高频跟随报错 → 周期从 200 Hz 降到 100 Hz（官方契约：≤ 10 ms）。

### L8 — 规划轨迹执行与末端跟踪（运动 + 外部测量）

- **目标**：让 Drake 规划的轨迹在真机跑完，并量出末端跟踪误差（§六 主张 8）。
- **前置**：L7 通过；场景 YAML 与模型就位；准备好外部测量手段（标定板 / 相机）。
- **命令**：
  ```bash
  python3 -m rm75_drake.cli exec --device-config devices/rm75_b_192_168_1_18.yaml \
      --recording <plan.json> --log-out run_logs/$(date +%Y%m%d_%H%M%S).json
  ```
- **判据**：`exec_exit_code == 0`；`tcp_tracking_error_mm ≤ 3.0`。
- **证据**：执行退出码、全链路运行日志、**外部测量的末端误差（mm）**、逐点指令-反馈偏差。
- **人工项**：末端误差必须外部测量（无 F/T 传感器），由操作者把实测值写进证据文件。
- **不通过时**：中途停机 → 看运行日志里的安全闸 `reason`；
  误差大 → 先怀疑零位偏移与基座外参，再怀疑规划本身。

### L9 — 夹爪供电 / 协议 / 开合与夹持反馈（运动）

- **目标**：夹爪在线、开合指令生效、回读位置与指令一致。
- **前置**：L4 通过；**夹爪独立 24 V 供电并与机械臂共地**（末端口最大 1.5 A）。
- **命令**：`python3 scripts/real_machine_ladder.py --stage L9 --allow-motion --json`
- **判据**：`gripper_online == true`；`open_error ≤ 20`；`close_error ≤ 20`（0–1000 无量纲）。
- **不通过时**：指令成功但无动作 → `rm_set_tool_voltage(3)` + 配置末端协议/波特率；
  掉电 → 独立电源并共地。

### L10 — 安全七闸逐闸验证（离线注入 + 真机围栏拒绝）

- **目标**：证明七闸**真的会拦**（§五.2）。
- **前置**：L2 通过。
- **命令**：`python3 scripts/real_machine_ladder.py --stage L10 --json`
- **判据**：`gates_checked ≥ 7`；`gates_blocked == 7`；`real_fence_rejected == true`。
- **做法**：离线替身逐闸注入（软限位 / 单步增量 / 加速度 / 看门狗 / 电流 / 围栏 / 状态 age），
  再用**真机当前位姿**构造一个低于桌面的姿态，确认真实安全层会拒绝它（**不下发**）。
- **不通过时**：任何一闸没拦住 ⇒ 该闸视为**未实现**，回 `device/safety.py` 修。

### L11 — 故障注入与恢复（五类，人工编排）

- **目标**：五类物理故障都能进入安全态并按官方顺序恢复（§九 故障注入）。
- **前置**：L7 通过；现场安全规程就位；急停可用。
- **命令**：`python3 scripts/fault_inject.py --hardware --json`
- **判据**：`faults_injected ≥ 5`；`faults_recovered ≥ 5`。
- **五类**：拔网线（`link_down`）/ 物体掉落（`gripper_offline`）/ 夹爪夹空
  （`gripper_not_at_target`）/ 关节超限位（`joint_error`）/ 供电跌落（`power_supply_drop`）。
- **不通过时**：恢复卡在 `resync_q` → 帧不完整 / 错误码未清 / 未全轴使能；
  恢复卡在 `check_gripper` → 夹爪仍离线。

### L12 — 长稳：100 Hz × 30 min → 8 h（运动）

- **目标**：长时间透传无异常停机、无安全违规累积（§七 P2 门槛、§九 长稳）。
- **前置**：L7 通过；先跑 5 min 冒烟再跑满。
- **命令**：
  ```bash
  python3 scripts/soak_test.py --joint-rad "0 -0.6 0 0.9 0 0.5 0" --hz 100 --duration-s 1800 --json
  python3 scripts/long_run.py --duration-s 28800 --json
  ```
- **判据**：`soak_minutes ≥ 30`；`final_state == "completed"`；`num_violations == 0`。
- **不通过时**：中途停机 → 查运行日志的 `stop_reason` 与违规 `reason`。

### L13 — 双臂接入、标定与相对位姿搬运

- **目标**：第二台臂接入后，共同点偏差 < 0.5 mm、相对位姿搬运成功率 ≥ 95%（§七 P4/P5）。
- **前置**：单臂阶梯全绿；**第二台臂可达**（出厂默认同为 `192.168.1.18`，必须先改 IP）。
- **命令**：
  ```bash
  python3 scripts/calibrate_dual_arm.py --scenario scenarios/dual_arm.yaml
  python3 scripts/coordinated_stats.py --samples 100
  ```
- **判据**：`dual_arm_reachable == true`；`common_point_error_mm ≤ 0.5`；
  `handover_success_rate ≥ 0.95`。
- **不通过时**：共同点偏差大 → 先修基座外参（尖对尖多姿态最小二乘），再谈规划；
  双臂 IP 冲突 → 逐台改 `.18` / `.19` 并独立网段。

### L14 — 录制回放与 golden-file 回归（离线）

- **目标**：同一条录制离线重放逐点一致（§六 主张 6）。
- **前置**：L8 至少跑过一次并留下录制。
- **命令**：`python3 scripts/gen_golden.py --check`
- **判据**：`golden_exit_code == 0`。
- **不通过时**：逐点不一致 → 先查规划版本漂移，再查时间参数化。

---

## 四、怎么跑

### 4.1 环境准备（一次性）

```bash
# SDK（分发名 robotic-arm，导入名 Robotic_Arm）
pip install robotic-arm==1.1.7
# 或离线：pip install --no-index vendor/wheels/robotic_arm-1.1.7-py3-none-any.whl

# 设备层只需要 numpy + PyYAML（**零 Drake 依赖**）
pip install numpy pyyaml
```

### 4.2 看阶梯定义（不连真机）

```bash
python3 scripts/real_machine_ladder.py --list
```

### 4.3 分批跑

```bash
# ① 只读批次（最安全）
python3 scripts/real_machine_ladder.py --to L5 \
    --device-config devices/rm75_b_192_168_1_18.yaml --json --out run_logs/ladder_readonly.json

# ② 含运动的批次（必须显式授权）
python3 scripts/real_machine_ladder.py --from L6 --to L7 --allow-motion \
    --device-config devices/rm75_b_192_168_1_18.yaml \
    --joint-rad "0 -0.6 0 0.9 0 0.5 0" --json

# ③ 离线级别
python3 scripts/real_machine_ladder.py --stage L10 --json
python3 scripts/real_machine_ladder.py --stage L14 --json
```

### 4.4 退出码

| 码 | 含义 |
|---|---|
| 0 | 全部通过（或真机不可达时的 `SKIP`） |
| 1 | 至少一级 FAIL |
| 2 | 用法错误（参数非法 / 设备配置缺失或非法） |

### 4.5 人工证据回填

需要人工的级别（L8 / L11 / L12 / L13 / L14）用证据 JSON 回填实测值：

```json
{
  "L8":  {"exec_exit_code": 0, "tcp_tracking_error_mm": 2.1},
  "L11": {"faults_injected": 5, "faults_recovered": 5},
  "L12": {"soak_minutes": 1800, "final_state": "completed", "num_violations": 0},
  "L14": {"golden_exit_code": 0}
}
```

```bash
python3 scripts/real_machine_ladder.py --stage L8 --allow-motion \
    --evidence-file evidence/manual_2026-11-19.json --json
```

---

## 五、与《设计文档》§九 验证体系的对应

| 《设计文档》§九 分层 | 本阶梯对应 | 门槛 |
|---|---|---|
| 模型层 | 离线（不占真机阶梯；见 `tests/test_model_regression.py`） | 惯量/FK/限位/质量全绿 |
| 规划层 | 离线（`scripts/plan_stats.py`） | 成功率 ≥ 95%，回放零逃逸 |
| 通信层 | **L0 / L3 / L7** | 5 ms 周期，P99.9 抖动 < 0.5 ms |
| HIL 层 | **L6 / L7 / L8 / L9 / L12** | 每级 30 min 无故障 |
| 物理层 | **L8 / L13** | 双臂 TCP 偏差 < 0.5 mm |
| 长稳 | **L12** | 8 h 无异常停机 |
| 故障注入 | **L10 / L11** | 全部进入安全态且可恢复 |

## 六、与 M001-acm1jg 切片的对应

| 切片 | 本阶梯中被验证的能力 |
|---|---|
| S01 模型与夹爪 | L5 围栏（MDH 正运动学）、L9 夹爪开口量映射 |
| S02 规划内核 | L8 轨迹下发、L14 回放回归 |
| S03 单臂真机执行与安全 | L0–L12（设备层 / 实时层 / 安全七闸 / 恢复流程） |
| S04 双臂几何与标定 | L13 共同点偏差 |
| S05 双臂紧协调 | L13 相对位姿搬运成功率 |
| S06 任务原语 | L8 / L13 的搬运与交接流程 |
| S07 工程化 | L12 长稳、L14 golden 回归 |

---

## 七、本次真机实测**已确认**与**未确认**的结论

### 7.1 已确认（2026-11-19，192.168.1.18 实测，详见证据文档）

1. `RM_TRIPLE_MODE_E = 2`（传 3 会被 SDK 判 `Invalid thread mode` 且拿不到句柄）；
2. 必须显式 `rm_create_robot_arm(ip, port)` 才会建立连接，句柄 `id == -1` 即连接失败；
3. SDK 的调用形态是**实例方法**（模块级同名 ctypes 函数签名不同，直接调会失败）；
4. `cycle` 单位是 **5 ms 的倍数**：`cycle=1` ⇒ 实测上报周期 P50 ≈ 4.998 ms；
   `cycle=5` ⇒ 实测 ≈ 24.954 ms ⇒《设计文档》「cycle=5 表示 5 ms」的读法被**否证**；
5. 逐关节接口（使能 / 清错 / 限位）是**逐轴**调用；
6. 厂商 SDK 的 UDP 接收线程在**解释器退出阶段**回调已释放对象会**随机段错误**
   ⇒ 回调对象必须进程级保活，退出用 `os._exit`；
7. 现场机械臂**当前处于急停状态**（`arm_current_status = 9`），尽管 7 轴均已使能且无错误码
   ⇒ L4 必须检查状态枚举，否则会「以为能动其实不能动」；
8. 静态保持姿态时 **J2 电流实测达 -1604 ~ -1824 mA**，而本项目保守兜底阈值是 1500 mA
   ⇒ **G5 电流闸在真机上会误触发**，必须先做逐轴电流标定。

### 7.2 仍未确认（**不得当作通过**）

* 5 ms 透传抖动 P99.9（需 L7，运动级）；
* 末端跟踪误差 < 3 mm（需 L8 + 外部测量）；
* 急停响应 < 100 ms（需物理急停 + 录像计时）；
* 100 Hz × 30 min / 8 h 长稳（需 L12）；
* 五类故障注入的物理验证（需 L11）；
* 双臂共同点偏差与相对位姿搬运成功率（需第二台臂 + L13）。

---

## 八、退出码与工具纪律小结

* 工具**永远不编造**下发目标：真机运动必须显式给 `--joint-rad`；
* 工具**永远不假装**通过：缺指标 = 不通过；人工项 = `manual`；真机不可达 = `SKIP` + 未验证清单；
* 工具**永远不越权**：越权动作只能经 `--allow-motion` 显式授权；
* 工具**永远不吞异常**：执行器异常进 `failures`，逐条打印。
