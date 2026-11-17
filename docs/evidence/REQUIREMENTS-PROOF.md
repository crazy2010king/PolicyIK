# 需求 Proof 回填口径（现状口径）· R002 / R007 / R011 / R012 / R014 / R019

- 任务：S09-T06（切片 S09「证据闭合与需求口径校准」，里程碑 M001-acm1jg）
- 生成时间：2026-11-17（+0800）
- 目的：为下列 6 条需求给出**可直接回填需求库**的「目标 Proof（validation 文本）」与
  「目标 Notes」最终措辞，并把每条 Proof 绑定到 `docs/evidence/` 下的**具体文档与小节**与
  **关键数字**（现状口径，**不沿用** S02 旧 UAT 数字）。
- 本机环境（**fallback 档，非受支持基线**）：Ubuntu 22.04 / Python **3.10.12** /
  Drake **1.51.1**；受支持基线为 Python 3.12 + Drake 1.55.0（见 `versions.lock`）。
  下文一切数字均为 fallback 档实测值，**不得表述为与受支持基线一致**。
- 需求库侧的 Proof 回填本身由**切片关闭阶段**统一执行（不属本任务范围）；
  本文件是**待回填文本**的正式载体。

**口径总览（回填后应达到的状态）**

| 需求 | 回填前状态 | 回填后 Proof 口径 | 支撑证据文档 |
|------|-----------|------------------|-------------|
| R002 | Proof 已写，但未含 FK 门槛量化口径 | 补「真实 URDF < 5e-6 m、残差 ≈ 2.2407e-6 m、来源为官方 URDF rpy 舍入」 | `docs/architecture/fk-tolerance.md` |
| R007 | Proof 字段为空（未回填） | scenario 单一真相源 + 录制回放 + golden 三指纹 + 替身/仿真同源 | `docs/evidence/S02-rerun.md` §2 |
| R011 | Proof 未反映 CLI 双臂缺口 | 原语链门槛 + **显式 CLI 缺口（仅支持单臂、退出码 2）** + 替代路径 | `runbooks/s06_task_primitives.md` §7 |
| R012 | Proof 字段为空（未回填） | CLI 族 7 个子命令 + bimanual 退出码 0 + Meshcat 三条原因与 2D 兜底 + scenario 一处配置 | `docs/evidence/S02-rerun.md` §2 |
| R014 | Proof 字段为空（未回填） | 零 MoveIt2/rm_control 引用 + 禁用名单扫描 0 命中 + 运行时 SDK 审计面 + 幂等连接语义 | `docs/evidence/S02-rerun.md` §2 / §2.3 |
| R019 | Proof 与 Notes 自相矛盾（Proof 称已物化、Notes 称未物化） | 字节级校验成立（`--verify` 退出码 0、sha256 逐字一致）+ 归档**不入 git** + **可复现配方** | `docs/evidence/S09-upstream-archive.md` |

---

## 1. R002 · 模型与 FK 基准可信

**支撑证据**：`docs/architecture/fk-tolerance.md`（第 1–3 节：残差来源、两条门槛、鉴别力证据）、
`docs/architecture/fk-tolerance.md` 第 6 节（本文的措辞草案来源）。

**目标 Proof（validation 文本）**：

> S01 已交付并回归验证：改造版 models/rm75_drake/rm75_drake.urdf（DoF=7、总质量
> 7.776 kg±1e-6、零位 link_7 高度 0.8505 m±1e-6、7 条位置/速度限位逐条等于官方参数表）+
> 自研 rmg24_drake.urdf（2 个 prismatic + tcp frame、零 mimic）；tests/test_model_regression.py
> 26 条断言覆盖逐 link 质量/质心/惯量（≤0.5%）、平行移轴自洽（正定 + 三角不等式）、限位与
> 自碰撞过滤（7 组相邻对 filtered=True，非相邻对 False），每条正向断言均配负例。FK 一致性验收
> （独立 MDH 正解 vs 改造后官方 URDF 的 Drake FK，100 组随机构型）：**位置误差 < 5e-6 m**、
> **姿态误差 < 1e-9 rad**；该门槛的唯一真相源是 src/rm75_drake/kinematics/mdh.py 的模块常量，
> 测试层不再散落裸字面量（由 tests/test_mdh_fk.py 的口径守护用例强制）。

**目标 Notes**：

> 惯量 getter 参考点约定（Drake get_unit_inertia 为原点参考且已除质量）经平行移轴定理换算后比对；
> drake:disable_collision 标签在 Drake 1.51/1.55 均不实现，实际生效的是默认父子相邻体过滤。
> **FK 门槛口径已显式接受并收敛为唯一真相源**（见 docs/architecture/fk-tolerance.md）：官方
> RM75-B URDF 的 joint rpy = 1.5708 是四舍五入值，导致端点存在**固有残差 ≈ 2.2407e-6 m**
> （零位 link_7 的 y 偏差实测 2.24065511e-06 m，已钉死为已知量，带宽 ±1e-8 m）；因此**真实 URDF
> 位置门槛放宽为 < 5e-6 m**，而去掉官方舍入的精确 ±π/2 变体仍以 **< 1e-6 m** 判定（证明
> D-H 约定与 URDF 链等价），错误 alpha/d 表与真值的差 **> 1e-3 m**（证明该比对具备鉴别力）。
> 旧口径「位置误差 < 1e-6 m」（见 01-01-PLAN.md:11、01-01-PLAN.md:86、01-CONTEXT.md:225）
> **自 docs/architecture/fk-tolerance.md 起被显式覆盖**。

**关键数字**：位置门槛 5e-6 m（真实 URDF）/ 1e-6 m（精确 ±π/2 变体）；残差 2.2407e-6 m ±1e-8；
鉴别力门槛 1e-3 m；姿态门槛 1e-9 rad。

---

## 2. R007 · scenario 单一真相源 + 录制回放 + golden 指纹 + 替身/仿真同源

**支撑证据**：`docs/evidence/S02-rerun.md` §2（检查 #1 `pytest-not-slow`、#5 `cli-single-replay`）、
§2.2（GCS 口径）、`docs/architecture/fk-tolerance.md` 第 6 节（R007 补充句）。
复算命令：`python3 scripts/gen_golden.py --check`、`python3 -m pytest -q tests/test_regression_golden.py`。

**目标 Proof（validation 文本）**：

> S02 已交付并离线回归验证：(1) **scenario 单一真相源** —— src/rm75_drake/scenario.py 解析
> scenarios/single_arm.yaml（robots = 1）与 scenarios/dual_arm.yaml（robots = 2）两种场景，
> 同一份 scenario YAML 同时驱动仿真与真机，切换只发生在驱动实现层（drivers/ 下的 fake / sim /
> rm_device）；(2) **可录制、可离线回放** —— src/rm75_drake/recording.py 的录制文件
> schema_version = 1，回放用 replay_check（与规划链路同一套碰撞复检口径），
> `cli single → cli replay` 复检位形数 155、**逃逸数 0**、最大单步关节变化 0.012166 rad；
> (3) **golden-file 回归** —— scripts/gen_golden.py --check 退出码 0，
> fingerprints_match=True、**points = 722**、tcp_mismatches = 0、max_tcp_diff_m = 0.000e+00、
> gripper_match=True、pointwise_identical=True；基线由 src/rm75_drake/regression/golden.py 的
> **固定三指纹**（urdf_sha256 / scenario_sha256 / task_sha256）锁定；(4) **仿真/替身驱动同源** ——
> 同一 Planner 输出被替身驱动与仿真驱动消费时关节序列逐点一致（tests/test_fake_driver.py 的
> pointwise_identical 断言）。

**目标 Notes**：

> 真机侧的回放/录制链路与替身、仿真共用同一份 scenario 真相源，但**真机行为未被验证**
> （无真机）。golden 指纹与录制回放的基线均在 docs/architecture/fk-tolerance.md 统一后的
> FK 口径（真实 URDF < 5e-6 m、精确 ±π/2 变体 < 1e-6 m）下产生，不受旧 1e-6 m 表述影响。
> GCS 数字须按**凸松弛**口径解读（本机无整数版 GCS 求解器，见 docs/evidence/S02-rerun.md §2.2）。

**关键数字**：`schema_version = 1`；golden points = 722；tcp_mismatches = 0；
回放复检位形数 155 / 逃逸数 0；三指纹 = urdf_sha256 + scenario_sha256 + task_sha256。

---

## 3. R011 · 原语链与可测门槛（含 CLI 双臂缺口的显式反映）

**支撑证据**：`runbooks/s06_task_primitives.md` §7「CLI 契约边界」、
`docs/evidence/S02-rerun.md` §2（检查 #6 `cli-bimanual`）、§4 漂移对照表第 3 行。

**目标 Proof（validation 文本）**：

> S06 已交付并离线回归验证：rm75_drake/tasks/ 提供 10 个原语白名单 + 单一主状态源任务状态机
> （原子 goal_id、单一求解器入口 solve_task、失败带 stage/reason）+ 单臂 numpy 快路径
> （解析 IK → 关节插值 → collision_recheck 零逃逸 → retime 5 ms 网格，整链离线 0.33 s）+
> 双臂 plan_carry / plan_handover（基于 S05 常数 X_LR，逐点相对位姿残差 6.8e-9 m / 3.0e-8 rad，
> 交接点 1.4e-16 m，均 ≤ 1e-6）；指标口径 scripts/task_stats.py --samples 20 --json 退出码 0，
> 逐原语 success_rate 1.0、position_error_p95 ≤ 1e-5 m、orientation_error_p95 ≤ 1e-5 rad、
> zero_escape_rate 1.0、thresholds_met = true。
> **CLI 能力边界（显式缺口，不假装已覆盖）**：`cli task` 的场景装配固定走单臂 build_scene（7 关节），
> **仅支持单臂**场景；多臂场景（scenario 的 robots > 1）被前置校验**显式拒绝**，
> **退出码 2** + 稳定 reason 标签 **`task_single_arm_only`**（不再是 SceneError 关节数不匹配的栈）。
> 因此**双臂交接编排不可经 CLI 复现**，替代路径为 `cli bimanual`（离线双臂协同求解，
> 退出码 0，相对位姿残差 max 1.0395e-08 m / 3.541e-08 rad，num_escapes = 0）与库层
> rm75_drake.tasks.bimanual 的 plan_carry / plan_handover。

**目标 Notes**：

> 真机物理指标（抓放成功率、夹持误差、交接保持漂移、感知延迟、夹爪力控/夹空检测、长稳节拍）
> 未验证，已逐条列入 rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS（唯一真相源，
> CLI/脚本只再导出），硬件门控 SKIP 且退出码仍为 0。已知限制：交接点存在关节级跳变
> （实测 max ~5 rad，TCP 连续 2.8e-16 m），交接点须当作一次离散的「重新抓取」事件处理；
> 携带物不进入碰撞几何（pydrake 1.51.1 无 SetGeometryPose），靠任务级不变式表达。
> **CLI 侧缺口**：`task` 子命令仅支持单臂（退出码 2、reason = task_single_arm_only），
> 双臂交接走 `cli bimanual` 与 bimanual/dual_scene 编排（详见 runbooks/s06_task_primitives.md §7）。

**关键数字**：10 个原语；success_rate 1.0；position_error_p95 ≤ 1e-5 m；
orientation_error_p95 ≤ 1e-5 rad；zero_escape_rate 1.0；CLI 缺口退出码 **2** + reason
**`task_single_arm_only`**；bimanual 残差 1.0395e-08 m / 3.541e-08 rad。

---

## 4. R012 · 统一 CLI 入口族 + 可视化 + scenario 一处配置

**支撑证据**：`docs/evidence/S02-rerun.md` §2（检查 #5 `cli-single-replay`、#6 `cli-bimanual`）、
§4 漂移对照表第 3 行；可视化后端状态由 `rm75_drake.viz.backend_status()` 直接给出。
复算命令：`PYTHONPATH=src python3 -m rm75_drake.cli --help`、
`PYTHONPATH=src python3 -c "from rm75_drake.viz import backend_status; print(backend_status())"`。

**目标 Proof（validation 文本）**：

> S02 已交付并离线回归验证：**统一 CLI 入口族现有 7 个子命令** —— single / pick / bimanual /
> replay / exec / task / calibrate（`cli --help` 逐条列出），每个子命令自带退出码语义
> （0 成功或硬件门控 SKIP、1 业务失败、2 用法错误或命令未支持）；**bimanual 已实现且退出码 0**
> （"success": true、29 个采样点（0.005 s 网格）、时长 0.140 s、区域 3 个、GCS 模式
> convex_relaxation、碰撞复检逃逸 0、相对位姿残差 max 1.0395e-08 m / 3.541e-08 rad）；
> single → replay 闭环退出码均 0（复检位形数 155、逃逸数 0）；**Meshcat 即时可视化在本机不可用，
> 三条原因逐条明示**（(1) 本机 PyPI 不可达、离线镜像不含 meshcat wheel；(2) 未安装 meshcat 包；
> (3) Drake wheel 内只有 meshcat.js 前端资源、没有 Python 服务端），并由
> `rm75_drake.viz.backend_status()` 给出结构化诊断，**2D 兜底后端（fallback）可用**，
> 兜底只发生在显式选择上，不存在「假装三维渲染」的降级路径；**scenario 以 YAML 一处配置**
> （scenarios/single_arm.yaml、scenarios/dual_arm.yaml 经 src/rm75_drake/scenario.py 单一真相源解析）。

**目标 Notes**：

> 真机侧 `exec` 的执行指标未验证（硬件门控 SKIP，逐条列入 unverified_metrics，退出码仍为 0）。
> Meshcat 不可用是**显式诊断**而非静默降级：需要三维呈现时改用 meshcat 官方镜像或 2D 兜底。
> CLI 族的边界补充见 R011 条目（`task` 仅支持单臂，退出码 2）。

**关键数字**：子命令 **7** 个；bimanual 退出码 **0**、29 采样点、3 区域、逃逸 0；
Meshcat 不可用原因 **3** 条；兜底后端 fallback = 可用。

---

## 5. R014 · 厂商解算与 MoveIt2 栈仅作离线对照，禁止与主链路并发写同一控制器

**支撑证据**：`docs/evidence/S02-rerun.md` §2（检查 #4 `forbidden-apis-scan`）、§2.3（文件数口径）。
复算命令：`python3 scripts/scan_forbidden_apis.py src tools scripts`、
`python3 -m pytest -q tests/test_device_audit_forbidden.py tests/test_forbidden_apis.py`。

**目标 Proof（validation 文本）**：

> 仓库内**零 MoveIt2 / rm_control 引用**（`grep -rn "moveit\|MoveIt\|rm_control"` 在
> src / tools / scripts / tests / 配置文件上 0 命中）；**禁用名单静态扫描 0 命中** ——
> `python3 scripts/scan_forbidden_apis.py src tools scripts` 退出码 0，输出
> 「未命中禁用接口（扫描 90 个代码文件，名单 13 项）」，名单以 versions.lock 为唯一真相源；
> **运行时 SDK 审计面** —— src/rm75_drake/device/sdk.py 的 SdkAudit 记录全部 SDK 调用、
> 白名单外调用（non_whitelist_calls）与禁用名单命中（forbidden_calls，二者必须为空），
> 该审计在 `cli exec --json` 的 `sdk_audit` 段中输出；**控制器连接单实例语义** ——
> src/rm75_drake/device/transport.py 的 open/close 为**幂等**（重复 open 不重建连接，
> close 可重复调用），SDK 传输在已打开时直接返回，避免同一进程内重复占用连接；
> 厂商解算与 MoveIt2 栈**仅作离线对照基线**，主链路不与其并发写入同一控制器连接。

**目标 Notes**：

> 禁用名单静态扫描与运行时 SDK 审计是同一份名单（versions.lock:forbidden_apis）的两半：
> 静态面扫源码引用，运行时面扫实际调用；两者均 0 命中才允许判绿。
> 文件计数随代码量增长（34 → 89 → 90），**名单项数与命中数才是稳定口径**。
> **边界**：跨进程并发写同一控制器连接**未做运行时互斥**（属真机作业范围，未验证）；
> 离线对照基线（厂商 SDK / MoveIt2）在本仓库内**没有**任何被主链路引用的代码路径。

**关键数字**：禁用名单 **13** 项、扫描 **90** 个代码文件、**0 命中**；
SdkAudit 三类计数（calls / non_whitelist_calls / forbidden_calls）。

---

## 6. R019 · 外部依赖按 commit/哈希锁定并离线归档，许可逐项登记

**支撑证据**：`docs/evidence/S09-upstream-archive.md`（第 1–4 节：物化结果、三方逐字一致性、
命令/退出码/耗时、入库凭据与复现配方；第 6 节：本措辞草案来源）、
`docs/evidence/S09-ci-fast.md` 第 1 节（`verify_env --upstream --licenses` 退出码 0）。

**目标 Proof（validation 文本）**：

> 所有外部依赖按 `versions.lock:upstream` 锁定（rm_models commit =
> `cff49bb40f5ed0728505c03bff0e521a0dc21acd`、two_finger_gripper commit =
> `91f587b1d4efaab235f3d227ee614b28f35060c9`）；在本机物化归档后
> `python3 tools/fetch_upstream.py --verify` **退出码 0** 并输出「校验通过：2 个资产」，
> 两条归档 sha256 与 `third_party/MANIFEST.yaml`、`versions.lock:upstream` **逐字一致**
> （rm_models = `fb7ab533ce42a146d676e95a58d115e2695909fe325e403bddc8f84313eb202b`，
> file_count = 17，total_bytes = 3 312 746；two_finger_gripper =
> `975b5474afbac9de17bf2ea2601d6eacd0267ecdc54c0f46640115ac64d154a5`，file_count = 480，
> total_bytes = 60 747）；许可逐项登记（third_party/MANIFEST.yaml + licenses/registry.yaml，
> `tools/verify_env.py --upstream --licenses` 退出码 0，夹爪为 OpenAtom Open Hardware 1.0）；
> **归档本体不入 git**（上游许可限制：rm_models 官方网格不可再分发，故 third_party/archives/ 与
> third_party/.work/ 被 .gitignore 排除），**入库凭据 = third_party/MANIFEST.yaml +
> versions.lock:upstream**，**可复现配方 = `python3 tools/fetch_upstream.py`**（冷抓取产出的
> sha256 与既有记录逐字一致，证明指纹可复现）。证据：docs/evidence/S09-upstream-archive.md。

**目标 Notes**：

> 字节级校验在本机**已成立**（2026-11-16 冷抓取，退出码 0，sha256 与既有记录逐字一致，见
> docs/evidence/S09-upstream-archive.md）；此前「本机未物化」的状态已由 S09 消除，Proof 与
> Notes 不再互相矛盾。`rm_models` 上游许可未明示、官方网格不可再分发，故归档与浅克隆目录
> （third_party/archives/、third_party/.work/）**不入 git**，仓库中只保留清单与 lock 指纹。
> 诚实边界：真机侧夹爪固件与 wheel 归档的**实体验证**仍未做。

**关键数字**：`--verify` 退出码 **0**、「校验通过：2 个资产」；sha256 两条；
file_count 17 / 480；total_bytes 3 312 746 / 60 747；抓取 782 s、`--verify` 0.04 s。

---

## 7. R017 与 R006 / R010 状态标注不一致的说明

**事实**：R006（双臂相对位姿构造性满足）、R010（30 分钟重建环境）、R017（双臂成功率 / P95 /
零逃逸门槛）三条需求的**证据结构相似**（都是「离线可复算指标 + 真机未验证」的双段式），
但状态标注口径历史上不统一：R017 曾与 R006/R010 采用不同的判定措辞。

**统一标注规则（自本文件起生效）**：

1. 状态只依据**当前 validation 文本是否可复算**判定：
   * 离线可执行证据（命令 + 退出码 + 关键数字）齐全、且真机部分被**显式标注为未验证**
     → `validated`；
   * 只有设计描述、缺可执行证据 → `active`；
   * 证据字段为空（未回填）→ **不得**标为 `validated`。
2. 「真机未验证」**不是**降级理由：离线口径达标且真机缺口逐条列出的需求，与真机已验证的需求
   在状态上**等价**（均为 `validated`），差别必须写在 Notes 里。
3. 状态变化**必须**伴随 validation/Notes 的口径变更，不得只改状态不改措辞。

**按此规则的现状判定**：R006 = `validated`（残差 ≤ 1e-6 m 构造性满足、CLI 端到端
1.04e-8 m / 3.54e-8 rad、14 维零逃逸）；R010 = `validated`（基线环境重建退出码 0、18/18 包
按 --require-hashes 安装、wall time 8 s）；R017 = `validated`（离线成功率 1.0 ≥ 95%、
求解 P95 = 0.213 s < 3 s、零逃逸率 1.0；真机口径未验证并逐条列入
UNVERIFIED_COORDINATED_METRICS）。三者**口径现已一致**，措辞差异仅剩「指标内容不同」，
不再是状态标注规则不同。

**R017 的量化口径补充（供回填时核对）**：本轮复跑（docs/evidence/S02-rerun.md §2 / §2.2）
确认 GCS 阶段数字为**凸松弛**口径（本机无整数版 GCS 求解器），因此引用 R017 的
「单臂 GCS 规划 P95 < 1 s」时必须带该限定语；凸松弛口径下 GCS P95 = 8.876e-4 s
（目标 1.0 s，`gcs_p95_target_met: true`），端到端 P95 = 10.799 s（单列，不作验收口径）。

---

## 8. 复算方式（可直接粘贴）

```bash
# R002 —— FK 门槛口径
python3 -m pytest -q tests/test_mdh_fk.py

# R007 —— golden 三指纹 + 录制回放
python3 scripts/gen_golden.py --check
python3 -m pytest -q tests/test_regression_golden.py tests/test_replay.py tests/test_fake_driver.py

# R011 / R012 —— CLI 族与双臂闭环
PYTHONPATH=src python3 -m rm75_drake.cli --help
PYTHONPATH=src python3 -m rm75_drake.cli bimanual --json
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/pick_place.yaml    # 退出码 0
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/handover.yaml      # 退出码 2（仅支持单臂）

# R014 —— 禁用名单扫描（0 命中）
python3 scripts/scan_forbidden_apis.py src tools scripts

# R019 —— 上游归档字节级校验（退出码 0 /「校验通过：2 个资产」）
python3 tools/fetch_upstream.py --verify

# 全部证据一次性复跑（唯一证据收集器）
python3 scripts/collect_evidence.py --output docs/evidence/s02-rerun.json
```
