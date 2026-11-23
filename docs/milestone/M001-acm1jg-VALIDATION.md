---
verdict: needs-attention
remediation_round: 1
---

# Milestone Validation: M001-acm1jg

## Success Criteria Checklist
### MV01 — Success Criteria Checklist

| # | Success Criterion | 证据（本轮复跑） | 判定 |
|---|---|---|---|
| SC1 | 真实 RM75-B 上 Drake 规划轨迹经自研实时层透传执行完毕，末端跟踪误差 < 3 mm，100 Hz 连续 30 min 无故障 | `scripts/soak_test.py` / `measure_jitter.py` 均 `SKIP(reason=hardware_unreachable)`（无真机：robotic-arm SDK 未安装，192.168.1.18/.19 不通）；抖动 P99.9、末端误差、30 min 长稳等逐条列入 `unverified_metrics`；离线侧 `device/safety.py` 七闸 + `exec_loop` 5 ms 采样器 + 虚拟时钟直方图（count==拍数）21 条专项测试全绿 | **UNMET（不可验证：真机不可达，证据缺位而非失败）** |
| SC2 | 两台 RM75-B 同一 world 共同抓持搬运，全程保持末端相对位姿，成功率 ≥ 95%、P95 < 3 s | 本轮复跑 `PYTHONPATH=src python3 scripts/coordinated_stats.py --samples 20 --json` → EXIT=0，`success_rate=1.0`、`solve_p95_s=0.2265`（目标 3.0 达标）、`zero_escape_rate=1.0`、相对位姿残差 max 1.835e-08 m / 9.892e-08 rad；真机 5 条指标列入 `unverified_metrics` | **UNMET（不可验证：真机不可达）／离线口径达标** |
| SC3 | 一次真机运行的全部消息被录制，离线重放后末端轨迹与原始记录逐点一致（golden-file 比对通过） | 本轮复跑 `python3 scripts/gen_golden.py --check` → EXIT=0，`fingerprints_match=True points=722 tcp_mismatches=0 max_tcp_diff_m=0.000e+00 gripper_match=True pointwise_identical=True`（三指纹 urdf/scenario/task sha256 锁定，负例「基线平移 1e-6 m ⇒ 非 0 退出」可鉴别）；`cli single` EXIT=0「碰撞复检逃逸数：0」→ `cli replay` EXIT=0「复检位形数：47」 | **PARTIAL（离线 golden-file 逐点一致通过；「一次真机运行」前提未满足）** |
| SC4 | 注入拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位四种故障，均进入安全态并按文档流程恢复成功 | 本轮复跑 `python3 scripts/fault_inject.py --json` → EXIT=0，5 条 case（link_down / object_dropped / gripper_miss / joint_overlimit / **power_supply_drop**）全部 ok=true / recovered=true / safe_state=fault / final_state=ready，`failed=[]`，`physical_faults_verified=false`（物理未验证的诚实标注保留）；`runbooks/s03_rm_device_bringup.md` §6/§6.1/§10 含五类故障与供电跌落真机复验步骤 | **PARTIAL（替身层五类全链闭合；物理注入与真机按文档恢复未验证）** |
| SC5 | 厂商解算接口调用次数在主链路中为 0（CI 静态检查 + 运行时审计双证） | 本轮复跑 `python3 scripts/scan_forbidden_apis.py src tools scripts` → EXIT=0「未命中禁用接口（扫描 90 个代码文件，名单 13 项）」；运行时 `SdkAudit` 面 `forbidden_call_count>0 ⇒ exit 1`，`tests/test_device_audit_forbidden.py` 18 passed 含「伪造一次禁用调用 ⇒ 计数 1 且退出码 1」负例；`cli exec --json` 的 `sdk_audit.forbidden_call_count=0` | **SATISFIED（双证齐备；真机连续运行计数仍为登记在册的未验证项）** |
| SC6 | 新机器 30 分钟内一条命令重建完整环境，且不依赖 ROS2 发行版 | S00 远端实测（受支持基线 ros_dev / py3.12.13 / Drake 1.55.0）：`bash tools/setup_env.sh --offline --profile py312 --venv <fresh>` → exit 0、18/18 包 `--require-hashes` 安装、`verify_env --check` 逐包全 OK、wall time 8–9 s，全程无 ROS2；本轮 `ci.sh fast` 含 `assets/lock` 与 `tests/upstream` 常驻项 PASS=17 | **SATISFIED（9 s ≪ 30 min 目标）** |

结论：SC5 / SC6 满足；SC3 / SC4 离线与替身口径成立、真机前提未满足；SC1 / SC2 因真机不可达零物理证据。全部未满足项均属**环境不可达导致的证据缺位**，无任何观察到的自动化失败。

## Slice Delivery Audit
### MV02 — Slice Delivery Audit

10/10 切片均有 `SUMMARY.md` 与 `01-0X-SUMMARY.md`；ASSESSMENT 判定如下：

| Slice | SUMMARY | ASSESSMENT | Verdict | 备注 |
|---|---|---|---|---|
| S00 | ✅ | ✅ | **PARTIAL** | round-0 未决项「上游归档正向字节级校验」已由 S09 闭合（`fetch_upstream.py --verify` 本轮 EXIT=0），但 `01-00-ASSESSMENT.md` 未同步更新 → 台账陈旧 |
| S01 | ✅ | ✅ | **FAIL（陈旧）** | 唯一 FAIL 项（`--joint-limits` 非法路径期望退出码 2、实测 1）已由 S08 在代码层修复；本轮复跑实测 **EXIT=2** + stderr 点名路径，`tests/test_verify_env_contract.py` 5 passed。**但 `01-01-ASSESSMENT.md` front-matter 仍为 `verdict: FAIL`，未被 S08 取代** |
| S02 | ✅ | ❌ **缺失** | — | 无 `01-02-ASSESSMENT.md` 文件（10 切片唯一缺口）。S09 已以 `scripts/collect_evidence.py` + `docs/evidence/S02-rerun.md/json` 补齐 runtime-executable 证据面（8/8 PASS、exit 0），但缺与其它切片同构的逐条 check + 证据 ID 判定记录 |
| S03 | ✅ | ✅ | PASS | 真机不可达；物理指标逐条列入 `unverified_metrics`；`accel_limits`/`current_thresholds` 为 measured=false/calibrated=false 保守默认值 |
| S04 | ✅ | ✅ | PASS | 192 条专项全绿；标定 CLI 默认 `status=skipped / calibration_not_measured` + 6 条未验证指标 |
| S05 | ✅ | ✅ | PASS | 相对位姿残差 ≤1e-6 m/rad；离线成功率 1.0、solve P95 0.2265 s |
| S06 | ✅ | ✅ | PASS | 原语级 `thresholds_met=true`；`cli task` 双臂缺口已由 S09 契约化为 exit 2 + `task_single_arm_only`（本轮实测 EXIT=2） |
| S07 | ✅ | ✅ | PASS | golden-file 回归 + 8 h 长稳载体（硬件门控 SKIP） |
| S08 | ✅ | ✅ | **PASS（新增）** | 退出码契约由 1 纠正为 2 并由 `tests/test_verify_env_contract.py` + CI fast 恒跑项守护；第五类故障「供电跌落」全链闭合 |
| S09 | ✅ | ✅ | **PASS（新增）** | 上游归档字节级校验 exit 0；S02 全量复跑证据入库；FK 门槛收敛为唯一真相源；CLI task 缺口契约化；六条需求 Proof 回填 |

- SUMMARY 覆盖：10/10 ✅。
- ASSESSMENT 分布：PASS 7（S03–S09）、PARTIAL 1（S00）、FAIL 1（S01）、缺失 1（S02）。
- **flag（记录级，非实现级）**：S01 的 FAIL 与 S00 的 PARTIAL 均属「台账未随代码更新」，其判据已被 S08/S09 的通过证据证伪；S02 缺独立 ASSESSMENT 文件。两者均不构成实现缺陷，但在台账层面需显式闭合。
- 其余切片无 outstanding blocker；follow-ups 均为真机复验类（合理登记项）。

## Cross-Slice Integration
### MV03 — Cross-Slice Integration

Roadmap Boundary Map 声明的 13 条原始边界 + S08→S09 修复边界，共 14 条，**全部 HONORED，0 GAP，无孤立切片**。

| Boundary | Producer Summary | Consumer Summary | Status |
|---|---|---|---|
| S00→S01 | versions.lock / constraints / fetch_upstream.py + MANIFEST / package:// 约定 / CI 三档 / 许可登记骨架 | S01：`models.py:74` 用 `package://rm75_drake/rm75_drake.urdf`；`package_map.py:121` 只走 `PopulateFromFolder` | HONORED |
| S01→S02 | rm75_drake.urdf / rmg24_drake.urdf / joint_limits.yaml / gripper.py / model_regression 套件 | S02：`planning/scene.py:134` 合并 package:// 后 `AddModels`；`planning/ik.py:196` 读 S01 限位 | HONORED |
| S01→S03 | 同上 + 速度/加速度限值表、电流阈值基线 | S03：`device/safety.py:65 from ..limits import load_joint_limits`；`devices/single_arm.yaml:49` 承载电流阈值 | HONORED |
| S02→S03 | Planner API / Trajectory 不变量 / scenario schema v1 / Driver 协议 | S03：`drivers/rm_device.py:56 from ..planning.trajectory import DEFAULT_DT, JOINT_COUNT` | HONORED |
| S02→S04 | scenario schema v1 + FakeDriver | S04：`bimanual/dual_scene.py:59` 复用 `planning.scene` 的 `SceneError` / `OBSTACLE_SOURCE_NAME`，scenario 不升版本 | HONORED |
| S02→S05 | Trajectory 不变量 / retime / gcs / regions | S05：`coordinated/limits.py:59`、`coordinated/planner.py:625/1142` 复用 | HONORED |
| S01→S05 | S01 `joint_limits.yaml` 单臂限位真值 | S05：`coordinated/limits.py:12` + `kinematics/analytic_ik.py:178` 程序派生 14 维限位，未写第二份 YAML | HONORED |
| S03→S04 | rm_device / rm_exec 实时层 / 9 态状态机 / device metrics 口径 / run_log.py | S04：`bimanual/sync_exec.py:14-21` 复用 `exec_loop` / `safety` / `metrics`（无第二套实时层） | HONORED |
| S04→S05 | 双臂单 plant（num_positions 14/18）/ X_LR / 碰撞过滤矩阵 | S05：`coordinated/parameterization.py:579 from ..bimanual.dual_scene import check_config_free` | HONORED |
| S04→S06 | ROADMAP 措辞「任务空间目标表达」（实际由 S05+S06 承载，传递链成立） | S06：`tasks/bimanual.py:79-87` 经 S05 `plan_coordinated` 取用常数 X_LR | HONORED（传递链） |
| S05→S06 | 最小坐标参数化 / MinimalRegion*Cache / `plan_coordinated → DualTrajectory` | S06：`tasks/bimanual.py:79-87`；`tests/test_tasks_bimanual.py` 钉死 S05 公开签名与逐点残差 ≤1e-6 | HONORED |
| S06→S07 | 10 原语 + 任务状态机 + 原语签名 + 感知适配接口 | S07：`scripts/gen_golden.py:127-128 from rm75_drake.tasks.schema/solver`；golden 绑定 `tasks/pick_place.yaml` sha256 | HONORED |
| S03→S07 | 录制/回放格式 `device/run_log.py`（+ S02 `recording.py`） | S07：`regression/session.py:7-9,30-31` 聚合两个 schema，checksum 口径一致 | HONORED |
| **S08→S09（修复边界）** | `tests/test_verify_env_contract.py`、`scripts/ci.sh:215` 恒跑项、`REASON_POWER_SUPPLY_DROP`、`state_machine.system_error_code()`、`ScriptedTransport.set_system_error()`、`fault_inject.py` 5 类、runbook §6.1/§10 | S09：`collect_evidence.py:194/204/216` 复用 `tools/verify_env.py` 与 `scan_forbidden_apis.py`；`docs/evidence/S09-ci-fast.md:77` 逐字记录 `[PASS] tests/verify-env-contract`；`mdh.py:145/151` FK 门槛常量 + `docs/architecture/fk-tolerance.md`；`cli.py:137` `task_single_arm_only` 契约 + `tests/test_tasks_cli.py:283` | HONORED |

**端到端组合流（S00→S09，证明咬合而非并列）**：`tools/verify_env.py --model` 从 `versions.lock` 读版本与禁名单 → `package_map` 以 `package://` 装配 S01 URDF → S02 `planning/scene.py` + `planning/ik.py` 产出 `PlanResult.trajectory` → S03 `drivers/rm_device.py` 走 `rm_movej_canfd` 唯一通道，`device/safety.py` 消费 S01 软限位 → S04 `bimanual/sync_exec.py` 复用 S03 七闸/指标下发双臂 → S05 `coordinated/parameterization.py:579` 调 S04 `check_config_free`，输出共享时间轴 → S06 `tasks/bimanual.py` 用 S05 X_LR 表达搬运/交接 → S07 `scripts/gen_golden.py` 调 S06 `solve_task` 生成 golden 并逐点比对 → S08 给同一链路加「退出码契约 + 第五类故障」两条护栏 → S09 把各链路复算结果、FK 门槛与 CLI 契约固化进 `docs/evidence/`。四个真实接缝均可在代码中点验：`parameterization.py:579`、`tasks/bimanual.py:79-85`、`gen_golden.py:127-128`、`state_machine.py:368` ↔ `recovery.py:381`。

**Advisory（不构成 GAP）**：A1 若干切片 SUMMARY front-matter 的 `requires: []` 与 ROADMAP Boundary Map 不一致（消费证据都在代码里）；A2 S09 证据面未收录第五类故障「供电跌落」的叙述（R013 未回填、`collect_evidence.py` 8 项检查无故障注入项），建议下一轮补一个 `fault-inject` 检查项；A3 S04→S06 声明产物归属为措辞偏差。

## Requirement Coverage
### MV04 — Requirement Coverage

19 条需求（R001–R019）逐条核对：**COVERED 7 / PARTIAL 12 / MISSING 0**。

| 需求 | 状态 | 证据 |
|---|---|---|
| R001 关节空间透传 | PARTIAL | `drivers/rm_device.py` 唯一 `rm_movej_canfd` 通道 + StreamingDriver 协议 + 三驱动逐点一致 atol 1e-12；「控制器不做逆解」仅替身层证据 |
| R002 自研模型回归 | **COVERED** | 26 断言回归（DoF=7、7.776 kg±1e-6、0.8505 m±1e-6、7 条限位、7 组自碰撞过滤）；FK 门槛常量唯一真相源 `kinematics/mdh.py`（URDF 5e-6 / 精确 ±π/2 1e-6 / 残差 2.2407e-6±1e-8）；偏差由 `docs/architecture/fk-tolerance.md` 正式接受并覆盖旧 <1e-6 表述；S09 回填 Proof |
| R003 5 ms 透传/抖动/时差/延迟 | PARTIAL | 虚拟时钟直方图 count==拍数 + 21 条专项；P99.9 抖动 / 双臂时差 / 命令→执行时延真机零实测 |
| R004 安全七闸 + 物理急停/STO | PARTIAL | `device/safety.py` 七闸纯函数正例+反例全绿；「独立物理急停与 STO」零证据 |
| R005 双臂同 plant 碰撞判定 | **COVERED** | `bimanual/dual_scene.py` num_positions 14/18 + S04 192 条专项全绿；臂间碰撞/自碰撞检出、零位形无碰 |
| R006 双臂相对位姿构造性满足 | **COVERED**（validated） | 残差 ≤1e-6 m/rad、14 维密采样回投零逃逸；真机执行未验证 |
| R007 同源 scenario + 录制/回放/golden | PARTIAL | Proof 已回填（schema_version=1、复检 155 位形逃逸 0、golden 722 点逐点一致、三指纹、替身/仿真同源）；「同一 YAML 驱动真机」未验证 |
| R008 厂商解算接口禁用 | PARTIAL | 静态扫描本轮实测 90 文件/13 项 0 命中 + 运行时 SdkAudit（forbidden_call_count>0 ⇒ exit 1，18 passed 含负例）；真机连续运行计数未验证 |
| R009 夹爪独立建模 + 力阈值夹取 | PARTIAL | `rmg24_drake.urdf` + `gripper.py` u↔mm 单调映射；u→N 力曲线缺失（`force_from_position` 显式抛「未标定」），力阈值夹取闭环未实现 |
| R010 一条命令重建环境 | **COVERED**（validated） | 远端 py3.12 + Drake 1.55.0：`setup_env.sh --offline` exit 0、18/18 包、`verify_env --check` 全 OK、wall 8–9 s、无 ROS2 |
| R011 任务状态机 + 原语门槛 | PARTIAL | 10 原语 + 单一 `solve_task` + `task_stats` success_rate 1.0 / thresholds_met=true；`cli task` 双臂已契约化为 exit 2 + `task_single_arm_only`（本轮实测 EXIT=2），单臂 pick_place EXIT=0；真机闭环未验证 |
| R012 统一 CLI 族 + Meshcat | PARTIAL | 7 子命令齐备、`bimanual` exit 0、残差 1.04e-08 m；**Meshcat 仍未实现**（探测式 NotAvailableError + 三条原因 + 2D 兜底），差异仅被登记未消除；Proof 已回填 |
| R013 故障诊断与恢复 | PARTIAL | 五类故障本轮复跑全链通（`ok=true / recovered=true / final_state=ready`，`failed=[]`，`physical_faults_verified=false`）；`state_machine.system_error_code()` 单一真相源 + runbook §6.1/§10；厂商 errCode ↔ 供电跌落映射未核实、真机注入未验；R013 Proof 未被 S09 回填 |
| R014 厂商栈仅离线对照 + 禁并发写 | PARTIAL | 零 MoveIt2/rm_control 引用 + 扫描 0 命中 + SdkAudit + transport 幂等连接，Proof 已回填；**离线对照基线仍无任何登记产物**，跨进程并发写未验证 |
| R015 标定链完整可执行 | PARTIAL | 纯 numpy 内核合成真值 tool_tcp RMS 2.16e-17 m / base_extrinsic 8.30e-17 m；真实标定精度零验证，`calibration/dual_arm.yaml measured=false`，CLI 默认 skipped |
| R016 真机关节空间位置伺服 | **COVERED** | `models/README.md` 文字约束 + `joint_limits.yaml` effort 仅仿真 + 白名单交叉印证 |
| R017 成功率/P95/零逃逸 | PARTIAL | 离线 success_rate 1.0、solve P95 0.2265 s <3 s、zero_escape 1.0、单臂 GCS P95 0.92 ms（凸松弛口径）；真机 5 条未验证 |
| R018 感知非阻塞接入 | **COVERED** | PerceptionBus 有界队列 + stale 标志 + 膨胀裕量三断言鉴别力（num_escapes=47>0 / margin=0 盒==0 / 旧 checker 仍自由） |
| R019 依赖锁定 / 许可登记 | **COVERED** | `fetch_upstream.py --verify` 本轮实测 **EXIT=0「校验通过：2 个资产」**，rm_models `fb7ab533…` / two_finger_gripper `975b5474…` 与 MANIFEST + versions.lock 三方逐字一致；归档不入 git + 可复现配方 + `licenses/registry.yaml` 逐项登记；Proof 已回填 |

**Round-0 遗留项闭合情况**：R-1（`--joint-limits` 退出码契约）**已闭合**（本轮实测 EXIT=2 + stderr 点名路径 + `tests/test_verify_env_contract.py` 5 passed + CI fast 恒跑项）｜R-2（第五类故障「供电跌落」）**已闭合于离线层**（5/5 全链）｜R-3（上游归档字节级校验）**已闭合**（exit 0）｜R-4（S02 runtime-executable UAT 留痕）**部分闭合**——证据面已入库（`docs/evidence/S02-rerun.md/json`），但 `01-02-ASSESSMENT.md` 文件仍缺失｜R-5（FK 门槛偏差显式接受）**已闭合**｜R-6（CLI task 双臂缺口契约化 + R011/R012/R014 口径）**已闭合**（exit 2 + `task_single_arm_only` + Proof 回填）｜R-7（真机物理指标转实测）**开放**（环境不可达，合理登记项）｜R-8（需求状态一致性）**部分闭合**——六条 Proof 已回填，但 R006/R010=validated 与 R017=active 的同类证据不同状态口径未统一。

**DB 与切片证据不一致项（flag，均为台账级）**：① `01-01-ASSESSMENT.md` 仍为 `verdict: FAIL`，其唯一 FAIL 判据已被 S08 + 本轮实测证伪；② `01-00-ASSESSMENT.md` 仍为 `verdict: PARTIAL`，其未决项已被 S09 闭合；③ `01-02-ASSESSMENT.md` 文件不存在；④ S09 证据面未收录第五类故障，R013 Proof 未回填。

**诚实边界**：本轮全部复跑在 fallback 档（Python 3.10.12 / Drake 1.51.1）执行；GCS 数字为凸松弛口径；真机物理指标零证据。

## Verification Class Compliance
| Class | Planned Check | Evidence | Verdict |
| --- | --- | --- | --- |
| Contract | 模型回归测试（惯量逐轴比对 ≤ 0.5%、FK vs 官方 MDH < 1e-6 m、零位末端 0.8505 m、质量 7.776 kg、限位一致、自碰撞过滤正确）；实时层单测（安全七闸逐条、状态 age、错误码恢复流程）；CI 静态检查确保主链路不调用厂商解算接口。 | 本轮复跑 `python3 -m pytest -q tests/test_verify_env_contract.py tests/test_model_regression.py tests/test_mdh_fk.py tests/test_device_safety.py tests/test_device_recovery.py tests/test_device_state_machine.py tests/test_device_scripts.py` → **EXIT=0，170 passed**（含模型回归、MDH/FK、七闸正例+反例、状态机、错误码恢复、退出码契约 5 条断言）；`verify_env --model` 报 DoF=7 / 7.776 kg / 零位末端 0.8505 m / 7 条限位；自碰撞相邻过滤鉴别力负例存在；`python3 scripts/scan_forbidden_apis.py src tools scripts` → **EXIT=0，扫描 90 个代码文件 / 名单 13 项 / 0 命中**；`bash scripts/ci.sh fast` → **EXIT=0，PASS=17 SKIP=0 FAIL=0**（含 `tests/verify-env-contract` 恒跑项）。FK 门槛说明：官方 URDF `joint rpy=1.5708` 四舍五入导致固有残差 ≈2.2407e-6 m（`ZERO_Y_ROUNDING_BIAS_M` 单独钉死），真实 URDF 门槛放宽为 `<5e-6 m`、精确 ±π/2 变体仍 `<1e-6 m`、错误 alpha/d 表须 `>1e-3 m`（鉴别力保留）；该偏差已由 `docs/architecture/fk-tolerance.md` 正式声明并覆盖 01-01-PLAN.md / 01-CONTEXT.md 的旧 <1e-6 m 表述，常量唯一真相源在 `src/rm75_drake/kinematics/mdh.py`，并由 S09 回填 R002 Proof。 | PASS |
| Integration | 单臂真机完成一次带夹爪抓放的完整任务；两臂完成一次相对位姿固定的共同搬运；规划结果回放仿真做碰撞复检，逃逸数为 0；录制回放可复现。 | 真机两项 **UNMET（环境不可达，证据缺位非失败）**：`scripts/soak_test.py` / `measure_jitter.py` 均 `SKIP(reason=hardware_unreachable)`，真机 6 条指标全在 `unverified_metrics`。替代离线口径已复跑：`PYTHONPATH=src python3 -m rm75_drake.cli single --target-xyz 0.0 0.0 0.85 --output /tmp/rec.json` → **EXIT=0「碰撞复检逃逸数：0」**，紧接 `cli replay --recording /tmp/rec.json` → **EXIT=0「复检位形数：47」**；`python3 scripts/gen_golden.py --check` → **EXIT=0，pointwise_identical=True，722 点，tcp_mismatches=0，max_tcp_diff_m=0.000e+00**，三指纹绑定且「基线平移 ⇒ 非 0 退出」负例通过；`PYTHONPATH=src python3 scripts/coordinated_stats.py --samples 20 --json` → **EXIT=0，success_rate=1.0，solve_p95_s=0.2265（<3 s），zero_escape_rate=1.0，相对位姿残差 1.835e-08 m / 9.892e-08 rad**；`cli task --task tasks/pick_place.yaml` → **EXIT=0**。 | NEEDS-ATTENTION |
| Operational | 8 h 连续 pick-place 成功率 ≥ 95% 且无异常停机；100 Hz × 30 min P99.9 抖动 < 1 ms；拔网线/物体掉落/夹空/超限位/供电跌落五类故障注入均进入安全态并按文档流程恢复；急停响应 < 100 ms。 | 本轮复跑 `python3 scripts/fault_inject.py --json` → **EXIT=0**，5 条 case（link_down / object_dropped / gripper_miss / joint_overlimit / **power_supply_drop**）全部 `ok=true / recovered=true / safe_state=fault / final_state=ready`，`failed=[]`，`physical_faults_verified=false`（物理未验证的诚实标注保留）；`power_supply_drop` 以帧级系统错误码 `errCode`（非 0 且非 -3）为载体，`state_machine.system_error_code()` 为判定与恢复共用的单一纯函数，`SAFE_STATE_BY_REASON[power_supply_drop] == DeviceState.FAULT`；`runbooks/s03_rm_device_bringup.md` §6/§6.1/§10 含五类故障与供电跌落真机复验步骤。**Round-0 的 FAIL（第五类未实现）已被更新的通过证据证伪。** 其余三项真机类判据 **UNMET（证据缺位）**：8 h 长稳、100 Hz×30 min P99.9 抖动 <1 ms、急停 <100 ms，均无真机且未伪造任何数据。 | NEEDS-ATTENTION |
| UAT | 无独立人工验收项：全部验收为可执行命令与客观数值门槛（误差 mm、成功率 %、P95 s、抖动 ms、接口调用数 0），由 CI 与 HIL 脚本复算，无需人工主观判断。 | 验收机制成立且可复算：本轮 `bash scripts/ci.sh fast` → **EXIT=0，PASS=17 SKIP=0 FAIL=0**；评审方独立复跑全量 `pytest -q -m "not slow"` → **1393 passed / 3 skipped / 44 deselected，0 failed**（高于 S09 的 1369/3，无回归）；退出码契约实测 `python3 tools/verify_env.py --model --joint-limits /nonexistent/path.yaml` → **EXIT=2** 且 stderr 点名 `/nonexistent/path.yaml`（与 `01-01-UAT.md:47` 逐字一致）；`PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/handover.yaml` → **EXIT=2 + reason `task_single_arm_only`**；`python3 tools/fetch_upstream.py --verify` → **EXIT=0「校验通过：2 个资产」**，两条 sha256 与 MANIFEST + versions.lock 三方逐字一致。残余台账缺口：**S01 的 `01-01-ASSESSMENT.md` 仍为 `verdict: FAIL`（判据已被 S08 证伪）**、**S02 缺 `01-02-ASSESSMENT.md`**、S00 的 ASSESSMENT 仍为 PARTIAL（未决项已被 S09 闭合）、S09 证据面未收录第五类故障且 R013 Proof 未回填。 | NEEDS-ATTENTION |


## Verdict Rationale
三位并行评审聚合：Reviewer A（需求覆盖）NEEDS-ATTENTION（19 条需求 7 COVERED / 12 PARTIAL / 0 MISSING，round-0 的 R-1/R-2/R-3/R-5/R-6 已实质闭合，R-4 与 R-8 仅部分闭合，R-7 因真机不可达开放）；Reviewer B（跨切片集成）PASS（13 条原始声明边界 + S08→S09 修复边界共 14 条全部 HONORED、0 GAP、无孤立切片，四个真实代码接缝可点验）；Reviewer C（验收与判定）NEEDS-ATTENTION（round-0 的两项判定性失败均已被更新的通过证据证伪——verify_env --joint-limits 非法路径实测 EXIT=2、第五类故障「供电跌落」5/5 全链恢复至 READY；全量 1393 passed / 3 skipped、ci.sh fast PASS=17 SKIP=0 FAIL=0 无回归；剩余全部为真机不可达导致的证据缺位，外加 S01 台账陈旧 FAIL 与 S02 缺 01-02-ASSESSMENT.md 两处记录缺口）。按聚合规则：无 FAIL ⇒ 不降级为 needs-remediation；存在 NEEDS-ATTENTION ⇒ 判定为 needs-attention。判定性依据是「证据缺位」而非「观察到的失败」：本轮所有可执行判据均已复跑且无一失败，故不进入再一轮整改切片。
