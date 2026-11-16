# rm_regression 体验基准与一键回归说明（段级 verdict · 退出码 0/3/4）

> 面向"第一次在这台机器上跑一键回归的人"：照着本文，你能在 **10 分钟内**
> 跑完全部回归段、读懂每一段的判定（**pass / blocked / fail / skipped**）、
> 知道**一段 blocked 不等于整体失败**，并且知道**哪些数字是离线实测、
> 哪些还等着真机回填**。
>
> 关联切片：M001-3kzz40 / S09（真机基线、体验基准与一键回归）；
> 关联代码：`dualarm_ws/src/rmctl/rmctl/regression/`
> （`segments.py` 段级判定模型 / `orchestrator.py` 编排器 /
> `experience.py` E1–E4 / `perf_baseline.py` S1–S5 /
> `baselines.py` 真机基线投影）；
> 关联 CLI：`scripts/rm_regression.py`（薄入口）、`rmctl doctor`；
> 关联文档：`docs/rm_planner规划服务说明.md`（规划服务）、
> `docs/rm_process单臂工艺闭环说明.md`（工艺闭环 + §7 真机五步回填）、
> `docs/rm_hw_bridge硬件适配层说明.md`（硬件适配层 + §6 doctor）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 / 文件 |
|---|---|
| 跑一次一键回归（全部段） | `python3 scripts/rm_regression.py` |
| 跑一次并指定报告路径 | `python3 scripts/rm_regression.py --out build/reports/s09_regression.json` |
| 只跑某一段 | `python3 scripts/rm_regression.py --only experience` |
| 在别的工作区跑 | `python3 scripts/rm_regression.py --ws /path/to/ws` |
| 看总体报告 | `build/reports/s09_regression.json`（顶层 `status` / `reason` / `next_action` / `evidence` + `segments[]`） |
| 看体验基准表（E1–E4） | `build/reports/s09_experience.json` + `.md` |
| 看性能基线表（S1–S5） | `build/reports/s09_perf_baseline.csv` + `.md` |
| 看真机基线汇总表 | `build/reports/s09_baselines.json` + `.md` |
| 看体检（机器可读） | `python3 -m rmctl doctor --json --report build/reports/s09_doctor.json` |

> **诚实分层（贯穿全文）**：本轮全部证据为**离线层** —— 真实规划内核
> （厂商 tesseract 0.35.0.8）+ 真实四道校验 + 真实进程级 `SIGKILL`
> + `rm_executor` **fake** 运动回放。**真机版未实证**（属真机窗口），
> 口径见 **§8**。

---

## 1. 段级语义（pass / blocked / fail / skipped）

一键回归不是「一串布尔值」，而是**若干段**（segment）各自给出一个
**段级 verdict**。四种 verdict **严格区分，不可混用**：

| verdict | 含义 | 判据 | 典型场景 |
|---|---|---|---|
| `pass` | 段按期望跑通 | 段内全部断言通过，或该段本身**执行成功**（结果落在"该段职责之外"的项逐条列入 evidence） | 黄金场景集全部符合 `expect_status`；doctor 体检执行成功 |
| `blocked` | **外部条件未满足** | 缺外部产物（`build/model`、规划配置、工艺规格、S08 报告、真机不可达……） | 干净机器上没跑 `rmctl init`；真机窗口还没到 |
| `fail` | **代码或契约回归** | 本该跑通的段没跑通 / 期望值不符 / 报告与退出码自相矛盾 / 段超时或被杀 | 规划结果与黄金场景不一致；子进程超时；doctor 报出硬红线 |
| `skipped` | **本切片范围外** | 该段/该场景在当前工位条件下不适用 | 双臂场景 S4/S5 在单臂工位 |

**关键纪律**

* **`blocked` ≠ 失败**：一段 blocked 只说明"外部条件还没满足"，
  **一段 blocked 不等于整体失败**。整体退出码由聚合规则决定（见下）。
* **缺产物 ≠ 回归**：编排器在跑脚本**之前**做**依赖预检**；
  缺外部产物 → 该段直接 `blocked`（带可粘贴的下一步），
  **不跑脚本、不产出任何数字**。
* **禁止静默**：报告 `status` 未知、报告缺失/不可解析、退出码与报告
  自相矛盾 → 一律 `fail`（契约回归），**绝不**静默当 pass。

### 1.1 与 `rm_executor.contract` 的映射（status / 退出码）

段级 verdict 的 status 与退出码**唯一来源** =
`rm_executor.contract`（`STATUS_OK` / `STATUS_BLOCKED` / `STATUS_ERROR`
与 `EXIT_CODES`）。`rmctl.regression.segments` **不自带第二套词表**，
段结果构造**必须**经 `rm_executor.contract.result_dict`。

| 段级 verdict | 执行层 status | 进程退出码 | 含义 |
|---|---|---|---|
| `pass` | `ok` | `0` | 段通过 |
| `blocked` | `blocked` | `3` | 外部条件未满足 |
| `fail` | `error` | `4` | 真回归 |
| `skipped` | `ok` | `0` | 范围外（不影响聚合） |

### 1.2 整体聚合规则

| 聚合前提 | 顶层 `status` | 顶层退出码 |
|---|---|---|
| 有任何一段 `fail` | `error` | **4** |
| 无 fail、但有 `blocked` 段 | `blocked` | **3** |
| 无 fail、无 blocked（`skipped` 不影响） | `ok` | **0** |

> **干净机器的正常结局是 3**：没有 `build/model`、没有真机窗口时，多段
> `blocked` 是**预期**，退出码 3 **不是**失败信号；出现 4 才需要定位到
> 具体那个 `fail` 段（段级 `reason` 里写明了脚本名与失败点）。

---

## 2. 一键回归用法

### 2.1 命令

```bash
# 全部段
python3 scripts/rm_regression.py

# 指定报告路径
python3 scripts/rm_regression.py --out build/reports/s09_regression.json

# 只跑指定段（逗号分隔；顺序按注册顺序）
python3 scripts/rm_regression.py --only experience,doctor

# 在别的工作区跑（工作区 = 含 workcell/ 与 build/model 的目录）
python3 scripts/rm_regression.py --ws /path/to/ws --out /tmp/s09_regression.json
```

### 2.2 段 id 一览（`--only` 的取值）

| 段 id | 内容 | mode | 依赖外部产物 |
|---|---|---|---|
| `gripper_offline` | 夹爪离线证据链（开合 100 次 / 六类判定 / 互锁） | `fake` | 无 |
| `process_offline` | 单臂工艺离线证据链（成功圈 / 故障注入 / 回放） | `fake` | `build/model`、`workcell/tasks/pick_place.yaml` |
| `planner_golden` | 规划黄金场景集（S07 场景集逐条比对 `expect_status`） | `offline` | 无 |
| `planner_acceptance` | 规划三条验收（确定性 / 崩溃隔离 / 超时） | `fake` | `build/model`、`workcell/planner/pipeline.yaml`、`workcell/planner/profiles.yaml` |
| `perf_baseline` | 性能基线表 S1–S5（只投影 S08 既有报告，不重测） | `offline` | `build/reports/s08_planner_perf.json` |
| `experience` | 体验基准 E1–E4 实测 | `offline` | 临时工作区 + S06 trace（缺失则对应指标 blocked） |
| `doctor` | 体检（10 类覆盖矩阵，机器可读） | `offline` | `workcell/rm_workcell.yaml` |
| `baselines` | 真机基线汇总表（四类逐行投影） | `offline` | `workcell/baselines/*.yaml` |

> `--only` 传未知段 id → argparse 拒绝，**退出码 2**（参数错误），
> 不静默跑空。

### 2.3 退出码语义

| 退出码 | 含义 | 现场动作 |
|---|---|---|
| `0` | 无 fail 段（可能仍有 blocked 段，逐条列出） | 看报告确认 blocked 明细 |
| `2` | 参数错误（未知 `--only` 段 id 等） | 修正命令行 |
| `3` | 无 fail、但有 blocked 段（**干净机器 / 无真机窗口的正常结局**） | 按 §2.4 清单补外部条件 |
| `4` | 存在 fail 段（**真回归**） | 定位该段的 `reason` / `evidence` |

### 2.4 常见 blocked 原因清单

| 段 | blocked 原因 | 可粘贴的下一步 |
|---|---|---|
| `process_offline` / `planner_acceptance` | 缺 `build/model` | `python3 -m rmctl init` |
| `planner_acceptance` | 缺 `workcell/planner/pipeline.yaml` / `profiles.yaml` | 从仓库复制规划配置到工作区 |
| `process_offline` | 缺 `workcell/tasks/pick_place.yaml` | 补工艺规格 |
| `perf_baseline` | 缺 `build/reports/s08_planner_perf.json` | `python3 scripts/rm_planner_perf.py` |
| `doctor` | 工作区未初始化（缺 `workcell/rm_workcell.yaml`） | `python3 -m rmctl init` |
| `baselines` | 四类真机基线未全部 `measured` | `python3 -m rmctl baseline scaffold` 生成占位（`status=pending`），真机窗口回填后改 `measured` |
| `experience` | E 指标数据源缺失（模型 / S06 trace / 变体 URDF 装不上） | 按报告里该指标的 `evidence` 逐条补齐；缺源的指标记 `null` + `blocked`，**不记 0** |
| 任意段 | 厂商库不可导入（GLIBCXX 不足）→ `model:load` 失败并级联跳过模型相关检查 | `source scripts/rmenv.sh`（提供 `GLIBCXX_3.4.31+` 的 libstdcxx） |

---

## 3. 报告族清单

| 文件 | 产出者 | 内容 |
|---|---|---|
| `build/reports/s09_regression.json` | `scripts/rm_regression.py` | **顶层**：`status` / `reason` / `next_action` / `evidence` + `mode` / `exit_code` / `verdict` + `segments[]` |
| `build/reports/s09_experience.json` / `.md` | `rmctl.regression.experience` | E1–E4 逐指标（值 + mode + status + 目标 + 出处） |
| `build/reports/s09_perf_baseline.csv` / `.md` | `rmctl.regression.perf_baseline` | S1–S5 逐套件一行（S4/S5 = `not_applicable`） |
| `build/reports/s09_baselines.json` / `.md` | `rmctl.regression.baselines` | 四类真机基线逐行（`status` 逐字取自 YAML） |
| `build/reports/s09_doctor.json` | `rmctl doctor --json --report` | 体检检查项 + 覆盖矩阵 + 退出码 |
| `build/reports/s09_segment_*.json` | 各段 | 段级明细（脚本报告原文 / 投影结果） |

每段结果字段固定为：`id` / `verdict` / `status` / `reason` / `next_action` /
`evidence[]` / `mode` / `report_path`。

---

## 4. 自检覆盖矩阵（≥ 8 类）

`rmctl doctor` 的机器可读报告带 `coverage_matrix[]`，**10 类**逐类打勾
（设计 §6.2 点名的 7 类 + 3 类补齐）：

| 类别 | label | 映射到的检查项 | 当前覆盖状态 |
|---|---|---|---|
| `network` | 网络连通 | `network:plan` | **covered**（未回填 → NEEDS_HUMAN，仍算有检查项） |
| `power` | 供电与共地 | `power:supply` | **covered** |
| `gripper` | 夹爪 | `hw:gripper` | **covered** |
| `zero` | 零位基线 | `zero:baseline` | **covered** |
| `fk` | FK 一致性 | `fk:self_vs_tesseract` | 随 `model:load` 级联（模型不可用 → `covered=false` + 缺口说明） |
| `limits` | 限值 | `limits:consistent` | 随 `model:load` 级联 |
| `controller` | 控制器状态 | `controller:state` | **covered** |
| `deps_model` | 依赖与模型 | `dep:nanobind` / `model:load` / `model:*` | **covered**（模型不可用 → 缺口说明） |
| `collision` | 碰撞（零位自碰撞） | `collision:home` | 随 `model:load` 级联 |
| `hardware_layer` | 硬件适配层（SDK / 符号 / 白黑名单 / 限值源） | `hw:sdk` / `hw:symbols` / `hw:contract` / `hw:limits_source` | **covered** |

**规则**：映射不到的类别**显式**标 `covered=false` + 缺口说明 + 可粘贴的
修复命令 —— **不会**被标成「通过」（厂商库不可导入导致 `model:load` 失败时，
FK / 碰撞 / 限值三类被级联跳过，见 §8 的诚实分层）。

检查项**条数**口径：模型可用时 **21 条**；模型未物化时 **17 条**
（4 条模型相关项被级联跳过）。退出码口径 **0 / 3 / 4**（与全项目一致），
详见 `docs/rm_hw_bridge硬件适配层说明.md` §6 与 §6.1。

---

## 5. E1–E4 与 S1–S5：引用出处与 mode 标注

### 5.1 出处

* **体验场景集 E1–E4**：设计 §6.9「体验与性能基准」表；
  **目标值**取自设计 §6.2「体验指标（可测、带目标值）」表。
  机器可读声明（**唯一来源**，代码只读）：
  `workcell/bench/experience_e1_e4.yaml`。
* **性能场景集 S1–S5**：设计 §6.9 性能场景集定义（S1 单臂自由空间 100 组 /
  S2 笛卡尔 20 组 / S3 绕障 20 组 / S4 双臂共存 20 组 / S5 双臂相对约束 10 组）。
  机器可读声明（**唯一来源**）：`workcell/bench/perf_scenarios.yaml`。

### 5.2 体验场景（E1–E4）

| 场景 | 名称 | 测什么 | 关键指标 |
|---|---|---|---|
| E1 | 接入一款新机型（换 URDF） | 上手成本 | `t2fm_s`（目标 ≤ 1800 s）、`command_count`、`config_lines_changed`（0）、`code_lines_changed`（0）、`error_count`、`doctor_check_count`（≥ 8） |
| E2 | 交付一条新工艺（新任务模板） | 编排成本 | `template_lines`、`command_count`（≤ 1）、`first_success_rate`（≥ 0.95）、`new_task_first_run_ok` |
| E3 | 现场故障恢复 | 恢复能力 | `mttr_proxy_s`（机器下界）、`mttr_human_s`（**人工**）、`diagnosability_rate`（= 1.0）、`restart_required` |
| E4 | 日常操作（改参数 / 跑一次 / 复现一次） | 操作效率 | `command_count`（≤ 3）、`operation_duration_s` |

### 5.3 性能场景（S1–S5）

| 套件 | 名称 | 组数 | 状态 |
|---|---|---|---|
| S1 | 单臂自由空间 | 100 | applicable |
| S2 | 笛卡尔目标 | 20 | applicable |
| S3 | 绕障 | 20 | applicable（**注意**：黄金场景集里绕障场景期望结局是"规划失败 / 超时"，本表记录的是这些请求的端到端耗时与符合期望率，**不得**据此宣称绕障成功路径达标） |
| S4 | 双臂共存 | 20 | **not_applicable**（本里程碑是单臂 7 DOF，需双臂工位；**不生成任何假数字**） |
| S5 | 双臂相对约束 | 10 | **not_applicable**（同上） |

### 5.4 mode 标注含义（每个指标都带）

| mode | 含义 | 数值来源 |
|---|---|---|
| `offline` | **离线实测**：有真实计算 / 真实规划，**无真机** | 本切片脚本实跑 |
| `human_pending` | **只能人工测**：当前 `value=null` + `status=pending` | 人工在真机窗口回填（当前仅 `E3.mttr_human_s`） |
| `fake` | 假执行 / 桩数据（如 `rm_executor` 的 fake 运动回放） | 段级 mode 标注 |
| `blocked` | **该指标本轮没测出来**：数据源缺失 → 值留 `null` | 缺源即 `null`，**不记 0 冒充** |

> **绝不冒充**：离线数字**不得**当成真机结论；`mode=live` 的数字本切片
> **不存在**（声明表里禁止出现 `mode=live`）。

---

## 6. 真机五步回填与本切片的关系

真机回填的执行脚本来源是
`docs/rm_process单臂工艺闭环说明.md` **§7 真机回填手册**（五步）：

| 步骤 | 动作 | 回填到哪 | 本切片如何自动反映 |
|---|---|---|---|
| 1 | 确认单臂可达（`rmctl connect`） | `workcell/baselines/network_plan.yaml` 的 `status` | `baselines` 段把该类从 `blocked` 变 `measured` → 段判定随之变化；`doctor` 的 `network:plan` 变 PASS |
| 2 | 回填夹爪标定四字段 + error_codes status | `workcell/rm_workcell.yaml::gripper`、`workcell/safety/error_codes.yaml` | `doctor` 的 `hw:gripper` 变 PASS；`limits:measured` 随 `measured_limits.yaml` 变 PASS |
| 3 | 夹爪自检（探针 + 100 次开合） | `workcell/baselines/zero_baseline.yaml` 等 | `baselines` 投影表逐行 `status` 逐字取自 YAML，**表不改写结论** |
| 4 | 真机 50 圈（核心验收） | `workcell/baselines/version_matrix.yaml` | 同上；E2 的 `first_success_rate` 在真机窗口可换口径 |
| 5 | 故障注入（人为放空工件 → 重抓 → 用尽转人工） | `workcell/baselines/measured_limits.yaml` + 人工 MTTR | `E3.mttr_human_s` 由 `human_pending` → 人工回填 |

**回填后如何自动更新**

* `baselines` 段是 `workcell/baselines/*.yaml` 的**只读投影**：
  判定来源唯一 = `rmctl.baseline.validate_all`；表的 `status` **逐字**取自
  YAML（另存 `yaml_status` 原值供反查）。**四类全部 `measured` → 段 `pass`**；
  任一类缺文件 / YAML 语法错误 / 缺 `status` 字段 → 该类 `blocked`；
  schema 违规（非 `measured` 却填了数值）→ 段 `fail`。
* E 指标：`experience` 段每次跑都重新实测；数据源齐备则出实测值，
  缺源则该指标 `blocked` + `null`。**真机未实证项一律标 blocked/pending，
  不以离线数字冒充。**

---

## 7. 干净机器从零复现

```bash
# 1) 环境（含零编译交付的 libstdcxx 依赖）
source scripts/rmenv.sh

# 2) 物化 build/model（URDF 绝对路径 + manifest.json）
python3 -m rmctl init

# 3) 跑一键回归
python3 scripts/rm_regression.py
```

### 7.1 零编译交付的 libstdcxx 依赖（MEM012 / MEM030）

厂商 nanobind wheel（`tesseract_robotics` 0.35.0.8）需要
**`GLIBCXX_3.4.31+`**；常见开发机系统 libstdc++ 只有 3.4.30。
零编译路线的约定：

* `scripts/rmenv.sh` 按 `$ROOT/.deps/libstdcxx` → `$ROOT/../../.deps/libstdcxx`
  → `$ROOT/../.deps/libstdcxx` **只读**回退（同一仓库其它 checkout 的
  `.deps/`，worktree 布局）；
* 找不到时**打印警告**而非静默失败；请安装 conda-forge 的 `libstdcxx-ng`
  （≥ 3.4.31）放到上述任一路径；
* `build/` 被 `.gitignore`，**每个新 worktree 必须先 `rmctl init`**，
  否则 URDF 相对路径 mesh 解析失败 → `model:load` 失败 → 模型相关的
  4 条 doctor 检查被级联跳过（体检从 21 条塌到 17 条，MEM030）。

### 7.2 预期结局

* 干净机器（已 `rmctl init` 但**无真机窗口**）：多数段 `pass`，
  `baselines` / `doctor` 段 `blocked`（真机未回填）→ **退出码 3**；
* 未 `rmctl init`：依赖模型的段 `blocked` → 仍是 **退出码 3**；
* **出现 4 才是回归**，必须定位到具体 `fail` 段。

---

## 8. 口径声明（诚实分层）

**本轮已实证（离线范围内）**

* 一键回归端到端：8 段全部按段级语义给出判定，退出码 0 / 3 / 4 可复现；
* E1–E4 的**可自动测部分**：15 项离线实测 + 1 项人工 pending；
* S1–S5 基线表：S1–S3 有投影（S4/S5 = `not_applicable`）；
* doctor：21/17 条检查项 + 10 类覆盖矩阵 + 机器可读输出。

**本轮未实证（真机窗口）**

* 真机版 E1–E4（`mode=live` 的任何数字）；
* `E3.mttr_human_s`（人工恢复耗时）；
* 四类真机基线的 `measured` 结论；
* 双臂场景（S4/S5）。

> 离线数字**不得**当成真机结论；真机未实证项一律标 `blocked` / `pending`。

---

## 9. 已知限制

1. **`perf_baseline` 只投影、不重测**：S08 报告缺失 → 该段 `blocked`；
   S08 报告 schema 变化 → 段 `fail`（并指名缺哪个键）。
2. **S3 绕障口径**：黄金场景集里绕障场景期望结局是"规划失败 / 超时"，
   因此 S3 行记录的是这些请求的端到端耗时与符合期望率，
   **不等同于**"绕障成功规划"的延迟基线。
3. **`doctor` 检查项条数随模型可用性变化**（21 / 17），
   `coverage_matrix` 里被级联跳过的类别标 `covered=false`。
4. **一键回归耗时**：无 `build/model` 时为**秒级**；以真实模型跑规划段时
   为 **300–600 s 量级**（各段超时上限见 `orchestrator.py` 的 `SegmentSpec.timeout_s`，
   超时 → 该段 `fail` 并指名脚本与超时秒数）。
