# S07 工程化与产品化作业手册（RM75-D）

> **适用范围**：S07「工程化与产品化」——会话产物（session v1）、golden-file 回归基线、
> 厂商解算接口**运行时零调用**审计、资产与许可门禁、任务级长稳载体、CI 三档接线。
>
> **本文件当前状态**：由 **S07-T07** 落笔（四段式运维口径齐备：
> 健康信号 / 故障信号 / 恢复流程 / 监控缺口），并覆盖门禁三档、golden 基线、
> 资产与许可、运行时审计、长稳、未验证清单六节。
>
> **诚实前提**：本仓库当前**没有真机、没有实测数据**。真机 8 h 任务级长稳、
> 真机全链路录制**均未验证**，由两个**唯一真相源**逐条列出：
> `rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS` 与
> `rm75_drake.device.hardware_gate.UNVERIFIED_HARDWARE_METRICS`。
> **绝不用合成数据 / 离线演练冒充真机达标**（沿用 K001 硬件门控纪律）。

---

## 0. 真相源一览

| 内容 | 真相源 |
|---|---|
| 会话产物 schema（独立 schema v1） | `rm75_drake.regression.session`（只读消费 `rm75_drake.recording` / `rm75_drake.device.run_log`） |
| golden 基线文件 | `tests/data/golden/single_arm_pick_place.json`（**唯一**基线入口） |
| golden 生成 / 校验器 | `scripts/gen_golden.py`（`--check` 只比对、不写盘） |
| golden 逐点比较器 | `rm75_drake.regression.golden.compare_golden` / `repo_fingerprints` |
| 禁用名单（静态 + 运行时同源） | `versions.lock:forbidden_apis`（`rm_algo_*` / `rm_movej*` 等厂商解算与运动下发接口） |
| 白名单（允许的非解算接口） | `versions.lock:allowed_apis`（如 `rm_movej_canfd`、`rm_get_current_joint_current`） |
| 静态禁用扫描器 | `scripts/scan_forbidden_apis.py`（退出码 0 = 未命中） |
| 运行时调用审计 | `rm75_drake.device.sdk.SdkAudit` / `audit_snapshot`（`forbidden_call_count` / `non_whitelist_call_count`） |
| 运行时审计 CLI 计数 | `python3 -m rm75_drake.cli exec --json`（`sdk_audit` 段） |
| 许可登记表 | `licenses/registry.yaml`（含 `commit` / `locked` 字段） |
| NOTICE 成品 | 仓库根 `NOTICE`（**只能**由 `tools/gen_notice.py` 从登记表派生） |
| 上游资产三方交叉校验 | `tools/verify_env.py --upstream`（MANIFEST ↔ `versions.lock:upstream` ↔ registry） |
| 许可门禁 | `tools/verify_env.py --licenses`（包集覆盖 / 必填字段 / commit 声明） |
| 长稳载体 | `scripts/long_run.py`（任务级循环；真机不可达 ⇒ SKIP，退出码仍为 0） |
| 未验证（任务级）指标 | `rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS` |
| 未验证（硬件级）指标 | `rm75_drake.device.hardware_gate.UNVERIFIED_HARDWARE_METRICS` |
| CI 三档唯一实现 | `scripts/ci.sh`（fast / medium / nightly） |

---

## 1. 门禁三档：fast / medium / nightly（S07-T06 接线）

CI 逻辑**只在** `scripts/ci.sh` 里；`.github/workflows/ci.yml` 是薄封装，
因此用 Actions、内网 cron 还是远程 runner，检查项都**不会分叉成两套**。

| 档位 | 目标耗时 | 包含内容 | 是否恒跑 |
|---|---|---|---|
| **fast** | < 10 min | lint（ruff，缺失时退回 `py_compile`）；骨架单测（路径层 / 锁 / 禁用名单扫描器）；`versions.lock` 自洽性；**禁用接口静态扫描**（`forbidden/apis`）；模型回归（Drake 不可用时显式 SKIP）；网格工具；视觉网格（缺上游归档时 SKIP）；规划回归（Drake 不可用时 SKIP）；**设备层与安全闸**（零硬件，恒跑）；**双臂几何与标定**（零硬件，恒跑）；**双臂紧协调**（零硬件，恒跑）；**任务原语与感知**（零硬件，恒跑）；**会话录制回归** `tests/regression`（零硬件，恒跑）；**上游资产与许可门禁** `assets/lock`（零硬件，恒跑）；**上游获取与许可门禁测试** `tests/upstream`（零硬件，恒跑） | 标注「恒跑」的项**永不 SKIP**；失败即整体失败（`run_check` 语义） |
| **medium** | fast + 追加 | 依赖锁一致性（`constraints` ↔ `versions.lock`；无本地 PyPI 缓存时明确 SKIP）；`package://` 解析测试；wheel 离线归档 sha256 校验（归档缺失时 SKIP） | 与 fast 相同 + 归档类可 SKIP |
| **nightly** | medium + 追加 | **规划统计真实调用**（`scripts/plan_stats.py`，超时按失败处理但打印实测值）；规划链路自检（`verify_env --plan`）；**golden-file 回归**（`gen_golden.py --check`，逐点一致，失败即整体失败）；**长稳离线演练**（`long_run.py --dry-run`，**不是真机长稳结论**） | golden 与长稳演练在 nightly 档**不允许 SKIP** |

### 1.1 SKIP 语义（**SKIP ≠ 通过**）

* 每一档的输出行首标签只有三种：`[PASS]` / `[SKIP]` / `[FAIL]`；
  **任一 `[FAIL]` ⇒ 整体退出码 1**；`[SKIP]` **不算通过**，
  只表示「**该检查在本环境不可执行**（缺 Drake / 缺上游归档 / 缺 PyPI 缓存）」，
  **不得**在验收口径里被当作「已验证」。
* 允许 SKIP 的只有**明确列举**的项：Drake 不可用（模型 / 规划 / nightly 自检）、
  上游归档缺失（视觉网格、wheel 归档）、无本地 PyPI 元数据缓存（lock/constraints）。
* **零硬件也能回归的项一律恒跑**（设备层 / 双臂 / 任务原语 / 会话录制 /
  资产许可 / 上游获取）：这些项被降级成 SKIP 即视为门禁被削弱。
* 退出码口径：`0` = 全部检查通过（SKIP 项不算失败）；`1` = 至少一项失败；
  `2` = 参数或运行环境错误。

```bash
bash scripts/ci.sh fast
bash scripts/ci.sh medium
bash scripts/ci.sh nightly
```

---

## 2. golden 基线：位置、生成、校验与**失效处置**（S07-T02 / S07-T06）

### 2.1 基线文件与生成命令

* **基线文件**：`tests/data/golden/single_arm_pick_place.json`
  （**唯一**基线入口；由 `scripts/gen_golden.py` 写入，不手工编辑）。
* **生成**（写盘，**只在人显式确认后**执行）：

```bash
PYTHONPATH=src python3 scripts/gen_golden.py
```

* **校验**（CI 用；**不写盘**）：

```bash
PYTHONPATH=src python3 scripts/gen_golden.py --check
```

`--check` 的退出码：`0` = 逐点一致；`1` = 指纹漂移（**基线失效**）或逐点漂移；
`2` = 用法错误。`--json` 输出机器可读结论
（`fingerprints_match` / `num_points` / `max_tcp_diff_m` / `num_tcp_mismatches` /
`pointwise_identical` / `gripper_match` / `joint_max_diff_rad` / `ok`）。

### 2.2 比较口径

* 「逐点一致」= **点数一致** 且 **超容差的 TCP 点数 == 0** 且 **夹爪调度表完全一致**；
  逐点关节角偏差以 `joint_max_diff_rad` 单独报出（供人工判读）。
* 末端位置用 `rm75_drake.kinematics.mdh` 的**理论 MDH 正解**复算
  （纯 numpy、零 pydrake），**不**与 Drake 实体 FK 混在同一口径里比。

### 2.3 **基线失效条件**（三指纹漂移）

基线携带三枚指纹，绑定**当前仓库文件**：

| 指纹 | 绑定对象 | 漂移意味着 |
|---|---|---|
| `urdf_sha256` | RM75 URDF 模型 | 模型本体改了 ⇒ 基线失效 |
| `scenario_sha256` | `scenarios/single_arm.yaml` | 场景真相源改了 ⇒ 基线失效 |
| `task_sha256` | `tasks/pick_place.yaml` | 任务文件改了 ⇒ 基线失效 |

任一指纹不一致 ⇒ 比较器判 **基线失效**（`fingerprints_match = False`），
**此时不得判定「通过」**——必须先走下面的处置流程。

### 2.4 **基线失效处置流程（禁止直接重录掩盖回归）**

**处理顺序固定为：先判失效 → 再查代码 → 最后才谈重录。**

1. **判失效**：跑 `gen_golden.py --check`（或 `--json`），读
   `fingerprint_details`：确认是三指纹中的哪一枚漂移 ⇒ 定位是
   **模型 / 场景 / 任务**哪一层变了。
2. **查代码**：漂移若是**预期内**的变更（模型升级、场景换目标、任务加原语），
   则**先在变更说明里写清为什么变**，再进入第 3 步；
   若是**非预期**漂移 ⇒ **这是回归**，回到代码查是不是把算法改坏了。
3. **重录基线（仅在第 2 步确认为预期变更后）**：
   * 重录**必须**与旧基线**逐点对比并把差异写进变更说明
     （`git diff` 旧/新基线文件）；
   * **禁止**用「直接重录基线」把红灯变绿——那等于把回归契约删掉。
   * 指纹漂移**且**逐点也变 ⇒ 说明行为真的变了，必须有人签字；
   * 指纹漂移**但**逐点完全一致 ⇒ 只是模型/场景/任务的文件级变更，
     可安全重录（逐点一致性仍然守着行为）。
4. **复跑门禁**：`gen_golden.py --check` 退出码 0，且 nightly 档整档退出码 0。

---

## 3. 资产与许可更新流程（S07-T04 / S07-T06）

### 3.1 流程（顺序不可改）

```bash
# 1) 改登记真相源：licenses/registry.yaml
#    - name / version / license / source / redistributable / usage_note（必填）
#    - commit: <40 位 hex>  ⇒ 上游资产**已锁定**
#    - commit: null + locked: false ⇒ **未锁定例外**（方法论/范式参考类资产）
#    - locked: true ⇒ commit 不得为 null

# 2) 重新生成 NOTICE 成品（NOTICE 只能是 registry 的派生物，禁止手工编辑）
PYTHONPATH=src python3 tools/gen_notice.py --out NOTICE

# 3) 跑三方交叉校验 + 许可门禁
PYTHONPATH=src python3 tools/verify_env.py --upstream --licenses

# 4) CI 的一致性校验（NOTICE 与登记表逐字一致）
PYTHONPATH=src python3 tools/gen_notice.py --check
```

### 3.2 上游 commit 锁定与未锁定例外的登记方式

* **已锁定**：条目写 `commit: <40 位 hex>` 且 `locked: true`；
  该 commit 必须与 `versions.lock:upstream` 段登记的 commit 一致
  （`verify_env.py --upstream` 做 **MANIFEST ↔ versions.lock ↔ registry 三方交叉校验**）。
* **未锁定例外**：`commit: null` **且** `locked: false`，
  并在 NOTICE 的「commit 未锁定例外」段落被列出。
  **不得编造 commit**；未锁定资产按**最保守口径**处理（默认禁止再分发）。
* 登记表与 `versions.lock:packages` 的关系：许可门禁要求 registry 的**包集包含**
  `versions.lock:packages` 的全部条目（含传递依赖，如 `mosek` 显式登记）。

### 3.3 解包环节 sha256 校验

上游归档的**解包环节**（`scripts/extract_upstream.py` / `tools/fetch_upstream.py`）
对每个文件做 **sha256 校验**：校验不过 ⇒ **解包失败并显式报错**，
**不**产出「看起来能用」的资产目录，也不把未校验资产登记进 registry。

### 3.4 失败信号

`verify_env.py --upstream --licenses` 退出码非 0 时，按消息里的**点名项**修：

* 「包集未覆盖」⇒ registry 缺该包（含传递依赖）⇒ 补条目并重生成 NOTICE；
* 「必填字段缺失」⇒ 按 `REQUIRED_ENTRY_FIELDS` 补齐；
* 「commit 未声明 / 不一致」⇒ 锁定或按未锁定例外登记（**不要编造**）；
* `gen_notice.py --check` 报不一致 ⇒ **重跑生成**（不要手改 NOTICE）。

---

## 4. 运行时厂商接口审计口径（S07-T03）

「主链路零调用厂商自带的运动学/轨迹求解接口」这件事有**双证**：

| 证 | 手段 | 真相源 | 判据 |
|---|---|---|---|
| **静态**那一证 | `scripts/scan_forbidden_apis.py src tools scripts` | `versions.lock:forbidden_apis` | 退出码 0 = **未命中**；1 = 命中；2 = 运行期错误 |
| **运行时**那一证 | `SdkAudit`（`rm75_drake.device.sdk`）+ `cli exec --json` 的 `sdk_audit` 段 | 同一份 `versions.lock:forbidden_apis` | `forbidden_call_count == 0` |

### 4.1 `forbidden_apis` 与 `allowed_apis` 的语义差别

* **`forbidden_apis`（禁用名单）**：主链路**一律不得调用**的厂商接口 =
  厂商**运动/轨迹下发**接口（`rm_movej` / `rm_movep` / `rm_movec` / `rm_moves` /
  `rm_movej_follow` / `rm_movep_canfd`）+ 厂商**解算**接口
  （`rm_algo_inverse_kinematics` / `rm_algo_forward_kinematics` / `rm_algo_*` 等）
  + **变体名**（如 `rm_algo_inverse_kinematics_new`）。
  禁用名单**必须显式登记变体名**：静态扫描器按「精确名 + 左括号」匹配，
  未登记的变体名**静态不会命中**（运行时规则另有兜底，但静态那一证同样不得漏）。
* **`allowed_apis`（白名单）**：**允许**调用的**非解算**接口 =
  自研 5 ms 透传通道（`rm_movej_canfd`）、状态读取（`rm_get_current_joint_current`）
  及设备层的使能 / 清错 / 停机 / 限位 / 夹爪类接口。
  白名单的作用是**避免前缀误伤**（`rm_movej_canfd` 不应被 `rm_movej` 误判）
  并在**名单冲突时以白名单为准**。

### 4.2 `forbidden_call_count` 与 `non_whitelist_call_count`

| 计数 | 含义 | 期望值 | 违反含义 |
|---|---|---|---|
| `total_call_count` | SDK 调用总次数 | —（仅参考） | — |
| `non_whitelist_call_count` | **不在白名单**里的符号被调用的次数 | **0** | 设备层越界调用了厂商接口（超出白名单范围） |
| `forbidden_call_count` | **命中禁用名单（含变体名规则）**的调用次数 | **0** | 主链路调用了厂商**解算/运动下发**接口 ⇒ **退出码 1** |

两者是**嵌套**关系：禁用名单的接口**通常也不在白名单**里 ⇒ 命中禁用名单时
`non_whitelist_call_count` 也会 +1。区别在于**后果不同**：
`forbidden_call_count > 0` 是**红线**（主链路把行为交给了不可审计的黑盒），
`non_whitelist_call_count > 0` 是**越界告警**（设备层调用了未经审核的接口）。

CLI 侧（`python3 -m rm75_drake.cli exec --json`）：

```json
{"sdk_audit": {"total_call_count": 0, "non_whitelist_call_count": 0,
               "forbidden_call_count": 0, "forbidden_calls": []}}
```

**未进入执行环**（例如真机不可达 ⇒ SKIP）时计数**恒为 0**——
这**不是**「已验证通过」，而是「没有 SDK 调用可审」，SKIP 的同时
必须**逐条**列出 `unverified_metrics`。

### 4.3 为什么**必须**有负例自检

审计若只有正例，就无法区分「真的没有调用禁用接口」与「审计器根本没在数」。
因此 `tests/test_device_audit_forbidden.py` 必须包含**负例自检**：

* 记一次禁用名 ⇒ `forbidden_call_count == 1` 且 `non_whitelist_call_count == 1`；
* **禁用名的变体名**（名单**未**登记的形态）同样必须被识别 ⇒ 证明变体兜底规则有效；
* CLI 注入一次禁用调用 ⇒ **退出码 1** 且 JSON 里的计数 > 0；
* 锁文件缺 `forbidden_apis` 段 ⇒ 读取必须**显式报错**（不得静默当成空名单）。

负例自检是**审计器自己的鉴别力证明**；删掉负例 ⇒ 运行时那一证失效。

---

## 5. 长稳执行与失败处置（S07-T05）

### 5.1 用法

```bash
# 离线演练（虚拟时钟 + 替身传输层；**不代表真机长稳结果**，退出码 0）
PYTHONPATH=src python3 scripts/long_run.py --dry-run --duration-s 5 --json

# 真机长稳（8 h；真机不可达时 SKIP，退出码仍为 0）
PYTHONPATH=src python3 scripts/long_run.py --duration-s 28800 --json
```

载体是**任务级循环**：每轮完整跑一遍 S06 的任务原语链
（`rm75_drake.tasks.solver.solve_task`），**不重写求解器** ⇒ 口径唯一。
默认时长 `--duration-s 28800`（8 h），默认任务级成功率门槛 `0.95`。

### 5.2 **8 h 真机执行的硬件门控 SKIP 语义**

* 真机不可达（SDK 不可导入 / TCP 指令通道不可连接）⇒ 输出
  `SKIP(reason=hardware_unreachable)` + **逐条**列出未验证指标，
  **退出码仍为 0**（**既不伪装成功，也不伪装失败**，K001）。
* 离线演练（`--dry-run`）⇒ JSON 里 `offline = true`、`physical_verified = false`、
  `thresholds_met` **恒为 false**（物理长稳未验证 ⇒ 不得报达标），
  输出必须带「**物理长稳未验证**」标注。
* 也就是说：**离线演练永远不能被引用为「8 h 长稳达标」的证据**。

### 5.3 成功率与异常停机的判定

| 判据 | 字段 | 含义 |
|---|---|---|
| 任务级成功率 | `success_rate = success_count / cycles` | 门槛 `thresholds.min_success_rate`（默认 0.95）；**分母 = 实际跑过的轮数** |
| 异常停机 | `aborted = true` | 未捕获异常（`exceptions > 0`）或求解硬失败（`stage ∈ {ik, path, collision, …}`）⇒ **立即停机并记录 `aborted_reason`** |
| 门槛结论 | `thresholds_met` / `threshold_failures` | 真机路径下未达标 ⇒ **退出码 1**；离线演练恒不报达标 |
| 失败分布 | `failure_breakdown` / `failures` | 按 `stage:reason` 聚合，并逐轮记录 `cycle / stage / reason / message` |

### 5.4 无异常停机的处置流程

1. **先看 `failures` 的 `stage:reason`**（不是看总成功率数字）；
   `stage` / `reason` 词表真相源 = `rm75_drake.tasks.solver.STAGES` / `REASONS`。
2. 若是 `ik` 类（`unreachable` / `branch_exhausted` / `limit_violation` / `wrist_singular`）
   ⇒ 属**任务/场景口径**问题，改目标位形而不是调门槛。
3. 若是 `collision` / `escape` ⇒ 属**场景/感知裕量**问题，查 `scenarios/*.yaml`
   与感知 `effective_margins`。
4. 若是 `transport_error` / `gripper_offline` ⇒ 属**真机通道**问题，
   查 S03 的设备通道与安全七闸记录（**不得**绕过安全闸直发指令）。
5. **禁止**用「放宽 `--min-success-rate`」或「缩小 `--duration-s`」把红灯变绿；
   真机长稳门槛变更属**验收口径变更**，必须走 S06 runbook §1 的口径纪律。

### 5.5 已知缺口（诚实边界）

载体当前**不包含**「把关节轨迹下发到真机并回读状态」的**运动执行闭环**
（真机路径由 `rm75_drake.device.exec_loop` 承担安全七闸，不在本载体里绕开）。
⇒ **真机 8 h 长稳仍然是未验证指标**。

---

## 6. 未验证指标清单（两个**唯一真相源**）

| 真相源 | 覆盖范围 | 出口 |
|---|---|---|
| `rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS` | 任务级物理指标：真机抓放成功率、真机夹持误差、真机交接保持漂移、真机感知延迟、真机夹爪力控 / 空夹判据、真机长稳节拍（含 **8 h 任务级长稳**条目 `UNVERIFIED_TASK_METRIC_LONG_RUN_8H`） | `scripts/task_stats.py`、`scripts/long_run.py`（`unverified_metrics` 字段）、`cli task --json` |
| `rm75_drake.device.hardware_gate.UNVERIFIED_HARDWARE_METRICS` | 硬件级指标：真机周期抖动、真机位置误差、真机通信往返延迟、真机安全闸触发次数、真机急停响应时间等 | `cli exec --json`（SKIP 时逐条列出）、`rm75_drake.device.hardware_gate.format_skip` |

**本切片交付时的诚实结论（三项均未验证）**：

1. **本机无真机** ⇒ 所有真机物理指标未验证；
2. **真机 8 h 长稳未验证** ⇒ 只交付了载体（harness）+ 离线演练 + 硬件门控 SKIP 契约；
3. **真机全链路录制未验证** ⇒ 只交付了 session v1 产物 schema、离线 golden 基线
   与逐点一致比较器，**没有**真机录制数据。

**解锁流程**：真机到位 ⇒ 走 S03 设备通道与 S04 实测采样作业 ⇒ 复算对应指标 ⇒
把对应条目从**唯一真相源**移除（CLI 与脚本只再导出）⇒ 并**保留复算命令**。
**清单之外**不得另立第二份未验证清单。

---

## 7. 运维（S07-T07）

### 7.1 健康信号（正常态应看到的）

| 信号 | 期望 | 出处 |
|---|---|---|
| `ci.sh fast` 汇总行 | `PASS=N SKIP=M FAIL=0`，退出码 0 | `scripts/ci.sh` 末尾汇总 |
| 禁用接口静态扫描 | 「未命中禁用接口」，退出码 0 | `scripts/scan_forbidden_apis.py` |
| 运行时审计计数 | `sdk_audit.forbidden_call_count == 0` 且 `non_whitelist_call_count == 0` | `cli exec --json` |
| golden 比对 | `pointwise_identical = true`、`num_tcp_mismatches == 0`、`fingerprints_match = true` | `scripts/gen_golden.py --check --json` |
| 长稳演练 | `offline = true` / `physical_verified = false` / `thresholds_met = false`（**物理长稳未验证**） | `scripts/long_run.py --dry-run --json` |
| 许可门禁 | `verify_env.py --upstream --licenses` 退出码 0；`gen_notice.py --check` 一致 | `tools/verify_env.py` / `tools/gen_notice.py` |
| 会话产物 | session v1 产物含顶层 `checksum`，校验通过 | `rm75_drake.regression.session` |

### 7.2 故障信号（异常态的识别特征）

| 信号 | 含义 | 首查 |
|---|---|---|
| `ci.sh` 出现 `[FAIL]` | 该检查项失败（**不是** SKIP） | 失败项回显的日志前 40 行 + 尾部 20 行 |
| `gen_golden --check` 退出码 1 且提示「**基线失效**」 | 三指纹漂移（模型/场景/任务） | `fingerprint_details` 里漂移的那一枚 |
| `gen_golden --check` 退出码 1 且 `num_tcp_mismatches > 0` | **行为漂移**（真回归） | 逐点差异（**禁止**直接重录基线） |
| `scan_forbidden_apis` 退出码 1 | 主链路出现禁用接口**调用点** | 命中的文件与符号；先查是不是新加了厂商调用 |
| `cli exec --json` 的 `forbidden_call_count > 0` | **运行时**调用了厂商解算接口 | `forbidden_calls` 的符号名；退出码应为 1 |
| `long_run.py` 输出 `SKIP(reason=hardware_unreachable)` | 真机不可达（**不是**失败，也**不是**通过） | SDK 可导入？TCP 指令通道可连接？ |
| `long_run.py` `aborted = true` | 异常停机 | `aborted_reason` + `failures[].stage:reason` |
| `verify_env --licenses` 非 0 | 许可登记与锁文件不一致 | 消息里的点名包 / 字段 |
| `gen_notice --check` 非 0 | NOTICE 与登记表不一致 | 重跑生成（**不要手改 NOTICE**） |

### 7.3 恢复流程（CI 失败时的排查顺序）

**固定顺序：先定位失败项 → 再判「门禁自身坏了」还是「代码回归」→ 最后才动基线/资产。**

1. **定位失败项**：从 `ci.sh` 汇总里的 `FAIL=` 与失败项名称出发，
   **单独复跑那一条命令**（不要整档重跑）：

```bash
bash scripts/ci.sh fast
PYTHONPATH=src python3 -m pytest -q tests/test_s07_runbook.py
PYTHONPATH=src python3 -m pytest -q -m "not slow"
PYTHONPATH=src python3 scripts/scan_forbidden_apis.py src tools scripts
PYTHONPATH=src python3 scripts/gen_golden.py --check
PYTHONPATH=src python3 scripts/long_run.py --dry-run --duration-s 5 --json
PYTHONPATH=src python3 tools/verify_env.py --upstream --licenses
```

2. **判失败类型**：
   * **代码回归**（新加的逻辑把行为改了）⇒ 修代码；**不得**改基线 / 改门槛 / 降级成 SKIP；
   * **门禁自身坏了**（断言写错、路径写错、环境缺依赖）⇒ 修门禁/环境，
     并**保留**该检查项的恒跑属性；
   * **基线失效**（指纹漂移）⇒ 走 §2.4 的「先判失效 → 再查代码 → 最后才重录」；
   * **资产/许可不一致** ⇒ 走 §3.1 的流程（registry → NOTICE → verify_env）。
3. **复跑对应档位**：`bash scripts/ci.sh fast`（常驻）→ 需要时再 `medium` / `nightly`。
4. **资产/许可恢复**：`registry.yaml` 改动后**必须**重生成 NOTICE 并重跑
   `verify_env --upstream --licenses`，否则 `assets/lock` 常驻项会持续红。
5. **禁止的“恢复”手段**（一律视为掩盖问题）：
   把恒跑项降级成 SKIP；放宽 `--min-success-rate`；用 `--duration-s` 缩短长稳；
   直接重录 golden 基线把红灯变绿；手工编辑 `NOTICE`。

### 7.4 监控缺口（当前**没有**的东西）

* **无在线监控**：没有指标采集 / 打点 / 时序库，CI 结果只存在于**单次运行的日志**里，
  没有跨运行的指标趋势。
* **无告警路由**：CI 失败**不会**主动通知任何人（无邮件 / IM / PagerDuty 路由），
  依赖人主动去看 CI 页面或本地跑 `scripts/ci.sh`。
* **无真机侧长稳在线看板**：8 h 长稳目前只能**离线**跑一次 `long_run.py`，
  没有持续运行、没有实时成功率曲线。
* **无基线漂移告警**：基线失效只在 `--check` 那一刻被看见，
  **没有**在每次模型/场景/任务改动时自动提示「基线需重录」。
* **无资产归档可用性监控**：上游归档缺失只会让相关项显式 SKIP，
  不会主动提醒「归档该补了」。
* **缺口补齐的前置条件**：本仓库当前**没有真机、没有在线运行环境**；
  上述缺口需在真机与 CI 平台接入后逐项关闭（关闭前**不得**宣称已具备监控能力）。

### 7.5 回滚与降级路径

* **回滚（代码层）**：S07 新增 `src/rm75_drake/regression/`、
  `src/rm75_drake/device/sdk.py` 的运行时禁用审计能力、`scripts/gen_golden.py`、
  `scripts/long_run.py`、`tools/gen_notice.py`、`licenses/registry.yaml`、
  `tests/data/golden/`、`runbooks/s07_engineering_productization.md`，
  以及 `scripts/ci.sh` 的**增量**（`tests/regression` / `assets/lock` /
  `tests/upstream` 常驻项 + nightly 的 golden / soak 项）
  ⇒ 回滚 = 撤掉这些增量，**其余切片（S01–S06）的门禁不受影响**。
* **降级（运行层，禁止静默降级）**：
  * **Drake 不可用** ⇒ 显式 SKIP（模型 / 规划 / nightly 自检），**不**伪装成通过；
  * **上游归档缺失** ⇒ 相关项显式 SKIP；**资产登记与 NOTICE 门禁仍恒跑**
    （三方交叉校验不需要归档实体）；
  * **真机不可达** ⇒ `SKIP(reason=hardware_unreachable)` + 未验证指标逐条列出，
    **退出码仍为 0**；
  * **离线演练** ⇒ `physical_verified = false`、`thresholds_met = false`，
    **不得**被引用为真机长稳结论。
