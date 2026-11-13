# rm_safety 失效安全停机四级链路说明

> 面向"没参与过本任务的人"：照着本文，可以在真机窗口内从零操作安全停机链路，
> 并知道**每一级为什么这么定、证据长什么样、失败了怎么排查**。
>
> 关联切片：M001-3kzz40 / S04（失效安全停机四级链路）；
> 关联需求：R003（厂商解耦）、R005（安全判据独立于规划器）、R010（分级失败可见）；
> 关联文档：`docs/rm_executor执行线程契约.md`（10 ms 单时钟执行器）、
> `docs/rm_hw_bridge硬件适配层说明.md`（硬件层白名单与符号契约）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 / 文件 |
|---|---|
| 看四级等级表（唯一真值） | `dualarm_ws/src/rm_safety/rm_safety/levels.py` |
| 看判据（故障类 → 等级，纯函数） | `dualarm_ws/src/rm_safety/rm_safety/policy.py` |
| 跑六条离线安全用例 | `PYTHONPATH=dualarm_ws/src/rm_safety:dualarm_ws/src/rm_hw_bridge:dualarm_ws/src/rm_executor python3 -m rm_executor.safety_batch --all` |
| 外部强制停机（真机） | `rmctl safety stop`（默认 L4 急停） |
| 查闸门状态 | `rmctl safety status` |
| 看错误码表（工程声明映射） | `workcell/safety/error_codes.yaml` |
| 看安全策略开关 | `workcell/rm_workcell.yaml` 的 `safety:` 段 |
| 复核分层红线（静态审计） | `PYTHONPATH=dualarm_ws/src/rm_hw_bridge python3 -m rm_hw_bridge.audit --repo`（退出码 0 = 零违规） |

---

## 1. 分层：安全层是独立的一层

```
rmctl（体验层：规划 / 基线 / CLI）
  ├── rm_safety（安全层：四级判据 + 唯一动作权威 + 安全闸门）   ← 本文档
  └── rm_executor（执行层：10 ms 单时钟执行器）
        └── rm_hw_bridge（硬件层：唯一允许接触 RM-API2 SDK 的层）
              └── RM-API2 1.1.7
```

**安全层 `rm_safety` 不得反向依赖体验层 / 执行层 / 规划内核**。
这条红线不是文档里的一句话，而是**代码层强制**：

* 契约声明（`dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/contract.yaml`）：

  ```yaml
  rules:
    layers:
      hardware: rm_hw_bridge
      safety: rm_safety
      executor: rm_executor
      experience: rmctl
    safety_must_not_import:
      - rmctl
      - rm_executor
      - "tesseract*"
  ```

* 静态审计按 `rules.layers` **声明式**判定（`rm_hw_bridge/audit.py`）：
  `layer_roots` / `layer_of` / `_layer_reverse_targets` 全部读契约，
  **新增一层只需改 YAML，审计器零改动**；
* 违规形态：安全层里出现 `import rm_executor` / `import rmctl` /
  `import tesseract_robotics...` → `reverse-dependency` 违规，CLI 退出码 **3**；
* 复核命令：

  ```bash
  PYTHONPATH=dualarm_ws/src/rm_hw_bridge python3 -m rm_hw_bridge.audit --repo
  # 期望：审计通过：0 项违规（退出码 0）
  ```

* 负例测试：`dualarm_ws/src/rm_hw_bridge/tests/test_audit.py`
  （`test_negative_safety_layer_imports_rm_executor` /
  `..._rmctl` / `..._tesseract`，均断言退出码 3；
  `test_safety_layer_has_no_reverse_dependency` 断言真实包内零违规）。

> **本次变更未触碰任何 SDK 符号清单**：`core` 22 / `support` 5 / `blacklist` 13
> 符号与 `rm_algo_*` 前缀一律不变（R003 结论原样成立），
> 审计期望仍是 0 违规。

---

## 2. 四级等级表（唯一真值）

唯一真值文件：`dualarm_ws/src/rm_safety/rm_safety/levels.py`（`LEVELS` 字典）。
**下游一律以本表为准**（动作执行器、执行层接线、注入用例、闸门、本文档）。

| 等级 | 判据（故障类） | 动作 | SDK 符号 | severity | 退出码 | 恢复方式 |
|---|---|---|---|---|---|---|
| **L1 降速** | `follow_error_light`：`follow_warn_deg < e ≤ follow_stop_deg` | `set_speed_scale` | `rm_set_joint_drive_max_speed` | `RECOVERABLE` | 3 | **auto**（误差回落到告警阈值内自动恢复原速，记 `safety_l1_slowdown_cleared`） |
| **L2 暂停** | `follow_error_heavy`：`e > follow_stop_deg`；`limit_exceeded`：运行期越过声明限位；`stream_lag_light`（仅当 `enable_l2_pause_on_stream_lag=true`） | `pause` | `rm_set_arm_pause` | `NEEDS_HUMAN` | 3 | manual（人工确认后手动 `rm_set_arm_continue`；安全层**不自动 continue_**） |
| **L3 缓停** | `collision`：错误码命中 `class=collision`；`unknown_error_code`：错误码不在表内（保守） | `slow_stop` | `rm_set_arm_slow_stop` | `NEEDS_HUMAN` | 3 | manual（人工读码复位；安全层**不自动清错误码**） |
| **L4 急停** | `protection_stop`：`arm_current_status ∈ {9,10}` 且非本进程发起；`stream_lost`：`age > stream_stop_ms` | `stop` | `rm_set_arm_stop` | `FATAL` | 4 | manual_reset（按硬件急停 + 人工复位控制器） |

**多故障并发**时按 **(等级降序, 故障类优先级)** 取主因，优先级：

```
protection_stop > stream_lost > collision > unknown_error_code
  > limit_exceeded > follow_error_heavy > follow_error_light
```

### 2.1 阈值（唯一来源：执行层冻结常量）

| 阈值 | 值 | 来源 |
|---|---|---|
| `follow_warn_deg` | `2.0` | `rm_executor/contract.py::FOLLOW_WARN_DEG` |
| `follow_stop_deg` | `5.0` | `rm_executor/contract.py::FOLLOW_STOP_DEG` |
| `stream_warn_ms` | `50` | `rm_executor/contract.py::STREAM_WARN_MS` |
| `stream_stop_ms` | `200` | `rm_executor/contract.py::STREAM_STOP_MS` |

安全层的 `SafetyThresholds` **不提供默认值**（避免第二处常量源），
阈值由调用方从上述唯一真值传入。

### 2.2 策略开关（唯一来源：`workcell/rm_workcell.yaml` 的 `safety:` 段）

| 键 | 默认 | 含义 |
|---|---|---|
| `l1_speed_scale` | `0.5` | L1 降速比例（目标速度 = 声明限值 × scale） |
| `enable_l1_slowdown` | `true` | 关闭时轻跟随误差**保守升到 L2**（不做没有动作的降速） |
| `enable_l2_pause_on_stream_lag` | `false` | 关闭时断流告警区间**只告警不动作**（保持 S03 口径） |
| `error_codes` | `safety/error_codes.yaml` | 错误码表路径（相对 `workcell/`） |
| `sdk_timeout_s` | `1.0` | SDK 调用超时（秒） |

### 2.3 动作权威链（唯一动作权威）

```
policy.evaluate(signals, thresholds)   ← 纯函数判据（无 I/O / 无 SDK / 无时钟）
        ↓ Decision(level, action, evidence, next_action)
SafetySupervisor                       ← 编排：等级单调、仅 L1 自动恢复、不碰 EventLog
        ↓
SafetyActuator                         ← 等级 → 传输端口调用（失败留证据并升级 L4）
        ↓
Transport（rm_executor 注入；rm_hw_bridge 落地 SDK）
```

* 等级 → 传输端口方法的唯一映射在 `rm_safety/actuator.py::ACTION_METHODS`；
* 动作失败 → **升级到 L4**（`escalated_to="L4"`）；
* **L4 动作本身失败 → 只留 FATAL 证据，绝不假装已停机**；
* 执行器**不再就地二分**（跟随误差→pause、断流→stop），避免双重动作。

---

## 3. 六条离线用例与复跑命令

跑批只对 **fake（离线假传输）** 开放；真机路径拒绝任何注入。

```bash
PYTHONPATH=dualarm_ws/src/rm_safety:dualarm_ws/src/rm_hw_bridge:dualarm_ws/src/rm_executor \
    python3 -m rm_executor.safety_batch --all
```

期望输出（末行 `passed=6 failed=0 → pass`，退出码 **0**）：

```
follow_error_light    L1   follow_error   set_speed_scale   pass
follow_error_heavy    L2   follow_error   pause             pass
limit_exceeded        L2   limit_exceeded pause             pass
collision             L3   collision      slow_stop         pass
protection_stop       L4   protection_stop stop              pass
stream_lost           L4   stream_lost    stop              pass
passed=6 failed=0 → pass
```

单例复跑：`--case <name>`（可重复），报告目录 `--out-dir <dir>`（默认
`<工作区根>/build/reports`）。

| 用例 | 注入 | 预期等级 | 预期故障类 | 预期动作 | 说明 |
|---|---|---|---|---|---|
| `follow_error_light` | `follow_error_light:3.0` | L1 | `follow_error` | `set_speed_scale` | 轻跟随误差（告警阈值之上、停机阈值之内） |
| `follow_error_heavy` | `follow_error:6.0` | L2 | `follow_error` | `pause` | 重跟随误差（超过停机阈值） |
| `limit_exceeded` | `limit_exceeded:2.0` | L2 | `limit_exceeded` | `pause` | 运行期位置越过声明限位 |
| `collision` | `collision:90001` | L3 | `collision` | `slow_stop` | 控制器碰撞错误码（**工程声明映射码**） |
| `protection_stop` | `protection_stop` | L4 | `protection_stop` | `stop` | 控制器保护停（`RM_STOP_E = 9`） |
| `stream_lost` | `feedback_stall:250` | L4 | `stream_lost` | `stop` | 反馈断流超过停机阈值 |

**注入参数在解析阶段就自证校验**（越不过阈值直接拒绝，退出码 3，
不会产出"跑了但没动作"的报告）：

* `follow_error_light` 必须落在告警阈值之上、停机阈值之内（`2.0° < deg ≤ 5.0°`）；
* `limit_exceeded` 的越界量必须 `> 0`；
* `collision` 的码必须是非负整数。

---

## 4. 证据字段清单

### 4.1 单例报告（`build/reports/s04_<case>.json`）

| 字段 | 含义 |
|---|---|
| `case` / `injection` / `note` | 用例名、注入规格、说明 |
| `expected_level` / `expected_fault_class` / `expected_action` | 预期值 |
| `level` | 实际落到的等级（L1–L4） |
| `fault_kind` / `fault_kind_detail` | 计划口径故障类 / 判据原始故障类 |
| `severity` | 严重度（`RECOVERABLE` / `NEEDS_HUMAN` / `FATAL`） |
| `exit_code` | 进程退出码（L1–L3 → 3，L4 → 4） |
| `threshold` / `observed` / `margin_to_limit` | 阈值 / 实测值 / 越界余量 |
| `next_action` | **可粘贴**的下一步 |
| `action_sequence` | 动作序列：`kind` / `level` / `action` / `transport_method` / `sdk_symbol` / `status` / `reason` |
| `sdk_call_sequence` / `sdk_call_counts` | 传输层调用序列与按方法计数 |
| `detected_fault_kinds` | 本次命中的全部故障类（主因优先） |
| `evidence` | 判据证据数组（见 4.2） |
| `recovery` | 恢复方式（`auto` / `manual` / `manual_reset`） |
| `auto_recovery_occurred` | 是否发生过 L1 自动恢复 |
| `executor_status` / `executor_exit_code` | 执行器状态与退出码 |
| `verdict` / `verdict_reason` | `pass` / `fail` / `invalid_spec` / `error` 与理由 |
| `honesty` | 诚实声明（离线代理结论，不代表真机结论） |

### 4.2 判据证据条目

每条证据至少含五个可核对字段：

| 字段 | 含义 |
|---|---|
| `fault_kind` | 故障类（`follow_error_light` / `limit_exceeded` / `collision` / `protection_stop` / `stream_lost` / `unknown_error_code` / `advisory`） |
| `level` | 命中等级（无动作的告警条目为空串） |
| `threshold` | 判据阈值（或判据文本，如 `error_code_map 中 class=collision`） |
| `observed` | 实测值（跟随误差度 / 断流毫秒 / 错误码列表 / 越界条目数 …） |
| `margin` | 越界余量（正 = 越界量，负 = 距阈值的余量） |
| `unit` | 单位（`deg` / `ms`），仅数值型判据有 |
| `note` | 补充说明（如"未知错误码，按保守档 L3 缓停处理"） |

### 4.3 动作审计证据（`kind` 固定取值，便于检索）

| `kind` | 含义 |
|---|---|
| `action_ok` | 动作成功落地 |
| `action_failed` | 动作失败（已升级 L4） |
| `escalated_to_l4` | 失败后升级到 L4 |
| `l4_stop_failed` | L4 急停本身失败（**FATAL，不假装成功**） |
| `l1_speed_restored` | L1 自动恢复原速成功 |
| `l1_speed_restore_failed` | L1 自动恢复失败 |
| `external_gate_request` | 外部安全闸门请求停机 |

### 4.4 执行器报告（evidence 行）

```
safety: current_level=L2 peak_level=L2 speed_scaled=false event_count=3 action_count=1
limit_check=ok            # 或 unavailable（原因）
safety gate: <path>（已关闭）   # 仅在闸门触发过停机时出现
```

### 4.5 状态面

* `Executor.safety_status` → `{current_level, peak_level, speed_scaled, event_count, auto_recovery_allowed, action_count}`；
* `SafetySupervisor.to_dict()` → 同上（供 `rmctl safety status` 与报告消费）；
* `SafetyActuator.action_log` → 动作审计序列；
* `SafetySupervisor.events` → 事件副本（`type` / `severity` / `reason` / `evidence` / `next_action`）；
* `SafetySupervisor.event_errors` → 宿主事件回调异常留痕（**不影响安全动作本身**）。

### 4.6 事件名（冻结）

安全等级事件：`safety_l1_slowdown` / `safety_l1_slowdown_cleared` /
`safety_l2_pause` / `safety_l3_slow_stop` / `safety_l4_emergency_stop`。

故障类事件：`fault_follow_error` / `fault_limit_exceeded` / `fault_stream_lost` /
`fault_collision` / `fault_protection_stop`。

执行层既有事件（S03 口径，全部保留）：`follow_error_warn` / `follow_error_exceeded` /
`feedback_stream_lag` / `feedback_stream_lost` / `controller_trajectory_conflict`。

---

## 5. 错误码表口径（**必须照此表述，不许说成厂商码表**）

真值文件：`workcell/safety/error_codes.yaml`（**声明式**，`status: unverified_offline`）。

> ### ⚠ 诚实声明
>
> **RM-API2 `err[]` 的厂商错误码表本轮未取得**（真机窗口前无法离线核对）。
>
> 本文件是**工程声明映射**（`source: engineering_declared/S04-2026-11-13`）：
> 码值与故障类的对应关系由本项目自行声明，用于离线判据单测与注入用例。
>
> **真机窗口必须逐条回填**（改 `code` / `class` / `level_hint` / `source`），
> 并把 `status` 从 `unverified_offline` 改成 `verified_on_robot`。
>
> 在回填完成前，**任何验收表述都不能说"碰撞码 90001 是厂商码"**。

| code | class | level_hint | 语义 | source |
|---|---|---|---|---|
| 90001 | `collision` | L3 | 控制器碰撞保护触发 | engineering_declared |
| 90002 | `over_limit` | L2 | 运行期位置/速度越声明限位 | engineering_declared |
| 90003 | `protection` | L4 | 控制器保护停 / 已进入停机状态 | engineering_declared |
| 90004 | `driver` | L3 | 驱动/伺服层报错（**仅记录证据**，不单独触发安全动作） | engineering_declared |
| 90005 | `comm` | L3 | 通信层报错（**仅记录证据**；断流由 `stream_age_ms` 判据单独处理） | engineering_declared |

**保守默认（不静默）**：

* 未知码 → `class=unknown` → 等级 **L3 缓停**（保守：可人工复位，不自动清错误码）；
* 码表文件缺失 / YAML 格式错 → 返回**空表 + 保守默认 + 明确 warning**
  （`ErrorCodeMap.warnings`），**绝不抛裸异常、也绝不静默**；
* 缺 `source` 的条目保留但产生 warning（来源不可追溯，必须补齐）。

---

## 6. 安全闸门（外部强制停机）

### 6.1 方向与协议

```
rmctl（客户端）  ──一行 JSON──▶  rm_runtime 的 Unix socket 闸门（服务端）
                                        │
                                        ▼
                              SafetySupervisor.apply_external → 唯一动作权威 → L4 急停
```

* socket 路径：默认 `<工作区根>/build/run/rm_runtime.sock`
  （`rm_runtime --gate-socket <path>` 可覆盖）；
* 协议：**一行 JSON 请求 → 一行 JSON 响应**；请求最大 8192 字节（防呆）；
* 命令：`{"cmd": "stop", "level": "L4"}` / `{"cmd": "status"}`；
* 服务线程**独立于 tick** —— 外部停机必须"立即"响应，不能等 10 ms 栅格；
* 等级**单调**：已有请求时保留**更重**的等级（L4 不会被后来的 L2 覆盖）；
* 闸门服务端异常**不静默**：`server_errors` / `server_error_count` /
  `handler_errors` / `handler_error_count` 可在 status 响应中取到。

### 6.2 客户端用法

```bash
# 强制 L4 急停（默认等级 L4）
rmctl safety stop

# 指定等级
rmctl safety stop --level L3

# 查询闸门状态
rmctl safety status

# 指定 socket 路径
rmctl safety stop --socket /tmp/rm_gate.sock
```

输出是 JSON（stdio），退出码沿用仓库口径：**0 ok / 3 blocked / 4 error**。

`rmctl safety status` 响应字段（`UnixSocketGate.to_dict()`）：

```json
{
  "status": "ok",
  "cmd": "status",
  "stop_requested": false,
  "requested_level": "",
  "reason": "",
  "source": "",
  "request_count": 0,
  "pid": 12345,
  "socket_path": "/abs/path/build/run/rm_runtime.sock",
  "running": true,
  "server_errors": [],
  "server_error_count": 0,
  "handler_errors": [],
  "handler_error_count": 0
}
```

失败路径全部**诚实映射为 blocked**（带 `reason` + 可粘贴 `next_action`）：
socket 不存在 / 存在但无人监听 / 连接失败 / 无响应超时（2 s） / 返回空 /
返回非 JSON / 返回格式非法 / 服务端拒绝 / 非法等级。

### 6.3 执行期证据

闸门请求 → 唯一动作权威立即 L4 急停 → tick 下一拍退出：

* 事件 `safety_l4_emergency_stop`（severity=FATAL，reason 含"外部闸门请求"，
  evidence 含 `kind=external_gate_request`）；
* 报告 evidence 追加 `safety gate: <path>（已关闭）`；
* `Executor.safety_status.current_level` 变为 `L4`。

---

## 7. 真机窗口回填表（本轮**未**执行，留给真机窗口）

> **诚实声明**：以下四项**真实动作本轮未执行**（真机窗口）。
> 本切片只交付**同代码路径的真机入口**与本回填表。
> 表内所有"实测值"列必须由真机窗口的**真实观测**填写，**禁止留空当作已通过**。

| # | 回填项 | 真机入口（命令 / 步骤） | 预期等级 / 期望行为 | 实测值 | 结论 | 备注 |
|---|---|---|---|---|---|---|
| 1 | **真实跟随误差** | 手动拖动或外力扰动关节，使跟随误差越过 `2.0°`（L1）/ `5.0°`（L2） | L1 → 降速且误差回落后自动恢复；L2 → 暂停并等人工 | 待填 | 待填 | 记录 `safety_l1_slowdown` / `safety_l2_pause` 事件与 `follow_error_deg` 实测 |
| 2 | **真实保护停行为** | 触发控制器保护停（如运动到机械限位 / 手动置保护停） | 判为 `protection_stop` → L4 急停（`rm_set_arm_stop`） | 待填 | 待填 | 记录 `arm_current_status` 实测值（是否落在 `{9,10}`） |
| 3 | **真实降速生效** | 触发 L1 后读回关节驱动最大速度 | 目标速度 = 声明限值 × `l1_speed_scale`（0.5） | 待填 | 待填 | 用 `rm_get_robot_info` / SDK 读回；记实际角速度 |
| 4 | **闸门实停** | 真机运行时另开终端执行 `rmctl safety stop` | 立即 L4 急停，执行器下一拍退出（退出码 4） | 待填 | 待填 | 记录停机响应延迟与 `request_count` |
| 5 | **厂商 `err[]` 码表回填** | 真机触发碰撞 / 越限 / 保护停，读取 `arm_err` 实际码值 | 逐条替换 `workcell/safety/error_codes.yaml` 的 `code`/`class`/`level_hint`/`source`，并把 `status` 改为 `verified_on_robot` | 待填 | 待填 | **本项未完成前，验收表述不得称"厂商码表"** |
| 6 | 断流实停 | 真机运行时拔网线 / 断链 | `stream_lost` → L4 急停 | 待填 | 待填 | 记录断流判定延迟 |
| 7 | 超限实停 | 真机运动越过声明限位 | `limit_exceeded` → L2 暂停 | 待填 | 待填 | 记录越界余量 `margin_to_limit` |

---

## 8. 按 `next_action` 排查

安全层的每条判定都带**可粘贴**的下一步。按下表对照处理：

| 看到的 `next_action` | 含义 | 该做什么 |
|---|---|---|
| L1：确认跟随误差已回落到告警阈值内，安全层会自动恢复原速并记录 `safety_l1_slowdown_cleared`；若误差持续，请 `rmctl safety status` 查看当前等级后人工处置 | 误差仍在告警区间 | 等误差回落；若持续 → 查机械/负载/加减速参数，再查 `rmctl safety status` |
| L2：人工确认现场安全后手动下发 `rm_set_arm_continue` 恢复运行（安全层不自动 `continue_`）；恢复前请查看证据里的阈值与实测值 | 已暂停 | 先看证据里的 `threshold` / `observed` / `margin`；现场确认安全后再手动 `continue_` |
| L3：人工读取控制器错误码并复位后手动恢复运行（安全层不自动清错误码）；未知错误码按保守档 L3 处理，请把证据里的 `code` 原样贴给操作者 | 已缓停 | 读控制器 `arm_err` → 对照码表定级 → 人工复位 → 手动恢复；未知码按 L3 保守处置并记录待回填 |
| L4：立即按下硬件急停并人工复位控制器后恢复运行（安全层不自动复位）；请把本次 `evidence` 原样贴给操作者 | 已急停 | **先按硬件急停**（软件路径可能已失效）→ 复位控制器 → 贴 evidence 归档 |

### 8.1 常见症状对照

| 症状 | 原因 | 处理 |
|---|---|---|
| `rmctl safety status` 报 "安全闸门 socket 不存在" | `rm_runtime` 没起或路径不同 | 先起 `rm_runtime`；或 `--socket <path>` 指定 |
| 报 "socket 存在但无人监听" | `rm_runtime` 已退出，残留 socket 文件 | `rm -f <path>` 后重试 |
| 报 "安全闸门无响应（超时 2.0 s）" | 服务端线程卡住 | 确认 `rm_runtime` 仍在运行；查闸门端日志与 `server_errors` |
| 报 "安全层 rm_safety 不可用" | `PYTHONPATH` 里没有 `rm_safety` | `export PYTHONPATH=$PWD/dualarm_ws/src/rm_safety:$PYTHONPATH` |
| 报告里 `limit_check=unavailable（原因）` | 声明限值不可用 → **跳过**超限判据（不是"无越限"） | 按证据里的原因补齐限值来源（`rm_hw_bridge.limits`） |
| 事件 `safety_layer_unavailable` / `safety_layer_unavailable_fallback` | 安全包导入失败，执行器降级到兜底路径 | 按证据里的 `next_action` 补 `PYTHONPATH`；**降级不等于安全** |
| 事件 `safety_action_failed` / 证据 `escalated_to_l4` / `l4_stop_failed` | 安全动作本身失败 | `l4_stop_failed` 是 FATAL：**立刻按硬件急停**；查 SDK 符号与控制器状态 |

---

## 9. 环境与复跑清单

```bash
# 1) 载入环境（含 rm_safety）
source scripts/rmenv.sh
# → PYTHONPATH 含 dualarm_ws/src/{rmctl,rm_hw_bridge,rm_safety,rm_executor}

# 2) 分层红线复核（期望：0 违规，退出码 0）
PYTHONPATH=dualarm_ws/src/rm_hw_bridge python3 -m rm_hw_bridge.audit --repo

# 3) 审计负例单测（期望全绿）
PYTHONPATH=dualarm_ws/src/rm_hw_bridge python3 -m pytest \
    dualarm_ws/src/rm_hw_bridge/tests/test_audit.py -q

# 4) 安全层单测（期望全绿）
PYTHONPATH=dualarm_ws/src/rm_safety python3 -m pytest \
    dualarm_ws/src/rm_safety/tests -q

# 5) 六条离线安全用例（期望 passed=6 failed=0，退出码 0）
PYTHONPATH=dualarm_ws/src/rm_safety:dualarm_ws/src/rm_hw_bridge:dualarm_ws/src/rm_executor \
    python3 -m rm_executor.safety_batch --all
```

---

## 10. 本轮结论的诚实边界

* 本文档描述的**全部等级、判据、动作、证据字段、闸门协议都是离线口径**；
* 六条用例是 **fake 模式离线代理结论**，**不代表真机结论**；
* 真机相关的七项（真实跟随误差 / 真实保护停 / 真实降速生效 / 闸门实停 /
  断流实停 / 超限实停 / 厂商码表回填）**本轮未执行**，见表 7；
* 厂商 `err[]` 码表**本轮未取得**，码表为工程声明映射（见第 5 节声明）。
