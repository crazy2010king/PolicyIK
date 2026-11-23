# rm_process 单臂工艺闭环说明（声明式取放工艺）

> 面向"第一次跑这条取放工艺的人"：照着本文，你能在 5 分钟内看懂
> **每圈跑什么段、失败了是什么等级、下一步该做什么**，并知道
> **哪些结论已实证、哪些还没有**。
>
> 关联切片：M001-3kzz40 / S06（单臂真机工艺闭环）；
> 关联代码：`dualarm_ws/src/rmctl/rmctl/process/spec.py`（任务规格加载）、
> `dualarm_ws/src/rmctl/rmctl/process/runner.py`（工艺状态机）、
> `dualarm_ws/src/rmctl/rmctl/process/trace.py`（trace 记录 / 回放）、
> `dualarm_ws/src/rmctl/rmctl/process/clients.py`（Planner / Motion / Gripper 客户端）、
> `dualarm_ws/src/rm_executor/rm_executor/session.py`（真机单连接多段执行）；
> 关联文档：`docs/rm_gripper接入与状态判定.md`（夹爪接入与六类判定）、
> `docs/rm_executor执行线程契约.md`（执行层 10 ms 契约）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 / 文件 |
|---|---|
| 跑一条离线工艺（假运动 / 假夹爪） | `python3 -m rmctl run pick_place --count 1` |
| 跑离线 50 圈证据链（四段断言） | `python3 scripts/pick_place_offline_evidence.py --out build/reports/s06_pick_place_offline.json` |
| 只读回放一份 trace（零运动） | `python3 -m rmctl run --replay build/run/pick_place_trace.jsonl` |
| 真机 50 圈（需真机窗口） | `python3 -m rmctl run pick_place --live --count 50 --trace build/run/pick_place_trace_live.jsonl --report build/reports/s06_pick_place_live.json` |
| 工艺长什么样（唯一规格源） | `workcell/tasks/pick_place.yaml` |
| 判定真值（唯一） | `rm_hw_bridge.gripper_state.judge` |
| 恢复阶梯（唯一） | `rm_hw_bridge.gripper_state.RECOVERY_LADDER` |
| 严重度排序（唯一） | `rm_executor.events.severity_rank` |
| 代理阈值（唯一） | `rmctl.gripper_proxy.effective_thresholds` |
| 真机回填手册 | 本文 **§7** |

> **诚实分层（贯穿全文）**：本轮**真机窗口 blocked**
> （`workcell/baselines/network_plan.yaml`：`status: blocked`；ping `192.168.1.18` /
> `192.168.1.19` 均 100% 丢包，TCP 8080 连接超时）。
> 因此本文中所有"跑出来的数字"一律来自**离线代理**（`mode=fake`），
> 它们**证明代码路径正确**，**不是真机结论**。
> 真机四项验收（真机 50 圈连续成功 / 真机 trace 来自真机执行 /
> 真机失败给出下一步 / 真机单连接多段建连）**本轮未实证**，
> 回填入口见 **§7 真机回填手册**；口径声明见 **§8**。
>
> 所有命令先 `source scripts/rmenv.sh`（设置 `PYTHONPATH` / `RM_WS`），
> 命令行下文省略这一行。

---

## 1. 工艺段定义

### 1.1 每圈的段序列

规格文件：**`workcell/tasks/pick_place.yaml`**（`version: 1`，`iterations: 50`，
7 段）。加载器唯一入口：`rmctl.process.spec.load_task_spec`。

| 顺序 | 段名 | 类型 | 动作 / 目标 | 期望判定 | 说明 |
|---|---|---|---|---|---|
| 1 | `approach_pick` | `motion` | `goal_joints_deg: [0, 20, 30, 0, 0, 0, 0]` | — | 走到取件位 |
| 2 | `grasp` | `gripper` | `action: close`，`speed: 100`，`force: 100` | `grasp_ok` | 夹取并判定 |
| 3 | `lift` | `motion` | `goal_joints_deg: [0, 25, 35, 0, 0, 0, 0]` | — | 抬起 |
| 4 | `place_approach` | `motion` | `goal_joints_deg: [10, 20, 30, 0, 0, 0, 0]` | — | 走到放件位 |
| 5 | `release` | `gripper` | `action: open`，`speed: 100` | `grasp_empty` | 松开并判定 |
| 6 | `retreat` | `motion` | `goal_joints_deg: [0, 20, 30, 0, 0, 0, 0]` | — | 撤离 |
| 7 | `home` | `motion` | `goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]` | — | **回到 home** |

**每圈必须回 home**：一段序列的**最后一段必须是 `home`**，
且必须**成功执行**。否则本圈判失败（失败类型 `home_not_reached`，
`NEEDS_HUMAN`）并**停循环**交人工。这保证下一圈的规划起点与第一圈一致
（`home_joints_deg`）。

### 1.2 规格字段含义

| 字段 | 含义 / 约束 |
|---|---|
| `version` | 规格 schema 版本（当前 `1`） |
| `name` | 工艺名（CLI 位置参数只接受 `pick_place`） |
| `iterations` | 圈数，**上限 50**（`--count` 可覆盖 1..50，越界 → 退出码 2） |
| `home_joints_deg` | 起始 / 回归位姿，**单位为度** |
| `segments[].name` | 段名（出现在 trace 的 `segment` 字段） |
| `segments[].type` | `motion` / `gripper` / `judge`（其它值 → `segment_type_invalid`） |
| `segments[].goal_joints_deg` | 运动段目标关节角，**单位为度** |
| `segments[].goal_pose` | 运动段笛卡尔目标 `[x, y, z, r, p, y]`（米 + 弧度），**可选**；与 `goal_joints_deg` **二选一**（都给或都缺 → blocked，代码不猜） |
| `segments[].action` | 夹爪段动作：`close` / `open` |
| `segments[].speed` / `force` | 夹爪速度 / 力度（0–1000 标度） |
| `segments[].expect` | 期望判定结果（`grasp_ok` / `grasp_empty` …），判定后与之比对 |

**单位与换算**：关节角一律**度**；度 ↔ 弧度的换算只在客户端层
（`rmctl.process.clients`）与 runner 的"最后一点关节角回读"处发生，
规格层只做值域校验。

**恢复上限不在本文件**：`max_regrasp` / `max_stall_retry` / `max_offline_retry`
的**唯一来源**是 `workcell/rm_workcell.yaml::gripper.max_regrasp` 与
`rm_hw_bridge.gripper_state.RECOVERY_LADDER`（默认上限）。
规格里**出现**这些字段 → 加载器**直接 blocked**（逐字段指名），
杜绝第二处配置源。

---

## 2. 分层与依赖

### 2.1 分层位置

工艺层**不新增第五层**，落在既有**体验层** `rmctl/process/`：

```
rmctl/process/
├── spec.py      # 任务规格加载与校验（唯一入口 load_task_spec）
├── clients.py   # Planner / MotionClient / GripperClient 三协议 + fake/live 实现
├── runner.py    # 工艺状态机（段执行 → 判定 → 恢复阶梯 → 分级 → 循环）
└── trace.py     # trace 记录与只读回放
```

### 2.2 依赖方向（单向）

```
rmctl（体验层 / 工艺层）
   └── rm_executor（执行层：10 ms 单时钟执行器 / 会话层）
          └── rm_hw_bridge（硬件适配层：厂商解耦 / 夹爪状态真值）
```

`rm_safety`（安全层）由执行层编排，工艺层只消费其事件。
**反向依赖一律禁止**（`rm_hw_bridge` 不 import `rm_executor` / `rmctl`）。

### 2.3 三张"真值表"（唯一来源，禁止复制）

| 真值 | 唯一位置 | 谁在用 |
|---|---|---|
| **判定判据** | `rm_hw_bridge.gripper_state.judge` | 夹爪客户端 → runner |
| **恢复阶梯** | `rm_hw_bridge.gripper_state.RECOVERY_LADDER` | runner（表驱动，零复制） |
| **严重度排序 / 退出码** | `rm_executor.events.severity_rank` / `SEVERITY_EXIT_CODES` / `worst_severity` | runner 取"最严重者" |
| **代理阈值** | `rmctl.gripper_proxy.effective_thresholds` | CLI → runner 注入 |

> runner 源码里**不允许**出现本地 `SEVERITY_RANK = {...}` 之类的字典，
> 也不允许出现自己的判定判据 —— 两者都有单测（AST 断言）守卫。

---

## 3. 失败分级表

### 3.1 运动段失败（执行器事件 → 等级 → 退出码 → 是否自动恢复 → next_action）

严重度取自 `rm_executor.events`（执行器事件的**最严重者**，
`rm_executor.events.worst_severity`）；进程退出码口径
`rm_executor.contract.EXIT_CODES`：`ok → 0`、`blocked → 3`、`error → 4`。

| 执行器事件 / 段失败类型 | severity | 报告 status | 退出码 | 是否自动恢复 | next_action（可粘贴） |
|---|---|---|---|---|---|
| `feedback_stream_lag`（反馈滞后） | `RECOVERABLE` | blocked | 3 | **否**（不重试该段，继续后续段） | 观察反馈链路是否恢复；若持续超过停机阈值执行器将停机 |
| `follow_error_warn`（跟随误差预警） | `RECOVERABLE` | blocked | 3 | **否** | 观察机械臂跟随情况；若持续超过停机阈值执行器将暂停 |
| `follow_error_exceeded`（跟随误差超限，已 `transport.pause()`） | `NEEDS_HUMAN` | blocked | 3 | **否**（停循环交人工） | 立即人工确认机械臂状态（负载 / 碰撞 / 指令可达性）；执行器已调用 `transport.pause()`，排查后重跑 |
| `feedback_stream_lost`（反馈断流，已 `transport.stop()`） | `FATAL` | error | 4 | **否**（停循环交人工） | 检查反馈链路（SDK 连接 / UDP 订阅 / 网线）后重跑；执行器已调用 `transport.stop()` |
| `gripper_interlock_blocked`（闸门未放行） | `NEEDS_HUMAN` | blocked | 3 | **否** | 先完成夹爪状态判定后再下发运动 |
| `plan_failed`（规划异常 / 规划未成功） | `RECOVERABLE` | blocked | 3 | **否**（该段不重试，继续后续段） | 检查目标可达性与碰撞场景后重跑 |
| `trajectory_invalid`（规划结果无可用轨迹） | `NEEDS_PARAM` | blocked | 3 | **否**（停循环） | 确认规划器返回符合 `rm_executor` 契约的轨迹后重跑 |
| `trajectory_write_failed`（轨迹落盘失败） | `NEEDS_PARAM` | blocked | 3 | **否**（停循环） | 检查临时目录写权限后重跑 |
| `trajectory_contract_failed`（轨迹不符合执行层契约） | `NEEDS_PARAM` | blocked | 3 | **否**（停循环） | 重新规划该段后重跑 |
| `motion_failed`（执行异常 / 报告非 ok） | 取执行器事件最严重者；无事件且 `status == error` → `FATAL`，否则 `RECOVERABLE` | blocked / error | 3 / 4 | `RECOVERABLE` → 继续后续段；其余停循环 | 查看执行报告的 `evidence` 后重跑 |
| `gripper_action_failed`（夹爪动作异常 / 失败） | `RECOVERABLE` | blocked | 3 | **否**（停循环） | 检查夹爪接线与配置后重跑 |
| `judge_failed`（判定调用异常） | `RECOVERABLE` | blocked | 3 | **否**（停循环） | 检查夹爪接线后重跑 |
| `gripper_indeterminate`（判定结论未定：`needs_param` / `unknown`） | `NEEDS_PARAM` | blocked | 3 | **否**（停循环；闸门保持 `UNJUDGED`） | 补齐夹爪标定 / 确认夹爪状态后重跑 |
| `gripper_mismatch`（判定与 `expect` 不符） | 取恢复阶梯该步的 severity（见 §3.2） | blocked | 3 | **阶梯允许时才重试**（见 §3.2） | 阶梯该步的 `next_action`（可粘贴） |
| `home_not_reached`（本圈未成功回到 home） | `NEEDS_HUMAN` | blocked | 3 | **否**（停循环） | 确认机械臂已回到 home 位姿后再继续（人工复位后重跑） |
| `segment_type_invalid` / `spec_no_segments` | `NEEDS_HUMAN` / `NEEDS_PARAM` | blocked | 3 | **否**（停循环） | 修正规格里的段类型后重跑 / 在 `workcell/tasks/*.yaml` 里补齐 `segments` 后重跑 |

**总则**：**只有 `RECOVERABLE` 且阶梯允许时才自动恢复**；
`NEEDS_PARAM` / `NEEDS_HUMAN` / `FATAL` **一律停循环交人工**。
循环级 `reason` / `next_action` 取**所有失败中最严重者**的那条。

### 3.2 夹爪判定（六类判定 × 恢复阶梯 × severity × 是否重试 × next_action）

阶梯来源：`rm_hw_bridge.gripper_state.RECOVERY_LADDER`（表驱动，runner 不复制）。
上限来源：`workcell/rm_workcell.yaml::gripper.max_regrasp`（默认 2）、
`DEFAULT_MAX_STALL_RETRY`（1）、`DEFAULT_MAX_OFFLINE_RETRY`（1）。

| 判定结果 | 阶梯第 1 步（未超上限） | severity | 自动重试？ | 上限用尽后（最后一步） | severity | next_action（模板原文） |
|---|---|---|---|---|---|---|
| `grasp_empty`（夹空） | 第 0 步 | `RECOVERABLE` | **是**（重抓：先 `open` 再 `close`） | 升级人工步 | `NEEDS_HUMAN` | 可重试：`退回到安全开口后重抓（第 {attempt}/{max_regrasp} 次尝试）`；升级人工：`重抓次数已用尽（{attempt}/{max_regrasp}），需人工确认工件与夹爪状态` |
| `stall`（堵转） | 第 0 步 | `RECOVERABLE` | **是**（松手后重试闭合） | 升级人工步 | `NEEDS_HUMAN` | 可重试：`松手后重试闭合（第 {attempt}/{max_stall_retry} 次尝试）`；升级人工：`松手重试后仍堵转（{attempt}/{max_stall_retry}），需人工检查夹爪机械卡滞` |
| `overtemp`（过温） | — | `NEEDS_HUMAN` | **否** | 同左（单步阶梯） | `NEEDS_HUMAN` | `停机冷却（当前 {temp_c} ℃），禁止自动重试；温度回落后由人工复位` |
| `overcurrent`（过流） | — | `NEEDS_PARAM` | **否** | 同左（单步阶梯） | `NEEDS_PARAM` | `确认夹爪使用外部 24 V 供电并与法兰共地（法兰供电上限 1.5 A < 额定 1.9 A），再重新下发夹爪动作` |
| `offline`（离线） | 第 0 步 | `RECOVERABLE` | **是**（重连后重试） | 升级人工步 | `NEEDS_HUMAN` | 可重试：`夹爪协议已开但设备离线：重连后重试（第 {attempt}/{max_offline_retry} 次）`；升级人工：`夹爪离线重试已用尽（{attempt}/{max_offline_retry}），需人工检查线缆与外部供电` |
| `offline_disabled`（协议未开） | — | `NEEDS_PARAM` | **否** | 同左（单步阶梯） | `NEEDS_PARAM` | `夹爪协议未开（disable）：先开启末端 Modbus-RTU 或 RM Plus 协议，再重试` |
| `needs_param`（标定缺失） | — | `NEEDS_PARAM` | **否**（**不查阶梯**，绝不重抓） | 同左 | `NEEDS_PARAM` | `补齐配置项（force_threshold, grasp_position_window, position_mm_per_unit, force_scale）后重跑夹爪判定` |
| `unknown`（结论未知） | — | `NEEDS_PARAM` | **否**（**不查阶梯**） | 同左 | `NEEDS_PARAM` | `补齐夹爪标定 / 确认夹爪状态后重跑` |

> `needs_param` / `unknown` 属"结论未定"分支：**不查恢复阶梯**，
> 因此**绝不会被当成 `grasp_empty` 重抓**（`open` / `close` 调用计数不增）。
> `grasp_ok` / `grasp_empty` 等"结论确定"的判定（`GATE_RESETTABLE_RESULTS`）
> 才会复位运动闸门，见 §4。

---

## 4. 互锁与闸门

闸门实现：`rm_hw_bridge.gripper_state.MotionGate`（进程内单例
`shared_motion_gate()`）。

| 时刻 | 闸门状态 | 含义 | 运动段是否放行 |
|---|---|---|---|
| 夹爪动作**期间** | `GRIPPER_BUSY` | 夹爪动作在途 | **拒绝**（事件 `gripper_interlock_blocked`） |
| 动作结束、判定**未完成** | `GRIPPER_UNJUDGED` | 动作完成但状态未判定 | **拒绝** |
| 判定**完成**（结论确定） | `IDLE` | 可下发运动 | 放行 |

**判定完成后必须复位闸门**：只有"结论确定"的判定
（`grasp_ok` / `grasp_empty` / `stall` / `overtemp` / `overcurrent` / `offline`）
才调 `gate.finish_action()` 把闸门恢复 `IDLE`；
`needs_param` / `unknown` **不复位**（保持 `GRIPPER_UNJUDGED`）并**停循环**。

若不复位，后续运动段会被执行层以 `gripper_interlock_blocked`
（`NEEDS_HUMAN`）拒绝 —— 这是**故意的**：宁可停机交人工，也不允许
"在夹爪状态未知时继续运动"。

> **已知限制**：`MotionGate` 是**进程内单例**。`rmctl` 与 `rm_runtime`
> 分进程部署时，闸门**静默失效**。本切片采用**同进程编排**
> （`rmctl run pick_place` 内部同时持有闸门与执行器）。

---

## 5. trace 格式与回放

### 5.1 文件形态

JSONL，**一行一事件**（UTF-8，追加写），`schema_version = 1`（**冻结**，
字段口径变化必须升版本）。默认路径：`build/run/pick_place_trace.jsonl`
（`--trace` 可覆盖）。

### 5.2 事件字段表

| 字段 | 类型 | 含义 |
|---|---|---|
| `schema_version` | int | 恒为 `1` |
| `ts` | float | 时间戳（Unix 秒） |
| `iteration` | int | 圈次（从 1 起；非循环事件为 0） |
| `phase` | str | `plan` / `execute` / `gripper` / `judge` / `recover` / `loop` |
| `segment` | str | 段名（如 `grasp`）；非段事件为空串 |
| `status` | str | `ok` / `blocked` / `error` |
| `severity` | str | `INFO` / `RECOVERABLE` / `NEEDS_PARAM` / `NEEDS_HUMAN` / `FATAL` |
| `reason` | str | 判定依据 / 失败原因（**原样保留，绝不改写**） |
| `next_action` | str | **可粘贴**的下一步 |
| `evidence` | list | 客观证据（键值行 / 字符串） |
| `report` | dict | *可选*：运动段报告摘要 |

写盘时**只补元数据**（缺 `ts` / `severity` / `status` / `phase` / `iteration` 才补），
`reason` / `next_action` / `evidence` **绝不改写**。
写失败（磁盘满 / 只读）不抛裸异常：记入 `TraceWriter.errors`，
工艺主流程不被拖垮。

### 5.3 回放用法

```bash
python3 -m rmctl run --replay build/run/pick_place_trace.jsonl
```

回放输出：逐事件人读时间线 + `已解析 / 跳过` 计数 +
**最后一条非空 `next_action`**（可直接粘贴执行）。

**回放不下发任何运动指令**：回放实现 `rmctl.process.trace.replay_trace`
**零 import** `rm_executor` / `rm_hw_bridge` / `ctypes` / `socket`
（单测用 AST 断言守卫），只读 JSONL 文件、只做计数与抽取。
`--replay` 与 `--traj` / `--live` / `--count` / `--task-file` / `--trace` /
`--report` **互斥**（同时给出 → 退出码 2）。

**坏行处理**：非 JSON 行 / 非字典行被跳过并计入 `skipped` 与 `warnings`
—— 现场看到"跳过数 > 0"本身就是"trace 被截断 / 被写坏"的信号。

---

## 6. 离线证据链

### 6.1 复跑命令

```bash
python3 scripts/pick_place_offline_evidence.py --out build/reports/s06_pick_place_offline.json
```

退出码 `0` = 四段证据全部通过。报告落
`build/reports/s06_pick_place_offline.json`。

### 6.2 四段证据与本次实测数字

| 证据段 | 内容 | 本次实测 |
|---|---|---|
| **A. 50 圈全流程** | 真实厂商规划 + 真实四道校验 + 假运动执行 + 假夹爪，`mode=fake` | `status=ok`，**50/50 圈**完成，`350` 段执行（7 段 × 50 圈），`100` 条判定（每圈 2 条），`50` 圈全部回到 home，`0` 次互锁阻塞，`0` 条失败；逐圈耗时 min `0.427 s` / mean `0.467 s` / max `0.918 s`；总墙钟 `23.3 s`；trace `550` 事件、`213 KB` |
| **B. 六类故障分支** | 只注入假夹爪的**物理状态**（寄存器 / 在线位），判定与严重度仍唯一来自 `judge` + `RECOVERY_LADDER` | **6/6 命中预期 severity 与 next_action**：`grasp_empty` → `RECOVERABLE` → 自动重抓 1 次 → 最终 `INFO/ok`；`stall` → `RECOVERABLE` → 重试 1 次 → 升级 `NEEDS_HUMAN/blocked`；`overtemp` → `NEEDS_HUMAN/blocked`（不自动重试）；`overcurrent` → `NEEDS_PARAM/blocked`（不自动重试）；`offline` → `RECOVERABLE` → 重试 1 次 → 升级 `NEEDS_HUMAN/blocked`；`needs_param` → `NEEDS_PARAM/blocked` 且**闸门保持 `GRIPPER_UNJUDGED`** |
| **C. trace 回放对照** | 7 份 trace（1 主 + 6 故障）逐份回放 | 逐份 `file_lines == parsed == events == replay_lines`（主 trace `550` 行），`skipped=0`，severity 计数一致，主 trace `INFO 500 / RECOVERABLE 50`，**回放期间运动下发次数 before=250 / after=250（零下发）** |
| **D. live_window 声明** | 真机四项是否已实证 | `status=blocked`，四项 `proven=false`，**显式标注本轮未实证**，不给假结论 |

### 6.3 现场读法

1. 先看报告顶层 `status` 与 `failures`（`failures` 为空 = 全绿）。
2. 再逐段看 `assertions`（每条带 `name` / `passed` / `observed`，
   失败时能直接定位"哪一段哪一条"）。
3. `success_run.iteration_durations_s` 是逐圈耗时。
4. `fault_cases.cases[*].next_action` 是**可直接粘贴**的下一步原文。
5. trace：主 trace `build/run/pick_place_trace_offline.jsonl`（550 事件），
   故障分支各一份 `build/run/pick_place_trace_fault_<kind>.jsonl`；
   只读回放：`python3 -m rmctl run --replay build/run/pick_place_trace_offline.jsonl`。

### 6.4 模型 / 阈值来源

| 项 | 本次取值 | 说明 |
|---|---|---|
| `model_source` | `prebuilt` | 模型已就绪（`build/model` 已物化，未重建） |
| `thresholds_info.thresholds_source` | `proxy_default` | 代理补齐 `force_threshold` / `grasp_position_window` / `position_mm_per_unit` / `force_scale` |
| `proxy_used_keys` | 上述四键 | 工位配置缺失时由 `rmctl.gripper_proxy` 代理补齐并**标注来源** |
| 恢复上限 | `max_regrasp=2` / `max_stall_retry=1` / `max_offline_retry=1` | 唯一来源 = 工位配置 + `RECOVERY_LADDER` 默认值 |
| `calibration_pending` | `true` | 标定四字段仍为 `null`，真机回填后方为实测值 |

---

## 7. 真机回填手册

> **当前状态：五步全部「待填」**（真机窗口 blocked）。
> 每步给出**期望输出**与**失败时怎么办**。
> 现场执行时**逐条回填**"实测值 / 结论 / 证据路径"三列，再把
> `workcell/baselines/network_plan.yaml` 的 `status` 由 `blocked`
> 改为 `measured`（连通性实测确认后）。

### 第 1 步：确认单臂可达

```bash
python3 -m rmctl connect --arm left --json
```

- **期望输出**：一个 JSON 对象，含控制器状态快照（`status=ok`，
  IP / 端口 / 关节当前值 / 型号等 R004 全字段）。
- **失败时怎么办**：
  - `status=blocked` 且 reason 指向连通性 → 查网线 / IP / 端口，
    用 `--ip` / `--port` / `--udp-port` 覆盖基线值后重跑。
  - 超时 / 连接被拒 → 确认控制器上电、IP 同网段、`ping <ip>` 通。
  - 端口规划值来自 `workcell/baselines/network_plan.yaml`（RX75 左右臂配置）；
    **实连确认前不得声称连通性已验证**。
- **待填**：______（IP / 端口 / 实测可达性 / 证据）

### 第 2 步：回填夹爪标定四字段 + 错误码 status

编辑 `workcell/rm_workcell.yaml::gripper`（**唯一配置来源**）：

```yaml
gripper:
  # ---- 标定类字段：真机实测之前一律留 null（不得写入猜测值）----
  position_mm_per_unit: null   # 位置换算系数 mm/unit
  force_scale: null            # 力度换算系数 N/源力度单位
  force_threshold: null        # 夹到判定阈值（源力度单位）
  grasp_position_window: null  # 夹持位置窗口 [lo, hi]（mm）
```

- **期望输出**：四个字段被替换为**实测数值**（非 `null`）；
  随后 `python3 -m rmctl doctor | grep hw:gripper` **不再**出现
  『标定系数未回填: position_mm_per_unit, force_scale, force_threshold,
  grasp_position_window』。
- **失败时怎么办**：
  - 字段写成非数字 / 窗口不是 `[lo, hi]` → 配置读取器 **blocked**，
    逐字段指名，改回合法形式。
  - 仍 `null` → 判定走 `needs_param` 分支（**这是当前真机路径的实际行为**，
    不是 bug），必须回填后才能判 `grasp_ok` / `grasp_empty`。
- 同时把 `workcell/safety/error_codes.yaml` 的 `status` 由
  `unverified_offline` 改为 `verified_on_robot`，并逐条回填厂商码表
  （`code` / `class` / `level_hint` / `source`）。
- **待填**：______（四字段实测值 / 回填人 / 时间 / 证据）

### 第 3 步：夹爪自检（探针 + 100 次开合）

```bash
python3 -m rmctl gripper probe --arm left --json
python3 -m rmctl gripper cycle --count 100 --json
```

- **期望输出**：
  - `probe` → `status=ok`，含协议 / 波特率 / 从站 ID / 寄存器基数 /
    供电 / 代际线索。
  - `cycle` → `status=ok`，`count=100`，**超时次数 0**。
- **失败时怎么办**：
  - `probe` 报协议未开 / 供电异常 → 按 reason 修硬件（外部 24 V + 共地、
    打开末端 Modbus-RTU），**不要**改 `path` 绕过。
  - `cycle` 出现超时 → 查 `action_timeout_ms`（默认 2000 ms）与机械卡滞；
    连续超时按 `stall` 处理（`NEEDS_HUMAN`，人工检查机械卡滞）。
  - `--count` 合法范围 `1..10000`，越界 → 退出码 2。
- **待填**：______（probe 结论 / cycle 超时次数 / 证据）

### 第 4 步：真机 50 圈（核心验收）

```bash
python3 -m rmctl run pick_place --live --arm left --count 50 \
  --trace build/run/pick_place_trace_live.jsonl \
  --report build/reports/s06_pick_place_live.json
```

- **期望输出**：
  - 报告 `build/reports/s06_pick_place_live.json`：`status=ok`，
    `iterations_completed == 50`，`iterations_planned == 50`，
    `failures` 为空，`mode` 为真机（**不是** `fake`），
    `trace_path` 指向上面的 trace 文件。
  - trace `build/run/pick_place_trace_live.jsonl` 存在且事件来自**真机执行层**。
  - 记录**真实耗时**（`success_run` 口径的逐圈耗时）与**成功率**。
- **失败时怎么办**：
  - 退出码 3（blocked）/ 4（error）→ 读报告 `reason` / `next_action` /
    `evidence`，按 §3 表对号入座；再 `python3 -m rmctl run --replay
    build/run/pick_place_trace_live.jsonl` 看最后一条非空 `next_action`。
  - 建连失败 → 回到第 1 步；标定相关失败 → 回到第 2 步；
    夹爪动作失败 → 回到第 3 步。
  - `--count` 越界（>50）→ 退出码 2，改成 1..50。
  - 注意：真机路径**单连接多段**（`rm_executor.session`），
    一次连接完成 50 圈 × 7 段，**不重复建连**。
- **待填**：______（成功率 / 真实耗时 / 报告路径 / trace 路径 / 结论）

### 第 5 步：故障注入（人为放空工件 → 重抓 → 用尽转人工）

真机窗口内**人为放空工件**（取件位不放工件），然后：

```bash
python3 -m rmctl run pick_place --live --arm left --count 1 \
  --trace build/run/pick_place_trace_live.jsonl \
  --report build/reports/s06_pick_place_live_fault.json
python3 -m rmctl run --replay build/run/pick_place_trace_live.jsonl
```

- **期望输出**：
  - 第一次：判定 `grasp_empty` → `RECOVERABLE` → **自动重抓**
    （先 `open` 再 `close`，`max_regrasp=2`）。
  - 放空工件持续存在 → 重试用尽 → 升级 `NEEDS_HUMAN`，
    停循环，`next_action` 形如
    `重抓次数已用尽（2/2），需人工确认工件与夹爪状态`（**可粘贴**）。
  - 回放输出里的**最后一条非空 `next_action`** 与上面一致。
- **失败时怎么办**：
  - 没看到重抓 → 检查 `max_regrasp`（唯一来源
    `workcell/rm_workcell.yaml::gripper.max_regrasp`）是否被改成 0。
  - 出现了重抓但没有升级人工 → 检查重试计数与上限是否一致。
  - 判定是 `needs_param` 而不是 `grasp_empty` → 回到第 2 步回填标定。
- 离线对照（**代理，非真机结论**）：
  `python3 -m rmctl gripper judge --fault grasp_empty --count 1 --json`
  —— 只验证判定文案与 `next_action` 模板，不是真机结论。
- **待填**：______（实际判定 / 重抓次数 / 最终 severity / next_action 原文）

---

## 8. 口径声明（诚实分层）

### 8.1 本轮**未实证**的四项（真机相关）

| 项 | 状态 | 原因 |
|---|---|---|
| 真机 50 圈连续成功（无人干预） | **未实证** | 真机窗口 blocked（`workcell/baselines/network_plan.yaml`：`status: blocked`） |
| 真机 trace 来自真机执行 | **未实证** | 同上 |
| 真机失败给出下一步 | **未实证** | 同上 |
| 真机单连接多段建连（`rm_executor.session`） | **未实证** | 同上 |

### 8.2 离线数字的口径

- 本文 §6 的所有数字一律 **`mode=fake`**（离线代理）：
  **真实厂商规划 + 真实四道校验 + 假运动执行 + 假夹爪**。
- 离线数字**证明代码路径正确**（段序列、判定、恢复阶梯、分级、循环、
  trace、回放），**不证明真机行为**。
- **不以离线代理冒充真机结论**：真机四项保持 `proven=false`，
  `live_window` 段显式 `status=blocked`。
- **R007（失败给出下一步）保持 `active`**，待真机回填（§7 第 4/5 步）后
  方可置 `validated`。

### 8.3 本轮已实证（离线范围内）

| 项 | 证据 |
|---|---|
| 声明式规格加载（逐字段 blocked、绝不补默认值） | `rmctl.process.spec.load_task_spec` + 单测 |
| 工艺状态机（段执行 → 判定 → 阶梯 → 分级 → 循环） | `rmctl.process.runner` + 单测 |
| 闸门复位红线（`needs_param` / `unknown` 不复位） | `tests/test_process_runner.py` |
| trace 记录与只读回放（零运动 / 零 SDK / 零网络） | `rmctl.process.trace` + AST 守卫单测 |
| 50 圈离线全流程 + 六类故障分支 | `build/reports/s06_pick_place_offline.json` |

---

## 9. 已知限制

1. **夹爪标定四字段仍为 `null`**
   （`position_mm_per_unit` / `force_scale` / `force_threshold` /
   `grasp_position_window`）。因此 `needs_param` 分支**是当前真机路径的
   实际行为**，不是异常路径；真机回填见 §7 第 2 步。
2. **MotionGate 是进程内单例**：`rmctl` 与 `rm_runtime` 分进程部署时，
   闸门**静默失效**（互锁不生效）。本切片采用**同进程编排**。
3. **live 路径的反馈泵此前从未启动**：`_run_live` 原来从不调用
   `start()`，导致反馈缓存恒空。本轮已在 `rm_executor.session`
   （`open` 里 `start`、`close` 里 `stop`，均幂等）修复，
   **报告与退出码不变**（缺陷修复不夹带行为变更）。
4. **规格加载器是白名单**：顶层与段字段出现**未知字段即 blocked**
   （防止恢复阶梯上限等第二处配置源）。
5. **恢复阶梯上限不注入时视为 0**（不自动重试，直接升级人工）——
   保守方向，宁可停机交人工。
6. **规划缓存**：50 圈 × 5 个运动段 = 250 次规划请求，但（起点, 目标）
   组合只有 8 种 → 实际只做 8 次真实规划（离线证据链 `planner_cache_entries=8`）。
7. **本文命令均为离线/真机两用形态**，但**真机形态（§7）本轮未执行过**；
   真机窗口首次执行时请对照 `python3 -m rmctl run pick_place --help`
   与 `python3 -m rmctl connect --help` / `python3 -m rmctl gripper --help`
   复核参数名（本文已按实现逐字校对）。
