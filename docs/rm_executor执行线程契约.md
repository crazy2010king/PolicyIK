# rm_executor 执行线程契约（10 ms 单时钟执行器）

> 适用范围：执行层包 `rm_executor`（进程角色 `rm_runtime`）。
> 本文是**可执行的契约**：每一条都能在代码或命令里对上号。
> 配套代码：`dualarm_ws/src/rm_executor/rm_executor/`（`contract.py` / `clock.py` /
> `resample.py` / `transport.py` / `feedback.py` / `executor.py` / `report.py` /
> `events.py` / `metrics.py` / `cli.py` / `runtime.py`）。

---

## 0. 诚实分层声明（先读这一节）

| 层级 | 是什么 | 不是什么 |
|------|--------|----------|
| **离线代理（fake）** | 假传输 + 脚本化反馈（镜像下发指令）的**确定性重放**。用于回归、契约验证、长跑等效负载。 | **不是真机结论**。跟随误差恒为 0、抖动是虚拟时钟的抖动，**不得**作为「真机通过」的证据。 |
| **真机（live）** | 同一 Executor + 同一报告 schema，真实下发到控制器。 | 本轮真机为**预约资源、不可达**，因此当前仓库内**没有**真机结论。 |

> 验收表述规则：**离线代理的证据只能写「离线代理通过」**，不能写「真机通过」。
> 真机结论只有在 §8 的真机窗口命令真实跑过、报告落盘后才成立。

---

## 1. 10 ms 单时钟语义与绝对截止时间

* **唯一时钟**：执行过程只有**一个**时钟（`rm_executor.clock.Clock`）。
  * `TickClock` —— 真实墙钟（真机路径默认）；
  * `ManualClock` —— 虚拟时钟（离线 fake 默认，可逐 tick 确定性重放）。
* **绝对截止时间栅格**：第 k 拍的截止时刻 = `start + k * period`
  （`period` 来自 `workcell/rm_workcell.yaml::execution.period_s`，当前 10 ms）。
  * 超期的拍**不补睡、不顺延**：下一拍的截止时刻仍是 `start + (k+1)*period`，
    因此**不会累积漂移**（漂移只会一次性体现在被超期的那一拍上）。
* **等待阶段与流式阶段共享时间来源**：预就位/到位等待会推进时钟；流式阶段的
  栅格起点单独取「阶段 C 起点」，但**时间连续**（ManualClock 不会回退）。
* **线程分工（硬约束）**：

  | 线程 | 允许做什么 | 禁止做什么 |
  |------|-----------|-----------|
  | tick 线程 | 读**快照**、调 `transport.transmit`、记指标 | ❌ 任何 SDK/TCP 往返（含 `current_trajectory_type`） |
  | 反馈采集线程（`FeedbackPump`，10–20 ms） | 读 UDP/SDK → 拷贝快照 | ❌ 写事件日志 |
  | 控制器看门狗线程（`ControllerWatchdog`，15 ms） | 查 `current_trajectory_type` → 事件经队列交接 | ❌ 直接写 `EventLog` |
  | 事件交接 | 看门狗线程 → 队列 → tick 线程并入 `EventLog` | ❌ 并发写事件日志 |

  理由：SDK/TCP 往返一旦进 tick 线程，10 ms 栅格的抖动 p99 < 2 ms 红线会被
  自己破坏。

---

## 2. 轨迹 JSON 契约

必需字段（`units` / `schema_version` 缺失 → **推定 + 显式标记** `schema_inferred=true`）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `group` | str | 固定 `"rm75_arm"` |
| `joint_names` | list[str] | 固定 7 个：`joint1..joint7` |
| `times` | list[float] | 单位 **秒**，长度 ≥ 2，**严格递增** |
| `points` | list[list[float]] | 每行 **7** 个关节角，单位 **rad**，行数 == `len(times)` |
| `units` | dict | 固定 `{"times": "s", "points": "rad"}`（缺失 → 按 rad 推定） |
| `schema_version` | int | 固定 `1`（缺失 → 推定） |
| `verify` | dict | 上游四道校验报告，须含 `"ok": true`；**缺失 → blocked** |

示例（合法轨迹）：

```json
{
  "schema_version": 1,
  "group": "rm75_arm",
  "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
  "units": {"times": "s", "points": "rad"},
  "times": [0.0, 0.05, 0.1],
  "points": [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
    [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
  ],
  "verify": {"ok": true, "checks": {}}
}
```

**单位换算只有一处**：rad → deg 用 `rm_hw_bridge.bridge.RAD2DEG`
（`executor._rad2deg`）；取不到即 `blocked` + 可粘贴 PYTHONPATH，
**绝不**退回 `math.degrees` 造第二处真相。

---

## 3. 执行阶段与冻结阈值

| 阶段 | 语义 | 失败后果 |
|------|------|----------|
| **A 预校验** | 轨迹契约 → `verify.ok == true` → 时间域限值复检（位置/速度/加速度/jerk，限值唯一来源 = `rm_hw_bridge.limits`） | 任一不过 → **不下发任何指令**，`blocked` |
| **B 预就位** | `transport.preposition(首点, v=pre_position_speed)` → 等容差**持续**保持时长 | 超时 → `blocked`（`NEEDS_HUMAN`），**未下发任何 tick** |
| **C tick 流** | 按 10 ms 栅格**逐拍** `transport.transmit(...)`（绝对截止时间） | 传输失败 → 立即 `stop()` + `FATAL` → `error` |
| **D 到位判定** | 角误差与角速度**同时**满足并**持续**保持时长 | 超时 → `blocked` |
| **E 收尾** | 末点保持若干拍 → 跟随误差统计 → 限值恢复（无下发则记录证据） → 输出统一报告 | —— |

### 冻结阈值表（`rm_executor.contract`，改这里 = 改契约）

| 阈值 | 值 | 含义 |
|------|----|------|
| `PRE_POSITION_TOL_DEG` | 0.5 ° | 预就位容差 |
| `PRE_POSITION_HOLD_MS` | 200 ms | 预就位保持时长 |
| `FOLLOW_WARN_DEG` | 2.0 ° | 跟随误差告警（`RECOVERABLE`） |
| `FOLLOW_STOP_DEG` | 5.0 ° | 跟随误差停机（`NEEDS_HUMAN` + pause） |
| `STREAM_WARN_MS` | 50 ms | 反馈断流告警（`RECOVERABLE`） |
| `STREAM_STOP_MS` | 200 ms | 反馈断流停机（`FATAL` + stop） |
| `ARRIVAL_TOL_DEG` | 0.1 ° | 到位角误差 |
| `ARRIVAL_SPEED_DEG_S` | 0.5 °/s | 到位角速度 |
| `ARRIVAL_HOLD_MS` | 300 ms | 到位保持时长 |

执行参数（`period_s` / `follow` / `trajectory_mode` / `radio` /
`pre_position_speed`）**唯一来源** = `workcell/rm_workcell.yaml::execution`；
缺字段即 `blocked`，执行层**不补默认值**。

---

## 4. 事件五级与证据字段

事件严重度（五级，S04 的 L1–L4 安全链路直接消费）：

| 级别 | 含义 | 典型动作 |
|------|------|----------|
| `INFO` | 正常记录 | —— |
| `NEEDS_PARAM` | 参数/配置/契约不满足 | 停下并给出可粘贴修复动作 |
| `RECOVERABLE` | 可恢复异常（跟随误差 > 2°、断流 > 50 ms） | 记录 + 降速/告警，继续观察 |
| `NEEDS_HUMAN` | 需要人工介入（跟随误差 > 5°、预就位未到位） | `pause` + 人工确认 |
| `FATAL` | 立即停机（断流 > 200 ms、控制器抢轨迹、传输失败） | `stop()` |

单条事件的字段（固定 6 个，顺序固定）：

```json
{"type": "follow_error_stop", "severity": "FATAL", "t": 12.34,
 "reason": "跟随误差 5.31° 超过停机阈值 5.00°",
 "evidence": ["follow_err_deg=5.312", "follow_stop_deg=5.0"],
 "next_action": "确认机械臂无干涉后重新下发"}
```

* `status`（报告顶层）由**最严重**事件推导：`FATAL → error`（退出码 4）、
  `RECOVERABLE/NEEDS_PARAM/NEEDS_HUMAN → blocked`（退出码 3）、无事件 → `ok`（0）。
* `blocked` / `error` 的报告**必须**带非空 `reason` + 可粘贴 `next_action`。

---

## 5. 报告 schema（离线与真机共用）

顶层 **7 个键**（顺序固定）：

```json
{
  "status": "ok|blocked|error",
  "reason": "",
  "next_action": "",
  "evidence": ["..."],
  "mode": "fake|live",
  "metrics": { "...15 个键..." },
  "events": [ {"type": "...", "severity": "...", "t": 0.0,
               "reason": "...", "evidence": ["..."], "next_action": "..."} ]
}
```

`metrics` 的 **15 个键**（顺序固定，缺值一律 `null`，**绝不返回 0 冒充**）：

| 键 | 含义 |
|----|------|
| `ticks` | 下发拍数 |
| `duration_s` | 实际耗时（秒） |
| `rt_factor` | 实际耗时 / 计划耗时 |
| `tick_jitter_p50_ms` / `tick_jitter_p95_ms` / `tick_jitter_p99_ms` / `tick_jitter_max_ms` | tick 抖动分位与峰值（毫秒） |
| `follow_err_rms_deg` / `follow_err_max_deg` | 跟随误差 RMS / 峰值（度） |
| `arrival_err_deg` / `arrival_hold_ms` | 到位角误差 / 到位保持时长 |
| `sched_fifo` | 是否用了 FIFO 调度（实时性提示） |
| `pre_position_hold_ms` | 预就位实际保持时长 |
| `hold_ticks` | 末点保持拍数 |
| `planned_duration_s` | 计划时长（秒） |

**同一份报告既是离线证据也是真机证据**：字段完全一致，只有 `mode` 与数值来源不同。

---

## 6. 两档断流口径（50 ms / 200 ms）与其取舍

* **判据用「帧计数是否变化」**（不是反馈时间戳）：与时钟域解耦，fake / live 同口径，
  ManualClock 可确定性重放；`FeedbackPump.last_age_ms()` 提供毫秒口径的 age。
* **50 ms → `RECOVERABLE`**：UDP 主动上报周期是 5 ms 的倍数，正常情况下丢几帧
  是常态（网络抖动、调度抖动）。**50 ms** 意味着「已经明显落后于 10 ms 栅格」，
  此时**记录 + 告警**，因为此时跟随误差通常还远小于 2°，继续跑比急停更安全
  （急停会打断正在进行的轨迹，留下未完成的状态）。
* **200 ms → `FATAL` + `stop()`**：200 ms ≈ 20 个栅格拍，此时反馈**已经失去
  闭环意义**——执行器无法知道机械臂在哪，继续下发就是「盲发」。宁可停机，
  也不在无反馈状态下继续运动。
* **取舍**：两档之间的 50–200 ms 区间选择「告警但不停」，是因为**急停的代价
  （轨迹中断 + 状态未知）大于短时跟随误差的代价**；而 200 ms 之后，代价关系
  反转（盲发的代价更大），因此必须停机。

跟随误差同理分两档：**> 2°** 告警（还能靠控制器跟随追上）、
**> 5°** 停并请求人工（已经明显偏离，继续下去可能撞限位）。

---

## 7. 入口与命令

### 7.1 离线代理（默认，零真机依赖）

```bash
source scripts/rmenv.sh
python3 -m rm_executor.cli --traj workcell/traj.json --mode fake --report /tmp/rep.json
```

* 退出码：`0` ok / `3` blocked / `4` error；
* `--duration 600` → **长跑**：重复执行该轨迹至等效 600 s，并合并成一份报告
  （每一轮都是完整的「预就位 → 流式 → 到位」，因此长跑的每一段都受同一套
  限值与闭环判据约束）；
* `--realtime` → 用真实墙钟而不是确定性虚拟时钟；
* `--allow-unverified` → 放行**缺 `verify`** 的轨迹（**仅限离线假执行**，
  放行会写进 `evidence`；真机路径**拒绝**该放行）。

### 7.2 真机窗口入口（进程角色 `rm_runtime`）

```bash
python3 -m rm_executor.runtime --arm left --traj workcell/traj.json --report /tmp/live_rep.json
```

* **强制 live**（没有 `--mode`，也没有 `--allow-unverified`）；
* 按 `rm_hw_bridge.connect` 的接口建链（TCP + UDP 主动上报）；
* **控制器不可达时诚实降级**：`status="error"` + reason「控制器不可达：…」+
  可粘贴 `next_action`，**不伪造任何快照数值**（实测类指标一律 `null`），
  退出码 `4`。

### 7.3 rmctl 侧接线

```bash
rmctl run --traj workcell/traj.json --executor rm_executor   # 新执行器（离线 fake）
rmctl run --traj workcell/traj.json --live                    # 新执行器（真机）
rmctl run --traj workcell/traj.json                           # 【legacy】旧 MockExecutor 回放
```

> **legacy 路径（`MockExecutor`）保留但不回退，且不作验收证据**：
> 它没有四道校验、无限值复检、没有反馈闭环、没有真机下行。
> `rmctl/executor.py` 的 `SdkExecutor` 已标注「真机路径已迁至 rm_executor」，
> 不再实现任何真机逻辑（避免两套真相）。

---

## 8. 真机窗口操作手册（真机可达时照此执行）

### 8.1 前置

```bash
source scripts/rmenv.sh          # PYTHONPATH 含 rmctl / rm_hw_bridge / rm_executor
python3 -m rmctl.cli doctor      # 确认 SDK、契约、限值来源都在位
python3 -m rmctl.cli connect --arm left --dry-run   # 确认 ip/port/udp_port 来自哪
```

### 8.2 出轨迹（带四道校验报告）

```bash
python3 -m rmctl.cli plan <q...> --out workcell/traj.json
python3 -m rmctl.cli check --traj workcell/traj.json     # 必须 ok
```

### 8.3 跑（真机）

```bash
python3 -m rm_executor.runtime --arm left \
    --traj workcell/traj.json --report /tmp/live_rep.json
echo "exit=$?"
```

### 8.4 期望看到什么

| 现象 | 含义 |
|------|------|
| 退出码 `0`，报告 `status="ok"` | 全程无保护停；`events` 为空或只有 `INFO` |
| `metrics.tick_jitter_p99_ms < 2` | 10 ms 栅格抖动达标 |
| `metrics.arrival_err_deg < 0.1` | 到位判定达标 |
| `metrics.follow_err_rms_deg` 平稳 | 跟随误差在闭环内 |
| 退出码 `3` / `4` | 见报告 `reason` 与 `next_action` |

### 8.5 失败时按 `next_action` 排查

| reason 片段 | 排查方向 |
|-------------|----------|
| 「控制器不可达」 | 机械臂上电？网线？`ip:port` 与 `network_plan.yaml` 一致？SDK 在位？ |
| 「轨迹未携带 verify 报告」 | 重新用 `rmctl plan --out` 出轨迹（四道校验结果要随轨迹落盘） |
| 「执行配置不可用」 | `workcell/rm_workcell.yaml::execution` 五个字段齐全 |
| 「反馈断流」 | UDP 主动上报是否开通（`rm_set_realtime_push`）、本机 IP 是否可达 |
| 「跟随误差超阈值」 | 负载/干涉/速度过高；先降速重定时再跑 |
| 「控制器抢轨迹」 | 控制器上有其它规划在跑，先清空（`RM_NO_PLANNING_E` 之外的类型即为抢轨迹） |

> 排查期间**不要**把离线代理报告当作真机结论；真机结论只认本节命令落盘的报告。

---

## 9. 退出码与进程角色

| 退出码 | 含义 |
|--------|------|
| `0` | `status="ok"` |
| `3` | `status="blocked"`（需要人工/外部条件：缺文件、缺 verify、配置缺字段…） |
| `4` | `status="error"`（运行错误：限值越界、控制器不可达、传输失败…） |

进程角色：`rm_executor.cli`（通用入口）/ `rm_executor.runtime`（**rm_runtime**，
真机窗口角色）。
