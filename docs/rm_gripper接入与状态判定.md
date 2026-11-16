# rm_gripper 接入与状态判定（RMG22-65 两指平行夹爪）

> 面向"第一次接这台夹爪的人"：照着本文，可以在真机窗口内从零把官方两指夹爪
> **RMG22-65** 接进本项目，并知道**每一条结论哪些已实证、哪些还没有**。
>
> 关联切片：M001-3kzz40 / S05（夹爪接入与状态判定）；
> 关联代码：`dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/gripper.py`（SDK 适配层）、
> `dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/gripper_state.py`（纯域逻辑：归一化 / 判定 / 恢复阶梯 / 运动闸门）、
> `dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/gripper_config.py`（配置读取器）、
> `dualarm_ws/src/rmctl/rmctl/cli.py`（`rmctl gripper` 体验层）；
> 关联文档：`docs/rm_hw_bridge硬件适配层说明.md`（硬件适配层总览）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 / 文件 |
|---|---|
| 看离线证据链报告（三段断言 + 数字） | `python3 scripts/gripper_offline_evidence.py --out build/reports/s05_offline_gripper.json` |
| 探夹爪（协议 / 波特率 / ID / 寄存器基数 / 供电 / 代际） | `python3 -m rmctl gripper probe --arm left --json` |
| 夹爪开合自检（离线代理） | `python3 -m rmctl gripper cycle --count 100 --json` |
| 六类判定自检（离线代理） | `python3 -m rmctl gripper judge --fault each --count 10 --json` |
| 体检夹爪配置与标定回填状态 | `python3 -m rmctl doctor \| grep hw:gripper` |
| 改夹爪配置（**唯一来源**） | `workcell/rm_workcell.yaml` 的 `gripper:` 段 |
| 判定真值（唯一） | `rm_hw_bridge.gripper_state.judge` |

> **诚实分层（贯穿全文）**：本轮**真机窗口 blocked**
> （`workcell/baselines/network_plan.yaml`：ping `192.168.1.18` / `192.168.1.19` 均 100% 丢包）。
> 因此本文中所有"跑出来的数字"一律来自**离线代理**（`mode=fake`），
> 它们**证明代码路径正确**，**不是真机结论**。真机四项验收
> （真实开合 100 次 / 真实六类判定 / 真实互锁拒绝 / 真实标定）**本轮未实证**，
> 回填入口见 §8。

---

## 1. 接线与供电

| 项 | 结论 | 依据 |
|---|---|---|
| 供电 | **必须外部 24 V**，且**与法兰共地** | 法兰供电上限 **1.5 A** < RMG22-65 额定 **1.9 A**，法兰供电必然掉压 |
| 工具端电源 | 默认 **0 V**，必须**主动打开**（`rm_set_tool_voltage`），并用 `rm_get_tool_voltage` **回读**确认 | 工具端电压寄存器只回读、不可假定 |
| 工具端 RS485 | **只走 Modbus-RTU**；官方夹爪快路径是控制器内置协议，与末端 Modbus 功能**互斥** | 厂商手爪控制协议与末端 Modbus 功能同一时刻只能开一条 |
| 工具端电压档位 | `rm_set_tool_voltage` 支持 0 V / 5 V / 12 V / 24 V | 白名单符号（支持层） |
| 地线 | 外部 24 V 负极与法兰 GND **必须连通** | 共地，否则 RS485 差分信号无参考地 |

配置字段（唯一来源 `workcell/rm_workcell.yaml::gripper`）：

```yaml
gripper:
  model: RMG22-65
  power: external_24v     # 外部供电 + 共地（法兰 1.5 A < 额定 1.9 A）
  protocol: modbus_rtu    # 线路协议
  path: modbus            # 驱动路径：modbus | gripper_api | rm_plus
  device_id: 1            # Modbus 从站地址
  baud: 115200            # 波特率
  timeout_ms: 500         # 单次寄存器读写超时
  poll_interval_ms: 50    # 状态轮询周期
  action_timeout_ms: 2000 # 一次开/合动作的等待上限
  max_regrasp: 2          # 夹空/堵转后的最大重夹次数
  # ---- 标定类字段：真机实测之前一律留 null（不得写入猜测值）----
  position_mm_per_unit: null
  force_scale: null
  force_threshold: null
  grasp_position_window: null
```

> **标定字段为 `null` 是合法状态**（不是配置错误）：配置读取器返回 `status=ok`，
> 判定层据此走 `needs_param`（见 §4）。**代码绝不补默认值**。

---

## 2. 协议与生命周期

### 2.1 三条路径的关系（互斥）

| 路径 | `path` | 用途 | 状态读取 |
|---|---|---|---|
| **Modbus-RTU 主路径** | `modbus` | 通用夹爪的**主路径**：读写厂商寄存器表（§3） | 有（读 6 个关键寄存器） |
| **官方夹爪快路径** | `gripper_api` | 控制器内置手爪协议（`rm_set_gripper_pick` / `rm_set_gripper_release` / `rm_get_gripper_state`） | 有（SDK 结构体，**mode 语义与手册不同**） |
| **RM Plus** | `rm_plus` | **仅作协议开关回读**（`rm_get_rm_plus_mode`），用来区分"协议未开（disable）"与"已开但设备离线（offline）" | **无**（`read_state` 直接返回 blocked） |

* 厂商手爪控制协议与末端 Modbus 功能**互斥**：同一时刻只能开一条。
* 三条路径的**模式码不等价**：手册状态码（`0x0014`，1–5）与 SDK `rm_gripper_state_t.mode`
  （1–6，多一个"闭合中力控停止"）**各自写死映射表**，判定层按路径选表解码，
  **不合并成一个表**（见 `gripper_state.mode_for_path`）。

### 2.2 生命周期（必须遵守）

```
rm_set_modbus_mode(port, baud, timeout_100ms)   # 开：协议配置会被控制器持久化
    → 读 / 写寄存器（FC03 / FC04，见 §3）
rm_close_modbus_mode(port)                      # 关：用完必须调用
```

* `rm_set_modbus_mode` 的配置**会被控制器持久化**，用完**必须** `rm_close_modbus_mode`，
  否则末端会一直停在 Modbus 模式（机械臂控制模式被顶掉）。
* 适配层把它收敛成 `GripperAdapter.open_protocol()` / `close_protocol()` /
  `with_protocol(fn)`（上下文管理器保证异常路径也会关协议）。

### 2.3 厂商返回码语义

| 返回码 | 语义 | 本项目的处理 |
|---|---|---|
| `0` | 成功 | — |
| `-1` | 数据发送失败 | `status=error` + 写明接口名与返回码 + 可粘贴下一步 |
| `-2` | 数据接收失败或**超时**（控制器未在超时时间内返回） | `status=error`；超时在开合自检里被单独计数（§7） |
| `-3` | 返回值解析失败（数据格式不正确或不完整） | `status=error` |
| `-4` | **四代控制器不支持该接口** | `status=blocked`（能力缺失，需换路径或人工确认控制器代际），**不**当普通错误重试 |

---

## 3. 寄存器表与故障码位

### 3.1 寄存器表（厂商《两指平行夹爪用户使用手册 V1.0》§4.1，0x0001–0x0014）

| 地址 | 名称 | 读写 | 含义 / 单位 |
|---|---|---|---|
| `0x0001` | 保存参数 | 读写 | 落盘参数 |
| `0x0002` | 恢复默认 | 读写 | 恢复出厂 |
| `0x0003` | 设备地址（ID） | 读写（重启生效） | Modbus 从站地址 |
| `0x0004` | 波特率档位 | 读写（重启生效） | 波特率档 |
| `0x0005` | 抓取模式 | 读写 | 0 单次 / 1 持续 |
| `0x0006` | 松开到最大开口 | 写 | 动作寄存器 |
| `0x0007` | 急停 | 写 | 动作寄存器 |
| `0x0008` | **清除错误码（需重启设备）** | 写 | **本项目红线：禁止写**（见 §5） |
| `0x0009` | 目标开口度 | 写 | 0–1000 |
| `0x000A` | 夹取速度 | 写 | 0–1000 |
| `0x000B` | 夹取力度 | 写 | 0–1000 |
| `0x000C` | 最大开口度 | 读写 | 0–1000 |
| `0x000D` | 最小开口度 | 读写 | 0–1000 |
| `0x000E` | 实际力度 | 只读 | 0–1000 标度 |
| `0x000F` | 实际位置 / 开口度 | 只读 | 0–1000 标度 |
| `0x0010` | 实际电流 | 只读 | mA |
| `0x0011` | 实际电压 | 只读 | V |
| `0x0012` | 实际温度 | 只读 | ℃ |
| `0x0013` | 夹爪错误码位图 | 只读 | 见 §3.3 |
| `0x0014` | 状态码 | 只读 | 1 最大空闲 / 2 最小空闲 / 3 停止空闲 / 4 正在闭合 / 5 正在张开 |

### 3.2 地址基数与功能码：**尚未实测**

* **地址基数（0-based / 1-based）未在真机确认**：手册按 1 写，PDU 是 0 基偏移。
  适配层用 `register_address(reg, base)` 支持两种基数（默认 `base=1`），
  **所用基数必须写进 evidence**；探针会**同时**按两种假设打分。
* **只读寄存器属 FC03（保持）还是 FC04（输入）未实测**：探针两条都试。
* **两者由 `rmctl gripper probe` 一次性解决**（见 §8.1）——本轮未实证。
* **红线写保护是"双闸门"**：既拦手册地址 `0x0008`，也拦"写出的线上地址恰为 `0x0008`"
  （基数猜错时宁可误伤，也绝不误清错误码）；拒绝路径 **0 次 SDK 写调用**。

### 3.3 故障码位（`0x0013` 位图，手册 §4.2.3.18）

| 位 | 掩码 | 含义 | 判定结果 |
|---|---|---|---|
| bit0 | `0x0001` | 堵转 | `stall` |
| bit1 | `0x0002` | 过温 | `overtemp` |
| bit2 | `0x0004` | 过流 | `overcurrent` |
| bit3 | `0x0008` | 驱动器运行故障 | `unknown`（需人工） |
| bit4 | `0x0010` | 内部通信故障 | `unknown`（需人工） |
| bit5–bit7 | — | 厂商保留位 | **忽略且不得报错** |

---

## 4. 归一化与判定表

### 4.1 归一化快照（`GripperState`）

| 字段 | 含义 | 单位 | 缺失时 |
|---|---|---|---|
| `path` | 协议路径 | - | 必填 |
| `mode` / `mode_raw` | 归一化模式 / 原始模式码 | - | `None` |
| `position_mm` | 开口度 | mm | 换算系数缺失 → `None` |
| `force_norm` | 归一化力度 | 0–1000 标度 | SDK 路径标度不明 → `None` |
| `force_N` | 力度 | N | `force_scale` 缺失 → `None` |
| `temp_c` | 温度 | ℃ | `None` |
| `current_mA` | 电流 | mA | `None` |
| `error_bits` / `error_flags` | 错误字 / 逐位解码 | - | 0 |
| `online` | 在线状态 | - | `None`（未知） |
| `protocol_state` | `unknown` / `disable` / `offline` / `online` | - | `unknown` |
| `raw` | 原始寄存器 / 结构体字段（原样留痕） | - | — |

### 4.2 六类判定（唯一真值 `rm_hw_bridge.gripper_state.judge`）

判定优先级（先命中先返回）：
**协议未开 → 已开但离线 → 错误位（过温 > 过流 > 堵转 > 驱动器/通信）→ 动作进行中 →
阈值缺失 → 夹到 / 夹空**。

| 结果 | 触发判据 | severity | 可重试 |
|---|---|---|---|
| `grasp_ok` 夹到 | 力度 ≥ `force_threshold` **且** 开口度落在 `grasp_position_window` | `OK` | 否 |
| `grasp_empty` 夹空 | 力度 < `force_threshold` | `RECOVERABLE`（重抓未用尽） | 是（≤ `max_regrasp`） |
| `stall` 堵转 | error bit0 | `RECOVERABLE`（重试未用尽） | 是（≤ 1 次） |
| `overtemp` 过温 | error bit1 | `NEEDS_HUMAN` | **否**（禁止自动重试） |
| `overcurrent` 过流 | error bit2 | `NEEDS_PARAM` | **否**（提示外部供电） |
| `offline` 离线 | `protocol_state=offline` 或 `online=false` | `RECOVERABLE` → 用尽后 `NEEDS_HUMAN` | 是（≤ 1 次） |
| `needs_param` | 阈值 / 换算系数缺失 | `NEEDS_PARAM` | 否 |
| `unknown` | 动作进行中 / 驱动器或通信错误位 | `NEEDS_HUMAN` | 否 |

> **「协议未开」与「已开但离线」必须区分**：前者 `protocol_state=disable`
> → `offline` + `NEEDS_PARAM`（先开协议），后者才是真正的离线（可重试）。
>
> **阈值缺失时绝不产出 `grasp_ok` / `grasp_empty`**——这是硬约束，
> §7 的离线证据链把它作为负例专门断言。

### 4.3 标定系数（实测前为 `null` → `NEEDS_PARAM`）

| 键 | 含义 | 实测前 |
|---|---|---|
| `position_mm_per_unit` | 开口度换算（mm / 源单位） | `null` |
| `force_scale` | 力度换算（N / 源单位） | `null` |
| `force_threshold` | 夹到判定阈值（源力度单位） | `null` |
| `grasp_position_window` | 夹持位置窗口 `[lo, hi]`（mm） | `null` |

Modbus 路径的 `0x000E` 本身即 0–1000 标度，可直接得 `force_norm`；
官方夹爪快路径的力度单位是 **g**，标度不明时 `force_norm` 保持 `None`，
由适配层折算后回填。

---

## 5. 恢复阶梯与红线

| 判定结果 | 阶梯 | 步骤 1 | 步骤 2（用尽） |
|---|---|---|---|
| `grasp_empty` 夹空 | `max_regrasp`（默认 **2**） | `RECOVERABLE`，可重试：退回到安全开口后重抓 | `NEEDS_HUMAN`：人工确认工件与夹爪状态 |
| `stall` 堵转 | `max_stall_retry`（默认 **1**） | `RECOVERABLE`，可重试：松手后重试闭合 | `NEEDS_HUMAN`：人工检查机械卡滞 |
| `overtemp` 过温 | — | `NEEDS_HUMAN`，**不重试**：停机冷却，温度回落后由人工复位 | — |
| `overcurrent` 过流 | — | `NEEDS_PARAM`：**不重试**，确认外部 24 V 供电并与法兰共地（法兰上限 1.5 A < 额定 1.9 A） | — |
| `offline` 离线 | `max_offline_retry`（默认 **1**） | `RECOVERABLE`，可重试：重连后重试 | `NEEDS_HUMAN`：人工检查线缆与外部供电 |
| `offline`（协议未开） | — | `NEEDS_PARAM`：先开启末端 Modbus-RTU 或 RM Plus 协议，再重试 | — |
| `needs_param` | — | `NEEDS_PARAM`：补齐缺失配置项后重跑 | — |
| `unknown` | — | `NEEDS_HUMAN`：无法归类，需人工确认 | — |

> ### 红线：**禁止写 `0x0008`（清错误码）**
>
> 写 `0x0008` 需要**重启设备**，本项目**禁止自动执行**。因此：
>
> * 恢复阶梯的任何 `next_action` 里**不得**出现"清错误码 / 0x0008 / clear_error"；
> * 适配层对 `0x0008` 做**双闸门写保护**（手册地址 + 线上地址，见 §3.2）；
> * 自检入口：`rm_hw_bridge.gripper_state.recovery_ladder_violations()`
>   （返回违规动作列表，空 = 干净），§7 的离线证据链每次运行都会断言它为空。

---

## 6. 运动互锁（夹爪动作期间禁止运动）

### 6.1 闸门三态

| 状态 | 含义 | 运动 |
|---|---|---|
| `IDLE` | 无夹爪动作在途，且状态已判定 | 放行 |
| `GRIPPER_BUSY` | 夹爪动作进行中（`gripper_action` 上下文内） | **拒绝** |
| `GRIPPER_UNJUDGED` | 夹爪动作已结束，但**状态还没判定** | **拒绝**（不得因"未判定"而放行） |

闸门是**进程内共享单例**（`shared_motion_gate()`）：同进程的夹爪动作客户端与执行器
共用同一对象——夹爪一动，闸门立刻非 `IDLE`。

### 6.2 两个检查点

```
阶段 A（准备）
阶段 B（预就位）──► ★检查点 1：阶段 B 之前
阶段 C（起流）  ──► ★检查点 2：阶段 C 起流之前
```

* 闸门 `check()` 返回 `None`（IDLE）→ 放行；返回 dict → **拒绝下发任何运动**，
  记事件 `gripper_interlock_blocked`（severity=`NEEDS_HUMAN`），
  报告 `status=blocked`，退出码 **3**；
* `check()` **抛异常 → 保守拒绝**（state=UNKNOWN），不假装"没有闸门"；
* 互锁**只落在阶段边界**，绝不进 10 ms tick 线程——夹爪动作是离散、慢、阻塞的，
  塞进栅格会直接破坏 p99 < 2 ms 红线；
* 被拒时传输层的 `preposition` / `transmit` **调用数为 0**（零运动下发，可审计）。

---

## 7. 离线证据索引（本轮实测数字）

### 7.1 一条命令

```bash
python3 scripts/gripper_offline_evidence.py --out build/reports/s05_offline_gripper.json
# 退出码：0 = 三段全过；4 = 有断言失败
```

脚本自插 `sys.path`（`rm_hw_bridge` / `rm_executor` / `rm_safety`），零新依赖，
自建输出目录，输出**单个 JSON 报告**。

### 7.2 报告结构

顶层 `status / reason / next_action / evidence / mode` + 三段明细
`cycle` / `judgment` / `interlock`（每段各自带 `evidence`，
**每一段的 evidence 都含 `mode=fake（离线假夹爪，代理模型，不代表真机结论）`**），
外加 `live_window` 段（真机回填入口）。

### 7.3 本轮实测数字（`build/reports/s05_offline_gripper.json`，mode=fake）

| 段 | 断言 | 实测 |
|---|---|---|
| `cycle` | 开合 100 次，超时 0 次、失败 0 次 | **100 次循环 / 200 次动作 / 0 次超时 / 0 次失败**；协议生命周期 open→close 均 ok；超时检测自检通过（证明超时计数器不是恒零的空断言） |
| `judgment` | 六类判定各 10 次，结果与 severity 全对 | 夹到 10/10（`OK`）、夹空 10/10（`RECOVERABLE`）、堵转 10/10（`RECOVERABLE`）、过温 10/10（`NEEDS_HUMAN`）、过流 10/10（`NEEDS_PARAM`）、离线 10/10（`RECOVERABLE`）；**共 60 次注入 / 0 处不符** |
| `judgment`（负例） | 标定系数为 `null` → `needs_param`，不得产出 `grasp_ok` / `grasp_empty` | `result=needs_param`、`severity=NEEDS_PARAM`、缺失键 4 个全列出 —— **通过** |
| `judgment`（红线自检） | 恢复阶梯无禁止动作（不写 `0x0008`） | 违规 **0** 条 —— **通过** |
| `interlock` | 闸门非 IDLE → `gripper_interlock_blocked`，且 `preposition=0` / `transmit=0` | `GRIPPER_BUSY`：`blocked_events=1`、`status=blocked`、`preposition=0`、`transmit=0`；`GRIPPER_UNJUDGED`：同上 —— **两个场景全过** |

阈值来源：本轮工位文件的四个标定字段**全为 `null`**（真机未回填），
因此离线证据链使用**代理阈值**（`force_threshold=100.0`、
`grasp_position_window=[10.0, 30.0] mm`、`position_mm_per_unit=0.05`、
`force_scale=0.01`），报告里 `thresholds_source="proxy_default"` 并**显式标注**。

> ### 这些数字**不是**真机结论
>
> 上表全部来自**离线假夹爪**（`FakeGripper`）与**假传输层**（`FakeTransport`）。
> 它们证明的是"代码路径、判定优先级、恢复阶梯、互锁接线**正确**"，
> **不**证明真实夹爪能开合 100 次不超时、真实六类故障能被正确识别。
> 真机四项验收**本轮未实证**（§8）。

---

## 8. 真机窗口操作手册与回填步骤

### 8.1 步骤 1：探针（一次性解决未实测假设）

```bash
export RM_API2_LIB=/abs/path/to/libapi_c.so
python3 -m rmctl gripper probe --arm left --json
```

探针输出：协议状态 / 波特率档 / 设备 ID / **寄存器地址基数（0-based 还是 1-based）** /
**只读寄存器属 FC03 还是 FC04** / 工具端供电档位 / 控制器代际线索，
并把**未实测假设**逐条列进 `assumptions`。
真机不可达时 `status=blocked` + 可粘贴 `next_action`，退出码 **3**。

### 8.2 步骤 2：回填配置（唯一来源 = `workcell/rm_workcell.yaml::gripper`）

| 回填项 | 怎么得到 |
|---|---|
| `position_mm_per_unit` | 已知开口度的实物标定：`mm / 源单位` |
| `force_scale` | 已知夹持力的实物标定：`N / 源单位`（Modbus 路径源单位 = 0–1000 标度） |
| `force_threshold` | 实测"能夹住工件"的力度下界 |
| `grasp_position_window` | 实测"夹住工件时的开口度区间" `[lo, hi]`（mm） |
| `device_id` / `baud` | 探针结果 |

回填后：

```bash
python3 -m rmctl doctor | grep hw:gripper
# 标定系数已回填 → 从 INFO 转 PASS
```

### 8.3 步骤 3：真机开合 100 次

```bash
python3 -m rmctl gripper cycle --count 100 --json
```

> 注意：`rmctl gripper cycle` **恒为离线代理**（`FakeGripper`，输出 `mode=fake`）。
> 真机开合 100 次要走 `rmctl gripper open` / `close`（逐次下发、逐次读状态），
> 或在真机窗口内用本仓库的适配层直接驱动 `CtypesSdkPort`。
> **绝不能**把代理输出当作真机结论。

### 8.4 步骤 4：真机六类判定各 10 次

```bash
python3 -m rmctl gripper judge --fault each --count 10 --json
```

真机上需要**真实制造**六种工况（夹到 / 夹空 / 堵转 / 过温 / 过流 / 离线），
并把实测读数喂给 `rm_hw_bridge.gripper_state.judge`；离线注入版只能验证判定逻辑。

### 8.5 步骤 5：回填后把"未实证"改写成结论

真机窗口跑通后，必须做三件事，否则文档与报告仍然是"离线结论"：

1. 把真机证据报告的 `mode` 从 `fake` 改为 `live`；
2. 把本文档 §0 与 §7 里的 **"本轮未实证"** 声明改写为真机结论（并写明采集时间、
   样本量与判据）；
3. 把 §9 的开放问题逐条结掉（未结掉的保留，不得默认已解决）。

---

## 9. 已知限制与开放问题

### 9.1 开放问题（真机窗口待结）

| # | 问题 | 影响 | 怎么结 |
|---|---|---|---|
| 1 | **寄存器地址基数**（0-based / 1-based）未实测 | 读错寄存器 → 状态全是 0 或垃圾值 | `rmctl gripper probe`（§8.1）；适配层 `base` 参数 + 证据里写明所用基数 |
| 2 | **只读寄存器属 FC03 还是 FC04** 未实测 | 读不到实际位置/力度/温度 → 判定恒 `needs_param` | 同上（探针两条都试） |
| 3 | **波特率档位与从站 ID** 未在真机确认 | Modbus 通信超时（`-2`） | 探针 + 厂商手册默认值（115200 / 1） |
| 4 | **控制器代际**未知（四代控制器可能返回 `-4`） | 官方夹爪快路径不可用 | 探针的代际线索；`-4` → `blocked`，改走 Modbus-RTU 主路径 |
| 5 | **力标度矛盾**：手册 0–1000 标度 ↔ 额定 130 N ↔ 另有"4 kg"说法 | `force_scale` 换算系数不确定 → 判定阈值不可信 | 真机标定（已知夹持力的实物标定），**不得**靠推算填入 |
| 6 | **法兰 24 V 是否必须打开** | 若法兰供电也能带夹爪，外部供电的必要性要重新评估 | 真机窗口实测两档供电下的动作表现 |

### 9.2 已知限制

| 限制 | 原因 | 现状 / 绕行 |
|---|---|---|
| 离线证据**不等于**真机结论 | 离线跑在 `FakeGripper` / `FakeTransport` 上 | 所有离线输出强制 `mode=fake`；真机结论一律 blocked 并给回填入口 |
| 标定系数缺失 → 夹到/夹空不可判 | 换算系数需实测 | 判定降级为 `needs_param`（**不产出** `grasp_ok` / `grasp_empty`） |
| 官方夹爪快路径的力度单位是 **g** | SDK 结构体与手册标度不同 | 标度不明时 `force_norm=None`，由适配层折算后回填 |
| RM Plus 路径**不能读状态** | 白名单只给了协议开关读写 | `read_state` 直接 `blocked`；RM Plus 仅用于区分 disable / offline |
| 恢复阶梯**不能自动清错误码** | 写 `0x0008` 需重启设备 | 红线 + 双闸门写保护 + `recovery_ladder_violations()` 自检 |
| 夹爪动作**不能**与运动并发 | 夹爪动作是离散、慢、阻塞的 | 运动闸门三态 + 两个阶段边界检查点（§6） |
| 协议配置**会被控制器持久化** | `rm_set_modbus_mode` 写的是控制器配置 | 用完必须 `rm_close_modbus_mode`（适配层 `with_protocol` 保证） |
| `rmctl run` 的 **legacy 回放路径**依赖物化后的绝对路径模型 | Tesseract 资源定位器不认源码里的相对 mesh 路径；`build/model` 是 gitignored 的机器本地物化产物 | 新 worktree 先跑一次 `python3 -m rmctl init`；**未物化时不再抛裸异常**，而是诚实降级为退出码 3 + 可粘贴下一步（`rmctl init` / `source scripts/rmenv.sh`），绝不伪造回放统计 |
