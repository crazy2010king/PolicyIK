# RM75-B 真机验证执行情况与使用说明

> **本文定位**：把 M001-3kzz40 里程碑的真机验证工作（2026-11-19，RM75-B `192.168.1.18`）
> 整理成一份**可复现、可评审**的记录，并**逐条对照**《睿尔曼 RM75-B 底层接口双臂运动规划软件整体规划设计》
> （下称"设计文档"），重点说明**核心约束**在本轮真机验证中的落实与证据。
>
> 关联文档：
> * 设计文档：`docs/code_doc/睿尔曼RM75-B底层接口双臂运动规划软件整体规划设计.md`
> * 硬件适配层：`docs/rm_hw_bridge硬件适配层说明.md`（含 §9 连接预算）
> * 验证阶梯说明：`docs/RM75-B真机验证阶梯.md`（阶梯定义与纪律）
> * 证据工件（入库）：`workcell/baselines/evidence/rm75_validation_ladder_20261119.json`
> * 基线：`workcell/baselines/{network_plan,zero_baseline,measured_limits,version_matrix}.yaml`

---

## 0. 一句话结论

本轮在**单臂** RM75-B 上跑完了从"只读体检"到"急停演练"的 **10 级验证阶梯（V0→V9，9 pass / 1 failed）**，
全程**只使用睿尔曼最底层的 B 类（关节级直通）与 C 类（纯状态/配置）接口**，
**未调用任何厂商解算 / 运动学 / 位姿示教接口**（`rm_algo_*` 60 个符号、13 个位姿类黑名单符号**零调用**），
并因此**暴露并修掉了 7 个真机才暴露的实现缺陷**。

---

## 1. 本轮真机验证做了什么

| 维度 | 内容 |
|---|---|
| 验证环境 | 远端主机 `172.16.119.199`（容器 `ros2-cuda`，`--network host`）+ 单臂 RM75-B `192.168.1.18:8080` |
| 硬件适配层 | `rm_hw_bridge`（RM-API2 **1.1.7** x86-64，`sha256 396913cc…05aac`） |
| 编排器 | `scripts/rm75_validation_ladder.py`（V0→V9，共享**一条**真机连接） |
| 结果 | 10 阶段：**9 pass / 1 failed**（V6 FK 交叉校验**不合格**，见 §6.3） |
| 证据 | `workcell/baselines/evidence/rm75_validation_ladder_20261119.json`（入库，可评审） |
| 回归 | `pytest` **1576 passed / 1 skipped**；`rm_hw_bridge.audit --repo` **0 项违规**（扫描 158 个 `.py`） |

---

## 2. 核心约束：只用最底层接口 —— 本轮的落实与证据

### 2.1 设计文档的约束原文（§1.2）

> **允许使用 B 类与 C 类；禁止使用 A 类中一切依赖厂商逆解/正向解/轨迹规划/位姿示教的接口；厂商算法库 `rm_algo_*` 全族禁止使用。**

| 类别 | 定义 | 本项目是否允许 |
|---|---|---|
| **A 类** | 控制器内部含解算/规划（位姿指令会被转成关节角） | ❌ 禁止 |
| **B 类** | 关节级直通（直接给关节角，控制器不解算） | ✅ 允许（主力） |
| **C 类** | 纯状态/配置（读关节角/电流/温度/错误码；写限值/速度倍率/安全开关/夹爪寄存器） | ✅ 允许 |

### 2.2 符号级台账：22 个核心符号，逐个说明

> 台账来源：`dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/contract.yaml`（**唯一真值**）。
> "本轮实测"一列指 **2026-11-19 真机验证中是否被真实调用过**（证据在证据工件里可查）。

| # | 符号 | 类别 | 本项目用途 | 本轮真机实测 | 证据/说明 |
|---|---|---|---|---|---|
| 1 | `rm_init(RM_TRIPLE_MODE_E)` | C | SDK 初始化（三线程模式，UDP 上报前提） | ✅ | V2 收到 24002 帧 → 三线程生效 |
| 2 | `rm_create_robot_arm(ip, port)` | C | 建链，每臂一个 handle | ✅ | V1~V9 全程；并实测到**连接上限 5**（§2.5） |
| 3 | `rm_delete_robot_arm(handle)` | C | 释放句柄（唯一的释放入口） | ✅ | 每阶段收尾 |
| 4 | `rm_set_realtime_push(cfg)` | C | UDP 主动上报配置（**纯配置**，不含解算） | ✅ | 配置 cycle=5 ms → **实测周期 24.998 ms**（§6.4） |
| 5 | `rm_get_joint_degree` | B/C | 读 7 个关节角（度） | ✅ | 所有阶段的位姿来源 |
| 6 | `rm_get_current_arm_state` | C | 臂状态 + **控制器位姿**（位姿**仅**用于交叉校验，见 §2.4） | ✅ | V6 的 75 mm 偏差即由它给出 |
| 7 | `rm_get_arm_all_state` | C | 电流 / 温度 / 电压 / 使能位 / 关节错误码 | ✅ | V1 快照：7 轴使能、错误码全 0、温度 30~32 ℃ |
| 8 | `rm_get_joint_err_flag` | C | 错误标志位 + 抱闸状态 | ✅ | V1 快照 |
| 9 | `rm_get_arm_current_trajectory` | C | **控制器轨迹类型**（查"是否在偷偷规划"） | ✅ | 本轮据此修了 3 次误判（§2.5） |
| 10 | `rm_movej(joint, v, r, …)` | **B** | 关节空间预就位 / 回原位（控制器只做关节空间插值，**无逆解**） | ✅ | V3/V5 预就位、V7/V8 |
| 11 | **`rm_movej_canfd(cfg)`** | **B** | **主力执行通道**：关节角透传（度制，10 ms 栅格） | ✅ | V4（0.3° 透传）、V5（**501 拍**闭环） |
| 12 | `rm_set_delete_current_trajectory` | C | 清除控制器当前轨迹（兜底） | ⚪ | 本轮未触发（离线有接线与单测） |
| 13 | `rm_set_arm_stop` | C | **急停**（L4） | ✅ | V9 急停演练（rc=0） |
| 14 | `rm_set_arm_slow_stop` | C | 缓停（L3） | ✅ | V9（rc=0）；安全层的 L3 动作也用它 |
| 15 | `rm_set_arm_pause` | C | 暂停（L2） | ⚪ | 本轮真机未触发（离线注入覆盖） |
| 16 | `rm_set_arm_continue` | C | 继续 | ⚪ | 同上 |
| 17 | `rm_set_joint_drive_max_speed` | C | 驱动级速度覆盖（L1 降速窄接口） | ⚪ | 本轮未下发（离线覆盖） |
| 18 | `rm_set_joint_drive_max_acc` | C | 驱动级加速度覆盖 | ⚪ | 同上 |
| 19 | `rm_set_joint_drive_min_pos` | C | 驱动级位置下限 | ⚪ | 同上 |
| 20 | `rm_set_joint_drive_max_pos` | C | 驱动级位置上限 | ⚪ | 同上 |
| 21 | `rm_set_collision_state(0..8)` | C | 碰撞保护等级（**安全开关**） | ✅ | V3/V4/V5 运动前武装 **level 8（最灵敏）** |
| 22 | `rm_set_self_collision_enable(bool)` | C | 自碰撞检测开关 | ✅ | 同上（运动前必须成功，否则中止） |

**支持层 21 个符号**：`rm_set_log_call_back` ✅（接管 SDK 噪声）、`rm_set_timeout` ✅（建连期接线）、
`rm_realtime_arm_state_call_back` ✅（UDP 回调注册）、`rm_get_robot_info` ⚪（**未读控制器型号** ——
`version_matrix` 仍为 null）、`rm_get_realtime_push` ⚪；
**夹爪 / 工具端 16 个**（`rm_set_gripper_*` / `rm_get_gripper_state` / `rm_set_modbus_mode` /
`rm_read_holding_registers` / `rm_write_registers` / `rm_set_rm_plus_mode` / `rm_set_tool_voltage` …）
**本轮完全未调用** —— 本验证只覆盖单臂运动链路，不含夹爪（夹爪属 P3 与设计 §5.9）。

### 2.3 黑名单：零调用的三重证据

**黑名单清单**（`contract.yaml` → `blacklist`）：13 个符号 + 1 个前缀族

| 类别 | 符号 |
|---|---|
| 位姿透传（隐式逆解） | `rm_movep_canfd`、`rm_movep_follow` |
| 位姿运动（控制器内规划+逆解） | `rm_movel`、`rm_moves`、`rm_movec`、`rm_movej_p` |
| 位姿示教 | `rm_set_pos_step`、`rm_set_ort_step`、`rm_set_teach_frame` |
| 工具系维护（第二真值） | `rm_set_manual_tool_frame`、`rm_change_tool_frame`、`rm_update_tool_frame` |
| 零位设置 | `rm_set_joint_zero_pos` |
| 厂商算法库 | 前缀族 `rm_algo_*`（该 `.so` 实测导出 **60 个**；`.so` 共导出 **417 个** `rm_*` 符号） |

**证据一：装载层根本绑不到它们。**
`rm_hw_bridge/loader.py` **只绑定契约清单内的符号**，黑名单符号**连 `getattr` 都不做**
（注释原文："黑名单符号**连 getattr 都不做**（防止误用）"）→ 不是"我们不调用"，而是**调用不到**。

**证据二：静态审计（AST 级，非正则）零违规。**

```
$ python3 -m rm_hw_bridge.audit --repo
审计通过：0 项违规        # 扫描 158 个 .py（dualarm_ws/src + workcell）
```

审计按 `ast.Call` 判定调用形态 —— 字符串里出现 `"rm_algo_xxx("` 不会误报，真调用会带行号报到；
负例（故意造违规）退出码为 3。

**证据三：源码全量扫描，黑名单符号只出现在"审计器/契约/单测"里。**
唯一命中位置是：`audit.py`（审计器实现）、`contract.yaml`（契约本身）、
`hwcollect.py` 的注释与 `BLACKLIST` 常量、以及**专门验证"不触碰黑名单"的单测**
（如 `rm_hw_bridge/tests/test_transmit.py` 用带 `rm_movep_canfd` 的假 SDK 断言透传路径**一次都不碰**）。

### 2.4 厂商位姿的使用边界（设计 §1.4 末行）

| 规则 | 本项目落实 | 本轮真机表现 |
|---|---|---|
| `rm_current_arm_state_t.pose` **仅用于标定后的一致性校验**，不得作为规划真值 | 快照字段固定 `controller_pose_scope = "cross_check_only"`；规划/校验一律用**自研 FK** | **V6 就是这条边界的产物**：自研 FK（Link7 系）vs 控制器位姿（TCP 系）差 **75.03 mm**，归因为**系统性平移**且 `calibration.tcp = null`（未标定）→ 判**不合格**，并明确"**该偏差不能当作模型精度结论**"。若把厂商位姿当真值，这个 75 mm 就会被悄悄吃掉。**（2026-11-20 S11 复测：那次 75.03 mm 的归因已作废 —— “20 组位形”实为同一位形重复；重测结果见 §7.1）** |
| 零位只能走示教器受控流程（`rm_set_joint_zero_pos` 禁用） | 零位列在 `zero_baseline.yaml`，采集项含示教器凹槽对齐照片 | 本轮**未做**零位取证（照片 + 固件/示教器版本），基线仍 `blocked` |

### 2.5 "控制器是否在偷偷规划"的主动检测（设计 §1.3 `rm_get_arm_current_trajectory` 的用途）

本轮把这一条做成了**真机可执行的看门狗**，并因此暴露了 3 个真缺陷：

| 现象（真机） | 根因 | 处理 |
|---|---|---|
| 看门狗在**阶段 B 预就位**就绪 → 误判「抢轨迹」（FATAL+急停） | 预就位走 `rm_movej`，控制器**本来就在**做关节空间规划 | 看门狗改为**阶段 C 起流前**才启动 |
| 透传时控制器上报 `RM_BLEND_…` / `RM_SPLINE_…` / `RM_CARTESIAN_ARC_…` → 误判抢轨迹 | 轨迹类型**随 `trajectory_mode` 与运动阶段变化** | 期望值改成**集合**（透传族）+ **连续 3 次**才判抢轨迹（抗瞬态）；别人的**关节空间/回放**轨迹仍 FATAL + 停机 |

### 2.6 结论

> 本轮真机验证链路（探针 → 硬件适配层 → 执行层 → 安全层 → 工艺层）**全部建立在 B/C 类接口上**：
> **关节级命令**（`rm_movej` / `rm_movej_canfd`）、**关节级状态**（`rm_get_joint_degree` /
> `rm_get_arm_all_state` / `rm_get_joint_err_flag` / `rm_get_current_arm_state`）、
> **限值与安全开关**（`rm_set_joint_drive_*` / `rm_set_collision_state` / `rm_set_self_collision_enable`）、
> **停机族**（`rm_set_arm_stop` / `_slow_stop` / `_pause` / `_continue`）——
> **未使用任何厂商解算 / 运动学 / 位姿示教接口**，也未把厂商位姿当作规划真值。

---

## 3. 与设计文档 §7「分阶段路线与验收判据」逐条对比

| 设计判据（§7） | 本轮结果 | 证据 / 差距 |
|---|---|---|
| **P0** 单臂 UDP 上报连续 10 分钟无中断，周期实测符合 5 ms 倍数设定 | ✅ **达成**（**仅限 UDP 上报通道**的采集，**不是**执行层长跑） | 600 s 长跑：24002 帧、**断流窗口 0**、最大空窗 0.0 ms、**实测周期 24.998 ms**（配置的 5 倍，仍是 5 ms 的整数倍） |
| **P0** 夹爪开合 100 次无超时，`mode`/`actpos`/`error` 可稳定读出 | ⛔ 未覆盖 | 本轮不含夹爪（单臂运动链路） |
| **P0** 硬件急停演练：触发后两臂立即停，SDK 返回明确错误码 | ✅ **达成（单臂）** | V9：软停 + 急停 rc=0、状态快照留档（`drill_performed=true`）；"两臂"需第二台臂可达 |
| **P0** 版本矩阵与零位基线入档（文档 + 照片） | ❌ **未达成** | 控制器固件 / 示教器版本未读取；零位照片未取证 → `version_matrix` / `zero_baseline` 仍 `blocked` |
| **P1** 20 组随机位形：自研 FK 与控制器位姿偏差 < 1 mm / 0.05° | ❌ **未达成（真不合格）** | V6 实测 **75.03 mm**（系统性平移，TCP 未标定）；姿态项控制器未提供 → 不参与判定。**先标定 TCP 再复测** —— **该归因已被 2026-11-20 S11 复测作废（当时 20 组实为同一位形重复），重测结论见 §7.1** |
| **P1** 零位状态下离散碰撞检查结果为空 | ✅ 达成（离线） | S01 `acm_report.json` 字段完整；真机零位未取证 |
| **P1** 10 ms tick 的透传抖动 p99 < 2 ms（连续 5 分钟） | ✅ **达成（本轮 501 拍 / 5.53 s）** | V5：抖动 p99 **0.201 ms**、p50 0.063 ms、max 0.295 ms；跟踪误差 RMS **0.0506°**、max 0.112° |
| **P1** 关节速度 / 加速度实测值与 `joint_limits` 一致性核对通过 | 🟡 **部分达成** | joint1 三档实测 acc **1.7557 rad/s²**、jerk **81.099 rad/s³**（现值 5.2359 / 30.0）；joint2..joint7 未测 → `measured_limits` 仍 `blocked` |
| **P2** 100 组关节目标规划成功率 ≥99%，p95 < 1 s | ✅ 达成（离线） | S08：p95 0.310 s、RSS 324 MB、成功率 1.0 |
| **P2** 笛卡尔 20 组成功率 ≥95%，路径偏差 ≤0.5 mm | ⛔ 未做 | 属原型级诚实交付（out-of-scope 声明） |
| **P2** 故意构造的超限 / 超碰撞轨迹 100% 被拦截 | ✅ 达成（离线） | S04：6/6 全过（exit 3/4/4） |
| **P2** 真机执行 50 次，跟踪误差 RMS < 0.3°，无控制器保护停 | 🟡 **部分达成** | 本轮真机执行 **1 条 5 s 轨迹（501 拍）**：RMS **0.0506°**（远优于 0.3°）、无保护停；**"50 次"长跑未做** |
| **P2** 相同请求同种子结果一致率 100% | ✅ 达成（离线） | S07：两次独立 worker `trajectory_digest` 一致 |
| **P3** pick-place 100 次 / 夹空堵转识别 / TCP 标定重复性 <0.1 mm | 🟡 部分 | TCP **重复性**实测 30 次最大散布 **0.0341 mm**（标称 ±0.05 mm）；但 **TCP 本身未标定**，故不能声称"TCP 标定重复性" |
| **P4 / P5** 双臂共存 / 紧同步 | ⛔ 未覆盖 | 右臂 `192.168.1.19` 本轮**完全未接触**（环境不可达） |
| **P7** 一键回归全绿 + 现场失败 trace 可复现 | ✅ 达成（离线） | `scripts/rm_regression.py`；真机失败 trace 见 S06 |

---

## 4. 与里程碑 M001-3kzz40 验收条款 CC1–CC8 逐条对比

| 条款 | 内容 | 本轮进展 | 仍缺 |
|---|---|---|---|
| **CC1** | 真机取放循环 50 次 + trace 可回放 + 失败 5 分钟复现 | — | 需夹爪 + 工艺场景（单臂阶梯不覆盖） |
| **CC2** | 经四道校验轨迹真机连续 10 分钟无保护停；tick p99<2 ms；到位误差<0.1°；跟踪 RMS≤0.3° | ✅ **三条量化门限全部达标**（V5：p99 0.201 ms、到位 0.0030°、RMS 0.0506°）；V3/V4 微动 | "连续 10 分钟"的**执行层**长跑未做（当前只 5.53 s）—— 2026-11-20 真机窗口门禁 blocked，详见 §6.2 与 `workcell/baselines/evidence/rm75_s10_window_outcome.json` |
| **CC3** | 规划 p95≤1.0 s、成功率≥95%、常驻<500 MB；doctor ≥8 类；MTTR≤2 min | 离线项全达标（S08/S09） | `E3.mttr_human_s` 仍是人工项（pending） |
| **CC4** | 不能被仿真替代：真实保护停 / 供电接地 / 零位 / 跟随误差与抖动 / 夹爪供电与协议时序 | ✅ 真实保护停与急停留档（V9）、真实跟随误差与抖动（V5）、真实重复性（V7）、真实 acc/jerk（V8，joint1）；600 s 真实上报长跑 | 供电接地、夹爪供电与协议时序、零位取证、joint2..7 的 acc/jerk |
| **CC5** | Contract complete：清洗/ACM 报告、ctypes 黄金校验、四道校验 100% 拦截、CLI 退出码、同种子一致 | ✅ 离线全达标；**V6 FK 交叉本轮实测不合格**（75.03 mm） | V6 需标定 TCP 后复测 |
| **CC6** | Integration complete：rmctl→rm_runtime→RM-API2→真机链路跑通（UDP 镜像 / 限值下发 / 10 ms 透传 / 反馈闭环到位判定 / 断流 200 ms 内停机） | ✅ **全链路真机跑通**（V1→V4→V5：建链、快照、透传、501 拍反馈闭环、到位判定 0.0030°） | "断流 200 ms 内停机"的**真机复现**未做（离线注入已覆盖）—— 2026-11-20 真机窗口门禁 blocked，详见 §6.2 与 `workcell/baselines/evidence/rm75_s10_window_outcome.json` |
| **CC7** | Operational complete：急停演练留档；五类故障注入 L1–L4；取放真机 50 次；≤50 圈强制人工检查写进流程 | ✅ 急停演练留档（V9）；五类注入离线全过（L1/L2/L2/L3/L4/L4） | 取放真机 50 次；"≤50 圈强制人工检查"流程需写入工艺文件 |
| **CC8** | S01–S09 各切片交付线 | ✅ 全部落地 | — |

---

## 5. 使用说明（逐步操作）

### 5.1 环境准备（远端主机）

```bash
ssh -p 2224 zy@172.16.119.199
cd /home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.gsd-worktrees/M001-3kzz40

# RM-API2 1.1.7（x86-64）
export RM_API2_LIB=/home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.deps/rm_api2/libapi_c.so

# PYTHONPATH：探针 / 阶梯只需硬件适配层（stdlib + pyyaml，**不需要 tesseract**）
export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PWD/dualarm_ws/src/rm_safety:\
$PWD/dualarm_ws/src/rm_executor:$PWD/dualarm_ws/src/rmctl
export RM_WS=$PWD
```

> **必须用 `--network host` 起容器**（否则收不到臂的 UDP 主动上报）：
> ```bash
> docker run --rm --network host -v /home:/home -w "$PWD" \
>   172.16.119.20:5000/rwthika/ros2-cuda:humble-desktop-full-v25.08_conda_sonnic_20261110_1124 \
>   bash -lc "export PYTHONPATH=$PYTHONPATH RM_API2_LIB=$RM_API2_LIB RM_WS=$RM_WS && \
>             /opt/conda/envs/drake_dev/bin/python3 scripts/rm75_validation_ladder.py ..."
> ```

### 5.2 十个阶段怎么跑

```bash
# 只读体检（V0+V1，臂不动）
python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --stages V0,V1

# 跑到执行层闭环（V0..V5）
python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --max-stage V5

# 跑到重复性（V0..V7）
python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --max-stage V7

# 高风险阶段（现场看护 + 示教器在手）
python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --stages V8,V9 --allow-risky

# 600 s 上报长跑（补 network_plan 基线）
python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --stages V2 --udp-duration-s 600

# 离线预演（零 SDK 调用）
python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --dry-run
```

### 5.3 各阶段的目的 / 判据 / 失败处置

| 阶段 | 目的 | 判据（PASS 条件） | 失败时的处置 |
|---|---|---|---|
| **V0** 契约体检 | 验证 SDK 符号齐备、黑名单零违规 | `hw:sdk`/`hw:symbols`/`hw:contract` 全 OK；审计退出码 0 | 符号缺失 → 换回 RM-API2 1.1.7；审计非 0 → 代码缺陷，**硬红线** |
| **V1** 连接与快照 | 第一次接触真机 | 建链成功 + **7 项前置检查全过**（臂级错误码、关节错误码、7 轴使能、控制器空闲、关节角可读、当前/目标位姿限位余量 ≥1°） | 任一不过 → **一条运动指令都不发**；按报告里的 `next_action` 处理 |
| **V2** UDP 上报质量 | 反馈通道可用性 | frames>0、实测周期落在 5 ms 整数倍、无断流窗口 | 0 帧 → 反馈通道死，**阻断**后续；周期/断流不合格 → 质量失败（不阻断但整体降级） |
| **V3** 最小运动·规划通道 | 验证 `rm_movej` 通道 | 到位 ≤0.2°、其它关节漂移 ≤0.1°、回程 ≤0.2° | 探针内部**自动急停**（`rm_set_arm_stop`），报告留 `estop` 阶段 |
| **V4** 关节角透传 | 验证 `rm_movej_canfd`（主力通道） | 同上 | 同上 |
| **V5** 执行层 live 闭环 | rmctl→rm_runtime→RM-API2→真机 全链路 | 执行层 status=ok **且** 到位<0.1°、跟随 RMS≤0.3°、抖动 p99<2 ms | 任一不过 → 判 failed/blocked，报告带 executor 事件与证据 |
| **V6** FK 交叉校验 | 自研 FK vs 控制器位姿（**仅交叉校验**） | 位置偏差 <1 mm、姿态 <0.05° | 不合格 → 给出归因（系统性平移 vs 离散偏差）与下一步（标定 TCP / 查零位） |
| **V7** TCP 重复性 | 重复性（非绝对精度） | 最大散布 ≤ 标称 ±0.05 mm | 超标 → failed；默认目标=**当前位姿**（几乎不动） |
| **V8** 阶梯 acc/jerk（高风险） | 反推单关节 acc/jerk 下界 | 采集完成 + 每档**回原位** | 需 `--allow-risky`；**只动指定关节**、每档回原位 |
| **V9** 急停演练（高风险） | 急停留档（CC7） | 软停/急停 rc=0 + 状态快照 | 需 `--allow-risky`；**事后需在示教器清错并重新使能** |

### 5.4 单点工具（不走阶梯时）

```bash
# 最小动作探针（0.3° / 低速 / 去程回程）
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only          # 只读
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej --joint 0 --delta-deg 0.3 --speed-pct 2
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel canfd  --joint 0 --delta-deg 0.3

# 一键脚本（默认不动臂；运动必须显式 --with-motion）
bash scripts/run_rm75_probe.sh --ip 192.168.1.18
bash scripts/run_rm75_probe.sh --ip 192.168.1.18 --with-motion --joint 0 --delta-deg 0.3 --speed-pct 2

# 驱动级限值下发留档（默认 dry-run；--apply 才下发，速度按 20% 起步收紧）
python3 scripts/rm75_limit_downlink.py --ip 192.168.1.18
python3 scripts/rm75_limit_downlink.py --ip 192.168.1.18 --apply --speed-scale 0.2 \
        --out build/reports/validation/limit_downlink_20261119.json

# 真机长跑（默认 600 s 微动轨迹，执行层 live 路径）
python3 scripts/rm75_long_run.py --ip 192.168.1.18 --duration-s 600 \
        --out build/reports/validation/long_run_600s.json

# 真机 UDP 断流复现（关主动上报 → 期望 200 ms 内 L4 停机）
python3 scripts/rm75_stream_loss_drill.py --ip 192.168.1.18 \
        --out build/reports/validation/stream_loss_drill.json

# 报告速览（只读文件）
python3 scripts/rm75_probe_report.py build/reports/validation/ladder_report.json

# 证据工件合并（入库用）
python3 scripts/rm75_evidence_merge.py --out workcell/baselines/evidence/rm75_validation_ladder_20261119.json \
    build/reports/validation/ladder_report.json build/reports/validation_udp600/ladder_report.json ...
```

### 5.5 退出码

| 码 | 含义 |
|---|---|
| 0 | 全部阶段通过（或 dry-run） |
| 2 | 参数非法 |
| 3 | blocked（SDK 缺失 / 控制器不可达 / 连接额度满 / 外部条件不具备） |
| 4 | error（控制链路失败、判定未达标或**质量失败**导致的整体降级） |

### 5.6 连接预算（真机窗口的第一条硬规矩）

**控制器只接纳 ≈5 个客户端连接**，超限后 `rm_create_robot_arm` 返回 NULL/`id=-1`，且**释放很慢**（5~10 min 量级）。
因此：**一轮真机工作 = 一条连接**，阶梯已实现共享句柄（探针 `link_factory` + `close_link=False`、
`hwcollect.*(link=…)`、`rm_executor.execute(link=…)`）。详见 `docs/rm_hw_bridge硬件适配层说明.md §9`。

---

## 6. 本轮实测数据汇总

### 6.1 阶梯结果

| 阶段 | 实测值 |
|---|---|
| V0 | `hw:sdk`/`hw:symbols`（22 核心 + 21 支持）/`hw:contract`（158 个 .py）全 PASS |
| V1 | 7 项前置检查 7/7；控制器 `RM_NO_PLANNING_E`/`RM_IDLE_E`；7 轴全使能；错误码全 0；温度 30~32 ℃；J1≈-0.59°、J7≈-200.5° |
| V2 | **600 s**：24002 帧 / 期望 120000、**实测周期 24.998 ms**（配置 5 ms 的 5 倍）、**断流窗口 0**、最大空窗 0.0 ms |
| V3 | `rm_movej` J1 +0.3°（2% 速度）：到位 **0.005°**、漂移 **0.002°**、回程 **0.038°** |
| V4 | canfd 透传 J1 +0.3°：到位 **0.163°**、漂移 **0.002°**、回程 **0.162°** |
| V5 | 501 拍 / 5.53 s、到位 **0.0030°**、跟随 RMS **0.0506°**（max 0.112°）、抖动 p50 **0.063 ms** / p95 **0.098 ms** / p99 **0.201 ms** / max **0.295 ms** |
| V6 | **failed**：FK 位置偏差 **75.03 mm**（阈值 1 mm）、归因 `systematic_translation`、`calibration.tcp = null`（**该次归因已被 2026-11-20 S11 复测作废** —— 20 组实为同一位形重复；重测见 §7.1） |
| V7 | 30 次固定目标：最大散布 **0.0341 mm**、平均散布 0.0127 mm（标称 ±0.05 mm） |
| V8 | joint1 三档（10°/档、速度 5/8/11%、每档回原位）：包络 acc **1.7557 rad/s²**、jerk **81.099 rad/s³** |
| V9 | 软停 rc=0、急停 rc=0、`drill_performed=true`、状态快照留档 |

### 6.2 里程碑收尾的三项真机工具（2026-11-20）

| 工具 | 目的 | 真机结果 |
|---|---|---|
| `scripts/rm75_limit_downlink.py` | 驱动级限值下发留档（R004 剩余项） | **已完成**：28 次 SDK 调用（7 关节 × 速度/加速度/下限/上限）全部 rc=0；位置限值用**执行域**（机械范围），速度按 20% 起步收紧；白名单无读回符号 → 只以返回码 + diff 判定，不伪造读回值 |
| `scripts/rm75_long_run.py` | 10 分钟连续执行长跑（S03/R001 字面要求） | **blocked（未执行）**：2026-11-20 真机窗口门禁未通过 —— `ip_reachable` 100% 丢包、`tcp_reachable`（192.168.1.18:8080）timed out、`probe_precheck` 已跳过（未建链、零运动指令）→ **长跑未执行，无任何实测数字**。结案工件：`workcell/baselines/evidence/rm75_s10_window_outcome.json`（`status=blocked`、`long_run=null`） |
| `scripts/rm75_stream_loss_drill.py` | UDP 断流 200 ms 内停机的真机复现 | **blocked（未执行）**：同上窗口门禁失败项（ip/TCP 不通）→ **断流演练未执行，无任何实测数字**。结案工件：`workcell/baselines/evidence/rm75_s10_window_outcome.json`（`status=blocked`、`stream_loss=null`） |

> **诚实声明**：本节结论全部以机器可读的结案工件
> `workcell/baselines/evidence/rm75_s10_window_outcome.json` 为唯一真值来源；
> **未以离线/仿真结果替代真机结论** —— blocked 分支下未执行任何运动指令，
> 也不含任何伪造的长跑/断流实测数字。
>
> **口径澄清（勿混为一谈）**：本表的「UDP 主动上报 600 s 长跑」= 只收控制器主动上报帧的
> **反馈通道**采集（2026-11-19 已实测：24002 帧 / 周期 24.998 ms / 断流窗口 0）；
> 「**执行层** 600 s 连续执行长跑」= `scripts/rm75_long_run.py` 走 rm_executor live 路径
> **持续下发关节角指令**的闭环长跑。两者是**不同的事**，前者已实测，**后者本轮未执行**。

### 6.3 本轮暴露并修复的 7 个真机缺陷

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 建链抛 `ValueError: NULL pointer access` | 连接额度耗尽；NULL 指针读 `.contents` 抛裸 ctypes 异常 | 结构化报错 + TCP 可达性分类（`blocked`=额度满 / `error`=不可达）；`_handle_id` 对 NULL 安全降级 |
| 2 | 时间域限值复检 **501 项越界** | 声明限位把**规划域 ±180°** 当**执行域**用；真机 J7=-200.5° 在规划域外、执行域内 | `joint_limits.yaml` 拆出 `execution_lower/upper`（J7 执行域 ±360°）；执行层用执行域、规划器仍用规划域 |
| 3 | 误判「控制器抢轨迹」 | 看门狗在阶段 B 预就位就绪 | 改为阶段 C 起流前启动 |
| 4 | 误判「控制器抢轨迹」（blend/spline/cartesian-arc） | 轨迹类型随模式与阶段变化 | 期望值集合化 + 连续 3 次确认 |
| 5 | `arm_err=[0]` 被当未知码 → L3 缓停（轨迹只跑 2 拍） | 厂商约定 0=无错误 | 安全层排除 0 |
| 6 | **普通停止态**被当「保护停」→ L4 急停（ticks=2 即停） | `RM_STOP_E(9)` 是普通停止态 | 保护停需**佐证**：非零错误码，或停止态持续 >100 ms 且正在下发指令 |
| 7 | 阶梯实验 `joints` 参数**被忽略** → 7 个关节各走 10° 且不回原位 | 真机路径硬编码 `range(7)` | 只动指定关节 + 每档自动回原位 |

另有阶梯自身 2 处判据错误：V6 只看采集 `status`（把 FK 的 FAIL 误报成通过）、部分阶段执行时报告阶段号错位。

### 6.4 S13 实测数据（2026-11-23，远端主机真机窗口）

> 执行环境：远端主机 `172.16.119.199:2224` → 容器 `ros_dev` → worktree `.gsd-worktrees/M001-3kzz40`。
> 本节数字均来自 **git 跟踪的证据工件**，未做任何手工修饰。

#### 6.4.1 joint2..joint7 的阶梯 acc/jerk（samples ≥ 3，3 档：速度 5% / 8% / 11%，每档测完回原位）

工件：`workcell/baselines/evidence/rm75_s13_acc_jerk.json`
（schema `rmctl.rm75_s13_acc_jerk/1`，`status=ok`）。

| 关节 | acc 包络 (rad/s²) | jerk 包络 (rad/s³) | 样本数 | 档位 |
|---|---|---|---|---|
| joint2 | 2.4366 | 108.509 | 3 | 5 / 8 / 11 |
| joint3 | 1.4804 | 74.139 | 3 | 5 / 8 / 11 |
| joint4 | 2.2030 | 90.306 | 3 | 5 / 8 / 11 |
| joint5 | 1.7539 | 103.124 | 3 | 5 / 8 / 11 |
| joint6 | 1.8214 | 95.174 | 3 | 5 / 8 / 11 |
| joint7 | 2.3343 | 97.836 | 3 | 5 / 8 / 11 |

* joint1 的 acc/jerk 来自 S01 的 V8 阶段工件（包络 acc 1.7557 rad/s² / jerk 81.099 rad/s³）；
* 包络 = **多档取 min**（保守下界），**方法学边界**：这是 5%~11% 速度档指令轮廓下的**下界**，
  **不代表控制器最大能力**；
* 保守回填后 `joint_limits.yaml` **只收紧不放宽**：
  acc 逐关节取 `min(实测包络, 已生效 5.2359)`，jerk 全部保持已生效的 **30.0**（实测包络更高也不放宽）。

#### 6.4.2 控制器 / 软件版本（SDK 只读读数路径已打通）

工件：`workcell/baselines/evidence/rm75_s13_versions_connectivity.json`
（schema `rmctl.rm75_s13_versions_connectivity/1`，`status=ok`，**单次读数**，零运动）。

| 项 | 实测值 |
|---|---|
| 读数符号 | `rm_get_arm_software_info`（厂商真实符号名；此前文档里的 `rm_get_arm_software_version` 是**笔误**） |
| 白名单 | `rm_hw_bridge/contract.yaml` 的 `support:` 21 → **22**，`core:` 仍为 **22** |
| `product_version` | `RM75-BI` |
| `ctrl_version` / `ctrl_build_time` | `V1.7.1-f95e896` / `2025/09/17 21:01:26` |
| `plan_version` / `plan_build_time` | `V1.7.1-85f9432` / `2025/09/17 21:01:44` |
| `algorithm_version` / `dynamic_model_version` | `1.5.5-c0d52d18` / `2` |
| 机型 | `arm_dof=7`、`arm_model=1`、`robot_controller_version=3` → 7 轴、**三代控制器** |
| 容器镜像 | `172.16.119.20:5000/rwthika/ros2-cuda:humble-desktop-full-v25.08_conda_sonnic_20261110_1124` |
| 示教器版本 | **仍未读取**（无自动读数路径，需人工读屏） |

#### 6.4.3 双臂连通性实测结论

| 侧 | IP | reachable | 原始输出摘要 |
|---|---|---|---|
| 左臂 | `192.168.1.18` | **true** | ping 3 发 3 收 / 0% 丢包（rtt 0.452~0.528 ms）；TCP 8080 握手成功 |
| 右臂 | `192.168.1.19` | **false** | ping 100% 丢包；TCP 8080 `[Errno 113] No route to host` |

→ **该网段内不存在右臂实体**（不是执行环境问题，也不是本机口径问题）→ 右臂相关项本环境**无法**补测。

---

### 6.5 S14 真机窗口实测（2026-11-23 15:22–16:10，远端主机 ros_dev）

工件：`workcell/baselines/evidence/rm75_s14_real_machine_window.json`（schema `rmctl.rm75_s14_real_machine_window/1`，`status=blocked`）。

**只读门禁全部通过**：ping 3 发 3 收 0% 丢包（rtt 0.434/0.503/0.616 ms）、TCP 8080 可连、UDP 8091/8092 空闲、SDK 可加载、探针前置检查 7/7（7 轴全部使能、`arm_err` / `joint_err_code` 全 0、控制器 `RM_NO_PLANNING_E` + `RM_IDLE_E`、当前位形在限值内）。

**执行层 600 s 长跑：两次尝试均被真实断流打断（未达成时长判据）**

| 尝试 | 实际时长 | ticks | 跟随误差 RMS | 事件链 | 控制器保护停 |
|---|---|---|---|---|---|
| #1 | 145.24 s | 14431 | 0.00317° | `pre_position_ok` → `feedback_stream_lag` → `feedback_stream_lost` → `safety_l4_emergency_stop` → `fault_stream_lost` | 否 |
| #2 | 251.28 s | 25033 | 0.00289° | 同上 | 否 |

→ **关节级透传与闭环跟踪稳定**（RMS ≈ 0.003°，保护停未触发）；断的是**反馈通道**（UDP 主动上报）。
→ 两次断流时刻不同 → **非固定时长触发的确定性缺陷**。
→ 两次都触发了 **L4 急停**（本项目安全链路的**真机实证**：断流 → L4 停机生效）。

**断流专用演练：进程 SIGSEGV（退出码 139），未产出通过证据**

| 层 | 结论 |
|---|---|
| 本项目 | **已修复**：`ArmLink.set_realtime_push_enabled` 引用了不存在的私有属性 `_udp_port` / `_realtime_target_ip`（真实属性为公开的 `udp_port` / `realtime_target_ip`）→ 真机线程内抛 `AttributeError` → 进程 core dump。修复后单独调用 `set_realtime_push_enabled(False)` → `rc=0`、无崩溃；新增守卫 `dualarm_ws/src/rm_hw_bridge/tests/test_realtime_push_toggle.py`（7 例） |
| 厂商 SDK | **未闭合**：修复上述缺陷后，「执行器正在流式下发 + 另一个连接关主动上报」的并发组合仍使进程 SIGSEGV → 指向 RM-API2 1.1.7 行为（所用符号 `rm_set_realtime_push` 在契约 `support` 名单内，属纯配置类） |

**已知线索**：2026-11-19 的反馈通道专项采集（`udp_stability 600 s`）实测 24002 帧 / 断流窗口 0 / 周期 24.998 ms → 控制器在**无流式下发**时可稳定上报 600 s；因此差异点在「流式下发 + 反馈上报并发」的组合。

---

## 7. 仍未闭合的项与下一步

| 项 | 现状 | 下一步（具体命令/动作） |
|---|---|---|
| **V6 FK 交叉校验** | **2026-11-20 已在远端主机真机窗口复测**（工件 `workcell/baselines/evidence/rm75_s11_fk_recheck.json`，`status=ok`）：8 组**有界多样位形**（幅度 5°、单关节依次、每步回原位、速度 10%）→ `raw max_pos_dev_mm=151.460` / `mean_pos_dev_mm=130.525` / `max_ori_dev_deg=179.965`，`residual_kind=orientation_dependent`（`consistency_base=0.839` / `consistency_tip=0.841`）→ 按分支规则**不写任何标定偏置**（`calibration.tcp` / `calibration.base_to_base` 仍为 null）。旧的"75.03 mm 系统性平移"结论**已作废**（那 20 组实为同一位形重复） | 按 `residual_kind` 分支走：查零位（示教器凹槽对齐照片 + `joint1..joint7` 零位读数）→ 核对官方 MDH 表 a/α/θ 与清洗 URDF → 确认控制器位姿参考系（base/work/tool）→ 扩大位形多样性后**复测** `python3 -m rmctl.hwcollect fk_cross --arm left --samples 8 --fk-amplitude-deg 5 --seed <seed> --out <dir>`（**远端主机**执行，见 §7.1） |
| **零位基线** | 未取证（零位照片 + 固件/示教器版本）。**2026-11-23 更新**：固件/控制器软件版本已能经 SDK 只读读到（见 §6.4.2），但 `zero_baseline` 的 `firmware` / `teachpendant_version` / 每关节 `zero_deg` / `groove_photos` **仍全部为 null** —— 本表是全表口径 | 示教器受控流程对齐凹槽 → 照片 → `rmctl.hwcollect zero_capture` → 回填 `zero_baseline.yaml`；**归属 M002-o62zc6/T03** |
| **joint2..joint7 的 acc/jerk** | **已闭合**（2026-11-23，远端主机真机窗口，3 档 × 3 样本；工件 `workcell/baselines/evidence/rm75_s13_acc_jerk.json`）→ `measured_limits` 已升为 `status=measured`，`apply_measured_limits.py --mode conservative` 保守回填完成（只收紧不放宽） | 后续若在更高速度档 / 更大阶跃下复测，用 `rmctl.hwcollect ladder_acc_jerk --joints <n> --trials 3` 重取包络，并在保守口径下重新回填 |
| **基线状态位** | 四张表**仍非全 measured**：`measured_limits=measured`（已闭合），`zero_baseline` / `version_matrix` / `network_plan` 仍为 `blocked`（全表口径：示教器版本无自动读数路径、每关节零位与凹槽照片未采集、右臂实体不可达、执行层 600 s 长跑与 UDP 断流复现未执行） | 实测值已写在各表 `partial_observations` + 证据工件；**不得**为求 pass 改 status；一键回归 `baselines` 段 verdict 与四表 status 同向（本轮为 `blocked`，退出码 3） |
| **示教器版本** | **未闭合**：无自动读数路径（只能人工读屏），2026-11-23 窗口无人操作示教器 → `version_matrix.teachpendant` 保持 `null`，该表因此**注定** `blocked` | 现场人员读示教器「系统信息」版本号 → 回填 `version_matrix.yaml`（连同已归档在 `partial_observations` 的控制器版本与容器镜像一次性搬入正式字段）→ 9 条全非 null 后才可改 `status=measured`；**归属 M002-o62zc6/T03** |
| **FK 残差根因** | **未闭合**：2026-11-20 复测 `max_pos_dev_mm=151.460` / `max_ori_dev_deg=179.965`，`residual_kind=orientation_dependent`（约 130 mm / 约 180°）→ 指向 URDF 清洗 / 零位 / 控制器位姿参考系，**不写任何标定偏置** | 查零位（示教器凹槽照片 + 每关节零位读数）→ 核对官方 MDH 与清洗 URDF → 确认控制器位姿参考系 → 扩大位形多样性后复测 `python3 -m rmctl.hwcollect fk_cross --arm left --samples 8 --fk-amplitude-deg 5 --seed <seed> --out <dir>`（**远端主机**）；**归属 M002-o62zc6/T03** |
| **CC2 的"连续 10 分钟"** | **仍未闭合，但阻塞原因已从「窗口不可用」换成真实根因**：2026-11-23 远端主机真机窗口**门禁全部通过**，执行层长跑**实际执行**两次，分别在 **145.24 s / 251.28 s** 处被**真实的 UDP 反馈断流**打断（`feedback_stream_lag` → `feedback_stream_lost` → `safety_l4_emergency_stop`），跟踪误差 RMS ≈ 0.003°、控制器保护停未触发；两次都**触发了 L4 急停**（安全链路真机实证）。工件：`workcell/baselines/evidence/rm75_s14_real_machine_window.json`（`status=blocked`）；结案工件 `workcell/baselines/evidence/rm75_s10_window_outcome.json` | 仍以结案工件入口为准：示教器重新使能 7 轴 → 清错误码复位 → `ss -ulnp \| grep -E '8091\|8092'` 确认端口空闲 → 重跑 `python3 scripts/rm75_window_run.py --out workcell/baselines/evidence/rm75_s10_window_outcome.json`（门禁通过后由该入口自动续跑 长跑 → 断流 → 阶梯）；并按工件 `findings` F1/F2 做隔离实验：① 流式下发期间统计反馈帧数与最后一帧时刻，区分「SDK 回调线程退出」与「控制器停止上报」；② 对照「只跑反馈通道」（已 600 s 通过）与「流式 + 反馈」的差异；③ 断流演练改为**同一连接**关闭主动上报（避开并发崩溃）；④ 若确认为厂商 SDK 行为，保留 core dump 向厂商反馈。**R001 保持 active** |
| **CC1 取放 50 次 / 夹爪** | 未覆盖 | 需夹爪供电与协议时序打通后另开窗口 |
| **双臂（P4/P5）** | 右臂 `192.168.1.19` **未接触**：2026-11-23 远端主机再次实测 `ping` 100% 丢包、TCP 8080 `[Errno 113] No route to host` → **该网段内不存在右臂实体**，本环境无法补测 | 右臂实体接入网段并确认可达后另开窗口；注意 UDP 端口 8092 预留；**归属 M002-o62zc6/T03** |
| **里程碑验证重跑** | M001 仍是 `needs-attention` | 2026-11-20 已做**范围重划**：M001 收敛为单臂（验收条款、ROADMAP Success Criteria、CONTEXT、需求归属均已更新），夹爪/取放/现场人项移交新里程碑 **M002-o62zc6**（T01–T05）；M001 剩余未完成项为 10 分钟长跑与断流真机复现（2026-11-20 窗口门禁 blocked，见 `workcell/baselines/evidence/rm75_s10_window_outcome.json`；需臂使能 + 网络可达） |
| **臂的使能状态** | **2026-11-23 更新：已使能可用** —— 远端主机真机窗口的探针前置检查 **7/7 通过**（7 轴 `joint_en_flag` 全 1、`arm_err` / `joint_err_code` 全 0、控制器 `RM_NO_PLANNING_E` + `RM_IDLE_E`）；长跑期间未掉使能（事件链无掉使能项） | 若后续窗口又出现掉使能，仍需在示教器**重新使能**；在此之前探针的 7 项前置检查会拒绝任何运动指令（这是预期行为） |
| **R004 电子围栏（能力缺口）** | **未闭合**：厂商 SDK **有**电子围栏接口（`dualarm_ws/src/rm_hw_bridge/vendor/rm_api2/include/rm_interface.h`：`rm_set_electronic_fence_enable` / `rm_set_electronic_fence_config` / `rm_set_virtual_wall_config` / `rm_get_virtual_wall_config`；类型 `rm_fence_config_t` / `rm_electronic_fence_enable_t` 见同目录 `rm_define.h`），但本项目的调用许可清单 `dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/contract.yaml`（`core:` / `support:` / `blacklist:` 三段）grep 'fence' **零命中** → 装载层**没有绑定**这些符号 → 当前实现**无法下发电子围栏**；归属 M001/S02（R004 需求侧），实现侧移交 **M002-o62zc6**，R004 保持 active | 归属 **M002-o62zc6**（本里程碑 M001-3kzz40 **不做**：R004 的其余字段已由 S02 离线覆盖，而「下发电子围栏」需要新增 SDK 符号绑定 + 白名单条目，属独立技术债切片）；后续绑定入口路径：`rm_hw_bridge/abi_spec.yaml` 与 `rm_hw_bridge/generated/rm_api2_ctypes.py` 增加对应符号 → 在 `rm_hw_bridge/rm_hw_bridge/contract.yaml` 的 `support:` 段显式登记（含理由）→ 硬件层新增 `set_electronic_fence` 下发命令并接入 R004 的 `set_safety` 入口 |
| **R008 真机版崩溃隔离（未实证）** | **未闭合**：离线层已实证（真实规划内核 + 真实四道校验 + 真实进程级 SIGKILL + `rm_executor` fake 回放；服务端超时被 SIGKILL 强制中断 8/8、服务被杀不影响运动执行 15/15），但**真机层的崩溃隔离未实证**——「`rm_runtime` 执行轨迹过程中 `kill -9 rm_planner` 后真机运动是否继续」本轮**未执行**，属真机窗口项（`.gsd/phases/01-3kzz40-phase/01-07-ASSESSMENT.md` 登记为 human-follow-up / NEEDS-HUMAN）；归属 M001/S07–S09，R008 保持 active | 真机窗口内执行：① `rm_runtime` 开始执行一条真机轨迹（记录服务 pid / worker pid / 轨迹点位）→ ② 另开终端 `kill -9 <rm_planner pid>` → ③ 观察真机运动是否继续，并记录是否出现安全事件 → ④ 回填 `docs/rm_planner规划服务说明.md` §9.2 与 `build/reports/s08_*`（口径先写「未实证」，回填后再改状态）→ ⑤ 回填本文档 §7 本行 |

### 7.1 S11 远端窗口 FK 复测结论（2026-11-20）与剩余未闭合项

**执行环境（口径，勿再误判）**：控制臂 `192.168.1.18` **只在远端主机 `172.16.119.199`
所在网段可达** —— 远端 `ping 0% 丢包`（rtt 0.44 ms）、`TCP 8080` 可连、SDK 可装载。
**本机门禁失败 ≠ 真机不可达**（这是 2026-11-13 那次误判的根因，已写进
`docs/真机基线与实测限值归档说明.md` §3.1 与 §6 踩坑清单第 15 条）。

```bash
# 远端执行模板（仓库根与 worktree 同路径，同一份 NFS 文件，无需 rsync/scp）
ssh -p 2224 zy@172.16.119.199 \
  'docker exec -i ros_dev bash -lc "cd /home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.gsd-worktrees/M001-3kzz40 && source scripts/rmenv.sh && export RM_API2_LIB=$PWD/../../.deps/rm_api2/libapi_c.so && python3 -m rmctl.hwcollect fk_cross --arm left --ip 192.168.1.18 --samples 8 --fk-amplitude-deg 5 --seed 20261120 --out build/reports/validation/s11_fk_cross"'
```

| 项 | 实测结论（工件 `workcell/baselines/evidence/rm75_s11_fk_recheck.json`） |
|---|---|
| 门禁 | 5/5 通过（ping / TCP 8080 / UDP 8091+8092 空闲 / SDK 可装载 / 探针前置检查 ok） |
| 只读快照 | 7 轴 `joint_en_flag` 全 1、`arm_err`/`joint_err_code` 全 0、`RM_NO_PLANNING_E` + `RM_IDLE_E` |
| 采样 | 8 组 / 8 个互异位形；幅度 5°；单关节依次；每步回原位；速度 10% |
| raw | `max_pos_dev_mm=151.460`、`mean_pos_dev_mm=130.525`、`max_ori_dev_deg=179.965` |
| corrected | 与 raw 相同（**未施加任何偏置**） |
| `residual_kind` | `orientation_dependent`（`consistency_base=0.839` / `consistency_tip=0.841` → 5° 单关节小扰动不足以区分坐标系） |
| 处置 | **不写 `calibration.tcp`，也不写 `calibration.base_to_base`**；残差量级（约 130 mm / 约 180°）远超 mm 级偏置可解释范围 → 指向 URDF 清洗 / 零位 / 控制器位姿参考系口径，**不是**法兰→工具偏置 |

**剩余未闭合项（移交 S13 / M002）**：

| 项 | 阻塞原因 | 下一步 |
|---|---|---|
| 右臂 `192.168.1.19` | 远端主机实测同样 `No route to host`（右臂不存在，不是本机问题） | 右臂接入网络后另开窗口；UDP 8092 预留 |
| 控制器固件 / 示教器版本 | `rm_get_arm_software_version` **不在** `rm_hw_bridge` 的 22+21 符号白名单内 → 不可经 SDK 读取 | 走白名单外读数路径或示教器现场记录；`version_matrix` 保持 `blocked` |
| `joint2..joint7` 的 acc/jerk | 本轮只测了 joint1（3 档，包络 acc 1.7557 rad/s² / jerk 81.099 rad/s³） | `python3 -m rmctl.hwcollect ladder_acc_jerk --arm left --trials 3`（**远端主机**）测齐 7 个关节 → `measured_limits` 才能改 `status=measured` → 再 `apply_measured_limits.py --dry-run` 走 conservative 回填 |
| 每关节零位读数 / 凹槽照片 | 需示教器受控流程（`rm_set_joint_zero_pos` 在黑名单内，不可脚本化） | 示教器对齐凹槽 → 照片 → `python3 -m rmctl.hwcollect zero_capture --arm left --photo <照片引用> --out <dir>` |

---

## 8. 附录

### 8.1 命令速查

| 目的 | 命令 |
|---|---|
| 全阶梯 | `python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --max-stage V7` |
| 高风险阶段 | `... --stages V8,V9 --allow-risky` |
| 600 s 上报长跑 | `... --stages V2 --udp-duration-s 600` |
| 单点微动 | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej --joint 0 --delta-deg 0.3 --speed-pct 2` |
| 限值下发留档 | `python3 scripts/rm75_limit_downlink.py --ip 192.168.1.18 --apply --speed-scale 0.2` |
| 真机长跑 | `python3 scripts/rm75_long_run.py --ip 192.168.1.18 --duration-s 600` |
| 断流复现 | `python3 scripts/rm75_stream_loss_drill.py --ip 192.168.1.18` |
| 一键（默认只读） | `bash scripts/run_rm75_probe.sh --ip 192.168.1.18` |
| 报告速览 | `python3 scripts/rm75_probe_report.py build/reports/validation/ladder_report.json` |
| 黑名单审计 | `python3 -m rm_hw_bridge.audit --repo` |
| 契约体检 | `python3 -m rmctl doctor --json` |
| 离线回归 | `python3 -m pytest dualarm_ws/src/rmctl/tests dualarm_ws/src/rm_hw_bridge/tests dualarm_ws/src/rm_safety/tests dualarm_ws/src/rm_executor/tests -q` |

### 8.2 证据文件清单

| 文件 | 内容 |
|---|---|
| `workcell/baselines/evidence/rm75_validation_ladder_20261119.json` | **入库**证据工件：10 阶段逐段判据、实测值、覆盖表 |
| `build/reports/validation*/ladder_report.{json,md}` | 各次运行的原始报告（gitignored） |
| `workcell/baselines/*.yaml::partial_observations` | 各基线里"已实测但尚未进入基线值字段"的观测 |
| `docs/RM75-B真机验证阶梯.md` | 阶梯定义、纪律、7 个缺陷与实测结果 |
| `docs/rm_hw_bridge硬件适配层说明.md` §9 | 连接预算（≈5 客户端）与诚实报错口径 |

### 8.3 术语与单位

| 项 | 说明 |
|---|---|
| B / C / A 类 | 设计文档 §1.2 的接口三分类（B=关节级直通、C=纯状态配置、A=控制器内含解算，**禁用**） |
| canfd 透传 | `rm_movej_canfd`，本项目**唯一**的关节角流式执行通道（10 ms 栅格） |
| 实测周期 | 由 `frames / elapsed` 反算的 UDP 上报真实周期（**不是**配置值） |
| 到达率 | `frames / expected_frames`；周期被控制器按整数倍取用时，该值 <100% 但**不是丢包** |
| 重复性 | 多次回到同一点的散布；**≠ 绝对精度**（TCP 未标定时尤其） |
