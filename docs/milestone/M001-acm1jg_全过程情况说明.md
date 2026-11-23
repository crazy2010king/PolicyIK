# 睿尔曼 RM75-B 双臂规划基线 M001-acm1jg：全过程情况说明

> 文档性质：对 GSD 里程碑 **M001-acm1jg《双臂规划与操作软件基线》** 从立项到当前状态的完整过程说明与技术剖析。
> 撰写日期：2026-11-23
> 适用范围：评审者、后续接手的工程师、以及需要复算/复验本里程碑全部结论的人。
> 结论口径：**所有数字要么给出可执行命令与退出码，要么给出代码位置（`文件:行`），要么明确标注为「未验证」**。本文不含任何模拟、推断或占位数据。

---

## 〇、阅读指引：本文回答的 8 个问题

| # | 问题 | 对应章节 |
|---|---|---|
| 1 | 这个里程碑到底要交付什么？验收标准是什么？ | §二 |
| 2 | 9 份设计文档的 6 个架构冲突是怎么被裁决的？ | §四 |
| 3 | 10 个切片分别产出了什么、怎么验的？ | §五 |
| 4 | 代码层面哪些设计是"有据可依"的？关键实现长什么样？ | §六 |
| 5 | 真机上到底测了什么、数是多少？ | §七 |
| 6 | 为什么 `/gsd auto` 现在跑不起来？根因是什么？ | §八 |
| 7 | 证据台账缺什么？哪些能补、哪些补不了？ | §九 |
| 8 | 下一步该怎么走？ | §十 |

---

## 一、环境与代码库事实（先对齐"东西在哪"）

### 1.1 代码位置（重要：不是当前目录）

本仓库工作区采用 GSD **worktree 隔离模式**，真实代码位于：

```
/mnt/test_cc_dev/git_source/groot_nvidia/PolicyIK_drake/.gsd-worktrees/M001-acm1jg/
```

- 当前目录（`.../PolicyIK_drake/`）只保留 `AGENTS.md` + `.gsd/`（规划与状态数据库）+ `docs/`；
- `.gsd-worktrees/M001-acm1jg/` 是**真实工程**：77 个源码文件、76 个测试文件、19 个脚本、84 次提交；
- 该路径是 **NFS 挂载**，与远端工作站看到的 `/home/zy/nfs/...` 是**同一份文件**，本地改动无需 rsync/scp 即在远端生效。

### 1.2 三层执行环境

| 层 | 位置 | 职责 | 实测事实 |
|---|---|---|---|
| 规划层 | 远端容器 `ros_dev` 的 `conda drake_dev`（Python 3.12.13 / Drake 1.55.0） | IK / GCS / 重定时 / 场景装配 | 容器 **host 网络**，可直连机械臂（实测 ping 通） |
| 设备层 | 远端宿主 venv `/tmp/rmvenv`（Python 3.12.3 / `robotic-arm==1.1.7`） | SDK 调用、5 ms 透传、UDP 状态接收 | **零 Drake 依赖**（设备层不 import pydrake） |
| 验证/开发层 | 本开发机（Python 3.10.12 / Drake 1.51.1 降级档） | 纯离线回归、证据复算 | **不可路由到机械臂**（无 SDK、`192.168.1.18` 不通） |

> ⚠️ 这条"三层环境"是理解 §八 阻塞根因的**前提**：验证主机与真机主机**不是同一台**，且真机网段只挂在远端工作站的 USB 网卡 `enx98fc84e3177a`（`192.168.1.198/24`）上。

### 1.3 真机与工位事实（2026-11-23 现场复测）

| 项 | 值 | 本次复核方式 |
|---|---|---|
| 机械臂 | 睿尔曼 RM75-B（7-DoF），`192.168.1.18`，控制器代次 3 | 远端 `ping 192.168.1.18` → 通 |
| 夹爪 | WHJ03 两指平行夹爪（65 mm 开口，位置 0–1000 无量纲） | 真机 L9 阶梯已实测（§七） |
| 指令通道 | TCP `192.168.1.18:8080`（`rm_*` SDK） | 同上 |
| 状态上报 | UDP → **开发机/执行机** `192.168.1.198:8080` | 同上 |
| SDK | `robotic-arm==1.1.7`（导入名 `Robotic_Arm`） | 远端 `/tmp/rmvenv` 已装，wheel 在 `/tmp/robotic_arm-1.1.7-py3-none-any.whl` |
| 第二台机械臂 | **现场没有** | 这是 L13/SC2 无法闭合的唯一硬约束 |

---

## 二、里程碑定义与验收标准（原文口径）

来源：`.gsd/phases/01-acm1jg-phase/01-ROADMAP.md`（DB 渲染物）。

**愿景**：在 2 套睿尔曼 RM75-B 上建成一套只依赖厂商**最底层接口**（关节角完全透传 + UDP 主动上报状态）、**不使用厂商任何解算与运动学能力**、以 **Drake 为规划内核**的双臂运动规划与操作软件；把既有 9 份设计文档的 6 个架构冲突收敛为可施工裁决，并把"体验优于 MoveIt Pro"落到可复算指标上。

### 六条验收标准（Success Criteria）

| # | 标准 | 当前判定（2026-11-23 验证轮） |
|---|---|---|
| SC1 | 真实 RM75-B 上 Drake 规划轨迹经自研实时层透传执行完毕，末端跟踪误差 < 3 mm，100 Hz 连续 30 min 无故障 | **UNMET（证据缺位，非失败）**：验证主机不可达真机；但真机侧数据已存在（§七） |
| SC2 | 两台 RM75-B 同一 world 共同抓持搬运，全程保持末端相对位姿，成功率 ≥ 95%、P95 < 3 s | **UNMET（硬约束）**：现场只有 1 台臂；离线口径达标（成功率 1.0 / P95 0.2265 s） |
| SC3 | 一次真机运行的全部消息被录制，离线重放后末端轨迹与原始记录逐点一致 | **PARTIAL（应可判 SATISFIED）**：产物已入库但验证轮未引用（§九） |
| SC4 | 注入拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位四类故障，均进入安全态并按文档恢复成功 | **PARTIAL**：替身层五类全链闭合；四类物理注入未做（需人手） |
| SC5 | 厂商解算接口调用次数在主链路中为 0（CI 静态检查 + 运行时审计双证） | **SATISFIED** |
| SC6 | 新机器 30 分钟内一条命令重建完整环境，且不依赖 ROS2 发行版 | **SATISFIED**（实测 8–9 s） |

> 六条标准的原文与判定差异分析见 §八。

---

## 三、交付物全景（10 切片 / 68 任务 / 全 done）

| 切片 | 主题 | 风险 | 依赖 | 任务数 | 状态 |
|---|---|---|---|---|---|
| S00 | 环境与骨架 | low | — | 6 | ✅ complete |
| S01 | 模型与夹爪 | medium | S00 | 6 | ✅ complete |
| S02 | 规划内核与同源框架 | medium | S01 | 8 | ✅ complete |
| S03 | 单臂真机执行与安全 | high | S02 | 8 | ✅ complete |
| S04 | 双臂几何与标定 | medium | S03 | 8 | ✅ complete |
| S05 | 双臂紧协调规划 | high | S04 | 7 | ✅ complete |
| S06 | 任务原语与感知接入 | medium | S05 | 8 | ✅ complete |
| S07 | 工程化与产品化 | low | S06 | 7 | ✅ complete |
| S08 | 验证契约与故障覆盖闭合 | medium | S01,S03 | 4 | ✅ complete |
| S09 | 证据闭合与需求口径校准 | low | S00,S02 | 6 | ✅ complete |

- 需求：**19 条（R001–R019）**，当前 16 active / 3 validated；
- 决策：**49 条（D001–D049）**，全部在 GSD 数据库（`.gsd/DECISIONS.md` 为渲染投影）；
- 任务：68/68 done，无 pending、无 blocker。

---

## 四、架构裁决：6 个冲突如何收敛成 49 条决策

历史背景：`docs/code_doc/设计文档综合/` 下 9 份设计文档在 6 个点上互相冲突（详见 `docs/code_doc/睿尔曼RM75-B方案评审与验证过程记录.md`）。这些冲突在本里程碑里被逐条裁决，关键条目如下（完整 49 条见 `.gsd/DECISIONS.md`）：

| 冲突点 | 裁决 | 决策号 | 代码/配置依据 |
|---|---|---|---|
| 主链路能否调用厂商解算 | **不能**。唯一运动通道 = `rm_movej_canfd(follow=true, trajectory_mode=0)` 完全透传关节角 | D001 | `src/rm75_drake/device/binding.py:111`、`device/exec_loop.py:16` |
| 冗余零空间归属 | 唯一 owner = Drake（`nominal_posture` / `JointCenteringCost`）；厂商 QP 接口全禁 | D003 | 禁用名单见 `versions.lock:146` 起 |
| 官方栈（MoveIt2/rm_control） | **仅可作离线对照基线**，禁止与主链路并发写同一控制器连接 | D006 | `device/hardware_lock.py`（同一 IP+port 独占） |
| 环境基线 | 受支持 = Python 3.12 + Drake 1.55.0；降级档 = 3.10 + 1.51.1（仅 fast 冒烟） | D007 | `tools/setup_env.sh` |
| GCS 实现 | 用 `GraphOfConvexSets.SolveShortestPath` 空间凸集图最短路，**不用** `GcsTrajectoryOptimization`（其 `AddRegions` 不可用） | D014 | `src/rm75_drake/planning/gcs.py` |
| 双臂紧协调路线 | leader 最小坐标参数化（R^8 = 7 关节 + follower 肘点方位角 ψ），相对位姿约束**构造性满足** | D029 / D033 / D034 | `coordinated/parameterization.py`、`kinematics/analytic_ik.py` |
| 场景装配 | 统一走 `RobotDiagramBuilder`；`CollisionCheckerParams.model` 必须传 `RobotDiagram` | D015 | `src/rm75_drake/planning/scene.py` |
| 感知/障碍物注册 | 必须"注册 + 显式 AssignRole + 重建 checker + 回填 bundle"四步合一 | D037 | `src/rm75_drake/tasks/perception.py` |
| 退出码契约 | `verify_env`：0 通过 / 1 检查失败 / 2 参数或环境错误；`--joint-limits` 非法路径 ⇒ 2 | D045 | `tools/verify_env.py:972-1221` |
| 第五类故障载体 | 「供电跌落」用帧级系统错误码 `err_code` 承载，稳定 reason `power_supply_drop`，安全态 FAULT | D044 | `device/state_machine.py:368`、`device/recovery.py:137,393` |
| FK 验收门槛 | 官方 URDF `rpy=1.5708` 舍入 ⇒ 固有残差 2.2407e-6 m；真实 URDF 门槛放宽为 < 5e-6 m，常量收敛为唯一真相源 | D047 | `src/rm75_drake/kinematics/mdh.py:145-192` |
| 证据存储位置 | 进 git 的证据一律落 `docs/evidence/`（JSON + Markdown 双份），`.gsd/` 只作过程记录 | D046 | `docs/evidence/` 现有 8 个文件 |

### 4.1 过程中踩过的"Drake 陷阱"（已被固化为项目记忆）

| 陷阱 | 现象 | 固化方式 |
|---|---|---|
| Drake `Parser` 只持 `MultibodyPlant*` 裸指针，Python 侧不保活 | plant 被 GC ⇒ 后续 `AddModels()` **SIGSEGV**（段错误而非异常） | `package_map.py:88` 用 `WeakKeyDictionary` 保活，暴露 `plant_for_parser()`（`:227`） |
| `SceneGraph` 静态障碍物必须 `RegisterAnchoredGeometry` + **显式** `AssignRole(ProximityProperties)` | 不赋角色 ⇒ 碰撞检查"看不见"障碍物，**静默失效** | `tasks/perception.py` 四步合一 API |
| Drake 1.51/1.55 不实现 `<drake:disable_collision>` 标签（`libdrake.so` 内零出现） | 写了也没用；且 `default_collide="false"` 将来会静默关掉全部自碰撞 | URDF 保留声明块但写 `default_collide="true"` |
| `CollisionCheckerParams.model` 只接受 `RobotDiagram` | 传 plant 抛 pybind cast 异常 | `planning/scene.py` 装配顺序固定 |
| 官方 URDF `rpy=1.5708` 四舍五入 | 任意独立解析模型与 Drake FK 之间固有残差 ≈ 2.2407e-6 m | 门槛常量唯一真相源 + `docs/architecture/fk-tolerance.md` 正式声明 |

---

## 五、切片交付全景（逐切片"交付了什么 / 怎么验的"）

> 每格的"验证"列都是**可执行命令**，复核方式见 §十。

### S00 环境与骨架（low）
- 交付：`versions.lock`（Drake / RM_API2 / rm_models / 两指夹爪 commit 锁定 + `forbidden_apis` 禁用名单 + `allowed_apis` 白名单）、`constraints/py312.txt`（18 个包，带 hash）、`tools/setup_env.sh`、`tools/fetch_upstream.py`、`tools/verify_env.py`、`scripts/ci.sh`（fast/medium/nightly 三档）、`licenses/registry.yaml`。
- 验证：`bash tools/setup_env.sh --offline --profile py312 --venv <fresh>` → exit 0、18/18 包 `--require-hashes` 安装、`verify_env --check` 逐包 OK、**wall time 8–9 s**、全程无 ROS2 ⇒ SC6 以 ~20 倍余量达标。

### S01 模型与夹爪（medium）
- 交付：`models/rm75_drake/rm75_drake.urdf`（官方 URDF 定点改造：速度限位 J1/J2=3.1416、J3–J7=3.927 rad/s、轴向量 snap、碰撞几何改 box、显式 weld 固定基座）、`rmg24_drake.urdf`（夹爪独立模型 + `tcp` frame）、`src/rm75_drake/gripper.py`（mm ↔ 0–1000 无量纲映射）。
- 验证：模型回归 26 断言（DoF=7、质量 7.776 kg±1e-6、零位末端 0.8505 m±1e-6、7 条限位、自碰撞过滤）。
- 遗留口径：FK 门槛从 `<1e-6 m` 放宽到 `<5e-6 m`（见 §四 D047），**这不是放水，是把舍入残差显式化并钉死**（`ZERO_Y_ROUNDING_BIAS_M = 2.2407e-6`，容差 ±1e-8；错误 alpha 表仍须 > 1e-3 m 以保鉴别力）。

### S02 规划内核与同源框架（medium）
- 交付：`planning/scene.py`（场景装配 + `package://` 合并 + `SceneError` 点名出错键）、`planning/ik.py`、`planning/gcs.py`、`planning/regions.py`、`planning/retime.py`、`planning/trajectory.py`（`Trajectory` 不变量）、`src/rm75_drake/recording.py`（录制 schema v1）。
- 验证：规划回归、场景 39 条负例、轨迹不变量、仿真回放碰撞复检逃逸数 0。
- 诚实边界：本机无整数 GCS 求解器，`plan_stats` 的 GCS P95 **恒为凸松弛口径**，不得表述为"整数 GCS 达标"。

### S03 单臂真机执行与安全（high）
- 交付：`device/` 全家桶——`safety.py`（安全七闸纯函数）、`exec_loop.py`（5 ms 执行环）、`state_machine.py`（9 态状态机）、`recovery.py`（故障分类 + 安全态映射）、`transport.py` / `sdk.py` / `binding.py`（SDK 绑定 + 审计）、`hardware_gate.py`（硬件门控）、`hardware_lock.py`（独占锁）、`run_log.py`（运行日志独立 schema v1）、`metrics.py`、`devices/*.yaml`（部署配置真相源）。
- 验证：七闸逐闸正例 + 反例、状态 age、错误码恢复流程全绿；真机侧 L0–L12 见 §七。

### S04 双臂几何与标定（medium）
- 交付：`bimanual/dual_scene.py`（**同一个 `MultibodyPlant` 装两臂**：Drake 不允许同一 URDF 装两次 ⇒ 每臂一份重命名 URDF 变体，生成器 `tools/gen_dual_arm_models.py` 入 git）、`bimanual/sync_exec.py`（复用 S03 七闸/指标，无第二套实时层）、`bimanual/collision_filter.py`、`calibration/`（纯 numpy 内核标定）。
- 验证：192 条专项全绿；合成真值 tool_tcp RMS 2.16e-17 m、base_extrinsic 8.30e-17 m；真实标定精度零验证（`calibration/dual_arm.yaml` 明写 `measured=false`）。

### S05 双臂紧协调规划（high）
- 交付：`coordinated/parameterization.py`（8 维最小坐标 + 常数 `X_LR`）、`coordinated/regions.py`（自研 R^8 凸区域生成，不依赖 Drake IRIS）、`coordinated/planner.py`（`plan_coordinated`）、`coordinated/trajectory.py`（`DualTrajectory`）、`coordinated/limits.py`。
- 验证：相对位姿残差 **max 1.835e-08 m / 9.892e-08 rad**；14 维密采样零逃逸；离线成功率 1.0、solve P95 **0.2265 s**（< 3 s）。
- 关键机制：共享时间轴由"14 维联合重定时 → 拆成两条 7 维 `Trajectory`"构造保证（决策 D032），因此不改动 S02 冻结的 `TrajectorySample`/`JOINT_COUNT=7`。

### S06 任务原语与感知接入（medium）
- 交付：`tasks/primitives.py`（10 原语目录 + 默认门槛）、`tasks/schema.py` + 仓库根 `tasks/*.yaml`（任务定义独立 schema）、`tasks/bimanual.py`（搬运/交接复用 S05 的 `X_LR`）、`tasks/perception.py`（非阻塞有界队列 + stale 标志 + 膨胀裕量）、`tasks/state_machine.py`（零 pydrake 依赖）。
- 验证：原语级 `thresholds_met=true`；感知三断言鉴别力（`num_escapes=47>0` / 膨胀为 0 的盒子仍碰撞 / 旧 checker 仍自由）。
- 缺口（已契约化）：`cli task` 不支持双臂场景 ⇒ 退出码 2 + 稳定 reason `task_single_arm_only`（决策 D048/D049，`cli.py:138`）。

### S07 工程化与产品化（low）
- 交付：`scripts/gen_golden.py`（golden-file 基线，绑定 urdf/scenario/task **三指纹** + 自校验和）、`regression/session.py`（独立 session schema v1，只读聚合 recording + run_log，不升版）、`scripts/long_run.py`（8 h 长稳载体）、`scripts/ci.sh` 接线（fast 档新增 regression / assets/lock / verify-env-contract 等恒跑项）。
- 验证：`python3 scripts/gen_golden.py --check` → exit 0、`pointwise_identical=True`、722 点、`tcp_mismatches=0`、`max_tcp_diff_m=0.000e+00`；负例"基线平移 1e-6 m ⇒ 非 0 退出"可鉴别。

### S08 验证契约与故障覆盖闭合（medium）
- 交付：`tests/test_verify_env_contract.py`（退出码契约 5 条断言）、`ci.sh` 恒跑项、`REASON_POWER_SUPPLY_DROP` + `state_machine.system_error_code()` 单一真相源、`ScriptedTransport.set_system_error()`、`scripts/fault_inject.py` 扩到 5 类、`runbooks/s03_rm_device_bringup.md` §6/§6.1/§10。
- 验证：`python3 tools/verify_env.py --model --joint-limits /nonexistent/path.yaml` → **exit 2** 且 stderr 点名路径（对齐 `01-01-UAT.md:47` 逐字契约）；`python3 scripts/fault_inject.py --json` → exit 0，5/5 case `ok=true / recovered=true / safe_state=fault / final_state=ready`、`failed=[]`。

### S09 证据闭合与需求口径校准（low）
- 交付：`scripts/collect_evidence.py`（**唯一证据收集器**，9 个检查项：`pytest-not-slow` / `verify-env-model` / `verify-env-plan` / `forbidden-apis-scan` / `cli-single-replay` / `cli-bimanual` / `plan-stats` / **`fault-inject`** / `ci-fast`）、`docs/evidence/`（机器可读 JSON + 人类可读 Markdown 双份）、`docs/architecture/fk-tolerance.md`、`tools/fetch_upstream.py --verify`（归档字节级校验）。
- 验证：`python3 tools/fetch_upstream.py --verify` → exit 0「校验通过：2 个资产」，sha256 与 MANIFEST + versions.lock 三方逐字一致（rm_models `fb7ab533…`、two_finger_gripper `975b5474…`）。

---

## 六、代码层面：有据可依的 12 条关键实现

| # | 设计主张 | 代码依据 | 可复算判据 |
|---|---|---|---|
| 1 | 唯一运动通道 = `rm_movej_canfd` 透传关节角 | `device/binding.py:111`、`device/exec_loop.py:16` | `scripts/scan_forbidden_apis.py src tools scripts` → 0 命中 |
| 2 | 5 ms 透传周期 | `device/exec_loop.py:81` `PERIOD_S = 0.005` | 真机 L7：P99.9 抖动 0.249 ms（阈值 0.5） |
| 3 | 安全七闸为纯函数，单点钳位 | `device/safety.py:728 apply_safety`；G1–G7 标签在 `:115-132` | 正例+反例单测；真机 L10：7/7 闸拦下 |
| 4 | 钳位类违规只记录、停机类才升级 | `device/safety.py:307-346`（`action` = clamp / slow_stop / stop） | `fault_inject.py` 断言 `safe_state` 迁移 |
| 5 | 厂商解算接口运行时零调用（审计那半） | `device/sdk.py:455 class SdkAudit`（`forbidden` / `forbidden_call_count`） | `cli exec --json` 输出 `sdk_audit.forbidden_call_count=0`；>0 ⇒ exit 1 |
| 6 | 禁用名单唯一真相源 | `versions.lock:146 forbidden_apis`（13 项）+ `allowed_apis` 白名单 | `scan_forbidden_apis.py` 扫描 90 文件 / 名单 13 项 / 0 命中 |
| 7 | Drake plant 保活（防 SIGSEGV） | `package_map.py:88 _PLANTS_BY_PARSER` + `:227 plant_for_parser` | 场景装配路径必须经 `package_map` |
| 8 | 相对位姿构造性满足 | `coordinated/parameterization.py:33-34,536`（`T_follower_world = T_leader_world · X_LR`） | 残差 ≤ 1e-6 m/rad，实测 1.8e-08 / 9.9e-08 |
| 9 | 解析 IK 为纯 numpy 闭式（8 分支 × ψ） | `kinematics/analytic_ik.py`（`REASON_BRANCH_EXHAUSTED` 等稳定 reason） | 与 Drake FK 往返误差 ~1e-15 m（精确表） |
| 10 | 第五类故障单一判定源 | `device/state_machine.py:368 system_error_code()` + `device/recovery.py:137 SAFE_STATE_BY_REASON` | `SAFE_STATE_BY_REASON[power_supply_drop] == DeviceState.FAULT` |
| 11 | golden 基线三指纹防"假绿灯" | `scripts/gen_golden.py:167,199-201` | 指纹漂移 ⇒ 基线主动失效 |
| 12 | CLI 退出码语义统一 | `cli.py:129-138`（EXIT_OK/EXIT_FAILURE/EXIT_USAGE + `REASON_TASK_SINGLE_ARM_ONLY`） | `cli task --task tasks/handover.yaml` → exit 2 + reason |

---

## 七、真机验证阶梯：实际测到了什么

工具：`scripts/real_machine_ladder.py`（L0–L14，会动级别必须显式 `--allow-motion`，见 `:124`）。
原始产物：`docs/evidence/rm75-b-ladder-2026-11-19.json`（第一轮）+ `docs/evidence/rm75-b-real-arm-ladder.md`（三轮纪要）。
执行环境：远端工作站（宿主网卡 `192.168.1.198/24`），规划在容器 `drake_dev`，硬件执行在宿主 venv。

| 级别 | 状态 | 关键实测数字 |
|---|---|---|
| L0 链路与网关 | ✅ | TCP 连接 0.55 ms；示教器网页 HTTP 200 |
| L1 SDK 与控制器 | ✅ | SDK 1.1.7；`arm_dof=7`；控制器代次 3 |
| L2 连接与只读状态 | ✅ | 句柄 id=1；关节角/位姿/电流/错误码全读回；错误码全 0 |
| L3 UDP 主动上报 | ✅ | 3 s 收 600 帧；周期 P50 4.998 ms；不完整帧 0（`cycle` 读法 A，单位 = 5 ms 的倍数） |
| L4 使能与就绪 | ✅（第二轮） | 7 轴全使能；`arm_current_status=0`（急停已释放） |
| L5 运动前静态自检 | ✅ | 关节全在软限位内；围栏零违规；**高度余量仅 ~4 mm**（现场风险） |
| L6 单轴点动与回零 | ✅ | 点动误差 0.653°；回零误差 0.412° |
| L7 5 ms 透传抖动 | ✅ | 抖动 **P99.9 = 0.249 ms**（阈值 0.5）；违规 0；传输错误 0 |
| L8 轨迹执行 + 末端跟踪 | ✅ | 131/131 点；关节误差 max 0.472°；末端误差 max **2.56 mm**（阈值 3） |
| L9 夹爪 | ✅ | 在线；张开回读 1000/1000（误差 0）；闭合回读 12/0（误差 12） |
| L10 安全七闸 | ✅ | 7/7 闸全部拦下；真机越界目标被安全层拒绝（未下发） |
| L11 故障注入与恢复 | 🟡 部分 | 夹爪未到位链在真机闭合（注入→检测→holding→恢复→ready）；**软件急停不可用**（`rm_set_arm_emergency_stop` 返回 -4）；其余四类物理故障需人手 |
| L12 长稳 30 min | ✅ | 180000 拍全部下发；违规 0；抖动 P99.9 0.314 ms；命令→执行延迟 P99.9 0.290 ms |
| L13 双臂 | ⛔ 阻塞 | **现场只有一台臂** |
| L14 golden 回归 | ✅ | 722 点逐点一致，TCP 失配 0，exit 0 |

### 7.1 真机跑出来并修掉的缺陷（提交 6d213bf / 54e8b7d / 4cfab73）

1. **1-based 轴号**：写控制器限位的 `joint_num` 是 1~7，不是 0~6；
2. **返回码假绿灯**：控制器拒绝请求时**返回 1 而不抛异常**，必须检查 SDK 返回码，否则失败被当成功；
3. **模型限位比控制器宽**：软件侧限位必须比硬件再收 3–5°；
4. **SDK 调用无超时保护** ⇒ 长稳中曾出现 `Send would block, retrying later...` 刷屏。

> 这四条是本里程碑最有价值的产出之一：**它们只有真机能暴露**，纯离线测试永远测不出来。

---

## 八、当前阻塞：`/gsd auto` 为什么跑不起来（根因剖析）

### 8.1 机制层面：阻塞不是"还有活没干"

- 切片 10/10 complete，任务 68/68 done，`gsd_milestone_status` 显示里程碑 `status=queued`、`lifecycleStatus=ready`；
- 真正的阻塞来自 **DB 里最新一条 milestone validation 的 verdict ≠ pass**：
  - `src/resources/extensions/gsd/state/derive/from-db.ts:312` —— verdict 为 `needs-attention` 时派生 `phase=blocked`，并写入 blocker 文案；
  - `src/resources/extensions/gsd/validation-block-guard.ts` 的 `VALIDATION_BLOCK_RE` 命中该文案后，把 `auto / next / start / ship / complete-milestone / do / discuss-phase / import …` 全部拦截，而 `validate-milestone / reassess / uat / blocker / audit-fix --dry-run` 仍可用；
  - `src/resources/extensions/gsd/milestone-validation-verdict.ts` —— 取 **最新一条** `scope=milestone-validation` 的 assessment 作为权威 verdict（`01-VALIDATION.md` 只是投影，不能自己进权威）。
- **结论**：`/gsd auto` 被挡 ≠ 有未完成工作，而是"上一轮验证的结论不是 pass"。解封 = 补齐证据/台账后**重跑一次 verdict=pass 的结构化验证**（`gsd_validate_milestone`，`remediationRound=2`）。

### 8.2 判定为 needs-attention 的三类成因

验证轮（2026-11-17 起，round 1）的四类验证结论：

| 验证类 | 判定 | 依据 |
|---|---|---|
| Contract | **PASS** | 170 passed（模型回归 / MDH-FK / 七闸正反例 / 状态机 / 恢复 / 退出码契约）；`scan_forbidden_apis` 0 命中；`ci.sh fast` PASS=17 SKIP=0 FAIL=0 |
| Integration | **NEEDS-ATTENTION** | 真机两项 UNMET：`soak_test.py` / `measure_jitter.py` 在**验证主机**上 `SKIP(reason=hardware_unreachable)` |
| Operational | **NEEDS-ATTENTION** | 五类故障离线替身全链通，但 8 h 长稳 / 30 min P99.9 / 急停 < 100 ms 无真机证据 |
| UAT | **NEEDS-ATTENTION** | 引用上述 + 台账不一致 |

> 聚合规则：无 FAIL ⇒ 不降级为 `needs-remediation`；存在 NEEDS-ATTENTION ⇒ 判 `needs-attention`。
> **关键判据是「证据缺位」而不是「观察到的失败」**：本轮所有可执行判据都复跑了，且无一失败。

### 8.3 成因拆解（逐条可核对）

#### 成因 A：验证主机与真机主机不是同一台 —— 证据"存在但没被引用"

- 验证轮在**本开发机**复跑 `soak_test.py` / `measure_jitter.py`，此处没有 `robotic-arm` SDK、也路由不到 `192.168.1.18` ⇒ 脚本按**设计**输出 `SKIP(reason=hardware_unreachable)`（`device/hardware_gate.py:93-156`：探测失败 → SKIP + 逐条列出未验证指标，**退出码 0**）。
- 于是 SC1/SC2 被判 UNMET。但事实上：
  - **SC1 的真机数字已经测出来了**（L8 末端 2.56 mm、L12 30 min 180000 拍、P99.9 0.314 ms），只是它们落在 `docs/evidence/rm75-b-real-arm-ladder.md` 的叙述里，**第二/三轮的机器可读 JSON 没有入库**（远端 `/tmp/soak30.json`、`/tmp/ladder_after_recovery.json`、`/tmp/evidence_final.json` 现仍在）；
  - **SC3 已被完全满足但被判 PARTIAL**：入库产物 `docs/evidence/rm75-b-real-exec-runlog.json`（`mode=hardware`、131 采样、violations=0）+ `rm75-b-real-exec-recording.json` + `rm75-b-real-exec-compare.json`（`pointwise_identical=true`、`max_joint_diff_rad=0.0`）就是"一次真机运行全部消息被录制 → 离线重放逐点一致"的完整证据链；验证轮只复跑了离线 722 点 golden，**没有引用这组三件套**。
- 结论：**SC1/SC3 的"缺证据"部分是引用口径问题，不是证据缺失**。
- **2026-11-23 补注（已执行）**：第三轮真机原始 JSON 已从远端 `/tmp` 取回并入库 `docs/evidence/`（全阶梯最终报告、30 min 长稳、末端跟踪、最终 runlog/plan、电流标定、网卡恢复回归），并新增人类可读纪要 `rm75-b-final-round-2026-11-20.md`；**8 h 长稳是失败记录**（`rm75-b-soak-8h-failed-2026-11-20.json`：`dropped=879 999`、抖动 P99.9 9.9276 ms ⇒ `status=failed`，与提交 `4cfab73` 的 SDK 超时保护同根因），只能作失败溯源，**不得当作达标证据**。

#### 成因 B：台账陈旧（4 处，均为记录级、非实现级）

| # | 位置 | 现状 | 事实 |
|---|---|---|---|
| B1 | `.gsd/phases/01-acm1jg-phase/01-01-ASSESSMENT.md` | `verdict: FAIL` | 其唯一 FAIL 判据（`--joint-limits` 非法路径期望 2、实测 1）已被 S08 修复，本轮实测 **exit 2** |
| B2 | `01-00-ASSESSMENT.md` | `verdict: PARTIAL` | 未决项"上游归档正向字节级校验"已被 S09 闭合（`fetch_upstream.py --verify` exit 0） |
| B3 | `01-02-ASSESSMENT.md` | **文件不存在** | 10 切片唯一缺口；S02 的 runtime-executable 证据已入库（`docs/evidence/S02-rerun.md/json`），但缺与其它切片同构的逐条 check + 证据 ID 记录 |
| B4 | S09 证据面 / 需求台账 | 缺第五类故障项；R013 Proof 未回填 | **已闭合**：`collect_evidence.py` 新增 `fault-inject` 检查项（5 类替身注入全链恢复，exit 0）；R013 Proof 已回填 |

#### 成因 C：硬约束（补不上的部分）

| 项 | 为什么补不上 |
|---|---|
| SC2 双臂共同搬运 | 现场只有 1 台 RM75-B（L13 阻塞） |
| SC4 四类物理故障注入 | 需人手在设备旁操作（脚本按回车推进） |
| 急停响应 < 100 ms | `rm_set_arm_emergency_stop` 返回 -4，**软件急停无效**，必须物理急停 |

### 8.4 与"代码是否有缺陷"的关系

**本轮验证未发现任何观察到的实现失败**：170 passed、1393 passed / 3 skipped（`pytest -m "not slow"` 全量）、`ci.sh fast` PASS=17 SKIP=0 FAIL=0、`gen_golden.py --check` exit 0、`fault_inject.py` 5/5、`coordinated_stats.py` 成功率 1.0 / P95 0.2265 s、`fetch_upstream.py --verify` exit 0。
即：**阻塞是"验收判定"层面的，不是"代码质量"层面的**——但它确实由**证据组织方式 + 台账维护**两件事造成，属于工程纪律问题，可以也应该修掉。

---

## 九、证据台账核对表（缺什么 / 能补 / 补不了）

| 判据 | 需要的证据 | 当前状态 | 能否在本环境闭合 |
|---|---|---|---|
| SC1 末端误差 < 3 mm、30 min 无故障 | 真机阶梯 L8/L12 的机器可读 JSON | ✅ **已补入**（`rm75-b-ladder-final-2026-11-20.json` 全阶梯 PASS=11/FAIL=0；`rm75-b-soak-30min-final-2026-11-20.json` 180000 拍零违规、抖动 P99.9 0.3145 ms） | ✅ 已闭合 |
| SC2 双臂搬运成功率 | 第二台机械臂 | ❌ 不可得 | ⛔ 需硬件 |
| SC3 真机录制逐点一致 | runlog + recording + compare | ✅ **已入库**（另有最终一轮 `rm75-b-runlog-final-2026-11-20.json` + `rm75-b-plan-final-2026-11-20.json`） | ✅ 已闭合（引用口径） |
| SC4 四类物理故障 | 人手注入记录 | ❌ 替身层已闭，真机未做 | ⚠️ 需人手 |
| SC5 厂商接口零调用 | 静态扫描 + 运行时审计 | ✅ 双证齐备 | ✅ |
| SC6 一条命令重建环境 | setup_env 日志 | ✅ 实测 8–9 s | ✅ |
| S01 台账 | 重写 ASSESSMENT | ❌ 仍 FAIL | ✅ 可补 |
| S02 台账 | 补 ASSESSMENT | ❌ 缺文件 | ✅ 可补 |
| S00 台账 | 重写 ASSESSMENT | ❌ 仍 PARTIAL | ✅ 可补 |
| S09 证据面 | 加 fault-inject 检查项 + 回填 R013 | ❌ | ✅ 可补 |
| 急停 < 100 ms | 物理急停实测 | ❌ 软件急停无效 | ⛔ 需物理急停 |

---

## 十、结论与下一步

### 10.1 结论

1. **M001-acm1jg 的软件交付是完整的**：10 切片 / 68 任务全 done，19 条需求 0 MISSING，14 条跨切片边界全部 HONORED、0 GAP，Contract 类验证 PASS，全量回归无回归。
2. **当前阻塞是判定层面的，不是实现层面的**：验证轮在**没有真机路由的开发机**上复跑硬件脚本，得到 `hardware_unreachable` 的 SKIP，从而把若干已被真机覆盖的判据判为 UNMET；叠加 4 处台账陈旧（两处 verdict 与事实不符、一处文件缺失、一处证据项缺失），聚合为 `needs-attention`。
3. **可恢复**：把远端已有的真机原始产物入库 + 修正台账 + 正确引用已有真机证据后重跑验证，**在不降低任何验收标准的前提下**具备拿到 `pass` 的条件；剩余不可闭合项（第二台臂、四类物理注入、物理急停）属硬件/人力前提，应**如实登记为未验证项**而非伪造通过。
4. **红线不变**：真机运动级复跑必须显式 `--allow-motion`、必须显式目标位姿、同一时刻只允许一个真机进程、**物理急停在手**。

### 10.2 三条可选路径（需决策）

| 路径 | 内容 | 代价 |
|---|---|---|
| **A. 证据补全（推荐）** | ① **已完成**：真机第三轮原始 JSON 取回入库 `docs/evidence/`（8 个 JSON + 1 份纪要），README 索引与速查表同步更新；② **已完成**：全过程说明 / ROADMAP / VALIDATION 归档进 `docs/milestone/`；③ **待做**：用 `gsd_summary_save` 重写 S00/S01 ASSESSMENT、补建 S02 ASSESSMENT；④ **已完成**：`collect_evidence.py` 增加 `fault-inject` 检查项 + `gsd_requirement_update` 回填 R013；⑤ **待做**：`gsd_validate_milestone`（round 2）引用上述新证据 | 台账类改动无硬件依赖；运动级复跑需人在场 |
| **B. 口径调整** | 只修台账，并把 SC1/SC2 中"需双真机/物理急停"明确改判为"现场条件不具备 ⇒ 登记项" | 等于承认降低验收标准，需评审同意 |
| **C. 只出报告** | 维持现状 | 里程碑持续 blocked |

### 10.3 复算命令清单（复核本文所有数字）

```bash
# —— 离线（开发机即可）——
cd .gsd-worktrees/M001-acm1jg
python3 -m pytest -q -m "not slow"                       # 1393 passed / 3 skipped
python3 scripts/scan_forbidden_apis.py src tools scripts   # 0 命中（90 文件 / 13 项）
bash scripts/ci.sh fast                                   # PASS=17 SKIP=0 FAIL=0
python3 scripts/gen_golden.py --check                     # 722 点逐点一致，exit 0
python3 scripts/fault_inject.py --json                    # 5/5 全链恢复，exit 0
PYTHONPATH=src python3 scripts/coordinated_stats.py --samples 20 --json   # 成功率 1.0 / P95 0.2265 s
python3 tools/verify_env.py --model --joint-limits /nonexistent/path.yaml # exit 2
python3 tools/fetch_upstream.py --verify                  # exit 0（2 个资产）
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/handover.yaml  # exit 2 + task_single_arm_only

# —— 真机（远端工作站，网卡 192.168.1.198/24；容器 drake_dev 做规划，宿主 venv 做执行）——
docker exec ros_dev bash -lc "source /opt/conda/etc/profile.d/conda.sh; conda activate drake_dev"
/tmp/rmvenv/bin/python scripts/real_machine_ladder.py --device-config devices/rm75_b_192_168_1_18.yaml --json
```

---

## 附录 A：本次核查动作清单（便于复核本文可信度）

| # | 动作 | 结果 |
|---|---|---|
| 1 | 读取 `.gsd/phases/01-acm1jg-phase/01-VALIDATION.md`（round 1, verdict=needs-attention） | 四类验证结论与 4 处台账缺口逐条确认 |
| 2 | 读取 `gsd_milestone_status` / `gsd_project_snapshot` | 10/10 切片 complete、68/68 任务 done、无 blocker 记录 |
| 3 | 读 GSD 引擎源码 `validation-block-guard.ts` / `from-db.ts` / `milestone-validation-verdict.ts` | 确认阻塞的**唯一**成因是 validation verdict |
| 4 | 读 `01-0*-ASSESSMENT.md` front-matter | 确认 S01=FAIL、S00=PARTIAL、S02 文件缺失 |
| 5 | 读 `docs/evidence/rm75-b-real-exec-{runlog,recording,compare}.json` | 确认 SC3 的真机证据链**已入库**且逐点一致 |
| 6 | 读 `docs/evidence/rm75-b-ladder-2026-11-19.json` | 确认第一轮 L0–L5/L10 机器可读，第二/三轮仅存于远端 `/tmp` |
| 7 | 读 `versions.lock` / `device/sdk.py` / `device/safety.py` / `device/exec_loop.py` / `kinematics/mdh.py` | 确认 §六 12 条代码依据的行号与语义 |
| 8 | SSH 至 `172.16.119.199` 实测：网卡 `192.168.1.198/24`、`ping 192.168.1.18` 通、容器内也通、`/tmp/rmvenv` 有 SDK 1.1.7、`drake_dev` 有 Drake 1.55 | 确认"证据补全"路径技术可行 |

## 附录 B：术语与路径速查

| 名称 | 含义 |
|---|---|
| `M001-acm1jg` | 本里程碑 ID（随机后缀来自 `unique_milestone_ids` 偏好） |
| `S00`–`S09` | 切片 ID；`T01`–`T08` 为切片内任务 |
| `R001`–`R019` | 需求 ID |
| `D001`–`D049` | 决策 ID |
| `.gsd-worktrees/M001-acm1jg/` | 本里程碑的真实代码工作区（NFS，与远端同文件） |
| `docs/evidence/` | 进 git 的交付证据（JSON + Markdown 双份） |
| `versions.lock` | 依赖锁 + 禁用/允许接口名单的唯一真相源 |
| `L0`–`L14` | 真机验证阶梯级别 |

---

*本文所有结论均可用 §10.3 的命令复算；凡未复算的数字均已在 §七/§八中标注其来源文件与口径。*
