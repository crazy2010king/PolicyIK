# S06 任务原语与感知接入作业手册（RM75-D）

> **适用范围**：S06「任务原语与感知接入」——10 个可组合任务原语、任务状态机、
> 单臂 numpy 快路径求解器、双臂搬运 / 交接原语、非阻塞感知适配接口、
> 原语级指标口径与 `task` CLI。
>
> **本文件当前状态**：由 **S06-T08** 落笔（六节齐备：口径定义 / 逐原语门槛表 /
> 失败模式词表 / 感知生效边界 / 未验证物理指标清单 / 运维三节）。
>
> **诚实前提**：本仓库当前**没有真机、没有实测数据**。所有真实物理指标
> （真机抓放成功率 / 夹持误差 / 交接保持漂移 / 感知延迟 / 夹爪力控 / 长稳节拍）
> **一律未验证**，由 `rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS`
> 与 `scripts/task_stats.py` **逐条列出**（硬件门控 SKIP，**退出码仍为 0**），
> **绝不用合成数据冒充物理达标**（K001）。

---

## 0. 真相源一览

| 内容 | 真相源 |
|---|---|
| 任务文件（部署实例数据） | 仓库根 `tasks/*.yaml`（`rm75_drake.paths.task_files_dir`） |
| 任务 / 原语 schema | `rm75_drake.tasks.schema`（`SCHEMA_VERSION = 1`） |
| 原语白名单（10 个） | `rm75_drake.tasks.schema.PRIMITIVE_NAMES` |
| 原语目录（种类 / 描述） | `rm75_drake.tasks.primitives.PRIMITIVE_SPECS` |
| 原子目标与 goal_id 唯一性 | `rm75_drake.tasks.goals`（`GoalRegistry`） |
| 任务状态机 | `rm75_drake.tasks.state_machine.TaskStateMachine` |
| 单臂快路径求解器 | `rm75_drake.tasks.solver.solve_task` / `solve_primitive` |
| 失败阶段 / 原因词表 | `rm75_drake.tasks.solver.STAGES` / `REASONS`（状态机追加 `state` 段） |
| 双臂搬运 / 交接 | `rm75_drake.tasks.bimanual.plan_carry` / `plan_handover`（常数 `X_LR`） |
| 感知适配（非阻塞事件 + 保守场景） | `rm75_drake.tasks.perception` |
| 原语级指标口径与门槛 | `rm75_drake.tasks.metrics`（`UNVERIFIED_TASK_METRICS` 为唯一真相源） |
| 任务 CLI | `rm75_drake.cli task`（退出码 0 / 1 / 2） |
| 指标脚本 | `scripts/task_stats.py`（退出码 0 / 1 / 2） |

> **单位约定（全表 SI，禁止在别处另立单位）**：长度 **m**，角度 **rad**，时间 **s**；
> 夹爪开口指令单位为 **0–1000**。

---

## 1. 口径定义：验收口径 vs 端到端口径（S06-T08）

> **纪律**：**验收口径**与**端到端口径**必须**分行**报，**不得混用**。
> 端到端口径**单列**，**不作为验收口径**；也**不得**用端到端数字去顶替验收数字。

| 口径 | 边界（**含什么 / 不含什么**） | 指标 | 门槛 |
|---|---|---|---|
| **验收口径** | **只含求解阶段**：解析 IK → 关节插值 → 碰撞复检 → 重定时 → 末端误差判定。**不含**场景装配、不含整链调度开销 | 每原语成功率 | **≥ 0.95** |
| **验收口径** | 同上 | 每原语位置误差 P95 | **≤ 1e-5 m**（默认容差同源） |
| **验收口径** | 同上 | 每原语姿态误差 P95 | **≤ 1e-5 rad**（默认容差同源） |
| **验收口径** | 同上 | 每原语零逃逸率 | **= 1.0**（100%，硬要求） |
| **端到端口径** | 含**场景装配** + **整条原语链**的开销。**单列，不混入验收** | 每样本耗时 p50 / p95 / mean / max | **仅记录**，无门槛 |

* **分母口径**：某样本在首个失败原语处终止 ⇒ 该样本中**后续原语不计入分母**
  （分母 = 「被尝试到的样本数」），否则失败样本会把后续原语的成功率凭空拉低。
* **零逃逸只统计碰撞逃逸**（`reason = "escape"` 或 `stage = "escape"`）；
  其余失败（IK 不可达等）**不计入逃逸**，也不因此把逃逸率拉低。
* **误差门槛三级解析**：**原语 `tolerance` → 任务 `thresholds` → 默认
  1e-5 m / 1e-5 rad**（`rm75_drake.tasks.solver._resolve_tolerances`）。
* **位置 / 姿态误差为空（`None`）表示该原语不产生该类误差**（夹爪类原语），
  **不伪造 0.0**。
* 复算命令（退出码 0，**物理指标一律列为未验证**）：

```bash
# 验收口径 + 端到端口径分行报告 + 未验证清单
python3 scripts/task_stats.py --samples 50
# 机器可读
python3 scripts/task_stats.py --samples 50 --json
# 单行 JSON 摘要（status / reason / stage / 逐原语结果与指标）
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/pick_place.yaml --json
```

## 2. 逐原语门槛表（S06-T08）

> **位置 / 姿态容差**列给出的是**默认容差**（`DEFAULT_POSITION_TOLERANCE_M`
> = 1e-5 m、`DEFAULT_ORIENTATION_TOLERANCE_RAD` = 1e-5 rad）；任务文件可按
> 「原语 `tolerance` → 任务 `thresholds`」**逐级覆盖**。
> **离线成功率门槛**对 10 个原语**一律 ≥ 0.95**；**物理未验证项**列指明该原语
> 在真机上**尚无实测结论**的部分。

| 原语 | 种类 | 成功判据（离线可测） | 位置容差（默认） | 姿态容差（默认） | 离线成功率门槛 | 物理未验证项 |
|---|---|---|---|---|---|---|
| `approach` | motion | 末端 TCP 到达目标位姿且误差 ≤ 容差；轨迹通过碰撞复检 | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机接近位姿复现精度；真机路径无碰撞 |
| `pre_grasp` | motion | 同上（到夹爪可闭合的预备位姿） | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机预抓取位姿偏差；真机无碰撞 |
| `close` | gripper | 夹爪动作调度成功且保持段挂进轨迹（**无 transport 时仍算离线成功，但 `command_issued=False` ⇒ 记为未验证**） | 不适用 | 不适用 | ≥ 0.95 | 真机夹持成功率；真机力控 / 空夹判据；真机夹持误差 |
| `verify` | gripper | 回读夹爪开口与期望值之差 ≤ `verify_position_tolerance`（默认 50 指令单位）；无 transport ⇒ `reason="no_transport"`（失败） | 不适用 | 不适用 | ≥ 0.95 | 真机夹爪回读可用性；真机空夹误判率 |
| `lift` | motion | 末端 TCP 抬起到位且误差 ≤ 容差 | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机抬升后物体是否跟随（滑落 / 掉落） |
| `transit` | motion | 同上（转运到投放点上方） | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机转运中的物体保持与节拍 |
| `place` | motion | 同上（到投放位姿） | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机放置偏差；真机放置后是否稳定 |
| `open` | gripper | 夹爪张开动作调度成功并含保持段（同 `close` 的 `command_issued` 口径） | 不适用 | 不适用 | ≥ 0.95 | 真机释放成功率；真机释放后物体落位 |
| `retreat` | motion | 末端 TCP 退回安全位姿且误差 ≤ 容差 | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机撤离路径无碰撞 |
| `handover` | bimanual_motion | 两阶段交接完成，交接点两 TCP 相对位姿残差 ≤ 1e-6（构造性保持） | 1e-5 m | 1e-5 rad | ≥ 0.95 | 真机交接保持漂移（目标 ≤ 2 mm / 10 s）；真机交接成功率；交接瞬间的关节级镜像跳变影响 |

> **双臂原语的相对位姿口径**：`handover` / 双臂 `transit` 的相对位姿由
> **常数 `X_LR`**（leader TCP 系 → follower TCP 系）**构造性**保持；
> 角色互换后由 `inv(X_LR)` 表达同一几何关系。
> **交接瞬间可能出现关节级镜像跳变**（TCP 不动），这是**已知限制**，不掩饰。

## 3. 失败模式清单：stage + reason 词表（S06-T08）

失败**必须点名**：`stage`（阶段）+ `reason`（稳定词表）+ `message`（人话）。
**禁止**返回“看起来正常”的半成品轨迹。
**词表真相源**：`rm75_drake.tasks.solver.STAGES` / `REASONS`；
状态机追加 `stage = "state"` 段（`rm75_drake.tasks.state_machine.TASK_STAGES`
/ `TASK_REASONS`）。

### 3.1 阶段词表（stage）

| stage | 含义 |
|---|---|
| `schema` | 任务文件 / 原语 / 目标解析层失败 |
| `ik` | 解析 IK 未解出（**reason 直接透出解析 IK 的词表**） |
| `path` | 关节插值或末端误差判定失败 |
| `collision` | 碰撞复检失败（逃逸 / 复检不通过） |
| `retime` | 重定时（TOPPRA）失败 |
| `gripper` | 夹爪动作下发 / 调度失败 |
| `verify` | 夹爪回读校验失败 |
| `state` | 任务状态机自身失败（重复推进 / 顺序错 / 求解器异常） |
| `done` | 成功（`reason = "completed"`）—— **不在失败词表内** |

### 3.2 原因词表（reason，按 stage 分组）

| stage | reason | 含义与排查方向 |
|---|---|---|
| `schema` | `unknown_primitive` | 原语名不在 10 个白名单内（先查拼写：`pre-grasp` 应写 `pre_grasp`） |
| `schema` | `missing_target` | 运动类原语缺 `target`（或缺 `target_object` 参照） |
| `schema` | `unknown_object` | `target_object` 未在 `objects` 中声明 |
| `schema` | `unsupported_kind` | `kind` 不在 `motion` / `gripper` / `bimanual_motion` 内 |
| `schema` | `missing_scene` | 场景文件缺失或装配失败（查 `--scenario` 与任务里的 `scenario` 字段） |
| `ik` | `unreachable` | 目标超出可达域（腕心半径不在 `d3−d5 ~ d3+d5` 内）⇒ 改目标位姿 |
| `ik` | `branch_exhausted` | 镜像分支与 ψ 网格内无解 ⇒ 改目标 / 放宽搜索 |
| `ik` | `limit_violation` | 有解但超软件限位 ⇒ 改目标或换分支 |
| `ik` | `wrist_singular` | 腕奇异（Z-X-Z 分解退化）⇒ 微调目标位姿 |
| `ik` | `invalid_target` | 目标非有限值 / 维度错 / 单位错（长度用 m、角度用 rad） |
| `path` | `interpolation_failed` | 关节插值不可行（步长 / 限位约束） |
| `path` | `tolerance_exceeded` | 末端误差超门槛 ⇒ 看 `message` 里的实测值与门槛（三级容差解析） |
| `collision` | `escape` | 碰撞复检发现逃逸（**硬失败**，计入零逃逸率） |
| `collision` | `recheck_failed` | 碰撞复检未通过（保守场景或几何不符） |
| `retime` | `parameterization` | 路径参数化失败 |
| `retime` | `continuity` | 连续性不可行 |
| `retime` | `velocity` / `acceleration` | 速度 / 加速度上限不可行 |
| `retime` | `duration` / `dt` | 时长或采样步长不可行 |
| `retime` | `dependency` / `path` / `shape` / `finite` / `limits` | 重定时包装层的其它标签（与 `rm75_drake.planning.retime` 同名） |
| `gripper` | `transport_error` | 夹爪指令下发失败（传输层；`verify` 的**回读**失败也复用该 reason） |
| `gripper` | `unsupported_action` | 夹爪动作不在 `close` / `open` 内 |
| `verify` | `gripper_offline` | 回读到的夹爪状态为离线（物体掉落 / 反馈丢失）⇒ 失败 |
| `verify` | `gripper_not_closed` | 回读开口与期望值偏差超 `verify_position_tolerance` |
| `verify` | `no_transport` | 没有传入 transport ⇒ 无法回读（离线默认）⇒ **失败且未验证** |
| `state` | `duplicate_goal` | 重复推进已推进过的 `goal_id` |
| `state` | `unknown_goal` | `goal_id` 不属于本任务 |
| `state` | `goal_out_of_order` | `goal_id` 不是下一个待推进的目标（顺序错） |
| `state` | `no_pending_goal` | 没有待推进的目标 |
| `state` | `terminal_state` | 终态下继续推进 |
| `state` | `solver_error` | 求解器抛异常 / 返回结果不自洽 |
| `state` | `aborted` / `wait` / `resumed` / `in_progress` | 显式中止 / 等待外部事件 / 恢复 / 推进中（审计轨迹用） |

> **`ik` 段的 reason 直接透出解析 IK 的词表**（`unreachable` / `branch_exhausted`
> / `limit_violation` / `wrist_singular` / `invalid_target`），
> **不另立第二套同义词**。

### 3.3 排查顺序总则

`stage` → `reason` → `message`（含实测值）→ 任务文件 `tolerance` / `thresholds`
→ 场景真相源（`scenarios/*.yaml`）→ 感知膨胀裕量（若走感知路径）。

## 4. 感知接入的生效边界（S06-T08）

感知以**非阻塞离散事件**接入（`rm75_drake.tasks.perception`）：

* **事件总线**：`PerceptionBus.subscribe(callback)` 注册回调，
  `publish(observation)` **立即返回**——观测进入**有界队列**
  （默认容量 64，队列满时**丢弃最旧**并计数 `dropped`，**不阻塞、不抛异常**）；
  订阅回调自身的异常被**吞掉并计入** `callback_errors`（一个坏订阅者不许
  拖垮发布方）；`poll(now_s)` 返回**最新快照**（含 `stale` 标志，
  默认过期阈值 0.5 s）。
* **保守化**：包围盒按 **`base_margin` + `latency_margin_rate × 观测年龄`**
  膨胀（上限 `max_margin`），即
  `effective_margin = min(max_margin, margin + latency_margin_rate × age)`；
  **延迟越大裕量越大** ⇒ 感知延迟被**保守场景表示**吸收。
* **落地步骤（顺序不可改）**：
  1. `RegisterAnchoredGeometry(bundle.source_id, GeometryInstance(...))`；
  2. `AssignRole(bundle.source_id, geometry_id, ProximityProperties())`
     ——**必须显式赋角色**；
  3. **重建碰撞检查器**并回填 `bundle.checker`；
  4. 返回 `PerceptionSceneResult`（含 `effective_margins` / `stale_observation_ids`
     / `dropped_observations`）。

### 4.1 生效边界（三条硬边界）

1. **只在「规划边界」生效**：`apply_perception_scene` 在**下一段规划之前**调用；
   执行环阻塞、点位在构造期固定 ⇒ **不支持在线重定向已经下发的轨迹**。
2. **必须重建 checker**：不重建碰撞检查器 ⇒ 新注册的几何**对碰撞检查不可见**
   ⇒ 保守场景**等于失效**。
3. **旧 checker 看不见新几何**（**本项目实测过的坑**）：注册完几何就以为
   “场景里有障碍了”是错的——Drake 的 `SceneGraph` 注册与碰撞检查器的
   构造是**两件事**，**必须重建 checker 才生效**。
   同样地，**不显式 `AssignRole(ProximityProperties)`** ⇒ 碰撞检查
   **完全看不见**这个几何（不是“看不见一点”，是**整个几何隐身**）。

### 4.2 幂等与丢弃

* 同一 `observation_id` 重复到达 ⇒ **不重复注册几何**（登记幂等），
  计入 `dropped_observations`。
* 观测年龄超过 `stale_after_s` ⇒ 快照标 `stale=True`，并在
  `stale_observation_ids` 中点名（**不静默丢弃**）。

## 5. **未验证**物理指标清单与解锁条件（S06-T08）

> 本仓库**无真机、无实测数据**。下列指标**一律未验证**，由
> `rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS` **逐条列出**
> （硬件门控 SKIP，**退出码仍为 0**），**不得用合成数据冒充**。

| # | 未验证指标 | 离线现状 | 解锁条件 |
|---|---|---|---|
| 1 | 真实（真机）抓放成功率 **≥ 95%**（10×10 目标网格） | 离线仿真逐原语成功率（`task_stats.py --samples`） | 真机在 10×10 网格上完成抓放并记录成功率 |
| 2 | 真实（真机）夹持误差（夹爪夹持后物体实际位姿偏差） | **未验证**（离线无夹持概念） | 真机夹持后测量物体位姿偏差 ≤ 1 mm |
| 3 | 真实（真机）交接保持漂移 | 仅**仿真内**构造性保持（交接点相对位姿残差 ≤ 1e-6） | 真机交接保持 10 s 且漂移 ≤ 2 mm |
| 4 | 真实（真机）感知延迟（检测到物体 → 指令下发的端到端延迟） | 离线只验证**裕量随延迟单调变大** | 真机时间戳实测 P95 ≤ 100 ms |
| 5 | 真实（真机）夹爪力控 / 夹空检测 | 离线只验证**回读指令量比对**逻辑 | 真机标定力曲线并统计空夹误判率 |
| 6 | 真实（真机）长稳节拍（连续运行的成功率与节拍） | **未验证** | 真机连续运行 ≥ 2 h 且任务成功率 ≥ 95% |

**解锁流程**：真机到位 ⇒ 走 S03 的设备通道与 S04 的实测采样作业 ⇒ 复算
上述 1/2/4 项 ⇒ 把对应条目从 `UNVERIFIED_TASK_METRICS` 移除
（**唯一真相源**，CLI 与脚本只再导出）并**保留复算命令**。

## 6. 运维（S06-T08）

### 6.1 常见故障 → 定位 → 处置

| 症状（看到的现象） | 定位（查什么） | 处置 |
|---|---|---|
| `task` CLI 退出码 **2** | stderr 的用法错误信息（任务文件不存在 / schema 非法 / 参数非法） | 按提示改参数；schema 违规信息**点名**了违规键，直接改该键 |
| 某原语失败且 `stage = "schema"` | `message` 里的违规键名 | 原语名拼写（`pre_grasp` 不写 `pre-grasp`）；`target_object` 必须在 `objects` 中声明；`kind = "gripper"` 必须同时有 `action` 与 `target_object` |
| 某原语失败且 `stage = "ik"` | `reason`（`unreachable` / `branch_exhausted` / `limit_violation` / `wrist_singular` / `invalid_target`）与 `message` | 目标位姿超出可达包络 ⇒ 往腕心半径 `d3−d5 ~ d3+d5` 内挪；单位错（mm 写成 m / 度写成 rad）⇒ 换算后重填 |
| 某原语失败且 `stage = "path"`、`reason = "tolerance_exceeded"` | `message` 里的**实测误差**与**门槛** | 先查容差三级解析（原语 `tolerance` → 任务 `thresholds` → 默认）；确认目标位姿不是故意留了偏差 |
| 某原语失败且 `stage = "collision"`、`reason = "escape"` | 场景真相源 + 感知裕量 | 逃逸是**硬失败**；检查感知观测的 `effective_margins` 是否过大（延迟高 ⇒ 裕量大），必要时调 `latency_margin_rate` / `max_margin` |
| 感知更新后**碰撞检查仍看不见障碍** | 是否**重建 checker**、是否 `AssignRole(ProximityProperties)` | 调用 `apply_perception_scene`（内部会重建 checker 并回填 `bundle.checker`）；**不要**绕过它手写注册 |
| `verify` 原语 `reason = "no_transport"` / `gripper_offline` | 是否注入了夹爪下发 / 回读通道 | 离线这是**设计如此**（`no_transport` 直接失败、`gripper_offline` 表示回读离线；都**不伪装成功**）；接真机通道后复算 |
| 状态机报 `goal_out_of_order` / `duplicate_goal` | `goal_id` 与原语顺序 | `goal_id` 在任务内唯一，且推进顺序必须与原语顺序一致 |
| 双臂原语报 `constraint_violation` | 交接点相对位姿残差 vs 1e-6 阈值 | 查 `X_LR` 是否与 `calibration/dual_arm.yaml` 一致（唯一真相源）；必要时重跑标定 |
| CI fast 档 `tests/tasks` 失败 | `bash scripts/ci.sh fast` 输出里该项的日志回显 | 逐个跑该测试文件定位；**不得**把该项降级成 `run_check_skippable`（零硬件即可回归，失败即整体失败） |
| `task_stats.py` 退出码 **1** | `threshold_failures`（哪个原语、哪条门槛） | 看是成功率不达标还是误差 / 零逃逸不达标；**不要**直接放宽门槛掩盖问题 |

### 6.2 离线复现命令

> **门禁执行纪律（S06-T08 实测踩坑）**：门禁对**单条命令**有超时上限，
> 默认 **120 s**。本仓的验证命令**必须**写成**独立多行**（每行一条命令），
> **不要**用 `&&` 把它们串成一条 —— 串成一条后整链耗时（fast 档 46 s +
> 门禁自检 229 s + 全量回归 277 s ≈ 550 s）会**超过 120 s**，
> 门禁报 **exit code 124（超时）**，而**不是**测试失败，容易被误判为
> 代码问题。若确实需要更长预算，调 `verification_timeout_ms`
> （GSD 偏好，`~/.gsd/PREFERENCES.md`），而不是把命令串成一条。

```bash
# 1) 常驻门禁（fast 档；含新增的 tests/tasks 常驻项）
bash scripts/ci.sh fast

# 2) 只跑 S06 任务套件
PYTHONPATH=src python3 -m pytest -q   tests/test_tasks_schema.py tests/test_tasks_primitives.py   tests/test_tasks_state_machine.py tests/test_tasks_bimanual.py   tests/test_tasks_perception.py tests/test_tasks_metrics.py   tests/test_tasks_cli.py

# 3) 单条任务链（逐原语结果 + 逐原语指标）
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/pick_place.yaml
#    双臂任务（handover）⇒ 退出码 2（task 仅支持单臂，见 §7）
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/handover.yaml --json

# 4) 指标脚本（验收口径 / 端到端口径分行 + 未验证清单）
python3 scripts/task_stats.py --samples 50
python3 scripts/task_stats.py --samples 50 --json

# 5) 全量非慢回归 + 禁用接口扫描
PYTHONPATH=src python3 -m pytest -q -m "not slow"
PYTHONPATH=src python3 scripts/scan_forbidden_apis.py src tools scripts
```

### 6.3 回滚与降级路径

* **回滚（代码层）**：本切片**只新增** `src/rm75_drake/tasks/` 包、
  `scripts/task_stats.py`、`tasks/*.yaml`、`runbooks/s06_task_primitives.md`，
  以及 `scripts/ci.sh` / `tests/test_ci_script.py` 的**增量**（新增常驻项
  与其断言）⇒ 回滚 = 撤掉新增的 `tests/tasks` 常驻项 + 对应断言，
  **其余切片（S01–S05）的门禁不受影响**。
* **降级（运行层，**禁止静默降级**）**：
  * **场景不可用**（Drake / 场景文件缺失）⇒ 任务链**显式失败**
    （`stage = "schema"`、`reason = "missing_scene"`），**不得**返回
    “看起来成功”的空轨迹；
  * **夹爪通道不可用** ⇒ `verify` 记 `reason = "no_transport"` /
    `gripper_offline`，**计入未验证**，**退出码仍为 0**
    （K001：既不伪装成功，也不伪装失败）；
  * **真机物理指标** ⇒ 硬件门控 SKIP，**逐条**列入
    `unverified_metrics`，**退出码仍为 0**；
  * **感知观测过期** ⇒ 快照标 `stale=True` 并点名
    `stale_observation_ids`，**不静默丢弃**，但**也不**因此在线改写
    已下发的轨迹（不支持在线重定向）。
* **门禁纪律**：`tests/tasks` 在 fast 档是**常驻项**（`run_check`）——
  **失败即整体失败**，**不得**降级成 `run_check_skippable`；
  该纪律由 `tests/test_ci_script.py` 自身断言守住
  （存在 `run_check "tests/tasks"`、**不存在** `run_check_skippable "tests/tasks"`、
  且覆盖 7 个 S06 测试文件）。

## 7. CLI 契约边界（S09-T05）

> **一句话**：`task` 子命令**仅支持单臂场景**；多臂场景**显式拒绝**，
> 退出码 **2** + 稳定 reason 标签 **`task_single_arm_only`**（**不是** SceneError 栈）。

### 7.1 能力边界（为何存在）

`task` 的场景装配固定走**单臂** `build_scene`（7 关节）。场景真相源里
`robots` 数量 **> 1**（如 `scenarios/dual_arm.yaml` 的 14 关节双臂场景）时，
装配层无法消费多臂 plant——旧行为是抛出
`SceneError: 关节顺序与 scenario 不一致` 的**栈**（信息不达意、退出码 1 像业务失败）。
S09 选择**契约收口**路线（**不**扩 CLI 双臂装配，避免扩散 blast radius）：
把限制**显式契约化**。

| 场景 | `task` 行为 | 退出码 | 稳定 reason |
|---|---|---|---|
| 单臂（`scenarios/single_arm.yaml`，`robots` = 1） | 正常跑完整条原语链 | **0** | `completed` |
| 单臂但业务失败 / 超门槛 | 逐原语结果 + 门槛失败明细 | **1** | 原语/指标自身 reason（如 `threshold_not_met`） |
| **多臂**（`scenarios/dual_arm.yaml`，`robots` = 2） | **显式拒绝**（前置校验，不进入装配） | **2** | **`task_single_arm_only`** |

### 7.2 退出码语义（0 / 1 / 2）

* **0**：成功（原语全部成功**且**逐原语门槛达标；真机物理指标硬件门控 SKIP，
  逐条列入 `unverified_metrics`，**不影响**退出码 0）；
* **1**：**业务失败**（原语失败 / 碰撞逃逸 / 超门槛）；
* **2**：**用法错误或命令未支持**——任务文件不存在 / schema 非法 / 参数非法，
  **或**命中「仅支持单臂」能力边界（`reason = task_single_arm_only`）。

### 7.3 双臂交接的现状与替代路径

* **现状**：`tasks/handover.yaml` 是**部署实例数据**（语义：单臂接近 → 抓取 →
  抬起 → 双臂共同搬运 → 交接 → 释放 → 撤离），其原语含
  `bimanual_motion`（`transit` / `handover`）。**双臂原语编排只在库层可用**
  （`rm75_drake.tasks.bimanual.plan_carry` / `plan_handover`，常数 `X_LR`），
  **不可经 `task` CLI 复现**——这是**已知缺口**，不得假装已覆盖。
* **替代路径**：双臂协调走
  * `cli bimanual`（离线双臂协同求解 + 残差报告，退出码 0）；
  * `rm75_drake.tasks.bimanual` 的 `plan_carry` / `plan_handover` 编排层；
  * 场景真相源 `scenarios/dual_arm.yaml`（两臂 URDF + 基座外参）+
    `calibration/dual_arm.yaml`（基座外参唯一真相源）。
* **不做的承诺**：**不**承诺「`task` CLI 能跑双臂」；若未来扩展，
  需同时给出 reason 标签与退出版本的迁移说明。

### 7.4 复算命令

```bash
# 双臂任务 ⇒ 退出码 2 + reason 标签（不是 SceneError 栈）
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/handover.yaml; echo "exit=$?"
# 单臂任务 ⇒ 退出码 0（行为不变）
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/pick_place.yaml; echo "exit=$?"
# 契约回归（守护用例）
python3 -m pytest -q tests/test_tasks_cli.py
```

> **需求口径**：R011 的 Proof 措辞（目标文本）见
> `docs/evidence/REQUIREMENTS-PROOF.md`（S09-T06 产出）——该处**必须**如实反映
> 本节的「仅支持单臂 + 退出码 2 + 双臂交接不可经 CLI 复现」缺口。
