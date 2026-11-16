# rm_planner 规划服务说明（独立进程 · 硬超时 · 确定性）

> 面向"第一次接这条规划链路的人"：照着本文，你能在 10 分钟内看懂
> **rm_planner 是什么进程、请求/响应长什么样、超时是怎么被强制中断的、
> 为什么同种子结果一定一致、失败了下一步该做什么**，并知道
> **哪些结论已实证、哪些还没有**。
>
> 关联切片：M001-3kzz40 / S07（自研规划服务 rm_planner）；
> 关联代码：`dualarm_ws/src/rmctl/rmctl/planner_service/`
> （`contract.py` 契约 / `config.py` 配置 / `engine.py` 引擎 /
> `worker.py` worker 子进程 / `server.py` 服务端 / `client.py` 薄客户端）；
> 关联决策：D027（代码落点）、**D028（对 D003 的落地偏差）**、D001（声明式配置）、
> D004（三进程模型）；关联需求：R002（四道校验）、R010（上手体验与操作效率）；
> 关联文档：`docs/rm_process单臂工艺闭环说明.md`（工艺编排层）、
> `docs/rm_executor离线证据.md`（执行层离线证据）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 / 文件 |
|---|---|
| 起一个规划服务 | `python3 -m rmctl.planner_service.server --ws <仓库根>` |
| 走服务规划一条轨迹 | `python3 -m rmctl plan --via-service --start 0,0,0,0,0,0,0 --goal 0.2,-0.3,0.1,0,0,0,0` |
| 看服务状态（在飞 worker / 计数器） | `echo '{"cmd":"status"}' \| nc -U <ws>/build/run/rm_planner.sock`（或见 §10.2） |
| 取消一条在飞的规划 | `{"cmd": "cancel", "plan_id": "<plan_id>"}` |
| 跑三条验收（一条命令） | `python3 scripts/rm_planner_acceptance.py` |
| 看三条验收报告 | `build/reports/s07_planner_acceptance.json` |
| 看黄金场景集（S08 性能基线复用它） | `dualarm_ws/src/rmctl/tests/golden/planner_scenarios.json` |
| 规划配置（唯一来源，只读） | `workcell/planner/pipeline.yaml` + `workcell/planner/profiles.yaml` |

> **诚实分层（贯穿全文）**：本轮证据为**离线层** —— 真实规划内核
> （厂商 tesseract 0.35.0.8）+ 真实四道校验 + 真实进程级 `SIGKILL`
> + `rm_executor` **fake** 运动回放。**真机版崩溃隔离未实证**（属真机窗口），
> 口径见 **§9**。

---

## 1. 进程模型与定位

### 1.1 rm_planner 是什么

`rm_planner` 是**独立进程**（进程角色名 `rm_planner`，入口
`python3 -m rmctl.planner_service.server`，别名命令 `rm-planner`），
实现 D004 的三进程模型之一：

```
rmctl（薄客户端）──Unix socket──▶ rm_planner（服务端进程，可崩溃可重启）
                                        │ spawn（每请求一个）
                                        ▼
                                    worker 子进程（独占）
                                        │ 一行 JSON 请求 / 一行 JSON 响应
                                        ▼
                                    厂商 Task Composer（不可中断）
```

* **rm_planner 可以随时被杀**：它不持有运动执行的任何句柄、不在 10 ms
  栅格里、不参与安全判据 → 服务进程死亡**不影响**正在进行的运动执行
  （离线证据见 §9，验收脚本 §10.4）。
* **每条 plan 请求 spawn 一个独占 worker 子进程**，用完即
  `SIGKILL` + `waitpid` 回收（不留僵尸，`status` 里可查
  `workers_spawned` / `workers_reaped`）。
* **自动复活属部署层**（systemd / supervisor 的 `Restart=always`），
  本切片**不承诺**；只承诺"服务死了不影响别的进程，且可重启继续受理"。

### 1.2 代码落在哪里（为什么不新增第五层）

代码落在 **`rmctl/planner_service/`**（`rmctl` 包内的子包），
**不新增第五层**（`rm_hw_bridge/contract.yaml::rules.layers` 仍是
hardware / safety / executor / experience 四层不变）—— 这是 **D027** 的决定，
与 S06 的 **D023**（工艺层落 `rmctl/process/`，同样不新增层）同一先例。

理由：

1. 四道校验与保路径降速自愈的**唯一实现**已经在 `rmctl.verify`
   （`verify_with_retime`），规划真值也已在 `rmctl` 内；另起一个包
   要么 `import rmctl`（体验层 → 反向依赖），要么**复制一份校验** →
   造出**第二处真值**（本项目最忌讳的事）。
2. 进程隔离与"可重启"由**进程角色 + worker 子进程**满足，与代码放在哪个
   包无关；层表与审计红线（`rm_hw_bridge.audit`）零改动。
3. 不改既有 `rmctl.cli` / `rmctl.process.clients` 的 import → 不动
   S03–S06 已验证的成果。

### 1.3 依赖方向

```
rmctl.plan / rmctl.process  →  rmctl.planner_service  →  rmctl.verify / rmctl.retime
                                                            rmctl.model（厂商内核）
```

* `planner_service/client.py`（薄客户端）**零 SDK**：不 import 厂商规划内核，
  只发/收一行 JSON → 客户端进程不需要加载模型就能校验请求（契约层是纯
  Python 校验）。
* `planner_service/server.py` 依赖 `rmctl.paths` / `rmctl.vendor_env`；
  worker 子进程会自己 `import rmctl.model`（厂商内核只在 worker 里加载）。

---

## 2. 契约（PlanRequest / PlanResponse）

契约层的**唯一实现**是 `rmctl/planner_service/contract.py`；引擎 / 服务端 /
客户端 / 验收四层共用这一份，不再各长一份隐式真相。

### 2.1 PlanRequest（请求，逐字段）

| 字段 | 类型 | 单位 | 必填 | 语义 / 校验规则 |
|---|---|---|---|---|
| `plan_id` | str | — | 是 | 请求标识（幂等 / 关联响应）；非空字符串 |
| `profile` | str | — | 是 | 规划 profile 名（`FREESPACE` / `CARTESIAN` / `DUAL_RELATIVE`）；本层只校验"非空字符串"，**不猜语义** |
| `group` | str | — | 是 | 规划组（`left_arm` / `right_arm` / `dual_arm`） |
| `start_state` | list[float] × 7 | rad | 是 | 起始关节角；必须**恰好 7 个有限数**（`NaN`/`Inf` 拒绝） |
| `goal` | mapping | — | 是 | 目标：`{"type": "joint", "values": [7 个 rad]}` 或 `{"type": "pose", "values": [x, y, z, roll, pitch, yaw]}`；未知类型 → 非法；只允许 `type` / `values` 两个键 |
| `constraints` | mapping | — | 是 | 约束白名单：`time_scaling` / `collision_margin` / `velocity_scaling` / `acceleration_scaling` / `jerk_scaling`；未知键 → 非法；`time_scaling` 必须 > 0 且 ≤ `profiles.yaml::max_time_scaling` |
| `seed` | int | — | 是 | 可复现随机种子（≥ 0） |
| `timeout_ms` | int | ms | 是 | **服务端强制中断**的硬超时（> 0）；见 §4 |
| `scene_version` | int | — | 是 | 世界模型版本号（须与 `profiles.yaml::scene_version` 一致） |
| `scene` | mapping \| null | — | 否 | 可选场景：`{"obstacles": [{"name", "box_size", "position"}]}`（盒子障碍物，`box_size` 三轴必须为正数） |
| `deterministic` | bool | — | 否（缺省 `True`） | 确定性开关；`True` 时强制 `num_planners=1` |

纪律：

* **逐字段指名**：字段缺失 / 类型错 / 非有限数 / 未知 goal 类型 →
  `InvalidRequest`（带 `field` 与 `reason`），**不静默忽略、不补默认值**。
* **有限数纪律**：`NaN` / `Inf` 一律拒绝（请求与响应皆然），
  `to_dict()` 的输出永远可 JSON 序列化。
* **客户端只补两样**：`plan_id`（缺省生成）与 `timeout_ms`（缺省取
  `profiles.yaml`）；**其余字段缺失即 `INVALID_REQUEST`** ——
  杜绝"客户端悄悄补业务缺省"造成的两份真相。

### 2.2 PlanResponse（响应，逐字段）

| 字段 | 类型 | 语义 |
|---|---|---|
| `plan_id` | str | 与请求一致（非法请求时可能为空） |
| `status` | str | 五选一：`SUCCESS` / `FAILED` / `TIMEOUT` / `CANCELLED` / `INVALID_REQUEST` |
| `trajectory` | mapping \| null | 轨迹：`{"times": [...], "points": [{"t":…, "positions":[…]}]}`；`INVALID_REQUEST` / 超时 / 失败时为 `null` |
| `verification` | mapping | 四道校验结论：`limits` / `discrete_collision` / `continuous_collision` 各自 `ok` + `issues`/`hits`，顶层 `ok` + `issues` |
| `failure_reason` | str | 可解释的失败原因（哪一节点失败 / 哪一约束不满足 / 被中断阶段） |
| `metrics` | mapping | 诊断面：per-node 耗时、种子、规划器个数、总耗时、复检 / 重定时缩放、`next_action`（见 §10.3） |

### 2.3 五种 status 的语义

| status | 语义 | 典型 `failure_reason` |
|---|---|---|
| `SUCCESS` | **四道校验全过**（不是"规划器说 OK"）；`trajectory` 非空 | — |
| `FAILED` | 规划本身失败 / worker 崩溃 / 四道校验未通过 | `规划失败: OMPLMotionPlannerTask: Failed to find valid solution: …`、`四道校验未通过: …`、`worker 异常退出（退出码 -9）…` |
| `TIMEOUT` | **服务端**在 deadline 处 `SIGKILL` worker（硬中断） | `规划超时：服务端在 deadline 1200 ms 处强制中断 worker（被中断阶段：plan）` |
| `CANCELLED` | 客户端 `cancel` 触发服务端 `SIGKILL` worker | `规划被取消：服务端已 SIGKILL worker（阶段 plan）` |
| `INVALID_REQUEST` | 请求非法（逐字段指名） | `请求非法（start_state）: 必须恰好 7 个有限数，当前是 3 个` |

> **TIMEOUT 与 FAILED 必须分得清**：`TIMEOUT` = 服务端到点收手（可复现、
> 与 deadline 无关地稳定）；`FAILED` 里若原因是"规划失败: … Exceeded allowed
> time"，那是**规划器自己**耗尽了 `planning_time_s` 放弃 —— 两者在
> §10.4 的验收里用**同一障碍物场景、不同 timeout_ms** 做面对照。

### 2.4 INVALID_REQUEST 的逐字段指名规则

* **构造方向**（`PlanRequest.from_dict`）失败 → 抛 `InvalidRequest`，
  带 `field`（如 `start_state` / `goal.type` / `constraints.time_scaling`）
  与 `reason`；服务端把它转成 `status="INVALID_REQUEST"` 响应，
  `failure_reason` 写明 `请求非法（<field>）: <reason>`，
  `metrics.field` 也带上字段名。
* **解析方向**（`parse_response_line`）失败 → **不抛裸异常**，返回
  `status="INVALID_REQUEST"` 的响应对象（客户端据此走错误路径，不崩）。
* 未知字段**不静默忽略**：请求里出现契约外的键 → 直接指名该键非法。

---

## 3. 协议

### 3.1 传输

* **Unix domain socket**，默认路径 `<ws>/build/run/rm_planner.sock`
  （`rmctl.paths.workspace_root()` 为 ws 根）；可用 `--socket` 覆盖
  （路径长度受 `sun_path` 上限约束：≤ 107 字节）。
* **一行 JSON 请求 → 一行 JSON 响应**（`\n` 分隔），与
  `rm_safety.gate` 同风格；请求体上限 256 KiB（超长 → `INVALID_REQUEST`）。

### 3.2 cmd 集合

| cmd | 请求 | 响应 |
|---|---|---|
| `plan` | `{"cmd": "plan", "request": {…契约字段…}}` | 契约层 `PlanResponse`（`status` / `trajectory` / `verification` / `failure_reason` / `metrics`） |
| `cancel` | `{"cmd": "cancel", "plan_id": "…"}` | `{"status": "ok", "plan_id":…, "worker_pid":…, "reason":…}`；找不到在飞 plan → `{"status": "error", "reason":…, "next_action":…}` |
| `status` | `{"cmd": "status"}` | `{"running", "socket_path", "ws", "worker_pool_size", "active_workers", "counters", "errors"}` |

* 非法 JSON / 未知 cmd / 缺字段 → 回一行 `status="INVALID_REQUEST"` 的响应
  （带 `reason` + 可粘贴 `next_action`），**服务进程继续存活**。
* `plan_id` 重复（在飞）或在飞 worker 数达上限（`worker_pool_size`，默认 1）
  → `INVALID_REQUEST` 并说明"换一个 plan_id / 稍后重试 / 先 cancel"。

### 3.3 错误码与可粘贴 next_action

失败路径一律**结构化**，不抛裸异常：

| 场景 | `status` | `failure_reason`（摘） | `metrics.next_action`（可粘贴） |
|---|---|---|---|
| 请求字段非法 | `INVALID_REQUEST` | `请求非法（<field>）: <reason>` | 按契约修正字段后重试（原因里指名了字段） |
| 服务端硬超时 | `TIMEOUT` | `规划超时：服务端在 deadline N ms 处强制中断 worker（被中断阶段：plan）` | 放宽 `timeout_ms`，或改用更容易的场景 / 更短的规划目标 |
| 被取消 | `CANCELLED` | `规划被取消：服务端已 SIGKILL worker（阶段 plan）` | 如需继续，请重新发起 plan 请求 |
| worker 崩溃 | `FAILED` | `worker 异常退出（退出码 -9）: <stderr 尾部>` | （见 §10.5） |
| 连不上 socket | `FAILED`（客户端造） | `连不上规划服务 socket …` | 先启动规划服务：`rm-planner --ws <ws> --socket <path>` |
| 服务端读超时 | `TIMEOUT`（客户端造） | `等待规划服务响应超时（…）` | 放宽 `timeout_ms`，或用 `{"cmd":"status"}` 查看服务是否还在跑 |
| 连接中断 / 响应非法 | `FAILED`（客户端造） | `与规划服务的连接中断 …` / `响应不是合法 JSON …` | 查看服务日志；服务进程可能已崩溃，重启 `rm-planner --ws <ws>` |

---

## 4. 硬超时语义（为什么必须进程级）

### 4.1 两个 deadline（都要诚实）

| deadline | 默认值 | 起算点 | 超时后果 |
|---|---|---|---|
| **启动宽限** `startup_timeout_s` | 30 s | worker spawn 时刻 | `TIMEOUT`（阶段 `startup`）—— 模型都没加载出来 |
| **规划 deadline** | 请求的 `timeout_ms` | worker 发出 `ready` 心跳（= 规划**真正开始**） | `SIGKILL` worker → `TIMEOUT`（阶段 = worker 最后一条心跳：`plan` / `verify` / `done`） |

> 规划 deadline **不含** worker 启动开销：模型加载时间不计入业务超时，
> 否则大模型会把短 deadline 场景全部变成"启动超时"。

### 4.2 为什么不能靠进程内 cancel

厂商 `TaskComposerFuture` **只有** `ready` / `valid` / `wait` / `waitFor` /
`context`：

* **没有 `cancel`** —— 没有任何"请求中止"的入口；
* `waitFor(float)` 的绑定本身抛 `TypeError` —— 连"带超时地等"都用不了；
* 厂商的 taskflow 一旦开跑，**Python 侧没有任何句柄**能打断它
  （线程不可从 Python 中断）。

因此"服务端**强制**硬超时"只有一条路：**deadline 到点 → `SIGKILL`
worker 子进程 → `waitpid` 回收**。同一机制顺带满足：

* **可中断**：`cancel` cmd = 对该 plan 的 worker 发 `SIGKILL`；
* **崩溃可替换**：worker 死，服务不死，下一条请求照常。

### 4.3 收尾纪律

* worker 响应后，服务端先 `close_stdin()`（发 `{"cmd":"quit"}` 再关）
  再 `reap()` —— 否则 `waitpid` 会干等满 5 s（历史上每条 plan 白等 5 s）。
* 超时 / 取消路径：`kill()` → `reap()` → 计数 `timeout` / `cancelled` +
  `workers_reaped`。
* `status` 里 `active_workers` 为空 + `workers_spawned == workers_reaped`
  ⇒ 无残留 worker。

---

## 5. 确定性语义

### 5.1 种子必须在 worker 进程启动时、任何规划之前设定

worker 的启动顺序**不可颠倒**：

```
RNG_setSeed(seed)   ← 进程生存期内第一次设定，必须在任何规划之前
  → 加载 Model
  → 进入请求循环
```

原因（厂商内核的硬事实）：

* OMPL 的 RNG 是**全局量**，且**一旦开始采样就不能再改种子** ——
  厂商会打印
  `Random number generation already started. Changing seed now will not lead to deterministic sampling.`
* 因此"一个进程只服务一个种子" → **每请求一个独占 worker 子进程**。

### 5.2 确定性开关

* `request.deterministic = true`（缺省）→ 强制 `num_planners = 1`
  （并行多规划器的胜者取决于调度顺序，会破坏逐点一致）。
* `pipeline.yaml::num_planners` 只在 `deterministic = false` 时生效。
* `metrics` 里记录 `seed` / `num_planners` / `deterministic` —— 复现所需参数。

### 5.3 一致性判据

同 **(request, seed, scene_version)** → **逐点一致**，用
`rmctl.planner_service.contract.trajectory_digest()` 判定：

* 摘要 = sha256( 规范化(时间 + 点位) )，规范化文本用 `repr(float)` 拼接
  → **不依赖 dict 顺序**；
* `NaN` / `Inf` 直接报错（不产生误导性摘要）；
* 两次请求各自 spawn 独立 worker → 摘要相同即"逐点一致"
  （验收脚本 §10.4 段 ③）。

---

## 6. 四道校验语义与自愈阶梯

### 6.1 四道校验（唯一实现 `rmctl.verify`）

| 道 | 含义 | 报告键 |
|---|---|---|
| 1 | **限位 + 时间可实现性**（位置 / 速度 / 加速度 / jerk 超限） | `verification.limits`（`ok` + `issues` + `min_margin`） |
| 2 | **离散碰撞** | `verification.discrete_collision`（`ok` + `hits`） |
| 3 | **连续碰撞**（稠密采样，步长 ≤ 0.02 rad） | `verification.continuous_collision`（`ok` + `hits`） |
| 4 | 顶层总判定 | `verification.ok` + `verification.issues` |

* **`SUCCESS` 的语义 = 四道全过**（R002），不是"规划器说 OK"。
* 规划成功但校验不过 → `status = FAILED`，
  `failure_reason` 写明 `四道校验未通过: …`。
* 连续碰撞是否执行由 `profiles.yaml::verify.continuous` 决定。

### 6.2 自愈阶梯第 1 级：保路径降速重定时

* 唯一实现：`rmctl.verify.verify_with_retime()`（CLI 与工艺客户端层共用，
  **不复制阶梯逻辑**）。
* 语义：先跑四道校验 → 不过 → 算最小时间缩放系数 `s` → `s > 1.001` 时
  **几何路径不变、只放慢时间轴** → 重跑四道校验 → 过了才算成功。
* **上限封顶**：实际缩放系数 ≤ `min(profiles.yaml::max_time_scaling,
  request.constraints.time_scaling)`（当前 `max_time_scaling = 1.5`）。
  封顶后仍不过 → `FAILED`，并在 `metrics.notes` 给出人工提示
  （"人工放宽约束 / 换路径"）。
* 结果落在 `metrics.retimed_scale`（`null` = 没降速）。

### 6.3 per-node 指标

`metrics.nodes` 逐节点给出 `name` / `status` / `elapsed_s` / `message`
（官方管线节点）。**注意**：自研校验在管线图外（见 §8），因此
per-node 指标**不覆盖**四道校验节点 —— 校验耗时看
`metrics.elapsed_total_s` 与 `metrics.verify`。

---

## 7. 配置来源（D001：声明式、只读、可 diff）

| 文件 | 键 | 消费方 |
|---|---|---|
| `workcell/planner/pipeline.yaml` | `pipeline`（管线名）、`num_planners`、`planning_time_s`、`planner_range`、`deterministic` | 服务端启动时读取，构造官方 Task Composer 管线 |
| `workcell/planner/profiles.yaml` | `timeout_ms`（默认硬超时）、`max_time_scaling`（时间缩放上限）、`collision_margin_m`、`verify.continuous`、`scene_version` | 服务端（硬超时 / 校验参数 / 场景版本）+ 客户端（请求缺省值） |

* Python **只读不改**；改配置 = 改文件 + git diff（换机型 / 换工艺不改代码）。
* **`max_time_scaling` 只有一个来源**：`profiles.yaml`。请求里
  `constraints.time_scaling` 超过它 → 请求非法（逐字段指名）；
  降速自愈的上限也取自同一处 —— 不存在第二份限值常量。
* 配置不可得时**不猜数值**：契约层不检查上限（并由配置加载器另行给出
  blocked 证据），而不是硬编码一个默认上限。

---

## 8. 与 D003 的落地偏差（**D028**）

**D003**（原决定）：自研 Task 节点（连续碰撞 / 限位可实现性）**插入官方
YAML 管线**，官方 TaskComposer 作管线内核。

**D028**（实测后的修订，amends D003）：自研 Task 节点**未插入**官方 YAML
管线，改为**服务层 Python 后置校验**。

原因（实测 tesseract-robotics-nanobind **0.35.0.8** 的绑定面）：

* Python 绑定**没有注册自定义 TaskComposer 节点的 API** ——
  只有 `TaskComposerPluginFactory.createTaskComposerNode`，工厂**全部来自
  C++ 插件库**；官方插件清单里只有 `DiscreteContactCheckTaskFactory`，
  没有连续碰撞 / 限位校验工厂；
* 要做"管线内自研"，就必须写 **C++ 工厂库** → 违反**零编译主线（R013）**。

**语义不变**：校验永远比规划保守；`SUCCESS` = 四道全过；四道校验的唯一
实现仍是 `rmctl.verify`（不新增第二份真值）。

**代价（明示）**：校验在**管线图之外**——

* 管线图里看不到校验节点（per-node 指标不覆盖它）；
* 管线内短路（规划阶段就拒绝）拿不到；
* 连续碰撞改用**稠密采样**（≤ 0.02 rad 步长）替代 0.35.0.8 绑定里带段
  错误的 `cast`。

**解冻条件**：若未来需要管线内增量校验或真机性能要求管线级短路，可重新
评估写 C++ 工厂库（与 R013 零编译约束冲突，需先决策解冻）。

---

## 9. 离线-真机口径声明（诚实分层）

### 9.1 本轮**已实证**（离线层）

`python3 scripts/rm_planner_acceptance.py` 的三条验收（报告落
`build/reports/s07_planner_acceptance.json`）：

| 验收 | 证据链 | 结论 |
|---|---|---|
| ① 规划超时被**服务端**强制中断 | 真实规划 + 真实 worker 子进程 + 服务端 `SIGKILL`；同一障碍物场景两种预算做面对照 | 小 deadline → `TIMEOUT`（`metrics.killed=true`、`interrupted_stage=plan`、耗时 ≈ deadline）；给足预算 → `FAILED`（规划器自己放弃） |
| ② 服务崩溃不影响运动执行 | 真实规划出的轨迹 + `rm_executor` **fake** 回放（统一报告 schema）+ 回放中 `kill -9` 服务进程**及其全部 worker 子进程** | 回放报告 `status=ok` / `mode=fake` / 无 FATAL / 无中断类事件 / 轨迹完整回放（到位判定通过）；服务与 worker 确实全死 |
| ③ 同请求同种子结果逐点一致 | 两次请求 → 两次独占 worker → `trajectory_digest` 比对 | digest 相同，点位数相同 |

本轮证据 = **真实规划内核 + 真实四道校验 + 真实进程级 SIGKILL +
`rm_executor` fake 回放**。

### 9.2 本轮**未实证**（真机层）

* **真机版崩溃隔离**：在 `rm_runtime` 执行过程中真的 `kill -9 rm_planner`，
  观察真机运动是否继续 —— **本轮未实证**（属真机窗口）。
  离线证据证明的是"进程级隔离与 fake 回放互不干扰"，**不等于**真机结论。
* 真机版的服务崩溃恢复（systemd `Restart=always` 等部署层能力）也未实证。
* 真机 p95 性能由 **S08** 负责（复用
  `tests/golden/planner_scenarios.json` 与 `build/reports/s07_planner_golden.json`）。

### 9.3 真机回填入口（留待 S08/S09）

1. 真机窗口开 → 在真机执行中 `kill -9 rm_planner` → 观察运动是否继续 →
   记录到 `build/reports/s08_*`（口径与本节一致：先写"未实证"，回填后
   再改状态）。
2. 回填时**必须**同时记录：服务进程 pid、worker pid、真机运动是否继续、
   是否有安全事件。

---

## 10. 运维

### 10.1 怎么起服务

```bash
# 方式 1：模块入口（推荐，路径确定）
python3 -m rmctl.planner_service.server --ws /path/to/repo
#   → socket: /path/to/repo/build/run/rm_planner.sock

# 方式 2：短路径（socket 路径有 107 字节上限时）
python3 -m rmctl.planner_service.server --ws /path/to/repo --socket /tmp/rm_planner.sock

# 方式 3：客户端接线（默认进程内行为逐字不变，加 --via-service 才走服务）
python3 -m rmctl plan --via-service \
    --start 0,0,0,0,0,0,0 --goal 0.2,-0.3,0.1,0,0,0,0 --seed 0 --timeout-ms 20000
```

* 启动失败（路径过长 / 目录不可写 / 路径被占用）→ 退出码 **3** 并打印
  可粘贴 `next_action`（例如"用 `--socket /tmp/rm_planner.sock` 重启"）。
* 客户端连不上服务 → `status=FAILED` + `next_action`（"先启动规划服务：…"），
  **不抛裸异常**。

### 10.2 怎么看服务状态

```bash
python3 - <<'PY'
import socket, json
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect("/path/to/repo/build/run/rm_planner.sock")
s.sendall(b'{"cmd": "status"}\n')
print(s.recv(65536).decode())
PY
```

返回要点：

* `active_workers`：在飞 worker（`plan_id` / `pid` / `stage` / `elapsed_s`）；
* `counters`：`requests_total` / `success` / `failed` / `timeout` /
  `cancelled` / `invalid` / `workers_spawned` / `workers_reaped` /
  `worker_crashes`；
* `errors`：最近 10 条服务端异常（留痕，不静默）。

健康判据：`workers_spawned == workers_reaped` 且 `active_workers` 为空
⇒ 无残留 worker、无僵尸。

### 10.3 怎么看 metrics（排障第一入口）

`metrics` 常用键：

| 键 | 含义 |
|---|---|
| `nodes` | per-node 指标（`name` / `status` / `elapsed_s` / `message`） |
| `verify` | 四道校验总判定 |
| `retimed_scale` | 实际用到的降速缩放系数（`null` = 没降速） |
| `notes` | 降速封顶等**人工提示** |
| `elapsed_total_s` / `planning_s` | 端到端耗时 / 纯规划耗时 |
| `seed` / `num_planners` / `deterministic` | 复现所需参数 |
| `interrupted_stage` / `worker_pid` / `killed` | 超时 / 取消时被中断的阶段与被杀 worker |
| `timeout_ms` / `elapsed_s` | 死线与实际耗时 |
| `next_action` | **可粘贴**的下一步 |

### 10.4 一键跑三条验收

```bash
python3 scripts/rm_planner_acceptance.py
# → build/reports/s07_planner_acceptance.json
# 退出码 0 = 三条全过；4 = 有段落失败（报告写明哪条断言 + 下一步）

# 只跑其中一段（冒烟 / 复现单条问题时用）
python3 scripts/rm_planner_acceptance.py --only deterministic_same_seed
# 可选值: timeout_forced_interrupt / crash_isolation / deterministic_same_seed
```

报告结构（对齐 S06 的离线证据脚本）：顶层 `status` / `reason` /
`next_action` / `mode` / `evidence` / `failures`，三个段落
`timeout_forced_interrupt` / `crash_isolation` /
`deterministic_same_seed`，每段含 `status` / `passed` / `reason` /
`assertions`（逐条断言 + 观察值）/ `evidence` / `numbers`。

### 10.5 失败了下一步做什么

| 现象 | 先看 | 下一步 |
|---|---|---|
| `INVALID_REQUEST` | `failure_reason` 里的字段名 | 按契约修正该字段（原因里已指名） |
| `TIMEOUT` | `metrics.interrupted_stage` / `metrics.elapsed_s` | 阶段 `startup` → 模型加载慢（查 `build/model` / `LD_LIBRARY_PATH`）；阶段 `plan` → 放宽 `timeout_ms` 或换更简单的目标 |
| `FAILED` + `规划失败: …` | `failure_reason` | 规划器自己放弃（不是超时）→ 放宽 `planning_time_s`（`pipeline.yaml`）或换目标 |
| `FAILED` + `四道校验未通过` | `verification.*.issues` | 看是哪一道：`limits` → 放宽 `constraints.time_scaling`（但受 `max_time_scaling` 封顶，封顶后需人工放宽配置 / 换路径）；碰撞 → 改场景或改目标 |
| 客户端 `连不上规划服务` | 服务进程是否在跑 | 起服务；`status` 看 `running` |
| 客户端 `等待规划服务响应超时` | 服务端 `status.active_workers` | 服务可能在规划中；或 worker 卡死 → 重启服务（worker 会被 `SIGKILL` 回收） |
| `worker 异常退出` | `metrics` / 服务端 stderr 尾部 | 多为环境问题（模型 / `LD_LIBRARY_PATH`）；重启服务后重试 |

---

## 11. 已知限制

1. **真机版崩溃隔离未实证**（见 §9.2）。
2. **校验在管线图外**（D028 的代价，见 §8）。
3. **并发上限默认 1**（`worker_pool_size`）：同一时刻只允许一条 plan 在飞，
   超出即 `INVALID_REQUEST`（"在飞 worker 数已达上限"）—— 保证机器不被
   打爆，也保证确定性（每请求独占 worker）。
4. **笛卡尔路径 / 相对约束**尚未实现（本层只冻结 `joint` 与 `pose` 两种
   goal 类型）。
5. **部署层自动复活**不属本切片承诺（systemd / supervisor）。

---

## 12. 关联索引

* 需求：R002（四道校验）、R010（上手体验与操作效率）、R013（零编译主线）。
* 决策：D001（声明式配置）、D003（+ **D028** 修订）、D004（三进程模型）、
  D023 / D027（不新增层）。
* 切片：M001-3kzz40 / **S07**（T1 契约与配置 → T2 引擎 → T3 服务端与 worker
  → T4 薄客户端与 CLI 接线 → T5 黄金场景集与崩溃隔离离线证据 → T6 本文与一键验收）。
* 黄金场景集：`dualarm_ws/src/rmctl/tests/golden/planner_scenarios.json`
  （7 场景，版本化、git 跟踪；**S08 性能 p95 直接复用**，不另造基准）。
* 验收脚本：`scripts/rm_planner_acceptance.py`
  → `build/reports/s07_planner_acceptance.json`。
