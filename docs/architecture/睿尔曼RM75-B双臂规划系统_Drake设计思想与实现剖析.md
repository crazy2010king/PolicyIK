# 睿尔曼 RM75-B 双臂规划系统：Drake 设计思想与实现逐层剖析

> 文档性质：对本仓（PolicyIK_drake / rm75_drake）已交付成果的**技术与思想剖析报告**。以 Drake（MIT CSAIL Robot Locomotion Group / Toyota Research Institute 主导的开源机器人工具箱）的**核心设计思想**为标尺，逐个功能剖析本仓的实现选择、代码位置、踩过的坑，以及与 Drake 原生做法的**分歧及其工程理由**。
> 撰写日期：2026-11-23
> 对应里程碑：**M001-acm1jg《双臂规划与操作软件基线》**（已关闭，验证 verdict=pass，10 切片 / 68 任务 / 19 需求 / 49 决策）
> 证据口径：文中每个数字要么来自可执行命令的退出码，要么来自代码位置（`文件:行`），要么来自入库的机器可读证据（`docs/evidence/*.json`）。**未验证项一律显式标注**，不以任何方式声称达标。
> 外部资料访问日期：2026-11-23（清单见附录 B）

---

## 〇、阅读指引

| 章节 | 回答什么 |
|---|---|
| §一 | 我们交付了什么（规模 / 数字 / 证据） |
| §二 | **Drake 的核心设计思想**（单独罗列 11 条，每条附官方出处） |
| §三 | 逐功能剖析：Drake 思想 → 本仓实现 → 代码位置 → 踩坑 → 分歧理由（14 个专题） |
| §四 | 我们与 Drake 的分歧点总表（正面冲突清单） |
| §五 | Drake 思想在本仓「尚未闭环」的部分（诚实缺口） |
| §六 | 总结与后续路线（对应 M002-h6quxw） |
| 附录 A | 代码索引表（按子系统） |
| 附录 B | 外部资料引用清单（URL + 用途 + 访问日期） |

---

## 一、交付成果总览

### 1.1 规模

| 维度 | 数字 | 出处 |
|---|---|---|
| 源码 | 77 个 `.py`（`src/rm75_drake/`：10 个子包 + 12 个顶层模块） | `find src -name '*.py'` |
| 测试 | 76 个测试文件；**1435 passed / 6 skipped / 44 deselected**（`pytest -m "not slow"`，退出码 0） | `.gsd/exec/pytest-m001-full3.stdout` |
| 脚本 | 19 个（阶梯 / 长稳 / 抖动 / 故障注入 / golden / 证据收集 / 协调统计 …） | `scripts/` |
| 提交 | 89 次（收口轮结束时） | `git log --oneline \| wc -l` |
| 决策 / 需求 | 49 条决策（D001–D049）/ 19 条需求（R001–R019，0 MISSING） | GSD DB |
| 切片 / 任务 | 10 / 68，全部 complete | `gsd_milestone_status` |

### 1.2 六条验收标准的最终状态（诚实口径）

| SC | 内容 | 状态 |
|---|---|---|
| SC1 | 真机透传执行，末端误差 < 3 mm，100 Hz × 30 min 无故障 | ✅ **达成**：末端 **2.6620 mm**；**180000** 拍全发、违规 0、错误 0、抖动 P99.9 **0.3145 ms** |
| SC2 | 双臂真机共同搬运，成功率 ≥ 95%、P95 < 3 s | ⛔ **未达成（显式搁置）**：现场仅 1 台 RM75-B；离线 success_rate 1.0 / P95 0.2491 s |
| SC3 | 真机运行全消息录制 → 离线重放逐点一致 | ✅ **达成**：**两轮**独立真机运行均 131 点 `pointwise_identical=true`；离线 golden 722 点 |
| SC4 | 四类物理故障注入 → 安全态 → 恢复 | 🟡 **替身层 5/5 达成**；四类**物理**注入未执行（搁置） |
| SC5 | 厂商解算接口调用数 = 0（静态 + 运行时双证） | ✅ **达成**：静态 103 文件 / 13 项名单 **0 命中**；运行时 `forbidden_call_count=0`（18 万次真实调用） |
| SC6 | 一条命令重建环境，不依赖 ROS2 | ✅ **达成**：18/18 包哈希钉死安装，wall **8–9 s** |

**未达成项共 6 条**（SC2 真机双臂、四类物理注入、物理急停、8 h 长稳、Meshcat、夹爪力曲线）已全部转入 **M002-h6quxw**，本文不把它们算作已达成。

### 1.3 系统的"形状"

```
        YAML 真相源                        Drake 侧                       自研侧
  scenarios/*.yaml   ─┐
  devices/*.yaml     ─┼─→  场景装配  ─→  MultibodyPlant + SceneGraph  ─→  轨迹 (5ms 网格)
  tasks/*.yaml       ─┘   (RobotDiagram)                                  │
                                                                          ├─→ 安全七闸 → rm_movej_canfd 透传
  models/*.urdf  ──→  package:// 解析 ─→  MultibodyPlant                 │
                                                  │                        └─→ UDP 主动上报状态
                                                  └─→ IK / 区域 / GCS / 重定时 / 碰撞复检
```

一句话概括本仓与 Drake 的分工：**Drake 负责"把机器人写成方程"，本仓负责"把厂商控制器写成透传通道"，中间用一条 5 ms 固定节拍的确定性执行缝合起来。**

---

## 二、Drake 的核心设计思想（单独罗列）

> 出处以官网与官方教程为主（附录 B 有完整 URL 与访问日期）。每条给出：**官方原话/事实 → 一句话释义 → 对本项目的约束含义**。

### 思想 1｜模型化优先，而不是黑箱仿真

- 官方定位（drake.mit.edu 首页）：*"Drake … is a collection of tools for analyzing the dynamics of our robots and building control systems for them, with a heavy emphasis on optimization-based design/analysis."* 并明确对比：*"While there are an increasing number of simulation tools available for robotics, most of them function like a black box: commands go in, sensors come out."*
- **释义**：Drake 要求机器人以**显式模型**（URDF/SDF + 参数）进入系统，随后一切（碰撞、IK、轨迹、验证）都从这份模型推导；不接受"传感器进、指令出"的黑箱接口。
- **对本项目的约束**：厂商 SDK 只能提供**关节角透传**这种"最底层、无模型"接口，因此**模型必须由我们自建**（`models/rm75_drake/*.urdf`），而厂商接口只允许出现在最末端。

### 思想 2｜暴露方程结构，让优化算法能用

- 官方原话：Drake 强调 *"an emphasis on exposing the structure in the governing equations (sparsity, analytical gradients, polynomial structure, uncertainty quantification, …) and making this information available for advanced planning, control, and analysis algorithms."*
- **释义**：模型不只是用来"画图"，而是要能被求导、能被展开成多项式、能被梯度算法直接消费。
- **对本项目的约束**：凡是能写成**解析式**的量就不走数值迭代——这正是本仓自研闭式解析 IK（`kinematics/analytic_ik.py`）与纯 numpy 正解（`kinematics/mdh.py`）的来源。

### 思想 3｜优化是设计语言

- 官方事实：Drake 提供 `pydrake.solvers`（数学规划）、`pydrake.geometry.optimization`（GCS / IRIS / HPolyhedron）、`pydrake.multibody.plant` 的优化型求解器接口，并支持 `drake::AutoDiff` / `drake::symbolic` 自动微分与符号化。
- **释义**：控制、规划、分析尽量表达成**可解的数学规划问题**，而不是写一堆启发式 if-else。
- **对本项目的约束**：能用凸集图最短路表达的，就不要用采样规划；这也是 S05 为 14 维双臂选择"最小坐标 + 自研凸区域"的路线依据。

### 思想 4｜System / Diagram / Context 三件套

- 官方教程（`tutorials/working_with_diagrams.ipynb`）：*"In Drake, a **System** is the building block that has input/output ports to connect with other Systems. … A **Diagram** is used to represent a meta-system that may have several interconnected Systems. Each System and Diagram has its own **Context** to represent its state."* 并强调：*"The Context and the Diagram are the only two pieces of information a simulator needs to run. Given the same Context of a Diagram, the simulation should be completely deterministic and repeatable."*
- **释义**：**组合**（Diagram）+ **状态**（Context）+ **确定性**是 Drake 的架构骨架。
- **对本项目的约束**：本仓把同一思想搬进**实时执行域**：`device/exec_loop.py` 定义 `Clock` 协议，注入 `SystemClock` 或 `VirtualClock`（`:101` / `:116`），使得"同一份轨迹 + 同一颗虚拟时钟 ⇒ 逐拍完全一致"，成为故障注入与 golden 回归可离线复算的前提。

### 思想 5｜动力学与几何分离，靠 source 关联

- 官方事实：`AddMultibodyPlantSceneGraph(builder, time_step)` 返回 `(plant, scene_graph)`；两者都是独立的 System，碰撞由 `SceneGraph` 负责、动力学由 `MultibodyPlant` 负责，二者通过 `plant.RegisterAsSource(scene_graph)` 关联。
- **释义**：**一份模型，两套语义**：力学算在 plant，几何算在 scene graph。
- **对本项目的约束**：场景装配必须同时拿到 plant 与 scene_graph（`planning/scene.py:121` `build_scene`），且静态障碍物注册到 scene graph 之后**必须显式赋 proximity 角色**（见 §三·专题 2）。

### 思想 6｜RobotDiagram / DiagramBuilder 与 finalization 时序

- 官方教程：`RobotDiagram` 是 *"simply a Diagram which is automatically populated with a MultibodyPlant and a SceneGraph"*；`builder.Build()` 之后 plant 已 Finalize，碰撞检查器参数 `CollisionCheckerParams.model` 必须传 `RobotDiagram`。
- **释义**：装配有**严格的阶段**：建模 → weld → 注册几何 → `Build()` → 取 context。
- **对本项目的约束**：本仓把这条时序写成了代码内的硬顺序（`models.py:138` 注释即"必须在 Finalize() 之前"），并把版本差异收口到 `drake_compat.py`。

### 思想 7｜模型描述语言与 `package://` 解析

- 官方事实：`pydrake.multibody.parsing.Parser` 支持 `AddModels(url="package://drake_models/...")`，`PackageMap` 用于把包名映射到磁盘目录；Drake 自带 `drake_models` 包（YCB 桌子、cracker box 等）。
- **释义**：模型走**包管理 + URL 引用**，而不是散落在代码里的绝对路径。
- **对本项目的约束**：本仓自建 `rm75_drake` 包（`models/rm75_drake/package.xml`），模型引用一律写成 `package://rm75_drake/rm75_drake.urdf`（`models.py:74`），并自研 `package_map.py` 负责合并 Drake 自带包与本仓包。

### 思想 8｜任务空间规划：GCS（凸集图）与 IRIS（区域）

- 官方事实：`GraphOfConvexSets.SolveShortestPath` 自 Drake v1.16 起发布（release notes v1.16.0）；配套论文 *Motion Planning around Obstacles with Convex Optimization*（arXiv:2205.04422，Marcucci / Petersen / von Wrangel / Tedrake）指出：*"the convex relaxation of our programs is very tight, and a cheap rounding of its solution is typically sufficient to design globally-optimal trajectories… automatically provides optimality bounds"*；论文的演示平台明确包含 *"a dual-arm manipulator (with fourteen degrees of freedom)"*。
- Drake 区域生成器：`IrisNp`（数值型）、`IrisZo`（zonotopic）、`IrisInConfigurationSpaceFromCliqueCover`（v1.27 新增，用最大团覆盖高效布点）。
- **2026-11-23 复核补充**：在本仓锁定的两个 Drake 版本（本地 1.51.1、远端 1.55.0）上实测，`pydrake.planning` 中**同时存在** `IrisNp2`、`IrisZo`、`IrisParameterizationFunction`、`IrisInConfigurationSpaceFromCliqueCover`；`IrisParameterizationFunction` 的官方 docstring 明确写着“允许用户指定函数 f: Q→C，**在 Q 里生长区域**”，正是“自定义最小坐标空间 + IRIS”的官方路径。官方 bimanual 示例用的就是它（见附录 B9）。
- **释义**：把"绕开障碍"表达成**凸集图上的最短路**；用 IRIS 把自由空间切成凸块。
- **对本项目的约束**：单臂 7 维直接用 Drake 的 `IrisNp`（`planning/regions.py:595`）；**双臂 14 维**（以及自研的 R⁸ 最小坐标空间）**无法**用 IRIS 表达 ⇒ 本仓自研区域生成器（`coordinated/regions.py:738`）。

### 思想 9｜声明式 schema + YAML 序列化

- 官方事实：`pydrake.common.yaml` 提供 `yaml_dump_typed(data, schema=…)` 与 `yaml_load_typed(…, private=True)`，*"mimics the C++ function drake::common::yaml::SaveYamlFile"*，用于非原始类型的**有 schema** YAML 落盘。
- **释义**：配置/产物用 **schema 化 YAML**，而非随手 `dict`。
- **对本项目的约束**：本仓为每个真相源写了**独立 schema + 未知键点名报错**：`scenario.py`（场景）、`device/config.py:360`（设备）、`tasks/schema.py`（任务）、`calibration/schema.py`（标定）、`device/run_log.py:185`（运行日志）、`regression/session.py:118`（会话）、`regression/golden.py:133`（golden）。

### 思想 10｜确定性、可复算、可回归

- 官方事实：Drake 的 System/Diagram 语义保证"同一 Context ⇒ 完全确定可重复的仿真"；官方还提供 YAML 快照用于模型/配置的版本化落盘。
- **释义**：**可复算**是架构级承诺，不是测试技巧。
- **对本项目的约束**：本仓把"可复算"做成三件套——① golden 基线绑定**三枚 sha256 指纹**（`regression/golden.py:190` `repo_fingerprints`），指纹漂移即基线失效；② 逐点比较（`:685` `compare_golden`）；③ 运行日志与 golden 用**同一套规范化 JSON + sha256 校验和**口径（`device/run_log.py:419` 与 `regression/golden.py:337`）。

### 思想 11｜示例即基准（examples / tutorials 作为 benchmark）

- 官方原话：*"we hope Drake provides many compelling examples that can help people get started and provide much needed benchmarks."*
- **释义**：官方希望用户用自己的实现去对标官方案例，形成可比较的基准。
- **对本项目的约束**：本仓把"对标"落成**可运行的证据面**：`docs/evidence/` 下 8 个真机 JSON + 4 份人类可读纪要 + 唯一证据收集器 `scripts/collect_evidence.py`（9 个声明式检查项），任何人可在自己的机器上重算同一组数字。

---

## 三、逐功能剖析：Drake 思想 → 本仓实现

> 每个专题结构固定：**Drake 怎么说 → 本仓怎么做 → 代码位置 → 踩过的坑 → 与 Drake 的分歧及理由**。

### 专题 1｜模型化优先：把 RM75-B 写成模型，而不是写成 SDK 调用

- **Drake 怎么说**：机器人要作为模型进入系统（URDF/SDF），一切下游结论从模型推导（思想 1、7）。
- **本仓怎么做**：以官方 `rm_models` 的 RM75-B URDF 为基线做**定点改造**（不重建 SDF，避免第二份模型真相源）：速度限位按官方表修正（J1/J2=3.1416、J3–J7=3.927 rad/s）、轴向量 snap、碰撞几何改为官方网格包围盒生成的 box、显式 weld 固定基座、补夹爪与 TCP；改造版 URDF 入 git 且**零外部资产依赖**。
- **代码位置**：`models/rm75_drake/rm75_drake.urdf`、`models.py:145 load_rm75`（装配）、`:138 _weld_base`（"必须在 Finalize() 之前"）、`:268 joint_limits_from_urdf`（直接读 URDF 的限位与 effort）。
- **踩过的坑**：
  1. `default_collide="false"` 的写法**无效且危险**——Drake 1.51/1.55 的解析器根本不实现 `<drake:disable_collision>` 标签（libdrake.so 内字符串零出现），相邻体过滤实际来自 Drake 默认的父子相邻过滤；一旦将来 Drake 支持该标签，`default_collide="false"` 会**静默关掉全部自碰撞检测**。因此本仓保留该块作声明性意图但写 `default_collide="true"`。
  2. 官方 URDF 的 `joint rpy=1.5708` 是 π/2 的四舍五入值，导致任何独立解析模型与 Drake FK 之间存在 **≈2.2407e-6 m** 的固有位置残差，"FK 误差 < 1e-6 m"在真实官方 URDF 上**不成立**。
- **分歧与理由**：Drake 社区惯例是直接用官方 URDF；本仓选择"改造 + 显式接受残差"，因为 R002 要求惯量/限位/质量可回归，而官方资产**许可未明示、网格不可再分发**（`licenses/registry.yaml` 登记）。FK 门槛因此收敛为唯一真相源常量：`kinematics/mdh.py:145 FK_POSITION_TOLERANCE_URDF_M=5e-6`、`:159 ZERO_Y_ROUNDING_BIAS_M=2.2407e-6`、`:168 WRONG_TABLE_DETECTION_FLOOR_M=1e-3`（保鉴别力），并由 `docs/architecture/fk-tolerance.md` 正式声明口径。

### 专题 2｜场景装配：RobotDiagram 的"时序纪律"与碰撞检查器

- **Drake 怎么说**：`RobotDiagramBuilder → AddModels(url=package://…) → WeldFrames → Build() → robot_diagram.plant_context(root_context)`；`CollisionCheckerParams.model` 只接受 `RobotDiagram`（思想 5、6）。
- **本仓怎么做**：`planning/scene.py:121 build_scene` 固化了这条顺序：建 plant/scene_graph → 注册 package map → `AddModels(package://…)` → weld 基座 → 注册障碍物 → `Build()` → 取 context（**必须传 root context**）。版本差异（1.51/1.55 的 pybind 签名、`CollisionCheckerParams.model` 类型）全部收口到 `drake_compat.py:170 _coerce_collision_model` / `:207 make_collision_checker_params` / `:241 make_collision_checker`。
- **踩过的坑**（三条，都是"配了但不生效"的静默失效）：
  1. `SceneGraph` 没有 `AddGeometry`，静态障碍物必须走 `RegisterAnchoredGeometry(source_id, GeometryInstance(pose, shape, name))`；
  2. 注册后**必须显式** `AssignRole(source_id, geometry_id, ProximityProperties())`，否则几何进了场景图但碰撞检查**完全看不见**——实测箱子正好放在末端正下方时 `CheckConfigCollisionFree` 仍返回 True；
  3. `CollisionCheckerParams.model` 传 `MultibodyPlant` 会抛 pybind cast 异常，必须传 `RobotDiagram`。
- **分歧与理由**：Drake 教程给的是"顺序示例"，本仓把它做成**代码即文档**（每个函数 docstring 写明"必须在此之前"），因为这些错误**不报错**，只会让碰撞检查静默失能。

### 专题 3｜plant 的生命周期：Drake 裸指针陷阱

- **Drake 怎么说**：`Parser` 持有 `MultibodyPlant*` 裸指针；官方教程里 plant 的生命周期由 `DiagramBuilder` 作用域保证。
- **本仓怎么做**：Drake 的 C++ `Parser` 在 Python 侧**不保活 plant**——若 plant 是函数局部变量，函数返回后被 GC 回收，parser 悬垂，后续 `AddModels()` 直接 **SIGSEGV（段错误而非异常）**。本仓在 `package_map.py:88` 用 `WeakKeyDictionary`（弱键 parser + 强引用 plant）保活，并对外暴露 `plant_for_parser()`（`:227`）。
- **分歧与理由**：Drake 官方靠"builder 作用域"规避；本仓因为要做"按需装配、随用随弃"的场景装配工厂，必须显式管理生命周期——这是**为支持"装配函数返回后仍可用"而必须偏离官方写法**的一处。

### 专题 4｜轨迹不变量：把 Drake 的"确定性"搬进数据流

- **Drake 怎么说**：Drake 用 `Context` 承载状态、`Diagram` 承载结构，二者共同保证可重复（思想 4、10）。
- **本仓怎么做**：`planning/trajectory.py:212 Trajectory` 是一条**通过全部不变量**的轨迹：5 ms 网格（`DEFAULT_DT=0.005`）、`seq` 从 0 连续、`t_abs` 严格递增、逐点落在软件软限位内、逐轴速度/加速度不超限；构造即校验（`:438 trajectory_from_arrays`），不满足即抛 `TrajectoryError`。重采样（`:508 resample`）同样走不变量。
- **踩过的坑**：C² 连续性不能用采样点差分"看起来连续"来判，本仓写了离散版复核 `retime.py:185 check_c2_continuity`。
- **分歧与理由**：Drake 的轨迹是 `PiecewisePolynomial`（可微、可求导），本仓刻意**不**用 Drake 的轨迹类型：实时层要的是"可被 5 ms 采样、可逐点断言、可序列化"的**离散真值**，与厂商透传通道逐拍对齐（`device/exec_loop.py` 直接消费点列）。

### 专题 5｜IK：Drake 的 DiffIK vs 自研闭式解析 IK

- **Drake 怎么说**：提供 `DifferentialInverseKinematics`（`DifferentialInverseKinematicsParameters` + `DiffIkParameters`，`drake_compat.py:281` 做了新旧签名兼容）与 `JacobianWrtConfiguration`；官方还提供 `multibody_plant` 的 `ComputeForwardKinematics` 等。
- **本仓怎么做**：两条 IK 并存，且**分工明确**：
  - `planning/ik.py:321 solve_ik`（DLS 数值法，Drake DiffIK 语义）用于通用单臂目标；
  - `kinematics/analytic_ik.py:596 solve_analytic_ik`（**纯 numpy 闭式解析解**：8 组镜像分支 × ψ 网格，`:288 _pair_solutions` 解 `u = Rz(qa)·Rx(αa)·Rz(qb)·Rx(αb)·ez`，`:332 _wrist_solutions` 解球腕）用于**双臂紧协调**。
- **理由（对应思想 2、3）**：相对位姿固定要求 follower 位姿**构造性可解**，数值 DLS 既慢又不保证构造性；而 RM75 的 MDH 中 `a_{i-1}` 全为 0（零偏置 S-R-S 构型），存在闭式几何解；闭式解往返误差 ~1e-15 m（精确表），且纯 numpy 可作为 Drake FK 的**独立对照**——这正是"暴露结构、让优化能用"的直接收益。
- **踩过的坑**：解析 IK 的失败必须**点名**（`analytic_ik.py:261 AnalyticIkResult` 带 `REASON_*` 词表），否则上层无法区分"不可达"与"超时"。

### 专题 6｜区域生成：IRIS 的边界与 R⁸ 自研凸区域

- **Drake 怎么说**：IRIS 系列（`IrisNp` / `IrisZo` / `IrisInConfigurationSpaceFromCliqueCover`）在**机器人关节空间**里切分凸区域；`HPolyhedron` 是统一的凸集表示。
- **本仓怎么做**：
  - 单臂 7 维：直接用 `IrisNp`（`planning/regions.py:595 _run_iris`），再与软件限位盒**精确求交**（`:582 _clip_to_soft_limits`，H-多面体约束行堆叠），区域身份 = 模型版本 sha256 + spec 短哈希（`:530 scene_model_version` / `:556 _region_id`），缓存惰性（`:745 RegionCache`）。
  - 双臂 14 维 / R⁸ 最小坐标空间：本仓**自研**生成器：`coordinated/regions.py:738 generate_minimal_region`（种子 + **确定性**方向探测 + 保守半空间切割 → R⁸ `HPolyhedron`），热循环是纯 numpy（为后续 pybind11 下沉留口）。
  - **选型复核（2026-11-23 新发现，见专题 15）**：当时的决策记录（D031）写的是“IRIS 绑定 plant 关节空间，无法表达自定义最小坐标空间”；但在**锁定版本上实测**，`IrisParameterizationFunction` 确实支持在自定义参数化空间里做 IRIS（官方 docstring：*allows the user to specify a function f:Q→C, and grow the region in Q instead*），官方 bimanual 示例正是这么做的。真正的门槛不在“IRIS 能不能”，而在**参数化函数的可微分签名**：`IrisZo` 需要 `Eigen::VectorX<double>` 版本，`IrisNp2` 还需要 `Eigen::VectorX<AutoDiffXd>` 版本，而本仓的 `CoordinatedParameterization` 是**纯 numpy 实现**、未做 autodiff 模板化 ⇒ 接入 Drake 原生 IRIS 需要把参数化重写成可微分形式。
- **踩过的坑**：区域生成必须**可复现**（方向集合确定性、随机种子显式），否则缓存与 golden 都会漂；区域为空/退化要显式 reason（`REASON_EMPTY_DOMAIN` 等）。

### 专题 7｜GCS 最短路：把官方论文的"凸松弛 + 廉价舍入"落到代码

- **Drake 怎么说**（思想 8）：`GraphOfConvexSets.SolveShortestPath`；官方论文强调凸松弛**很紧**、**廉价舍入**通常就够、且**自动给出最优性界**；本机缺整数求解器时（无 Mosek 许可）官方 `SolveShortestPath` 会抛 RuntimeError。
- **本仓怎么做**：`planning/gcs.py:320 build_graph` 建图、`:451 solve_path` 求解——**先尝试混合整数，缺整数求解器则退到凸松弛**（`:431 _is_missing_integer_solver` 精确识别"缺求解器"而非"问题不可行"），`DEFAULT_MAX_ROUNDED_PATHS=10` 控制舍入条数；解出后 `:587 _densify` 把角点序列稠密化、`:603 path_to_q_sequence` 转成**分段线性插值**的关节点列，最后 `:676 collision_recheck` 密采样复检，**逃逸数必须为 0** 才算成功。
- **诚实边界**：本机所有 GCS 数字都是**凸松弛口径**，日志恒有「整数版 GCS 不可用，回退凸松弛」WARNING；文档与脚本都禁止把它表述为"整数 GCS 达标"。
- **选型复核（2026-11-23 新发现，见专题 15）**：当时决策（D014）记为“`GcsTrajectoryOptimization.AddRegions` 所需的 `GraphOfConvexSets.Region` / `ContinuousConvexSet` 组合在 1.51/1.55 不可用”；但实测 1.51.1 上 `pydrake.planning.GcsTrajectoryOptimization.AddRegions(regions: list[ConvexSet], edges_between_regions, order, h_min, h_max, …)` **存在且接受普通 `ConvexSet` 列表** ⇒ 轨迹级 GCS 在锁定版本上**是可用的原生路径**。本仓仍选择“空间 GCS + 自研轨迹整形/重定时”，其收益是：结果与我们的稠密 q 点列 / 5 ms 网格 / 自研不变量口径直接对齐，且不引入轨迹级优化的凸松弛-舍入二次近似；代价是重复实现了一条 Drake 已有的路径。

### 专题 8｜重定时：Drake 没有的部件（TOPPRA 封装 + 维度无关核）

- **Drake 怎么说**：Drake 1.51/1.55 的 `pydrake.multibody.plant` 侧没有可直接用于"关节路径 → 5 ms 时间参数化"的接口；官方生态里重定时通常由 `toppra` 这类外部库承担（官方 bimanual 示例即如此）。
- **本仓怎么做**：`planning/retime.py:279 retime`（7 维，产出 `RetimedTrajectory`）与 `:690 retime_path`（**维度无关**，14 维双臂复用）共享同一个**通用核** `:542 _retime_core`；底层用 `toppra 0.6.11` 参数化（`:837 _load_toppra` / `:869 _parameterize`），再在**固定 5 ms 网格**上采样（`:902 _sample_grid`），最后 `:925 _first_limit_violation` 逐轴核验速度/加速度。
- **工程细节**：`toppra` 的新 API 与旧版差异极大（`TOPPRA(constraints, path)` + `compute_trajectory()`，旧的 `prepare(ss, path)` 已移除），本仓把这层差异封在 `_load_toppra` 内部。
- **选型复核（2026-11-23 新发现，见专题 15）**：Drake **自带** TOPPRA 实现 `pydrake.multibody.optimization.Toppra`（`AddJointVelocityLimit` / `AddJointAccelerationLimit` / `SolvePathParameterization` / `CalcGridPoints` + `ToppraDiscretization`），在 1.51.1 / 1.55.0 上均可用 ⇒ 外部 `toppra 0.6.11` 依赖**并非必需**。本仓选择外部库的真实收益是“纯 python 依赖、API 可控、与我们的采样口径一致”；代价是多一条依赖与一次 API 漂移适配。
- **分歧与理由**：Drake 侧不重造轮子，但**重定时是本系统的实时性根基**（5 ms 网格 + 逐轴不超限），必须有自有的不变量与回归，因此选择"外部库做内核、自研做不变量与采样"。

### 专题 9｜双臂装配：Drake 的硬限制与变体路线

- **Drake 怎么说**：`MultibodyPlant` 内同一份 URDF **不允许装两次**（model instance names must be unique）；两台机器人要么两个 plant，要么一个 plant 两个**变体**。
- **本仓怎么做**：选**同一个 plant + 每臂一份重命名 URDF 变体**（`models/rm75_drake/rm75_{left,right}.urdf`、`rmg24_{left,right}.urdf`），变体由确定性生成器 `tools/gen_dual_arm_models.py` 产出并入 git（CI 校验幂等）。装配入口 `bimanual/dual_scene.py:244 build_dual_scene`；夹爪作为独立 model instance 并**显式 weld** 到各自法兰；碰撞查询 `check_config_free`（`:457`）在**同一次查询**里同时判臂间与自碰撞。
- **踩过的坑**：维度不一致（14 vs 18 vs 7）会一路传导到 Drake 的 abort，必须在装配层就校验并抛 `SceneError`；`coordinated/limits.py:257 pad_gripper_positions` 负责把 14 维补齐成 plant 的完整位置向量。

### 专题 10｜协调规划：构造性满足 + 显式降级链

- **Drake 怎么说**：Drake 的紧协调路线依赖"参数化把等式约束从流形上消掉"——官方 bimanual 示例（ICRA 2024, Cohn/Shaw/Simchowitz/Tedrake）用的正是**解析 IK 最小坐标参数化 + IRIS 区域 + Toppra 重定时**，论文并称 *"C++ 参数化后区域生成降到约 1.2 秒"*。
- **本仓怎么做**：把该方法论落到 RM75：
  - 最小坐标：leader 7 关节 + follower 肘点方位角 ψ ⇒ **R⁸**（`coordinated/parameterization.py:250`）；相对位姿由常数 `X_LR`（leader TCP 系 → follower TCP 系）**构造性满足**（`:536` 语义 `T_follower_world = T_leader_world · X_LR`）；
  - 区域：IRIS 不可用 ⇒ 自研 R⁸ 凸区域（专题 6）；
  - 求解：先参数化 → ψ 网格扫描并**选余量最大且离上一拍最近**的可行 ψ（`coordinated/planner.py:885 _psi_margins` / `:915 _select_psi`）→ 映射回 14 维（`:1200 _map_path`）→ **14 维密采样碰撞复检**（`:1220`，逃逸数必须 0）→ 逐点相对位姿残差（`:1263`）；
  - 降级：区域链不完整 / GCS 无路 ⇒ 显式降级到**同一套 oracle** 的采样基线（`coordinated/regions.py:1029 generate_region_with_fallback`），降级必须可观测（`degraded=True` + `degradation_reason`）。
- **踩过的坑**：降级路径的 PRM 只有两个端点，必须在**最小坐标空间稠密化**后再过残差闸，否则残差 ~O(结点间距²) 会超标；`coordinated/planner.py:1297 _densify_minimal_path` 就是这条补丁。
- **实测**：相对位姿残差 max **1.835e-08 m / 9.892e-08 rad**；离线成功率 1.0、求解 P95 **0.2491 s**（< 3 s）。

### 专题 11｜安全七闸：把"显式失败"变成"纯函数 + 稳定 reason"

- **Drake 怎么说**：Drake 的失败路径是**带类型的异常 + 明确诊断**（`System` 端口错误、`InvalidArgument` 等），且大量能力可探测（不可用就抛 `NotAvailableError`）。
- **本仓怎么做**：安全七闸实现为**纯函数** `device/safety.py:728 apply_safety`：输入上一拍指令 + 本拍状态，输出**钳位后指令 + 有序违规列表**；G1 软限位 / G2 增量 / G3 加速度是**位置约束**，按"每轴区间交集"一次性钳位（`:682 _clamp_joint`），因此钳位后**同时**满足三闸；G4–G7 是监视闸（看门狗 / 电流 / 围栏 / 状态 age）。
- **动作分级**：钳位类违规（`action=clamp`）只记录，停机类（`slow_stop` / `stop`）才升级为失败（`:137-142` 动作与严重度表）——这条口径被 CLI 与脚本共用。
- **踩过的坑**：单位混用会把关节瞬间送到 57 倍位置，因此 rad↔deg 转换被收敛到**唯一实现点**（`device/sdk.py:128 deg_to_rad`），并由 `assert_single_conversion_point`（`:178`）扫描源码守护"只有一份实现"。

### 专题 12｜实时执行环：把 Drake 的确定性搬到 5 ms 域

- **Drake 怎么说**：同一 Context ⇒ 确定性可重复（思想 4、10）。
- **本仓怎么做**：`device/exec_loop.py:368 ExecLoop` 是 **5 ms 固定节拍**（`:81 PERIOD_S = 0.005`）执行环，**零 SDK 依赖**（只依赖传输层边界 `ArmTransport`）：取轨迹点 → 读状态（age / 电流）→ 安全七闸钳位 → 下发。时钟通过 `Clock` 协议注入（`:91`），生产用 `SystemClock`（`time.perf_counter_ns` + sleep），测试用 `VirtualClock`（`:116`，零真实等待）。
- **可观测性**：调度结果 `RtSchedulingReport`（`:244`，尝试 `SCHED_FIFO` + CPU 亲和性，失败**必须显式可见**）；抖动/延迟用**预分配直方图**统计（`device/metrics.py:91 MetricHistogram`，`:283 JitterHistogram`），并同时给出 p50/p99/p999。
- **真机数字**：L7 抖动 P99.9 **0.1952 ms**；30 min 长稳 180000 拍、违规 0、抖动 P99.9 **0.3145 ms**、命令→执行延迟 P99.9 0.2901 ms。

### 专题 13｜状态机、故障与恢复：稳定 reason + 逐步留痕

- **Drake 怎么说**：失败要有类型与诊断（思想 1 的"暴露结构"在失败路径上的体现）。
- **本仓怎么做**：
  - 9 态设备状态机（`device/state_machine.py:110`）= 纯转移表 + 守卫（`:303 _build_table`），非法转移抛 `IllegalTransitionError`、守卫不满足抛 `GuardRejectedError`，**每次转移都写进运行日志**；
  - 帧级系统错误码由**唯一纯函数**判定（`:368 system_error_code`），状态机与恢复层共用（避免两处口径漂移）；
  - 恢复流程 `device/recovery.py:480 RecoveryProcedure` 每一步都走传输层且**每步留痕**（`RecoveryStep` 带 `ok / warning / failed / skipped`），失败抛带可执行诊断的 `RecoveryError`，**绝不静默跳过**；
  - 五类故障（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落）各有稳定 reason；`SAFE_STATE_BY_REASON`（`recovery.py:137`）是"reason → 安全态"的唯一映射。
- **真机数字**：替身层 5/5 全链（`ok / recovered / final_state=ready`，`failed=[]`）；真机侧"夹爪未到位链"闭合。

### 专题 14｜任务层、感知与回归：schema 化 + 单一主状态源

- **Drake 怎么说**：schema/YAML 化（思想 9）+ 可复算基准（思想 11）。
- **本仓怎么做**：
  - 任务定义另立 schema 与仓库根 `tasks/*.yaml`（不改 `scenarios/*.yaml` 的封闭顶层键，避免击穿 golden 指纹）；10 个原语目录（`tasks/primitives.py:80`）与**单一求解器入口** `tasks/solver.py:855 solve_task`（运动类原语 = 解析 IK → 关节插值 → 碰撞复检 → 重定时）；
  - 任务状态机是**单一主状态源**（`tasks/state_machine.py:147`），审计轨迹只读，不是真相源；
  - 感知以**离散事件总线**接入（`tasks/perception.py:226 PerceptionBus`，非阻塞、无线程、无 asyncio），且"延迟 ⇒ 膨胀裕量"保守化（`:174 PerceptionPolicy`）；场景更新必须"注册 + 显式赋角色 + 重建 checker + 回填 bundle"四步合一（`:370 apply_perception_scene`），否则保守场景**静默失效**；
  - 回归侧：golden 绑定**三指纹**（`regression/golden.py:190`）、**逐点**比较（`:685`）、校验和与运行日志同口径（`:337` vs `device/run_log.py:419`）；会话产物 `regression/session.py:335 build_session` 只读聚合 recording + run_log，且**断言被消费的 schema 版本未变**（`:442`）。

### 专题 15｜选型复核：锁定版本上 Drake 其实提供了什么（2026-11-23 新发现，诚实自查）

- **复核方法**：遍历 `pydrake` 全部子模块检索关键符号，在**本地 1.51.1** 与**远端 1.55.0** 两个锁定版本上各跑一次，并直接读官方 docstring 确认语义（不依赖记忆）。可复算脚本见 **§七**。
- **普查结果**：

| 符号 | 1.51.1（本地） | 1.55.0（远端） | 本仓是否使用 | 备注 |
|---|---|---|---|---|
| `pydrake.planning.IrisNp2` / `IrisZo` | ✅ 存在 | ✅ 存在 | ❌ 未用 | 官方 bimanual 示例用它在**参数化空间**里长区域 |
| `pydrake.planning.IrisParameterizationFunction` | ✅ 存在 | ✅ 存在 | ❌ 未用 | docstring：可给 f: Q→C，**在 Q 里生长区域** |
| `pydrake.planning.IrisInConfigurationSpaceFromCliqueCover` | ✅ 存在 | ✅ 存在 | ❌ 未用 | v1.27 起 |
| `pydrake.planning.GcsTrajectoryOptimization`（含 `AddRegions(list[ConvexSet], edges, order, h_min, h_max)`） | ✅ 存在 | ✅ 存在 | ❌ 未用 | 轨迹级 GCS 原生路径可用 |
| `pydrake.multibody.optimization.Toppra` / `ToppraDiscretization` | ✅ 存在 | ✅ 存在 | ⚠️ 改用外部 `toppra 0.6.11` | Drake 自带 TOPPRA（Hung Pham & Quang Cuong Pham, T-RO 2018） |
| `pydrake.planning.KinematicTrajectoryOptimization` | ✅ 存在 | ✅ 存在 | ❌ 未用 | 官方示例用它做轨迹精修 |
| `pydrake.trajectories.FunctionHandleTrajectory` | ✅ 存在 | ✅ 存在 | ❌ 未用 | 官方示例用它把参数化空间轨迹映射回全位形空间 |
| `pydrake.geometry.optimization.IrisNp`（legacy） | ✅ 存在 | ✅ 存在 | ✅ 单臂使用 | 本仓单臂区域生成 |
| `pydrake.geometry.optimization.GraphOfConvexSets` | ✅ 存在 | ✅ 存在 | ✅ 空间 GCS | 本仓双臂/单臂最短路 |

- **三处结论**（与当时决策记录的对照）：

| 决策 | 当时的记录 | 实测 | 判定 |
|---|---|---|---|
| D031（区域生成） | “IRIS 绑定 plant 关节空间，无法表达自定义最小坐标空间” | `IrisParameterizationFunction` 明确支持自定义参数化空间 | **原判断有误**；真正门槛是参数化需提供 `VectorX<double>`（IrisZo）与 `VectorX<AutoDiffXd>`（IrisNp2）两种可微分签名，而本仓参数化是纯 numpy、未模板化 |
| D014（GCS 入口） | “`GcsTrajectoryOptimization.AddRegions` 所需组合在 1.51/1.55 不可用” | `AddRegions(list[ConvexSet], edges, order, h_min, h_max)` 可用 | **原判断有误**（很可能是把“顶层 `pydrake.geometry.optimization` 列表里没有”误读成“不存在”，实际在 `pydrake.planning` 子模块） |
| 重定时依赖 | 生态用 toppra；本仓固定外部 `toppra 0.6.11` | Drake 自带 `multibody.optimization.Toppra` | **依赖可替换**；当前选择保留（见下） |

- **为什么会漏**：早期调研以“顶层模块成员清单 + 记忆”判断 API 可用性，未做**全子模块符号普查 + docstring 语义确认**；`IrisNp2` / `IrisZo` / `IrisParameterizationFunction` / `GcsTrajectoryOptimization` 都在 `pydrake.planning` 而非 `pydrake.geometry.optimization`，于是被漏判。这是一条**方法论教训**，已列入 §六.4。

- **对已交付成果的影响评估**（诚实结论）：
  - **不影响正确性**：三处自研路线都能跑通、有不变量、有回归与真机证据（末端 2.6620 mm、残差 ≤1e-8、P95 0.2491 s、零逃逸），没有观察到失败；
  - **确实增加了维护面**：重复实现意味着未来 Drake 升级时要多维护一份逻辑，且失去了“跟随上游生态”的红利（例如官方示例用 `FunctionHandleTrajectory` + `GcsTrajectoryOptimization` 的组合能直接给出参数化空间到全位形空间的映射，本仓要自研映射与稠密化）；
  - **自研路线仍有其价值**：numpy-only 热循环（便于后续 pybind11 下沉）、结果与 5 ms 网格/稠密 q 点列直接对齐、显式降级链与确定性可复现（种子 + 确定性方向集合）——这些是官方示例没有直接给出的。

- **建议动作（M002-S00，可离线完成）**：对三处各做一次“原生 vs 自研”的**同场景 A/B 对照**：① `IrisParameterizationFunction + IrisZo` 与自研 R⁸ 生成器在同一场景下比较区域体积、覆盖率、生成耗时；② `GcsTrajectoryOptimization` 与空间 GCS 比较解质量（路径长度、逃逸数）与工程量；③ `multibody.optimization.Toppra` 与外部 toppra 比较 5 ms 网格采样结果与耗时。用数据决定保留哪条，并把结论写回 `docs/architecture/`。


---

## 四、我们与 Drake 的分歧点总表

| # | 分歧 | Drake 原生做法 | 本仓做法 | 工程理由 |
|---|---|---|---|---|
| 1 | 轨迹表示 | `PiecewisePolynomial`（连续、可导） | 离散 5 ms 网格点列 + 不变量 | 实时层与厂商透传通道逐拍对齐，序列化与断言成本最低 |
| 2 | 逆解 | `DifferentialInverseKinematics` | DLS（通用）+ **闭式解析 IK**（双臂） | 相对位姿需构造性满足；闭式解可作独立对照 |
| 3 | 区域生成 | `IrisNp` / `IrisNp2` / `IrisZo` / `IrisParameterizationFunction`（锁定版本实测**均可用**） | 单臂用 legacy `IrisNp`；双臂/最小坐标**自研 R⁸ 凸区域** | 当时判断“IRIS 无法表达自定义最小坐标空间”**有误**（见专题 15）。原生路径的门槛是参数化需满足 autodiff 签名；自研换来 numpy-only 热循环、确定性与显式降级链 |
| 4 | GCS 入口 | `GcsTrajectoryOptimization.AddRegions(list[ConvexSet], edges, order, h_min, h_max)`（实测**可用**） | `GraphOfConvexSets.SolveShortestPath`（空间级）+ 自研整形/重定时 | 原判断“AddRegions 不可用”**有误**；自研换来与 5 ms 网格 / 稠密 q 点列直接对齐的结果形态与自有不变量 |
| 5 | 重定时 | `pydrake.multibody.optimization.Toppra`（**Drake 自带 TOPPRA**，实测可用） | 外部 `toppra 0.6.11` 内核 + 自研不变量/采样/维度无关核 | 5 ms 网格与逐轴限幅需要自有不变量与回归；但依赖可换（见专题 15） |
| 6 | 碰撞检查器 | `CollisionCheckerParams.model = RobotDiagram` | 同上，但**版本差异收口**到 `drake_compat.py` | 1.51/1.55 pybind 签名不同 |
| 7 | plant 生命周期 | 靠 builder 作用域保活 | weakref 保活表 + `plant_for_parser()` | 装配工厂需"返回后仍可用" |
| 8 | 障碍物角色 | 需显式 `AssignRole(ProximityProperties)` | 同上，并写进代码顺序与文档 | 配错**不报错**，只是静默失能 |
| 9 | 可视化 | Meshcat 原生可用 | 探测式 Meshcat + matplotlib 2D 兜底 | 本机 PyPI 不可达，meshcat wheel 不在离线镜像 |
| 10 | 力 / 夹持 | 无（属控制与标定） | `gripper.py:189 force_from_position` 显式抛「未标定」 | 没有力曲线就不能谎称力阈值夹取 |
| 11 | 厂商接口 | — | 全禁（除 `rm_movej_canfd` 透传），静态 + 运行时双证 | 用户约束：不用厂商任何解算/运动学接口 |
| 12 | 冗余零空间 | Drake `nominal_posture` / `JointCenteringCost` 唯一 owner | 同上（厂商 QP 全禁） | 厂商 QP 与 Drake 抢零空间会导致肘部翻转 |

---

## 五、Drake 思想在本仓尚未闭环的部分（诚实缺口）

按"Drake 思想 → 本仓现状 → 缺什么 → 归属"列出**未达成**项，不做任何达标表述。

| # | Drake 思想 | 本仓现状 | 缺什么 | 归属 |
|---|---|---|---|---|
| 1 | 思想 8/10：GCS 与双机器人 14 自由度演示（官方论文明确以 14-DoF 双臂为平台） | 双臂紧协调**离线**口径达标（成功率 1.0、P95 0.2491 s、残差 ≤1e-8） | **真机**双臂共同搬运证据（现场仅 1 台臂） | M002-S01 |
| 2 | 思想 3：优化设计的完整闭环（含整数 GCS 与最优性界） | 全部为**凸松弛口径**，本机无整数求解器（Mosek 许可不可得） | 整数 GCS 复算与最优性界记录 | M002（登记） |
| 3 | 思想 5/6：IRIS 在自定义最小坐标空间的原生支持 | 自研 R⁸ 凸区域生成器（方向探测 + 半空间切割） | 与 Drake IRIS 的数值对照（同一场景下区域体积/覆盖率对拍） | M002-S00（可离线做） |
| 4 | 思想 4：Diagram 级可视化闭环 | Meshcat **未实现**（探测式 `NotAvailableError` + 2D 兜底，`viz.py:53`） | 真实 Meshcat 后端（缺 wheel/无 PyPI） | M002-S04 |
| 5 | 思想 1：完整动力学模型（含接触/柔顺） | 本仓只做**运动学规划 + 位置伺服**，不建模接触与力 | 力阈值夹取闭环、u→N 力曲线标定（`gripper.py:189` 显式抛「未标定」） | M002-S04 |
| 6 | 思想 4/10：确定性执行的**物理**对拍 | 5 ms 执行环已有虚拟时钟确定性，但真机侧无"同一 Context ⇒ 同一轨迹"的物理复现 | 8 h 长稳重跑（上次 `status=failed`，`dropped=879999`、抖动 P99.9 9.9276 ms） | M002-S03 |
| 7 | 思想 9：schema 化 YAML 的**原生**类型 | 本仓自写 schema（19 个模块各自校验） | 与 `pydrake.common.yaml` 的 schema 对齐（可选） | 低优先（登记） |
| 8 | 思想 11：以官方示例为基准对标 | 本仓自建证据面（8 个真机 JSON + 4 份纪要 + 9 项检查收集器） | 与官方 bimanual 示例的**同场景**数值对拍 | M002-S01（依赖第二台臂） |
| 9 | 思想 3/8：**原生路径 A/B 对照**（2026-11-23 复核新发现） | 区域生成、GCS 入口、重定时三处均自研；锁定版本上 Drake 其实提供 `IrisParameterizationFunction` / `GcsTrajectoryOptimization` / `multibody.optimization.Toppra` | 三处各做一次“原生 vs 自研”的同场景数值对拍与成本核算，用数据决定保留哪条 | M002-S00（可离线做） |

**另有 4 类"物理前提"缺口不在上表**（属工位条件而非软件能力）：四类物理故障注入（需人手在设备旁）、物理急停响应（软件急停接口返回 -4）、第二台机械臂接入、围栏高度余量仅 ~4 mm 的现场重标定。

---

## 六、总结：这套系统到底交付了什么

### 6.1 三句话总结

1. **Drake 侧**：把 RM75-B 变成**可被优化算法直接消费**的模型（改造 URDF + package:// + RobotDiagram 场景 + IRIS 区域 + GCS 最短路 + numpy 闭式解析 IK + 自研 R⁸ 凸区域 + TOPPRA 重定时），并把 Drake 的"确定性 / 显式失败 / schema 化 / 基准化"四条纪律原样继承到**实时执行域**。
2. **自研侧**：把厂商控制器压缩成**唯一一条关节角透传通道**（`rm_movej_canfd`）+ 一条 UDP 状态回传，中间以 **5 ms 固定节拍**缝合；安全七闸是纯函数、状态机是转移表 + 守卫、故障恢复逐步留痕——**任何失败都有稳定 reason 与可复算证据**。
3. **工程侧**：所有结论都带退出码与证据文件；**真机数字与离线数字分属不同环境且分别标注**；未达成的 6 项显式搁置并转入 M002，不以任何方式混入"已完成"。

### 6.2 与 Drake 思想的对应关系一览（思想 → 落地）

| Drake 思想 | 本仓落地形态 | 代表代码 |
|---|---|---|
| 1 模型化优先 | 改造 URDF + 自建包 + 零厂商解算 | `models.py:74`、`models/rm75_drake/` |
| 2 暴露结构 | 闭式解析 IK、纯 numpy 正解、常量唯一真相源 | `kinematics/analytic_ik.py:596`、`kinematics/mdh.py:238` |
| 3 优化即语言 | GCS 凸集图最短路 + 凸松弛/舍入 | `planning/gcs.py:451` |
| 4 System/Diagram/Context | 场景装配分层 + `Clock` 协议 + 虚拟时钟 | `planning/scene.py:121`、`device/exec_loop.py:91/116` |
| 5 动力学/几何分离 | plant + scene_graph 双 source 注册 | `models.py:123`、`planning/scene.py:111` |
| 6 DiagramBuilder 时序 | 装配顺序固化 + 版本差异收口 | `models.py:138`、`drake_compat.py:241` |
| 7 package:// | `package_map.py` 合并包与保活 | `package_map.py:104/197` |
| 8 GCS / IRIS | 单臂用 IRIS；双臂自研 R⁸ 凸区域 | `planning/regions.py:595`、`coordinated/regions.py:738` |
| 9 schema + YAML | 19 个模块各自的严格 schema | `device/config.py:360`、`tasks/schema.py` |
| 10 确定性可复算 | 三指纹 golden + 校验和 + 逐点比较 | `regression/golden.py:190/685` |
| 11 示例即基准 | 唯一证据收集器 + 入库证据面 | `scripts/collect_evidence.py`、`docs/evidence/` |

### 6.3 下一步（对应 M002-h6quxw）

| 切片 | 内容 | 对应本文 §五 的缺口 |
|---|---|---|
| S00 | 硬件前提与工位就绪：第二台臂接入、围栏重标定、归档镜像回退进 `fetch_upstream.py` | #3、#8 的前置 |
| S01 | 双臂真机共同搬运（SC2） | #1、#8 |
| S02 | 物理故障注入与恢复（SC4 物理层） | 物理前提缺口 |
| S03 | 物理急停响应 + 8 h 长稳重跑 | #6 |
| S04 | Meshcat 三维可视化 + 夹爪力曲线与力阈值夹取 | #4、#5 |

### 6.4 给下一位工程师的三条提醒

1. **改文件不要用 `edit` 工具**：它会把中文全角标点批量转半角，产生上千行无关 diff（本仓中文注释/文档一律全角）。改用 bash heredoc + python 读写。
2. **FK 门槛只认 `kinematics/mdh.py` 的常量**：测试层禁止裸阈值字面量；官方 URDF 的 `rpy=1.5708` 舍入残差 2.2407e-6 m 是**已知固有量**，不是 bug。
3. **判断 Drake 能力前先做符号普查**：本仓曾因只查 `pydrake.geometry.optimization` 顶层成员而漏判 `pydrake.planning` 里的 `IrisNp2` / `IrisZo` / `IrisParameterizationFunction` / `GcsTrajectoryOptimization`（专题 15）。**用脚本遍历全部子模块 + 读 docstring**，不要凭记忆或单个模块的成员列表下结论。
4. **真机操作红线**：物理急停在手；会动真机的入口必须显式 `--allow-motion` 且显式给目标位姿；同一时刻只允许一个真机进程（`device/hardware_lock.py`）；控制器拒绝请求时**返回 1 而不抛异常**，必须检查 SDK 返回码。

---

## 七、复核命令（本文所有结论的复算入口）

### 7.1 Drake 能力符号普查（专题 15 的复核方法）

```bash
python3 - <<'EOF'
import importlib, pkgutil, warnings, pydrake
warnings.filterwarnings("ignore")
targets = ["GcsTrajectoryOptimization","IrisParameterizationFunction","IrisNp2","IrisZo",
           "FunctionHandleTrajectory","Toppra","KinematicTrajectoryOptimization",
           "IrisInConfigurationSpaceFromCliqueCover","IrisNp","GraphOfConvexSets"]
hits = {}
for m in pkgutil.walk_packages(pydrake.__path__, prefix="pydrake."):
    try: mod = importlib.import_module(m.name)
    except Exception: continue
    for t in targets:
        if hasattr(mod, t): hits.setdefault(t, []).append(m.name)
# 显式补齐若干可能未被 walk 命中的模块（walk 时若有导入失败会被跳过）
for extra in ("pydrake.geometry.optimization", "pydrake.multibody.optimization",
              "pydrake.planning", "pydrake.trajectories"):
    try: mod = importlib.import_module(extra)
    except Exception: continue
    for t in targets:
        if hasattr(mod, t) and extra not in hits.setdefault(t, []): hits[t].append(extra)
for t in targets:
    print(f"{t:42} -> {hits.get(t, 'NOT FOUND')}")
EOF
```

> 关键点：**必须遍历全部子模块**。`IrisNp2` / `IrisZo` / `IrisParameterizationFunction` / `GcsTrajectoryOptimization` 都在 `pydrake.planning`，而 `Toppra` 在 `pydrake.multibody.optimization`——只看 `pydrake.geometry.optimization` 的顶层成员列表会漏判。

### 7.2 离线结论复算（开发机即可）

```bash
cd .gsd-worktrees/M001-acm1jg   # 或已并入 feat-drake 的仓库根
python3 -m pytest -q -m "not slow"                                  # 1435 passed / 6 skipped
bash scripts/ci.sh fast                                            # PASS=19 SKIP=0 FAIL=0
python3 scripts/scan_forbidden_apis.py src tools scripts            # 0 命中（103 文件 / 13 项）
python3 scripts/gen_golden.py --check                              # 722 点逐点一致
python3 scripts/fault_inject.py --json                             # 五类故障 5/5，failed=[]
python3 tools/fetch_upstream.py --verify                           # exit 0「校验通过：2 个资产」
python3 tools/verify_env.py --model --joint-limits /nonexistent/path.yaml   # exit 2
PYTHONPATH=src python3 -m rm75_drake.cli task --task tasks/handover.yaml      # exit 2 + task_single_arm_only
PYTHONPATH=src python3 scripts/coordinated_stats.py --samples 20 --json      # 成功率 1.0 / P95 0.2491 s
```

### 7.3 真机结论复算（远端工作站，网卡 `192.168.1.198/24`；容器 `drake_dev` 做规划，宿主 `/tmp/rmvenv` 做执行）

```bash
# 入库证据的门槛断言（本文 §一.2 / §三 各处数字的来源）
python3 - <<'EOF'
import json, pathlib
E = pathlib.Path("docs/evidence")
lad = json.loads((E/"rm75-b-ladder-final-2026-11-20.json").read_text())
soak = json.loads((E/"rm75-b-soak-30min-final-2026-11-20.json").read_text())
trk = json.loads((E/"rm75-b-tracking-final-2026-11-20.json").read_text())
print("ladder  PASS/FAIL:", lad["summary"]["pass"], lad["summary"]["fail"])
print("soak    ticks/viol/err/p999:", soak["ticks"], soak["num_violations"], soak["num_errors"], round(soak["jitter_ms"]["p999"],4))
print("soak    forbidden_call_count:", soak["sdk_audit"]["forbidden_call_count"])
print("tracking tcp max/p95 mm:", round(trk["tcp_error_mm"]["max"],4), round(trk["tcp_error_mm"]["p95"],4), trk["status"])
EOF

# 真机复跑（运动级必须显式 --allow-motion，且物理急停在手）
/tmp/rmvenv/bin/python scripts/real_machine_ladder.py --to L10     --device-config devices/rm75_b_192_168_1_18.yaml --allow-motion --json
```

---

## 附录 A：代码索引表（按子系统）

| 子系统 | 关键文件 | 规模 | 职责一句话 |
|---|---|---|---|
| 模型与装配 | `models.py` / `package_map.py` | 294 / 232 行 | 把 RM75-B 与夹爪装进 plant，并让 `package://` 可解析、plant 不被 GC |
| 运动学 | `kinematics/mdh.py` / `kinematics/analytic_ik.py` | 297 / 721 行 | 纯 numpy 正解（对照基准）与闭式解析 IK（8 分支 × ψ） |
| 场景 | `planning/scene.py` / `bimanual/dual_scene.py` | 239 / 483 行 | 单臂/双臂场景装配、障碍物注册、碰撞查询 |
| 规划 | `planning/ik.py` / `regions.py` / `gcs.py` / `retime.py` / `trajectory.py` / `planner.py` | 632 / 951 / 760 / 951 / 606 / 913 行 | IK → 区域 → GCS → 重定时 → 轨迹 → 计划结果 |
| 协调规划 | `coordinated/parameterization.py` / `regions.py` / `planner.py` / `trajectory.py` / `limits.py` | 613 / 1405 / 1319 / 217 / 310 行 | R⁸ 最小坐标、自研凸区域、紧协调求解、共享时间轴 |
| 设备层 | `device/safety.py` / `exec_loop.py` / `state_machine.py` / `recovery.py` / `sdk.py` / `transport.py` / `config.py` / `state_source.py` / `metrics.py` / `run_log.py` / `realtime.py` / `binding.py` / `hardware_gate.py` / `hardware_lock.py` | 950 / 667 / 662 / 1027 / 604 / 1049 / 499 / 382 / 305 / 673 / 400 / 369 / 159 / 129 行 | 七闸、5 ms 环、状态机、恢复、SDK 审计、传输层、配置、状态源、指标、日志、UDP、解码、门控、独占锁 |
| 任务层 | `tasks/primitives.py` / `solver.py` / `state_machine.py` / `perception.py` / `metrics.py` / `bimanual.py` / `schema.py` | 225 / 915 / 541 / 486 / 489 / 806 / 712 行 | 原语目录、单一求解器、任务状态机、感知总线、指标、搬运/交接、schema |
| 回归与验证 | `regression/golden.py` / `session.py` / `validation/ladder.py` / `validation/hardware.py` | 804 / 463 / 846 / 697 行 | 三指纹 golden、会话产物、L0–L14 阶梯、真机执行器 |
| CLI 与工具 | `cli.py` / `scripts/*` / `tools/*` | 2617 行 + 19 脚本 | 统一 CLI 族与全部测量/取证脚本 |

---

## 附录 B：外部资料引用清单

| # | 资料 | URL | 本文中用于 |
|---|---|---|---|
| B1 | Drake 官网首页（定位与设计哲学） | https://drake.mit.edu/ | 思想 1、2、3、11 的官方原话："Model-Based Design and Verification for Robotics"、"commands go in, sensors come out"、暴露方程结构 |
| B2 | Drake 官方教程：Working with Diagrams | https://github.com/robotlocomotion/drake/blob/master/tutorials/working_with_diagrams.ipynb | 思想 4：System / Diagram / Context 与"确定性可重复"承诺 |
| B3 | Drake 官方教程：Authoring Multibody Simulation | https://github.com/robotlocomotion/drake/blob/master/tutorials/authoring_multibody_simulation.ipynb | 思想 5、6、7：`AddMultibodyPlantSceneGraph`、`Parser.AddModels(package://…)`、`WeldFrames` → `Finalize` → `plant_context` |
| B4 | Drake 官方教程：Rendering a MultibodyPlant（DiagramBuilder） | https://github.com/robotlocomotion/drake/blob/master/tutorials/rendering_multibody_plant.ipynb | 思想 6：RobotDiagram = Diagram + MultibodyPlant + SceneGraph |
| B5 | Drake Release Notes v1.16.0（GCS 首发） | https://github.com/robotlocomotion/drake/blob/master/doc/_release-notes/v1.16.0.md | 思想 8：GCS 首次发布及其定位 |
| B6 | 论文：Motion Planning around Obstacles with Convex Optimization（arXiv:2205.04422） | https://arxiv.org/abs/2205.04422 | 专题 7/10：凸松弛紧、廉价舍入、最优性界；**官方演示含 14-DoF 双臂** |
| B7 | Drake Release Notes v1.27.0 / v1.32.0（IRIS 与 GCS 演进） | https://github.com/robotlocomotion/drake/blob/master/doc/_release-notes/v1.27.0.md | 专题 6：`IrisInConfigurationSpaceFromCliqueCover` 等区域生成器谱系 |
| B8 | pydrake.common.yaml（schema 化 YAML 序列化） | https://drake.mit.edu/pydrake/pydrake.common.yaml.html | 思想 9：`yaml_dump_typed` / `yaml_load_typed` / `SaveYamlFile` |
| B9 | 官方 bimanual 示例仓库（ICRA 2024）**cohnt/constrained-bimanual-planning-example**（MIT） | https://github.com/cohnt/constrained-bimanual-planning-example | 专题 6/7/10/15：解析 IK 最小坐标 → **约束流形被彻底消元**、自动微分贯穿参数化、轨迹**构造性**满足约束、IRIS-NP2/IRIS-ZO 在参数化空间长区域、GCS 轨迹优化 + Toppra 重定时；README 亦说明 C++ 化后“区域生成降到约 1.2 秒” |
| B14 | 论文：Constrained bimanual planning with analytic inverse kinematics（Cohn / Shaw / Simchowitz / Tedrake, ICRA 2024, pp. 6935–6942） | https://ieeexplore.ieee.org/abstract/document/10610675 | 专题 10：最小坐标方法论的正式出处（README 引用格式亦按此） |
| B15 | 论文：Faster Algorithms for Growing Collision-Free Convex Polytopes in Robot Configuration Space（Werner / Cohn / Jiang / Seyde / Simchowitz / Tedrake / Rus, arXiv:2410.12649, ISRR 2024） | https://arxiv.org/abs/2410.12649 | 专题 6/15：IRIS-NP2 / IRIS-ZO 的算法出处 |
| B16 | 论文：Faster Algorithms for Growing Collision-Free Regions of Constrained Bimanual Configuration Spaces（Cohn / Werner / Tedrake, IROS 2025  workshop） | https://github.com/cohnt/constrained-bimanual-planning-example/blob/master/other/iris_iros_workshop.pdf | 专题 15：**在最小坐标里用 IRIS** 的专门工作，直接对应本仓 §五 缺口 #9 |
| B10 | Drake API 文档首页（Python / C++） | https://drake.mit.edu/pydrake/ 、 https://drake.mit.edu/doxygen_cxx/ | 各类 API 事实核对（IrisNp、GraphOfConvexSets、CollisionCheckerParams 等） |
| B11 | 睿尔曼 RM_API2（Python 包 `robotic-arm==1.1.7`） | https://github.com/RealManRobot/RM_API2 | 厂商接口事实：`rm_movej_canfd` 透传语义、返回码约定、UDP 主动上报 |
| B12 | 睿尔曼官方开发者中心：RM75 参数与 MDH | https://develop.realman-robotics.com/en/robot4d/robotParameter/RM75OntologyParameters/ （`robot4th` 别名同样可达） | MDH 真值（d1=240.5, d3=256, d5=210, d7=144 mm）、`a_{i-1}` 全 0 ⇒ 闭式解析 IK 可解 |
| B13 | 官方两指夹爪开源工程（WHJ03） | https://github.com/RealManRobot/Two-Finger-Gripper | 夹爪事实：65 mm 开口、Modbus RTU、许可为 OpenAtom Open Hardware License 1.0 |

> 访问日期均为 2026-11-23。B13 的上游仓库在本次收口轮实测已 **404**（GitHub 上消失）；归档改由镜像 `Simo-realman/Two-Finger-Gripper` 的同一 pin 提交 `91f587b1d4ef` 物化，产物 sha256 `975b5474afba…` 与 `third_party/MANIFEST.yaml` 登记值逐字一致，**镜像回退机制尚未进入 `tools/fetch_upstream.py`**（已列入 M002-S00）。

---

*本文所有代码位置均以 M001-acm1jg 收口时的仓库状态（`feat-drake` @ `8c07f0d`）为准；所有真机数字可在 `docs/evidence/` 下按文件名复算，所有离线数字可用 §六.4 的命令复算。*
