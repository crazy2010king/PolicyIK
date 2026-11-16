# S05 双臂紧协调规划作业手册（RM75-D 双臂）

> **适用范围**：S05「双臂相对位姿固定」的紧协调规划链。
>
> **本文件当前状态**：本文件由 **S05-T02** 先行落笔（限位口径 / 共享时间轴 /
> 夹爪自由度口径 / 重定时维度泛化），并由 **S05-T07** 补齐
> **验收口径 vs 端到端口径 / 失败模式清单 / 未验证物理指标清单 /
> C++ 下沉路线 / 运维** 五节（不得在本文件里出现未经实测的物理达标结论）。
>
> **诚实前提**：本仓库当前**没有真机、没有实测数据**。所有真实物理指标
> （真实成功率 ≥95%、真实求解 P95 <3 s、真实共持位姿漂移 <2 mm/10 s、
> 真机零逃逸）**一律未验证**，由 S05-T06/T07 逐条列入 `unverified_metrics`。

---

## 0. 真相源一览

| 内容 | 真相源 |
|---|---|
| 单臂软件限位（位置 / 速度） | `models/rm75_drake/joint_limits.yaml`（S01 交接件，**协调层不得修改**） |
| 自研默认加速度上限 | `rm75_drake.planning.trajectory.DEFAULT_ACCELERATION_LIMITS_RAD_S2` |
| 7 维轨迹不变量 | `rm75_drake.planning.trajectory.Trajectory`（S02 冻结交接面） |
| 双臂 14 维限位派生 | `rm75_drake.coordinated.limits.derive_dual_limits` |
| 共享时间轴双轨迹 | `rm75_drake.coordinated.trajectory.DualTrajectory` |
| 维度无关重定时核 | `rm75_drake.planning.retime.retime_path`（与 7 维 `retime` **共用**一份 TOPPRA 包装） |
| 双臂场景 / 关节名顺序 | `scenarios/dual_arm.yaml`（左 `left_joint_1..7` → 右 `right_joint_1..7`） |

---

## 1. 双臂 14 维限位口径（S05-T02）

* **同一台机器人 ⇒ 两臂逐项相同**：14 维限位 = S01 单臂真值的**程序派生**
  （左 7 + 右 7），**不另写一份 YAML**（避免第二个真相源）。
* 位置软限位 / 速度上限来自 `joint_limits.yaml` 的 `software` 段；
  **加速度上限**沿用 S02 的**自研默认值**（J1/J2 = 6.0、J3–J7 = 8.0 rad/s²），
  **不另起数值**。
* **按 plant 的实际 `joint_names` 顺序装配**，且断言名字形如
  `left_joint_i` / `right_joint_i`（i = 1..7）。数量错 / 名字形态错 /
  顺序错配 ⇒ `CoordinatedLimitError`（标签 `[joint_count]` / `[joint_names]`），
  错误信息**点名期望与实际**，**不静默纠正**。

```python
from rm75_drake.coordinated.limits import derive_dual_limits
limits14 = derive_dual_limits(bundle.joint_names)   # bundle: bimanual.dual_scene.DualSceneBundle
```

## 2. 共享时间轴口径（S05-T02）

* **不新建 14 维轨迹类型**：14 维联合路径重定时后**拆成两个 7 维**
  `Trajectory`（`trajectory_from_arrays`），组成 `DualTrajectory`。
* **共享时间轴由构造保证**：`DualTrajectory` 构造期断言两条轨迹的
  `t_abs` / `dt` **逐点相同**（标签 `[time_axis]`）；每条 7 维轨迹各自走一遍
  S02 的全部不变量（软限位 / 速度 / 加速度 / seq / 时间网格）——
  **零重复实现**，不存在「第二份不变量校验」。
* 14 维列顺序 = **左臂 7 → 右臂 7**（与 plant 的位置变量顺序一致）。

```python
from rm75_drake.coordinated.trajectory import dual_trajectory_from_arrays
dual = dual_trajectory_from_arrays(q14_seq, t0=0.0, dt=0.005)
assert dual.left.dt == dual.right.dt
```

## 3. 重定时维度泛化（S05-T02）

* `rm75_drake.planning.retime` 是 **S05-T02 对 S02 冻结模块的唯一改动点**：
  抽出**维度无关的通用核**（接受任意维度的 `q_seq` 与逐轴上下限数组）。
* `retime()` / `retime_from_trajectory()` 的**公开签名与 7 维行为逐字不变**
  （默认仍用 `software_limits()` 与 `DEFAULT_ACCELERATION_LIMITS_RAD_S2`）；
  **零回归**由 `tests/data/retime_golden.npz`（重构前录下的逐点 golden）钉住。
* 14 维入口：`retime_path(q14, velocity_limits=..., acceleration_limits=...)`
  ——**禁止新建第二套 TOPPRA 包装**；非 7 维必须**显式**给出上下限
  （否则 `[limits]` 报错，不静默套用单臂数值）。

## 4. 夹爪自由度口径（S05-T02，**查询碰撞前必须执行**）

* 场景装夹爪时 `plant.num_positions() == 18`（双臂 14 + 两夹爪各 2 个
  prismatic）；**协调规划只产出 14 维臂关节**。
* **plant 的位置变量顺序** = model instance 顺序 =
  **左臂 7 → 右臂 7 → 左夹爪 2 → 右夹爪 2**
  （见 `bimanual/dual_scene.py` 的装配顺序：两臂先 `AddModels`，夹爪随后）。
* 因此把 14 维配置交给 Drake 做碰撞查询**之前**，必须按上述顺序补齐夹爪
  prismatic（固定开度或当前值）：

```python
from rm75_drake.coordinated.limits import pad_gripper_positions
q18 = pad_gripper_positions(
    q14,
    gripper_joint_names=bundle.gripper_joint_names,   # 左夹爪 2 → 右夹爪 2
    gripper_value=0.0,                                 # 固定开度（m）
)
```

* S04 的 `include_grippers_in_collision` 默认 **`False`**：夹爪默认**不**进入
  碰撞判定；置 `True` 时 q **必须**包含这 4 个 prismatic（否则夹爪几何不随
  q 更新，检不出「手指插进物体」）。
* 维度错 / NaN / 非法开度 ⇒ `CoordinatedLimitError`（标签 `[joint_count]` /
  `[limits]` / `[gripper]`），**不静默补齐**。

---

## 5. 口径定义：验收指标 vs 端到端指标（S05-T07）

> **纪律**：**验收口径**与**端到端口径**必须**分行**报，**不得混用**，
> **不得用合成数据冒充物理达标**（同 `scripts/plan_stats.py` 的做法）。

| 指标 | 定义（口径） | 阈值 | 来源 |
|---|---|---|---|
| **离线成功率** | N 组确定性任务中 `plan_coordinated(...).success == True` 的比例 | **≥ 95%** | R017 |
| **求解阶段 P95** | 参数化 + GCS + 映射回 14 维 + 重定时 四段耗时之和的 P95（**不含区域生成**） | **< 3 s** | R017 |
| **端到端 P95** | 整条链路（含区域生成）的 P95 | **仅记录**，**不作为验收口径** | S02 先例（IRIS 单区域 8–18 s） |
| **相对位姿残差** | 轨迹**逐点**比对 `T_L(q_L)·X_LR` 与 `T_R(q_R)`：位置误差 (m) / 姿态误差 (rad) | 逐点 **≤ 1e-6 m / ≤ 1e-6 rad**（构造性满足，非统计口径） | R006 |
| **零逃逸** | 14 维轨迹密采样（步长 `collision_edge_step_size`）回投 plant 后的逃逸点数 | **必须 = 0**（硬要求） | S05 硬要求 |
| **区域生成单区域耗时** | `diagnostics["regions"]["duration_s"] / num_regions` | 仅记录 | 观测面 |

* **求解阶段**的计时来自 `CoordinatedPlanResult.diagnostics` 的
  `parameterize` / `gcs` / `map` / `retime` 四段 `duration_s` 之和；
  `regions` 段**单独**计入端到端口径。
* **逐点**残差是**构造性满足**的验收口径（不是统计口径）：随机 x →
  `to_joint_config(x)` → `relative_pose(q14)` **逐点** ≤ 1e-6 m / 1e-6 rad。
* 复算命令（退出码 0，**物理指标一律列为未验证**）：

```bash
# 验收口径 + 端到端口径分行报告 + 未验证清单
python3 scripts/coordinated_stats.py --samples 20
# 机器可读
python3 scripts/coordinated_stats.py --samples 20 --json
# 单行 JSON 摘要（status / reason / stage / diagnostics）
PYTHONPATH=src python3 -m rm75_drake.cli bimanual --scenario scenarios/dual_arm.yaml --json
```

## 6. 失败模式清单与排查顺序（S05-T07）

失败**必须点名**：`stage`（阶段）+ `reason`（稳定词表）+ `message`（人话）。
**禁止**返回“看起来正常”的半成品轨迹。

| stage | reason | 含义 | 排查顺序 |
|---|---|---|---|
| `parameterize` | `invalid_input` | 输入非法（leader 维度错 / 含 NaN / leader == follower / X_LR 非法 / 选项参数非法） | 先查参数：`--start-joints` / `--goal-joints` 必须是 7 项有限数；`--leader != --follower`；`--x-matrix` / `--x-xyz` / `--x-rpy` 必须给出合法 4×4 |
| `parameterize` | `unreachable` | follower 的目标 TCP 位姿超出可达域（`d3−d5 ≤ L ≤ d3+d5` 不满足） | 减小 leader 目标位移幅度（`--goal-step`）或换 `--leader`/`--follower` 指派；确认 `X_LR` 不是离谱的平移 |
| `parameterize` | `branch_exhausted` | 8 组镜像分支 × ψ 网格内**没有**满足限位的解 | 放宽 ψ 网格（`--psi-grid`）或换更靠内的起点；确认起点本身可行（`select_bimanual_start`） |
| `parameterize` | `limit_violation` | 解存在但超出软件限位 | 看 `diagnostics` 里 follower 越界轴；换更保守的 leader 目标 |
| `parameterize` | `wrist_singular` | 腕奇异（`sin q6 ≈ 0`，Z-X-Z 分解退化） | 避开奇异位形：微调 leader 目标或换分支 |
| `parameterize` | `path_infeasible` | 起点可行但 leader 路径推进过程中 follower 无解 | 缩小步长（`--path-step`）或缩小目标位移 |
| `regions` | `empty_region` | 最小坐标凸区域为空（保守收缩后已知自由集也空了） | 看 `diagnostics["regions"]`：种子是否本身碰撞自由；缩小 `--path-step` 以抬高种子密度 |
| `regions` | （`degraded=True` + `degradation_reason`） | 区域链不完整 ⇒ **降级**到采样基线（同一套 oracle） | **降级是显式标注的**：查 `diagnostics["degraded"]` / `["degradation_reason"]`；若需硬失败，加 `--no-degradation` |
| `gcs` | `no_path` | 区域图上无路（且采样基线也失败） | 看 `num_regions` / 区域数量是否足够；放宽任务位移或增大 `--num-regions` |
| `retime` | （重定时错误标签） | TOPPRA 重定时失败（速度 / 加速度不可行） | 缩小位移或放慢 `dt`；确认加速度上限仍是 S02 默认值 |
| `collision` | `collision_escape` | 14 维密采样复检发现逃逸（**硬失败**） | 减小 `--collision-edge-step-size` 找真实冲突点；调整 `X_LR` 或任务位移 |
| `collision` | `constraint_violation` | 相对位姿残差超阈值 | 检查 `X_LR` 是否与标定一致（见 §9）；检查分支/ψ 切换次数（`diagnostics["branch_switches"]`） |

**降级链三级**（与 CONTEXT 的降级策略链一致，**禁止静默降级**）：

1. **区域生成成功** ⇒ 直接建图走 GCS（正常路径）。
2. **区域生成失败 / 为空** ⇒ `degraded=True` + `degradation_reason` ⇒ **采样基线**
   （最小空间 PRM/RRT，**同一套纯 numpy oracle**）+ 局部精修。
3. **采样基线也失败** ⇒ `success=False` + `stage` / `reason` / `message`，
   **不返回任何轨迹**。

**排查顺序总则**：`stage` → `reason` → `diagnostics`（区域数、各阶段耗时、
残差 max、逃逸数、`branch_switches` / `psi_switches`）→ 场景/标定真相源。

## 7. **未验证**物理指标清单与解锁条件（S05-T07）

> 本仓库**无真机、无实测数据**。下列指标**一律未验证**，由
> `rm75_drake.cli.UNVERIFIED_COORDINATED_METRICS` 与
> `scripts/coordinated_stats.py` **逐条列出**（硬件门控 SKIP，**退出码 0**），
> **不得用合成数据冒充**。

| # | 未验证指标 | 离线现状 | 解锁条件 |
|---|---|---|---|
| 1 | 真实（真机）双臂紧协调规划成功率 **≥ 95%** | 离线仿真成功率（`coordinated_stats.py --samples`） | 接入真机 + `devices/` 通道，按 S03 采样作业采集 N 组真实任务后复算 |
| 2 | 真实（真机）协调**求解阶段 P95 < 3 s** | 离线求解阶段 P95（`acceptance_metrics.solve_p95_s`） | 同上；需在**目标控制机**上实测（含真实模型版本） |
| 3 | 真实（真机）共持位姿漂移 **< 2 mm / 10 s** | **未验证**（离线无漂移概念） | 真机共持 10 s + 外部测量（激光跟踪 / 标定靶） |
| 4 | 真实（真机）双臂相对位姿保持 | 仅**仿真内**构造性满足（逐点 ≤1e-6 m / rad） | 真机实测末端相对位姿（同一测量手段） |
| 5 | 真机 **14 维零逃逸** | 离线仿真零逃逸（`num_escapes == 0`） | 真机执行前后的真实碰撞复检（离线零逃逸 ≠ 真机零逃逸） |

**解锁流程**：真机到位 ⇒ 走 S04 的实测采样作业 ⇒ 写回
`calibration/dual_arm.yaml` ⇒ 复算上述 1/2 项 ⇒ 把对应条目从
`UNVERIFIED_COORDINATED_METRICS` 移除并**保留复算命令**。

## 8. C++（pybind11）下沉路线（S05-T07，**本切片显式延期**）

* **为什么延期**：ROADMAP 里“参数化下沉 C++ 后区域生成 ≤ 2 s”需要**仓库内
  C++ 构建链**（pybind11 / CMake / 编译器工具链），本仓库当前**无构建链**
  （见决策 **D031**）⇒ 本切片**不**新建构建体系，**不**伪造性能结论。
* **热循环接口（已经是纯 numpy 批量判定，可机械移植）**：

  ```python
  # rm75_drake.coordinated.regions.make_parameterization_oracle / make_numpy_oracle
  oracle(x_batch: np.ndarray) -> np.ndarray   # 输入一批 R^8 点，输出 free/busy 布尔数组
  ```

  * **契约**：`oracle` 是**纯 numpy 函数**（输入一批最小坐标点，输出布尔数组），
    **不依赖 Drake 类型**、不读文件、不带状态 ⇒ 可 1:1 映射为 pybind11 函数
    （`py::array_t<double>` → `py::array_t<bool>`）。
  * **移植顺序**：① `oracle`（碰撞判定）→ ② `_direction_set` / `_extent_along`
    （边界探测）→ ③ `generate_minimal_region`（支撑不等式 + 半空间切割）→
    ④ `sample_baseline_path`（PRM/RRT）。
  * **接口不变性要求**：移植后 Python 侧的**函数签名与返回布局逐字不变**，
    `tests/test_coordinated_regions.py` 的**保守性 / 可复现 / 缓存**断言
    必须在 C++ 版本上**原样通过**（用同一组种子做交叉比对）。
* **风险与护栏**：C++ 版本与 numpy 版本**并存**，由同一套单测交叉验证；
  未通过交叉验证前**不得**切换默认后端。

## 9. 运维（S05-T07）

### 9.1 区域缓存的版本门禁与清理

* 缓存类：`rm75_drake.coordinated.regions.MinimalRegionCache`
  （**另立类名**，不改 S02 冻结的 `planning/regions.py` API）。
* 落盘格式：YAML（自研：`schema_version` / `model_version` / `generator` / `regions`），
  入口 `save_cache(path)` / `load_cache(path, strict=...)`。
* **版本门禁**：缓存里的 `model_version` 与当前场景的
  `minimal_model_version(bundle)` 不一致 ⇒ 陈旧条目被**丢弃**
  （观测字段 `stale_entries_dropped` / `version_mismatch`），
  随后的查询**明确 miss 并重新生成** ⇒ **绝不复用陈旧区域**。
* **观测字段**：`hit_count` / `miss_count` / `generation_count` /
  `stale_entries_dropped` / `version_mismatch`。
* **清理方式**（按需）：
  * 只想丢弃陈旧条目 ⇒ 重新 `load_cache`（版本门禁自动丢弃）或直接删缓存文件；
  * 全部重建 ⇒ 删除缓存 YAML 后重跑（区域生成会重新落盘）；
  * **模型几何变更后**（URDF/URDF 变体重生成）⇒ `model_version` 变化 ⇒
    旧缓存自动失效，**不需要**手工清理。
* **回归护栏**：`python3 tools/gen_dual_arm_models.py --check`（退出码 0）
  确认双臂几何**未漂移**；`bash scripts/ci.sh fast` 的
  `tests/bimanual-coordination` 项失败即整体失败。

### 9.2 标定文件对 `X_LR` 的影响（唯一真相源）

* `calibration/dual_arm.yaml`（S04 产出）是**双臂基座外参 `X_LR` 的唯一真相源**；
  `scenarios/dual_arm.yaml` 的场景外参**必须等于**它。
* **影响链**：`X_LR` 错 ⇒ follower 的目标 TCP 位姿错 ⇒ 解析 IK 失败
  （`unreachable` / `branch_exhausted` / `limit_violation`）或相对位姿残差超阈
  （`constraint_violation`）⇒ **规划不可行**。
* 因此**标定一变就要复算**：
  `python3 scripts/calibrate_dual_arm.py`（或 S04 的复算命令）→
  `python3 tools/gen_dual_arm_models.py --check` →
  `bash scripts/ci.sh fast`。
* CLI 里给的 `--x-xyz` / `--x-rpy` / `--x-matrix` 是**显式覆盖**（便于实验），
  默认值是**占位值**（`bimanual` 输出会显式标注“占位”）⇒ **不得**把占位值
  当成实测外参汇报。

### 9.3 常驻回归门禁

* `scripts/ci.sh` **fast 档**新增**常驻**项 `tests/bimanual-coordination`
  （用 `run_check`，**失败即整体失败**，**不得**降级成 `run_check_skippable`）：

  | 测试文件 | 覆盖 |
  |---|---|
  | `tests/test_analytic_ik.py` | RM75 解析 IK（分支 / ψ / 失败路径 / 零 pydrake 依赖） |
  | `tests/test_coordinated_limits.py` | 双臂 14 维限位派生与 joint_names 断言 |
  | `tests/test_coordinated_trajectory.py` | 共享时间轴双轨迹与不变量 |
  | `tests/test_coordinated_parameterization.py` | 最小坐标参数化（构造性满足 / 分支连续性） |
  | `tests/test_coordinated_regions.py` | 最小坐标凸区域 + 缓存版本门禁 + 降级 |
  | `tests/test_coordinated_planner.py` | 协调规划端到端（零逃逸 / 残差 / 失败点名） |
  | `tests/test_coordinated_stats.py` | 指标脚本（验收 / 端到端分行 / 未验证清单） |
  | `tests/test_coordinated_cli.py` | `bimanual` CLI（单行 JSON / 退出码语义） |

* 本套件**零硬件、零真机**即可真实回归 ⇒ **恒跑**，不是“可选”。
* 门禁自身由 `tests/test_ci_script.py` 守住（新项用 `run_check`、不得出现
  `run_check_skippable`、`--help` 必须提到该套件）。

