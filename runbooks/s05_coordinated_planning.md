# S05 双臂紧协调规划作业手册（RM75-D 双臂）

> **适用范围**：S05「双臂相对位姿固定」的紧协调规划链。
>
> **本文件当前状态（阶段性）**：本文件由 **S05-T02** 先行落笔，只覆盖
> **限位口径 / 共享时间轴 / 夹爪自由度口径 / 重定时维度泛化** 四项**约定**。
> 完整的「验收口径 vs 端到端口径 / 失败模式清单 / 未验证物理指标 /
> C++ 下沉路线」四节由 **S05-T07** 补齐（不得在本文件里出现未经实测的
> 物理达标结论）。
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

<!-- S05-T07 将补齐：验收口径 vs 端到端口径、失败模式清单、未验证物理指标
     清单与解锁条件、C++（pybind11）下沉路线、区域缓存版本门禁运维。 -->
