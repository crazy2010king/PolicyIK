"""双臂搬运 / 交接原语（S06-T05，常数 ``X_LR``）。

为什么**不新建规划器**
--------------------
S05（:mod:`rm75_drake.coordinated`）已经把「双臂末端相对位姿固定」变成
**构造性成立**的事：常数 ``X_LR``（leader TCP 系 → follower TCP 系）+ 8 维
最小坐标参数化 ⇒ 逐点相对位姿残差恒为机器精度量级（~1e-8 m / rad）。
而「共同搬运 ⇒ 两夹爪相对位姿恒定 ⇒ 常数 ``X_LR``」正是搬运 / 交接的
数学底座 ⇒ **搬运与交接原语只做编排**，一律复用
:func:`rm75_drake.coordinated.planner.plan_coordinated`
（**不写第二套协调规划**，也不改它的公开签名）。

``X_LR`` 的语义（必须与 S05 一致）
--------------------------------
``X_LR`` 是 **leader TCP 系 → follower TCP 系** 的常数变换，即

.. code-block:: text

    T_follower_world = T_leader_world · X_LR

当 ``leader="left"`` / ``follower="right"`` 时，它就是计划书所说的
**TCP_L→TCP_R**。角色互换（leader=right）后，同一几何关系由
``numpy.linalg.inv(X_LR)`` 表达 ⇒ **交接点两 TCP 的相对位姿仍然构造性保持**。

本模块的两个入口
----------------
* :func:`plan_carry` —— **单段共同搬运**：leader 按 ``X_LR`` 带着 follower
  从起点走到终点（leader 的关节路径是规划变量，follower 由解析 IK 跟随）。
* :func:`plan_handover` —— **两阶段交接**：
  阶段 1（leader = 初始 leader）用 ``X_LR`` 走到交接位；
  阶段 2（leader / follower **角色互换**）用 ``inv(X_LR)`` 走后续段。
  交接点处两 TCP 的相对位姿残差必须 ≤ 1e-6（与 S05 的 R006 口径一致），
  超差即 ``reason="constraint_violation"``。

关节角的坐标系约定（**必读**）
------------------------------
* :func:`plan_carry` 的 ``start_leader_q`` / ``goal_leader_q`` 都在
  **leader 臂**的关节空间里（默认左臂）。
* :func:`plan_handover` 的 ``start_leader_q`` / ``handover_leader_q`` /
  ``goal_leader_q`` **全部在「初始 leader 臂」的关节空间里**（默认左臂）。
  阶段 2 的 leader 是**另一条臂**（默认右臂），它在交接位与终点的关节角
  由本模块**内部用解析 IK 推导**（种子 = 交接位的 follower 解 ⇒ 分支连续），
  因此调用方**不需要**自己去解另一条臂的 IK。

携带物的表达纪律（**必读，不许假装已实现随动几何**）
--------------------------------------------------
pydrake 1.51.1 的 ``SceneGraph`` **没有 ``SetGeometryPose``**
（只有 ``RegisterAnchoredGeometry`` / ``RegisterGeometry`` /
``RegisterDeformableGeometry`` / ``RemoveGeometry`` / ``RenameGeometry``）
⇒ **被抓物无法在场景图里随 TCP 移动**。因此本切片采用**任务级不变式**
路线：

* **携带物不进入碰撞几何**（不注册进 ``SceneGraph``）；
* 携带的可靠性由「**两臂相对位姿约束 + leader 承载**」保证 —— 即
  ``X_LR`` 恒定 ⇒ 两夹爪的相对位姿恒定 ⇒ 被夹持物的相对位姿恒定；
* 本模块**不**假装已实现「物体随 TCP 移动的随动几何」，也**不**把它写进
  任何碰撞判定路径。

交接点的连续性口径（**必读：TCP 级严格连续，关节级可能跳变**）
----------------------------------------------------------
1. **相对位姿**：交接点两 TCP 的相对位姿在「角色互换」约定（``inv(X_LR)``）
   下仍然**构造性保持**（残差 ~1e-16 m / rad，阈值 1e-6）。
2. **TCP 连续性**：阶段 2 的起点与阶段 1 的终点**同一时刻的 TCP 位姿完全
   相同**（偏差 ~1e-16 m）⇒ 被携带物在交接瞬间**不发生任何位移**。
3. **关节级跳变（已知限制，不掩饰）**：阶段 2 里「初始 leader 臂」变成了
   **follower**，而 follower 的位形由解析 IK 按 **ψ 网格**求解 ⇒ 交接瞬间
   该臂可能出现**镜像分支 / ψ 量化的关节跳变**（实测最大 ~5 rad，TCP 不动）。
   根因：S05 的 ``plan_coordinated`` **公开 API 不暴露镜像分支 / ψ 连续性
   入口**，follower 的分支在其首次求解时按「限位余量最大」选取 ⇒ 本切片
   **不假装**该跳变不存在，而是**量化并上报**
   （``diagnostics["switch_joint_discontinuity_max_rad"]`` /
   ``diagnostics["switch_tcp_discontinuity"]``）。
   工程含义：交接点应被当作一次**离散的「重新抓取」事件**处理（新 leader
   承载，旧 leader 重新就位），而不是一条连续平滑的关节轨迹。

零 pydrake 依赖
---------------
本模块顶层只 import ``numpy`` + 标准库；Drake 侧的一切（基座位姿、碰撞）
都经由 :mod:`rm75_drake.coordinated.parameterization` **惰性**取用 ⇒
``import rm75_drake.tasks.bimanual`` **不会**把 pydrake 拉进 ``sys.modules``。

依赖方向（硬约束）
------------------
本模块**不** import :mod:`rm75_drake.drivers`（P002 边界纪律），
只调 :mod:`rm75_drake.coordinated.planner` /
:mod:`rm75_drake.coordinated.parameterization` /
:mod:`rm75_drake.coordinated.trajectory` /
:mod:`rm75_drake.kinematics.analytic_ik`。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "BimanualPlanError",
    "CarryPlan",
    "HandoverPlan",
    "plan_carry",
    "plan_handover",
    "RESIDUAL_TOLERANCE",
    "REASON_INVALID_INPUT",
    "REASON_CONSTRAINT_VIOLATION",
]


#: 逐点相对位姿残差阈值（m / rad）—— 与 S05 的 R006 口径**一致**。
RESIDUAL_TOLERANCE = 1e-6

#: 失败标签（稳定词表）。
REASON_INVALID_INPUT = "invalid_input"
REASON_CONSTRAINT_VIOLATION = "constraint_violation"

#: 单臂关节数（7 DoF）。
ARM_JOINT_COUNT = 7

#: 双臂联合关节数（14 DoF）。
DUAL_JOINT_COUNT = 2 * ARM_JOINT_COUNT

#: 合法侧别。
_VALID_SIDES = ("left", "right")


# ── 错误 ────────────────────────────────────────────────────────────────
class BimanualPlanError(RuntimeError):
    """**用法 / 输入**错误（失败必须点名）。

    ``stage`` / ``reason`` 是稳定标签，便于调用方分支处理：

    * ``stage="input"``、``reason="invalid_input"`` —— 非法 ``X_LR``
      （形状非 4×4 / 奇异 / 镜像 / 含 NaN）、关节角维度错、侧别非法；
    * ``stage="ik"`` —— 阶段 2 的目标 IK 无解（``reason`` 沿用解析 IK 词表）。
    """

    def __init__(
        self,
        message: str,
        *,
        stage: str = "input",
        reason: str = REASON_INVALID_INPUT,
    ) -> None:
        super().__init__(message)
        self.stage = str(stage)
        self.reason = str(reason)


# ── 输入规整 ────────────────────────────────────────────────────────────
def _side_slice(side: str) -> slice:
    """14 维联合向量的侧别切片（左臂恒在前 7 维）。"""
    return slice(0, ARM_JOINT_COUNT) if side == "left" else slice(ARM_JOINT_COUNT, DUAL_JOINT_COUNT)


def _as_transform(
    matrix: Any,
    *,
    name: str = "X_LR",
    require_invertible: bool = False,
) -> np.ndarray:
    """把输入规整成 4×4 齐次矩阵；非法输入**点名报错**。

    校验顺序（错误信息按此顺序点名）：形状 → 有限性 → 最后一行 →
    **可逆性** → 旋转正交性 → 旋转行列式符号。
    """
    try:
        array = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError) as exc:
        raise BimanualPlanError(
            f"[transform] {name} 必须是 4×4 数值矩阵：{exc}"
        ) from exc
    if array.shape != (4, 4):
        raise BimanualPlanError(
            f"[transform] {name} 必须是 4×4 齐次矩阵，实际形状 {array.shape}"
        )
    if not np.all(np.isfinite(array)):
        raise BimanualPlanError(f"[transform] {name} 含 NaN/Inf")
    if not np.allclose(array[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9):
        raise BimanualPlanError(
            f"[transform] {name} 的最后一行必须是 [0, 0, 0, 1]，"
            f"实际：{array[3].tolist()}"
        )
    determinant = float(np.linalg.det(array))
    if abs(determinant) < 1e-12:
        raise BimanualPlanError(
            f"[transform] {name} 奇异（det={determinant:.3e}），"
            "无法求逆（交接原语需要 inv(X_LR)）"
        )
    if require_invertible and abs(determinant) < 1e-9:
        raise BimanualPlanError(
            f"[transform] {name} 近似奇异（det={determinant:.3e}），inv(X_LR) 不可靠"
        )
    rotation = array[:3, :3]
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
        raise BimanualPlanError(
            f"[transform] {name} 的旋转部分不是正交阵"
        )
    if float(np.linalg.det(rotation)) < 0.0:
        raise BimanualPlanError(
            f"[transform] {name} 的旋转部分行列式为负（镜像）"
        )
    return array.copy()


def _as_joint_vector(values: Any, label: str) -> np.ndarray:
    """把关节角规整成 7 维 float 数组（非法 ⇒ 点名报错）。"""
    try:
        array = np.asarray(values, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise BimanualPlanError(
            f"[input] {label} 必须是数值序列：{exc}"
        ) from exc
    if array.size != ARM_JOINT_COUNT:
        raise BimanualPlanError(
            f"[dimension] {label} 必须是 {ARM_JOINT_COUNT} 维"
            f"（单臂 {ARM_JOINT_COUNT} 关节），实际：{array.size} 维"
        )
    if not np.all(np.isfinite(array)):
        raise BimanualPlanError(f"[input] {label} 含 NaN/Inf")
    return array


def _check_sides(leader: str, follower: str) -> Tuple[str, str]:
    """校验侧别指派（非法 / 相同 ⇒ 点名报错）。"""
    left = str(leader)
    right = str(follower)
    if left not in _VALID_SIDES or right not in _VALID_SIDES:
        raise BimanualPlanError(
            f"[input] leader / follower 必须是 'left' 或 'right'，实际："
            f"{left!r} / {right!r}"
        )
    if left == right:
        raise BimanualPlanError(
            f"[input] leader 与 follower 不能相同（实际都是 {left!r}）"
        )
    return left, right


def _rotation_angle(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（rad）——``atan2(|skew|, cos)`` 口径。"""
    rel = rot_a.T @ rot_b
    skew = 0.5 * (rel - rel.T)
    sin_theta = float(np.linalg.norm(skew))
    cos_theta = (float(np.trace(rel)) - 1.0) * 0.5
    cos_theta = min(1.0, max(-1.0, cos_theta))
    return float(math.atan2(sin_theta, cos_theta))


# ── 结果类型 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class CarryPlan:
    """一次双臂搬运（carry）段的规划结果。

    字段
    ----
    success:
        是否规划成功。
    message:
        **始终非空**的可读说明。
    reason / stage:
        失败标签（成功时为空串）。
    dual_trajectory:
        成功时的 :class:`rm75_drake.coordinated.trajectory.DualTrajectory`
        （**左臂 / 右臂**各一条 7 维轨迹，共享时间轴）；失败时为 ``None``。
    diagnostics:
        诊断字典（含 ``relative_pose_residual_max_position_m`` /
        ``relative_pose_residual_max_orientation_rad`` / ``num_escapes`` /
        ``degraded`` 四个**口径键**）。
    """

    success: bool
    message: str
    reason: str = ""
    stage: str = ""
    dual_trajectory: Optional[Any] = None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __bool__(self) -> bool:
        """``bool(plan)`` 等价于 ``plan.success``。"""
        return bool(self.success)

    @property
    def relative_pose_residual_max_position_m(self) -> float:
        """逐点相对位姿残差 max（m）。"""
        return float(self.diagnostics.get("relative_pose_residual_max_position_m", 0.0))

    @property
    def relative_pose_residual_max_orientation_rad(self) -> float:
        """逐点相对位姿残差 max（rad）。"""
        return float(
            self.diagnostics.get("relative_pose_residual_max_orientation_rad", 0.0)
        )

    @property
    def num_escapes(self) -> int:
        """14 维碰撞复检的逃逸数（**零逃逸是硬要求**）。"""
        return int(self.diagnostics.get("num_escapes", 0))

    @property
    def degraded(self) -> bool:
        """是否发生**显式降级**（区域链失败 → 采样基线）。"""
        return bool(self.diagnostics.get("degraded", False))


@dataclass(frozen=True)
class HandoverPlan:
    """两阶段交接（handover）的规划结果。

    字段
    ----
    success:
        两个阶段是否都成功。
    message:
        **始终非空**的可读说明。
    reason / stage:
        失败标签（成功时为空串）；交接点残差超差时
        ``reason="constraint_violation"``。
    phase1 / phase2:
        两个阶段的 :class:`CarryPlan`（阶段 2 失败时为 ``None``）。
    x_lr / x_lr_inverse:
        常数相对变换与它的逆（4×4）。
    switch_relative_residual_position_m / switch_relative_residual_orientation_rad:
        **交接点**在「角色互换」约定（``inv(X_LR)``）下的相对位姿残差
        （m / rad）—— 必须 ≤ :data:`RESIDUAL_TOLERANCE`。
    num_escapes:
        两个阶段的碰撞逃逸数之和。
    diagnostics:
        诊断字典（含两阶段的诊断、交接点连续性度量）。
    """

    success: bool
    message: str
    reason: str = ""
    stage: str = ""
    phase1: Optional[CarryPlan] = None
    phase2: Optional[CarryPlan] = None
    x_lr: np.ndarray = field(default_factory=lambda: np.eye(4))
    x_lr_inverse: np.ndarray = field(default_factory=lambda: np.eye(4))
    switch_relative_residual_position_m: float = 0.0
    switch_relative_residual_orientation_rad: float = 0.0
    num_escapes: int = 0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __bool__(self) -> bool:
        """``bool(plan)`` 等价于 ``plan.success``。"""
        return bool(self.success)

    @property
    def degraded(self) -> bool:
        """两个阶段里是否发生过**显式降级**。"""
        return bool(self.diagnostics.get("degraded", False))


# ── 内部工具 ────────────────────────────────────────────────────────────
def _carry_from_result(result: Any, *, dual: Optional[Any] = None) -> CarryPlan:
    """把 :class:`CoordinatedPlanResult` 包成 :class:`CarryPlan`。"""
    trajectory = dual if dual is not None else result.dual_trajectory
    diagnostics: Dict[str, Any] = dict(result.diagnostics)
    diagnostics.setdefault("relative_pose_residual_max_position_m", 0.0)
    diagnostics.setdefault("relative_pose_residual_max_orientation_rad", 0.0)
    diagnostics.setdefault("num_escapes", 0)
    diagnostics.setdefault("degraded", False)
    return CarryPlan(
        success=bool(result.success),
        message=str(result.message),
        reason=str(result.reason),
        stage=str(result.stage),
        dual_trajectory=trajectory if bool(result.success) else None,
        diagnostics=diagnostics,
    )


def _arm_analytic_ik(
    parameterization: Any,
    side: str,
    target_world: np.ndarray,
    *,
    seed: Optional[Sequence[float]] = None,
    psi_count: int,
) -> Any:
    """在**单臂基座系**里做解析 IK（失败由调用方点名）。

    ``target_world`` 是该臂 TCP 在**世界系**下的 4×4 位姿。
    """
    from ..kinematics import analytic_ik

    base = parameterization.base_pose(side)
    target_base = np.linalg.inv(base) @ np.asarray(target_world, dtype=float)
    lower = parameterization.dual_limits.lower_array()[_side_slice(side)]
    upper = parameterization.dual_limits.upper_array()[_side_slice(side)]
    limits = tuple((float(lo), float(hi)) for lo, hi in zip(lower, upper))
    return analytic_ik.solve_analytic_ik(
        target_base,
        seed=None if seed is None else np.asarray(seed, dtype=float),
        branch_policy="nearest",
        psi=None,
        tool_offset=float(parameterization.tool_offsets[side]),
        limits=limits,
        psi_count=max(2, int(psi_count)),
    )


def _side_configs(dual: Any) -> Tuple[np.ndarray, np.ndarray]:
    """从 ``DualTrajectory`` 里取回**左臂 / 右臂**的关节角矩阵。

    14 维联合向量的**槽位**恒为「左 7 + 右 7」（与 plant 的位置变量顺序、
    :mod:`rm75_drake.coordinated.limits` 的装配顺序一致）⇒ ``q_matrix()``
    的前 7 列**永远**是左臂、与 leader / follower 角色**无关**。

    需要注意的是 S05 的**标签**：:func:`rm75_drake.coordinated.planner.plan_coordinated`
    用 ``f"{leader}_arm"`` / ``f"{follower}_arm"`` 给两条轨迹命名，角色互换
    （leader=right）时**标签与内容对不上**（内容仍按侧别排布）⇒ 本模块用
    :func:`_relabel_dual` 重新装配一次。
    """
    q = np.asarray(dual.q_matrix(), dtype=float)
    return q[:, :ARM_JOINT_COUNT], q[:, ARM_JOINT_COUNT:]


def _relabel_dual(
    dual: Any,
    *,
    acceleration_limits: Optional[Sequence[float]],
) -> Any:
    """把 ``DualTrajectory`` 重新按「**左臂 / 右臂**」装配（修 S05 的标签）。

    S05 在 ``leader="right"`` 时会用 ``leader`` / ``follower`` 的名字去标注
    ``DualTrajectory`` 的两条轨迹，而列槽位恒为「左 7 + 右 7」⇒ 角色互换时
    **标签与内容对不上**。本模块按「槽位恒定」的真相重新装配一次
    （**不改动 S05 的公开签名**），并在诊断里留痕
    （``phase2_arm_labels_relabelled``）。
    """
    from ..coordinated.trajectory import dual_trajectory_from_arrays

    left, right = _side_configs(dual)
    return dual_trajectory_from_arrays(
        np.hstack([left, right]),
        t0=float(dual.start_time),
        dt=float(dual.dt),
        left_robot_name="left_arm",
        right_robot_name="right_arm",
        acceleration_limits=acceleration_limits,
    )


# ── 公开入口 ────────────────────────────────────────────────────────────
def plan_carry(
    bundle: Any,
    *,
    x_lr: Any,
    start_leader_q: Sequence[float],
    goal_leader_q: Sequence[float],
    leader: str = "left",
    follower: str = "right",
    options: Optional[Any] = None,
    tool_offsets: Optional[Mapping[str, float]] = None,
) -> CarryPlan:
    """规划**一段共同搬运**（常数 ``X_LR`` 下的紧协调运动）。

    参数
    ----
    bundle:
        :class:`rm75_drake.bimanual.dual_scene.DualSceneBundle`
        （**同一个 plant**；碰撞判定与 S04 / S05 完全一致）。
    x_lr:
        **常数**相对变换（4×4，leader TCP 系 → follower TCP 系）。
    start_leader_q / goal_leader_q:
        **leader 臂**的 7 维关节角（rad）——起点与终点。
    leader / follower:
        指派（``"left"`` / ``"right"``），必须不同。
    options:
        :class:`rm75_drake.coordinated.planner.CoordinatedPlanOptions`
        （``None`` → 该类的默认值）。
    tool_offsets:
        两侧各自的工具偏置（m，沿 J7 轴）；``None`` → 裸臂默认值。

    返回
    ----
    :class:`CarryPlan`：成功时含 :class:`DualTrajectory` 与诊断
    （``relative_pose_residual_max_position_m`` /
    ``relative_pose_residual_max_orientation_rad`` / ``num_escapes`` /
    ``degraded``）；失败时 ``success=False`` 且带 ``stage`` / ``reason``。

    失败模式
    --------
    * **用法错误**（非法 ``X_LR`` / 关节角维度错 / 侧别非法）⇒ 抛
      :class:`BimanualPlanError`（**点名**）；
    * **规划失败**（follower 无解 / 区域为空 / 无路 / 重定时失败 /
      碰撞逃逸 / 残差超差）⇒ 返回 ``success=False``，``stage`` /
      ``reason`` 沿用 S05 的稳定词表。
    """
    from ..coordinated.planner import plan_coordinated

    matrix = _as_transform(x_lr)
    left, right = _check_sides(leader, follower)
    start = _as_joint_vector(start_leader_q, "start_leader_q")
    goal = _as_joint_vector(goal_leader_q, "goal_leader_q")

    result = plan_coordinated(
        bundle,
        x_lr=matrix,
        start_leader_q=start,
        goal_leader_q=goal,
        leader=left,
        follower=right,
        tool_offsets=tool_offsets,
        options=options,
    )
    return _carry_from_result(result)


def plan_handover(
    bundle: Any,
    *,
    x_lr: Any,
    start_leader_q: Sequence[float],
    handover_leader_q: Sequence[float],
    goal_leader_q: Sequence[float],
    leader: str = "left",
    follower: str = "right",
    options: Optional[Any] = None,
    tool_offsets: Optional[Mapping[str, float]] = None,
) -> HandoverPlan:
    """规划**两阶段交接**（角色互换 + ``X_LR`` ↔ ``inv(X_LR)``）。

    阶段 1
        leader = 初始 leader（默认左臂），用 ``X_LR`` 从 ``start_leader_q``
        走到 ``handover_leader_q``（交接位）。

    阶段 2
        **leader / follower 角色互换**：leader = 另一条臂（默认右臂），
        用 ``inv(X_LR)`` 从交接位走到终点。
        阶段 2 的 leader 关节路径由本模块**内部推导**：交接位处 follower 的
        解（来自阶段 1）+ 终点处由 ``T_leader_goal · X_LR`` 解出的解析 IK
        （种子 = 交接位解 ⇒ 镜像分支连续）。

    关节角坐标系
    ------------
    ``start_leader_q`` / ``handover_leader_q`` / ``goal_leader_q``
    **全部在初始 leader 臂的关节空间里**（默认左臂）。

    返回
    ----
    :class:`HandoverPlan`：含 ``phase1`` / ``phase2`` / ``x_lr`` /
    ``x_lr_inverse`` / ``switch_relative_residual_position_m`` /
    ``switch_relative_residual_orientation_rad`` / ``num_escapes``。

    失败模式
    --------
    * **用法错误** ⇒ 抛 :class:`BimanualPlanError`（``X_LR`` 非法含
      **奇异**情形、关节角维度错、侧别非法）；
    * 阶段 1 失败 ⇒ 返回 ``success=False``（``reason`` / ``stage`` 透传）；
    * 阶段 2 的目标 IK 无解 ⇒ ``stage="ik"``，``reason`` 沿用解析 IK 词表；
    * 阶段 2 规划失败 ⇒ ``reason`` / ``stage`` 透传 S05 的词表；
    * **交接点残差 > 1e-6** ⇒ ``reason="constraint_violation"``。
    """
    from ..coordinated.parameterization import CoordinatedParameterization
    from ..coordinated.planner import CoordinatedPlanOptions, plan_coordinated

    matrix = _as_transform(x_lr, require_invertible=True)
    inverse = np.linalg.inv(matrix)
    left, right = _check_sides(leader, follower)
    start = _as_joint_vector(start_leader_q, "start_leader_q")
    handover = _as_joint_vector(handover_leader_q, "handover_leader_q")
    goal = _as_joint_vector(goal_leader_q, "goal_leader_q")
    opts = options if options is not None else CoordinatedPlanOptions()
    psi_count = max(2, int(getattr(opts, "psi_grid", 17)))

    # ── 阶段 1：初始 leader 带 follower 走到交接位 ─────────────────────
    phase1 = plan_carry(
        bundle,
        x_lr=matrix,
        start_leader_q=start,
        goal_leader_q=handover,
        leader=left,
        follower=right,
        options=options,
        tool_offsets=tool_offsets,
    )
    if not phase1.success or phase1.dual_trajectory is None:
        return HandoverPlan(
            success=False,
            message=f"交接阶段 1 失败：{phase1.message}",
            reason=phase1.reason,
            stage=phase1.stage or "phase1",
            phase1=phase1,
            x_lr=matrix,
            x_lr_inverse=inverse,
            num_escapes=phase1.num_escapes,
            diagnostics={
                "phase1": dict(phase1.diagnostics),
                "phase2": {},
                "degraded": phase1.degraded,
                "num_escapes": phase1.num_escapes,
                "total_duration_s": float(
                    phase1.diagnostics.get("total_duration_s", 0.0)
                ),
            },
        )

    dual1 = phase1.dual_trajectory
    left_end, right_end = _side_configs(dual1)
    q14_switch = np.concatenate([left_end[-1], right_end[-1]])
    leader_end = q14_switch[_side_slice(left)]
    follower_end = q14_switch[_side_slice(right)]

    # ── 交接点残差：角色互换约定（inv(X_LR)）下复核相对位姿 ─────────────
    parameterization = CoordinatedParameterization(
        bundle,
        x_lr=matrix,
        leader=left,
        follower=right,
        tool_offsets=tool_offsets,
    )
    swapped = CoordinatedParameterization(
        bundle,
        x_lr=inverse,
        leader=right,
        follower=left,
        tool_offsets=tool_offsets,
    )
    residual = swapped.relative_pose(q14_switch)
    switch_position = float(residual.position_error)
    switch_orientation = float(residual.orientation_error)

    base_diagnostics: Dict[str, Any] = {
        "phase1": dict(phase1.diagnostics),
        "phase2": {},
        "degraded": phase1.degraded,
        "num_escapes": phase1.num_escapes,
        "switch_relative_residual_position_m": switch_position,
        "switch_relative_residual_orientation_rad": switch_orientation,
        "phase1_relative_residual_position_m": phase1.relative_pose_residual_max_position_m,
        "phase1_relative_residual_orientation_rad": (
            phase1.relative_pose_residual_max_orientation_rad
        ),
        "total_duration_s": float(phase1.diagnostics.get("total_duration_s", 0.0)),
    }

    if (
        switch_position > RESIDUAL_TOLERANCE
        or switch_orientation > RESIDUAL_TOLERANCE
    ):
        base_diagnostics["degraded"] = phase1.degraded
        return HandoverPlan(
            success=False,
            message=(
                f"交接点相对位姿残差超阈值：位置 {switch_position:.3e} m / "
                f"姿态 {switch_orientation:.3e} rad"
                f"（阈值 {RESIDUAL_TOLERANCE:g}）"
            ),
            reason=REASON_CONSTRAINT_VIOLATION,
            stage="constraint",
            phase1=phase1,
            x_lr=matrix,
            x_lr_inverse=inverse,
            switch_relative_residual_position_m=switch_position,
            switch_relative_residual_orientation_rad=switch_orientation,
            num_escapes=phase1.num_escapes,
            diagnostics=base_diagnostics,
        )

    # ── 阶段 2 的 leader 目标：T_follower = T_leader · X_LR ─────────────
    leader_goal_world = parameterization.base_pose(left) @ parameterization.arm_tcp_pose(
        left, goal
    )
    follower_goal_world = leader_goal_world @ matrix
    ik_result = _arm_analytic_ik(
        parameterization,
        right,
        follower_goal_world,
        seed=follower_end,
        psi_count=psi_count,
    )
    if not bool(ik_result.success):
        return HandoverPlan(
            success=False,
            message=(
                f"交接阶段 2 的目标 IK 无解（reason={ik_result.reason}）："
                f"{ik_result.message}"
            ),
            reason=str(ik_result.reason),
            stage="ik",
            phase1=phase1,
            x_lr=matrix,
            x_lr_inverse=inverse,
            switch_relative_residual_position_m=switch_position,
            switch_relative_residual_orientation_rad=switch_orientation,
            num_escapes=phase1.num_escapes,
            diagnostics=base_diagnostics,
        )
    phase2_goal_q = np.asarray(ik_result.q, dtype=float)

    # ── 阶段 2 的 follower ψ 提示：交接位处初始 leader 臂的 ψ ─────────
    leader_end_world = parameterization.base_pose(left) @ parameterization.arm_tcp_pose(
        left, leader_end
    )
    psi_hint: Optional[float] = None
    psi_result = _arm_analytic_ik(
        parameterization,
        left,
        leader_end_world,
        seed=leader_end,
        psi_count=psi_count,
    )
    if bool(psi_result.success):
        psi_hint = float(psi_result.psi)

    # ── 阶段 2：角色互换，用 inv(X_LR) 走后续段 ───────────────────────
    result2 = plan_coordinated(
        bundle,
        x_lr=inverse,
        start_leader_q=follower_end,
        goal_leader_q=phase2_goal_q,
        leader=right,
        follower=left,
        psi_start=psi_hint,
        tool_offsets=tool_offsets,
        options=options,
    )
    dual2_raw = result2.dual_trajectory
    dual2 = None
    if dual2_raw is not None:
        dual2 = _relabel_dual(
            dual2_raw,
            acceleration_limits=parameterization.dual_limits.acceleration_array()[
                :ARM_JOINT_COUNT
            ],
        )
    phase2 = _carry_from_result(result2, dual=dual2)
    base_diagnostics["phase2"] = dict(phase2.diagnostics)
    base_diagnostics["phase2_arm_labels_relabelled"] = True
    base_diagnostics["degraded"] = bool(phase1.degraded or phase2.degraded)
    base_diagnostics["num_escapes"] = int(phase1.num_escapes + phase2.num_escapes)
    base_diagnostics["relative_pose_residual_max_position_m"] = max(
        phase1.relative_pose_residual_max_position_m,
        phase2.relative_pose_residual_max_position_m,
    )
    base_diagnostics["relative_pose_residual_max_orientation_rad"] = max(
        phase1.relative_pose_residual_max_orientation_rad,
        phase2.relative_pose_residual_max_orientation_rad,
    )
    base_diagnostics["total_duration_s"] = float(
        phase1.diagnostics.get("total_duration_s", 0.0)
    ) + float(phase2.diagnostics.get("total_duration_s", 0.0))

    if not phase2.success or dual2 is None:
        return HandoverPlan(
            success=False,
            message=f"交接阶段 2 失败：{phase2.message}",
            reason=phase2.reason,
            stage=phase2.stage or "phase2",
            phase1=phase1,
            phase2=phase2,
            x_lr=matrix,
            x_lr_inverse=inverse,
            switch_relative_residual_position_m=switch_position,
            switch_relative_residual_orientation_rad=switch_orientation,
            num_escapes=int(phase1.num_escapes + phase2.num_escapes),
            diagnostics=base_diagnostics,
        )

    # ── 交接点连续性度量（TCP 级连续；关节级可能换镜像分支）──────────
    # 阶段 2 的轨迹已按「左臂 / 右臂」重新装配 ⇒ 列槽位即侧别。
    q14_phase2_start = np.asarray(dual2.q_matrix()[0], dtype=float)
    joint_gap = {
        side: float(np.max(np.abs(q14_phase2_start[_side_slice(side)] - q14_switch[_side_slice(side)])))
        for side in ("left", "right")
    }
    tcp_gap = {}
    for side in ("left", "right"):
        pose_before = parameterization.arm_tcp_pose(side, q14_switch[_side_slice(side)])
        pose_after = parameterization.arm_tcp_pose(
            side, q14_phase2_start[_side_slice(side)]
        )
        tcp_gap[side] = {
            "position_m": float(np.linalg.norm(pose_after[:3, 3] - pose_before[:3, 3])),
            "orientation_rad": _rotation_angle(pose_before[:3, :3], pose_after[:3, :3]),
        }
    base_diagnostics["switch_joint_discontinuity_max_rad"] = joint_gap
    base_diagnostics["switch_tcp_discontinuity"] = tcp_gap
    base_diagnostics["switch_tcp_position_discontinuity_m"] = float(
        max(entry["position_m"] for entry in tcp_gap.values())
    )
    base_diagnostics["switch_tcp_orientation_discontinuity_rad"] = float(
        max(entry["orientation_rad"] for entry in tcp_gap.values())
    )

    return HandoverPlan(
        success=True,
        message=(
            f"交接规划成功：阶段 1 {len(dual1)} 点 / 阶段 2 {len(dual2)} 点，"
            f"交接点残差 {switch_position:.3e} m / {switch_orientation:.3e} rad"
            f"（阈值 {RESIDUAL_TOLERANCE:g}），"
            f"碰撞逃逸 {int(phase1.num_escapes + phase2.num_escapes)}"
        ),
        phase1=phase1,
        phase2=phase2,
        x_lr=matrix,
        x_lr_inverse=inverse,
        switch_relative_residual_position_m=switch_position,
        switch_relative_residual_orientation_rad=switch_orientation,
        num_escapes=int(phase1.num_escapes + phase2.num_escapes),
        diagnostics=base_diagnostics,
    )
