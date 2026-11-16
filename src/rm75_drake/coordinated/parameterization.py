"""**最小坐标参数化**：把「双臂相对位姿固定」变成**构造性成立**的约束（S05-T03）。

为什么需要它
------------
R006 要求「相对位姿固定」**构造性满足**——不是软约束、不是采样碰运气。
但在 14 维（两臂各 7 DoF）联合空间里，「相对位姿固定」是一个**零测集**的
约束流形：采样规划几乎采不到满足约束的点（见记忆 MEM004）。参数化的作用
就是把该流形**参数化掉**：

* 8 维参数 ``x = (q_leader[7], ψ_follower[1])``；
* 映射 ``x → q14`` 由 **leader 的 FK + 常数相对变换 + follower 的解析 IK**
  组成 ⇒ 约束由构造保证成立，**残差恒为机器精度量级**（~1e-15 m / rad）；
* 碰撞/限位只在映射**之后**的 14 维位形上判定（走**同一个 plant**）。

与 :mod:`rm75_drake.bimanual.consistency` 的语义差别（**不要混用**）
----------------------------------------------------------------
``consistency.py`` 回答的是「两臂**共同点**（抓取点）是否**重合**」——
一个布尔 / 三态判定；本模块回答的是「两臂**相对位姿**是否**保持**」——
一个**逐点残差**（m / rad），字段名与三态口径刻意与 ``consistency.py``
对齐（``position_error`` / ``orientation_error`` / ``reason``），但**语义不同**，
**不得**用同一个函数名冒充同一个指标。

模型口径（重要）
----------------
1. **FK 走 MDH 模型**（:mod:`rm75_drake.kinematics.mdh`，numpy-only）：本模块
   的「构造性满足」是在 **MDH 模型**内成立的。该模型与 Drake 载入的 URDF
   链之间存在**官方 rpy 舍入残差**（~2.24e-6 m，见 :mod:`rm75_drake.kinematics.mdh`
   的模块 docstring）——那是**模型差异**，不是本参数化的构造误差。
2. **工具偏置沿 J7 轴**：解析 IK 绑定这一前提（球腕 + ``alpha_7 = 0``）。
   裸臂 ``tool_offset = 0.144 m``（TCP 即 link_7 原点）；装夹爪
   ``tool_offset = 0.204 m``（RMG24 的 TCP 在法兰 z 向 0.06 m 处，
   ``0.144 + 0.06``）。带倾角的工具 ⇒ 解耦失效，**不适用**本模块。
3. **相对变换 X_LR 的定义**：``X_LR`` 是 **leader TCP 系 → follower TCP 系**
   的常数变换，即 ``T_follower_world = T_leader_world · X_LR``。当
   ``leader="left"`` / ``follower="right"`` 时，它就是计划书所说的
   **TCP_L→TCP_R**。

分支 / ψ 连续性
---------------
* **ψ 是参数**（x 的第 8 维）⇒ ψ 的连续性由**参数本身**保证，
  ``diagnostics.psi_switches`` 恒为 0（不是"忘了做"，是**构造上不可能**）。
* **分支（8 组镜像）**由**上一解最近邻**规则挑选（``branch_policy="nearest"``）：
  上一分支仍可行 ⇒ 继续用它（热路径）；不可行 ⇒ 全分支重解并**计一次分支切换**。
  ``diagnostics.branch_switches`` 回报切换次数。

失败必须点名
------------
follower 无解时抛 :class:`CoordinatedParameterizationError`，其 ``reason``
直接沿用解析 IK 的稳定词表：``unreachable`` / ``branch_exhausted`` /
``limit_violation`` / ``wrist_singular`` / ``invalid_target``；参数化自身的
失败用 ``invalid_dimension`` / ``invalid_transform`` / ``invalid_input``。
**绝不静默换目标、绝不返回近似解**。

零 pydrake 依赖（模块级）
-------------------------
本模块顶层只 import numpy + :mod:`rm75_drake.kinematics`（numpy-only）+
:mod:`rm75_drake.coordinated.limits`（numpy-only）。Drake 侧依赖
（:func:`rm75_drake.bimanual.dual_scene.check_config_free`、
:func:`rm75_drake.drake_compat.eval_body_pose_in_world`）一律**函数内惰性导入**，
因此 ``import rm75_drake.coordinated.parameterization`` **不会**把 pydrake 拉进
``sys.modules``。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from ..kinematics import analytic_ik, mdh
from .limits import DualLimits, derive_dual_limits, pad_gripper_positions

__all__ = [
    "ARM_JOINT_COUNT",
    "PARAMETER_DIM",
    "CoordinatedParameterizationError",
    "RelativePoseResidual",
    "ParameterizationDiagnostics",
    "CoordinatedParameterization",
]


#: 单臂关节数（7 DoF）。
ARM_JOINT_COUNT = mdh.N_JOINTS

#: 最小坐标参数化的维度：leader 7 关节 + follower ψ 1 = **8**。
PARAMETER_DIM = ARM_JOINT_COUNT + 1

#: 双臂联合关节数（14 DoF）。
DUAL_JOINT_COUNT = 2 * ARM_JOINT_COUNT

#: 默认的相对位姿残差阈值（构造性满足的判定口径）。
DEFAULT_POSITION_TOLERANCE = 1e-6
DEFAULT_ORIENTATION_TOLERANCE = 1e-6

#: 失败标签（稳定词表）。
REASON_INVALID_DIMENSION = "invalid_dimension"
REASON_INVALID_TRANSFORM = "invalid_transform"
REASON_INVALID_INPUT = "invalid_input"
REASON_INVALID_SCENE = "invalid_scene"

_VALID_SIDES = ("left", "right")


# ── 小工具 ───────────────────────────────────────────────────────────────
def _as_transform(matrix: Any, *, name: str = "X_LR") -> np.ndarray:
    """把输入规整成 4×4 齐次矩阵；非法输入抛（**点名期望 / 实际**）。"""
    try:
        array = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError) as exc:  # pragma: no cover - 防御性
        raise CoordinatedParameterizationError(
            f"[transform] {name} 必须是 4×4 数值矩阵：{exc}",
            reason=REASON_INVALID_TRANSFORM,
        ) from exc
    if array.shape != (4, 4):
        raise CoordinatedParameterizationError(
            f"[transform] {name} 必须是 4×4 齐次矩阵，实际形状 {array.shape}",
            reason=REASON_INVALID_TRANSFORM,
        )
    if not np.all(np.isfinite(array)):
        raise CoordinatedParameterizationError(
            f"[transform] {name} 含 NaN/Inf",
            reason=REASON_INVALID_TRANSFORM,
        )
    rot = array[:3, :3]
    if not np.allclose(rot @ rot.T, np.eye(3), atol=1e-6):
        raise CoordinatedParameterizationError(
            f"[transform] {name} 的旋转部分不是正交阵",
            reason=REASON_INVALID_TRANSFORM,
        )
    if float(np.linalg.det(rot)) < 0.0:
        raise CoordinatedParameterizationError(
            f"[transform] {name} 的旋转部分行列式为负（镜像）",
            reason=REASON_INVALID_TRANSFORM,
        )
    if not np.allclose(array[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9):
        raise CoordinatedParameterizationError(
            f"[transform] {name} 的最后一行必须是 [0, 0, 0, 1]，实际 {array[3]}",
            reason=REASON_INVALID_TRANSFORM,
        )
    return array.copy()


def _rotation_angle(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（rad）。

    用 ``atan2(|skew|, cos)`` 而不是 ``acos(cos)``：后者在夹角趋近 0 时会因
    ``1 − θ²/2`` 的抵消把 ~1e-15 rad 的残差放大成 ~1e-8 rad（与
    :mod:`rm75_drake.kinematics.analytic_ik` 同一处理）。
    """
    rel = rot_a.T @ rot_b
    skew = 0.5 * (rel - rel.T)
    sin_theta = float(np.linalg.norm(skew))
    cos_theta = (float(np.trace(rel)) - 1.0) * 0.5
    cos_theta = min(1.0, max(-1.0, cos_theta))
    return float(math.atan2(sin_theta, cos_theta))


# ── 数据结构 ─────────────────────────────────────────────────────────────
class CoordinatedParameterizationError(RuntimeError):
    """最小坐标参数化的失败（**失败必须点名**）。

    ``reason`` 是稳定标签：

    * 解析 IK 透传：``unreachable`` / ``branch_exhausted`` / ``limit_violation``
      / ``wrist_singular`` / ``invalid_target``；
    * 参数化自身：``invalid_dimension`` / ``invalid_transform`` /
      ``invalid_input`` / ``invalid_scene``。
    """

    def __init__(self, message: str, *, reason: str = REASON_INVALID_INPUT) -> None:
        super().__init__(message)
        self.reason = str(reason)


@dataclass(frozen=True)
class RelativePoseResidual:
    """「相对位姿是否保持」的**逐点**残差。

    字段（与 :mod:`rm75_drake.bimanual.consistency` 的口径对齐，但**语义不同**：
    那里是「共同点是否重合」，这里是「相对位姿是否保持」）
    ------------------------------------------------------------------
    position_error:
        位置残差（m）：``||p_follower − p_expected||``。
    orientation_error:
        姿态残差（rad）：两个旋转矩阵之间的夹角。
    position_error_vector:
        位置残差向量（m，世界系，follower's 期望位姿减实际位姿）。
    """

    position_error: float
    orientation_error: float
    position_error_vector: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def within(
        self,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
        orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE,
    ) -> bool:
        """是否在容差内（**逐点**判定，不是统计口径）。"""
        return (
            self.position_error <= float(position_tolerance)
            and self.orientation_error <= float(orientation_tolerance)
        )

    def as_dict(self) -> Dict[str, Any]:
        """可 JSON 序列化的字典（进 diagnostics）。"""
        return {
            "position_error": float(self.position_error),
            "orientation_error": float(self.orientation_error),
            "position_error_vector": [
                float(v) for v in self.position_error_vector
            ],
        }


@dataclass
class ParameterizationDiagnostics:
    """参数化的运行期诊断（**分支切换次数可观测**）。"""

    solve_count: int = 0
    branch_switches: int = 0
    psi_switches: int = 0
    failure_count: int = 0
    last_reason: Optional[str] = None
    last_message: Optional[str] = None
    last_branch: Optional[Tuple[int, int, int]] = None
    last_psi: Optional[float] = None
    #: 历次成功求解选中的镜像分支（与 ``solve_count`` 等长）。
    branch_history: list = field(default_factory=list)

    def as_dict(self) -> Dict[str, Any]:
        """可 JSON 序列化的字典。"""
        return {
            "solve_count": int(self.solve_count),
            "branch_switches": int(self.branch_switches),
            "psi_switches": int(self.psi_switches),
            "failure_count": int(self.failure_count),
            "last_reason": self.last_reason,
            "last_message": self.last_message,
            "last_branch": list(self.last_branch) if self.last_branch else None,
            "last_psi": None if self.last_psi is None else float(self.last_psi),
            "branch_history": [list(b) for b in self.branch_history],
        }


# ── 主类 ─────────────────────────────────────────────────────────────────
class CoordinatedParameterization:
    """双臂相对位姿固定的**最小坐标参数化**（8 维 → 14 维）。

    参数
    ----
    bundle:
        :class:`rm75_drake.bimanual.dual_scene.DualSceneBundle`（**同一个 plant**）。
        只用到 ``plant`` / ``context`` / ``joint_names`` / ``base_links`` /
        ``gripper_joint_names`` / ``robot_names``。
    x_lr:
        **常数**相对变换（4×4，leader TCP 系 → follower TCP 系）。
        ``leader="left"`` / ``follower="right"`` 时即 **TCP_L→TCP_R**。
        **不由本模块优化**（任务给定）。
    leader / follower:
        指派（``"left"`` / ``"right"``），必须不同且都在场景里。
    tool_offsets:
        两侧各自的工具偏置（m，**沿 J7 轴**）：``{"left": 0.144, "right": 0.204}``。
        缺省两侧都用裸臂值（:data:`rm75_drake.kinematics.mdh.MDH_D` 的末项，0.144 m）。
    psi_count:
        ψ 网格数（仅在**未给定分支**时用于全分支重解；正常路径 ψ 由 x 给定）。
    position_tolerance / orientation_tolerance:
        :meth:`is_feasible` 使用的**逐点**残差阈值（m / rad）。
    gripper_value:
        碰撞查询前补齐夹爪 prismatic 用的**固定开度**（m，默认 0.0）。
    """

    def __init__(
        self,
        bundle: Any,
        *,
        x_lr: Any,
        leader: str = "left",
        follower: str = "right",
        tool_offsets: Optional[Mapping[str, float]] = None,
        psi_count: int = analytic_ik.DEFAULT_PSI_COUNT,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
        orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE,
        gripper_value: float = 0.0,
        alpha: Optional[Sequence[float]] = None,
        d: Optional[Sequence[float]] = None,
    ) -> None:
        self._bundle = bundle
        self._leader = str(leader)
        self._follower = str(follower)
        if self._leader not in _VALID_SIDES or self._follower not in _VALID_SIDES:
            raise CoordinatedParameterizationError(
                f"[scene] leader / follower 必须是 'left' 或 'right'，实际："
                f"{self._leader!r} / {self._follower!r}",
                reason=REASON_INVALID_SCENE,
            )
        if self._leader == self._follower:
            raise CoordinatedParameterizationError(
                f"[scene] leader 与 follower 不能相同（实际都是 {self._leader!r}）",
                reason=REASON_INVALID_SCENE,
            )
        self._psi_count = int(psi_count)
        if self._psi_count < 1:
            raise CoordinatedParameterizationError(
                f"[input] psi_count 必须是正整数，实际 {psi_count}",
                reason=REASON_INVALID_INPUT,
            )
        self._x_lr = _as_transform(x_lr)
        self._alpha = tuple(mdh.MDH_ALPHA if alpha is None else (float(a) for a in alpha))
        self._d = tuple(mdh.MDH_D if d is None else (float(v) for v in d))
        if len(self._alpha) != mdh.N_JOINTS or len(self._d) != mdh.N_JOINTS:
            raise CoordinatedParameterizationError(
                f"[input] d / alpha 表长度必须是 {mdh.N_JOINTS}，"
                f"实际 d={len(self._d)} alpha={len(self._alpha)}",
                reason=REASON_INVALID_INPUT,
            )
        self._tool_offsets = self._resolve_tool_offsets(tool_offsets)
        self._position_tolerance = float(position_tolerance)
        self._orientation_tolerance = float(orientation_tolerance)
        self._gripper_value = float(gripper_value)

        # 双臂 14 维软件限位（**程序派生**自 S01 单臂真值；名字错配 ⇒ 显式报错）。
        self._dual_limits: DualLimits = derive_dual_limits(bundle.joint_names)
        self._base_poses: Dict[str, np.ndarray] = {}
        self._last_follower_q: Optional[np.ndarray] = None
        self._last_branch: Optional[Tuple[int, int, int]] = None
        self.diagnostics = ParameterizationDiagnostics()

    # -- 构造辅助 -------------------------------------------------------
    def _resolve_tool_offsets(
        self, tool_offsets: Optional[Mapping[str, float]]
    ) -> Dict[str, float]:
        default = float(mdh.MDH_D[6])
        out = {"left": default, "right": default}
        if tool_offsets is None:
            return out
        for side, value in dict(tool_offsets).items():
            if side not in out:
                raise CoordinatedParameterizationError(
                    f"[input] tool_offsets 的键必须是 'left' / 'right'，实际 {side!r}",
                    reason=REASON_INVALID_INPUT,
                )
            number = float(value)
            if not math.isfinite(number) or number < 0.0:
                raise CoordinatedParameterizationError(
                    f"[input] 工具偏置必须是有限的非负数，实际 {value!r}",
                    reason=REASON_INVALID_INPUT,
                )
            out[side] = number
        return out

    # -- 只读属性 -------------------------------------------------------
    @property
    def leader(self) -> str:
        """leader 侧别。"""
        return self._leader

    @property
    def follower(self) -> str:
        """follower 侧别。"""
        return self._follower

    @property
    def x_lr(self) -> np.ndarray:
        """常数相对变换（4×4，leader TCP 系 → follower TCP 系）。"""
        return self._x_lr.copy()

    @property
    def tool_offsets(self) -> Dict[str, float]:
        """两侧的工具偏置（m，沿 J7 轴）。"""
        return dict(self._tool_offsets)

    @property
    def parameter_dim(self) -> int:
        """参数维度（恒为 8）。"""
        return PARAMETER_DIM

    @property
    def dual_limits(self) -> DualLimits:
        """双臂 14 维软件限位。"""
        return self._dual_limits

    # -- 关节切片 -------------------------------------------------------
    def _side_slice(self, side: str) -> slice:
        if side == "left":
            return slice(0, ARM_JOINT_COUNT)
        return slice(ARM_JOINT_COUNT, DUAL_JOINT_COUNT)

    def _side_limits(self, side: str) -> Tuple[Tuple[float, float], ...]:
        lower = self._dual_limits.lower_array()[self._side_slice(side)]
        upper = self._dual_limits.upper_array()[self._side_slice(side)]
        return tuple((float(lo), float(hi)) for lo, hi in zip(lower, upper))

    # -- 基座位姿（惰性，Drake 侧） --------------------------------------
    def base_pose(self, side: str) -> np.ndarray:
        """某臂基座在**世界系**下的位姿（4×4）。

        取自**同一个 plant**（基座 link 的 body 位姿；基座被 weld 到世界系，
        因此该位姿是常数 = 场景的 ``base_extrinsics``）。
        """
        if side in self._base_poses:
            return self._base_poses[side].copy()
        from ..drake_compat import eval_body_pose_in_world

        bundle = self._bundle
        base_link = bundle.base_links[side]
        plant = bundle.plant
        context = bundle.context
        body = plant.GetBodyByName(base_link)
        pose = eval_body_pose_in_world(context, body, plant=plant)
        matrix = np.eye(4)
        matrix[:3, :3] = np.asarray(pose.rotation().matrix(), dtype=float)
        matrix[:3, 3] = np.asarray(pose.translation(), dtype=float)
        self._base_poses[side] = matrix
        return matrix.copy()

    # -- 正解 -----------------------------------------------------------
    def arm_tcp_pose(self, side: str, q: Sequence[float]) -> np.ndarray:
        """某臂在**自身基座系**下的 TCP 位姿（4×4）。

        TCP = link_7 原点沿 J7 轴偏移 ``(tool_offset − d7)``（球腕 + ``alpha_7=0``）。
        """
        frame = mdh.fk(q, d=self._d, alpha=self._alpha)
        tool = self._tool_offsets[side]
        out = frame.copy()
        out[:3, 3] = frame[:3, 3] + (tool - float(self._d[6])) * frame[:3, 2]
        return out

    # -- 映射：x → q14 --------------------------------------------------
    def to_joint_config(self, x: Sequence[float]) -> np.ndarray:
        """把 8 维最小坐标参数映射成 14 维双臂关节角。

        ``x = (q_leader[7], ψ_follower[1])``；follower 的目标 TCP 位姿 =
        ``T_leader(q_leader) · X_LR``（再换到 follower 的基座系），
        由 :mod:`rm75_drake.kinematics.analytic_ik` **闭式反解**。

        失败
        ----
        :class:`CoordinatedParameterizationError`
            ``reason`` ∈ ``invalid_dimension`` / ``invalid_input`` /
            ``unreachable`` / ``branch_exhausted`` / ``limit_violation`` /
            ``wrist_singular`` / ``invalid_target``。
        """
        array = np.asarray(x, dtype=float).reshape(-1)
        if array.size != PARAMETER_DIM:
            raise CoordinatedParameterizationError(
                f"[dimension] 参数化向量必须是 {PARAMETER_DIM} 维"
                f"（leader {ARM_JOINT_COUNT} 关节 + follower ψ 1），实际 {array.size} 维",
                reason=REASON_INVALID_DIMENSION,
            )
        if not np.all(np.isfinite(array)):
            raise CoordinatedParameterizationError(
                "[input] 参数化向量含 NaN/Inf",
                reason=REASON_INVALID_INPUT,
            )
        q_leader = array[:ARM_JOINT_COUNT]
        psi = float(array[ARM_JOINT_COUNT])

        q_follower = self._solve_follower(q_leader, psi)

        q14 = np.zeros(DUAL_JOINT_COUNT, dtype=float)
        q14[self._side_slice(self._leader)] = q_leader
        q14[self._side_slice(self._follower)] = q_follower
        self.diagnostics.solve_count += 1
        self.diagnostics.last_psi = psi
        return q14

    def _solve_follower(self, q_leader: np.ndarray, psi: float) -> np.ndarray:
        """解 follower 的 7 维关节角（**分支连续性** + **失败点名**）。"""
        leader_tcp_world = self.base_pose(self._leader) @ self.arm_tcp_pose(
            self._leader, q_leader
        )
        target_world = leader_tcp_world @ self._x_lr
        target_base = np.linalg.inv(self.base_pose(self._follower)) @ target_world

        follower_tool = self._tool_offsets[self._follower]
        follower_limits = self._side_limits(self._follower)
        seed = self._last_follower_q

        result = analytic_ik.solve_analytic_ik(
            target_base,
            psi=psi,
            tool_offset=follower_tool,
            seed=seed,
            branch_policy="nearest",
            limits=follower_limits,
            branch=self._last_branch,
        )
        switched = False
        if not result.success and self._last_branch is not None:
            # 上一分支失效（限位 / 奇异）⇒ 全分支重解，按「离上一个解最近」挑。
            if result.reason in (
                analytic_ik.REASON_LIMIT_VIOLATION,
                analytic_ik.REASON_BRANCH_EXHAUSTED,
                analytic_ik.REASON_WRIST_SINGULAR,
            ):
                retry = analytic_ik.solve_analytic_ik(
                    target_base,
                    psi=psi,
                    tool_offset=follower_tool,
                    seed=seed,
                    branch_policy="nearest",
                    limits=follower_limits,
                )
                if retry.success:
                    result = retry
                    switched = True

        if not result.success:
            self.diagnostics.failure_count += 1
            self.diagnostics.last_reason = result.reason
            self.diagnostics.last_message = result.message
            raise CoordinatedParameterizationError(
                f"[follower] follower 无解（reason={result.reason}）：{result.message}",
                reason=result.reason,
            )

        branch = tuple(result.branch) if result.branch is not None else None
        if switched or (
            self._last_branch is not None and branch is not None and branch != self._last_branch
        ):
            self.diagnostics.branch_switches += 1
        self.diagnostics.branch_history.append(branch)
        self.diagnostics.last_branch = branch
        self._last_branch = branch
        self._last_follower_q = np.asarray(result.q, dtype=float).copy()
        return self._last_follower_q

    # -- 残差 -----------------------------------------------------------
    def relative_pose(self, q14: Sequence[float]) -> RelativePoseResidual:
        """14 维位形的**逐点**相对位姿残差（m / rad）。

        期望关系：``T_follower_world = T_leader_world · X_LR``。
        """
        array = np.asarray(q14, dtype=float).reshape(-1)
        if array.size != DUAL_JOINT_COUNT:
            raise CoordinatedParameterizationError(
                f"[dimension] 位形必须是 {DUAL_JOINT_COUNT} 维，实际 {array.size} 维",
                reason=REASON_INVALID_DIMENSION,
            )
        if not np.all(np.isfinite(array)):
            raise CoordinatedParameterizationError(
                "[input] 位形含 NaN/Inf",
                reason=REASON_INVALID_INPUT,
            )
        leader_tcp = self.base_pose(self._leader) @ self.arm_tcp_pose(
            self._leader, array[self._side_slice(self._leader)]
        )
        follower_tcp = self.base_pose(self._follower) @ self.arm_tcp_pose(
            self._follower, array[self._side_slice(self._follower)]
        )
        expected = leader_tcp @ self._x_lr
        delta = follower_tcp[:3, 3] - expected[:3, 3]
        return RelativePoseResidual(
            position_error=float(np.linalg.norm(delta)),
            orientation_error=_rotation_angle(follower_tcp[:3, :3], expected[:3, :3]),
            position_error_vector=(float(delta[0]), float(delta[1]), float(delta[2])),
        )

    # -- 可行性 ---------------------------------------------------------
    def within_limits(self, q14: Sequence[float]) -> bool:
        """14 维位形是否都在软件限位内。"""
        array = np.asarray(q14, dtype=float).reshape(-1)
        if array.size != DUAL_JOINT_COUNT:
            return False
        lower = self._dual_limits.lower_array()
        upper = self._dual_limits.upper_array()
        return bool(np.all(array >= lower - 1e-12) and np.all(array <= upper + 1e-12))

    def is_collision_free(self, q14: Sequence[float]) -> bool:
        """14 维位形是否**碰撞自由**（走**同一个 plant**，S04 的过滤矩阵）。

        装夹爪时 ``plant.num_positions() == 18`` ⇒ 查询前按约定补齐 4 个
        夹爪 prismatic（固定开度）。
        """
        from ..bimanual.dual_scene import check_config_free

        array = np.asarray(q14, dtype=float).reshape(-1)
        if array.size != DUAL_JOINT_COUNT:
            raise CoordinatedParameterizationError(
                f"[dimension] 位形必须是 {DUAL_JOINT_COUNT} 维，实际 {array.size} 维",
                reason=REASON_INVALID_DIMENSION,
            )
        padded = pad_gripper_positions(
            array,
            gripper_joint_names=self._bundle.gripper_joint_names,
            gripper_value=self._gripper_value,
        )
        return bool(check_config_free(self._bundle, padded[0]))

    def is_feasible(self, x: Sequence[float]) -> bool:
        """最小坐标参数 x 是否**可行**。

        三者**同时**满足才算可行：

        1. 映射成功（follower 有解）；
        2. 14 维位形在软件限位内；
        3. 相对位姿残差在阈值内（构造性满足 ⇒ 恒成立）；
        4. 14 维位形碰撞自由（**同一个 plant**）。
        """
        try:
            q14 = self.to_joint_config(x)
        except CoordinatedParameterizationError:
            return False
        if not self.within_limits(q14):
            return False
        residual = self.relative_pose(q14)
        if not residual.within(self._position_tolerance, self._orientation_tolerance):
            return False
        return self.is_collision_free(q14)
