"""RM75 的**解析**逆运动学（numpy-only，零 Drake 依赖）。

为什么需要它
------------
S05 的「双臂相对位姿固定」要求 follower 的目标 TCP 位姿可以**闭式反解**成
关节角（否则约束只能靠采样碰运气，违反 R006）。本模块提供：

* RM75 的球腕解析反解（8 组镜像分支 × 连续自由量 ψ）；
* 肘点圆几何（可达域、肘点方位角 ψ）；
* 稳定的失败标签（``unreachable`` / ``branch_exhausted`` / ``limit_violation`` /
  ``wrist_singular`` / ``invalid_target``）——**失败必须点名**，
  不返回「看起来正常」的结果。

结构事实（由 :mod:`rm75_drake.kinematics.mdh` 的 MDH 表推出）
----------------------------------------------------------
约定 ``T = ∏ Rotz(q_i)·Transz(d_i)·Rotx(alpha_i)``，且 ``a_i ≡ 0``：

* 肩球 ``P1 = (0, 0, d1)``：J1/J2/J3 三轴共点；
* 肘点 ``E = P1 + d3·z2``：J4/J5 两轴共点（``d4 = 0``）；
* 腕心 ``W = E + d5·z4``：``d6 = 0`` ⇒ J5/J6/J7 三轴共点于 W（**真球腕**）；
* ``alpha_7 = 0`` ⇒ **工具 z 轴 = J7 轴**，故 TCP 原点沿工具轴偏移
  ``tool_offset``，且 ``W = p_tcp − tool_offset·(R·e_z)`` 可闭式反解。

自由自由度是 **ψ = 肘点 E 在以 P1–W 为轴、半径 h 的圆周上的方位角**
（**不是 q4**：q3 与 q4 都影响位置）。给定 W 与目标姿态 R，
ψ 的每一个取值都能配出球腕解 ⇒ 目标位姿**不唯一确定** q，
但**每一条解都精确复现目标位姿**（这是本模块往返误差 ~1e-15 m 的原因）。

前提与限制（务必阅读）
----------------------
1. **工具偏置沿 J7 轴**：本模块假定 TCP 与腕心 W 的连线平行于工具 z 轴。
   裸臂 ``tool_offset = d7 = 0.144 m``；装夹爪场景 0.204 m。
   若换成**带倾角**的工具，``W = p − d·(R e_z)`` 的解耦**立即失效**，
   本模块不再适用（必须改用数值 IK）。
2. **alpha 表一致性**：解析 IK 与 :func:`rm75_drake.kinematics.mdh.fk`
   必须使用**同一张** alpha 表（舍入表 ``MDH_ALPHA`` 或精确表
   ``MDH_ALPHA_EXACT_PI_2``），否则会引入 ~1e-6 m 级不一致。

零 Drake 依赖
-------------
本模块只 import numpy（不 import pydrake，也不 import 会拉起 pydrake 的
``rm75_drake.models`` / ``rm75_drake.drake_compat``），保持与
:mod:`rm75_drake.kinematics.mdh` 相同的「独立对照」性质。
软件限位默认从 S01 的真值文件 ``models/rm75_drake/joint_limits.yaml``
**惰性**读取（读 YAML 不引入 Drake 依赖）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

from . import mdh

__all__ = [
    "AnalyticIkResult",
    "Solution",
    "analytic_solutions",
    "solve_analytic_ik",
    "REASON_BRANCH_EXHAUSTED",
    "REASON_INVALID_TARGET",
    "REASON_LIMIT_VIOLATION",
    "REASON_UNREACHABLE",
    "REASON_WRIST_SINGULAR",
]


# ── 失败标签（稳定词表） ────────────────────────────────────────────────
#: 目标腕心落在手臂可达包络之外（``d3 − d5 ≤ L ≤ d3 + d5`` 不满足）。
REASON_UNREACHABLE = "unreachable"
#: 可达，但 8 组镜像分支 × ψ 网格枚举完后仍没有可行解（分支耗尽）。
REASON_BRANCH_EXHAUSTED = "branch_exhausted"
#: 有几何解，但全部违反软件限位。
REASON_LIMIT_VIOLATION = "limit_violation"
#: 几何解全部落在腕奇异位形（``sin q6 ≈ 0``），球腕分解退化。
REASON_WRIST_SINGULAR = "wrist_singular"
#: 目标位姿本身非法（形状错、含 NaN/Inf、非正交旋转等）。
REASON_INVALID_TARGET = "invalid_target"

#: 默认 ψ 采样数（``psi_count``）。
DEFAULT_PSI_COUNT = 16

#: 腕奇异判定阈值：``|sin q6|`` 小于该值即视为奇异。
WRIST_SINGULAR_SIN_TOL = 1e-7

#: 目标位姿的合法性容差（正交性 / 行列式 / 有限性）。
_POSE_TOL = 1e-6

_EZ = np.array([0.0, 0.0, 1.0])
_EX = np.array([1.0, 0.0, 0.0])
_EY = np.array([0.0, 1.0, 0.0])


# ── 小工具 ───────────────────────────────────────────────────────────────
def _rot_z(angle: float) -> np.ndarray:
    """绕 z 轴的 3×3 旋转矩阵。"""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_x(angle: float) -> np.ndarray:
    """绕 x 轴的 3×3 旋转矩阵。"""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _angle_between(r1: np.ndarray, r2: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（rad）。

    用 ``atan2(|skew|, cos)`` 而不是 ``acos(cos)``：后者在夹角趋近 0 时
    会因 ``1 − θ²/2`` 的抵消把 ~1e-15 rad 的残差放大成 ~1e-8 rad。
    """
    rel = r1.T @ r2
    skew = 0.5 * (rel - rel.T)
    sin_theta = float(np.linalg.norm(skew))
    cos_theta = (float(np.trace(rel)) - 1.0) * 0.5
    cos_theta = min(1.0, max(-1.0, cos_theta))
    return float(math.atan2(sin_theta, cos_theta))


def _as_pose(target_pose: Any) -> np.ndarray:
    """把目标位姿规整成 4×4 齐次矩阵；非法输入抛 :class:`ValueError`。

    支持三种写法：4×4 数组、``(3,) 位置 + (3,3) 旋转`` 二元组、3×4 数组。
    """
    if isinstance(target_pose, (tuple, list)) and len(target_pose) == 2:
        pos = np.asarray(target_pose[0], dtype=float).reshape(-1)
        rot = np.asarray(target_pose[1], dtype=float)
        if pos.size != 3 or rot.shape != (3, 3):
            raise ValueError(
                f"目标位姿二元组必须是 (3,) 位置 + (3,3) 旋转，收到 {pos.size} / {rot.shape}"
            )
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = pos
    else:
        pose = np.asarray(target_pose, dtype=float)
        if pose.shape == (3, 4):
            pose = np.vstack([pose, np.array([0.0, 0.0, 0.0, 1.0])])
        if pose.shape != (4, 4):
            raise ValueError(f"目标位姿必须是 4×4 齐次矩阵，收到形状 {np.shape(target_pose)}")
    if not np.all(np.isfinite(pose)):
        raise ValueError("目标位姿含 NaN/Inf")
    rot = pose[:3, :3]
    if not np.allclose(rot @ rot.T, np.eye(3), atol=_POSE_TOL):
        raise ValueError("目标位姿的旋转部分不是正交阵")
    if float(np.linalg.det(rot)) < 0.0:
        raise ValueError("目标位姿的旋转部分行列式为负（镜像）")
    return pose


def _resolve_tables(
    d: Optional[Sequence[float]], alpha: Optional[Sequence[float]]
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """解析 d / alpha 表（**引用** mdh 的表，不复制数值）。"""
    d_table = tuple(mdh.MDH_D if d is None else (float(x) for x in d))
    alpha_table = tuple(mdh.MDH_ALPHA if alpha is None else (float(x) for x in alpha))
    if len(d_table) != mdh.N_JOINTS:
        raise ValueError(f"d 表长度必须是 {mdh.N_JOINTS}，收到 {len(d_table)}")
    if len(alpha_table) != mdh.N_JOINTS:
        raise ValueError(f"alpha 表长度必须是 {mdh.N_JOINTS}，收到 {len(alpha_table)}")
    return d_table, alpha_table


@lru_cache(maxsize=1)
def software_limits() -> Tuple[Tuple[float, float], ...]:
    """软件限位（rad），来自 S01 真值 ``models/rm75_drake/joint_limits.yaml``。

    惰性读取（模块导入不触碰文件系统），按 ``joint_1..joint_7`` 顺序返回
    ``(lower, upper)``。文件缺失 / 结构非法时**显式抛错**，不静默兜底。
    """
    from ..limits import load_joint_limits  # 惰性：避免包导入期的循环依赖

    data = load_joint_limits()
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != mdh.N_JOINTS:
        raise ValueError("joint_limits.yaml 的 joints 段必须是 7 项")
    out: List[Tuple[float, float]] = []
    for index, joint in enumerate(joints, start=1):
        name = joint.get("name")
        if name != f"joint_{index}":
            raise ValueError(f"joint_limits.yaml 的第 {index} 项名字是 {name!r}，期望 'joint_{index}'")
        sw = joint.get("software")
        if not isinstance(sw, dict):
            raise ValueError(f"joint_limits.yaml 的 {name} 缺少 software 段")
        out.append(
            (float(sw["position_lower_rad"]), float(sw["position_upper_rad"]))
        )
    return tuple(out)


def within_limits(
    q: Sequence[float],
    limits: Optional[Sequence[Tuple[float, float]]] = None,
    tol: float = 1e-9,
) -> bool:
    """判断关节角是否落在软件限位内（闭区间 ± tol）。"""
    bounds = software_limits() if limits is None else tuple(limits)
    arr = np.asarray(q, dtype=float).reshape(-1)
    if arr.size != mdh.N_JOINTS:
        return False
    for value, (lower, upper) in zip(arr, bounds):
        if value < lower - tol or value > upper + tol:
            return False
    return True


def limit_margin(
    q: Sequence[float],
    limits: Optional[Sequence[Tuple[float, float]]] = None,
) -> float:
    """返回到最近限位边界的裕度（rad，越大越好；越界时为负）。"""
    bounds = software_limits() if limits is None else tuple(limits)
    arr = np.asarray(q, dtype=float).reshape(-1)
    margin = math.inf
    for value, (lower, upper) in zip(arr, bounds):
        margin = min(margin, value - lower, upper - value)
    return float(margin)


# ── 数据结构 ─────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Solution:
    """一条解析 IK 解。

    属性
    ----
    q:
        7 个关节角（rad）。
    psi:
        肘点方位角（rad），解的连续自由度。
    branch:
        镜像分支编号 ``(q2 镜像, q4 镜像, q6 镜像)``，取值 0/1，
        8 组组合对应肩/肘/腕三个二元镜像。
    wrist_singular:
        是否落在腕奇异位形（``|sin q6|`` 极小，球腕分解退化）。
    position_error / orientation_error:
        用同一张 d/alpha 表复算 FK 后的位置误差（m）/ 姿态误差（rad）。
    """

    q: np.ndarray
    psi: float
    branch: Tuple[int, int, int]
    wrist_singular: bool = False
    position_error: float = 0.0
    orientation_error: float = 0.0

    @property
    def q_tuple(self) -> Tuple[float, ...]:
        """关节角的元组形式（便于哈希 / 比较）。"""
        return tuple(float(x) for x in self.q)


@dataclass
class AnalyticIkResult:
    """解析 IK 的求解结果（**失败必须点名**）。

    ``reason`` 取值见模块常量：``unreachable`` / ``branch_exhausted`` /
    ``limit_violation`` / ``wrist_singular`` / ``invalid_target``；
    成功时为 ``"solved"``。
    """

    success: bool
    reason: str
    message: str
    q: Optional[np.ndarray] = None
    solutions: List[Solution] = field(default_factory=list)
    psi: Optional[float] = None
    branch: Optional[Tuple[int, int, int]] = None
    wrist_singular: bool = False
    stage: str = "ik"

    @property
    def q_tuple(self) -> Optional[Tuple[float, ...]]:
        """关节角的元组形式（未求解时为 ``None``）。"""
        if self.q is None:
            return None
        return tuple(float(x) for x in self.q)


# ── 核心几何 ─────────────────────────────────────────────────────────────
def _pair_solutions(
    u: np.ndarray, alpha_a: float, alpha_b: float
) -> List[Tuple[float, float, int]]:
    """求解 ``u = Rz(qa)·Rx(alpha_a)·Rz(qb)·Rx(alpha_b)·e_z``。

    ``u`` 必须是单位向量。返回 ``[(qa, qb, mirror_flag), ...]``：
    ``mirror_flag = 0`` 是 ``qb = +acos(...)`` 分支，``= 1`` 是镜像分支
    ``qb = -acos(...)``（对应 ``qa`` 相差约 π）。

    推导：记 ``m(qb) = Rx(alpha_a)·Rz(qb)·Rx(alpha_b)·e_z``，则
    ``m_z = P·cos(qb) + Q``（``P = -sin(alpha_b)·sin(alpha_a)``，
    ``Q = cos(alpha_b)·cos(alpha_a)``），先由 ``u_z`` 解出 ``cos(qb)``，
    再由水平分量定出 ``qa``。
    """
    norm = float(np.linalg.norm(u))
    if abs(norm - 1.0) > 1e-6:
        raise ValueError(f"方向向量必须是单位向量（模长 {norm}）")
    p_coef = -math.sin(alpha_b) * math.sin(alpha_a)
    q_coef = math.cos(alpha_b) * math.cos(alpha_a)
    if abs(p_coef) < 1e-8:
        raise ValueError(
            "alpha 表组合退化（无法闭式反解两连杆对）："
            f"sin(alpha_a)={math.sin(alpha_a)}, sin(alpha_b)={math.sin(alpha_b)}"
        )
    cos_qb = (float(u[2]) - q_coef) / p_coef
    if cos_qb > 1.0 + 1e-9 or cos_qb < -1.0 - 1e-9:
        return []
    cos_qb = min(1.0, max(-1.0, cos_qb))
    out: List[Tuple[float, float, int]] = []
    for mirror, qb in ((0, math.acos(cos_qb)), (1, -math.acos(cos_qb))):
        m = _rot_x(alpha_a) @ _rot_z(qb) @ _rot_x(alpha_b) @ _EZ
        m_h = math.hypot(float(m[0]), float(m[1]))
        u_h = math.hypot(float(u[0]), float(u[1]))
        if m_h < 1e-9 or u_h < 1e-9:
            # 退化：目标方向与 z 轴平行，qa 不唯一 —— 取 0（后续仍会被 FK 复算校验）。
            qa = 0.0
        else:
            qa = math.atan2(float(u[1]), float(u[0])) - math.atan2(
                float(m[1]), float(m[0])
            )
        out.append((qa, qb, mirror))
    return out


def _wrist_solutions(
    M: np.ndarray, alpha5: float, alpha6: float
) -> List[Tuple[float, float, float, int, bool]]:
    """球腕反解：``M = Rz(q5)·Rx(alpha5)·Rz(q6)·Rx(alpha6)·Rz(q7)``。

    返回 ``[(q5, q6, q7, mirror_flag, wrist_singular), ...]``（两个腕镜像分支）。

    推导：记 ``T(q6) = Rx(alpha5)·Rz(q6)·Rx(alpha6)``，
    则 ``M·e_z = Rz(q5)·T(q6)·e_z``，而 ``T(q6)·e_z`` 的 z 分量只依赖
    ``cos(q6)`` ⇒ 先解出 ``cos(q6)``，再由水平分量定 ``q5``，
    最后由 ``M[:,0] = cos(q7)·A + sin(q7)·B``（``A``/``B`` 正交单位）
    解出 ``q7``。
    """
    z7 = M[:, 2]
    e_coef = -math.sin(alpha6) * math.sin(alpha5)
    f_coef = math.cos(alpha5) * math.cos(alpha6)
    if abs(e_coef) < 1e-8:
        raise ValueError("alpha5/alpha6 组合退化（球腕无法闭式反解）")
    cos_q6 = (float(z7[2]) - f_coef) / e_coef
    if cos_q6 > 1.0 + 1e-9 or cos_q6 < -1.0 - 1e-9:
        return []
    cos_q6 = min(1.0, max(-1.0, cos_q6))
    out: List[Tuple[float, float, float, int, bool]] = []
    for mirror, sin_q6 in ((0, math.sqrt(max(0.0, 1.0 - cos_q6 * cos_q6))),
                           (1, -math.sqrt(max(0.0, 1.0 - cos_q6 * cos_q6)))):
        q6 = math.atan2(sin_q6, cos_q6)
        singular = abs(sin_q6) < WRIST_SINGULAR_SIN_TOL
        w_x = math.sin(q6) * math.sin(alpha6)
        w_y = (
            -math.cos(q6) * math.sin(alpha6) * math.cos(alpha5)
            - math.cos(alpha6) * math.sin(alpha5)
        )
        w_h = math.hypot(w_x, w_y)
        z_h = math.hypot(float(z7[0]), float(z7[1]))
        if w_h < 1e-9 or z_h < 1e-9:
            q5 = 0.0
        else:
            q5 = math.atan2(float(z7[1]), float(z7[0])) - math.atan2(w_y, w_x)
        T = _rot_x(alpha5) @ _rot_z(q6) @ _rot_x(alpha6)
        rot5 = _rot_z(q5)
        vec_a = rot5 @ (T @ _EX)
        vec_b = rot5 @ (T @ _EY)
        target = M[:, 0]
        q7 = math.atan2(float(vec_b @ target), float(vec_a @ target))
        out.append((q5, q6, q7, mirror, singular))
    return out


def _elbow_circle(
    wrist_center: np.ndarray, d: Sequence[float]
) -> Optional[Tuple[float, float, np.ndarray, np.ndarray, np.ndarray]]:
    """构造肘点圆。

    返回 ``(a, h, u_hat, u1, u2)``：``a`` 是肘心在 P1–W 轴上的投影距离，
    ``h`` 是圆半径，``u_hat`` 是 P1→W 单位方向，``u1``/``u2`` 是圆周的
    正交基（ψ 的参考系）。不可达时返回 ``None``。
    """
    d1, d3, d5 = float(d[0]), float(d[2]), float(d[4])
    p1 = np.array([0.0, 0.0, d1])
    delta = wrist_center - p1
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return None
    lo = abs(d3 - d5)
    hi = d3 + d5
    if length < lo - 1e-9 or length > hi + 1e-9:
        return None
    length = min(max(length, lo), hi)
    u_hat = delta / float(np.linalg.norm(delta))
    a = (d3 * d3 - d5 * d5 + length * length) / (2.0 * length)
    h_sq = d3 * d3 - a * a
    if h_sq < -1e-12:
        return None
    h = math.sqrt(max(0.0, h_sq))
    ref = _EX if abs(float(u_hat[0])) < 0.9 else _EY
    u1 = np.cross(ref, u_hat)
    n1 = float(np.linalg.norm(u1))
    if n1 < 1e-9:
        u1 = np.cross(_EZ, u_hat)
        n1 = float(np.linalg.norm(u1))
    u1 = u1 / n1
    u2 = np.cross(u_hat, u1)
    return a, h, u_hat, u1, u2


def _solutions_for_psi(
    wrist_center: np.ndarray,
    rot_target: np.ndarray,
    psi: float,
    d: Sequence[float],
    alpha: Sequence[float],
    circle: Tuple[float, float, np.ndarray, np.ndarray, np.ndarray],
) -> List[Tuple[np.ndarray, Tuple[int, int, int], bool]]:
    """给定 ψ 与肘点圆，枚举 8 组镜像分支的候选解。"""
    d1, d3, d5 = float(d[0]), float(d[2]), float(d[4])
    a, h, u_hat, u1, u2 = circle
    p1 = np.array([0.0, 0.0, d1])
    elbow = p1 + a * u_hat + h * (math.cos(psi) * u1 + math.sin(psi) * u2)
    z2 = (elbow - p1) / d3
    out: List[Tuple[np.ndarray, Tuple[int, int, int], bool]] = []
    for q1, q2, mirror2 in _pair_solutions(z2, alpha[0], alpha[1]):
        rot2 = _rot_z(q1) @ _rot_x(alpha[0]) @ _rot_z(q2) @ _rot_x(alpha[1])
        z4_world = (wrist_center - elbow) / d5
        local = rot2.T @ z4_world
        for q3, q4, mirror4 in _pair_solutions(local, alpha[2], alpha[3]):
            rot4 = (
                rot2
                @ _rot_z(q3)
                @ _rot_x(alpha[2])
                @ _rot_z(q4)
                @ _rot_x(alpha[3])
            )
            matrix = rot4.T @ rot_target
            for q5, q6, q7, mirror6, singular in _wrist_solutions(
                matrix, alpha[4], alpha[5]
            ):
                q = np.array([q1, q2, q3, q4, q5, q6, q7], dtype=float)
                out.append((q, (mirror2, mirror4, mirror6), singular))
    return out


def _analyze(
    target_pose: Any,
    *,
    psi: Optional[float],
    alpha: Optional[Sequence[float]],
    tool_offset: Optional[float],
    d: Optional[Sequence[float]],
    psi_count: int,
    branch: Optional[Tuple[int, int, int]] = None,
) -> Tuple[str, List[Solution]]:
    """内部求解：返回 ``(status, solutions)``。

    ``status`` ∈ ``{"ok", "unreachable", "invalid"}``；``solutions`` 是
    **未经限位过滤**的全部几何解（限位过滤由调用方做，便于区分
    「限位外」与「无解」两类失败）。
    ``branch`` 给定时只保留该镜像分支（热循环用：已知分支后只需验 1 条解）。
    """
    try:
        pose = _as_pose(target_pose)
        d_table, alpha_table = _resolve_tables(d, alpha)
    except ValueError as exc:
        raise _InvalidTarget(str(exc)) from exc

    if psi_count < 1:
        raise ValueError(f"psi_count 必须是正整数，收到 {psi_count}")
    # 工具偏置默认跟随 d 表末项（裸臂 d7 = 0.144 m）。
    tool = float(d_table[6] if tool_offset is None else tool_offset)

    position = pose[:3, 3]
    rot_target = pose[:3, :3]
    wrist_center = position - tool * rot_target[:, 2]

    circle = _elbow_circle(wrist_center, d_table)
    if circle is None:
        return "unreachable", []

    if psi is None:
        psi_values = [2.0 * math.pi * k / psi_count for k in range(psi_count)]
    else:
        psi_values = [float(psi)]

    solutions: List[Solution] = []
    seen: set = set()
    for psi_value in psi_values:
        for q, cand_branch, singular in _solutions_for_psi(
            wrist_center, rot_target, psi_value, d_table, alpha_table, circle
        ):
            key = tuple(np.round(q, 9))
            if key in seen:
                continue
            seen.add(key)
            frame = mdh.fk(q, d=d_table, alpha=alpha_table)
            # TCP 相对 link_7 原点沿工具 z 轴偏移 (tool - d7)。
            tcp_pos = frame[:3, 3] + (tool - float(d_table[6])) * frame[:3, 2]
            pos_err = float(np.linalg.norm(tcp_pos - position))
            ori_err = _angle_between(frame[:3, :3], rot_target)
            if pos_err > 1e-7 or ori_err > 1e-7:
                # 数值上不可信的候选（分支退化等）直接丢弃。
                continue
            solutions.append(
                Solution(
                    q=q,
                    psi=float(psi_value),
                    branch=cand_branch,
                    wrist_singular=bool(singular),
                    position_error=pos_err,
                    orientation_error=ori_err,
                )
            )
    if branch is not None:
        wanted = tuple(branch)
        solutions = [s for s in solutions if s.branch == wanted]
    return "ok", solutions


class _InvalidTarget(ValueError):
    """内部信号：目标位姿非法。"""


# ── 公开 API ─────────────────────────────────────────────────────────────
def analytic_solutions(
    target_pose: Any,
    *,
    psi: Optional[float] = None,
    alpha: Optional[Sequence[float]] = None,
    tool_offset: Optional[float] = None,
    d: Optional[Sequence[float]] = None,
    psi_count: int = DEFAULT_PSI_COUNT,
    limits: Optional[Sequence[Tuple[float, float]]] = None,
    branch: Optional[Tuple[int, int, int]] = None,
) -> List[Solution]:
    """枚举目标位姿的**全部**解析解（8 组镜像分支 × ψ 网格）。

    参数
    ----
    target_pose:
        目标 TCP 位姿：4×4 齐次矩阵，或 ``(3,) 位置 + (3,3) 旋转`` 二元组。
    psi:
        指定肘点方位角（rad）。``None`` 表示按 ``psi_count`` 在
        ``[0, 2π)`` 上均匀扫描。
    alpha:
        alpha 表覆盖（默认 :data:`rm75_drake.kinematics.mdh.MDH_ALPHA`；
        理论对照可用 ``MDH_ALPHA_EXACT_PI_2``）。
    tool_offset:
        工具偏置（m），默认 :data:`rm75_drake.kinematics.mdh.MDH_D[6]`
        （裸臂 0.144 m）。**假定偏置沿 J7 轴**。
    d:
        d 表覆盖（默认 ``MDH_D``）。
    psi_count:
        ψ 采样数（默认 16）。
    limits:
        软件限位 ``[(lower, upper)] × 7``；``None`` 表示使用
        :func:`software_limits`（S01 真值 YAML）。
    branch:
        只保留该镜像分支 ``(q2 镜像, q4 镜像, q6 镜像)``。热循环（已知上一
        步分支）用它把每次求解从 ~130 条候选降到 1 条。

    返回
    ----
    list[Solution]
        全部候选解（已用 FK 复算校验）。目标不可达时返回空列表。

    异常
    ----
    ValueError
        目标位姿非法、参数表长度不对、``psi_count`` 非法等。
    """
    status, solutions = _analyze(
        target_pose,
        psi=psi,
        alpha=alpha,
        tool_offset=tool_offset,
        d=d,
        psi_count=psi_count,
        branch=branch,
    )
    if status == "unreachable":
        return []
    if limits is None:
        limits = software_limits()
    return [s for s in solutions if within_limits(s.q, limits)]


def solve_analytic_ik(
    target_pose: Any,
    *,
    seed: Optional[Sequence[float]] = None,
    branch_policy: str = "nearest",
    psi: Optional[float] = None,
    alpha: Optional[Sequence[float]] = None,
    tool_offset: Optional[float] = None,
    d: Optional[Sequence[float]] = None,
    psi_count: int = DEFAULT_PSI_COUNT,
    limits: Optional[Sequence[Tuple[float, float]]] = None,
    branch: Optional[Tuple[int, int, int]] = None,
) -> AnalyticIkResult:
    """求解单条解析 IK 解；**失败必须点名**。

    参数
    ----
    seed:
        参考关节角（rad）。``branch_policy="nearest"`` 时用它挑选「离上一个解
        最近」的分支/ψ（避免跳变，见 S05-T03 的连续性管理）。
    branch_policy:
        * ``"nearest"``：离 ``seed`` 最近的解（无 ``seed`` 时退化为限位裕度最大）；
        * ``"max_margin"``：限位裕度最大的解；
        * ``"first"``：枚举顺序上的第一条解（便于复现）。
    其余参数见 :func:`analytic_solutions`。

    返回
    ----
    AnalyticIkResult
        ``success=True`` 时 ``q``/``psi``/``branch`` 有效；
        失败时 ``reason`` ∈ ``unreachable`` / ``branch_exhausted`` /
        ``limit_violation`` / ``wrist_singular`` / ``invalid_target``。
    """
    if branch_policy not in ("nearest", "max_margin", "first"):
        raise ValueError(
            f"branch_policy 必须是 'nearest' / 'max_margin' / 'first'，收到 {branch_policy!r}"
        )
    try:
        status, solutions = _analyze(
            target_pose,
            psi=psi,
            alpha=alpha,
            tool_offset=tool_offset,
            d=d,
            psi_count=psi_count,
            branch=branch,
        )
    except _InvalidTarget as exc:
        return AnalyticIkResult(
            success=False,
            reason=REASON_INVALID_TARGET,
            message=f"目标位姿非法：{exc}",
        )
    except ValueError as exc:
        return AnalyticIkResult(
            success=False,
            reason=REASON_INVALID_TARGET,
            message=f"参数非法：{exc}",
        )

    if status == "unreachable":
        return AnalyticIkResult(
            success=False,
            reason=REASON_UNREACHABLE,
            message="目标腕心落在手臂可达包络之外（d3−d5 ≤ |W−P1| ≤ d3+d5 不满足）",
        )

    bounds = software_limits() if limits is None else tuple(limits)
    admissible = [s for s in solutions if within_limits(s.q, bounds)]
    non_singular = [s for s in admissible if not s.wrist_singular]
    if not solutions:
        return AnalyticIkResult(
            success=False,
            reason=REASON_BRANCH_EXHAUSTED,
            message="8 组镜像分支 × ψ 网格枚举完毕，没有可行解（球腕与臂的几何不相容）",
        )
    if not admissible:
        return AnalyticIkResult(
            success=False,
            reason=REASON_LIMIT_VIOLATION,
            message="全部几何解都违反软件限位",
            solutions=list(solutions),
        )
    if not non_singular:
        best = max(admissible, key=lambda s: limit_margin(s.q, bounds))
        return AnalyticIkResult(
            success=False,
            reason=REASON_WRIST_SINGULAR,
            message="全部可行解都落在腕奇异位形（sin q6 ≈ 0），球腕分解退化",
            q=best.q,
            solutions=list(admissible),
            psi=best.psi,
            branch=best.branch,
            wrist_singular=True,
        )

    pool = non_singular
    if branch_policy == "first":
        chosen = pool[0]
    elif branch_policy == "max_margin":
        chosen = max(pool, key=lambda s: limit_margin(s.q, bounds))
    elif branch_policy == "nearest":
        if seed is None:
            chosen = max(pool, key=lambda s: limit_margin(s.q, bounds))
        else:
            seed_arr = np.asarray(seed, dtype=float).reshape(-1)
            if seed_arr.size != mdh.N_JOINTS:
                raise ValueError(
                    f"seed 需要 {mdh.N_JOINTS} 个关节角，收到 {seed_arr.size} 个"
                )
            chosen = min(pool, key=lambda s: float(np.linalg.norm(s.q - seed_arr)))
    else:  # pragma: no cover - 入口已校验
        raise ValueError(
            f"branch_policy 必须是 'nearest' / 'max_margin' / 'first'，收到 {branch_policy!r}"
        )

    return AnalyticIkResult(
        success=True,
        reason="solved",
        message="解析 IK 求解成功",
        q=chosen.q,
        solutions=list(pool),
        psi=chosen.psi,
        branch=chosen.branch,
        wrist_singular=False,
    )
