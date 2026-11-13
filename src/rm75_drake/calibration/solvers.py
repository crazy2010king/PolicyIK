"""标定求解器（S04-T04）：**纯 numpy、零 pydrake 依赖**。

三个求解器
----------

``solve_joint_zero_offsets(samples, *, fk_fn)``
    逐关节 **+10° 对比 FK** 反解零位偏差表（rad）。
    做法：对每个关节，把「nominal 配置」与「+10° 配置」两条**观测 FK 位姿**
    同时作为约束，在该关节角上做 1 维搜索（粗网格定位 + 黄金分割细化），
    最小化模型 FK 与观测 FK 的位姿差。``fk_fn`` 由调用方注入
    （生产用 :func:`rm75_drake.kinematics.mdh.fk`），本模块**不 import Drake**。
``solve_tool_tcp(point_pairs)``
    **Kabsch / Umeyama**（SVD 闭式解）解 TCP 的 6D（旋转 + 平移）。
``solve_base_extrinsic(point_pairs)``
    尖对尖最小二乘解 ``X_LR``（``argmin Σ‖X·p_R^i − p_L^i‖²``，与上者是同一个闭式解）。

失败纪律
--------

三个求解器统一返回 :class:`ResidualReport`（逐点残差 / RMS / 最大残差 / 样本数 /
阈值 / ``within_threshold``）。**残差超阈值一律抛**
:class:`rm75_drake.calibration.schema.CalibrationError`（消息点名超差量与阈值），
**禁止**返回「看起来正常」的结果（研究结论第 3 条）。

单位：长度残差 m，角度 rad。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from .samples import BaseExtrinsicSample, JointZeroSample, ToolTcpSample
from .schema import CalibrationError

__all__ = [
    "ResidualReport",
    "JointZeroSolution",
    "ToolTcpSolution",
    "BaseExtrinsicSolution",
    "solve_joint_zero_offsets",
    "solve_tool_tcp",
    "solve_base_extrinsic",
    "kabsch",
    "DEFAULT_JOINT_ZERO_THRESHOLD_M",
    "DEFAULT_TOOL_TCP_THRESHOLD_M",
    "DEFAULT_BASE_EXTRINSIC_THRESHOLD_M",
]

#: 关节零位残差默认阈值（m）。设计文档目标：关节零位 < 0.1°（≈1.7e-3 rad）。
DEFAULT_JOINT_ZERO_THRESHOLD_M = 1.0e-3

#: 工具 TCP 残差默认阈值（m）。设计文档目标：四点/六点法 < 0.5 mm。
DEFAULT_TOOL_TCP_THRESHOLD_M = 5.0e-4

#: 基座外参残差默认阈值（m）。设计文档目标：双基座相对位姿 < 0.5 mm。
DEFAULT_BASE_EXTRINSIC_THRESHOLD_M = 5.0e-4

#: 工具 TCP 最少点对数（四点法下限）。
_MIN_TOOL_POINTS = 4

#: 基座外参最少点对数（设计文档 15–20 组）。
_MIN_BASE_POINTS = 15

#: 零位搜索的粗网格点数（δ ∈ [-π, π]）。
_N_OFFSET_GRID = 129

#: 不可辨识判定：目标函数在 0.1 rad 扰动下的变化量低于此值即认为该关节
#: 的零位偏差**不可辨识**（FK 对该关节不敏感）。
_UNIDENTIFIABLE_EPS = 1.0e-12


# ── 残差报告 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ResidualReport:
    """残差报告（统一口径：逐点残差 + RMS + 最大残差 + 样本数 + 阈值）。"""

    name: str
    residuals: Tuple[float, ...]
    threshold: float
    unit: str = "m"

    @property
    def sample_count(self) -> int:
        return len(self.residuals)

    @property
    def rms(self) -> float:
        if not self.residuals:
            return 0.0
        return math.sqrt(sum(r * r for r in self.residuals) / len(self.residuals))

    @property
    def max_residual(self) -> float:
        if not self.residuals:
            return 0.0
        return max(self.residuals)

    @property
    def within_threshold(self) -> bool:
        return self.max_residual <= self.threshold

    def as_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "rms": self.rms,
            "max_residual": self.max_residual,
            "sample_count": self.sample_count,
            "threshold": self.threshold,
            "within_threshold": self.within_threshold,
            "unit": self.unit,
            "residuals": list(self.residuals),
        }

    def raise_if_exceeded(self) -> None:
        """残差超阈值 → 抛 :class:`CalibrationError`（点名超差量与阈值）。"""
        if self.within_threshold:
            return
        raise CalibrationError(
            f"{self.name} 残差超阈值：最大残差 {self.max_residual:.6e} {self.unit} "
            f"> 阈值 {self.threshold:.6e} {self.unit}"
            f"（RMS {self.rms:.6e} {self.unit}，样本数 {self.sample_count}）；"
            "禁止返回超差结果——请检查采样质量 / 工装 / 或放宽阈值前先确认数据可信"
        )


@dataclass(frozen=True)
class JointZeroSolution:
    """关节零位偏差解。"""

    offsets: Mapping[str, float]
    report: ResidualReport

    def as_dict(self) -> Dict[str, Any]:
        return {
            "offsets": {k: float(v) for k, v in self.offsets.items()},
            "report": self.report.as_dict(),
        }


@dataclass(frozen=True)
class ToolTcpSolution:
    """工具 TCP 6D 解（旋转矩阵 + 平移）。"""

    rotation: np.ndarray
    translation: np.ndarray
    report: ResidualReport

    def as_dict(self) -> Dict[str, Any]:
        return {
            "rotation": [[float(v) for v in row] for row in self.rotation],
            "translation": [float(v) for v in self.translation],
            "report": self.report.as_dict(),
        }


@dataclass(frozen=True)
class BaseExtrinsicSolution:
    """双臂基座外参解 ``X_LR``（4×4 齐次矩阵：右臂基座系 → 左臂基座系）。"""

    matrix: np.ndarray
    report: ResidualReport

    def as_dict(self) -> Dict[str, Any]:
        return {
            "matrix": [[float(v) for v in row] for row in self.matrix],
            "report": self.report.as_dict(),
        }


# ── Kabsch / Umeyama 闭式解 ─────────────────────────────────────────────
def kabsch(source: Sequence[Sequence[float]], target: Sequence[Sequence[float]]) -> Tuple[np.ndarray, np.ndarray]:
    """Kabsch / Umeyama 闭式解（SVD）：求 ``R, t`` 使 ``target ≈ R @ source + t``。

    参数
    ----
    source, target:
        ``(N, 3)`` 点集（N ≥ 3）。

    返回
    ----
    ``(R, t)``：3×3 旋转矩阵（行列式为 +1）与 3 维平移。

    异常
    ----
    CalibrationError
        点数不足 3、维度不对或点集退化（共面且退化等）。
    """
    src = np.asarray(source, dtype=float)
    dst = np.asarray(target, dtype=float)
    if src.ndim != 2 or src.shape[1] != 3:
        raise CalibrationError(f"source 必须是 (N, 3)，收到形状 {src.shape}")
    if dst.ndim != 2 or dst.shape[1] != 3:
        raise CalibrationError(f"target 必须是 (N, 3)，收到形状 {dst.shape}")
    if src.shape != dst.shape:
        raise CalibrationError(f"source/target 点数不一致：{src.shape} vs {dst.shape}")
    if src.shape[0] < 3:
        raise CalibrationError(f"Kabsch 至少需要 3 组点对，收到 {src.shape[0]} 组")
    if not np.all(np.isfinite(src)) or not np.all(np.isfinite(dst)):
        raise CalibrationError("点集中含 NaN/Inf")

    mu_s = src.mean(axis=0)
    mu_t = dst.mean(axis=0)
    x = src - mu_s
    y = dst - mu_t
    cov = x.T @ y
    try:
        u, _s, vt = np.linalg.svd(cov)
    except np.linalg.LinAlgError as exc:  # pragma: no cover - 数值退化
        raise CalibrationError(f"Kabsch 的 SVD 失败（点集退化）：{exc}") from exc
    v = vt.T
    d = np.sign(np.linalg.det(v @ u.T))
    if d == 0.0:  # pragma: no cover - 理论上不会发生
        d = 1.0
    rot = v @ np.diag([1.0, 1.0, d]) @ u.T
    trans = mu_t - rot @ mu_s
    return rot, trans


# ── 内部工具 ────────────────────────────────────────────────────────────
def _as_points(value: Any, path: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise CalibrationError(f"{path} 必须是 (N, 3) 点集，收到形状 {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise CalibrationError(f"{path} 含 NaN/Inf")
    return arr


def _as_point_pairs(point_pairs: Any, path: str) -> Tuple[np.ndarray, np.ndarray]:
    """把点对序列统一成 ``(source, target)`` 两个 ``(N, 3)`` 数组。

    支持两种写法：
    * ``((local, world), ...)`` 二元组序列；
    * :class:`rm75_drake.calibration.samples.ToolTcpSample` /
      :class:`rm75_drake.calibration.samples.BaseExtrinsicSample` 对象序列。
    """
    items = list(point_pairs)
    if not items:
        raise CalibrationError(f"{path} 不能为空")
    sources = []
    targets = []
    for i, item in enumerate(items):
        if isinstance(item, ToolTcpSample):
            sources.append(item.local_point)
            targets.append(item.world_point)
        elif isinstance(item, BaseExtrinsicSample):
            sources.append(item.right_point)
            targets.append(item.left_point)
        elif hasattr(item, "local_point") and hasattr(item, "world_point"):
            sources.append(item.local_point)
            targets.append(item.world_point)
        elif hasattr(item, "right_point") and hasattr(item, "left_point"):
            sources.append(item.right_point)
            targets.append(item.left_point)
        elif isinstance(item, (tuple, list)) and len(item) == 2:
            sources.append(item[0])
            targets.append(item[1])
        else:
            raise CalibrationError(
                f"{path}[{i}] 必须是 (source, target) 二元组或采样对象，"
                f"收到 {type(item).__name__}"
            )
    return _as_points(sources, f"{path}.source"), _as_points(targets, f"{path}.target")


def _golden_min(
    func: Callable[[float], float], a: float, b: float, *, tol: float = 1.0e-14, max_iter: int = 400
) -> float:
    """黄金分割法一维极小（区间 [a, b] 内假定单峰）。"""
    inv_phi = (math.sqrt(5.0) - 1.0) / 2.0
    c = b - inv_phi * (b - a)
    d = a + inv_phi * (b - a)
    fc = func(c)
    fd = func(d)
    for _ in range(max_iter):
        if b - a < tol:
            break
        if fc <= fd:
            b, d, fd = d, c, fc
            c = b - inv_phi * (b - a)
            fc = func(c)
        else:
            a, c, fc = c, d, fd
            d = a + inv_phi * (b - a)
            fd = func(d)
    return 0.5 * (a + b)


def _pose_error(model: np.ndarray, observed: np.ndarray) -> float:
    """4×4 齐次位姿的 Frobenius 差（位置 m + 旋转无量纲，作为同一目标函数用）。"""
    diff = model - observed
    return float(np.sum(diff * diff))


# ── 求解器 1：关节零位偏差 ──────────────────────────────────────────────
def solve_joint_zero_offsets(
    samples: Sequence[JointZeroSample],
    *,
    fk_fn: Callable[[Sequence[float]], np.ndarray],
    threshold: float = DEFAULT_JOINT_ZERO_THRESHOLD_M,
) -> JointZeroSolution:
    """逐关节 +10° 对比 FK，反解零位偏差表（rad）。

    参数
    ----
    samples:
        :class:`rm75_drake.calibration.samples.JointZeroSample` 序列
        （每个关节一条：nominal 与 +10° 两条 commanded 角 + 两条观测 FK 位姿）。
    fk_fn:
        注入的模型正运动学：``fk_fn(q) -> 4×4`` 齐次位姿矩阵（**不 import Drake**）。
    threshold:
        残差阈值（m，默认 :data:`DEFAULT_JOINT_ZERO_THRESHOLD_M`）。

    返回
    ----
    JointZeroSolution
        ``offsets``（关节名 → rad）与 ``report``（残差报告）。

    异常
    ----
    CalibrationError
        采样为空 / 采样结构非法 / 该关节零位**不可辨识** / 残差超阈值。
    """
    items = list(samples)
    if not items:
        raise CalibrationError("joint_zero 采样为空：无法反解零位偏差")

    offsets: Dict[str, float] = {}
    residuals: list[float] = []

    for i, sample in enumerate(items):
        path = f"joint_zero[{i}]"
        sample.validate(path)

        q_nom = np.asarray(sample.q_nominal, dtype=float)
        q_off = np.asarray(sample.q_offset, dtype=float)
        p_nom = np.asarray(sample.pose_nominal, dtype=float)
        p_off = np.asarray(sample.pose_offset, dtype=float)
        if p_nom.shape != (4, 4) or p_off.shape != (4, 4):
            raise CalibrationError(f"{path}.pose_nominal / pose_offset 必须是 4×4 矩阵")
        idx = sample.index

        def objective(delta: float, _q_nom=q_nom, _q_off=q_off, _p_nom=p_nom, _p_off=p_off, _i=idx):
            q1 = _q_nom.copy()
            q1[_i] = _q_nom[_i] + delta
            q2 = _q_off.copy()
            q2[_i] = _q_off[_i] + delta
            model1 = np.asarray(fk_fn(q1.tolist()), dtype=float)
            model2 = np.asarray(fk_fn(q2.tolist()), dtype=float)
            if model1.shape != (4, 4) or model2.shape != (4, 4):
                raise CalibrationError(
                    f"{path}: fk_fn 必须返回 4×4 齐次位姿矩阵，收到 {model1.shape} / {model2.shape}"
                )
            return _pose_error(model1, _p_nom) + _pose_error(model2, _p_off)

        # 不可辨识检查：目标函数对 δ 扰动不敏感 ⇒ 该关节的零位无法从 FK 反解。
        sensitivity = abs(objective(0.0) - objective(0.1))
        if sensitivity < _UNIDENTIFIABLE_EPS:
            raise CalibrationError(
                f"{path}（关节 {sample.joint!r}）的零位偏差**不可辨识**："
                "FK 对该关节不敏感（目标函数扰动量 "
                f"{sensitivity:.3e} < {_UNIDENTIFIABLE_EPS:.1e}）；"
                "该关节必须换用编码器读数或外部测量手段标定"
            )

        # 粗网格定位（δ ∈ [-π, π]，周期 2π）。
        grid = np.linspace(-math.pi, math.pi, _N_OFFSET_GRID)
        values = [objective(float(g)) for g in grid]
        best = int(np.argmin(values))
        step = float(grid[1] - grid[0])
        lo = max(-math.pi, float(grid[best]) - step)
        hi = min(math.pi, float(grid[best]) + step)

        delta = _golden_min(objective, lo, hi)
        offsets[sample.joint] = float(delta)

        # 残差（m）：取 nominal / +10° 两条观测位姿的**位置**误差较大者。
        q1 = q_nom.copy()
        q1[idx] = q_nom[idx] + delta
        q2 = q_off.copy()
        q2[idx] = q_off[idx] + delta
        e1 = float(np.linalg.norm(np.asarray(fk_fn(q1.tolist()))[:3, 3] - p_nom[:3, 3]))
        e2 = float(np.linalg.norm(np.asarray(fk_fn(q2.tolist()))[:3, 3] - p_off[:3, 3]))
        residuals.append(max(e1, e2))

    report = ResidualReport(
        name="joint_zero_offset",
        residuals=tuple(residuals),
        threshold=float(threshold),
        unit="m",
    )
    report.raise_if_exceeded()
    return JointZeroSolution(offsets=offsets, report=report)


# ── 求解器 2：工具 TCP 6D ──────────────────────────────────────────────
def solve_tool_tcp(
    point_pairs: Any,
    *,
    threshold: float = DEFAULT_TOOL_TCP_THRESHOLD_M,
) -> ToolTcpSolution:
    """Kabsch / Umeyama 解工具 TCP 的 6D（旋转 + 平移）。

    ``point_pairs`` 是「局部点 ↔ 世界点」序列（≥ 4 组，四点法下限）。

    异常
    ----
    CalibrationError
        点对不足 4 组 / 维度错 / 残差超阈值（**不静默输出**）。
    """
    src, dst = _as_point_pairs(point_pairs, "tool_tcp")
    if src.shape[0] < _MIN_TOOL_POINTS:
        raise CalibrationError(
            f"tool_tcp 至少需要 {_MIN_TOOL_POINTS} 组点对（四点法下限），收到 {src.shape[0]} 组"
        )

    rot, trans = kabsch(src, dst)
    fitted = src @ rot.T + trans
    residuals = tuple(float(np.linalg.norm(fitted[i] - dst[i])) for i in range(dst.shape[0]))

    report = ResidualReport(
        name="tool_tcp",
        residuals=residuals,
        threshold=float(threshold),
        unit="m",
    )
    report.raise_if_exceeded()
    return ToolTcpSolution(rotation=rot, translation=trans, report=report)


# ── 求解器 3：双臂基座外参 X_LR ────────────────────────────────────────
def solve_base_extrinsic(
    point_pairs: Any,
    *,
    threshold: float = DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
) -> BaseExtrinsicSolution:
    """尖对尖最小二乘解 ``X_LR``（右臂基座系 → 左臂基座系）。

    ``point_pairs`` 是「左点 ↔ 右点」序列（≥ 15 组，设计文档要求 15–20 组）；
    求解 ``argmin Σ‖X·p_R^i − p_L^i‖²``（同一个 Kabsch 闭式解）。

    异常
    ----
    CalibrationError
        点对不足 15 组 / 维度错 / 残差超阈值（**不静默输出**）。
    """
    src, dst = _as_point_pairs(point_pairs, "base_extrinsic")
    if src.shape[0] < _MIN_BASE_POINTS:
        raise CalibrationError(
            f"base_extrinsic 至少需要 {_MIN_BASE_POINTS} 组「尖对尖」点对，"
            f"收到 {src.shape[0]} 组"
        )

    rot, trans = kabsch(src, dst)
    matrix = np.eye(4)
    matrix[:3, :3] = rot
    matrix[:3, 3] = trans
    fitted = src @ rot.T + trans
    residuals = tuple(float(np.linalg.norm(fitted[i] - dst[i])) for i in range(dst.shape[0]))

    report = ResidualReport(
        name="base_extrinsic",
        residuals=residuals,
        threshold=float(threshold),
        unit="m",
    )
    report.raise_if_exceeded()
    return BaseExtrinsicSolution(matrix=matrix, report=report)
