"""标定作业流水线（S04-T08）：**CLI 与脚本共用的唯一实现**。

为什么单独一个模块
------------------

``python3 -m rm75_drake.cli calibrate`` 与 ``scripts/calibrate_dual_arm.py``
必须给出**同一份**结论（同样的 status / reason / 残差 / 未验证清单），
否则「CLI 说 OK、脚本说 SKIP」就会变成第二套真相源。因此解算编排放在**包内**，
CLI 与脚本都只是它的薄壳（渲染 JSON + 决定退出码）。

纪律
----

* **不把未实测的物理指标呈现为通过**（对齐 MEM083 / 研究结论第 2、9 条）：
  采样是合成的、或标定文件 ``measured=false`` 时，整体状态只能是
  ``skipped``（原因标签 ``calibration_not_measured``），
  并**逐条列出未验证的物理指标**；
* **残差超阈值 ⇒ 失败**（退出码 1），绝不返回「看起来正常」的结果；
* **用法错误**（路径缺失 / 参数非法）单独一类异常
  (:class:`CalibrationUsageError`) → 退出码 2，与既有 ``cli.replay`` /
  ``cli.exec`` 的用法错误语义一致；
* 本模块**零 pydrake 依赖**（只用 numpy + 标准库 + PyYAML）。

FK 口径（``--fk``）
------------------

关节零位求解需要「模型正运动学」当作**被测对照**：

``model``
    :func:`rm75_drake.kinematics.mdh.fk`（MDH 独立正解，纯 numpy）——
    **实测数据的口径**。
``synthetic``
    合成演练 FK（``fk(q)`` 的平移 = ``[0.5*q0, 0.5*q1, 0.5*q2]``，旋转为单位阵），
    与 ``calibration/samples/demo.yaml`` 头部声明的约定一致 —— **仅用于合成数据的
    离线演练**，不得用于实测结论。
``auto``（默认）
    采样里的 ``joint_zero`` **全部**标注 ``synthetic=true`` ⇒ 用 ``synthetic``；
    否则用 ``model``。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .report import repeatability_summary
from .samples import SampleSet, load_samples_file
from .schema import CalibrationDoc, CalibrationError, load_calibration_file
from .solvers import (
    DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
    DEFAULT_JOINT_ZERO_THRESHOLD_M,
    DEFAULT_TOOL_TCP_THRESHOLD_M,
    kabsch,
    solve_base_extrinsic,
    solve_joint_zero_offsets,
    solve_tool_tcp,
)

__all__ = [
    "REASON_CALIBRATION_NOT_MEASURED",
    "REASON_RESIDUAL_EXCEEDED",
    "REASON_SOLVER_ERROR",
    "UNVERIFIED_CALIBRATION_METRICS",
    "CalibrationUsageError",
    "SectionResult",
    "CalibrationRun",
    "run_calibration",
    "synthetic_fk",
    "model_fk",
]


#: 「无实测数据」时的**稳定原因标签**（脚本与 CI 按它判定 SKIP）。
REASON_CALIBRATION_NOT_MEASURED = "calibration_not_measured"

#: 残差超阈值的原因标签。
REASON_RESIDUAL_EXCEEDED = "residual_exceeded"

#: 求解器其它错误的原因标签。
REASON_SOLVER_ERROR = "solver_error"

#: 无实测数据时**必须逐条列出**的未验证物理指标（绝不假装通过、绝不省略）。
UNVERIFIED_CALIBRATION_METRICS: Tuple[str, ...] = (
    "真实标定精度：关节零位偏差（逐关节 +10° 实测对比，目标 < 0.1°）",
    "真实标定精度：工具 TCP 6D（四点/六点法，目标 < 0.5 mm）",
    "真实标定精度：双臂基座外参 X_LR（尖对尖最小二乘，目标 < 0.5 mm / 0.2°）",
    "双臂共同工作区 TCP 同一点偏差 < 0.5 mm（依赖真实标定数据）",
    "双臂同一 tick 两臂下发时间差 < 1 ms（依赖真机）",
    "100 次双臂同空间运动无碰撞（依赖真机）",
)

#: 标定本身**不能**证明的指标（即便数据是实测的，这些仍然未验证）。
_METRICS_NOT_COVERED_BY_CALIBRATION: Tuple[str, ...] = UNVERIFIED_CALIBRATION_METRICS[3:]

#: 「不可辨识」关节的错误特征（求解器消息里的稳定措辞）。
_UNIDENTIFIABLE_MARK = "不可辨识"


class CalibrationUsageError(ValueError):
    """标定作业的**用法错误**（路径缺失 / 参数非法）→ CLI 退出码 2。"""


# ── FK 对照 ────────────────────────────────────────────────────────────
def model_fk(q: Sequence[float]) -> np.ndarray:
    """模型正运动学（MDH 独立正解，纯 numpy）——**实测数据的口径**。"""
    from ..kinematics.mdh import fk

    return fk(q)


def synthetic_fk(q: Sequence[float]) -> np.ndarray:
    """合成演练 FK：平移 ``[0.5*q0, 0.5*q1, 0.5*q2]``，旋转为单位阵。

    与 ``calibration/samples/demo.yaml`` 头部声明的约定一致；**仅用于合成数据演练**。
    """
    angles = np.asarray(q, dtype=float).reshape(-1)
    matrix = np.eye(4)
    for i in range(min(3, angles.size)):
        matrix[i, 3] = 0.5 * float(angles[i])
    return matrix


# ── 结果结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class SectionResult:
    """单个标定段（joint_zero / tool_tcp / base_extrinsic）的结果。"""

    name: str
    status: str  # ok | failed | absent | partial
    reason: str = ""
    detail: str = ""
    residuals: Tuple[float, ...] = ()
    threshold: float = 0.0
    unit: str = "m"
    extra: Mapping[str, Any] = field(default_factory=dict)

    @property
    def rms(self) -> float:
        if not self.residuals:
            return 0.0
        return math.sqrt(sum(r * r for r in self.residuals) / len(self.residuals))

    @property
    def max_residual(self) -> float:
        return max(self.residuals) if self.residuals else 0.0

    @property
    def within_threshold(self) -> bool:
        return self.max_residual <= self.threshold

    def as_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "reason": self.reason,
            "detail": self.detail,
            "rms": self.rms,
            "max": self.max_residual,
            "max_residual": self.max_residual,
            "sample_count": len(self.residuals),
            "threshold": self.threshold,
            "within_threshold": self.within_threshold,
            "unit": self.unit,
            "residuals": list(self.residuals),
            **dict(self.extra),
        }


@dataclass(frozen=True)
class CalibrationRun:
    """一次标定作业的完整结果（CLI / 脚本的共同真相源）。"""

    status: str  # ok | skipped | failed
    reason: str
    measured: bool
    samples_synthetic: bool
    accept_synthetic: bool
    fk_mode: str
    samples_path: str
    calibration_path: str
    calibration: Optional[CalibrationDoc] = None
    sections: Mapping[str, SectionResult] = field(default_factory=dict)
    repeatability: Mapping[str, Any] = field(default_factory=dict)
    cross_validation: Mapping[str, Any] = field(default_factory=dict)
    unverified_metrics: Tuple[str, ...] = ()
    detail: str = ""

    @property
    def exit_code(self) -> int:
        """退出码：0 = ok / skipped（硬件门控语义），1 = 失败。"""
        return 1 if self.status == "failed" else 0

    @property
    def residuals(self) -> Dict[str, Any]:
        """逐段残差摘要（RMS / 最大残差 / 逐点残差 / 阈值）。"""
        return {
            name: {
                "rms": section.rms,
                "max": section.max_residual,
                "max_residual": section.max_residual,
                "sample_count": len(section.residuals),
                "threshold": section.threshold,
                "within_threshold": section.within_threshold,
                "unit": section.unit,
                "residuals": list(section.residuals),
            }
            for name, section in self.sections.items()
            if section.status in ("ok", "partial", "failed")
        }

    def as_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "command": "calibrate",
            "status": self.status,
            "reason": self.reason,
            "measured": self.measured,
            "samples_synthetic": self.samples_synthetic,
            "accept_synthetic": self.accept_synthetic,
            "fk_mode": self.fk_mode,
            "samples_path": self.samples_path,
            "calibration_path": self.calibration_path,
            "calibration": (
                None
                if self.calibration is None
                else {
                    "measured": self.calibration.measured,
                    "calibrated": self.calibration.calibrated,
                    "method": self.calibration.method,
                    "date": self.calibration.date,
                }
            ),
            "residuals": self.residuals,
            "sections": {name: section.as_dict() for name, section in self.sections.items()},
            "repeatability": dict(self.repeatability),
            "cross_validation": dict(self.cross_validation),
            "unverified_metrics": list(self.unverified_metrics),
            "detail": self.detail,
        }
        return payload


# ── 内部工具 ────────────────────────────────────────────────────────────
def _require_path(value: Any, label: str) -> Path:
    """把路径参数收敛成 :class:`Path`；空值 → :class:`CalibrationUsageError`。"""
    if value is None:
        raise CalibrationUsageError(f"{label} 不能为空")
    text = str(value).strip()
    if not text:
        raise CalibrationUsageError(f"{label} 不能为空（用法错误）")
    return Path(text)


def _load_samples(path: Path) -> SampleSet:
    if not path.is_file():
        raise CalibrationUsageError(f"采样文件不存在：{path}")
    try:
        return load_samples_file(path)
    except FileNotFoundError as exc:  # pragma: no cover - 上面已判存在
        raise CalibrationUsageError(f"采样文件不存在：{path}") from exc
    except CalibrationError as exc:
        raise CalibrationError(f"采样文件非法：{path}（{exc}）") from exc


def _load_calibration(path: Path) -> CalibrationDoc:
    if not path.is_file():
        raise CalibrationUsageError(f"标定文件不存在：{path}")
    try:
        return load_calibration_file(path)
    except FileNotFoundError as exc:  # pragma: no cover - 上面已判存在
        raise CalibrationUsageError(f"标定文件不存在：{path}") from exc
    except CalibrationError as exc:
        raise CalibrationError(f"标定文件非法：{path}（{exc}）") from exc


def _rotation_angle(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（rad）。"""
    rel = rot_a.T @ rot_b
    cos = (float(np.trace(rel)) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, cos))))


def _fold_solutions(
    source: np.ndarray, target: np.ndarray, k: int = 3
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """留一折（连续分块）重解：每折留出 ``n // k`` 个点，用其余点求解。

    折数会**自动下调**（3 → 2），保证「被留出的点 ≥ 3 且剩余点 ≥ 3」
    （Kabsch 至少需要 3 组点对）。
    """
    n = int(source.shape[0])
    for candidate in (k, 2):
        if candidate < 2 or n < 6:
            continue
        fold = n // candidate
        if fold < 3 or n - fold < 3:
            continue
        solutions: List[Tuple[np.ndarray, np.ndarray]] = []
        for i in range(candidate):
            held = set(range(i * fold, (i + 1) * fold))
            rest = [j for j in range(n) if j not in held]
            try:
                solutions.append(kabsch(source[rest], target[rest]))
            except CalibrationError:  # pragma: no cover - 折内点集退化
                continue
        if solutions:
            return solutions
    return []


def _jackknife(
    source: np.ndarray, target: np.ndarray, full_rot: np.ndarray, full_trans: np.ndarray
) -> Dict[str, Any]:
    """重复性摘要（留一折重解的离散度）。"""
    solutions = _fold_solutions(source, target, 3)
    if not solutions:
        return {"count": 0, "note": "样本不足，未做留一折重解"}
    trans_devs = [float(np.linalg.norm(trans - full_trans)) for _rot, trans in solutions]
    rot_devs = [_rotation_angle(full_rot, rot) for rot, _trans in solutions]
    summary = repeatability_summary(trans_devs)
    summary["max_rotation_deviation_rad"] = max(rot_devs)
    summary["unit"] = "m"
    return summary


def _split_half(
    source: np.ndarray, target: np.ndarray
) -> Dict[str, Any]:
    """交叉验证差异（连续两半各解一次，比较两份解）。"""
    n = int(source.shape[0])
    if n < 6:
        return {"count": 0, "note": "样本不足，未做两半交叉验证"}
    half = n // 2
    try:
        rot_a, trans_a = kabsch(source[:half], target[:half])
        rot_b, trans_b = kabsch(source[half:], target[half:])
    except CalibrationError as exc:  # pragma: no cover - 半集退化
        return {"count": 0, "note": f"两半交叉验证失败：{exc}"}
    return {
        "count": 2,
        "split": "contiguous_half",
        "translation_difference_m": float(np.linalg.norm(trans_a - trans_b)),
        "rotation_difference_rad": _rotation_angle(rot_a, rot_b),
        "first_half_points": half,
        "second_half_points": n - half,
    }


# ── 各标定段 ────────────────────────────────────────────────────────────
def _solve_joint_zero(
    samples: SampleSet, fk_fn, threshold: float
) -> SectionResult:
    """关节零位段：**逐关节**独立求解（不可辨识的关节单独记账，不拖垮整段）。"""
    offsets: Dict[str, float] = {}
    residuals: List[float] = []
    unidentifiable: List[str] = []
    failures: List[str] = []
    for sample in samples.joint_zero:
        try:
            solution = solve_joint_zero_offsets(
                [sample], fk_fn=fk_fn, threshold=float("inf")
            )
        except CalibrationError as exc:
            if _UNIDENTIFIABLE_MARK in str(exc):
                unidentifiable.append(sample.joint)
            else:
                failures.append(f"{sample.joint}：{exc}")
            continue
        offsets[sample.joint] = float(solution.offsets[sample.joint])
        residuals.extend(solution.report.residuals)

    extra: Dict[str, Any] = {"offsets": offsets}
    if unidentifiable:
        extra["unidentifiable_joints"] = unidentifiable
    if failures:
        extra["failures"] = failures

    if failures:
        status, reason = "failed", REASON_SOLVER_ERROR
        detail = "；".join(failures)
    elif unidentifiable and residuals:
        status, reason = "partial", "unidentifiable_joints"
        detail = (
            "以下关节的零位偏差不可辨识（FK 对其不敏感，须换编码器读数或外部测量）："
            + "、".join(unidentifiable)
        )
    elif unidentifiable:
        status, reason = "partial", "unidentifiable_joints"
        detail = (
            "全部关节的零位偏差均不可辨识（FK 对其不敏感）：" + "、".join(unidentifiable)
        )
    else:
        status, reason, detail = "ok", "", ""

    return SectionResult(
        name="joint_zero",
        status=status,
        reason=reason,
        detail=detail,
        residuals=tuple(residuals),
        threshold=float(threshold),
        unit="m",
        extra=extra,
    )


def _solve_tool_tcp(samples: SampleSet, threshold: float) -> SectionResult:
    if not samples.tool_tcp:
        return SectionResult(name="tool_tcp", status="absent", reason="no_samples")
    source = np.asarray([s.local_point for s in samples.tool_tcp], dtype=float)
    target = np.asarray([s.world_point for s in samples.tool_tcp], dtype=float)
    try:
        solution = solve_tool_tcp(samples.tool_tcp, threshold=float("inf"))
    except CalibrationError as exc:
        return SectionResult(
            name="tool_tcp",
            status="failed",
            reason=REASON_SOLVER_ERROR,
            detail=str(exc),
            threshold=float(threshold),
        )
    rot = np.asarray(solution.rotation, dtype=float)
    trans = np.asarray(solution.translation, dtype=float)
    extra = {
        "rotation": [[float(v) for v in row] for row in rot],
        "translation": [float(v) for v in trans],
        "repeatability": _jackknife(source, target, rot, trans),
        "cross_validation": _split_half(source, target),
    }
    return SectionResult(
        name="tool_tcp",
        status="ok",
        residuals=solution.report.residuals,
        threshold=float(threshold),
        extra=extra,
    )


def _solve_base_extrinsic(samples: SampleSet, threshold: float) -> SectionResult:
    if not samples.base_extrinsic:
        return SectionResult(name="base_extrinsic", status="absent", reason="no_samples")
    source = np.asarray([s.right_point for s in samples.base_extrinsic], dtype=float)
    target = np.asarray([s.left_point for s in samples.base_extrinsic], dtype=float)
    try:
        solution = solve_base_extrinsic(samples.base_extrinsic, threshold=float("inf"))
    except CalibrationError as exc:
        return SectionResult(
            name="base_extrinsic",
            status="failed",
            reason=REASON_SOLVER_ERROR,
            detail=str(exc),
            threshold=float(threshold),
        )
    rot = np.asarray(solution.matrix, dtype=float)[:3, :3]
    trans = np.asarray(solution.matrix, dtype=float)[:3, 3]
    extra = {
        "matrix": [[float(v) for v in row] for row in solution.matrix],
        "repeatability": _jackknife(source, target, rot, trans),
        "cross_validation": _split_half(source, target),
    }
    return SectionResult(
        name="base_extrinsic",
        status="ok",
        residuals=solution.report.residuals,
        threshold=float(threshold),
        extra=extra,
    )


def _apply_threshold(section: SectionResult) -> SectionResult:
    """把「残差超阈值」从 ok 翻成 failed（求解器本身用 inf 阈值以便拿到残差）。"""
    if section.status != "ok":
        return section
    if section.within_threshold:
        return section
    return SectionResult(
        name=section.name,
        status="failed",
        reason=REASON_RESIDUAL_EXCEEDED,
        detail=(
            f"{section.name} 残差超阈值：最大残差 {section.max_residual:.6e} {section.unit} "
            f"> 阈值 {section.threshold:.6e} {section.unit}"
            f"（RMS {section.rms:.6e} {section.unit}，样本数 {len(section.residuals)}）"
        ),
        residuals=section.residuals,
        threshold=section.threshold,
        unit=section.unit,
        extra=section.extra,
    )


# ── 主入口 ──────────────────────────────────────────────────────────────
def run_calibration(
    *,
    samples_path: Any,
    calibration_path: Any,
    accept_synthetic: bool = False,
    fk_mode: str = "auto",
    joint_zero_threshold: float = DEFAULT_JOINT_ZERO_THRESHOLD_M,
    tool_tcp_threshold: float = DEFAULT_TOOL_TCP_THRESHOLD_M,
    base_extrinsic_threshold: float = DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
) -> CalibrationRun:
    """跑一次标定作业，返回统一结果（**不抛业务异常**，失败体现在结果里）。

    异常
    ----
    CalibrationUsageError
        路径缺失 / 参数非法（→ 退出码 2）。
    CalibrationError
        采样文件或标定文件**内容**非法（→ 退出码 1）。
    """
    if fk_mode not in ("auto", "model", "synthetic"):
        raise CalibrationUsageError(f"--fk 只能是 auto / model / synthetic，收到 {fk_mode!r}")

    samples_file = _require_path(samples_path, "--samples")
    calibration_file = _require_path(calibration_path, "--calibration")

    samples = _load_samples(samples_file)
    doc = _load_calibration(calibration_file)

    groups = (samples.joint_zero, samples.tool_tcp, samples.base_extrinsic)
    all_samples = [item for group in groups for item in group]
    samples_synthetic = bool(all_samples) and all(bool(item.synthetic) for item in all_samples)

    # FK 口径选择
    effective_fk_mode = fk_mode
    if fk_mode == "auto":
        joint_all_synthetic = bool(samples.joint_zero) and all(
            bool(item.synthetic) for item in samples.joint_zero
        )
        effective_fk_mode = "synthetic" if joint_all_synthetic else "model"
    fk_fn = synthetic_fk if effective_fk_mode == "synthetic" else model_fk

    sections: Dict[str, SectionResult] = {
        "joint_zero": _solve_joint_zero(samples, fk_fn, joint_zero_threshold),
        "tool_tcp": _solve_tool_tcp(samples, tool_tcp_threshold),
        "base_extrinsic": _solve_base_extrinsic(samples, base_extrinsic_threshold),
    }
    sections = {name: _apply_threshold(section) for name, section in sections.items()}

    repeatability = {
        name: section.extra["repeatability"]
        for name, section in sections.items()
        if "repeatability" in section.extra
    }
    cross_validation = {
        name: section.extra["cross_validation"]
        for name, section in sections.items()
        if "cross_validation" in section.extra
    }

    measured = bool(doc.measured) and not samples_synthetic

    failed = [s for s in sections.values() if s.status == "failed"]
    if failed:
        status = "failed"
        reason = (
            REASON_RESIDUAL_EXCEEDED
            if any(s.reason == REASON_RESIDUAL_EXCEEDED for s in failed)
            else REASON_SOLVER_ERROR
        )
        detail = "；".join(s.detail for s in failed if s.detail)
    elif not accept_synthetic and not measured:
        status = "skipped"
        reason = REASON_CALIBRATION_NOT_MEASURED
        detail = (
            "标定数据未实测（标定文件 measured=false 或采样为合成数据）："
            "只做离线解算演练，不构成任何实测精度结论。"
        )
    else:
        status = "ok"
        reason = ""
        detail = (
            "合成数据演练已显式接受（--accept-synthetic）；结论仍**不构成实测精度**。"
            if (accept_synthetic and not measured)
            else "标定解算完成。"
        )

    unverified: Tuple[str, ...] = (
        _METRICS_NOT_COVERED_BY_CALIBRATION
        if measured
        else UNVERIFIED_CALIBRATION_METRICS
    )

    return CalibrationRun(
        status=status,
        reason=reason,
        measured=measured,
        samples_synthetic=samples_synthetic,
        accept_synthetic=bool(accept_synthetic),
        fk_mode=effective_fk_mode,
        samples_path=str(samples_file),
        calibration_path=str(calibration_file),
        calibration=doc,
        sections=sections,
        repeatability=repeatability,
        cross_validation=cross_validation,
        unverified_metrics=unverified,
        detail=detail,
    )
