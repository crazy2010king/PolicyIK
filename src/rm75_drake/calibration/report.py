"""标定报告器（S04-T04）：残差 / 重复性 / 第三方交叉验证差异 + 「未实测」声明。

纪律（对齐研究结论第 2、9 条）
-------------------------------

* **合成数据只能自证算法，不能自证精度**。因此当标定文件的
  ``measured = false`` 时，报告**必须显式声明「未实测」**并**逐条列出未验证项**，
  不得让任何人把合成数据的残差当成实测精度；
* 报告内容一律来自 :class:`rm75_drake.calibration.solvers.ResidualReport`
  （逐点残差 / RMS / 最大残差 / 样本数 / 阈值 / ``within_threshold``），
  报告器**不自己重算**残差，避免两套口径。

**本模块零 pydrake 依赖**。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .schema import CalibrationDoc
from .solvers import ResidualReport

__all__ = [
    "UNVERIFIED_ITEMS",
    "CalibrationReport",
    "build_report",
    "repeatability_summary",
    "cross_validation_summary",
]

#: ``measured = false`` 时必须逐条列出的**未验证项**。
UNVERIFIED_ITEMS: Tuple[str, ...] = (
    "真实标定精度：关节零位偏差（逐关节 +10° 实测对比，目标 < 0.1°）",
    "真实标定精度：工具 TCP 6D（四点/六点法，目标 < 0.5 mm）",
    "真实标定精度：双臂基座外参 X_LR（尖对尖最小二乘，目标 < 0.5 mm / 0.2°）",
    "重复性：同一工装多次标定结果的离散度",
    "第三方交叉验证：相机 / 激光跟踪仪等独立手段的差异",
    "双臂共同工作区 TCP 同一点偏差 < 0.5 mm（依赖真实标定数据）",
)


@dataclass(frozen=True)
class CalibrationReport:
    """标定报告（残差 + 重复性 + 交叉验证 + 未实测声明）。"""

    measured: bool
    calibrated: bool
    method: str
    date: str
    sections: Mapping[str, ResidualReport] = field(default_factory=dict)
    repeatability: Optional[Mapping[str, Any]] = None
    cross_validation: Optional[Mapping[str, Any]] = None
    unverified: Tuple[str, ...] = ()

    @property
    def is_measured(self) -> bool:
        return self.measured

    @property
    def all_within_threshold(self) -> bool:
        return all(report.within_threshold for report in self.sections.values())

    def as_dict(self) -> Dict[str, Any]:
        return {
            "measured": self.measured,
            "calibrated": self.calibrated,
            "method": self.method,
            "date": self.date,
            "sections": {name: report.as_dict() for name, report in self.sections.items()},
            "repeatability": self.repeatability,
            "cross_validation": self.cross_validation,
            "unverified_items": list(self.unverified),
        }

    def render(self) -> str:
        """渲染成人类可读的中文报告。"""
        lines: list[str] = []
        lines.append("=" * 68)
        lines.append("标定报告（rm75_drake.calibration）")
        lines.append("=" * 68)
        lines.append(f"方法 method      : {self.method}")
        lines.append(f"日期 date        : {self.date}")
        lines.append(f"实测 measured    : {self.measured}")
        lines.append(f"已标定 calibrated: {self.calibrated}")

        if self.sections:
            lines.append("-" * 68)
            lines.append("残差（统一单位 m）")
            for name, report in self.sections.items():
                lines.append(
                    f"  {name}: RMS={report.rms:.6e} 最大={report.max_residual:.6e} "
                    f"样本数={report.sample_count} 阈值={report.threshold:.6e} "
                    f"within_threshold={report.within_threshold}"
                )
        else:
            lines.append("-" * 68)
            lines.append("残差：本报告未附带任何残差段（无求解结果）")

        if self.repeatability:
            lines.append("-" * 68)
            lines.append("重复性")
            for key, value in self.repeatability.items():
                lines.append(f"  {key}: {value}")

        if self.cross_validation:
            lines.append("-" * 68)
            lines.append("第三方交叉验证差异")
            for key, value in self.cross_validation.items():
                lines.append(f"  {key}: {value}")

        if not self.measured:
            lines.append("-" * 68)
            lines.append("⚠ 未实测声明：本报告基于合成/离线数据，**不构成实测精度结论**")
            lines.append("未验证项：")
            for item in self.unverified:
                lines.append(f"  - {item}")
        elif self.unverified:
            lines.append("-" * 68)
            lines.append("未验证项：")
            for item in self.unverified:
                lines.append(f"  - {item}")

        return "\n".join(lines)


def repeatability_summary(values: Sequence[float]) -> Dict[str, Any]:
    """多次标定结果的离散度摘要（均值 / 标准差 / 极差 / 样本数）。"""
    data = [float(v) for v in values]
    if not data:
        return {"count": 0}
    mean = sum(data) / len(data)
    if len(data) > 1:
        var = sum((v - mean) ** 2 for v in data) / (len(data) - 1)
        std = math.sqrt(var)
    else:
        std = 0.0
    return {
        "count": len(data),
        "mean": mean,
        "std": std,
        "max": max(data),
        "min": min(data),
        "range": max(data) - min(data),
    }


def cross_validation_summary(
    primary: ResidualReport, secondary: ResidualReport, *, label: str = "交叉验证"
) -> Dict[str, Any]:
    """两条独立求解结果的差异摘要（第三方交叉验证用）。

    ``primary`` 与 ``secondary`` 的残差口径必须一致（都是 m）。
    """
    if primary.unit != secondary.unit:
        raise ValueError(
            f"交叉验证的两份残差单位不一致：{primary.unit} vs {secondary.unit}"
        )
    diffs = [abs(a - b) for a, b in zip(primary.residuals, secondary.residuals)]
    if len(diffs) != len(primary.residuals) or len(diffs) != len(secondary.residuals):
        raise ValueError("交叉验证的两份残差样本数不一致")
    rms_diff = math.sqrt(sum(d * d for d in diffs) / len(diffs)) if diffs else 0.0
    return {
        "label": label,
        "primary": primary.name,
        "secondary": secondary.name,
        "rms_difference": rms_diff,
        "max_difference": max(diffs) if diffs else 0.0,
        "primary_rms": primary.rms,
        "secondary_rms": secondary.rms,
        "unit": primary.unit,
    }


def build_report(
    doc: CalibrationDoc,
    *,
    sections: Optional[Mapping[str, ResidualReport]] = None,
    repeatability: Optional[Mapping[str, Any]] = None,
    cross_validation: Optional[Mapping[str, Any]] = None,
    unverified: Optional[Sequence[str]] = None,
) -> CalibrationReport:
    """构造 :class:`CalibrationReport`。

    ``measured = false`` 时**自动**附带 :data:`UNVERIFIED_ITEMS` 全部未验证项
    （除非调用方显式传入 ``unverified`` 覆盖）。
    """
    if unverified is None:
        items: Tuple[str, ...] = () if doc.measured else UNVERIFIED_ITEMS
    else:
        items = tuple(unverified)
    return CalibrationReport(
        measured=doc.measured,
        calibrated=doc.calibrated,
        method=doc.method,
        date=doc.date,
        sections=dict(sections or {}),
        repeatability=dict(repeatability) if repeatability else None,
        cross_validation=dict(cross_validation) if cross_validation else None,
        unverified=items,
    )
