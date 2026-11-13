"""双臂共同点一致性判定器（S04-T06）。

判据是什么
----------
里程碑 ROADMAP 的验收口径是「**两臂 TCP 在共同工作区同一点偏差 < 0.5 mm**」。
本模块把这条口径落成**可执行判据**：

* 位置判据：两臂 TCP frame 在 **world 系**下的位置差 ``position_error_mm``
  **严格小于** ``threshold_mm``（默认 :data:`DEFAULT_THRESHOLD_MM` = 0.5 mm，
  与 ROADMAP 一致，**写死为默认参数**并在结果里回报实际使用的阈值）。
* 姿态判据（可选）：两 TCP 的相对旋转角 ``orientation_error_deg``；只有在
  调用方显式给出 ``max_orientation_error_deg`` 时才参与判定。

为什么必须有「未验证」状态
--------------------------
这条指标依赖**真实标定数据**（基座外参 / 工具 TCP 6D）。本环境**无真机、
无实测数据**（``calibration/dual_arm.yaml`` 里 ``measured: false``），因此
**不得**用合成数据冒充「物理达标」。调用方必须显式传 ``measured``：

* ``measured=False`` → ``verdict == "unverified"``、``reason ==
  "calibration_not_measured"``、``unverified_metrics`` **非空**，
  **绝不返回 ``"pass"``**；
* ``measured=True`` → 才可能给出 ``"pass"`` / ``"fail"``。

这正是 MEM083 纪律：**指标不达标式声称**——未实测就报「未验证」，不报「达标」。

模块边界
--------
本模块**只做判定**：读 plant 里的两个 frame 位姿 → 算误差 → 给结论。
它**不**做 IK、不做标定、不做碰撞判定（那些分别在
:mod:`rm75_drake.planning.ik` / :mod:`rm75_drake.calibration` /
:mod:`rm75_drake.bimanual.collision_filter`）。

失败形状
--------
* frame 名不存在 → 抛 :class:`ConsistencyError`（``reason="unknown_frame"``），
  消息**点名该 frame**；
* plant 未 ``Finalize()`` → 抛 :class:`ConsistencyError`
  （``reason="plant_not_finalized"``），消息点名两个 frame 名；
* context 与 plant 不匹配 → ``reason="invalid_context"``；
* ``threshold_mm`` 非法（非正 / 非有限）→ ``reason="invalid_threshold"``。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

#: rad→deg 转换**全项目唯一实现**在 :mod:`rm75_drake.device.sdk`
#: （``assert_single_conversion_point`` 扫描整个包，发现第二份实现即报错）。
from ..device.sdk import rad_to_deg

__all__ = [
    "ConsistencyError",
    "ConsistencyResult",
    "VERDICT_PASS",
    "VERDICT_FAIL",
    "VERDICT_UNVERIFIED",
    "VERDICTS",
    "REASON_WITHIN_THRESHOLD",
    "REASON_EXCEEDS_THRESHOLD",
    "REASON_ORIENTATION_EXCEEDS_THRESHOLD",
    "REASON_CALIBRATION_NOT_MEASURED",
    "DEFAULT_THRESHOLD_MM",
    "evaluate_common_point",
]


#: 判定结论的三个稳定取值。
VERDICT_PASS = "pass"
VERDICT_FAIL = "fail"
VERDICT_UNVERIFIED = "unverified"

#: 全部结论取值（供调用方做集合校验）。
VERDICTS: Tuple[str, ...] = (VERDICT_PASS, VERDICT_FAIL, VERDICT_UNVERIFIED)

#: 位置偏差在阈值内。
REASON_WITHIN_THRESHOLD = "within_threshold"

#: 位置偏差超出阈值。
REASON_EXCEEDS_THRESHOLD = "exceeds_threshold"

#: 姿态偏差超出显式给定的阈值。
REASON_ORIENTATION_EXCEEDS_THRESHOLD = "orientation_exceeds_threshold"

#: 标定未实测 → 指标**未验证**（不得声称达标）。
REASON_CALIBRATION_NOT_MEASURED = "calibration_not_measured"

#: 位置阈值默认值（mm）——里程碑 ROADMAP 口径「偏差 < 0.5 mm」。
DEFAULT_THRESHOLD_MM = 0.5

#: 未实测时列为「未验证」的指标名（稳定标签，供报告/看板消费）。
_UNVERIFIED_METRICS: Tuple[str, ...] = ("position_error_mm", "orientation_error_deg")


class ConsistencyError(RuntimeError):
    """一致性判定的输入/环境非法。

    属性
    ----
    reason:
        稳定标签（``unknown_frame`` / ``plant_not_finalized`` /
        ``invalid_context`` / ``invalid_threshold``）。
    """

    def __init__(self, message: str, *, reason: str = "invalid_input") -> None:
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class ConsistencyResult:
    """一次共同点一致性判定的结果。

    字段
    ----
    verdict:
        ``"pass"`` / ``"fail"`` / ``"unverified"``（稳定取值，见 :data:`VERDICTS`）。
    position_error_mm:
        两臂 TCP 在 world 系下的位置偏差（mm）。
    orientation_error_deg:
        两臂 TCP 的相对旋转角（deg）。
    threshold_mm:
        **本次实际使用**的位置阈值（mm）。
    exceed_mm:
        位置偏差**超出阈值**的量（mm）；未超阈为 0.0。
    exceed_deg:
        姿态偏差超出阈值的量（deg）；未给姿态阈值或未超阈为 0.0。
    reason:
        稳定标签（见模块常量）。
    unverified_metrics:
        未验证项清单（稳定标签）。实测时为空元组。
    measured:
        调用方声明「数据来自实测」与否。
    """

    verdict: str
    position_error_mm: float
    orientation_error_deg: float
    threshold_mm: float
    exceed_mm: float
    reason: str
    unverified_metrics: Tuple[str, ...] = ()
    exceed_deg: float = 0.0
    measured: bool = True

    @property
    def is_pass(self) -> bool:
        """是否「在阈值内且已实测」。"""
        return self.verdict == VERDICT_PASS

    @property
    def is_fail(self) -> bool:
        """是否「超阈值」。"""
        return self.verdict == VERDICT_FAIL

    @property
    def is_unverified(self) -> bool:
        """是否「未验证」（标定未实测）。"""
        return self.verdict == VERDICT_UNVERIFIED

    def as_dict(self) -> Dict[str, Any]:
        """可序列化视图（供 CLI / 报告消费）。"""
        return {
            "verdict": self.verdict,
            "position_error_mm": self.position_error_mm,
            "orientation_error_deg": self.orientation_error_deg,
            "threshold_mm": self.threshold_mm,
            "exceed_mm": self.exceed_mm,
            "exceed_deg": self.exceed_deg,
            "reason": self.reason,
            "unverified_metrics": list(self.unverified_metrics),
            "measured": self.measured,
        }

    def summary(self) -> str:
        """人类可读的一行摘要（诊断用）。"""
        head = {
            VERDICT_PASS: "PASS",
            VERDICT_FAIL: "FAIL",
            VERDICT_UNVERIFIED: "UNVERIFIED",
        }.get(self.verdict, self.verdict)
        text = (
            f"[{head}] 位置偏差 {self.position_error_mm:.4f} mm "
            f"(阈值 {self.threshold_mm:.3f} mm, 超出 {self.exceed_mm:.4f} mm), "
            f"姿态偏差 {self.orientation_error_deg:.4f} deg, reason={self.reason}"
        )
        if self.unverified_metrics:
            text += "；未验证项：" + "、".join(self.unverified_metrics)
        return text


def _rotation_angle_deg(rotation) -> float:
    """返回旋转矩阵对应的旋转角（deg，范围 [0, 180]）。

    弧度→度走 :func:`rm75_drake.device.sdk.rad_to_deg`（唯一转换点）。
    """
    matrix = rotation.matrix() if hasattr(rotation, "matrix") else rotation
    trace = float(matrix[0][0]) + float(matrix[1][1]) + float(matrix[2][2])
    cos_angle = (trace - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return rad_to_deg(math.acos(cos_angle))


def evaluate_common_point(
    plant: Any,
    context: Any,
    *,
    left_tcp_frame: str,
    right_tcp_frame: str,
    measured: bool,
    threshold_mm: float = DEFAULT_THRESHOLD_MM,
    max_orientation_error_deg: Optional[float] = None,
) -> ConsistencyResult:
    """判定两臂 TCP 是否落在共同工作区的同一点（阈值内）。

    参数
    ----
    plant:
        已 ``Finalize()`` 的 :class:`MultibodyPlant`（双臂在**同一个** plant 里）。
    context:
        该 plant 的上下文（决定本次判定使用的位形）。
    left_tcp_frame:
        左臂 TCP frame 名（plant 里必须存在）。
    right_tcp_frame:
        右臂 TCP frame 名（plant 里必须存在）。
    measured:
        **数据是否来自实测**。``False`` → 结论必为 ``"unverified"``，
        ``unverified_metrics`` 非空，**绝不返回 ``"pass"``**。
    threshold_mm:
        位置阈值（mm）。默认 :data:`DEFAULT_THRESHOLD_MM`（0.5 mm，ROADMAP 口径）。
    max_orientation_error_deg:
        姿态阈值（deg）。``None`` = **不参与判定**（只回报数值）。

    返回
    ----
    :class:`ConsistencyResult`

    异常
    ----
    ConsistencyError
        frame 不存在 / plant 未 finalize / context 不匹配 / 阈值非法。
    """
    if not isinstance(threshold_mm, (int, float)) or isinstance(threshold_mm, bool):
        raise ConsistencyError(
            f"threshold_mm 必须是数值，收到 {threshold_mm!r}",
            reason="invalid_threshold",
        )
    threshold = float(threshold_mm)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ConsistencyError(
            f"threshold_mm 必须是正有限数（mm），收到 {threshold_mm!r}",
            reason="invalid_threshold",
        )
    if max_orientation_error_deg is not None:
        if not isinstance(max_orientation_error_deg, (int, float)) or isinstance(
            max_orientation_error_deg, bool
        ):
            raise ConsistencyError(
                f"max_orientation_error_deg 必须是数值或 None，收到 {max_orientation_error_deg!r}",
                reason="invalid_threshold",
            )
        max_orientation = float(max_orientation_error_deg)
        if not math.isfinite(max_orientation) or max_orientation < 0.0:
            raise ConsistencyError(
                "max_orientation_error_deg 必须是非负有限数（deg），"
                f"收到 {max_orientation_error_deg!r}",
                reason="invalid_threshold",
            )
    else:
        max_orientation = None

    if plant is None:
        raise ConsistencyError("plant 不能为 None", reason="plant_not_finalized")
    if not plant.is_finalized():
        raise ConsistencyError(
            f"plant 尚未 Finalize，无法读取 frame 位姿（frame: {left_tcp_frame!r} / "
            f"{right_tcp_frame!r}）",
            reason="plant_not_finalized",
        )

    for name in (left_tcp_frame, right_tcp_frame):
        if not isinstance(name, str) or not name:
            raise ConsistencyError(
                f"frame 名必须是非空字符串，收到 {name!r}",
                reason="unknown_frame",
            )
        if not plant.HasFrameNamed(name):
            raise ConsistencyError(
                f"plant 里没有名为 {name!r} 的 frame（可用 frame 列表见 "
                "plant.GetFrameNames()）；无法判定共同点一致性",
                reason="unknown_frame",
            )

    try:
        plant.GetPositions(context)
    except Exception as exc:  # context 与 plant 不匹配 / 已失效
        raise ConsistencyError(
            f"context 与 plant 不匹配或已失效：{exc}",
            reason="invalid_context",
        ) from exc

    world = plant.world_frame()
    left_pose = plant.CalcRelativeTransform(
        context, world, plant.GetFrameByName(left_tcp_frame)
    )
    right_pose = plant.CalcRelativeTransform(
        context, world, plant.GetFrameByName(right_tcp_frame)
    )

    left_position = left_pose.translation()
    right_position = right_pose.translation()
    position_error_m = float(
        (
            (left_position[0] - right_position[0]) ** 2
            + (left_position[1] - right_position[1]) ** 2
            + (left_position[2] - right_position[2]) ** 2
        )
        ** 0.5
    )
    position_error_mm = position_error_m * 1000.0

    relative_rotation = left_pose.rotation().matrix() @ right_pose.rotation().matrix().T
    orientation_error_deg = _rotation_angle_deg(relative_rotation)

    exceed_mm = max(0.0, position_error_mm - threshold)
    exceed_deg = (
        max(0.0, orientation_error_deg - max_orientation)
        if max_orientation is not None
        else 0.0
    )

    if not measured:
        # 未实测 → 只能报「未验证」，**不得**声称达标。
        return ConsistencyResult(
            verdict=VERDICT_UNVERIFIED,
            position_error_mm=position_error_mm,
            orientation_error_deg=orientation_error_deg,
            threshold_mm=threshold,
            exceed_mm=exceed_mm,
            exceed_deg=exceed_deg,
            reason=REASON_CALIBRATION_NOT_MEASURED,
            unverified_metrics=_UNVERIFIED_METRICS,
            measured=False,
        )

    if position_error_mm >= threshold:
        return ConsistencyResult(
            verdict=VERDICT_FAIL,
            position_error_mm=position_error_mm,
            orientation_error_deg=orientation_error_deg,
            threshold_mm=threshold,
            exceed_mm=exceed_mm,
            exceed_deg=exceed_deg,
            reason=REASON_EXCEEDS_THRESHOLD,
            measured=True,
        )

    if max_orientation is not None and orientation_error_deg > max_orientation:
        return ConsistencyResult(
            verdict=VERDICT_FAIL,
            position_error_mm=position_error_mm,
            orientation_error_deg=orientation_error_deg,
            threshold_mm=threshold,
            exceed_mm=exceed_mm,
            exceed_deg=exceed_deg,
            reason=REASON_ORIENTATION_EXCEEDS_THRESHOLD,
            measured=True,
        )

    return ConsistencyResult(
        verdict=VERDICT_PASS,
        position_error_mm=position_error_mm,
        orientation_error_deg=orientation_error_deg,
        threshold_mm=threshold,
        exceed_mm=0.0,
        exceed_deg=0.0,
        reason=REASON_WITHIN_THRESHOLD,
        measured=True,
    )
