"""夹爪开口量映射层（S01-T04）。

职责边界
--------
* **模型真值**（几何）放在 ``models/rm75_drake/rmg24_drake.urdf``（自研，原始几何体）；
* 本模块只做**映射与调度**：把官方 SDK 的无量纲开口指令 ``u``（0–1000）
  换算成物理开口（mm / m）与两指关节位移，并把夹爪动作表达成可复用的数据结构。

真值来源（不可改写）
--------------------
* 官方手册（``Related Files/Two-Finger Parallel Gripper User Manual V1.0.pdf``）：
  开口指令参数范围 **0–1000，无量纲，其中 1000 对应标准夹指 65 mm**。
* SDK（``robotic_arm`` 1.1.7）：``rm_set_gripper_position(position:int, ...)``、
  ``rm_set_gripper_pick(speed:int, force:int, ...)``、
  ``rm_set_gripper_release(speed, block, timeout)``、``rm_get_gripper_state()``。
* **u→N 力曲线官方未给出** → 落地为显式"未标定"（见
  ``src/rm75_drake/calibration/gripper_force.yaml``），本模块**禁止**返回伪造牛顿值。

约定
----
* 线性映射：``opening = u / 1000 * 65 mm``（默认线性，与手册一致）。
* **越界显式抛 :class:`ValueError`**，绝不静默截断（截断会把"开口过大"变成
  一个看起来成功的指令）。
* 双指对称：两指的**偏移量**（相对对称中心线）各为 ``opening / 2``，
  因此两指偏移量之**和 = 总开口**；单指最大行程 = 32.5 mm = 65 mm / 2。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from . import paths

__all__ = [
    "GRIPPER_POSITION_MIN",
    "GRIPPER_POSITION_MAX",
    "GRIPPER_OPENING_MAX_M",
    "GRIPPER_OPENING_MAX_MM",
    "GRIPPER_FINGER_TRAVEL_MAX_M",
    "GRIPPER_JOINT_NAMES",
    "GripperNotCalibratedError",
    "GripperAction",
    "opening_mm",
    "opening_m",
    "position_from_mm",
    "finger_positions",
    "force_from_position",
    "calibration_path",
    "load_calibration",
    "open_action",
    "close_action",
    "pick_action",
]


# ── 常量（真值来源：官方手册 + SDK）────────────────────────────────────
#: 开口指令下界（无量纲）。
GRIPPER_POSITION_MIN = 0

#: 开口指令上界（无量纲，官方手册：0–1000）。
GRIPPER_POSITION_MAX = 1000

#: ``u = GRIPPER_POSITION_MAX`` 对应的标准夹指总开口（m）。
GRIPPER_OPENING_MAX_M = 0.065

#: 同上，单位 mm（= 65 mm，官方手册原文）。
GRIPPER_OPENING_MAX_MM = 65.0

#: 单指最大行程（m）= 65 mm / 2。
GRIPPER_FINGER_TRAVEL_MAX_M = 0.0325

#: 夹爪两个开合关节名（顺序 = 左、右；与 rmg24_drake.urdf 一致）。
GRIPPER_JOINT_NAMES: Tuple[str, ...] = ("finger_joint_left", "finger_joint_right")

#: 标定文件路径（相对 ``rm75_drake.paths``，不依赖 cwd）。
_CALIBRATION_FILE = "gripper_force.yaml"


class GripperNotCalibratedError(RuntimeError):
    """夹爪力曲线未标定时抛出。

    异常信息**必须**包含"未标定"三个字（下游与测试据此断言）。
    """


# ── 校验 ──────────────────────────────────────────────────────────────
def _validate_u(u: Any) -> float:
    """校验开口指令 ``u`` 并返回其浮点值。

    越界（``u < 0`` 或 ``u > 1000``）抛 :class:`ValueError`——**不静默截断**。
    非数值（``bool`` 除外）同样抛 :class:`ValueError`。
    """
    if isinstance(u, bool) or not isinstance(u, (int, float)):
        raise ValueError(f"开口指令 u 必须是数值，收到：{u!r}")
    value = float(u)
    if not math.isfinite(value):
        raise ValueError(f"开口指令 u 必须是有限数值，收到：{u!r}")
    if value < GRIPPER_POSITION_MIN or value > GRIPPER_POSITION_MAX:
        raise ValueError(
            f"开口指令 u 越界：{value} 不在 "
            f"[{GRIPPER_POSITION_MIN}, {GRIPPER_POSITION_MAX}] 内"
        )
    return value


def _validate_mm(mm: Any) -> float:
    """校验物理开口（mm）并返回其浮点值（范围 0–65 mm）。"""
    if isinstance(mm, bool) or not isinstance(mm, (int, float)):
        raise ValueError(f"开口量 mm 必须是数值，收到：{mm!r}")
    value = float(mm)
    if not math.isfinite(value):
        raise ValueError(f"开口量 mm 必须是有限数值，收到：{mm!r}")
    if value < 0.0 or value > GRIPPER_OPENING_MAX_MM:
        raise ValueError(
            f"开口量越界：{value} mm 不在 [0, {GRIPPER_OPENING_MAX_MM}] mm 内"
        )
    return value


# ── 映射 ──────────────────────────────────────────────────────────────
def opening_mm(u: Any) -> float:
    """把开口指令 ``u``（0–1000）线性映射为总开口（mm）。

    端点：``u = 0`` → 0 mm；``u = 1000`` → 65 mm（官方手册）。
    越界抛 :class:`ValueError`（不静默截断）。
    """
    value = _validate_u(u)
    return value * GRIPPER_OPENING_MAX_MM / GRIPPER_POSITION_MAX


def opening_m(u: Any) -> float:
    """把开口指令 ``u`` 线性映射为总开口（m）。"""
    return opening_mm(u) / 1000.0


def position_from_mm(mm: Any) -> int:
    """把物理开口（mm）反向映射为开口指令 ``u``（0–1000 的整数）。

    越界（``mm < 0`` 或 ``mm > 65``）抛 :class:`ValueError`。
    四舍五入到整数（SDK 的 ``position`` 是 ``int``）。
    """
    value = _validate_mm(mm)
    raw = value * GRIPPER_POSITION_MAX / GRIPPER_OPENING_MAX_MM
    return int(round(raw))


def finger_positions(u: Any) -> Tuple[float, float]:
    """返回两指的**对称偏移量**（m）：``(左指, 右指)``。

    两指的偏移量**都是相对对称中心线的距离**（非负），因此：

    * 两指偏移量之**和 = 总开口**（``u = 1000`` 时 0.0325 + 0.0325 = 0.065 m）；
    * 单指最大行程 = :data:`GRIPPER_FINGER_TRAVEL_MAX_M`（= 32.5 mm）。

    方向（左指 +y / 右指 -y）由 URDF 里 prismatic 关节的 ``axis`` 符号承担，
    **不**在这里编码；因此本函数返回值可直接当作两个关节的 ``q``。
    """
    half = opening_m(u) / 2.0
    return (half, half)


# ── 力曲线（未标定 → 显式抛错）────────────────────────────────────────
def calibration_path() -> Path:
    """返回标定文件路径（``src/rm75_drake/calibration/gripper_force.yaml``）。"""
    return paths.calibration_dir / _CALIBRATION_FILE


def load_calibration() -> Dict[str, Any]:
    """读取标定文件并返回其内容。

    文件缺失或 YAML 解析失败都会**显式冒泡**（不返回"看起来正常"的默认值），
    避免把"没有标定数据"当成"标定结果为 0"。
    """
    import yaml

    path = calibration_path()
    if not path.is_file():
        raise FileNotFoundError(f"夹爪标定文件不存在：{path}")
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"夹爪标定文件格式错误（应为映射）：{path}")
    return data


def force_from_position(u: Any) -> float:
    """返回开口指令 ``u`` 对应的夹持力（N）。

    **未标定时必须抛 :class:`GripperNotCalibratedError`**（异常信息含"未标定"），
    绝不返回伪造的牛顿数值——官方未发布 u→N 曲线（R009 / 标定占位）。

    标定文件 ``calibrated: true`` 时，按 ``curve``（``[{u: .., force_n: ..}]``）
    做线性插值；曲线缺失或为空同样按"未标定"处理。
    """
    value = _validate_u(u)
    data = load_calibration()
    if not bool(data.get("calibrated", False)):
        raise GripperNotCalibratedError(
            f"夹爪夹持力**未标定**（calibration file: {calibration_path()}）："
            "官方未发布 u→N 力曲线，本项目禁止返回未经标定的力值。"
        )
    curve = data.get("curve") or []
    points: List[Tuple[float, float]] = []
    for item in curve:
        try:
            points.append((float(item["u"]), float(item["force_n"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"标定曲线条目格式错误：{item!r}") from exc
    if len(points) < 2:
        raise GripperNotCalibratedError(
            "夹爪夹持力**未标定**：标定文件标记为已标定，但曲线为空或只有一个点。"
        )
    points.sort()
    if value <= points[0][0]:
        return points[0][1]
    if value >= points[-1][0]:
        return points[-1][1]
    for (u0, f0), (u1, f1) in zip(points, points[1:]):
        if u0 <= value <= u1:
            ratio = (value - u0) / (u1 - u0)
            return f0 + ratio * (f1 - f0)
    raise GripperNotCalibratedError(  # pragma: no cover - 分段覆盖后不可达
        "夹爪夹持力**未标定**：标定曲线存在空档。"
    )


# ── 动作调度数据结构（供 S06 复用）────────────────────────────────────
@dataclass(frozen=True)
class GripperAction:
    """一次夹爪动作的调度描述。

    字段
    ----
    kind:
        ``"open"`` / ``"close"`` / ``"pick"``。
    position:
        绝对开口指令 ``u``（0–1000）。``"pick"`` 动作为 ``None``（力控夹取，
        目标位置由夹持对象决定）。
    speed:
        速度档位（SDK 语义：0–100 的百分比档位；具体档位由真机手册定标）。
    force:
        力档位（SDK ``rm_set_gripper_pick`` 的 ``force`` 参数，0–100 档位）。
        **注意**：档位≠牛顿；牛顿值需标定（见 :func:`force_from_position`）。
    hold_seconds:
        动作后的保持时间（s）——**占位时序**，真机实现必须用
        ``rm_get_gripper_state()`` 的反馈替换固定等待。
    """

    kind: str
    position: Optional[int] = None
    speed: int = 100
    force: Optional[int] = None
    hold_seconds: float = 0.0
    notes: str = ""

    def __post_init__(self) -> None:
        if self.kind not in ("open", "close", "pick"):
            raise ValueError(
                f"夹爪动作类型必须是 open/close/pick，收到：{self.kind!r}"
            )
        if self.position is not None:
            _validate_u(self.position)
        if not isinstance(self.speed, int) or isinstance(self.speed, bool):
            raise ValueError(f"speed 必须是整数，收到：{self.speed!r}")
        if not 0 <= self.speed <= 100:
            raise ValueError(f"speed 越界：{self.speed} 不在 [0, 100] 内")
        if self.force is not None:
            if not isinstance(self.force, int) or isinstance(self.force, bool):
                raise ValueError(f"force 必须是整数，收到：{self.force!r}")
            if not 0 <= self.force <= 100:
                raise ValueError(f"force 越界：{self.force} 不在 [0, 100] 内")
        if self.hold_seconds < 0.0:
            raise ValueError(f"hold_seconds 不能为负，收到：{self.hold_seconds!r}")


def open_action(speed: int = 100, hold_seconds: float = 0.5) -> GripperAction:
    """构造"完全张开"动作（``u = GRIPPER_POSITION_MAX``）。"""
    return GripperAction(
        kind="open",
        position=GRIPPER_POSITION_MAX,
        speed=speed,
        hold_seconds=hold_seconds,
        notes="完全张开（标准夹指 65 mm）",
    )


def close_action(speed: int = 100, hold_seconds: float = 0.5) -> GripperAction:
    """构造"完全闭合"动作（``u = 0``）。"""
    return GripperAction(
        kind="close",
        position=GRIPPER_POSITION_MIN,
        speed=speed,
        hold_seconds=hold_seconds,
        notes="完全闭合（开口 0 mm）",
    )


def pick_action(speed: int = 100, force: int = 50, hold_seconds: float = 1.0) -> GripperAction:
    """构造"力控夹取"动作（SDK ``rm_set_gripper_pick(speed, force)``）。

    ``force`` 是**档位**不是牛顿；牛顿值需要标定（未标定时
    :func:`force_from_position` 会显式抛"未标定"）。
    """
    return GripperAction(
        kind="pick",
        position=None,
        speed=speed,
        force=force,
        hold_seconds=hold_seconds,
        notes="力控夹取（力档位，力矩超限自动停止）",
    )
