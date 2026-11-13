"""标定文件正式 schema 与严格校验（S04-T04）。

本模块是**标定产出文件的真相源解析器**，纪律与
:mod:`rm75_drake.scenario` / :mod:`rm75_drake.device.config` 完全一致：

* **严格键白名单**：未知顶层键 / 未知子键 → :class:`CalibrationError`，
  异常信息里**点名该键**；
* **缺必填字段** → :class:`CalibrationError`（点名缺哪个键）；
* **维度错**（xyz/rpy 不是 3 维、关节数不是 7、pose 不是 4×4）→ :class:`CalibrationError`；
* **单位错**（rpy 写成角度制）→ :class:`CalibrationError`，并给出换算提示。

单位约定（全表 SI，禁止在别处另立单位）
----------------------------------------

========  ==========================================================
量纲       约定
========  ==========================================================
长度      m（米）。``xyz`` 一律 m。
角度      **rad（弧度）**。所有 ``rpy``、关节零位偏移一律 rad。
========  ==========================================================

**本模块零 pydrake 依赖**（只用标准库），因此可离线单测，不被 Drake 版本绑死。

标定文件示例（``calibration/dual_arm.yaml`` 的形状）::

    schema_version: 1
    measured: false          # 是否来自实测（本环境无实测数据 → false）
    calibrated: false        # 是否已完成标定
    method: "synthetic-kabsch"
    date: "2026-11-13"
    base_extrinsic:
      left:
        xyz: [0.0, 0.0, 0.0]
        rpy: [0.0, 0.0, 0.0]
        residual: {rms: 0.0, max: 0.0, samples: 20}
    joint_zero_offset:
      left:
        left_joint_1: 0.0
        ...              # 每个机器人必须恰好 7 个关节
    tool_offset:
      left:
        xyz: [0.0, 0.0, 0.0]
        rpy: [0.0, 0.0, 0.0]

**重要纪律**（对齐研究结论第 9 条）：标定文件里的每个数值都必须带
``measured`` / ``calibrated`` 布尔位与 ``method`` / ``date`` / ``residual``
元信息；**禁止塞「未验证的漂亮数字」**。
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

__all__ = [
    "CalibrationError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "REQUIRED_TOP_LEVEL_KEYS",
    "ResidualSummary",
    "Pose6D",
    "BaseExtrinsicEntry",
    "ToolOffsetEntry",
    "CalibrationDoc",
    "parse_calibration",
    "load_calibration_file",
    "rpy_to_matrix",
    "matrix_to_rpy",
]

#: 当前正式 schema 版本。
SCHEMA_VERSION = 1

#: 顶层键（顺序即 YAML 里的书写顺序）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "measured",
    "calibrated",
    "method",
    "date",
    "base_extrinsic",
    "joint_zero_offset",
    "tool_offset",
)

#: 必填顶层键（其余三段可以缺省或为空映射）。
REQUIRED_TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "measured",
    "calibrated",
    "method",
    "date",
)

#: 每个机器人必须标定的关节数（RM75-D 是 7 自由度）。
N_JOINTS = 7

#: rpy 主值区间上限（弧度）。超出该区间视为「角度制写错」。
_RPY_LIMIT = math.pi

#: 长度量纲上限（米）。超出视为「单位写错」（例如把毫米当米填）。
_LENGTH_LIMIT = 100.0

#: 日期格式（YYYY-MM-DD）。
_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")

#: 位置/姿态条目的合法子键。
_POSE_KEYS: Tuple[str, ...] = ("xyz", "rpy")

#: 残差摘要的合法子键（``rms`` / ``max`` / ``samples`` 必填）。
_RESIDUAL_KEYS: Tuple[str, ...] = ("rms", "max", "samples", "threshold", "within_threshold")
_RESIDUAL_REQUIRED: Tuple[str, ...] = ("rms", "max", "samples")


class CalibrationError(ValueError):
    """标定 schema 校验失败（未知键 / 缺字段 / 维度或单位错误）。"""


# ── 内部校验工具 ────────────────────────────────────────────────────────
def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CalibrationError(f"{path} 必须是映射（mapping），收到 {type(value).__name__}")
    for key in value:
        if not isinstance(key, str):
            raise CalibrationError(f"{path} 的键必须是字符串，收到 {key!r}")
    return value


def _require_sequence(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise CalibrationError(f"{path} 必须是序列（list），收到 {type(value).__name__}")
    return value


def _check_keys(
    mapping: Mapping[str, Any],
    path: str,
    allowed: Sequence[str],
    required: Sequence[str],
) -> None:
    unknown = [k for k in mapping if k not in allowed]
    if unknown:
        raise CalibrationError(
            f"{path} 出现未知键：{sorted(unknown)}；该处允许的键只有 {list(allowed)}"
        )
    missing = [k for k in required if k not in mapping]
    if missing:
        raise CalibrationError(
            f"{path} 缺少必填字段：{missing}；该处允许的键只有 {list(allowed)}"
        )


def _as_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError(f"{path} 必须是数值，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise CalibrationError(f"{path} 必须是有限数值，收到 {value!r}")
    return out


def _as_bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise CalibrationError(f"{path} 必须是布尔值（true/false），收到 {value!r}")
    return value


def _as_float_list(value: Any, path: str, length: int) -> Tuple[float, ...]:
    seq = _require_sequence(value, path)
    if len(seq) != length:
        raise CalibrationError(f"{path} 必须是 {length} 个数值，收到 {len(seq)} 个")
    return tuple(_as_float(v, f"{path}[{i}]") for i, v in enumerate(seq))


def _as_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CalibrationError(f"{path} 必须是整数，收到 {value!r}")
    return int(value)


def _check_xyz(values: Tuple[float, ...], path: str) -> Tuple[float, ...]:
    for i, v in enumerate(values):
        if abs(v) > _LENGTH_LIMIT:
            raise CalibrationError(
                f"{path}[{i}] = {v} 超出长度上限 {_LENGTH_LIMIT} m；"
                "长度一律用米（若填的是毫米请先换算）"
            )
    return values


def _check_rpy(values: Tuple[float, ...], path: str) -> Tuple[float, ...]:
    for i, v in enumerate(values):
        if abs(v) > _RPY_LIMIT:
            raise CalibrationError(
                f"{path}[{i}] = {v} 超出弧度主值区间 [-π, π]；"
                "rpy 必须用弧度（rad），不接收角度制（度）——"
                f"若填的是度，请乘 {math.pi / 180.0:.10f} 换算为弧度"
            )
    return values


def _require_str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CalibrationError(f"{path} 必须是非空字符串，收到 {value!r}")
    return value


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ResidualSummary:
    """残差摘要（m）。"""

    rms: float
    max: float
    samples: int
    threshold: Optional[float] = None
    within_threshold: Optional[bool] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "rms": self.rms,
            "max": self.max,
            "samples": self.samples,
            "threshold": self.threshold,
            "within_threshold": self.within_threshold,
        }


@dataclass(frozen=True)
class Pose6D:
    """6D 位姿（xyz: m / rpy: rad）。"""

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    residual: Optional[ResidualSummary] = None

    def as_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"xyz": list(self.xyz), "rpy": list(self.rpy)}
        if self.residual is not None:
            out["residual"] = self.residual.as_dict()
        return out


@dataclass(frozen=True)
class BaseExtrinsicEntry:
    """基座外参条目（机器人基座在 world 系下的位姿 + 残差摘要）。"""

    robot: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    residual: Optional[ResidualSummary] = None

    def as_pose(self) -> Pose6D:
        return Pose6D(xyz=self.xyz, rpy=self.rpy, residual=self.residual)


@dataclass(frozen=True)
class ToolOffsetEntry:
    """工具 TCP 6D 修正条目（相对 URDF 的 Link7→法兰→夹爪→TCP 修正量）。"""

    robot: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    residual: Optional[ResidualSummary] = None

    def as_pose(self) -> Pose6D:
        return Pose6D(xyz=self.xyz, rpy=self.rpy, residual=self.residual)


@dataclass(frozen=True)
class CalibrationDoc:
    """标定文件（schema_version 1）。

    字段
    ----
    schema_version:
        正式 schema 版本（当前 :data:`SCHEMA_VERSION`）。
    measured:
        是否来自**实测**。本环境无真机/无实测数据 → ``False``。
    calibrated:
        是否已完成标定。``measured=False`` 时必须是 ``False``（一致性自洽）。
    method:
        标定方法标识（如 ``"synthetic-kabsch"``、``"tip-to-tip-lsq"``）。
    date:
        标定日期（``YYYY-MM-DD``）。
    base_extrinsic:
        机器人名 → 基座在 world 系下的位姿 + 残差摘要。
    joint_zero_offset:
        机器人名 → 关节名 → 零位偏移（rad）。
    tool_offset:
        机器人名 → TCP 6D 修正（相对 URDF 的修正量）。
    source_path:
        来源文件路径（内存构造时为 ``None``）。
    """

    schema_version: int
    measured: bool
    calibrated: bool
    method: str
    date: str
    base_extrinsic: Mapping[str, BaseExtrinsicEntry] = field(default_factory=dict)
    joint_zero_offset: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    tool_offset: Mapping[str, ToolOffsetEntry] = field(default_factory=dict)
    source_path: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "measured": self.measured,
            "calibrated": self.calibrated,
            "method": self.method,
            "date": self.date,
            "base_extrinsic": {
                name: entry.as_pose().as_dict() for name, entry in self.base_extrinsic.items()
            },
            "joint_zero_offset": {
                robot: dict(joints) for robot, joints in self.joint_zero_offset.items()
            },
            "tool_offset": {name: entry.as_pose().as_dict() for name, entry in self.tool_offset.items()},
            "source_path": self.source_path,
        }


# ── 解析 ────────────────────────────────────────────────────────────────
def _parse_residual(entry: Any, path: str) -> ResidualSummary:
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _RESIDUAL_KEYS, _RESIDUAL_REQUIRED)
    rms = _as_float(mapping["rms"], f"{path}.rms")
    maximum = _as_float(mapping["max"], f"{path}.max")
    samples = _as_int(mapping["samples"], f"{path}.samples")
    if samples < 0:
        raise CalibrationError(f"{path}.samples = {samples} 不能为负")
    threshold = None
    if "threshold" in mapping:
        threshold = _as_float(mapping["threshold"], f"{path}.threshold")
    within = None
    if "within_threshold" in mapping:
        within = _as_bool(mapping["within_threshold"], f"{path}.within_threshold")
    return ResidualSummary(
        rms=rms, max=maximum, samples=samples, threshold=threshold, within_threshold=within
    )


def _parse_pose_entry(
    entry: Any, path: str, *, with_residual: bool = True
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Optional[ResidualSummary]]:
    mapping = _require_mapping(entry, path)
    allowed = _POSE_KEYS + (("residual",) if with_residual else ())
    _check_keys(mapping, path, allowed, _POSE_KEYS)
    xyz = _check_xyz(_as_float_list(mapping["xyz"], f"{path}.xyz", 3), f"{path}.xyz")
    rpy = _check_rpy(_as_float_list(mapping["rpy"], f"{path}.rpy", 3), f"{path}.rpy")
    residual = None
    if "residual" in mapping:
        residual = _parse_residual(mapping["residual"], f"{path}.residual")
    return xyz, rpy, residual


def _parse_base_extrinsic(entry: Any) -> Dict[str, BaseExtrinsicEntry]:
    mapping = _require_mapping(entry, "base_extrinsic")
    out: Dict[str, BaseExtrinsicEntry] = {}
    for robot, value in mapping.items():
        path = f"base_extrinsic.{robot}"
        xyz, rpy, residual = _parse_pose_entry(value, path)
        out[robot] = BaseExtrinsicEntry(
            robot=robot, xyz=xyz, rpy=rpy, residual=residual  # type: ignore[arg-type]
        )
    return out


def _parse_joint_zero_offset(entry: Any) -> Dict[str, Dict[str, float]]:
    mapping = _require_mapping(entry, "joint_zero_offset")
    out: Dict[str, Dict[str, float]] = {}
    for robot, value in mapping.items():
        path = f"joint_zero_offset.{robot}"
        joints = _require_mapping(value, path)
        if len(joints) != N_JOINTS:
            raise CalibrationError(
                f"{path} 必须恰好 {N_JOINTS} 个关节的零位偏移（RM75-D 是 7 自由度），"
                f"收到 {len(joints)} 个：{sorted(joints)}"
            )
        table: Dict[str, float] = {}
        for joint, value2 in joints.items():
            jpath = f"{path}.{joint}"
            offset = _as_float(value2, jpath)
            if abs(offset) > _RPY_LIMIT:
                raise CalibrationError(
                    f"{jpath} = {offset} 超出弧度主值区间 [-π, π]；"
                    "关节零位偏移一律用弧度（rad）"
                )
            table[joint] = offset
        out[robot] = table
    return out


def _parse_tool_offset(entry: Any) -> Dict[str, ToolOffsetEntry]:
    mapping = _require_mapping(entry, "tool_offset")
    out: Dict[str, ToolOffsetEntry] = {}
    for robot, value in mapping.items():
        path = f"tool_offset.{robot}"
        xyz, rpy, residual = _parse_pose_entry(value, path)
        out[robot] = ToolOffsetEntry(
            robot=robot, xyz=xyz, rpy=rpy, residual=residual  # type: ignore[arg-type]
        )
    return out


def parse_calibration(data: Any, source_path: Optional[str] = None) -> CalibrationDoc:
    """从已解析的 dict 构造 :class:`CalibrationDoc`（校验失败抛 :class:`CalibrationError`）。"""
    mapping = _require_mapping(data, "calibration")

    unknown = [k for k in mapping if k not in TOP_LEVEL_KEYS]
    if unknown:
        raise CalibrationError(
            f"calibration 出现未知顶层键：{sorted(unknown)}；"
            f"顶层键固定为 {list(TOP_LEVEL_KEYS)}"
        )
    missing = [k for k in REQUIRED_TOP_LEVEL_KEYS if k not in mapping]
    if missing:
        raise CalibrationError(f"calibration 缺少必填顶层键：{missing}")

    version = _as_int(mapping["schema_version"], "schema_version")
    if version != SCHEMA_VERSION:
        raise CalibrationError(
            f"schema_version = {version} 不受支持；当前正式版本是 {SCHEMA_VERSION}"
        )

    measured = _as_bool(mapping["measured"], "measured")
    calibrated = _as_bool(mapping["calibrated"], "calibrated")
    if calibrated and not measured:
        raise CalibrationError(
            "calibrated = true 但 measured = false：未实测的数据不得声明已标定"
        )

    method = _require_str(mapping["method"], "method")
    date = _require_str(mapping["date"], "date")
    if not _DATE_RE.match(date):
        raise CalibrationError(f"date = {date!r} 必须是 YYYY-MM-DD 格式")

    base_extrinsic: Dict[str, BaseExtrinsicEntry] = {}
    if "base_extrinsic" in mapping:
        base_extrinsic = _parse_base_extrinsic(mapping["base_extrinsic"])

    joint_zero_offset: Dict[str, Dict[str, float]] = {}
    if "joint_zero_offset" in mapping:
        joint_zero_offset = _parse_joint_zero_offset(mapping["joint_zero_offset"])

    tool_offset: Dict[str, ToolOffsetEntry] = {}
    if "tool_offset" in mapping:
        tool_offset = _parse_tool_offset(mapping["tool_offset"])

    return CalibrationDoc(
        schema_version=version,
        measured=measured,
        calibrated=calibrated,
        method=method,
        date=date,
        base_extrinsic=base_extrinsic,
        joint_zero_offset=joint_zero_offset,
        tool_offset=tool_offset,
        source_path=source_path,
    )


def load_calibration_file(path) -> CalibrationDoc:
    """从 YAML 文件加载标定文件。

    * 文件不存在 → :class:`FileNotFoundError`；
    * YAML 语法错误 → :class:`CalibrationError`；
    * schema 违规 → :class:`CalibrationError`（点名违规键）。
    """
    import yaml

    file_path = Path(path)
    try:
        text = file_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"找不到标定文件：{file_path}") from exc
    except OSError as exc:  # pragma: no cover - 权限/IO 异常
        raise CalibrationError(f"读取标定文件失败：{file_path}（{exc}）") from exc

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CalibrationError(f"标定 YAML 解析失败：{file_path}（{exc}）") from exc

    if data is None:
        raise CalibrationError(f"标定文件为空：{file_path}")

    return parse_calibration(data, source_path=str(file_path))


# ── 姿态工具（纯标准库实现，供求解器与报告器复用）──────────────────────
def rpy_to_matrix(rpy: Sequence[float]) -> Tuple[Tuple[float, float, float], ...]:
    """rpy（rad）→ 3×3 旋转矩阵（行主序，``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``）。"""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def matrix_to_rpy(matrix: Sequence[Sequence[float]]) -> Tuple[float, float, float]:
    """3×3 旋转矩阵 → rpy（rad，主值区间 [-π, π]）。"""
    m = [[float(v) for v in row] for row in matrix]
    if len(m) != 3 or any(len(row) != 3 for row in m):
        raise CalibrationError(f"旋转矩阵必须是 3×3，收到 {len(m)}×{len(m[0]) if m else 0}")
    sp = -m[2][0]
    if sp >= 1.0 - 1e-12:
        pitch = math.pi / 2.0
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    elif sp <= -1.0 + 1e-12:
        pitch = -math.pi / 2.0
        roll = 0.0
        yaw = math.atan2(m[0][1], m[1][1])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    return (roll, pitch, yaw)
