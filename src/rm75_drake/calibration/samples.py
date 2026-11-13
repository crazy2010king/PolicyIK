"""标定采样点集 schema（S04-T04）。

**实测数据将来放这里**：本模块只定义「采样点长什么样」与「怎么严格校验」，
不含任何求解逻辑（求解在 :mod:`rm75_drake.calibration.solvers`）。

三类采样
--------

``joint_zero``
    **逐关节 +10° 的角度与 FK 对比记录**。每个关节一条：给出一条 nominal
    （关节在基准角）与一条 ``+10°`` 的 commanded 关节角，以及两次的**观测 FK 位姿**
    （机器人自身 FK 或外部测量给出）。求解器据此反解该关节的零位偏差。
``tool_tcp``
    **局部点 ↔ 世界点** 对（四点法 / 六点法），≥ 4 组。
``base_extrinsic``
    **左臂 ↔ 右臂「尖对尖」点对**，≥ 15 组（左点给在左臂基座系、右点给在右臂基座系）。

纪律
----
* 每条采样**必须**带 ``synthetic``（true/false）与 ``source``（来源说明）——
  合成数据只自证算法，**不得冒充实测**（对齐研究结论第 2 条）；
* 未知键 / 缺字段 / 维度错 → :class:`rm75_drake.calibration.schema.CalibrationError`
  （**点名该键**）；
* **本模块零 pydrake 依赖**（只用标准库）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .schema import CalibrationError, N_JOINTS

__all__ = [
    "JointZeroSample",
    "ToolTcpSample",
    "BaseExtrinsicSample",
    "SampleSet",
    "parse_samples",
    "load_samples_file",
    "MIN_TOOL_TCP_SAMPLES",
    "MIN_BASE_EXTRINSIC_SAMPLES",
]

#: 工具 TCP 标定最少点对数（四点法下限）。
MIN_TOOL_TCP_SAMPLES = 4

#: 基座外参标定最少「尖对尖」点对数（设计文档要求 15–20 组）。
MIN_BASE_EXTRINSIC_SAMPLES = 15

#: 采样文件顶层键。
TOP_LEVEL_KEYS: Tuple[str, ...] = ("joint_zero", "tool_tcp", "base_extrinsic")

_JOINT_ZERO_KEYS: Tuple[str, ...] = (
    "joint",
    "index",
    "q_nominal",
    "q_offset",
    "pose_nominal",
    "pose_offset",
    "synthetic",
    "source",
)
_TOOL_TCP_KEYS: Tuple[str, ...] = ("local_point", "world_point", "synthetic", "source")
_BASE_EXTRINSIC_KEYS: Tuple[str, ...] = ("left_point", "right_point", "synthetic", "source")


# ── 内部工具 ────────────────────────────────────────────────────────────
def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CalibrationError(f"{path} 必须是映射（mapping），收到 {type(value).__name__}")
    return value


def _require_sequence(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise CalibrationError(f"{path} 必须是序列（list），收到 {type(value).__name__}")
    return value


def _check_keys(mapping: Mapping[str, Any], path: str, allowed: Sequence[str]) -> None:
    unknown = [k for k in mapping if k not in allowed]
    if unknown:
        raise CalibrationError(
            f"{path} 出现未知键：{sorted(unknown)}；该处允许的键只有 {list(allowed)}"
        )
    missing = [k for k in allowed if k not in mapping]
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


def _as_float_list(value: Any, path: str, length: int) -> Tuple[float, ...]:
    seq = _require_sequence(value, path)
    if len(seq) != length:
        raise CalibrationError(f"{path} 必须是 {length} 个数值，收到 {len(seq)} 个")
    return tuple(_as_float(v, f"{path}[{i}]") for i, v in enumerate(seq))


def _as_bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise CalibrationError(f"{path} 必须是布尔值（true/false），收到 {value!r}")
    return value


def _as_str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CalibrationError(f"{path} 必须是非空字符串，收到 {value!r}")
    return value


def _as_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CalibrationError(f"{path} 必须是整数，收到 {value!r}")
    return int(value)


def _as_matrix4(value: Any, path: str) -> Tuple[Tuple[float, float, float, float], ...]:
    rows = _require_sequence(value, path)
    if len(rows) != 4:
        raise CalibrationError(f"{path} 必须是 4×4 齐次位姿矩阵，收到 {len(rows)} 行")
    out = []
    for i, row in enumerate(rows):
        out.append(_as_float_list(row, f"{path}[{i}]", 4))
    return tuple(out)


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class JointZeroSample:
    """单个关节的 +10° FK 对比记录。"""

    joint: str
    index: int
    q_nominal: Tuple[float, ...]
    q_offset: Tuple[float, ...]
    pose_nominal: Tuple[Tuple[float, float, float, float], ...]
    pose_offset: Tuple[Tuple[float, float, float, float], ...]
    synthetic: bool = True
    source: str = ""

    def validate(self, path: str = "joint_zero") -> None:
        if len(self.q_nominal) != N_JOINTS or len(self.q_offset) != N_JOINTS:
            raise CalibrationError(
                f"{path}.q_nominal / q_offset 必须各 {N_JOINTS} 个关节角，"
                f"收到 {len(self.q_nominal)} / {len(self.q_offset)}"
            )
        if not 0 <= self.index < N_JOINTS:
            raise CalibrationError(f"{path}.index = {self.index} 超出 [0, {N_JOINTS - 1}]")
        for i, (a, b) in enumerate(zip(self.q_nominal, self.q_offset)):
            if i == self.index:
                if abs(b - a) < 1e-9:
                    raise CalibrationError(
                        f"{path}.q_offset[{i}] 与 q_nominal[{i}] 相同："
                        "该关节必须是 +10° 对比采样"
                    )
            elif abs(a - b) > 1e-12:
                raise CalibrationError(
                    f"{path}.q_offset[{i}] 与 q_nominal[{i}] 不一致："
                    "单关节采样只允许该关节发生变化"
                )


@dataclass(frozen=True)
class ToolTcpSample:
    """工具 TCP 的「局部点 ↔ 世界点」对。"""

    local_point: Tuple[float, float, float]
    world_point: Tuple[float, float, float]
    synthetic: bool = True
    source: str = ""


@dataclass(frozen=True)
class BaseExtrinsicSample:
    """基座外参的「左臂点 ↔ 右臂点」（尖对尖）点对。"""

    left_point: Tuple[float, float, float]
    right_point: Tuple[float, float, float]
    synthetic: bool = True
    source: str = ""


@dataclass(frozen=True)
class SampleSet:
    """三类采样的集合。"""

    joint_zero: Tuple[JointZeroSample, ...] = ()
    tool_tcp: Tuple[ToolTcpSample, ...] = ()
    base_extrinsic: Tuple[BaseExtrinsicSample, ...] = ()

    def as_dict(self) -> Dict[str, Any]:
        return {
            "joint_zero": [
                {
                    "joint": s.joint,
                    "index": s.index,
                    "q_nominal": list(s.q_nominal),
                    "q_offset": list(s.q_offset),
                    "pose_nominal": [list(r) for r in s.pose_nominal],
                    "pose_offset": [list(r) for r in s.pose_offset],
                    "synthetic": s.synthetic,
                    "source": s.source,
                }
                for s in self.joint_zero
            ],
            "tool_tcp": [
                {
                    "local_point": list(s.local_point),
                    "world_point": list(s.world_point),
                    "synthetic": s.synthetic,
                    "source": s.source,
                }
                for s in self.tool_tcp
            ],
            "base_extrinsic": [
                {
                    "left_point": list(s.left_point),
                    "right_point": list(s.right_point),
                    "synthetic": s.synthetic,
                    "source": s.source,
                }
                for s in self.base_extrinsic
            ],
        }


# ── 解析 ────────────────────────────────────────────────────────────────
def _parse_joint_zero(entry: Any, index: int) -> JointZeroSample:
    path = f"joint_zero[{index}]"
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _JOINT_ZERO_KEYS)
    sample = JointZeroSample(
        joint=_as_str(mapping["joint"], f"{path}.joint"),
        index=_as_int(mapping["index"], f"{path}.index"),
        q_nominal=_as_float_list(mapping["q_nominal"], f"{path}.q_nominal", N_JOINTS),
        q_offset=_as_float_list(mapping["q_offset"], f"{path}.q_offset", N_JOINTS),
        pose_nominal=_as_matrix4(mapping["pose_nominal"], f"{path}.pose_nominal"),
        pose_offset=_as_matrix4(mapping["pose_offset"], f"{path}.pose_offset"),
        synthetic=_as_bool(mapping["synthetic"], f"{path}.synthetic"),
        source=_as_str(mapping["source"], f"{path}.source"),
    )
    sample.validate(path)
    return sample


def _parse_tool_tcp(entry: Any, index: int) -> ToolTcpSample:
    path = f"tool_tcp[{index}]"
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _TOOL_TCP_KEYS)
    return ToolTcpSample(
        local_point=_as_float_list(mapping["local_point"], f"{path}.local_point", 3),
        world_point=_as_float_list(mapping["world_point"], f"{path}.world_point", 3),
        synthetic=_as_bool(mapping["synthetic"], f"{path}.synthetic"),
        source=_as_str(mapping["source"], f"{path}.source"),
    )


def _parse_base_extrinsic(entry: Any, index: int) -> BaseExtrinsicSample:
    path = f"base_extrinsic[{index}]"
    mapping = _require_mapping(entry, path)
    _check_keys(mapping, path, _BASE_EXTRINSIC_KEYS)
    return BaseExtrinsicSample(
        left_point=_as_float_list(mapping["left_point"], f"{path}.left_point", 3),
        right_point=_as_float_list(mapping["right_point"], f"{path}.right_point", 3),
        synthetic=_as_bool(mapping["synthetic"], f"{path}.synthetic"),
        source=_as_str(mapping["source"], f"{path}.source"),
    )


def parse_samples(data: Any, source_path: Optional[str] = None) -> SampleSet:
    """从已解析的 dict 构造 :class:`SampleSet`（校验失败抛 :class:`CalibrationError`）。"""
    mapping = _require_mapping(data, "samples")
    unknown = [k for k in mapping if k not in TOP_LEVEL_KEYS]
    if unknown:
        raise CalibrationError(
            f"samples 出现未知顶层键：{sorted(unknown)}；顶层键固定为 {list(TOP_LEVEL_KEYS)}"
        )

    joint_zero: list[JointZeroSample] = []
    if "joint_zero" in mapping:
        seq = _require_sequence(mapping["joint_zero"], "joint_zero")
        joint_zero = [_parse_joint_zero(e, i) for i, e in enumerate(seq)]
    names = [s.joint for s in joint_zero]
    if len(set(names)) != len(names):
        raise CalibrationError(f"joint_zero 里有重复的关节名：{sorted(names)}")

    tool_tcp: list[ToolTcpSample] = []
    if "tool_tcp" in mapping:
        seq = _require_sequence(mapping["tool_tcp"], "tool_tcp")
        tool_tcp = [_parse_tool_tcp(e, i) for i, e in enumerate(seq)]
    if tool_tcp and len(tool_tcp) < MIN_TOOL_TCP_SAMPLES:
        raise CalibrationError(
            f"tool_tcp 至少需要 {MIN_TOOL_TCP_SAMPLES} 组点对（四点法下限），"
            f"收到 {len(tool_tcp)} 组"
        )

    base_extrinsic: list[BaseExtrinsicSample] = []
    if "base_extrinsic" in mapping:
        seq = _require_sequence(mapping["base_extrinsic"], "base_extrinsic")
        base_extrinsic = [_parse_base_extrinsic(e, i) for i, e in enumerate(seq)]
    if base_extrinsic and len(base_extrinsic) < MIN_BASE_EXTRINSIC_SAMPLES:
        raise CalibrationError(
            f"base_extrinsic 至少需要 {MIN_BASE_EXTRINSIC_SAMPLES} 组「尖对尖」点对，"
            f"收到 {len(base_extrinsic)} 组"
        )

    return SampleSet(
        joint_zero=tuple(joint_zero),
        tool_tcp=tuple(tool_tcp),
        base_extrinsic=tuple(base_extrinsic),
    )


def load_samples_file(path) -> SampleSet:
    """从 YAML 文件加载采样点集。

    * 文件不存在 → :class:`FileNotFoundError`；
    * YAML 语法错误 / schema 违规 → :class:`CalibrationError`。
    """
    import yaml

    file_path = Path(path)
    try:
        text = file_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"找不到采样文件：{file_path}") from exc
    except OSError as exc:  # pragma: no cover - 权限/IO 异常
        raise CalibrationError(f"读取采样文件失败：{file_path}（{exc}）") from exc

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CalibrationError(f"采样 YAML 解析失败：{file_path}（{exc}）") from exc

    if data is None:
        raise CalibrationError(f"采样文件为空：{file_path}")

    return parse_samples(data)
