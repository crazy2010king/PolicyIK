"""golden-file 基线与「逐点一致」离线回放比较器（S07-T02）。

职责边界
--------
本模块提供**一份可复算、可判失效**的基线：

* :func:`build_golden` —— 把一次 :class:`~rm75_drake.regression.session.Session`
  （S07-T01）压成一份 **golden 基线**（关节样本 + 末端点 + 夹爪调度 + 指纹）；
* :func:`write_golden` / :func:`load_golden` —— 落盘 / 读回（**严格校验**）；
* :func:`compare_golden` —— **离线回放比较器**：用**同一实现**
  （:mod:`rm75_drake.kinematics.mdh` 的 numpy 正解）复算 session 的关节样本，
  与基线**逐点**比较，报告 ``num_tcp_mismatches``。

为什么必须绑指纹（假绿灯防线）
------------------------------
若基线不绑定 **模型 / 场景 / 任务**，换了机器人或场景之后基线仍然「通过」，
基线就成了自欺。因此基线固定携带三枚指纹：

============  =====================================================
指纹          绑定的文件
============  =====================================================
urdf_sha256   ``models/rm75_drake/rm75_drake.urdf``
scenario_sha256  ``scenarios/single_arm.yaml``
task_sha256   ``tasks/pick_place.yaml``
============  =====================================================

比较器每次都会用**当前仓库文件**重算指纹；不一致时
:attr:`GoldenComparison.fingerprints_match` 为 ``False``，
**不得**被静默判为「通过」——调用方据此判定「基线失效 → 需重新生成并复盘差异」。

口径纪律（**不得混算**）
------------------------
* 本模块的「复算」**只用** :mod:`rm75_drake.kinematics.mdh`（纯 numpy、零 pydrake）
  的**理论 MDH 正解**；
* **不**与 :func:`rm75_drake.models.eval_link_pose_in_world`（Drake 实体 FK）
  在同一口径里比较——两者相差官方 URDF 的 rpy 舍入残差（~2.2e-6 m，见
  :mod:`rm75_drake.kinematics.mdh` 的模块 docstring），混算会把「舍入残差」
  误判成「逐点漂移」；
* 基线**自带期望值**，比较器只做「同一实现复算 vs 基线」；
  若另需 Drake 侧对拍，必须写成**独立、可 SKIP** 的用例，不得作为 fast 档恒跑断言。

「逐点一致」的定义
------------------
**逐点一致** = :attr:`GoldenComparison.num_tcp_mismatches == 0`
（**每一个**采样点的三维末端位置都在容差内），**不是**「平均误差小」。
长度不一致（点数不同）直接判为不一致。

失败可解释
----------
* 结构 / 字段非法（未知键 / 缺键 / 维度不对 / NaN / 长度不一致 / 校验和不符）
  → :class:`GoldenError`，消息**点名违规键**；
* 比较器**不抛异常**地报告差异（差异是数据，不是错误），
  只有「被比较的对象类型不对」才抛 :class:`GoldenError`。

零 pydrake 依赖
--------------
本模块只依赖 numpy + :mod:`rm75_drake.kinematics.mdh`（纯 numpy）+
既有产物 schema（``recording`` / ``run_log`` / ``session``，均零 pydrake）
⇒ 基线的**存在性 / 指纹 / 负例**断言可以在**没有 pydrake** 的环境里恒跑。
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from ..kinematics import mdh

__all__ = [
    "GOLDEN_SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "FINGERPRINT_KEYS",
    "FINGERPRINT_FILES",
    "GoldenError",
    "GoldenBaseline",
    "GoldenComparison",
    "compute_golden_checksum",
    "build_golden",
    "write_golden",
    "load_golden",
    "compare_golden",
    "repo_fingerprints",
    "tcp_positions_from_joints",
]


#: golden 基线 schema 版本（**独立常量**，与 RECORDING / RUN_LOG / SESSION 互不影响）。
GOLDEN_SCHEMA_VERSION = 1

#: 顶层键（固定集合；未知键 / 缺键 → :class:`GoldenError`）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "created_at",
    "robot_name",
    "dt",
    "fingerprints",
    "joint_samples",
    "tcp_positions",
    "gripper_schedule",
    "checksum",
)

#: 指纹的**固定三键**（顺序即书写顺序）。
FINGERPRINT_KEYS: Tuple[str, ...] = (
    "urdf_sha256",
    "scenario_sha256",
    "task_sha256",
)

#: 夹爪调度表项的合法键（与 ``Recording.gripper_schedule`` 的口径一致）。
_GRIPPER_KEYS: Tuple[str, ...] = (
    "kind",
    "position",
    "speed",
    "force",
    "hold_seconds",
)

#: 关节维度（RM75 = 7 DoF）。
_JOINT_COUNT = 7

#: 末端位置维度（xyz）。
_TCP_COUNT = 3


class GoldenError(ValueError):
    """golden 基线结构 / 内容非法。

    消息必须指出**具体的键**（未知键 / 缺字段 / 维度不对 / NaN / 长度不一致 /
    校验和不符），与既有产物 schema 的校验纪律一致。
    """


# ── 指纹 ────────────────────────────────────────────────────────────────
def _file_sha256(path: Path) -> str:
    """返回文件的 sha256（十六进制）；文件不存在 → 空串（**不抛错**）。"""
    if not path.is_file():
        return ""
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fingerprint_files() -> Dict[str, Path]:
    """返回「指纹键 → 仓库文件」的映射（**惰性**解析，避免 import 期副作用）。"""
    from .. import paths

    return {
        "urdf_sha256": paths.models_dir / "rm75_drake" / "rm75_drake.urdf",
        "scenario_sha256": paths.scenarios_dir / "single_arm.yaml",
        "task_sha256": paths.task_files_dir / "pick_place.yaml",
    }


class _FingerprintFiles:
    """指纹文件映射的惰性代理（模块级常量 :data:`FINGERPRINT_FILES`）。"""

    def __getitem__(self, key: str) -> Path:
        return _fingerprint_files()[key]

    def __iter__(self):
        return iter(_fingerprint_files())

    def __len__(self) -> int:
        return len(_fingerprint_files())

    def items(self):
        return _fingerprint_files().items()

    def get(self, key: str, default: Any = None) -> Any:
        return _fingerprint_files().get(key, default)

    def __repr__(self) -> str:  # pragma: no cover - 调试辅助
        return f"FINGERPRINT_FILES({sorted(_fingerprint_files())})"


#: 指纹键 → 仓库文件的映射（惰性代理；``FINGERPRINT_FILES["urdf_sha256"]`` 取路径）。
FINGERPRINT_FILES = _FingerprintFiles()


def repo_fingerprints() -> Dict[str, str]:
    """返回**当前仓库文件**的三枚指纹（sha256）。

    * ``urdf_sha256`` 用 :func:`rm75_drake.recording.urdf_sha256`
      （与录制文件里的模型版本标识**同一个**口径）；
    * ``scenario_sha256`` / ``task_sha256`` 用文件 sha256。

    文件缺失 → 对应指纹为空串（**不抛错**），此时与基线的比对结果为 ``False``
    （即「基线失效」，而不是「静默通过」）。
    """
    from ..recording import urdf_sha256

    files = _fingerprint_files()
    return {
        "urdf_sha256": urdf_sha256(files["urdf_sha256"]),
        "scenario_sha256": _file_sha256(files["scenario_sha256"]),
        "task_sha256": _file_sha256(files["task_sha256"]),
    }


# ── 内部工具 ────────────────────────────────────────────────────────────
def _canonical_json(material: Any) -> str:
    """规范化 JSON（与 ``device/run_log.py::compute_checksum`` 同口径）。"""
    return json.dumps(material, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GoldenError(f"{path} 必须是对象（mapping），实际：{type(value).__name__}")
    return value


def _check_fingerprints(value: Any) -> Dict[str, str]:
    """校验指纹集合：未知键 / 缺键 / 类型非法 → :class:`GoldenError`（点名键）。"""
    mapping = _require_mapping(value, "golden.fingerprints")
    unknown = sorted(str(key) for key in mapping if key not in FINGERPRINT_KEYS)
    if unknown:
        raise GoldenError(
            f"golden.fingerprints 出现未知键：{unknown}；允许的键只有 {list(FINGERPRINT_KEYS)}"
        )
    missing = [key for key in FINGERPRINT_KEYS if key not in mapping]
    if missing:
        raise GoldenError(
            f"golden.fingerprints 缺少必填键：{missing}；允许的键只有 {list(FINGERPRINT_KEYS)}"
        )
    clean: Dict[str, str] = {}
    for key in FINGERPRINT_KEYS:
        item = mapping[key]
        if not isinstance(item, str):
            raise GoldenError(
                f"golden.fingerprints.{key} 必须是字符串，实际：{item!r}"
            )
        clean[key] = item
    return clean


def _check_vector_list(
    value: Any, *, path: str, width: int, allow_empty: bool = False
) -> List[List[float]]:
    """校验「N×width 的数值列表」；维度不对 / 非数值 / NaN → 点名报错。"""
    if not isinstance(value, (list, tuple)):
        raise GoldenError(f"{path} 必须是数组，实际：{type(value).__name__}")
    if not value and not allow_empty:
        raise GoldenError(f"{path} 必须是非空数组")
    out: List[List[float]] = []
    for index, item in enumerate(value):
        where = f"{path}[{index}]"
        if not isinstance(item, (list, tuple)):
            raise GoldenError(f"{where} 必须是数组，实际：{type(item).__name__}")
        if len(item) != width:
            raise GoldenError(
                f"{where} 维度必须是 {width}，实际 {len(item)} 维"
            )
        row: List[float] = []
        for axis, number in enumerate(item):
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise GoldenError(f"{where}[{axis}] 必须是数值，实际：{number!r}")
            value_f = float(number)
            if not math.isfinite(value_f):
                raise GoldenError(f"{where}[{axis}] 含 NaN/Inf：{number!r}")
            row.append(value_f)
        out.append(row)
    return out


def _check_gripper_schedule(value: Any) -> List[Dict[str, Any]]:
    """校验夹爪调度表（逐项点名违规键）。"""
    if not isinstance(value, (list, tuple)):
        raise GoldenError(
            f"golden.gripper_schedule 必须是数组，实际：{type(value).__name__}"
        )
    out: List[Dict[str, Any]] = []
    for index, item in enumerate(value):
        where = f"golden.gripper_schedule[{index}]"
        mapping = _require_mapping(item, where)
        unknown = sorted(str(key) for key in mapping if key not in _GRIPPER_KEYS)
        if unknown:
            raise GoldenError(f"{where} 出现未知键：{unknown}；允许的键只有 {list(_GRIPPER_KEYS)}")
        missing = [key for key in _GRIPPER_KEYS if key not in mapping]
        if missing:
            raise GoldenError(f"{where} 缺少必填键：{missing}；允许的键只有 {list(_GRIPPER_KEYS)}")
        kind = mapping["kind"]
        if not isinstance(kind, str) or not kind.strip():
            raise GoldenError(f"{where}.kind 必须是非空字符串，实际：{kind!r}")
        position = mapping["position"]
        if position is not None and (isinstance(position, bool) or not isinstance(position, int)):
            raise GoldenError(f"{where}.position 必须是整数或 null，实际：{position!r}")
        speed = mapping["speed"]
        if isinstance(speed, bool) or not isinstance(speed, int):
            raise GoldenError(f"{where}.speed 必须是整数，实际：{speed!r}")
        force = mapping["force"]
        if force is not None and (isinstance(force, bool) or not isinstance(force, int)):
            raise GoldenError(f"{where}.force 必须是整数或 null，实际：{force!r}")
        hold = mapping["hold_seconds"]
        if isinstance(hold, bool) or not isinstance(hold, (int, float)):
            raise GoldenError(f"{where}.hold_seconds 必须是数值，实际：{hold!r}")
        hold_f = float(hold)
        if not math.isfinite(hold_f) or hold_f < 0.0:
            raise GoldenError(f"{where}.hold_seconds 必须是非负有限值，实际：{hold!r}")
        out.append(
            {
                "kind": kind,
                "position": position,
                "speed": int(speed),
                "force": force,
                "hold_seconds": hold_f,
            }
        )
    return out


def _action_to_dict(action: Any) -> Dict[str, Any]:
    """把 :class:`~rm75_drake.gripper.GripperAction` 转成 golden 的调度表项。"""
    return {
        "kind": str(action.kind),
        "position": None if action.position is None else int(action.position),
        "speed": int(action.speed),
        "force": None if action.force is None else int(action.force),
        "hold_seconds": float(action.hold_seconds),
    }


# ── 校验和 ──────────────────────────────────────────────────────────────
def compute_golden_checksum(material: Mapping[str, Any]) -> str:
    """计算 golden 校验和（sha256，覆盖**除 ``checksum`` 外**的全部内容）。

    规范化方式与 :func:`rm75_drake.device.run_log.compute_checksum` **完全一致**
    （``json.dumps(..., ensure_ascii=False, sort_keys=True, separators=(",", ":"))``）。
    """
    payload = {key: value for key, value in material.items() if key != "checksum"}
    return _sha256_text(_canonical_json(payload))


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class GoldenBaseline:
    """一份 golden 基线（**不可变**）。

    字段
    ----
    schema_version:
        格式版本（当前 :data:`GOLDEN_SCHEMA_VERSION`）。
    created_at:
        基线生成时间（ISO-8601 UTC 字符串）。
    robot_name:
        机器人名。
    dt:
        采样周期（s，默认 5 ms）。
    fingerprints:
        三枚指纹（键集合固定为 :data:`FINGERPRINT_KEYS`）。
    joint_samples:
        ``N×7`` 关节角序列（rad）。
    tcp_positions:
        ``N×3`` 末端位置序列（m，与 ``joint_samples`` 一一对应）。
    gripper_schedule:
        夹爪调度表（逐项为 ``kind/position/speed/force/hold_seconds``）。
    checksum:
        除 ``checksum`` 外全部内容的规范化 JSON 的 sha256。
    """

    schema_version: int = GOLDEN_SCHEMA_VERSION
    created_at: str = ""
    robot_name: str = "rm75"
    dt: float = 0.005
    fingerprints: Mapping[str, str] = field(default_factory=dict)
    joint_samples: Tuple[Tuple[float, ...], ...] = ()
    tcp_positions: Tuple[Tuple[float, ...], ...] = ()
    gripper_schedule: Tuple[Mapping[str, Any], ...] = ()
    checksum: str = ""

    # -- 序列化 ----------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的字典（顶层键顺序固定）。"""
        return {
            "schema_version": int(self.schema_version),
            "created_at": str(self.created_at),
            "robot_name": str(self.robot_name),
            "dt": float(self.dt),
            "fingerprints": {key: self.fingerprints.get(key) for key in FINGERPRINT_KEYS},
            "joint_samples": [[float(v) for v in row] for row in self.joint_samples],
            "tcp_positions": [[float(v) for v in row] for row in self.tcp_positions],
            "gripper_schedule": [dict(item) for item in self.gripper_schedule],
            "checksum": str(self.checksum),
        }

    @classmethod
    def from_dict(cls, data: Any, *, verify_checksum: bool = True) -> "GoldenBaseline":
        """从字典构造（结构非法 → :class:`GoldenError`，**点名违规键**）。"""
        mapping = _require_mapping(data, "golden")
        unknown = sorted(str(key) for key in mapping if key not in TOP_LEVEL_KEYS)
        if unknown:
            raise GoldenError(
                f"golden 出现未知顶层键：{unknown}；允许的键只有 {list(TOP_LEVEL_KEYS)}"
            )
        missing = [key for key in TOP_LEVEL_KEYS if key not in mapping]
        if missing:
            raise GoldenError(
                f"golden 缺少必填顶层键：{missing}；允许的键只有 {list(TOP_LEVEL_KEYS)}"
            )
        version = mapping["schema_version"]
        if not isinstance(version, int) or isinstance(version, bool):
            raise GoldenError(f"golden.schema_version 必须是整数，实际：{version!r}")
        if version != GOLDEN_SCHEMA_VERSION:
            raise GoldenError(
                f"golden.schema_version={version}，本实现只支持 {GOLDEN_SCHEMA_VERSION}"
            )
        created_at = mapping["created_at"]
        if not isinstance(created_at, str):
            raise GoldenError(f"golden.created_at 必须是字符串，实际：{created_at!r}")
        robot_name = mapping["robot_name"]
        if not isinstance(robot_name, str) or not robot_name.strip():
            raise GoldenError(f"golden.robot_name 必须是非空字符串，实际：{robot_name!r}")
        dt = mapping["dt"]
        if isinstance(dt, bool) or not isinstance(dt, (int, float)):
            raise GoldenError(f"golden.dt 必须是数值，实际：{dt!r}")
        dt_f = float(dt)
        if not math.isfinite(dt_f) or dt_f <= 0.0:
            raise GoldenError(f"golden.dt 必须是正有限值，实际：{dt!r}")
        fingerprints = _check_fingerprints(mapping["fingerprints"])
        joint_samples = _check_vector_list(
            mapping["joint_samples"], path="golden.joint_samples", width=_JOINT_COUNT
        )
        tcp_positions = _check_vector_list(
            mapping["tcp_positions"], path="golden.tcp_positions", width=_TCP_COUNT
        )
        if len(joint_samples) != len(tcp_positions):
            raise GoldenError(
                "golden.joint_samples 与 golden.tcp_positions 长度不一致："
                f"{len(joint_samples)} vs {len(tcp_positions)}（必须一一对应）"
            )
        gripper_schedule = _check_gripper_schedule(mapping["gripper_schedule"])
        checksum = mapping["checksum"]
        if not isinstance(checksum, str):
            raise GoldenError(f"golden.checksum 必须是字符串，实际：{checksum!r}")

        baseline = cls(
            schema_version=version,
            created_at=created_at,
            robot_name=robot_name,
            dt=dt_f,
            fingerprints=fingerprints,
            joint_samples=tuple(tuple(row) for row in joint_samples),
            tcp_positions=tuple(tuple(row) for row in tcp_positions),
            gripper_schedule=tuple(dict(item) for item in gripper_schedule),
            checksum=checksum,
        )
        if verify_checksum:
            expected = compute_golden_checksum(baseline.to_dict())
            if expected != checksum:
                raise GoldenError(
                    "golden.checksum 不匹配（疑似篡改）："
                    f"记录值 {checksum!r}，重算值 {expected!r}"
                )
        return baseline


@dataclass(frozen=True)
class GoldenComparison:
    """一次「基线 vs 复算结果」的比对结论。

    字段
    ----
    fingerprints_match:
        基线携带的三枚指纹是否与**当前仓库文件**一致。
        ``False`` ⇒ **基线失效**（模型 / 场景 / 任务漂移），
        调用方**不得**据此判定「通过」。
    fingerprint_details:
        逐键明细（``{"urdf_sha256": {"baseline": ..., "current": ...}}``）。
    num_points:
        实际逐点比对的点数（两序列长度的较小值）。
    max_tcp_diff_m:
        逐点末端位置偏差的最大值（m）。
    num_tcp_mismatches:
        **超出容差的点数**（逐点一致的定义：``== 0``）。
    pointwise_identical:
        是否**逐点一致**（``num_tcp_mismatches == 0`` 且点数一致）。
    gripper_match:
        夹爪调度表是否与基线**完全一致**。
    joint_max_diff_rad:
        逐点关节角偏差的最大值（rad）。
    num_joint_mismatches:
        超出容差的关节点数。
    lengths_match:
        基线与复算结果的点数是否一致。
    baseline_points / session_points:
        两者的点数（便于定位「长度不一致」）。
    """

    fingerprints_match: bool = True
    fingerprint_details: Mapping[str, Mapping[str, str]] = field(default_factory=dict)
    num_points: int = 0
    max_tcp_diff_m: float = 0.0
    num_tcp_mismatches: int = 0
    pointwise_identical: bool = False
    gripper_match: bool = True
    joint_max_diff_rad: float = 0.0
    num_joint_mismatches: int = 0
    lengths_match: bool = True
    baseline_points: int = 0
    session_points: int = 0
    tcp_tolerance_m: float = 1e-9
    joint_tolerance_rad: float = 1e-9

    @property
    def baseline_invalidated(self) -> bool:
        """基线是否**已失效**（指纹与当前仓库文件不一致）。"""
        return not self.fingerprints_match

    @property
    def ok(self) -> bool:
        """整体结论：指纹有效 **且** 逐点一致 **且** 夹爪调度一致 **且** 关节一致。"""
        return (
            self.fingerprints_match
            and self.pointwise_identical
            and self.gripper_match
            and self.num_joint_mismatches == 0
            and self.lengths_match
        )

    def summary(self) -> str:
        """返回一行式摘要（人读；异常时定位用）。"""
        return (
            f"fingerprints_match={self.fingerprints_match} "
            f"points={self.num_points} "
            f"tcp_mismatches={self.num_tcp_mismatches} "
            f"max_tcp_diff_m={self.max_tcp_diff_m:.3e} "
            f"max_joint_diff_rad={self.joint_max_diff_rad:.3e} "
            f"gripper_match={self.gripper_match} "
            f"pointwise_identical={self.pointwise_identical} "
            f"ok={self.ok}"
        )


# ── 构造 / 读写 ─────────────────────────────────────────────────────────
def build_golden(
    session: Any,
    *,
    fingerprints: Mapping[str, str],
    tcp_positions: Sequence[Sequence[float]],
    created_at: Optional[str] = None,
) -> GoldenBaseline:
    """从一次会话产物构造 golden 基线。

    参数
    ----
    session:
        :class:`~rm75_drake.regression.session.Session`（S07-T01）。
    fingerprints:
        三枚指纹（键集合必须是 :data:`FINGERPRINT_KEYS`）。
    tcp_positions:
        ``N×3`` 末端位置（m），``N`` 必须等于 session 的采样点数。
    created_at:
        基线时间戳（默认取 ``session.created_at``）。

    失败模式
    --------
    * ``session`` 类型不对 → :class:`GoldenError`（点名 ``session``）；
    * 指纹集合非法 → :class:`GoldenError`（点名违规键）；
    * ``tcp_positions`` 维度 / 长度 / NaN 非法 → :class:`GoldenError`（点名违规键）。
    """
    from .session import Session

    if not isinstance(session, Session):
        raise GoldenError(
            f"session 必须是 Session 实例，实际：{type(session).__name__}"
        )
    recording = session.recording
    if recording is None:
        raise GoldenError("session.recording 未设置（无法构造 golden 基线）")

    clean_fingerprints = _check_fingerprints(fingerprints)
    joints = _check_vector_list(
        [[float(v) for v in sample.q] for sample in recording.samples],
        path="golden.joint_samples",
        width=_JOINT_COUNT,
    )
    tcp = _check_vector_list(
        [list(row) for row in tcp_positions],
        path="golden.tcp_positions",
        width=_TCP_COUNT,
    )
    if len(tcp) != len(joints):
        raise GoldenError(
            "golden.tcp_positions 与 session 采样点数不一致："
            f"{len(tcp)} vs {len(joints)}（必须一一对应）"
        )
    schedule = _check_gripper_schedule(
        [_action_to_dict(action) for action in recording.gripper_schedule]
    )
    baseline = GoldenBaseline(
        schema_version=GOLDEN_SCHEMA_VERSION,
        created_at=str(created_at or session.created_at),
        robot_name=str(recording.robot_name),
        dt=float(recording.dt),
        fingerprints=clean_fingerprints,
        joint_samples=tuple(tuple(row) for row in joints),
        tcp_positions=tuple(tuple(row) for row in tcp),
        gripper_schedule=tuple(dict(item) for item in schedule),
        checksum="",
    )
    return GoldenBaseline(
        schema_version=baseline.schema_version,
        created_at=baseline.created_at,
        robot_name=baseline.robot_name,
        dt=baseline.dt,
        fingerprints=baseline.fingerprints,
        joint_samples=baseline.joint_samples,
        tcp_positions=baseline.tcp_positions,
        gripper_schedule=baseline.gripper_schedule,
        checksum=compute_golden_checksum(baseline.to_dict()),
    )


def write_golden(baseline: GoldenBaseline, path: Any) -> Path:
    """把 golden 基线写到 JSON 文件（UTF-8、缩进 2、键序固定）。"""
    if not isinstance(baseline, GoldenBaseline):
        raise GoldenError(
            f"baseline 必须是 GoldenBaseline 实例，实际：{type(baseline).__name__}"
        )
    target = Path(path)
    if target.parent and not target.parent.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(baseline.to_dict(), ensure_ascii=False, indent=2)
    target.write_text(text + "\n", encoding="utf-8")
    return target


def load_golden(path: Any) -> GoldenBaseline:
    """读回 golden 基线并**严格校验**（含校验和 ⇒ 篡改可检出）。

    失败模式
    --------
    * 文件不存在 → :class:`FileNotFoundError`；
    * JSON 语法错误 → :class:`GoldenError`；
    * 结构 / 字段非法 → :class:`GoldenError`（点名违规键）；
    * 校验和不匹配 → :class:`GoldenError`（点名 ``checksum``）。
    """
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GoldenError(f"golden 文件 JSON 解析失败（{target}）：{exc}") from exc
    return GoldenBaseline.from_dict(data)


# ── 比较器 ──────────────────────────────────────────────────────────────
def tcp_positions_from_joints(joint_samples: Sequence[Sequence[float]]) -> List[List[float]]:
    """用 **:mod:`rm75_drake.kinematics.mdh`** 的理论正解复算末端位置。

    参数
    ----
    joint_samples:
        ``N×7`` 关节角序列（rad）。

    返回
    ----
    ``N×3`` 的末端位置列表（m，base_link 系）。

    异常
    ----
    ValueError
        关节维度不是 7（含 NaN/Inf）。
    """
    out: List[List[float]] = []
    for row in joint_samples:
        position = mdh.link7_position(row)
        out.append([float(position[0]), float(position[1]), float(position[2])])
    return out


def compare_golden(
    baseline: GoldenBaseline,
    session: Any,
    *,
    joint_tolerance_rad: float = 1e-9,
    tcp_tolerance_m: float = 1e-9,
    current_fingerprints: Optional[Mapping[str, str]] = None,
) -> GoldenComparison:
    """比较 golden 基线与一次会话产物（**逐点**）。

    复算口径
    --------
    用 **同一实现**（:mod:`rm75_drake.kinematics.mdh`，纯 numpy、零 pydrake）
    对 session 的关节样本**逐点**复算 link_7 位置，再与基线的
    ``tcp_positions`` **逐点**比较。

    「逐点一致」的定义
    ------------------
    :attr:`GoldenComparison.num_tcp_mismatches == 0`（每个点的三维位置都在
    ``tcp_tolerance_m`` 内）**且**点数一致。**不是**「平均误差小」。

    基线失效语义
    ------------
    指纹与当前仓库文件不一致时 :attr:`GoldenComparison.fingerprints_match`
    为 ``False``，比较器**不会**因此抛错，但也**不得**被判为「通过」——
    调用方必须据此判定「基线失效 → 需重新生成并复盘差异」
    （见 :attr:`GoldenComparison.ok` 与 :attr:`GoldenComparison.baseline_invalidated`）。

    异常
    ----
    GoldenError
        ``baseline`` / ``session`` 类型不对。
    """
    from .session import Session

    if not isinstance(baseline, GoldenBaseline):
        raise GoldenError(
            f"baseline 必须是 GoldenBaseline 实例，实际：{type(baseline).__name__}"
        )
    if not isinstance(session, Session):
        raise GoldenError(
            f"session 必须是 Session 实例，实际：{type(session).__name__}"
        )
    recording = session.recording
    if recording is None:
        raise GoldenError("session.recording 未设置（无法比较）")

    # -- 指纹（与当前仓库文件比对）--------------------------------------
    current = (
        dict(current_fingerprints)
        if current_fingerprints is not None
        else repo_fingerprints()
    )
    details: Dict[str, Dict[str, str]] = {}
    fingerprints_match = True
    for key in FINGERPRINT_KEYS:
        baseline_value = str(baseline.fingerprints.get(key, ""))
        current_value = str(current.get(key, ""))
        details[key] = {"baseline": baseline_value, "current": current_value}
        if baseline_value != current_value:
            fingerprints_match = False

    # -- 关节 / 末端逐点比较 ---------------------------------------------
    base_joints = np.asarray(baseline.joint_samples, dtype=float)
    base_tcp = np.asarray(baseline.tcp_positions, dtype=float)
    session_joints = np.asarray([sample.q for sample in recording.samples], dtype=float)

    baseline_points = int(base_joints.shape[0]) if base_joints.size else 0
    session_points = int(session_joints.shape[0]) if session_joints.size else 0
    lengths_match = baseline_points == session_points
    num_points = min(baseline_points, session_points)

    joint_max_diff = 0.0
    num_joint_mismatches = 0
    max_tcp_diff = 0.0
    num_tcp_mismatches = 0
    if num_points > 0:
        joint_diff = np.abs(base_joints[:num_points] - session_joints[:num_points])
        joint_max_diff = float(np.max(joint_diff))
        num_joint_mismatches = int(
            np.count_nonzero(np.max(joint_diff, axis=1) > float(joint_tolerance_rad))
        )
        recomputed = np.asarray(
            tcp_positions_from_joints(session_joints[:num_points]), dtype=float
        )
        tcp_diff = np.abs(base_tcp[:num_points] - recomputed)
        max_tcp_diff = float(np.max(tcp_diff))
        num_tcp_mismatches = int(
            np.count_nonzero(np.max(tcp_diff, axis=1) > float(tcp_tolerance_m))
        )
    # 点数不一致 ⇒ 多出来的点必然「对不上」。
    num_tcp_mismatches += abs(baseline_points - session_points)
    num_joint_mismatches += abs(baseline_points - session_points)

    # -- 夹爪调度 --------------------------------------------------------
    session_schedule = [_action_to_dict(action) for action in recording.gripper_schedule]
    baseline_schedule = [dict(item) for item in baseline.gripper_schedule]
    gripper_match = session_schedule == baseline_schedule

    return GoldenComparison(
        fingerprints_match=fingerprints_match,
        fingerprint_details=details,
        num_points=num_points,
        max_tcp_diff_m=max_tcp_diff,
        num_tcp_mismatches=num_tcp_mismatches,
        pointwise_identical=(num_tcp_mismatches == 0) and lengths_match,
        gripper_match=gripper_match,
        joint_max_diff_rad=joint_max_diff,
        num_joint_mismatches=num_joint_mismatches,
        lengths_match=lengths_match,
        baseline_points=baseline_points,
        session_points=session_points,
        tcp_tolerance_m=float(tcp_tolerance_m),
        joint_tolerance_rad=float(joint_tolerance_rad),
    )


def _utc_now() -> str:
    """当前 UTC 时间戳（ISO-8601，秒精度）。"""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
