"""录制 / 回放格式（S02-T08）。

职责边界
--------
本模块定义**一次真实运行的关节流**的落盘格式，并提供**离线回放复检**：

* :func:`record_trajectory` —— 把 :class:`~rm75_drake.planning.trajectory.Trajectory`
  落成一个 JSON 录制文件（S07 的「一次真机运行全部消息录制 → 离线重放」骨架）；
* :func:`load_recording` —— 读回并**严格校验**（未知字段 / 缺字段 / 维度不对都报错，
  **不静默默认值**）；
* :func:`replay_check` —— 用 **S02-T05 的同一套碰撞复检口径**
  （同一个 ``edge_step_size``）回放录制内容，报告逃逸数。

格式（JSON / UTF-8）
--------------------
顶层键固定为::

    {
      "schema_version": 1,
      "robot_name": "rm75",
      "dt": 0.005,
      "samples": [{"seq": 0, "t_abs": 0.0, "q": [0.0, ...7 项...]}],
      "gripper_schedule": [{"kind": "open", "position": 1000,
                            "speed": 100, "force": null,
                            "hold_seconds": 0.5}],
      "created_at": "2026-01-01T00:00:00Z",
      "meta": {"urdf_sha256": "...", "scenario_path": "scenarios/single_arm.yaml"}
    }

单位约定：关节角 rad、时刻 s、夹爪开口 u ∈ [0, 1000]（与
:mod:`rm75_drake.scenario` 一致）。

失败可解释
----------
* 录制文件结构非法 → :class:`RecordingError`（消息指出**哪个键**非法）；
* 回放时位形非法（非有限值 / 维度不对）→ :class:`RecordingError`；
* 回放发现碰撞逃逸 → **不抛异常**，由 :attr:`ReplayResult.num_escapes` 表达，
  由调用方（CLI）转成**非 0 退出码**。
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

from .gripper import GripperAction
from .planning.gcs import DEFAULT_EDGE_STEP_SIZE, collision_recheck
from .planning.trajectory import JOINT_COUNT

__all__ = [
    "RECORDING_SCHEMA_VERSION",
    "RecordingError",
    "RecordingSample",
    "Recording",
    "ReplayResult",
    "record_trajectory",
    "write_recording",
    "load_recording",
    "replay_check",
    "urdf_sha256",
]


#: 录制文件 schema 版本。
RECORDING_SCHEMA_VERSION = 1

#: 顶层键（固定集合；出现未知键 → 报错）。
_TOP_LEVEL_KEYS = (
    "schema_version",
    "robot_name",
    "dt",
    "samples",
    "gripper_schedule",
    "created_at",
    "meta",
)

#: meta 的键（固定集合）。
_META_KEYS = ("urdf_sha256", "scenario_path")


class RecordingError(ValueError):
    """录制文件结构 / 内容非法。

    消息必须指出**具体的键**与期望值，便于人工排查。
    """


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RecordingSample:
    """录制文件里的一个采样点。

    字段
    ----
    seq:
        序号（从 0 逐点 +1）。
    t_abs:
        绝对时刻（s）。
    q:
        关节角（rad，7 项）。
    """

    seq: int
    t_abs: float
    q: Tuple[float, ...]

    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的字典。"""
        return {"seq": int(self.seq), "t_abs": float(self.t_abs), "q": [float(v) for v in self.q]}

    @classmethod
    def from_dict(cls, data: Any, *, where: str = "sample") -> "RecordingSample":
        """从字典构造（结构非法 → :class:`RecordingError`）。"""
        if not isinstance(data, Mapping):
            raise RecordingError(f"{where} 必须是对象，实际：{type(data).__name__}")
        unknown = set(data) - {"seq", "t_abs", "q"}
        if unknown:
            raise RecordingError(f"{where} 含未知字段：{sorted(unknown)}")
        for key in ("seq", "t_abs", "q"):
            if key not in data:
                raise RecordingError(f"{where} 缺少必填字段 {key!r}")
        try:
            seq = int(data["seq"])
            t_abs = float(data["t_abs"])
            q = tuple(float(v) for v in data["q"])
        except (TypeError, ValueError) as exc:
            raise RecordingError(f"{where} 的字段类型非法：{exc}") from exc
        if len(q) != JOINT_COUNT:
            raise RecordingError(
                f"{where}.q 必须是 {JOINT_COUNT} 维（RM75-B 7 DoF），实际 {len(q)} 维"
            )
        if not all(math.isfinite(v) for v in q):
            raise RecordingError(f"{where}.q 含 NaN/Inf：{list(q)}")
        if not math.isfinite(t_abs):
            raise RecordingError(f"{where}.t_abs 必须是有限值，实际：{t_abs!r}")
        return cls(seq=seq, t_abs=t_abs, q=q)


@dataclass(frozen=True)
class Recording:
    """一次录制的完整内容。

    字段
    ----
    schema_version:
        格式版本（当前 :data:`RECORDING_SCHEMA_VERSION`）。
    robot_name:
        机器人名（回放时用于校验模型）。
    dt:
        采样周期（s，默认 5 ms）。
    samples:
        采样点序列（至少 1 个）。
    gripper_schedule:
        夹爪动作调度表。
    created_at:
        录制时间（ISO-8601 UTC 字符串）。
    meta:
        元信息（``urdf_sha256`` / ``scenario_path``）。
    """

    schema_version: int = RECORDING_SCHEMA_VERSION
    robot_name: str = "rm75"
    dt: float = 0.005
    samples: Tuple[RecordingSample, ...] = ()
    gripper_schedule: Tuple[GripperAction, ...] = ()
    created_at: str = ""
    meta: Mapping[str, Any] = field(default_factory=dict)

    # -- 便捷视图 --------------------------------------------------------
    def q_matrix(self) -> np.ndarray:
        """返回 ``(N, 7)`` 的关节角矩阵（numpy）。"""
        if not self.samples:
            return np.zeros((0, JOINT_COUNT), dtype=float)
        return np.asarray([s.q for s in self.samples], dtype=float)

    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的字典（顶层键顺序固定）。"""
        return {
            "schema_version": int(self.schema_version),
            "robot_name": str(self.robot_name),
            "dt": float(self.dt),
            "samples": [s.to_dict() for s in self.samples],
            "gripper_schedule": [_action_to_dict(a) for a in self.gripper_schedule],
            "created_at": str(self.created_at),
            "meta": {key: self.meta.get(key) for key in _META_KEYS},
        }

    @classmethod
    def from_dict(cls, data: Any) -> "Recording":
        """从字典构造（结构非法 → :class:`RecordingError`）。"""
        if not isinstance(data, Mapping):
            raise RecordingError(
                f"录制文件顶层必须是对象（JSON object），实际：{type(data).__name__}"
            )
        unknown = set(data) - set(_TOP_LEVEL_KEYS)
        if unknown:
            raise RecordingError(
                f"录制文件含未知顶层键：{sorted(unknown)}"
                f"（允许：{list(_TOP_LEVEL_KEYS)}）"
            )
        for key in _TOP_LEVEL_KEYS:
            if key not in data:
                raise RecordingError(f"录制文件缺少必填顶层键 {key!r}")
        version = data["schema_version"]
        if not isinstance(version, int) or isinstance(version, bool):
            raise RecordingError(f"schema_version 必须是整数，实际：{version!r}")
        if version != RECORDING_SCHEMA_VERSION:
            raise RecordingError(
                f"录制文件 schema_version={version}，本实现只支持 "
                f"{RECORDING_SCHEMA_VERSION}"
            )
        robot_name = data["robot_name"]
        if not isinstance(robot_name, str) or not robot_name.strip():
            raise RecordingError(f"robot_name 必须是非空字符串，实际：{robot_name!r}")
        try:
            dt = float(data["dt"])
        except (TypeError, ValueError) as exc:
            raise RecordingError(f"dt 必须是数值，实际：{data['dt']!r}") from exc
        if not math.isfinite(dt) or dt <= 0.0:
            raise RecordingError(f"dt 必须是正有限值，实际：{dt!r}")
        raw_samples = data["samples"]
        if not isinstance(raw_samples, list) or not raw_samples:
            raise RecordingError("samples 必须是非空数组")
        samples = tuple(
            RecordingSample.from_dict(item, where=f"samples[{index}]")
            for index, item in enumerate(raw_samples)
        )
        for index in range(1, len(samples)):
            if samples[index].seq != samples[index - 1].seq + 1:
                raise RecordingError(
                    f"samples[{index}].seq 必须逐点 +1："
                    f"{samples[index - 1].seq} → {samples[index].seq}"
                )
            if samples[index].t_abs <= samples[index - 1].t_abs:
                raise RecordingError(
                    f"samples[{index}].t_abs 必须严格递增："
                    f"{samples[index - 1].t_abs} → {samples[index].t_abs}"
                )
        raw_actions = data["gripper_schedule"]
        if not isinstance(raw_actions, list):
            raise RecordingError("gripper_schedule 必须是数组")
        actions = tuple(
            _action_from_dict(item, where=f"gripper_schedule[{index}]")
            for index, item in enumerate(raw_actions)
        )
        created_at = data["created_at"]
        if not isinstance(created_at, str):
            raise RecordingError(f"created_at 必须是字符串，实际：{created_at!r}")
        meta = data["meta"]
        if not isinstance(meta, Mapping):
            raise RecordingError(f"meta 必须是对象，实际：{type(meta).__name__}")
        unknown_meta = set(meta) - set(_META_KEYS)
        if unknown_meta:
            raise RecordingError(f"meta 含未知键：{sorted(unknown_meta)}")
        return cls(
            schema_version=version,
            robot_name=robot_name,
            dt=dt,
            samples=samples,
            gripper_schedule=actions,
            created_at=created_at,
            meta=dict(meta),
        )


@dataclass(frozen=True)
class ReplayResult:
    """回放复检结果。

    字段
    ----
    num_samples:
        实际复检的位形数（**稠密化之后**，与 S02-T05 口径一致）。
    num_escapes:
        **逃逸数**（发生碰撞的位形数）。硬要求为 0。
    max_joint_step:
        相邻采样点之间的**最大单轴关节步长**（rad）——用于发现录制被篡改 /
        丢帧。
    escape_indices:
        逃逸位形在稠密点列中的下标（升序）。
    """

    num_samples: int
    num_escapes: int
    max_joint_step: float
    escape_indices: Tuple[int, ...] = ()

    @property
    def ok(self) -> bool:
        """是否零逃逸。"""
        return self.num_escapes == 0


# ── 录制 ────────────────────────────────────────────────────────────────
def urdf_sha256(path: Optional[Any] = None) -> str:
    """返回机器人 URDF 的 sha256（模型版本标识）。

    ``path`` 为 ``None`` → 用仓库内 ``models/rm75_drake/rm75_drake.urdf``。
    文件不存在 → 返回空串（**不抛错**：录制不应因为版本标识缺失而失败）。
    """
    from . import paths

    if path is None:
        path = paths.models_dir / "rm75_drake" / "rm75_drake.urdf"
    file_path = Path(path)
    if not file_path.is_file():
        return ""
    digest = hashlib.sha256()
    with file_path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def record_trajectory(
    trajectory: Any,
    path: Optional[Any] = None,
    *,
    robot_name: Optional[str] = None,
    scenario_path: Optional[Any] = None,
    meta: Optional[Mapping[str, Any]] = None,
    created_at: Optional[str] = None,
) -> Recording:
    """把一条轨迹录制成 :class:`Recording`（并可选写盘）。

    参数
    ----
    trajectory:
        :class:`~rm75_drake.planning.trajectory.Trajectory`。
    path:
        可选的输出路径；给出时**同时**写盘（JSON / UTF-8）。
    robot_name:
        覆盖轨迹里的机器人名（默认取轨迹自身的 ``robot_name``）。
    scenario_path:
        场景真相源路径（写进 ``meta.scenario_path``，供回放重建场景）。
    meta:
        额外的 meta 项（只允许已知键，其余 → :class:`RecordingError`）。
    created_at:
        录制时间戳（默认当前 UTC 时间）。

    返回
    ----
    :class:`Recording`。

    失败模式
    --------
    * 轨迹类型不对 → :class:`RecordingError`；
    * ``meta`` 含未知键 → :class:`RecordingError`（不静默丢弃）。
    """
    from .planning.trajectory import Trajectory

    if not isinstance(trajectory, Trajectory):
        raise RecordingError(f"trajectory 必须是 Trajectory，实际：{type(trajectory).__name__}")
    meta_map: Dict[str, Any] = {
        "urdf_sha256": urdf_sha256(),
        "scenario_path": str(scenario_path) if scenario_path is not None else "",
    }
    if meta:
        unknown = set(meta) - set(_META_KEYS)
        if unknown:
            raise RecordingError(f"meta 含未知键：{sorted(unknown)}（允许：{list(_META_KEYS)}）")
        meta_map.update(meta)
    stamp = created_at or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    recording = Recording(
        schema_version=RECORDING_SCHEMA_VERSION,
        robot_name=robot_name or trajectory.robot_name,
        dt=float(trajectory.dt),
        samples=tuple(
            RecordingSample(seq=s.seq, t_abs=s.t_abs, q=tuple(s.q)) for s in trajectory.samples
        ),
        gripper_schedule=tuple(trajectory.gripper_schedule),
        created_at=stamp,
        meta=meta_map,
    )
    if path is not None:
        write_recording(recording, path)
    return recording


def write_recording(recording: Recording, path: Any) -> Path:
    """把录制写到 JSON 文件（UTF-8、缩进 2、键序固定）。"""
    if not isinstance(recording, Recording):
        raise RecordingError(f"recording 必须是 Recording，实际：{type(recording).__name__}")
    target = Path(path)
    if target.parent and not target.parent.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(recording.to_dict(), ensure_ascii=False, indent=2)
    target.write_text(text + "\n", encoding="utf-8")
    return target


def load_recording(path: Any) -> Recording:
    """读回录制文件并严格校验。

    失败模式
    --------
    * 文件不存在 → :class:`FileNotFoundError`；
    * JSON 语法错误 → :class:`RecordingError`；
    * 结构 / 字段非法 → :class:`RecordingError`（消息指出具体的键）。
    """
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise RecordingError(f"录制文件不是合法 JSON（{target}）：{exc}") from exc
    return Recording.from_dict(data)


# ── 回放复检 ────────────────────────────────────────────────────────────
def replay_check(
    recording: Recording,
    scene: Any,
    *,
    edge_step_size: float = DEFAULT_EDGE_STEP_SIZE,
) -> ReplayResult:
    """回放录制内容并做**碰撞复检**（与 S02-T05 同一口径）。

    参数
    ----
    recording:
        录制内容（:class:`Recording`）。
    scene:
        场景装配产物（提供 ``checker``）。
    edge_step_size:
        复检密采样步长（rad）；**必须与 S02-T05 的默认值一致**
        （:data:`rm75_drake.planning.gcs.DEFAULT_EDGE_STEP_SIZE`），
        否则「零逃逸」口径会漂移。

    返回
    ----
    :class:`ReplayResult`（``num_escapes`` 必须为 0 才算通过）。

    失败模式
    --------
    * 录制为空 / 类型不对 → :class:`RecordingError`；
    * 场景没有 checker → :class:`RecordingError`。
    """
    if not isinstance(recording, Recording):
        raise RecordingError(f"recording 必须是 Recording，实际：{type(recording).__name__}")
    if not recording.samples:
        raise RecordingError("录制内容为空（samples 为空），无法回放复检")
    if getattr(scene, "checker", None) is None:
        raise RecordingError("场景没有 checker（请用 build_scene 装配场景），无法回放复检")
    matrix = recording.q_matrix()
    recheck = collision_recheck(scene, matrix, edge_step_size=edge_step_size)
    steps = np.abs(np.diff(matrix, axis=0)) if matrix.shape[0] > 1 else np.zeros((0, JOINT_COUNT))
    max_step = float(np.max(steps)) if steps.size else 0.0
    return ReplayResult(
        num_samples=int(recheck.num_samples),
        num_escapes=int(recheck.num_escapes),
        max_joint_step=max_step,
        escape_indices=tuple(int(i) for i in recheck.escape_indices),
    )


# ── 夹爪动作序列化 ──────────────────────────────────────────────────────
def _action_to_dict(action: GripperAction) -> Dict[str, Any]:
    """夹爪动作 → JSON 字典。"""
    return {
        "kind": action.kind,
        "position": action.position,
        "speed": int(action.speed),
        "force": action.force,
        "hold_seconds": float(action.hold_seconds),
    }


def _action_from_dict(data: Any, *, where: str) -> GripperAction:
    """JSON 字典 → 夹爪动作（非法 → :class:`RecordingError`）。"""
    if not isinstance(data, Mapping):
        raise RecordingError(f"{where} 必须是对象，实际：{type(data).__name__}")
    unknown = set(data) - {"kind", "position", "speed", "force", "hold_seconds"}
    if unknown:
        raise RecordingError(f"{where} 含未知字段：{sorted(unknown)}")
    try:
        return GripperAction(
            kind=str(data["kind"]),
            position=None if data.get("position") is None else int(data["position"]),
            speed=int(data.get("speed", 100)),
            force=None if data.get("force") is None else int(data.get("force")),
            hold_seconds=float(data.get("hold_seconds", 0.0)),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RecordingError(f"{where} 的字段非法：{exc}") from exc
