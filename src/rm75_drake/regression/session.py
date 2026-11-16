"""全链路会话产物（session schema v1，S07-T01）。

职责边界
--------
把一次运行的**两份既有产物**内联进**一个自洽文件**：

* ``recording`` 段 = :meth:`rm75_drake.recording.Recording.to_dict`
  （S02，关节流 + 夹爪调度）；
* ``run_log`` 段 = :meth:`rm75_drake.device.run_log.RunLog.to_dict`
  （S03，设备层消息流）；
* 顶层再加 ``meta``（模型 / 场景 / 任务指纹）与 **``checksum``**
  （覆盖 ``created_at + meta + recording + run_log`` 的 sha256）。

**只读消费**：本模块**不修改** ``recording.py`` / ``run_log.py`` 任何一行
（既有 schema 版本由 :func:`assert_source_schemas_unchanged` 钉住）。

格式（JSON / UTF-8）
--------------------
顶层键固定为（未知键 / 缺键 / 类型错 ⇒ :class:`SessionError`，**点名该键**）::

    {
      "schema_version": 1,
      "created_at": "2026-01-01T00:00:00Z",
      "meta": {"robot_name": "rm75",
               "task_path": "tasks/pick.yaml",
               "scenario_path": "scenarios/single_arm.yaml",
               "urdf_sha256": "...",
               "task_sha256": "...",
               "scenario_sha256": "..."},
      "recording": { ... Recording.to_dict() ... },
      "run_log": { ... RunLog.to_dict() ... },
      "checksum": "<sha256 hex>"
    }

校验和口径
----------
``checksum`` = ``sha256(规范化 JSON(created_at + meta + recording + run_log))``，
规范化方式与 :func:`rm75_drake.device.run_log.compute_checksum` **完全一致**
（``json.dumps(..., ensure_ascii=False, sort_keys=True, separators=(",", ":"))``）。

**关于「是哪一段不符」的说明（诚实口径）**
----------------------------------------
单一 sha256 覆盖整份材料时，信息论上**无法**从哈希反推是哪一段被改；
因此本模块采用「**段内自校验优先 + 候选段列举 + 各段当前摘要**」的方式报告：

* ``run_log`` 段**自带**校验和 ⇒ 该段被改会**精确**报 ``run_log``；
* ``recording`` 段**没有**自带校验和 ⇒ 段内只能查结构（维度 / seq 连续性）；
  数值级篡改只能定位到「``recording`` / ``meta`` / ``created_at`` 候选段」，
  错误消息里给出**各段当前摘要**（:func:`session_segment_digests`），
  供下游比较器（S07-T02 golden-file 比对）逐段定位。

失败可解释
----------
* 顶层 / meta 结构非法 → :class:`SessionError`（点名具体键）；
* ``recording`` 段非法 → :class:`SessionError`（前缀 ``session.recording``，
  内嵌 :class:`~rm75_drake.recording.RecordingError` 的原始键名信息）；
* ``run_log`` 段非法 / 段内校验和不符 → :class:`SessionError`
  （前缀 ``session.run_log``）；
* 顶层 ``checksum`` 不符 → :class:`SessionError`（点名 ``checksum``）。
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from ..device.run_log import RUN_LOG_SCHEMA_VERSION, RunLog, RunLogError
from ..device.run_log import compute_checksum as compute_run_log_checksum
from ..recording import RECORDING_SCHEMA_VERSION, Recording, RecordingError

__all__ = [
    "SESSION_SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "META_KEYS",
    "SessionError",
    "Session",
    "build_session",
    "write_session",
    "load_session",
    "compute_session_checksum",
    "session_segment_digests",
    "source_schema_versions",
    "assert_source_schemas_unchanged",
]


#: 会话文件 schema 版本（**独立常量**，与 RECORDING / RUN_LOG 互不影响）。
SESSION_SCHEMA_VERSION = 1

#: 顶层键（固定集合；出现未知键 / 缺键 → :class:`SessionError`）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "created_at",
    "meta",
    "recording",
    "run_log",
    "checksum",
)

#: ``meta`` 的合法键（固定集合）。
META_KEYS: Tuple[str, ...] = (
    "robot_name",
    "task_path",
    "scenario_path",
    "urdf_sha256",
    "task_sha256",
    "scenario_sha256",
)

#: 校验和覆盖的段（顺序固定，便于人读）。
_CHECKSUM_SEGMENTS: Tuple[str, ...] = ("created_at", "meta", "recording", "run_log")


class SessionError(ValueError):
    """会话文件结构 / 内容非法。

    消息必须指出**具体的键**（未知键 / 缺字段 / 校验和不一致），
    与 :class:`rm75_drake.scenario` / :class:`rm75_drake.device.run_log.RunLogError`
    的校验纪律一致。
    """


# ── 内部工具 ────────────────────────────────────────────────────────────
def _canonical_json(material: Any) -> str:
    """规范化 JSON（与 ``device/run_log.py::compute_checksum`` 同口径）。"""
    return json.dumps(material, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SessionError(f"{path} 必须是对象（mapping），实际：{type(value).__name__}")
    return value


def _check_meta(meta: Mapping[str, Any]) -> Dict[str, Any]:
    """校验 ``meta``：未知键 / 值类型非法 → :class:`SessionError`（点名键名）。"""
    unknown = sorted(str(key) for key in meta if key not in META_KEYS)
    if unknown:
        raise SessionError(
            f"session.meta 出现未知键：{unknown}；允许的键只有 {list(META_KEYS)}"
        )
    clean: Dict[str, Any] = {}
    for key in META_KEYS:
        if key not in meta:
            continue
        value = meta[key]
        if value is None:
            clean[key] = None
            continue
        if not isinstance(value, str):
            raise SessionError(
                f"session.meta.{key} 必须是字符串或 null，实际：{value!r}"
            )
        clean[key] = value
    return clean


# ── 校验和 ──────────────────────────────────────────────────────────────
def compute_session_checksum(
    created_at: Any,
    meta: Mapping[str, Any],
    recording: Any,
    run_log: Any,
) -> str:
    """计算会话校验和（sha256，覆盖 ``created_at + meta + recording + run_log``）。

    参数
    ----
    created_at:
        会话时间戳（字符串）。
    meta:
        会话元信息（未知键会被忽略——本函数只取白名单内的键）。
    recording:
        :class:`~rm75_drake.recording.Recording`（或等价的 ``to_dict`` 载体）。
    run_log:
        :class:`~rm75_drake.device.run_log.RunLog`（或等价的 ``to_dict`` 载体）。
    """
    material = {
        "created_at": str(created_at),
        "meta": {key: meta.get(key) for key in META_KEYS},
        "recording": recording.to_dict(),
        "run_log": run_log.to_dict(),
    }
    return _sha256_text(_canonical_json(material))


def session_segment_digests(session: "Session") -> Dict[str, str]:
    """返回各段的 sha256 摘要（供下游逐段比对 / 定位篡改）。

    键：``created_at`` / ``meta`` / ``recording`` / ``run_log`` / ``checksum``。
    """
    digests = {
        "created_at": _sha256_text(str(session.created_at)),
        "meta": _sha256_text(
            _canonical_json({key: session.meta.get(key) for key in META_KEYS})
        ),
        "recording": _sha256_text(_canonical_json(session.recording.to_dict())),
        "run_log": _sha256_text(_canonical_json(session.run_log.to_dict())),
    }
    digests["checksum"] = str(session.checksum)
    return digests


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Session:
    """一次运行的**全链路会话产物**。

    字段
    ----
    schema_version:
        格式版本（当前 :data:`SESSION_SCHEMA_VERSION`）。
    created_at:
        会话创建时间（ISO-8601 UTC 字符串）。
    meta:
        元信息（键集合见 :data:`META_KEYS`；模型 / 场景 / 任务指纹）。
    recording:
        :class:`~rm75_drake.recording.Recording`（关节流 + 夹爪调度）。
    run_log:
        :class:`~rm75_drake.device.run_log.RunLog`（设备层消息流）。
    checksum:
        ``created_at + meta + recording + run_log`` 的规范化 JSON 的 sha256。
    """

    schema_version: int = SESSION_SCHEMA_VERSION
    created_at: str = ""
    meta: Mapping[str, Any] = field(default_factory=dict)
    recording: Optional[Recording] = None
    run_log: Optional[RunLog] = None
    checksum: str = ""

    # -- 序列化 ----------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 可序列化的字典（顶层键顺序固定）。"""
        if self.recording is None:
            raise SessionError("session.recording 未设置（不能序列化为 JSON）")
        if self.run_log is None:
            raise SessionError("session.run_log 未设置（不能序列化为 JSON）")
        return {
            "schema_version": int(self.schema_version),
            "created_at": str(self.created_at),
            "meta": {key: self.meta.get(key) for key in META_KEYS},
            "recording": self.recording.to_dict(),
            "run_log": self.run_log.to_dict(),
            "checksum": str(self.checksum),
        }

    @classmethod
    def from_dict(cls, data: Any, *, verify_checksum: bool = True) -> "Session":
        """从字典构造（结构非法 → :class:`SessionError`，点名违规键）。"""
        mapping = _require_mapping(data, "session")
        unknown = sorted(str(key) for key in mapping if key not in TOP_LEVEL_KEYS)
        if unknown:
            raise SessionError(
                f"session 出现未知顶层键：{unknown}；允许的键只有 {list(TOP_LEVEL_KEYS)}"
            )
        missing = [key for key in TOP_LEVEL_KEYS if key not in mapping]
        if missing:
            raise SessionError(
                f"session 缺少必填顶层键：{missing}；允许的键只有 {list(TOP_LEVEL_KEYS)}"
            )
        version = mapping["schema_version"]
        if not isinstance(version, int) or isinstance(version, bool):
            raise SessionError(
                f"session.schema_version 必须是整数，实际：{version!r}"
            )
        if version != SESSION_SCHEMA_VERSION:
            raise SessionError(
                f"session.schema_version={version}，本实现只支持 {SESSION_SCHEMA_VERSION}"
            )
        created_at = mapping["created_at"]
        if not isinstance(created_at, str):
            raise SessionError(
                f"session.created_at 必须是字符串，实际：{created_at!r}"
            )
        meta = _check_meta(_require_mapping(mapping["meta"], "session.meta"))

        # -- recording 段（只读消费 S02 的 schema）------------------------
        try:
            recording = Recording.from_dict(mapping["recording"])
        except RecordingError as exc:
            raise SessionError(f"session.recording 段非法：{exc}") from exc

        # -- run_log 段（只读消费 S03 的 schema）--------------------------
        try:
            run_log = RunLog.from_dict(mapping["run_log"], verify_checksum=False)
        except RunLogError as exc:
            raise SessionError(f"session.run_log 段非法：{exc}") from exc
        expected_run_log = compute_run_log_checksum(run_log)
        if expected_run_log != run_log.checksum:
            raise SessionError(
                "session.run_log 段 checksum 不匹配（疑似篡改）："
                f"记录值 {run_log.checksum!r}，重算值 {expected_run_log!r}"
            )

        checksum = mapping["checksum"]
        if not isinstance(checksum, str):
            raise SessionError(
                f"session.checksum 必须是字符串，实际：{checksum!r}"
            )
        session = cls(
            schema_version=version,
            created_at=created_at,
            meta=meta,
            recording=recording,
            run_log=run_log,
            checksum=checksum,
        )
        if verify_checksum:
            expected = compute_session_checksum(
                session.created_at, session.meta, session.recording, session.run_log
            )
            if expected != session.checksum:
                digests = session_segment_digests(session)
                raise SessionError(
                    "session checksum 不匹配（疑似篡改）："
                    f"记录值 {session.checksum!r}，重算值 {expected!r}；"
                    "分段定位：run_log 段自身 checksum 通过（篡改不在 run_log 段），"
                    "recording / meta / created_at 段结构合法"
                    " ⇒ 候选段：recording / meta / created_at；"
                    f"各段当前摘要：{digests}"
                )
        return session


# ── 构造 / 读写 ─────────────────────────────────────────────────────────
def build_session(
    recording: Recording,
    run_log: RunLog,
    *,
    meta: Optional[Mapping[str, Any]] = None,
    created_at: Optional[str] = None,
) -> Session:
    """把录制 + 运行日志合并成**一份自洽的会话产物**。

    参数
    ----
    recording:
        :class:`~rm75_drake.recording.Recording`（**只读消费**）。
    run_log:
        :class:`~rm75_drake.device.run_log.RunLog`（**只读消费**）。
    meta:
        会话元信息（键白名单 :data:`META_KEYS`；未知键 → :class:`SessionError`）。
        **未显式给出的键**会从 ``recording.meta`` 里回填
        （``urdf_sha256`` / ``scenario_path``），
        ``robot_name`` 回填自 ``recording.robot_name``。
    created_at:
        会话时间戳（默认当前 UTC 时间）。

    失败模式
    --------
    * 类型不对 → :class:`SessionError`（点名 ``recording`` / ``run_log``）；
    * ``meta`` 含未知键 → :class:`SessionError`（点名该键）。
    """
    if not isinstance(recording, Recording):
        raise SessionError(
            f"recording 必须是 Recording 实例，实际：{type(recording).__name__}"
        )
    if not isinstance(run_log, RunLog):
        raise SessionError(
            f"run_log 必须是 RunLog 实例，实际：{type(run_log).__name__}"
        )
    # 白名单内的键**全部出现**（缺失者填 null），保证序列化前后 meta 形状一致。
    merged: Dict[str, Any] = {key: None for key in META_KEYS}
    merged.update(
        {
            "robot_name": recording.robot_name,
            "urdf_sha256": recording.meta.get("urdf_sha256"),
            "scenario_path": recording.meta.get("scenario_path"),
        }
    )
    if meta:
        unknown = sorted(str(key) for key in meta if key not in META_KEYS)
        if unknown:
            raise SessionError(
                f"session.meta 出现未知键：{unknown}；允许的键只有 {list(META_KEYS)}"
            )
        merged.update(meta)
    clean_meta = _check_meta(merged)
    stamp = created_at or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    checksum = compute_session_checksum(stamp, clean_meta, recording, run_log)
    return Session(
        schema_version=SESSION_SCHEMA_VERSION,
        created_at=stamp,
        meta=clean_meta,
        recording=recording,
        run_log=run_log,
        checksum=checksum,
    )


def write_session(session: Session, path: Any) -> Path:
    """把会话产物写到 JSON 文件（UTF-8、缩进 2、键序固定）。"""
    if not isinstance(session, Session):
        raise SessionError(
            f"session 必须是 Session 实例，实际：{type(session).__name__}"
        )
    target = Path(path)
    if target.parent and not target.parent.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(session.to_dict(), ensure_ascii=False, indent=2)
    target.write_text(text + "\n", encoding="utf-8")
    return target


def load_session(path: Any, *, verify_checksum: bool = True) -> Session:
    """读回会话产物并**严格校验**（含顶层校验和 ⇒ 篡改可检出）。

    失败模式
    --------
    * 文件不存在 → :class:`FileNotFoundError`；
    * JSON 语法错误 → :class:`SessionError`；
    * 结构 / 字段非法 → :class:`SessionError`（消息指出具体的键）；
    * 校验和不匹配 → :class:`SessionError`（疑似篡改）。
    """
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise SessionError(f"session 文件 JSON 解析失败：{exc}") from exc
    return Session.from_dict(data, verify_checksum=verify_checksum)


# ── 既有 schema 的「禁改」契约 ───────────────────────────────────────────
def source_schema_versions() -> Dict[str, int]:
    """返回被消费的既有 schema 版本（``recording`` / ``run_log``）。"""
    return {
        "recording": int(RECORDING_SCHEMA_VERSION),
        "run_log": int(RUN_LOG_SCHEMA_VERSION),
    }


def assert_source_schemas_unchanged() -> Dict[str, int]:
    """断言被消费的既有 schema 版本**未被改动**。

    失败模式
    --------
    * 版本漂移 → :class:`SessionError`（点名是哪一份 schema、期望值、实际值）。
    """
    expected = {"recording": 1, "run_log": 1}
    actual = source_schema_versions()
    drifted = {
        name: (expected[name], actual[name])
        for name in expected
        if actual[name] != expected[name]
    }
    if drifted:
        raise SessionError(
            "既有 schema 版本被改动（本切片只读消费，不得修改）："
            + "；".join(
                f"{name} 期望 v{want}，实际 v{got}" for name, (want, got) in drifted.items()
            )
        )
    return actual
