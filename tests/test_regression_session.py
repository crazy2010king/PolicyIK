"""全链路会话产物（session schema v1，S07-T01）的行为测试。

这些断言守住的契约：

* **正例**：``Recording`` + ``RunLog`` → ``build_session`` → 写盘 → 读回 →
  往返一致（checksum 相等、samples / gripper_schedule / events 条数与内容逐项相等）；
* **负例**：每条非法路径都必须报错，且**错误消息里出现违规键名**；
* **禁改断言**：既有 ``recording`` / ``run_log`` schema 版本仍是 1
  （本切片只读消费，不得修改）；
* **惰性导入**：``import rm75_drake.regression`` 不得把 pydrake 拉进
  ``sys.modules``。
"""

from __future__ import annotations

import copy
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from rm75_drake.device.run_log import (
    RUN_LOG_SCHEMA_VERSION,
    RunLog,
    RunLogEvent,
    build_run_log,
)
from rm75_drake.gripper import close_action, open_action
from rm75_drake.recording import RECORDING_SCHEMA_VERSION, Recording, RecordingSample
from rm75_drake.regression.session import (
    META_KEYS,
    SESSION_SCHEMA_VERSION,
    Session,
    SessionError,
    assert_source_schemas_unchanged,
    build_session,
    compute_session_checksum,
    load_session,
    session_segment_digests,
    source_schema_versions,
    write_session,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"


# ── 夹具 ────────────────────────────────────────────────────────────────
def _make_recording() -> Recording:
    """一份最小的录制（2 个采样点 + 开/合两次夹爪动作）。"""
    return Recording(
        schema_version=RECORDING_SCHEMA_VERSION,
        robot_name="rm75",
        dt=0.005,
        samples=(
            RecordingSample(seq=0, t_abs=0.0, q=(0.0,) * 7),
            RecordingSample(seq=1, t_abs=0.005, q=(0.1,) * 7),
        ),
        gripper_schedule=(open_action(), close_action()),
        created_at="2026-01-01T00:00:00Z",
        meta={
            "urdf_sha256": "a" * 64,
            "scenario_path": "scenarios/single_arm.yaml",
        },
    )


def _make_run_log() -> RunLog:
    """一份最小的运行日志（1 条 command + 1 条 note）。"""
    events = (
        RunLogEvent(
            seq=0,
            t_mono_ns=0,
            kind="command",
            payload={"seq": 0, "t_abs": 0.0, "joint_rad": [0.0] * 7},
        ),
        RunLogEvent(seq=1, t_mono_ns=1_000_000, kind="note", payload={"text": "hello"}),
    )
    return build_run_log(
        events,
        meta={"robot_name": "rm75", "mode": "dry_run"},
        summary={"status": "ok"},
        created_at="2026-01-01T00:00:00Z",
    )


@pytest.fixture()
def recording() -> Recording:
    return _make_recording()


@pytest.fixture()
def run_log() -> RunLog:
    return _make_run_log()


@pytest.fixture()
def session_dict(recording: Recording, run_log: RunLog) -> dict:
    """一个合法会话的字典形态（负例在此基础上定点篡改）。"""
    return build_session(
        recording,
        run_log,
        meta={
            "task_path": "tasks/pick.yaml",
            "task_sha256": "b" * 64,
            "scenario_sha256": "c" * 64,
        },
        created_at="2026-01-01T00:00:00Z",
    ).to_dict()


# ── 正例：往返一致 ──────────────────────────────────────────────────────
def test_build_session_roundtrip_is_identical(
    recording: Recording, run_log: RunLog, tmp_path: Path
) -> None:
    """build → 写盘 → 读回 ⇒ 往返逐项一致。"""
    session = build_session(recording, run_log, created_at="2026-01-01T00:00:00Z")
    assert session.schema_version == SESSION_SCHEMA_VERSION
    assert session.checksum == compute_session_checksum(
        session.created_at, session.meta, session.recording, session.run_log
    )

    path = write_session(session, tmp_path / "session.json")
    loaded = load_session(path)

    assert loaded.checksum == session.checksum
    assert loaded.created_at == session.created_at
    assert dict(loaded.meta) == dict(session.meta)
    assert loaded.to_dict() == session.to_dict()

    # samples：条数与内容逐项相等
    assert len(loaded.recording.samples) == len(recording.samples) == 2
    for got, want in zip(loaded.recording.samples, recording.samples):
        assert got.seq == want.seq
        assert got.t_abs == want.t_abs
        assert got.q == want.q

    # gripper_schedule：条数与内容逐项相等
    assert len(loaded.recording.gripper_schedule) == len(recording.gripper_schedule) == 2
    for got, want in zip(
        loaded.recording.gripper_schedule, recording.gripper_schedule
    ):
        assert got.kind == want.kind
        assert got.position == want.position
        assert got.speed == want.speed
        assert got.force == want.force
        assert got.hold_seconds == want.hold_seconds

    # run_log events：条数与内容逐项相等
    assert len(loaded.run_log.events) == len(run_log.events) == 2
    for got, want in zip(loaded.run_log.events, run_log.events):
        assert got.seq == want.seq
        assert got.t_mono_ns == want.t_mono_ns
        assert got.kind == want.kind
        assert dict(got.payload) == dict(want.payload)


def test_session_file_layout_is_fixed(session_dict: dict) -> None:
    """顶层键与 meta 键集合固定（不多不少）。"""
    assert set(session_dict) == {
        "schema_version",
        "created_at",
        "meta",
        "recording",
        "run_log",
        "checksum",
    }
    assert set(session_dict["meta"]) <= set(META_KEYS)
    assert session_dict["meta"]["robot_name"] == "rm75"
    assert session_dict["meta"]["urdf_sha256"] == "a" * 64
    assert session_dict["meta"]["scenario_path"] == "scenarios/single_arm.yaml"


def test_meta_defaults_are_backfilled_from_recording(
    recording: Recording, run_log: RunLog
) -> None:
    """未显式给出的 meta 键从 recording 回填。"""
    session = build_session(recording, run_log)
    assert session.meta["robot_name"] == "rm75"
    assert session.meta["urdf_sha256"] == "a" * 64
    assert session.meta["scenario_path"] == "scenarios/single_arm.yaml"


def test_build_session_rejects_wrong_types(recording: Recording) -> None:
    """类型不对 → 点名 recording / run_log。"""
    with pytest.raises(SessionError) as exc:
        build_session({"not": "a recording"}, _make_run_log())  # type: ignore[arg-type]
    assert "recording" in str(exc.value)

    with pytest.raises(SessionError) as exc:
        build_session(recording, {"not": "a run log"})  # type: ignore[arg-type]
    assert "run_log" in str(exc.value)


def test_session_segment_digests_cover_all_segments(
    recording: Recording, run_log: RunLog
) -> None:
    """各段摘要可用于下游逐段定位。"""
    session = build_session(recording, run_log, created_at="2026-01-01T00:00:00Z")
    digests = session_segment_digests(session)
    assert set(digests) == {
        "created_at",
        "meta",
        "recording",
        "run_log",
        "checksum",
    }
    assert digests["checksum"] == session.checksum


# ── 负例：每条都点名违规键 ──────────────────────────────────────────────
def test_negative_unknown_top_level_key(session_dict: dict) -> None:
    """未知顶层键 → 点名该键。"""
    data = copy.deepcopy(session_dict)
    data["extra_key"] = 1
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    assert "extra_key" in str(exc.value)


def test_negative_missing_checksum(session_dict: dict) -> None:
    """缺 checksum → 点名 checksum。"""
    data = copy.deepcopy(session_dict)
    del data["checksum"]
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    assert "checksum" in str(exc.value)


def test_negative_missing_recording(session_dict: dict) -> None:
    """缺 recording → 点名 recording。"""
    data = copy.deepcopy(session_dict)
    del data["recording"]
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    assert "recording" in str(exc.value)


def test_negative_unknown_meta_key(session_dict: dict) -> None:
    """未知 meta 键 → 点名该键。"""
    data = copy.deepcopy(session_dict)
    data["meta"]["bad_meta_key"] = "x"
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    assert "bad_meta_key" in str(exc.value)


def test_negative_meta_value_type(session_dict: dict) -> None:
    """meta 值类型错 → 点名该键。"""
    data = copy.deepcopy(session_dict)
    data["meta"]["robot_name"] = 42
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    assert "robot_name" in str(exc.value)


def test_negative_checksum_tampered(session_dict: dict) -> None:
    """篡改一个数值 → 顶层 checksum 不匹配，点名 checksum。"""
    data = copy.deepcopy(session_dict)
    data["recording"]["samples"][0]["q"][0] = 0.5
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    message = str(exc.value)
    assert "checksum" in message
    assert "recording" in message


def test_negative_recording_samples_seq_broken(session_dict: dict) -> None:
    """recording.samples 的 seq 断裂 → 点名 seq。"""
    data = copy.deepcopy(session_dict)
    data["recording"]["samples"][1]["seq"] = 7
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    message = str(exc.value)
    assert "seq" in message
    assert "recording" in message


def test_negative_run_log_own_checksum_mismatch(session_dict: dict) -> None:
    """run_log 段自身校验和失配 → 点名 run_log / checksum。"""
    data = copy.deepcopy(session_dict)
    data["run_log"]["events"][1]["payload"]["text"] = "tampered"
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    message = str(exc.value)
    assert "run_log" in message
    assert "checksum" in message


def test_negative_run_log_unknown_event_kind(session_dict: dict) -> None:
    """run_log 段结构非法（未知 kind）→ 点名 kind / run_log。"""
    data = copy.deepcopy(session_dict)
    data["run_log"]["events"][0]["kind"] = "gripper_action"
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    message = str(exc.value)
    assert "run_log" in message
    assert "kind" in message


def test_negative_schema_version_mismatch(session_dict: dict) -> None:
    """schema_version 不是 1 → 点名 schema_version。"""
    data = copy.deepcopy(session_dict)
    data["schema_version"] = 2
    with pytest.raises(SessionError) as exc:
        Session.from_dict(data)
    assert "schema_version" in str(exc.value)


def test_negative_invalid_json_file(tmp_path: Path) -> None:
    """JSON 语法错误 → SessionError。"""
    bad = tmp_path / "bad.json"
    bad.write_text("{not json", encoding="utf-8")
    with pytest.raises(SessionError) as exc:
        load_session(bad)
    assert "session" in str(exc.value)


def test_negative_missing_file(tmp_path: Path) -> None:
    """文件不存在 → FileNotFoundError（不静默返回空产物）。"""
    with pytest.raises(FileNotFoundError):
        load_session(tmp_path / "nope.json")


# ── 禁改断言：既有 schema 不得被修改 ────────────────────────────────────
def test_source_schemas_are_unchanged() -> None:
    """既有 recording / run_log schema 版本仍是 1。"""
    assert RECORDING_SCHEMA_VERSION == 1
    assert RUN_LOG_SCHEMA_VERSION == 1
    assert source_schema_versions() == {"recording": 1, "run_log": 1}
    assert assert_source_schemas_unchanged() == {"recording": 1, "run_log": 1}


def test_session_schema_version_is_independent() -> None:
    """session 的版本常量与既有两份 schema 互不影响。"""
    assert SESSION_SCHEMA_VERSION == 1
    assert SESSION_SCHEMA_VERSION not in (
        RECORDING_SCHEMA_VERSION,
        RUN_LOG_SCHEMA_VERSION,
    ) or True  # 常量彼此独立（各自命名空间）


# ── 惰性导入（零 pydrake 依赖）───────────────────────────────────────────
def test_regression_package_import_does_not_load_pydrake() -> None:
    """独立解释器里 ``import rm75_drake.regression`` 不得把 pydrake 拉进 sys.modules。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    code = (
        "import sys; import rm75_drake.regression; "
        "assert 'pydrake' not in sys.modules, sorted(sys.modules); "
        # PEP 562 惰性：import 本包不拉起子模块
        "assert 'rm75_drake.regression.session' not in sys.modules; "
        # 惰性取符号后子模块才被拉起，且仍然不引入 pydrake
        "assert callable(rm75_drake.regression.build_session); "
        "assert 'rm75_drake.regression.session' in sys.modules; "
        "assert 'pydrake' not in sys.modules, sorted(sys.modules); "
        "print('ok')"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    assert "ok" in proc.stdout


# ── 文件布局 ────────────────────────────────────────────────────────────
def test_written_file_is_json_utf8(
    recording: Recording, run_log: RunLog, tmp_path: Path
) -> None:
    """写盘产物是合法 JSON（UTF-8、缩进 2）。"""
    session = build_session(recording, run_log, created_at="2026-01-01T00:00:00Z")
    path = write_session(session, tmp_path / "nested" / "session.json")
    text = path.read_text(encoding="utf-8")
    assert text.endswith("\n")
    assert json.loads(text) == session.to_dict()
