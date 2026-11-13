"""全链路运行日志测试（S03-T07）：独立 schema + 严格校验 + 篡改检出。

被测对象
--------
:mod:`rm75_drake.device.run_log` —— 设备层**独立 schema** 的运行日志
（``schema_version=1``，**不修改** S02 的 :mod:`rm75_drake.recording`）。

覆盖的契约
----------
**正例**
* 写入 → 读回**逐字段一致**（顶层键、meta、summary、events 全等）；
* 六类事件（command / state_frame / safety_violation / state_transition /
  metric / note）都能构造并读回；
* 校验和自洽（:func:`verify_run_log` 为真）。

**负例（可解释失败）**
* 未知顶层键 → :class:`RunLogError`，消息**点名该键**；
* 未知事件类型 → :class:`RunLogError`，点名该类型；
* payload 含未知键 → :class:`RunLogError`，点名该键；
* 缺必填字段 → :class:`RunLogError`，点名字段；
* **篡改检出**：改掉日志里的一个数值 → 读回抛「校验和不匹配」；
* schema 版本不符 → :class:`RunLogError`。

**边界**
* :mod:`rm75_drake.recording` 的 ``schema_version`` **仍是 1**（S02 基线未被破坏）。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake.device import run_log as rl  # noqa: E402
from rm75_drake.device.run_log import (  # noqa: E402
    RUN_LOG_SCHEMA_VERSION,
    RunLog,
    RunLogBuilder,
    RunLogError,
    build_run_log,
    load_run_log,
    metric_payload,
    run_log_from_dict,
    verify_run_log,
    write_run_log,
)


def _sample_events() -> list:
    """构造覆盖全部事件类型的事件序列。"""
    builder = RunLogBuilder(start_ns=1000)
    builder.append(
        "state_transition",
        {
            "from_state": "idle",
            "to_state": "streaming",
            "event": "submit_trajectory",
            "reason": "",
        },
    )
    builder.append(
        "command",
        {"seq": 0, "t_abs": 0.0, "joint_rad": [0.0] * 7},
    )
    builder.append(
        "state_frame",
        {
            "seq": 0,
            "t_abs": 0.0,
            "age_s": 0.001,
            "joint_rad": [0.0] * 7,
            "currents_ma": [10.0] * 7,
            "enabled": [True] * 7,
            "joint_err_codes": [0] * 7,
            "controller_status": 5,
            "gripper_online": True,
            "complete": True,
        },
    )
    builder.append(
        "safety_violation",
        {
            "reason": "soft_limit",
            "gate": "G1",
            "action": "clamp",
            "severity": "warn",
            "detail": "joint_1 请求 1.9 rad 超出限位",
            "joints": [0],
        },
    )
    builder.append(
        "metric",
        {
            "name": "jitter",
            "count": 3,
            "p50": 0.0001,
            "p99": 0.0003,
            "p999": 0.0004,
            "max": 0.0005,
            "min": 0.0,
            "mean": 0.0002,
            "dropped": 0,
        },
    )
    builder.append("note", {"text": "离线 dry-run，未连真机"})
    return list(builder.events)


# ── 正例：往返一致 ──────────────────────────────────────────────────────
def test_roundtrip_is_field_by_field_identical(tmp_path):
    """正例：写入 → 读回逐字段一致。"""
    log = build_run_log(
        _sample_events(),
        meta={"command": "exec", "mode": "dry_run", "device_name": "rm75_single_arm"},
        summary={"status": "ok", "reason": "", "num_samples": 1},
        created_at="2026-01-01T00:00:00Z",
    )
    path = write_run_log(log, tmp_path / "run.json")
    loaded = load_run_log(path)

    assert loaded.to_dict() == log.to_dict()
    assert loaded.meta["mode"] == "dry_run"
    assert loaded.summary["num_samples"] == 1
    assert [event.kind for event in loaded.events] == [
        "state_transition",
        "command",
        "state_frame",
        "safety_violation",
        "metric",
        "note",
    ]
    assert loaded.events[3].payload["reason"] == "soft_limit"


def test_checksum_is_self_consistent(tmp_path):
    """正例：校验和自洽（verify_run_log 为真）。"""
    log = build_run_log(_sample_events(), created_at="2026-01-01T00:00:00Z")
    assert verify_run_log(log) is True
    path = write_run_log(log, tmp_path / "run.json")
    assert verify_run_log(load_run_log(path)) is True


def test_builder_assigns_sequential_seq():
    """正例：构造器自动分配连续的事件序号。"""
    builder = RunLogBuilder(start_ns=7)
    first = builder.append("note", {"text": "a"})
    second = builder.append("note", {"text": "b"})
    assert (first.seq, second.seq) == (0, 1)
    assert first.t_mono_ns == 7


def test_metric_payload_matches_histogram_summary():
    """正例：metric 载荷与直方图 summary 同口径（不另算一套分位数）。"""
    from rm75_drake.device.metrics import JitterHistogram

    histogram = JitterHistogram()
    for value in (0.001, 0.002, 0.003):
        histogram.add(value)
    payload = metric_payload("jitter", histogram)
    assert payload["count"] == 3
    assert payload["p50"] == histogram.p50
    assert payload["p99"] == histogram.p99
    assert payload["max"] == histogram.max


# ── 负例：未知键 / 缺字段 / 未知类型 ───────────────────────────────────
def test_unknown_top_level_key_names_the_key():
    """负例：未知顶层键 → 报错并点名该键。"""
    log = build_run_log(_sample_events())
    data = log.to_dict()
    data["extra_key"] = 1
    with pytest.raises(RunLogError) as excinfo:
        run_log_from_dict(data)
    assert "extra_key" in str(excinfo.value)


def test_unknown_event_kind_names_the_kind():
    """负例：未知事件类型 → 报错并点名该类型。"""
    log = build_run_log(_sample_events())
    data = log.to_dict()
    data["events"][0]["kind"] = "mystery_event"
    with pytest.raises(RunLogError) as excinfo:
        run_log_from_dict(data, verify_checksum=False)
    assert "mystery_event" in str(excinfo.value)


def test_unknown_payload_key_names_the_key():
    """负例：payload 含未知键 → 报错并点名该键。"""
    log = build_run_log(_sample_events())
    data = log.to_dict()
    data["events"][1]["payload"]["bogus_field"] = 1
    with pytest.raises(RunLogError) as excinfo:
        run_log_from_dict(data, verify_checksum=False)
    assert "bogus_field" in str(excinfo.value)


def test_missing_required_payload_key_names_the_field():
    """负例：缺必填字段 → 报错并点名字段。"""
    log = build_run_log(_sample_events())
    data = log.to_dict()
    del data["events"][1]["payload"]["t_abs"]
    with pytest.raises(RunLogError) as excinfo:
        run_log_from_dict(data, verify_checksum=False)
    assert "t_abs" in str(excinfo.value)


def test_builder_rejects_unknown_payload_key():
    """负例：构造器同样拒绝未知 payload 键（点名）。"""
    builder = RunLogBuilder()
    with pytest.raises(RunLogError) as excinfo:
        builder.append("note", {"text": "x", "extra": 1})
    assert "extra" in str(excinfo.value)


def test_builder_rejects_unknown_kind():
    """负例：构造器拒绝未知事件类型（点名）。"""
    builder = RunLogBuilder()
    with pytest.raises(RunLogError) as excinfo:
        builder.append("no_such_kind", {"text": "x"})
    assert "no_such_kind" in str(excinfo.value)


def test_unknown_meta_and_summary_keys_are_rejected():
    """负例：meta / summary 的未知键 → 报错并点名。"""
    with pytest.raises(RunLogError) as excinfo:
        build_run_log([], meta={"not_a_meta_key": 1})
    assert "not_a_meta_key" in str(excinfo.value)
    with pytest.raises(RunLogError) as excinfo:
        build_run_log([], summary={"not_a_summary_key": 1})
    assert "not_a_summary_key" in str(excinfo.value)


def test_wrong_schema_version_is_rejected():
    """负例：schema 版本不符 → 报错。"""
    log = build_run_log(_sample_events())
    data = log.to_dict()
    data["schema_version"] = RUN_LOG_SCHEMA_VERSION + 1
    with pytest.raises(RunLogError) as excinfo:
        run_log_from_dict(data, verify_checksum=False)
    assert "schema_version" in str(excinfo.value)


def test_broken_json_is_rejected(tmp_path):
    """负例：非法 JSON → 报错（点名文件）。"""
    path = tmp_path / "broken.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(RunLogError) as excinfo:
        load_run_log(path)
    assert "broken.json" in str(excinfo.value)


# ── 篡改检出 ────────────────────────────────────────────────────────────
def test_tampered_value_is_detected(tmp_path):
    """负例：篡改日志里的一个数值 → 读回时报「校验和不匹配」。"""
    log = build_run_log(_sample_events(), created_at="2026-01-01T00:00:00Z")
    path = write_run_log(log, tmp_path / "run.json")

    data = json.loads(path.read_text(encoding="utf-8"))
    data["events"][3]["payload"]["detail"] = "篡改过的描述"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    with pytest.raises(RunLogError) as excinfo:
        load_run_log(path)
    message = str(excinfo.value)
    assert "校验和" in message
    assert "篡改" in message


def test_tampered_event_count_is_detected(tmp_path):
    """负例：删除一个事件 → 校验和不匹配（事件序列不可静默截断）。"""
    log = build_run_log(_sample_events(), created_at="2026-01-01T00:00:00Z")
    path = write_run_log(log, tmp_path / "run.json")

    data = json.loads(path.read_text(encoding="utf-8"))
    data["events"].pop()
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    with pytest.raises(RunLogError):
        load_run_log(path)


def test_verify_run_log_detects_in_memory_tampering():
    """负例：内存里改了事件内容 → verify_run_log 为假。"""
    log = build_run_log(_sample_events())
    tampered = RunLog(
        schema_version=log.schema_version,
        created_at=log.created_at,
        meta=log.meta,
        events=log.events,
        summary=dict(log.summary),
        checksum=log.checksum,
    )
    object.__setattr__(
        tampered,
        "summary",
        dict(tampered.summary, status="tampered"),
    )
    assert verify_run_log(tampered) is False


# ── 边界：不得破坏 S02 的录制 schema ───────────────────────────────────
def test_recording_schema_version_untouched():
    """边界：S02 的 recording.py 仍是 schema_version=1（本任务不得修改它）。"""
    source = (SRC_DIR / "rm75_drake" / "recording.py").read_text(encoding="utf-8")
    assert "RECORDING_SCHEMA_VERSION = 1" in source
    assert "RECORDING_SCHEMA_VERSION = 2" not in source


def test_run_log_schema_version_is_independent():
    """边界：运行日志有自己的 schema 版本常量（与录制文件互不影响）。"""
    assert rl.RUN_LOG_SCHEMA_VERSION == 1
    assert rl.TOP_LEVEL_KEYS != rl.PAYLOAD_KEYS


# ── 负载：事件数上限（超限计数、不静默丢弃）────────────────────────────
def test_builder_caps_events_and_counts_drops():
    """负载：达到事件上限后停止追加并**计数**（不静默丢弃）。"""
    builder = RunLogBuilder(max_events=3)
    for index in range(6):
        builder.append("note", {"text": f"event-{index}"})
    assert len(builder.events) == 3
    assert builder.dropped == 3
    log = builder.build()
    assert log.summary["dropped_events"] == 3
    assert log.summary["num_events"] == 3
    assert verify_run_log(log) is True


def test_builder_rejects_invalid_max_events():
    """负例：非法的事件上限 → 显式报错。"""
    with pytest.raises(ValueError):
        RunLogBuilder(max_events=0)
    with pytest.raises(ValueError):
        RunLogBuilder(max_events="many")  # type: ignore[arg-type]
