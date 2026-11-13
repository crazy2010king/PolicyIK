"""录制 / 回放测试（S02-T08）。

被测对象
--------
:mod:`rm75_drake.recording` —— 录制格式（JSON）、严格校验、离线回放碰撞复检。

覆盖的契约
----------
**正例**
* 轨迹 → 录制文件 → 读回，字段逐项一致（schema_version / robot_name / dt /
  samples / gripper_schedule / meta）；
* 正常录制回放**逃逸数 = 0**（用单臂场景的碰撞检查器，S02-T05 同一口径）；
* 夹爪调度表能完整往返（kind / position / speed / force / hold_seconds）。

**负例（可解释失败）**
* 未知顶层键 / 未知 meta 键 → :class:`RecordingError`（消息指出键名）；
* 缺必填键 → :class:`RecordingError`；
* ``q`` 维度不是 7 / 含 NaN → :class:`RecordingError`；
* ``seq`` 不连续 / ``t_abs`` 不递增 → :class:`RecordingError`；
* 非法 JSON → :class:`RecordingError`；
* 空录制回放 → :class:`RecordingError`；
* **人为篡改**（插入一个碰撞构型）→ 回放**逃逸数 > 0**（不是崩溃，是可解释的计数）。
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from rm75_drake.recording import (
    RECORDING_SCHEMA_VERSION,
    Recording,
    RecordingError,
    load_recording,
    record_trajectory,
    replay_check,
)

#: 抬肘名义姿态（碰撞自由）。
NOMINAL_Q = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)

#: 人为构造的**碰撞构型**（本机实测：与工作台 box 相撞）。
COLLIDING_Q = (0.0, 1.2, 0.0, 1.5, 0.0, 0.0, 0.0)


def _tiny_trajectory():
    """构造一条极短的合法轨迹（3 点、5 ms 网格、速度/加速度都在限内）。"""
    from rm75_drake.planning.trajectory import trajectory_from_arrays

    rows = [
        list(NOMINAL_Q),
        [NOMINAL_Q[0] + 0.005] + list(NOMINAL_Q[1:]),
        [NOMINAL_Q[0] + 0.010] + list(NOMINAL_Q[1:]),
    ]
    return trajectory_from_arrays(rows, t0=0.0, dt=0.005, robot_name="rm75")


# ── 正例 ────────────────────────────────────────────────────────────────
def test_record_and_load_roundtrip(tmp_path):
    """正例：轨迹 → JSON → 读回，字段逐项一致。"""
    from rm75_drake.gripper import open_action

    action = open_action(speed=90, hold_seconds=0.25)
    trajectory = _tiny_trajectory()
    recording = record_trajectory(
        trajectory,
        tmp_path / "rec.json",
        scenario_path="scenarios/single_arm.yaml",
    )
    assert recording.schema_version == RECORDING_SCHEMA_VERSION
    assert recording.robot_name == "rm75"
    assert recording.dt == 0.005
    assert len(recording.samples) == len(trajectory.samples)
    assert recording.meta["scenario_path"] == "scenarios/single_arm.yaml"
    assert recording.meta["urdf_sha256"]

    loaded = load_recording(tmp_path / "rec.json")
    assert loaded.robot_name == recording.robot_name
    assert loaded.dt == recording.dt
    assert [s.q for s in loaded.samples] == [s.q for s in recording.samples]
    assert [s.seq for s in loaded.samples] == [s.seq for s in recording.samples]
    assert np.allclose(loaded.q_matrix(), recording.q_matrix())

    # 夹爪调度表往返（这里补一个带动作的录制）
    with_action = record_trajectory(
        _tiny_trajectory_with_action(action), tmp_path / "rec_action.json"
    )
    reloaded = load_recording(tmp_path / "rec_action.json")
    assert len(reloaded.gripper_schedule) == 1
    assert reloaded.gripper_schedule[0].kind == "open"
    assert reloaded.gripper_schedule[0].position == 1000
    assert reloaded.gripper_schedule[0].speed == 90
    assert reloaded.gripper_schedule[0].hold_seconds == 0.25


def _tiny_trajectory_with_action(action):
    """构造一条带夹爪动作的短轨迹。"""
    from rm75_drake.planning.trajectory import trajectory_from_arrays

    rows = [
        list(NOMINAL_Q),
        [NOMINAL_Q[0] + 0.005] + list(NOMINAL_Q[1:]),
    ]
    return trajectory_from_arrays(rows, t0=0.0, dt=0.005, gripper_schedule=[action])


def test_replay_of_normal_recording_has_zero_escapes(tmp_path, single_arm_scene):
    """正例：正常录制回放 → 逃逸数 = 0。"""
    recording = record_trajectory(
        _tiny_trajectory(),
        tmp_path / "rec.json",
        scenario_path="scenarios/single_arm.yaml",
    )
    result = replay_check(recording, single_arm_scene)
    assert result.num_escapes == 0
    assert result.ok is True
    assert result.num_samples > 0
    assert result.max_joint_step >= 0.0


def test_replay_detects_tampered_recording(tmp_path, single_arm_scene):
    """负例：人为插入碰撞构型 → 逃逸数 > 0（可解释的计数，不是崩溃）。"""
    recording = record_trajectory(_tiny_trajectory(), tmp_path / "rec.json")
    samples = list(recording.samples)
    tampered = Recording(
        schema_version=recording.schema_version,
        robot_name=recording.robot_name,
        dt=recording.dt,
        samples=(
            samples[0],
            type(samples[1])(
                seq=1,
                t_abs=samples[1].t_abs,
                q=COLLIDING_Q,
            ),
            samples[2],
        ),
        gripper_schedule=recording.gripper_schedule,
        created_at=recording.created_at,
        meta=recording.meta,
    )
    result = replay_check(tampered, single_arm_scene)
    assert result.num_escapes > 0
    assert result.ok is False
    assert result.escape_indices


# ── 负例（结构非法）──────────────────────────────────────────────────────
def _write_json(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return path


def _valid_payload():
    recording = record_trajectory(_tiny_trajectory())
    return recording.to_dict()


def test_load_recording_rejects_unknown_top_level_key(tmp_path):
    """负例：未知顶层键 → 报错并指出键名。"""
    payload = _valid_payload()
    payload["surprise"] = 1
    path = _write_json(tmp_path / "bad.json", payload)
    with pytest.raises(RecordingError) as excinfo:
        load_recording(path)
    assert "surprise" in str(excinfo.value)


def test_load_recording_rejects_missing_key(tmp_path):
    """负例：缺必填键 → 报错并指出键名。"""
    payload = _valid_payload()
    del payload["robot_name"]
    path = _write_json(tmp_path / "bad.json", payload)
    with pytest.raises(RecordingError) as excinfo:
        load_recording(path)
    assert "robot_name" in str(excinfo.value)


def test_load_recording_rejects_unknown_meta_key(tmp_path):
    """负例：未知 meta 键 → 报错（不静默丢弃）。"""
    payload = _valid_payload()
    payload["meta"]["extra"] = "x"
    path = _write_json(tmp_path / "bad.json", payload)
    with pytest.raises(RecordingError) as excinfo:
        load_recording(path)
    assert "extra" in str(excinfo.value)


def test_load_recording_rejects_bad_joint_dimension(tmp_path):
    """负例：q 维度不是 7 → 报错。"""
    payload = _valid_payload()
    payload["samples"][0]["q"] = [0.0, 0.1]
    path = _write_json(tmp_path / "bad.json", payload)
    with pytest.raises(RecordingError) as excinfo:
        load_recording(path)
    assert "7" in str(excinfo.value)


def test_load_recording_rejects_non_finite_joint(tmp_path):
    """负例：q 含 NaN → 报错。"""
    payload = _valid_payload()
    payload["samples"][0]["q"][2] = float("nan")
    path = _write_json(tmp_path / "bad.json", payload)
    with pytest.raises(RecordingError):
        load_recording(path)


def test_load_recording_rejects_broken_seq_and_time(tmp_path):
    """负例：seq 不连续 / t_abs 不递增 → 报错。"""
    payload = _valid_payload()
    payload["samples"][1]["seq"] = 7
    path = _write_json(tmp_path / "bad_seq.json", payload)
    with pytest.raises(RecordingError):
        load_recording(path)

    payload = _valid_payload()
    payload["samples"][1]["t_abs"] = payload["samples"][0]["t_abs"]
    path = _write_json(tmp_path / "bad_time.json", payload)
    with pytest.raises(RecordingError):
        load_recording(path)


def test_load_recording_rejects_malformed_json(tmp_path):
    """负例：非法 JSON → 报错（不是崩溃）。"""
    path = tmp_path / "broken.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(RecordingError):
        load_recording(path)


def test_record_trajectory_rejects_bad_input(tmp_path):
    """负例：轨迹类型不对 / meta 含未知键 → 报错。"""
    with pytest.raises(RecordingError):
        record_trajectory(object(), tmp_path / "x.json")
    with pytest.raises(RecordingError):
        record_trajectory(_tiny_trajectory(), tmp_path / "x.json", meta={"nope": 1})


def test_replay_check_rejects_empty_recording(single_arm_scene):
    """负例：空录制回放 → 报错。"""
    empty = Recording(schema_version=RECORDING_SCHEMA_VERSION, samples=())
    with pytest.raises(RecordingError):
        replay_check(empty, single_arm_scene)
