"""rm_executor 执行契约测试（轨迹契约 / 执行配置 / 限值复检 / 事件模型 / 指标口径）。

覆盖正例与负例：
times 非递增、点数不匹配、joint_names 长度/顺序错、units 推断标记、units 不符、
verify 缺失被拒 / verify.ok=false 被拒、限值越界被拒、配置字段缺失被拒、
事件分级映射、指标分位数与空样本、以及「import rm_executor 不引入 rmctl /
tesseract*」的分层红线。
"""
from __future__ import annotations

import copy
import json
import os
import subprocess
import sys

import pytest

from rm_executor import contract, events, metrics


# ============================================================ 轨迹契约：正例
def test_load_trajectory_ok(base_payload, write_traj):
    path = write_traj(base_payload)
    res = contract.load_trajectory(path)
    assert res["status"] == "ok", res
    traj = res["trajectory"]
    assert traj["schema_version"] == contract.SCHEMA_VERSION
    assert traj["group"] == contract.JOINT_GROUP
    assert traj["joint_names"] == list(contract.JOINT_NAMES)
    assert traj["units"] == {"times": "s", "points": "rad"}
    assert len(traj["times"]) == len(base_payload["times"])
    assert len(traj["points"]) == len(base_payload["points"])
    assert all(isinstance(v, float) for v in traj["times"])
    assert res["schema_inferred"] is False


def test_load_trajectory_units_inferred_not_silent(base_payload, write_traj):
    """缺 units / schema_version → 按 rad 推定并**显式标记**（不静默）。"""
    payload = copy.deepcopy(base_payload)
    payload.pop("units")
    payload.pop("schema_version")
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "ok", res
    assert res["schema_inferred"] is True
    assert res["trajectory"]["units"] == {"times": "s", "points": "rad"}
    assert res["trajectory"]["schema_version"] == contract.SCHEMA_VERSION
    assert any("units 缺失" in e for e in res["evidence"])


def test_load_trajectory_verify_missing_blocked(base_payload, write_traj):
    """缺 verify → blocked + 可粘贴 next_action（离线假执行需显式放行）。"""
    payload = copy.deepcopy(base_payload)
    payload.pop("verify")
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert res["trajectory"] is None
    assert "verify" in res["reason"]
    assert "rmctl" in res["next_action"]


def test_load_trajectory_verify_missing_allowed_explicitly(base_payload, write_traj):
    """显式 allow_unverified（离线假执行）→ 放行，且写进 evidence。"""
    payload = copy.deepcopy(base_payload)
    payload.pop("verify")
    res = contract.load_trajectory(write_traj(payload), allow_unverified=True)
    assert res["status"] == "ok", res
    assert res["trajectory"]["verify"] is None
    assert any("allow_unverified" in e for e in res["evidence"])


def test_load_trajectory_verify_ok_false_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["verify"] = {"ok": False, "checks": {"limits": {"ok": False}}}
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "verify.ok=false" in res["reason"]
    assert "limits" in res["reason"]


def test_load_trajectory_verify_without_ok_key_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["verify"] = {"checks": {}}
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "ok" in res["reason"]


# ============================================================ 轨迹契约：负例
def test_load_trajectory_missing_file_blocked(tmp_path):
    res = contract.load_trajectory(str(tmp_path / "nope.json"))
    assert res["status"] == "blocked", res
    assert res["trajectory"] is None
    assert res["next_action"]


def test_load_trajectory_malformed_json_blocked(write_traj):
    res = contract.load_trajectory(write_traj("{not json"))
    assert res["status"] == "blocked", res
    assert "JSON" in res["reason"]


def test_load_trajectory_times_not_increasing_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["times"][3] = payload["times"][2]
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "严格递增" in res["reason"]


def test_load_trajectory_times_too_short_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["times"] = payload["times"][:1]
    payload["points"] = payload["points"][:1]
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "≥ 2" in res["reason"]


def test_load_trajectory_points_count_mismatch_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["points"] = payload["points"][:-1]
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "行数" in res["reason"]


def test_load_trajectory_row_width_wrong_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["points"][2] = payload["points"][2][:6]
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "7 个关节角" in res["reason"]


def test_load_trajectory_non_finite_value_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["points"][1][3] = float("nan")
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "有限数" in res["reason"]


def test_load_trajectory_joint_names_wrong_length_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["joint_names"] = payload["joint_names"][:6]
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "joint_names" in res["reason"]


def test_load_trajectory_joint_names_wrong_order_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    names = list(payload["joint_names"])
    names[0], names[1] = names[1], names[0]
    payload["joint_names"] = names
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "joint_names" in res["reason"]


def test_load_trajectory_units_wrong_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["units"] = {"times": "s", "points": "deg"}
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "单位不符" in res["reason"]


def test_load_trajectory_schema_version_unsupported_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["schema_version"] = 99
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "schema_version" in res["reason"]


def test_load_trajectory_group_wrong_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload["group"] = "other_arm"
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "group" in res["reason"]


def test_load_trajectory_missing_strict_field_blocked(base_payload, write_traj):
    payload = copy.deepcopy(base_payload)
    payload.pop("points")
    res = contract.load_trajectory(write_traj(payload))
    assert res["status"] == "blocked", res
    assert "points" in res["reason"]


# ============================================================ 执行配置
def test_load_execution_config_from_workcell(repo_root):
    res = contract.load_execution_config(repo_root)
    assert res["status"] == "ok", res
    cfg = res["config"]
    assert cfg["period_s"] == pytest.approx(0.010)
    assert cfg["follow"] is False
    assert cfg["trajectory_mode"] == 1
    assert cfg["radio"] == 30
    assert cfg["pre_position_speed"] == pytest.approx(10)


def test_load_execution_config_missing_field_blocked(fake_ws):
    cfg_path = fake_ws / "workcell" / "rm_workcell.yaml"
    cfg_path.write_text(
        "execution:\n  period_s: 0.010\n  follow: false\n", encoding="utf-8"
    )
    res = contract.load_execution_config(str(fake_ws))
    assert res["status"] == "blocked", res
    assert res["config"] is None
    assert "trajectory_mode" in res["reason"]
    assert "radio" in res["reason"]
    assert res["next_action"]


def test_load_execution_config_missing_section_blocked(fake_ws):
    cfg_path = fake_ws / "workcell" / "rm_workcell.yaml"
    cfg_path.write_text("robot:\n  description: rm_description\n", encoding="utf-8")
    res = contract.load_execution_config(str(fake_ws))
    assert res["status"] == "blocked", res
    assert "execution" in res["reason"]


def test_load_execution_config_missing_file_blocked(fake_ws):
    res = contract.load_execution_config(str(fake_ws))
    assert res["status"] == "blocked", res
    assert "找不到工位定义文件" in res["reason"]


def test_load_execution_config_bad_type_blocked(fake_ws):
    cfg_path = fake_ws / "workcell" / "rm_workcell.yaml"
    cfg_path.write_text(
        "execution:\n  period_s: 0.010\n  follow: false\n  trajectory_mode: 1\n"
        "  radio: 30\n  pre_position_speed: 0\n",
        encoding="utf-8",
    )
    res = contract.load_execution_config(str(fake_ws))
    assert res["status"] == "blocked", res
    assert "pre_position_speed" in res["reason"]


# ============================================================ 限值复检
def test_check_time_domain_limits_ok(smooth_trajectory, joint_limits):
    times, points = smooth_trajectory
    res = contract.check_time_domain_limits(times, points, limits=joint_limits)
    assert res["status"] == "ok", res
    assert res["violations"] == []


def test_check_time_domain_limits_position_violation_blocked(smooth_trajectory,
                                                            joint_limits):
    times, points = smooth_trajectory
    bad = copy.deepcopy(list(points))
    bad[3][0] = 10.0  # 远超 joint1 上限 3.106 rad
    res = contract.check_time_domain_limits(times, bad, limits=joint_limits)
    assert res["status"] == "blocked", res
    assert any("位置越界" in v for v in res["violations"])
    assert res["next_action"]


def test_check_time_domain_limits_velocity_violation_blocked(joint_limits):
    times = [0.0, 0.01, 0.02, 0.03]
    points = [[0.0] * 7, [2.0] * 7, [4.0] * 7, [6.0] * 7]  # 200 rad/s ≫ 3.14
    res = contract.check_time_domain_limits(times, points, limits=joint_limits)
    assert res["status"] == "blocked", res
    assert any("速度越界" in v for v in res["violations"])


def test_check_time_domain_limits_bad_shape_blocked(joint_limits):
    res = contract.check_time_domain_limits([0.0], [[0.0] * 7], limits=joint_limits)
    assert res["status"] == "blocked", res
    assert res["violations"] == []


def test_check_time_domain_limits_uses_hardware_layer_source(repo_root):
    """不限值注入时，限值来源必须是硬件层 rm_hw_bridge.limits。"""
    times = [0.0, 0.05, 0.1]
    points = [[0.0] * 7, [0.01] * 7, [0.0] * 7]
    res = contract.check_time_domain_limits(times, points, ws=repo_root)
    assert res["status"] == "ok", res
    assert any("rm_hw_bridge.limits" in e for e in res["evidence"])


# ============================================================ 阈值冻结
def test_thresholds_frozen():
    assert contract.PRE_POSITION_TOL_DEG == 0.5
    assert contract.PRE_POSITION_HOLD_MS == 200
    assert contract.FOLLOW_WARN_DEG == 2.0
    assert contract.FOLLOW_STOP_DEG == 5.0
    assert contract.STREAM_WARN_MS == 50
    assert contract.STREAM_STOP_MS == 200
    assert contract.ARRIVAL_TOL_DEG == 0.1
    assert contract.ARRIVAL_SPEED_DEG_S == 0.5
    assert contract.ARRIVAL_HOLD_MS == 300


# ============================================================ 事件模型
def test_event_severity_to_status_and_exit_code():
    assert events.SEVERITIES == (
        "INFO", "RECOVERABLE", "NEEDS_PARAM", "NEEDS_HUMAN", "FATAL",
    )
    assert events.status_for_severity("INFO") == "ok"
    assert events.exit_code_for_severity("INFO") == 0
    for sev in ("RECOVERABLE", "NEEDS_PARAM", "NEEDS_HUMAN"):
        assert events.status_for_severity(sev) == "blocked"
        assert events.exit_code_for_severity(sev) == 3
    assert events.status_for_severity("FATAL") == "error"
    assert events.exit_code_for_severity("FATAL") == 4


def test_event_severity_exit_codes_match_hw_bridge():
    """与 rm_hw_bridge.errors 的 status→退出码口径一致（0/3/4）。"""
    from rm_hw_bridge import errors as hw_errors

    assert contract.EXIT_CODES == hw_errors.EXIT_CODES
    for severity, status in events.SEVERITY_STATUS.items():
        assert events.SEVERITY_EXIT_CODES[severity] == hw_errors.EXIT_CODES[status]


def test_event_unknown_severity_raises():
    with pytest.raises(ValueError):
        events.Event(type="x", severity="CRITICAL")
    with pytest.raises(ValueError):
        events.severity_rank("CRITICAL")


def test_event_fields_and_to_dict():
    evt = events.Event(
        type="follow_error_warn", severity="RECOVERABLE", t=1.25,
        reason="跟随误差 2.4° 超过告警阈值 2.0°",
        evidence=["follow_err=2.400000 deg", "FOLLOW_WARN_DEG=2.000000"],
        next_action="降速后继续观察",
    )
    d = evt.to_dict()
    assert set(d) == {"type", "severity", "t", "reason", "evidence", "next_action"}
    assert d["type"] == "follow_error_warn"
    assert d["t"] == pytest.approx(1.25)
    assert len(d["evidence"]) == 2
    assert events.Event.from_dict(d).to_dict() == d


def test_event_log_filter_and_worst_status():
    log = events.EventLog()
    log.add("info_tick", "INFO", t=0.1, reason="启动")
    log.add("follow_error_warn", "RECOVERABLE", t=0.2, reason="跟随误差偏大")
    log.add("feedback_stream_lost", "FATAL", t=0.3, reason="反馈断流 250 ms")
    assert len(log) == 3
    assert log.filter(["FATAL"])[0].type == "feedback_stream_lost"
    assert len(log.at_least("NEEDS_PARAM")) == 1
    assert len(log.by_type("follow_error_warn")) == 1
    assert log.worst_severity() == "FATAL"
    assert log.status() == "error"
    assert log.exit_code() == 4
    ordered = log.to_list()
    assert [e["severity"] for e in ordered] == ["INFO", "RECOVERABLE", "FATAL"]


def test_event_log_empty_is_ok_not_fake():
    log = events.EventLog()
    assert log.worst_severity() is None
    assert log.status() == "ok"
    assert log.exit_code() == 0
    assert log.to_list() == []


def test_event_log_rejects_non_event():
    with pytest.raises(TypeError):
        events.EventLog().append({"type": "x"})


# ============================================================ 指标口径
def test_percentile_linear_interpolation():
    samples = [0.0, 1.0, 2.0, 3.0, 4.0]
    assert metrics.percentile(samples, 0) == pytest.approx(0.0)
    assert metrics.percentile(samples, 100) == pytest.approx(4.0)
    assert metrics.percentile(samples, 50) == pytest.approx(2.0)
    assert metrics.percentile(samples, 95) == pytest.approx(3.8)
    assert metrics.percentile([2.0], 99) == pytest.approx(2.0)


def test_percentile_empty_returns_none_not_zero():
    assert metrics.percentile([], 99) is None
    assert metrics.percentile([None, float("nan")], 99) is None


def test_jitter_stats():
    stats = metrics.jitter_stats([0.1, 0.2, 0.3, 0.4])
    assert stats["count"] == 4
    assert stats["p50_ms"] == pytest.approx(0.25)
    assert stats["max_ms"] == pytest.approx(0.4)
    assert stats["p99_ms"] is not None


def test_jitter_stats_empty_returns_none_not_zero():
    stats = metrics.jitter_stats([])
    assert stats["count"] == 0
    for key in ("p50_ms", "p95_ms", "p99_ms", "max_ms"):
        assert stats[key] is None, "空样本不得返回 0 冒充：%s" % key


def test_rt_factor():
    assert metrics.rt_factor(10.0, 10.5) == pytest.approx(1.05)
    assert metrics.rt_factor(0.0, 10.0) is None
    assert metrics.rt_factor(None, 10.0) is None
    assert metrics.rt_factor(10.0, None) is None


def test_follow_error_stats():
    stats = metrics.follow_error_stats([1.0, -1.0, 2.0])
    assert stats["count"] == 3
    assert stats["rms_deg"] == pytest.approx(((1.0 + 1.0 + 4.0) / 3.0) ** 0.5)
    assert stats["max_deg"] == pytest.approx(2.0)
    empty = metrics.follow_error_stats([])
    assert empty["rms_deg"] is None and empty["max_deg"] is None


def test_arrival_metrics():
    assert metrics.arrival_metrics(0.05, 320.0) == {
        "arrival_err_deg": pytest.approx(0.05),
        "arrival_hold_ms": pytest.approx(320.0),
    }
    assert metrics.arrival_metrics() == {
        "arrival_err_deg": None,
        "arrival_hold_ms": None,
    }


def test_duration_s():
    assert metrics.duration_s(1.0, 3.5) == pytest.approx(2.5)
    assert metrics.duration_s(3.5, 1.0) is None
    assert metrics.duration_s(None, 1.0) is None


# ============================================================ 分层红线
def test_import_rm_executor_does_not_pull_rmctl_or_tesseract(executor_pkg_root):
    """import rm_executor 后 sys.modules 不含 rmctl / tesseract*（子进程隔离断言）。"""
    script = (
        "import sys\n"
        "import rm_executor\n"
        "import rm_executor.contract, rm_executor.events, rm_executor.metrics\n"
        "import rm_executor.paths\n"
        "bad = sorted(m for m in sys.modules\n"
        "            if m == 'rmctl' or m.startswith('rmctl.')\n"
        "            or m == 'tesseract' or m.startswith('tesseract'))\n"
        "assert not bad, '执行层引入了禁用依赖: %s' % bad\n"
        "print('OK')\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = executor_pkg_root + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run(
        [sys.executable, "-c", script], env=env, capture_output=True, text=True,
    )
    assert proc.returncode == 0, proc.stderr
    assert "OK" in proc.stdout
