"""rm_safety 等级表 / 判据 / 配置 / 错误码表 的离线单测（**纯函数级**）。

覆盖
----
* 四级表**冻结值**（动作 / SDK 符号 / 严重度 / 退出码 / 恢复方式 / 事件名）；
* 八类故障 → 等级的**逐条**断言（外加「无故障」一条，共九条判据）；
* 多故障并发时的主因选择；
* 未知错误码的**保守 L3**；
* ``load_safety_config`` 的正负例（缺文件 blocked / 缺键用默认值并留痕）；
* ``load_error_codes`` 缺文件 / 格式错 → 保守默认且**不抛异常**；
* ``import rm_safety`` 后 ``sys.modules`` 里**没有** ``rmctl`` /
  ``rm_executor`` / ``tesseract*``（安全层不得反向依赖）。
"""
from __future__ import annotations

import os
import subprocess
import sys

import pytest

from rm_safety import levels as L
from rm_safety.config import load_safety_config
from rm_safety.error_codes import ErrorCodeMap, load_error_codes
from rm_safety.policy import (
    FAULT_KIND_COLLISION,
    FAULT_KIND_FOLLOW_ERROR_HEAVY,
    FAULT_KIND_FOLLOW_ERROR_LIGHT,
    FAULT_KIND_LIMIT_EXCEEDED,
    FAULT_KIND_PROTECTION_STOP,
    FAULT_KIND_STREAM_LAG_LIGHT,
    FAULT_KIND_STREAM_LOST,
    FAULT_KIND_UNKNOWN_ERROR_CODE,
    Decision,
    SafetyThresholds,
    can_auto_recover,
    evaluate,
)


# ---------------------------------------------------------------------------
# 一、四级表冻结值
# ---------------------------------------------------------------------------
def test_level_order_is_frozen():
    assert L.LEVEL_ORDER == ("L1", "L2", "L3", "L4")
    assert sorted(L.LEVELS.keys()) == ["L1", "L2", "L3", "L4"]


@pytest.mark.parametrize(
    "level,action,sdk,severity,exit_code,recovery,event",
    [
        ("L1", "set_speed_scale", "rm_set_joint_drive_max_speed",
         "RECOVERABLE", 3, "auto", "safety_l1_slowdown"),
        ("L2", "pause", "rm_set_arm_pause",
         "NEEDS_HUMAN", 3, "manual", "safety_l2_pause"),
        ("L3", "slow_stop", "rm_set_arm_slow_stop",
         "NEEDS_HUMAN", 3, "manual", "safety_l3_slow_stop"),
        ("L4", "stop", "rm_set_arm_stop",
         "FATAL", 4, "manual_reset", "safety_l4_emergency_stop"),
    ],
)
def test_level_table_frozen(level, action, sdk, severity, exit_code,
                            recovery, event):
    """四级表的每一格都是冻结值（改了就是契约变更）。"""
    assert L.level_action(level) == action
    assert L.level_sdk_symbol(level) == sdk
    assert L.level_severity(level) == severity
    assert L.level_exit_code(level) == exit_code
    assert L.level_recovery(level) == recovery
    assert L.level_event_type(level) == event
    assert L.level_rank(level) == int(level[1])


def test_event_name_constants_frozen():
    assert L.EVENT_L1_SLOWDOWN == "safety_l1_slowdown"
    assert L.EVENT_L1_SLOWDOWN_CLEARED == "safety_l1_slowdown_cleared"
    assert L.EVENT_L2_PAUSE == "safety_l2_pause"
    assert L.EVENT_L3_SLOW_STOP == "safety_l3_slow_stop"
    assert L.EVENT_L4_EMERGENCY_STOP == "safety_l4_emergency_stop"
    assert L.FAULT_FOLLOW_ERROR == "fault_follow_error"
    assert L.FAULT_LIMIT_EXCEEDED == "fault_limit_exceeded"
    assert L.FAULT_STREAM_LOST == "fault_stream_lost"
    assert L.FAULT_COLLISION == "fault_collision"
    assert L.FAULT_PROTECTION_STOP == "fault_protection_stop"


@pytest.mark.parametrize("accessor", [
    L.level_rank, L.level_action, L.level_sdk_symbol, L.level_severity,
    L.level_exit_code, L.level_recovery, L.level_event_type,
])
def test_unknown_level_raises_value_error(accessor):
    """未知等级必须抛 ValueError（不静默兜底）。"""
    with pytest.raises(ValueError):
        accessor("L9")


def test_unknown_level_type_raises_value_error():
    with pytest.raises(ValueError):
        L.level_rank(None)


# ---------------------------------------------------------------------------
# 二、故障类 → 等级（逐条钉死）
# ---------------------------------------------------------------------------
def test_no_fault_signals_yields_no_level(thresholds):
    decision = evaluate({}, thresholds)
    assert decision.level == ""
    assert decision.fault_kind == ""
    assert decision.exit_code == 0
    assert decision.detected == ()
    assert decision.next_action


def test_follow_error_light_is_l1(thresholds):
    decision = evaluate({"follow_error_deg": 3.0}, thresholds)
    assert decision.fault_kind == FAULT_KIND_FOLLOW_ERROR_LIGHT
    assert decision.level == "L1"
    assert decision.severity == "RECOVERABLE"
    assert decision.exit_code == 3
    assert decision.action == "set_speed_scale"
    assert decision.next_action


def test_follow_error_heavy_is_l2(thresholds):
    decision = evaluate({"follow_error_deg": 6.0}, thresholds)
    assert decision.fault_kind == FAULT_KIND_FOLLOW_ERROR_HEAVY
    assert decision.level == "L2"
    assert decision.severity == "NEEDS_HUMAN"
    assert decision.exit_code == 3
    assert decision.action == "pause"


def test_follow_error_light_disabled_escalates_to_l2(thresholds):
    """L1 降速被关闭 → 保守升到 L2（证据里必须写明原因）。"""
    off = SafetyThresholds(
        follow_warn_deg=thresholds.follow_warn_deg,
        follow_stop_deg=thresholds.follow_stop_deg,
        stream_warn_ms=thresholds.stream_warn_ms,
        stream_stop_ms=thresholds.stream_stop_ms,
        enable_l1_slowdown=False,
    )
    decision = evaluate({"follow_error_deg": 3.0}, off)
    assert decision.fault_kind == FAULT_KIND_FOLLOW_ERROR_LIGHT
    assert decision.level == "L2"
    notes = " ".join(str(item.get("note", "")) for item in decision.evidence)
    assert "enable_l1_slowdown=false" in notes


def test_limit_exceeded_is_l2(thresholds):
    decision = evaluate(
        {"limit_violations": [{"joint": "joint1", "margin": 0.2}]}, thresholds)
    assert decision.fault_kind == FAULT_KIND_LIMIT_EXCEEDED
    assert decision.level == "L2"
    assert decision.action == "pause"


def test_stream_lag_light_default_is_alert_only(thresholds):
    """默认口径：断流告警区间**只告警不动作**（保持 S03 行为）。"""
    decision = evaluate({"stream_age_ms": 100.0}, thresholds)
    assert decision.level == ""
    assert decision.fault_kind == ""
    assert decision.detected == ()
    assert any(item.get("fault_kind") == "advisory" for item in decision.evidence)


def test_stream_lag_light_is_l2_when_enabled():
    enabled = SafetyThresholds(
        follow_warn_deg=2.0, follow_stop_deg=5.0,
        stream_warn_ms=50.0, stream_stop_ms=200.0,
        enable_l2_pause_on_stream_lag=True,
    )
    decision = evaluate({"stream_age_ms": 100.0}, enabled)
    assert decision.fault_kind == FAULT_KIND_STREAM_LAG_LIGHT
    assert decision.level == "L2"
    assert decision.action == "pause"


def test_stream_lost_is_l4(thresholds):
    decision = evaluate({"stream_age_ms": 250.0}, thresholds)
    assert decision.fault_kind == FAULT_KIND_STREAM_LOST
    assert decision.level == "L4"
    assert decision.severity == "FATAL"
    assert decision.exit_code == 4
    assert decision.action == "stop"


def test_collision_is_l3(thresholds, error_code_map):
    code = 90001  # 表内 class=collision
    assert error_code_map.classify(code) == "collision"
    decision = evaluate({"arm_err": [code], "error_code_map": error_code_map},
                        thresholds)
    assert decision.fault_kind == FAULT_KIND_COLLISION
    assert decision.level == "L3"
    assert decision.action == "slow_stop"
    assert decision.exit_code == 3


def test_unknown_error_code_is_l3_conservative(thresholds, error_code_map):
    """表内没有的码 → 保守 L3（证据里写明「未知错误码」）。"""
    decision = evaluate({"arm_err": [424242], "error_code_map": error_code_map},
                        thresholds)
    assert decision.fault_kind == FAULT_KIND_UNKNOWN_ERROR_CODE
    assert decision.level == "L3"
    assert decision.action == "slow_stop"
    notes = " ".join(str(item.get("note", "")) for item in decision.evidence)
    assert "未知错误码" in notes


def test_error_code_without_map_is_l3_conservative(thresholds):
    """没有码表时，任何错误码都按未知码保守处理。"""
    decision = evaluate({"arm_err": [1]}, thresholds)
    assert decision.fault_kind == FAULT_KIND_UNKNOWN_ERROR_CODE
    assert decision.level == "L3"


def test_protection_stop_is_l4(thresholds):
    decision = evaluate({"arm_current_status": 9}, thresholds)
    assert decision.fault_kind == FAULT_KIND_PROTECTION_STOP
    assert decision.level == "L4"
    assert decision.exit_code == 4
    assert decision.action == "stop"


def test_self_triggered_stop_is_not_protection_stop(thresholds):
    """本进程发起的停机**不**判保护停（避免自我升级成 L4）。"""
    decision = evaluate(
        {"arm_current_status": 10, "self_triggered_stop": True}, thresholds)
    assert decision.level == ""
    assert decision.fault_kind == ""


# ---------------------------------------------------------------------------
# 三、多故障并发的主因选择
# ---------------------------------------------------------------------------
def test_primary_is_highest_level(thresholds):
    decision = evaluate(
        {"follow_error_deg": 3.0, "stream_age_ms": 250.0}, thresholds)
    assert decision.level == "L4"
    assert decision.fault_kind == FAULT_KIND_STREAM_LOST
    assert set(decision.detected) == {
        FAULT_KIND_FOLLOW_ERROR_LIGHT, FAULT_KIND_STREAM_LOST}


def test_primary_uses_kind_priority_within_same_level(thresholds):
    """同等级内按故障类优先级：保护停 > 断流。"""
    decision = evaluate(
        {"stream_age_ms": 250.0, "arm_current_status": 9}, thresholds)
    assert decision.level == "L4"
    assert decision.fault_kind == FAULT_KIND_PROTECTION_STOP


def test_primary_prefers_collision_over_limit(thresholds, error_code_map):
    decision = evaluate(
        {"arm_err": [90001], "error_code_map": error_code_map,
         "limit_violations": [{"joint": "joint1"}]},
        thresholds)
    assert decision.level == "L3"
    assert decision.fault_kind == FAULT_KIND_COLLISION


# ---------------------------------------------------------------------------
# 四、证据字段
# ---------------------------------------------------------------------------
def test_evidence_has_required_fields(thresholds):
    decision = evaluate({"follow_error_deg": 6.0}, thresholds)
    assert decision.evidence
    for item in decision.evidence:
        for key in ("fault_kind", "level", "threshold", "observed", "margin"):
            assert key in item
    primary = [i for i in decision.evidence
               if i["fault_kind"] == FAULT_KIND_FOLLOW_ERROR_HEAVY][0]
    assert primary["threshold"] == 5.0
    assert primary["observed"] == 6.0
    assert primary["margin"] == pytest.approx(1.0)


def test_next_action_is_pasteable_non_empty(thresholds):
    for signals in ({"follow_error_deg": 3.0}, {"follow_error_deg": 6.0},
                    {"stream_age_ms": 250.0}, {"arm_current_status": 9}):
        decision = evaluate(signals, thresholds)
        assert isinstance(decision.next_action, str)
        assert decision.next_action.strip()


def test_can_auto_recover_only_for_l1(thresholds):
    assert can_auto_recover("L1", 1.0, thresholds) is True
    assert can_auto_recover("L1", 3.0, thresholds) is False
    for level in ("L2", "L3", "L4"):
        assert can_auto_recover(level, 0.1, thresholds) is False


# ---------------------------------------------------------------------------
# 五、阈值自检（负例）
# ---------------------------------------------------------------------------
def test_thresholds_reject_inverted_warn_stop():
    with pytest.raises(ValueError):
        SafetyThresholds(follow_warn_deg=5.0, follow_stop_deg=2.0,
                         stream_warn_ms=50.0, stream_stop_ms=200.0)


def test_thresholds_reject_non_positive():
    with pytest.raises(ValueError):
        SafetyThresholds(follow_warn_deg=0.0, follow_stop_deg=5.0,
                         stream_warn_ms=50.0, stream_stop_ms=200.0)


def test_thresholds_reject_bad_speed_scale():
    with pytest.raises(ValueError):
        SafetyThresholds(follow_warn_deg=2.0, follow_stop_deg=5.0,
                         stream_warn_ms=50.0, stream_stop_ms=200.0,
                         l1_speed_scale=0.0)


def test_evaluate_rejects_none_thresholds():
    with pytest.raises(ValueError):
        evaluate({}, None)


def test_evaluate_rejects_non_mapping_signals(thresholds):
    with pytest.raises(TypeError):
        evaluate(["follow_error_deg"], thresholds)


def test_nan_follow_error_is_ignored(thresholds):
    decision = evaluate({"follow_error_deg": float("nan")}, thresholds)
    assert decision.level == ""


# ---------------------------------------------------------------------------
# 六、错误码表
# ---------------------------------------------------------------------------
def test_repo_error_codes_table_loads(error_code_map):
    assert len(error_code_map) > 0
    assert error_code_map.status == "unverified_offline"
    for code, item in error_code_map.codes.items():
        assert item["source"], "码 %d 缺 source（来源不可追溯）" % code
        assert item["class"]
        assert item["level_hint"]


def test_error_codes_unknown_code_is_l3(error_code_map):
    assert error_code_map.classify(999999) == "unknown"
    assert error_code_map.level_for(999999) == "L3"


def test_error_codes_missing_file_is_conservative_not_raising(tmp_path):
    table = load_error_codes(str(tmp_path / "nope.yaml"))
    assert len(table) == 0
    assert table.classify(1) == "unknown"
    assert table.level_for(1) == "L3"
    assert table.warnings and "不存在" in table.warnings[0]


def test_error_codes_malformed_file_is_conservative_not_raising(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("codes: [oops\n", encoding="utf-8")
    table = load_error_codes(str(bad))
    assert len(table) == 0
    assert table.level_for(1) == "L3"
    assert table.warnings


def test_error_codes_from_dict_missing_source_warns():
    table = ErrorCodeMap.from_dict(
        {"codes": [{"code": 1, "class": "collision", "level_hint": "L3"}]})
    assert table.classify(1) == "collision"
    assert any("source" in w for w in table.warnings)


def test_error_codes_from_dict_bad_shapes_do_not_raise():
    table = ErrorCodeMap.from_dict({"codes": "not-a-list"})
    assert len(table) == 0
    assert table.warnings


# ---------------------------------------------------------------------------
# 七、安全配置
# ---------------------------------------------------------------------------
def test_safety_config_from_repo_workcell(repo_root):
    result = load_safety_config(repo_root)
    assert result["status"] == "ok"
    config = result["config"]
    assert config["error_codes"] == "safety/error_codes.yaml"
    assert os.path.isfile(config["error_codes_path"])
    assert config["l1_speed_scale"] == pytest.approx(0.5)
    assert config["enable_l1_slowdown"] is True
    assert config["enable_l2_pause_on_stream_lag"] is False
    assert config["sdk_timeout_s"] == pytest.approx(1.0)


def test_safety_config_missing_file_is_blocked(fake_ws):
    result = load_safety_config(str(fake_ws))
    assert result["status"] == "blocked"
    assert result["next_action"].strip()
    assert result["config"] is None


def test_safety_config_missing_keys_use_defaults_and_leave_trace(fake_ws):
    (fake_ws / "workcell" / "rm_workcell.yaml").write_text(
        "version: 1\nsafety: {}\n", encoding="utf-8")
    result = load_safety_config(str(fake_ws))
    assert result["status"] == "ok"
    config = result["config"]
    assert config["l1_speed_scale"] == pytest.approx(0.5)
    assert config["enable_l1_slowdown"] is True
    assert config["enable_l2_pause_on_stream_lag"] is False
    assert config["sdk_timeout_s"] == pytest.approx(1.0)
    joined = " ".join(result["evidence"])
    assert "使用默认值" in joined


def test_safety_config_invalid_type_is_blocked(fake_ws):
    (fake_ws / "workcell" / "rm_workcell.yaml").write_text(
        "version: 1\nsafety:\n  l1_speed_scale: 7.5\n", encoding="utf-8")
    result = load_safety_config(str(fake_ws))
    assert result["status"] == "blocked"
    assert result["next_action"].strip()


# ---------------------------------------------------------------------------
# 八、层边界：import rm_safety 不得拉入上层 / 规划内核
# ---------------------------------------------------------------------------
def test_import_rm_safety_does_not_pull_upper_layers(safety_pkg_root):
    """``import rm_safety`` 后 sys.modules 里不得出现 rmctl / rm_executor /
    tesseract*（安全层不得反向依赖）。"""
    code = (
        "import sys, json\n"
        "import rm_safety\n"
        "bad = sorted(m for m in sys.modules\n"
        "             if m == 'rmctl' or m.startswith('rmctl.')\n"
        "             or m == 'rm_executor' or m.startswith('rm_executor.')\n"
        "             or m == 'rm_hw_bridge' or m.startswith('rm_hw_bridge.')\n"
        "             or m.startswith('tesseract'))\n"
        "print(json.dumps(bad))\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = safety_pkg_root
    proc = subprocess.run([sys.executable, "-c", code], env=env,
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip().endswith("[]"), proc.stdout


def test_decision_is_pure_dataclass():
    """Decision 是纯数据（frozen），可安全跨线程传递。"""
    decision = Decision(fault_kind="x", level="L1")
    with pytest.raises(Exception):
        decision.level = "L2"  # type: ignore[misc]
