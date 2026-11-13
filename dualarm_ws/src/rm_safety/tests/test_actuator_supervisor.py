"""安全动作执行器与监督器的离线单测（T4）。

覆盖：

* 等级 → 动作映射（L1 set_speed_scale / L2 pause / L3 slow_stop / L4 stop）；
* 动作失败 → 留证据 + 升级到 L4（L4 失败 → FATAL，**不假装成功**）；
* **仅 L1 自动恢复**（回落到告警阈值内 → set_speed_scale(1.0) +
  ``safety_l1_slowdown_cleared``）；
* L2/L3/L4 **不自动恢复**（无 ``continue_`` / 无 ``restore_speed``）；
* 等级单调（不回落）；
* 安全层零反向依赖（不 import rm_executor / rm_hw_bridge / rmctl / tesseract*）。

假传输端口**在本文件内自建**（不 import 执行层，保持安全层边界）。
"""
from __future__ import annotations

import sys

import pytest

from rm_safety import levels as _levels
from rm_safety.actuator import (
    STATUS_ESCALATED,
    STATUS_FATAL,
    STATUS_OK,
    SafetyActuator,
)
from rm_safety.policy import SafetyThresholds
from rm_safety.supervisor import SafetySupervisor


# --------------------------------------------------------------- 假传输端口
class FakePort:
    """记录调用序列的假传输端口（鸭子类型，同 rm_executor.Transport 协议）。"""

    name = "fake"

    def __init__(self, fail_methods=(), raise_methods=()):
        self.calls = []
        self._fail = set(fail_methods)
        self._raise = set(raise_methods)

    # -- 记录 --------------------------------------------------------------
    def calls_of(self, method):
        return [c for c in self.calls if c["method"] == method]

    def methods(self):
        return [c["method"] for c in self.calls]

    def _record(self, method, **args):
        self.calls.append({"method": method, "args": args})
        if method in self._raise:
            raise RuntimeError("假端口 %s 抛异常" % method)
        if method in self._fail:
            return {
                "status": "error",
                "reason": "假端口 %s 脚本化失败" % method,
                "next_action": "恢复 %s 的下发条件后重跑" % method,
                "evidence": ["%s rc=-1" % method],
            }
        return {
            "status": "ok",
            "reason": "假端口 %s 成功" % method,
            "next_action": "",
            "evidence": ["fake:%s" % method],
        }

    # -- 端口协议 ----------------------------------------------------------
    def set_speed_scale(self, scale, joint_index=None):
        return self._record("set_speed_scale", scale=scale,
                            joint_index=joint_index)

    def pause(self):
        return self._record("pause")

    def slow_stop(self):
        return self._record("slow_stop")

    def stop(self):
        return self._record("stop")

    def continue_(self):
        return self._record("continue_")

    def clear_error_code(self):
        return self._record("clear_error_code")


@pytest.fixture()
def thresholds():
    """判据阈值（与 S03 冻结值一致；**不 import 执行层**）。"""
    return SafetyThresholds(
        follow_warn_deg=2.0,
        follow_stop_deg=5.0,
        stream_warn_ms=50.0,
        stream_stop_ms=200.0,
    )


# ------------------------------------------------------------- 执行器：映射
def test_l1_calls_set_speed_scale_only(thresholds):
    port = FakePort()
    actuator = SafetyActuator(port, l1_speed_scale=0.5)
    result = actuator.apply(_levels.LEVEL_L1)
    assert result.status == STATUS_OK
    assert result.transport_method == "set_speed_scale"
    assert result.sdk_symbol == "rm_set_joint_drive_max_speed"
    assert result.action == "set_speed_scale"
    assert port.methods() == ["set_speed_scale"]
    assert port.calls[0]["args"]["scale"] == 0.5
    # 未调用 pause / stop / slow_stop
    assert port.calls_of("pause") == []
    assert port.calls_of("stop") == []
    assert port.calls_of("slow_stop") == []


def test_l2_calls_pause_only(thresholds):
    port = FakePort()
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L2)
    assert result.status == STATUS_OK
    assert result.transport_method == "pause"
    assert result.sdk_symbol == "rm_set_arm_pause"
    assert port.methods() == ["pause"]
    assert port.calls_of("stop") == []


def test_l3_calls_slow_stop_only(thresholds):
    port = FakePort()
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L3)
    assert result.status == STATUS_OK
    assert result.transport_method == "slow_stop"
    assert result.sdk_symbol == "rm_set_arm_slow_stop"
    assert port.methods() == ["slow_stop"]
    assert port.calls_of("stop") == []


def test_l4_calls_stop_only(thresholds):
    port = FakePort()
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L4)
    assert result.status == STATUS_OK
    assert result.transport_method == "stop"
    assert result.sdk_symbol == "rm_set_arm_stop"
    assert port.methods() == ["stop"]


def test_unknown_level_raises():
    port = FakePort()
    actuator = SafetyActuator(port)
    with pytest.raises(ValueError):
        actuator.apply("L9")
    assert port.calls == []


def test_invalid_l1_speed_scale_rejected():
    port = FakePort()
    with pytest.raises(ValueError):
        SafetyActuator(port, l1_speed_scale=0.0)
    with pytest.raises(ValueError):
        SafetyActuator(port, l1_speed_scale=1.5)


# --------------------------------------------------------- 执行器：失败升级
def test_l2_failure_escalates_to_l4_with_evidence():
    port = FakePort(fail_methods=("pause",))
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L2)
    assert result.status == STATUS_ESCALATED
    assert result.escalated_to == "L4"
    assert result.effective_level == "L4"
    assert result.severity == _levels.SEVERITY_FATAL
    # 调用序列：pause（失败） → stop（升级）
    assert port.methods() == ["pause", "stop"]
    kinds = [e["kind"] for e in result.evidence]
    assert "action_failed" in kinds
    assert "escalated_to_l4" in kinds
    # 失败证据必须带上失败原因
    failed = [e for e in result.evidence if e["kind"] == "action_failed"][0]
    assert "未成功" in failed["reason"]


def test_l1_failure_escalates_to_l4():
    port = FakePort(fail_methods=("set_speed_scale",))
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L1)
    assert result.status == STATUS_ESCALATED
    assert result.escalated_to == "L4"
    assert port.methods() == ["set_speed_scale", "stop"]


def test_l4_failure_is_fatal_and_not_faked():
    port = FakePort(fail_methods=("stop",))
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L4)
    assert result.status == STATUS_FATAL
    assert result.escalated_to == ""
    assert not result.ok
    kinds = [e["kind"] for e in result.evidence]
    assert "action_failed" in kinds
    assert "l4_stop_failed" in kinds
    # 只调用了一次 stop（失败不再递归升级）
    assert port.methods() == ["stop"]


def test_transport_exception_is_escalated_not_swallowed():
    port = FakePort(raise_methods=("pause",))
    actuator = SafetyActuator(port)
    result = actuator.apply(_levels.LEVEL_L2)
    assert result.status == STATUS_ESCALATED
    assert result.escalated_to == "L4"
    assert port.methods() == ["pause", "stop"]
    failed = [e for e in result.evidence if e["kind"] == "action_failed"][0]
    assert "异常" in failed["reason"]


def test_missing_transport_method_is_failure():
    class Empty:
        pass

    actuator = SafetyActuator(Empty())
    result = actuator.apply(_levels.LEVEL_L2)
    # 端口既没有 pause 也没有 stop → 升级尝试也失败 → FATAL（不假装已停机）
    assert result.status == STATUS_FATAL
    assert result.escalated_to == "L4"
    kinds = [e["kind"] for e in result.evidence]
    assert "action_failed" in kinds
    assert "l4_stop_failed" in kinds


def test_action_log_bounded_but_count_accurate():
    port = FakePort()
    actuator = SafetyActuator(port, max_records=2)
    for _ in range(5):
        actuator.apply(_levels.LEVEL_L2)
    assert len(actuator.action_log) == 2
    assert actuator.action_count == 5


# --------------------------------------------------------------- 恢复语义
def test_restore_speed_sets_scale_to_one():
    port = FakePort()
    actuator = SafetyActuator(port)
    outcome = actuator.restore_speed()
    assert outcome["status"] == "ok"
    assert port.methods() == ["set_speed_scale"]
    assert port.calls[0]["args"]["scale"] == 1.0


def test_restore_speed_failure_leaves_evidence():
    port = FakePort(fail_methods=("set_speed_scale",))
    actuator = SafetyActuator(port)
    outcome = actuator.restore_speed()
    assert outcome["status"] == "failed"
    assert outcome["evidence"]
    # 恢复失败**不**升级（恢复失败不是更危险的方向）
    assert port.calls_of("stop") == []


# ---------------------------------------------------------------- 监督器
def test_supervisor_l1_downscales_only(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    decision = sup.evaluate({"follow_error_deg": 3.0})
    assert decision.level == "L1"
    assert sup.current_level == "L1"
    assert sup.speed_scaled is True
    assert port.methods() == ["set_speed_scale"]
    assert port.calls_of("pause") == []
    assert port.calls_of("stop") == []
    types = [e["type"] for e in sup.events]
    assert _levels.EVENT_L1_SLOWDOWN in types


def test_supervisor_l2_pauses(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    decision = sup.evaluate({"follow_error_deg": 6.0})
    assert decision.level == "L2"
    assert sup.current_level == "L2"
    assert port.methods() == ["pause"]


def test_supervisor_l3_slow_stop(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    decision = sup.evaluate({"arm_err": [90001], "error_code_map": _FakeCodeMap()})
    assert decision.level == "L3"
    assert sup.current_level == "L3"
    assert port.methods() == ["slow_stop"]


def test_supervisor_l4_stops(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    decision = sup.evaluate({"stream_age_ms": 250.0})
    assert decision.level == "L4"
    assert sup.current_level == "L4"
    assert port.methods() == ["stop"]


class _FakeCodeMap:
    """极小错误码映射（避免依赖仓库文件，保证单测自洽）。"""

    def known(self, code):
        return code == 90001

    def classify(self, code):
        return "collision" if code == 90001 else "unknown"


def test_supervisor_emits_events_to_callback(thresholds):
    port = FakePort()
    seen = []
    sup = SafetySupervisor(port, thresholds,
                           on_event=lambda *a: seen.append(a))
    sup.evaluate({"follow_error_deg": 6.0})
    assert seen
    event_type, severity, reason, evidence, next_action = seen[0]
    assert event_type == _levels.EVENT_L2_PAUSE
    assert severity == _levels.SEVERITY_NEEDS_HUMAN
    assert reason
    assert isinstance(evidence, list) and evidence
    assert next_action


def test_supervisor_l1_auto_recovery(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    sup.evaluate({"follow_error_deg": 3.0})
    assert sup.speed_scaled is True
    # 跟随误差回落到告警阈值内 → 自动恢复原速
    sup.evaluate({"follow_error_deg": 1.0})
    scales = [c["args"]["scale"] for c in port.calls_of("set_speed_scale")]
    assert scales == [0.5, 1.0]
    assert sup.speed_scaled is False
    types = [e["type"] for e in sup.events]
    assert _levels.EVENT_L1_SLOWDOWN_CLEARED in types


def test_supervisor_l1_retriggers_after_recovery(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    sup.evaluate({"follow_error_deg": 3.0})
    sup.evaluate({"follow_error_deg": 1.0})
    sup.evaluate({"follow_error_deg": 3.0})
    scales = [c["args"]["scale"] for c in port.calls_of("set_speed_scale")]
    assert scales == [0.5, 1.0, 0.5]
    assert sup.current_level == "L1"


def test_supervisor_l2_never_auto_recovers(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    sup.evaluate({"follow_error_deg": 6.0})
    port.calls.clear()
    # 信号回到正常范围（L2 仍为当前等级）
    sup.evaluate({"follow_error_deg": 0.5})
    assert port.calls == []
    assert sup.current_level == "L2"
    assert "continue_" not in port.methods()
    assert "clear_error_code" not in port.methods()


def test_supervisor_l3_does_not_downgrade_to_l2(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    sup.evaluate({"arm_err": [90001], "error_code_map": _FakeCodeMap()})
    assert sup.current_level == "L3"
    port.calls.clear()
    # 碰撞消失 → 回到 L2 判据，但等级单调，不降级、不动作
    decision = sup.evaluate({"follow_error_deg": 6.0})
    assert decision.level == "L2"
    assert sup.current_level == "L3"
    assert port.calls == []
    assert sup.peak_level == "L3"


def test_supervisor_action_failure_escalates_and_records_l4(thresholds):
    port = FakePort(fail_methods=("pause",))
    sup = SafetySupervisor(port, thresholds)
    sup.evaluate({"follow_error_deg": 6.0})
    assert sup.current_level == "L4"
    assert port.methods() == ["pause", "stop"]
    types = [e["type"] for e in sup.events]
    assert _levels.EVENT_L4_EMERGENCY_STOP in types
    assert any(e["severity"] == _levels.SEVERITY_FATAL for e in sup.events)


def test_supervisor_does_not_touch_event_log():
    """监督器**不 import** 执行层 / 硬件层 / 体验层 / 规划内核（AST 判定）。"""
    import ast
    import rm_safety.supervisor as supervisor_mod

    tree = ast.parse(open(supervisor_mod.__file__, encoding="utf-8").read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    for forbidden in ("rm_executor", "rm_hw_bridge", "rmctl", "tesseract"):
        assert forbidden not in imported, "监督器不得 import %s" % forbidden
    # 事件日志（执行层组件）也不得被引用
    assert not [name for name in dir(supervisor_mod) if "EventLog" in name]


def test_safety_layer_has_no_reverse_dependency():
    """import rm_safety 后不得出现执行层 / 体验层 / 规划内核模块。"""
    for name in ("rm_executor", "rm_hw_bridge", "rmctl"):
        assert name not in sys.modules, "安全层不得 import %s" % name
    assert not [m for m in sys.modules if m.startswith("tesseract")]


def test_supervisor_status_dict(thresholds):
    port = FakePort()
    sup = SafetySupervisor(port, thresholds)
    sup.evaluate({"follow_error_deg": 3.0})
    status = sup.to_dict()
    assert status["current_level"] == "L1"
    assert status["speed_scaled"] is True
    assert status["auto_recovery_allowed"] is True
    assert status["event_count"] == len(sup.events)
