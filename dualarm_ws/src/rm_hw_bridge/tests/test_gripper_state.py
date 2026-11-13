"""夹爪纯域逻辑单测（零 SDK、零 I/O）。

覆盖：
* 六类判定各 ≥10 组参数化夹具（grasp_ok / grasp_empty / stall / overtemp / overcurrent / offline）；
* 恢复阶梯（夹空重抓与越限升级、堵转重试上限、过温不重试、过流供电提示、离线两种形态）；
* 错误位解码（单 bit / 多 bit / bit5–bit7 保留位）；
* NEEDS_PARAM 负例（阈值或换算系数缺失 → 绝不产出 grasp_ok / grasp_empty）；
* MotionGate 三态迁移与 check() 返回结构；
* **零 SDK 负例**：模块不得 import ctypes / rm_hw_bridge.generated / rm_hw_bridge.loader。
"""
from __future__ import annotations

import ast
import os

import pytest

from rm_hw_bridge import gripper_state as gs

# --------------------------------------------------------------------------
# 公共夹具
# --------------------------------------------------------------------------

#: 判定阈值：力度阈值（0–1000 标度）、夹持窗口（mm）、两个换算系数
BASE_THRESHOLDS = {
    "force_threshold": 300.0,
    "grasp_position_window": (10.0, 40.0),
    "position_mm_per_unit": 0.065,  # 0–1000 源单位 ↔ 0–65 mm
    "force_scale": 0.13,  # N / 源力度单位（满标度 1000 → 130 N）
}


def make_state(raw: dict, path: str = "modbus") -> gs.GripperState:
    """构造归一化快照（**不**给换算系数 → 位置/牛顿由判定阶段按阈值换算）。"""
    return gs.normalize_state(path, raw)


def base_raw(**overrides) -> dict:
    raw = {
        "mode": 2,  # 闭合到最小且空闲
        "position_raw": 500,
        "force_raw": 500,
        "temp_c": 32,
        "current_mA": 210,
        "error": 0,
        "online": True,
        "protocol_state": "online",
    }
    raw.update(overrides)
    return raw


def thresholds(**overrides) -> dict:
    th = dict(BASE_THRESHOLDS)
    th.update(overrides)
    return th


# --------------------------------------------------------------------------
# 1. 错误位解码
# --------------------------------------------------------------------------

def test_error_bit_constants_match_manual():
    assert gs.ERROR_BIT_STALL == 1 << 0
    assert gs.ERROR_BIT_OVERTEMP == 1 << 1
    assert gs.ERROR_BIT_OVERCURRENT == 1 << 2
    assert gs.ERROR_BIT_DRIVER == 1 << 3
    assert gs.ERROR_BIT_COMM == 1 << 4


@pytest.mark.parametrize(
    "error,expected",
    [
        (0, {"stall": False, "overtemp": False, "overcurrent": False, "driver": False, "comm": False}),
        (1, {"stall": True, "overtemp": False, "overcurrent": False, "driver": False, "comm": False}),
        (2, {"stall": False, "overtemp": True, "overcurrent": False, "driver": False, "comm": False}),
        (4, {"stall": False, "overtemp": False, "overcurrent": True, "driver": False, "comm": False}),
        (8, {"stall": False, "overtemp": False, "overcurrent": False, "driver": True, "comm": False}),
        (16, {"stall": False, "overtemp": False, "overcurrent": False, "driver": False, "comm": True}),
    ],
)
def test_decode_error_bits_single(error, expected):
    assert gs.decode_error_bits(error) == expected


def test_decode_error_bits_multiple():
    bits = gs.decode_error_bits(0b0000_0111)  # 堵转 + 过温 + 过流
    assert bits["stall"] and bits["overtemp"] and bits["overcurrent"]
    assert not bits["driver"] and not bits["comm"]


@pytest.mark.parametrize("error", [0b0010_0000, 0b0100_0000, 0b1000_0000, 0b1110_0000])
def test_decode_error_bits_reserved_bits_ignored_without_error(error):
    """bit5–bit7 是厂商保留位：忽略，但不得报错、不得产生额外键。"""
    bits = gs.decode_error_bits(error)
    assert set(bits) == {"stall", "overtemp", "overcurrent", "driver", "comm"}
    assert not any(bits.values())


def test_decode_error_bits_rejects_non_int():
    with pytest.raises(TypeError):
        gs.decode_error_bits("1")
    with pytest.raises(TypeError):
        gs.decode_error_bits(True)


def test_error_bits_text_ignores_reserved():
    assert "堵转" in gs.error_bits_text(1)
    assert gs.error_bits_text(0b1110_0000) == "无"


# --------------------------------------------------------------------------
# 2. 模式映射（两条路径不等价）
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "code,expected",
    [(1, "open_max"), (2, "closed_min"), (3, "stopped"), (4, "closing"), (5, "opening")],
)
def test_manual_mode_map(code, expected):
    assert gs.mode_from_manual_code(code) == expected


def test_sdk_mode_has_force_stop_but_manual_does_not():
    assert gs.mode_from_sdk_code(6) == "force_stop"
    assert gs.mode_from_manual_code(6) is None


def test_mode_for_path_uses_path_specific_table():
    assert gs.mode_for_path("gripper_api", 6) == "force_stop"
    assert gs.mode_for_path("modbus", 6) is None
    assert gs.mode_for_path("rm_plus", 3) == "stopped"


def test_mode_for_path_rejects_unknown_path():
    with pytest.raises(ValueError):
        gs.mode_for_path("can_open", 1)


# --------------------------------------------------------------------------
# 3. 归一化
# --------------------------------------------------------------------------

def test_normalize_state_modbus_force_norm_is_0_1000_scale():
    state = make_state(base_raw(force_raw=640))
    assert state.force_norm == 640.0
    assert state.position_mm is None  # 换算系数缺失 → None（不猜、不填 0）
    assert state.force_N is None
    assert state.mode == "closed_min"
    assert state.raw["force_raw"] == 640  # 原始值留痕


def test_normalize_state_with_coefficients_fills_derived_fields():
    state = gs.normalize_state(
        "modbus", base_raw(force_raw=1000, position_raw=1000), position_mm_per_unit=0.065,
        force_scale=0.13,
    )
    assert state.position_mm == pytest.approx(65.0)
    assert state.force_N == pytest.approx(130.0)


def test_normalize_state_gripper_api_force_norm_requires_adapter_folding():
    """SDK 路径力度单位是 g：标度不明时 force_norm 必须为 None（纯逻辑不猜标度）。"""
    state = make_state(base_raw(force_raw=1500), path="gripper_api")
    assert state.force_norm is None
    folded = make_state(base_raw(force_raw=1500, force_norm=700), path="gripper_api")
    assert folded.force_norm == 700.0


def test_normalize_state_rejects_bad_inputs():
    with pytest.raises(ValueError):
        gs.normalize_state("nope", {})
    with pytest.raises(ValueError):
        gs.normalize_state("modbus", {"protocol_state": "weird"})
    with pytest.raises(TypeError):
        gs.normalize_state("modbus", {"error": "0x01"})


# --------------------------------------------------------------------------
# 4. 六类判定（每类 ≥10 组夹具）
# --------------------------------------------------------------------------

GRASP_OK_CASES = [
    ("ok-force-300-pos-154", {"force_raw": 300, "position_raw": 154}),
    ("ok-force-320-pos-200", {"force_raw": 320, "position_raw": 200}),
    ("ok-force-350-pos-250", {"force_raw": 350, "position_raw": 250}),
    ("ok-force-400-pos-300", {"force_raw": 400, "position_raw": 300}),
    ("ok-force-450-pos-350", {"force_raw": 450, "position_raw": 350}),
    ("ok-force-500-pos-400", {"force_raw": 500, "position_raw": 400}),
    ("ok-force-550-pos-450", {"force_raw": 550, "position_raw": 450}),
    ("ok-force-600-pos-500", {"force_raw": 600, "position_raw": 500}),
    ("ok-force-700-pos-550", {"force_raw": 700, "position_raw": 550}),
    ("ok-force-800-pos-600", {"force_raw": 800, "position_raw": 600}),
    ("ok-force-1000-pos-615", {"force_raw": 1000, "position_raw": 615}),
    ("ok-mode-stopped", {"force_raw": 600, "position_raw": 400, "mode": 3}),
]


@pytest.mark.parametrize("case_id,raw", GRASP_OK_CASES, ids=[c[0] for c in GRASP_OK_CASES])
def test_judge_grasp_ok(case_id, raw):
    judgment = gs.judge(make_state(base_raw(**raw)), thresholds())
    assert judgment.result == gs.RESULT_GRASP_OK, judgment.reason
    assert judgment.severity == gs.SEV_OK
    assert judgment.retryable is False
    assert judgment.next_action


@pytest.mark.parametrize("case_id,raw", GRASP_OK_CASES, ids=[c[0] for c in GRASP_OK_CASES])
def test_judge_grasp_ok_via_gripper_api_path(case_id, raw):
    """官方夹爪 API 路径（mode=6 力控停止）同样能判夹到。"""
    api_raw = base_raw(**raw)
    api_raw["force_norm"] = api_raw["force_raw"]
    api_raw["mode"] = 6
    state = make_state(api_raw, path="gripper_api")
    assert state.mode == "force_stop"
    judgment = gs.judge(state, thresholds())
    assert judgment.result == gs.RESULT_GRASP_OK, judgment.reason
    assert judgment.severity == gs.SEV_OK


GRASP_EMPTY_CASES = [
    ("empty-force-0", {"force_raw": 0, "position_raw": 0}),
    ("empty-force-10", {"force_raw": 10, "position_raw": 50}),
    ("empty-force-50", {"force_raw": 50, "position_raw": 100}),
    ("empty-force-100", {"force_raw": 100, "position_raw": 150}),
    ("empty-force-150", {"force_raw": 150, "position_raw": 200}),
    ("empty-force-200", {"force_raw": 200, "position_raw": 250}),
    ("empty-force-250", {"force_raw": 250, "position_raw": 300}),
    ("empty-force-280", {"force_raw": 280, "position_raw": 350}),
    ("empty-force-299", {"force_raw": 299, "position_raw": 400}),
    ("empty-force-100-mid", {"force_raw": 100, "position_raw": 500}),
    ("empty-force-200-open", {"force_raw": 200, "position_raw": 900}),
    ("empty-mode-1", {"force_raw": 50, "position_raw": 1000, "mode": 1}),
]


@pytest.mark.parametrize("case_id,raw", GRASP_EMPTY_CASES, ids=[c[0] for c in GRASP_EMPTY_CASES])
def test_judge_grasp_empty(case_id, raw):
    judgment = gs.judge(make_state(base_raw(**raw)), thresholds())
    assert judgment.result == gs.RESULT_GRASP_EMPTY, judgment.reason
    assert judgment.severity == gs.SEV_RECOVERABLE
    assert judgment.retryable is True
    assert "重抓" in judgment.next_action


STALL_CASES = [
    ("stall-pos-0", {"position_raw": 0}),
    ("stall-pos-100", {"position_raw": 100}),
    ("stall-pos-200", {"position_raw": 200}),
    ("stall-pos-300", {"position_raw": 300}),
    ("stall-pos-400", {"position_raw": 400}),
    ("stall-pos-500", {"position_raw": 500}),
    ("stall-pos-600", {"position_raw": 600}),
    ("stall-pos-700", {"position_raw": 700}),
    ("stall-pos-800", {"position_raw": 800}),
    ("stall-pos-900", {"position_raw": 900}),
    ("stall-pos-1000", {"position_raw": 1000}),
    ("stall-hot", {"position_raw": 400, "temp_c": 55}),
]


@pytest.mark.parametrize("case_id,raw", STALL_CASES, ids=[c[0] for c in STALL_CASES])
def test_judge_stall(case_id, raw):
    judgment = gs.judge(make_state(base_raw(error=1, **raw)), thresholds())
    assert judgment.result == gs.RESULT_STALL, judgment.reason
    assert judgment.severity == gs.SEV_RECOVERABLE
    assert judgment.retryable is True
    assert judgment.evidence["error_flags"]["stall"] is True


OVERTEMP_CASES = [
    ("ot-40", 40),
    ("ot-45", 45),
    ("ot-50", 50),
    ("ot-55", 55),
    ("ot-60", 60),
    ("ot-65", 65),
    ("ot-70", 70),
    ("ot-75", 75),
    ("ot-80", 80),
    ("ot-85", 85),
    ("ot-90", 90),
    ("ot-95", 95),
]


@pytest.mark.parametrize("case_id,temp", OVERTEMP_CASES, ids=[c[0] for c in OVERTEMP_CASES])
def test_judge_overtemp(case_id, temp):
    judgment = gs.judge(make_state(base_raw(error=2, temp_c=temp)), thresholds())
    assert judgment.result == gs.RESULT_OVERTEMP, judgment.reason
    assert judgment.severity == gs.SEV_NEEDS_HUMAN
    assert judgment.retryable is False
    assert str(temp) in judgment.next_action  # 带温度证据


OVERCURRENT_CASES = [
    ("oc-200", 200),
    ("oc-400", 400),
    ("oc-600", 600),
    ("oc-800", 800),
    ("oc-1000", 1000),
    ("oc-1200", 1200),
    ("oc-1400", 1400),
    ("oc-1600", 1600),
    ("oc-1800", 1800),
    ("oc-2000", 2000),
    ("oc-2200", 2200),
    ("oc-2400", 2400),
]


@pytest.mark.parametrize("case_id,mA", OVERCURRENT_CASES, ids=[c[0] for c in OVERCURRENT_CASES])
def test_judge_overcurrent(case_id, mA):
    judgment = gs.judge(make_state(base_raw(error=4, current_mA=mA)), thresholds())
    assert judgment.result == gs.RESULT_OVERCURRENT, judgment.reason
    assert judgment.severity == gs.SEV_NEEDS_PARAM
    assert judgment.retryable is False
    assert "24 V" in judgment.next_action
    assert "共地" in judgment.next_action


OFFLINE_CASES = [
    ("off-disable", {"protocol_state": "disable", "online": False}),
    ("off-disable-unknown-online", {"protocol_state": "disable"}),
    ("off-offline", {"protocol_state": "offline", "online": False}),
    ("off-offline-online-true", {"protocol_state": "offline", "online": True}),
    ("off-online-false", {"protocol_state": "online", "online": False}),
    ("off-unknown-online-false", {"protocol_state": "unknown", "online": False}),
    ("off-disable-hot", {"protocol_state": "disable", "temp_c": 60}),
    ("off-offline-cold", {"protocol_state": "offline", "temp_c": 20}),
    ("off-offline-highforce", {"protocol_state": "offline", "force_raw": 900}),
    ("off-disable-pos0", {"protocol_state": "disable", "position_raw": 0}),
    ("off-offline-pos1000", {"protocol_state": "offline", "position_raw": 1000}),
    ("off-disable-mode1", {"protocol_state": "disable", "mode": 1}),
]


@pytest.mark.parametrize("case_id,raw", OFFLINE_CASES, ids=[c[0] for c in OFFLINE_CASES])
def test_judge_offline(case_id, raw):
    judgment = gs.judge(make_state(base_raw(**raw)), thresholds())
    assert judgment.result == gs.RESULT_OFFLINE, judgment.reason
    assert judgment.severity in (gs.SEV_NEEDS_PARAM, gs.SEV_RECOVERABLE)
    assert judgment.next_action


def test_offline_distinguishes_disabled_protocol_from_offline_device():
    disabled = gs.judge(make_state(base_raw(protocol_state="disable")), thresholds())
    offline = gs.judge(make_state(base_raw(protocol_state="offline")), thresholds())
    assert disabled.severity == gs.SEV_NEEDS_PARAM
    assert disabled.retryable is False
    assert "协议" in disabled.next_action
    assert offline.severity == gs.SEV_RECOVERABLE
    assert offline.retryable is True


# --------------------------------------------------------------------------
# 5. 恢复阶梯
# --------------------------------------------------------------------------

def test_ladder_grasp_empty_regrasp_then_escalate():
    for attempt, severity, retryable in ((0, gs.SEV_RECOVERABLE, True),
                                         (1, gs.SEV_RECOVERABLE, True),
                                         (2, gs.SEV_NEEDS_HUMAN, False)):
        judgment = gs.judge(
            make_state(base_raw(force_raw=100)), thresholds(attempt=attempt)
        )
        assert judgment.result == gs.RESULT_GRASP_EMPTY
        assert judgment.severity == severity, attempt
        assert judgment.retryable is retryable, attempt
        assert judgment.next_action


def test_ladder_grasp_empty_never_exceeds_max_regrasp():
    judgment = gs.judge(make_state(base_raw(force_raw=100)), thresholds(attempt=9))
    assert judgment.severity == gs.SEV_NEEDS_HUMAN
    assert judgment.retryable is False


def test_ladder_stall_single_retry_then_human():
    first = gs.judge(make_state(base_raw(error=1)), thresholds(attempt=0))
    assert first.result == gs.RESULT_STALL
    assert first.severity == gs.SEV_RECOVERABLE
    assert first.retryable is True
    assert "松手" in first.next_action

    again = gs.judge(make_state(base_raw(error=1)), thresholds(attempt=1))
    assert again.severity == gs.SEV_NEEDS_HUMAN
    assert again.retryable is False


def test_ladder_overtemp_never_retries():
    judgment = gs.judge(make_state(base_raw(error=2, temp_c=78)), thresholds(attempt=0))
    assert judgment.result == gs.RESULT_OVERTEMP
    assert judgment.severity == gs.SEV_NEEDS_HUMAN
    assert judgment.retryable is False
    assert "78" in judgment.next_action


def test_ladder_overcurrent_mentions_external_supply_and_common_ground():
    judgment = gs.judge(make_state(base_raw(error=4)), thresholds())
    assert judgment.result == gs.RESULT_OVERCURRENT
    assert judgment.severity == gs.SEV_NEEDS_PARAM
    assert "外部 24 V" in judgment.next_action
    assert "共地" in judgment.next_action
    assert "1.5 A" in judgment.next_action
    assert "1.9 A" in judgment.next_action


def test_ladder_offline_retry_then_human():
    first = gs.judge(make_state(base_raw(protocol_state="offline")), thresholds(attempt=0))
    assert first.severity == gs.SEV_RECOVERABLE
    assert first.retryable is True
    again = gs.judge(make_state(base_raw(protocol_state="offline")), thresholds(attempt=1))
    assert again.severity == gs.SEV_NEEDS_HUMAN
    assert again.retryable is False


def test_recovery_ladder_has_no_forbidden_clear_error_action():
    """红线：阶梯里不得出现「写 0x0008 清错误码」。"""
    assert gs.recovery_ladder_violations() == []
    for steps in gs.RECOVERY_LADDER.values():
        for step in steps:
            assert "0x0008" not in step["next_action"]
            assert "清错误" not in step["next_action"]


# --------------------------------------------------------------------------
# 6. NEEDS_PARAM 负例
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "missing_key",
    ["force_threshold", "grasp_position_window", "position_mm_per_unit", "force_scale"],
)
def test_missing_threshold_or_coefficient_never_yields_grasp_result(missing_key):
    th = thresholds()
    th[missing_key] = None
    judgment = gs.judge(make_state(base_raw(force_raw=900, position_raw=500)), th)
    assert judgment.result == gs.RESULT_NEEDS_PARAM, judgment.reason
    assert judgment.result not in (gs.RESULT_GRASP_OK, gs.RESULT_GRASP_EMPTY)
    assert judgment.severity == gs.SEV_NEEDS_PARAM
    assert missing_key in judgment.evidence["thresholds_missing"]


def test_missing_thresholds_entirely_is_needs_param():
    judgment = gs.judge(make_state(base_raw(force_raw=900)), None)
    assert judgment.result == gs.RESULT_NEEDS_PARAM
    assert judgment.severity == gs.SEV_NEEDS_PARAM


def test_invalid_grasp_window_is_treated_as_missing():
    th = thresholds(grasp_position_window=(40.0, 10.0))
    judgment = gs.judge(make_state(base_raw(force_raw=900)), th)
    assert judgment.result == gs.RESULT_NEEDS_PARAM


def test_empty_thresholds_dict_is_needs_param():
    judgment = gs.judge(make_state(base_raw(force_raw=900)), {})
    assert judgment.result == gs.RESULT_NEEDS_PARAM


def test_gripper_api_force_scale_unknown_is_needs_param():
    """SDK 路径力度标度不明 → force_norm 为 None → 不得产出夹到/夹空。"""
    state = make_state(base_raw(force_raw=1500), path="gripper_api")
    judgment = gs.judge(state, thresholds())
    assert judgment.result == gs.RESULT_NEEDS_PARAM
    assert judgment.result not in (gs.RESULT_GRASP_OK, gs.RESULT_GRASP_EMPTY)


# --------------------------------------------------------------------------
# 7. MotionGate
# --------------------------------------------------------------------------

def test_motion_gate_initial_state_is_idle_and_allows_motion():
    gate = gs.MotionGate()
    assert gate.state == gs.MotionGate.STATE_IDLE
    assert gate.check() is None
    assert gate.is_blocked is False


def test_motion_gate_begin_action_blocks_with_reason():
    gate = gs.MotionGate()
    gate.begin_action("夹爪闭合中")
    assert gate.state == gs.MotionGate.STATE_GRIPPER_BUSY
    assert gate.is_blocked is True
    blocked = gate.check()
    assert blocked is not None
    assert set(blocked) == {"state", "reason", "next_action"}
    assert blocked["state"] == gs.MotionGate.STATE_GRIPPER_BUSY
    assert blocked["reason"] == "夹爪闭合中"
    assert blocked["next_action"]


def test_motion_gate_unjudged_blocks_after_action():
    gate = gs.MotionGate()
    gate.begin_action("夹爪闭合中")
    gate.mark_unjudged("夹爪状态未判定")
    assert gate.state == gs.MotionGate.STATE_GRIPPER_UNJUDGED
    blocked = gate.check()
    assert blocked["state"] == gs.MotionGate.STATE_GRIPPER_UNJUDGED
    assert "judge" in blocked["next_action"]


def test_motion_gate_finish_action_returns_to_idle():
    gate = gs.MotionGate()
    gate.begin_action("夹爪闭合中")
    gate.finish_action()
    assert gate.state == gs.MotionGate.STATE_IDLE
    assert gate.check() is None


def test_motion_gate_reset_clears_any_state():
    for state in gs.MotionGate.STATES:
        gate = gs.MotionGate(state=state, reason="x")
        gate.reset()
        assert gate.state == gs.MotionGate.STATE_IDLE
        assert gate.check() is None


def test_motion_gate_rejects_unknown_state():
    with pytest.raises(ValueError):
        gs.MotionGate(state="WAT")


def test_motion_gate_state_transitions_are_exhaustive():
    assert set(gs.MotionGate.STATES) == {"IDLE", "GRIPPER_BUSY", "GRIPPER_UNJUDGED"}


# --------------------------------------------------------------------------
# 8. 零 SDK 负例
# --------------------------------------------------------------------------

def _module_source() -> str:
    import rm_hw_bridge

    pkg_dir = os.path.dirname(os.path.abspath(rm_hw_bridge.__file__))
    with open(os.path.join(pkg_dir, "gripper_state.py"), "r", encoding="utf-8") as fh:
        return fh.read()


def test_gripper_state_module_does_not_import_sdk_bindings():
    """纯逻辑不得触碰 SDK 绑定（ctypes / generated / loader）。"""
    tree = ast.parse(_module_source())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module or "")
            if node.level:  # 相对 import 同样不允许（纯逻辑应完全自洽）
                imported.add(".")
    assert not any(name.startswith("rm_hw_bridge") for name in imported), imported
    assert "ctypes" not in imported
    assert not any(name.startswith("rm_hw_bridge.generated") for name in imported)
    assert "rm_hw_bridge.loader" not in imported
    # 也不许出现 ctypes 名字的使用
    assert not any(
        isinstance(node, ast.Name) and node.id == "ctypes" for node in ast.walk(tree)
    )


def test_gripper_state_module_is_importable_without_generated_bindings():
    """在全新解释器里导入纯逻辑模块：不得连带加载 ctypes / SDK 绑定。"""
    import subprocess
    import sys

    import rm_hw_bridge

    pkg_src = os.path.dirname(os.path.dirname(os.path.abspath(rm_hw_bridge.__file__)))
    script = (
        "import sys; import rm_hw_bridge.gripper_state as gs; "
        "print('ctypes' in sys.modules, "
        "'rm_hw_bridge.generated.rm_api2_ctypes' in sys.modules, "
        "'rm_hw_bridge.loader' in sys.modules)"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = pkg_src + os.pathsep + env.get("PYTHONPATH", "")
    out = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env=env,
        check=True,
    )
    loaded_ctypes, loaded_generated, loaded_loader = out.stdout.strip().split()
    assert loaded_ctypes == "False"
    assert loaded_generated == "False"
    assert loaded_loader == "False"


# --------------------------------------------------------------------------
# 9. 证据结构
# --------------------------------------------------------------------------

def test_judgment_evidence_contains_raw_snapshot_and_thresholds():
    judgment = gs.judge(make_state(base_raw(force_raw=900)), thresholds())
    assert judgment.evidence["raw"]["force_raw"] == 900
    assert judgment.evidence["error_bits"] == 0
    assert judgment.evidence["thresholds"]["force_threshold"] == 300.0
    assert judgment.evidence["position_mm"] == pytest.approx(32.5)
    assert judgment.evidence["force_N"] == pytest.approx(117.0)
    assert judgment.evidence["protocol_state"] == "online"
    payload = judgment.to_dict()
    assert set(payload) == {"result", "severity", "reason", "evidence", "next_action", "retryable"}
