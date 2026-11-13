"""SDK 懒加载、rad↔deg 唯一转换点与调用审计的测试（S03-T01）。

覆盖：

* SDK 缺失 → **显式** :class:`NotAvailableError`（消息含导入名 / 分发名 / 原因），
  **绝不**抛裸 ``ImportError``；
* 懒加载路径：模块存在时被正确导入（用假模块注入验证）；
* rad↔deg 换算数值正确 + 往返一致；
* 「唯一转换点」断言：仓库源码合规（正例）+ 造出第二份实现会被抓到（负例）；
* 运行时调用审计：白名单符号不计数、非白名单符号计数。
"""

from __future__ import annotations

import math
import sys
import types

import pytest

from rm75_drake.device import sdk as sdk_mod


# ── SDK 缺失（显式报错）─────────────────────────────────────────────────
def test_load_sdk_raises_not_available_error(monkeypatch):
    """SDK 缺失 → NotAvailableError（消息说明为什么），不是裸 ImportError。"""
    import importlib

    def _boom(name, *args, **kwargs):
        raise ImportError(f"No module named {name!r}")

    monkeypatch.setattr(importlib, "import_module", _boom)
    with pytest.raises(sdk_mod.NotAvailableError) as excinfo:
        sdk_mod.load_sdk()
    message = str(excinfo.value)
    assert sdk_mod.SDK_IMPORT_NAME in message
    assert sdk_mod.SDK_DIST_NAME in message
    assert "ImportError" not in type(excinfo.value).__name__


def test_not_available_error_is_runtime_error():
    """NotAvailableError 必须是 RuntimeError（沿用 S02 viz.py 约定）。"""
    assert issubclass(sdk_mod.NotAvailableError, RuntimeError)


def test_sdk_unavailable_reason_mentions_import_and_dist_names():
    """不可用原因说明必须同时给出导入名与分发名（包名陷阱）。"""
    assert sdk_mod.SDK_IMPORT_NAME == "Robotic_Arm"
    assert sdk_mod.SDK_DIST_NAME == "robotic-arm"
    assert sdk_mod.SDK_IMPORT_NAME in sdk_mod.SDK_UNAVAILABLE_REASON
    assert sdk_mod.SDK_DIST_NAME in sdk_mod.SDK_UNAVAILABLE_REASON


def test_sdk_available_does_not_raise():
    """探测函数永不抛异常（本机无 SDK 时返回 False）。"""
    assert isinstance(sdk_mod.sdk_available(), bool)


@pytest.mark.skipif(
    sdk_mod.sdk_available(), reason="本机已安装 robotic-arm，缺失路径不可测"
)
def test_real_environment_sdk_is_absent_and_reported():
    """本机环境（无 SDK）：load_sdk 必须抛 NotAvailableError 而不是 ImportError。"""
    with pytest.raises(sdk_mod.NotAvailableError):
        sdk_mod.load_sdk()


# ── 懒加载（模块存在时）────────────────────────────────────────────────
def test_load_sdk_returns_module_when_importable(monkeypatch):
    """模块可导入时，load_sdk 返回该模块（懒加载而非硬依赖）。"""
    import importlib

    fake = types.ModuleType(sdk_mod.SDK_IMPORT_NAME)
    monkeypatch.setitem(sys.modules, sdk_mod.SDK_IMPORT_NAME, fake)

    def _fake_import(name, *args, **kwargs):
        if name == sdk_mod.SDK_IMPORT_NAME:
            return fake
        raise ImportError(f"No module named {name!r}")

    monkeypatch.setattr(importlib, "import_module", _fake_import)
    assert sdk_mod.load_sdk() is fake


def test_load_rm_interface_reports_missing_submodule(monkeypatch):
    """子模块缺失 → NotAvailableError（消息点名 rm_robot_interface）。"""
    import importlib

    def _boom(name, *args, **kwargs):
        raise ModuleNotFoundError(f"No module named {name!r}")

    monkeypatch.setattr(importlib, "import_module", _boom)
    with pytest.raises(sdk_mod.NotAvailableError) as excinfo:
        sdk_mod.load_rm_interface()
    assert "rm_robot_interface" in str(excinfo.value)


# ── rad↔deg 唯一转换点 ────────────────────────────────────────────────
def test_deg_to_rad_known_values():
    assert sdk_mod.deg_to_rad(180.0) == pytest.approx(math.pi)
    assert sdk_mod.deg_to_rad(90.0) == pytest.approx(math.pi / 2.0)
    assert sdk_mod.deg_to_rad(0.0) == pytest.approx(0.0)
    assert sdk_mod.deg_to_rad(-180.0) == pytest.approx(-math.pi)


def test_rad_to_deg_known_values():
    assert sdk_mod.rad_to_deg(math.pi) == pytest.approx(180.0)
    assert sdk_mod.rad_to_deg(math.pi / 2.0) == pytest.approx(90.0)
    assert sdk_mod.rad_to_deg(0.0) == pytest.approx(0.0)


def test_conversion_round_trip():
    for rad in (-3.0, -0.5, 0.0, 0.5, 3.0):
        assert sdk_mod.deg_to_rad(sdk_mod.rad_to_deg(rad)) == pytest.approx(rad)


def test_conversion_guards_57x_error():
    """把 1 rad 当成 1° 少发 57 倍；换算必须给出正确角度（防呆断言）。"""
    assert sdk_mod.rad_to_deg(1.0) == pytest.approx(57.29577951308232)
    assert sdk_mod.rad_to_deg(1.0) != pytest.approx(1.0)


def test_repo_has_single_conversion_point():
    """正例：整个包里 rad↔deg 转换只有 device/sdk.py 一处实现。"""
    offenders = sdk_mod.assert_single_conversion_point()
    assert offenders == []


def test_second_conversion_implementation_is_detected(tmp_path):
    """负例：另写一份 deg_to_rad 实现 → 断言必须失败并点名文件。"""
    bad = tmp_path / "rogue.py"
    bad.write_text(
        "import math\n\n\ndef deg_to_rad(deg):\n    return deg * math.pi / 180.0\n",
        encoding="utf-8",
    )
    with pytest.raises(AssertionError) as excinfo:
        sdk_mod.assert_single_conversion_point(tmp_path)
    assert "rogue.py" in str(excinfo.value)


def test_inline_conversion_is_detected(tmp_path):
    """负例：内联换算（math.radians / 180/math.pi）同样被抓。"""
    bad = tmp_path / "rogue2.py"
    bad.write_text("import math\n\nx = math.radians(30)\n", encoding="utf-8")
    with pytest.raises(AssertionError) as excinfo:
        sdk_mod.assert_single_conversion_point(tmp_path)
    assert "rogue2.py" in str(excinfo.value)


def test_string_literal_mention_is_not_an_offender(tmp_path):
    """负例（防误伤）：提示语里的换算说明不算第二份实现。"""
    ok = tmp_path / "docs.py"
    ok.write_text('MSG = "请乘 math.pi / 180.0 换算为弧度"\n', encoding="utf-8")
    assert sdk_mod.assert_single_conversion_point(tmp_path) == []


# ── 能力探测 ───────────────────────────────────────────────────────────
def test_probe_capabilities_reports_unknown_without_arm():
    """没给 arm 实例时，能力字段必须是 None/False（**不猜**）。"""
    caps = sdk_mod.probe_capabilities()
    assert caps.arm_dof is None
    assert caps.controller_generation is None
    assert caps.triple_mode_supported is False
    assert caps.detail


def test_probe_capabilities_reads_arm_attributes():
    """给了 arm 实例时，探测 arm_dof 与控制器代次。"""

    class FakeArm:
        arm_dof = 7
        controller_type = 4
        triple_mode = True

    caps = sdk_mod.probe_capabilities(FakeArm())
    assert caps.arm_dof == 7
    assert caps.controller_generation == 4
    assert caps.triple_mode_supported is True


# ── 调用审计 ───────────────────────────────────────────────────────────
def test_audit_counts_whitelist_calls_as_clean():
    audit = sdk_mod.SdkAudit()
    audit.record("rm_movej_canfd", "[]")
    audit.record("rm_set_joint_en_state", "True")
    assert audit.total_call_count == 2
    assert audit.non_whitelist_call_count == 0
    assert audit.called("rm_movej_canfd") == 1


def test_audit_counts_non_whitelist_calls():
    audit = sdk_mod.SdkAudit()
    audit.record("rm_algo_inverse_kinematics", "[]")
    assert audit.non_whitelist_call_count == 1
    assert audit.non_whitelist_calls[0][0] == "rm_algo_inverse_kinematics"


def test_audit_reset_clears_counters():
    audit = sdk_mod.SdkAudit()
    audit.record("rm_movej_canfd")
    audit.reset()
    assert audit.total_call_count == 0
    assert audit.non_whitelist_call_count == 0


def test_whitelist_does_not_intersect_forbidden_apis():
    """白名单与 versions.lock:forbidden_apis **不得相交**。"""
    from rm75_drake import paths

    import yaml

    lock = yaml.safe_load(paths.versions_lock_path.read_text(encoding="utf-8"))
    forbidden = set(lock["forbidden_apis"])
    assert not (forbidden & set(sdk_mod.SDK_CALL_WHITELIST))


def test_proxy_delegates_and_audits():
    calls = []

    def rm_movej_canfd(joint, follow=False, trajectory_mode=0):
        calls.append((joint, follow, trajectory_mode))
        return "ok"

    target = types.SimpleNamespace(rm_movej_canfd=rm_movej_canfd)
    audit = sdk_mod.SdkAudit()
    proxy = sdk_mod.SdkProxy(target, audit)
    assert proxy.rm_movej_canfd([0.0], follow=True, trajectory_mode=0) == "ok"
    assert calls == [([0.0], True, 0)]
    assert audit.total_call_count == 1
    assert audit.non_whitelist_call_count == 0


def test_proxy_flags_non_whitelist_symbol():
    target = types.SimpleNamespace(rm_algo_inverse_kinematics=lambda: 0)
    audit = sdk_mod.SdkAudit()
    proxy = sdk_mod.SdkProxy(target, audit)
    proxy.rm_algo_inverse_kinematics()
    assert audit.non_whitelist_call_count == 1


def test_proxy_raises_for_unknown_attribute():
    target = types.SimpleNamespace()
    proxy = sdk_mod.SdkProxy(target)
    with pytest.raises(AttributeError):
        proxy.rm_not_a_real_symbol
