"""厂商解算接口「运行时零调用」审计的测试（S07-T03 / R008）。

双证闭环
--------
* **静态**那一证 = ``scripts/scan_forbidden_apis.py``（名单唯一真相源 =
  ``versions.lock:forbidden_apis``）；
* **运行时**那一证 = 本文件：禁用名单被读进 :class:`~rm75_drake.device.sdk.SdkAudit`，
  命中即计数；``cli exec --json`` 给出计数；命中 ⇒ **退出码 1**。

覆盖的契约
----------
**正例**
* 禁用集合 == ``versions.lock:forbidden_apis``（逐项一致，不另立名单）；
* 白名单符号调用后：非白名单计数 = 0、禁用计数 = 0；
* ``get_submodule``（取子模块）**不**增加调用数 / 非白名单 / 禁用计数；
* 零硬件 ``exec --dry-run --json``：两个计数都是 0，退出码 0。

**负例（证明审计不是"永远绿灯"）**
* 记一次禁用名 ⇒ 禁用计数 = 1 且非白名单计数 = 1；
* **禁用名的变体名**（名单**未**登记的变体）同样必须被识别 ⇒ 不得漏检；
* CLI 注入一次禁用调用 ⇒ 退出码 1 且 JSON 里的计数 > 0；
* 锁文件缺 ``forbidden_apis`` 段 ⇒ 读取必须**显式报错**（不得静默空名单）。
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake import cli as cli_mod  # noqa: E402
from rm75_drake.device import sdk as sdk_mod  # noqa: E402
from rm75_drake.device.transport import ScriptedTransport  # noqa: E402

#: 禁用名（取自锁文件，用于正例 / 负例；**不**在测试里另立名单）。
FORBIDDEN_NAME = "rm_algo_inverse_kinematics"
#: 禁用名的**变体名**形态：名单里**没有**这一项 ⇒ 运行时规则必须自己识别出来。
FORBIDDEN_VARIANT = "rm_algo_inverse_kinematics_v2"
#: 白名单符号（允许的透传接口；与禁用名 ``rm_movej`` 前缀相近 ⇒ 不得误伤）。
WHITELIST_NAME = "rm_movej_canfd"

#: 设备配置模板（夹具）：围栏放宽 ⇒ 零位附近也能执行。
DEVICE_CONFIG_TEMPLATE = """schema_version: 1
name: rm75_single_arm
arm_ip: "192.168.1.18"
tcp_port: 8080
udp_port: 8080
udp_cycle: 1
joint_zero_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
accel_limits:
  measured: false
  values: [6.0, 6.0, 8.0, 8.0, 8.0, 8.0, 8.0]
current_thresholds:
  calibrated: false
  values: [1500.0, 1500.0, 1200.0, 1200.0, 1000.0, 1000.0, 1000.0]
fence:
  table_height_m: 0.5
  singularity_radius_m: 0.01
"""


@pytest.fixture(scope="module")
def lock() -> dict:
    data = yaml.safe_load((REPO_ROOT / "versions.lock").read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def _write_device_config(path: Path) -> Path:
    path.write_text(DEVICE_CONFIG_TEMPLATE, encoding="utf-8")
    return path


def _write_recording(path: Path, samples) -> Path:
    payload = {
        "schema_version": 1,
        "robot_name": "rm75",
        "dt": 0.005,
        "samples": [
            {"seq": index, "t_abs": index * 0.005, "q": list(q)}
            for index, q in enumerate(samples)
        ],
        "gripper_schedule": [],
        "created_at": "2026-01-01T00:00:00Z",
        "meta": {"urdf_sha256": "", "scenario_path": str(SCENARIO_PATH)},
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def _ramp_samples(base, *, count: int = 20, accel: float = 1.0):
    """一段**平滑起步**的关节序列（满足速度 / 加速度不变量）。"""
    dt = 0.005
    rows = []
    for index in range(count):
        t = index * dt
        q = list(base)
        q[0] = base[0] + 0.5 * accel * t * t
        rows.append(q)
    return rows


# ══════════════════════════════════════════════════════════════════════
# 名单加载（唯一真相源）
# ══════════════════════════════════════════════════════════════════════
def test_forbidden_set_equals_lock_section(lock):
    """正例：运行时禁用集合 == versions.lock:forbidden_apis（逐项一致）。"""
    audit = sdk_mod.SdkAudit()
    assert set(audit.forbidden) == set(lock["forbidden_apis"])
    assert audit.forbidden == frozenset(lock["forbidden_apis"])


def test_load_forbidden_apis_reads_given_lock(tmp_path):
    """正例：可从指定锁文件读取名单（内容即 ``forbidden_apis`` 段）。"""
    lock_path = tmp_path / "lock.yaml"
    lock_path.write_text(
        yaml.safe_dump({"forbidden_apis": ["a_api", "b_api"], "allowed_apis": []}),
        encoding="utf-8",
    )
    assert sdk_mod.load_forbidden_apis(lock_path) == frozenset({"a_api", "b_api"})


def test_load_forbidden_apis_cache_follows_lock_content(tmp_path):
    """正例：锁文件内容变了 ⇒ 缓存自动失效（不拿旧名单当新名单用）。"""
    lock_path = tmp_path / "lock.yaml"
    lock_path.write_text(
        yaml.safe_dump({"forbidden_apis": ["a_api"]}), encoding="utf-8"
    )
    assert sdk_mod.load_forbidden_apis(lock_path) == frozenset({"a_api"})
    lock_path.write_text(
        yaml.safe_dump({"forbidden_apis": ["a_api", "c_api"]}), encoding="utf-8"
    )
    assert sdk_mod.load_forbidden_apis(lock_path) == frozenset({"a_api", "c_api"})


def test_missing_lock_section_raises(tmp_path):
    """负例：锁文件缺 ``forbidden_apis`` ⇒ 显式报错（不得静默空名单）。"""
    lock_path = tmp_path / "bad.yaml"
    lock_path.write_text(yaml.safe_dump({"schema_version": 1}), encoding="utf-8")
    with pytest.raises(sdk_mod.ForbiddenApisError):
        sdk_mod.load_forbidden_apis(lock_path)


def test_missing_lock_file_raises(tmp_path):
    """负例：锁文件不存在 ⇒ 显式报错。"""
    with pytest.raises(sdk_mod.ForbiddenApisError):
        sdk_mod.load_forbidden_apis(tmp_path / "nope.yaml")


# ══════════════════════════════════════════════════════════════════════
# 审计计数（正例 / 负例）
# ══════════════════════════════════════════════════════════════════════
def test_whitelist_call_keeps_both_counters_zero():
    """正例：白名单符号调用后，非白名单计数与禁用计数都为 0。"""
    audit = sdk_mod.SdkAudit()
    audit.record(WHITELIST_NAME, "[]")
    assert audit.total_call_count == 1
    assert audit.non_whitelist_call_count == 0
    assert audit.forbidden_call_count == 0
    assert audit.forbidden_calls == []


def test_forbidden_call_counts_in_both_counters(lock):
    """负例：记一次禁用名 ⇒ 禁用计数 = 1 且非白名单计数 = 1。"""
    assert FORBIDDEN_NAME in lock["forbidden_apis"]
    audit = sdk_mod.SdkAudit()
    audit.record(FORBIDDEN_NAME, "[]")
    assert audit.forbidden_call_count == 1
    assert audit.non_whitelist_call_count == 1
    assert audit.forbidden_calls[0][0] == FORBIDDEN_NAME


def test_forbidden_variant_not_listed_is_still_detected(lock):
    """负例：禁用名的**变体名**（名单未登记）也必须被识别 ⇒ 不得漏检。"""
    assert FORBIDDEN_VARIANT not in lock["forbidden_apis"]
    audit = sdk_mod.SdkAudit()
    audit.record(FORBIDDEN_VARIANT, "[]")
    assert audit.forbidden_call_count == 1
    assert audit.non_whitelist_call_count == 1


def test_allowed_api_with_forbidden_prefix_is_not_flagged():
    """负例（防前缀误伤）：``rm_movej_canfd`` 不得被 ``rm_movej`` 误判。"""
    audit = sdk_mod.SdkAudit()
    audit.record(WHITELIST_NAME, "[]")
    assert audit.forbidden_call_count == 0
    assert audit.non_whitelist_call_count == 0


def test_reset_clears_forbidden_counter():
    """正例：reset() 一并清空禁用计数（分段统计用）。"""
    audit = sdk_mod.SdkAudit()
    audit.record(FORBIDDEN_NAME, "[]")
    audit.reset()
    assert audit.total_call_count == 0
    assert audit.non_whitelist_call_count == 0
    assert audit.forbidden_call_count == 0
    assert audit.forbidden_calls == []


def test_audit_snapshot_is_json_serializable():
    """正例：审计快照可 JSON 序列化（CLI / 脚本输出用）。"""
    import json as _json

    audit = sdk_mod.SdkAudit()
    audit.record(WHITELIST_NAME, "[]")
    snapshot = sdk_mod.audit_snapshot(audit)
    assert snapshot["total_call_count"] == 1
    assert snapshot["non_whitelist_call_count"] == 0
    assert snapshot["forbidden_call_count"] == 0
    assert _json.loads(_json.dumps(snapshot))["forbidden_calls"] == []


# ══════════════════════════════════════════════════════════════════════
# SdkProxy.get_submodule（属性访问不计调用）
# ══════════════════════════════════════════════════════════════════════
def test_get_submodule_returns_real_submodule_and_counts_nothing():
    """正例：get_submodule 返回真实子模块，且不增加任何计数。"""
    submodule = types.SimpleNamespace(RoboticArm=object)
    target = types.SimpleNamespace(rm_robot_interface=submodule)
    audit = sdk_mod.SdkAudit()
    proxy = sdk_mod.SdkProxy(target, audit)

    assert proxy.get_submodule("rm_robot_interface") is submodule
    assert audit.total_call_count == 0
    assert audit.non_whitelist_call_count == 0
    assert audit.forbidden_call_count == 0
    assert proxy.submodule_accesses == ["rm_robot_interface"]


def test_get_submodule_rejects_private_names():
    """负例：私有属性名不得被取出（与 ``__getattr__`` 的约定一致）。"""
    proxy = sdk_mod.SdkProxy(types.SimpleNamespace(), sdk_mod.SdkAudit())
    with pytest.raises(AttributeError):
        proxy.get_submodule("_secret")


# ══════════════════════════════════════════════════════════════════════
# CLI（零硬件 dry-run）
# ══════════════════════════════════════════════════════════════════════
def _exec_json(tmp_path, capsys, **extra):
    config = _write_device_config(tmp_path / "device.yaml")
    recording = _write_recording(
        tmp_path / "rec.json",
        _ramp_samples((0.3, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)),
    )
    argv = [
        "exec",
        "--dry-run",
        "--recording",
        str(recording),
        "--device-config",
        str(config),
        "--json",
    ]
    argv.extend(extra.get("extra_argv", []))
    code = cli_mod.main(argv)
    captured = capsys.readouterr()
    return code, captured


def test_cli_dry_run_json_reports_zero_audit_counters(tmp_path, capsys):
    """正例：零硬件 dry-run ⇒ 两个计数都是 0，退出码 0。"""
    code, captured = _exec_json(tmp_path, capsys)
    assert code == 0, captured.out + captured.err
    payload = json.loads(captured.out)
    audit = payload["sdk_audit"]
    assert audit["forbidden_call_count"] == 0
    assert audit["non_whitelist_call_count"] == 0
    assert audit["forbidden_calls"] == []


def test_cli_forbidden_call_exits_one_with_count(tmp_path, capsys, monkeypatch):
    """负例：注入一次禁用调用 ⇒ 退出码 1 且 JSON 计数 > 0（不是假绿灯）。"""
    original = cli_mod._build_exec_transport

    def _leaky(*, dry_run, device_config, audit):
        transport = original(dry_run=dry_run, device_config=device_config, audit=audit)
        audit.record(FORBIDDEN_NAME, "[]")
        return transport

    monkeypatch.setattr(cli_mod, "_build_exec_transport", _leaky)
    code, captured = _exec_json(tmp_path, capsys)
    assert code == 1, captured.out + captured.err
    payload = json.loads(captured.out)
    audit = payload["sdk_audit"]
    assert audit["forbidden_call_count"] == 1
    assert audit["non_whitelist_call_count"] == 1
    assert audit["forbidden_calls"][0][0] == FORBIDDEN_NAME


def test_cli_forbidden_variant_exits_one(tmp_path, capsys, monkeypatch):
    """负例：注入一次禁用名的**变体名** ⇒ 退出码 1（变体不得漏检）。"""
    original = cli_mod._build_exec_transport

    def _leaky(*, dry_run, device_config, audit):
        transport = original(dry_run=dry_run, device_config=device_config, audit=audit)
        audit.record(FORBIDDEN_VARIANT, "[]")
        return transport

    monkeypatch.setattr(cli_mod, "_build_exec_transport", _leaky)
    code, captured = _exec_json(tmp_path, capsys)
    assert code == 1, captured.out + captured.err
    payload = json.loads(captured.out)
    assert payload["sdk_audit"]["forbidden_call_count"] == 1


def test_cli_audit_counts_are_present_without_json_flag(tmp_path, capsys):
    """正例：人类可读输出里也给出审计计数（不得只在 JSON 里可见）。"""
    config = _write_device_config(tmp_path / "device.yaml")
    recording = _write_recording(
        tmp_path / "rec.json",
        _ramp_samples((0.3, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)),
    )
    code = cli_mod.main(
        [
            "exec",
            "--dry-run",
            "--recording",
            str(recording),
            "--device-config",
            str(config),
        ]
    )
    captured = capsys.readouterr()
    assert code == 0, captured.out + captured.err
    assert "命中禁用名单 0" in captured.out


def test_scripted_transport_has_no_sdk_audit_surface():
    """正例：离线替身**不**自带 SDK 审计（真机路径才有 SDK 调用可审）。"""
    transport = ScriptedTransport()
    assert not hasattr(transport, "audit")
