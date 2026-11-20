"""``scripts/rm75_window_gate.py``（真机窗口**只读**门禁）的离线用例。

为什么测：门禁是 S10「窗口不可用 → blocked 结案」这条路径的**唯一入口**。
它一旦被改坏（把不通过判成通过、把 blocked 判成 ok、或偷偷发运动指令），
下游的结案工件就会把「没跑成」说成「跑成了」。所以这里钉死：

* 全部门禁通过 → ``status="ok"`` / 退出码 0；
* 任一项不过 → ``status="blocked"`` / 退出码 3 / reason 非空 /
  next_action 非空；
* **零运动指令**：探针只以 ``precheck_only=True`` 被调用，网络不通时
  连探针都不调用；
* 工件必含 ``honesty`` 声明与逐项 ``checks`` 结果；
* status / 退出码词表**唯一来源** = ``rm_executor.contract``。

全部用**假探测**注入 → 不连真机、不发任何运动指令。
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_window_gate.py")

for _rel in ("dualarm_ws/src/rm_executor", "dualarm_ws/src/rm_hw_bridge",
             "dualarm_ws/src/rm_safety"):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rm_executor import contract  # noqa: E402


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_window_gate_under_test",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


# ------------------------------------------------------------------ 假探测
class FakeProbe(object):
    """假探针：记录调用参数，绝不发任何运动指令。"""

    def __init__(self, report=None):
        self.calls = []
        self.report = report if report is not None else {
            "status": "ok", "reason": "",
            "phases": [{"name": "preflight", "status": "ok",
                        "data": {"checks": [{"name": "all_joints_enabled",
                                             "ok": True, "detail": "7 轴全部使能"}]}}],
        }

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return json.loads(json.dumps(self.report))


class Recorder(object):
    """记录各探测的调用次数（用于断言「未调用任何运动入口」）。"""

    def __init__(self):
        self.motion_calls = []
        self.calls = []

    def ping(self, ip, **kwargs):
        self.calls.append(("ping", ip))
        return True, "ping 通"

    def tcp(self, ip, port, **kwargs):
        self.calls.append(("tcp", ip, port))
        return True, "TCP %s:%s 可连接" % (ip, port)

    def udp_free(self, port, **kwargs):
        self.calls.append(("udp_free", port))
        return True, "本地 UDP %s 空闲" % port

    def sdk(self):
        self.calls.append(("sdk",))
        return True, "rm_hw_bridge 可导入且关键符号齐全"


def _all_pass(script, recorder=None, probe=None):
    rec = recorder or Recorder()
    p = probe or FakeProbe()
    artifact = script.run_gate(ping=rec.ping, tcp=rec.tcp, udp_free=rec.udp_free,
                                sdk=rec.sdk, probe=p)
    return artifact, rec, p


# ------------------------------------------------------------------ 全通过
def test_all_checks_pass_gives_status_ok(script):
    artifact, _, _ = _all_pass(script)
    assert artifact["status"] == contract.STATUS_OK
    assert artifact["reason"]
    assert artifact["checks"]
    assert all(c["ok"] for c in artifact["checks"])


def test_all_checks_pass_gives_exit_code_0(script, tmp_path):
    artifact, _, _ = _all_pass(script)
    out = tmp_path / "window_gate.json"
    rc = script.main(["--ip", "192.168.1.18", "--out", str(out)],
                     gate=lambda **kwargs: artifact)
    assert rc == contract.EXIT_CODES[contract.STATUS_OK] == 0
    assert out.exists()


def test_artifact_contains_honesty_and_per_check_results(script):
    artifact, _, _ = _all_pass(script)
    assert artifact["honesty"]
    assert "未执行任何运动指令" in artifact["honesty"]
    names = [c["name"] for c in artifact["checks"]]
    for expected in (script.CHECK_IP, script.CHECK_TCP, script.CHECK_UDP,
                     script.CHECK_SDK, script.CHECK_PROBE):
        assert expected in names
    for c in artifact["checks"]:
        assert set(("name", "ok", "detail", "evidence")) <= set(c)


def test_probe_is_called_without_any_motion_argument(script):
    """门禁传给探针的参数里**不含任何运动相关项**。"""
    artifact, _, probe = _all_pass(script)
    assert artifact["status"] == contract.STATUS_OK
    assert len(probe.calls) == 1
    call = probe.calls[0]
    for key in ("joint_index", "delta_deg", "speed_pct", "channel",
                "precheck_only", "dry_run"):
        assert key not in call


def test_default_probe_calls_probe_with_precheck_only(script, monkeypatch):
    """默认探针实现固定 ``precheck_only=True``（只读），不得改成运动模式。"""
    calls = []

    def fake_run_probe(**kwargs):
        calls.append(kwargs)
        return {"status": "ok", "reason": "", "phases": []}

    fake_module = type(sys)("rm75_safe_probe")
    fake_module.run_probe = fake_run_probe
    monkeypatch.setitem(sys.modules, "rm75_safe_probe", fake_module)
    script._default_probe(arm="left", ip="192.168.1.18", port=8080,
                          udp_port=8091, ws=None)
    assert len(calls) == 1
    assert calls[0]["precheck_only"] is True
    for key in ("joint_index", "delta_deg", "speed_pct"):
        assert key not in calls[0]


# ------------------------------------------------------------------ blocked
def test_failed_check_gives_blocked_and_exit_code_3(script, tmp_path):
    rec = Recorder()

    def bad_ping(ip, **kwargs):
        rec.calls.append(("ping", ip))
        return False, "100% 丢包"

    rec.ping = bad_ping
    artifact = script.run_gate(ping=rec.ping, tcp=rec.tcp, udp_free=rec.udp_free,
                                sdk=rec.sdk, probe=FakeProbe())
    assert artifact["status"] == contract.STATUS_BLOCKED
    assert artifact["reason"]
    assert artifact["next_action"]
    out = tmp_path / "window_gate.json"
    rc = script.main(["--out", str(out)], gate=lambda **kwargs: artifact)
    assert rc == contract.EXIT_CODES[contract.STATUS_BLOCKED] == 3
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["status"] == contract.STATUS_BLOCKED


def test_blocked_when_probe_reports_failed_precheck(script):
    """探针 7 项前置检查不过（如 7 轴掉使能）→ blocked。"""
    rec = Recorder()
    probe = FakeProbe({
        "status": "error",
        "reason": "前置检查未通过：all_joints_enabled",
        "phases": [{"name": "preflight", "status": "error", "data": {"checks": [
            {"name": "all_joints_enabled", "ok": False,
             "detail": "未使能关节: [1, 2, 3, 4, 5, 6, 7]"}]}}],
    })
    artifact = script.run_gate(ping=rec.ping, tcp=rec.tcp, udp_free=rec.udp_free,
                                sdk=rec.sdk, probe=probe)
    assert artifact["status"] == contract.STATUS_BLOCKED
    check = [c for c in artifact["checks"] if c["name"] == script.CHECK_PROBE][0]
    assert check["ok"] is False
    assert "all_joints_enabled" in check["detail"]
    assert any("使能" in a for a in artifact["next_action"])


def test_blocked_when_udp_port_occupied(script):
    rec = Recorder()

    def busy(port, **kwargs):
        rec.calls.append(("udp_free", port))
        return False, "本地 UDP %d 已被占用: Address already in use" % port

    artifact = script.run_gate(ping=rec.ping, tcp=rec.tcp, udp_free=busy,
                                sdk=rec.sdk, probe=FakeProbe())
    assert artifact["status"] == contract.STATUS_BLOCKED
    check = [c for c in artifact["checks"] if c["name"] == script.CHECK_UDP][0]
    assert check["ok"] is False
    assert any("ss -ulnp" in a for a in artifact["next_action"])


def test_blocked_when_sdk_not_importable(script):
    rec = Recorder()
    artifact = script.run_gate(ping=rec.ping, tcp=rec.tcp, udp_free=rec.udp_free,
                                sdk=lambda: (False, "rm_hw_bridge 不可导入: ImportError"),
                                probe=FakeProbe())
    assert artifact["status"] == contract.STATUS_BLOCKED
    check = [c for c in artifact["checks"] if c["name"] == script.CHECK_SDK][0]
    assert check["ok"] is False
    assert any("PYTHONPATH" in a for a in artifact["next_action"])


def test_probe_skipped_when_network_gate_fails(script):
    """网络不通 → 连探针都不调用（零连接、零运动指令）。"""
    rec = Recorder()
    probe = FakeProbe()
    artifact = script.run_gate(ping=lambda ip, **kw: (False, "100% 丢包"),
                                tcp=rec.tcp, udp_free=rec.udp_free,
                                sdk=rec.sdk, probe=probe)
    assert artifact["status"] == contract.STATUS_BLOCKED
    assert probe.calls == [], "网络门禁不过时不得调用探针"
    check = [c for c in artifact["checks"] if c["name"] == script.CHECK_PROBE][0]
    assert check["ok"] is False
    assert "跳过" in check["detail"]


def test_no_motion_entry_point_is_ever_called(script):
    """门禁全程只读：不得触碰任何运动入口（转置 / 主动位姿 / 急停）。"""
    rec = Recorder()
    probe = FakeProbe()
    script.run_gate(ping=rec.ping, tcp=rec.tcp, udp_free=rec.udp_free,
                    sdk=rec.sdk, probe=probe)
    forbidden = ("transmit_joint", "preposition_movej", "rm_movej", "rm_movej_canfd",
                 "set_safety", "stop", "rm_set_arm_stop", "movej")
    for call in rec.calls:
        assert not any(f in str(call) for f in forbidden)
    for call in probe.calls:
        for key in ("transmit_joint", "preposition_movej", "movej", "delta_deg",
                    "joint_index", "speed_pct"):
            assert key not in call


def test_attempted_actions_recorded_for_every_probe(script):
    artifact, _, _ = _all_pass(script)
    actions = [a["action"] for a in artifact["attempted_actions"]]
    for expected in ("icmp_ping", "tcp_connect", "udp_port_check", "sdk_import",
                     "probe_precheck"):
        assert expected in actions
    for a in artifact["attempted_actions"]:
        assert a["command"]
        assert a["result"]


def test_next_action_is_pasteable_and_non_empty(script):
    artifact, _, _ = _all_pass(script)
    assert isinstance(artifact["next_action"], list)
    assert artifact["next_action"]
    assert all(isinstance(a, str) and a.strip() for a in artifact["next_action"])
    assert any("rm75_window_gate.py" in a for a in artifact["next_action"])


# ------------------------------------------------------------------ 词表纪律
def test_status_vocabulary_comes_from_contract(script):
    """status / 退出码唯一来源 = rm_executor.contract（禁止第二套字面量）。"""
    assert script.STATUS_OK == contract.STATUS_OK
    assert script.STATUS_BLOCKED == contract.STATUS_BLOCKED
    assert script.EXIT_CODES == contract.EXIT_CODES


def test_schema_is_stable(script):
    assert script.SCHEMA == "rmctl.rm75_window_gate/1"
    artifact, _, _ = _all_pass(script)
    assert artifact["schema"] == "rmctl.rm75_window_gate/1"


def test_artifact_written_to_disk_has_full_shape(script, tmp_path):
    artifact, _, _ = _all_pass(script)
    out = tmp_path / "nested" / "window_gate.json"
    rc = script.main(["--out", str(out)], gate=lambda **kwargs: artifact)
    assert rc == 0
    saved = json.loads(out.read_text(encoding="utf-8"))
    for field in ("schema", "status", "reason", "checks", "attempted_actions",
                  "next_action", "honesty"):
        assert field in saved, "工件缺字段 %s" % field
