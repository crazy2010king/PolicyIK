"""``scripts/rm75_stream_loss_drill.py``（真机 UDP 断流演练）的**离线**用例。

为什么测：断流演练是 S10「200 ms 内 L4 停机」的唯一真机证据入口，退出码
语义（0 复现成功 / 3 前置检查不过 / 4 未复现）被改坏就等于伪造结论。所以钉死：

* 前置检查不过 → 退出码 3，且**零运动指令**（执行器与建链都没被调用）；
* 事件链里没有 ``feedback_stream_lost`` → 退出码 4；
* 事件链里有 ``feedback_stream_lost`` → 退出码 0；
* 两种情况下证据的 ``mode`` 取值合法（hardware / offline / fake），
  **假结果不得标成 hardware**。

全部用**假探针 / 假执行器 / 假桥接**注入 → 不连真机、不发任何运动指令。
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import types

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_stream_loss_drill.py")


def _load_script():
    spec = importlib.util.spec_from_file_location("rm75_stream_loss_drill_under_test",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def script():
    return _load_script()


# ------------------------------------------------------------------ 假依赖
JOINTS_DEG = [0.0, 42.0, 19.0, -22.0, -1.0, 5.0, -200.0]


def _probe_ok(**kwargs):
    return {
        "status": "ok",
        "reason": "",
        "phases": [
            {"name": "snapshot", "status": "ok",
             "data": {"snapshot": {"joint_deg": list(JOINTS_DEG)}}},
        ],
    }


def _probe_fail(**kwargs):
    return {"status": "blocked", "reason": "控制器忙", "phases": []}


class FakeExecutor(object):
    """假执行器：记录调用次数，绝不发任何运动指令。"""

    def __init__(self, report=None):
        self.calls = []
        self.report = report if report is not None else {
            "status": "ok", "reason": "", "metrics": {}, "events": []}

    def execute(self, **kwargs):
        self.calls.append(kwargs)
        return json.loads(json.dumps(self.report))


def _install(monkeypatch, probe_result=None, executor=None):
    """注入假探针 / 假执行器 / 假桥接到 sys.modules。"""
    probe = types.ModuleType("rm75_safe_probe")
    probe.run_probe = probe_result or _probe_ok

    connect_calls = []

    def _connect_link(*args, **kwargs):
        connect_calls.append((args, kwargs))
        return None, {"status": "error", "reason": "离线用例不建链"}

    probe._connect_link = _connect_link
    probe.connect_calls = connect_calls
    monkeypatch.setitem(sys.modules, "rm75_safe_probe", probe)

    fake_executor = executor or FakeExecutor()
    cli = types.ModuleType("rm_executor.cli")
    cli.__file__ = os.path.join(TESTS_DIR, "fake_rm_executor_cli.py")
    cli.execute = fake_executor.execute
    monkeypatch.setitem(sys.modules, "rm_executor.cli", cli)

    bridge = types.ModuleType("rm_hw_bridge.bridge")
    monkeypatch.setitem(sys.modules, "rm_hw_bridge.bridge", bridge)
    return fake_executor, probe


# ------------------------------------------------------------------ 退出码语义
def test_precheck_failure_returns_3_with_zero_motion(script, tmp_path, monkeypatch):
    executor, probe = _install(monkeypatch, probe_result=_probe_fail)
    out = tmp_path / "stream_loss.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "0.2", "--cut-after-s", "0.01"])
    assert rc == 3
    assert executor.calls == [], "前置检查不过时执行器一次都不能被调用"
    assert probe.connect_calls == [], "前置检查不过时不得建链"
    assert not out.exists()


def test_missing_stream_lost_event_returns_4(script, tmp_path, monkeypatch):
    executor, _probe = _install(monkeypatch, executor=FakeExecutor({
        "status": "ok", "reason": "", "metrics": {},
        "events": [{"type": "tick"}],
    }))
    out = tmp_path / "stream_loss.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "0.2", "--cut-after-s", "0.01"])
    assert rc == 4
    assert len(executor.calls) == 1
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["stream_lost"] is False
    assert "feedback_stream_lost" not in evidence["events"]


def test_stream_lost_event_returns_0(script, tmp_path, monkeypatch):
    _executor, _probe = _install(monkeypatch, executor=FakeExecutor({
        "status": "error", "reason": "L4 断流停机", "metrics": {},
        "events": [{"type": "feedback_stream_lost", "level": "L4"}],
    }))
    out = tmp_path / "stream_loss.json"
    rc = script.main(["--ws", str(tmp_path), "--out", str(out),
                      "--duration-s", "0.2", "--cut-after-s", "0.01"])
    assert rc == 0
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["stream_lost"] is True
    assert evidence["events"] == ["feedback_stream_lost"]
    assert evidence["stream_lost_evidence"]


def test_evidence_fields_are_complete(script, tmp_path, monkeypatch):
    _install(monkeypatch, executor=FakeExecutor({
        "status": "error", "reason": "L4 断流停机", "metrics": {},
        "events": [{"type": "feedback_stream_lost"}],
    }))
    out = tmp_path / "stream_loss.json"
    script.main(["--ws", str(tmp_path), "--out", str(out),
                 "--duration-s", "0.2", "--cut-after-s", "0.01"])
    evidence = json.loads(out.read_text(encoding="utf-8"))
    for field in ("schema", "mode", "status", "reason", "events", "stream_lost",
                  "wall_s", "cut_result", "params", "expectation"):
        assert field in evidence, "证据缺字段 %s" % field
    assert evidence["schema"] == "rmctl.rm75_stream_loss_drill/1"


# ------------------------------------------------------------------ 诚实性
def test_mode_is_valid_and_not_hardware_for_fake_run(script, tmp_path, monkeypatch):
    """假执行器产出的证据 mode 必须合法，且**不得**是 hardware。"""
    _install(monkeypatch, executor=FakeExecutor({
        "status": "error", "reason": "L4 断流停机", "metrics": {},
        "events": [{"type": "feedback_stream_lost"}],
    }))
    out = tmp_path / "stream_loss.json"
    script.main(["--ws", str(tmp_path), "--out", str(out),
                 "--duration-s", "0.2", "--cut-after-s", "0.01"])
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["mode"] in script.MODES
    assert evidence["mode"] != "hardware"


def test_mode_is_valid_also_without_stream_lost(script, tmp_path, monkeypatch):
    _install(monkeypatch, executor=FakeExecutor({
        "status": "ok", "reason": "", "metrics": {}, "events": []}))
    out = tmp_path / "stream_loss.json"
    script.main(["--ws", str(tmp_path), "--out", str(out),
                 "--duration-s", "0.2", "--cut-after-s", "0.01"])
    evidence = json.loads(out.read_text(encoding="utf-8"))
    assert evidence["mode"] in script.MODES
    assert evidence["mode"] != "hardware"


def test_real_executor_module_is_labelled_hardware(script):
    real_cli = importlib.import_module("rm_executor.cli")
    assert script._evidence_mode(real_cli) == "hardware"


def test_mode_vocabulary_is_explicit(script):
    assert script.MODES == ("hardware", "offline", "fake")
