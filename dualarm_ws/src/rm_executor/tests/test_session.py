"""rm_executor 真机会话层测试（S06/T5）。

覆盖契约：

* :meth:`LiveSession.open` 在**硬件层不可用 / 未提供 ip / 建链异常 /
  未连接** 四种情况下 → **error 报告**（status=error、reason 含
  「控制器不可达」、**指标全 None**、next_action 非空），**不抛裸异常**；
* :meth:`LiveSession.open` **启动反馈泵**（会话期间 ``running is True``，
  关闭后为 False）—— 这是 S03 遗留缺陷的修复点；
* ``close()`` **幂等**；
* ``gripper_port()`` 与机械臂**共用同一条连接**；硬件层不可用 → None +
  证据，**不抛裸异常**；
* live 路径**拒绝** ``allow_unverified=True`` → blocked + 明确 reason；
* :class:`FakeSession` 跑一条合法轨迹 → status=ok、顶层 7 键齐全；
* ``rm_executor.cli._run_live`` **复用**会话层（行为等价：仍是退出码 4）。
"""
from __future__ import annotations

import ast
import os
import sys

import pytest

from rm_executor import cli as exec_cli
from rm_executor import contract
from rm_executor import report as report_mod
from rm_executor import session as session_mod
from rm_executor.feedback import FeedbackPump, empty_snapshot
from rm_executor.session import FakeSession, LiveSession

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))


# --------------------------------------------------------------------- 夹具
class _FakeSource:
    """假的反馈源（只暴露 ``latest()``，**不碰真机**）。"""

    name = "fake-source"

    def __init__(self) -> None:
        self.calls = 0

    def latest(self):
        self.calls += 1
        snap = empty_snapshot()
        snap["frames"] = self.calls
        return snap


class _FakeLink:
    """假的 ``ArmLink``（鸭子类型）：只提供会话层用到的属性。"""

    def __init__(self, connected: bool = True, reason: str = "",
                 next_action: str = "", evidence=None) -> None:
        self.connected = connected
        self.reason = reason
        self.next_action = next_action
        self.evidence = list(evidence or [])
        self.handle = object()
        self.bridge = type("_B", (), {"binding": object()})()
        self.closed = False

    def close(self) -> None:
        self.closed = True


def _payload(verify=True):
    return {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": [0.0, 0.05],
        "points": [[0.0] * 7, [0.05] * 7],
        "verify": {"ok": bool(verify), "checks": {}},
    }


#: **实测类指标**（不可达时一律 None，绝不返回 0 冒充）
_MEASURED_METRICS = (
    "tick_jitter_p99_ms", "tick_jitter_max_ms", "follow_err_rms_deg",
    "follow_err_max_deg", "arrival_err_deg", "arrival_hold_ms",
    "duration_s", "rt_factor", "planned_duration_s", "pre_position_hold_ms",
)


def _metrics_all_none(rep) -> bool:
    """报告里**没有任何伪造数值**（实测类指标一律 None，ticks 为 0）。"""
    metrics = rep.get("metrics") or {}
    for key in _MEASURED_METRICS:
        assert metrics.get(key) is None, "%s 不应被伪造" % key
    return metrics.get("ticks") == 0


# ------------------------------------------------------------ open 失败路径
def test_open_without_ip_returns_unreachable_error_report():
    """未提供 ip → error 报告（不可达语义 + 零伪造数值），**不抛裸异常**。"""
    session, err = LiveSession.open(arm="left", ip=None)
    assert session is None
    assert err is not None
    assert err["status"] == contract.STATUS_ERROR
    assert "控制器不可达" in err["reason"]
    assert err["next_action"]
    assert _metrics_all_none(err)
    assert err["mode"] == "live"
    assert err["events"][0]["type"] == "controller_unreachable"


def test_open_hardware_layer_unavailable_returns_error_report(monkeypatch):
    """硬件层不可用 → error 报告（**不抛裸异常**）。"""
    monkeypatch.setitem(sys.modules, "rm_hw_bridge", None)
    session, err = LiveSession.open(arm="left", ip="127.0.0.1")
    assert session is None
    assert err is not None
    assert err["status"] == contract.STATUS_ERROR
    assert "控制器不可达" in err["reason"]
    assert "rm_hw_bridge" in err["reason"]
    assert err["next_action"]
    assert _metrics_all_none(err)


def test_open_connect_exception_returns_error_report(monkeypatch):
    """建链异常 → error 报告（**不抛裸异常**）。"""
    import rm_hw_bridge.bridge as hw_bridge

    def _boom(*_args, **_kwargs):
        raise RuntimeError("no route to host")

    monkeypatch.setattr(hw_bridge, "connect", _boom)
    session, err = LiveSession.open(arm="left", ip="127.0.0.1", port=59999)
    assert session is None
    assert err["status"] == contract.STATUS_ERROR
    assert "控制器不可达" in err["reason"]
    assert "no route to host" in err["reason"]
    assert err["next_action"]
    assert _metrics_all_none(err)


def test_open_link_not_connected_returns_error_report(monkeypatch):
    """``link.connected is False`` → error 报告（沿用 link 的 reason/next_action）。"""
    import rm_hw_bridge.bridge as hw_bridge

    link = _FakeLink(connected=False, reason="控制器未上电",
                     next_action="给控制器上电后重跑", evidence=["ping 超时"])
    monkeypatch.setattr(hw_bridge, "connect", lambda *a, **k: link)

    session, err = LiveSession.open(arm="left", ip="127.0.0.1", port=59999)
    assert session is None
    assert err["status"] == contract.STATUS_ERROR
    assert "控制器不可达" in err["reason"]
    assert "控制器未上电" in err["reason"]
    assert err["next_action"] == "给控制器上电后重跑"
    assert "ArmLink.connected = False" in err["evidence"]
    assert _metrics_all_none(err)


# ------------------------------------------------------------ 反馈泵与生命周期
def test_open_starts_feedback_pump_and_close_stops_it():
    """**反馈泵修复**：会话期间 running=True，关闭后 False。"""
    pump = FeedbackPump(_FakeSource(), period_s=0.005)
    assert pump.running is False

    session, err = LiveSession.open(link=_FakeLink(), feedback=pump)
    assert err is None and session is not None
    assert pump.running is True

    session.close()
    assert pump.running is False


def test_close_is_idempotent():
    """``close()`` 幂等（调用两次不抛异常，连接只关一次）。"""
    link = _FakeLink()
    pump = FeedbackPump(_FakeSource(), period_s=0.005)
    session, err = LiveSession.open(link=link, feedback=pump)
    assert err is None

    session.close()
    session.close()  # 幂等：不抛异常
    assert link.closed is True
    assert session.closed is True


def test_open_stops_and_reports_feedback_start_failure():
    """反馈泵启动失败 → error 报告（诚实降级，不假装在跑）。"""

    class _BrokenPump:
        def start(self):
            raise RuntimeError("cannot start thread")

        def stop(self):
            pass

    session, err = LiveSession.open(link=_FakeLink(), feedback=_BrokenPump())
    assert session is None
    assert err is not None
    assert err["status"] == contract.STATUS_ERROR
    assert "反馈泵" in err["reason"]
    assert _metrics_all_none(err)


# ------------------------------------------------------------------ 夹爪端口
def test_gripper_port_shares_the_arm_connection():
    """``gripper_port()`` 与机械臂**共用同一条连接**（binding + handle）。"""
    from rm_hw_bridge.gripper import CtypesSdkPort

    link = _FakeLink()
    session, err = LiveSession.open(link=link,
                                    feedback=FeedbackPump(_FakeSource(), 0.005))
    assert err is None
    port = session.gripper_port()
    assert isinstance(port, CtypesSdkPort)
    assert port.handle is link.handle
    assert port.binding is link.bridge.binding
    session.close()


def test_gripper_port_returns_none_without_raising_when_unavailable(monkeypatch):
    """硬件层不可用 → ``None`` + 证据，**不抛裸异常**。"""
    import rm_hw_bridge

    session, err = LiveSession.open(link=_FakeLink(),
                                    feedback=FeedbackPump(_FakeSource(), 0.005))
    assert err is None
    monkeypatch.setitem(sys.modules, "rm_hw_bridge.gripper", None)
    monkeypatch.delattr(rm_hw_bridge, "gripper", raising=False)
    try:
        port = session.gripper_port()
    finally:
        session.close()
    assert port is None
    assert session.gripper_port_evidence


def test_gripper_port_without_link_returns_none_with_evidence():
    """未建链 → ``None`` + 证据，**不抛裸异常**。"""
    session = LiveSession()
    assert session.gripper_port() is None
    assert session.gripper_port_evidence


# ------------------------------------------------------------------ 执行路径
def test_live_session_rejects_allow_unverified(tmp_path, write_traj):
    """live 路径拒绝未校验轨迹 → blocked + 明确 reason。"""
    traj = write_traj(_payload(verify=False))
    session, err = LiveSession.open(link=_FakeLink(),
                                    feedback=FeedbackPump(_FakeSource(), 0.005))
    assert err is None
    try:
        rep = session.run_trajectory(traj, allow_unverified=True)
    finally:
        session.close()
    assert rep["status"] == contract.STATUS_BLOCKED
    assert "未校验轨迹" in rep["reason"]
    assert rep["next_action"]
    assert _metrics_all_none(rep)


def test_fake_session_runs_valid_trajectory_with_same_schema(tmp_path, write_traj,
                                                             repo_root):
    """离线等价物：跑一条合法轨迹 → status=ok + 顶层 7 键齐全。"""
    traj = write_traj(_payload())
    session = FakeSession(ws=repo_root)
    rep = session.run_trajectory(traj)
    assert rep["status"] == contract.STATUS_OK, rep
    assert set(rep.keys()) == set(report_mod.TOP_LEVEL_KEYS)
    assert len(report_mod.TOP_LEVEL_KEYS) == 7
    assert rep["mode"] == "fake"
    assert report_mod.validate_report(rep) == []
    session.close()


def test_fake_session_close_is_idempotent():
    """离线会话 close 幂等。"""
    session = FakeSession()
    session.close()
    session.close()
    assert session.closed is True


# --------------------------------------------------------------- CLI 复用回归
def test_run_live_reuses_live_session(monkeypatch):
    """``_run_live`` **复用** ``LiveSession``（open → run → close）。"""
    calls = {"open": 0, "run": 0, "close": 0}

    class _StubSession:
        def __init__(self):
            self.closed = False

        @classmethod
        def open(cls, **_kwargs):
            calls["open"] += 1
            return cls(), None

        def run_trajectory(self, _path, allow_unverified=False, **_kwargs):
            calls["run"] += 1
            assert allow_unverified is False
            return {"status": "ok", "reason": "", "next_action": "",
                    "evidence": [], "mode": "live", "metrics": {}, "events": []}

        def close(self):
            calls["close"] += 1
            self.closed = True

    monkeypatch.setattr(exec_cli, "LiveSession", _StubSession)
    rep = exec_cli._run_live("traj.json", None, "left", "127.0.0.1", 59999,
                             None, None, None)
    assert calls == {"open": 1, "run": 1, "close": 1}
    assert rep["status"] == "ok"


def test_cli_live_unreachable_still_exits_4(tmp_path, write_traj, capsys):
    """既有真机不可达用例仍是**退出码 4**（复用会话后行为等价）。"""
    traj = write_traj(_payload())
    out = str(tmp_path / "report.json")
    rc = exec_cli.main(["--traj", traj, "--mode", "live", "--ip", "127.0.0.1",
                         "--port", "59999", "--report", out])
    assert rc == 4, capsys.readouterr().out
    with open(out, "r", encoding="utf-8") as fh:
        import json
        rep = json.load(fh)
    assert rep["status"] == "error"
    assert "控制器不可达" in rep["reason"]
    assert _metrics_all_none(rep)


# ------------------------------------------------------------------ 红线守卫
def test_session_module_has_no_rmctl_or_tesseract_dependency():
    """执行层红线：``session.py`` 零 rmctl / 零 tesseract 依赖（AST 判定）。"""
    path = os.path.join(TESTS_DIR, "..", "rm_executor", "session.py")
    with open(path, "r", encoding="utf-8") as fh:
        tree = ast.parse(fh.read(), filename=path)
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        for name in names:
            root = name.split(".")[0]
            assert root not in ("rmctl", "tesseract_robotics", "tesseract"), name
