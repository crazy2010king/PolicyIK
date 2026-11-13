"""安全闸门（进程外强制停机通道）的离线测试（S04 / T7）。

覆盖

* **进程内状态**：停机请求置标志 + 记等级；等级**单调**（L4 不会被 L2 覆盖）；
* **停机回调**：注册一次、异常被记录（不掀翻服务端）；
* **socket 协议**：stop / status 请求被正确接收；**非法 JSON 被拒**、
  **非法等级被拒**、未知命令被拒（都回一行错误 JSON）；
* **客户端断开**不导致服务端异常；
* **诚实报错**：socket 路径被占用 → :class:`GateError` + 可粘贴 next_action；
* **默认路径**：``build/run/rm_runtime.sock``（工作区根下）。

全部跑在 pytest 临时目录（``tmp_path``），不依赖 ``build/`` 等 gitignore 目录。
"""
from __future__ import annotations

import json
import os
import socket
import threading

import pytest

from rm_safety.gate import (
    CMD_STATUS,
    CMD_STOP,
    DEFAULT_SOCKET_RELPATH,
    GateError,
    GateServer,
    InProcessGate,
    UnixSocketGate,
    default_socket_path,
    handle_gate_request,
)


# --------------------------------------------------------------------- 工具
def _request(sock_path, payload, timeout=3.0):
    """向闸门发一行 JSON 并读一行响应。"""
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout)
    try:
        client.connect(sock_path)
        client.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        data = client.recv(65536)
    finally:
        client.close()
    return json.loads(data.decode("utf-8").strip())


def _raw_request(sock_path, raw, timeout=3.0):
    """发**原始文本**并读回响应（用于非法 JSON 负例）。"""
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout)
    try:
        client.connect(sock_path)
        client.sendall(raw.encode("utf-8"))
        data = client.recv(65536)
    finally:
        client.close()
    return json.loads(data.decode("utf-8").strip())


# ------------------------------------------------------- 进程内状态（纯逻辑）
def test_request_stop_sets_flag_and_level():
    gate = InProcessGate()
    assert gate.stop_requested is False
    assert gate.requested_level == ""

    result = gate.request_stop(level="L4", reason="外部闸门请求", source="socket")

    assert result["status"] == "ok"
    assert gate.stop_requested is True
    assert gate.requested_level == "L4"
    assert gate.request_count == 1
    assert gate.reason == "外部闸门请求"


def test_request_stop_rejects_unknown_level():
    gate = InProcessGate()
    with pytest.raises(ValueError):
        gate.request_stop(level="L9")
    assert gate.stop_requested is False


def test_request_stop_level_is_monotonic():
    """等级单调：先 L4 再 L2 → 仍是 L4（不被更轻的请求覆盖）。"""
    gate = InProcessGate()
    gate.request_stop(level="L4")
    gate.request_stop(level="L2")
    assert gate.requested_level == "L4"
    assert gate.request_count == 2


def test_handler_is_called_once_per_request():
    calls = []
    gate = InProcessGate(on_stop=lambda level, reason, source: calls.append(
        (level, reason, source)))
    gate.request_stop(level="L2", reason="r", source="socket")
    assert calls == [("L2", "r", "socket")]


def test_handler_exception_is_recorded_not_raised():
    def _boom(level, reason, source):
        raise RuntimeError("boom")

    gate = InProcessGate(on_stop=_boom)
    gate.request_stop(level="L4")  # 不抛
    assert gate.handler_errors
    assert "boom" in gate.handler_errors[0]


def test_add_handler_is_idempotent():
    calls = []
    gate = InProcessGate()

    def _h(level, reason, source):
        calls.append(level)

    gate.add_handler(_h)
    gate.add_handler(_h)
    gate.request_stop(level="L4")
    assert calls == ["L4"]


def test_clear_resets_state():
    gate = InProcessGate()
    gate.request_stop(level="L4")
    gate.clear()
    assert gate.stop_requested is False
    assert gate.requested_level == ""


# ------------------------------------------------------------- 协议（纯函数）
def test_handle_request_status_ok():
    gate = InProcessGate()
    resp = handle_gate_request(gate, json.dumps({"cmd": "status"}))
    assert resp["status"] == "ok"
    assert resp["cmd"] == CMD_STATUS
    assert resp["stop_requested"] is False


def test_handle_request_stop_defaults_to_l4():
    gate = InProcessGate()
    resp = handle_gate_request(gate, json.dumps({"cmd": "stop"}))
    assert resp["status"] == "ok"
    assert resp["level"] == "L4"
    assert gate.stop_requested is True


def test_handle_request_invalid_json_is_rejected():
    gate = InProcessGate()
    resp = handle_gate_request(gate, "not-json")
    assert resp["status"] == "error"
    assert resp["next_action"]
    assert gate.stop_requested is False


def test_handle_request_invalid_level_is_rejected():
    gate = InProcessGate()
    resp = handle_gate_request(gate, json.dumps({"cmd": "stop", "level": "L9"}))
    assert resp["status"] == "error"
    assert "L9" in resp["reason"]
    assert gate.stop_requested is False


def test_handle_request_unknown_cmd_is_rejected():
    gate = InProcessGate()
    resp = handle_gate_request(gate, json.dumps({"cmd": "reboot"}))
    assert resp["status"] == "error"
    assert gate.stop_requested is False


def test_handle_request_empty_is_rejected():
    gate = InProcessGate()
    resp = handle_gate_request(gate, "")
    assert resp["status"] == "error"


# ------------------------------------------------------------- socket 服务端
@pytest.fixture()
def gate_server(tmp_path):
    """在 pytest 临时目录起一个闸门服务端。"""
    server = GateServer(socket_path=str(tmp_path / "run" / "rm_runtime.sock"))
    server.start()
    try:
        yield server
    finally:
        server.stop()


def test_gate_server_start_stop(gate_server):
    assert os.path.exists(gate_server.socket_path)
    assert gate_server.running is True
    gate_server.stop()
    assert gate_server.running is False
    assert not os.path.exists(gate_server.socket_path)


def test_socket_stop_request_is_received(gate_server):
    resp = _request(gate_server.socket_path, {"cmd": "stop", "level": "L4"})
    assert resp["status"] == "ok"
    assert resp["level"] == "L4"
    assert gate_server.gate.stop_requested is True
    assert gate_server.gate.requested_level == "L4"


def test_socket_status_request_reports_state(gate_server):
    resp = _request(gate_server.socket_path, {"cmd": "status"})
    assert resp["status"] == "ok"
    assert resp["stop_requested"] is False
    assert resp["socket_path"] == gate_server.socket_path


def test_socket_invalid_json_is_rejected(gate_server):
    resp = _raw_request(gate_server.socket_path, "{not json}\n")
    assert resp["status"] == "error"
    assert resp["next_action"]
    assert gate_server.gate.stop_requested is False


def test_socket_invalid_level_is_rejected(gate_server):
    resp = _request(gate_server.socket_path, {"cmd": "stop", "level": "L7"})
    assert resp["status"] == "error"
    assert gate_server.gate.stop_requested is False


def test_socket_client_disconnect_does_not_break_server(gate_server):
    """客户端连上就断开（不发数据）→ 服务端不异常、仍能服务下一个请求。"""
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(3.0)
    client.connect(gate_server.socket_path)
    client.close()

    resp = _request(gate_server.socket_path, {"cmd": "status"})
    assert resp["status"] == "ok"
    assert gate_server.gate.server_errors == []


def test_socket_stop_triggers_handler(gate_server):
    """闸门请求 → 停机回调被调用（执行器就是通过它立即停机）。"""
    calls = []
    gate_server.gate.add_handler(
        lambda level, reason, source: calls.append((level, reason, source)))
    resp = _request(gate_server.socket_path, {"cmd": "stop", "level": "L3"})
    assert resp["status"] == "ok"
    assert calls and calls[0][0] == "L3"


def test_socket_path_occupied_raises_gate_error(tmp_path):
    """路径被占用 → 诚实报错（GateError + 可粘贴 next_action），不假装成功。"""
    path = str(tmp_path / "busy.sock")
    first = GateServer(socket_path=path)
    first.start()
    try:
        second = GateServer(socket_path=path)
        with pytest.raises(GateError) as excinfo:
            second.start()
        assert "已被占用" in excinfo.value.reason
        assert excinfo.value.next_action
    finally:
        first.stop()


def test_socket_path_too_long_raises_gate_error(tmp_path):
    long_path = str(tmp_path / ("x" * 200) / "rm.sock")
    server = GateServer(socket_path=long_path)
    with pytest.raises(GateError) as excinfo:
        server.start()
    assert "路径过长" in excinfo.value.reason


def test_stale_socket_file_is_replaced(tmp_path):
    """上次进程异常退出留下的陈旧 socket 文件 → 自动清理后重建。"""
    path = str(tmp_path / "stale.sock")
    stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stale.bind(path)
    stale.close()  # 留下文件但没人监听

    server = GateServer(socket_path=path)
    server.start()
    try:
        assert server.running is True
        resp = _request(path, {"cmd": "status"})
        assert resp["status"] == "ok"
    finally:
        server.stop()


# ------------------------------------------------------------------- 默认路径
def test_default_socket_path_is_under_build_run():
    path = default_socket_path()
    assert path.endswith(DEFAULT_SOCKET_RELPATH.replace(os.sep, "/"))
    assert "build" in path and "run" in path
