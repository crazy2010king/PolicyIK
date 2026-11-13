"""``rmctl safety stop|status``（安全闸门客户端）的离线测试（S04 / T7）。

覆盖

* 子命令**存在**且可解析；
* **socket 不存在** → 退出码 **3** + JSON 输出 + **可粘贴 next_action**；
* 正常流程（对着真实起的闸门服务端）→ 退出码 **0** + JSON 输出，
  且服务端确实收到停机请求；
* **非法等级** → 退出码 3 + 明确 reason（不发出请求）。

rmctl 是**客户端**（方向 ``rmctl → rm_runtime``），因此这里起的是
``rm_safety.gate.GateServer`` 服务端。
"""
from __future__ import annotations

import json

import pytest

from rm_safety.gate import GateServer
from rmctl import cli as rmctl_cli


@pytest.fixture()
def gate_server(tmp_path):
    """在 pytest 临时目录起一个真实闸门服务端。"""
    server = GateServer(socket_path=str(tmp_path / "run" / "rm_runtime.sock"))
    server.start()
    try:
        yield server
    finally:
        server.stop()


def _run(argv, capsys):
    """跑一次 rmctl 子命令，返回 ``(退出码, 解析后的 JSON)``。"""
    rc = rmctl_cli.main(argv)
    out = capsys.readouterr().out.strip()
    try:
        payload = json.loads(out)
    except ValueError:
        payload = {}
    return rc, payload


# ------------------------------------------------------------------ 子命令存在
def test_safety_subcommand_exists():
    parser = rmctl_cli.build_parser()
    args = parser.parse_args(["safety", "status"])
    assert args.action == "status"
    args = parser.parse_args(["safety", "stop", "--level", "L3"])
    assert args.action == "stop"
    assert args.level == "L3"


# ------------------------------------------------------------- 负例：socket 缺失
def test_safety_status_missing_socket_exits_3(tmp_path, capsys):
    missing = str(tmp_path / "nope.sock")
    rc, payload = _run(["safety", "status", "--socket", missing], capsys)
    assert rc == 3
    assert payload["status"] == "blocked"
    assert payload["ok"] is False
    assert payload["reason"]
    assert payload["next_action"]


def test_safety_stop_missing_socket_exits_3(tmp_path, capsys):
    missing = str(tmp_path / "nope.sock")
    rc, payload = _run(["safety", "stop", "--socket", missing], capsys)
    assert rc == 3
    assert payload["status"] == "blocked"
    assert payload["next_action"]


def test_safety_stop_invalid_level_exits_3(gate_server, capsys):
    rc, payload = _run(
        ["safety", "stop", "--socket", gate_server.socket_path, "--level", "L9"],
        capsys)
    assert rc == 3
    assert payload["status"] == "blocked"
    assert "L9" in payload["reason"]
    # 非法等级**不会**发出停机请求
    assert gate_server.gate.stop_requested is False


# ----------------------------------------------------------------- 正例：正常流程
def test_safety_status_against_running_gate_exits_0(gate_server, capsys):
    rc, payload = _run(
        ["safety", "status", "--socket", gate_server.socket_path], capsys)
    assert rc == 0
    assert payload["status"] == "ok"
    assert payload["stop_requested"] is False
    assert payload["socket_path"] == gate_server.socket_path


def test_safety_stop_against_running_gate_exits_0(gate_server, capsys):
    rc, payload = _run(
        ["safety", "stop", "--socket", gate_server.socket_path], capsys)
    assert rc == 0
    assert payload["status"] == "ok"
    assert payload["level"] == "L4"
    # 服务端确实收到停机请求
    assert gate_server.gate.stop_requested is True
    assert gate_server.gate.requested_level == "L4"


def test_safety_stop_with_explicit_level(gate_server, capsys):
    rc, payload = _run(
        ["safety", "stop", "--socket", gate_server.socket_path, "--level", "L2"],
        capsys)
    assert rc == 0
    assert payload["level"] == "L2"
    assert gate_server.gate.requested_level == "L2"
