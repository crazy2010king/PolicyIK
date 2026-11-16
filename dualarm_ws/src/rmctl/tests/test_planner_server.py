"""rm_planner 规划服务端（``rmctl.planner_service.server``）的测试。

覆盖：

* 正常 plan → ``SUCCESS``（含四道校验与 per-node 指标）；
* 非法请求 → ``INVALID_REQUEST``，**服务仍存活**（紧接着一个正常请求成功）；
* 未知 cmd / 非法 JSON → 错误 JSON，服务仍存活；
* **worker 崩溃** → ``FAILED``（含退出码），**服务仍存活**且下一个请求正常；
* 同请求同种子 → 服务端两次结果逐点一致（确定性）；
* socket 路径过长 → 诚实报错（带 next_action），不假装成功。

夹具纪律：模型在 pytest 临时目录里用 ``build_model.build_model()`` 物化
（绝对路径 mesh），规划配置从仓库复制进临时工作区；**不依赖 gitignore 目录**
（``build/``、``.gsd/``）。
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import socket
import sys
import threading
import time

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")
RM_DESC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description")
RM_DESC_SCRIPTS = os.path.join(RM_DESC_SRC, "scripts")
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

for _path in (PKG_SRC, RM_DESC_SCRIPTS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import build_model  # noqa: E402

from rmctl.planner_service.contract import trajectory_digest  # noqa: E402
from rmctl.planner_service.server import (  # noqa: E402
    PlannerServer,
    PlannerServerError,
)

START = [0.0] * 7
GOAL = [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]

#: 一个**必然规划失败**的场景（目标处放一个盒子障碍物 → 规划器找不到解，
#: 会烧满 pipeline.yaml 里的 planning_time_s）。
BLOCKING_SCENE = {
    "obstacles": [
        {"name": "blocker", "box_size": [0.5, 0.5, 0.5], "position": GOAL[:3]},
    ]
}


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_server_ws")
    build_model.build_model(
        ws=str(ws),
        out=str(ws / "build" / "model"),
        pkg=RM_DESC_SRC,
    )
    cfg_dst = ws / "workcell" / "planner"
    cfg_dst.mkdir(parents=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name), str(cfg_dst / name))
    return str(ws)


@pytest.fixture
def server(planner_ws, tmp_path):
    """起在 pytest 临时 socket 上的规划服务。"""
    sock_path = os.path.join(str(tmp_path), "rm_planner.sock")
    srv = PlannerServer(socket_path=sock_path, ws=planner_ws,
                        worker_pool_size=1)
    srv.start()
    try:
        yield srv
    finally:
        srv.stop()


def make_request(plan_id="p-1", **overrides):
    """构造一个合法请求字典（正例底座）。"""
    data = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": list(START),
        "goal": {"type": "joint", "values": list(GOAL)},
        "constraints": {},
        "seed": 7,
        "timeout_ms": 20000,
        "scene_version": 1,
    }
    data.update(overrides)
    return data


# --------------------------------------------------------------------------
# 客户端助手
# --------------------------------------------------------------------------

def send_line(server, payload, timeout=60.0):
    """发一行 JSON → 读回一行 JSON 响应。"""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(server.socket_path)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
        text = buf.decode("utf-8", errors="replace").strip()
        return json.loads(text) if text else {}
    finally:
        sock.close()


def send_raw(server, raw_text, timeout=60.0):
    """发一段**原始文本**（非法 JSON 用例）。"""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(server.socket_path)
        sock.sendall(raw_text.encode("utf-8"))
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
        text = buf.decode("utf-8", errors="replace").strip()
        return json.loads(text) if text else {}
    finally:
        sock.close()


def wait_for_worker(server, plan_id, timeout=15.0):
    """等在飞 worker 出现（返回 pid）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        for item in server.active_workers():
            if item["plan_id"] == plan_id:
                return item["pid"]
        time.sleep(0.05)
    raise AssertionError("在 %.1f s 内没看到在飞 worker（plan_id=%s）"
                         % (timeout, plan_id))


# --------------------------------------------------------------------------
# 用例
# --------------------------------------------------------------------------

def test_plan_success_returns_verification_and_metrics(server):
    """正常规划 → SUCCESS（四道校验 + per-node 指标齐全）。"""
    resp = send_line(server, {"cmd": "plan", "request": make_request("p-ok")})
    assert resp["status"] == "SUCCESS", resp
    assert resp["plan_id"] == "p-ok"
    verification = resp["verification"]
    for key in ("limits", "discrete_collision", "continuous_collision",
                "ok", "issues"):
        assert key in verification, verification
    assert verification["ok"] is True, verification
    metrics = resp["metrics"]
    assert metrics["nodes"], "per-node 指标不能为空"
    assert metrics["seed"] == 7
    assert metrics["pipeline"]
    assert resp["trajectory"]["points"]


def test_same_seed_same_request_is_deterministic(server):
    """同请求同种子 → 两次结果逐点一致（服务端每次都 spawn 独占 worker）。"""
    first = send_line(server, {"cmd": "plan", "request": make_request("p-d1")})
    second = send_line(server, {"cmd": "plan", "request": make_request("p-d2")})
    assert first["status"] == "SUCCESS", first
    assert second["status"] == "SUCCESS", second
    assert trajectory_digest(first["trajectory"]) == \
        trajectory_digest(second["trajectory"])


def test_invalid_request_keeps_server_alive(server):
    """非法请求 → INVALID_REQUEST；**服务仍存活**（下一个请求成功）。"""
    bad = make_request("p-bad")
    del bad["goal"]
    resp = send_line(server, {"cmd": "plan", "request": bad})
    assert resp["status"] == "INVALID_REQUEST", resp
    assert "goal" in resp["failure_reason"], resp
    assert resp["metrics"].get("next_action"), resp
    assert server.running
    good = send_line(server, {"cmd": "plan", "request": make_request("p-after")})
    assert good["status"] == "SUCCESS", good


def test_unknown_command_keeps_server_alive(server):
    """未知 cmd → 错误 JSON；服务仍存活。"""
    resp = send_line(server, {"cmd": "nope"})
    assert resp["status"] == "INVALID_REQUEST", resp
    assert "nope" in resp["failure_reason"], resp
    assert server.running
    status = send_line(server, {"cmd": "status"})
    assert status["status"] == "ok"
    good = send_line(server, {"cmd": "plan", "request": make_request("p-after2")})
    assert good["status"] == "SUCCESS", good


def test_malformed_json_keeps_server_alive(server):
    """非法 JSON → 错误 JSON；服务仍存活。"""
    resp = send_raw(server, "{not json\n")
    assert resp["status"] == "INVALID_REQUEST", resp
    assert server.running
    good = send_line(server, {"cmd": "plan", "request": make_request("p-after3")})
    assert good["status"] == "SUCCESS", good


def test_worker_crash_returns_failed_and_server_survives(server):
    """worker 崩溃 → FAILED（含退出码）；**服务仍存活**且下一个请求正常。"""
    result = {}

    def _run():
        result["resp"] = send_line(
            server,
            {"cmd": "plan",
             "request": make_request("p-crash", timeout_ms=30000,
                                     scene=BLOCKING_SCENE)},
            timeout=60.0,
        )

    thread = threading.Thread(target=_run)
    thread.start()
    pid = wait_for_worker(server, "p-crash")
    time.sleep(0.5)  # 让 worker 进入规划阶段
    os.kill(pid, signal.SIGKILL)
    thread.join(timeout=30.0)
    assert not thread.is_alive()
    resp = result["resp"]
    assert resp["status"] == "FAILED", resp
    assert "退出码" in resp["failure_reason"], resp
    assert server.running
    assert server.worker_pids() == []
    good = send_line(server, {"cmd": "plan", "request": make_request("p-after4")})
    assert good["status"] == "SUCCESS", good


def test_status_reports_counters_and_no_active_worker(server):
    """status → 计数器 + 在飞 worker 列表（诊断入口）。"""
    status = send_line(server, {"cmd": "status"})
    assert status["status"] == "ok"
    assert status["socket_path"] == server.socket_path
    assert status["active_workers"] == []
    counters = status["counters"]
    assert counters["workers_spawned"] == 0
    send_line(server, {"cmd": "plan", "request": make_request("p-c1")})
    status = send_line(server, {"cmd": "status"})
    counters = status["counters"]
    assert counters["workers_spawned"] == 1
    assert counters["workers_reaped"] == 1
    assert counters["success"] == 1
    assert status["active_workers"] == []


def test_socket_path_too_long_fails_honestly(planner_ws):
    """socket 路径过长 → 诚实报错（带 next_action），不假装成功。"""
    long_path = os.path.join("/tmp", "x" * 200, "rm_planner.sock")
    srv = PlannerServer(socket_path=long_path, ws=planner_ws)
    with pytest.raises(PlannerServerError) as excinfo:
        srv.start()
    assert "路径过长" in excinfo.value.reason
    assert excinfo.value.next_action


def test_duplicate_plan_id_is_rejected(server):
    """同一个 plan_id 重复在飞 → 明确拒绝（不静默忽略）。"""
    result = {}

    def _run():
        result["resp"] = send_line(
            server,
            {"cmd": "plan",
             "request": make_request("p-dup", timeout_ms=30000,
                                     scene=BLOCKING_SCENE)},
            timeout=60.0,
        )

    thread = threading.Thread(target=_run)
    thread.start()
    wait_for_worker(server, "p-dup")
    dup = send_line(server, {"cmd": "plan", "request": make_request("p-dup")})
    assert dup["status"] == "INVALID_REQUEST", dup
    assert "已在执行中" in dup["failure_reason"], dup
    # 结束在飞的那个，避免用例悬挂
    send_line(server, {"cmd": "cancel", "plan_id": "p-dup"})
    thread.join(timeout=30.0)
