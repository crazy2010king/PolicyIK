"""rm_planner 规划服务**薄客户端**（``rmctl.planner_service.client``）与
``rmctl plan --via-service`` 接线的测试。

覆盖：

* 客户端对着**真实起起来的 server**（临时 socket）plan 一个简单请求 →
  ``SUCCESS``（trajectory / verification / metrics 字段齐）；
* **服务未启动** → 结构化错误（reason 含 socket 路径、next_action 非空），
  **不抛裸异常**；
* ``cancel()`` / ``status()`` 可用（status 返回服务存活信息）；
* 客户端**读超时** → 结构化 ``TIMEOUT``（不是裸异常）；
* 客户端**本地契约校验** → 结构化 ``INVALID_REQUEST``（且**不打扰服务端**）；
* ``raise_for_status()``：成功原样返回、失败抛 ``PlannerClientError``；
* CLI ``--via-service``：打到真实服务 → 输出四道校验 + 节点耗时 + 退出码 0；
* CLI ``--via-service`` 与**进程内模式**的关键字段一致（status 相同、
  点数相同、verification 四道结论相同）——在**同一条轨迹**上比较
  （规划器自身的随机性由种子 / 规划器个数决定，**不是 CLI 接线的职责**）；
* CLI ``--via-service`` 在服务缺席时 → 退出码 1 + 可粘贴下一步。

夹具纪律：模型在 pytest 临时目录里用 ``build_model.build_model()`` 物化
（绝对路径 mesh）；规划配置从仓库复制进临时工作区；**不依赖 gitignore 目录**
（``build/``、``.gsd/``）。
"""
from __future__ import annotations

import os
import shutil
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

from rmctl.cli import main as cli_main  # noqa: E402
from rmctl.planner_service.client import (  # noqa: E402
    PlannerClient,
    PlannerClientError,
    raise_for_status,
)
from rmctl.planner_service.server import PlannerServer  # noqa: E402

START = [0.0] * 7
GOAL = [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]

#: 一个**必然规划失败**的场景（目标处放盒子障碍物 → 规划器烧满
#: pipeline.yaml 的 planning_time_s）——用来验证 cancel 真的能中断。
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
    ws = tmp_path_factory.mktemp("planner_client_ws")
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
    srv = PlannerServer(socket_path=sock_path, ws=planner_ws, worker_pool_size=1)
    srv.start()
    try:
        yield srv
    finally:
        srv.stop()


@pytest.fixture
def client(server, planner_ws):
    """对着上面那个真实服务的客户端。"""
    return PlannerClient(socket_path=server.socket_path, ws=planner_ws)


def make_request(plan_id="p-1", **overrides):
    """构造一个合法请求字典（正例底座；与 CLI 服务模式发的一致）。"""
    data = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": list(START),
        "goal": {"type": "joint", "values": list(GOAL)},
        "constraints": {},
        "seed": 0,
        "timeout_ms": 20000,
        "scene_version": 1,
        "deterministic": True,
    }
    data.update(overrides)
    return data


# --------------------------------------------------------------------------
# 客户端：正例
# --------------------------------------------------------------------------

def test_client_plans_against_real_server(client):
    """对着真实服务 plan → SUCCESS，字段齐（trajectory/verification/metrics）。"""
    resp = client.plan(make_request("p-ok"))
    assert resp["status"] == "SUCCESS", resp
    assert resp["plan_id"] == "p-ok"
    trajectory = resp["trajectory"]
    assert isinstance(trajectory, dict)
    assert len(trajectory["points"]) > 0
    assert len(trajectory["times"]) == len(trajectory["points"])
    verification = resp["verification"]
    for key in ("limits", "discrete_collision", "continuous_collision"):
        assert key in verification
        assert verification[key]["ok"] is True, verification
    assert verification["ok"] is True
    metrics = resp["metrics"]
    assert metrics["pipeline"] == "FreespacePipeline"
    assert metrics["seed"] == 0
    assert metrics["points"] == len(trajectory["points"])
    assert metrics["nodes"], "per-node 指标应随响应返回"
    assert all("elapsed_s" in node for node in metrics["nodes"])
    assert resp["failure_reason"] == ""


def test_client_plan_does_not_wait_for_worker_shutdown(client):
    """单次 plan 的墙钟时间不得被 worker 收尾拖住（回归防线）。

    服务端收到响应后必须**请 worker 收工**（关 stdin）再回收；否则
    ``reap()`` 会一直等到 5 s 超时，每条 plan 白等 5 s（历史上整包测试
    因此超时）。
    """
    t0 = time.time()
    resp = client.plan(make_request("p-fast"))
    elapsed = time.time() - t0
    assert resp["status"] == "SUCCESS", resp
    assert elapsed < 3.0, "plan 墙钟 %.2f s 太慢（worker 收尾在干等）" % elapsed


def test_client_fills_plan_id_and_timeout(client):
    """客户端负责补 plan_id（缺省生成）与 timeout_ms（缺省值）。"""
    request = make_request("p-x")
    del request["plan_id"]
    del request["timeout_ms"]
    resp = client.plan(request)
    assert resp["status"] == "SUCCESS", resp
    assert resp["plan_id"].startswith("plan-")


def test_client_status_reports_liveness(client, server):
    """status() 返回服务存活信息（running / 计数器 / socket 路径）。"""
    info = client.status()
    assert info.get("status") == "ok"
    assert info.get("running") is True
    assert info["socket_path"] == server.socket_path
    assert "counters" in info and "requests_total" in info["counters"]


def test_client_cancel_unknown_plan_is_structured(client):
    """cancel 一个不存在的 plan_id → 结构化错误（不抛异常）。"""
    resp = client.cancel("no-such-plan")
    assert resp.get("status") == "error"
    assert "no-such-plan" in resp.get("reason", "")
    assert resp.get("next_action")


def test_client_cancel_interrupts_inflight_plan(client):
    """在飞的 plan 可以被客户端 cancel（服务端 SIGKILL worker → CANCELLED）。"""
    request = make_request("p-cancel", scene=BLOCKING_SCENE, timeout_ms=20000)
    result = {}

    def _run():
        result["resp"] = client.plan(request)

    thread = threading.Thread(target=_run)
    thread.start()
    # 等 worker 真正开始规划（服务端的 status 里有在飞 worker）。
    deadline = time.time() + 20.0
    while time.time() < deadline:
        info = client.status()
        if info.get("active_workers"):
            break
        time.sleep(0.2)
    cancel = client.cancel("p-cancel")
    assert cancel.get("status") == "ok", cancel
    thread.join(timeout=60)
    assert not thread.is_alive()
    assert result["resp"]["status"] == "CANCELLED", result["resp"]


# --------------------------------------------------------------------------
# 客户端：负例（失败路径一律结构化，不抛裸异常）
# --------------------------------------------------------------------------

def test_client_reports_structured_error_when_server_absent(planner_ws, tmp_path):
    """服务未启动 → 结构化错误（reason 含 socket 路径、next_action 非空）。"""
    missing = os.path.join(str(tmp_path), "not-there.sock")
    client = PlannerClient(socket_path=missing, ws=planner_ws,
                           connect_timeout_s=1.0)
    resp = client.plan(make_request("p-none"))
    assert resp["status"] == "FAILED"
    assert resp["trajectory"] is None
    assert missing in resp["failure_reason"]
    assert resp["metrics"]["next_action"]
    assert "rm-planner" in resp["metrics"]["next_action"]
    # status() / cancel() 同样不抛异常。
    assert client.status()["status"] == "FAILED"
    assert client.cancel("p-none")["status"] == "FAILED"


def test_client_read_timeout_is_structured(planner_ws, tmp_path):
    """服务端连上但不回话 → 读超时 → 结构化 TIMEOUT（不是裸异常）。"""
    sock_path = os.path.join(str(tmp_path), "silent.sock")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(sock_path)
    listener.listen(1)

    def _accept():
        try:
            conn, _ = listener.accept()
            time.sleep(5.0)                # 故意不回应
        except OSError:
            pass

    thread = threading.Thread(target=_accept, daemon=True)
    thread.start()
    try:
        client = PlannerClient(socket_path=sock_path, ws=planner_ws,
                               timeout_ms=100, read_grace_s=0.3)
        # 请求里的 timeout_ms 也给 100 ms → 客户端读超时 = 0.1 + 0.3 s。
        resp = client.plan(make_request("p-slow", timeout_ms=100))
        assert resp["status"] == "TIMEOUT", resp
        assert resp["metrics"]["next_action"]
        assert resp["metrics"]["error"] == "read_timeout"
    finally:
        listener.close()


def test_client_rejects_invalid_request_locally(client, server):
    """非法请求 → 结构化 INVALID_REQUEST（逐字段指名），且不打扰服务端。"""
    before = server.counters["requests_total"]
    resp = client.plan(make_request("p-bad", start_state=[0.0] * 6))
    assert resp["status"] == "INVALID_REQUEST", resp
    assert "start_state" in resp["failure_reason"]
    assert resp["metrics"]["next_action"]
    assert server.counters["requests_total"] == before


def test_client_treats_malformed_response_as_failure(planner_ws, tmp_path):
    """服务端回的不是合法 JSON → 结构化 FAILED（不抛裸异常）。"""
    sock_path = os.path.join(str(tmp_path), "garbage.sock")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(sock_path)
    listener.listen(1)

    def _serve():
        try:
            conn, _ = listener.accept()
            conn.recv(65536)
            conn.sendall(b"not-json-at-all\n")
            conn.close()
        except OSError:
            pass

    thread = threading.Thread(target=_serve, daemon=True)
    thread.start()
    try:
        client = PlannerClient(socket_path=sock_path, ws=planner_ws)
        resp = client.plan(make_request("p-garbage"))
        assert resp["status"] == "FAILED", resp
        assert resp["metrics"]["error"] == "malformed_response"
        assert resp["metrics"]["next_action"]
    finally:
        listener.close()


def test_client_treats_closed_connection_as_failure(planner_ws, tmp_path):
    """服务端连上后直接断开（连接丢失） → 结构化 FAILED + 可粘贴下一步。"""
    sock_path = os.path.join(str(tmp_path), "hangup.sock")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(sock_path)
    listener.listen(1)

    def _serve():
        try:
            conn, _ = listener.accept()
            conn.recv(65536)
            conn.close()               # 不回任何东西就断
        except OSError:
            pass

    thread = threading.Thread(target=_serve, daemon=True)
    thread.start()
    try:
        client = PlannerClient(socket_path=sock_path, ws=planner_ws)
        resp = client.plan(make_request("p-hangup"))
        assert resp["status"] == "FAILED", resp
        assert resp["metrics"]["error"] == "empty_response"
        assert resp["metrics"]["next_action"]
    finally:
        listener.close()


def test_raise_for_status_raises_only_on_failure(client):
    """raise_for_status：成功原样返回、失败抛 PlannerClientError。"""
    resp = client.plan(make_request("p-raise"))
    assert resp["status"] == "SUCCESS"
    assert raise_for_status(resp) is resp
    with pytest.raises(PlannerClientError) as excinfo:
        raise_for_status({"status": "FAILED", "failure_reason": "规划失败"})
    assert "规划失败" in str(excinfo.value)


# --------------------------------------------------------------------------
# CLI：`rmctl plan --via-service`
# --------------------------------------------------------------------------

def _parse_service_output(text):
    """把 CLI 服务模式输出解析成可比对的字段（与进程内模式同构）。"""
    lines = [line for line in text.splitlines() if line.strip()]
    header = lines[0] if lines else ""
    checks = {}
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[PASS] ") or stripped.startswith("[FAIL] "):
            verdict = stripped[1:5]
            name = stripped[6:].strip()
            checks[name] = verdict
    return {
        "header": header,
        "checks": checks,
        "text": text,
    }


def test_cli_via_service_plans_and_reports(server, planner_ws, capsys):
    """CLI --via-service 打到真实服务 → 四道校验 + 节点耗时 + 退出码 0。"""
    code = cli_main([
        "--ws", planner_ws,
        "plan", "--via-service",
        "--socket", server.socket_path,
        "--goal", ",".join(str(v) for v in GOAL),
    ])
    out = capsys.readouterr().out
    assert code == 0, out
    parsed = _parse_service_output(out)
    assert "成功=True" in parsed["header"]
    assert "管线 FreespacePipeline" in parsed["header"]
    assert "四道校验: 全部通过" in parsed["text"]
    assert parsed["checks"] == {
        "limits": "PASS",
        "discrete_collision": "PASS",
        "continuous_collision": "PASS",
    }
    assert "节点耗时:" in parsed["text"]


def test_cli_via_service_matches_inproc_on_same_trajectory(
    server, planner_ws, capsys, monkeypatch
):
    """服务模式与进程内模式关键字段一致（status / 点数 / 四道校验结论）。

    比较在**同一条轨迹**上做：服务模式用真实服务返回的轨迹，进程内模式
    用被替换成同一条轨迹的规划器（规划器自身的随机性由种子与规划器个数
    决定，不是 CLI 接线的职责）。
    """
    client = PlannerClient(socket_path=server.socket_path, ws=planner_ws)
    resp = client.plan(make_request("p-cmp"))
    assert resp["status"] == "SUCCESS", resp
    trajectory = resp["trajectory"]
    points = [list(item["positions"]) for item in trajectory["points"]]
    times = [float(t) for t in trajectory["times"]]

    import rmctl.planner as rmctl_planner

    def _fake_plan_joint(model, start, goal, pipeline="FreespacePipeline",
                         profiles=None, tip=None):
        from rmctl.planner import PlanOutcome

        return PlanOutcome(True, "", times, points, 0.0, pipeline)

    monkeypatch.setattr(rmctl_planner, "plan_joint", _fake_plan_joint)

    code = cli_main([
        "--ws", planner_ws,
        "plan", "--via-service",
        "--socket", server.socket_path,
        "--goal", ",".join(str(v) for v in GOAL),
    ])
    service_out = capsys.readouterr().out
    assert code == 0, service_out
    service = _parse_service_output(service_out)

    code = cli_main([
        "--ws", planner_ws,
        "plan",
        "--goal", ",".join(str(v) for v in GOAL),
    ])
    inproc_out = capsys.readouterr().out
    assert code == 0, inproc_out
    inproc = _parse_service_output(inproc_out)

    # status 相同
    assert ("成功=True" in service["header"]) == ("成功=True" in inproc["header"])
    # 点数相同
    assert service["header"].split("点数 ")[-1] == inproc["header"].split("点数 ")[-1]
    assert int(service["header"].split("点数 ")[-1]) == len(points)
    # 四道校验结论相同
    assert service["checks"] == inproc["checks"]
    assert inproc["checks"] == {
        "limits": "PASS",
        "discrete_collision": "PASS",
        "continuous_collision": "PASS",
    }


def test_cli_via_service_timeout_uses_exit_code_4(planner_ws, capsys,
                                                  tmp_path):
    """服务端回 TIMEOUT → CLI 退出码 4（与校验未过的 3 区分开）。"""
    sock_path = os.path.join(str(tmp_path), "timeout.sock")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(sock_path)
    listener.listen(1)

    def _serve():
        try:
            conn, _ = listener.accept()
            conn.recv(65536)
            conn.sendall((
                '{"plan_id": "p-to", "status": "TIMEOUT", "trajectory": null,'
                ' "verification": {},'
                ' "failure_reason": "规划超时：服务端在 deadline 20000 ms 处强制中断 worker",'
                ' "metrics": {"timeout_ms": 20000, "interrupted_stage": "plan",'
                ' "next_action": "放宽 timeout_ms"}}\n'
            ).encode("utf-8"))
            conn.close()
        except OSError:
            pass

    thread = threading.Thread(target=_serve, daemon=True)
    thread.start()
    try:
        code = cli_main([
            "--ws", planner_ws,
            "plan", "--via-service",
            "--socket", sock_path,
            "--goal", ",".join(str(v) for v in GOAL),
        ])
    finally:
        listener.close()
    out = capsys.readouterr().out
    assert code == 4, out
    assert "失败原因:" in out
    assert "强制中断" in out


def test_cli_via_service_without_server_reports_next_action(planner_ws, capsys,
                                                            tmp_path):
    """服务缺席 → 退出码 1（FAILED）+ 可粘贴的下一步（不抛异常）。"""
    missing = os.path.join(str(tmp_path), "absent.sock")
    code = cli_main([
        "--ws", planner_ws,
        "plan", "--via-service",
        "--socket", missing,
        "--goal", ",".join(str(v) for v in GOAL),
    ])
    out = capsys.readouterr().out
    assert code == 1, out
    assert "失败原因:" in out
    assert missing in out
    assert "下一步:" in out


def test_cli_via_service_invalid_goal_is_usage_error(planner_ws, capsys,
                                                      server):
    """缺目标 → 退出码 2（参数错误），不打服务。"""
    code = cli_main([
        "--ws", planner_ws,
        "plan", "--via-service",
        "--socket", server.socket_path,
    ])
    out = capsys.readouterr().out
    assert code == 2, out
    assert "--goal" in out


def test_cli_inproc_default_output_shape_is_unchanged(planner_ws, capsys):
    """进程内模式的输出格式逐字保持（既有行为不变）。"""
    code = cli_main([
        "--ws", planner_ws,
        "plan",
        "--goal", ",".join(str(v) for v in GOAL),
    ])
    out = capsys.readouterr().out
    assert code == 0, out
    lines = out.splitlines()
    assert lines[0].startswith("管线 FreespacePipeline | 成功=True | 耗时 ")
    assert " | 点数 " in lines[0]
    assert "四道校验: 全部通过" in out
    for name in ("limits", "discrete_collision", "continuous_collision"):
        assert "  [PASS] %s" % name in out
    # 服务模式专属的段落**不**出现在进程内模式里。
    assert "节点耗时:" not in out
