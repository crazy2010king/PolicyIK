"""rm_planner 的**硬超时 / 可中断**用例（切片验收项 1 与「可中断」）。

核心命题：**超时是服务端强制中断的，不是规划器自己放弃**。

因此每个用例都做两件事：

1. **先自证场景确实慢**：不经服务，直接用同一请求调引擎，断言它在
   ``3 × deadline`` 内**仍未返回**（子进程被超时杀掉）→ 场景真的慢；
2. **再经服务发同一请求**（``timeout_ms = deadline``）→ 断言
   ``status == "TIMEOUT"``、**被中断阶段**写明、耗时**远小于**规划器自己
   放弃所需的时间（``pipeline.yaml::planning_time_s``）→ 证明是服务端在
   deadline 处 SIGKILL 的。

还覆盖：

* 超时后**没有残留 worker 进程**（服务自己的 worker 计数 + 对已知 pid 做
  ``os.kill(pid, 0)`` 探测）；
* 超时后服务仍能正常规划（**服务进程本身没死**）；
* **可中断**：另一条连接发 ``cancel`` → 原请求方收到 ``CANCELLED``。
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
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

from rmctl.planner_service.server import PlannerServer  # noqa: E402

START = [0.0] * 7
GOAL = [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]

#: 服务端硬超时（毫秒）。要**明显小于**规划器自己的放弃时间
#: （``workcell/planner/pipeline.yaml::planning_time_s = 5.0`` 秒）。
DEADLINE_MS = 1200

#: 规划器自己放弃所需的时间上限（秒）——用于断言「服务端比它快得多」。
PLANNER_GIVE_UP_S = 5.0

#: 一个**必然规划失败**的场景：目标处放一个盒子障碍物 → 规划器找不到解，
#: 会烧满 ``planning_time_s`` 才放弃（≈ 5 s）。
BLOCKING_SCENE = {
    "obstacles": [
        {"name": "blocker", "box_size": [0.5, 0.5, 0.5], "position": GOAL[:3]},
    ]
}

#: 直接调引擎的子进程脚本（自证「场景确实慢」用）。
DIRECT_SCRIPT = """
import json, os, sys, time
sys.path.insert(0, os.environ["RMCTL_SRC"])
from rmctl.vendor_env import ensure_libstdcxx
ensure_libstdcxx()
from rmctl.model import Model
from rmctl.planner_service.contract import PlanRequest
from rmctl.planner_service import engine
ws = sys.argv[1]
request = PlanRequest.from_dict(json.loads(sys.argv[2]), ws=ws)
model = Model(ws=ws)
started = time.time()
response = engine.plan(request, ws=ws, model=model)
print(json.dumps({"elapsed_s": time.time() - started,
                  "status": response.status}))
"""


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_timeout_ws")
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


def make_request(plan_id="p-to", **overrides):
    """构造请求字典（默认是**必然慢**的阻塞场景）。"""
    data = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": list(START),
        "goal": {"type": "joint", "values": list(GOAL)},
        "constraints": {},
        "seed": 7,
        "timeout_ms": DEADLINE_MS,
        "scene_version": 1,
        "scene": BLOCKING_SCENE,
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


def pid_alive(pid):
    """进程是否还活着（``os.kill(pid, 0)`` 探测）。"""
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:  # pragma: no cover - 存在但不属于我们
        return True
    return True


# --------------------------------------------------------------------------
# 用例
# --------------------------------------------------------------------------

def test_slow_scenario_is_genuinely_slow(planner_ws):
    """**自证**：同一请求直接调引擎，在 3× deadline 内仍未返回。"""
    env = os.environ.copy()
    env["RMCTL_SRC"] = PKG_SRC
    request = make_request("p-direct")
    with pytest.raises(subprocess.TimeoutExpired):
        subprocess.run(
            [sys.executable, "-c", DIRECT_SCRIPT, planner_ws,
             json.dumps(request)],
            timeout=3 * DEADLINE_MS / 1000.0,
            env=env,
            capture_output=True,
        )


def test_server_forces_timeout(server):
    """**验收项 1**：规划超时被**服务端强制中断**（不是规划器自己放弃）。"""
    request = make_request("p-to")
    started = time.time()
    resp = send_line(server, {"cmd": "plan", "request": request}, timeout=60.0)
    elapsed = time.time() - started

    assert resp["status"] == "TIMEOUT", resp
    metrics = resp["metrics"]
    assert metrics["timeout_ms"] == DEADLINE_MS
    assert metrics["killed"] is True
    # 被中断阶段：worker 最后一条心跳（规划阶段被 SIGKILL）
    assert metrics["interrupted_stage"] == "plan", resp
    assert "被中断阶段" in resp["failure_reason"], resp
    assert str(DEADLINE_MS) in resp["failure_reason"], resp
    # 服务端在 deadline 处动手：远快于规划器自己放弃（≈ 5 s）
    assert elapsed < PLANNER_GIVE_UP_S, (
        "服务端耗时 %.2f s 不应接近规划器自己的放弃时间 %.1f s"
        % (elapsed, PLANNER_GIVE_UP_S)
    )
    # 超时窗口 = deadline + worker 启动开销（启动宽限独立计量）
    assert elapsed < DEADLINE_MS / 1000.0 + 3.0, elapsed


def test_timeout_leaves_no_worker_behind(server):
    """超时后**没有残留 worker 进程**（服务计数 + pid 探测）。"""
    resp = send_line(server, {"cmd": "plan", "request": make_request("p-leak")},
                     timeout=60.0)
    assert resp["status"] == "TIMEOUT", resp
    pid = resp["metrics"]["worker_pid"]
    assert not pid_alive(pid), "超时后 worker 进程 %s 还活着" % pid
    assert server.worker_pids() == []
    counters = server.counters
    assert counters["workers_spawned"] == counters["workers_reaped"]
    assert counters["timeout"] == 1


def test_server_survives_timeout(server):
    """超时之后**服务仍然可用**（下一个请求正常成功）。"""
    resp = send_line(server, {"cmd": "plan", "request": make_request("p-to2")},
                     timeout=60.0)
    assert resp["status"] == "TIMEOUT", resp
    good = send_line(server, {"cmd": "plan", "request": make_request(
        "p-to3", timeout_ms=30000, scene=None)})
    assert good["status"] == "SUCCESS", good


def test_cancel_interrupts_running_plan(server):
    """**可中断**：另一条连接发 cancel → 原请求方收到 CANCELLED。"""
    result = {}

    def _run():
        result["resp"] = send_line(
            server,
            {"cmd": "plan",
             "request": make_request("p-cancel", timeout_ms=30000)},
            timeout=60.0,
        )

    thread = threading.Thread(target=_run)
    thread.start()
    pid = wait_for_worker(server, "p-cancel")
    time.sleep(0.3)
    cancel = send_line(server, {"cmd": "cancel", "plan_id": "p-cancel"})
    assert cancel["status"] == "ok", cancel
    thread.join(timeout=30.0)
    assert not thread.is_alive()
    resp = result["resp"]
    assert resp["status"] == "CANCELLED", resp
    assert resp["metrics"]["killed"] is True
    assert not pid_alive(pid), "取消后 worker 进程 %s 还活着" % pid
    assert server.worker_pids() == []
    # 服务仍然可用
    good = send_line(server, {"cmd": "plan", "request": make_request(
        "p-after-cancel", timeout_ms=30000, scene=None)})
    assert good["status"] == "SUCCESS", good


def test_cancel_unknown_plan_id_is_explicit(server):
    """取消一个不存在的 plan_id → 明确错误（不静默忽略）。"""
    resp = send_line(server, {"cmd": "cancel", "plan_id": "p-nope"})
    assert resp["status"] == "error", resp
    assert "p-nope" in resp["reason"], resp
