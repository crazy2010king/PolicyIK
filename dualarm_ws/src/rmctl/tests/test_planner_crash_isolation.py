"""rm_planner 的**崩溃隔离**验收（S07/T5，切片验收项 2）。

命题：**规划服务进程被杀，不影响正在进行的运动执行**。

本文件给出三条场景：

* **场景 A（验收项 2 的主证据）** —— 进程 A 用 ``rm_executor`` 的 **fake
  执行路径**回放一条**真实规划出的**轨迹（统一报告 schema：
  ``python3 -m rm_executor.cli --traj <traj.json> --mode fake --report
  <report.json>``）。回放进行到约 1 s 时，对**规划服务进程**与它的**全部
  worker 子进程**发 ``SIGKILL``。随后断言：

  1. 进程 A 的统一报告 ``status == "ok"``、``mode == "fake"``；
  2. 报告里**没有中断类事件**（无 FATAL / 无控制器不可达 / 无中止）；
  3. 轨迹被**完整回放**（栅格拍数 > 0 + 末点到位判定通过）；
  4. 规划服务进程**确实已死**（``os.kill(pid, 0)`` 抛
     ``ProcessLookupError``），它的 worker 也**全部已死**。

  **刻意不使用 ``rmctl.executor.MockExecutor``** —— 它被明确标注为 legacy，
  验收结论不得引用其统计；本文件的证据全部来自 ``rm_executor`` 的统一报告。

* **场景 B（服务可恢复）** —— 重新起一个服务实例 → 新请求 ``SUCCESS``：
  证明服务进程死亡**不影响其它进程**，且服务可重启继续受理。

  **自动复活机制属部署层**（systemd / supervisor 的 ``Restart=always`` 等），
  **不在本切片承诺范围**：本切片只承诺「服务进程死亡不影响其他进程」。

* **场景 C（真机映射）** —— 本轮**不做**真机版崩溃隔离（真机为预约资源，
  且 S06 已确立「离线代理 + 真机入口」两层口径）。真机等价证明 =
  **在 ``rm_runtime`` 执行过程中 ``kill -9 rm_planner``**，观察真机运动
  是否继续；留待 S06/S09 集成测试执行。本文件只固定这条口径（docstring +
  一条守卫用例），**不声称真机已实证**。

子进程纪律：全部用 ``sys.executable`` + **显式 env**（``PYTHONPATH`` 含
``dualarm_ws/src/rmctl`` / ``dualarm_ws/src/rm_executor`` /
``dualarm_ws/src/rm_hw_bridge`` / ``dualarm_ws/src/rm_safety``；
``LD_LIBRARY_PATH`` 由 ``rmctl.vendor_env.ensure_libstdcxx()`` 预加载的目录
继承），**不依赖 shell 环境**；临时 socket / 报告 / 轨迹都落 pytest 临时目录。
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl")
EXEC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_executor")
HW_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge")
SAFETY_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_safety")
RM_DESC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description")
RM_DESC_SCRIPTS = os.path.join(RM_DESC_SRC, "scripts")
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

for _path in (PKG_SRC, RM_DESC_SCRIPTS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import build_model  # noqa: E402

from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.vendor_env import ensure_libstdcxx  # noqa: E402

#: 会让规划器「烧满 planning_time_s」的场景（规划服务侧有在飞 worker 时用）。
BLOCKING_SCENE = {
    "obstacles": [
        {"name": "blocker", "box_size": [0.5, 0.5, 0.5], "position": [0.2, -0.3, 0.1]},
    ]
}

#: 中断类事件（出现即判定「运动执行被中断」）。
INTERRUPTING_EVENT_TYPES = (
    "controller_unreachable",
    "transport_failure",
    "trajectory_contract_rejected",
    "time_domain_limits_rejected",
    "safety_stop",
    "external_stop",
)


# --------------------------------------------------------------------------
# 夹具
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_crash_ws")
    build_model.build_model(
        ws=str(ws),
        out=str(ws / "build" / "model"),
        pkg=RM_DESC_SRC,
    )
    cfg_dst = ws / "workcell" / "planner"
    cfg_dst.mkdir(parents=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name),
                        str(cfg_dst / name))
    return str(ws)


@pytest.fixture(scope="session")
def executor_ws(tmp_path_factory):
    """执行器用的**工作区**（只用 git 跟踪的文件物化，不碰 gitignore 目录）。

    布局照抄仓库根（``workcell/`` + ``dualarm_ws/src/rm_description/``），
    因为 ``rm_executor.paths`` / ``rm_hw_bridge.paths`` 都按「仓库根」解析
    ``workcell/rm_workcell.yaml`` 与关节限值。**刻意不复制 ``build/model``**
    —— 那是 gitignore 目录；限值走 ``rm_description/config/joint_limits.yaml``
    （git 跟踪）这条回退路径，于是用例在任何机器上取到的限值一致。
    """
    ws = tmp_path_factory.mktemp("planner_crash_exec_ws")
    workcell = ws / "workcell"
    workcell.mkdir(parents=True)
    shutil.copyfile(os.path.join(REPO_ROOT, "workcell", "rm_workcell.yaml"),
                    str(workcell / "rm_workcell.yaml"))
    safety = workcell / "safety"
    safety.mkdir(parents=True)
    shutil.copyfile(os.path.join(REPO_ROOT, "workcell", "safety",
                                 "error_codes.yaml"),
                    str(safety / "error_codes.yaml"))
    cfg_dir = ws / "dualarm_ws" / "src" / "rm_description" / "config"
    cfg_dir.mkdir(parents=True)
    shutil.copyfile(os.path.join(RM_DESC_SRC, "config", "joint_limits.yaml"),
                    str(cfg_dir / "joint_limits.yaml"))
    return str(ws)


@pytest.fixture(scope="session")
def child_env():
    """子进程的**显式**环境（不依赖调用者的 shell）。"""
    env = os.environ.copy()
    paths = [PKG_SRC, EXEC_SRC, HW_SRC, SAFETY_SRC]
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(
        paths + ([existing] if existing else [])
    )
    lib = ensure_libstdcxx()
    if lib:
        lib_dir = os.path.dirname(os.path.abspath(lib))
        current = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = (
            lib_dir + (os.pathsep + current if current else "")
        )
    return env


def _start_server(sock_path, ws, env):
    """起一个**独立进程**的 rm_planner 服务（返回 Popen）。"""
    proc = subprocess.Popen(
        [sys.executable, "-m", "rmctl.planner_service.server",
         "--socket", sock_path, "--ws", ws],
        env=env,
        cwd=ws,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if os.path.exists(sock_path):
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                probe.settimeout(0.5)
                probe.connect(sock_path)
                probe.close()
                return proc
            except OSError:
                probe.close()
        if proc.poll() is not None:
            out, err = proc.communicate()
            raise AssertionError("规划服务进程提前退出（%s）: %s"
                                 % (proc.returncode, err))
        time.sleep(0.05)
    raise AssertionError("规划服务在 30 s 内没有起来（socket=%s）" % sock_path)


# --------------------------------------------------------------------------
# 进程工具
# --------------------------------------------------------------------------

def _ppid_map():
    """``{pid: ppid}``（扫 ``/proc``；不依赖 ps / psutil）。"""
    out = {}
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            with open("/proc/%d/stat" % pid, "r", encoding="utf-8") as fh:
                text = fh.read()
        except OSError:
            continue
        try:
            tail = text[text.rindex(")") + 1:].split()
            out[pid] = int(tail[1])       # tail[0]=state, tail[1]=ppid
        except (ValueError, IndexError):
            continue
    return out


def _descendants(pid):
    """某进程的**全部后代**（含 worker 子进程），深度优先。"""
    ppids = _ppid_map()
    out = []
    frontier = [int(pid)]
    while frontier:
        current = frontier.pop()
        for child, parent in ppids.items():
            if parent == current and child not in out:
                out.append(child)
                frontier.append(child)
    return out


def _pid_running(pid):
    """进程是否**仍在运行**（僵尸算已死：它不再执行任何指令）。"""
    try:
        with open("/proc/%d/stat" % int(pid), "r", encoding="utf-8") as fh:
            text = fh.read()
    except OSError:
        return False
    try:
        state = text[text.rindex(")") + 1:].split()[0]
    except (ValueError, IndexError):       # pragma: no cover - 竞态
        return False
    return state != "Z"


def _kill_tree(pid):
    """对进程及其**全部后代**发 ``SIGKILL``（先子孙后自己）。"""
    victims = _descendants(pid) + [int(pid)]
    for victim in victims:
        try:
            os.kill(victim, signal.SIGKILL)
        except OSError:
            pass
    return victims


def _wait_dead(pids, timeout=10.0):
    """等这些 pid 全部不再运行。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(_pid_running(pid) for pid in pids):
            return True
        time.sleep(0.05)
    return not any(_pid_running(pid) for pid in pids)


# --------------------------------------------------------------------------
# 轨迹工具
# --------------------------------------------------------------------------

def _plan_trajectory(client, plan_id="crash-traj"):
    """经服务规划一条真实轨迹 → 统一报告可吃的轨迹 JSON 形状。"""
    request = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": [0.0] * 7,
        "goal": {"type": "joint", "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
        "constraints": {},
        "seed": 0,
        "timeout_ms": 20000,
        "scene_version": 1,
    }
    resp = client.plan(request)
    assert resp["status"] == "SUCCESS", resp
    trajectory = resp["trajectory"]
    return {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": [float(t) for t in trajectory["times"]],
        "points": [[float(v) for v in point["positions"]]
                   for point in trajectory["points"]],
        "verify": resp["verification"],
    }


def _write_traj(payload, path):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False)
    return path


def _read_report(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


# --------------------------------------------------------------------------
# 场景 C（真机映射口径）：文档级守卫
# --------------------------------------------------------------------------

def test_true_machine_equivalence_mapping_is_documented():
    """真机等价证明的口径必须写在本文件里（留待 S06/S09 真机执行）。

    本轮**不**声称真机已实证 —— 断言的只是「口径写在案」，防止后续切片
    把「离线代理结论」误当成「真机结论」。
    """
    text = __doc__ or ""
    assert "rm_runtime" in text
    assert "kill -9 rm_planner" in text
    assert "S06/S09" in text


# --------------------------------------------------------------------------
# 场景 A + B
# --------------------------------------------------------------------------

def test_service_death_does_not_break_motion_execution(planner_ws, executor_ws,
                                                       child_env, tmp_path):
    """**验收项 2**：规划服务进程被杀，不影响正在进行的运动执行。

    步骤：起独立服务进程 → 规划一条真轨迹 → 起 ``rm_executor`` fake 回放
    → 回放到约 1 s 时 ``SIGKILL`` 服务进程及其全部 worker → 断言回放完整
    成功、服务进程确实已死。
    """
    sock_path = os.path.join(str(tmp_path), "rm_planner.sock")
    traj_path = os.path.join(str(tmp_path), "traj.json")
    report_path = os.path.join(str(tmp_path), "report.json")

    server = _start_server(sock_path, planner_ws, child_env)
    client = PlannerClient(socket_path=sock_path, ws=planner_ws)
    try:
        # -- 1) 规划一条真实轨迹（并落成执行器可吃的形状）--
        payload = _plan_trajectory(client)
        _write_traj(payload, traj_path)

        # -- 2) 让规划服务侧有一个**在飞 worker**（证明 worker 也被杀）--
        inflight = {}

        def _inflight_plan():
            inflight["resp"] = client.plan({
                "plan_id": "inflight",
                "profile": "FREESPACE",
                "group": "rm75_arm",
                "start_state": [0.0] * 7,
                "goal": {"type": "joint",
                         "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
                "constraints": {},
                "seed": 0,
                "timeout_ms": 60000,
                "scene_version": 1,
                "scene": BLOCKING_SCENE,
            })

        thread = threading.Thread(target=_inflight_plan, daemon=True)
        thread.start()
        worker_pids = _wait_for_worker(client)
        assert worker_pids, "没看到在飞 worker（场景 A 需要一个活 worker）"

        # -- 3) 进程 A：rm_executor fake 回放（统一报告 schema）--
        #    ``--realtime`` 让回放占用**墙钟**（默认 ManualClock 是虚拟时间，
        #    瞬间就跑完，「回放进行到约 1 s 时」无从谈起）。
        replay = subprocess.Popen(
            [sys.executable, "-m", "rm_executor.cli",
             "--traj", traj_path, "--mode", "fake", "--realtime",
             "--report", report_path, "--ws", executor_ws],
            env=child_env,
            cwd=executor_ws,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # -- 4) 回放到约 1 s：对服务进程与它的全部 worker 发 SIGKILL--
        time.sleep(1.0)
        assert replay.poll() is None, "回放进程提前退出"
        victims = _kill_tree(server.pid)
        assert server.pid in victims
        assert set(worker_pids) & set(victims), (
            "worker %s 不在 SIGKILL 名单 %s 里" % (worker_pids, victims)
        )
        server.wait(timeout=10.0)
        assert _wait_dead(victims), "仍有进程存活: %s" % victims

        # -- 5) 回放必须完整结束且成功--
        stdout, stderr = replay.communicate(timeout=120)
        assert replay.returncode == 0, "回放退出码 %s\n%s\n%s" % (
            replay.returncode, stdout, stderr)
        report = _read_report(report_path)
        assert report["status"] == "ok", report
        assert report["mode"] == "fake", report

        # -- 6) 无中断类事件--
        events = report.get("events") or []
        for event in events:
            assert event.get("severity") != "FATAL", event
            assert event.get("type") not in INTERRUPTING_EVENT_TYPES, event
        joined = " ".join(str(event.get("reason") or "") for event in events)
        assert "控制器不可达" not in joined, events

        # -- 7) 轨迹被完整回放--
        metrics = report.get("metrics") or {}
        assert metrics.get("ticks", 0) > 0, metrics
        assert metrics.get("arrival_err_deg") is not None, metrics
        assert any(event.get("type") == "arrival_ok" for event in events), events

        # -- 8) 规划服务进程确实已死--
        with pytest.raises(ProcessLookupError):
            os.kill(server.pid, 0)
        for pid in worker_pids:
            assert not _pid_running(pid), "worker %s 还活着" % pid

        # -- 9) 在飞请求拿到的是**结构化错误**（不是裸异常）--
        thread.join(timeout=30.0)
        assert not thread.is_alive()
        resp = inflight.get("resp")
        assert resp is not None
        assert resp["status"] == "FAILED", resp
        assert resp["failure_reason"], resp
    finally:
        try:
            _kill_tree(server.pid)
        except Exception:  # noqa: BLE001 - 清理不掩盖断言失败
            pass
        if server.poll() is None:      # pragma: no cover - 兜底
            server.kill()
            server.wait(timeout=5.0)


def test_service_can_be_restarted_after_death(planner_ws, child_env, tmp_path):
    """**场景 B**：服务进程死后，重新起一个实例 → 新请求 ``SUCCESS``。

    证明「服务进程死亡不影响其他进程」，且服务**可重启继续受理**。
    （自动复活属部署层，本切片不承诺。）
    """
    sock_path = os.path.join(str(tmp_path), "rm_planner.sock")
    first = _start_server(sock_path, planner_ws, child_env)
    client = PlannerClient(socket_path=sock_path, ws=planner_ws)
    try:
        payload = _plan_trajectory(client, "before-death")
        _kill_tree(first.pid)
        first.wait(timeout=10.0)
        with pytest.raises(ProcessLookupError):
            os.kill(first.pid, 0)
    finally:
        if first.poll() is None:      # pragma: no cover - 兜底
            first.kill()
            first.wait(timeout=5.0)

    # -- 重启（socket 路径会被新实例接管；陈旧 socket 文件由服务端自己清理）--
    second = _start_server(sock_path, planner_ws, child_env)
    try:
        client2 = PlannerClient(socket_path=sock_path, ws=planner_ws)
        resp = client2.plan({
            "plan_id": "after-restart",
            "profile": "FREESPACE",
            "group": "rm75_arm",
            "start_state": [0.0] * 7,
            "goal": {"type": "joint",
                     "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
            "constraints": {},
            "seed": 0,
            "timeout_ms": 20000,
            "scene_version": 1,
        })
        assert resp["status"] == "SUCCESS", resp
        assert resp["trajectory"]["points"], resp
    finally:
        _kill_tree(second.pid)
        if second.poll() is None:     # pragma: no cover - 兜底
            second.kill()
            second.wait(timeout=5.0)


def _wait_for_worker(client, timeout=30.0):
    """等服务侧出现**在飞 worker**（返回它的 pid 列表）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        info = client.status()
        if isinstance(info, dict):
            workers = info.get("active_workers") or []
            pids = [int(item["pid"]) for item in workers
                    if item.get("pid")]
            if pids:
                return pids
        time.sleep(0.05)
    return []
