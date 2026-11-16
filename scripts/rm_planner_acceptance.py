#!/usr/bin/env python3
"""S07 rm_planner **一键验收脚本**（三条验收 → build/reports/s07_planner_acceptance.json）。

用法
----
    python3 scripts/rm_planner_acceptance.py
    python3 scripts/rm_planner_acceptance.py --out build/reports/s07_planner_acceptance.json
    python3 scripts/rm_planner_acceptance.py --only deterministic_same_seed

退出码
------
* ``0`` —— 三条验收**全部通过**（报告逐条 ``status=ok``）；
* ``4`` —— 存在失败的验收段（报告写明**哪一段、哪一条断言**失败 + 下一步）。

三条验收（与切片 S07 的验收项逐条对应）
--------------------------------------

1. ``timeout_forced_interrupt`` —— **规划超时被服务端强制中断**。
   同一障碍物场景给**小 deadline**（1200 ms）→ 服务端在 deadline 处
   ``SIGKILL worker`` → ``status=TIMEOUT``；再用**同一场景**给足预算
   （20 s）→ 规划器**自己**放弃 → ``status=FAILED``。
   两条对照起来才证明「TIMEOUT 是服务端强制中断，不是规划器自己放弃」。

2. ``crash_isolation`` —— **服务进程被杀不影响运动执行**。
   真实规划出一条轨迹 → ``rm_executor`` fake 回放（统一报告 schema）→
   回放进行中 ``kill -9`` 规划服务进程及其**全部 worker 子进程** →
   断言回放报告 ``status=ok`` / ``mode=fake`` / 无中断类事件 / 轨迹完整回放，
   且规划服务进程**确实已死**。

3. ``deterministic_same_seed`` —— **同请求同种子结果逐点一致**。
   同一请求（request + seed + scene_version）发两次 → 两次各自 spawn
   **独占** worker 子进程（进程级 ``RNG_setSeed``）→ 比对
   ``trajectory_digest``（sha256 规范化摘要）。

**诚实分层（重要）**
--------------------
本脚本跑在 **真实规划**（厂商 tesseract 内核）+ **真实四道校验**
（``rmctl.verify``）+ **真实进程级 SIGKILL** 之上，但**运动执行是假的**
（``rm_executor`` 的 fake 传输）。因此崩溃隔离的结论是**离线层**结论：

* 离线层（**本脚本已实证**）= 真实规划内核 + 真实四道校验 + 真实进程级
  SIGKILL + ``rm_executor`` fake 回放；
* 真机层（**本轮未实证**）= ``rm_runtime`` 执行过程中真的 ``kill -9
  rm_planner``，观察真机运动是否继续 —— 属真机窗口，详见
  ``docs/rm_planner规划服务说明.md`` 的「离线-真机口径声明」章节。

零新依赖：只用标准库 + 仓库内既有包（``rmctl`` / ``rm_executor`` /
``rm_hw_bridge`` / ``rm_safety`` / ``rm_description``）。
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------
# 0. 自插 sys.path（脚本可从任意目录调用）
# --------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

for _rel in (
    "dualarm_ws/src/rmctl",
    "dualarm_ws/src/rm_executor",
    "dualarm_ws/src/rm_hw_bridge",
    "dualarm_ws/src/rm_safety",
    "dualarm_ws/src/rm_description/scripts",
):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.planner_service.contract import trajectory_digest  # noqa: E402
from rmctl.vendor_env import ensure_libstdcxx  # noqa: E402

#: 验收项 1 的场景面：障碍物堵死目标（规划器本来要烧 5 s）。
BLOCKING_SCENE = {
    "obstacles": [
        {"name": "blocker", "box_size": [0.5, 0.5, 0.5],
         "position": [0.2, -0.3, 0.1]},
    ]
}

#: 服务端强制中断用的**小 deadline**（毫秒）。
TIMEOUT_DEADLINE_MS = 1200

#: 同一场景给足预算（毫秒）→ 规划器自己放弃（对照面）。
GIVE_UP_BUDGET_MS = 20000

#: 验收项 1 的耗时上限：deadline × 1.5（服务端必须在 deadline 附近收手）。
TIMEOUT_ELAPSED_FACTOR = 1.5

#: 中断类事件（出现即判定「运动执行被中断」）。
INTERRUPTING_EVENT_TYPES = (
    "controller_unreachable",
    "transport_failure",
    "trajectory_contract_rejected",
    "time_domain_limits_rejected",
    "safety_stop",
    "external_stop",
)

#: 离线代理的**固定声明**（必须出现在报告里）。
FAKE_NOTE = (
    "mode=fake（离线层：真规划 + 真校验 + 真进程级 SIGKILL + 假运动回放，"
    "不是真机结论）"
)

#: 真机版崩溃隔离的**未实证声明**。
LIVE_NOTE = (
    "真机版崩溃隔离（rm_runtime 执行中 kill -9 rm_planner 后真机运动是否继续）"
    "本轮未实证：属真机窗口；本脚本的证据是离线层。"
)


# --------------------------------------------------------------------------
# 1. 小工具
# --------------------------------------------------------------------------

def _now() -> str:
    """当前时间（ISO 字符串）。"""
    return datetime.now().isoformat(timespec="seconds")


def _write_json(path: str, payload: Dict[str, Any]) -> str:
    """写 JSON（UTF-8、缩进 2、键序稳定）。"""
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, default=str)
    return path


def _child_env() -> Dict[str, str]:
    """子进程的**显式**环境（不依赖调用者的 shell）。"""
    env = os.environ.copy()
    paths = [
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_executor"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_safety"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description", "scripts"),
    ]
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


def _pid_running(pid: int) -> bool:
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


def _ppid_map() -> Dict[int, int]:
    """``{pid: ppid}``（扫 ``/proc``；不依赖 ps / psutil）。"""
    out: Dict[int, int] = {}
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


def _descendants(pid: int) -> List[int]:
    """某进程的**全部后代**（含 worker 子进程），深度优先。"""
    ppids = _ppid_map()
    out: List[int] = []
    frontier = [int(pid)]
    while frontier:
        current = frontier.pop()
        for child, parent in ppids.items():
            if parent == current and child not in out:
                out.append(child)
                frontier.append(child)
    return out


def _kill_tree(pid: int) -> List[int]:
    """对进程及其**全部后代**发 ``SIGKILL``（先子孙后自己）。"""
    victims = _descendants(pid) + [int(pid)]
    for victim in victims:
        try:
            os.kill(victim, signal.SIGKILL)
        except OSError:
            pass
    return victims


def _wait_dead(pids: List[int], timeout: float = 10.0) -> bool:
    """等这些 pid 全部不再运行。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(_pid_running(pid) for pid in pids):
            return True
        time.sleep(0.05)
    return not any(_pid_running(pid) for pid in pids)


def _wait_for_worker(client: PlannerClient, timeout: float = 30.0) -> List[int]:
    """等服务侧出现**在飞 worker**（返回它的 pid 列表）。

    用来证明「SIGKILL 名单里含 worker 子进程」—— 即服务被杀时它的 worker
    也一起没了，不会变成孤儿进程继续烧 CPU。
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        info = client.status()
        if isinstance(info, dict):
            workers = info.get("active_workers") or []
            pids = [int(item["pid"]) for item in workers if item.get("pid")]
            if pids:
                return pids
        time.sleep(0.05)
    return []


# --------------------------------------------------------------------------
# 2. 模型 / 服务 / 工作区
# --------------------------------------------------------------------------

def ensure_model(ws: str) -> Tuple[bool, str, List[str]]:
    """确保 ``build/model`` 可用（必要时**重新物化**）。

    :return: ``(ok, source, evidence)``；``source`` ∈
        ``{"prebuilt", "rebuilt", "unavailable"}``
    """
    evidence: List[str] = []
    try:
        from rmctl.model import Model

        Model(ws=ws)
        evidence.append("模型已就绪（build/model 已物化，未重建）")
        return True, "prebuilt", evidence
    except Exception as exc:  # noqa: BLE001 - 先记录再尝试重建
        evidence.append("模型加载失败（%s）: %s" % (type(exc).__name__, exc))

    try:
        import build_model  # dualarm_ws/src/rm_description/scripts/build_model.py

        build_model.build_model(ws=ws)
        evidence.append("已调用 build_model.build_model(ws=%s) 重新物化 build/model"
                        % ws)
        from rmctl.model import Model

        Model(ws=ws)
        evidence.append("重新物化后模型加载成功")
        return True, "rebuilt", evidence
    except Exception as exc:  # noqa: BLE001 - 模型不可得 → 诚实失败
        evidence.append("重新物化仍失败（%s）: %s" % (type(exc).__name__, exc))
        return False, "unavailable", evidence


def _start_server(sock_path: str, ws: str, env: Dict[str, str]):
    """起一个**独立进程**的 rm_planner 服务（返回 ``Popen``）。

    独立进程是「服务崩溃不影响运动执行」这条验收的前提 —— 被杀掉的必须
    是**真的服务进程**，不是同进程内的对象。
    """
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
            raise RuntimeError("规划服务进程提前退出（%s）: %s"
                               % (proc.returncode, err))
        time.sleep(0.05)
    raise RuntimeError("规划服务在 30 s 内没有起来（socket=%s）" % sock_path)


def _exec_ws(tmp_root: str) -> str:
    """物化一个**执行器工作区**（只用 git 跟踪的文件）。

    布局照抄仓库根：``workcell/rm_workcell.yaml`` + ``workcell/safety/`` +
    ``dualarm_ws/src/rm_description/config/joint_limits.yaml``（限值回退路径，
    刻意不复制 ``build/model`` 那个 gitignore 目录）。
    """
    ws = os.path.join(tmp_root, "exec_ws")
    workcell = os.path.join(ws, "workcell")
    os.makedirs(workcell, exist_ok=True)
    shutil.copyfile(os.path.join(REPO_ROOT, "workcell", "rm_workcell.yaml"),
                    os.path.join(workcell, "rm_workcell.yaml"))
    safety = os.path.join(workcell, "safety")
    os.makedirs(safety, exist_ok=True)
    shutil.copyfile(os.path.join(REPO_ROOT, "workcell", "safety",
                                 "error_codes.yaml"),
                    os.path.join(safety, "error_codes.yaml"))
    cfg_dir = os.path.join(ws, "dualarm_ws", "src", "rm_description", "config")
    os.makedirs(cfg_dir, exist_ok=True)
    shutil.copyfile(os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description",
                                 "config", "joint_limits.yaml"),
                    os.path.join(cfg_dir, "joint_limits.yaml"))
    return ws


# --------------------------------------------------------------------------
# 3. 请求构造（不补业务缺省：场景写什么就发什么）
# --------------------------------------------------------------------------

def _request(plan_id: str, timeout_ms: int, seed: int = 0,
             blocking: bool = False) -> Dict[str, Any]:
    """构造一条契约层请求（阻塞场景 = 障碍物堵死目标）。"""
    data: Dict[str, Any] = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": [0.0] * 7,
        "goal": {"type": "joint",
                 "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
        "constraints": {},
        "seed": int(seed),
        "timeout_ms": int(timeout_ms),
        "scene_version": 1,
    }
    if blocking:
        data["scene"] = json.loads(json.dumps(BLOCKING_SCENE))
    return data


def _assert(name: str, passed: bool, observed: Any) -> Dict[str, Any]:
    """一条断言记录（名字 + 结论 + 观察值）。"""
    return {"name": name, "passed": bool(passed), "observed": observed}


# --------------------------------------------------------------------------
# 4. 验收 1：规划超时被服务端强制中断
# --------------------------------------------------------------------------

def check_timeout_forced_interrupt(client: PlannerClient) -> Dict[str, Any]:
    """验收 ①：小 deadline → TIMEOUT（服务端 SIGKILL）；同场景给足预算 → FAILED。"""
    assertions: List[Dict[str, Any]] = []
    evidence: List[str] = []

    # -- 面 A：服务端强制中断 ------------------------------------------
    started = time.time()
    resp = client.plan(_request("acc-timeout", TIMEOUT_DEADLINE_MS,
                                 blocking=True))
    wall = time.time() - started
    metrics = resp.get("metrics") or {}
    failure_reason = str(resp.get("failure_reason") or "")
    elapsed_s = metrics.get("elapsed_s")

    assertions.append(_assert("A1 status == TIMEOUT",
                              resp.get("status") == "TIMEOUT",
                              resp.get("status")))
    assertions.append(_assert("A2 服务端确实 SIGKILL 了 worker（metrics.killed）",
                              metrics.get("killed") is True,
                              metrics.get("killed")))
    assertions.append(_assert("A3 失败原因写明「强制中断」",
                              "强制中断" in failure_reason,
                              failure_reason))
    assertions.append(_assert(
        "A4 指名了被中断阶段（metrics.interrupted_stage）",
        bool(metrics.get("interrupted_stage")),
        metrics.get("interrupted_stage")))
    elapsed_ok = (elapsed_s is not None
                  and float(elapsed_s) <=
                  (TIMEOUT_DEADLINE_MS / 1000.0) * TIMEOUT_ELAPSED_FACTOR)
    assertions.append(_assert(
        "A5 规划耗时 <= deadline × %.1f（服务端到点收手）"
        % TIMEOUT_ELAPSED_FACTOR, elapsed_ok, elapsed_s))
    assertions.append(_assert(
        "A6 worker 已被回收（服务端 status 里没有在飞 worker）",
        not (client.status().get("active_workers") or []),
        client.status().get("active_workers")))

    evidence.append(
        "面 A（deadline=%d ms）：status=%s，metrics.killed=%r，"
        "interrupted_stage=%r，elapsed_s=%r，墙钟=%.3f s"
        % (TIMEOUT_DEADLINE_MS, resp.get("status"), metrics.get("killed"),
           metrics.get("interrupted_stage"), elapsed_s, wall))
    evidence.append("面 A 失败原因：%s" % failure_reason)

    # -- 面 B：同一场景给足预算 → 规划器自己放弃（对照组）---------------
    started_b = time.time()
    resp_b = client.plan(_request("acc-planner-gives-up", GIVE_UP_BUDGET_MS,
                                   blocking=True))
    wall_b = time.time() - started_b
    reason_b = str(resp_b.get("failure_reason") or "")
    assertions.append(_assert(
        "B1 同场景给足预算 → status == FAILED（规划器自己放弃，不是服务端）",
        resp_b.get("status") == "FAILED", resp_b.get("status")))
    assertions.append(_assert(
        "B2 对照面的失败原因指向规划节点（不是「强制中断」）",
        "规划失败" in reason_b and "强制中断" not in reason_b,
        reason_b))
    evidence.append(
        "面 B（预算=%d ms，同一障碍物场景）：status=%s，墙钟=%.3f s，失败原因=%s"
        % (GIVE_UP_BUDGET_MS, resp_b.get("status"), wall_b, reason_b))
    evidence.append(
        "对照结论：TIMEOUT（服务端 SIGKILL）与 FAILED（规划器自己放弃）"
        "在同一场景面上被区分开 —— TIMEOUT 不是规划器放弃。")

    passed = all(item["passed"] for item in assertions)
    numbers = {
        "deadline_ms": TIMEOUT_DEADLINE_MS,
        "give_up_budget_ms": GIVE_UP_BUDGET_MS,
        "timeout_elapsed_s": elapsed_s,
        "timeout_wall_s": round(wall, 3),
        "give_up_wall_s": round(wall_b, 3),
        "interrupted_stage": metrics.get("interrupted_stage"),
        "timeout_status": resp.get("status"),
        "give_up_status": resp_b.get("status"),
    }
    failed = [item["name"] for item in assertions if not item["passed"]]
    return {
        "id": "timeout_forced_interrupt",
        "title": "验收①：规划超时被服务端强制中断（SIGKILL worker）",
        "status": "ok" if passed else "error",
        "passed": passed,
        "reason": ("服务端在 deadline %d ms 处 SIGKILL worker → status=TIMEOUT；"
                   "同一场景给足预算 → 规划器自己放弃 → status=FAILED"
                   % TIMEOUT_DEADLINE_MS) if passed else
                  "断言失败: %s" % ", ".join(failed),
        "next_action": "" if passed else
                       "对照 docs/rm_planner规划服务说明.md「硬超时语义」章节，"
                       "检查服务端 deadline 与 worker 心跳是否正常",
        "assertions": assertions,
        "evidence": evidence,
        "numbers": numbers,
    }


# --------------------------------------------------------------------------
# 5. 验收 2：服务崩溃不影响运动执行
# --------------------------------------------------------------------------

def check_crash_isolation(sock_path: str, ws: str, env: Dict[str, str],
                          tmp_root: str) -> Dict[str, Any]:
    """验收 ②：服务进程被 ``kill -9`` 不影响 ``rm_executor`` 的运动回放。"""
    assertions: List[Dict[str, Any]] = []
    evidence: List[str] = []

    server = _start_server(sock_path, ws, env)
    client = PlannerClient(socket_path=sock_path, ws=ws)
    traj_path = os.path.join(tmp_root, "traj.json")
    report_path = os.path.join(tmp_root, "report.json")
    exec_ws = _exec_ws(tmp_root)
    points = 0
    victims: List[int] = []

    try:
        # -- 1) 经服务规划一条**真实**轨迹 --------------------------------
        plan_resp = client.plan(_request("acc-crash-traj", 20000))
        assertions.append(_assert("C1 规划成功（拿到真实轨迹）",
                                  plan_resp.get("status") == "SUCCESS",
                                  plan_resp.get("status")))
        if plan_resp.get("status") != "SUCCESS":
            raise RuntimeError("规划失败，无法做崩溃隔离: %s"
                               % plan_resp.get("failure_reason"))
        trajectory = plan_resp["trajectory"]
        payload = {
            "schema_version": 1,
            "group": "rm75_arm",
            "joint_names": ["joint%d" % i for i in range(1, 8)],
            "units": {"times": "s", "points": "rad"},
            "times": [float(t) for t in trajectory["times"]],
            "points": [[float(v) for v in point["positions"]]
                       for point in trajectory["points"]],
            "verify": plan_resp["verification"],
        }
        _write_json(traj_path, payload)
        points = len(payload["points"])

        # -- 2) 让规划服务侧有一个**在飞 worker**（证明 worker 也被杀）----
        inflight: Dict[str, Any] = {}

        def _inflight_plan() -> None:
            inflight["resp"] = client.plan(
                _request("acc-crash-inflight", 60000, blocking=True))

        inflight_thread = threading.Thread(target=_inflight_plan, daemon=True)
        inflight_thread.start()
        worker_pids = _wait_for_worker(client)
        assertions.append(_assert(
            "C2a 服务侧出现**在飞 worker**（证明 worker 子进程也会被杀）",
            bool(worker_pids), worker_pids))

        # -- 3) 起 rm_executor fake 回放（统一报告 schema）----------------
        replay = subprocess.Popen(
            [sys.executable, "-m", "rm_executor.cli",
             "--traj", traj_path, "--mode", "fake", "--realtime",
             "--report", report_path, "--ws", exec_ws],
            env=env,
            cwd=exec_ws,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # -- 4) 回放到约 1 s：对服务进程与它的全部 worker 发 SIGKILL -----
        time.sleep(1.0)
        assertions.append(_assert("C2 回放进程在 SIGKILL 时仍存活",
                                  replay.poll() is None, replay.poll()))
        victims = _kill_tree(server.pid)
        server.wait(timeout=10.0)
        dead = _wait_dead(victims)
        assertions.append(_assert("C3 规划服务进程及其后代全部已死",
                                  dead, victims))
        assertions.append(_assert("C3a 在飞 worker 子进程也在 SIGKILL 名单里",
                                  bool(set(worker_pids) & set(victims)),
                                  {"workers": worker_pids, "victims": victims}))
        assertions.append(_assert("C4 服务进程确实已死（os.kill 抛 "
                                  "ProcessLookupError）",
                                  _process_gone(server.pid), server.pid))
        evidence.append("SIGKILL 名单（服务进程 + 全部 worker 子进程）: %s"
                        % victims)
        evidence.append("在飞 worker pid: %s" % worker_pids)

        # -- 5) 回放必须完整结束且成功 -----------------------------------
        stdout, stderr = replay.communicate(timeout=180)
        assertions.append(_assert("C5 回放进程退出码为 0",
                                  replay.returncode == 0, replay.returncode))
        report = _read_json(report_path)
        assertions.append(_assert("C6 统一报告 status == ok",
                                  report.get("status") == "ok",
                                  report.get("status")))
        assertions.append(_assert("C7 统一报告 mode == fake",
                                  report.get("mode") == "fake",
                                  report.get("mode")))

        events = report.get("events") or []
        fatals = [event for event in events
                  if event.get("severity") == "FATAL"]
        interrupting = [event for event in events
                        if event.get("type") in INTERRUPTING_EVENT_TYPES]
        assertions.append(_assert("C8 回放无 FATAL 事件", not fatals, fatals))
        assertions.append(_assert("C9 回放无中断类事件",
                                  not interrupting, interrupting))
        joined = " ".join(str(event.get("reason") or "") for event in events)
        assertions.append(_assert("C10 事件原因里无「控制器不可达」",
                                  "控制器不可达" not in joined, joined))

        rm_metrics = report.get("metrics") or {}
        assertions.append(_assert("C11 轨迹被完整回放（ticks > 0）",
                                  int(rm_metrics.get("ticks") or 0) > 0,
                                  rm_metrics.get("ticks")))
        assertions.append(_assert("C12 到位判定通过（arrival_ok 事件）",
                                  any(event.get("type") == "arrival_ok"
                                      for event in events),
                                  [event.get("type") for event in events]))
        evidence.append("回放报告：status=%s，mode=%s，ticks=%s，arrival_err_deg=%s"
                        % (report.get("status"), report.get("mode"),
                           rm_metrics.get("ticks"),
                           rm_metrics.get("arrival_err_deg")))
        if replay.returncode != 0:
            evidence.append("回放 stdout: %s" % stdout[-400:])
            evidence.append("回放 stderr: %s" % stderr[-400:])

        # -- 6) 在飞请求拿到的是**结构化错误**（不是裸异常）----------------
        inflight_thread.join(timeout=30.0)
        inflight_resp = inflight.get("resp")
        assertions.append(_assert(
            "C13 服务死亡时在飞请求拿到结构化失败（不是裸异常）",
            isinstance(inflight_resp, dict)
            and inflight_resp.get("status") in ("FAILED", "TIMEOUT")
            and bool(inflight_resp.get("failure_reason")),
            (inflight_resp or {}).get("status")))
    finally:
        try:
            _kill_tree(server.pid)
        except Exception:  # noqa: BLE001 - 清理不掩盖断言失败
            pass
        if server.poll() is None:      # pragma: no cover - 兜底
            server.kill()
            server.wait(timeout=5.0)

    passed = all(item["passed"] for item in assertions)
    failed = [item["name"] for item in assertions if not item["passed"]]
    return {
        "id": "crash_isolation",
        "title": "验收②：服务进程被杀不影响运动执行（rm_executor fake 回放）",
        "status": "ok" if passed else "error",
        "passed": passed,
        "reason": ("kill -9 规划服务进程及其全部 worker 后，"
                   "rm_executor fake 回放仍完整成功（status=ok，无中断类事件）"
                   if passed else "断言失败: %s" % ", ".join(failed)),
        "next_action": "" if passed else
                       "对照 docs/rm_planner规划服务说明.md「离线-真机口径声明」，"
                       "确认 rm_executor 报告里是否出现中断类事件",
        "assertions": assertions,
        "evidence": evidence,
        "numbers": {
            "trajectory_points": points,
            "killed_pids": victims,
        },
    }


def _process_gone(pid: int) -> bool:
    """进程是否已经不存在（``os.kill(pid, 0)`` 抛 ``ProcessLookupError``）。"""
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return True
    except OSError:
        return True
    return False


def _read_json(path: str) -> Dict[str, Any]:
    """读 JSON 文件。"""
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


# --------------------------------------------------------------------------
# 6. 验收 3：同请求同种子结果逐点一致
# --------------------------------------------------------------------------

def check_deterministic_same_seed(client: PlannerClient) -> Dict[str, Any]:
    """验收 ③：同 (request, seed, scene_version) 两次 → digest 相同。"""
    assertions: List[Dict[str, Any]] = []
    evidence: List[str] = []

    seed = 11
    before = (client.status().get("counters") or {})
    req_a = _request("acc-det-a", 20000, seed=seed)
    req_b = _request("acc-det-b", 20000, seed=seed)
    # 两次请求**除了 plan_id 之外完全一致**（seed / scene_version / 场景相同）
    assert req_a["seed"] == req_b["seed"]
    assert req_a["scene_version"] == req_b["scene_version"]

    resp_a = client.plan(req_a)
    resp_b = client.plan(req_b)
    after = (client.status().get("counters") or {})

    assertions.append(_assert("D1 第一次规划成功",
                              resp_a.get("status") == "SUCCESS",
                              resp_a.get("status")))
    assertions.append(_assert("D2 第二次规划成功",
                              resp_b.get("status") == "SUCCESS",
                              resp_b.get("status")))

    digest_a = trajectory_digest(resp_a.get("trajectory"))
    digest_b = trajectory_digest(resp_b.get("trajectory"))
    assertions.append(_assert("D3 两次 trajectory_digest 相同（逐点一致）",
                              digest_a == digest_b,
                              {"first": digest_a, "second": digest_b}))

    points_a = len((resp_a.get("trajectory") or {}).get("points") or [])
    points_b = len((resp_b.get("trajectory") or {}).get("points") or [])
    assertions.append(_assert("D4 两次点位数量一致",
                              points_a == points_b and points_a > 0,
                              {"first": points_a, "second": points_b}))

    spawned = int(after.get("workers_spawned") or 0) - \
        int(before.get("workers_spawned") or 0)
    assertions.append(_assert("D5 两次请求各自 spawn 了独立 worker 进程",
                              spawned >= 2, spawned))

    metrics_a = resp_a.get("metrics") or {}
    metrics_b = resp_b.get("metrics") or {}
    assertions.append(_assert("D6 响应里记录的种子与请求一致",
                              int(metrics_a.get("seed", -1)) == seed
                              and int(metrics_b.get("seed", -1)) == seed,
                              {"first": metrics_a.get("seed"),
                               "second": metrics_b.get("seed")}))

    evidence.append("digest（第一次）= %s" % digest_a)
    evidence.append("digest（第二次）= %s" % digest_b)
    evidence.append("seed=%d，scene_version=%d，两次各 spawn 一个独占 worker"
                    "（workers_spawned 增量=%d）"
                    % (seed, req_a["scene_version"], spawned))
    evidence.append("点位数量：第一次=%d，第二次=%d" % (points_a, points_b))

    passed = all(item["passed"] for item in assertions)
    failed = [item["name"] for item in assertions if not item["passed"]]
    return {
        "id": "deterministic_same_seed",
        "title": "验收③：同请求同种子结果逐点一致（trajectory_digest）",
        "status": "ok" if passed else "error",
        "passed": passed,
        "reason": ("同 (request, seed=%d, scene_version=%d) 两次规划"
                   "（各自独占 worker 进程）digest 相同：%s"
                   % (seed, req_a["scene_version"], digest_a[:16] + "…")
                   if passed else "断言失败: %s" % ", ".join(failed)),
        "next_action": "" if passed else
                       "检查 worker 启动顺序（RNG_setSeed 必须在任何规划之前）"
                       "与 metrics.deterministic",
        "assertions": assertions,
        "evidence": evidence,
        "numbers": {
            "seed": seed,
            "scene_version": req_a["scene_version"],
            "digest": digest_a,
            "digest_second": digest_b,
            "points": points_a,
            "workers_spawned_delta": spawned,
        },
    }


# --------------------------------------------------------------------------
# 7. 主流程
# --------------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S07 rm_planner 一键验收（三条验收 → "
                    "build/reports/s07_planner_acceptance.json）")
    parser.add_argument("--out",
                        default=os.path.join("build", "reports",
                                             "s07_planner_acceptance.json"),
                        help="报告输出路径（默认 build/reports/"
                             "s07_planner_acceptance.json）")
    parser.add_argument("--ws", default=REPO_ROOT,
                        help="工作区根（含 workcell/planner 与 build/model）")
    parser.add_argument("--only", default="",
                        help="只跑指定验收（逗号分隔；可选值: "
                             "timeout_forced_interrupt / crash_isolation / "
                             "deterministic_same_seed；缺省 = 全部）")
    args = parser.parse_args(argv)

    all_ids = ("timeout_forced_interrupt", "crash_isolation",
               "deterministic_same_seed")
    selected = [item.strip() for item in str(args.only).split(",")
                if item.strip()]
    unknown = [item for item in selected if item not in all_ids]
    if unknown:
        parser.error("未知的验收 id: %s（可选: %s）"
                     % (", ".join(unknown), " / ".join(all_ids)))
    selected = selected or list(all_ids)

    ws = os.path.abspath(args.ws)
    out_path = args.out if os.path.isabs(args.out) \
        else os.path.join(REPO_ROOT, args.out)

    base_report: Dict[str, Any] = {
        "version": 1,
        "source": "S07/T6 rm_planner 一键验收脚本"
                  "（scripts/rm_planner_acceptance.py）",
        "mode": "fake",
        "evidence": [FAKE_NOTE, LIVE_NOTE],
        "generated_at": _now(),
        "repo_root": REPO_ROOT,
        "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
    }

    # -- 依赖就绪：模型 --------------------------------------------------
    model_ok, model_source, model_evidence = ensure_model(ws)
    base_report["model_source"] = model_source
    base_report["model_evidence"] = model_evidence
    if not model_ok:
        report = dict(base_report)
        report.update({
            "status": "error",
            "reason": "模型不可用（厂商规划内核无法加载 build/model）",
            "next_action": ("执行 `python3 -m rmctl init` 物化 build/model 后重跑"
                            "（若提示 GLIBCXX，先 `source scripts/rmenv.sh`）"),
            "failures": ["model"],
        })
        _write_json(out_path, report)
        print("模型不可用，验收未运行")
        return 4

    env = _child_env()
    tmp_root = tempfile.mkdtemp(prefix="rm-planner-acceptance-")
    sections: List[Tuple[str, Dict[str, Any]]] = []
    service_counters: Dict[str, Any] = {}
    try:
        # -- 阶段 1：验收 ① 与 ③（共用一个服务进程）----------------------
        if {"timeout_forced_interrupt", "deterministic_same_seed"} \
                & set(selected):
            sock_a = os.path.join(tmp_root, "a.sock")
            server = _start_server(sock_a, ws, env)
            try:
                client = PlannerClient(socket_path=sock_a, ws=ws)
                if "timeout_forced_interrupt" in selected:
                    sections.append(("timeout_forced_interrupt",
                                     check_timeout_forced_interrupt(client)))
                if "deterministic_same_seed" in selected:
                    sections.append(("deterministic_same_seed",
                                     check_deterministic_same_seed(client)))
                service_counters = client.status().get("counters") or {}
            finally:
                try:
                    _kill_tree(server.pid)
                except Exception:  # noqa: BLE001 - 清理不掩盖断言失败
                    pass
                if server.poll() is None:
                    server.wait(timeout=5.0)

        # -- 阶段 2：验收 ②（服务进程被 kill -9）-------------------------
        if "crash_isolation" in selected:
            sock_b = os.path.join(tmp_root, "b.sock")
            sections.append(("crash_isolation",
                             check_crash_isolation(sock_b, ws, env, tmp_root)))
    except Exception as exc:  # noqa: BLE001 - 任何未预期异常 → 诚实失败
        report = dict(base_report)
        report.update({
            "status": "error",
            "reason": "验收脚本异常终止（%s）: %s" % (type(exc).__name__, exc),
            "next_action": "查看 stdout/stderr 后重跑本脚本",
            "failures": ["harness"],
        })
        for name, section in sections:
            report[name] = section
        _write_json(out_path, report)
        print("验收脚本异常终止: %s: %s" % (type(exc).__name__, exc))
        return 4
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)

    failed_sections = [name for name, section in sections
                       if not bool(section.get("passed"))]
    report = dict(base_report)
    report.update({
        "status": "ok" if not failed_sections else "error",
        "reason": ("rm_planner 三条验收全部通过："
                   "① 规划超时被服务端强制中断 / ② 服务崩溃不影响运动执行 / "
                   "③ 同请求同种子结果逐点一致"
                   if not failed_sections else
                   "验收失败段落: %s" % ", ".join(failed_sections)),
        "selected": list(selected),
        "next_action": ("" if not failed_sections else
                        "对照失败段落的 assertions 字段修正后重跑本脚本"),
        "service_counters": service_counters,
        "failures": failed_sections,
    })
    for name, section in sections:
        report[name] = section

    _write_json(out_path, report)

    print("rm_planner 验收报告: %s" % out_path)
    for name, section in sections:
        print("  %-28s %-6s %s" % (name, section.get("status"),
                                   section.get("reason")))
    print("  总体: %s（%s）" % (report["status"], report["reason"]))
    print("注意: %s" % LIVE_NOTE)
    return 0 if not failed_sections else 4


if __name__ == "__main__":
    sys.exit(main())
