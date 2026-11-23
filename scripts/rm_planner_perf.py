#!/usr/bin/env python3
"""S08 rm_planner **性能基准脚本**（「优化前 vs 优化后」对比表）。

用途
----
同一个黄金场景集（``dualarm_ws/src/rmctl/tests/golden/planner_scenarios.json``，
**S07 交付、版本化、git 跟踪**）跑**两档**配置，产出可复现的前后对比表：

* ``baseline``  —— **S07 冻结档位**（常量写在本脚本里，见 :data:`BASELINE_PROFILE`）；
* ``optimized`` —— **当前** ``workcell/planner/pipeline.yaml`` 的取值
  （唯一配置来源；脚本**只读不改**仓库里的那份）。

两档之间通过**改写临时工作区里的 pipeline.yaml** 实现：引擎
（``rmctl.planner_service.engine``）**每条请求都重新加载**管线配置
（``load_pipeline_config()``），因此换档**不需要重启服务**。若这一点将来
不成立（引擎开始缓存配置），脚本会在报告里留下 ``profiles.*.observed``
与请求里的 ``metrics.max_solutions / optimize / planner_range`` 不一致的
事实（见 :func:`_profile_observed_mismatch`），而不是悄悄跑错档位。

每个场景记录三组口径（**互不替代**，避免后续误读数字）：

1. **e2e 墙钟**（客户端侧，**含 worker 冷启动**：python 启动 + import +
   模型加载）；
2. **纯规划耗时** ``metrics.planning_s``（剔除冷启动）与
   ``metrics.elapsed_total_s``（worker 内全程）；
3. **worker 子进程峰值 RSS**（``/proc/<pid>/status`` 的 ``VmRSS``，10 ms
   轮询）—— 规划内存的**真正**载体是 worker 子进程，服务端进程 RSS
   （十几 MB）**不代表**规划内存。

用法
----
    python3 scripts/rm_planner_perf.py --smoke \
        --out build/reports/s08_planner_perf_smoke.json
    python3 scripts/rm_planner_perf.py \
        --out build/reports/s08_planner_perf.json
    python3 scripts/rm_planner_perf.py --profile optimized

退出码
------
* ``0`` —— 跑完且两档的每个场景 ``status`` 都与 ``expect_status`` 一致；
* ``4`` —— 有场景不匹配（回归破了）或脚本异常终止（报告写明**哪一档、
  哪条场景**、期望与实际）。

零新依赖：只用标准库 + 仓库内既有包（``rmctl`` / ``rm_executor`` /
``rm_hw_bridge`` / ``rm_safety`` / ``rm_description``）+ ``yaml``。
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

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

import yaml  # noqa: E402

from rmctl.planner_service.client import PlannerClient  # noqa: E402
from rmctl.planner_service.server import PlannerServer  # noqa: E402
from rmctl.vendor_env import ensure_libstdcxx  # noqa: E402

ensure_libstdcxx()

#: 黄金场景集（git 跟踪；**不是** gitignore 目录里的东西）。
SCENARIOS_PATH = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl",
                              "tests", "golden", "planner_scenarios.json")

#: 规划配置目录（唯一配置来源）。
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")

#: 报告默认落盘位置（运行时产物，gitignore）。
DEFAULT_OUT = os.path.join("build", "reports", "s08_planner_perf.json")

# --------------------------------------------------------------------------
# 1. 档位定义
# --------------------------------------------------------------------------

#: **S07 冻结档位**（"优化前"）。
#:
#: 等价于 S07 交付时 ``workcell/planner/pipeline.yaml`` 的取值：
#: ``planner_range=0.01``、``max_solutions=10``、``optimize=true``、
#: ``planning_time_s=5.0``、``num_planners=2``。
#:
#: 冻结成常量是刻意的：**S08 的"优化前"必须永远指向 S07 的基线**，
#: 不能因为后来有人改了 pipeline.yaml 就悄悄漂移。
BASELINE_PROFILE: Dict[str, Any] = {
    "planner_range": 0.01,
    "max_solutions": 10,
    "optimize": True,
    "planning_time_s": 5.0,
    "num_planners": 2,
}

#: "优化后"档位 = 当前 ``workcell/planner/pipeline.yaml`` 的取值
#: （脚本**只读不改**仓库里的那份）。
OPTIMIZED_PROFILE_SOURCE = "workcell/planner/pipeline.yaml"

#: 冒烟模式跑的代表场景（一个成功 + 一个「规划成功但校验不过」）。
SMOKE_SCENARIO_IDS = ("joint_small_local", "limit_violation_jerk")

#: RSS 采样间隔（秒）—— 10 ms。
RSS_SAMPLE_INTERVAL_S = 0.01


# --------------------------------------------------------------------------
# 2. 小工具
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


def _read_json(path: str) -> Dict[str, Any]:
    """读 JSON 文件。"""
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    """线性插值分位数（与 S07 黄金场景集的算法一致，便于横向比数字）。"""
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def median(values: Sequence[float]) -> Optional[float]:
    """中位数（偶数个取中间两点的均值）。"""
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return float(ordered[mid])
    return float((ordered[mid - 1] + ordered[mid]) / 2.0)


def _r3(value: Optional[float]) -> Optional[float]:
    """四舍五入到 3 位（报告可读性）。"""
    return None if value is None else round(float(value), 3)


def _git_rev() -> Optional[str]:
    """当前 git 提交（读不到就诚实留 ``None``，不伪造）。"""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=10,
        )
    except Exception:  # noqa: BLE001 - 没有 git / 超时 → 留空
        return None
    if out.returncode != 0:
        return None
    rev = (out.stdout or "").strip()
    return rev or None


def _machine_info() -> Dict[str, Any]:
    """机器信息（uname / CPU 数 / 负载）—— 性能数字的可复现前提。"""
    loadavg: List[float] = []
    try:
        loadavg = [round(float(x), 2) for x in os.getloadavg()]
    except OSError:  # pragma: no cover - 平台差异
        loadavg = []
    return {
        "uname": list(platform.uname()),
        "cpu_count": os.cpu_count(),
        "loadavg": loadavg,
        "python": platform.python_version(),
    }


# --------------------------------------------------------------------------
# 3. 临时工作区（模型 + 规划配置）
# --------------------------------------------------------------------------

def _materialize_workspace(tmp_root: str) -> str:
    """物化临时工作区：绝对路径模型 + 复制进来的规划配置。

    **不读 gitignore 目录**（``build/`` 等）：模型用
    ``build_model.build_model()`` 现场物化，规划配置从 git 跟踪的
    ``workcell/planner/*.yaml`` 复制进来。

    :return: 工作区根路径
    """
    ws = os.path.join(tmp_root, "perf_ws")
    os.makedirs(ws, exist_ok=True)

    import build_model  # dualarm_ws/src/rm_description/scripts/build_model.py

    build_model.build_model(
        ws=ws,
        out=os.path.join(ws, "build", "model"),
        pkg=os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description"),
    )

    cfg_dst = os.path.join(ws, "workcell", "planner")
    os.makedirs(cfg_dst, exist_ok=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name),
                        os.path.join(cfg_dst, name))
    return ws


def _read_repo_pipeline() -> Dict[str, Any]:
    """读仓库里的 ``workcell/planner/pipeline.yaml``（只读）。"""
    path = os.path.join(PLANNER_CFG_DIR, "pipeline.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError("pipeline.yaml 顶层不是映射: %r" % (data,))
    return data


def _write_pipeline(ws: str, data: Dict[str, Any]) -> str:
    """把管线配置写进临时工作区（**只改临时工作区的那份**）。

    引擎**每条请求**都重新 ``load_pipeline_config()`` → 换档无需重启服务。
    """
    path = os.path.join(ws, "workcell", "planner", "pipeline.yaml")
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    return path


def _profile_params(data: Dict[str, Any]) -> Dict[str, Any]:
    """管线配置里与**性能档位**有关的字段（报告里逐档列出）。"""
    keys = ("planner_range", "max_solutions", "optimize", "planning_time_s",
            "num_planners", "pipeline", "deterministic",
            "verify_resample_step_rad", "output_resample_step_rad")
    return {key: data.get(key) for key in keys}


# --------------------------------------------------------------------------
# 4. worker 峰值 RSS 采样
# --------------------------------------------------------------------------

def _read_vmrss_kb(pid: int) -> Optional[int]:
    """读 ``/proc/<pid>/status`` 的 ``VmRSS``（kB）；读不到 → ``None``。"""
    try:
        with open("/proc/%d/status" % int(pid), "r", encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    parts = line.split()
                    return int(parts[1])
    except (OSError, ValueError, IndexError):
        return None
    return None


def _ppid_map() -> Dict[int, int]:
    """``{pid: ppid}``（扫 ``/proc``；不依赖 ps / psutil）。"""
    out: Dict[int, int] = {}
    try:
        names = os.listdir("/proc")
    except OSError:  # pragma: no cover - 非 Linux
        return out
    for name in names:
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


def _descendant_pids(root_pid: int) -> List[int]:
    """某进程的**全部后代**（深度优先）。"""
    ppids = _ppid_map()
    out: List[int] = []
    frontier = [int(root_pid)]
    while frontier:
        current = frontier.pop()
        for child, parent in ppids.items():
            if parent == current and child not in out:
                out.append(child)
                frontier.append(child)
    return out


def _is_worker_cmdline(pid: int) -> bool:
    """进程是不是规划 worker（``rmctl.planner_service.worker``）。"""
    try:
        with open("/proc/%d/cmdline" % int(pid), "rb") as fh:
            raw = fh.read().decode("utf-8", errors="replace")
    except OSError:
        return False
    return "planner_service.worker" in raw


class RssSampler:
    """worker 子进程峰值 RSS 采样器（10 ms 轮询 ``/proc/<pid>/status``）。

    两个 pid 来源（按优先级）：

    1. ``PlannerServer.active_workers()`` —— 服务端在飞 worker 的**权威** pid；
    2. ``/proc`` 进程树扫描（``rm_planner_acceptance.py`` 的既有模式）——
       取不到（1）时的退路；只认 ``rmctl.planner_service.worker`` 的 cmdline。

    采样结果保留 ``(时刻, pid, VmRSS)`` 三元组，**按请求窗口**切片 → 每个
    场景拿到**自己那条请求**期间的 worker 峰值 RSS。
    """

    #: 退路（进程树扫描）的最小间隔（秒）—— 扫 ``/proc`` 全表很贵，
    #: 不能每 10 ms 扫一次（会把采样本身变成负载）。
    FALLBACK_INTERVAL_S = 0.5

    def __init__(self, server: PlannerServer, interval_s: float = RSS_SAMPLE_INTERVAL_S):
        self.server = server
        self.interval = float(interval_s)
        self.samples: List[Tuple[float, int, int]] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._last_fallback = 0.0

    # -- 生命周期 ------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="rm-perf-rss",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

    # -- 采样 ----------------------------------------------------------
    def _worker_pids(self) -> List[int]:
        """当前 worker pid 列表（权威来源优先，退路兜底）。"""
        pids: List[int] = []
        try:
            pids = [int(pid) for pid in self.server.worker_pids() if pid]
        except Exception:  # noqa: BLE001 - 服务端状态取不到 → 走退路
            pids = []
        if pids:
            return pids
        # 退路：进程树扫描（只认 worker 的 cmdline）；限频（扫全表很贵）。
        now = time.time()
        if now - self._last_fallback < self.FALLBACK_INTERVAL_S:
            return []
        self._last_fallback = now
        out: List[int] = []
        for pid in _descendant_pids(os.getpid()):
            if _is_worker_cmdline(pid):
                out.append(pid)
        return out

    def _run(self) -> None:
        while not self._stop.is_set():
            stamp = time.time()
            for pid in self._worker_pids():
                rss = _read_vmrss_kb(pid)
                if rss is None:
                    continue
                with self._lock:
                    self.samples.append((stamp, pid, int(rss)))
            self._stop.wait(self.interval)

    # -- 查询 ----------------------------------------------------------
    def window(self, t0: float, t1: float) -> Dict[str, Any]:
        """某个时间窗口内的 worker 峰值 RSS（kB / MB）与涉及的 pid。"""
        with self._lock:
            picked = [(pid, rss) for (stamp, pid, rss) in self.samples
                      if t0 <= stamp <= t1]
        if not picked:
            return {"peak_rss_kb": None, "peak_rss_mb": None, "pids": []}
        peak_kb = max(rss for _, rss in picked)
        pids = sorted({pid for pid, _ in picked})
        return {
            "peak_rss_kb": int(peak_kb),
            "peak_rss_mb": round(peak_kb / 1024.0, 2),
            "pids": pids,
        }


# --------------------------------------------------------------------------
# 5. 场景 → 请求
# --------------------------------------------------------------------------

def build_request(scenario: Dict[str, Any], plan_id: str) -> Dict[str, Any]:
    """场景 → 契约层请求字典（**不补业务缺省**：场景写什么就发什么）。

    与 ``tests/test_planner_golden.py::build_request`` 同构：同一份场景集，
    同一份请求构造（否则前后对比不可比）。
    """
    data: Dict[str, Any] = {
        "plan_id": plan_id,
        "profile": "FREESPACE",
        "group": "rm75_arm",
        "start_state": list(scenario["start"]),
        "goal": {
            "type": scenario["goal"]["type"],
            "values": list(scenario["goal"]["values"]),
        },
        "constraints": dict(scenario.get("constraints") or {}),
        "seed": int(scenario["seed"]),
        "timeout_ms": int(scenario["timeout_ms"]),
        "scene_version": 1,
    }
    obstacle = scenario.get("obstacle")
    if obstacle:
        data["scene"] = {"obstacles": [dict(obstacle)]}
    return data


def _observed_profile(metrics: Dict[str, Any]) -> Dict[str, Any]:
    """从响应 metrics 里读回**实际生效**的档位（防止跑错档还看不出来）。"""
    return {
        "max_solutions": metrics.get("max_solutions"),
        "optimize": metrics.get("optimize"),
        "planner_range": metrics.get("planner_range"),
        "num_planners": metrics.get("num_planners"),
    }


def run_profile(
    profile_name: str,
    pipeline_data: Dict[str, Any],
    ws: str,
    scenarios: Sequence[Dict[str, Any]],
    client: PlannerClient,
    sampler: RssSampler,
) -> List[Dict[str, Any]]:
    """跑一档配置的全部场景，返回逐场景行。"""
    _write_pipeline(ws, pipeline_data)
    rows: List[Dict[str, Any]] = []
    for scenario in scenarios:
        plan_id = "perf-%s-%s" % (profile_name, scenario["id"])
        request = build_request(scenario, plan_id)
        t0 = time.time()
        resp = client.plan(request)
        t1 = time.time()
        wall = t1 - t0
        metrics = resp.get("metrics") or {}
        window = sampler.window(t0, t1)
        status = str(resp.get("status") or "")
        expect = str(scenario.get("expect_status") or "")
        rows.append({
            "profile": profile_name,
            "id": scenario["id"],
            "name": scenario.get("name") or "",
            "expect_status": expect,
            "status": status,
            "status_match": status == expect,
            "verify_ok": (resp.get("verification") or {}).get("ok"),
            "expect_verify_ok": scenario.get("expect_verify_ok"),
            "e2e_wall_s": round(wall, 3),
            "planning_s": metrics.get("planning_s"),
            "elapsed_total_s": metrics.get("elapsed_total_s"),
            "points": metrics.get("points"),
            "trajectory_points": metrics.get("trajectory_points"),
            "verified_points": metrics.get("verified_points"),
            "output_points": metrics.get("output_points"),
            "worker_peak_rss_mb": window["peak_rss_mb"],
            "worker_peak_rss_kb": window["peak_rss_kb"],
            "worker_pids": window["pids"],
            "observed_profile": _observed_profile(metrics),
            "failure_reason": str(resp.get("failure_reason") or ""),
        })
    return rows


# --------------------------------------------------------------------------
# 6. 汇总
# --------------------------------------------------------------------------

def summarize(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """一档配置的汇总（延迟两组 / 成功率 / 峰值 RSS / 点数中位数）。"""
    walls = [float(row["e2e_wall_s"]) for row in rows]
    success_rows = [row for row in rows if row["status"] == "SUCCESS"]
    success_walls = [float(row["e2e_wall_s"]) for row in success_rows]
    matched = [row for row in rows if row["status_match"]]
    rss_values = [float(row["worker_peak_rss_mb"]) for row in rows
                  if row.get("worker_peak_rss_mb") is not None]
    point_values = [float(row["points"]) for row in rows
                    if row.get("points") is not None]
    planning_values = [float(row["planning_s"]) for row in rows
                       if row.get("planning_s") is not None]
    return {
        "scenario_count": len(rows),
        "p50_s": _r3(percentile(walls, 0.50)),
        "p95_s": _r3(percentile(walls, 0.95)),
        "success_only": {
            "count": len(success_rows),
            "p50_s": _r3(percentile(success_walls, 0.50)),
            "p95_s": _r3(percentile(success_walls, 0.95)),
        },
        "planning_only": {
            "count": len(planning_values),
            "p50_s": _r3(percentile(planning_values, 0.50)),
            "p95_s": _r3(percentile(planning_values, 0.95)),
        },
        "success_rate": round(len(matched) / len(rows), 4) if rows else 0.0,
        "status_matches": len(matched),
        "peak_rss_mb_max": (round(max(rss_values), 2) if rss_values else None),
        "points_median": (round(median(point_values), 1)
                          if point_values else None),
    }


def _delta(before: Optional[float], after: Optional[float]) -> Optional[float]:
    """变化量（after - before）；任一缺失 → ``None``。"""
    if before is None or after is None:
        return None
    return round(float(after) - float(before), 3)


def compare(summaries: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """优化前 vs 优化后 并排对比表。"""
    before = summaries.get("baseline") or {}
    after = summaries.get("optimized") or {}
    before_success = before.get("success_only") or {}
    after_success = after.get("success_only") or {}
    metrics = {
        "success_only_p95_s": (before_success.get("p95_s"),
                               after_success.get("p95_s")),
        "success_only_p50_s": (before_success.get("p50_s"),
                               after_success.get("p50_s")),
        "p95_s_all": (before.get("p95_s"), after.get("p95_s")),
        "planning_only_p95_s": (
            (before.get("planning_only") or {}).get("p95_s"),
            (after.get("planning_only") or {}).get("p95_s"),
        ),
        "peak_rss_mb_max": (before.get("peak_rss_mb_max"),
                            after.get("peak_rss_mb_max")),
        "points_median": (before.get("points_median"),
                          after.get("points_median")),
        "success_rate": (before.get("success_rate"), after.get("success_rate")),
    }
    table: Dict[str, Any] = {}
    for key, (old, new) in metrics.items():
        table[key] = {
            "baseline": old,
            "optimized": new,
            "delta": _delta(old, new),
        }
    return table


# --------------------------------------------------------------------------
# 7. 主流程
# --------------------------------------------------------------------------

def _load_scenarios(smoke: bool) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """读黄金场景集（``smoke`` → 只留两个代表场景）。"""
    data = _read_json(SCENARIOS_PATH)
    scenarios = list(data.get("scenarios") or [])
    if smoke:
        picked = [item for item in scenarios
                  if item.get("id") in SMOKE_SCENARIO_IDS]
        missing = [sid for sid in SMOKE_SCENARIO_IDS
                   if sid not in {item.get("id") for item in picked}]
        if missing:
            raise ValueError("场景集里缺少冒烟场景: %s" % ", ".join(missing))
        scenarios = picked
    meta = {
        "path": os.path.relpath(SCENARIOS_PATH, REPO_ROOT),
        "version": data.get("version"),
        "scenario_count": len(scenarios),
        "scenario_ids": [item.get("id") for item in scenarios],
        "smoke": bool(smoke),
    }
    return scenarios, meta


def _profile_observed_mismatch(rows: Sequence[Dict[str, Any]],
                               expected: Dict[str, Any]) -> List[str]:
    """响应里读回的档位与本档期望不一致 → 记一条不一致（防"跑错档"）。"""
    problems: List[str] = []
    for row in rows:
        observed = row.get("observed_profile") or {}
        for key in ("max_solutions", "optimize", "planner_range"):
            want = expected.get(key)
            got = observed.get(key)
            if want is None or got is None:
                continue
            if isinstance(want, float) or isinstance(got, float):
                same = abs(float(want) - float(got)) < 1e-9
            else:
                same = want == got
            if not same:
                problems.append(
                    "%s/%s: 期望 %r 实际 %r（档位未生效？）"
                    % (row.get("profile"), key, want, got))
    return problems


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="S08 rm_planner 性能基准（优化前 vs 优化后 → "
                    "build/reports/s08_planner_perf.json）")
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="报告输出路径（默认 %s）" % DEFAULT_OUT)
    parser.add_argument("--profile", default="both",
                        choices=("baseline", "optimized", "both"),
                        help="跑哪一档（默认 both）")
    parser.add_argument("--smoke", action="store_true",
                        help="冒烟：只跑 %s 两个代表场景"
                             % ", ".join(SMOKE_SCENARIO_IDS))
    args = parser.parse_args(argv)

    out_path = args.out if os.path.isabs(args.out) \
        else os.path.join(REPO_ROOT, args.out)

    base_report: Dict[str, Any] = {
        "version": 1,
        "source": "S08/T3 rm_planner 性能基准脚本"
                  "（scripts/rm_planner_perf.py）",
        "generated_at": _now(),
        "repo_root": REPO_ROOT,
        "script": os.path.relpath(os.path.abspath(__file__), REPO_ROOT),
        "machine": _machine_info(),
        "git_rev": _git_rev(),
        "notes": [
            "e2e 墙钟 = 客户端侧耗时（含 worker 冷启动：python 启动 + import "
            "+ 模型加载）",
            "纯规划耗时 = metrics.planning_s（剔除冷启动）",
            "内存口径 = worker 子进程峰值 RSS（/proc/<pid>/status 的 VmRSS，"
            "10 ms 轮询）；服务端进程 RSS 不代表规划内存",
            "baseline = S07 冻结档位常量；optimized = 当前 "
            "workcell/planner/pipeline.yaml 的取值",
        ],
    }

    try:
        scenarios, scenario_meta = _load_scenarios(bool(args.smoke))
        base_report["scenario_set"] = scenario_meta

        repo_pipeline = _read_repo_pipeline()
        profiles: Dict[str, Any] = {}
        pipelines: Dict[str, Dict[str, Any]] = {}
        if args.profile in ("baseline", "both"):
            data = dict(repo_pipeline)
            data.update(BASELINE_PROFILE)
            pipelines["baseline"] = data
            profiles["baseline"] = {
                "source": "S07 冻结档位（脚本常量 BASELINE_PROFILE）",
                "params": _profile_params(data),
            }
        if args.profile in ("optimized", "both"):
            data = dict(repo_pipeline)
            pipelines["optimized"] = data
            profiles["optimized"] = {
                "source": OPTIMIZED_PROFILE_SOURCE,
                "params": _profile_params(data),
            }
        base_report["profiles"] = profiles

        tmp_root = tempfile.mkdtemp(prefix="rm-planner-perf-")
        try:
            ws = _materialize_workspace(tmp_root)
            sock_path = os.path.join(tmp_root, "rm_planner.sock")
            server = PlannerServer(socket_path=sock_path, ws=ws,
                                   worker_pool_size=1)
            server.start()
            client = PlannerClient(socket_path=sock_path, ws=ws)
            sampler = RssSampler(server)
            sampler.start()
            rows: List[Dict[str, Any]] = []
            try:
                for name in ("baseline", "optimized"):
                    if name not in pipelines:
                        continue
                    rows.extend(run_profile(name, pipelines[name], ws,
                                            scenarios, client, sampler))
            finally:
                sampler.stop()
                server.stop()
        finally:
            shutil.rmtree(tmp_root, ignore_errors=True)

        summaries = {name: summarize([row for row in rows
                                       if row["profile"] == name])
                     for name in pipelines}
        base_report["results"] = rows
        base_report["summary"] = summaries
        base_report["comparison"] = compare(summaries)

        mismatches = [
            "%s/%s: status=%s，期望 %s（failure_reason=%s）"
            % (row["profile"], row["id"], row["status"], row["expect_status"],
               row["failure_reason"])
            for row in rows if not row["status_match"]
        ]
        drift: List[str] = []
        for name, data in pipelines.items():
            drift.extend(_profile_observed_mismatch(
                [row for row in rows if row["profile"] == name],
                BASELINE_PROFILE if name == "baseline" else
                _profile_params(data)))

        base_report["status"] = "ok" if not mismatches and not drift else "error"
        base_report["reason"] = (
            "两档跑完，%d 条场景的 status 全部与 expect_status 一致"
            % len(rows) if not mismatches and not drift else
            "不匹配: %s" % "; ".join(mismatches or drift))
        base_report["next_action"] = (
            "" if not mismatches and not drift else
            "对照 build/reports/s08_planner_perf.json 的 results 行定位；"
            "档位漂移说明引擎开始缓存 pipeline 配置 → 需每档各起一个服务")
        base_report["failures"] = mismatches
        base_report["profile_drift"] = drift
        _write_json(out_path, base_report)
    except Exception as exc:  # noqa: BLE001 - 任何未预期异常 → 诚实失败
        report = dict(base_report)
        report.update({
            "status": "error",
            "reason": "性能基准脚本异常终止（%s）: %s" % (type(exc).__name__, exc),
            "next_action": "查看 stdout/stderr 后重跑本脚本",
            "failures": ["harness"],
        })
        _write_json(out_path, report)
        print("性能基准脚本异常终止: %s: %s" % (type(exc).__name__, exc))
        return 4

    print("rm_planner 性能报告: %s" % out_path)
    for name, summary in base_report["summary"].items():
        success = summary.get("success_only") or {}
        print("  %-9s success_only p50/p95 = %s / %s s | 峰值 RSS = %s MB | "
              "点数中位数 = %s | 成功率 = %s"
              % (name, success.get("p50_s"), success.get("p95_s"),
                 summary.get("peak_rss_mb_max"), summary.get("points_median"),
                 summary.get("success_rate")))
    print("  总体: %s（%s）" % (base_report["status"], base_report["reason"]))
    return 0 if base_report["status"] == "ok" else 4


if __name__ == "__main__":
    sys.exit(main())
