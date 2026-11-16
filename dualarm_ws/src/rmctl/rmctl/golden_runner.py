"""黄金场景集 runner（S09/T3）：把 ``tests/golden/planner_scenarios.json``
的**执行逻辑**提成可被脚本 / pytest 共用的函数。

为什么提出来（S07/T5 原本只在 pytest 里）
----------------------------------------

S07 的黄金场景集执行器只存在于 ``tests/test_planner_golden.py``（pytest
夹具 + 断言），一键回归编排器**没法**把它当子进程调用。本模块提供**同一份
实现**：pytest 薄封装（``tests/test_planner_golden.py``）与脚本入口
（``scripts/rm_planner_golden.py``）都调这里的函数 → 断言口径、报告结构、
场景集来源**只有一份**，S08/S09 不另造基准。

场景集来源（**唯一**）
-----------------------

``dualarm_ws/src/rmctl/tests/golden/planner_scenarios.json``（version 1，
git 跟踪、可 diff）。本模块**只读**它，**禁止**另建场景集、**禁止**修改
其中的 ``expect_*`` 冻结值。

报告结构（**与 S07 同构**）
---------------------------

汇总报告 = S07 报告的**超集**：

* S07 既有键（``version`` / ``source`` / ``scenario_count`` / ``p50_s`` /
  ``p95_s`` / ``success_rate`` / ``success_only`` / ``scenarios``）——
  S08 性能基线直接取用，键名与语义**不变**；
* 追加**执行层契约**键（``status`` / ``reason`` / ``next_action`` /
  ``evidence`` / ``failures``）—— 让一键回归编排器用同一套归一化规则
  （``rmctl.regression.orchestrator.normalize_report``）判定段级
  pass / blocked / fail。

口径纪律：

* **缺外部产物 ≠ 回归**：厂商内核不可用（缺 libstdcxx / 缺 build/model）
  → 报告 ``status="blocked"``（**不是** fail），reason 指名缺什么；
* **不静默**：段内断言不过 → ``status="error"`` + ``failures`` 逐条列出
  哪个场景、哪条断言不过；
* **两组延迟分开给**：顶层 ``p50_s`` / ``p95_s`` = 全部场景的端到端墙钟；
  ``success_only.*`` = 只统计 ``status == SUCCESS`` 的场景。

不依赖 pytest：本模块只依赖标准库 + 仓库内既有包（``rmctl``），
厂商内核的可用性检查是**显式**的一步（:func:`check_vendor`）。
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = [
    "MIN_SCENARIOS",
    "REQUIRED_STATUSES",
    "build_request",
    "check_vendor",
    "default_repo_root",
    "determinism_check",
    "evaluate_rows",
    "load_scenarios",
    "percentile",
    "prepare_workspace",
    "run_scenarios",
    "scenarios_path",
    "summarize",
    "write_report",
]

#: 场景集里要求的最小场景数（防止有人把场景集删到只剩一两条还「全绿」）。
MIN_SCENARIOS = 6

#: 场景集必须覆盖的结局（每一种至少一条）——防止场景集退化成「只有成功」。
REQUIRED_STATUSES = ("SUCCESS", "FAILED", "TIMEOUT")

#: 确定性探针用的固定场景（同 seed → 逐点一致）。
_DETERMINISM_SCENARIO = {
    "start": [0.0] * 7,
    "goal": {"type": "joint",
             "values": [0.2, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0]},
    "constraints": {},
    "seed": 11,
    "timeout_ms": 20000,
}


# --------------------------------------------------------------------------
# 0. 路径
# --------------------------------------------------------------------------

def default_repo_root() -> str:
    """仓库根（本文件位于 ``<repo>/dualarm_ws/src/rmctl/rmctl/`` 下）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def scenarios_path() -> str:
    """场景集文件路径（**唯一**来源，git 跟踪）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(
        os.path.join(here, "..", "tests", "golden", "planner_scenarios.json"))


def _ensure_sys_path(repo_root: str) -> None:
    """把 ``rmctl`` 与 ``rm_description/scripts`` 加进 ``sys.path``。"""
    for rel in ("dualarm_ws/src/rmctl", "dualarm_ws/src/rm_description/scripts"):
        path = os.path.join(repo_root, rel)
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)


# --------------------------------------------------------------------------
# 1. 场景集加载（校验规则与 S07 一致）
# --------------------------------------------------------------------------

def load_scenarios(path: Optional[str] = None,
                   only: Optional[Iterable[str]] = None) -> List[Dict[str, Any]]:
    """读场景集（version 1）并做**结构校验**。

    校验项（与 S07/T5 的夹具一致，**不放宽**）：

    * ``version == 1``；
    * 场景数 ≥ :data:`MIN_SCENARIOS`；
    * 场景 id 唯一；
    * 覆盖 :data:`REQUIRED_STATUSES` 里的每一种结局。

    :param only: 只保留这些场景 id（``None`` = 全部）；未知 id → ``ValueError``。
    """
    path = path or scenarios_path()
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("version") != 1:
        raise ValueError("场景集 version 必须是 1，实际 %r"
                         % data.get("version"))
    items = list(data.get("scenarios") or [])
    if len(items) < MIN_SCENARIOS:
        raise ValueError("场景集至少 %d 条，当前 %d 条"
                         % (MIN_SCENARIOS, len(items)))
    ids = [item["id"] for item in items]
    if len(set(ids)) != len(ids):
        raise ValueError("场景 id 必须唯一: %s" % ids)
    covered = {item.get("expect_status") for item in items}
    missing = [status for status in REQUIRED_STATUSES
               if status not in covered]
    if missing:
        raise ValueError("场景集缺少这些结局的覆盖: %s" % missing)
    if only is not None:
        wanted = [str(item) for item in only]
        unknown = [item for item in wanted if item not in set(ids)]
        if unknown:
            raise ValueError("未知的场景 id: %s（可选: %s）"
                             % (", ".join(unknown), ", ".join(ids)))
        keep = set(wanted)
        items = [item for item in items if item["id"] in keep]
    return items


# --------------------------------------------------------------------------
# 2. 请求构造（不补业务缺省：场景写什么就发什么）
# --------------------------------------------------------------------------

def build_request(scenario: Dict[str, Any], plan_id: str) -> Dict[str, Any]:
    """场景 → 契约层请求字典（**不补业务缺省**）。"""
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


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    """线性插值分位数（p50 / p95）。"""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = fraction * (len(ordered) - 1)
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


# --------------------------------------------------------------------------
# 3. 逐场景执行
# --------------------------------------------------------------------------

def run_scenarios(scenarios: Sequence[Dict[str, Any]],
                  client: Any) -> List[Dict[str, Any]]:
    """逐场景跑一遍真实规划服务，返回逐场景行。

    ``client`` 是 :class:`rmctl.planner_service.client.PlannerClient`
    （鸭子类型：只要有 ``plan(request)`` 即可 → 可测）。
    """
    rows: List[Dict[str, Any]] = []
    for scenario in scenarios:
        plan_id = "golden-%s" % scenario["id"]
        request = build_request(scenario, plan_id)
        started = time.time()
        resp = client.plan(request)
        elapsed = time.time() - started
        metrics = resp.get("metrics") or {}
        rows.append({
            "id": scenario["id"],
            "name": scenario.get("name") or "",
            "expect_status": scenario.get("expect_status"),
            "status": resp.get("status"),
            "expect_verify_ok": scenario.get("expect_verify_ok"),
            "verify_ok": (resp.get("verification") or {}).get("ok"),
            "elapsed_s": round(elapsed, 3),
            "max_elapsed_s": float(scenario["max_elapsed_s"]),
            "seed": int(scenario["seed"]),
            "timeout_ms": int(scenario["timeout_ms"]),
            "points": metrics.get("points"),
            "failure_reason": resp.get("failure_reason") or "",
        })
    return rows


def evaluate_rows(rows: Sequence[Dict[str, Any]],
                  scenarios: Optional[Sequence[Dict[str, Any]]] = None
                  ) -> List[str]:
    """逐场景断言 → 失败说明列表（空 = 全部通过）。

    断言三件事（与 S07/T5 一致，**不修改 expect_* 冻结值**）：

    1. ``status == expect_status``；
    2. ``verify_ok == expect_verify_ok``（``None`` = 该场景无 verification，
       如 TIMEOUT 被服务端 SIGKILL）；
    3. ``elapsed_s <= max_elapsed_s``。

    外加场景可选声明的 ``expect_failure_reason_contains``（失败原因指名）。
    """
    by_id = {str(item.get("id")): item for item in (scenarios or [])}
    failures: List[str] = []
    for row in rows:
        sid = str(row.get("id"))
        scenario = by_id.get(sid, {})
        expect_status = row.get("expect_status")
        if row.get("status") != expect_status:
            failures.append(
                "场景 %s: status=%r，期望 %r（failure_reason=%r）"
                % (sid, row.get("status"), expect_status,
                   row.get("failure_reason")))
        expect_verify = row.get("expect_verify_ok")
        if expect_verify is not None and row.get("verify_ok") is not expect_verify:
            failures.append(
                "场景 %s: verification.ok=%r，期望 %r（failure_reason=%r）"
                % (sid, row.get("verify_ok"), expect_verify,
                   row.get("failure_reason")))
        max_elapsed = row.get("max_elapsed_s")
        elapsed = row.get("elapsed_s")
        if max_elapsed is not None and elapsed is not None \
                and float(elapsed) > float(max_elapsed):
            failures.append("场景 %s: 耗时 %.2f s 超过上限 %.2f s"
                            % (sid, float(elapsed), float(max_elapsed)))
        needle = scenario.get("expect_failure_reason_contains")
        if needle and needle not in str(row.get("failure_reason") or ""):
            failures.append("场景 %s: failure_reason=%r，未包含 %r"
                            % (sid, row.get("failure_reason"), needle))
    return failures


def determinism_check(client: Any) -> Dict[str, Any]:
    """同 seed 两次请求 → ``trajectory_digest`` 逐点一致？

    返回 ``{"ok", "digest_a", "digest_b", "status_a", "status_b",
    "reason"}``；``ok`` 为假时 ``reason`` 说明失败原因。
    """
    from rmctl.planner_service.contract import trajectory_digest

    first = client.plan(build_request(_DETERMINISM_SCENARIO, "det-a"))
    second = client.plan(build_request(_DETERMINISM_SCENARIO, "det-b"))
    status_a = first.get("status")
    status_b = second.get("status")
    if status_a != "SUCCESS" or status_b != "SUCCESS":
        return {
            "ok": False,
            "status_a": status_a,
            "status_b": status_b,
            "digest_a": None,
            "digest_b": None,
            "reason": "确定性探针未成功（status_a=%r / status_b=%r）"
                      % (status_a, status_b),
        }
    digest_a = trajectory_digest(first.get("trajectory"))
    digest_b = trajectory_digest(second.get("trajectory"))
    return {
        "ok": digest_a == digest_b,
        "status_a": status_a,
        "status_b": status_b,
        "digest_a": digest_a,
        "digest_b": digest_b,
        "reason": "" if digest_a == digest_b else "同种子两次结果不一致",
    }


# --------------------------------------------------------------------------
# 4. 工作区 / 厂商内核可用性
# --------------------------------------------------------------------------

def prepare_workspace(root: str, repo_root: Optional[str] = None) -> str:
    """物化一个可跑的规划工作区（绝对路径模型 + 复制进来的规划配置）。

    * 模型用 ``build_model.build_model()`` **现场物化**（绝对路径 mesh）；
    * 规划配置从 **git 跟踪** 的 ``workcell/planner/*.yaml`` 复制进来；
    * **不依赖** gitignore 目录（``build/`` 只是产物落点）。

    :return: 工作区根路径
    """
    repo_root = os.path.abspath(repo_root or default_repo_root())
    _ensure_sys_path(repo_root)
    ws = os.path.join(os.path.abspath(root), "golden_ws")
    os.makedirs(ws, exist_ok=True)

    import build_model  # dualarm_ws/src/rm_description/scripts/build_model.py

    # rm_description 源码与规划配置的**唯一来源**：仓库根（git 跟踪）。
    # 调用方给的 repo_root 不是仓库根时（例如只指工作区）→ 回落到本仓库根，
    # 绝不另造第二份配置。
    src_root = repo_root
    if not os.path.isdir(os.path.join(src_root, "dualarm_ws", "src",
                                      "rm_description")):
        src_root = default_repo_root()

    build_model.build_model(
        ws=ws,
        out=os.path.join(ws, "build", "model"),
        pkg=os.path.join(src_root, "dualarm_ws", "src", "rm_description"),
    )
    cfg_dst = os.path.join(ws, "workcell", "planner")
    os.makedirs(cfg_dst, exist_ok=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(src_root, "workcell", "planner", name),
                        os.path.join(cfg_dst, name))
    return ws


def check_vendor(ws: str) -> Tuple[bool, str]:
    """厂商规划内核 + 模型是否可用？

    :return: ``(ok, reason)``；不可用时 ``reason`` 指名「缺 libstdcxx /
        缺 build/model」等可粘贴的下一步。
    """
    try:
        from rmctl.model import Model
    except Exception as exc:  # noqa: BLE001 - 厂商内核 import 失败
        return False, ("厂商规划内核不可用（缺 libstdcxx / 缺 build/model）: %s"
                       % exc)
    try:
        Model(ws=ws)
    except Exception as exc:  # noqa: BLE001 - 模型加载失败
        return False, ("厂商规划内核或模型不可用（缺 libstdcxx / 缺 "
                       "build/model）: %s" % exc)
    return True, ""


# --------------------------------------------------------------------------
# 5. 汇总与落盘
# --------------------------------------------------------------------------

def summarize(rows: Sequence[Dict[str, Any]],
              source: str = "S09/T3 黄金场景集 runner"
                            "（tests/golden/planner_scenarios.json）",
              failures: Optional[Sequence[str]] = None,
              extra: Optional[Dict[str, Any]] = None
              ) -> Dict[str, Any]:
    """逐场景行 → 汇总报告（S07 同构超集）。

    ``status`` 取**执行层契约**口径：``ok``（全通过）/ ``error``（断言不过）。
    blocked 情形由 :func:`blocked_report` 单独产出。
    """
    rows = list(rows)
    failures = list(failures or [])
    elapsed = [float(row["elapsed_s"]) for row in rows]
    success_elapsed = [float(row["elapsed_s"]) for row in rows
                       if row.get("status") == "SUCCESS"]
    matched = [row for row in rows
               if row.get("status") == row.get("expect_status")]
    report: Dict[str, Any] = {
        # -- S07 既有键（语义不变，S08 直接取用）-------------------------
        "version": 1,
        "source": source,
        "scenario_count": len(rows),
        "p50_s": (round(percentile(elapsed, 0.50), 3) if elapsed else None),
        "p95_s": (round(percentile(elapsed, 0.95), 3) if elapsed else None),
        "success_rate": (len(matched) / len(rows)) if rows else 0.0,
        "success_only": {
            "count": len(success_elapsed),
            "p50_s": (round(percentile(success_elapsed, 0.50), 3)
                      if success_elapsed else None),
            "p95_s": (round(percentile(success_elapsed, 0.95), 3)
                      if success_elapsed else None),
        },
        "scenarios": rows,
        # -- 执行层契约键（编排器归一化用）-----------------------------
        "status": "error" if failures else "ok",
        "reason": ("%d 条场景的 status 全部与 expect_status 一致"
                   % len(rows) if not failures else
                   "有 %d 条断言不过: %s" % (len(failures),
                                             "; ".join(failures))),
        "next_action": ("" if not failures else
                        "对照报告 scenarios 行定位失败场景；场景集 "
                        "expect_* 为冻结值，不得改场景集迁就实现"),
        "evidence": ["场景数: %d" % len(rows),
                     "status 匹配数: %d" % len(matched)],
        "failures": failures,
    }
    if extra:
        report.update(extra)
    return report


def blocked_report(reason: str, next_action: str = "",
                   evidence: Optional[Sequence[str]] = None,
                   source: str = "S09/T3 黄金场景集 runner"
                                 "（tests/golden/planner_scenarios.json）"
                   ) -> Dict[str, Any]:
    """外部条件未满足 → blocked 报告（**不是** fail）。"""
    return {
        "version": 1,
        "source": source,
        "scenario_count": 0,
        "p50_s": None,
        "p95_s": None,
        "success_rate": 0.0,
        "success_only": {"count": 0, "p50_s": None, "p95_s": None},
        "scenarios": [],
        "status": "blocked",
        "reason": reason,
        "next_action": next_action or (
            "先 `source scripts/rmenv.sh` 并 `python3 -m rmctl init` "
            "物化 build/model 后重跑"),
        "evidence": list(evidence or []),
        "failures": ["model"],
    }


def write_report(report: Dict[str, Any], path: str) -> str:
    """写报告 JSON（UTF-8、缩进 2、键序稳定）。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=2, default=str)
    return path
