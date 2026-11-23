"""性能基线表 S1–S5（S09/T4）：把 S08 性能报告**投影**成可版本化、可 diff 的
CSV + Markdown 基线表。

做什么 / 不做什么
------------------

* **做**：读声明式场景表（``workcell/bench/perf_scenarios.yaml``）+ 读 S08
  性能脚本产出的报告 JSON（``build/reports/s08_planner_perf.json``）→ 投影成
  ``build/reports/s09_perf_baseline.csv`` 与 ``.md``（逐套件一行）。
* **不做**：**不重测**、**不另造基准**、**不重定义场景集**。S1–S5 的场景与组数
  要求来自设计 §6.9，实测数字只来自 S08 的既有报告。

诚实口径（硬纪律）
------------------

1. **S4 / S5 = not_applicable**：本里程碑是单臂（``JOINT_GROUP=rm75_arm``，
   7 DOF），双臂场景需双臂工位 → 这两行**不产出任何数字**（空单元格），
   只写理由（理由来自 YAML 的 ``not_applicable_reason``）。
2. **组数差距必须写出来**：实测组数 < 设计 §6.9 要求的组数 → 写进 ``notes``
   （CSV 与 Markdown 都写），例如「黄金场景集 2 组，S1 要求 100 组 → 未达组数
   要求」。**不得**假装达标。
3. **缺外部产物 ≠ 回归**：S08 报告缺失（干净机器上没跑过性能脚本）→ 段级
   ``blocked``，``next_action`` 指名 ``python3 scripts/rm_planner_perf.py``。
4. **契约回归 = fail**：S08 报告结构变化（缺 ``summary`` / ``comparison`` /
   ``results`` 键）→ 段级 ``fail``，reason **指名缺哪个键**。
5. **每个数值行带 mode 标注**：本表的数字是**离线实测**（真实规划内核 + 离线
   工位，无真机）→ ``mode=offline``，不得冒充真机结论。
6. **不 import tesseract**（顶层禁止）：数据只来自 JSON / YAML 投影。

依赖方向：本模块属于体验层 ``rmctl``，只读**数据文件**（YAML/JSON）与
执行层契约常量（经 :mod:`rmctl.regression.segments`）；**不** import 硬件适配
层、**不** import 厂商规划内核。
"""
from __future__ import annotations

import csv
import json
import os
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Sequence

from rmctl.regression import segments as seg_mod

__all__ = [
    "CSV_COLUMNS",
    "DEFAULT_PERF_REPORT",
    "MODE",
    "PROFILE",
    "REQUIRED_REPORT_KEYS",
    "build_rows",
    "load_suites",
    "run_perf_baseline",
    "scenarios_path",
    "validate_suites",
    "write_csv",
    "write_markdown",
]

#: 场景声明文件（**唯一**来源，git 跟踪）
SCENARIOS_REL = os.path.join("workcell", "bench", "perf_scenarios.yaml")

#: S08 性能报告默认路径（相对工作区；运行时产物）
DEFAULT_PERF_REPORT = os.path.join("build", "reports", "s08_planner_perf.json")

#: S08 性能报告的**版本库留存副本**（git 跟踪）——干净 checkout 上也能复现本段。
#:
#: 背景（S16）：``build/`` 被 gitignore → 性能报告只存在于运行时目录 →
#: 干净 checkout 跑一键回归时 ``perf_baseline`` 段会以「S08 性能报告缺失」
#: 判 blocked（**可复现性缺口**：不是回归失败，而是证据产物没入库）。
#: 因此把同一份报告固化进版本库，本段在运行时产物缺失时回落到它。
FALLBACK_PERF_REPORT_REL = os.path.join(
    "workcell", "bench", "perf_baseline", "s08_planner_perf.json")

#: 本表产出的两个文件（相对工作区 ``build/reports/``）
CSV_NAME = "s09_perf_baseline.csv"
MD_NAME = "s09_perf_baseline.md"

#: 取哪一档的实测行（S08 报告里 ``baseline`` = S07 冻结档、``optimized`` =
#: 当前 ``workcell/planner/pipeline.yaml``）→ 基线表取**当前配置**那一档。
PROFILE = "optimized"

#: 本表所有数字的 mode 标注（离线实测：真实规划内核 + 离线工位，无真机）
MODE = "offline"

#: S08 报告必须具备的键（缺任一 → 契约回归 → 段级 fail，并指名缺哪个键）
REQUIRED_REPORT_KEYS = ("summary", "comparison", "results")

#: CSV 列（计划要求的列在前，附加列在后）
CSV_COLUMNS = (
    "suite",
    "name",
    "scenario_id",
    "group_count",
    "measured_count",
    "p50_s",
    "p95_s",
    "max_s",
    "success_rate",
    "peak_rss_mb",
    "mode",
    "source_report",
    "git_rev",
    "notes",
)


# ---------------------------------------------------------------------------
# 1. 场景声明（唯一来源）
# ---------------------------------------------------------------------------
def default_repo_root() -> str:
    """仓库根（本文件位于 ``<repo>/dualarm_ws/src/rmctl/rmctl/regression/`` 下）。

    2026-11-23（S16）修正：此前多写了一个 ``..``（按模块在 ``rmctl/rmctl/``
    时的层级计算），实际返回 ``<repo>/dualarm_ws`` → 场景声明与版本库留存
    副本都找不到（默认调用路径会误判「声明缺失」）。此处按真实层级上溯。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def scenarios_path(repo_root: Optional[str] = None) -> str:
    """场景声明文件路径（``workcell/bench/perf_scenarios.yaml``）。"""
    root = os.path.abspath(repo_root or default_repo_root())
    return os.path.join(root, SCENARIOS_REL)


def load_suites(repo_root: Optional[str] = None,
                path: Optional[str] = None) -> List[Dict[str, Any]]:
    """读场景声明 YAML → 套件列表（并做**结构校验**）。

    校验失败 → ``ValueError``（绝不静默按 pass 处理）。

    * 顶层是映射、含 ``suites`` 列表；
    * 每个套件有唯一 ``id``；
    * ``applicable`` 为布尔；
    * ``applicable: false`` → 必须有非空 ``not_applicable_reason``；
    * ``applicable: true`` → 必须有非空 ``scenario_ids``（**负例护栏**：
      把 S4 改成适用却不给数据 → 直接报错，杜绝用假数据冒充双臂场景）；
    * ``group_count`` 是正整数。
    """
    target = path or scenarios_path(repo_root)
    import yaml  # 顶层不 import 厂商内核；yaml 是纯声明解析

    with open(target, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError("场景声明顶层不是映射: %r" % (type(data).__name__,))
    raw = data.get("suites")
    if not isinstance(raw, (list, tuple)) or not raw:
        raise ValueError("场景声明缺 suites 列表")
    suites: List[Dict[str, Any]] = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError("第 %d 个套件不是映射: %r" % (index, item))
        suite = dict(item)
        suite["scenario_ids"] = [str(s) for s in (item.get("scenario_ids") or [])]
        suite["not_applicable_reason"] = str(
            item.get("not_applicable_reason") or "").strip()
        suite["caveat"] = str(item.get("caveat") or "").strip()
        suites.append(suite)
    validate_suites(suites)
    return suites


def validate_suites(suites: Sequence[Dict[str, Any]]) -> None:
    """套件声明结构校验（:func:`load_suites` 复用；测试可直接调用）。"""
    seen = set()
    for index, suite in enumerate(suites):
        if not isinstance(suite, dict):
            raise ValueError("第 %d 个套件不是映射: %r" % (index, suite))
        sid = str(suite.get("id") or "").strip()
        if not sid:
            raise ValueError("第 %d 个套件缺 id" % index)
        if sid in seen:
            raise ValueError("套件 id 重复: %s" % sid)
        seen.add(sid)
        group_count = suite.get("group_count")
        if not isinstance(group_count, int) or isinstance(group_count, bool) \
                or group_count <= 0:
            raise ValueError("套件 %s 的 group_count 必须是正整数，实际 %r"
                             % (sid, group_count))
        applicable = suite.get("applicable")
        if not isinstance(applicable, bool):
            raise ValueError("套件 %s 的 applicable 必须是布尔，实际 %r"
                             % (sid, applicable))
        scenario_ids = suite.get("scenario_ids") or []
        if applicable:
            if not scenario_ids:
                raise ValueError(
                    "套件 %s 声明 applicable=true 却没有映射任何黄金场景 id"
                    "（禁止用假数据冒充；双臂场景需要双臂工位，本里程碑不适用）"
                    % sid)
        else:
            if not suite.get("not_applicable_reason"):
                raise ValueError("套件 %s 声明 applicable=false 但缺 "
                                 "not_applicable_reason" % sid)


# ---------------------------------------------------------------------------
# 2. 投影：S08 报告 → 基线表行
# ---------------------------------------------------------------------------
def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    """线性插值分位数（与 S07/S08 的算法一致，便于横向比数字）。"""
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


def _rows_for_profile(results: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """取当前配置档（``optimized``）的行；没有则退回全部行。"""
    picked = [row for row in results
              if str(row.get("profile") or "") == PROFILE]
    if picked:
        return picked
    return [row for row in results if not row.get("profile")]


def build_rows(suites: Sequence[Dict[str, Any]],
               report: Dict[str, Any],
               source_report: str = "",
               profile: str = PROFILE) -> List[Dict[str, Any]]:
    """场景声明 + S08 报告 → **逐套件一行**的基线表行。

    行字段（= :data:`CSV_COLUMNS`）：

    * ``suite`` / ``name``：套件 id 与名称（来自 YAML）；
    * ``scenario_id``：映射到的黄金场景 id（逗号分隔）；
    * ``group_count``：设计 §6.9 要求的组数（来自 YAML）；
    * ``measured_count``：本次实际实测到的组数（S08 报告里的行数）；
    * ``p50_s`` / ``p95_s`` / ``max_s``：端到端墙钟（毫秒级精度，3 位小数）；
    * ``success_rate``：**与 S07 同口径** = 实测 status 与期望一致的占比；
    * ``peak_rss_mb``：worker 子进程峰值 RSS（MB）；
    * ``mode``：恒为 ``offline``（离线实测）；
    * ``source_report`` / ``git_rev``：数字出处（S08 报告路径 + 该报告记录的
      git 版本；读不到就留空，**不伪造**）；
    * ``notes``：**诚实标注**（组数差距 / not_applicable 理由 / caveat）。

    ``applicable: false`` 的套件 → 数值列**全为空**（不生成任何假数字）。
    """
    results = report.get("results")
    if not isinstance(results, (list, tuple)):
        results = []
    picked = _rows_for_profile(results)
    by_id: Dict[str, List[Dict[str, Any]]] = {}
    for row in picked:
        if isinstance(row, dict):
            by_id.setdefault(str(row.get("id") or ""), []).append(row)

    git_rev = str(report.get("git_rev") or "").strip()
    out: List[Dict[str, Any]] = []
    for suite in suites:
        sid = str(suite.get("id") or "")
        group_count = int(suite.get("group_count") or 0)
        scenario_ids = list(suite.get("scenario_ids") or [])
        applicable = bool(suite.get("applicable"))
        notes: List[str] = []

        if not applicable:
            out.append({
                "suite": sid,
                "name": str(suite.get("name") or ""),
                "scenario_id": "",
                "group_count": group_count,
                "measured_count": "",
                "p50_s": "",
                "p95_s": "",
                "max_s": "",
                "success_rate": "",
                "peak_rss_mb": "",
                "mode": MODE,
                "source_report": source_report,
                "git_rev": git_rev,
                "notes": "not_applicable: %s"
                         % (suite.get("not_applicable_reason") or "（缺理由）"),
            })
            continue

        rows: List[Dict[str, Any]] = []
        for scenario_id in scenario_ids:
            rows.extend(by_id.get(scenario_id, []))
        walls = [float(row["e2e_wall_s"]) for row in rows
                 if row.get("e2e_wall_s") is not None]
        rss = [float(row["worker_peak_rss_mb"]) for row in rows
               if row.get("worker_peak_rss_mb") is not None]
        matched = [row for row in rows if row.get("status_match")]
        success_rate = (len(matched) / len(rows)) if rows else None

        measured = len(rows)
        if measured < group_count:
            notes.append("黄金场景集 %d 组，%s 要求 %d 组 → 未达组数要求"
                         % (measured, sid, group_count))
        if not rows:
            notes.append("S08 报告里没有 %s 的实测行（黄金场景 id 未出现），"
                         "本行不产出任何数字" % sid)
        if suite.get("caveat"):
            notes.append(str(suite.get("caveat")))
        if profile != PROFILE and picked:
            notes.append("注意：S08 报告里没有 %s 档行，本表取用了其它档的行"
                         % PROFILE)

        out.append({
            "suite": sid,
            "name": str(suite.get("name") or ""),
            "scenario_id": ",".join(scenario_ids),
            "group_count": group_count,
            "measured_count": measured,
            "p50_s": (round(_percentile(walls, 0.50), 3) if walls else ""),
            "p95_s": (round(_percentile(walls, 0.95), 3) if walls else ""),
            "max_s": (round(max(walls), 3) if walls else ""),
            "success_rate": (round(success_rate, 4)
                             if success_rate is not None else ""),
            "peak_rss_mb": (round(max(rss), 2) if rss else ""),
            "mode": MODE,
            "source_report": source_report,
            "git_rev": git_rev,
            "notes": "；".join(notes),
        })
    return out


# ---------------------------------------------------------------------------
# 3. 落盘（CSV + Markdown）
# ---------------------------------------------------------------------------
def write_csv(rows: Sequence[Dict[str, Any]], path: str) -> str:
    """写基线表 CSV（UTF-8、列序固定 = :data:`CSV_COLUMNS`）。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(CSV_COLUMNS))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in CSV_COLUMNS})
    return path


def write_markdown(rows: Sequence[Dict[str, Any]], path: str,
                   meta: Optional[Dict[str, Any]] = None) -> str:
    """写人读版基线表 Markdown（表格 + 逐行诚实标注）。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    meta = dict(meta or {})
    lines: List[str] = []
    lines.append("# S09 性能基线表（设计 §6.9 场景集 S1–S5）")
    lines.append("")
    lines.append("- 数据来源：`%s`（S08 性能脚本产物，**不重测、不另造基准**）"
                 % meta.get("source_report", ""))
    lines.append("- git 版本：`%s`" % (meta.get("git_rev") or "（报告未记录）"))
    lines.append("- 生成时间：%s" % meta.get("generated_at", ""))
    lines.append("- mode：`%s`（离线实测：真实规划内核 + 离线工位，**非真机**）"
                 % MODE)
    lines.append("- 场景声明：`%s`（唯一来源）"
                 % meta.get("scenarios_rel", SCENARIOS_REL))
    lines.append("")
    lines.append("## 基线表")
    lines.append("")
    lines.append("| " + " | ".join(CSV_COLUMNS) + " |")
    lines.append("|" + "|".join(["---"] * len(CSV_COLUMNS)) + "|")
    for row in rows:
        lines.append("| " + " | ".join(
            str(row.get(key, "")) for key in CSV_COLUMNS) + " |")
    lines.append("")
    lines.append("## 诚实标注")
    lines.append("")
    for row in rows:
        note = str(row.get("notes") or "")
        if note:
            lines.append("- **%s**：%s" % (row.get("suite"), note))
    lines.append("")
    lines.append("## 口径说明")
    lines.append("")
    lines.append("- `success_rate` 与 S07 黄金场景集**同口径**：实测 status 与"
                 "期望一致的占比。")
    lines.append("- `p50_s` / `p95_s` / `max_s` 是**端到端墙钟**（含 worker "
                 "冷启动），不是纯规划耗时。")
    lines.append("- `peak_rss_mb` 是**规划 worker 子进程**峰值 RSS（服务端进程 "
                 "RSS 不代表规划内存）。")
    lines.append("- S4 / S5 为 `not_applicable`（本里程碑单臂，双臂场景需双臂"
                 "工位），**不生成任何假数字**。")
    lines.append("- 组数未达设计 §6.9 要求时，本表**如实标注**，不假装达标。")
    lines.append("")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    return path


# ---------------------------------------------------------------------------
# 4. 段级入口（供一键回归编排器调用）
# ---------------------------------------------------------------------------
def _read_report(path: str) -> Dict[str, Any]:
    """读 S08 报告 JSON（**抛异常**由调用方归一化成 fail/blocked）。"""
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if not isinstance(data, dict):
        raise ValueError("S08 报告顶层不是 JSON 对象: %r" % (type(data).__name__,))
    return data


def run_perf_baseline(repo_root: Optional[str] = None,
                      ws: Optional[str] = None,
                      perf_report: Optional[str] = None,
                      reports_dir: Optional[str] = None) -> Dict[str, Any]:
    """跑「性能基线表」段 → 段级结果字典。

    * S08 报告缺失 → ``blocked``（外部产物未生成），``next_action`` 指名
      ``python3 scripts/rm_planner_perf.py``；
    * S08 报告结构变化（缺 ``summary`` / ``comparison`` / ``results``）→
      ``fail``（契约回归），reason **指名缺哪个键**；
    * 场景声明缺失 / 非法 → ``fail``（仓库声明被破坏）。

    :param perf_report: S08 报告路径（默认 ``<ws>/build/reports/
        s08_planner_perf.json``）。
    :param reports_dir: 本表两个产物的落盘目录（默认 ``<ws>/build/reports``）。
    """
    repo_root = os.path.abspath(repo_root or default_repo_root())
    ws = os.path.abspath(ws or repo_root)
    reports_dir = os.path.abspath(
        reports_dir or os.path.join(ws, "build", "reports"))
    perf_report_path = os.path.abspath(
        perf_report or os.path.join(ws, DEFAULT_PERF_REPORT))
    #: 是否用了版本库留存副本（干净 checkout 兜底）
    used_fallback = False
    if not os.path.isfile(perf_report_path) and perf_report is None:
        fallback_path = os.path.join(repo_root, FALLBACK_PERF_REPORT_REL)
        if os.path.isfile(fallback_path):
            perf_report_path = os.path.abspath(fallback_path)
            used_fallback = True

    # -- (1) 场景声明：仓库文件缺失/非法 → fail（契约回归）---------------
    try:
        suites = load_suites(repo_root)
    except FileNotFoundError:
        return seg_mod.segment_result(
            "fail",
            reason="场景声明文件缺失: %s" % scenarios_path(repo_root),
            next_action=("确认 workcell/bench/perf_scenarios.yaml 已随仓库提交"
                         "（S1–S5 的唯一声明处）后重跑"),
            evidence=["缺失路径: %s" % scenarios_path(repo_root)],
            mode=MODE,
            report_path="",
            id="perf_baseline",
        )
    except Exception as exc:  # noqa: BLE001 - 声明非法 → fail（指名原因）
        return seg_mod.segment_result(
            "fail",
            reason="场景声明非法（%s）: %s" % (type(exc).__name__, exc),
            next_action=("修正 workcell/bench/perf_scenarios.yaml 后重跑"
                         "（applicable=true 必须给出 scenario_ids；"
                         "applicable=false 必须给出 not_applicable_reason）"),
            evidence=[repr(exc)],
            mode=MODE,
            report_path="",
            id="perf_baseline",
        )

    # -- (2) S08 报告缺失 → blocked（外部产物未生成）---------------------
    if not os.path.isfile(perf_report_path):
        return seg_mod.segment_result(
            "blocked",
            reason="S08 性能报告缺失（%s）：本表只投影既有报告，不重测"
                   % perf_report_path,
            next_action="执行 `python3 scripts/rm_planner_perf.py` 生成性能"
                        "报告后重跑一键回归",
            evidence=["缺失路径: %s" % perf_report_path,
                      "场景声明套件数: %d" % len(suites)],
            mode=MODE,
            report_path="",
            id="perf_baseline",
        )

    try:
        report = _read_report(perf_report_path)
    except Exception as exc:  # noqa: BLE001 - 报告不可解析 → fail
        return seg_mod.segment_result(
            "fail",
            reason="S08 性能报告无法解析（%s）: %s"
                   % (type(exc).__name__, exc),
            next_action=("删除并重跑 `python3 scripts/rm_planner_perf.py` "
                         "重新生成报告后重跑一键回归"),
            evidence=[repr(exc)],
            mode=MODE,
            report_path="",
            id="perf_baseline",
        )

    missing = [key for key in REQUIRED_REPORT_KEYS if key not in report]
    if missing:
        return seg_mod.segment_result(
            "fail",
            reason="S08 性能报告结构变化，缺键: %s（契约回归）"
                   % ", ".join(missing),
            next_action=("对齐 scripts/rm_planner_perf.py 的报告 schema"
                         "（必须含 %s）后重跑性能脚本与一键回归"
                         % ", ".join(REQUIRED_REPORT_KEYS)),
            evidence=["缺失键: %s" % ", ".join(missing),
                      "报告路径: %s" % perf_report_path],
            mode=MODE,
            report_path="",
            id="perf_baseline",
        )

    # -- (3) 投影 + 落盘 ------------------------------------------------
    source_report = os.path.relpath(perf_report_path, ws)
    rows = build_rows(suites, report, source_report=source_report)
    csv_path = os.path.join(reports_dir, CSV_NAME)
    md_path = os.path.join(reports_dir, MD_NAME)
    try:
        write_csv(rows, csv_path)
        write_markdown(rows, md_path, meta={
            "source_report": source_report,
            "git_rev": str(report.get("git_rev") or ""),
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "scenarios_rel": SCENARIOS_REL,
        })
    except OSError as exc:
        return seg_mod.segment_result(
            "fail",
            reason="写基线表失败（%s）: %s" % (type(exc).__name__, exc),
            next_action="修正报告目录权限后重跑一键回归",
            evidence=[repr(exc)],
            mode=MODE,
            report_path="",
            id="perf_baseline",
        )

    measured = sum(1 for row in rows if row.get("measured_count") != "")
    gap = [row for row in rows
           if row.get("measured_count") != ""
           and isinstance(row.get("measured_count"), int)
           and int(row["measured_count"]) < int(row.get("group_count") or 0)]
    na = [row for row in rows if str(row.get("notes") or "").startswith(
        "not_applicable")]
    evidence = ["套件数: %d（实测套件 %d、not_applicable %d）"
                % (len(rows), measured, len(na)),
                "CSV: %s" % csv_path,
                "Markdown: %s" % md_path,
                "数据来源: %s" % source_report]
    for row in gap:
        evidence.append("组数未达要求 %s: 实测 %s / 要求 %s"
                        % (row.get("suite"), row.get("measured_count"),
                           row.get("group_count")))

    reason = ("S1–S5 共 %d 个套件已投影成基线表（实测 %d、not_applicable %d）"
              % (len(rows), measured, len(na)))
    if gap:
        reason += "；其中 %d 个套件未达设计 §6.9 的组数要求（详见 notes）" \
            % len(gap)

    return seg_mod.segment_result(
        "pass",
        reason=reason,
        next_action="",
        evidence=evidence,
        mode=MODE,
        report_path=csv_path,
        id="perf_baseline",
        csv_path=csv_path,
        md_path=md_path,
        rows=rows,
    )
