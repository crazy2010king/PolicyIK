"""真机基线汇总表（S09/T7）：把 ``workcell/baselines/*.yaml`` **只读投影**成
一张可版本化、可 diff 的汇总表（JSON + Markdown）。

做什么 / 不做什么
------------------

* **做**：读四类真机基线 YAML（``zero_baseline`` / ``measured_limits`` /
  ``version_matrix`` / ``network_plan``）→ 投影成
  ``build/reports/s09_baselines.json`` 与 ``build/reports/s09_baselines.md``
  （逐类一行：kind / version / status / captured_at / 结论 / 阻塞原因 /
  下一步）。
* **不做**：**不写** ``workcell/**``（基线回填是人在真机窗口里做的事，不属本
  切片）；**不重算**任何结论；**不推断**任何数值。

诚实口径（硬纪律）
------------------

1. **判定结论唯一来源 = ``rmctl.baseline.validate_all(ws)``**（``cli.py``
   注释明写「基线结论的唯一来源」）。本模块**不另写一套判定逻辑**，只做
   投影；schema 违规项直接取 ``validate_all`` 的 issues。
2. **status 列逐字取自 YAML**：表**不得自带真相**。真机窗口回填后，同一脚本
   自动更新表；**绝不**把 ``blocked`` 改写成「已验证 / measured」。
3. **缺文件 / YAML 语法错误 / 缺 status 字段 → 该类 blocked**（外部条件未
   满足 / 需人工介入），reason **指名文件与行**，并给出**可粘贴**的下一步
   （``python3 -m rmctl baseline scaffold``）；**绝不抛裸异常**、绝不整体崩。
4. **schema 违规（有 error 级 issue 且不属于结构性缺失）→ 段级 fail**：那是
   仓库侧的契约破坏（例如在 ``status != measured`` 的基线里填占位数值）。
5. **禁止 import tesseract**（顶层禁止）：基线校验只依赖 ``yaml`` + ``numpy``
   （``rmctl.baseline`` 本身如此），保证没有 GLIBCXX 的机器也能跑。

措辞纪律（Q7 负例）
-------------------

投影**自身生成的文案**不得把 blocked 描述成「已验证 / measured」。为此：

* :data:`FORBIDDEN_CLAIM_PHRASES` 是**禁语**集合（中文断言措辞）；
* :data:`FORBIDDEN_STATUS_TOKENS` 是英文 status 词（``measured``）——它**不能**
  参与全文扫描（基线类型 id ``measured_limits`` 本身就含该词），只在**结论
  列**这类投影自撰文案上判定；
* YAML 原文一律以「」引用，:func:`strip_quoted` 去掉引用后，剩余的投影文案
  不得含任何禁语；**YAML 原文不被改写**（引用内容不参与措辞判定）。

依赖方向：本模块属于体验层 ``rmctl``，只读**数据文件**（YAML）+ 执行层契约
常量（经 :mod:`rmctl.regression.segments`）；**不** import 硬件适配层、
**不** import 厂商规划内核。
"""
from __future__ import annotations

import json
import os
import re
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

from rmctl import baseline as bl
from rmctl import paths
from rmctl.regression import segments as seg_mod

__all__ = [
    "FORBIDDEN_CLAIM_PHRASES",
    "FORBIDDEN_STATUS_TOKENS",
    "JSON_NAME",
    "MD_NAME",
    "MODE",
    "STRUCTURAL_ERROR_CODES",
    "build_rows",
    "collect_rows",
    "run_baselines",
    "strip_quoted",
    "write_json",
    "write_markdown",
]

#: 本表产出的两个文件（相对工作区 ``build/reports/``）
JSON_NAME = "s09_baselines.json"
MD_NAME = "s09_baselines.md"

#: 本表的 mode 标注：内容是真机基线**声明**，但本表是在离线机器上投影生成，
#: 且当前不含任何实测数值 → ``offline``（有真实计算，但无真机）。
MODE = "offline"

#: 投影自身文案里的**禁语**（把 blocked 说成「已验证 / measured」的措辞）。
#: YAML 原文以「」引用，不参与本判定（见 :func:`strip_quoted`）。
#: 只放中文断言措辞——英文 status 词见 :data:`FORBIDDEN_STATUS_TOKENS`
#: （``measured_limits`` 这个基线类型 id 本身就含 ``measured``，不能全文扫）。
FORBIDDEN_CLAIM_PHRASES = (
    "已验证",
    "验证通过",
    "已确认",
    "已实测",
    "已冻结",
    "校验通过",
)

#: 英文 status 词（只在**结论列**这类投影自撰文案上判定）
FORBIDDEN_STATUS_TOKENS = ("measured",)

#: ``validate_all`` 里属于**结构性**问题的 error code：它们对应的不是
#: 「schema 违规」，而是「文件缺失 / 不可解析 / status 字段不可用」→
#: 该类记 **blocked**（外部条件未满足），不升级成段级 fail。
STRUCTURAL_ERROR_CODES = (
    "file_missing",
    "unreadable",
    "not_mapping",
    "status_invalid",
)

#: YAML 里的 ``kind`` 字段与文件名不符 → 结构性 → blocked（不判 fail）
_KIND_MISMATCH_CODE = "kind_mismatch"

#: 缺文件时的可粘贴下一步（与 ``rmctl baseline scaffold`` 一致）
SCAFFOLD_CMD = "python3 -m rmctl baseline scaffold"

#: ``status`` → 投影自身的结论措辞（禁语安全）
_CLAIM_BY_STATUS = {
    "measured": "含实测证据（详见 YAML）",
    "pending": "未实测（待真机窗口）",
    "blocked": "未实测（外部条件未满足）",
}


# ---------------------------------------------------------------------------
# 1. 工具
# ---------------------------------------------------------------------------
def strip_quoted(text: str) -> str:
    """去掉 YAML 原文引用（``「...」``），返回**投影自身生成**的文案。

    测试与审阅用它把「YAML 原文」与「投影措辞」分开：禁语判定只看后者
    （YAML 原文不得被改写，也不得因为历史措辞而误伤）。
    """
    return re.sub(r"「[^」]*」", "", text or "", flags=re.S)


def _quote(text: str) -> str:
    """把 YAML 原文包进「」（供 Markdown / 措辞判定区分引用与自撰文案）。"""
    text = (text or "").strip()
    if not text:
        return ""
    return "「%s」" % text.replace("「", "『").replace("」", "』")


def _yaml_error_location(exc: Exception) -> str:
    """YAML 解析错误 → 「<文件> 第 N 行」片段（拿不到就只给文件）。"""
    mark = getattr(exc, "problem_mark", None)
    context = getattr(exc, "context", None) or getattr(exc, "problem", None)
    line = getattr(mark, "line", None)
    where = ""
    if isinstance(line, int):
        where = " 第 %d 行" % (line + 1)
    detail = str(context or exc).strip().splitlines()
    detail = detail[0] if detail else type(exc).__name__
    return "%s%s: %s" % (where, "", detail)


# ---------------------------------------------------------------------------
# 2. 逐类投影
# ---------------------------------------------------------------------------
def build_rows(ws: Optional[str] = None,
               report: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
    """把四类真机基线 YAML 投影成**逐类一行**的表行。

    行字段：

    * ``kind``：基线类型（四类，顺序 = ``rmctl.baseline.KINDS``）；
    * ``version`` / ``captured_at``：逐字取自 YAML（读不到 → ``None``）；
    * ``yaml_status``：YAML 里的 ``status`` **原样**（缺失 → ``None``）；
    * ``status``：**有效 status**（结构性缺失时为 ``blocked``；否则 ==
      ``yaml_status``）——表**不得自带真相**；
    * ``claim``：投影自身的结论措辞（禁语安全）；
    * ``blocked_reason``：YAML 的 ``blocked.reason``，或结构性问题的指名原因；
    * ``attempted_actions``：YAML 的 ``blocked.attempted_actions[]``；
    * ``next_action``：**可粘贴**的下一步；
    * ``present`` / ``issues`` / ``schema_errors``：存在性 + ``validate_all``
      的 issues（判定来源唯一）。

    ``report`` 缺省时自行调用 :func:`rmctl.baseline.validate_all`（唯一判定
    来源）；调用方可传入已算好的报告以免重复校验。
    """
    ws = os.path.abspath(ws or paths.workspace_root())
    if report is None:
        report = bl.validate_all(ws)
    kinds = report.get("kinds") or {}
    bdir = paths.baselines_dir(ws)
    rows: List[Dict[str, Any]] = []
    for kind in bl.KINDS:
        entry = kinds.get(kind) or {}
        path = os.path.join(bdir, "%s.yaml" % kind)
        rel = os.path.relpath(path, ws)
        issues = list(entry.get("issues") or [])
        row: Dict[str, Any] = {
            "kind": kind,
            "yaml": rel,
            "version": None,
            "captured_at": None,
            "yaml_status": None,
            "status": "blocked",
            "claim": _CLAIM_BY_STATUS["blocked"],
            "blocked_reason": "",
            "attempted_actions": [],
            "next_action": "",
            "present": bool(entry.get("present")),
            "issues": issues,
            "schema_errors": [],
        }

        # -- (1) 文件缺失 → blocked（可粘贴的下一步）---------------------
        if not os.path.isfile(path):
            row["blocked_reason"] = "基线文件缺失: %s" % path
            row["next_action"] = ("执行 `%s` 生成占位基线（status=pending），"
                                  "真机窗口内回填后把 status 改为 measured"
                                  % SCAFFOLD_CMD)
            row["schema_errors"] = []
            rows.append(row)
            continue

        # -- (2) YAML 不可解析 → blocked（指名文件与行）------------------
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh)
        except yaml.YAMLError as exc:
            row["blocked_reason"] = ("YAML 语法错误: %s%s（%s）"
                                     % (path, _yaml_error_location(exc),
                                        type(exc).__name__))
            row["next_action"] = ("修正 %s 的 YAML 语法后重跑一键回归"
                                  "（基线结论的唯一来源是 "
                                  "rmctl.baseline.validate_all）" % rel)
            row["schema_errors"] = []
            rows.append(row)
            continue
        except OSError as exc:
            row["blocked_reason"] = "读取基线文件失败: %s（%s）" % (path, exc)
            row["next_action"] = "确认 %s 可读后重跑一键回归" % rel
            row["schema_errors"] = []
            rows.append(row)
            continue

        if not isinstance(data, dict):
            row["blocked_reason"] = ("YAML 顶层不是映射（实际 %s）: %s"
                                     % (type(data).__name__, rel))
            row["next_action"] = "修正 %s 的顶层结构后重跑一键回归" % rel
            row["schema_errors"] = []
            rows.append(row)
            continue

        # -- (3) 逐字取 YAML 的字段 --------------------------------------
        raw_status = data.get("status")
        row["yaml_status"] = raw_status
        row["version"] = data.get("version")
        row["captured_at"] = data.get("captured_at")
        blocked = data.get("blocked") if isinstance(data.get("blocked"), dict) \
            else {}
        actions = blocked.get("attempted_actions")
        row["attempted_actions"] = [str(a) for a in actions] \
            if isinstance(actions, (list, tuple)) else []

        if raw_status is None:
            row["status"] = "blocked"
            row["claim"] = "未实测（YAML 缺 status 字段）"
            row["blocked_reason"] = ("YAML 缺 status 字段: %s（status 字段是"
                                     "基线结论的载体，缺失即不可用）" % rel)
            row["next_action"] = ("在 %s 补 status 字段（pending / measured / "
                                  "blocked）后重跑一键回归" % rel)
            row["schema_errors"] = []
            rows.append(row)
            continue
        if raw_status not in bl.STATUSES:
            row["status"] = "blocked"
            row["claim"] = "未实测（YAML status 字段非法）"
            row["blocked_reason"] = ("YAML status 字段非法: %r（只允许 %s）: %s"
                                     % (raw_status, "/".join(bl.STATUSES), rel))
            row["next_action"] = ("把 %s 的 status 改成 %s 之一后重跑一键回归"
                                  % (rel, "/".join(bl.STATUSES)))
            row["schema_errors"] = []
            rows.append(row)
            continue

        # -- (4) status 合法 → 逐字采用 ---------------------------------
        row["status"] = raw_status
        row["claim"] = _CLAIM_BY_STATUS.get(raw_status,
                                            _CLAIM_BY_STATUS["blocked"])
        if raw_status == "blocked":
            row["blocked_reason"] = str(blocked.get("reason") or "").strip()
            row["next_action"] = str(blocked.get("next_action") or "").strip()
        elif raw_status == "pending":
            row["next_action"] = ("取得真机窗口后回填 %s，并把 status 改为 "
                                  "measured（补齐 source/method/samples）" % rel)
        else:
            row["next_action"] = ""

        # -- (5) schema 违规（非结构性）→ 段级 fail ----------------------
        row["schema_errors"] = [
            {"code": str(i.get("code") or ""),
             "path": str(i.get("path") or ""),
             "message": str(i.get("message") or "")}
            for i in issues
            if str(i.get("level")) == "error"
            and str(i.get("code")) not in STRUCTURAL_ERROR_CODES
            and str(i.get("code")) != _KIND_MISMATCH_CODE
        ]
        if any(str(i.get("code")) == _KIND_MISMATCH_CODE for i in issues):
            row["blocked_reason"] = (
                row["blocked_reason"]
                or "YAML 的 kind 字段与文件名不一致: %s" % rel)
            row["next_action"] = row["next_action"] or (
                "把 %s 的 kind 字段改成 %s 后重跑一键回归" % (rel, kind))
            row["claim"] = "未实测（YAML kind 字段与文件名不符）"
            row["status"] = "blocked"
        rows.append(row)
    return rows


def collect_rows(ws: Optional[str] = None
                 ) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """``(rows, validate_report)`` —— 判定来源与投影结果一起返回。"""
    ws = os.path.abspath(ws or paths.workspace_root())
    report = bl.validate_all(ws)
    return build_rows(ws, report=report), report


# ---------------------------------------------------------------------------
# 3. 落盘（JSON + Markdown）
# ---------------------------------------------------------------------------
def write_json(rows: Sequence[Dict[str, Any]], path: str,
               meta: Optional[Dict[str, Any]] = None) -> str:
    """写机器可读汇总表 JSON。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    meta = dict(meta or {})
    summary = {
        "total": len(rows),
        "measured": sum(1 for r in rows if r.get("status") == "measured"),
        "pending": sum(1 for r in rows if r.get("status") == "pending"),
        "blocked": sum(1 for r in rows if r.get("status") == "blocked"),
        "missing": sum(1 for r in rows if not r.get("present")),
    }
    payload = {
        "schema_version": 1,
        "generated_at": meta.get("generated_at", ""),
        "ws": meta.get("ws", ""),
        "baselines_dir": meta.get("baselines_dir", ""),
        "source_of_truth": "workcell/baselines/*.yaml",
        "verdict_source": "rmctl.baseline.validate_all",
        "mode": MODE,
        "summary": summary,
        "rows": [dict(r) for r in rows],
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")
    return path


#: 汇总表列（计划要求的列在前，附加列在后）
MD_COLUMNS = (
    "kind",
    "version",
    "status",
    "captured_at",
    "claim",
    "blocked_reason",
    "next_action",
)


def write_markdown(rows: Sequence[Dict[str, Any]], path: str,
                   meta: Optional[Dict[str, Any]] = None) -> str:
    """写人读版汇总表 Markdown（表格 + 逐类原文引用 + 口径说明）。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    meta = dict(meta or {})
    lines: List[str] = []
    lines.append("# S09 真机基线汇总表（`workcell/baselines` 的只读投影）")
    lines.append("")
    lines.append("- 真相来源：`%s`（git 跟踪、可 diff、可评审）"
                 % meta.get("source_of_truth", "workcell/baselines/*.yaml"))
    lines.append("- 判定来源：`%s`（基线结论的**唯一**来源）"
                 % meta.get("verdict_source", "rmctl.baseline.validate_all"))
    lines.append("- 基线目录：「%s」" % meta.get("baselines_dir", ""))
    lines.append("- 生成时间：%s" % meta.get("generated_at", ""))
    lines.append("- mode：`%s`（本表在离线机器上投影生成；表内内容是真机基线"
                 "**声明**，当前不含任何实测数值）" % MODE)
    lines.append("")
    lines.append("## 汇总表")
    lines.append("")
    lines.append("| " + " | ".join(MD_COLUMNS) + " |")
    lines.append("|" + "|".join(["---"] * len(MD_COLUMNS)) + "|")
    for row in rows:
        cells = []
        for col in MD_COLUMNS:
            value = row.get(col, "")
            if col in ("blocked_reason", "next_action"):
                # YAML 原文一律加「」引用：与投影自身措辞区分（措辞判定只
                # 看后者），也避免 YAML 里的历史措辞被误判成投影在宣称什么。
                value = _quote(str(value or ""))
            cells.append(str(value if value is not None else "")
                         .replace("|", "\\|"))
        lines.append("| " + " | ".join(cells) + " |")
    lines.append("")
    lines.append("## 逐类原文引用")
    lines.append("")
    for row in rows:
        lines.append("### %s" % row.get("kind"))
        lines.append("")
        lines.append("- 源文件：「%s」" % row.get("yaml"))
        lines.append("- status：「%s」" % row.get("yaml_status"))
        if row.get("blocked_reason"):
            lines.append("- 阻塞原因：%s" % _quote(str(row["blocked_reason"])))
        actions = row.get("attempted_actions") or []
        if actions:
            lines.append("- 已尝试动作：")
            for action in actions:
                lines.append("  - %s" % _quote(str(action)))
        if row.get("next_action"):
            lines.append("- 下一步：%s" % _quote(str(row["next_action"])))
        for issue in row.get("schema_errors") or []:
            lines.append("- schema 违规：%s %s: %s"
                         % (issue.get("code"), issue.get("path"),
                            issue.get("message")))
        lines.append("")
    lines.append("## 口径说明")
    lines.append("")
    lines.append("- 本表是**只读投影**：status 列逐字取自 YAML，本表不改写、"
                 "不推断、不补齐任何字段。")
    lines.append("- status=blocked / pending 的行表示该类基线**尚无实测数值**"
                 "（真机窗口未到），表如实保留该结论。")
    lines.append("- 「结论」列是投影自身的措辞；YAML 原文一律以「」引用，"
                 "不参与措辞判定。")
    lines.append("- 缺文件 / YAML 语法错误 / 缺 status 字段 → 该类记 blocked，"
                 "并指名文件与行，同时给出可粘贴的下一步。")
    lines.append("- 基线回填是人在真机窗口做的事，本切片**不写** "
                 "`workcell/**`。")
    lines.append("")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    return path


# ---------------------------------------------------------------------------
# 4. 段级入口（供一键回归编排器调用）
# ---------------------------------------------------------------------------
def run_baselines(repo_root: Optional[str] = None,
                  ws: Optional[str] = None,
                  reports_dir: Optional[str] = None) -> Dict[str, Any]:
    """跑「真机基线汇总表」段 → 段级结果字典。

    段级判定（唯一口径，写在此处以免散落）：

    * 校验器本身异常 / 报告目录不可写 → ``fail``（编排器契约问题）；
    * 任一类基线存在 **schema 违规**（error 级 issue，且非结构性）→
      ``fail``（仓库侧契约破坏，例如在非 measured 基线里填占位数值）；
    * 四类基线**全部** status=measured → ``pass``；
    * 其余（存在 blocked / pending / 缺文件 / 不可解析）→ ``blocked``
      （外部条件未满足 / 需人工介入），并逐类列出 blocked 明细。

    无论 verdict 如何，**产物一定会落盘**（``s09_baselines.json`` +
    ``s09_baselines.md``）——表本身就是本段的交付物。
    """
    repo_root = os.path.abspath(repo_root or paths.workspace_root())
    ws = os.path.abspath(ws or repo_root)
    reports_dir = os.path.abspath(
        reports_dir or os.path.join(ws, "build", "reports"))
    json_path = os.path.join(reports_dir, JSON_NAME)
    md_path = os.path.join(reports_dir, MD_NAME)

    # -- (1) 判定来源（唯一） -------------------------------------------
    try:
        report = bl.validate_all(ws)
    except Exception as exc:  # noqa: BLE001 - 校验器异常 → fail（指名原因）
        return seg_mod.segment_result(
            "fail",
            reason="基线校验器异常（%s）: %s" % (type(exc).__name__, exc),
            next_action=("检查 workcell/baselines/*.yaml 是否可读，并确认 "
                         "rmctl.baseline 可导入后重跑一键回归"),
            evidence=[repr(exc)],
            mode=MODE,
            report_path="",
            id="baselines",
        )

    # -- (2) 投影（结构性缺失不抛异常，逐类降级为 blocked）--------------
    try:
        rows = build_rows(ws, report=report)
    except Exception as exc:  # noqa: BLE001 - 投影异常 → fail
        return seg_mod.segment_result(
            "fail",
            reason="真机基线投影失败（%s）: %s" % (type(exc).__name__, exc),
            next_action="修正 workcell/baselines/*.yaml 后重跑一键回归",
            evidence=[repr(exc)],
            mode=MODE,
            report_path="",
            id="baselines",
        )

    # -- (3) 落盘 -------------------------------------------------------
    meta = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "ws": ws,
        "baselines_dir": paths.baselines_dir(ws),
        "source_of_truth": "workcell/baselines/*.yaml",
        "verdict_source": "rmctl.baseline.validate_all",
    }
    try:
        write_json(rows, json_path, meta=meta)
        write_markdown(rows, md_path, meta=meta)
    except OSError as exc:
        return seg_mod.segment_result(
            "fail",
            reason="写真机基线汇总表失败（%s）: %s" % (type(exc).__name__, exc),
            next_action="修正报告目录权限后重跑一键回归",
            evidence=[repr(exc)],
            mode=MODE,
            report_path="",
            id="baselines",
        )

    # -- (4) 段级判定 ---------------------------------------------------
    schema_bad = [row for row in rows if row.get("schema_errors")]
    evidence: List[str] = [
        "基线目录: %s" % meta["baselines_dir"],
        "基线类别数: %d（validate_all ok=%s, error_count=%s）"
        % (len(rows), report.get("ok"), report.get("error_count")),
        "JSON: %s" % json_path,
        "Markdown: %s" % md_path,
    ]
    for row in rows:
        evidence.append("%s: status=%s version=%s"
                        % (row.get("kind"), row.get("status"),
                           row.get("version")))
        if row.get("blocked_reason"):
            evidence.append("%s 阻塞原因: %s"
                            % (row.get("kind"), row.get("blocked_reason")))

    if schema_bad:
        for row in schema_bad:
            for issue in row.get("schema_errors") or []:
                evidence.append("schema 违规 %s %s: %s"
                                % (row.get("kind"), issue.get("code"),
                                   issue.get("message")))
        return seg_mod.segment_result(
            "fail",
            reason=("基线 YAML 违反 schema（%d 类）: %s"
                    % (len(schema_bad),
                       "，".join("%s(%s)" % (row.get("kind"), ",".join(
                           str(i.get("code")) for i in row.get("schema_errors")
                       )) for row in schema_bad))),
            next_action=("修正上述基线 YAML（未实测项必须写 null，禁止占位数值）"
                         "后重跑一键回归"),
            evidence=evidence,
            mode=MODE,
            report_path=json_path,
            id="baselines",
            json_path=json_path,
            md_path=md_path,
            rows=rows,
        )

    unmeasured = [row for row in rows if row.get("status") != "measured"]
    if unmeasured:
        detail = "，".join("%s=%s" % (row.get("kind"), row.get("status"))
                           for row in unmeasured)
        next_action = next((str(row.get("next_action")) for row in unmeasured
                            if row.get("next_action")), "")
        return seg_mod.segment_result(
            "blocked",
            reason=("真机基线未全部实测（%d/%d 类）: %s；本表如实保留该结论，"
                    "不改写 status" % (len(unmeasured), len(rows), detail)),
            next_action=next_action,
            evidence=evidence,
            mode=MODE,
            report_path=json_path,
            id="baselines",
            json_path=json_path,
            md_path=md_path,
            rows=rows,
        )

    return seg_mod.segment_result(
        "pass",
        reason="四类真机基线均为 measured（%d 类），汇总表已生成" % len(rows),
        next_action="",
        evidence=evidence,
        mode=MODE,
        report_path=json_path,
        id="baselines",
        json_path=json_path,
        md_path=md_path,
        rows=rows,
    )
