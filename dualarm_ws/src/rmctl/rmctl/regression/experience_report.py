"""体验基准 E1–E4 的**指标构造 / 纪律校验 / 报告落盘**（S09/T5）。

与 :mod:`rmctl.regression.experience` 的分工：

* ``experience`` —— 声明表装载 + E1–E4 的**测量实现**（跑命令、读 trace）；
* 本模块 —— 指标字典构造（:func:`build_metric`）、目标比较
  （:func:`target_met`）、**纪律校验**（:func:`validate_metrics`）与报告
  落盘（:func:`write_json` / :func:`write_markdown`）。

纪律（本模块是它的**唯一**实现处，experience 只是调用方）：

1. 每个指标都带 ``mode``（``offline`` / ``live`` / ``human_pending``）与
   ``status``（``measured`` / ``blocked`` / ``pending``）。
2. ``human_pending`` 指标的 value **必须是 null**（禁止用 proxy / 0 顶替
   人工字段）。
3. ``blocked`` 指标的 value **必须是 null**（缺数据源不许用 0 冒充）。
4. ``mode=live`` 的指标必须显式带 ``live_backed=True``（D012 分层纪律：
   离线数字不得冒充真机结论）。
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional, Sequence

__all__ = [
    "METRIC_MODES",
    "METRIC_STATUS",
    "SCENARIOS_REL",
    "SEGMENT_MODE",
    "TARGET_OPS",
    "build_metric",
    "fmt_value",
    "target_met",
    "validate_metrics",
    "write_json",
    "write_markdown",
]

#: 段级 mode（本段全部是离线口径）
SEGMENT_MODE = "offline"

#: 场景声明文件（**唯一**来源，git 跟踪）
SCENARIOS_REL = os.path.join("workcell", "bench", "experience_e1_e4.yaml")

#: 指标级 mode 词表（**段级** mode 词表在 rmctl.regression.segments）
METRIC_MODES = ("offline", "live", "human_pending")

#: 指标级 status 词表
METRIC_STATUS = ("measured", "blocked", "pending")

#: 目标比较算子
TARGET_OPS = ("<=", ">=", "==", "<", ">")


# ---------------------------------------------------------------------------
def build_metric(spec: Dict[str, Any], scenario_id: str, value: Any,
                 status: str = "measured",
                 evidence: Optional[Sequence[str]] = None,
                 note: str = "",
                 source: str = "") -> Dict[str, Any]:
    """按 YAML 声明构造**一个**指标字典。"""
    return {
        "id": "%s.%s" % (scenario_id, spec.get("id")),
        "scenario": scenario_id,
        "metric": str(spec.get("id")),
        "name": str(spec.get("name") or spec.get("id")),
        "unit": str(spec.get("unit") or ""),
        "value": value,
        "target": spec.get("target"),
        "target_op": spec.get("target_op"),
        "mode": str(spec.get("mode") or "offline"),
        "status": status,
        "source": source or str(spec.get("source") or ""),
        "evidence": [str(e) for e in (evidence or [])],
        "note": note or str(spec.get("note") or ""),
    }


def target_met(metric: Dict[str, Any]) -> Optional[bool]:
    """指标是否达标（不可判定 → ``None``：无目标 / 无值 / pending）。"""
    target = metric.get("target")
    op = metric.get("target_op")
    value = metric.get("value")
    if target is None or op is None or value is None:
        return None
    if metric.get("status") != "measured":
        return None
    if isinstance(target, bool) or isinstance(value, bool):
        if op == "==":
            return bool(value) == bool(target)
        return None
    try:
        lhs = float(value)
        rhs = float(target)
    except (TypeError, ValueError):
        return None
    if op == "<=":
        return lhs <= rhs
    if op == ">=":
        return lhs >= rhs
    if op == "==":
        return lhs == rhs
    if op == "<":
        return lhs < rhs
    if op == ">":
        return lhs > rhs
    return None


def validate_metrics(metrics: Sequence[Dict[str, Any]],
                     scenarios: Optional[Dict[str, Any]] = None) -> None:
    """指标集合的结构 / 纪律校验（违规 → ``ValueError``，绝不静默）。

    校验项：

    * ``mode`` 在词表内、``status`` 在词表内；
    * ``mode=human_pending`` 的指标：``status`` 必须是 ``pending`` 且
      **value 必须是 null**（禁止用 proxy / 0 顶替人工字段）；
    * ``status=pending`` 的指标：value 必须是 null；
    * ``mode=live`` 的指标必须显式带 ``live_backed=True``（禁止离线数字冒充
      真机，D012 分层纪律）；
    * 声明里的每个场景、每个指标都必须在结果里出现（缺项 = 漏测）。
    """
    seen: Dict[str, Dict[str, Any]] = {}
    for index, metric in enumerate(metrics):
        if not isinstance(metric, dict):
            raise ValueError("第 %d 个指标不是字典: %r" % (index, type(metric).__name__))
        mode = metric.get("mode")
        if mode not in METRIC_MODES:
            raise ValueError("指标 %r 的 mode=%r 非法（只允许 %s）"
                             % (metric.get("id"), mode, list(METRIC_MODES)))
        status = metric.get("status")
        if status not in METRIC_STATUS:
            raise ValueError("指标 %r 的 status=%r 非法（只允许 %s）"
                             % (metric.get("id"), status, list(METRIC_STATUS)))
        if mode == "human_pending":
            if status != "pending":
                raise ValueError("人工 pending 指标 %r 的 status 必须是 pending"
                                 % metric.get("id"))
            if metric.get("value") is not None:
                raise ValueError(
                    "人工 pending 指标 %r 不允许有值（实际 %r）：人工字段只能由"
                    "人回填，禁止用 proxy / 0 顶替"
                    % (metric.get("id"), metric.get("value")))
        if status == "pending" and metric.get("value") is not None:
            raise ValueError("pending 指标 %r 不允许有值（实际 %r）"
                             % (metric.get("id"), metric.get("value")))
        if status == "measured" and metric.get("value") is None:
            raise ValueError("measured 指标 %r 必须有值（null 只能配 blocked / "
                             "pending）" % metric.get("id"))
        if status == "blocked" and metric.get("value") is not None:
            raise ValueError("blocked 指标 %r 不允许有值（实际 %r）：缺数据源"
                             "时必须留 null，不得用 0 冒充"
                             % (metric.get("id"), metric.get("value")))
        if mode == "live" and not metric.get("live_backed"):
            raise ValueError("live 指标 %r 必须显式标注 live_backed=True"
                             "（禁止用离线数字冒充真机结论）"
                             % metric.get("id"))
        seen[str(metric.get("id"))] = metric

    if scenarios:
        for sid, scenario in scenarios["scenarios"].items():
            for mid in scenario.get("metric_order") or []:
                key = "%s.%s" % (sid, mid)
                if key not in seen:
                    raise ValueError("声明里的指标 %s 没有出现在测量结果里"
                                     "（漏测）" % key)


def _metric_map(metrics: Sequence[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {str(m.get("id")): m for m in metrics}


# ---------------------------------------------------------------------------
# 落盘（JSON + 人读 Markdown）
def fmt_value(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, float):
        return ("%.6f" % value).rstrip("0").rstrip(".")
    return str(value)


def write_json(payload: Dict[str, Any], path: str) -> str:
    """写体验基准 JSON（UTF-8、缩进 2）。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def write_markdown(payload: Dict[str, Any], path: str) -> str:
    """写人读版体验基准 Markdown（**每行都带 mode 标注**）。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    lines: List[str] = []
    lines.append("# S09 体验基准 E1–E4（设计 §6.9 场景集 / §6.2 目标值）")
    lines.append("")
    lines.append("- 场景声明：`%s`（**唯一**来源）"
                 % payload.get("scenarios_rel", SCENARIOS_REL))
    lines.append("- 生成时间：%s" % payload.get("generated_at", ""))
    lines.append("- 段 mode：`%s`（离线实测，**非真机结论**）"
                 % payload.get("mode", SEGMENT_MODE))
    lines.append("- 设计出处：%s" % "、".join(payload.get("design_refs") or []))
    lines.append("")
    for scenario in payload.get("scenarios") or []:
        lines.append("## %s %s（测什么：%s）"
                     % (scenario.get("id"), scenario.get("name"),
                        scenario.get("measures") or "—"))
        lines.append("")
        lines.append("| 指标 | 值 | 单位 | 目标 | 达标 | mode | 状态 | 说明 |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for metric in scenario.get("metrics") or []:
            met = target_met(metric)
            mark = {True: "✅", False: "❌", None: "—"}[met]
            target = "—"
            if metric.get("target") is not None:
                target = "%s %s" % (metric.get("target_op"),
                                    fmt_value(metric.get("target")))
            lines.append("| %s | %s | %s | %s | %s | `%s` | %s | %s |"
                         % (metric.get("name"),
                            fmt_value(metric.get("value")),
                            metric.get("unit") or "",
                            target, mark, metric.get("mode"),
                            metric.get("status"),
                            (metric.get("note") or "").replace("|", "／")))
        lines.append("")
    lines.append("## 口径与诚实声明")
    lines.append("")
    lines.append("- 全部指标带 `mode`：`offline` = 离线实测；"
                 "`human_pending` = **人工**才能测（值必须为 null + pending）。")
    lines.append("- 本表**没有任何** `mode=live` 的数字：真机口径本切片未实证，"
                 "绝不冒充。")
    lines.append("- `mttr_human_s`（真 MTTR）必须是 null + pending，"
                 "**禁止**用 `mttr_proxy_s` 顶替。")
    lines.append("- 数据源缺失的指标一律 null + `blocked`，"
                 "**不产出 0 冒充真实值**。")
    lines.append("- 设计没给数值目标的指标，目标列写「—」并注明出处，"
                 "**不编造目标值**。")
    lines.append("")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    return path
