"""段级判定模型（一键回归的语义骨架，**零 I/O、零硬件、零厂商依赖**）。

本模块只做两件事：

1. 定义**段级 verdict 词表**（:data:`VERDICTS`）与 verdict → status /
   verdict → 退出码的映射；
2. 提供**统一报告骨架**的构造器（:func:`segment_result` / :func:`aggregate`）。

两条硬纪律（与 S06/S07 已固化的执行层契约一致）：

* **禁止第二套 status 词表**：status 常量与退出码表**只从**
  ``rm_executor.contract`` 取（``STATUS_OK`` / ``STATUS_BLOCKED`` /
  ``STATUS_ERROR`` / ``EXIT_CODES``），本模块不复制任何 status 字符串；
  段结果的构造**必须**经 ``rm_executor.contract.result_dict``。
* **禁止静默**：未知 verdict → 抛 ``ValueError``（绝不静默当 pass）；
  失败状态缺 ``next_action`` → 由 contract 补一条可粘贴的兜底动作。

verdict 语义（三态严格区分，不可混用）：

* ``blocked`` —— **外部条件未满足**：无 build / 无 model、真机不可达、
  需要人工介入。**不是**代码回归。
* ``fail`` —— **代码或契约回归**：本该跑通的段没跑通 / 期望值不符。
* ``pass`` —— 段按期望跑通。
* ``skipped`` —— **本切片范围外**（例如双臂场景在单机回归里跳过）。

mode 语义（每个指标都必须标注，禁止冒充）：

* ``fake`` —— 假执行 / 桩数据；
* ``offline`` —— 离线实测（有真实计算，但无真机）；
* ``live`` —— 真机实测。

退出码口径（与 ``rm_executor.contract.EXIT_CODES`` **同口径**）：

* ``0`` —— 无 fail 段（允许存在 blocked 段，但必须逐条列出）；
* ``3`` —— 无 fail 段、但存在 blocked 段（干净机器 / 无真机窗口的正常结局）；
* ``4`` —— 存在 fail 段（真回归）。

依赖方向：本模块属于**体验层 rmctl**，读取执行层 ``rm_executor`` 的契约常量
（experience → executor，方向合法）；**不** import 硬件适配层，
**不** import 厂商规划内核（MEM010）。
"""
from __future__ import annotations

import os
import sys
from typing import Any, Dict, Iterable, List, Optional

__all__ = [
    "MODES",
    "SUMMARY_KEY",
    "SUMMARY_VERDICTS",
    "VERDICTS",
    "aggregate",
    "segment_result",
    "summary_of",
    "verdict_to_exit_code",
    "verdict_to_status",
]

#: 段级 verdict 词表（**唯一**的段级判定集合；顺序 = 报告里的展示顺序）
VERDICTS = ("pass", "blocked", "fail", "skipped")

#: 运行模式词表（每个指标都要带 mode 标注）
MODES = ("fake", "offline", "live")

#: 顶层**汇总键**（增量键：既有顶层键一个都不删改）。
#: 语义固定：顶层报告带一个**只含 fail / blocked 段**的结构化清单，
#: 让现场「5 分钟判因」不必翻 7 段明细去找哪一段失败。
SUMMARY_KEY = "problem_segments"

#: 汇总里**只收**这两种 verdict（``fail`` = 回归，``blocked`` = 外部条件
#: 未满足）；``pass`` / ``skipped`` 一律不进入汇总。
#: 注意：这里的取值**取自** :data:`VERDICTS`，不引入第二套 verdict 词表。
SUMMARY_VERDICTS = ("fail", "blocked")

#: verdict → 执行层 status 常量**名**（值本身只在 rm_executor.contract 里定义，
#: 本模块不复制任何 status 字符串，避免第二套词表）
_VERDICT_STATUS_CONST = {
    "pass": "STATUS_OK",
    "blocked": "STATUS_BLOCKED",
    "fail": "STATUS_ERROR",
    "skipped": "STATUS_OK",
}

#: verdict → 退出码（**由 contract.EXIT_CODES 派生**，此处不写死数字）
_VERDICT_EXIT_SOURCE = {
    "pass": "STATUS_OK",
    "blocked": "STATUS_BLOCKED",
    "fail": "STATUS_ERROR",
    "skipped": "STATUS_OK",
}

_CONTRACT_CACHE: Dict[str, Any] = {}


# --------------------------------------------------------------- 契约装载
def _bootstrap_executor_path() -> None:
    """把执行层包根加入 ``sys.path``（兜底：仅在直接 import 失败时执行）。

    与 ``rmctl.process.clients`` 的惰性导入 + 兜底路径同一套做法，
    不在这里另写一份路径常量。
    """
    try:
        from rmctl import paths as _paths
    except Exception:  # noqa: BLE001 - rmctl 自身不可见时直接放弃兜底
        return
    try:
        root = _paths.package_src()
    except Exception:  # noqa: BLE001
        return
    pkg_parent = os.path.join(root, "rm_executor")
    if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
        sys.path.insert(0, pkg_parent)


def contract():
    """取执行层契约模块（``rm_executor.contract``）。

    取不到时抛 ``RuntimeError``（带可粘贴的下一步），**绝不**退化成本地
    复制的 status 常量——那会立刻变成第二套词表。
    """
    if "contract" in _CONTRACT_CACHE:
        return _CONTRACT_CACHE["contract"]
    try:
        from rm_executor import contract as _contract
    except Exception as exc:  # noqa: BLE001
        first = repr(exc)
        _bootstrap_executor_path()
        try:
            from rm_executor import contract as _contract  # type: ignore[no-redef]
        except Exception as exc2:  # noqa: BLE001
            raise RuntimeError(
                "无法导入 rm_executor.contract（%s / 兜底路径后 %s）：段级判定模型"
                "必须复用执行层契约常量，不得自带第二套 status 词表；"
                "请把执行层加入 PYTHONPATH 后重跑" % (first, repr(exc2))
            )
    _CONTRACT_CACHE["contract"] = _contract
    return _contract


# --------------------------------------------------------------- 映射函数
def _check_verdict(verdict: Any) -> str:
    """校验 verdict（未知值 → ValueError，绝不静默当 pass）。"""
    if not isinstance(verdict, str) or verdict not in VERDICTS:
        raise ValueError(
            "未知 verdict: %r（只允许 %s）" % (verdict, list(VERDICTS))
        )
    return verdict


def _check_mode(mode: Any) -> str:
    """校验 mode（未知值 → ValueError）。"""
    if not isinstance(mode, str) or mode not in MODES:
        raise ValueError("未知 mode: %r（只允许 %s）" % (mode, list(MODES)))
    return mode


def verdict_to_status(verdict: str) -> str:
    """verdict → 执行层 status（常量值只来自 ``rm_executor.contract``）。"""
    verdict = _check_verdict(verdict)
    const = _VERDICT_STATUS_CONST[verdict]
    return getattr(contract(), const)


def verdict_to_exit_code(verdict: str) -> int:
    """verdict → 进程退出码（**同口径** ``rm_executor.contract.EXIT_CODES``）。

    blocked → 3、fail → 4、pass / skipped → 0。
    """
    verdict = _check_verdict(verdict)
    const = _VERDICT_EXIT_SOURCE[verdict]
    status = getattr(contract(), const)
    return int(contract().EXIT_CODES[status])


# --------------------------------------------------------------- 段结果
def segment_result(verdict: str, reason: str = "", next_action: str = "",
                   evidence: Optional[Iterable[Any]] = None,
                   mode: str = "offline", report_path: str = "",
                   **extra: Any) -> Dict[str, Any]:
    """构造**一个段**的结果字典（统一报告 schema）。

    返回键固定为 ``id`` / ``verdict`` / ``reason`` / ``next_action`` /
    ``evidence`` / ``mode`` / ``report_path``，另附执行层契约给出的
    ``status`` 与调用方传入的额外键。

    * ``verdict`` 未知 → ``ValueError``；
    * ``mode`` 未知 → ``ValueError``；
    * blocked / fail 段缺 ``next_action`` → 由
      ``rm_executor.contract.result_dict`` 补一条**可粘贴**的兜底动作
      （绝不静默丢失）。
    """
    verdict = _check_verdict(verdict)
    mode = _check_mode(mode)
    status = verdict_to_status(verdict)

    seg_id = extra.pop("id", "")
    res = contract().result_dict(status, reason, next_action, evidence, **extra)

    out: Dict[str, Any] = {
        "id": seg_id,
        "verdict": verdict,
        "status": res["status"],
        "reason": res["reason"],
        "next_action": res["next_action"],
        "evidence": list(res["evidence"]),
        "mode": mode,
        "report_path": report_path,
    }
    # 额外键（调用方自定义字段）保留在末尾
    for key, value in res.items():
        if key not in out:
            out[key] = value
    return out


# --------------------------------------------------------------- 汇总
def summary_of(segments: Optional[Iterable[Dict[str, Any]]] = None
                ) -> List[Dict[str, Any]]:
    """从段列表抽**汇总段**：**只含** ``fail`` / ``blocked`` 段。

    每条汇总项固定带 ``id`` / ``verdict`` / ``reason`` / ``next_action``：

    * ``pass`` / ``skipped`` 段**一律不进汇总**（不制造噪音）；
    * ``next_action`` 取段结果里 contract 已补好的兜底值；段结果缺失时
      用 ``rm_executor.contract.NEXT_ACTION_FALLBACK`` 兜底，**不得为空**。

    该函数是**纯函数**（零 I/O），汇总语义唯一口径。
    """
    out: List[Dict[str, Any]] = []
    for seg in list(segments or []):
        if not isinstance(seg, dict):
            continue
        verdict = seg.get("verdict")
        if verdict not in SUMMARY_VERDICTS:
            continue
        next_action = str(seg.get("next_action") or "")
        if not next_action:
            next_action = str(contract().NEXT_ACTION_FALLBACK)
        out.append({
            "id": str(seg.get("id") or ""),
            "verdict": verdict,
            "reason": str(seg.get("reason") or ""),
            "next_action": next_action,
        })
    return out


# --------------------------------------------------------------- 聚合
def aggregate(segments: Optional[Iterable[Dict[str, Any]]] = None,
              mode: str = "offline", reason: str = "",
              next_action: str = "",
              evidence: Optional[Iterable[Any]] = None) -> Dict[str, Any]:
    """聚合若干段 → **顶层报告骨架**。

    聚合规则（唯一口径）：

    * 任一段 ``fail`` → 顶层 ``status=error``、退出码 4；
    * 否则任一段 ``blocked`` → 顶层 ``status=blocked``、退出码 3；
    * 否则 → 顶层 ``status=ok``、退出码 0（``skipped`` 不影响聚合）。

    顶层键固定为 ``status`` / ``reason`` / ``next_action`` / ``evidence``
    + ``segments[]`` + ``mode``，另附 ``exit_code``（供 CLI 直接使用）。

    **增量键** :data:`SUMMARY_KEY`（``problem_segments``）：顶层带一个
    **只含 fail / blocked 段**的结构化清单（每项含 ``id`` / ``verdict`` /
    ``reason`` / ``next_action``），供现场直接定位「哪一段失败」。
    ``pass`` / ``skipped`` 段**不进汇总**。

    负例行为：

    * 段里出现未知 verdict → ``ValueError``（不静默当 pass）；
    * ``segments`` 为空 → 合法报告（``status=ok``、退出码 0）。
    """
    mode = _check_mode(mode)
    segs: List[Dict[str, Any]] = []
    for idx, seg in enumerate(list(segments or [])):
        if not isinstance(seg, dict):
            raise ValueError("第 %d 段不是字典: %r" % (idx, seg))
        if "verdict" not in seg:
            raise ValueError("第 %d 段缺 verdict 字段" % idx)
        _check_verdict(seg["verdict"])
        segs.append(seg)

    verdicts = [seg["verdict"] for seg in segs]
    if "fail" in verdicts:
        top_verdict = "fail"
    elif "blocked" in verdicts:
        top_verdict = "blocked"
    else:
        top_verdict = "pass"

    status = verdict_to_status(top_verdict)
    exit_code = verdict_to_exit_code(top_verdict)

    if not reason:
        reason = _default_reason(top_verdict, segs)
    res = contract().result_dict(status, reason, next_action, evidence,
                                 segments=segs, mode=mode, exit_code=exit_code)

    out: Dict[str, Any] = {
        "status": res["status"],
        "reason": res["reason"],
        "next_action": res["next_action"],
        "evidence": list(res["evidence"]),
        "segments": list(res["segments"]),
        "mode": res["mode"],
        "exit_code": res["exit_code"],
        "verdict": top_verdict,
        # 增量键：只含 fail / blocked 段的汇总（pass / skipped 不入）
        SUMMARY_KEY: summary_of(res["segments"]),
    }
    for key, value in res.items():
        if key not in out:
            out[key] = value
    return out


def _default_reason(verdict: str, segments: List[Dict[str, Any]]) -> str:
    """聚合层默认 reason（**不编造数字**，**指名失败段**）。

    * ``fail`` → 只列**判定为 fail 的段 id**（代码 / 契约回归）；
    * ``blocked`` → 只列**被阻塞的段 id**（外部条件未满足）；
    * ``pass`` → 只报段数（没有失败段可指名）。

    ``pass`` / ``skipped`` 段的 id **绝不**出现在失败原因里
    （否则现场会去找一段根本没失败的段）。
    """
    if not segments:
        return "没有任何回归段（空报告，退出码 0）"

    def _ids_of(kind: str) -> List[str]:
        return [str(seg.get("id") or "") for seg in segments
                if seg.get("verdict") == kind and str(seg.get("id") or "")]

    if verdict == "fail":
        failed = _ids_of("fail")
        return "有段判定为 fail（fail 段 %d 个：%s）" % (
            len(failed), ", ".join(failed) if failed else "未命名段")
    if verdict == "blocked":
        blocked = _ids_of("blocked")
        return "有段因外部条件未满足被阻塞（blocked 段 %d 个：%s）" % (
            len(blocked), ", ".join(blocked) if blocked else "未命名段")
    return "所有可跑段通过（共 %d 段）" % len(segments)
