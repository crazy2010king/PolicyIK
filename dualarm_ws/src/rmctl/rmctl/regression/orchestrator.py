"""一键回归编排器（S09/T2）：把 S05/S06/S07 三条**既有证据链**收编成段级结果。

本模块**不重写**任何证据链，只做三件事：

1. **子进程调用**既有脚本（各自 ``--out`` 指向 ``build/reports/`` 下的独立
   文件，互不覆盖），并把脚本的 ``--ws`` 指向调用方指定的工作区；
2. **归一化**脚本产出的报告 JSON + 进程退出码 → 段级结果
   （用 :mod:`rmctl.regression.segments` 的 ``segment_result``，
   status / 退出码口径**唯一来源** = ``rm_executor.contract``）；
3. **聚合**成顶层报告（``status`` / ``reason`` / ``next_action`` /
   ``evidence`` + ``segments[]``），退出码 0 / 3 / 4。

归一化规则（**唯一口径**，写在 docstring 里以免散落在代码里）
--------------------------------------------------------------

:func:`normalize_report` 把 ``(exit_code, report)`` 映射成段级判定：

===========================  ==========================================
输入                         段级 verdict
===========================  ==========================================
报告缺失 / 不是 JSON 对象     ``fail``（编排器契约问题，不是外部条件）
``status == ok`` 且退出码 0   ``pass``
``status == ok`` 但退出码 ≠ 0 ``fail``（退出码与报告自相矛盾 = 契约回归）
``status == blocked``         ``blocked``（外部条件未满足）
``status == error`` 且
``failures`` 含 ``model``     ``blocked``（**降级陷阱**（a)(b)）
``status == error`` 其它      ``fail``（段内断言不过 / 代码或契约回归）
``status`` 未知               ``fail``（契约回归，绝不静默当 pass）
===========================  ==========================================

**降级陷阱（必须在此处理，不得在下游重复判断）**

* (a) ``scripts/rm_planner_acceptance.py`` 在**模型不可用**时退出码 4、
  报告 ``status="error"`` 且 ``failures == ["model"]`` → 归一化为
  ``verdict=blocked``（**不得**当成 fail：这是环境缺 ``build/model``，
  不是代码回归）。
* (b) ``scripts/pick_place_offline_evidence.py`` 的 ``ensure_model()``
  失败路径同理（``failures == ["model"]``）→ ``verdict=blocked``。

其余 ``failures``（例如 ``spec`` / ``thresholds`` / 段落名）**一律**
``fail``：它们表示「本该跑通却没跑通」，属于代码或契约回归。

**缺产物 ≠ 回归**：编排器在跑脚本**之前**做**依赖预检**
（:attr:`SegmentSpec.requires`）。缺 ``build/model``、缺工艺规格、缺规划
配置等**外部产物** → 该段直接 ``blocked``（带可粘贴的下一步），**不**跑
脚本、不产出任何数字。段内断言不过才是 ``fail``。

**失效模式（Q5）**

* 子进程**超时 / 挂死** → 该段 ``fail``（**不是** blocked——脚本本该在
  预算内跑完），reason 指名**脚本名 + 超时秒数**；
* 子进程**被信号杀死**（退出码 < 0）→ 该段 ``fail``，reason 指名**退出信号**；
* 子进程**无法启动**（脚本文件缺失 / OSError）→ 该段 ``fail``（编排器契约
  问题），reason 指名脚本路径；
* 报告**缺失 / 无法解析** → 该段 ``fail``（同上，编排器契约问题）。

段级 ``mode`` 标注：既有三条证据链的报告自身标注 ``mode=fake``
（真实规划 + 假运动 / 假夹爪），因此段级 ``mode`` 同样取 ``fake``，
**不得**用离线数字冒充真机结论。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from rmctl.regression import segments as seg_mod

__all__ = [
    "SEGMENTS",
    "SPECS",
    "ExecResult",
    "SegmentContext",
    "SegmentSpec",
    "build_context",
    "normalize_report",
    "register_segment",
    "run_regression",
    "run_segment",
    "segment_ids",
]

#: 子进程 stdout / stderr 保留的尾部长度（字节）——只留证据，不刷屏
_TAIL_LIMIT = 2000


# ---------------------------------------------------------------------------
# 1. 段级规格
# ---------------------------------------------------------------------------
@dataclass
class SegmentSpec:
    """一段回归的**声明式规格**（脚本 + 依赖 + 预算 + mode）。

    :param seg_id: 段 id（CLI ``--only`` 的取值集合）。
    :param script: 相对仓库根的脚本路径。
    :param report_name: 该段报告文件名（写在 ``<ws>/build/reports/`` 下，
        各段互不覆盖）。
    :param mode: 段级 mode 标注（``fake`` / ``offline`` / ``live``）。
    :param requires: 运行前的**外部产物预检**（``(标签, 相对 ws 的路径)``）；
        缺任一 → 该段 ``blocked``（不跑脚本、不产出数字）。
    :param timeout_s: 子进程超时上限（秒）；超时 → 该段 ``fail``。
    :param extra_args: 追加给脚本的固定参数。
    """

    seg_id: str
    script: str
    report_name: str
    mode: str = "fake"
    requires: Tuple[Tuple[str, str], ...] = ()
    timeout_s: float = 900.0
    extra_args: Tuple[str, ...] = ()


@dataclass
class SegmentContext:
    """跑一段回归所需的上下文（仓库根 / 工作区 / 报告目录 / 子进程环境）。"""

    repo_root: str
    ws: str
    reports_dir: str
    python: str = field(default_factory=lambda: sys.executable)
    env: Dict[str, str] = field(default_factory=lambda: dict(os.environ))


@dataclass
class ExecResult:
    """一次子进程调用的结果（**可测**：测试可构造它来模拟超时 / 信号 / 回归）。"""

    exit_code: Optional[int] = None
    report: Optional[Dict[str, Any]] = None
    report_error: str = ""
    timed_out: bool = False
    timeout_s: float = 0.0
    signal_num: Optional[int] = None
    spawn_error: str = ""
    stdout_tail: str = ""
    stderr_tail: str = ""


# ---------------------------------------------------------------------------
# 2. 段注册表
# ---------------------------------------------------------------------------
#: 段 id → runner（``SegmentContext`` → 段级结果字典）
SEGMENTS: Dict[str, Callable[[SegmentContext], Dict[str, Any]]] = {}

#: 段 id → 规格
SPECS: Dict[str, SegmentSpec] = {}


def register_segment(spec: SegmentSpec) -> None:
    """注册一段（后续任务各自追加注册行即可扩展段集合）。"""
    SPECS[spec.seg_id] = spec
    SEGMENTS[spec.seg_id] = _make_script_runner(spec)


def segment_ids() -> List[str]:
    """已注册段的 id 列表（注册顺序 = 报告展示顺序）。"""
    return list(SEGMENTS.keys())


# ---------------------------------------------------------------------------
# 3. 归一化（纯函数，可测）
# ---------------------------------------------------------------------------
def normalize_report(exit_code: Optional[int], report: Any,
                     script_name: str = "") -> Dict[str, Any]:
    """把「子进程退出码 + 既有脚本报告 JSON」归一化成段级判定（**纯函数**）。

    返回 ``{"verdict", "reason", "next_action", "evidence"}``；规则见模块
    docstring 的归一化表。**不抛异常**（未知 status → ``fail``）。
    """
    contract = seg_mod.contract()
    ok_status = contract.STATUS_OK
    blocked_status = contract.STATUS_BLOCKED
    error_status = contract.STATUS_ERROR

    name = script_name or "既有证据链脚本"
    if not isinstance(report, dict):
        return {
            "verdict": "fail",
            "reason": "段报告缺失或无法解析为 JSON 对象（脚本 %s）" % name,
            "next_action": ("确认脚本 %s 的 --out 路径可写、且脚本确实产出了"
                            "报告 JSON 后重跑" % name),
            "evidence": [],
        }

    status = report.get("status")
    raw_failures = report.get("failures")
    failures: List[str] = [str(f) for f in raw_failures] \
        if isinstance(raw_failures, (list, tuple)) else []
    raw_evidence = report.get("evidence")
    evidence: List[str] = [e if isinstance(e, str) else str(e)
                           for e in (raw_evidence or [])]
    reason = str(report.get("reason") or "")
    next_action = str(report.get("next_action") or "")

    if status == ok_status:
        if exit_code not in (0, None):
            return {
                "verdict": "fail",
                "reason": "脚本 %s 退出码 %s 与报告 status=ok 自相矛盾"
                          % (name, exit_code),
                "next_action": ("查看脚本 %s 的 stdout/stderr，修正报告写入与"
                                "退出码的一致性后重跑" % name),
                "evidence": evidence,
            }
        return {
            "verdict": "pass",
            "reason": reason or "脚本 %s 报告 status=ok" % name,
            "next_action": next_action,
            "evidence": evidence,
        }

    if status == blocked_status:
        return {
            "verdict": "blocked",
            "reason": reason or "脚本 %s 报告 status=blocked" % name,
            "next_action": next_action,
            "evidence": evidence,
        }

    if status == error_status:
        # 降级陷阱 (a)(b)：模型不可用 = 外部条件未满足 → blocked，绝不当 fail
        if "model" in failures:
            return {
                "verdict": "blocked",
                "reason": "模型不可用（外部条件未满足，缺 build/model）: %s"
                          % (reason or "脚本 %s 报告 failures=[model]" % name),
                "next_action": next_action or (
                    "执行 `python3 -m rmctl init` 物化 build/model 后重跑"
                    "（若提示 GLIBCXX，先 `source scripts/rmenv.sh`）"),
                "evidence": evidence,
            }
        return {
            "verdict": "fail",
            "reason": reason or "脚本 %s 报告 status=error（failures=%s）"
                                % (name, ", ".join(failures) or "无"),
            "next_action": next_action,
            "evidence": evidence + [
                "失败段落: %s" % ", ".join(failures)] if failures else evidence,
        }

    return {
        "verdict": "fail",
        "reason": "脚本 %s 报告 status=%r 非法（契约回归）" % (name, status),
        "next_action": ("对齐脚本 %s 的报告 schema（status 只能是执行层契约的"
                        "ok / blocked / error）后重跑" % name),
        "evidence": evidence,
    }


# ---------------------------------------------------------------------------
# 4. 子进程调用
# ---------------------------------------------------------------------------
def _tail(text: str) -> str:
    text = text or ""
    if len(text) <= _TAIL_LIMIT:
        return text
    return "...(截断)...\n" + text[-_TAIL_LIMIT:]


def _execute(spec: SegmentSpec, ctx: SegmentContext,
             out_path: str) -> ExecResult:
    """跑一次既有脚本，返回 :class:`ExecResult`（**可测**：测试可 monkeypatch）。"""
    script_abs = os.path.join(ctx.repo_root, spec.script)
    cmd = [ctx.python, script_abs]
    cmd.extend(list(spec.extra_args))
    cmd.extend(["--out", out_path, "--ws", ctx.ws])

    try:
        proc = subprocess.run(
            cmd,
            cwd=ctx.repo_root,
            env=ctx.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=spec.timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        return ExecResult(
            exit_code=None,
            timed_out=True,
            timeout_s=spec.timeout_s,
            stdout_tail=_tail(exc.stdout if isinstance(exc.stdout, str) else ""),
            stderr_tail=_tail(exc.stderr if isinstance(exc.stderr, str) else ""),
        )
    except OSError as exc:
        return ExecResult(exit_code=None, spawn_error="%s: %s"
                          % (type(exc).__name__, exc))

    result = ExecResult(
        exit_code=proc.returncode,
        stdout_tail=_tail(proc.stdout or ""),
        stderr_tail=_tail(proc.stderr or ""),
    )
    if proc.returncode is not None and proc.returncode < 0:
        result.signal_num = -proc.returncode
    if os.path.isfile(out_path):
        try:
            with open(out_path, "r", encoding="utf-8") as fh:
                loaded = json.load(fh)
            result.report = loaded if isinstance(loaded, dict) else None
            if result.report is None:
                result.report_error = "报告 JSON 顶层不是对象"
        except (OSError, ValueError) as exc:
            result.report_error = "%s: %s" % (type(exc).__name__, exc)
    else:
        result.report_error = "脚本未产出报告文件: %s" % out_path
    return result


# ---------------------------------------------------------------------------
# 5. 段执行
# ---------------------------------------------------------------------------
def _preflight(spec: SegmentSpec, ctx: SegmentContext) -> List[str]:
    """外部产物预检：返回**缺失项**列表（空 = 齐备）。

    缺外部产物 → 该段 ``blocked``（不是 fail）：环境没准备好，不是回归。
    """
    missing: List[str] = []
    for label, rel in spec.requires:
        if not os.path.exists(os.path.join(ctx.ws, rel)):
            missing.append("%s（%s）" % (label, rel))
    return missing


def _make_script_runner(spec: SegmentSpec) -> Callable[[SegmentContext],
                                                        Dict[str, Any]]:
    """为一段生成 runner（子进程调用 + 归一化）。"""

    def _run(ctx: SegmentContext) -> Dict[str, Any]:
        # (1) 依赖预检：缺外部产物 → blocked，不跑脚本
        missing = _preflight(spec, ctx)
        if missing:
            return seg_mod.segment_result(
                "blocked",
                reason="外部依赖未就绪，段未运行: %s" % "；".join(missing),
                next_action=("先补齐上述外部产物后重跑一键回归"
                             "（模型用 `python3 -m rmctl init` 物化）"),
                evidence=["段 %s 预检缺: %s" % (spec.seg_id, m)
                          for m in missing],
                mode=spec.mode,
                report_path="",
                id=spec.seg_id,
                script=spec.script,
            )

        # (2) 子进程调用
        out_path = os.path.join(ctx.reports_dir, spec.report_name)
        try:
            os.makedirs(ctx.reports_dir, exist_ok=True)
        except OSError as exc:
            return seg_mod.segment_result(
                "fail",
                reason="无法创建报告目录 %s: %s" % (ctx.reports_dir, exc),
                next_action="修正报告目录权限后重跑",
                evidence=[repr(exc)],
                mode=spec.mode,
                report_path=out_path,
                id=spec.seg_id,
                script=spec.script,
            )

        outcome = _execute(spec, ctx, out_path)

        # (3) 失效模式：超时 / 信号 / 启动失败 / 报告缺失
        if outcome.timed_out:
            return seg_mod.segment_result(
                "fail",
                reason="脚本 %s 超时（>%.0f s，被编排器中止）"
                       % (spec.script, outcome.timeout_s),
                next_action=("排查 %s 挂死原因（进程 / 厂商内核 / 网络）后"
                             "重跑；或缩短该段负载再跑一键回归" % spec.script),
                evidence=["脚本: %s" % spec.script,
                          "超时上限: %.0f s" % outcome.timeout_s,
                          outcome.stderr_tail[-500:] if outcome.stderr_tail else ""],
                mode=spec.mode,
                report_path=out_path,
                id=spec.seg_id,
                script=spec.script,
            )
        if outcome.spawn_error:
            return seg_mod.segment_result(
                "fail",
                reason="脚本 %s 无法启动: %s" % (spec.script, outcome.spawn_error),
                next_action="确认脚本路径 %s 存在且可执行后重跑" % spec.script,
                evidence=[outcome.spawn_error],
                mode=spec.mode,
                report_path=out_path,
                id=spec.seg_id,
                script=spec.script,
            )
        if outcome.signal_num is not None:
            return seg_mod.segment_result(
                "fail",
                reason="脚本 %s 被信号 %d 杀死（退出码 %s）"
                       % (spec.script, outcome.signal_num, outcome.exit_code),
                next_action=("查看 %s 的 stderr 判断是否被 OOM / 外部 kill；"
                             "修复后重跑" % spec.script),
                evidence=["脚本: %s" % spec.script,
                          "信号: %d" % outcome.signal_num,
                          outcome.stderr_tail[-500:] if outcome.stderr_tail else ""],
                mode=spec.mode,
                report_path=out_path,
                id=spec.seg_id,
                script=spec.script,
            )

        # (4) 报告缺失 / 无法解析 → fail（编排器契约问题，不是 blocked）
        if outcome.report is None:
            return seg_mod.segment_result(
                "fail",
                reason="脚本 %s 的报告缺失或无法解析（%s）"
                       % (spec.script, outcome.report_error or "未知原因"),
                next_action=("查看 %s 的 stdout/stderr；确认它写出了 %s 且内容"
                             "是合法 JSON 对象后重跑"
                             % (spec.script, out_path)),
                evidence=[outcome.report_error,
                          outcome.stdout_tail[-500:] if outcome.stdout_tail else "",
                          outcome.stderr_tail[-500:] if outcome.stderr_tail else ""],
                mode=spec.mode,
                report_path=out_path,
                id=spec.seg_id,
                script=spec.script,
            )

        # (5) 归一化
        norm = normalize_report(outcome.exit_code, outcome.report,
                                script_name=spec.script)
        return seg_mod.segment_result(
            norm["verdict"],
            reason=norm["reason"],
            next_action=norm["next_action"],
            evidence=norm["evidence"],
            mode=spec.mode,
            report_path=out_path,
            id=spec.seg_id,
            script=spec.script,
            exit_code=outcome.exit_code,
        )

    return _run


def run_segment(seg_id: str, ctx: SegmentContext) -> Dict[str, Any]:
    """跑一段 → 段级结果字典（``id`` / ``verdict`` / ``reason`` / ...）。"""
    runner = SEGMENTS.get(seg_id)
    if runner is None:
        raise KeyError("未注册的段 id: %r（已注册: %s）"
                       % (seg_id, ", ".join(segment_ids())))
    return runner(ctx)


# ---------------------------------------------------------------------------
# 6. 上下文与聚合
# ---------------------------------------------------------------------------
def build_context(repo_root: Optional[str] = None,
                  ws: Optional[str] = None,
                  python: Optional[str] = None) -> SegmentContext:
    """构造 :class:`SegmentContext`（默认仓库根 = 当前工作区根）。"""
    if repo_root is None:
        from rmctl import paths as _paths

        repo_root = _paths.workspace_root()
    repo_root = os.path.abspath(repo_root)
    ws = os.path.abspath(ws or repo_root)
    reports_dir = os.path.join(ws, "build", "reports")
    env = dict(os.environ)
    extra = [os.path.join(repo_root, "dualarm_ws", "src", name)
             for name in ("rmctl", "rm_executor", "rm_hw_bridge", "rm_safety")]
    extra.append(os.path.join(repo_root, "dualarm_ws", "src",
                               "rm_description", "scripts"))
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join([p for p in extra if os.path.isdir(p)]
                                        + ([existing] if existing else []))
    return SegmentContext(
        repo_root=repo_root,
        ws=ws,
        reports_dir=reports_dir,
        python=python or sys.executable,
        env=env,
    )


def run_regression(ctx: SegmentContext,
                   only: Optional[Iterable[str]] = None) -> Dict[str, Any]:
    """跑一键回归 → 顶层报告（``status`` / ``reason`` / ``next_action`` /
    ``evidence`` + ``segments[]`` + ``exit_code``）。

    ``only`` 为空 → 跑全部已注册段；否则只跑指定段（顺序按注册顺序）。
    """
    wanted = list(only) if only else segment_ids()
    unknown = [seg_id for seg_id in wanted if seg_id not in SEGMENTS]
    if unknown:
        raise KeyError("未知的段 id: %s（已注册: %s）"
                       % (", ".join(unknown), ", ".join(segment_ids())))
    ordered = [seg_id for seg_id in segment_ids() if seg_id in set(wanted)]
    segments = [run_segment(seg_id, ctx) for seg_id in ordered]
    evidence = ["%s: %s" % (seg.get("id"), seg.get("verdict"))
                for seg in segments]
    return seg_mod.aggregate(segments, mode="offline", evidence=evidence)


# ---------------------------------------------------------------------------
# 7. 已注册段（S05 / S06 / S07 三条既有证据链）
# ---------------------------------------------------------------------------
register_segment(SegmentSpec(
    seg_id="gripper_offline",
    script="scripts/gripper_offline_evidence.py",
    report_name="s09_segment_gripper_offline.json",
    mode="fake",
    requires=(),
    timeout_s=600.0,
))

register_segment(SegmentSpec(
    seg_id="process_offline",
    script="scripts/pick_place_offline_evidence.py",
    report_name="s09_segment_process_offline.json",
    mode="fake",
    requires=(
        ("模型 build/model", "build/model"),
        ("工艺规格", os.path.join("workcell", "tasks", "pick_place.yaml")),
    ),
    timeout_s=3600.0,
))

register_segment(SegmentSpec(
    seg_id="planner_golden",
    script="scripts/rm_planner_golden.py",
    report_name="s09_segment_planner_golden.json",
    mode="offline",
    requires=(),
    timeout_s=300.0,
))

register_segment(SegmentSpec(
    seg_id="planner_acceptance",
    script="scripts/rm_planner_acceptance.py",
    report_name="s09_segment_planner_acceptance.json",
    mode="fake",
    requires=(
        ("模型 build/model", "build/model"),
        ("规划配置", os.path.join("workcell", "planner", "pipeline.yaml")),
        ("规划配置", os.path.join("workcell", "planner", "profiles.yaml")),
    ),
    timeout_s=3600.0,
))
