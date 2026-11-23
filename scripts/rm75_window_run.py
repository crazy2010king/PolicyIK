#!/usr/bin/env python3
"""RM75-B 真机窗口**单一入口**编排器（门禁 → 长跑 → 断流 → 阶梯 → 结案工件）。

为什么需要它
------------
S10 需要的是一个**单一入口**，把「只读门禁 →（仅当门禁 ready）600 s 连续执行长跑
→ UDP 断流演练 → 阶梯刷新 → 证据入库 → 统一结案工件」串成一条流程，并把
**「门禁不过就绝不运动」**这条纪律钉死在代码与测试里。

硬纪律
------
1. **门禁不过 → 绝不运动**：只有 ``rm75_window_gate`` 给出 ``status="ok"`` 才会
   调用长跑 / 断流 / 阶梯 runner；否则三个 runner **一次都不会被调用**，
   结案工件里对应项为 ``null`` 且写明「未执行原因」。
2. **单连接纪律**：长跑与断流**顺序执行、绝不并发**（控制器客户端上限 ≈5，
   连接释放需要 5–10 min 量级）；长跑必须有人在场（R022：不做夜间/无人值守长跑）。
3. **判据唯一来源**：长跑 / 断流的判据表写在本脚本的模块级常量里
   （:data:`LONG_RUN_CRITERIA` / :data:`STREAM_LOSS_CRITERIA`），测试直接引用；
   断流停机阈值的唯一来源 = ``rm_executor.contract.STREAM_STOP_MS``（200 ms）。
4. **不伪造结论**：任何未执行 / 未达标的项都**如实**记为未达标，并写明原因；
   结案工件固定带 ``honesty`` 声明——**未以离线/仿真结果替代真机结论**。
5. **status / 退出码词表唯一来源** = ``rm_executor.contract``（ok=0 / blocked=3 /
   error=4），本脚本不自造第二套字面量。

用法
----
::

    # 正常跑一遍（门禁不过会自动走 blocked 分支并落盘结案工件）
    python3 scripts/rm75_window_run.py \
        --out workcell/baselines/evidence/rm75_s10_window_outcome.json

    # 自校验结案工件（0 = 自洽 / 3 = 不自洽）
    python3 scripts/rm75_window_run.py \
        --validate workcell/baselines/evidence/rm75_s10_window_outcome.json

离线可测：门禁与长跑 / 断流 / 阶梯 / 入库的调用都是**可注入的 runner**
（默认实现跑真脚本），测试注入假 runner 即可，全程不连真机。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

# 与 scripts/rm_regression.py 同一套自举方式：裸环境也能 import 到各层包
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

from rm_executor import contract  # noqa: E402

# --------------------------------------------------------------------------
# 词表（唯一来源 = rm_executor.contract）
# --------------------------------------------------------------------------
STATUS_OK = contract.STATUS_OK
STATUS_BLOCKED = contract.STATUS_BLOCKED
STATUS_ERROR = contract.STATUS_ERROR
EXIT_CODES = contract.EXIT_CODES

#: 结案工件 schema
SCHEMA = "rmctl.rm75_s10_window_outcome/1"

#: 结论工件里 mode 的合法取值
MODES = ("hardware", "offline", "fake", "blocked", "mixed")

DEFAULT_IP = "192.168.1.18"
DEFAULT_PORT = 8080
DEFAULT_UDP_PORTS = (8091, 8092)

#: 长跑时长（秒）——「连续执行 10 分钟」的字面要求
LONG_RUN_DURATION_S = 600.0

#: L4 急停的 SDK 符号（唯一来源 = rm_safety.levels；不可用时退化为字面量）
try:  # pragma: no cover - 依赖存在性由环境决定
    from rm_safety import levels as _safety_levels

    L4_SDK_SYMBOL = str(_safety_levels.level_sdk_symbol(_safety_levels.LEVEL_L4))
    EVENT_L4_EMERGENCY_STOP = str(_safety_levels.EVENT_L4_EMERGENCY_STOP)
except Exception:  # pragma: no cover - 无 rm_safety 时的兜底
    L4_SDK_SYMBOL = "rm_set_arm_stop"
    EVENT_L4_EMERGENCY_STOP = "safety_l4_emergency_stop"

#: 断流停机阈值（毫秒）—— **唯一来源** = rm_executor.contract.STREAM_STOP_MS
STREAM_STOP_MS = contract.STREAM_STOP_MS

#: 固定诚实声明（每个工件必带）
HONESTY = ("本工件只记录真机窗口（门禁 → 长跑 → 断流 → 阶梯）的真实执行结果；"
           "未以离线/仿真结果替代真机结论；blocked 分支下未执行任何运动指令，"
           "工件内不含任何伪造的长跑/断流实测数字。")

#: 参照值（2026-11-19 真机 501 拍 / 5.53 s）——只作对照，不是判据
REFERENCE_VALUES = {
    "date": "2026-11-19",
    "ticks": 501,
    "wall_s": 5.53,
    "tick_jitter_p99_ms": 0.201,
    "arrival_err_deg": 0.0030,
    "follow_err_rms_deg": 0.0506,
    "follow_err_max_deg": 0.112,
    "note": "2026-11-19 真机短跑参照值；当时时长未达标（5.53 s < 600 s），"
            "量化门限已达标。本次结论以实测工件为准。",
}

# --------------------------------------------------------------------------
# 判据表（**唯一来源**：写成脚本常量并被测试引用）
# --------------------------------------------------------------------------
#: 长跑判据：status ok / 墙钟 ≥ 600 s / 无保护停 / 无断流 / 量化门限
LONG_RUN_CRITERIA: Tuple[Dict[str, Any], ...] = (
    {"name": "status_ok", "path": "status", "op": "==", "value": STATUS_OK},
    {"name": "wall_s_at_least_600s", "path": "wall_s", "op": ">=",
     "value": LONG_RUN_DURATION_S},
    {"name": "no_protection_stop", "path": "protection_stop", "op": "is_false"},
    {"name": "no_stream_lost", "path": "stream_lost", "op": "is_false"},
    {"name": "tick_jitter_p99_lt_2ms", "path": "metrics.tick_jitter_p99_ms",
     "op": "<", "value": 2.0},
    {"name": "arrival_err_lt_0p1deg", "path": "metrics.arrival_err_deg",
     "op": "<", "value": 0.1},
    {"name": "follow_err_rms_le_0p3deg", "path": "metrics.follow_err_rms_deg",
     "op": "<=", "value": 0.3},
)

#: 断流判据：断流事件 / L4 动作 / 「关主动上报 → 判流」时延 ≤ STREAM_STOP_MS
STREAM_LOSS_CRITERIA: Tuple[Dict[str, Any], ...] = (
    {"name": "stream_lost_true", "path": "stream_lost", "op": "is_true"},
    {"name": "event_feedback_stream_lost", "path": "events", "op": "contains",
     "value": "feedback_stream_lost"},
    {"name": "l4_action_rm_set_arm_stop", "path": "l4_action", "op": "==",
     "value": L4_SDK_SYMBOL},
    {"name": "stop_latency_le_stream_stop_ms", "path": "stop_latency_ms", "op": "<=",
     "value": STREAM_STOP_MS},
)

#: 判据表与工件段的对应关系
CRITERIA_BY_SECTION = {
    "long_run": LONG_RUN_CRITERIA,
    "stream_loss": STREAM_LOSS_CRITERIA,
}

#: 结案工件必需字段（自校验用）
REQUIRED_TOP_FIELDS = (
    "schema", "mode", "status", "reason", "gate", "long_run", "stream_loss",
    "ladder", "not_executed", "attempted_actions", "next_action", "honesty",
)


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------
def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def _today() -> str:
    return time.strftime("%Y%m%d")


def _get_path(obj: Any, path: str) -> Tuple[Any, bool]:
    """按 ``a.b.c`` 取值，返回 ``(值, 是否存在)``。"""
    cur = obj
    for part in str(path).split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return None, False
    return cur, True


def _is_num(value: Any) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return value == value and abs(value) != float("inf")
    except Exception:  # pragma: no cover - 防御
        return False


def _compare(observed: Any, op: str, expected: Any) -> Optional[bool]:
    """按判据算子比较；**无法判定**返回 ``None``（绝不静默当成通过）。"""
    if op == "is_true":
        return observed is True
    if op == "is_false":
        return observed is False
    if op == "contains":
        if isinstance(observed, (list, tuple)):
            return expected in list(observed)
        return False
    if op == "==":
        if isinstance(expected, str):
            return isinstance(observed, str) and observed == expected
        return observed == expected
    if op in (">=", "<=", ">", "<"):
        if not (_is_num(observed) and _is_num(expected)):
            return None
        if op == ">=":
            return float(observed) >= float(expected)
        if op == "<=":
            return float(observed) <= float(expected)
        if op == ">":
            return float(observed) > float(expected)
        return float(observed) < float(expected)
    return None


def evaluate_criteria(evidence: Optional[Dict[str, Any]],
                      criteria: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """逐条评估判据 → ``[{name, ok, expected, observed, detail}]``。

    ``ok`` 为 ``None`` 表示**无法判定**（证据缺字段）——按未达标处理。
    """
    results: List[Dict[str, Any]] = []
    evidence = evidence if isinstance(evidence, dict) else {}
    for crit in criteria:
        observed, present = _get_path(evidence, crit["path"])
        op = crit["op"]
        expected = crit.get("value")
        verdict = _compare(observed, op, expected) if present else None
        if verdict is None:
            detail = ("证据缺字段 %s，无法判定" % crit["path"]) if not present \
                else ("实测 %r 不满足 %s %r，无法判定" % (observed, op, expected))
            ok = False
        else:
            ok = bool(verdict)
            detail = ("实测 %r 满足 %s %r" % (observed, op, expected)) if ok \
                else ("实测 %r 不满足 %s %r" % (observed, op, expected))
        results.append({
            "name": crit["name"],
            "path": crit["path"],
            "op": op,
            "expected": expected,
            "observed": observed if present else None,
            "present": bool(present),
            "ok": ok,
            "detail": detail,
        })
    return results


def _dedup(items: Sequence[str]) -> List[str]:
    seen = set()
    out: List[str] = []
    for item in items:
        text = str(item).strip()
        if not text or text in seen:
            continue
        seen.add(text)
        out.append(text)
    return out


# --------------------------------------------------------------------------
# 断流证据派生（L4 动作 / 「关主动上报 → 判流」时延）
# --------------------------------------------------------------------------
def _event_types(evidence: Dict[str, Any]) -> List[str]:
    """事件类型列表（工件里可能是字符串列表，也可能是事件字典列表）。"""
    out: List[str] = []
    for item in evidence.get("events") or []:
        if isinstance(item, str):
            out.append(item)
        elif isinstance(item, dict) and item.get("type"):
            out.append(str(item.get("type")))
    for item in evidence.get("stream_lost_evidence") or []:
        if isinstance(item, dict) and item.get("type"):
            out.append(str(item.get("type")))
    for item in evidence.get("safety_events") or []:
        if isinstance(item, dict) and item.get("type"):
            out.append(str(item.get("type")))
    return out


def derive_l4_action(evidence: Dict[str, Any]) -> Optional[str]:
    """从断流证据里派生 L4 动作（应为 ``rm_set_arm_stop``）。

    优先取工件显式字段 ``l4_action``；其次看事件里是否出现 L4 急停事件
    （``rm_safety.levels.EVENT_L4_EMERGENCY_STOP``，唯一来源）→ 记 SDK 符号。
    """
    if not isinstance(evidence, dict):
        return None
    explicit = evidence.get("l4_action")
    if isinstance(explicit, str) and explicit.strip():
        return explicit.strip()
    for item in list(evidence.get("safety_events") or []) \
            + list(evidence.get("stream_lost_evidence") or []) \
            + list(evidence.get("events") or []):
        if not isinstance(item, dict):
            continue
        if str(item.get("type") or "") == EVENT_L4_EMERGENCY_STOP:
            return L4_SDK_SYMBOL
        for key in ("action", "action_call", "sdk_call", "sdk_symbol", "command"):
            value = item.get(key)
            if isinstance(value, str) and "rm_set_arm_stop" in value:
                return value
    if EVENT_L4_EMERGENCY_STOP in _event_types(evidence):
        return L4_SDK_SYMBOL
    return None


def derive_stop_latency_ms(evidence: Dict[str, Any]) -> Optional[float]:
    """派生「关主动上报 → 判流」时延（毫秒）。

    优先取工件显式字段 ``stop_latency_ms`` / ``cut_to_lost_ms``；
    其次用**同一时钟域**的两个时刻相减：断流演练记录的关上报时刻
    （``cut_result.cut_at_perf``）与判流事件的 ``t``（执行器主时钟 = perf_counter）。
    派生不出来 → 返回 ``None``（**无法判定**，绝不填占位数值）。
    """
    if not isinstance(evidence, dict):
        return None
    for key in ("stop_latency_ms", "cut_to_lost_ms", "latency_ms"):
        value = evidence.get(key)
        if _is_num(value):
            return float(value)
    cut = evidence.get("cut_result")
    cut_at = cut.get("cut_at_perf") if isinstance(cut, dict) else None
    if not _is_num(cut_at):
        return None
    lost_t = None
    for item in (evidence.get("stream_lost_evidence") or []) \
            + list(evidence.get("events") or []):
        if isinstance(item, dict) and str(item.get("type") or "") \
                == "feedback_stream_lost":
            value = item.get("t")
            if _is_num(value):
                lost_t = float(value)
                break
    if lost_t is None:
        return None
    return round((lost_t - float(cut_at)) * 1000.0, 3)


# --------------------------------------------------------------------------
# 默认 runner（跑真脚本；测试注入假 runner）
# --------------------------------------------------------------------------
def _run_script(script_name: str, argv: Sequence[str], out_path: Optional[str],
                timeout_s: float) -> Dict[str, Any]:
    """跑一个 scripts/ 下的脚本，返回 ``{exit_code, evidence, path, output}``。"""
    cmd = [sys.executable, os.path.join(HERE, script_name)] + [str(a) for a in argv]
    result: Dict[str, Any] = {"exit_code": None, "evidence": None,
                               "path": out_path, "command": " ".join(cmd),
                               "output": ""}
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=float(timeout_s))
    except subprocess.TimeoutExpired:
        result["exit_code"] = EXIT_CODES[STATUS_ERROR]
        result["output"] = "超时（>%ss）" % timeout_s
        return result
    except OSError as exc:  # pragma: no cover - 环境异常
        result["exit_code"] = EXIT_CODES[STATUS_ERROR]
        result["output"] = "无法启动: %r" % (exc,)
        return result
    out = proc.stdout or b""
    text = out.decode("utf-8", "replace") if isinstance(out, bytes) else str(out)
    result["exit_code"] = int(proc.returncode)
    result["output"] = text[-2000:]
    if out_path and os.path.isfile(out_path):
        try:
            with open(out_path, "r", encoding="utf-8") as fh:
                result["evidence"] = json.load(fh)
        except (OSError, ValueError) as exc:
            result["output"] += "\n[无法解析产物 %s: %r]" % (out_path, exc)
    return result


def _gate_module():
    """取得 rm75_window_gate 模块（测试缝隙：优先复用 sys.modules 注入的模块）。"""
    injected = sys.modules.get("rm75_window_gate")
    if injected is not None:
        return injected
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "rm75_window_gate", os.path.join(HERE, "rm75_window_gate.py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def default_gate(arm: str = "left", ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 udp_ports: Sequence[int] = DEFAULT_UDP_PORTS,
                 ws: Optional[str] = None) -> Dict[str, Any]:
    """默认门禁实现：直接调用 ``rm75_window_gate.run_gate``（真实只读探测）。"""
    gate = _gate_module()
    return gate.run_gate(arm=arm, ip=ip, port=port, udp_ports=udp_ports, ws=ws)


def default_long_run(argv: Sequence[str], out_path: str) -> Dict[str, Any]:
    return _run_script("rm75_long_run.py", argv, out_path, timeout_s=2400.0)


def default_stream_loss(argv: Sequence[str], out_path: str) -> Dict[str, Any]:
    return _run_script("rm75_stream_loss_drill.py", argv, out_path, timeout_s=600.0)


def default_ladder(argv: Sequence[str], out_path: str) -> Dict[str, Any]:
    return _run_script("rm75_validation_ladder.py", argv, out_path, timeout_s=2400.0)


def default_merge(argv: Sequence[str], out_path: str) -> Dict[str, Any]:
    return _run_script("rm75_evidence_merge.py", argv, out_path, timeout_s=300.0)


# --------------------------------------------------------------------------
# 主体
# --------------------------------------------------------------------------
def _section_from_evidence(name: str, evidence: Optional[Dict[str, Any]],
                           criteria: Sequence[Dict[str, Any]],
                           evidence_path: Optional[str],
                           exit_code: Optional[int]) -> Dict[str, Any]:
    """把一份证据变成结案工件里的一个段（判据逐条 + 实测值）。"""
    criteria_results = evaluate_criteria(evidence, criteria)
    passed = all(c["ok"] for c in criteria_results) if criteria_results else False
    failed = [c["name"] for c in criteria_results if not c["ok"]]
    measured: Dict[str, Any] = {}
    if isinstance(evidence, dict):
        for key in ("status", "reason", "mode", "wall_s", "protection_stop",
                    "stream_lost", "planned_points", "events"):
            if key in evidence:
                measured[key] = evidence[key]
        if isinstance(evidence.get("metrics"), dict):
            measured["metrics"] = evidence["metrics"]
        for key in ("l4_action", "stop_latency_ms"):
            if key in evidence:
                measured[key] = evidence[key]
    reason = ("全部判据通过" if passed else
              "未达标判据: %s" % ", ".join(failed))
    return {
        "section": name,
        "executed": True,
        "passed": bool(passed),
        "criteria": criteria_results,
        "failed_criteria": failed,
        "measured": measured,
        "evidence_path": evidence_path,
        "runner_exit_code": exit_code,
        "reason": reason,
    }


def run_window(arm: str = "left", ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
               udp_ports: Sequence[int] = DEFAULT_UDP_PORTS,
               ws: Optional[str] = None,
               duration_s: float = LONG_RUN_DURATION_S,
               settle_s: float = 0.0,
               today: Optional[str] = None,
               gate: Optional[Callable[..., Dict[str, Any]]] = None,
               long_run: Optional[Callable[..., Dict[str, Any]]] = None,
               stream_loss: Optional[Callable[..., Dict[str, Any]]] = None,
               ladder: Optional[Callable[..., Dict[str, Any]]] = None,
               merge: Optional[Callable[..., Dict[str, Any]]] = None,
               long_run_out: str = "build/reports/validation/long_run_600s.json",
               stream_loss_out: str = "build/reports/validation/stream_loss_drill.json",
               ladder_report: str = "build/reports/validation/ladder_report.json",
               ladder_evidence: Optional[str] = None,
               merged_evidence: Optional[str] = None,
               ) -> Dict[str, Any]:
    """跑一遍窗口编排，返回结案工件字典。

    门禁不过 → **不调用任何 runner**（零运动指令），工件走 blocked 分支。
    """
    stamp = today or _today()
    gate_fn = gate or default_gate
    long_run_fn = long_run or default_long_run
    stream_loss_fn = stream_loss or default_stream_loss
    ladder_fn = ladder or default_ladder
    merge_fn = merge or default_merge

    ladder_evidence = ladder_evidence or os.path.join(
        "workcell", "baselines", "evidence",
        "rm75_validation_ladder_%s.json" % stamp)
    merged_evidence = merged_evidence or ladder_evidence

    attempted: List[Dict[str, Any]] = []
    next_actions: List[str] = []
    not_executed: Dict[str, str] = {}

    def act(action: str, command: str, result: str, ok: bool) -> None:
        attempted.append({"action": action, "command": command,
                          "result": result, "ok": bool(ok)})

    # ---------------- 阶段 1：只读门禁 ----------------
    gate_started = _now_iso()
    gate_artifact = gate_fn(arm=arm, ip=ip, port=port, udp_ports=udp_ports, ws=ws)
    gate_artifact = dict(gate_artifact or {})
    gate_status = str(gate_artifact.get("status") or STATUS_ERROR)
    gate_ok = gate_status == STATUS_OK
    gate_artifact.setdefault("attempted_actions", [])
    for item in gate_artifact.get("attempted_actions") or []:
        attempted.append(dict(item))
    next_actions.extend(str(a) for a in (gate_artifact.get("next_action") or []))
    gate_artifact["artifact_path"] = os.path.join(
        "build", "reports", "validation", "window_gate.json")
    gate_artifact["gate_started_at"] = gate_started

    long_run_section: Optional[Dict[str, Any]] = None
    stream_section: Optional[Dict[str, Any]] = None
    ladder_section: Optional[Dict[str, Any]] = None

    if not gate_ok:
        # -------- 纪律：门禁不过 → 一个 runner 都不调用（零运动指令） --------
        reason = ("真机窗口门禁未通过（status=%s）：%s；"
                  "未执行任何运动指令，长跑 / 断流 / 阶梯均未执行。"
                  % (gate_status, gate_artifact.get("reason") or "（无原因）"))
        for name in ("long_run", "stream_loss", "ladder"):
            not_executed[name] = ("门禁未通过（status=%s）：%s；本项未执行，"
                                  "无任何实测数字。"
                                  % (gate_status, gate_artifact.get("reason") or ""))
        next_actions.append("python3 scripts/rm75_window_run.py --out %s"
                            % os.path.join("workcell", "baselines", "evidence",
                                           "rm75_s10_window_outcome.json"))
        status = STATUS_BLOCKED
        mode = "blocked"
    else:
        # ---------------- 阶段 2：长跑 → 断流 → 阶梯（顺序，绝不并发） --------
        long_argv = ["--ip", str(ip), "--port", str(int(port)),
                     "--udp-port", str(int(list(udp_ports)[0]) if udp_ports else 8091),
                     "--arm", arm, "--duration-s", str(duration_s),
                     "--out", long_run_out]
        if ws:
            long_argv += ["--ws", ws]
        long_result = long_run_fn(long_argv, long_run_out)
        long_run_section = _section_from_evidence(
            "long_run", long_result.get("evidence"), LONG_RUN_CRITERIA,
            long_result.get("path") or long_run_out, long_result.get("exit_code"))
        act("long_run", " ".join(["python3", "scripts/rm75_long_run.py"] +
                                 [str(a) for a in long_argv]),
            long_run_section["reason"],
            bool(long_run_section["passed"]))

        if settle_s > 0:
            time.sleep(float(settle_s))
            act("connection_release_wait",
                "sleep %s（等待控制器客户端连接释放，单连接纪律）" % settle_s,
                "已等待 %.0f s" % float(settle_s), True)

        loss_argv = ["--ip", str(ip), "--port", str(int(port)),
                     "--udp-port", str(int(list(udp_ports)[0]) if udp_ports else 8091),
                     "--arm", arm, "--out", stream_loss_out]
        if ws:
            loss_argv += ["--ws", ws]
        loss_result = stream_loss_fn(loss_argv, stream_loss_out)
        loss_evidence = loss_result.get("evidence")
        if isinstance(loss_evidence, dict):
            enriched = dict(loss_evidence)
            if "l4_action" not in enriched:
                enriched["l4_action"] = derive_l4_action(enriched)
            if "stop_latency_ms" not in enriched:
                enriched["stop_latency_ms"] = derive_stop_latency_ms(enriched)
            loss_evidence = enriched
        stream_section = _section_from_evidence(
            "stream_loss", loss_evidence, STREAM_LOSS_CRITERIA,
            loss_result.get("path") or stream_loss_out, loss_result.get("exit_code"))
        act("stream_loss_drill",
            " ".join(["python3", "scripts/rm75_stream_loss_drill.py"] +
                     [str(a) for a in loss_argv]),
            stream_section["reason"], bool(stream_section["passed"]))

        ladder_argv = ["--ip", str(ip), "--port", str(int(port)),
                       "--udp-port", str(int(list(udp_ports)[0]) if udp_ports else 8091),
                       "--arm", arm, "--max-stage", "V9", "--allow-risky",
                       "--evidence-out", ladder_evidence]
        if ws:
            ladder_argv += ["--ws", ws]
        ladder_result = ladder_fn(ladder_argv, ladder_evidence)
        ladder_evidence = ladder_result.get("evidence")
        ladder_section = {
            "section": "ladder",
            "executed": True,
            "status": (ladder_evidence or {}).get("status"),
            "summary": (ladder_evidence or {}).get("summary"),
            "evidence_path": ladder_result.get("path") or ladder_evidence,
            "runner_exit_code": ladder_result.get("exit_code"),
            "reason": "阶梯刷新结果: status=%s"
                      % ((ladder_evidence or {}).get("status")),
        }
        act("validation_ladder",
            " ".join(["python3", "scripts/rm75_validation_ladder.py"] +
                     [str(a) for a in ladder_argv]),
            ladder_section["reason"],
            ladder_section["status"] in ("pass", "dry_run"))

        merge_argv = [ladder_report, "--out", merged_evidence]
        merge_result = merge_fn(merge_argv, merged_evidence)
        act("evidence_merge",
            " ".join(["python3", "scripts/rm75_evidence_merge.py"] +
                     [str(a) for a in merge_argv]),
            "证据入库: %s" % merged_evidence, merge_result.get("exit_code") == 0)
        ladder_section["merged_evidence_path"] = merged_evidence

        long_ok = bool(long_run_section["passed"])
        loss_ok = bool(stream_section["passed"])
        if long_ok and loss_ok:
            status = STATUS_OK
            reason = ("真机窗口内完成 600 s 连续执行长跑与 UDP 断流演练，"
                      "全部判据通过（长跑墙钟 %.1f s；断流 L4 动作 %s，"
                      "「关主动上报 → 判流」时延 %s ms ≤ %d ms）。"
                      % (float((long_run_section["measured"].get("wall_s") or 0.0)),
                         stream_section["measured"].get("l4_action"),
                         stream_section["measured"].get("stop_latency_ms"),
                         STREAM_STOP_MS))
        else:
            status = STATUS_ERROR
            bad = []
            if not long_ok:
                bad.append("长跑未达标（%s）" % long_run_section["reason"])
            if not loss_ok:
                bad.append("断流未达标（%s）" % stream_section["reason"])
            reason = ("真机窗口已执行，但 S10 判据未通过：%s。"
                      "工件如实记录实测值，不得据此声称真机长跑/断流已达标。"
                      % "；".join(bad))
        mode = _derive_mode(long_run_section, stream_section, ladder_section)

    artifact = {
        "schema": SCHEMA,
        "mode": mode,
        "status": status,
        "reason": reason,
        "generated_at": _now_iso(),
        "arm": arm,
        "endpoint": {"ip": ip, "port": int(port),
                     "udp_ports": [int(p) for p in (udp_ports or ())]},
        "gate": gate_artifact,
        "long_run": long_run_section,
        "stream_loss": stream_section,
        "ladder": ladder_section,
        "not_executed": not_executed,
        "reference_values": REFERENCE_VALUES,
        "attempted_actions": attempted,
        "next_action": _dedup(next_actions),
        "honesty": HONESTY,
        "exit_code": int(EXIT_CODES.get(status, EXIT_CODES[STATUS_BLOCKED])),
    }
    return artifact


def _derive_mode(long_run: Optional[Dict[str, Any]],
                 stream_loss: Optional[Dict[str, Any]],
                 ladder: Optional[Dict[str, Any]]) -> str:
    """结论 mode：全真机 = hardware；混入假/离线结果 = fake/offline/mixed。"""
    modes = set()
    for section in (long_run, stream_loss):
        if section:
            measured = section.get("measured") or {}
            modes.add(str(measured.get("mode") or ""))
    if not modes:
        return "blocked"
    modes.discard("")
    if modes == {"hardware"}:
        return "hardware"
    if "fake" in modes:
        return "fake" if len(modes) == 1 else "mixed"
    if "offline" in modes:
        return "offline" if len(modes) == 1 else "mixed"
    return "mixed"


# --------------------------------------------------------------------------
# 自校验
# --------------------------------------------------------------------------
def validate_artifact(artifact: Any) -> Tuple[bool, List[str]]:
    """校验结案工件自洽性，返回 ``(是否自洽, 问题列表)``。"""
    problems: List[str] = []
    if not isinstance(artifact, dict):
        return False, ["工件不是 JSON 对象"]

    if artifact.get("schema") != SCHEMA:
        problems.append("schema 不符：%r（应为 %s）" % (artifact.get("schema"), SCHEMA))
    for field in REQUIRED_TOP_FIELDS:
        if field not in artifact:
            problems.append("工件缺字段 %s" % field)
    status = str(artifact.get("status") or "")
    if status not in EXIT_CODES:
        problems.append("status 不合法：%r（只允许 %s）" % (status, sorted(EXIT_CODES)))
    mode = str(artifact.get("mode") or "")
    if mode not in MODES:
        problems.append("mode 不合法：%r（只允许 %s）" % (mode, list(MODES)))
    if not str(artifact.get("reason") or "").strip():
        problems.append("reason 为空")
    honesty = str(artifact.get("honesty") or "")
    if not honesty.strip():
        problems.append("honesty 声明缺失")
    elif "真机" not in honesty:
        problems.append("honesty 声明未提「真机」")
    if status != STATUS_OK:
        if not artifact.get("next_action"):
            problems.append("status=%s 时 next_action 不得为空" % status)
    else:
        if not artifact.get("next_action"):
            problems.append("next_action 为空（结案也必须给出可粘贴的下一步）")

    gate = artifact.get("gate")
    if not isinstance(gate, dict):
        problems.append("gate 段缺失或不是对象")
    else:
        if not gate.get("checks"):
            problems.append("gate 段缺少逐项 checks")
        if not gate.get("status"):
            problems.append("gate 段缺少 status")

    not_executed = artifact.get("not_executed")
    if not isinstance(not_executed, dict):
        not_executed = {}
        problems.append("not_executed 段缺失或不是对象")

    for name, criteria in CRITERIA_BY_SECTION.items():
        section = artifact.get(name)
        if section is None:
            reason = str((not_executed.get(name) or "")).strip()
            if not reason:
                problems.append("%s 未执行但未写明未执行原因" % name)
            continue
        if not isinstance(section, dict):
            problems.append("%s 段不是对象" % name)
            continue
        results = section.get("criteria")
        if not isinstance(results, list) or not results:
            problems.append("%s 段缺少判据逐条结果" % name)
            continue
        got = {c.get("name") for c in results if isinstance(c, dict)}
        want = {c["name"] for c in criteria}
        if got != want:
            problems.append("%s 段判据不齐：缺 %s / 多 %s"
                            % (name, sorted(want - got), sorted(got - want)))
        for c in results:
            if not isinstance(c, dict) or "ok" not in c:
                problems.append("%s 段判据条目缺 ok 字段" % name)
        if "passed" not in section:
            problems.append("%s 段缺 passed 字段" % name)
        if not str(section.get("reason") or "").strip():
            problems.append("%s 段缺 reason" % name)
        if not str(section.get("evidence_path") or "").strip():
            problems.append("%s 段缺证据路径" % name)

    ladder = artifact.get("ladder")
    if ladder is None:
        if not str((not_executed.get("ladder") or "")).strip():
            problems.append("ladder 未执行但未写明未执行原因")
    elif not isinstance(ladder, dict):
        problems.append("ladder 段不是对象")
    elif not ladder.get("status"):
        problems.append("ladder 段缺 status")

    exit_code = artifact.get("exit_code")
    if status in EXIT_CODES and exit_code is not None:
        if int(exit_code) != int(EXIT_CODES[status]):
            problems.append("exit_code=%r 与 status=%s 不一致（应为 %d）"
                            % (exit_code, status, EXIT_CODES[status]))
    return (not problems), problems


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rm75_window_run",
        description="RM75-B 真机窗口单一入口（门禁 → 长跑 → 断流 → 阶梯 → 结案工件）")
    p.add_argument("--arm", choices=["left", "right"], default="left")
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--udp-ports", dest="udp_ports",
                   default=",".join(str(p) for p in DEFAULT_UDP_PORTS))
    p.add_argument("--ws", default=None)
    p.add_argument("--duration-s", dest="duration_s", type=float,
                   default=LONG_RUN_DURATION_S)
    p.add_argument("--settle-s", dest="settle_s", type=float, default=0.0,
                   help="长跑与断流之间等待连接释放的秒数（单连接纪律）")
    p.add_argument("--out", default=os.path.join(
        "workcell", "baselines", "evidence", "rm75_s10_window_outcome.json"))
    p.add_argument("--validate", dest="validate", default=None,
                   help="只校验一份结案工件（不自洽 → 退出码 3）")
    p.add_argument("--json", action="store_true")
    return p


def main(argv: Optional[List[str]] = None,
         gate: Optional[Callable[..., Dict[str, Any]]] = None,
         long_run: Optional[Callable[..., Dict[str, Any]]] = None,
         stream_loss: Optional[Callable[..., Dict[str, Any]]] = None,
         ladder: Optional[Callable[..., Dict[str, Any]]] = None,
         merge: Optional[Callable[..., Dict[str, Any]]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.validate:
        path = os.path.abspath(args.validate)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                artifact = json.load(fh)
        except (OSError, ValueError) as exc:
            sys.stderr.write("无法读取结案工件 %s: %r\n" % (path, exc))
            return EXIT_CODES[STATUS_BLOCKED]
        ok, problems = validate_artifact(artifact)
        if ok:
            print("结案工件自洽: %s（status=%s, mode=%s）"
                  % (path, artifact.get("status"), artifact.get("mode")))
            return 0
        print("结案工件不自洽: %s" % path)
        for problem in problems:
            print("  - %s" % problem)
        return EXIT_CODES[STATUS_BLOCKED]

    try:
        ports = [int(x) for x in str(args.udp_ports).split(",") if x.strip()]
    except ValueError:
        sys.stderr.write("--udp-ports 非法: %r\n" % args.udp_ports)
        return EXIT_CODES[STATUS_ERROR]

    artifact = run_window(
        arm=args.arm, ip=args.ip, port=args.port, udp_ports=ports, ws=args.ws,
        duration_s=args.duration_s, settle_s=args.settle_s,
        gate=gate, long_run=long_run, stream_loss=stream_loss,
        ladder=ladder, merge=merge,
    )

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(artifact, fh, ensure_ascii=False, indent=2)

    if args.json:
        print(json.dumps(artifact, ensure_ascii=False, indent=2))
    else:
        print("真机窗口结案 | status=%s | mode=%s" % (artifact["status"],
                                                      artifact["mode"]))
        print("  原因: %s" % artifact["reason"])
        for name in ("long_run", "stream_loss", "ladder"):
            section = artifact.get(name)
            if section is None:
                print("  %s: 未执行（%s）" % (name,
                                             artifact["not_executed"].get(name, "")))
            elif name == "ladder":
                print("  %s: status=%s" % (name, section.get("status")))
            else:
                print("  %s: passed=%s（%s）"
                      % (name, section.get("passed"), section.get("reason")))
        for action in artifact.get("next_action", []):
            print("  下一步: %s" % action)
        print("  诚实声明: %s" % artifact["honesty"])
        print("  工件已写入: %s" % out)

    status = str(artifact.get("status") or STATUS_ERROR)
    if status not in EXIT_CODES:
        status = STATUS_ERROR
    return int(EXIT_CODES[status])


if __name__ == "__main__":
    sys.exit(main())
