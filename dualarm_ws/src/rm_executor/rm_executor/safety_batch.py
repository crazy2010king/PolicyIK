"""安全用例批量跑批（S04/T6）——五类故障各自落到预期等级 L1–L4 并落盘证据。

用法::

    PYTHONPATH=dualarm_ws/src/rm_safety:dualarm_ws/src/rm_hw_bridge:dualarm_ws/src/rm_executor \
        python3 -m rm_executor.safety_batch --all

六条离线用例（**只对 fake 开放**，真机路径拒绝任何注入）：

==================  ============================  ======  ==================
用例                 注入                          等级    故障类
==================  ============================  ======  ==================
follow_error_light  ``follow_error_light:3.0``   L1     follow_error
follow_error_heavy  ``follow_error:6.0``          L2     follow_error
limit_exceeded      ``limit_exceeded:2.0``        L2     limit_exceeded
collision           ``collision:90001``           L3     collision
protection_stop     ``protection_stop``           L4     protection_stop
stream_lost         ``feedback_stall:250``        L4     stream_lost
==================  ============================  ======  ==================

每条用例落一份 ``<out-dir>/s04_<case>.json``（等级 / 故障类 / 严重度 /
退出码 / 动作序列 / SDK 调用序列 / 证据字段 / 恢复判定 / verdict），
汇总落 ``<out-dir>/s04_safety_cases.json``。

退出码：全部用例命中预期等级 → **0**；期望不符 → **3**；内部错误 → **4**。

**诚实声明**
------------

* 离线跑批得到的是**离线代理结论**，不代表真机结论（真机结论见真机窗口）；
* 碰撞只能靠控制器错误码表达，而 RM-API2 ``err[]`` 的**厂商错误码表本轮
  未取得**——``collision`` 用例用的是 ``workcell/safety/error_codes.yaml``
  里的**工程声明映射码**（``class=collision``），真机窗口必须逐条回填。
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import tempfile
from typing import Any, Dict, List, Optional, Sequence, Tuple

from rm_executor import cli as cli_mod
from rm_executor import contract
from rm_executor import injection as injection_mod
from rm_executor import paths as paths_mod
from rm_executor import sample as sample_mod
from rm_executor.clock import ManualClock
from rm_executor.executor import Executor
from rm_executor.transport import FakeTransport

__all__ = [
    "CASES",
    "FAULT_CLASS",
    "build_parser",
    "main",
    "run_all",
    "run_case",
]

#: 跑批用的栅格周期（秒）——与执行层 10 ms 单时钟一致
DEFAULT_PERIOD_S = 0.010

#: 离线假传输的调用记录保留上限（长跑时防内存无界增长；计数不受影响）
FAKE_CALL_RECORD_LIMIT = 2000

#: 报告默认输出目录名（相对工作区根）
DEFAULT_OUT_DIRNAME = os.path.join("build", "reports")

#: 汇总报告文件名
SUMMARY_FILENAME = "s04_safety_cases.json"

#: 轨迹文件名（跑批输入，合成轨迹）
TRAJECTORY_FILENAME = "s04_sample_traj.json"


def _fault_class_table() -> Dict[str, str]:
    """policy 故障类 → 计划口径的「故障类」（粗类）。

    跟随误差的轻/重两级（``follow_error_light`` / ``follow_error_heavy``）
    在计划口径里同属 ``follow_error``；其余一一对应。
    """
    from rm_safety import policy as safety_policy

    return {
        safety_policy.FAULT_KIND_FOLLOW_ERROR_LIGHT: "follow_error",
        safety_policy.FAULT_KIND_FOLLOW_ERROR_HEAVY: "follow_error",
        safety_policy.FAULT_KIND_LIMIT_EXCEEDED: "limit_exceeded",
        safety_policy.FAULT_KIND_COLLISION: "collision",
        safety_policy.FAULT_KIND_PROTECTION_STOP: "protection_stop",
        safety_policy.FAULT_KIND_STREAM_LOST: "stream_lost",
        safety_policy.FAULT_KIND_UNKNOWN_ERROR_CODE: "unknown_error_code",
    }


#: policy 故障类 → 计划口径故障类（懒加载填充，见 :func:`_fault_class_table`）
FAULT_CLASS: Dict[str, str] = {}


#: 六条离线用例（注入 / 预期等级 / 预期故障类 / 预期动作）
CASES: Tuple[Dict[str, Any], ...] = (
    {
        "case": "follow_error_light",
        "injection": "follow_error_light:3.0",
        "expected_level": "L1",
        "expected_fault_class": "follow_error",
        "expected_action": "set_speed_scale",
        "note": "轻跟随误差（告警阈值之上、停机阈值之内）→ L1 降速",
    },
    {
        "case": "follow_error_heavy",
        "injection": "follow_error:6.0",
        "expected_level": "L2",
        "expected_fault_class": "follow_error",
        "expected_action": "pause",
        "note": "重跟随误差（超过停机阈值）→ L2 暂停",
    },
    {
        "case": "limit_exceeded",
        "injection": "limit_exceeded:2.0",
        "expected_level": "L2",
        "expected_fault_class": "limit_exceeded",
        "expected_action": "pause",
        "note": "运行期位置越过声明限位 → L2 暂停",
    },
    {
        "case": "collision",
        "injection": "collision:90001",
        "expected_level": "L3",
        "expected_fault_class": "collision",
        "expected_action": "slow_stop",
        "note": "控制器碰撞错误码（工程声明映射码）→ L3 缓停",
    },
    {
        "case": "protection_stop",
        "injection": "protection_stop",
        "expected_level": "L4",
        "expected_fault_class": "protection_stop",
        "expected_action": "stop",
        "note": "控制器保护停（RM_STOP_E）→ L4 急停",
    },
    {
        "case": "stream_lost",
        "injection": "feedback_stall:250",
        "expected_level": "L4",
        "expected_fault_class": "stream_lost",
        "expected_action": "stop",
        "note": "反馈断流（超过断流停机阈值）→ L4 急停",
    },
)


# --------------------------------------------------------------------- 工具
def _safety_levels():
    """导入安全层等级表（唯一真值）；不可用 → 抛 :class:`RuntimeError`。"""
    try:
        from rm_safety import levels as safety_levels
    except Exception as exc:  # noqa: BLE001 - 安全层缺失 → 明确报错
        raise RuntimeError(
            "无法导入安全层 rm_safety（%s: %s）；"
            "export PYTHONPATH=$PWD/dualarm_ws/src/rm_safety:$PYTHONPATH 后重跑"
            % (type(exc).__name__, exc)) from exc
    return safety_levels


def _level_event_map() -> Dict[str, str]:
    """安全等级事件名 → 等级（如 ``safety_l1_slowdown`` → ``L1``）。"""
    levels = _safety_levels()
    return {levels.level_event_type(level): level
            for level in levels.LEVEL_ORDER}


def _parse_evidence(items: Sequence[Any]) -> List[Dict[str, Any]]:
    """把事件 evidence 里的 JSON 字符串还原成结构化 dict（非 JSON 的忽略）。"""
    out: List[Dict[str, Any]] = []
    for item in items or []:
        if isinstance(item, dict):
            out.append(dict(item))
            continue
        if not isinstance(item, str):
            continue
        try:
            parsed = json.loads(item)
        except (TypeError, ValueError):
            continue
        if isinstance(parsed, dict):
            out.append(parsed)
    return out


def _primary_fault_kind(events: Sequence[Dict[str, Any]]) -> str:
    """从事件里取**主因**故障类（安全包判据的原始 fault_kind）。

    * 故障类事件（``fault_*``）的 reason 形如「主因故障 X → Y」；
    * 没有故障类事件时（例如未知错误码）回退到等级事件 reason 的前缀。
    """
    for event in events or []:
        if str(event.get("type") or "").startswith("fault_"):
            reason = str(event.get("reason") or "")
            marker = "主因故障 "
            if marker in reason:
                tail = reason.split(marker, 1)[1]
                return tail.split("→")[0].strip()
    for event in events or []:
        if str(event.get("type") or "") in _level_event_map():
            reason = str(event.get("reason") or "")
            head = reason.split("｜")[0]
            return head.split("→")[0].strip()
    return ""


def _extract_actions(evidence: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """从证据里抽出安全动作序列（等级 → 动作 → 传输方法 → SDK 符号 → 状态）。"""
    out: List[Dict[str, Any]] = []
    for item in evidence or []:
        if "action" not in item or "transport_method" not in item:
            continue
        out.append({
            "kind": item.get("kind"),
            "level": item.get("level"),
            "action": item.get("action"),
            "transport_method": item.get("transport_method"),
            "sdk_symbol": item.get("sdk_symbol"),
            "status": item.get("kind"),
            "reason": item.get("reason"),
        })
    return out


# ----------------------------------------------------------------- 单例跑批
def run_case(spec: Dict[str, Any], ws: Optional[str] = None,
             out_dir: Optional[str] = None,
             trajectory_path: Optional[str] = None) -> Dict[str, Any]:
    """跑一条安全用例，返回该用例的报告字典（**不抛裸异常**）。

    参数
    ----
    spec
        用例定义（见 :data:`CASES`）：``case`` / ``injection`` /
        ``expected_level`` / ``expected_fault_class`` / ``expected_action``。
    ws
        工作区根（``None`` → 自动发现）。
    out_dir
        报告输出目录（``None`` → 不落盘）。
    trajectory_path
        轨迹 JSON 路径（``None`` → 由调用方生成）。
    """
    levels = _safety_levels()
    if not FAULT_CLASS:
        FAULT_CLASS.update(_fault_class_table())

    name = str(spec.get("case") or "case")
    injection_spec = str(spec.get("injection") or "")
    expected_level = str(spec.get("expected_level") or "")
    expected_class = str(spec.get("expected_fault_class") or "")
    expected_action = str(spec.get("expected_action") or "")

    report: Dict[str, Any] = {
        "case": name,
        "injection": injection_spec,
        "note": str(spec.get("note") or ""),
        "expected_level": expected_level,
        "expected_fault_class": expected_class,
        "expected_action": expected_action,
        "level": "",
        "fault_kind": "",
        "fault_kind_detail": "",
        "severity": "",
        "exit_code": 0,
        "threshold": None,
        "observed": None,
        "margin_to_limit": None,
        "next_action": "",
        "action_sequence": [],
        "sdk_call_sequence": [],
        "sdk_call_counts": {},
        "detected_fault_kinds": [],
        "evidence": [],
        "recovery": "",
        "auto_recovery_occurred": False,
        "executor_status": "",
        "executor_exit_code": 0,
        "verdict": "fail",
        "verdict_reason": "",
        "honesty": "离线代理结论（fake 模式），不代表真机结论",
    }

    # -- 注入规格解析（非法 → **拒绝**，不落「空报告」）-------------------
    try:
        parsed = [injection_mod.parse_injection(injection_spec)]
    except ValueError as exc:
        # 解析阶段就被拒：既不执行也不落盘（避免产出「跑了但没动作」的空报告）
        report["verdict"] = "invalid_spec"
        report["verdict_reason"] = "注入规格非法：%s" % exc
        report["executor_status"] = contract.STATUS_BLOCKED
        report["executor_exit_code"] = contract.EXIT_CODES[contract.STATUS_BLOCKED]
        return report

    if not trajectory_path:
        report["verdict"] = "error"
        report["verdict_reason"] = "缺少轨迹文件（trajectory_path 为空）"
        report["executor_status"] = contract.STATUS_ERROR
        report["executor_exit_code"] = contract.EXIT_CODES[contract.STATUS_ERROR]
        if out_dir:
            _dump(report, os.path.join(out_dir, "s04_%s.json" % name))
        return report

    # -- 离线假执行 -------------------------------------------------------
    transport = FakeTransport(max_records=FAKE_CALL_RECORD_LIMIT)
    feedback = cli_mod._mirror_feedback(transport, DEFAULT_PERIOD_S)
    transport, feedback = injection_mod.apply_injections(
        transport, feedback, parsed, period_s=DEFAULT_PERIOD_S, ws=ws)
    executor = Executor(transport, feedback,
                        clock=ManualClock(period_s=DEFAULT_PERIOD_S),
                        mode="fake", ws=ws)
    exec_report = executor.run(trajectory_path)
    events = list(exec_report.get("events") or [])

    # -- 提取安全判定结果 --------------------------------------------------
    level_map = _level_event_map()
    level = ""
    for event in events:
        candidate = level_map.get(str(event.get("type") or ""))
        if candidate and (not level or
                          levels.level_rank(candidate) > levels.level_rank(level)):
            level = candidate
    detail_kind = _primary_fault_kind(events)
    fault_class = FAULT_CLASS.get(detail_kind, detail_kind)

    # 证据只从**等级事件**取（故障类事件携带的是同一份证据，取两份会把
    # 动作序列重复计数——动作权威必须「只算一次」）
    evidence: List[Dict[str, Any]] = []
    for event in events:
        if str(event.get("type") or "") in level_map:
            evidence.extend(_parse_evidence(event.get("evidence")))
    primary = None
    for item in evidence:
        if item.get("fault_kind") == detail_kind:
            primary = item
            break

    actions = _extract_actions(evidence)
    detected = []
    for item in evidence:
        kind = item.get("fault_kind")
        if kind and kind not in detected:
            detected.append(kind)

    calls = [str(c.get("method")) for c in getattr(transport, "calls", [])]
    counts: Dict[str, int] = {}
    for method in calls:
        counts[method] = counts.get(method, 0) + 1

    status = str(exec_report.get("status") or contract.STATUS_ERROR)
    next_action = ""
    for event in events:
        if level_map.get(str(event.get("type") or "")) == level and \
                event.get("next_action"):
            next_action = str(event["next_action"])
            break

    report.update({
        "level": level,
        "fault_kind": fault_class,
        "fault_kind_detail": detail_kind,
        "severity": levels.level_severity(level) if level else "",
        "exit_code": levels.level_exit_code(level) if level else 0,
        "threshold": (primary or {}).get("threshold"),
        "observed": (primary or {}).get("observed"),
        "margin_to_limit": (primary or {}).get("margin"),
        "next_action": next_action or (primary or {}).get("next_action") or "",
        "action_sequence": actions,
        "sdk_call_sequence": calls,
        "sdk_call_counts": counts,
        "detected_fault_kinds": detected,
        "evidence": evidence,
        "recovery": levels.level_recovery(level) if level else "",
        "auto_recovery_occurred": any(
            str(e.get("type") or "") == levels.EVENT_L1_SLOWDOWN_CLEARED
            for e in events),
        "executor_status": status,
        "executor_exit_code": contract.EXIT_CODES.get(status, 4),
        "report": exec_report,
    })

    # -- verdict：等级 + 故障类 + 动作三者逐条命中 --------------------------
    action_names = [str(a.get("action")) for a in actions]
    problems: List[str] = []
    if level != expected_level:
        problems.append("等级 %s ≠ 预期 %s" % (level or "无", expected_level))
    if fault_class != expected_class:
        problems.append("故障类 %s ≠ 预期 %s"
                        % (fault_class or "无", expected_class))
    if expected_action and expected_action not in action_names:
        problems.append("动作序列 %s 不含预期动作 %s"
                        % (action_names or "无", expected_action))
    if problems:
        report["verdict"] = "fail"
        report["verdict_reason"] = "；".join(problems)
    else:
        report["verdict"] = "pass"
        report["verdict_reason"] = "等级/故障类/动作序列逐条命中预期"

    if out_dir:
        _dump(report, os.path.join(out_dir, "s04_%s.json" % name))
    return report


def _dump(payload: Dict[str, Any], path: str) -> str:
    """把报告落盘（UTF-8，不转义）；返回绝对路径。"""
    target = os.path.abspath(path)
    parent = os.path.dirname(target)
    if parent and not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    with open(target, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, sort_keys=False,
                  default=str)
        fh.write("\n")
    return target


# --------------------------------------------------------------------- 跑批
def run_all(ws: Optional[str] = None, out_dir: Optional[str] = None,
            cases: Optional[Sequence[Dict[str, Any]]] = None) -> Dict[str, Any]:
    """跑一批用例并落盘（每例一份 + 汇总一份）；返回汇总字典。

    汇总字典含 ``passed`` / ``failed`` 计数与逐例结论（``cases``）。
    """
    specs = list(cases if cases is not None else CASES)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    # 跑批输入：确定性合成轨迹（落 out_dir；无 out_dir 时落临时目录）
    # **采样周期用 sample 模块的默认 50 ms**（不是执行层的 10 ms 栅格）：
    # 10 ms 采样的正弦轨迹在中心差分下导数会超限（位置/速度/加速度/jerk），
    # 轨迹会被阶段 A 拒收 → 跑批就会「跑了但没故障」。
    tmp_dir = None
    if out_dir:
        trajectory_path = sample_mod.write_sample_trajectory(
            os.path.join(out_dir, TRAJECTORY_FILENAME))
    else:
        tmp_dir = tempfile.mkdtemp(prefix="rm_s04_batch_")
        trajectory_path = sample_mod.write_sample_trajectory(
            os.path.join(tmp_dir, TRAJECTORY_FILENAME))

    try:
        results: List[Dict[str, Any]] = []
        for spec in specs:
            results.append(run_case(spec, ws=ws, out_dir=out_dir,
                                     trajectory_path=trajectory_path))
    finally:
        if tmp_dir and os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir, ignore_errors=True)

    passed = sum(1 for r in results if r.get("verdict") == "pass")
    summary = {
        "total": len(results),
        "passed": passed,
        "failed": len(results) - passed,
        "verdict": "pass" if passed == len(results) and results else "fail",
        "honesty": ("离线代理结论（fake 模式），不代表真机结论；"
                    "碰撞码为工程声明映射（厂商 err[] 码表本轮未取得，待真机回填）"),
        "cases": [
            {
                "case": r.get("case"),
                "injection": r.get("injection"),
                "expected_level": r.get("expected_level"),
                "level": r.get("level"),
                "expected_fault_class": r.get("expected_fault_class"),
                "fault_kind": r.get("fault_kind"),
                "fault_kind_detail": r.get("fault_kind_detail"),
                "severity": r.get("severity"),
                "exit_code": r.get("exit_code"),
                "expected_action": r.get("expected_action"),
                "action_sequence": [a.get("action") for a in
                                    r.get("action_sequence") or []],
                "recovery": r.get("recovery"),
                "verdict": r.get("verdict"),
                "verdict_reason": r.get("verdict_reason"),
                "report": "s04_%s.json" % r.get("case"),
            }
            for r in results
        ],
    }
    if out_dir:
        _dump(summary, os.path.join(out_dir, SUMMARY_FILENAME))
    return summary


# --------------------------------------------------------------------- CLI
def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="rm_executor.safety_batch",
        description="安全用例批量跑批（五类故障 → L1–L4，离线 fake）",
    )
    parser.add_argument("--all", action="store_true",
                        help="跑全部六条用例（默认行为）")
    parser.add_argument("--case", action="append", default=None, metavar="NAME",
                        help="只跑指定用例（可重复）：%s"
                        % " / ".join(c["case"] for c in CASES))
    parser.add_argument("--out-dir", dest="out_dir", default=None,
                        help="报告输出目录（默认 <工作区根>/%s）" % DEFAULT_OUT_DIRNAME)
    parser.add_argument("--ws", default=None,
                        help="工作区根（默认自动发现或 $RM_WS）")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 主入口；返回进程退出码（0 命中 / 3 不符 / 4 内部错误）。"""
    args = build_parser().parse_args(argv)
    out_dir = args.out_dir or os.path.join(paths_mod.workspace_root(args.ws),
                                           DEFAULT_OUT_DIRNAME)
    specs = list(CASES)
    if args.case:
        wanted = list(args.case)
        known = {c["case"]: c for c in CASES}
        missing = [name for name in wanted if name not in known]
        if missing:
            print("未知用例：%s（可用：%s）"
                  % (" / ".join(missing), " / ".join(known)), file=sys.stderr)
            return contract.EXIT_CODES[contract.STATUS_BLOCKED]
        specs = [known[name] for name in wanted]
    try:
        summary = run_all(ws=args.ws, out_dir=out_dir, cases=specs)
    except Exception as exc:  # noqa: BLE001 - 内部错误 → 退出码 4
        print("跑批内部错误（%s: %s）" % (type(exc).__name__, exc),
              file=sys.stderr)
        return contract.EXIT_CODES[contract.STATUS_ERROR]

    for item in summary["cases"]:
        print("%-20s %-4s %-16s %-14s %s"
              % (item["case"], item["level"] or "-", item["fault_kind"] or "-",
                 ",".join(item["action_sequence"]) or "-", item["verdict"]))
    print("passed=%d failed=%d → %s"
          % (summary["passed"], summary["failed"], summary["verdict"]))
    print("reports   : %s" % out_dir)
    if summary["failed"]:
        return contract.EXIT_CODES[contract.STATUS_BLOCKED]
    return contract.EXIT_CODES[contract.STATUS_OK]


if __name__ == "__main__":
    sys.exit(main())
