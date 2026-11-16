"""体验基准 E1–E4 的可执行测量（S09/T5）：**离线口径**实测 + 人工 pending 字段。

做什么 / 不做什么
------------------

* **做**：读声明式场景表（``workcell/bench/experience_e1_e4.yaml``）→ 在
  **临时工作区**里跑 E1–E4 的可自动测部分 → 产出
  ``build/reports/s09_experience.json`` + ``build/reports/s09_experience.md``。
* **不做**：**不重测**性能基线、**不另造基准**、**不写真机数字**。E1–E4 的
  权威定义来自设计 §6.9，目标值来自设计 §6.2（都写在 YAML 里，Python 只读）。

诚实分层（硬纪律）
------------------

1. **每个指标都带 ``mode``**（口径在 YAML 里声明，代码只读）：
   ``offline``（离线实测）/ ``human_pending``（人工才能测，值必须为
   ``null`` + ``status=pending``）。本模块**永不产出** ``mode=live`` 的数字
   ——真机口径本切片未实证，冒充即造假（D012 分层纪律）。
2. **人工字段不许被顶替**：``mttr_human_s`` 的 value 必须是 ``null``；
   校验器 :func:`validate_metrics` 会拒绝「pending 指标却有值」（含填 0）。
3. **缺数据源 ≠ 0**：数据源缺失 → 相关指标记 ``value=null`` +
   ``status=blocked`` + reason，**绝不**用 0 冒充真实值。
4. **目标值只来自 YAML**：设计没给数值目标的指标 ``target=null`` + note
   说明，**不编造目标值**。

四条体验场景（设计 §6.9）：E1 接入新机型（第二个 URDF 变体作代理，临时
工作区里走 prepare→init→doctor→plan 链路，测 T2FM / 命令条数 / 配置行数 /
改码行数 / 报错次数；变体装不上 → 计入报错次数且 T2FM 记 null）；E2 交付
新工艺（临时目录写最小新任务 YAML，测模板行数 / 命令行数 / 首次成功率）；
E3 现场故障恢复（从 S06 trace 派生 MTTR_proxy 与报错可诊断率，真 MTTR 必须是
null + pending）；E4 日常操作（命令条数与单次操作耗时）。

依赖方向：本模块属于体验层 ``rmctl``，只读**数据文件**与执行层契约常量
（经 :mod:`rmctl.regression.segments`）；**不** import 硬件适配层、
**不** import 厂商规划内核。
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from rmctl.regression import segments as seg_mod
from rmctl.regression.experience_helpers import (
    LIMIT_TOL,
    diff_py_lines,
    parse_doctor,
    urdf_limits,
    yaml_limits,
)
from rmctl.regression.experience_report import (
    METRIC_MODES,
    METRIC_STATUS,
    SCENARIOS_REL,
    SEGMENT_MODE,
    TARGET_OPS,
    build_metric,
    fmt_value,
    target_met,
    validate_metrics,
    write_json,
    write_markdown,
)

__all__ = [
    "JSON_NAME",
    "MD_NAME",
    "METRIC_MODES",
    "METRIC_STATUS",
    "SCENARIOS_REL",
    "CommandResult",
    "build_metric",
    "derive_e3_from_traces",
    "load_scenarios",
    "measure_e1",
    "measure_e2",
    "measure_e3",
    "measure_e4",
    "run_command",
    "run_experience",
    "scenarios_path",
    "target_met",
    "validate_metrics",
]

#: 本模块产出的两个文件（相对工作区 ``build/reports/``）
JSON_NAME = "s09_experience.json"
MD_NAME = "s09_experience.md"

#: 段报告文件（与其它段同构，写在 ``build/reports/`` 下）
SEGMENT_REPORT_NAME = "s09_segment_experience.json"

#: E3 的失败事件判定：severity 非 INFO 或 status 非 ok
_FAIL_SEVERITIES = ("RECOVERABLE", "NEEDS_PARAM", "NEEDS_HUMAN", "FATAL")


# ---------------------------------------------------------------------------
# 0. 通用工具
# ---------------------------------------------------------------------------
def default_repo_root() -> str:
    """仓库根（含 ``workcell/`` 与 ``dualarm_ws/`` 的那一层）。

    优先用 ``rmctl.paths.workspace_root()``（编排器同源），失败时回退到
    「本文件向上 5 层」（``<repo>/dualarm_ws/src/rmctl/rmctl/regression/``）。
    """
    try:
        from rmctl import paths as _paths

        return os.path.abspath(_paths.workspace_root())
    except Exception:  # noqa: BLE001 - 兜底：按层级推算
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def scenarios_path(repo_root: Optional[str] = None) -> str:
    """场景声明文件路径（``workcell/bench/experience_e1_e4.yaml``）。"""
    root = os.path.abspath(repo_root or default_repo_root())
    return os.path.join(root, SCENARIOS_REL)


def _child_env(repo_root: str,
               base: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    """子进程环境：把四个包根加进 PYTHONPATH，并清掉外部 RM_WS。"""
    env = dict(base if base is not None else os.environ)
    extra = [os.path.join(repo_root, "dualarm_ws", "src", name)
             for name in ("rmctl", "rm_executor", "rm_hw_bridge", "rm_safety")]
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(
        [p for p in extra if os.path.isdir(p)]
        + ([existing] if existing else []))
    # RM_WS 会污染子进程的工作区自动解析（本模块一律显式 --ws）
    env.pop("RM_WS", None)
    return env


@dataclass
class CommandResult:
    """一次子进程调用的结果（**可测**：测试可注入假 runner）。"""

    cmd: List[str] = field(default_factory=list)
    exit_code: Optional[int] = None
    stdout: str = ""
    stderr: str = ""
    duration_s: float = 0.0
    timed_out: bool = False
    error: str = ""

    @property
    def ok(self) -> bool:
        return self.exit_code == 0 and not self.timed_out and not self.error

    def label(self) -> str:
        return " ".join(self.cmd)


def run_command(cmd: Sequence[str], cwd: Optional[str] = None,
                env: Optional[Dict[str, str]] = None,
                timeout_s: float = 600.0) -> CommandResult:
    """跑一条命令并计时（超时 / 启动失败都**不抛裸异常**）。"""
    argv = [str(c) for c in cmd]
    start = time.monotonic()
    try:
        proc = subprocess.run(
            argv,
            cwd=cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        return CommandResult(
            cmd=argv,
            exit_code=None,
            stdout=exc.stdout if isinstance(exc.stdout, str) else "",
            stderr=exc.stderr if isinstance(exc.stderr, str) else "",
            duration_s=time.monotonic() - start,
            timed_out=True,
            error="命令超时（>%.0f s）" % timeout_s,
        )
    except OSError as exc:
        return CommandResult(
            cmd=argv,
            exit_code=None,
            duration_s=time.monotonic() - start,
            error="%s: %s" % (type(exc).__name__, exc),
        )
    return CommandResult(
        cmd=argv,
        exit_code=proc.returncode,
        stdout=proc.stdout or "",
        stderr=proc.stderr or "",
        duration_s=time.monotonic() - start,
    )


# ---------------------------------------------------------------------------
# 1. 场景声明（唯一来源）
# ---------------------------------------------------------------------------
def load_scenarios(repo_root: Optional[str] = None,
                   path: Optional[str] = None) -> Dict[str, Any]:
    """读场景声明 YAML → ``{"scenarios": {id: {...}}}``（并做结构校验）。

    校验失败 → ``ValueError``（绝不静默按 pass 处理）：顶层是映射、含非空
    ``scenarios`` 列表；场景 id 唯一且恰好覆盖 E1–E4；每个指标有唯一 id、
    带 ``mode``（词表内）与 ``unit``；``target`` 为 null 时 ``target_op``
    必须也是 null（不编造目标）；声明表**不得**出现 ``mode=live`` 的指标。
    """
    target = path or scenarios_path(repo_root)
    import yaml  # 纯声明解析（不引入厂商内核）

    with open(target, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError("场景声明顶层不是映射: %r" % (type(data).__name__,))
    raw = data.get("scenarios")
    if not isinstance(raw, (list, tuple)) or not raw:
        raise ValueError("场景声明缺 scenarios 列表")

    out: Dict[str, Any] = {
        "version": data.get("version"),
        "source": str(data.get("source") or ""),
        "design_refs": [str(x) for x in (data.get("design_refs") or [])],
        "path": target,
        "scenarios": {},
    }
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError("第 %d 个场景不是映射: %r" % (index, type(item).__name__))
        sid = str(item.get("id") or "").strip()
        if not sid:
            raise ValueError("第 %d 个场景缺 id" % index)
        if sid in out["scenarios"]:
            raise ValueError("场景 id 重复: %s" % sid)
        metrics_raw = item.get("metrics")
        if not isinstance(metrics_raw, (list, tuple)) or not metrics_raw:
            raise ValueError("场景 %s 缺 metrics 列表" % sid)
        metrics: Dict[str, Dict[str, Any]] = {}
        for mindex, metric in enumerate(metrics_raw):
            if not isinstance(metric, dict):
                raise ValueError("场景 %s 的第 %d 个指标不是映射" % (sid, mindex))
            mid = str(metric.get("id") or "").strip()
            if not mid:
                raise ValueError("场景 %s 的第 %d 个指标缺 id" % (sid, mindex))
            if mid in metrics:
                raise ValueError("场景 %s 的指标 id 重复: %s" % (sid, mid))
            mode = str(metric.get("mode") or "").strip()
            if mode not in METRIC_MODES:
                raise ValueError("指标 %s.%s 的 mode=%r 非法（只允许 %s）"
                                 % (sid, mid, mode, list(METRIC_MODES)))
            if mode == "live":
                raise ValueError(
                    "指标 %s.%s 声明 mode=live：本声明表只承载离线口径与人工"
                    "pending 字段，真机数字不得进表（请另建真机回填表）"
                    % (sid, mid))
            if not str(metric.get("unit") or "").strip():
                raise ValueError("指标 %s.%s 缺 unit" % (sid, mid))
            target_value = metric.get("target")
            target_op = metric.get("target_op")
            if target_value is None and target_op is not None:
                raise ValueError("指标 %s.%s 的 target 为 null 时 target_op 必须"
                                 "为 null（不编造目标值）" % (sid, mid))
            if target_value is not None and target_op not in TARGET_OPS:
                raise ValueError("指标 %s.%s 的 target_op=%r 非法（只允许 %s）"
                                 % (sid, mid, target_op, list(TARGET_OPS)))
            metrics[mid] = dict(metric)
            metrics[mid]["id"] = mid
            metrics[mid]["mode"] = mode
            metrics[mid]["unit"] = str(metric.get("unit"))
            metrics[mid]["target"] = target_value
            metrics[mid]["target_op"] = target_op
            metrics[mid]["name"] = str(metric.get("name") or mid)
            metrics[mid]["source"] = str(metric.get("source") or "")
            metrics[mid]["note"] = str(metric.get("note") or "")
        scenario = dict(item)
        scenario["metrics"] = metrics
        scenario["metric_order"] = [str(m.get("id")) for m in metrics_raw]
        out["scenarios"][sid] = scenario

    missing = [sid for sid in ("E1", "E2", "E3", "E4") if sid not in out["scenarios"]]
    if missing:
        raise ValueError("场景声明缺场景: %s（E1–E4 必须齐全）"
                         % ", ".join(missing))
    return out


# ---------------------------------------------------------------------------
# 3. E1：接入新机型
# ---------------------------------------------------------------------------
def measure_e1(repo_root: str, ws: str, scenarios: Dict[str, Any],
               env: Optional[Dict[str, str]] = None,
               runner: Optional[Callable[..., CommandResult]] = None,
               tmp_root: Optional[str] = None) -> List[Dict[str, Any]]:
    """E1（接入新机型）→ 指标列表（**全部在临时工作区里**）。

    变体装不上（plan 失败）→ 计入「报错次数」，T2FM 记 ``null`` + reason，
    **不伪造** T2FM 数字。
    """
    spec = scenarios["scenarios"]["E1"]
    runner = runner or run_command
    child_env = _child_env(repo_root, env)
    metrics: List[Dict[str, Any]] = []

    def _metric(mid: str, value: Any, status: str = "measured",
                evidence: Optional[Sequence[str]] = None,
                note: str = "") -> None:
        metrics.append(build_metric(spec["metrics"][mid], "E1", value,
                                    status=status, evidence=evidence,
                                    note=note))

    variant_rel = str(spec.get("variant_urdf")
                      or os.path.join("dualarm_ws", "src", "rm_description",
                                      "urdf", "rm_75_official.urdf"))
    variant_src = os.path.join(repo_root, variant_rel)
    src_desc = os.path.join(repo_root, "dualarm_ws", "src", "rm_description")
    workcell_src = os.path.join(repo_root, "workcell")

    if not os.path.isfile(variant_src):
        for mid in spec["metric_order"]:
            _metric(mid, None, status="blocked",
                    evidence=["变体 URDF 缺失: %s" % variant_src],
                    note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
        return metrics

    tmp = tempfile.mkdtemp(prefix="rm_s09_e1_", dir=tmp_root)
    try:
        dst_desc = os.path.join(tmp, "dualarm_ws", "src", "rm_description")
        os.makedirs(os.path.dirname(dst_desc), exist_ok=True)
        shutil.copytree(src_desc, dst_desc,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        shutil.copytree(workcell_src, os.path.join(tmp, "workcell"),
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        # 「新机型」URDF 直接替换（保留原文件不动）
        installed = os.path.join(dst_desc, "urdf", "rm75_clean.urdf")
        shutil.copyfile(variant_src, installed)

        goal = str(spec.get("plan_goal_rad") or "0,0,0,0,0,0,0")
        # clean_urdf.py 的 --report 需要目录已存在
        os.makedirs(os.path.join(tmp, "build"), exist_ok=True)
        steps: List[Tuple[str, List[str]]] = [
            ("prepare", [
                sys.executable,
                os.path.join(dst_desc, "scripts", "clean_urdf.py"),
                "--src", os.path.join(dst_desc, "urdf", "rm_75_official.urdf"),
                "--dst", installed,
                "--report", os.path.join(tmp, "build", "clean_report.json"),
            ]),
            ("init", [sys.executable, "-m", "rmctl", "--ws", tmp, "init"]),
            ("doctor", [sys.executable, "-m", "rmctl", "--ws", tmp, "doctor"]),
            ("plan", [sys.executable, "-m", "rmctl", "--ws", tmp, "plan",
                      "--goal", goal]),
        ]

        results: List[Tuple[str, CommandResult]] = []
        for name, cmd in steps:
            results.append((name, runner(cmd, cwd=tmp, env=child_env,
                                         timeout_s=300.0)))

        chain_ok = all(res.ok for _name, res in results)
        t2fm = sum(res.duration_s for _name, res in results)

        # -- T2FM -------------------------------------------------------
        if chain_ok:
            _metric("t2fm_s", round(t2fm, 3), status="measured",
                    evidence=["prepare→init→doctor→plan 全链路墙钟 %.3f s"
                              % t2fm,
                              "各步: " + ", ".join(
                                  "%s=%.3fs" % (name, res.duration_s)
                                  for name, res in results)])
        else:
            failed = [name for name, res in results if not res.ok]
            _metric("t2fm_s", None, status="blocked",
                    evidence=["链路未跑通（失败步: %s）" % ", ".join(failed)]
                    + ["%s: %s" % (name, (res.error or "退出码 %s" % res.exit_code))
                       for name, res in results if not res.ok],
                    note="变体装不上 / 链路未跑通 → T2FM 记 null + reason，"
                         "**不伪造** T2FM 数字")

        # -- 命令条数 ---------------------------------------------------
        _metric("command_count", len(results), status="measured",
                evidence=["实际执行: " + ", ".join(name for name, _ in results)])

        # -- 报错次数（doctor 需处理项）--------------------------------
        doctor_out = ""
        doctor_res: Optional[CommandResult] = None
        for name, res in results:
            if name == "doctor":
                doctor_out = res.stdout
                doctor_res = res
        parsed = parse_doctor(doctor_out)
        _metric("error_count", parsed["errors"], status="measured",
                evidence=["doctor 检查项 %d 条（非 PASS %d，其中需处理 %d）"
                          % (parsed["checks"], parsed["non_pass"],
                             parsed["errors"])]
                + (["doctor 退出码 %s" % doctor_res.exit_code]
                   if doctor_res else []))
        _metric("doctor_check_count", parsed["checks"], status="measured",
                evidence=["检查项: " + ", ".join(parsed["names"])])

        # -- 需改配置行数 -----------------------------------------------
        limits_yaml = os.path.join(repo_root, "dualarm_ws", "src",
                                   "rm_description", "config",
                                   "joint_limits.yaml")
        urdf_lims = urdf_limits(installed)
        yaml_lims = yaml_limits(limits_yaml)
        diff_lines = 0
        diff_evidence: List[str] = []
        for joint in sorted(set(urdf_lims) | set(yaml_lims)):
            urdf_pair = urdf_lims.get(joint)
            yaml_pair = yaml_lims.get(joint)
            if urdf_pair is None or yaml_pair is None:
                diff_lines += 1
                diff_evidence.append("%s: 一侧缺失（urdf=%s, yaml=%s）"
                                     % (joint, urdf_pair, yaml_pair))
                continue
            if (abs(urdf_pair[0] - yaml_pair[0]) > LIMIT_TOL
                    or abs(urdf_pair[1] - yaml_pair[1]) > LIMIT_TOL):
                diff_lines += 1
                diff_evidence.append(
                    "%s: urdf=(%.6f, %.6f) yaml=(%.6f, %.6f)"
                    % (joint, urdf_pair[0], urdf_pair[1],
                       yaml_pair[0], yaml_pair[1]))
        _metric("config_lines_changed", diff_lines, status="measured",
                evidence=diff_evidence or ["变体限值与 joint_limits.yaml 完全一致"])

        # -- 需改代码行数 -----------------------------------------------
        # 代码改动范围 = 机器人描述包（接入新机型时唯一可能需要改码的地方）
        changed, code_evidence = diff_py_lines(
            os.path.join(repo_root, "dualarm_ws", "src", "rm_description"),
            os.path.join(tmp, "dualarm_ws", "src", "rm_description"))
        _metric("code_lines_changed", changed, status="measured",
                evidence=code_evidence or ["临时工作区与仓库的 .py 文件零差异"])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return metrics


# ---------------------------------------------------------------------------
# 4. E2：交付新工艺
# ---------------------------------------------------------------------------
#: 最小新工艺模板（参照 workcell/tasks/pick_place.yaml 的段式写法）
_MINIMAL_TASK = """\
version: 1
name: pick_place_min
iterations: 1
home_joints_deg: [0, 0, 0, 0, 0, 0, 0]
segments:
  - name: approach_pick
    type: motion
    goal_joints_deg: [0, 20, 30, 0, 0, 0, 0]

  - name: grasp
    type: gripper
    action: close
    speed: 100
    force: 100
    expect: grasp_ok

  - name: home
    type: motion
    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]
"""


def _ws_missing(ws: str) -> List[str]:
    """工作区的**外部产物预检** → 缺失项列表（空 = 齐备）。

    缺外部产物 → 相关指标 ``blocked``（环境没准备好，**不是**回归）。
    """
    missing: List[str] = []
    if not os.path.exists(os.path.join(ws, "build", "model")):
        missing.append("模型 build/model")
    if not os.path.isfile(os.path.join(ws, "workcell", "tasks",
                                       "pick_place.yaml")):
        missing.append("工艺规格 workcell/tasks/pick_place.yaml")
    return missing


def measure_e2(repo_root: str, ws: str, scenarios: Dict[str, Any],
               env: Optional[Dict[str, str]] = None,
               runner: Optional[Callable[..., CommandResult]] = None,
               tmp_root: Optional[str] = None) -> List[Dict[str, Any]]:
    """E2（交付新工艺）→ 指标列表。"""
    spec = scenarios["scenarios"]["E2"]
    runner = runner or run_command
    child_env = _child_env(repo_root, env)
    metrics: List[Dict[str, Any]] = []

    def _metric(mid: str, value: Any, status: str = "measured",
                evidence: Optional[Sequence[str]] = None,
                note: str = "") -> None:
        metrics.append(build_metric(spec["metrics"][mid], "E2", value,
                                    status=status, evidence=evidence,
                                    note=note))

    tmp = tempfile.mkdtemp(prefix="rm_s09_e2_", dir=tmp_root)
    try:
        task_path = os.path.join(tmp, "pick_place_min.yaml")
        with open(task_path, "w", encoding="utf-8") as fh:
            fh.write(_MINIMAL_TASK)
        with open(task_path, "r", encoding="utf-8") as fh:
            template_lines = len([line for line in fh.readlines()
                                  if line.strip()])
        _metric("template_lines", template_lines, status="measured",
                evidence=["新任务 YAML: %s（参照 workcell/tasks/pick_place.yaml）"
                          % os.path.basename(task_path)])

        _metric("command_count", 1, status="measured",
                evidence=["rmctl run pick_place --task-file <新任务> --count 1"
                          "（一条命令）"])

        # -- 首次成功率：离线 rmctl run pick_place 连跑 N 次 --------------
        # 口径：**先跑再判**——测量值由 runner 的真实结果决定；只有当工作区
        # 缺外部产物（build/model 等）导致一次都没跑通时，才记 blocked
        # （null + reason），**不产出 0 冒充真实值**；跑通了就是实测值。
        runs = int(spec.get("first_success_runs") or 3)
        successes = 0
        run_evidence: List[str] = []
        for index in range(runs):
            res = runner(
                [sys.executable, "-m", "rmctl", "--ws", ws, "run", "pick_place",
                 "--count", "1",
                 "--trace", os.path.join(tmp, "pp_%d.jsonl" % index),
                 "--report", os.path.join(tmp, "pp_%d.json" % index)],
                cwd=repo_root, env=child_env, timeout_s=600.0)
            if res.ok:
                successes += 1
            run_evidence.append("第 %d 次: 退出码 %s%s"
                                % (index + 1, res.exit_code,
                                   "（%s）" % res.error if res.error else ""))
        rate = (float(successes) / float(runs)) if runs else None
        missing = _ws_missing(ws)
        if rate is None:
            _metric("first_success_rate", None, status="blocked",
                    evidence=run_evidence + ["连跑次数为 0 → 无样本"],
                    note="无样本 → 该指标留 null（不产出 0 冒充真实值）")
        elif successes > 0:
            _metric("first_success_rate", rate, status="measured",
                    evidence=run_evidence
                    + ["成功 %d / 共 %d 次" % (successes, runs)])
        elif missing:
            _metric("first_success_rate", None, status="blocked",
                    evidence=run_evidence
                    + ["工作区缺外部产物: %s" % "；".join(missing)],
                    note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
        else:
            _metric("first_success_rate", rate, status="measured",
                    evidence=run_evidence
                    + ["成功 0 / 共 %d 次（每次都真失败，非缺数据源）" % runs])

        # -- 新工艺首跑 ---------------------------------------------------
        new_res = runner(
            [sys.executable, "-m", "rmctl", "--ws", ws, "run", "pick_place",
             "--task-file", task_path, "--count", "1",
             "--trace", os.path.join(tmp, "new_task.jsonl"),
             "--report", os.path.join(tmp, "new_task.json")],
            cwd=repo_root, env=child_env, timeout_s=600.0)
        if new_res.ok:
            _metric("new_task_first_run_ok", True, status="measured",
                    evidence=["新工艺首跑退出码 0"])
        elif missing:
            _metric("new_task_first_run_ok", None, status="blocked",
                    evidence=["新工艺首跑退出码 %s%s"
                              % (new_res.exit_code,
                                 "（%s）" % new_res.error if new_res.error else ""),
                              "工作区缺外部产物: %s" % "；".join(missing)],
                    note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
        else:
            _metric("new_task_first_run_ok", False, status="measured",
                    evidence=["新工艺首跑退出码 %s%s"
                              % (new_res.exit_code,
                                 "（%s）" % new_res.error if new_res.error else "")])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return metrics


# ---------------------------------------------------------------------------
# 5. E3：现场故障恢复（从 S06 trace 派生）
# ---------------------------------------------------------------------------
def _trace_paths(ws: str) -> List[str]:
    """收集 S06 离线证据链产出的 trace（主 trace + 故障分支 trace）。"""
    run_dir = os.path.join(ws, "build", "run")
    if not os.path.isdir(run_dir):
        return []
    out: List[str] = []
    main = os.path.join(run_dir, "pick_place_trace_offline.jsonl")
    if os.path.isfile(main):
        out.append(main)
    for name in sorted(os.listdir(run_dir)):
        if name.startswith("pick_place_trace_fault_") and name.endswith(".jsonl"):
            out.append(os.path.join(run_dir, name))
    return out


def _is_failure(event: Dict[str, Any]) -> bool:
    """失败事件判定：severity 非 INFO 或 status 非 ok。"""
    severity = str(event.get("severity") or "INFO").upper()
    status = str(event.get("status") or "ok").lower()
    return severity in _FAIL_SEVERITIES or status != "ok"


def derive_e3_from_traces(paths: Sequence[str]) -> Dict[str, Any]:
    """从 trace JSONL 派生 E3 的机器侧指标（**流式读**，不整文件进内存）。

    * ``diagnosability_rate``：失败事件里带非空 next_action 的比例；
    * ``mttr_proxy_s``：失败事件 → 其后**第一条**带非空 next_action 的事件
      的机器耗时（均值；**机器侧下界**，不是人的恢复时间）；
    * ``restart_required``：出现 FATAL 事件即判需要重启。
    """
    events: List[Dict[str, Any]] = []
    skipped = 0
    for path in paths:
        try:
            with open(path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        event = json.loads(line)
                    except ValueError:
                        skipped += 1
                        continue
                    if isinstance(event, dict):
                        event["_src"] = os.path.basename(path)
                        events.append(event)
        except OSError:
            continue
    events.sort(key=lambda e: float(e.get("ts") or 0.0))

    failures = [e for e in events if _is_failure(e)]
    diagnosable = [e for e in failures
                   if str(e.get("next_action") or "").strip()]
    rate = (float(len(diagnosable)) / float(len(failures))
            if failures else None)

    deltas: List[float] = []
    for index, event in enumerate(events):
        if not _is_failure(event):
            continue
        t0 = float(event.get("ts") or 0.0)
        for later in events[index + 1:]:
            if not str(later.get("next_action") or "").strip():
                continue
            t1 = float(later.get("ts") or 0.0)
            if t1 >= t0:
                deltas.append(t1 - t0)
            break
    proxy = (sum(deltas) / len(deltas)) if deltas else None
    restart = any(str(e.get("severity") or "").upper() == "FATAL"
                  for e in failures)

    return {
        "events": len(events),
        "failures": len(failures),
        "diagnosable": len(diagnosable),
        "skipped_lines": skipped,
        "diagnosability_rate": rate,
        "mttr_proxy_s": proxy,
        "mttr_proxy_max_s": (max(deltas) if deltas else None),
        "restart_required": restart,
        "trace_files": [os.path.basename(p) for p in paths],
    }


#: S06 离线证据链的报告文件名（编排器指定的 --out / 脚本默认名）
S06_REPORT_NAMES = ("s09_segment_process_offline.json",
                     "s06_pick_place_offline.json")


def _s06_report(reports_dir: str) -> Optional[str]:
    """找 S06 离线证据链的报告（编排器指定的 --out 优先）。"""
    for name in S06_REPORT_NAMES:
        path = os.path.join(reports_dir, name)
        if os.path.isfile(path):
            return path
    return None


def measure_e3(repo_root: str, ws: str, scenarios: Dict[str, Any],
               env: Optional[Dict[str, str]] = None,
               runner: Optional[Callable[..., CommandResult]] = None,
               tmp_root: Optional[str] = None,
               reports_dir: Optional[str] = None) -> List[Dict[str, Any]]:
    """E3（现场故障恢复）→ 指标列表。

    数据源 = S06 离线证据链的 trace；trace 缺失且工作区外部产物齐备时，
    **本段自己跑一次**该证据链（``--out`` 路径由调用方指定），再派生指标。
    真 MTTR 永远是 ``null`` + ``pending``（人工回填）。
    """
    spec = scenarios["scenarios"]["E3"]
    runner = runner or run_command
    child_env = _child_env(repo_root, env)
    metrics: List[Dict[str, Any]] = []

    def _metric(mid: str, value: Any, status: str = "measured",
                evidence: Optional[Sequence[str]] = None,
                note: str = "") -> None:
        metrics.append(build_metric(spec["metrics"][mid], "E3", value,
                                    status=status, evidence=evidence,
                                    note=note))

    paths = _trace_paths(ws)
    chain_evidence: List[str] = []
    s06_report = _s06_report(os.path.abspath(
        reports_dir or os.path.join(ws, "build", "reports")))
    if s06_report:
        chain_evidence.append("S06 离线证据链报告: %s" % s06_report)
    missing = _ws_missing(ws)
    if missing and not paths:
        # 外部产物不齐 → 不跑证据链，直接 blocked（环境没准备好）
        for mid in spec["metric_order"]:
            if spec["metrics"][mid].get("mode") == "human_pending":
                _metric(mid, None, status="pending",
                        evidence=["人工字段：本切片未实证，等待真机窗口人工回填"],
                        note="禁止用 proxy / 0 顶替人工 MTTR")
            else:
                _metric(mid, None, status="blocked",
                        evidence=["工作区缺外部产物: %s" % "；".join(missing)],
                        note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
        return metrics
    if not paths:
        # trace 缺失 → 跑一次 S06 离线证据链（--out 由编排器指定）
        out_dir = os.path.abspath(
            reports_dir or os.path.join(ws, "build", "reports"))
        out_path = os.path.join(out_dir, "s09_experience_s06.json")
        try:
            os.makedirs(out_dir, exist_ok=True)
        except OSError:
            pass
        script = os.path.join(repo_root, "scripts",
                              "pick_place_offline_evidence.py")
        res = runner([sys.executable, script, "--out", out_path, "--ws", ws],
                     cwd=repo_root, env=child_env, timeout_s=3600.0)
        chain_evidence.append("S06 证据链退出码 %s%s"
                              % (res.exit_code,
                                 "（%s）" % res.error if res.error else ""))
        chain_evidence.append("S06 报告: %s" % out_path)
        s06_report = out_path
        paths = _trace_paths(ws)

    if not paths:
        for mid in spec["metric_order"]:
            if spec["metrics"][mid].get("mode") == "human_pending":
                _metric(mid, None, status="pending",
                        evidence=["人工字段：本切片未实证，等待真机窗口人工回填"],
                        note="禁止用 proxy / 0 顶替人工 MTTR")
            else:
                _metric(mid, None, status="blocked",
                        evidence=chain_evidence
                        + ["S06 离线证据链未产出 trace（缺数据源）"],
                        note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
        return metrics

    derived = derive_e3_from_traces(paths)
    base_evidence = chain_evidence + [
        "trace 文件: %s" % ", ".join(derived["trace_files"]),
        "事件数 %d（失败事件 %d，其中带 next_action %d；跳过坏行 %d）"
        % (derived["events"], derived["failures"], derived["diagnosable"],
           derived["skipped_lines"]),
    ]

    if derived["diagnosability_rate"] is None:
        _metric("diagnosability_rate", None, status="blocked",
                evidence=base_evidence + ["trace 里没有失败事件 → 无法测可诊断率"],
                note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
        _metric("mttr_proxy_s", None, status="blocked",
                evidence=base_evidence + ["没有失败事件 → 无 MTTR_proxy 样本"],
                note="数据源缺失 → 该指标留 null（不产出 0 冒充真实值）")
    else:
        _metric("diagnosability_rate", derived["diagnosability_rate"],
                status="measured", evidence=base_evidence)
        if derived["mttr_proxy_s"] is None:
            _metric("mttr_proxy_s", None, status="blocked",
                    evidence=base_evidence
                    + ["失败事件之后没有带 next_action 的事件 → 无 MTTR_proxy 样本"],
                    note="无样本 → 该指标留 null（不产出 0 冒充真实值）")
        else:
            _metric("mttr_proxy_s",
                    round(derived["mttr_proxy_s"], 6),
                    status="measured",
                    evidence=base_evidence
                    + ["MTTR_proxy 最大值 %.6f s"
                       % derived["mttr_proxy_max_s"]])

    _metric("restart_required", bool(derived["restart_required"]),
            status="measured",
            evidence=base_evidence
            + ["判定规则：trace 中出现 FATAL 事件即判需要重启"])

    # -- 真 MTTR：永远是 pending（人工回填）----------------------------
    _metric("mttr_human_s", None, status="pending",
            evidence=["人工字段：本切片未实证（真机窗口人工测量后回填）"],
            note="禁止用 proxy / 0 顶替人工 MTTR")
    return metrics


# ---------------------------------------------------------------------------
# 6. E4：日常操作
# ---------------------------------------------------------------------------
def measure_e4(repo_root: str, ws: str, scenarios: Dict[str, Any],
               env: Optional[Dict[str, str]] = None,
               runner: Optional[Callable[..., CommandResult]] = None,
               tmp_root: Optional[str] = None) -> List[Dict[str, Any]]:
    """E4（日常操作）→ 指标列表（命令条数 + 单次操作耗时）。"""
    spec = scenarios["scenarios"]["E4"]
    runner = runner or run_command
    child_env = _child_env(repo_root, env)
    metrics: List[Dict[str, Any]] = []

    def _metric(mid: str, value: Any, status: str = "measured",
                evidence: Optional[Sequence[str]] = None,
                note: str = "") -> None:
        metrics.append(build_metric(spec["metrics"][mid], "E4", value,
                                    status=status, evidence=evidence,
                                    note=note))

    tmp = tempfile.mkdtemp(prefix="rm_s09_e4_", dir=tmp_root)
    try:
        # 口径：**先跑再判**——耗时由 runner 的真实墙钟决定；run 失败
        # （缺外部产物 / 真回归）→ 记 blocked + null，**不产出 0 冒充**。
        trace_path = os.path.join(tmp, "daily_trace.jsonl")
        run_res = runner(
            [sys.executable, "-m", "rmctl", "--ws", ws, "run", "pick_place",
             "--count", "1", "--trace", trace_path],
            cwd=repo_root, env=child_env, timeout_s=600.0)
        if run_res.ok:
            replay_res = runner(
                [sys.executable, "-m", "rmctl", "--ws", ws, "run", "--replay",
                 trace_path],
                cwd=repo_root, env=child_env, timeout_s=300.0)
            replay_note = ("复现一次: `rmctl run --replay %s`（退出码 %s）"
                           % (os.path.basename(trace_path), replay_res.exit_code))
        else:
            replay_note = "run 失败 → 未执行复现命令（剧本仍为两条命令）"
        commands = 2
        _metric("command_count", commands, status="measured",
                evidence=["剧本: `rmctl run pick_place --count 1` + "
                          "`rmctl run --replay <trace>`（两条命令）",
                          "改参数+跑一次: `rmctl run pick_place --count 1`"
                          "（退出码 %s）" % run_res.exit_code,
                          replay_note])
        if run_res.ok:
            _metric("operation_duration_s", round(run_res.duration_s, 3),
                    status="measured",
                    evidence=["run 命令墙钟 %.3f s" % run_res.duration_s])
        else:
            missing = _ws_missing(ws)
            _metric("operation_duration_s", None, status="blocked",
                    evidence=["run 命令失败（退出码 %s%s）"
                              % (run_res.exit_code,
                                 "，%s" % run_res.error if run_res.error else "")]
                    + (["工作区缺外部产物: %s" % "；".join(missing)]
                       if missing else []),
                    note="数据源缺失 / 未跑通 → 该指标留 null（不产出 0 冒充真实值）")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return metrics


def run_experience(repo_root: Optional[str] = None,
                   ws: Optional[str] = None,
                   reports_dir: Optional[str] = None,
                   env: Optional[Dict[str, str]] = None,
                   runner: Optional[Callable[..., CommandResult]] = None,
                   tmp_root: Optional[str] = None) -> Dict[str, Any]:
    """跑「体验基准 E1–E4」段 → 段级结果字典。

    * 场景声明缺失 / 非法 → ``fail``（仓库声明被破坏，契约回归）；
    * 某个场景测量抛异常 → 该场景的指标记 ``blocked``（**不中断整段**）；
    * 目标未达标 → ``fail``（真回归）；数据源缺失 → ``blocked``；
    * 全部达标 → ``pass``。
    """
    repo_root = os.path.abspath(repo_root or default_repo_root())
    ws = os.path.abspath(ws or repo_root)
    reports_dir = os.path.abspath(
        reports_dir or os.path.join(ws, "build", "reports"))

    try:
        scenarios = load_scenarios(repo_root)
    except FileNotFoundError:
        return seg_mod.segment_result(
            "fail",
            reason="场景声明文件缺失: %s" % scenarios_path(repo_root),
            next_action=("确认 workcell/bench/experience_e1_e4.yaml 已随仓库提交"
                         "（E1–E4 的唯一声明处）后重跑"),
            evidence=["缺失路径: %s" % scenarios_path(repo_root)],
            mode=SEGMENT_MODE,
            report_path="",
            id="experience",
        )
    except Exception as exc:  # noqa: BLE001 - 声明非法 → fail（指名原因）
        return seg_mod.segment_result(
            "fail",
            reason="场景声明非法（%s）: %s" % (type(exc).__name__, exc),
            next_action=("修正 workcell/bench/experience_e1_e4.yaml 后重跑"
                         "（每个指标必须带 mode + unit；target 为 null 时"
                         " target_op 必须为 null）"),
            evidence=[repr(exc)],
            mode=SEGMENT_MODE,
            report_path="",
            id="experience",
        )

    runner = runner or run_command
    metrics: List[Dict[str, Any]] = []
    for sid in ("E1", "E2", "E3", "E4"):
        measurer = {"E1": measure_e1, "E2": measure_e2,
                    "E3": measure_e3, "E4": measure_e4}[sid]
        kwargs: Dict[str, Any] = {}
        if sid == "E3":
            kwargs["reports_dir"] = reports_dir
        try:
            metrics.extend(measurer(repo_root, ws, scenarios, env=env,
                                    runner=runner, tmp_root=tmp_root,
                                    **kwargs))
        except Exception as exc:  # noqa: BLE001 - 单场景失败不中断整段
            scenario = scenarios["scenarios"][sid]
            for mid in scenario["metric_order"]:
                metric_spec = scenario["metrics"][mid]
                if metric_spec.get("mode") == "human_pending":
                    metrics.append(build_metric(
                        metric_spec, sid, None, status="pending",
                        evidence=["人工字段：本切片未实证"],
                        note="禁止用 proxy / 0 顶替人工字段"))
                else:
                    metrics.append(build_metric(
                        metric_spec, sid, None, status="blocked",
                        evidence=["场景测量异常（%s）: %s"
                                  % (type(exc).__name__, exc)],
                        note="测量失败 → 该指标留 null（不产出 0 冒充真实值）"))

    # -- 纪律校验（违规 → fail，绝不静默）------------------------------
    try:
        validate_metrics(metrics, scenarios)
    except ValueError as exc:
        return seg_mod.segment_result(
            "fail",
            reason="体验指标纪律校验失败: %s" % exc,
            next_action=("修正 rmctl.regression.experience 的指标构造"
                         "（人工 pending 字段不许有值；离线数字不许冒充 live）"
                         "后重跑"),
            evidence=[str(exc)],
            mode=SEGMENT_MODE,
            report_path="",
            id="experience",
        )

    by_scenario: Dict[str, List[Dict[str, Any]]] = {}
    for metric in metrics:
        by_scenario.setdefault(str(metric.get("scenario")), []).append(metric)

    scenario_payload: List[Dict[str, Any]] = []
    for sid in ("E1", "E2", "E3", "E4"):
        scenario = scenarios["scenarios"][sid]
        rows = by_scenario.get(sid) or []
        order = {mid: index for index, mid
                 in enumerate(scenario.get("metric_order") or [])}
        rows.sort(key=lambda m: order.get(str(m.get("metric")), 99))
        scenario_payload.append({
            "id": sid,
            "name": str(scenario.get("name") or ""),
            "measures": str(scenario.get("measures") or ""),
            "metrics": rows,
        })

    violations = [m for m in metrics if target_met(m) is False]
    blocked = [m for m in metrics if m.get("status") == "blocked"]
    pending = [m for m in metrics if m.get("status") == "pending"]
    measured = [m for m in metrics if m.get("status") == "measured"]

    payload: Dict[str, Any] = {
        "schema_version": 1,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "mode": SEGMENT_MODE,
        "scenarios_rel": SCENARIOS_REL,
        "design_refs": scenarios.get("design_refs") or [],
        "source": scenarios.get("source") or "",
        "scenarios": scenario_payload,
        "metrics": metrics,
        "summary": {
            "measured": len(measured),
            "blocked": len(blocked),
            "pending": len(pending),
            "violations": ["%s=%s（目标 %s %s）"
                           % (m.get("id"), fmt_value(m.get("value")),
                              m.get("target_op"), fmt_value(m.get("target")))
                           for m in violations],
        },
    }

    json_path = write_json(payload, os.path.join(reports_dir, JSON_NAME))
    md_path = write_markdown(payload, os.path.join(reports_dir, MD_NAME))

    evidence = [
        "场景数 %d（E1–E4）" % len(scenario_payload),
        "指标数 %d：实测 %d、blocked %d、pending %d"
        % (len(metrics), len(measured), len(blocked), len(pending)),
        "JSON: %s" % json_path,
        "Markdown: %s" % md_path,
    ]
    for metric in blocked:
        evidence.append("blocked %s: %s"
                        % (metric.get("id"),
                           "; ".join(metric.get("evidence") or [])[:200]))
    for metric in pending:
        evidence.append("pending %s: %s" % (metric.get("id"),
                                            metric.get("note") or "人工回填"))

    if violations:
        verdict = "fail"
        reason = ("E1–E4 有 %d 项实测未达设计 §6.2 目标: %s"
                  % (len(violations), ", ".join(str(m.get("id"))
                                                for m in violations)))
        next_action = ("对照报告里的目标列复核这几项：是回归就修代码，"
                       "是测量口径问题就修 rmctl.regression.experience")
    elif blocked:
        verdict = "blocked"
        reason = ("E1–E4 有 %d 项因数据源缺失未测出（留 null，不产出 0 冒充）"
                  % len(blocked))
        next_action = ("按报告里 blocked 项的 evidence 补齐数据源后重跑"
                       "一键回归")
    else:
        verdict = "pass"
        reason = ("E1–E4 共 %d 项指标全部测出并满足设计 §6.2 目标"
                  "（另有 %d 项人工 pending 字段待真机窗口回填）"
                  % (len(measured), len(pending)))
        next_action = ""

    result = seg_mod.segment_result(
        verdict,
        reason=reason,
        next_action=next_action,
        evidence=evidence,
        mode=SEGMENT_MODE,
        report_path=json_path,
        id="experience",
        json_path=json_path,
        md_path=md_path,
        metrics=metrics,
    )
    try:
        write_json(result, os.path.join(reports_dir, SEGMENT_REPORT_NAME))
    except OSError:
        pass
    return result
