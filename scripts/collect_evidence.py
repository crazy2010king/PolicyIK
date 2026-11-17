#!/usr/bin/env python3
"""证据收集器（S09-T03）——**唯一**的复算证据产出入口。

用途
----
把「一次可复现的验收复跑」变成**声明式检查清单**：每条检查写明

* ``id`` / ``group``（用于 ``--checks`` 选择）
* 一到多个步骤，每个步骤含 **命令 / 期望退出码 / 判定正则 / 超时**
* （可选）额外环境变量（例如 ``PYTHONPATH=src``）

收集器逐项执行并记录 **命令 / 退出码 / 耗时 / 关键输出摘要**，输出两种形态：

* 人类可读报告（默认，stdout）；
* 机器可读 JSON（``--json`` 打印到 stdout，``--output`` 写文件）。

诚实原则
--------
* **任一检查都不得被静默跳过**：命令不存在（环境缺失）标为 ``SKIP`` 并计入汇总，
  绝不伪装成 ``PASS``；正则不匹配 / 退出码不符 / 超时一律标为 ``FAIL``。
* ``SKIP`` 与 ``PASS`` 分开计数，汇总行必须分别打印。
* 只跑**声明式清单**里的命令，不做隐式副作用（不装依赖、不改 git 状态）。

用法
----
::

    python3 scripts/collect_evidence.py --list
    python3 scripts/collect_evidence.py --checks unit,static --json
    python3 scripts/collect_evidence.py --output docs/evidence/s02-rerun.json

退出码
------
``0``  无 FAIL（可能存在 SKIP，汇总行会显式列出）；
``1``  至少一条检查 FAIL；
``2``  参数或环境错误（未知检查 id / 组、解释器不可用、输出路径不可写）。
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

#: 输出摘要最多保留的字符数（超出截断，防止 JSON 膨胀）。
SUMMARY_MAX_CHARS = 6000

#: 关键输出摘录的行数（头部 + 尾部）。
SUMMARY_HEAD_LINES = 15
SUMMARY_TAIL_LINES = 40


# ── 数据模型 ───────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Step:
    """一个检查步骤（一条命令 + 判定契约）。"""

    command: Tuple[str, ...]
    expect_exit: int = 0
    pattern: Optional[str] = None
    #: 附加判定正则（必须**全部**命中才算 PASS）。
    patterns: Tuple[str, ...] = ()
    timeout_s: float = 600.0
    env: Optional[Dict[str, str]] = None
    #: 允许「环境未就绪」的退出码（记为 SKIP，而不是 FAIL）。缺省为空。
    skip_exit_codes: Tuple[int, ...] = ()

    def as_dict(self) -> Dict[str, Any]:
        return {
            "command": list(self.command),
            "expect_exit": self.expect_exit,
            "pattern": self.pattern,
            "patterns": list(self.patterns),
            "timeout_s": self.timeout_s,
            "env": self.env,
            "skip_exit_codes": list(self.skip_exit_codes),
        }


@dataclass(frozen=True)
class Check:
    """一条检查（一个或多个步骤串联；任一步失败 ⇒ 整条 FAIL）。"""

    id: str
    group: str
    description: str
    steps: Tuple[Step, ...]

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "group": self.group,
            "description": self.description,
            "steps": [s.as_dict() for s in self.steps],
        }


@dataclass
class StepResult:
    """步骤执行结果。"""

    command: List[str]
    status: str  # PASS / FAIL / SKIP
    exit_code: Optional[int]
    duration_s: float
    reason: str = ""
    summary: str = ""
    matched: bool = False

    def as_dict(self) -> Dict[str, Any]:
        return {
            "command": self.command,
            "status": self.status,
            "exit_code": self.exit_code,
            "duration_s": round(self.duration_s, 3),
            "reason": self.reason,
            "summary": self.summary,
            "pattern_matched": self.matched,
        }


@dataclass
class CheckResult:
    """检查结果（步骤级细节 + 汇总）。"""

    id: str
    group: str
    description: str
    status: str  # PASS / FAIL / SKIP
    duration_s: float
    steps: List[StepResult] = field(default_factory=list)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "group": self.group,
            "description": self.description,
            "status": self.status,
            "duration_s": round(self.duration_s, 3),
            "steps": [s.as_dict() for s in self.steps],
        }


# ── 检查清单（声明式）────────────────────────────────────────────────
# 说明：清单只描述「跑什么、怎么判」，判定逻辑集中在下方执行器，
# 保证「同一份清单 = 同一份证据」。
def _py(*args: str) -> Step:
    """构造一条 ``python3`` 步骤（解释器用当前 sys.executable）。"""

    return Step(command=(sys.executable,) + args)


def _build_checks() -> List[Check]:
    """构造 S02 全量复跑的检查清单。"""

    checks: List[Check] = []

    # ── 组 unit：全量单测（排除 slow 重负载）─────────────────────────
    checks.append(
        Check(
            id="pytest-not-slow",
            group="unit",
            description="全量单测（pytest -m 'not slow'）：1350 级用例回归",
            steps=(
                Step(
                    command=(sys.executable, "-m", "pytest", "-q", "-m", "not slow"),
                    pattern=r"\d+ passed",
                    timeout_s=3600.0,
                ),
            ),
        )
    )

    # ── 组 env：环境 / 场景自检 ─────────────────────────────────────
    checks.append(
        Check(
            id="verify-env-model",
            group="env",
            description="模型自检（verify_env --model）：DoF / 质量 / 零位末端高度 / 7 条限位",
            steps=(
                _py("tools/verify_env.py", "--model"),
            ),
        )
    )
    checks.append(
        Check(
            id="verify-env-plan",
            group="env",
            description="规划链路自检（verify_env --plan）：场景装配 + IK + 短规划 + 碰撞复检",
            steps=(
                _py("tools/verify_env.py", "--plan"),
            ),
        )
    )

    # ── 组 static：静态扫描 ─────────────────────────────────────────
    checks.append(
        Check(
            id="forbidden-apis-scan",
            group="static",
            description="厂商解算接口禁用名单静态扫描（src tools scripts）：0 命中",
            steps=(
                _py("scripts/scan_forbidden_apis.py", "src", "tools", "scripts"),
            ),
        )
    )

    # ── 组 cli：CLI single → replay（零逃逸）─────────────────────────
    checks.append(
        Check(
            id="cli-single-replay",
            group="cli",
            description="CLI single → replay：录制 + 碰撞复检零逃逸（num_escapes=0）",
            steps=(
                Step(
                    command=(
                        sys.executable,
                        "-m",
                        "rm75_drake.cli",
                        "single",
                        "--scenario",
                        "scenarios/single_arm.yaml",
                        "--target-joints",
                        "0.1",
                        "-0.5",
                        "0.0",
                        "0.8",
                        "0.0",
                        "0.4",
                        "0.0",
                        "--region-spacing",
                        "0.25",
                        "--output",
                        "@RECORDING@",
                        "--json",
                    ),
                    pattern=r'"num_escapes"\s*:\s*0',
                    timeout_s=1800.0,
                    env={"PYTHONPATH": "src"},
                ),
                Step(
                    command=(
                        sys.executable,
                        "-m",
                        "rm75_drake.cli",
                        "replay",
                        "--recording",
                        "@RECORDING@",
                        "--scenario",
                        "scenarios/single_arm.yaml",
                    ),
                    pattern=r"逃逸数：\s*0",
                    timeout_s=900.0,
                    env={"PYTHONPATH": "src"},
                ),
            ),
        )
    )

    # ── 组 cli：双臂紧协调 CLI（S05 已实现，用于对照 S02 旧口径）────
    checks.append(
        Check(
            id="cli-bimanual",
            group="cli",
            description="CLI bimanual：双臂紧协调规划成功 + 14 维零逃逸 + 相对位姿残差 ~1e-8 m",
            steps=(
                Step(
                    command=(
                        sys.executable,
                        "-m",
                        "rm75_drake.cli",
                        "bimanual",
                        "--json",
                    ),
                    patterns=(
                        r'"success"\s*:\s*true',
                        r'"num_escapes"\s*:\s*0',
                        r'"relative_pose_residual_max"\s*:\s*[0-9.eE+-]+',
                    ),
                    timeout_s=1800.0,
                    env={"PYTHONPATH": "src"},
                ),
            ),
        )
    )

    # ── 组 plan：规划指标（有界采样）───────────────────────────────
    checks.append(
        Check(
            id="plan-stats",
            group="plan",
            description="规划指标有界采样（plan_stats --samples 2 --seed 0 --json）：成功率 / P95 / GCS P95",
            steps=(
                Step(
                    command=(
                        sys.executable,
                        "scripts/plan_stats.py",
                        "--samples",
                        "2",
                        "--seed",
                        "0",
                        "--json",
                    ),
                    pattern=r'"success_rate"',
                    timeout_s=1800.0,
                ),
            ),
        )
    )

    # ── 组 ci：CI fast 档 ──────────────────────────────────────────
    checks.append(
        Check(
            id="ci-fast",
            group="ci",
            description="CI fast 档（bash scripts/ci.sh fast）：整档 FAIL=0",
            steps=(
                Step(
                    command=("bash", "scripts/ci.sh", "fast"),
                    pattern=r"FAIL=0",
                    timeout_s=3600.0,
                ),
            ),
        )
    )

    return checks


CHECKS: List[Check] = _build_checks()

#: 收集器自身的检查 id / 组名（用于 ``--checks`` 校验）。
ALL_CHECK_IDS = tuple(c.id for c in CHECKS)
ALL_GROUPS = tuple(sorted({c.group for c in CHECKS}))


# ── 执行器 ───────────────────────────────────────────────────────────
def _summarize(stdout: str, stderr: str, pattern: Optional[str]) -> Tuple[str, bool]:
    """生成关键输出摘要，并报告判定正则是否命中。"""
    lines_out = stdout.splitlines()
    lines_err = stderr.splitlines()
    head = lines_out[:SUMMARY_HEAD_LINES]
    tail = lines_out[-SUMMARY_TAIL_LINES:] if len(lines_out) > SUMMARY_HEAD_LINES else []
    blocks: List[str] = []
    if head:
        blocks.append("--- stdout (head) ---\n" + "\n".join(head))
    if tail:
        blocks.append("--- stdout (tail) ---\n" + "\n".join(tail))
    if lines_err:
        blocks.append("--- stderr (tail) ---\n" + "\n".join(lines_err[-SUMMARY_TAIL_LINES:]))
    text = "\n".join(blocks)
    if len(text) > SUMMARY_MAX_CHARS:
        text = text[:SUMMARY_MAX_CHARS] + "\n...（摘要截断）"
    matched = True
    if pattern:
        matched = re.search(pattern, stdout + "\n" + stderr) is not None
    return text, matched


def _all_patterns_match(
    stdout: str,
    stderr: str,
    pattern: Optional[str],
    patterns: Sequence[str],
) -> bool:
    """全部判定正则都命中才为 True。"""

    blob = stdout + "\n" + stderr
    if pattern and re.search(pattern, blob) is None:
        return False
    for extra in patterns:
        if re.search(extra, blob) is None:
            return False
    return True


def run_step(step: Step, cwd: Path, extra_placeholders: Optional[Dict[str, str]] = None) -> StepResult:
    """执行一个步骤并给出判定结果。

    判定链（顺序固定）：

    1. 可执行文件不存在 → ``SKIP``（环境缺失，不伪装成 PASS/FAIL）；
    2. 超时 → ``FAIL``（reason=timeout）；
    3. 退出码在 ``skip_exit_codes`` 内 → ``SKIP``；
    4. 退出码 ≠ ``expect_exit`` → ``FAIL``（reason=exit_code_mismatch）；
    5. 判定正则未命中 → ``FAIL``（reason=pattern_mismatch）；
    6. 否则 ``PASS``。
    """

    command = list(step.command)
    for key, value in (extra_placeholders or {}).items():
        command = [c.replace(key, value) for c in command]

    exe = command[0]
    resolved = shutil.which(exe)
    if resolved is None:
        return StepResult(
            command=command,
            status="SKIP",
            exit_code=None,
            duration_s=0.0,
            reason=f"command_not_found: {exe}（环境缺失，不计为通过）",
            summary="",
            matched=False,
        )

    env = dict(os.environ)
    if step.env:
        env.update(step.env)

    start = time.monotonic()
    try:
        proc = subprocess.run(
            command,
            cwd=str(cwd),
            env=env,
            capture_output=True,
            text=True,
            timeout=step.timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        duration = time.monotonic() - start
        out = exc.stdout or ""
        err = exc.stderr or ""
        if isinstance(out, bytes):  # pragma: no cover - 防御
            out = out.decode("utf-8", "replace")
        if isinstance(err, bytes):  # pragma: no cover - 防御
            err = err.decode("utf-8", "replace")
        summary, _ = _summarize(out, err, step.pattern)
        matched = _all_patterns_match(out, err, step.pattern, step.patterns)
        return StepResult(
            command=command,
            status="FAIL",
            exit_code=None,
            duration_s=duration,
            reason=f"timeout: 超过 {step.timeout_s}s 未结束",
            summary=summary,
            matched=matched,
        )

    duration = time.monotonic() - start
    summary, _ = _summarize(proc.stdout, proc.stderr, step.pattern)
    matched = _all_patterns_match(proc.stdout, proc.stderr, step.pattern, step.patterns)

    if proc.returncode in step.skip_exit_codes:
        return StepResult(
            command=command,
            status="SKIP",
            exit_code=proc.returncode,
            duration_s=duration,
            reason=f"skip_exit_code: {proc.returncode}（环境未就绪）",
            summary=summary,
            matched=matched,
        )
    if proc.returncode != step.expect_exit:
        return StepResult(
            command=command,
            status="FAIL",
            exit_code=proc.returncode,
            duration_s=duration,
            reason=f"exit_code_mismatch: 期望 {step.expect_exit}，实际 {proc.returncode}",
            summary=summary,
            matched=matched,
        )
    if not matched:
        expected = [p for p in (step.pattern,) + tuple(step.patterns) if p]
        return StepResult(
            command=command,
            status="FAIL",
            exit_code=proc.returncode,
            duration_s=duration,
            reason=f"pattern_mismatch: 未命中判定正则 {expected}",
            summary=summary,
            matched=False,
        )
    return StepResult(
        command=command,
        status="PASS",
        exit_code=proc.returncode,
        duration_s=duration,
        reason="",
        summary=summary,
        matched=True,
    )


def run_check(check: Check, cwd: Path) -> CheckResult:
    """执行一条检查（多步骤串联）。"""

    placeholders: Dict[str, str] = {}
    workdir: Optional[str] = None
    if any("@RECORDING@" in " ".join(s.command) for s in check.steps):
        workdir = tempfile.mkdtemp(prefix="collect-evidence-")
        placeholders["@RECORDING@"] = str(Path(workdir) / "recording.json")

    step_results: List[StepResult] = []
    overall = "PASS"
    start = time.monotonic()
    for step in check.steps:
        result = run_step(step, cwd, placeholders)
        step_results.append(result)
        if result.status == "FAIL":
            overall = "FAIL"
            break  # 失败即停，避免后续步骤产出误导性数字
        if result.status == "SKIP" and overall != "FAIL":
            overall = "SKIP"
    duration = time.monotonic() - start
    return CheckResult(
        id=check.id,
        group=check.group,
        description=check.description,
        status=overall,
        duration_s=duration,
        steps=step_results,
    )


def select_checks(selector: Optional[str]) -> List[Check]:
    """按 ``--checks`` 选择检查（组名或 id，逗号分隔）。"""

    if not selector:
        return list(CHECKS)
    wanted = [s.strip() for s in selector.split(",") if s.strip()]
    unknown = [s for s in wanted if s not in ALL_CHECK_IDS and s not in ALL_GROUPS]
    if unknown:
        raise ValueError(
            f"未知的检查组或 id：{unknown}；可用组：{list(ALL_GROUPS)}；"
            f"可用 id：{list(ALL_CHECK_IDS)}"
        )
    return [c for c in CHECKS if c.id in wanted or c.group in wanted]


# ── 环境信息 ─────────────────────────────────────────────────────────
def environment_info() -> Dict[str, Any]:
    """收集本机环境指纹（用于标注 fallback 档 / 受支持基线）。"""

    info: Dict[str, Any] = {
        "python_version": platform.python_version(),
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "repo_root": str(REPO_ROOT),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    drake_version = None
    try:
        sys.path.insert(0, str(SRC_DIR))
        from rm75_drake.drake_compat import drake_version as _dv  # noqa: PLC0415

        drake_version = _dv()
    except Exception as exc:  # pragma: no cover - 环境异常分支
        info["drake_version_error"] = repr(exc)
    info["drake_version"] = drake_version
    try:
        lock = REPO_ROOT / "versions.lock"
        if lock.exists():
            info["versions_lock_present"] = True
    except OSError:  # pragma: no cover
        info["versions_lock_present"] = False
    info["baseline_note"] = (
        "受支持基线为 Python 3.12 + Drake 1.55.0（versions.lock）；"
        "本机若为 Python 3.10 / Drake 1.51.1，则属于 fallback 档，"
        "结果不得表述为与基线一致。"
    )
    return info


# ── 报告渲染 ─────────────────────────────────────────────────────────
def render_human(results: List[CheckResult], info: Dict[str, Any]) -> str:
    """人类可读报告。"""

    lines: List[str] = []
    lines.append("=== collect_evidence 报告 ===")
    lines.append(
        f"环境：Python {info['python_version']} / Drake {info['drake_version']} "
        f"/ {info['platform']}"
    )
    lines.append(f"时间：{info['timestamp']}")
    lines.append("")
    for res in results:
        lines.append(f"[{res.status}] {res.id} — {res.description}（{res.duration_s:.1f}s）")
        for step in res.steps:
            cmd = " ".join(step.command)
            code = "n/a" if step.exit_code is None else str(step.exit_code)
            lines.append(f"    - {cmd}")
            lines.append(
                f"      退出码={code} 耗时={step.duration_s:.1f}s 状态={step.status}"
                + (f" 原因={step.reason}" if step.reason else "")
            )
    counts = summarize(results)
    lines.append("")
    lines.append(
        "--- 汇总：PASS={passed} SKIP={skipped} FAIL={failed}，共 {total} 条 ---".format(**counts)
    )
    if counts["skipped"]:
        lines.append("注意：存在 SKIP 项（环境未就绪），不得当作通过。")
    return "\n".join(lines)


def summarize(results: List[CheckResult]) -> Dict[str, int]:
    """汇总计数。"""

    return {
        "passed": sum(1 for r in results if r.status == "PASS"),
        "skipped": sum(1 for r in results if r.status == "SKIP"),
        "failed": sum(1 for r in results if r.status == "FAIL"),
        "total": len(results),
    }


def build_document(results: List[CheckResult], info: Dict[str, Any]) -> Dict[str, Any]:
    """机器可读证据文档。"""

    return {
        "schema_version": 1,
        "generator": "scripts/collect_evidence.py",
        "environment": info,
        "summary": summarize(results),
        "checks": [r.as_dict() for r in results],
    }


# ── CLI ──────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="证据收集器：声明式检查清单 → 命令 / 退出码 / 耗时 / 摘要",
    )
    parser.add_argument(
        "--checks",
        default=None,
        help="逗号分隔的检查组或 id（可选组：%s）" % ",".join(ALL_GROUPS),
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 打印到 stdout")
    parser.add_argument("--output", default=None, help="把 JSON 结果写入该文件")
    parser.add_argument("--list", action="store_true", help="列出全部检查 id / 组")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list:
        for check in CHECKS:
            print(f"{check.group}\t{check.id}\t{check.description}")
        return 0

    try:
        checks = select_checks(args.checks)
    except ValueError as exc:
        print(f"collect_evidence: {exc}", file=sys.stderr)
        return 2

    if not checks:
        print("collect_evidence: 没有选中任何检查", file=sys.stderr)
        return 2

    info = environment_info()
    results: List[CheckResult] = []
    for check in checks:
        print(f"[collect] {check.id} …", file=sys.stderr)
        results.append(run_check(check, REPO_ROOT))

    document = build_document(results, info)

    if args.output:
        out_path = Path(args.output)
        try:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(
                json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        except OSError as exc:
            print(f"collect_evidence: 无法写入 {out_path}: {exc}", file=sys.stderr)
            return 2

    if args.json:
        print(json.dumps(document, ensure_ascii=False, indent=2))
    else:
        print(render_human(results, info))

    counts = document["summary"]
    if counts["failed"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
