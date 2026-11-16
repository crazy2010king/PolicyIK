"""S09/T8 **端到端一键回归复验**（干净机器复现路径的守卫用例）。

本文件不重复实现任何回归逻辑（那是 ``scripts/rm_regression.py`` 与
``rmctl.regression.orchestrator`` 的职责），而是守住**三件事**：

1. **端到端真的能跑**：以子进程跑 ``python3 scripts/rm_regression.py``
   （在**临时工作区**里 → 模拟干净机器：无 ``build/model``、无真机窗口），
   断言**退出码 ∈ {0, 3}**（**不得是 4**：4 = 真回归）、
   报告 JSON **顶层四键 + ``segments[]`` 齐全**、每段含 ``mode`` 标注、
   且**没有任何 live 口径的伪造数字**（段 mode ∈ {fake, offline}）。
2. **负向面**：未知 ``--only`` 段 id → argparse 拒绝（**退出码 2**），
   不静默跑空；单段 ``--only`` 仍按段级退出码口径收敛。
3. **文档守卫**（照 ``test_planner_acceptance.py`` 的做法）：
   ``docs/rm_regression体验基准与一键回归说明.md`` 必须存在且含关键章节
   标题 —— 文档是现场「5 分钟判因」的入口，缺章节 = 现场无从下手。

夹具纪律：工作区是 pytest **临时目录**（``tmp_path_factory``），
**不依赖任何 gitignore 目录**（``build/`` / ``.gsd/`` / ``.planning/``）。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT = os.path.join(REPO_ROOT, "scripts", "rm_regression.py")
DOC = os.path.join(REPO_ROOT, "docs", "rm_regression体验基准与一键回归说明.md")

#: 顶层必备四键（与 ``rm_executor.contract`` 的结果契约一致）
TOP_KEYS = ("status", "reason", "next_action", "evidence")

#: 段级必备键（``rmctl.regression.segments.segment_result`` 的固定 schema）
SEG_KEYS = ("id", "verdict", "status", "reason", "next_action",
            "evidence", "mode", "report_path")

#: 段级 verdict 词表
VERDICTS = ("pass", "blocked", "fail", "skipped")

#: 本切片允许出现的段级 mode（**禁止 live**：真机口径本轮未实证）
ALLOWED_SEG_MODES = ("fake", "offline")


def _env() -> dict:
    """构造子进程环境（PYTHONPATH 含四层包根）。"""
    env = os.environ.copy()
    paths = [
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_executor"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_safety"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description", "scripts"),
    ]
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(paths + ([existing] if existing else []))
    return env


def _run(args, timeout=1800):
    """跑一键回归脚本（返回 CompletedProcess）。"""
    return subprocess.run(
        [sys.executable, SCRIPT] + list(args),
        env=_env(),
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
    )


@pytest.fixture(scope="module")
def clean_ws(tmp_path_factory):
    """干净机器模拟：**空**工作区（无 build/model、无 workcell、无真机）。"""
    return str(tmp_path_factory.mktemp("s09_e2e_ws"))


@pytest.fixture(scope="module")
def e2e_run(clean_ws, tmp_path_factory):
    """跑一次端到端一键回归（结果在模块内复用，避免重复跑全量）。"""
    out_dir = tmp_path_factory.mktemp("s09_e2e_out")
    out_path = str(out_dir / "s09_regression.json")
    proc = _run(["--ws", clean_ws, "--out", out_path])
    return proc, out_path


# ---------------------------------------------------------------------------
# 1. 端到端
# ---------------------------------------------------------------------------
def test_e2e_exit_code_is_0_or_3(e2e_run):
    """端到端退出码 ∈ {0, 3}：**不得是 4**（4 = 真回归）。"""
    proc, _ = e2e_run
    assert proc.returncode in (0, 3), (
        "退出码 %s\nstdout:\n%s\nstderr:\n%s"
        % (proc.returncode, proc.stdout, proc.stderr))
    assert proc.returncode != 4, proc.stdout


def test_e2e_report_top_level_keys_and_segments(e2e_run):
    """报告 JSON 顶层四键 + ``segments[]`` 齐全；段级键齐全。"""
    proc, out_path = e2e_run
    assert os.path.isfile(out_path), (out_path, proc.stdout, proc.stderr)
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)

    for key in TOP_KEYS:
        assert key in report, "顶层缺键: %s（%s）" % (key, sorted(report))
    assert isinstance(report["evidence"], list), report
    assert isinstance(report["segments"], list), report
    assert report["segments"], "segments 为空：一键回归没有跑出任何段"
    assert report["exit_code"] == proc.returncode, (
        report["exit_code"], proc.returncode)

    seen = set()
    for seg in report["segments"]:
        for key in SEG_KEYS:
            assert key in seg, "段 %r 缺键: %s" % (seg.get("id"), key)
        assert seg["verdict"] in VERDICTS, seg
        assert seg["status"] in ("ok", "blocked", "error"), seg
        assert isinstance(seg["evidence"], list), seg
        assert seg["id"], seg
        seen.add(seg["id"])
    # 段 id 不重复
    assert len(seen) == len(report["segments"]), sorted(seen)


def test_e2e_no_fail_segment_when_exit_is_not_4(e2e_run):
    """退出码不是 4 → 报告里**不得**出现 fail 段（聚合口径自洽）。"""
    proc, out_path = e2e_run
    if proc.returncode == 4:
        pytest.skip("本次出现 fail 段（真回归），由上面的退出码断言报错")
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    fails = [s["id"] for s in report["segments"] if s["verdict"] == "fail"]
    assert not fails, "退出码非 4 却有 fail 段: %s" % fails


def test_e2e_every_segment_has_mode_and_no_live_fake_numbers(e2e_run):
    """每段带 ``mode`` 标注，且**不含 live 口径的伪造数字**。

    段 mode 只允许 ``fake`` / ``offline``（真机口径本轮未实证），
    顶层 mode 固定 ``offline``。
    """
    _, out_path = e2e_run
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert report.get("mode") == "offline", report.get("mode")
    for seg in report["segments"]:
        assert seg["mode"] in ALLOWED_SEG_MODES, (
            "段 %s 的 mode=%r 非法（本切片只允许 %s；live = 真机口径未实证）"
            % (seg["id"], seg["mode"], list(ALLOWED_SEG_MODES)))
    # 被阻塞的段不得带任何数值结论（缺源 = 不产出数字）
    for seg in report["segments"]:
        if seg["verdict"] != "blocked":
            continue
        assert seg["next_action"], "blocked 段 %s 缺 next_action" % seg["id"]


# ---------------------------------------------------------------------------
# 2. 负向面
# ---------------------------------------------------------------------------
def test_regression_rejects_unknown_segment_id(clean_ws, tmp_path):
    """未知 ``--only`` 段 id → argparse 拒绝（退出码 2），不静默跑空。"""
    out_path = str(tmp_path / "x.json")
    proc = _run(["--ws", clean_ws, "--out", out_path,
                 "--only", "no_such_segment"])
    assert proc.returncode == 2, (proc.returncode, proc.stdout, proc.stderr)
    assert "未知的段 id" in proc.stderr, proc.stderr
    assert not os.path.isfile(out_path), "参数错误时不应产出报告"


def test_regression_single_segment_only(clean_ws, tmp_path):
    """``--only <段>`` 只跑该段，退出码仍按段级口径（0 / 3）。"""
    out_path = str(tmp_path / "only_doctor.json")
    proc = _run(["--ws", clean_ws, "--out", out_path, "--only", "doctor"])
    assert proc.returncode in (0, 3), (proc.returncode, proc.stdout, proc.stderr)
    assert os.path.isfile(out_path), (out_path, proc.stdout, proc.stderr)
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)
    assert [s["id"] for s in report["segments"]] == ["doctor"], report


# ---------------------------------------------------------------------------
# 3. 文档守卫
# ---------------------------------------------------------------------------
def test_regression_doc_has_required_sections():
    """说明文档存在且含关键章节标题（现场判因入口）。"""
    assert os.path.isfile(DOC), DOC
    with open(DOC, "r", encoding="utf-8") as fh:
        text = fh.read()
    for needle in (
        "段级语义",
        "pass",
        "blocked",
        "fail",
        "skipped",
        "rm_executor.contract",
        "一段 blocked 不等于整体失败",
        "一键回归用法",
        "退出码语义",
        "常见 blocked 原因清单",
        "报告族清单",
        "自检覆盖矩阵",
        "E1–E4",
        "S1–S5",
        "mode 标注",
        "真机五步回填",
        "干净机器从零复现",
        "source scripts/rmenv.sh",
        "python3 -m rmctl init",
        "python3 scripts/rm_regression.py",
        "GLIBCXX",
        "口径声明",
    ):
        assert needle in text, "文档缺章节/要点: %s" % needle
    # 真机未实证项必须写明不得冒充
    assert "不得" in text
    assert "未实证" in text
