"""S07/T6 一键验收脚本与说明文档的**守卫用例**。

本文件不重复跑三条验收（那是 ``scripts/rm_planner_acceptance.py`` 的职责，
一次全跑约 40 s），而是守住两件事：

1. **脚本真的能跑**：以 ``--only deterministic_same_seed`` 跑一遍
   （最便宜的一段：两次规划 + 两次 worker spawn），断言**退出码 0**、
   报告结构齐全（逐段 ``status`` / ``reason`` / ``assertions`` / ``evidence``
   / ``numbers``）、该段 ``passed``。
2. **文档章节齐全**：``docs/rm_planner规划服务说明.md`` 必须写明
   **D003 的落地偏差**（自研 Task 节点未插入官方 YAML 管线）与
   **离线-真机口径声明**（真机版崩溃隔离本轮未实证）——
   这两段是 S08 不重新踩坑的前提。

负向面：

* 未知验收 id → argparse 拒绝（退出码 2），不静默跑空。

夹具纪律：模型与规划配置都在 **pytest 临时目录**里物化
（``build_model.build_model()`` + 从 git 跟踪的 ``workcell/planner/*.yaml``
复制），**不依赖任何 gitignore 目录**（``build/model`` / ``.gsd/``）。
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
RM_DESC_SRC = os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_description")
RM_DESC_SCRIPTS = os.path.join(RM_DESC_SRC, "scripts")
PLANNER_CFG_DIR = os.path.join(REPO_ROOT, "workcell", "planner")
SCRIPT = os.path.join(REPO_ROOT, "scripts", "rm_planner_acceptance.py")
DOC = os.path.join(REPO_ROOT, "docs", "rm_planner规划服务说明.md")

for _path in (RM_DESC_SCRIPTS,):
    if _path not in sys.path:
        sys.path.insert(0, _path)


@pytest.fixture(scope="module")
def planner_ws(tmp_path_factory):
    """临时工作区：物化好的绝对路径模型 + 复制进来的规划配置。"""
    ws = tmp_path_factory.mktemp("planner_acceptance_ws")
    import build_model  # noqa: PLC0415 - 运行时导入（路径靠 sys.path 注入）

    build_model.build_model(
        ws=str(ws),
        out=str(ws / "build" / "model"),
        pkg=RM_DESC_SRC,
    )
    cfg_dst = ws / "workcell" / "planner"
    cfg_dst.mkdir(parents=True)
    for name in ("pipeline.yaml", "profiles.yaml"):
        shutil.copyfile(os.path.join(PLANNER_CFG_DIR, name),
                        str(cfg_dst / name))
    return str(ws)


def _run(args):
    """跑验收脚本（返回 CompletedProcess）。"""
    env = os.environ.copy()
    paths = [
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rmctl"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_executor"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge"),
        os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_safety"),
        RM_DESC_SCRIPTS,
    ]
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(paths + ([existing] if existing else []))
    return subprocess.run(
        [sys.executable, SCRIPT] + args,
        env=env,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=600,
    )


def test_acceptance_script_single_acceptance_passes(planner_ws, tmp_path):
    """只跑「同请求同种子逐点一致」这一段：退出码 0 + 报告结构齐全。"""
    out_path = tmp_path / "acceptance.json"
    proc = _run([
        "--ws", planner_ws,
        "--out", str(out_path),
        "--only", "deterministic_same_seed",
    ])
    assert proc.returncode == 0, (
        "退出码 %s\nstdout:\n%s\nstderr:\n%s"
        % (proc.returncode, proc.stdout, proc.stderr)
    )
    assert out_path.is_file(), out_path
    with open(out_path, "r", encoding="utf-8") as fh:
        report = json.load(fh)

    assert report["status"] == "ok", report
    assert report["failures"] == [], report
    assert report["selected"] == ["deterministic_same_seed"], report
    section = report["deterministic_same_seed"]
    assert section["status"] == "ok", section
    assert section["passed"] is True, section
    for key in ("status", "reason", "assertions", "evidence", "numbers"):
        assert key in section, section
    assert section["assertions"], section
    assert all(item["passed"] for item in section["assertions"]), section
    numbers = section["numbers"]
    assert numbers["digest"] == numbers["digest_second"], numbers
    assert numbers["points"] > 0, numbers


def test_acceptance_script_rejects_unknown_acceptance_id(tmp_path):
    """未知验收 id → argparse 拒绝（退出码 2），不静默跑空。"""
    proc = _run(["--only", "no_such_acceptance",
                 "--out", str(tmp_path / "x.json")])
    assert proc.returncode == 2, (proc.returncode, proc.stdout, proc.stderr)
    assert "未知的验收 id" in proc.stderr, proc.stderr


def test_doc_has_required_sections():
    """说明文档章节齐全（含 D003 偏差与离线-真机口径声明）。"""
    assert os.path.isfile(DOC), DOC
    with open(DOC, "r", encoding="utf-8") as fh:
        text = fh.read()
    for needle in (
        "进程模型与定位",
        "PlanRequest",
        "PlanResponse",
        "INVALID_REQUEST",
        "cmd 集合",
        "硬超时语义",
        "确定性语义",
        "四道校验",
        "配置来源",
        "D003",
        "D028",
        "离线-真机口径声明",
        "运维",
    ):
        assert needle in text, "文档缺章节/要点: %s" % needle
    # 真机版崩溃隔离必须写明"未实证"
    assert "真机版崩溃隔离" in text
    assert "未实证" in text
