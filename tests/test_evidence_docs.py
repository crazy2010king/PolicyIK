"""证据文档口径守护用例（S09-T06）。

守护两件事，防止「证据闭合」在后续切片里悄悄退回旧口径：

1. ``docs/evidence/REQUIREMENTS-PROOF.md`` 必须覆盖 R002 / R007 / R011 / R012 / R014 / R019
   六条需求，每条都带「目标 Proof（validation 文本）」、「目标 Notes」与「关键数字」，
   且文件内**不得**再出现旧的未回填占位字面量；
2. ``docs/evidence/README.md`` 索引必须覆盖 ``docs/evidence/`` 下的**全部** ``*.md``，
   且 R019 / R011 两条要点（归档不入 git + 可复现配方；CLI 仅支持单臂）必须在位。

本文件只读取 **git 跟踪的仓库内路径**（``docs/evidence/*.md``），
不读取任何被 ``.gitignore`` 排除的本地目录（``.gsd/`` 等）。
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_DIR = REPO_ROOT / "docs" / "evidence"
PROOF_DOC = EVIDENCE_DIR / "REQUIREMENTS-PROOF.md"
INDEX_DOC = EVIDENCE_DIR / "README.md"

#: 必须被回填覆盖的需求 ID。
REQUIRED_REQUIREMENT_IDS = ("R002", "R007", "R011", "R012", "R014", "R019")

#: 旧的占位字面量：出现即视为口径回退。
FORBIDDEN_PLACEHOLDER = "unmapped"


def _read(path: Path) -> str:
    assert path.exists(), f"证据文档缺失：{path}"
    return path.read_text(encoding="utf-8")


def _section(text: str, requirement_id: str) -> str:
    """截取某个需求的小节正文（从该需求标题到下一个一级需求标题或文末）。"""
    lines = text.splitlines()
    starts = [
        index
        for index, line in enumerate(lines)
        if line.startswith("## ") and requirement_id in line
    ]
    assert starts, f"REQUIREMENTS-PROOF.md 缺少 {requirement_id} 的小节"
    begin = starts[0]
    end = len(lines)
    for index in range(begin + 1, len(lines)):
        line = lines[index]
        if line.startswith("## ") and requirement_id not in line:
            end = index
            break
    return "\n".join(lines[begin:end])


# --------------------------------------------------------------------------
# REQUIREMENTS-PROOF.md
# --------------------------------------------------------------------------
def test_proof_doc_exists_and_covers_six_requirements():
    text = _read(PROOF_DOC)
    for requirement_id in REQUIRED_REQUIREMENT_IDS:
        assert requirement_id in text, f"{requirement_id} 未在 REQUIREMENTS-PROOF.md 中出现"
        section = _section(text, requirement_id)
        assert section.strip(), f"{requirement_id} 的小节为空"


def test_proof_doc_has_no_unfilled_placeholder_literal():
    text = _read(PROOF_DOC)
    assert FORBIDDEN_PLACEHOLDER not in text.lower(), (
        "REQUIREMENTS-PROOF.md 仍残留旧占位字面量（未回填口径），"
        "请改为中文措辞（如「未回填」）"
    )


@pytest.mark.parametrize("requirement_id", REQUIRED_REQUIREMENT_IDS)
def test_each_requirement_section_has_proof_notes_and_numbers(requirement_id):
    section = _section(_read(PROOF_DOC), requirement_id)
    assert "目标 Proof" in section, f"{requirement_id} 缺少「目标 Proof」段"
    assert "目标 Notes" in section, f"{requirement_id} 缺少「目标 Notes」段"
    assert "关键数字" in section, f"{requirement_id} 缺少「关键数字」段"
    assert any(
        anchor in section
        for anchor in ("docs/evidence/", "docs/architecture/", "runbooks/")
    ), f"{requirement_id} 未绑定具体支撑证据文档"


def test_r019_section_states_archive_not_in_git_and_recipe():
    section = _section(_read(PROOF_DOC), "R019")
    assert "不入 git" in section, "R019 条目缺少「归档不入 git」要点"
    assert "可复现配方" in section, "R019 条目缺少「可复现配方」要点"
    assert "fetch_upstream.py --verify" in section, "R019 条目缺少字节级校验命令"


def test_r011_section_states_single_arm_only_gap():
    section = _section(_read(PROOF_DOC), "R011")
    assert "仅支持单臂" in section, "R011 条目缺少「仅支持单臂」要点"
    assert "task_single_arm_only" in section, "R011 条目缺少稳定 reason 标签"
    assert "退出码 2" in section, "R011 条目缺少 CLI 缺口退出码口径"


def test_r002_section_carries_fk_threshold_doctrine():
    section = _section(_read(PROOF_DOC), "R002")
    assert "5e-6" in section, "R002 条目缺少真实 URDF 的 FK 位置门槛"
    assert "2.2407e-6" in section, "R002 条目缺少固有残差量级"
    assert "fk-tolerance.md" in section, "R002 条目未指向 FK 口径文档"


def test_r014_section_states_zero_forbidden_hits():
    section = _section(_read(PROOF_DOC), "R014")
    assert "0 命中" in section, "R014 条目缺少禁用名单 0 命中的结论"
    assert "SdkAudit" in section, "R014 条目缺少运行时 SDK 审计面"
    assert "rm_control" in section, "R014 条目缺少 MoveIt2 / rm_control 零引用口径"


# --------------------------------------------------------------------------
# README.md（证据索引）
# --------------------------------------------------------------------------
def test_index_doc_exists():
    assert INDEX_DOC.exists(), "docs/evidence/README.md 缺失"


def test_index_lists_every_evidence_markdown_file():
    text = _read(INDEX_DOC)
    files = sorted(path.name for path in EVIDENCE_DIR.glob("*.md"))
    assert files, "docs/evidence/ 下没有任何 Markdown 文档"
    for name in files:
        assert name in text, f"证据索引未列出 docs/evidence/{name}"


def test_index_states_fallback_environment_and_recompute_commands():
    text = _read(INDEX_DOC)
    assert "3.10" in text and "1.51.1" in text, "证据索引未写明本机 fallback 档环境"
    assert "fallback" in text, "证据索引未标注 fallback 档口径"
    assert "python3 scripts/collect_evidence.py" in text, "证据索引缺少唯一证据收集器的复算命令"
    assert "python3 tools/fetch_upstream.py --verify" in text, "证据索引缺少归档字节级校验复算命令"


def test_index_has_no_unfilled_placeholder_literal():
    assert FORBIDDEN_PLACEHOLDER not in _read(INDEX_DOC).lower(), (
        "证据索引仍残留旧占位字面量"
    )
