"""S12-T1：docs §7「仍未闭合的项与下一步」必须显式声明 R004 / R008 两项缺口。

为什么测：仓库里唯一的未闭合清单是
``docs/RM75-B真机验证执行情况与使用说明.md`` 的 §7 三列表（项 / 现状 / 下一步）。
本切片要求把两条**此前缺席**的未闭合项写进去，否则下游会误以为需求已闭合：

* **R004 电子围栏能力缺口**：厂商 SDK **有**电子围栏接口，但本项目的调用许可清单
  ``dualarm_ws/src/rm_hw_bridge/rm_hw_bridge/contract.yaml`` 里 grep 'fence' 零命中
  → 装载层未绑定 → 当前实现**无法下发电子围栏**；
* **R008 真机版崩溃隔离未实证**：离线层（进程级 SIGKILL + fake 回放）已实证，
  真机层「rm_runtime 执行中 kill -9 rm_planner 后真机运动是否继续」本轮未执行。

所以这里钉死（全部**只读**，不连真机、不发任何运动指令）：

* §7 表格里存在 R004 电子围栏行与 R008 崩溃隔离行；
* 两行「现状」栏含关键锚点（R004 → 「电子围栏」；R008 → 「崩溃隔离」与「未实证」），
  且「现状」是**自洽句子**（含「未闭合 / 保持 active / 归属」三要素），
  可以直接粘进需求 notes 而不依赖表格上下文；
* 两行都有非空「下一步」栏（不得只写一句「未实现」）；
* §7 全文不含**无证据断言**黑名单（长跑已达标 / 断流已复现 / 连续 10 分钟已达标 /
  断流复现成功 / 已复现断流）。

覆盖的负例：注入无证据断言 → 被禁语黑名单判定出来；删掉任一行 → 锚点断言失败。
"""
from __future__ import annotations

import os
import re

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

DOC_PATH = os.path.join(REPO_ROOT, "docs", "RM75-B真机验证执行情况与使用说明.md")

SECTION_7 = "## 7. 仍未闭合的项与下一步"
SECTION_7_1 = "### 7.1"
SECTION_8 = "## 8. 附录"

#: 无证据的「达标 / 已复现」措辞黑名单（与 test_s10_honesty_docs.py 口径一致）。
FORBIDDEN_CLAIMS = (
    "长跑已达标",
    "断流已复现",
    "连续 10 分钟已达标",
    "断流复现成功",
    "已复现断流",
)


def _read(path: str) -> str:
    assert os.path.isfile(path), "缺少文件：%s" % path
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


@pytest.fixture(scope="module")
def doc_text() -> str:
    return _read(DOC_PATH)


@pytest.fixture(scope="module")
def doc_7(doc_text: str) -> str:
    """截取 §7 正文（标题级切片），不含 §7.1 子节之后的 §8 附录。"""
    start = doc_text.index(SECTION_7)
    end = doc_text.index(SECTION_8, start + len(SECTION_7))
    return doc_text[start:end]


def _rows(section: str) -> list:
    """§7 表格正文行（以 '|' 开头、以 '|' 结尾，且不是表头/分隔行）。"""
    out = []
    for line in section.splitlines():
        stripped = line.strip()
        if not stripped.startswith("|") or not stripped.endswith("|"):
            continue
        if set(stripped.replace("|", "").replace(" ", "")) <= {"-", ":"}:
            continue
        # 表格单元里可能出现转义竖线（如 `ss -ulnp \| grep`），只在**未转义**的
        # 竖线处切列，否则该行会被误切成多列。
        cells = [cell.strip()
                 for cell in re.split(r"(?<!\\)\|", stripped.strip("|"))]
        if cells and cells[0] in ("项",):
            continue
        out.append(cells)
    return out


@pytest.fixture(scope="module")
def rows(doc_7: str) -> list:
    """§7 **主表**的正文行（§7.1 子节里的两列表不属于本清单，故排除）。"""
    start = doc_7.index(SECTION_7)
    end = doc_7.index(SECTION_7_1, start + len(SECTION_7))
    return _rows(doc_7[start:end])


def _find_row(rows: list, needle: str):
    for cells in rows:
        if needle in cells[0]:
            return cells
    return None


# ------------------------------------------------------------- 行存在性
def test_section_7_table_is_parsable(rows):
    """§7 必须是可解析的三列表（项 / 现状 / 下一步）。"""
    assert rows, "§7 未解析到任何表格行"
    for cells in rows:
        assert len(cells) == 3, "§7 行不是三列：%r" % (cells,)


def test_section_7_has_r004_electronic_fence_row(rows):
    """R004 电子围栏能力缺口必须在 §7 有独立一行。"""
    row = _find_row(rows, "R004")
    assert row is not None, "§7 缺少 R004 行"
    assert "电子围栏" in row[0], "R004 行必须点名『电子围栏』：%r" % (row[0],)


def test_section_7_has_r008_crash_isolation_row(rows):
    """R008 真机版崩溃隔离未实证必须在 §7 有独立一行。"""
    row = _find_row(rows, "R008")
    assert row is not None, "§7 缺少 R008 行"
    assert "崩溃隔离" in row[0], "R008 行必须点名『崩溃隔离』：%r" % (row[0],)


# ------------------------------------------------------- 现状栏的锚点与自洽
def test_r004_row_status_mentions_gap_and_active_ownership(rows):
    """R004 现状栏：自洽句子（未闭合 / 保持 active / 归属），可直接粘进需求 notes。"""
    row = _find_row(rows, "R004")
    status = row[1]
    assert "电子围栏" in status
    assert "未闭合" in status
    assert "保持 active" in status
    assert "归属" in status
    # 能力缺口的根因必须点明（厂商有接口 / 白名单未绑）
    assert "contract.yaml" in status or "白名单" in status


def test_r008_row_status_mentions_not_evidenced_on_real_arm(rows):
    """R008 现状栏：离线已实证 + 真机未实证 + 归属，三要素齐备。"""
    row = _find_row(rows, "R008")
    status = row[1]
    assert "崩溃隔离" in status
    assert "未实证" in status
    assert "未闭合" in status
    assert "保持 active" in status
    assert "归属" in status
    # 真机口径必须点名被杀的进程与被观察的现象
    assert "kill -9" in status
    assert "rm_planner" in status


# --------------------------------------------------------------- 下一步非空
def test_r004_row_next_action_is_actionable(rows):
    row = _find_row(rows, "R004")
    nxt = row[2]
    assert nxt.strip(), "R004 行的『下一步』不得为空"
    assert "M002-o62zc6" in nxt, "R004 必须写明归属里程碑"
    assert "contract.yaml" in nxt, "R004 必须给出后续绑定入口路径"


def test_r008_row_next_action_is_actionable(rows):
    row = _find_row(rows, "R008")
    nxt = row[2]
    assert nxt.strip(), "R008 行的『下一步』不得为空"
    assert "kill -9" in nxt, "R008 必须给出真机窗口的具体动作"
    assert "回填" in nxt, "R008 必须写明回填位置"


# --------------------------------------------------------------- 禁语黑名单
def test_section_7_has_no_unevidenced_passing_claims(doc_7):
    for phrase in FORBIDDEN_CLAIMS:
        assert phrase not in doc_7, "docs §7 无证据断言：%r" % phrase


def test_forbidden_claims_list_is_not_empty():
    assert len(FORBIDDEN_CLAIMS) >= 5


# ------------------------------------------------------------- 负例（护栏自检）
def test_guard_detects_injected_claim(doc_7):
    """护栏自检：把一句假结论注入 §7，断言确实能被判定出来。"""
    injected = doc_7 + "\n| 注入行 | 长跑已达标 | 断流已复现 |\n"
    assert any(phrase in injected for phrase in FORBIDDEN_CLAIMS)


def test_row_lookup_is_precise(rows):
    """行查找必须按『项』列命中，而不是靠整行子串误配。"""
    assert _find_row(rows, "R004")[0].startswith("**R004")
    assert _find_row(rows, "R008")[0].startswith("**R008")
    assert _find_row(rows, "不存在的需求号") is None
