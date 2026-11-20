"""S10 结案工件 ↔ 文档 ↔ 基线 的**措辞一致性**护栏（可执行）。

为什么测：S10 的真值是**机器可读**的结案工件
``workcell/baselines/evidence/rm75_s10_window_outcome.json``。
下游（S11 基线回填 / S12 未闭合项声明 / 一键回归报告）读的是 **docs** 与
``workcell/baselines/*.yaml``。如果三处措辞不一致，下游就会读到两套结论；
更糟的是——只要 docs/baseline 里出现一句**无证据的**「长跑已达标 / 断流已复现」，
就等于用离线/仿真结论冒充真机结论，这条红线就此被破。

所以这里钉死（全部**只读**，不连真机、不发任何运动指令）：

* docs §6.2 与 §7 都**指向**结案工件路径 ``rm75_s10_window_outcome``；
* docs §6.2 的两条真机工具行如实反映工件的 status（blocked → 写明未执行，
  绝不写实测数字），实测分支则必须写实测数字；
* ``workcell/baselines/network_plan.yaml`` 可被 ``yaml.safe_load`` 解析，
  ``status`` **仍是** ``blocked``（全表口径：右臂 192.168.1.19 未接触），
  ``blocked.reason`` / ``blocked.next_action`` 非空，
  ``blocked.attempted_actions`` 含本次窗口尝试条目；
* docs 与基线中**不残留**任何无证据的「长跑已达标 / 断流已复现」措辞；
* **口径澄清**：「执行层 600 s 连续执行长跑」与「UDP 主动上报 600 s 长跑」
  在 docs 与基线里都被显式区分，不得被下游混为一谈。
"""
from __future__ import annotations

import json
import os

import pytest
import yaml

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))

DOC_PATH = os.path.join(REPO_ROOT, "docs", "RM75-B真机验证执行情况与使用说明.md")
PLAN_PATH = os.path.join(REPO_ROOT, "workcell", "baselines", "network_plan.yaml")
OUTCOME_PATH = os.path.join(REPO_ROOT, "workcell", "baselines", "evidence",
                            "rm75_s10_window_outcome.json")

OUTCOME_TOKEN = "rm75_s10_window_outcome"

#: 无证据的「达标 / 已复现」措辞黑名单（docs 与基线正文都不得出现）。
#: 注意：只收「无证据的达标断言」；「UDP 主动上报 600 s 长跑已完成」是有实测
#: 证据的（2026-11-19，24002 帧 / 24.998 ms），因此**不在**黑名单里。
FORBIDDEN_CLAIMS = (
    "长跑已达标",
    "长跑达标",
    "连续 10 分钟已达标",
    "断流已复现",
    "断流复现成功",
    "已复现断流",
    "600 s 执行层长跑已完成",
    "执行层长跑已达标",
)


def _read(path: str) -> str:
    assert os.path.isfile(path), "缺少文件：%s" % path
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def _section(text: str, start_marker: str, end_marker: str) -> str:
    """截取 ``start_marker`` 与 ``end_marker`` 之间的正文（标题级切片）。"""
    start = text.index(start_marker)
    end = text.index(end_marker, start + len(start_marker))
    return text[start:end]


@pytest.fixture(scope="module")
def doc_text() -> str:
    return _read(DOC_PATH)


@pytest.fixture(scope="module")
def doc_62(doc_text: str) -> str:
    return _section(doc_text, "### 6.2", "### 6.3")


@pytest.fixture(scope="module")
def doc_7(doc_text: str) -> str:
    return _section(doc_text, "## 7. 仍未闭合的项与下一步", "## 8. 附录")


@pytest.fixture(scope="module")
def plan_text() -> str:
    return _read(PLAN_PATH)


@pytest.fixture(scope="module")
def plan(plan_text: str) -> dict:
    data = yaml.safe_load(plan_text)
    assert isinstance(data, dict), "network_plan.yaml 顶层必须是映射"
    return data


@pytest.fixture(scope="module")
def outcome() -> dict:
    data = json.loads(_read(OUTCOME_PATH))
    assert isinstance(data, dict)
    return data


# --------------------------------------------------------- 结案工件存在且自洽
def test_outcome_artifact_exists_and_is_blocked(outcome):
    """本轮工件是 blocked（2026-11-20 窗口门禁不通），不得出现实测数字。"""
    assert outcome["status"] == "blocked"
    assert outcome["mode"] == "blocked"
    assert outcome["long_run"] is None
    assert outcome["stream_loss"] is None
    assert outcome["ladder"] is None
    assert outcome["honesty"].strip()


# ------------------------------------------------------------ docs §6.2 指向工件
def test_docs_section_6_2_references_outcome_artifact(doc_62):
    assert OUTCOME_TOKEN in doc_62, \
        "docs §6.2 必须指向结案工件 %s" % OUTCOME_TOKEN


def test_docs_section_7_references_outcome_artifact(doc_7):
    assert OUTCOME_TOKEN in doc_7, \
        "docs §7 必须指向结案工件 %s" % OUTCOME_TOKEN


def test_docs_6_2_long_run_row_is_blocked_and_has_no_measured_numbers(doc_62):
    row = [line for line in doc_62.splitlines()
           if "scripts/rm75_long_run.py" in line]
    assert row, "docs §6.2 缺少 rm75_long_run.py 行"
    line = row[0]
    assert "blocked" in line
    assert "未执行" in line
    assert OUTCOME_TOKEN in line


def test_docs_6_2_stream_loss_row_is_blocked_and_has_no_measured_numbers(doc_62):
    row = [line for line in doc_62.splitlines()
           if "scripts/rm75_stream_loss_drill.py" in line]
    assert row, "docs §6.2 缺少 rm75_stream_loss_drill.py 行"
    line = row[0]
    assert "blocked" in line
    assert "未执行" in line
    assert OUTCOME_TOKEN in line


def test_docs_6_2_rows_do_not_claim_passing(outcome, doc_62):
    """blocked 结论下，两行都不得出现「达标 / 已复现」式断言。"""
    assert outcome["status"] == "blocked"
    for line in doc_62.splitlines():
        if ("rm75_long_run.py" in line or "rm75_stream_loss_drill.py" in line):
            for phrase in FORBIDDEN_CLAIMS:
                assert phrase not in line, \
                    "docs §6.2 无证据断言：%r" % phrase


def test_docs_7_cc2_row_points_to_next_actions(doc_7):
    row = [line for line in doc_7.splitlines()
           if "CC2" in line and "连续 10 分钟" in line]
    assert row, "docs §7 缺少 CC2 的\"连续 10 分钟\"行"
    line = row[0]
    assert OUTCOME_TOKEN in line
    assert "rm75_window_run.py" in line, "下一步命令必须来自结案工件入口"


# --------------------------------------------------------------- 基线护栏
def test_network_plan_is_parseable(plan):
    assert plan["kind"] == "network_plan"


def test_network_plan_status_is_still_blocked(plan):
    """status 是全表口径：右臂 192.168.1.19 未接触 → 仍是 blocked。"""
    assert plan["status"] == "blocked"
    assert plan["status"] != "measured"
    assert plan["captured_at"] is None


def test_network_plan_blocked_reason_and_next_action_non_empty(plan):
    blocked = plan["blocked"]
    assert blocked["reason"].strip()
    assert blocked["next_action"].strip()


def test_network_plan_attempted_actions_contain_this_window_attempt(plan):
    actions = plan["blocked"]["attempted_actions"]
    assert isinstance(actions, list) and actions
    joined = "\n".join(str(a) for a in actions)
    assert "2026-11-20" in joined
    assert OUTCOME_TOKEN in joined
    assert "ping" in joined
    assert "8080" in joined


def test_network_plan_distinguishes_the_two_kinds_of_600s_long_run(plan):
    """「执行层长跑」与「UDP 主动上报长跑」必须被显式区分。"""
    joined = "\n".join(str(a) for a in plan["blocked"]["attempted_actions"])
    joined += "\n" + plan["blocked"]["next_action"]
    assert "执行层 600 s 连续执行长跑" in joined
    assert "UDP 主动上报 600 s 长跑" in joined
    assert "未执行" in joined


def test_network_plan_does_not_claim_passing(plan_text):
    for phrase in FORBIDDEN_CLAIMS:
        assert phrase not in plan_text, \
            "network_plan.yaml 无证据断言：%r" % phrase


# --------------------------------------------------------- 全文无残留断言
def test_docs_has_no_unevidenced_passing_claims(doc_text):
    for phrase in FORBIDDEN_CLAIMS:
        assert phrase not in doc_text, \
            "docs 无证据断言：%r" % phrase


def test_docs_distinguishes_the_two_kinds_of_600s_long_run(doc_text):
    assert "执行层" in doc_text
    assert "UDP 主动上报 600 s 长跑" in doc_text or \
        "UDP 主动上报" in doc_text


def test_docs_honesty_statement_present(doc_text):
    """docs 必须携带诚实声明（不得以离线/仿真替代真机结论）。"""
    assert "未以离线" in doc_text or "离线/仿真结果" in doc_text


# ------------------------------------------------------------- 负例（护栏自检）
def test_forbidden_claims_list_is_not_empty():
    assert len(FORBIDDEN_CLAIMS) >= 5


def test_guard_detects_injected_claim(doc_62):
    """护栏自检：把一句假结论注入 §6.2，断言确实能被判定出来。"""
    injected = doc_62 + "\n| scripts/rm75_long_run.py | 长跑已达标 | 601 s |\n"
    assert any(phrase in injected for phrase in FORBIDDEN_CLAIMS)


def test_guard_detects_injected_plan_claim(plan_text):
    injected = plan_text + "\n  # 长跑已达标\n"
    assert any(phrase in injected for phrase in FORBIDDEN_CLAIMS)
