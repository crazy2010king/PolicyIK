"""S11-T6：归档/验证文档与 FK 复测证据工件的一致性守卫。

本测试把三份文档里的 FK 复测结论**钉死**在 ``workcell/baselines/evidence/rm75_s11_fk_recheck.json``
的实测值上，防止下一轮有人手改文档数值、或工件更新后文档悄悄过期。

覆盖的负例：
- 文档把 raw/corrected 残差、residual_kind、一致性数值写错 → 断言失败；
- 文档宣称"未写任何标定偏置"，但 workcell 的 calibration.tcp / base_to_base 已非 null → 断言失败；
- 文档丢掉远端执行环境口径（172.16.119.199）或 residual_kind 判读表 → 断言失败；
- 文档丢掉"同一位形重复采样不能判定偏置性质"这条纪律 → 断言失败。
"""

import json
import pathlib

import pytest

yaml = pytest.importorskip("yaml")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]

#: 正本归档文档（git 跟踪；docs/code_doc/ 下的是外部文档仓库的符号链接副本）
ARCHIVE_DOC = REPO_ROOT / "docs" / "真机基线与实测限值归档说明.md"
#: 真机验证执行情况说明（git 跟踪正本）
LADDER_DOC = REPO_ROOT / "docs" / "RM75-B真机验证执行情况与使用说明.md"
#: S11 真机窗口 FK 复测结案工件
EVIDENCE = REPO_ROOT / "workcell" / "baselines" / "evidence" / "rm75_s11_fk_recheck.json"
#: 标定偏置定义
WORKCELL = REPO_ROOT / "workcell" / "rm_workcell.yaml"

REMOTE_HOST = "172.16.119.199"
RESIDUAL_KIND = "orientation_dependent"


@pytest.fixture(scope="module")
def evidence():
    return json.loads(EVIDENCE.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def archive_text():
    return ARCHIVE_DOC.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def ladder_text():
    return LADDER_DOC.read_text(encoding="utf-8")


def _fmt(value, digits=3):
    return "%.*f" % (digits, value)


def test_evidence_artifact_is_ok_and_has_the_expected_shape(evidence):
    """前置：工件本身是 ok 且带齐本轮实测字段（否则下面的比对无意义）。"""
    assert evidence["schema"] == "rmctl.rm75_s11_fk_recheck/1"
    assert evidence["status"] == "ok"
    fk = evidence["fk_recheck"]
    assert fk["samples"] >= 8
    assert fk["distinct_configs"] >= 3
    assert fk["residual_kind"] == RESIDUAL_KIND
    assert fk["raw"]["max_pos_dev_mm"] > 0.0
    assert fk["corrected"]["max_pos_dev_mm"] > 0.0


def test_archive_doc_reports_residual_kind_and_remote_host(archive_text):
    """正本必须写清 residual_kind 判读口径与远端执行环境。"""
    assert "residual_kind" in archive_text
    assert REMOTE_HOST in archive_text
    assert RESIDUAL_KIND in archive_text
    # 五类判读表齐全
    for kind in ("within_tolerance", "tool_frame_offset", "base_frame_offset",
                 "orientation_dependent", "insufficient_diversity"):
        assert kind in archive_text, "缺 residual_kind 分类：%s" % kind


def test_archive_doc_numbers_match_the_evidence_artifact(archive_text, evidence):
    """文档里的 raw 数值必须与工件逐位一致（不允许手写漂移）。"""
    fk = evidence["fk_recheck"]
    assert _fmt(fk["raw"]["max_pos_dev_mm"]) in archive_text
    assert _fmt(fk["raw"]["mean_pos_dev_mm"]) in archive_text
    assert _fmt(fk["raw"]["max_ori_dev_deg"]) in archive_text
    assert _fmt(fk["consistency_base"]) in archive_text
    assert _fmt(fk["consistency_tip"]) in archive_text
    # corrected 与 raw 相同（未施加任何偏置）
    assert fk["corrected"]["max_pos_dev_mm"] == fk["raw"]["max_pos_dev_mm"]


def test_docs_keep_the_no_fabrication_and_diversity_discipline(archive_text, ladder_text):
    """两条永久纪律必须留在文档里。"""
    for text in (archive_text, ladder_text):
        assert "同一位形重复" in text, "缺『同一位形重复采样不能判定偏置性质』纪律"
    # 不得出现"已标定 TCP / 已写偏置"这类与工件不符的主张
    for text in (archive_text, ladder_text):
        assert "不写" in text and "偏置" in text, "缺『不写任何标定偏置』结论"


def test_ladder_doc_reports_remote_execution_and_supersedes_the_old_verdict(ladder_text):
    """§7/§7.1 必须给出远端执行口径，并标明旧的 75.03 mm 归因已作废。"""
    assert REMOTE_HOST in ladder_text
    assert RESIDUAL_KIND in ladder_text
    assert "rm75_s11_fk_recheck.json" in ladder_text
    assert "75.03" in ladder_text and "已作废" in ladder_text


def test_calibration_stays_null_while_docs_claim_no_offset_written():
    """文档声称"未写任何偏置" → workcell 里两个偏置键必须仍是 null。"""
    data = yaml.safe_load(WORKCELL.read_text(encoding="utf-8"))
    calibration = data["calibration"]
    assert calibration.get("tcp") is None, "calibration.tcp 非 null：文档结论与实际不符"
    assert calibration.get("base_to_base") is None, \
        "calibration.base_to_base 非 null：文档结论与实际不符"
