"""S09/T7 真机基线汇总表（``workcell/baselines`` 只读投影）的验收用例。

覆盖（计划要求 + Q5/Q7 负例）：

* 四类基线齐全，**status 与 YAML 逐项一致**（期望值从 YAML 读，**不写死**
  blocked——真机窗口回填后同一断言仍然成立）；
* JSON 与 Markdown 的行数一致；
* 缺某类 YAML → 该类 ``blocked`` + **可粘贴** next_action
  （``python3 -m rmctl baseline scaffold``），不抛裸异常；
* YAML 语法错误 → 该类 ``blocked`` + reason **指名文件与行**；
* YAML 缺 ``status`` 字段 → 该类 ``blocked`` + reason 指名「YAML 缺 status
  字段」；
* **Q7 负例**：把某 YAML 的 ``status`` 改成 ``measured`` → 投影表**随之变化**
  （证明表是投影而非自带真相）；反向断言：YAML 是 blocked 时，投影表里**绝不**
  出现 ``measured``；
* **措辞纪律**：投影自身生成的文案不得把 blocked 描述成「已验证 / measured」
  （YAML 原文以「」引用，不参与措辞判定）；
* **只读**：跑完一段后 ``workcell/baselines/**`` 逐字节不变；
* 顶层 import 不含厂商内核（``tesseract``）——保证无 GLIBCXX 机器可跑。

只使用 git 跟踪的仓库文件 + pytest 临时目录（绝不依赖 .gitignore 目录）。
"""
import ast
import hashlib
import json
import os
import re

import pytest
import yaml

from rmctl import baseline as bl
from rmctl import paths
from rmctl.regression import baselines as blp
from rmctl.regression import orchestrator as orch


# ---------------------------------------------------------------------------
# 夹具 / 工具
# ---------------------------------------------------------------------------
def _yaml_path(ws, kind):
    return os.path.join(paths.baselines_dir(ws), "%s.yaml" % kind)


def _read_yaml(ws, kind):
    with open(_yaml_path(ws, kind), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _write_yaml(ws, kind, data):
    with open(_yaml_path(ws, kind), "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)


def _md_data_rows(md_text):
    """从汇总表 Markdown 里取出**数据行**（表头与分隔行不算）。"""
    rows = []
    for line in md_text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("|"):
            continue
        cells = [c.strip() for c in re.split(r"(?<!\\)\|", stripped.strip("|"))]
        if not cells:
            continue
        if set("".join(cells)) <= set("-: "):
            continue  # 分隔行
        if cells[0] in bl.KINDS:
            rows.append(cells)
    return rows


def _dir_digest(directory):
    """目录内容摘要（用于断言「只读」）。"""
    digest = hashlib.sha256()
    for name in sorted(os.listdir(directory)):
        path = os.path.join(directory, name)
        if not os.path.isfile(path):
            continue
        with open(path, "rb") as fh:
            digest.update(name.encode("utf-8"))
            digest.update(fh.read())
    return digest.hexdigest()


def _expected_verdict(ws):
    """段级期望 verdict —— **从 YAML 读**，不写死 blocked。

    四类基线全部 ``measured`` → ``pass``；其余 → ``blocked``。
    （schema 违规场景由专门用例覆盖，不进这里。）
    """
    statuses = []
    for kind in bl.KINDS:
        path = _yaml_path(ws, kind)
        if not os.path.isfile(path):
            statuses.append("blocked")
            continue
        with open(path, "r", encoding="utf-8") as fh:
            try:
                data = yaml.safe_load(fh)
            except yaml.YAMLError:
                statuses.append("blocked")
                continue
        status = (data or {}).get("status")
        statuses.append(status if status in bl.STATUSES else "blocked")
    return "pass" if all(s == "measured" for s in statuses) else "blocked"


# ---------------------------------------------------------------------------
# 1. 四类齐全 + status 与 YAML 逐项一致
# ---------------------------------------------------------------------------
def test_four_kinds_present_and_status_matches_yaml(repo_root):
    rows, report = blp.collect_rows(repo_root)
    assert [r["kind"] for r in rows] == list(bl.KINDS)
    for row in rows:
        data = _read_yaml(repo_root, row["kind"])
        assert row["yaml_status"] == data.get("status"), row["kind"]
        # 关键：表里的 status 必须等于 YAML 里的 status（表不得自带真相）
        assert row["status"] == data.get("status"), row["kind"]
        assert row["version"] == data.get("version"), row["kind"]
        assert row["captured_at"] == data.get("captured_at"), row["kind"]
    assert report["ok"], report


def test_status_not_hardcoded_but_follows_yaml(tmp_workspace):
    """把某类 YAML 的 status 改成 measured → 投影表随之变化（Q7 负例）。"""
    kind = "version_matrix"
    data = _read_yaml(tmp_workspace, kind)
    data["status"] = "measured"
    data["captured_at"] = "2026-01-01T00:00:00+08:00"
    for field in ("controller_firmware", "teachpendant", "rm_api2_sdk",
                  "ros2_rm_robot_tag", "tesseract_robotics_nanobind",
                  "libstdcxx", "python", "os", "container_image"):
        data[field] = "test-value"
    data.pop("blocked", None)
    _write_yaml(tmp_workspace, kind, data)

    rows, _ = blp.collect_rows(tmp_workspace)
    row = next(r for r in rows if r["kind"] == kind)
    assert row["yaml_status"] == "measured"
    assert row["status"] == "measured"
    # 其余**本用例未改写**的表仍是 blocked（反向断言：不会因为一张表而全体变 measured）
    # 注：measured_limits 自 S13（2026-11-23）起已实测（status=measured），
    #     它不是本用例改写的对象，故不参与这条反向断言。
    for other in rows:
        if other["kind"] in (kind, "measured_limits"):
            continue
        assert other["status"] != "measured", other["kind"]


def test_blocked_yaml_never_projected_as_measured(repo_root):
    """反向断言：YAML 是 blocked 时，投影表里绝不出现 measured。"""
    rows, _ = blp.collect_rows(repo_root)
    for row in rows:
        if row["yaml_status"] == "blocked":
            assert row["status"] == "blocked", row
            assert "measured" not in row["claim"], row
            assert row["status"] != "measured", row


# ---------------------------------------------------------------------------
# 2. 产物：JSON + Markdown 行数一致
# ---------------------------------------------------------------------------
def test_json_and_markdown_row_counts_match(tmp_path, repo_root):
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=repo_root, ws=repo_root,
                               reports_dir=reports_dir)
    json_path = os.path.join(reports_dir, blp.JSON_NAME)
    md_path = os.path.join(reports_dir, blp.MD_NAME)
    assert os.path.isfile(json_path), result
    assert os.path.isfile(md_path), result

    with open(json_path, "r", encoding="utf-8") as fh:
        payload = json.load(fh)
    with open(md_path, "r", encoding="utf-8") as fh:
        md_text = fh.read()
    md_rows = _md_data_rows(md_text)
    assert len(md_rows) == len(payload["rows"]) == len(bl.KINDS)
    assert [r["kind"] for r in payload["rows"]] == list(bl.KINDS)
    assert [r[0] for r in md_rows] == list(bl.KINDS)
    # JSON 与 Markdown 的 status 列逐行一致
    for row, cells in zip(payload["rows"], md_rows):
        assert cells[0] == row["kind"]
        assert cells[2] == row["status"]


def test_segment_verdict_matches_yaml_expectation(tmp_path, repo_root):
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=repo_root, ws=repo_root,
                               reports_dir=reports_dir)
    assert result["verdict"] == _expected_verdict(repo_root), result["reason"]
    assert result["mode"] == blp.MODE
    assert result["report_path"].endswith(blp.JSON_NAME)


# ---------------------------------------------------------------------------
# 3. 结构性问题：缺文件 / YAML 语法错误 / 缺 status 字段
# ---------------------------------------------------------------------------
def test_missing_yaml_is_blocked_with_pasteable_next_action(tmp_path,
                                                            tmp_workspace):
    missing = "network_plan"
    os.remove(_yaml_path(tmp_workspace, missing))
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=tmp_workspace, ws=tmp_workspace,
                               reports_dir=reports_dir)
    assert result["verdict"] == "blocked", result["reason"]
    row = next(r for r in result["rows"] if r["kind"] == missing)
    assert row["status"] == "blocked"
    assert row["present"] is False
    assert "缺失" in row["blocked_reason"]
    assert blp.SCAFFOLD_CMD in row["next_action"]
    # 其它三类不受影响（表仍然齐全）
    assert len(result["rows"]) == len(bl.KINDS)


def test_yaml_syntax_error_names_file_and_line(tmp_path, tmp_workspace):
    kind = "zero_baseline"
    path = _yaml_path(tmp_workspace, kind)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("kind: zero_baseline\nversion: [1, 2\nstatus: blocked\n")
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=tmp_workspace, ws=tmp_workspace,
                               reports_dir=reports_dir)
    assert result["verdict"] == "blocked", result["reason"]
    row = next(r for r in result["rows"] if r["kind"] == kind)
    assert row["status"] == "blocked"
    assert "YAML 语法错误" in row["blocked_reason"]
    assert "zero_baseline.yaml" in row["blocked_reason"]
    assert "行" in row["blocked_reason"]


def test_missing_status_field_is_blocked_and_named(tmp_path, tmp_workspace):
    kind = "measured_limits"
    data = _read_yaml(tmp_workspace, kind)
    data.pop("status", None)
    _write_yaml(tmp_workspace, kind, data)
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=tmp_workspace, ws=tmp_workspace,
                               reports_dir=reports_dir)
    assert result["verdict"] == "blocked", result["reason"]
    row = next(r for r in result["rows"] if r["kind"] == kind)
    assert row["status"] == "blocked"
    assert row["yaml_status"] is None
    assert "YAML 缺 status 字段" in row["blocked_reason"]


def test_schema_violation_is_fail_not_blocked(tmp_path, tmp_workspace):
    """实测值与已生效限值冲突 → schema 违规 → 段级 fail（仓库侧契约破坏）。

    2026-11-23（S13）起 measured_limits 已是 measured（S13 前这里是"blocked 表里
    填了占位数值"）。两种状态下的违规本质相同——**实测值与基线口径不自洽**，
    这里用"基线值高于已生效限值"（limit_above_joint_limits）触发同一个 fail 路径。
    """
    kind = "measured_limits"
    data = _read_yaml(tmp_workspace, kind)
    limits = bl.load_joint_limits(tmp_workspace)
    data["joints"][0]["acceleration"]["value"] = \
        limits["joint1"]["acceleration"] * 1.5  # 高于已生效限值
    _write_yaml(tmp_workspace, kind, data)
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=tmp_workspace, ws=tmp_workspace,
                               reports_dir=reports_dir)
    assert result["verdict"] == "fail", result["reason"]
    assert "measured_limits" in result["reason"]
    assert result["next_action"]


# ---------------------------------------------------------------------------
# 4. 措辞纪律（表不得把 blocked 说成「已验证 / measured」）
# ---------------------------------------------------------------------------
def test_projection_wording_never_claims_verified(tmp_path, repo_root):
    reports_dir = str(tmp_path / "reports")
    blp.run_baselines(repo_root=repo_root, ws=repo_root,
                      reports_dir=reports_dir)
    md_path = os.path.join(reports_dir, blp.MD_NAME)
    with open(md_path, "r", encoding="utf-8") as fh:
        md_text = fh.read()
    own_text = blp.strip_quoted(md_text)
    # 表格数据行里的 status/version/captured_at 是**从 YAML 逐字搬来的数据**
    # （status=measured 是 YAML 的原话，不是本表的宣称）→ 不参与措辞判定；
    # 禁语只看投影自身生成的文案（说明段 + 每行的「结论」列）。
    non_table = "\n".join(line for line in own_text.splitlines()
                          if not line.strip().startswith("|"))
    for phrase in blp.FORBIDDEN_CLAIM_PHRASES:
        assert phrase not in non_table, phrase
    # 表内每行的「结论」列（投影自身措辞）同样不得含禁语
    for cells in _md_data_rows(md_text):
        claim_cell = cells[4]
        status_cell = cells[2]
        if status_cell != "measured":
            for phrase in (blp.FORBIDDEN_CLAIM_PHRASES
                           + blp.FORBIDDEN_STATUS_TOKENS):
                assert phrase not in claim_cell, (cells[0], phrase)


def test_row_claim_wording_is_safe(tmp_path, repo_root):
    reports_dir = str(tmp_path / "reports")
    result = blp.run_baselines(repo_root=repo_root, ws=repo_root,
                               reports_dir=reports_dir)
    for row in result["rows"]:
        claim = str(row.get("claim") or "")
        assert claim, row
        if row.get("status") != "measured":
            for phrase in (blp.FORBIDDEN_CLAIM_PHRASES
                           + blp.FORBIDDEN_STATUS_TOKENS):
                assert phrase not in claim, (row["kind"], phrase)


# ---------------------------------------------------------------------------
# 5. 只读 + 依赖纪律
# ---------------------------------------------------------------------------
def test_segment_does_not_write_workcell(tmp_path, repo_root):
    bdir = paths.baselines_dir(repo_root)
    before = _dir_digest(bdir)
    reports_dir = str(tmp_path / "reports")
    blp.run_baselines(repo_root=repo_root, ws=repo_root,
                      reports_dir=reports_dir)
    assert _dir_digest(bdir) == before


def test_module_does_not_import_vendor_kernel():
    """基线投影只依赖 yaml + numpy（保证无 GLIBCXX 机器可跑）。"""
    src_path = os.path.join(os.path.dirname(os.path.abspath(blp.__file__)),
                            "baselines.py")
    with open(src_path, "r", encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
    assert not any(name.startswith("tesseract") for name in imported), imported


def test_segment_registered_in_orchestrator(repo_root):
    assert "baselines" in orch.segment_ids()
    ctx = orch.build_context(repo_root=repo_root, ws=repo_root)
    result = orch.run_segment("baselines", ctx)
    assert result["id"] == "baselines"
    assert result["verdict"] == _expected_verdict(repo_root), result["reason"]
    assert os.path.isfile(result["report_path"])


def test_orchestrator_only_rejects_unknown_ids(repo_root):
    ctx = orch.build_context(repo_root=repo_root, ws=repo_root)
    with pytest.raises(KeyError):
        orch.run_regression(ctx, only=["baselines_typo"])
