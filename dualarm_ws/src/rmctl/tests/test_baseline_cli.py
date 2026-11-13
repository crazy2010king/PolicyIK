"""rmctl baseline 子命令 + doctor 基线检查项的测试（T3）。

覆盖：
  - `baseline check` 齐备时退出码 0；缺文件 / 字段损坏 / 接缝漂移时退出码 3 并给修复命令
  - `baseline list` / `show` / `scaffold` 行为（scaffold 产物 status=pending 且自身合法）
  - `doctor` 输出包含 baselines:present 与 limits:measured 两项
  - WORKCELL_TEMPLATE 与仓库 workcell/rm_workcell.yaml 逐字一致（防模板漂移）
"""
import os
import shutil

import pytest
import yaml

from rmctl import baseline as bl
from rmctl import cli


def _run(capsys, argv):
    """跑一次 CLI，返回 (exit_code, stdout)。"""
    code = cli.main(argv)
    out = capsys.readouterr().out
    return code, out


def _workcell_yaml_in(ws, repo_root):
    """把仓库里的 workcell/rm_workcell.yaml 拷到临时工作区（git 跟踪文件）。"""
    dst = os.path.join(ws, "workcell", "rm_workcell.yaml")
    shutil.copyfile(os.path.join(repo_root, "workcell", "rm_workcell.yaml"), dst)
    return dst


# ---------------------------------------------------------------- baseline check
def test_check_exit_zero_when_baselines_present(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "check"])
    assert code == 0, out
    for kind in bl.KINDS:
        assert kind in out
    assert "基线检查:" in out


def test_check_is_default_action(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "baseline"])
    assert code == 0, out
    assert "基线检查:" in out


def test_check_missing_file_exits_3_with_fix_command(capsys, tmp_workspace, repo_root):
    """负例 1：基线缺失 → 退出码 3，且输出给出可粘贴的修复命令。"""
    _workcell_yaml_in(tmp_workspace, repo_root)
    os.remove(os.path.join(tmp_workspace, "workcell", "baselines", "zero_baseline.yaml"))
    code, out = _run(capsys, ["--ws", tmp_workspace, "baseline", "check"])
    assert code == 3, out
    assert "zero_baseline" in out
    assert "python3 -m rmctl baseline scaffold" in out
    assert "文件缺失" in out


def test_check_corrupted_field_exits_3(capsys, tmp_workspace, repo_root):
    """负例 2：字段损坏（缺必填字段）→ 退出码 3，报错文本指向具体字段。"""
    _workcell_yaml_in(tmp_workspace, repo_root)
    path = os.path.join(tmp_workspace, "workcell", "baselines", "measured_limits.yaml")
    data = yaml.safe_load(open(path, encoding="utf-8"))
    del data["tcp_repeatability"]
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    code, out = _run(capsys, ["--ws", tmp_workspace, "baseline", "check"])
    assert code == 3, out
    assert "tcp_repeatability" in out
    assert "missing_field" in out


def test_check_detects_workcell_seam_drift(capsys, tmp_workspace, repo_root):
    """负例 3：workcell 声明路径与实际基线文件漂移 → 退出码 3。"""
    yml = _workcell_yaml_in(tmp_workspace, repo_root)
    data = yaml.safe_load(open(yml, encoding="utf-8"))
    data["calibration"]["network_plan"] = "baselines/network_plan_v2.yaml"
    with open(yml, "w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    code, out = _run(capsys, ["--ws", tmp_workspace, "baseline", "check"])
    assert code == 3, out
    assert "workcell 接缝" in out
    assert "network_plan" in out


def test_check_pending_does_not_block(capsys, repo_root):
    """pending 态合法：不阻塞（退出码 0），但必须显式提示未实测。"""
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "check"])
    assert code == 0, out
    assert "BLOCKED" in out
    assert "阻塞" in out


def test_check_blocked_does_not_block(capsys, repo_root):
    """blocked 态（真机窗口不可用）合法：退出码 0，且必须打印阻塞原因。"""
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "check"])
    assert code == 0, out
    assert "基线被阻塞" in out
    for kind in bl.KINDS:
        assert kind in out


def test_check_blocked_reports_reason(capsys, repo_root):
    """blocked 条目必须在输出里带出阻塞原因（可见性，不是静默）。"""
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "check"])
    assert code == 0, out
    assert "原因=" in out
    assert "192.168.1.18" in out


# ---------------------------------------------------------------- list / show
def test_list_shows_four_kinds_with_status(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "list"])
    assert code == 0, out
    for kind in bl.KINDS:
        assert kind in out
    assert "status" in out


def test_show_prints_single_baseline(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "show", "network_plan"])
    assert code == 0, out
    assert "network_plan" in out
    assert "192.168.1.18" in out


def test_show_unknown_kind_exits_2(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "show", "nope"])
    assert code == 2, out
    assert "未知基线类型" in out


def test_show_without_kind_exits_2(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "baseline", "show"])
    assert code == 2, out
    assert "show 需要基线类型" in out


def test_show_missing_file_exits_3(capsys, tmp_workspace):
    """负例：show 一份不存在的基线 → 退出码 3 并给出修复命令。"""
    os.remove(os.path.join(tmp_workspace, "workcell", "baselines", "zero_baseline.yaml"))
    code, out = _run(capsys, ["--ws", tmp_workspace, "baseline", "show", "zero_baseline"])
    assert code == 3, out
    assert "baseline scaffold" in out


# ---------------------------------------------------------------- scaffold
def test_scaffold_creates_pending_placeholders(capsys, tmp_path):
    ws = str(tmp_path / "ws")
    os.makedirs(os.path.join(ws, "workcell", "baselines"))
    code, out = _run(capsys, ["--ws", ws, "baseline", "scaffold"])
    assert code == 0, out
    for kind in bl.KINDS:
        path = os.path.join(ws, "workcell", "baselines", "%s.yaml" % kind)
        assert os.path.exists(path)
        data = yaml.safe_load(open(path, encoding="utf-8"))
        assert data["status"] == "pending"
        assert data["kind"] == kind
    assert "status=pending" in out


def test_scaffold_output_passes_schema_validation(capsys, tmp_path):
    """scaffold 生成的占位文件本身必须是合法基线（无 error 级 issue）。"""
    ws = str(tmp_path / "ws")
    os.makedirs(os.path.join(ws, "workcell", "baselines"))
    code, _ = _run(capsys, ["--ws", ws, "baseline", "scaffold"])
    assert code == 0
    report = bl.validate_all(ws)
    assert report["missing"] == []
    assert report["error_count"] == 0, report["issues"]


def test_scaffold_keeps_existing_files_unless_force(capsys, tmp_workspace):
    path = os.path.join(tmp_workspace, "workcell", "baselines", "zero_baseline.yaml")
    before = open(path, encoding="utf-8").read()
    code, out = _run(capsys, ["--ws", tmp_workspace, "baseline", "scaffold"])
    assert code == 0, out
    assert "已存在" in out
    assert open(path, encoding="utf-8").read() == before
    code, out = _run(capsys, ["--ws", tmp_workspace, "baseline", "scaffold", "--force"])
    assert code == 0, out
    assert "新建" in out


# ---------------------------------------------------------------- doctor
def test_doctor_contains_baseline_checks(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "doctor"])
    assert "baselines:present" in out
    assert "limits:measured" in out
    assert code in (0, 1)  # 取决于本机是否有厂商库，不影响检查项存在性


def test_doctor_limits_measured_is_info_not_blocking(capsys, repo_root):
    """limits:measured 未实测时是 INFO 级，不应把医生结果变成阻塞。"""
    code, out = _run(capsys, ["--ws", repo_root, "doctor"])
    line = [ln for ln in out.splitlines() if "limits:measured" in ln]
    assert line, out
    assert "[INFO]" in line[0] or "[PASS]" in line[0]


def test_doctor_limits_measured_reports_blocked_status(capsys, repo_root):
    """基线为 blocked 时，doctor 必须如实显示 status=blocked。"""
    code, out = _run(capsys, ["--ws", repo_root, "doctor"])
    line = [ln for ln in out.splitlines() if "limits:measured" in ln]
    assert line, out
    assert "status=blocked" in line[0]


def test_doctor_check_count_increased_by_two(capsys, repo_root):
    """doctor 的检查项数：**稳健断言**（≥11 条 + 关键项在列）。

    早期版本断言"恰好 11 条"，但检查项总数会随本切片（S02 新增 4 条 hw:*）
    与本机环境（是否加载到厂商库 → model:load/fk/collision 是否有）漂移，
    脆断。改为：非占位检查项 ≥ 11，且基线两项与 4 条 hw:* 都必须在列。
    """
    _, out = _run(capsys, ["--ws", repo_root, "doctor"])
    names = []
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("[") and "]" in s:
            rest = s.split("]", 1)[1].strip()
            if rest:
                names.append(rest.split()[0])
    assert "baselines:present" in names
    assert "limits:measured" in names
    for hw in ("hw:sdk", "hw:symbols", "hw:contract", "hw:limits_source"):
        assert hw in names, (hw, names)
    checked = [n for n in names if n != "hardware:connected"]
    assert len(checked) >= 11, checked


def test_doctor_baselines_present_reports_fix_command(capsys, tmp_workspace, repo_root):
    _workcell_yaml_in(tmp_workspace, repo_root)
    os.remove(os.path.join(tmp_workspace, "workcell", "baselines", "network_plan.yaml"))
    code, out = _run(capsys, ["--ws", tmp_workspace, "doctor"])
    assert "baselines:present" in out
    assert "python3 -m rmctl baseline scaffold" in out


# ---------------------------------------------------------------- workcell 接缝
def test_workcell_template_matches_repo_file(repo_root):
    """模板与仓库文件必须逐字一致（防模板漂移）。"""
    repo_yml = os.path.join(repo_root, "workcell", "rm_workcell.yaml")
    with open(repo_yml, encoding="utf-8") as fh:
        assert cli.WORKCELL_TEMPLATE == fh.read()


def test_workcell_declares_baseline_paths(repo_root):
    with open(os.path.join(repo_root, "workcell", "rm_workcell.yaml"), encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert data["robot"]["baselines_dir"] == "baselines"
    calibration = data["calibration"]
    assert calibration["tcp"] is None
    assert calibration["base_to_base"] is None
    for kind in bl.KINDS:
        assert calibration[kind] == "baselines/%s.yaml" % kind
        assert os.path.exists(os.path.join(repo_root, "workcell", calibration[kind]))


def test_workcell_seam_check_passes_for_repo(repo_root):
    issues, notes = cli._workcell_seam(repo_root)
    assert issues == []
    assert notes == []
