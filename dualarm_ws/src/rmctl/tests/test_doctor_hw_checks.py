"""doctor 的 5 条硬件适配层检查项（hw:*）测试（T7 / S02 + T5 / S05）。

覆盖：
  - 5 条检查项都出现在 doctor 输出里，且字段齐全（name/ok/detail/action/level）；
  - hw:contract 在干净仓库上是 PASS；有黑名单调用时是**阻塞级** FAIL 并给出
    可粘贴的修复命令；
  - SDK 缺失时 hw:sdk / hw:symbols 是 **INFO**（不阻塞退出码）且带可粘贴 action；
  - 限值基线未实测时 hw:limits_source 是 INFO，且 action 指向回填流程。

**不依赖本机是否装了 SDK / 厂商库**：所有"缺失"场景都在临时工作区里构造。
"""
import os
import shutil

import pytest

from rmctl import cli


def _fake_repo_without_baseline(tmp_path, repo_root, monkeypatch):
    """造一个"仓库根"形态的临时目录：有 ``dualarm_ws/src/rm_description`` 与
    ``workcell/``，但**没有** ``measured_limits.yaml``（= 基线未实测）。

    为什么不用 ``tmp_workspace``：``rm_hw_bridge.paths.workspace_root()`` 以
    ``$RM_WS`` 为最高优先级，而临时工作区缺 ``dualarm_ws/src/rm_description``
    标记目录 → 硬件层解析不到"仓库根" → 回退到**仓库本身**的基线，
    "删掉临时副本"根本不起作用（这条负例必须可复现）。故用 ``$RM_WS``
    指向一个真正的假仓库根。
    """
    root = tmp_path / "fake_repo"
    (root / "dualarm_ws" / "src" / "rm_description" / "config").mkdir(parents=True)
    (root / "workcell" / "baselines").mkdir(parents=True)
    shutil.copyfile(
        os.path.join(repo_root, "dualarm_ws", "src", "rm_description", "config",
                     "joint_limits.yaml"),
        str(root / "dualarm_ws" / "src" / "rm_description" / "config" /
            "joint_limits.yaml"))
    monkeypatch.setenv("RM_WS", str(root))
    return str(root)


def _run(capsys, argv):
    code = cli.main(argv)
    out = capsys.readouterr().out
    return code, out


def _line_of(out, name):
    """取 doctor 输出里某条检查项所在行。"""
    for line in out.splitlines():
        if name in line and line.strip().startswith("["):
            return line
    return ""


@pytest.fixture(autouse=True)
def _no_sdk_env(monkeypatch):
    """清掉 RM_API2_LIB：让"缺 SDK"场景可复现（不依赖本机环境）。"""
    monkeypatch.delenv("RM_API2_LIB", raising=False)


# ------------------------------------------------------------- 存在性与字段
def test_doctor_lists_five_hw_checks(capsys, repo_root):
    code, out = _run(capsys, ["--ws", repo_root, "doctor"])
    for name in cli.HW_CHECK_NAMES:
        assert name in out, (name, out)
    assert len(cli.HW_CHECK_NAMES) == 5
    assert code in (0, 3, 4)  # 与本机是否有厂商库 / 真机无关


def test_hw_checks_expose_required_fields(repo_root):
    """每条 hw:* 必须有 name/ok/detail/action/level，且 action 非空（可粘贴）。"""
    checks = cli._hw_checks(repo_root)
    assert [c["name"] for c in checks] == list(cli.HW_CHECK_NAMES)
    for c in checks:
        for key in ("name", "ok", "detail", "action", "level"):
            assert key in c, (c["name"], key)
        assert c["detail"], c["name"]
        assert c["action"], c["name"]
        assert isinstance(c["ok"], bool)


def test_hw_contract_passes_on_clean_repo(capsys, repo_root):
    """干净仓库上 hw:contract 必须是 PASS（零违规）。"""
    _, out = _run(capsys, ["--ws", repo_root, "doctor"])
    line = _line_of(out, "hw:contract")
    assert line, out
    assert "[PASS]" in line, line


# --------------------------------------------------------------- 负例：SDK 缺失
def test_hw_sdk_missing_is_info_with_action(capsys, tmp_workspace):
    """缺 SDK → INFO（不阻塞退出码），且给出可粘贴的 export 命令。"""
    code, out = _run(capsys, ["--ws", tmp_workspace, "doctor"])
    line = _line_of(out, "hw:sdk")
    assert line, out
    assert "[INFO]" in line, line
    assert "RM_API2_LIB=" in out, out
    assert ".deps/rm_api2" in out, out


def test_hw_symbols_missing_sdk_is_info_pointing_to_hw_sdk(capsys, tmp_workspace):
    """缺 SDK → hw:symbols 是 INFO，action 指向 hw:sdk 的修复命令。"""
    _, out = _run(capsys, ["--ws", tmp_workspace, "doctor"])
    line = _line_of(out, "hw:symbols")
    assert line, out
    assert "[INFO]" in line, line
    assert "RM_API2_LIB=" in out, out


def test_hw_symbols_check_reports_missing_sdk(tmp_workspace):
    """单元级：缺 SDK 时 hw:symbols 的 detail 说明原因、level=INFO。"""
    chk = cli._hw_symbols_check(tmp_workspace)
    assert chk["name"] == "hw:symbols"
    assert chk["ok"] is False
    assert chk["level"] == "INFO"
    assert "SDK" in chk["detail"]


# --------------------------------------------------------- 负例：契约违规阻塞
def _write_blacklist_violation(ws):
    src_dir = os.path.join(ws, "dualarm_ws", "src")
    os.makedirs(src_dir, exist_ok=True)
    path = os.path.join(src_dir, "bad_sample.py")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("def go(cfg):\n"
                 "    rm_movep_canfd(cfg)\n"
                 "    return 0\n")
    return path


def test_hw_contract_violation_is_blocking_fail(capsys, tmp_workspace):
    """有黑名单调用 → hw:contract 是 FAIL 且**阻塞**退出码。"""
    _write_blacklist_violation(tmp_workspace)
    code, out = _run(capsys, ["--ws", tmp_workspace, "doctor"])
    line = _line_of(out, "hw:contract")
    assert line, out
    assert "[FAIL]" in line, line
    assert "bad_sample.py" in out, out
    assert "rm_movep_canfd" in out, out
    assert "python3 -m rm_hw_bridge.audit --repo" in out, out
    assert code == 4, out


def test_hw_contract_check_reports_violation_detail(tmp_workspace):
    """单元级：违规 detail 含文件与符号，action 是可粘贴命令。"""
    _write_blacklist_violation(tmp_workspace)
    chk = cli._hw_contract_check(tmp_workspace)
    assert chk["name"] == "hw:contract"
    assert chk["ok"] is False
    assert chk["level"] == "FAIL"
    assert "bad_sample.py" in chk["detail"]
    assert "rm_movep_canfd" in chk["detail"]
    assert "rm_hw_bridge.audit" in chk["action"]


# ------------------------------------------------------- 负例：限值未实测
def test_hw_limits_source_not_measured_is_info(capsys, tmp_path, repo_root, monkeypatch):
    """基线未实测 → hw:limits_source 是 INFO（不阻塞），action 指向回填流程。"""
    ws = _fake_repo_without_baseline(tmp_path, repo_root, monkeypatch)
    _, out = _run(capsys, ["--ws", ws, "doctor"])
    line = _line_of(out, "hw:limits_source")
    assert line, out
    assert "[INFO]" in line, line
    assert "apply_measured_limits.py" in out, out


def test_hw_limits_source_check_status_not_measured(tmp_path, repo_root, monkeypatch):
    """单元级：限值来源不是 measured 时 ok=False、level=INFO、detail 带 status。"""
    ws = _fake_repo_without_baseline(tmp_path, repo_root, monkeypatch)
    chk = cli._hw_limits_source_check(ws)
    assert chk["name"] == "hw:limits_source"
    assert chk["ok"] is False
    assert chk["level"] == "INFO"
    assert "status=" in chk["detail"]
    assert "apply_measured_limits.py" in chk["action"]
