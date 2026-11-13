"""静态审计器测试（``rm_hw_bridge/audit.py``）。

覆盖：
* **负例必须失败**：含 ``rm_algo_forward_kinematics(`` 与 ``import Robotic_Arm``
  的源码 → ``scan_paths`` 必须报违规；
* 黑名单符号的**调用形态**被拦下，但**字符串常量**不误判；
* **正例**：仓库自扫描零违规；硬件层包内无 tesseract / Robotic_Arm / moveit / rmctl import；
* CLI 退出码：0 通过 / 3 存在违规。
"""
from __future__ import annotations

import os

import pytest

from rm_hw_bridge import audit


# ------------------------------------------------------------------ 负例夹具
BAD_SOURCE = '''\
# -*- coding: utf-8 -*-
import os
from Robotic_Arm import RoboticArm


def run():
    os.system("echo hi")
    rm_algo_forward_kinematics(1, 2, 3)
    return rm_set_joint_zero_pos(0)
'''

STRING_ONLY_SOURCE = '''\
# -*- coding: utf-8 -*-
BLACKLIST = ("rm_set_joint_zero_pos",)
PREFIX = "rm_algo_"
NOTE = "rm_movep_canfd("  # 仅字符串常量，不是调用


def run():
    return BLACKLIST
'''

TESSERACT_SOURCE = '''\
import tesseract_robotics.planning
'''

RMCTL_SOURCE = '''\
from rmctl import hwcollect
'''

EXECUTOR_RMCTL_SOURCE = '''\
from rmctl import hwcollect
'''

EXECUTOR_TESSERACT_SOURCE = '''\
import tesseract_robotics.planning
'''

EXECUTOR_HW_BRIDGE_SOURCE = '''\
from rm_hw_bridge import bridge
'''

EXECUTOR_CLEAN_SOURCE = '''\
"""干净的执行层文件（无任何禁止 import）。"""

CONST = 1
'''

# --- 安全层负例（S04/T8）：安全层不得反向依赖体验层 / 执行层 / 规划内核 ---
SAFETY_RM_EXECUTOR_SOURCE = '''\
from rm_executor import executor
'''

SAFETY_RMCTL_SOURCE = '''\
from rmctl import hwcollect
'''

SAFETY_TESSERACT_SOURCE = '''\
import tesseract_robotics.planning
'''

SAFETY_CLEAN_SOURCE = '''\
"""干净的安全层文件（零反向依赖）。"""

from rm_safety import levels

CONST = levels.LEVEL_L1
'''


@pytest.fixture()
def bad_file(tmp_path):
    p = tmp_path / "bad_hw.py"
    p.write_text(BAD_SOURCE, encoding="utf-8")
    return p


@pytest.fixture()
def string_only_file(tmp_path):
    p = tmp_path / "strings_only.py"
    p.write_text(STRING_ONLY_SOURCE, encoding="utf-8")
    return p


# ------------------------------------------------------------------ 负例
def test_negative_blacklist_call_and_forbidden_import(bad_file):
    """负例：黑名单调用 + import Robotic_Arm 必须被报出。"""
    violations = audit.scan_paths([str(bad_file)])
    kinds = {v.kind for v in violations}
    assert audit.KIND_BLACKLIST_CALL in kinds
    assert audit.KIND_FORBIDDEN_IMPORT in kinds
    details = {v.detail for v in violations}
    assert "rm_algo_forward_kinematics" in details
    assert "rm_set_joint_zero_pos" in details
    assert "Robotic_Arm" in details


def test_negative_blacklist_prefix_is_reported(tmp_path):
    p = tmp_path / "algo.py"
    p.write_text("def f():\n    return rm_algo_inverse_kinematics(0)\n", encoding="utf-8")
    violations = audit.scan_paths([str(p)])
    assert any(v.kind == audit.KIND_BLACKLIST_CALL for v in violations)
    assert any(v.detail == "rm_algo_inverse_kinematics" for v in violations)


def test_negative_hardware_layer_tesseract_import(tmp_path):
    """硬件层内 import tesseract → 违规（scope=hardware）。"""
    p = tmp_path / "hw.py"
    p.write_text(TESSERACT_SOURCE, encoding="utf-8")
    # 临时目录不在硬件层包内 → scope=hardware 不判违规
    assert audit.scan_paths([str(p)]) == []
    # 放进硬件层包内（tests/ 下）→ 判违规
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(audit.__file__), ".."))
    target = os.path.join(pkg_root, "tests", "_tmp_audit_fixture.py")
    try:
        with open(target, "w", encoding="utf-8") as fh:
            fh.write(TESSERACT_SOURCE)
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_FORBIDDEN_IMPORT for v in violations)
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_negative_reverse_dependency_rmctl(tmp_path):
    """硬件层 import rmctl → 反向依赖违规。"""
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(audit.__file__), ".."))
    target = os.path.join(pkg_root, "tests", "_tmp_audit_fixture2.py")
    try:
        with open(target, "w", encoding="utf-8") as fh:
            fh.write(RMCTL_SOURCE)
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_REVERSE_DEPENDENCY for v in violations)
    finally:
        if os.path.exists(target):
            os.remove(target)


# ------------------------------------------------------------------ 分层负例
def _executor_fixture(repo_root, name, source):
    """在执行层包目录里放一个临时文件（模拟执行层源码），返回路径。"""
    target_dir = os.path.join(repo_root, "dualarm_ws", "src", "rm_executor",
                              "rm_executor")
    target = os.path.join(target_dir, name)
    with open(target, "w", encoding="utf-8") as fh:
        fh.write(source)
    return target


def _safety_fixture(repo_root, name, source):
    """在安全层包目录里放一个临时文件（模拟安全层源码），返回路径。

    安全层的层根是 ``dualarm_ws/src/rm_safety``（见 ``rules.layers``），
    因此放在包内的 ``tests/`` 下即可被判为「safety 层」。
    """
    target_dir = os.path.join(repo_root, "dualarm_ws", "src", "rm_safety",
                              "tests")
    if not os.path.isdir(target_dir):
        os.makedirs(target_dir, exist_ok=True)
    target = os.path.join(target_dir, name)
    with open(target, "w", encoding="utf-8") as fh:
        fh.write(source)
    return target


def test_layers_are_declared_in_contract():
    """contract.yaml 必须声明 rules.layers（层 → 包根）。"""
    from rm_hw_bridge import loader

    contract = audit._load_contract()
    layers = (contract.get("rules") or {}).get("layers") or {}
    assert layers.get("hardware") == "rm_hw_bridge"
    assert layers.get("executor") == "rm_executor"
    assert layers.get("experience") == "rmctl"
    # 核心/支持符号清单不变（R003 结论原样成立）
    assert contract["limits"]["core_count"] == 22
    # 支持层数量**按契约读取**（不写死数字）
    assert contract["limits"]["support_count"] == len(
        loader.support_symbols(contract)
    )


def test_layer_roots_are_resolved(repo_root):
    """层 → 目录的解析结果正确。"""
    roots = audit.layer_roots()
    assert roots["hardware"].endswith(os.path.join("src", "rm_hw_bridge"))
    assert roots["executor"].endswith(os.path.join("src", "rm_executor"))
    assert roots["experience"].endswith(os.path.join("src", "rmctl"))


def test_negative_executor_layer_imports_rmctl(repo_root):
    """执行层 import rmctl → 反向依赖违规（CLI 退出码 3）。"""
    target = _executor_fixture(repo_root, "_tmp_audit_exec_rmctl.py",
                               EXECUTOR_RMCTL_SOURCE)
    try:
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_REVERSE_DEPENDENCY for v in violations)
        assert audit.main(["--paths", target]) == 3
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_negative_executor_layer_imports_tesseract(repo_root):
    """执行层 import tesseract*（规划内核）→ 反向依赖违规（CLI 退出码 3）。"""
    target = _executor_fixture(repo_root, "_tmp_audit_exec_tesseract.py",
                               EXECUTOR_TESSERACT_SOURCE)
    try:
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_REVERSE_DEPENDENCY for v in violations)
        assert audit.main(["--paths", target]) == 3
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_positive_executor_layer_imports_hw_bridge(repo_root):
    """执行层 import rm_hw_bridge → 合规（允许向下依赖硬件层，退出码 0）。"""
    target = _executor_fixture(repo_root, "_tmp_audit_exec_hwbridge.py",
                               EXECUTOR_HW_BRIDGE_SOURCE)
    try:
        assert audit.scan_paths([target]) == []
        assert audit.main(["--paths", target]) == 0
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_positive_clean_executor_file(repo_root):
    """干净的执行层文件 → 0 违规（退出码 0）。"""
    target = _executor_fixture(repo_root, "_tmp_audit_exec_clean.py",
                               EXECUTOR_CLEAN_SOURCE)
    try:
        assert audit.scan_paths([target]) == []
        assert audit.main(["--paths", target]) == 0
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_negative_hardware_layer_imports_rmctl_still_violation(repo_root):
    """硬件层 import rmctl → 仍违规（退出码 3，行为不变）。"""
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(audit.__file__), ".."))
    target = os.path.join(pkg_root, "tests", "_tmp_audit_fixture3.py")
    try:
        with open(target, "w", encoding="utf-8") as fh:
            fh.write(RMCTL_SOURCE)
        assert audit.main(["--paths", target]) == 3
    finally:
        if os.path.exists(target):
            os.remove(target)


# ------------------------------------------- 安全层反向依赖红线（S04/T8）
def test_safety_layer_is_declared_in_contract():
    """contract.yaml 必须声明 safety 层与它的反向依赖禁令。"""
    from rm_hw_bridge import loader

    contract = audit._load_contract()
    rules = contract.get("rules") or {}
    layers = rules.get("layers") or {}
    assert layers.get("safety") == "rm_safety"
    targets = audit._reverse_dep_targets(contract)
    assert set(targets.get("safety") or []) == {"rmctl", "rm_executor", "tesseract*"}
    # 核心/支持符号清单**不变**（R003 结论原样成立）
    assert contract["limits"]["core_count"] == 22
    assert contract["limits"]["support_count"] == len(
        loader.support_symbols(contract)
    )


def test_safety_layer_root_is_resolved(repo_root):
    """安全层的层根目录解析正确（dualarm_ws/src/rm_safety）。"""
    roots = audit.layer_roots()
    assert roots["safety"].endswith(os.path.join("src", "rm_safety"))
    assert audit.layer_of(os.path.join(roots["safety"], "rm_safety", "levels.py"),
                          roots) == "safety"


def test_negative_safety_layer_imports_rm_executor(repo_root):
    """安全层 import rm_executor → 反向依赖违规（CLI 退出码 3）。"""
    target = _safety_fixture(repo_root, "_tmp_audit_safety_exec.py",
                             SAFETY_RM_EXECUTOR_SOURCE)
    try:
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_REVERSE_DEPENDENCY for v in violations)
        assert audit.main(["--paths", target]) == 3
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_negative_safety_layer_imports_rmctl(repo_root):
    """安全层 import rmctl → 反向依赖违规（CLI 退出码 3）。"""
    target = _safety_fixture(repo_root, "_tmp_audit_safety_rmctl.py",
                             SAFETY_RMCTL_SOURCE)
    try:
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_REVERSE_DEPENDENCY for v in violations)
        assert audit.main(["--paths", target]) == 3
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_negative_safety_layer_imports_tesseract(repo_root):
    """安全层 import tesseract*（规划内核）→ 反向依赖违规（CLI 退出码 3）。"""
    target = _safety_fixture(repo_root, "_tmp_audit_safety_tesseract.py",
                             SAFETY_TESSERACT_SOURCE)
    try:
        violations = audit.scan_paths([target])
        assert any(v.kind == audit.KIND_REVERSE_DEPENDENCY for v in violations)
        assert audit.main(["--paths", target]) == 3
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_positive_clean_safety_file(repo_root):
    """干净的安全层文件 → 0 违规（退出码 0）。"""
    target = _safety_fixture(repo_root, "_tmp_audit_safety_clean.py",
                             SAFETY_CLEAN_SOURCE)
    try:
        assert audit.scan_paths([target]) == []
        assert audit.main(["--paths", target]) == 0
    finally:
        if os.path.exists(target):
            os.remove(target)


def test_safety_package_has_no_reverse_dependency(repo_root):
    """真实安全层包内零反向依赖（不 import rm_executor / rmctl / tesseract*）。"""
    pkg_dir = os.path.join(repo_root, "dualarm_ws", "src", "rm_safety")
    violations = audit.scan_paths([pkg_dir])
    assert violations == [], "安全层包内存在违规: %s" % [str(v) for v in violations]


def test_symbol_lists_unchanged_after_safety_layer_added():
    """新增安全层后，22 核心 / 21 支持符号名单与黑名单一律不变。"""
    from rm_hw_bridge import loader

    contract = audit._load_contract()
    core = loader.core_symbols(contract)
    support = loader.support_symbols(contract)
    assert len(core) == 22
    # 支持层数量**按契约读取**（不写死数字）
    assert len(support) == contract["limits"]["support_count"]
    assert contract["limits"]["core_count"] == len(core)
    assert contract["limits"]["support_count"] == len(support)
    symbols, prefixes = audit._blacklist(contract)
    assert len(symbols) == 13
    assert prefixes == ["rm_algo_"]


# ------------------------------------------------------------------ 正例
def test_strings_are_not_mistaken_for_calls(string_only_file):
    """字符串常量不算调用（避免把黑名单清单误判为违规）。"""
    assert audit.scan_paths([str(string_only_file)]) == []


def test_repo_self_scan_has_no_violations(repo_root):
    """仓库自扫描零违规。"""
    report = audit.audit_repo(root=repo_root)
    assert report["files_scanned"] > 0
    assert report["ok"], "仓库自扫描发现违规: %s" % report["violations"]


def test_hardware_package_has_no_forbidden_imports(pkg_root):
    """rm_hw_bridge 包内不含 tesseract / Robotic_Arm / moveit / rmctl 的 import。"""
    pkg_dir = os.path.join(pkg_root, "rm_hw_bridge")
    violations = audit.scan_paths([pkg_dir])
    assert violations == [], "硬件层包内存在违规: %s" % [str(v) for v in violations]


def test_audit_scanner_skips_gsd_and_deps(repo_root):
    """不扫 .gsd/ / .deps/ / docs/。"""
    assert {".gsd", ".deps", "docs"} <= audit.SKIP_DIR_NAMES
    paths = audit.repo_scan_paths(repo_root)
    for p in paths:
        parts = set(os.path.abspath(p).split(os.sep))
        assert ".gsd" not in parts
        assert ".deps" not in parts
        assert "docs" not in parts
    # 扫描根只包含 dualarm_ws/src 与 workcell
    rel = sorted(os.path.relpath(p, repo_root) for p in paths)
    assert rel == sorted(audit.REPO_SCAN_ROOTS)


# ------------------------------------------------------------------ CLI
def test_cli_exit_code_3_on_violations(bad_file, capsys):
    assert audit.main(["--paths", str(bad_file)]) == 3
    err = capsys.readouterr().err
    assert "违规" in err


def test_cli_exit_code_0_on_clean(string_only_file, capsys):
    assert audit.main(["--paths", str(string_only_file)]) == 0


def test_cli_repo_scan_exit_code_0(repo_root, capsys):
    assert audit.main(["--repo"]) == 0


def test_cli_json_output(bad_file, capsys):
    import json

    assert audit.main(["--paths", str(bad_file), "--json"]) == 3
    payload = json.loads(capsys.readouterr().out)
    assert payload["ok"] is False
    assert payload["violations"]


# ------------------------------------------------------------------ 符号核对
def test_check_symbols_against_real_sdk(repo_root):
    """若仓库已物化真实 .so，则 22 核心符号必须全部可解析。"""
    from rm_hw_bridge import loader

    lib_path = loader.find_library(ws=repo_root)
    if not lib_path:
        pytest.skip("仓库内未物化 RM-API2 .so（.deps/ 是 gitignored，可选校验）")
    report = audit.check_symbols(lib_path)
    assert report["missing_core"] == [], "缺少核心符号: %s" % report["missing_core"]
    assert report["missing_support"] == []
    assert report["ok"]
