"""S13-T2 真机只读采集工件的验收用例（``rm75_s13_versions_connectivity.json``）。

覆盖三层：

* **工件合规层**：schema / status / 版本字段「非空或显式 null」纪律 /
  左右臂连通性都有明确布尔结论 + 原始输出摘要 / honesty 语义 /
  **工件里没有任何声称做过运动的字段**（零运动红线）；
* **脚本行为层**：``--dry-run`` 不连任何设备、输出 ``status=dry_run`` 骨架，
  且**不会覆盖**真机工件；
* **负例层**：``status=blocked`` 的工件若不自证（缺门禁结果 / 缺 next_action）
  必须被判为不合规 —— blocked 不等于"什么都不写"。
"""
from __future__ import annotations

import copy
import importlib.util
import json
import os
import subprocess
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
SCRIPT_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_s13_versions_probe.py")
ARTIFACT_PATH = os.path.join(
    REPO_ROOT, "workcell", "baselines", "evidence",
    "rm75_s13_versions_connectivity.json",
)


def _load_script():
    """按路径加载脚本模块（脚本不是包，用 importlib 加载）。"""
    spec = importlib.util.spec_from_file_location("rm75_s13_versions_probe",
                                                  SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def probe():
    return _load_script()


@pytest.fixture(scope="module")
def artifact():
    assert os.path.exists(ARTIFACT_PATH), (
        "缺少 S13-T2 证据工件：%s" % ARTIFACT_PATH)
    with open(ARTIFACT_PATH, "r", encoding="utf-8") as fh:
        return json.load(fh)


# --------------------------------------------------------------------------
# 工件合规层
# --------------------------------------------------------------------------

def test_artifact_schema_and_status(artifact, probe):
    """schema 固定，status 只能 ok / blocked。"""
    assert artifact["schema"] == "rmctl.rm75_s13_versions_connectivity/1"
    assert artifact["schema"] == probe.SCHEMA
    assert artifact["status"] in ("ok", "blocked")


def test_artifact_passes_self_validation(artifact, probe):
    """工件必须通过脚本内置的自检（问题列表为空）。"""
    assert probe.validate_artifact(artifact) == []


def test_version_fields_are_non_empty_or_explicit_null(artifact, probe):
    """版本字段「非空字符串 或 显式 null」；blocked 时允许全 null。"""
    sw = artifact["controller_software"]
    for name in probe.VERSION_FIELDS:
        assert name in sw, "缺版本字段 %s" % name
        value = sw[name]
        assert value is None or (isinstance(value, str) and value.strip()), (
            "版本字段 %s 既不是非空字符串也不是 null：%r" % (name, value))
    if artifact["status"] == "ok":
        assert any(sw[name] for name in probe.VERSION_FIELDS), (
            "status=ok 却没有任何非空版本串")


def test_robot_info_fields(artifact):
    """机型 / 自由度 / 控制器代数：有值就是数值，无值就是 null。"""
    ri = artifact["robot_info"]
    for name in ("arm_dof", "arm_model", "robot_controller_version"):
        assert name in ri
        value = ri[name]
        assert value is None or isinstance(value, int)


def test_connectivity_has_explicit_boolean_and_raw_output(artifact):
    """左右臂都必须有明确布尔结论 + 原始 ping / TCP 输出摘要。"""
    for side in ("left_connectivity", "right_connectivity"):
        conn = artifact[side]
        assert isinstance(conn["reachable"], bool), (
            "%s.reachable 必须是明确布尔结论" % side)
        assert conn["ip"]
        for proto in ("ping", "tcp"):
            summary = conn[proto]["summary"]
            assert isinstance(summary, str) and summary.strip(), (
                "%s.%s 缺原始输出摘要" % (side, proto))


def test_right_arm_connectivity_conclusion_is_recorded(artifact):
    """右臂 192.168.1.19 必须有实测结论（本轮允许结论为「不可达」）。"""
    right = artifact["right_connectivity"]
    assert right["ip"] == "192.168.1.19"
    assert isinstance(right["reachable"], bool)
    assert right["ping"]["summary"].strip()
    assert right["tcp"]["summary"].strip()


def test_honesty_declares_no_motion_and_no_teachpendant(artifact):
    """honesty 必须写明「未读取示教器版本」「未下发任何运动指令」。"""
    honesty = artifact["honesty"]
    assert honesty["no_motion_command_issued"] is True
    assert honesty["teachpendant_version_read"] is False
    notes = " ".join(honesty["notes"])
    assert "运动指令" in notes
    assert "示教器" in notes


def test_artifact_contains_no_motion_claiming_field(artifact, probe):
    """工件里不得出现任何声称做过运动的字段名（零运动红线）。"""
    keys = set(probe._walk_keys(artifact))
    assert not (keys & set(probe.FORBIDDEN_MOTION_FIELDS)), (
        "工件出现运动字段：%s" % sorted(keys & set(probe.FORBIDDEN_MOTION_FIELDS)))


def test_container_image_recorded(artifact):
    """容器镜像标签必须来自宿主机 docker inspect（非空字符串或 null）。"""
    image = artifact["container_image"]
    assert image is None or (isinstance(image, str) and image.strip())


def test_next_action_is_pasteable(artifact):
    """必须给出可粘贴的下一步。"""
    assert str(artifact["next_action"]).strip()


# --------------------------------------------------------------------------
# 负例层
# --------------------------------------------------------------------------

def test_blocked_artifact_must_self_certify(artifact, probe):
    """负例：status=blocked 的工件若缺门禁结果 / next_action ⇒ 不合规。"""
    broken = copy.deepcopy(artifact)
    broken["status"] = "blocked"
    broken["gates"] = []
    broken["next_action"] = ""
    problems = probe.validate_artifact(broken)
    assert problems, "blocked 工件不自证时必须被判为不合规"
    assert any("gates" in p for p in problems)
    assert any("next_action" in p for p in problems)


def test_empty_version_string_is_rejected(artifact, probe):
    """负例：版本字段写成空串（冒充已读到）⇒ 不合规。"""
    broken = copy.deepcopy(artifact)
    broken["status"] = "ok"
    broken["controller_software"]["ctrl_version"] = ""
    problems = probe.validate_artifact(broken)
    assert any("ctrl_version" in p for p in problems)


def test_missing_connectivity_summary_is_rejected(artifact, probe):
    """负例：连通性没有原始输出摘要 ⇒ 不合规。"""
    broken = copy.deepcopy(artifact)
    broken["right_connectivity"]["tcp"]["summary"] = "   "
    problems = probe.validate_artifact(broken)
    assert any("right_connectivity" in p for p in problems)


def test_motion_field_in_artifact_is_rejected(artifact, probe):
    """负例：工件出现运动字段名 ⇒ 不合规。"""
    broken = copy.deepcopy(artifact)
    broken["motion"] = {"movej": 1}
    problems = probe.validate_artifact(broken)
    assert any("运动字段" in p for p in problems)


# --------------------------------------------------------------------------
# 脚本行为层
# --------------------------------------------------------------------------

def test_dry_run_does_not_connect_and_writes_skeleton(tmp_path):
    """--dry-run：不连任何设备、status=dry_run、且不覆盖真机工件。"""
    out = tmp_path / "s13_versions"
    proc = subprocess.run(
        [sys.executable, SCRIPT_PATH, "--dry-run", "--out", str(out)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=120,
    )
    assert proc.returncode == 0, proc.stdout.decode("utf-8", "replace")
    written = tmp_path / "s13_versions.dryrun.json"
    assert written.exists()
    with open(written, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    assert data["status"] == "dry_run"
    assert data["controller_software"]["product_version"] is None
    assert data["left_connectivity"]["reachable"] is None
    assert data["right_connectivity"]["reachable"] is None
    assert data["honesty"]["no_motion_command_issued"] is True


def test_dry_run_never_overwrites_the_real_artifact(tmp_path):
    """dry-run 的输出路径必须与真机工件路径错开。"""
    real = ARTIFACT_PATH
    assert os.path.exists(real)
    before = open(real, "rb").read()
    out = tmp_path / os.path.basename(real)
    proc = subprocess.run(
        [sys.executable, SCRIPT_PATH, "--dry-run", "--out", str(out)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=120,
    )
    assert proc.returncode == 0
    assert open(real, "rb").read() == before
