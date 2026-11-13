"""``models/rm75_drake/joint_limits.yaml`` 的交接件契约测试（S01-T06）。

这些断言守住的契约（下游 S03 安全闸直接消费）：

* 交接件**逐条**等于改造版 URDF 的 ``<limit>``（hardware 段）——
  否则下游拿到的是过期限位（模型漂移检测）；
* 软件限位**恰好**比硬件限位每侧再收 4°（0.0698132 rad）——
  收紧规则被悄悄改掉必须判红；
* 软件限位严格落在硬件限位之内（安全闸不允许越界）；
* 每个关节都标注了来源（官方参数表 + 上游 commit），缺来源必须判红。

负例同样重要：越界、未收紧、来源缺失、文件损坏都必须被显式报错。
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from rm75_drake import limits as limits_mod  # noqa: E402
from rm75_drake import models as models_mod  # noqa: E402

LIMITS_PATH = REPO_ROOT / "models" / "rm75_drake" / "joint_limits.yaml"


@pytest.fixture(scope="module")
def limits_data() -> dict:
    return limits_mod.load_joint_limits(LIMITS_PATH)


@pytest.fixture(scope="module")
def urdf_limits() -> list:
    return models_mod.joint_limits_from_urdf()


# --------------------------------------------------------------------------- #
# 正例
# --------------------------------------------------------------------------- #
def test_limits_file_is_tracked_and_parseable():
    assert LIMITS_PATH.is_file(), "限位交接件必须入 git（下游 S03 直接消费）"
    data = yaml.safe_load(LIMITS_PATH.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    assert data["schema_version"] == 1


def test_margin_is_four_degrees_in_radians():
    # 常量取计划里钉死的十进制值 0.0698132（= 4°，与 math.radians 差 ~3e-8）
    assert limits_mod.SOFTWARE_MARGIN_RAD == pytest.approx(math.radians(4.0), abs=1e-6)
    assert limits_mod.SOFTWARE_MARGIN_RAD == 0.0698132
    assert limits_mod.SOFTWARE_MARGIN_DEG == 4.0


def test_joint_names_and_order(limits_data):
    names = [j["name"] for j in limits_data["joints"]]
    assert names == [f"joint_{i}" for i in range(1, 8)]


def test_hardware_limits_match_urdf_declaration(limits_data, urdf_limits):
    """交接件的 hardware 段必须逐条等于 URDF 的 <limit>（漂移检测）。"""
    by_name = {item["name"]: item for item in urdf_limits}
    assert set(by_name) == {f"joint_{i}" for i in range(1, 8)}
    for joint in limits_data["joints"]:
        urdf_item = by_name[joint["name"]]
        hardware = joint["hardware"]
        assert hardware["position_lower_rad"] == pytest.approx(urdf_item["lower"], abs=1e-9)
        assert hardware["position_upper_rad"] == pytest.approx(urdf_item["upper"], abs=1e-9)
        assert hardware["velocity_rad_s"] == pytest.approx(urdf_item["velocity"], abs=1e-9)
        assert hardware["effort_nm"] == pytest.approx(urdf_item["effort"], abs=1e-9)


def test_software_limits_are_tightened_by_four_degrees(limits_data):
    for joint in limits_data["joints"]:
        expected = limits_mod.expected_software_limits(
            joint["hardware"]["position_lower_rad"],
            joint["hardware"]["position_upper_rad"],
        )
        assert joint["software"]["position_lower_rad"] == pytest.approx(
            expected["position_lower_rad"], abs=1e-6
        )
        assert joint["software"]["position_upper_rad"] == pytest.approx(
            expected["position_upper_rad"], abs=1e-6
        )


def test_software_limits_are_strictly_inside_hardware_limits(limits_data):
    for joint in limits_data["joints"]:
        hardware = joint["hardware"]
        software = joint["software"]
        assert hardware["position_lower_rad"] < software["position_lower_rad"]
        assert software["position_lower_rad"] < software["position_upper_rad"]
        assert software["position_upper_rad"] < hardware["position_upper_rad"]
        assert software["velocity_rad_s"] <= hardware["velocity_rad_s"]
        assert software["effort_nm"] <= hardware["effort_nm"]


def test_joint_7_is_continuous_turn_and_also_tightened(limits_data):
    joint_7 = next(j for j in limits_data["joints"] if j["name"] == "joint_7")
    assert joint_7["hardware"]["position_lower_rad"] == pytest.approx(
        -6.283185307179586, abs=1e-12
    )
    assert joint_7["hardware"]["position_upper_rad"] == pytest.approx(
        6.283185307179586, abs=1e-12
    )
    # 连续圈没有机械硬限位 → 同样每侧收紧 4°
    assert joint_7["software"]["position_upper_rad"] == pytest.approx(
        6.283185307179586 - limits_mod.SOFTWARE_MARGIN_RAD, abs=1e-6
    )
    assert "joint_7_reason" in limits_data["software_limits"]


def test_source_block_records_official_table_and_upstream_commit(limits_data):
    source = limits_data["source"]
    assert "官方" in source["official_parameter_table"]
    commit = source["upstream_commit"]
    assert len(commit) == 40 and all(c in "0123456789abcdef" for c in commit)
    assert commit.startswith("cff49bb4")
    assert "effort" in source["effort_source"]


def test_control_constraint_is_documented(limits_data):
    """R016：真机仅关节空间位置伺服，力矩级控制仅用于仿真——必须写进交接件。"""
    assert "位置伺服" in limits_data["source"]["control_constraint"]
    assert "仿真" in limits_data["source"]["control_constraint"]


def test_checker_reports_no_problem_on_the_real_file(limits_data, urdf_limits):
    assert limits_mod.check_joint_limits(limits_data, urdf_limits) == []


# --------------------------------------------------------------------------- #
# 负例
# --------------------------------------------------------------------------- #
def test_not_tightened_software_limit_is_detected(limits_data, urdf_limits):
    """负例：软件限位没收紧（等于硬件限位）→ 必须判红。"""
    broken = copy.deepcopy(limits_data)
    broken["joints"][0]["software"]["position_upper_rad"] = broken["joints"][0]["hardware"][
        "position_upper_rad"
    ]
    problems = limits_mod.check_joint_limits(broken, urdf_limits)
    assert problems
    assert any("position_upper_rad" in p for p in problems)


def test_software_limit_outside_hardware_is_detected(limits_data, urdf_limits):
    """负例：软件限位跑到硬件限位之外（安全闸越界）→ 必须判红。"""
    broken = copy.deepcopy(limits_data)
    broken["joints"][2]["software"]["position_upper_rad"] = (
        broken["joints"][2]["hardware"]["position_upper_rad"] + 0.01
    )
    problems = limits_mod.check_joint_limits(broken, urdf_limits)
    assert problems
    assert any("joint_3" in p for p in problems)


def test_drifted_hardware_limit_is_detected(limits_data, urdf_limits):
    """负例：hardware 与 URDF 漂移（交接件过期）→ 必须判红。"""
    broken = copy.deepcopy(limits_data)
    broken["joints"][1]["hardware"]["velocity_rad_s"] = 3.14
    problems = limits_mod.check_joint_limits(broken, urdf_limits)
    assert problems
    assert any("joint_2" in p for p in problems)


def test_missing_source_block_is_detected(limits_data, urdf_limits):
    """负例：来源信息被删（限位真值不可追溯）→ 必须判红。"""
    broken = copy.deepcopy(limits_data)
    broken.pop("source")
    problems = limits_mod.check_joint_limits(broken, urdf_limits)
    assert problems
    assert any("source" in p for p in problems)


def test_bad_commit_is_detected(limits_data, urdf_limits):
    broken = copy.deepcopy(limits_data)
    broken["source"]["upstream_commit"] = "not-a-commit"
    problems = limits_mod.check_joint_limits(broken, urdf_limits)
    assert problems
    assert any("upstream_commit" in p for p in problems)


def test_wrong_margin_is_detected(limits_data, urdf_limits):
    """负例：收紧余量被改成别的值 → 必须判红。"""
    broken = copy.deepcopy(limits_data)
    broken["software_limits"]["margin_rad"] = 0.1
    problems = limits_mod.check_joint_limits(broken, urdf_limits)
    assert problems
    assert any("margin_rad" in p for p in problems)


def test_missing_file_raises_limits_error(tmp_path):
    """负例：交接件缺失 → 显式抛错（不得静默返回空字典）。"""
    with pytest.raises(limits_mod.LimitsError):
        limits_mod.load_joint_limits(tmp_path / "nope.yaml")


def test_malformed_file_raises_limits_error(tmp_path):
    """负例：交接件损坏 → 显式抛错。"""
    broken = tmp_path / "joint_limits.yaml"
    broken.write_text("joints: [oops\n", encoding="utf-8")
    with pytest.raises(limits_mod.LimitsError):
        limits_mod.load_joint_limits(broken)


def test_non_mapping_top_level_raises_limits_error(tmp_path):
    broken = tmp_path / "joint_limits.yaml"
    broken.write_text("- just\n- a\n- list\n", encoding="utf-8")
    with pytest.raises(limits_mod.LimitsError):
        limits_mod.load_joint_limits(broken)
