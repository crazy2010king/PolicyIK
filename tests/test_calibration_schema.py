"""标定文件 / 采样点集 schema 测试（S04-T04）。

覆盖
----
* **正例**：合法标定文件（内存 dict + YAML 文件）→ 正常解析；
* **反例逐条点名**：未知顶层键 / 缺必填字段 / xyz 非 3 维 / rpy 用角度制（超 π）/
  ``measured`` 缺类型 / 关节数不是 7 / 日期格式错 / 未知子键
  → 抛 ``CalibrationError`` 且消息**点名该键**；
* **采样 schema 反例**：缺 ``synthetic``/``source``、未知键、点数不足（TCP < 4、
  外参 < 15）；
* **报告纪律**：``measured=false`` 的报告**显式声明「未实测」**并列出未验证项。
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from rm75_drake.calibration import (
    UNVERIFIED_ITEMS,
    CalibrationDoc,
    CalibrationError,
    build_report,
    cross_validation_summary,
    load_calibration_file,
    matrix_to_rpy,
    parse_calibration,
    parse_samples,
    repeatability_summary,
    rpy_to_matrix,
)
from rm75_drake.calibration.solvers import ResidualReport


def _valid_doc_dict() -> dict:
    """一份合法的标定文件内容（measured=false 的占位语义）。"""
    return {
        "schema_version": 1,
        "measured": False,
        "calibrated": False,
        "method": "synthetic-kabsch",
        "date": "2026-11-13",
        "base_extrinsic": {
            "left": {
                "xyz": [0.0, 0.0, 0.0],
                "rpy": [0.0, 0.0, 0.0],
                "residual": {"rms": 0.0, "max": 0.0, "samples": 20},
            },
            "right": {
                "xyz": [0.6, 0.0, 0.0],
                "rpy": [0.0, 0.0, 0.0],
                "residual": {"rms": 1e-9, "max": 2e-9, "samples": 20},
            },
        },
        "joint_zero_offset": {
            "left": {f"left_joint_{i}": 0.001 * i for i in range(1, 8)},
        },
        "tool_offset": {
            "left": {
                "xyz": [0.0, 0.0, 0.06],
                "rpy": [0.0, 0.0, 0.0],
                "residual": {"rms": 1e-10, "max": 2e-10, "samples": 4},
            },
        },
    }


# ── 正例 ────────────────────────────────────────────────────────────────
def test_parse_valid_calibration_doc() -> None:
    """合法标定文件 → 正常解析，字段齐全。"""
    doc = parse_calibration(_valid_doc_dict())

    assert isinstance(doc, CalibrationDoc)
    assert doc.schema_version == 1
    assert doc.measured is False
    assert doc.calibrated is False
    assert doc.method == "synthetic-kabsch"
    assert doc.date == "2026-11-13"
    assert set(doc.base_extrinsic) == {"left", "right"}
    assert doc.base_extrinsic["right"].xyz == (0.6, 0.0, 0.0)
    assert doc.base_extrinsic["left"].residual is not None
    assert doc.base_extrinsic["left"].residual.samples == 20
    assert len(doc.joint_zero_offset["left"]) == 7
    assert doc.tool_offset["left"].xyz == (0.0, 0.0, 0.06)


def test_load_calibration_file_from_yaml(tmp_path: Path) -> None:
    """YAML 文件 → load_calibration_file 正常解析（source_path 回填）。"""
    import yaml

    path = tmp_path / "dual_arm.yaml"
    path.write_text(
        yaml.safe_dump(_valid_doc_dict(), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    doc = load_calibration_file(path)

    assert doc.source_path == str(path)
    assert doc.method == "synthetic-kabsch"


def test_load_calibration_file_missing_raises_file_not_found(tmp_path: Path) -> None:
    """文件不存在 → FileNotFoundError（不是静默默认值）。"""
    with pytest.raises(FileNotFoundError):
        load_calibration_file(tmp_path / "nope.yaml")


# ── 反例：逐条点名 ──────────────────────────────────────────────────────
def test_unknown_top_level_key_names_the_key() -> None:
    """未知顶层键 → CalibrationError 且点名该键。"""
    data = _valid_doc_dict()
    data["unknown_top_key"] = 1

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "unknown_top_key" in message
    assert "未知顶层键" in message


def test_missing_required_key_names_the_key() -> None:
    """缺必填字段 → CalibrationError 且点名该键。"""
    data = _valid_doc_dict()
    del data["method"]

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "method" in message
    assert "缺少必填" in message


def test_xyz_not_three_dimensions_names_the_key() -> None:
    """xyz 非 3 维 → CalibrationError 且点名 xyz。"""
    data = _valid_doc_dict()
    data["base_extrinsic"]["left"]["xyz"] = [0.0, 0.0]

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "base_extrinsic.left.xyz" in message


def test_rpy_in_degrees_is_rejected_with_conversion_hint() -> None:
    """rpy 写成角度制（超 π）→ 报错点名 rpy，并给换算提示。"""
    data = _valid_doc_dict()
    data["base_extrinsic"]["left"]["rpy"] = [90.0, 0.0, 0.0]

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "base_extrinsic.left.rpy" in message
    assert "弧度" in message
    assert "度" in message


def test_measured_wrong_type_names_the_key() -> None:
    """measured 缺类型（写成字符串）→ 报错点名 measured。"""
    data = _valid_doc_dict()
    data["measured"] = "yes"

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "measured" in message
    assert "布尔" in message


def test_calibrated_true_without_measured_raises() -> None:
    """calibrated=true 但 measured=false → 报错（未实测不得声明已标定）。"""
    data = _valid_doc_dict()
    data["calibrated"] = True

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    assert "measured" in str(excinfo.value)
    assert "calibrated" in str(excinfo.value)


def test_joint_zero_offset_wrong_joint_count_names_the_key() -> None:
    """关节数不是 7 → 报错点名 joint_zero_offset。"""
    data = _valid_doc_dict()
    data["joint_zero_offset"]["left"] = {"left_joint_1": 0.0}

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "joint_zero_offset.left" in message
    assert "7" in message


def test_bad_date_format_names_the_key() -> None:
    """日期格式错 → 报错点名 date。"""
    data = _valid_doc_dict()
    data["date"] = "2026/11/13"

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    assert "date" in str(excinfo.value)


def test_unknown_sub_key_names_the_key() -> None:
    """未知子键 → 报错点名该键。"""
    data = _valid_doc_dict()
    data["tool_offset"]["left"]["extra"] = 1.0

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    message = str(excinfo.value)
    assert "tool_offset.left" in message
    assert "extra" in message


def test_schema_version_mismatch_names_the_key() -> None:
    """schema_version 不受支持 → 报错点名 schema_version。"""
    data = _valid_doc_dict()
    data["schema_version"] = 99

    with pytest.raises(CalibrationError) as excinfo:
        parse_calibration(data)

    assert "schema_version" in str(excinfo.value)


# ── 采样点集 schema ─────────────────────────────────────────────────────
def _valid_samples_dict() -> dict:
    return {
        "tool_tcp": [
            {
                "local_point": [0.0, 0.0, 0.0],
                "world_point": [0.0, 0.0, 0.0],
                "synthetic": True,
                "source": "synthetic",
            }
            for _ in range(4)
        ],
        "base_extrinsic": [
            {
                "left_point": [0.0, 0.0, 0.0],
                "right_point": [0.0, 0.0, 0.0],
                "synthetic": True,
                "source": "synthetic",
            }
            for _ in range(15)
        ],
    }


def test_parse_valid_samples() -> None:
    """合法采样 → 正常解析。"""
    samples = parse_samples(_valid_samples_dict())

    assert len(samples.tool_tcp) == 4
    assert len(samples.base_extrinsic) == 15
    assert samples.tool_tcp[0].synthetic is True
    assert samples.tool_tcp[0].source == "synthetic"


def test_samples_missing_synthetic_names_the_key() -> None:
    """采样缺 ``synthetic`` → 报错点名 synthetic。"""
    data = _valid_samples_dict()
    del data["tool_tcp"][0]["synthetic"]

    with pytest.raises(CalibrationError) as excinfo:
        parse_samples(data)

    message = str(excinfo.value)
    assert "synthetic" in message
    assert "tool_tcp[0]" in message


def test_samples_missing_source_names_the_key() -> None:
    """采样缺 ``source`` → 报错点名 source。"""
    data = _valid_samples_dict()
    del data["tool_tcp"][0]["source"]

    with pytest.raises(CalibrationError) as excinfo:
        parse_samples(data)

    assert "source" in str(excinfo.value)


def test_samples_unknown_key_names_the_key() -> None:
    """采样未知键 → 报错点名该键。"""
    data = _valid_samples_dict()
    data["tool_tcp"][0]["oops"] = 1

    with pytest.raises(CalibrationError) as excinfo:
        parse_samples(data)

    assert "oops" in str(excinfo.value)


def test_samples_too_few_tool_points_names_the_key() -> None:
    """tool_tcp 点对不足 4 组 → 报错点名 tool_tcp。"""
    data = _valid_samples_dict()
    data["tool_tcp"] = data["tool_tcp"][:3]

    with pytest.raises(CalibrationError) as excinfo:
        parse_samples(data)

    message = str(excinfo.value)
    assert "tool_tcp" in message
    assert "4" in message


def test_samples_too_few_base_extrinsic_points_names_the_key() -> None:
    """base_extrinsic 点对不足 15 组 → 报错点名 base_extrinsic。"""
    data = _valid_samples_dict()
    data["base_extrinsic"] = data["base_extrinsic"][:14]

    with pytest.raises(CalibrationError) as excinfo:
        parse_samples(data)

    message = str(excinfo.value)
    assert "base_extrinsic" in message
    assert "15" in message


def test_samples_unknown_top_level_key_names_the_key() -> None:
    """采样文件未知顶层键 → 报错点名该键。"""
    data = _valid_samples_dict()
    data["joint_zero_typo"] = []

    with pytest.raises(CalibrationError) as excinfo:
        parse_samples(data)

    assert "joint_zero_typo" in str(excinfo.value)


# ── 报告纪律 ────────────────────────────────────────────────────────────
def test_report_declares_unmeasured_and_lists_unverified_items() -> None:
    """measured=false → 报告显式声明「未实测」并逐条列出未验证项。"""
    doc = parse_calibration(_valid_doc_dict())
    report = build_report(doc)

    rendered = report.render()
    assert "未实测" in rendered
    assert "measured    : False" in rendered
    for item in UNVERIFIED_ITEMS:
        assert item in report.unverified
        assert item in rendered
    assert report.all_within_threshold is True


def test_report_measured_true_has_no_unmeasured_declaration() -> None:
    """measured=true → 不再输出「未实测声明」段。"""
    doc = parse_calibration(_valid_doc_dict())
    doc = CalibrationDoc(
        schema_version=doc.schema_version,
        measured=True,
        calibrated=True,
        method=doc.method,
        date=doc.date,
        base_extrinsic=doc.base_extrinsic,
        joint_zero_offset=doc.joint_zero_offset,
        tool_offset=doc.tool_offset,
    )
    report = build_report(doc)

    assert report.unverified == ()
    assert "未实测声明" not in report.render()


def test_repeatability_summary_reports_spread() -> None:
    """重复性摘要：均值 / 标准差 / 极差。"""
    summary = repeatability_summary([1.0, 1.2, 0.8])

    assert summary["count"] == 3
    assert abs(summary["mean"] - 1.0) < 1e-12
    assert summary["range"] > 0.0
    assert summary["std"] > 0.0


def test_cross_validation_summary_reports_difference() -> None:
    """第三方交叉验证摘要：两条残差序列的差异。"""
    primary = ResidualReport("primary", (0.001, 0.002), threshold=0.01)
    secondary = ResidualReport("secondary", (0.0015, 0.0025), threshold=0.01)

    summary = cross_validation_summary(primary, secondary)

    assert summary["max_difference"] > 0.0
    assert summary["unit"] == "m"
    assert summary["primary"] == "primary"
    assert summary["secondary"] == "secondary"


# ── 姿态工具 ────────────────────────────────────────────────────────────
def test_rpy_matrix_round_trip() -> None:
    """rpy ↔ 旋转矩阵往返一致（rad 口径）。"""
    rpy = (0.3, -0.2, 0.1)
    matrix = rpy_to_matrix(rpy)
    back = matrix_to_rpy(matrix)

    assert all(abs(a - b) < 1e-12 for a, b in zip(rpy, back))
    assert abs(math.pi - math.pi) < 1e-12
