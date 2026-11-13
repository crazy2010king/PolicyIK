"""设备配置加载与**严格校验**的测试（S03-T01）。

覆盖：

* 正例：仓库自带的 ``devices/single_arm.yaml`` 可加载，字段语义正确；
* 正例：``udp_cycle`` 的**两种读法**都给出候选周期；
* 正例：``accel_limits`` / ``current_thresholds`` 整段缺失 → 保守兜底 +
  ``measured=False`` / ``calibrated=False`` 标注（**不静默**）；
* 负例：未知键 / 缺字段 / 越界 / 维度错 / 量纲错 → :class:`DeviceConfigError`
  且消息**点名出错键名**。
"""

from __future__ import annotations

import copy

import pytest

from rm75_drake import paths
from rm75_drake.device.config import (
    CONSERVATIVE_CURRENT_THRESHOLDS_MA,
    SCHEMA_VERSION,
    DeviceConfigError,
    default_device_config_path,
    device_config_from_dict,
    load_device_config,
)
from rm75_drake.planning.trajectory import DEFAULT_ACCELERATION_LIMITS_RAD_S2


def _base_dict() -> dict:
    """返回一份**合法**的最小配置字典（负例在其上做单点破坏）。"""
    return {
        "schema_version": 1,
        "name": "rm75_single_arm",
        "arm_ip": "192.168.1.18",
        "tcp_port": 8080,
        "udp_port": 8080,
        "udp_cycle": 1,
        "joint_zero_offset": [0.0] * 7,
        "accel_limits": {"measured": False, "values": [6.0, 6.0, 8.0, 8.0, 8.0, 8.0, 8.0]},
        "current_thresholds": {
            "calibrated": False,
            "values": [1500.0, 1500.0, 1200.0, 1200.0, 1000.0, 1000.0, 1000.0],
        },
        "fence": {"table_height_m": 0.75, "singularity_radius_m": 0.15},
    }


# ── 正例 ────────────────────────────────────────────────────────────────
def test_repo_example_config_loads():
    """正例：仓库自带的示例配置可加载（部署方改 IP 即可用）。"""
    path = default_device_config_path()
    assert path == paths.devices_dir / "single_arm.yaml"
    assert path.is_file()
    config = load_device_config(path)
    assert config.schema_version == SCHEMA_VERSION
    assert config.name == "rm75_single_arm"
    assert config.arm_ip == "192.168.1.18"
    assert config.tcp_port == 8080
    assert config.udp_port == 8080
    assert config.udp_cycle == 1
    assert len(config.joint_zero_offset) == 7
    assert config.source_path == path


def test_default_config_path_is_under_devices_dir():
    """默认配置路径必须落在 paths.devices_dir（路径推导唯一）。"""
    assert default_device_config_path().parent == paths.devices_dir


def test_example_config_is_marked_unmeasured():
    """示例配置里的加速度/电流阈值必须显式标注未标定（不得冒充真值）。"""
    config = load_device_config()
    assert config.accel_limits_measured is False
    assert config.current_thresholds_calibrated is False
    assert config.accel_limits_rad_s2 == tuple(
        float(v) for v in DEFAULT_ACCELERATION_LIMITS_RAD_S2
    )


def test_udp_period_candidates_cover_both_readings():
    """udp_cycle 的两种读法都要给出（相差 5 倍，必须现场标定）。"""
    config = device_config_from_dict(_base_dict())
    five_ms_reading, millis_reading = config.udp_period_candidates_s()
    assert five_ms_reading == pytest.approx(0.005)
    assert millis_reading == pytest.approx(0.001)
    assert five_ms_reading == pytest.approx(5.0 * millis_reading)


def test_accel_limits_section_missing_falls_back_to_s02_defaults():
    """整段缺失 → 用 S02 自研默认，且标注 measured=False（未标定）。"""
    data = _base_dict()
    del data["accel_limits"]
    config = device_config_from_dict(data)
    assert config.accel_limits_rad_s2 == tuple(
        float(v) for v in DEFAULT_ACCELERATION_LIMITS_RAD_S2
    )
    assert config.accel_limits_measured is False


def test_current_thresholds_section_missing_falls_back_to_conservative():
    """整段缺失 → 用保守兜底，且标注 calibrated=False（未标定）。"""
    data = _base_dict()
    del data["current_thresholds"]
    config = device_config_from_dict(data)
    assert config.current_thresholds_ma == CONSERVATIVE_CURRENT_THRESHOLDS_MA
    assert config.current_thresholds_calibrated is False


def test_to_dict_is_json_friendly():
    """to_dict 必须可 JSON 序列化（写进运行日志）。"""
    import json

    config = device_config_from_dict(_base_dict())
    payload = json.dumps(config.to_dict(), ensure_ascii=False)
    assert "rm75_single_arm" in payload


# ── 负例：逐条点名键名 ──────────────────────────────────────────────────
def test_unknown_top_level_key_is_named():
    data = _base_dict()
    data["ip"] = "192.168.1.18"
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "ip" in str(excinfo.value)
    assert "未知键" in str(excinfo.value)


def test_missing_required_key_is_named():
    data = _base_dict()
    del data["udp_cycle"]
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "udp_cycle" in str(excinfo.value)


def test_wrong_schema_version_is_named():
    data = _base_dict()
    data["schema_version"] = 2
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "schema_version" in str(excinfo.value)


def test_invalid_ip_is_named():
    data = _base_dict()
    data["arm_ip"] = "not-an-ip"
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "arm_ip" in str(excinfo.value)


@pytest.mark.parametrize("port", [0, -1, 70000])
def test_out_of_range_port_is_named(port):
    data = _base_dict()
    data["tcp_port"] = port
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "tcp_port" in str(excinfo.value)


def test_non_integer_port_is_named():
    data = _base_dict()
    data["udp_port"] = 8080.5
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "udp_port" in str(excinfo.value)


def test_zero_udp_cycle_is_named():
    data = _base_dict()
    data["udp_cycle"] = 0
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "udp_cycle" in str(excinfo.value)


def test_wrong_length_joint_zero_offset_is_named():
    data = _base_dict()
    data["joint_zero_offset"] = [0.0] * 6
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "joint_zero_offset" in str(excinfo.value)


def test_unknown_accel_limits_key_is_named():
    data = _base_dict()
    data["accel_limits"]["measured_at"] = "2026-11-13"
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "accel_limits" in str(excinfo.value)
    assert "measured_at" in str(excinfo.value)


def test_missing_accel_limits_measured_is_named():
    data = _base_dict()
    del data["accel_limits"]["measured"]
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    message = str(excinfo.value)
    assert "accel_limits" in message
    assert "measured" in message


def test_non_positive_accel_limit_is_named():
    data = _base_dict()
    data["accel_limits"]["values"][2] = 0.0
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "accel_limits.values" in str(excinfo.value)


def test_non_boolean_measured_flag_is_named():
    data = _base_dict()
    data["accel_limits"]["measured"] = "yes"
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "accel_limits.measured" in str(excinfo.value)


def test_unknown_current_threshold_key_is_named():
    data = _base_dict()
    data["current_thresholds"]["source"] = "vendor"
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "current_thresholds" in str(excinfo.value)


def test_non_positive_current_threshold_is_named():
    data = _base_dict()
    data["current_thresholds"]["values"][0] = -1.0
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "current_thresholds.values" in str(excinfo.value)


def test_unknown_fence_key_is_named():
    data = _base_dict()
    data["fence"]["ceiling_m"] = 2.0
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "fence" in str(excinfo.value)
    assert "ceiling_m" in str(excinfo.value)


def test_missing_fence_is_named():
    data = _base_dict()
    del data["fence"]
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "fence" in str(excinfo.value)


def test_table_height_out_of_range_is_named():
    data = _base_dict()
    data["fence"]["table_height_m"] = 750.0  # 疑似填了毫米
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "fence.table_height_m" in str(excinfo.value)


def test_non_positive_singularity_radius_is_named():
    data = _base_dict()
    data["fence"]["singularity_radius_m"] = 0.0
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "fence.singularity_radius_m" in str(excinfo.value)


def test_non_finite_value_is_named():
    data = _base_dict()
    data["joint_zero_offset"][0] = float("nan")
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "joint_zero_offset" in str(excinfo.value)


def test_empty_name_is_named():
    data = _base_dict()
    data["name"] = "   "
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict(data)
    assert "name" in str(excinfo.value)


def test_non_mapping_top_level_is_named():
    with pytest.raises(DeviceConfigError) as excinfo:
        device_config_from_dict([1, 2, 3])  # type: ignore[arg-type]
    assert "映射" in str(excinfo.value)


def test_missing_file_is_reported():
    with pytest.raises(DeviceConfigError) as excinfo:
        load_device_config(paths.devices_dir / "no_such_device.yaml")
    assert "不存在" in str(excinfo.value)


def test_broken_yaml_is_reported(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: [unclosed\n", encoding="utf-8")
    with pytest.raises(DeviceConfigError) as excinfo:
        load_device_config(bad)
    assert "解析失败" in str(excinfo.value)


def test_config_is_immutable():
    """配置对象必须不可变（避免运行期被悄悄改掉）。"""
    config = device_config_from_dict(_base_dict())
    with pytest.raises(Exception):
        config.arm_ip = "10.0.0.1"  # type: ignore[misc]


def test_base_dict_is_not_mutated_by_parsing():
    """解析过程不得修改调用方传入的字典。"""
    data = _base_dict()
    snapshot = copy.deepcopy(data)
    device_config_from_dict(data)
    assert data == snapshot
