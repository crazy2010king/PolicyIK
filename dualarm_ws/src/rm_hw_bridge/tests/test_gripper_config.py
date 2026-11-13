"""夹爪配置契约测试（``rm_hw_bridge.gripper_config``）。

覆盖：

* 正例：仓库内 ``workcell/rm_workcell.yaml`` 的 ``gripper:`` 段可读、
  必需字段齐全、标定字段为 ``None``（实测前不伪造）；
* 负例：缺 ``gripper:`` 段 / 缺 ``device_id`` / ``baud`` 类型错 /
  工位文件缺失 / YAML 解析失败 → ``status="blocked"`` 且 ``reason``
  **指名**缺失或非法字段；
* 标定字段为 ``null`` 时 load 仍返回 ``ok``，但 ``config["thresholds"]``
  的对应值为 ``None``（供判定层走 NEEDS_PARAM）。

只用 git 跟踪的仓库文件 + pytest 临时目录；**不改仓库文件**。
"""
from __future__ import annotations

import os
import textwrap

import pytest

from rm_hw_bridge import gripper_config, paths


# --------------------------------------------------------------------- 夹具
def _make_ws(tmp_path, yaml_text: str) -> str:
    """造一个临时工作区：含 ``workcell/rm_workcell.yaml`` 与仓库根标记目录。"""
    os.makedirs(os.path.join(str(tmp_path), "dualarm_ws", "src", "rm_description"))
    workcell = os.path.join(str(tmp_path), "workcell")
    os.makedirs(workcell)
    with open(os.path.join(workcell, "rm_workcell.yaml"), "w", encoding="utf-8") as fh:
        fh.write(textwrap.dedent(yaml_text))
    return str(tmp_path)


_VALID_GRIPPER = """
    version: 1
    gripper:
      model: RMG22-65
      power: external_24v
      protocol: modbus_rtu
      path: modbus
      device_id: 1
      baud: 115200
      timeout_ms: 500
      poll_interval_ms: 50
      action_timeout_ms: 2000
      max_regrasp: 2
      position_mm_per_unit: null
      force_scale: null
      force_threshold: null
      grasp_position_window: null
"""


@pytest.fixture(autouse=True)
def _no_rm_ws_env(monkeypatch):
    """``$RM_WS`` 会覆盖显式 ``ws`` 参数——测试里必须清掉。"""
    monkeypatch.delenv(paths.ENV_WS, raising=False)


# --------------------------------------------------------------------- 正例
def test_repo_workcell_gripper_section_loads(repo_root):
    """仓库内工位文件的 gripper 段可读，必需字段齐全，标定字段为 None。"""
    res = gripper_config.load_gripper_config(repo_root)
    assert res["status"] == "ok", res["reason"]
    cfg = res["config"]
    assert cfg["model"] == "RMG22-65"
    assert cfg["protocol"] == "modbus_rtu"
    assert cfg["path"] == "modbus"
    assert cfg["device_id"] == 1
    assert cfg["baud"] == 115200
    assert cfg["timeout_ms"] == 500
    assert cfg["poll_interval_ms"] == 50
    assert cfg["action_timeout_ms"] == 2000
    assert cfg["max_regrasp"] == 2
    # 标定字段：实测前必须是 None（不得伪造）
    assert cfg["position_mm_per_unit"] is None
    assert cfg["force_scale"] is None
    assert cfg["force_threshold"] is None
    assert cfg["grasp_position_window"] is None


def test_repo_workcell_gripper_has_all_required_fields(repo_root):
    """仓库工位文件覆盖了契约要求的全部必需字段（防止漏写）。"""
    import yaml

    with open(
        os.path.join(repo_root, "workcell", "rm_workcell.yaml"),
        "r",
        encoding="utf-8",
    ) as fh:
        section = yaml.safe_load(fh)["gripper"]
    missing = [f for f in gripper_config.REQUIRED_FIELDS if f not in section]
    assert not missing, missing
    assert section["path"] in gripper_config.PATHS


def test_result_shape_matches_executor_contract(repo_root):
    """结果字典与 ``rm_executor.contract.result_dict`` 同构。"""
    res = gripper_config.load_gripper_config(repo_root)
    for key in ("status", "reason", "next_action", "evidence", "config"):
        assert key in res, key
    assert isinstance(res["evidence"], list)


def test_thresholds_keys_present_and_none_when_uncalibrated(repo_root):
    """标定字段为 null → load 仍 ok，但 thresholds 对应值为 None（NEEDS_PARAM）。"""
    res = gripper_config.load_gripper_config(repo_root)
    assert res["status"] == "ok"
    thresholds = res["config"]["thresholds"]
    assert set(thresholds) == set(gripper_config.THRESHOLD_KEYS)
    for key in gripper_config.THRESHOLD_KEYS:
        assert thresholds[key] is None
    # 未标定清单应覆盖四个标定字段
    pending = res["config"]["calibration_pending"]
    assert set(pending) == set(gripper_config.CALIBRATION_FIELDS)


def test_thresholds_feed_judge_needs_param(repo_root):
    """未标定阈值喂给判定层 → needs_param（不产出 grasp_ok/grasp_empty）。"""
    from rm_hw_bridge import gripper_state

    res = gripper_config.load_gripper_config(repo_root)
    thresholds = res["config"]["thresholds"]
    state = gripper_state.GripperState(
        path=gripper_state.PATH_MODBUS,
        protocol_state="online",
        online=True,
        mode=gripper_state.MODE_CLOSED_MIN,
        position_mm=1.0,
        force_N=1.0,
    )
    judgment = gripper_state.judge(state, thresholds)
    assert judgment.result == "needs_param", judgment.reason
    assert judgment.severity == "NEEDS_PARAM"


def test_calibrated_values_are_parsed(tmp_path):
    """有实测标定值时按数值解析（正例：解析路径也通）。"""
    ws = _make_ws(
        tmp_path,
        """
        version: 1
        gripper:
          model: RMG22-65
          protocol: modbus_rtu
          path: gripper_api
          device_id: 2
          baud: 9600
          timeout_ms: 300
          poll_interval_ms: 40
          action_timeout_ms: 1500
          max_regrasp: 3
          position_mm_per_unit: 0.1
          force_scale: 2.5
          force_threshold: 120
          grasp_position_window: [10.0, 20.0]
        """,
    )
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "ok", res["reason"]
    cfg = res["config"]
    assert cfg["position_mm_per_unit"] == 0.1
    assert cfg["force_scale"] == 2.5
    assert cfg["force_threshold"] == 120.0
    assert cfg["grasp_position_window"] == [10.0, 20.0]
    assert cfg["calibration_pending"] == []


# --------------------------------------------------------------------- 负例
def test_missing_gripper_section_is_blocked(tmp_path):
    """缺 gripper: 段 → blocked，reason 指名。"""
    ws = _make_ws(tmp_path, "version: 1\nexecution:\n  period_s: 0.01\n")
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "gripper" in res["reason"]
    assert res["config"] is None
    assert res["next_action"]


def test_missing_device_id_is_blocked_and_named(tmp_path):
    """缺 device_id → blocked，reason 指名 device_id（代码不补默认值）。"""
    ws = _make_ws(
        tmp_path,
        """
        version: 1
        gripper:
          model: RMG22-65
          protocol: modbus_rtu
          path: modbus
          baud: 115200
          timeout_ms: 500
          poll_interval_ms: 50
          action_timeout_ms: 2000
          max_regrasp: 2
        """,
    )
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "device_id" in res["reason"]
    assert res["config"] is None
    assert "device_id" in res["next_action"]


def test_bad_baud_type_is_blocked_and_named(tmp_path):
    """baud 类型错（字符串）→ blocked，reason 指名 baud。"""
    ws = _make_ws(
        tmp_path,
        """
        version: 1
        gripper:
          model: RMG22-65
          protocol: modbus_rtu
          path: modbus
          device_id: 1
          baud: "115200"
          timeout_ms: 500
          poll_interval_ms: 50
          action_timeout_ms: 2000
          max_regrasp: 2
        """,
    )
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "baud" in res["reason"]
    assert res["config"] is None


def test_invalid_path_value_is_blocked(tmp_path):
    """path 取值不在允许集合 → blocked。"""
    text = _VALID_GRIPPER.replace("path: modbus", "path: can_open")
    ws = _make_ws(tmp_path, text)
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "path" in res["reason"]


def test_bad_calibration_type_is_blocked(tmp_path):
    """标定字段类型错（字符串）→ blocked（不是 None，是错值）。"""
    text = _VALID_GRIPPER.replace("force_threshold: null", "force_threshold: abc")
    ws = _make_ws(tmp_path, text)
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "force_threshold" in res["reason"]


def test_bad_grasp_window_is_blocked(tmp_path):
    """夹持窗口非法（元素数不对 / lo > hi）→ blocked。"""
    text = _VALID_GRIPPER.replace(
        "grasp_position_window: null", "grasp_position_window: [20.0, 10.0]"
    )
    ws = _make_ws(tmp_path, text)
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "grasp_position_window" in res["reason"]


def test_missing_workcell_file_is_blocked(tmp_path):
    """工位文件不存在 → blocked（不静默用默认值）。"""
    os.makedirs(os.path.join(str(tmp_path), "dualarm_ws", "src", "rm_description"))
    os.makedirs(os.path.join(str(tmp_path), "workcell"))
    res = gripper_config.load_gripper_config(str(tmp_path))
    assert res["status"] == "blocked"
    assert "rm_workcell.yaml" in res["reason"]
    assert res["config"] is None


def test_broken_yaml_is_blocked(tmp_path):
    """YAML 解析失败 → blocked（不抛裸异常）。"""
    ws = _make_ws(tmp_path, "version: 1\ngripper:\n  model: [unclosed\n")
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert "解析失败" in res["reason"]
    assert res["config"] is None


def test_blocked_result_has_no_fabricated_values(tmp_path):
    """blocked 结果不携带任何夹爪数值（不伪造配置）。"""
    ws = _make_ws(tmp_path, "version: 1\n")
    res = gripper_config.load_gripper_config(ws)
    assert res["status"] == "blocked"
    assert res["config"] is None
    assert res["next_action"]
