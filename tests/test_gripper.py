"""自研两指夹爪模型 + 开口量映射层 + 力标定占位的测试（S01-T04）。

覆盖计划要求的断言：

* 端点：``u = 0`` → 0 mm，``u = 1000`` → 65 mm（官方手册真值）；
* 单调性；
* 越界拒绝（``u = -1`` 与 ``u = 1001`` 抛错，**不静默截断**）；
* 两指对称：两指偏移量之和 = 总开口；单指最大行程 = 32.5 mm；
* 力未标定时抛"未标定"（异常信息含"未标定"），**不返回伪造牛顿值**；
* 夹爪 URDF 可被 Drake 加载：恰好 2 个 prismatic 位置自由度、TCP frame 存在
  且在两指中点、指尖高度；
* 夹爪 URDF 零外部资产依赖（无 mesh / 无 visual）、不使用 ``<mimic>``。

每条正向断言都配负例，杜绝"恒真绿灯"。
"""

from __future__ import annotations

import math
import pathlib
import xml.etree.ElementTree as ET

import pytest

from rm75_drake import gripper
from rm75_drake import models
from rm75_drake import package_map


# ── 端点与单调性 ──────────────────────────────────────────────────────
def test_opening_endpoints_match_manual():
    """官方手册：0–1000 无量纲，1000 ↔ 标准夹指 65 mm。"""
    assert gripper.opening_mm(0) == pytest.approx(0.0, abs=1e-12)
    assert gripper.opening_mm(1000) == pytest.approx(65.0, abs=1e-12)
    assert gripper.opening_m(1000) == pytest.approx(0.065, abs=1e-12)


def test_opening_is_monotonic():
    previous = -1.0
    for u in range(0, 1001, 25):
        current = gripper.opening_mm(u)
        assert current > previous
        previous = current


def test_opening_is_linear():
    for u in (0, 1, 137, 500, 999, 1000):
        assert gripper.opening_mm(u) == pytest.approx(
            u * 65.0 / 1000.0, abs=1e-12
        )


def test_position_from_mm_roundtrip():
    assert gripper.position_from_mm(0) == 0
    assert gripper.position_from_mm(65) == 1000
    for u in (0, 100, 500, 1000):
        mm = gripper.opening_mm(u)
        assert gripper.position_from_mm(mm) == u


# ── 越界拒绝（负例）───────────────────────────────────────────────────
@pytest.mark.parametrize("bad_u", [-1, 1001, -0.001, 1000.001])
def test_out_of_range_u_raises(bad_u):
    with pytest.raises(ValueError):
        gripper.opening_mm(bad_u)
    with pytest.raises(ValueError):
        gripper.opening_m(bad_u)
    with pytest.raises(ValueError):
        gripper.finger_positions(bad_u)


@pytest.mark.parametrize("bad_mm", [-0.1, 65.1, -1, 100])
def test_out_of_range_mm_raises(bad_mm):
    with pytest.raises(ValueError):
        gripper.position_from_mm(bad_mm)


@pytest.mark.parametrize("bad_u", ["500", None, [500], True, float("nan"), float("inf")])
def test_non_numeric_u_raises(bad_u):
    with pytest.raises(ValueError):
        gripper.opening_mm(bad_u)


def test_mapping_rejects_instead_of_silently_clamping():
    """负例导向：若实现改成截断，这里必须变红。"""
    with pytest.raises(ValueError):
        gripper.opening_mm(1001)
    # 截断版实现会返回 65.0；此处显式确认"没有静默返回边界值"。
    assert gripper.opening_mm(1000) == pytest.approx(65.0, abs=1e-12)


# ── 两指对称 ──────────────────────────────────────────────────────────
def test_finger_positions_sum_to_total_opening():
    for u in (0, 100, 500, 1000):
        left, right = gripper.finger_positions(u)
        assert left == pytest.approx(right, abs=1e-15)
        assert left + right == pytest.approx(gripper.opening_m(u), abs=1e-15)


def test_single_finger_travel_matches_half_of_max_opening():
    assert gripper.GRIPPER_FINGER_TRAVEL_MAX_M == pytest.approx(0.0325, abs=1e-12)
    left, right = gripper.finger_positions(1000)
    assert left == pytest.approx(gripper.GRIPPER_FINGER_TRAVEL_MAX_M, abs=1e-12)
    assert right == pytest.approx(gripper.GRIPPER_FINGER_TRAVEL_MAX_M, abs=1e-12)


# ── 力标定占位（未标定 → 显式抛错）────────────────────────────────────
def test_force_raises_when_not_calibrated():
    with pytest.raises(gripper.GripperNotCalibratedError) as excinfo:
        gripper.force_from_position(500)
    assert "未标定" in str(excinfo.value)


def test_force_rejects_out_of_range_before_reading_calibration():
    with pytest.raises(ValueError):
        gripper.force_from_position(1001)


def test_calibration_file_declares_not_calibrated():
    data = gripper.load_calibration()
    assert data["calibrated"] is False
    assert data["curve"] == []
    # 说明字段必须存在（"为何不可用"不能是空的）。
    assert data["method"].strip()
    assert data["unavailable_reason"].strip()
    assert data["required_equipment"]


def test_missing_calibration_file_raises(monkeypatch, tmp_path):
    """标定文件缺失必须显式报错，不能返回"看起来正常的 0 N"。"""
    monkeypatch.setattr(gripper, "calibration_path", lambda: tmp_path / "nope.yaml")
    with pytest.raises(FileNotFoundError):
        gripper.force_from_position(0)


def test_calibrated_curve_interpolates(monkeypatch, tmp_path):
    """标定完成后按曲线插值（用临时文件，不动仓库里的占位文件）。"""
    path = tmp_path / "curve.yaml"
    path.write_text(
        "calibrated: true\n"
        "curve:\n"
        "  - {u: 0, force_n: 0.0}\n"
        "  - {u: 1000, force_n: 10.0}\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(gripper, "calibration_path", lambda: path)
    assert gripper.force_from_position(500) == pytest.approx(5.0, abs=1e-9)
    assert gripper.force_from_position(0) == pytest.approx(0.0, abs=1e-9)
    assert gripper.force_from_position(1000) == pytest.approx(10.0, abs=1e-9)


def test_empty_curve_even_if_flagged_is_not_calibrated(monkeypatch, tmp_path):
    path = tmp_path / "curve.yaml"
    path.write_text("calibrated: true\ncurve: []\n", encoding="utf-8")
    monkeypatch.setattr(gripper, "calibration_path", lambda: path)
    with pytest.raises(gripper.GripperNotCalibratedError) as excinfo:
        gripper.force_from_position(0)
    assert "未标定" in str(excinfo.value)


# ── 动作调度数据结构 ──────────────────────────────────────────────────
def test_action_factories():
    open_act = gripper.open_action()
    close_act = gripper.close_action()
    pick_act = gripper.pick_action(speed=80, force=30)
    assert open_act.kind == "open" and open_act.position == 1000
    assert close_act.kind == "close" and close_act.position == 0
    assert pick_act.kind == "pick" and pick_act.position is None
    assert pick_act.force == 30 and pick_act.speed == 80


@pytest.mark.parametrize(
    "kwargs",
    [
        {"kind": "grasp"},          # 非法动作类型
        {"kind": "open", "position": 1001},  # 越界开口
        {"kind": "open", "speed": 101},      # 速度越界
        {"kind": "pick", "force": -1},       # 力档位越界
        {"kind": "open", "hold_seconds": -1.0},  # 负保持时间
    ],
)
def test_invalid_actions_raise(kwargs):
    with pytest.raises(ValueError):
        gripper.GripperAction(**kwargs)


# ── URDF 静态检查 ─────────────────────────────────────────────────────
def _gripper_urdf_root():
    pkg_map = package_map.build_package_map()
    path = package_map.resolve_package_url(pkg_map, models.GRIPPER_URDF_URL)
    return ET.parse(path).getroot(), pathlib.Path(path)


def test_urdf_has_no_mesh_and_no_visual():
    """夹爪模型必须零外部资产依赖。"""
    root, _path = _gripper_urdf_root()
    assert root.findall(".//mesh") == []
    assert root.findall(".//visual") == []
    for element in root.iter():
        assert "filename" not in element.attrib


def test_urdf_does_not_use_mimic():
    """禁止 <mimic>：Drake 解析后仍把两个 prismatic 各自计入 DoF。"""
    root, _path = _gripper_urdf_root()
    assert root.findall(".//mimic") == []


def test_urdf_collision_names_follow_convention():
    root, _path = _gripper_urdf_root()
    names = [
        element.get("name")
        for element in root.findall(".//collision")
    ]
    assert names == [
        "gripper_base_collision",
        "finger_left_collision",
        "finger_right_collision",
    ]


def test_urdf_collision_geometry_is_box_only():
    root, _path = _gripper_urdf_root()
    geometries = root.findall(".//collision/geometry")
    assert geometries, "夹爪必须有碰撞几何体"
    for geometry in geometries:
        assert geometry.find("box") is not None
        assert len(list(geometry)) == 1


# ── Drake 加载 ────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def gripper_plant():
    plant, _instance, _parser = models.load_rm75_gripper()
    plant.Finalize()
    return plant


def test_gripper_loads_with_exactly_two_prismatic_dofs(gripper_plant):
    plant = gripper_plant
    assert plant.num_positions() == 2
    assert plant.num_velocities() == 2
    types = [
        plant.get_joint(index).type_name()
        for index in plant.GetJointIndices()
        # weld 关节（world–gripper_base）与 fixed 关节（tcp_joint）都不计入 DoF。
        if plant.get_joint(index).type_name() not in ("fixed", "weld")
    ]
    assert types == ["prismatic", "prismatic"]


def test_gripper_joint_names_and_limits(gripper_plant):
    plant = gripper_plant
    limits = models.joint_limits_from_plant(plant)
    assert [item["name"] for item in limits] == list(models.GRIPPER_JOINT_NAMES)
    for item in limits:
        assert item["lower"] == pytest.approx(0.0, abs=1e-12)
        assert item["upper"] == pytest.approx(
            gripper.GRIPPER_FINGER_TRAVEL_MAX_M, abs=1e-12
        )


def test_tcp_frame_exists_at_finger_midpoint(gripper_plant):
    """TCP 位于两指中点、指尖高度。"""
    plant = gripper_plant
    context = plant.CreateDefaultContext()
    # 两指开到最大（各 0.0325 m）→ 内侧面间距 = 65 mm。
    # Drake 1.51 的 MultibodyPlant 没有 SetPosition(context, index, value)，
    # 也没有 plant.position_start()，因此用 GetPositionNames() 把关节名映射到
    # 位置向量下标（位置名形如 "RMG24_finger_joint_left_x"）。
    q = plant.GetPositions(context)
    position_names = list(plant.GetPositionNames())
    for index in plant.GetJointIndices():
        joint = plant.get_joint(index)
        if joint.type_name() != "prismatic":
            continue
        slot = next(
            i for i, name in enumerate(position_names) if joint.name() in name
        )
        q[slot] = gripper.GRIPPER_FINGER_TRAVEL_MAX_M
    plant.SetPositions(context, q)

    tcp = plant.GetFrameByName("tcp").CalcPoseInWorld(context).translation()
    left = plant.GetFrameByName("finger_left").CalcPoseInWorld(context).translation()
    right = plant.GetFrameByName("finger_right").CalcPoseInWorld(context).translation()

    # 两指 frame 的中点与 TCP 的 y 一致（对称中心线）。
    assert (left[1] + right[1]) / 2.0 == pytest.approx(tcp[1], abs=1e-12)
    assert tcp[0] == pytest.approx((left[0] + right[0]) / 2.0, abs=1e-12)
    # TCP 的 z = 指尖高度（本体 0.02 + 指长 0.04）。
    assert tcp[2] == pytest.approx(0.06, abs=1e-12)
    # 两指 frame 的间距 = 单指偏移量之和（= 总开口的最大值）。
    gap = left[1] - right[1]
    assert gap == pytest.approx(gripper.GRIPPER_OPENING_MAX_M, abs=1e-12)


def test_fingers_close_together_at_zero_position(gripper_plant):
    """u = 0 时两指 frame 重合（开口 0 mm）。"""
    plant = gripper_plant
    context = plant.CreateDefaultContext()
    left = plant.GetFrameByName("finger_left").CalcPoseInWorld(context).translation()
    right = plant.GetFrameByName("finger_right").CalcPoseInWorld(context).translation()
    assert left[1] == pytest.approx(right[1], abs=1e-12)


def test_gripper_can_be_loaded_into_same_plant_as_arm():
    """夹爪与机械臂同 plant：frame 名不冲突，且挂到法兰后 DoF = 7 + 2。"""
    plant = models.make_plant()
    models.load_rm75(plant=plant, weld=True)
    models.load_rm75_gripper(plant=plant, attach_to=models.RM75_TIP_LINK)
    plant.Finalize()
    assert plant.num_positions() == 7 + 2
    assert plant.HasFrameNamed("base_link")
    assert plant.HasFrameNamed(models.GRIPPER_BASE_LINK)
    assert plant.HasFrameNamed(models.GRIPPER_TCP_LINK)


def test_gripper_attach_to_unknown_frame_raises():
    """负例：挂载目标不存在时必须显式报错。"""
    plant = models.make_plant()
    with pytest.raises(Exception):
        models.load_rm75_gripper(plant=plant, attach_to="no_such_frame")


def test_gripper_loader_fails_loudly_when_model_missing(monkeypatch, tmp_path):
    from rm75_drake import paths

    monkeypatch.setenv(paths.MODEL_PATH_ENV_VAR, str(tmp_path))
    with pytest.raises(Exception):
        models.load_rm75_gripper()


def test_gripper_module_has_no_hardcoded_absolute_paths():
    """库内不得硬编码绝对路径。"""
    import inspect as _inspect

    source = _inspect.getsource(gripper)
    assert "/mnt/" not in source and "/home/" not in source
