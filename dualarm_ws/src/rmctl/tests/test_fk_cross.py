"""FK 与控制器位姿交叉校验台架的测试（设计 §5.3 / R003 / R011）。

台架只依赖自研 FK（rmctl.selfkin），不 import tesseract——因此这些用例在
没有厂商库（GLIBCXX_3.4.31 缺失）的环境里也能跑。

使用的 URDF 是 **git 跟踪的** 源 URDF（dualarm_ws/src/rm_description/urdf/），
不依赖 build/model（.gitignore）。
"""
import ast
import json
import math
import os

import numpy as np
import pytest

from rmctl import fk_cross
from rmctl.fk_cross import FkCrossError


@pytest.fixture(scope="module")
def urdf(repo_root):
    """git 跟踪的源 URDF（绝对路径）。"""
    p = os.path.join(repo_root, "dualarm_ws", "src", "rm_description", "urdf",
                     "rm75_clean.urdf")
    assert os.path.exists(p), p
    return p


def _synth(urdf, **kw):
    return fk_cross.synthesize_samples(urdf_path=urdf, **kw)["samples"]


# ---------------------------------------------------------------- 正例
def test_zero_offset_synthetic_passes(urdf):
    """正例：零偏移合成样本 → 偏差在数值噪声量级（~1e-12 m），ok=True。"""
    samples = _synth(urdf, n=20, seed=42, offset_mm=0.0)
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["ok"] is True
    assert rep["samples"] == 20
    # 1e-12 m == 1e-9 mm
    assert rep["position"]["max_mm"] < 1e-9
    assert rep["position"]["status"] == "pass"
    assert rep["violations"] == []
    assert rep["orientation"]["status"] == "provided"
    assert rep["orientation"]["max_deg"] < 1e-9
    assert rep["attribution"]["pattern"] == "within_tolerance"


def test_report_is_json_serialisable_and_has_detail(urdf):
    """报告必须能直接 JSON 落盘，且含逐样本明细与偏差向量方向。"""
    samples = _synth(urdf, n=5, seed=7)
    rep = fk_cross.cross_check(samples, urdf)
    text = json.dumps(rep, ensure_ascii=False)
    back = json.loads(text)
    assert back["ok"] is True
    assert len(back["samples_detail"]) == 5
    d0 = back["samples_detail"][0]
    for key in ("index", "joint_deg", "fk_position_m", "controller_position_m",
                "dev_mm", "dev_vector_mm", "dev_direction", "pos_ok",
                "orientation"):
        assert key in d0
    assert len(d0["dev_vector_mm"]) == 3
    assert back["controller_pose_usage"].startswith("仅用于 FK 交叉校验")


# ---------------------------------------------------------------- 负例
def test_injected_5mm_offset_fails_and_names_violation(urdf):
    """负例：注入 5 mm 偏移 → ok=False，且能指出越界样本。"""
    samples = _synth(urdf, n=20, seed=42, offset_mm=5.0, offset_index=0)
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["ok"] is False
    assert rep["violations"] == [0]
    assert rep["position"]["status"] == "fail"
    assert rep["position"]["max_mm"] == pytest.approx(5.0, abs=1e-6)
    bad = rep["samples_detail"][0]
    assert bad["pos_ok"] is False
    assert abs(bad["dev_mm"] - 5.0) < 1e-6
    # 偏差向量方向：沿 x 轴（注入方向）
    assert bad["dominant_axis"] == "x"
    assert abs(bad["dev_vector_mm"][0]) == pytest.approx(5.0, abs=1e-6)
    assert abs(bad["dev_vector_mm"][1]) < 1e-9
    # 归因：单个样本越界 → 分散型（单关节/建模参数），并给出 §5.3 排查序
    assert rep["attribution"]["pattern"] == "scattered"
    assert rep["attribution"]["next_checks"] == ["zero_offset", "tool_frame", "mdh_offset"]


def test_systematic_offset_is_flagged_as_translation(urdf):
    """整体平移（零位漂移典型形态）→ systematic_translation 归因。"""
    samples = _synth(urdf, n=10, seed=3)
    for s in samples:
        s["controller_pose"]["x"] += 3.0 / 1000.0
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["ok"] is False
    assert len(rep["violations"]) == 10
    assert rep["attribution"]["pattern"] == "systematic_translation"
    assert rep["attribution"]["consistency"] == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------- 阈值边界
def test_threshold_boundary_0_9mm_passes(urdf):
    """边界：0.9 mm < 1 mm → 通过。"""
    samples = _synth(urdf, n=20, seed=42, offset_mm=0.9, offset_index=0)
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["ok"] is True
    assert rep["position"]["max_mm"] == pytest.approx(0.9, abs=1e-6)


def test_threshold_boundary_1_5mm_fails(urdf):
    """边界：1.5 mm > 1 mm → 不通过。"""
    samples = _synth(urdf, n=20, seed=42, offset_mm=1.5, offset_index=0)
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["ok"] is False
    assert rep["violations"] == [0]


def test_orientation_threshold_is_enforced(urdf):
    """姿态超限 → 判定失败（位置合格也不放过）。"""
    samples = _synth(urdf, n=3, seed=5, with_orientation=True)
    # 把第一个样本的姿态绕 z 轴偏 1°（控制器位姿与自研 FK 不一致）
    import rmctl.fk_cross as fc
    q = samples[0]
    T = fc.UrdfModel(urdf).fk(np.asarray(q["joint_deg"]) * math.pi / 180.0, "Link7")
    R = np.asarray(T[:3, :3], dtype=float)
    ang = math.radians(1.0)
    Rz = np.array([[math.cos(ang), -math.sin(ang), 0.0],
                   [math.sin(ang), math.cos(ang), 0.0],
                   [0.0, 0.0, 1.0]])
    q["controller_pose"]["quat"] = fc._matrix_to_quat(Rz @ R)
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["position"]["max_mm"] < 1e-9
    assert rep["ok"] is False
    assert rep["orientation_violations"] == [0]
    assert rep["orientation"]["max_deg"] == pytest.approx(1.0, abs=1e-6)


def test_orientation_not_provided_does_not_block(urdf):
    """控制器未提供姿态 → 姿态项标 not_provided 且不参与判定。"""
    samples = _synth(urdf, n=4, seed=11, with_orientation=False)
    rep = fk_cross.cross_check(samples, urdf)
    assert rep["orientation"]["status"] == "not_provided"
    assert rep["orientation"]["max_deg"] is None
    assert rep["ok"] is True
    assert all(d["orientation"]["state"] == "not_provided" for d in rep["samples_detail"])


# ---------------------------------------------------------------- 单位
def test_degree_and_radian_samples_agree(urdf):
    """单位例：用度构造与用弧度构造的样本，结论与偏差完全一致。"""
    deg_samples = _synth(urdf, n=6, seed=13)
    rad_samples = []
    for s in deg_samples:
        rad_samples.append({
            "joint_deg": [v * math.pi / 180.0 for v in s["joint_deg"]],
            "joint_unit": "rad",
            "controller_pose": dict(s["controller_pose"]),
        })
    r1 = fk_cross.cross_check(deg_samples, urdf)
    r2 = fk_cross.cross_check(rad_samples, urdf)
    assert r1["ok"] == r2["ok"] is True
    assert r1["position"]["max_mm"] == pytest.approx(r2["position"]["max_mm"], abs=1e-12)
    for a, b in zip(r1["samples_detail"], r2["samples_detail"]):
        assert a["dev_mm"] == pytest.approx(b["dev_mm"], abs=1e-12)
        assert a["joint_deg"] == pytest.approx(b["joint_deg"], abs=1e-9)


# ---------------------------------------------------------------- 异常
def test_missing_controller_pose_raises_clear_error(urdf):
    """样本缺 controller_pose → 明确错误，而非崩溃。"""
    samples = _synth(urdf, n=3, seed=1)
    del samples[1]["controller_pose"]
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, urdf)
    assert "controller_pose" in str(exc.value)
    assert "样本 #1" in str(exc.value)


def test_empty_samples_raises_clear_error(urdf):
    """空样本 → 明确错误。"""
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check([], urdf)
    assert "空" in str(exc.value)


def test_missing_joint_deg_raises_clear_error(urdf):
    """样本缺关节读数 → 明确错误。"""
    samples = _synth(urdf, n=2, seed=1)
    del samples[0]["joint_deg"]
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, urdf)
    assert "joint_deg" in str(exc.value)


def test_wrong_joint_count_raises_clear_error(urdf):
    """关节数不符 → 明确错误。"""
    samples = _synth(urdf, n=1, seed=1)
    samples[0]["joint_deg"] = samples[0]["joint_deg"][:3]
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, urdf)
    assert "关节数" in str(exc.value)


def test_incomplete_controller_pose_raises_clear_error(urdf):
    """controller_pose 缺位置字段 → 明确错误。"""
    samples = _synth(urdf, n=1, seed=1)
    del samples[0]["controller_pose"]["z"]
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, urdf)
    assert "位置字段" in str(exc.value)


def test_missing_urdf_raises_clear_error(urdf, tmp_path):
    """URDF 不存在 → 明确错误（不静默用别的模型）。"""
    samples = _synth(urdf, n=1, seed=1)
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, str(tmp_path / "nope.urdf"))
    assert "URDF 不存在" in str(exc.value)


# ---------------------------------------------------------------- 基线回填口径
def test_fk_recheck_fields_map_to_zero_baseline_schema(urdf):
    """报告可直接落进 zero_baseline.yaml 的 fk_recheck 字段（T07 回填用）。"""
    samples = _synth(urdf, n=20, seed=42)
    rep = fk_cross.cross_check(samples, urdf)
    fields = fk_cross.fk_recheck_fields(rep)
    assert set(["method", "samples", "max_pos_dev_mm", "max_ori_dev_deg",
                "conclusion"]) <= set(fields)
    assert fields["samples"] == 20
    assert fields["max_pos_dev_mm"] < 1e-9
    assert fields["max_ori_dev_deg"] is not None
    assert fields["conclusion"].startswith("PASS")
    assert "仅用于 FK 交叉校验" in fields["controller_pose_usage"]

    bad = fk_cross.cross_check(_synth(urdf, n=4, seed=1, offset_mm=5.0), urdf)
    bad_fields = fk_cross.fk_recheck_fields(bad)
    assert bad_fields["conclusion"].startswith("FAIL")
    assert bad_fields["max_pos_dev_mm"] == pytest.approx(5.0, abs=1e-6)


# ---------------------------------------------------------------- CLI
def test_cli_exit_codes(urdf, tmp_path, capsys):
    """CLI：判定通过 0 / 偏差超限 3 / 输入非法 2。"""
    samples_file = tmp_path / "samples.json"
    ok_samples = _synth(urdf, n=5, seed=2)
    samples_file.write_text(json.dumps(ok_samples), encoding="utf-8")
    out = tmp_path / "report.json"

    rc = fk_cross.main(["--samples", str(samples_file), "--urdf", urdf, "--out", str(out)])
    assert rc == 0
    rep = json.loads(out.read_text(encoding="utf-8"))
    assert rep["ok"] is True

    bad = _synth(urdf, n=5, seed=2, offset_mm=5.0, offset_index=1)
    samples_file.write_text(json.dumps(bad), encoding="utf-8")
    rc = fk_cross.main(["--samples", str(samples_file), "--urdf", urdf, "--out", str(out)])
    assert rc == 3

    samples_file.write_text(json.dumps([]), encoding="utf-8")
    rc = fk_cross.main(["--samples", str(samples_file), "--urdf", urdf])
    assert rc == 2

    rc = fk_cross.main(["--samples", str(tmp_path / "missing.json"), "--urdf", urdf])
    assert rc == 2


def test_cli_synthesize_mode(urdf, tmp_path):
    """CLI 合成模式：离线自测台架（不依赖真机数据）。"""
    out = tmp_path / "rep.json"
    rc = fk_cross.main(["--samples", "unused.json", "--urdf", urdf,
                        "--synthesize", "20", "--offset-mm", "5.0", "--out", str(out)])
    assert rc == 3
    rep = json.loads(out.read_text(encoding="utf-8"))
    assert rep["samples"] == 20
    assert rep["ok"] is False


# ---------------------------------------------------------------- 标定偏置
def _samples_with_ctrl_transform(urdf, n=8, seed=5, left=None, right=None):
    """合成样本：控制器位姿 = left · T_link7 · right（left/right 为 4x4）。

    ``right`` 模拟**tip 系**偏置（法兰→工具），``left`` 模拟**base 系**偏置。
    """
    model = fk_cross.UrdfModel(urdf)
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        q = []
        for j in model.rev:
            lo, hi = j["lower"], j["upper"]
            span = max(hi - lo, 1e-6)
            q.append(float(rng.uniform(lo + 0.05 * span, hi - 0.05 * span)))
        q = np.asarray(q, dtype=float)
        T = model.fk(q, "Link7")
        if left is not None:
            T = left @ T
        if right is not None:
            T = T @ right
        pose = {"x": float(T[0, 3]), "y": float(T[1, 3]), "z": float(T[2, 3])}
        pose["quat"] = fk_cross._matrix_to_quat(np.asarray(T[:3, :3], dtype=float))
        out.append({"joint_deg": [float(v) for v in q * fk_cross.RAD2DEG],
                    "controller_pose": pose})
    return out


def _translation(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def test_tip_frame_offset_is_removed_by_tool_offset(urdf):
    """tip 系固定偏置 → 施加对应 tool_offset 后残差回到 ~1e-9 mm 量级。"""
    d = np.array([0.010, -0.004, 0.002])       # 10 mm / -4 mm / 2 mm
    samples = _samples_with_ctrl_transform(urdf, n=8, seed=5, right=_translation(*d))

    raw = fk_cross.cross_check(samples, urdf)
    assert raw["ok"] is False
    assert raw["position"]["max_mm"] > 1.0
    assert raw["corrections"]["any_applied"] is False

    tool = {"x": float(d[0]), "y": float(d[1]), "z": float(d[2]),
            "frame": "tip_link7", "method": "unit-test", "source": "test:tip"}
    rep = fk_cross.cross_check(samples, urdf, tool_offset=tool)

    # 校正后残差回到数值噪声量级（1e-9 mm ≈ 1e-12 m）
    assert rep["corrections"]["corrected"]["max_pos_dev_mm"] < 1e-9
    assert rep["corrections"]["corrected"]["ok"] is True
    assert rep["ok_corrected"] is True
    # raw 侧仍记录未校正的偏差（不因施加偏置而篡改历史量）
    assert rep["corrections"]["raw"]["max_pos_dev_mm"] > 1.0
    assert rep["corrections"]["improved"] is True


def test_base_frame_offset_cannot_be_cancelled_by_tool_offset(urdf):
    """base 系偏置不能被 tip 偏置抵消（反证坐标系口径写反的风险）。"""
    samples = _samples_with_ctrl_transform(urdf, n=8, seed=9,
                                           left=_translation(0.0, 0.005, 0.0))
    tool = {"x": 0.010, "y": 0.0, "z": 0.0, "frame": "tip_link7"}
    rep = fk_cross.cross_check(samples, urdf, tool_offset=tool)
    # 施加 tip 偏置后残差仍然很大 → 口径没被写反
    assert rep["corrections"]["corrected"]["max_pos_dev_mm"] > 1.0
    assert rep["ok_corrected"] is False

    # 对照：base 系偏置能被 base_offset 正确抵消
    base = {"x": 0.0, "y": 0.005, "z": 0.0, "frame": "base"}
    rep2 = fk_cross.cross_check(samples, urdf, base_offset=base)
    assert rep2["corrections"]["corrected"]["max_pos_dev_mm"] < 1e-9
    assert rep2["ok_corrected"] is True


def test_reversed_offset_makes_residual_worse(urdf):
    """防呆：偏置方向取反 → 残差变大。"""
    d = np.array([0.010, 0.0, 0.0])
    samples = _samples_with_ctrl_transform(urdf, n=8, seed=13,
                                           right=_translation(*d))
    raw = fk_cross.cross_check(samples, urdf)
    rep = fk_cross.cross_check(samples, urdf,
                               tool_offset={"x": -float(d[0]), "y": 0.0, "z": 0.0})
    assert rep["corrections"]["corrected"]["max_pos_dev_mm"] > \
        raw["position"]["max_mm"]
    assert rep["ok_corrected"] is False


def test_corrections_echo_source_and_values(urdf):
    """report.corrections 回显偏置来源与数值。"""
    samples = _samples_with_ctrl_transform(urdf, n=4, seed=17,
                                           right=_translation(0.010, 0.0, 0.0))
    tool = {"x": 0.010, "y": 0.0, "z": 0.0, "frame": "tip_link7",
            "method": "四点标定", "source": "workcell/rm_workcell.yaml::calibration.tcp",
            "captured_at": "2026-11-20T00:00:00", "uncertainty_mm": 0.2}
    rep = fk_cross.cross_check(samples, urdf, tool_offset=tool)
    applied = rep["corrections"]["applied"]["tool_offset"]
    assert applied["x"] == pytest.approx(0.010)
    assert applied["frame"] == "tip_link7"
    assert applied["method"] == "四点标定"
    assert applied["source"].endswith("calibration.tcp")
    assert applied["captured_at"] == "2026-11-20T00:00:00"
    assert applied["uncertainty_mm"] == pytest.approx(0.2)
    assert rep["corrections"]["applied"]["base_offset"] is None


def test_fk_recheck_fields_include_corrections(urdf):
    """fk_recheck_fields 增加 corrections 段（既有键保持不变）。"""
    samples = _samples_with_ctrl_transform(urdf, n=4, seed=19,
                                           right=_translation(0.010, 0.0, 0.0))
    rep = fk_cross.cross_check(samples, urdf,
                               tool_offset={"x": 0.010, "y": 0.0, "z": 0.0})
    fields = fk_cross.fk_recheck_fields(rep)
    assert set(["method", "samples", "max_pos_dev_mm", "max_ori_dev_deg",
                "conclusion"]) <= set(fields)
    assert "corrections" in fields
    c = fields["corrections"]
    assert c["tool_offset"]["x"] == pytest.approx(0.010)
    assert c["raw_max_pos_dev_mm"] > 1.0
    assert c["corrected_max_pos_dev_mm"] < 1e-9
    assert c["improved"] is True
    assert c["ok_corrected"] is True


def test_invalid_offsets_raise_clear_error(urdf):
    """负例：非法偏置（缺字段 / frame 口径不符）→ 明确错误。"""
    samples = _synth(urdf, n=2, seed=1)
    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, urdf, tool_offset={"x": 0.01})
    assert "tool_offset" in str(exc.value)

    with pytest.raises(FkCrossError) as exc:
        fk_cross.cross_check(samples, urdf, base_offset={"x": 0.0, "y": 0.0,
                                                        "z": 0.0, "frame": "tip_link7"})
    assert "frame" in str(exc.value)


def test_cli_accepts_offset_string_and_calibration_file(urdf, tmp_path):
    """CLI：--tool-offset 字符串 / --base-offset / --calibration 都能生效。"""
    samples_file = tmp_path / "samples.json"
    samples = _samples_with_ctrl_transform(urdf, n=6, seed=23,
                                           right=_translation(0.010, 0.0, 0.0))
    samples_file.write_text(json.dumps(samples), encoding="utf-8")
    out = tmp_path / "rep.json"

    rc = fk_cross.main(["--samples", str(samples_file), "--urdf", urdf,
                        "--tool-offset", "0.010,0,0", "--out", str(out)])
    assert rc == 3          # raw 仍超限（判定看 raw）
    rep = json.loads(out.read_text(encoding="utf-8"))
    assert rep["ok_corrected"] is True
    assert rep["corrections"]["applied"]["tool_offset"]["source"] == "cli:--tool-offset"

    # --calibration 直接读 workcell yaml 的 calibration 段
    wc = tmp_path / "rm_workcell.yaml"
    wc.write_text(
        "calibration:\n"
        "  tcp:\n"
        "    frame: tip_link7\n"
        "    x: 0.010\n"
        "    y: 0.0\n"
        "    z: 0.0\n"
        "    source: unit-test\n"
        "  base_to_base: null\n", encoding="utf-8")
    rc = fk_cross.main(["--samples", str(samples_file), "--urdf", urdf,
                        "--calibration", str(wc), "--out", str(out)])
    assert rc == 3
    rep = json.loads(out.read_text(encoding="utf-8"))
    assert rep["ok_corrected"] is True
    assert rep["corrections"]["applied"]["tool_offset"]["source"] == "unit-test"
    assert rep["corrections"]["applied"]["base_offset"] is None

    # 非法偏置 → 退出码 2（输入非法）
    rc = fk_cross.main(["--samples", str(samples_file), "--urdf", urdf,
                        "--tool-offset", "1,2"])
    assert rc == 2


# ---------------------------------------------------------------- 约束
def _module_ast():
    import ast
    path = os.path.join(os.path.dirname(fk_cross.__file__), "fk_cross.py")
    with open(path, encoding="utf-8") as fh:
        return ast.parse(fh.read())


def test_module_does_not_import_vendor_solver():
    """禁止 import tesseract（厂商解算）——否则无 GLIBCXX 环境无法被 pytest 覆盖。

    用 AST 判断真实 import（文档字符串里出现 "import tesseract" 只是禁令说明）。
    """
    imported = set()
    for node in ast.walk(_module_ast()):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add(a.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    assert "tesseract" not in imported
    assert "tesseract_robotics" not in imported
    # 只允许依赖自研运动学 + 标准库 + numpy（calibration 为本包内的标定口径模块）
    assert imported <= {"argparse", "json", "math", "os", "sys", "numpy",
                         "paths", "selfkin", "fk_cross", "calibration"}


def test_module_has_no_controller_pose_writeback_path():
    """控制器位姿不得写回轨迹/模型/基线（R003）：源码里不得出现写回调用。"""
    import ast
    called = set()
    for node in ast.walk(_module_ast()):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute):
                called.add(f.attr)
            elif isinstance(f, ast.Name):
                called.add(f.id)
    for forbidden in ("set_state", "save_traj", "set_joints", "safe_dump",
                      "dump", "write_text"):
        assert forbidden not in called, forbidden
    # 只允许写出一份报告（--out）：open 只出现读样本 + 写报告两处
    src = open(os.path.join(os.path.dirname(fk_cross.__file__), "fk_cross.py"),
               encoding="utf-8").read()
    assert src.count("open(") == 2  # _load_samples 读 + 报告写
    # 标定口径模块同样只读不写（不得写回轨迹/模型/基线）
    cal_src = open(os.path.join(os.path.dirname(fk_cross.__file__), "calibration.py"),
                   encoding="utf-8").read()
    for forbidden in ("safe_dump", "write_text", "save_traj", "set_joints"):
        assert forbidden not in cal_src, forbidden
    assert ast is not None
