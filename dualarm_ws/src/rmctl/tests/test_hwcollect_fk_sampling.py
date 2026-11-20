"""hwcollect 的 FK 采样链路测试（离线，不碰真机）。

覆盖 S11-T3 的三个核心断言：

* **采样不再重复同一位形**：目标位形互不相同、幅度有界、每步测完回原位；
* **控制器姿态进入样本**：姿态项不再恒为 ``not_provided``；
* **标定偏置接入 cross_check**：``calibration.tcp`` / ``base_to_base``
  从工位配置读到并传给 ``fk_cross.cross_check``。

用假 link stub（不接 SDK）驱动真机路径；dry-run 路径断言"合成样本"标注。
"""
import importlib.util
import json
import os
import sys

import numpy as np
import pytest

from rmctl import fk_cross
from rmctl import hwcollect

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "..", "..", ".."))
LADDER_PATH = os.path.join(REPO_ROOT, "scripts", "rm75_validation_ladder.py")


def _load_ladder():
    spec = importlib.util.spec_from_file_location("rm75_validation_ladder_t3",
                                                  LADDER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def ladder():
    """阶梯编排脚本（按路径加载，脚本不是包）。"""
    return _load_ladder()


@pytest.fixture(scope="module")
def urdf(repo_root):
    """git 跟踪的源 URDF（绝对路径）。"""
    p = os.path.join(repo_root, "dualarm_ws", "src", "rm_description", "urdf",
                     "rm75_clean.urdf")
    assert os.path.exists(p), p
    return p


# ------------------------------------------------------------------ 假 link
class FakeBinding(object):
    """假 SDK binding：只实现 ``rm_get_joint_degree``。"""

    def __init__(self, owner):
        self.owner = owner

    def rm_get_joint_degree(self, handle, buf):
        for i, v in enumerate(self.owner.current_deg):
            buf[i] = float(v)
        return 0


class FakeBridge(object):
    def __init__(self, owner):
        self.binding = FakeBinding(owner)


class FakeLink(object):
    """假机械臂句柄：能读关节角、能预就位、能出快照。"""

    def __init__(self, urdf_path, base_deg=None):
        from rmctl.selfkin import UrdfModel
        self.urdf = UrdfModel(urdf_path)
        self.base_deg = list(base_deg or [0.0] * 7)
        self.current_deg = list(self.base_deg)
        self.bridge = FakeBridge(self)
        self.handle = object()
        self.moves = []
        self.closed = False

    def preposition_movej(self, deg7, speed=10, radius=0, block=0,
                          trajectory_connect=0):
        self.moves.append({"goal": [float(v) for v in deg7], "speed": speed,
                           "block": block})
        self.current_deg = [float(v) for v in deg7]
        return {"status": "ok"}

    def snapshot(self):
        q = np.radians(np.asarray(self.current_deg, dtype=float))
        T = self.urdf.fk(q, "Link7")
        pos = T[:3, 3]
        quat = fk_cross._matrix_to_quat(np.asarray(T[:3, :3], dtype=float))
        return {"status": "ok", "snapshot": {"controller_pose": {
            "position": {"x": float(pos[0]), "y": float(pos[1]),
                         "z": float(pos[2]), "unit": "m"},
            "quaternion": {"w": quat[3], "x": quat[0], "y": quat[1], "z": quat[2]},
            "euler": {"rx": 0.0, "ry": 0.0, "rz": 0.0, "unit": "rad"},
        }}}

    def close(self):
        self.closed = True


def _workcell_with_calibration(tmp_path, tcp=True):
    """写一个带标定段的工位配置（返回 ws 根目录）。"""
    ws = tmp_path / "ws"
    (ws / "workcell").mkdir(parents=True)
    cal = {"frame": "tip_link7", "x": 0.001, "y": 0.0, "z": -0.0005,
           "rpy": [0.0, 0.0, 0.0], "method": "unit-test", "source": "pytest",
           "captured_at": "2026-11-20T00:00:00Z", "uncertainty_mm": 0.2}
    body = {
        "calibration": {
            "tcp": cal if tcp else None,
            "base_to_base": None,
        }
    }
    (ws / "workcell" / "rm_workcell.yaml").write_text(
        yaml_dump(body), encoding="utf-8")
    return str(ws)


def yaml_dump(data):
    import yaml
    return yaml.safe_dump(data, allow_unicode=True, sort_keys=False)


# ------------------------------------------------------------------ 采样计划
def test_sampling_plan_targets_are_distinct_and_bounded():
    """目标位形互不相同、幅度受限、只动一个关节。"""
    base = [0.0] * 7
    plan = hwcollect.fk_sampling_plan(base, n=12, amplitude_deg=5.0, seed=7)
    assert len(plan) == 12
    keys = {tuple(step["target_deg"]) for step in plan}
    assert len(keys) == 12, "目标位形必须互不相同"
    for step in plan:
        assert abs(step["delta_deg"]) <= 5.0 + 1e-9
        changed = [i for i in range(7)
                   if abs(step["target_deg"][i] - base[i]) > 1e-9]
        assert len(changed) == 1, "每次只允许动一个关节"
        assert changed[0] == step["joint_index"]


def test_sampling_plan_respects_joint_limits():
    """位形必须落在关节限位内（超限即夹回）。"""
    base = [0.0] * 7
    limits = [(-10.0, 10.0)] * 7
    plan = hwcollect.fk_sampling_plan(base, n=10, amplitude_deg=5.0, seed=3,
                                      limits=limits)
    for step in plan:
        for value in step["target_deg"]:
            assert -10.0 <= value <= 10.0


def test_sampling_plan_does_not_cross_home_neighbourhood():
    """不越过 home 邻域：目标必须留在 base 所在的一侧。"""
    base = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    plan = hwcollect.fk_sampling_plan(base, n=8, amplitude_deg=5.0, seed=11,
                                      home_deg=[0.0] * 7)
    for step in plan:
        assert step["target_deg"][0] >= 0.0


def test_sampling_plan_rejects_oversized_amplitude():
    """幅度超过硬上限直接报错（安全边界不得放宽）。"""
    with pytest.raises(hwcollect.HwCollectError):
        hwcollect.fk_sampling_plan([0.0] * 7, n=4, amplitude_deg=6.0)


def test_sampling_plan_is_reproducible_with_seed():
    """同 seed → 同计划（可复现）。"""
    a = hwcollect.fk_sampling_plan([0.0] * 7, n=8, amplitude_deg=5.0, seed=5)
    b = hwcollect.fk_sampling_plan([0.0] * 7, n=8, amplitude_deg=5.0, seed=5)
    assert [s["target_deg"] for s in a] == [s["target_deg"] for s in b]


# ------------------------------------------------------------------ 真机路径
def test_hardware_path_samples_are_distinct_and_return_to_origin(urdf):
    """真机路径：目标位形互不相同、每步测完回原位。"""
    link = FakeLink(urdf)
    res = hwcollect.fk_cross_experiment(arm="left", n=6, seed=42, link=link,
                                        urdf_path=urdf)
    assert res["status"] == hwcollect.STATUS_OK
    targets = [s["commanded_joint_deg"] for s in res["samples"]]
    assert len({tuple(t) for t in targets}) == len(targets)
    # 每个目标步之后必有一次回原位（移动次数 = 2 × 样本数）
    assert len(link.moves) == 2 * len(res["samples"])
    for i, move in enumerate(link.moves):
        if i % 2 == 1:
            assert move["goal"] == pytest.approx(link.base_deg)
        assert move["speed"] <= hwcollect.FK_MAX_SPEED_PCT


def test_hardware_path_samples_carry_controller_orientation(urdf):
    """控制器姿态必须进入样本（姿态项不再恒为 not_provided）。"""
    link = FakeLink(urdf)
    res = hwcollect.fk_cross_experiment(arm="left", n=6, seed=42, link=link,
                                        urdf_path=urdf)
    for sample in res["samples"]:
        pose = sample["controller_pose"]
        assert set(pose) >= {"x", "y", "z"}
        assert "quat" in pose or "rpy" in pose
    summary = res["summary"]
    assert summary["raw"]["max_ori_dev_deg"] is not None
    assert summary["corrected"]["max_ori_dev_deg"] is not None


def test_hardware_path_reports_sampling_metadata(urdf):
    """summary 必须带采样元数据（distinct 位形数 / 幅度 / 参与关节 / 回原位）。"""
    link = FakeLink(urdf)
    res = hwcollect.fk_cross_experiment(arm="left", n=6, seed=42, link=link,
                                        urdf_path=urdf)
    sampling = res["summary"]["sampling"]
    assert sampling["distinct_configs"] == 6
    assert sampling["amplitude_deg"] == pytest.approx(5.0)
    assert sampling["joints"]
    assert sampling["return_to_origin"] is True
    assert sampling["source"] == "hardware"
    assert res["summary"]["residual_kind"]


def test_hardware_path_passes_calibration_offsets_to_cross_check(urdf, tmp_path,
                                                                  monkeypatch):
    """标定偏置必须传给 fk_cross.cross_check（raw / corrected 两组指标）。"""
    ws = _workcell_with_calibration(tmp_path, tcp=True)
    captured = {}
    real = fk_cross.cross_check

    def spy(samples, urdf_path=None, *args, **kwargs):
        captured.update(kwargs)
        return real(samples, urdf_path, *args, **kwargs)

    monkeypatch.setattr(hwcollect.fk_cross, "cross_check", spy)
    link = FakeLink(urdf)
    res = hwcollect.fk_cross_experiment(arm="left", n=6, seed=42, link=link,
                                        urdf_path=urdf, ws=ws)
    assert captured.get("tool_offset") is not None
    assert captured["tool_offset"]["frame"] == "tip_link7"
    assert captured.get("base_offset") is None
    assert res["summary"]["calibration"]["tcp"] is True
    assert "raw" in res["summary"] and "corrected" in res["summary"]


def test_hardware_path_without_calibration_uses_none_offsets(urdf, tmp_path,
                                                             monkeypatch):
    """未标定 → 两个偏置都是 None（不得用默认值顶替）。"""
    ws = _workcell_with_calibration(tmp_path, tcp=False)
    captured = {}
    real = fk_cross.cross_check

    def spy(samples, urdf_path=None, *args, **kwargs):
        captured.update(kwargs)
        return real(samples, urdf_path, *args, **kwargs)

    monkeypatch.setattr(hwcollect.fk_cross, "cross_check", spy)
    link = FakeLink(urdf)
    res = hwcollect.fk_cross_experiment(arm="left", n=6, seed=42, link=link,
                                        urdf_path=urdf, ws=ws)
    assert captured.get("tool_offset") is None
    assert captured.get("base_offset") is None
    assert res["summary"]["calibration"]["tcp"] is False


def test_dry_run_marks_synthetic_source(urdf):
    """dry-run 也走新采样器，并标注"合成样本，非真机位姿"。"""
    res = hwcollect.fk_cross_experiment(arm="left", n=6, seed=42, dry_run=True,
                                        urdf_path=urdf)
    assert res["status"] == hwcollect.STATUS_DRY_RUN
    assert res["summary"]["sampling"]["source"] == "synthetic"
    assert "合成" in res["summary"]["note"]
    assert "非真机" in res["summary"]["note"]
    kinds = {e.get("kind") for e in res["evidence"]}
    assert "synthetic" in kinds
    # dry-run 样本同样互不相同（不是同一位形重复）
    targets = [tuple(s["joint_deg"]) for s in res["samples"]]
    assert len(set(targets)) == len(targets)


def test_cli_exposes_fk_sampling_arguments(urdf, tmp_path, capsys):
    """CLI 必须能调 --fk-amplitude-deg / --fk-joints。"""
    out = tmp_path / "out"
    code = hwcollect.main(["fk_cross", "--dry-run", "--samples", "6",
                           "--fk-amplitude-deg", "3",
                           "--fk-joints", "0,1", "--out", str(out)])
    assert code == 0
    payload = json.load(open(str(out / "fk_cross.left.json"), encoding="utf-8"))
    assert payload["params"]["amplitude_deg"] == pytest.approx(3.0)
    assert payload["params"]["joints"] == [0, 1]
    assert payload["summary"]["sampling"]["amplitude_deg"] == pytest.approx(3.0)
    assert payload["summary"]["sampling"]["joints"] == ["joint1", "joint2"]


def test_cli_rejects_invalid_fk_joints(urdf, tmp_path):
    """非法 --fk-joints → 退出码 2（输入非法）。"""
    code = hwcollect.main(["fk_cross", "--dry-run", "--samples", "4",
                           "--fk-joints", "abc",
                           "--out", str(tmp_path / "out2")])
    assert code == 2


# ------------------------------------------------------------------ 阶梯分流
def _ladder_ctx(ladder, tmp_path, summary):
    """构造一个 V6 只跑、hwcollect 返回给定 summary 的上下文。"""

    class _Link(object):
        def close(self):
            pass

    return ladder.LadderContext(
        out_dir=str(tmp_path / "validation"),
        link_provider=lambda: (_Link(), None),
        probe_runner=lambda **kw: {"status": "ok", "reason": "",
                                   "evidence": [], "phases": []},
        hwcollect_runner=lambda name, **kw: {"status": "ok", "reason": "",
                                             "summary": summary},
        subprocess_runner=lambda args, cwd=None: (0, "", ""),
        executor_runner=lambda **kw: {"status": "ok", "metrics": {}})


def _v6(ladder, tmp_path, summary):
    report = ladder.run_ladder(_ladder_ctx(ladder, tmp_path, summary),
                               stage_ids=["V6"])
    return report["stages"][0]


def test_v6_insufficient_diversity_is_blocked_not_failed(ladder, tmp_path):
    """样本多样性不足 → blocked（不是 failed）：样本不足不是“不合格”。"""
    stage = _v6(ladder, tmp_path, {
        "samples": 20, "max_pos_dev_mm": 0.4, "ok": False,
        "residual_kind": "insufficient_diversity",
        "attribution": {"pattern": "within_tolerance",
                        "residual_kind": "insufficient_diversity",
                        "diversity": {"sufficient": False,
                                      "reasons": ["distinct 位形数 1 < 3"]}},
    })
    assert stage["status"] == ladder.STATUS_BLOCKED
    assert "多样性不足" in stage["reason"]
    assert "同一位形重复采样不能判定偏置性质" in stage["reason"]


def test_v6_insufficient_diversity_blocks_even_when_position_within_tolerance(ladder,
                                                                          tmp_path):
    """多样性不足 **优先于** summary.ok：不能对偏置性质下结论就不能结案。

    口径决定：V6 的目的是**归因**，而"同一位形重复采样"得到的 within_tolerance
    只能说明那一位形上残差小，不能说明模型对；因此多样性不足一律 blocked。
    """
    stage = _v6(ladder, tmp_path, {
        "samples": 20, "max_pos_dev_mm": 0.4, "ok": True,
        "residual_kind": "insufficient_diversity",
        "attribution": {"pattern": "within_tolerance",
                        "residual_kind": "insufficient_diversity",
                        "diversity": {"sufficient": False,
                                      "reasons": ["姿态展布 0.0000° < 0.5000°"]}},
    })
    assert stage["status"] == ladder.STATUS_BLOCKED
    assert "多样性不足" in stage["reason"]


def test_v6_tool_frame_offset_points_to_calibration_tcp(ladder, tmp_path):
    """residual_kind=tool_frame_offset 且未标定 → 提示先标 calibration.tcp。"""
    stage = _v6(ladder, tmp_path, {
        "samples": 10, "max_pos_dev_mm": 75.0, "ok": False,
        "residual_kind": "tool_frame_offset",
        "attribution": {"pattern": "scattered",
                        "residual_kind": "tool_frame_offset"},
    })
    assert stage["status"] == ladder.STATUS_FAILED
    assert "calibration.tcp" in stage["reason"]
    assert "先标" in stage["reason"]


def test_v6_base_frame_offset_forbids_writing_tcp(ladder, tmp_path):
    """residual_kind=base_frame_offset → 提示查 URDF/基座，且禁止写 tcp。"""
    stage = _v6(ladder, tmp_path, {
        "samples": 10, "max_pos_dev_mm": 12.0, "ok": False,
        "residual_kind": "base_frame_offset",
        "attribution": {"pattern": "systematic_translation",
                        "residual_kind": "base_frame_offset"},
    })
    assert stage["status"] == ladder.STATUS_FAILED
    assert "base_to_base" in stage["reason"]
    assert "禁止" in stage["reason"]


def test_v6_orientation_dependent_points_to_zero_and_mdh(ladder, tmp_path):
    """residual_kind=orientation_dependent → 提示零位与 MDH。"""
    stage = _v6(ladder, tmp_path, {
        "samples": 10, "max_pos_dev_mm": 5.0, "ok": False,
        "residual_kind": "orientation_dependent",
        "attribution": {"pattern": "scattered",
                        "residual_kind": "orientation_dependent"},
    })
    assert stage["status"] == ladder.STATUS_FAILED
    assert "零位" in stage["reason"] and "MDH" in stage["reason"]


def test_v6_tool_calibration_returns_structured_dict(ladder, tmp_path):
    """tool_calibration() 改为新 schema：返回 tcp / base_to_base 两个键。"""
    ctx = ladder.LadderContext(out_dir=str(tmp_path / "v"))
    cal = ctx.tool_calibration()
    assert set(["tcp", "base_to_base"]).issubset(set(cal))
    # 工位配置里两处未标定 → 都是 None（不得填占位值）
    assert cal["tcp"] is None
    assert cal["base_to_base"] is None
