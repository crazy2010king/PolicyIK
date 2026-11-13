"""FK 回归测试（S01-T03）：官方 MDH 表 vs Drake FK。

被测对象
--------
* :mod:`rm75_drake.kinematics.mdh` —— 纯 numpy 的独立正解（**零 Drake 依赖**）；
* :mod:`rm75_drake.models` + :mod:`rm75_drake.drake_compat` —— Drake 载入改造后
  URDF 后的 FK 真值。

关于"位置误差"阈值的说明（重要，勿随意放宽）
----------------------------------------------
本切片实测出一个**必须显式记录**的事实：

* 改造后的 URDF 用的是官方 **rpy 舍入值 1.5708**（而非精确 ±π/2）；
* 经典 D-H 链（Rotz·Transz·Rotx）在 ±π/2 **精确**时与 URDF 链**完全等价**
  （残差 ~1e-16 m，见 ``test_exact_pi2_urdf_matches_mdh``）；
* 但把两侧都换成舍入值 1.5708 时，两条链出现 **≈2.24e-6 m** 的位置残差
  （经典 D-H 把 d 沿"关节转过之后的 z 轴"平移，URDF 把偏移放在父系 y 上，
  只有在 ±90° 精确成立时两者才重合）。

因此 **Drake 载入的 FK 相对精确模型本身就偏 2.24e-6 m**（零位 link_7 的
y 偏差实测 2.24065511e-06 m）。任何独立解析模型都不可能在 1e-6 m 以内与
"用舍入 rpy 的官方 URDF"对齐。

本文件因此把残差拆成两条**可判定**的断言：

1. **对着真实 URDF（舍入 rpy）**：位置误差 < 5e-6 m、姿态误差 < 1e-9 rad，
   并单独断言零位 y 偏差落在 ``2.2407e-6 ± 1e-8``（把残差钉成**已知量**，
   而不是未知误差）；
2. **对着"精确 ±π/2 的 URDF 变体"**（测试内生成的临时文件）：位置误差
   < 1e-6 m、姿态误差 < 1e-9 rad —— 这是计划要求的 1e-6 m 阈值在
   **去掉官方舍入后**的真实达成情况，用它证明约定本身没有错。

负例（鉴别力）：用错误 alpha 表（全 0）时与 Drake 的差必须 > 1e-3 m，
证明这套比对不是恒真。
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from rm75_drake.kinematics import mdh
from rm75_drake.models import eval_link_pose_in_world, load_rm75

REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "models" / "rm75_drake" / "rm75_drake.urdf"

#: 零位 link_7 的 y 偏差（m）—— 官方 rpy=1.5708 舍入的**已知量**。
EXPECTED_ZERO_Y_ROUNDING_BIAS = 2.2407e-06

#: 零位 y 偏差的允许带宽（m）。取值远大于数值噪声，但远小于残差本身，
#: 因此"残差来源被钉死"这件事本身是可判定的。
ZERO_Y_BIAS_TOLERANCE = 1e-8

#: 随机位形的数量（计划要求 100 组）。
N_RANDOM_CONFIGS = 100

#: 随机位形的种子（固定，保证回归可复现）。
RANDOM_SEED = 20260113


# ── fixtures ────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def drake_plant():
    """装配好的改造版 RM75（weld 基座，DoF=7），返回 ``(plant, context)``。"""
    from pydrake.multibody.plant import MultibodyPlant

    plant = MultibodyPlant(0.0)
    plant, _index, _parser = load_rm75(plant, weld=True)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context


def _drake_link7_pose(plant, context, q):
    """返回 Drake FK 的 link_7 位姿（位置数组 + 旋转矩阵）。"""
    plant.SetPositions(context, np.asarray(q, dtype=float))
    X = eval_link_pose_in_world(plant, context, "link_7")
    return np.asarray(X.translation()), np.asarray(X.rotation().matrix())


def _random_configs(count=N_RANDOM_CONFIGS, seed=RANDOM_SEED):
    """固定种子的随机位形（q_i ∈ [-1, 1] rad）。"""
    rng = np.random.default_rng(seed)
    return [rng.uniform(-1.0, 1.0, mdh.N_JOINTS) for _ in range(count)]


# ── 零 Drake 依赖 ───────────────────────────────────────────────────────
def test_mdh_module_has_no_drake_dependency():
    """MDH 模块不得 import pydrake（否则就不是"独立对照"）。"""
    source = Path(mdh.__file__).read_text(encoding="utf-8")
    offenders = [
        line.strip()
        for line in source.splitlines()
        if re.search(r"^\s*(import|from)\s+pydrake", line)
    ]
    assert offenders == [], f"mdh.py 出现了 Drake 导入：{offenders}"
    # 也不能通过 rm75_drake 包绕回 Drake（models/drake_compat 会拉起 pydrake）
    offenders = [
        line.strip()
        for line in source.splitlines()
        if re.search(r"^\s*(import|from)\s+rm75_drake", line)
    ]
    assert offenders == [], f"mdh.py 绕回了 rm75_drake：{offenders}"


def test_kinematics_package_import_does_not_load_pydrake():
    """import rm75_drake.kinematics 不应把 pydrake 拉进 sys.modules。"""
    import subprocess
    import sys

    code = (
        "import sys; sys.path.insert(0, %r);"
        "import rm75_drake.kinematics;"
        "print('pydrake' in sys.modules)" % str(REPO_ROOT / "src")
    )
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.strip().endswith("False"), out.stdout


# ── 参数表与零位锚点 ────────────────────────────────────────────────────
def test_parameter_table_shape_and_values():
    """MDH 表必须是 7 维，且数值等于官方参数表。"""
    assert mdh.N_JOINTS == 7
    assert len(mdh.MDH_D) == 7
    assert len(mdh.MDH_A) == 7
    assert len(mdh.MDH_ALPHA) == 7
    assert mdh.MDH_D == (0.2405, 0.0, 0.256, 0.0, 0.210, 0.0, 0.144)
    assert mdh.MDH_A == (0.0,) * 7
    assert mdh.MDH_ALPHA == (
        -1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0.0
    )


def test_zero_height_constant_matches_table():
    """MDH_ZERO_HEIGHT_M 必须等于 d1+d3+d5+d7（评审记录 E9 的锚点）。"""
    assert mdh.MDH_ZERO_HEIGHT_M == pytest.approx(0.8505, abs=1e-12)
    total = mdh.MDH_D[0] + mdh.MDH_D[2] + mdh.MDH_D[4] + mdh.MDH_D[6]
    assert mdh.MDH_ZERO_HEIGHT_M == pytest.approx(total, abs=1e-12)


def test_zero_pose_anchor():
    """零位锚点：fk(zeros) 的 z == 0.8505 m、x == 0。"""
    T = mdh.fk(np.zeros(7))
    assert T[2, 3] == pytest.approx(mdh.MDH_ZERO_HEIGHT_M, abs=1e-9)
    assert T[0, 3] == pytest.approx(0.0, abs=1e-12)
    # 零位姿态 = 单位阵（±90° 扭角成对抵消）
    assert np.allclose(T[:3, :3], np.eye(3), atol=1e-12)


def test_urdf_rounding_cause_of_residual():
    """官方 URDF 的 rpy 舍入（1.5708 vs ±π/2）就是残差来源：两者不等。"""
    text = URDF_PATH.read_text(encoding="utf-8")
    assert "1.5708" in text, "URDF 应保留官方 rpy 舍入值 1.5708"
    # 精确 π/2 与舍入值之差 ~3.67e-6 rad —— 乘上 ~0.6 m 的力臂即 ~2.2e-6 m，
    # 正好解释实测到的位置残差量级。
    assert abs(mdh.MDH_ALPHA[1] - mdh.MDH_ALPHA_EXACT_PI_2[1]) == pytest.approx(
        3.6732e-06, abs=1e-9
    )
    # 舍入偏差必须真实存在（否则本文件对残差的解释就失去依据）
    assert mdh.MDH_ALPHA[1] != mdh.MDH_ALPHA_EXACT_PI_2[1]


# ── 与 Drake FK 的比对 ──────────────────────────────────────────────────
def test_random_configs_match_drake(drake_plant):
    """100 组随机位形：位置误差 < 5e-6 m、姿态误差 < 1e-9 rad。

    位置阈值为 5e-6 m 而非 1e-6 m，原因是官方 URDF 自身的 rpy 舍入
    （见模块 docstring）。该残差的下界由 ``test_zero_pose_rounding_bias``
    单独钉死，因此不会被"悄悄放宽"。
    """
    plant, context = drake_plant
    max_pos = 0.0
    max_rot = 0.0
    for q in _random_configs():
        T = mdh.fk(q)
        p_drake, R_drake = _drake_link7_pose(plant, context, q)
        max_pos = max(max_pos, float(np.linalg.norm(T[:3, 3] - p_drake)))
        max_rot = max(max_rot, float(np.linalg.norm(T[:3, :3] - R_drake)))
    assert max_rot < 1e-9, f"姿态误差过大：{max_rot:.3e} rad"
    assert max_pos < 5e-6, f"位置误差过大：{max_pos:.3e} m（舍入残差理论上限 ~2.3e-6）"


def test_zero_pose_rounding_bias(drake_plant):
    """零位 link_7 的 y 偏差必须是官方舍入的**已知量**（不是未知误差）。"""
    plant, context = drake_plant
    p_drake, _ = _drake_link7_pose(plant, context, np.zeros(7))
    p_mdh = mdh.link7_position(np.zeros(7))
    assert p_drake[1] - p_mdh[1] == pytest.approx(
        EXPECTED_ZERO_Y_ROUNDING_BIAS, abs=ZERO_Y_BIAS_TOLERANCE
    )
    # z 必须精确对齐（舍入只影响水平方向）
    assert p_drake[2] == pytest.approx(mdh.MDH_ZERO_HEIGHT_M, abs=1e-6)
    assert p_drake[0] == pytest.approx(0.0, abs=1e-12)


# ── 约定等价性：精确 ±π/2 的 URDF 变体 ──────────────────────────────────
def _write_exact_pi2_urdf(tmp_path: Path) -> Path:
    """把官方 URDF 的 rpy 舍入值 1.5708 换成精确 ±π/2，生成临时 URDF。

    这是**测试内的临时夹具**（不写入仓库、不依赖 gitignore 路径）。
    """
    tree = ET.parse(URDF_PATH)
    root = tree.getroot()
    exact = repr(np.pi / 2)
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        if origin is None:
            continue
        rpy = origin.get("rpy")
        if not rpy:
            continue
        parts = rpy.split()
        if abs(abs(float(parts[0])) - 1.5708) < 1e-12:
            sign = "-" if float(parts[0]) < 0 else ""
            parts[0] = f"{sign}{exact}"
            origin.set("rpy", " ".join(parts))
    out = tmp_path / "rm75_exact_pi2.urdf"
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return out


def test_exact_pi2_urdf_matches_mdh(drake_plant, tmp_path):
    """去掉官方舍入后：MDH 与 Drake 的位置误差 < 1e-6 m、姿态误差 < 1e-9 rad。

    这一条证明**经典 D-H 链与 URDF 链的约定本身完全等价**，残差确实只来自
    官方 rpy 的舍入，而不是参数表写错。
    """
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant

    urdf = _write_exact_pi2_urdf(tmp_path)

    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    indices = parser.AddModels(file_name=str(urdf))
    assert len(indices) == 1
    # 与真实模型同构：weld 基座（否则基座自由 → DoF=14，位姿定义也不同）
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))
    plant.Finalize()
    assert plant.num_positions() == 7
    context = plant.CreateDefaultContext()

    max_pos = 0.0
    max_rot = 0.0
    for q in _random_configs():
        T = mdh.fk(q, alpha=mdh.MDH_ALPHA_EXACT_PI_2)
        plant.SetPositions(context, np.asarray(q, dtype=float))
        X = eval_link_pose_in_world(plant, context, "link_7")
        p_drake = np.asarray(X.translation())
        R_drake = np.asarray(X.rotation().matrix())
        max_pos = max(max_pos, float(np.linalg.norm(T[:3, 3] - p_drake)))
        max_rot = max(max_rot, float(np.linalg.norm(T[:3, :3] - R_drake)))
    assert max_rot < 1e-9, f"姿态误差过大：{max_rot:.3e} rad"
    assert max_pos < 1e-6, f"位置误差过大：{max_pos:.3e} m"


# ── 负例：证明比对有鉴别力 ──────────────────────────────────────────────
def test_wrong_alpha_table_is_detected(drake_plant):
    """错误 alpha 表（全 0）与 Drake 的差必须 > 1e-3 m（比对不是恒真）。"""
    plant, context = drake_plant
    wrong_alpha = (0.0,) * 7
    worst = 0.0
    for q in _random_configs(count=20):
        T = mdh.fk(q, alpha=wrong_alpha)
        p_drake, _ = _drake_link7_pose(plant, context, q)
        worst = max(worst, float(np.linalg.norm(T[:3, 3] - p_drake)))
    assert worst > 1e-3, f"错误 alpha 表竟然没被测出来（差 {worst:.3e} m）"


def test_wrong_d_table_is_detected(drake_plant):
    """错误 d 表（末项改成 0.144 → 0.3）与 Drake 的差必须 > 1e-3 m。"""
    plant, context = drake_plant
    wrong_d = list(mdh.MDH_D)
    wrong_d[6] = 0.3
    worst = 0.0
    for q in _random_configs(count=20):
        T = mdh.fk(q, d=wrong_d)
        p_drake, _ = _drake_link7_pose(plant, context, q)
        worst = max(worst, float(np.linalg.norm(T[:3, 3] - p_drake)))
    assert worst > 1e-3, f"错误 d 表竟然没被测出来（差 {worst:.3e} m）"


# ── 负例：输入校验 ─────────────────────────────────────────────────────
@pytest.mark.parametrize(
    "bad_q",
    [
        [0.0] * 6,
        [0.0] * 8,
        [],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("nan")],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("inf")],
    ],
)
def test_fk_rejects_bad_joint_angles(bad_q):
    """非法关节角必须显式抛 ValueError（不静默产出垃圾结果）。"""
    with pytest.raises(ValueError):
        mdh.fk(bad_q)


def test_fk_rejects_bad_tables():
    """长度不对的 d/alpha 表必须抛 ValueError。"""
    with pytest.raises(ValueError):
        mdh.fk(np.zeros(7), d=[0.1] * 6)
    with pytest.raises(ValueError):
        mdh.fk(np.zeros(7), alpha=[0.1] * 8)


def test_fk_output_is_not_shared_state():
    """fk 返回的数组必须是独立副本（调用方改返回值不能污染后续调用）。"""
    T = mdh.fk(np.zeros(7))
    T[2, 3] = 999.0
    assert mdh.fk(np.zeros(7))[2, 3] == pytest.approx(mdh.MDH_ZERO_HEIGHT_M, abs=1e-12)
