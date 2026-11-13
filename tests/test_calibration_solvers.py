"""标定求解器测试（S04-T04）。

覆盖
----
* **正例**：合成真值（随机旋转 + 平移）→ 残差 < 1e-9；
  工具 TCP 的 **4 点**与 **6 点**两条路径都覆盖；基座外参用 **≥ 15 组**点对；
  关节零位用逐关节 +10° 合成记录反解。
* **噪声单调性**：注入 σ = 0 / 1e-4 / 1e-3 / 1e-2 m 噪声 → RMS 残差**单调不减**。
* **超阈值报错**：阈值调到极小 → 抛 ``CalibrationError``（消息含超差量与阈值）。
* **零 Drake 依赖**：本包模块**不得** import ``pydrake``（源码扫描 + 独立进程断言）。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from rm75_drake.calibration import (
    CalibrationError,
    JointZeroSample,
    solve_base_extrinsic,
    solve_joint_zero_offsets,
    solve_tool_tcp,
)
from rm75_drake.kinematics.mdh import fk as mdh_fk

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_DIR = REPO_ROOT / "src" / "rm75_drake" / "calibration"

#: 合成真值残差的判定阈值（比计划要求的 1e-9 更严一档，留舍入余量）。
SYNTHETIC_RESIDUAL_LIMIT = 1e-9


def _rot(axis: str, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _random_rotation(rng: np.random.Generator) -> np.ndarray:
    """随机旋转矩阵（QR 分解法，行列式恒为 +1）。"""
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q @ np.diag(np.sign(np.diag(r)))
    if np.linalg.det(q) < 0:
        q[:, 0] = -q[:, 0]
    return q


# ── 工具 TCP（4 点 / 6 点两条路径）────────────────────────────────────────
@pytest.mark.parametrize("n_points", [4, 6])
def test_tool_tcp_recovers_synthetic_truth(n_points: int) -> None:
    """合成真值 → 4 点法与 6 点法都解回原旋转/平移，残差 < 1e-9。"""
    rng = np.random.default_rng(20261113 + n_points)
    local = rng.normal(size=(n_points, 3)) * 0.2
    rot = _rot("z", 0.3) @ _rot("y", -0.2) @ _rot("x", 0.1)
    trans = np.array([0.012, -0.021, 0.033])
    world = local @ rot.T + trans

    solution = solve_tool_tcp(list(zip(local, world)))

    assert np.max(np.abs(solution.rotation - rot)) < SYNTHETIC_RESIDUAL_LIMIT
    assert np.max(np.abs(solution.translation - trans)) < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.rms < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.max_residual < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.sample_count == n_points
    assert solution.report.within_threshold is True


def test_tool_tcp_random_rotation_truth() -> None:
    """随机旋转 + 平移的合成真值 → 残差 < 1e-9。"""
    rng = np.random.default_rng(7)
    local = rng.normal(size=(8, 3))
    rot = _random_rotation(rng)
    trans = rng.normal(size=3)
    world = local @ rot.T + trans

    solution = solve_tool_tcp(list(zip(local, world)))

    assert solution.report.max_residual < SYNTHETIC_RESIDUAL_LIMIT
    assert np.max(np.abs(solution.rotation - rot)) < SYNTHETIC_RESIDUAL_LIMIT
    assert np.max(np.abs(solution.translation - trans)) < SYNTHETIC_RESIDUAL_LIMIT


# ── 基座外参 X_LR ───────────────────────────────────────────────────────
def test_base_extrinsic_recovers_synthetic_truth() -> None:
    """≥ 15 组「尖对尖」点对 → 解回 X_LR，残差 < 1e-9。"""
    rng = np.random.default_rng(99)
    n = 20
    right = rng.normal(size=(n, 3)) * 0.3
    truth = np.eye(4)
    truth[:3, :3] = _rot("z", 0.2) @ _rot("y", 0.05)
    truth[:3, 3] = np.array([0.6, 0.0, 0.0])
    left = right @ truth[:3, :3].T + truth[:3, 3]

    solution = solve_base_extrinsic(list(zip(right, left)))

    assert solution.report.sample_count == n
    assert solution.report.rms < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.max_residual < SYNTHETIC_RESIDUAL_LIMIT
    assert np.max(np.abs(solution.matrix - truth)) < SYNTHETIC_RESIDUAL_LIMIT


# ── 噪声单调性 ──────────────────────────────────────────────────────────
def test_noise_injection_rms_is_monotonic_non_decreasing() -> None:
    """σ = 0 / 1e-4 / 1e-3 / 1e-2 m 噪声 → RMS 残差单调不减。"""
    rng = np.random.default_rng(2026)
    local = rng.normal(size=(20, 3)) * 0.25
    rot = _random_rotation(rng)
    trans = rng.normal(size=3)
    world = local @ rot.T + trans

    sigmas = [0.0, 1e-4, 1e-3, 1e-2]
    rms_values = []
    for sigma in sigmas:
        noise = rng.normal(scale=sigma, size=world.shape) if sigma > 0 else 0.0
        noisy = world + noise
        # 阈值放宽，避免超阈值直接报错（这里测的是残差随噪声的单调性）。
        solution = solve_tool_tcp(list(zip(local, noisy)), threshold=1.0)
        rms_values.append(solution.report.rms)

    for i in range(1, len(rms_values)):
        assert rms_values[i] >= rms_values[i - 1] - 1e-18, (
            f"噪声单调性被破坏：{rms_values}"
        )
    assert rms_values[-1] > rms_values[0]


# ── 超阈值必须报错（禁止静默输出）────────────────────────────────────────
def test_tool_tcp_raises_when_residual_exceeds_threshold() -> None:
    """阈值调到极小 → 抛 CalibrationError，消息含超差量与阈值。"""
    rng = np.random.default_rng(5)
    local = rng.normal(size=(6, 3))
    # 逐点随机扰动 ⇒ 刚体拟合**无法**完全吸收 ⇒ 残差 > 0。
    world = local + rng.normal(scale=1e-3, size=local.shape)

    with pytest.raises(CalibrationError) as excinfo:
        solve_tool_tcp(list(zip(local, world)), threshold=1e-12)

    message = str(excinfo.value)
    assert "残差超阈值" in message
    assert "阈值" in message
    # 消息必须同时给出超差量与阈值（数值）。
    assert "1.000000e-12" in message


def test_base_extrinsic_raises_when_residual_exceeds_threshold() -> None:
    """基座外参同样遵守「超阈值报错」。"""
    rng = np.random.default_rng(6)
    right = rng.normal(size=(15, 3))
    # 逐点随机扰动 ⇒ 拟合残差 > 0（纯平移会被刚体拟合完全吸收，测不出超阈值）。
    left = right + rng.normal(scale=1e-3, size=right.shape)

    with pytest.raises(CalibrationError) as excinfo:
        solve_base_extrinsic(list(zip(right, left)), threshold=1e-9)

    assert "残差超阈值" in str(excinfo.value)


# ── 关节零位偏差 ─────────────────────────────────────────────────────────
def _synthetic_joint_zero_samples(delta: np.ndarray) -> list[JointZeroSample]:
    """按「逐关节 +10°」生成合成采样（其它关节偏移为 0，逐关节独立标定）。"""
    samples = []
    for j in range(7):
        q_nom = np.zeros(7)
        q_nom[j] = 0.2
        q_off = q_nom.copy()
        q_off[j] = q_nom[j] + np.deg2rad(10.0)
        mask = np.zeros(7)
        mask[j] = delta[j]
        pose_nom = mdh_fk((q_nom + mask).tolist())
        pose_off = mdh_fk((q_off + mask).tolist())
        samples.append(
            JointZeroSample(
                joint=f"joint_{j + 1}",
                index=j,
                q_nominal=tuple(q_nom.tolist()),
                q_offset=tuple(q_off.tolist()),
                pose_nominal=tuple(tuple(row) for row in pose_nom),
                pose_offset=tuple(tuple(row) for row in pose_off),
                synthetic=True,
                source="synthetic-joint-zero",
            )
        )
    return samples


def test_joint_zero_offsets_recover_synthetic_truth() -> None:
    """逐关节 +10° 对比 FK → 零位偏差反解回真值（< 1e-9 rad），残差 < 1e-9 m。"""
    delta = np.array([0.003, -0.002, 0.0015, -0.001, 0.002, -0.0015, 0.0025])
    samples = _synthetic_joint_zero_samples(delta)

    solution = solve_joint_zero_offsets(samples, fk_fn=mdh_fk)

    assert set(solution.offsets) == {f"joint_{j + 1}" for j in range(7)}
    for j in range(7):
        assert abs(solution.offsets[f"joint_{j + 1}"] - delta[j]) < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.rms < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.max_residual < SYNTHETIC_RESIDUAL_LIMIT
    assert solution.report.sample_count == 7
    assert solution.report.within_threshold is True


def test_joint_zero_unidentifiable_joint_raises() -> None:
    """FK 对该关节不敏感 → 零位不可辨识 → 显式报错（不返回假结果）。"""
    delta = np.zeros(7)
    samples = _synthetic_joint_zero_samples(delta)
    # 用一个与关节角无关的恒定 FK：目标函数对 δ 完全不敏感。
    constant = np.eye(4)

    with pytest.raises(CalibrationError) as excinfo:
        solve_joint_zero_offsets(samples, fk_fn=lambda _q: constant)

    assert "不可辨识" in str(excinfo.value)


def test_joint_zero_invalid_sample_raises() -> None:
    """采样结构非法（+10° 缺失 / 关节数不对）→ CalibrationError。"""
    delta = np.zeros(7)
    samples = _synthetic_joint_zero_samples(delta)
    broken = samples[0]
    broken = JointZeroSample(
        joint=broken.joint,
        index=broken.index,
        q_nominal=broken.q_nominal,
        q_offset=broken.q_nominal,  # 与 nominal 相同 → 不是 +10° 采样
        pose_nominal=broken.pose_nominal,
        pose_offset=broken.pose_offset,
        synthetic=True,
        source="broken",
    )

    with pytest.raises(CalibrationError) as excinfo:
        solve_joint_zero_offsets([broken], fk_fn=mdh_fk)

    assert "+10°" in str(excinfo.value)


def test_joint_zero_empty_samples_raises() -> None:
    """采样为空 → 显式报错。"""
    with pytest.raises(CalibrationError):
        solve_joint_zero_offsets([], fk_fn=mdh_fk)


# ── 零 Drake 依赖断言 ───────────────────────────────────────────────────
def test_calibration_package_source_has_no_pydrake_import() -> None:
    """源码扫描：本包任何模块都不得 **import** ``pydrake``（文档里提到不算）。"""
    import re

    pattern = re.compile(r"^\s*(?:from\s+pydrake|import\s+pydrake)", re.MULTILINE)
    offenders = []
    for path in sorted(PACKAGE_DIR.glob("*.py")):
        text = path.read_text(encoding="utf-8")
        if pattern.search(text):
            offenders.append(path.name)
    assert offenders == [], f"标定包不得依赖 pydrake：{offenders}"


def test_importing_calibration_does_not_import_pydrake() -> None:
    """独立进程断言：import 本包后 ``sys.modules`` 里不得出现 ``pydrake``。"""
    code = (
        "import sys; import rm75_drake.calibration as c; "
        "bad=[m for m in sys.modules if m.split('.')[0]=='pydrake']; "
        "assert not bad, bad; print('ok')"
    )
    env = {"PYTHONPATH": str(REPO_ROOT / "src")}
    import os

    merged_env = dict(os.environ)
    merged_env.update(env)
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        cwd=str(REPO_ROOT),
        env=merged_env,
    )
    assert result.returncode == 0, f"import 断言失败：{result.stdout}\n{result.stderr}"
    assert "ok" in result.stdout
