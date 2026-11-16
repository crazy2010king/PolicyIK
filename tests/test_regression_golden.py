"""golden-file 基线与「逐点一致」离线回放比较器（S07-T02）的行为测试。

这些断言守住的契约
------------------
* **基线存在性 / 结构**：基线文件在库里、可加载、校验和自洽、维度正确（``N×7`` /
  ``N×3``）、无 NaN、含三枚指纹；
* **指纹活性**（**防假绿灯**）：基线携带的三枚指纹必须与**当前仓库文件**
  （模型 URDF / 场景 YAML / 任务 YAML）的 sha256 一致 ⇒ 模型 / 场景 / 任务漂移
  会让本用例**主动失败**（报「基线失效」）而不是「通过」；
* **逐点一致**：真实求解一次 → 比较器复算 ⇒ ``pointwise_identical is True``、
  ``num_tcp_mismatches == 0``；
* **有鉴别力负例 A**：人为平移基线里的**一个**末端点 ⇒ 比较器必须报
  ``num_tcp_mismatches > 0`` / ``pointwise_identical is False``
  ⇒ 证明比较器对被测变量敏感（不会恒绿）；
* **有鉴别力负例 B**：伪造指纹 ⇒ ``fingerprints_match is False``
  ⇒ 证明「基线失效」通路可达；
* **加载器负例**：未知键 / 缺键 / 维度不对 / NaN / 长度不一致 / 校验和不符
  ⇒ :class:`GoldenError` 且**错误消息点名违规键**。

零 pydrake 依赖
---------------
基线存在性 / 指纹 / 两条负例 **恒跑**（不需要 pydrake）：它们用**基线自身**
构造一个合成的 session（``Recording`` + ``RunLog`` 都是零 pydrake 的既有产物）。
只有「真实求解」那条用例需要 pydrake（``pytest.importorskip`` 跳过）。
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

from rm75_drake.device.run_log import build_run_log
from rm75_drake.gripper import GripperAction
from rm75_drake.recording import Recording, RecordingSample
from rm75_drake.regression.golden import (
    FINGERPRINT_KEYS,
    GOLDEN_SCHEMA_VERSION,
    GoldenBaseline,
    GoldenError,
    build_golden,
    compare_golden,
    compute_golden_checksum,
    load_golden,
    repo_fingerprints,
    tcp_positions_from_joints,
)
from rm75_drake.regression.session import build_session

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

#: 基线文件（**唯一**基线入口，由 ``scripts/gen_golden.py`` 生成）。
GOLDEN_PATH = REPO_ROOT / "tests" / "data" / "golden" / "single_arm_pick_place.json"

#: 指纹绑定的仓库文件（测试里**独立**重算 sha256，不复用生产代码的封装）。
FINGERPRINT_FILES = {
    "urdf_sha256": REPO_ROOT / "models" / "rm75_drake" / "rm75_drake.urdf",
    "scenario_sha256": REPO_ROOT / "scenarios" / "single_arm.yaml",
    "task_sha256": REPO_ROOT / "tasks" / "pick_place.yaml",
}

#: 任务文件 / 场景文件（生成器用）。
TASK_PATH = REPO_ROOT / "tasks" / "pick_place.yaml"
SCENARIO_PATH = REPO_ROOT / "scenarios" / "single_arm.yaml"


# ── 工具 ────────────────────────────────────────────────────────────────
def _sha256(path: Path) -> str:
    """独立重算文件 sha256（不复用生产代码的封装，避免「自证」）。"""
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_generator() -> ModuleType:
    """按文件路径加载 ``scripts/gen_golden.py``（生成器的**唯一**实现来源）。"""
    script = REPO_ROOT / "scripts" / "gen_golden.py"
    spec = importlib.util.spec_from_file_location("gen_golden_script", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _session_from_baseline(baseline: GoldenBaseline):
    """用基线自身构造一份**零 pydrake** 的 session（供比较器的负例使用）。

    关节样本与基线逐点相同 ⇒ 比较器复算出的末端位置与基线**逐点相同**
    ⇒ 任何「不一致」都只能来自被篡改的基线（负例 A）或指纹漂移（负例 B）。
    """
    recording = Recording(
        robot_name=baseline.robot_name,
        dt=float(baseline.dt),
        samples=tuple(
            RecordingSample(
                seq=index,
                t_abs=index * float(baseline.dt),
                q=tuple(float(v) for v in row),
            )
            for index, row in enumerate(baseline.joint_samples)
        ),
        gripper_schedule=tuple(
            GripperAction(
                kind=str(item["kind"]),
                position=item["position"],
                speed=int(item["speed"]),
                force=item["force"],
                hold_seconds=float(item["hold_seconds"]),
            )
            for item in baseline.gripper_schedule
        ),
        meta={"urdf_sha256": baseline.fingerprints.get("urdf_sha256", "")},
    )
    run_log = build_run_log([], meta={"robot_name": baseline.robot_name}, summary={})
    return build_session(
        recording,
        run_log,
        meta={
            "robot_name": baseline.robot_name,
            "urdf_sha256": baseline.fingerprints.get("urdf_sha256", ""),
        },
    )


def _mutated_baseline(baseline: GoldenBaseline, mutate) -> GoldenBaseline:
    """返回一份**校验和自洽**的基线副本（``mutate`` 就地改字典）。"""
    data = baseline.to_dict()
    mutate(data)
    data["checksum"] = compute_golden_checksum(data)
    return GoldenBaseline.from_dict(data)


@pytest.fixture(scope="module")
def baseline() -> GoldenBaseline:
    """库里的 golden 基线（严格校验 ⇒ 加载失败即测试失败）。"""
    assert GOLDEN_PATH.is_file(), f"基线文件不存在：{GOLDEN_PATH}"
    return load_golden(GOLDEN_PATH)


@pytest.fixture(scope="module")
def synthetic_session(baseline: GoldenBaseline):
    """由基线构造的**零 pydrake** session（负例用）。"""
    return _session_from_baseline(baseline)


# ── 基线存在性 / 结构 ────────────────────────────────────────────────────
def test_golden_file_exists_and_is_structurally_valid(baseline: GoldenBaseline) -> None:
    """基线文件存在、可加载、校验和自洽、维度正确、无 NaN、含三枚指纹。"""
    assert GOLDEN_PATH.is_file()
    assert baseline.schema_version == GOLDEN_SCHEMA_VERSION
    assert baseline.robot_name.strip()
    assert baseline.dt > 0.0
    # 校验和自洽（load_golden 已校验；这里再显式复算一次）。
    assert compute_golden_checksum(baseline.to_dict()) == baseline.checksum

    # 三枚指纹齐全。
    assert set(baseline.fingerprints) == set(FINGERPRINT_KEYS)

    # 维度：N×7 关节样本 + N×3 末端点，且长度一致。
    assert baseline.joint_samples, "基线不得为空"
    assert len(baseline.joint_samples) == len(baseline.tcp_positions)
    assert all(len(row) == 7 for row in baseline.joint_samples)
    assert all(len(row) == 3 for row in baseline.tcp_positions)

    # 无 NaN / Inf。
    for row in baseline.joint_samples + baseline.tcp_positions:
        for value in row:
            assert value == value and abs(value) != float("inf")

    # 落盘内容与内存对象一致（文件里就是这三枚指纹 + N×7 + N×3）。
    data = json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))
    assert set(data["fingerprints"]) == set(FINGERPRINT_KEYS)
    assert len(data["joint_samples"]) == len(baseline.joint_samples)
    assert len(data["tcp_positions"]) == len(baseline.tcp_positions)
    assert all(len(row) == 7 for row in data["joint_samples"])
    assert all(len(row) == 3 for row in data["tcp_positions"])


def test_golden_module_is_pydrake_free() -> None:
    """golden 模块（含其依赖链）**不得**把 pydrake 拉进 ``sys.modules``。"""
    code = (
        "import sys; sys.path.insert(0, %r);"
        "import rm75_drake.regression.golden as g;"
        "assert 'pydrake' not in sys.modules, sorted(sys.modules);"
        "print('ok')" % str(SRC_DIR)
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        cwd=str(REPO_ROOT),
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout


# ── 指纹活性（防假绿灯）─────────────────────────────────────────────────
def test_fingerprints_bind_current_repo_files(baseline: GoldenBaseline) -> None:
    """基线的三枚指纹必须等于**当前仓库文件**的 sha256。

    ⇒ 模型 / 场景 / 任务一旦漂移，本用例**主动失败**（而不是「基线照样通过」）。
    """
    for key, path in FINGERPRINT_FILES.items():
        assert path.is_file(), f"{key} 绑定的文件不存在：{path}"
        assert baseline.fingerprints[key] == _sha256(path), (
            f"基线失效：{key} 与当前仓库文件 {path.name} 不一致"
            f"（baseline={baseline.fingerprints[key]}，current={_sha256(path)}）"
            " ⇒ 需重新生成基线并复盘差异"
        )
    # 与生产代码的「当前仓库指纹」封装一致（两者必须同口径）。
    assert dict(baseline.fingerprints) == repo_fingerprints()


def test_fingerprint_mismatch_is_reported_not_silently_passed(
    baseline: GoldenBaseline, synthetic_session
) -> None:
    """指纹漂移 ⇒ ``fingerprints_match is False``（比较器**不得**静默通过）。"""
    forged = _mutated_baseline(
        baseline, lambda data: data["fingerprints"].update({"urdf_sha256": "0" * 64})
    )
    comparison = compare_golden(forged, synthetic_session)
    assert comparison.fingerprints_match is False
    assert comparison.baseline_invalidated is True
    assert comparison.ok is False
    # 逐点数据仍然一致 ⇒ 失效原因**唯一**是指纹（证明判据不串味）。
    assert comparison.pointwise_identical is True
    detail = comparison.fingerprint_details["urdf_sha256"]
    assert detail["baseline"] != detail["current"]


# ── 比较器：正例（零 pydrake）───────────────────────────────────────────
def test_compare_against_synthetic_session_is_pointwise_identical(
    baseline: GoldenBaseline, synthetic_session
) -> None:
    """基线 vs 「由基线构造的 session」⇒ 逐点一致（零 pydrake 恒跑）。"""
    comparison = compare_golden(baseline, synthetic_session)
    assert comparison.fingerprints_match is True
    assert comparison.num_points == len(baseline.joint_samples)
    assert comparison.num_tcp_mismatches == 0
    assert comparison.pointwise_identical is True
    assert comparison.max_tcp_diff_m <= 1e-9
    assert comparison.joint_max_diff_rad <= 1e-9
    assert comparison.num_joint_mismatches == 0
    assert comparison.gripper_match is True
    assert comparison.lengths_match is True
    assert comparison.ok is True


def test_tcp_recompute_uses_mdh_fk(baseline: GoldenBaseline) -> None:
    """复算口径 = :mod:`rm75_drake.kinematics.mdh` 的理论正解（纯 numpy）。"""
    from rm75_drake.kinematics import mdh

    recomputed = tcp_positions_from_joints(baseline.joint_samples)
    assert len(recomputed) == len(baseline.tcp_positions)
    for index in (0, len(recomputed) // 2, len(recomputed) - 1):
        direct = mdh.link7_position(baseline.joint_samples[index])
        assert recomputed[index] == pytest.approx(
            [float(direct[0]), float(direct[1]), float(direct[2])], abs=0.0
        )
        assert recomputed[index] == pytest.approx(
            list(baseline.tcp_positions[index]), abs=1e-12
        )


# ── 比较器：负例（有鉴别力）──────────────────────────────────────────────
def test_negative_shifted_tcp_point_is_detected(
    baseline: GoldenBaseline, synthetic_session
) -> None:
    """负例 A：平移基线里的**一个**末端点 ⇒ 比较器必须报不一致（不会恒绿）。"""
    index = 17
    assert index < len(baseline.tcp_positions)

    def mutate(data) -> None:
        data["tcp_positions"][index][0] += 1e-6

    mutated = _mutated_baseline(baseline, mutate)
    comparison = compare_golden(mutated, synthetic_session)
    assert comparison.num_tcp_mismatches > 0, "比较器对被测变量不敏感（恒绿）"
    assert comparison.num_tcp_mismatches == 1
    assert comparison.pointwise_identical is False
    assert comparison.max_tcp_diff_m >= 1e-6 - 1e-15
    assert comparison.ok is False
    # 篡改的是末端点 ⇒ 关节序列仍一致（判据分离）。
    assert comparison.num_joint_mismatches == 0


def test_negative_length_mismatch_is_detected(
    baseline: GoldenBaseline, synthetic_session
) -> None:
    """负例：基线少一个点 ⇒ 点数不一致 ⇒ 判为不一致。"""

    def mutate(data) -> None:
        data["joint_samples"] = data["joint_samples"][:-1]
        data["tcp_positions"] = data["tcp_positions"][:-1]

    mutated = _mutated_baseline(baseline, mutate)
    comparison = compare_golden(mutated, synthetic_session)
    assert comparison.lengths_match is False
    assert comparison.pointwise_identical is False
    assert comparison.num_tcp_mismatches > 0
    assert comparison.ok is False


# ── 比较器：真实求解（需要 pydrake）────────────────────────────────────
def test_pointwise_identical_against_real_solve(baseline: GoldenBaseline) -> None:
    """真实求解一次 → 比较器复算 ⇒ 逐点一致（``num_tcp_mismatches == 0``）。"""
    pytest.importorskip("pydrake")
    generator = _load_generator()
    session = generator.build_chain_session(TASK_PATH, SCENARIO_PATH)

    comparison = compare_golden(baseline, session)
    assert comparison.fingerprints_match is True
    assert comparison.num_points == len(baseline.joint_samples)
    assert comparison.num_tcp_mismatches == 0
    assert comparison.pointwise_identical is True
    assert comparison.max_tcp_diff_m <= 1e-9
    assert comparison.joint_max_diff_rad <= 1e-9
    assert comparison.gripper_match is True
    assert comparison.ok is True


def test_gen_golden_check_mode_exits_zero() -> None:
    """``scripts/gen_golden.py --check`` 必须退出码 0（比对通过、不写盘）。"""
    pytest.importorskip("pydrake")
    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "scripts" / "gen_golden.py"), "--check"],
        capture_output=True,
        text=True,
        cwd=str(REPO_ROOT),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "pointwise_identical=True" in result.stdout


def test_gen_golden_check_mode_detects_drift(tmp_path: Path) -> None:
    """负例：把基线里的一个末端点挪动后，``--check`` 必须**失败**（退出码 1）。"""
    pytest.importorskip("pydrake")
    baseline = load_golden(GOLDEN_PATH)
    drifted = _mutated_baseline(
        baseline, lambda data: data["tcp_positions"][3].__setitem__(1, data["tcp_positions"][3][1] + 1e-5)
    )
    from rm75_drake.regression.golden import write_golden

    target = tmp_path / "drifted.json"
    write_golden(drifted, target)
    result = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "scripts" / "gen_golden.py"),
            "--check",
            "--out",
            str(target),
        ],
        capture_output=True,
        text=True,
        cwd=str(REPO_ROOT),
    )
    assert result.returncode == 1, result.stdout + result.stderr


# ── 加载器负例（点名违规键）──────────────────────────────────────────────
def _expect_golden_error(data, needle: str) -> None:
    """断言构造基线时报错，且**错误消息里出现 ``needle``（点名违规键）。"""
    with pytest.raises(GoldenError) as excinfo:
        GoldenBaseline.from_dict(data)
    assert needle in str(excinfo.value), (
        f"错误消息未点名违规键 {needle!r}：{excinfo.value}"
    )


def test_loader_rejects_unknown_key(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["unexpected_key"] = 1
    _expect_golden_error(data, "unexpected_key")


def test_loader_rejects_missing_key(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    del data["tcp_positions"]
    _expect_golden_error(data, "tcp_positions")


def test_loader_rejects_wrong_joint_dimension(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["joint_samples"][0] = data["joint_samples"][0][:6]
    _expect_golden_error(data, "joint_samples[0]")


def test_loader_rejects_wrong_tcp_dimension(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["tcp_positions"][0] = data["tcp_positions"][0][:2]
    _expect_golden_error(data, "tcp_positions[0]")


def test_loader_rejects_nan(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["joint_samples"][2][1] = float("nan")
    _expect_golden_error(data, "joint_samples[2][1]")


def test_loader_rejects_length_mismatch(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["tcp_positions"] = data["tcp_positions"][:-1]
    _expect_golden_error(data, "长度不一致")


def test_loader_rejects_checksum_mismatch(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["robot_name"] = "other_robot"
    _expect_golden_error(data, "checksum")


def test_loader_rejects_unknown_fingerprint_key(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    data["fingerprints"]["extra_sha256"] = "0" * 64
    _expect_golden_error(data, "extra_sha256")


def test_loader_rejects_missing_fingerprint_key(baseline: GoldenBaseline) -> None:
    data = baseline.to_dict()
    del data["fingerprints"]["task_sha256"]
    _expect_golden_error(data, "task_sha256")


def test_build_golden_rejects_tcp_count_mismatch(baseline: GoldenBaseline) -> None:
    """构造基线时末端点数与采样点数不一致 ⇒ :class:`GoldenError`。"""
    session = _session_from_baseline(baseline)
    with pytest.raises(GoldenError) as excinfo:
        build_golden(
            session,
            fingerprints=baseline.fingerprints,
            tcp_positions=baseline.tcp_positions[:-1],
        )
    assert "tcp_positions" in str(excinfo.value)


def test_compare_rejects_wrong_types(baseline: GoldenBaseline) -> None:
    """比较器对类型不对的输入必须显式报错（不静默通过）。"""
    with pytest.raises(GoldenError):
        compare_golden(baseline, object())  # type: ignore[arg-type]
    with pytest.raises(GoldenError):
        compare_golden(object(), object())  # type: ignore[arg-type]
