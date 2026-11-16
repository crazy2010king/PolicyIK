"""原语级指标口径的测试（S06-T07）。

覆盖面
------
* **未验证物理指标清单**：canonical 唯一真相源、非空、含关键条目，
  且 CLI 只是**再导出**（不复制第二份）；
* **逐原语指标**：成功率、误差分位数（P95）、零逃逸率、样本数；
* **失败即停止**：失败原语之后的原语**不计入**分母；
* **验收口径 / 端到端口径分行**：两段都在，且端到端内容**不混入**验收段；
* **门槛判定**：成功率 / 位置误差 / 姿态误差 / 零逃逸率任一不达标 ⇒ 判失败；
* **零 Drake 依赖**：``import rm75_drake.tasks.metrics`` 不拉起 pydrake。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

from rm75_drake.tasks.metrics import (
    DEFAULT_PRIMITIVE_ORIENTATION_ERROR_P95_RAD,
    DEFAULT_PRIMITIVE_POSITION_ERROR_P95_M,
    DEFAULT_PRIMITIVE_SUCCESS_RATE_THRESHOLD,
    DEFAULT_ZERO_ESCAPE_RATE_THRESHOLD,
    UNVERIFIED_TASK_METRICS,
    PrimitiveMetric,
    TaskMetricsReport,
    build_task_metrics_report,
    percentile,
)


# ── 替身：结果对象（鸭子类型）────────────────────────────────────────────
def _primitive_result(
    name: str,
    *,
    success: bool = True,
    stage: str = "done",
    reason: str = "completed",
    position_error_m: Optional[float] = None,
    orientation_error_rad: Optional[float] = None,
) -> Any:
    """构造一个「原语结果」替身（字段与求解器结果一致）。"""
    return SimpleNamespace(
        name=name,
        success=success,
        stage=stage,
        reason=reason,
        position_error_m=position_error_m,
        orientation_error_rad=orientation_error_rad,
    )


def _sample(results: Sequence[Any], *, success: bool = True, duration_s: float = 1.0) -> Any:
    """构造一个「整链运行结果」替身。"""
    return SimpleNamespace(results=tuple(results), success=success, duration_s=duration_s)


# ── 未验证物理指标清单 ──────────────────────────────────────────────────
def test_unverified_task_metrics_is_non_empty_and_covers_key_items() -> None:
    """清单非空，且逐条列出真机未验证的物理指标（含关键条目）。"""
    assert isinstance(UNVERIFIED_TASK_METRICS, tuple)
    assert len(UNVERIFIED_TASK_METRICS) >= 6
    text = "\n".join(UNVERIFIED_TASK_METRICS)
    for keyword in ("抓放成功率", "夹持误差", "交接保持漂移", "感知延迟", "力控", "长稳"):
        assert keyword in text, keyword
    # 每条都必须写明「未验证」与**解锁条件**
    for item in UNVERIFIED_TASK_METRICS:
        assert "未验证" in item
        assert "解锁条件" in item


def test_cli_reexports_the_canonical_tuple_without_copying() -> None:
    """CLI 侧只**再导出** canonical 清单（禁止复制第二份）。"""
    import rm75_drake.cli as cli

    assert tuple(cli.UNVERIFIED_TASK_METRICS) == tuple(UNVERIFIED_TASK_METRICS)
    assert "UNVERIFIED_TASK_METRICS" in cli.__all__


def test_metrics_module_does_not_load_pydrake() -> None:
    """``import rm75_drake.tasks.metrics`` 不得把 pydrake 拉进 sys.modules。"""
    code = (
        "import sys; import rm75_drake.tasks.metrics as m; "
        "print('pydrake' in sys.modules)"
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(SRC_DIR), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == "False"


# ── 分位数 ──────────────────────────────────────────────────────────────
def test_percentile_uses_linear_interpolation() -> None:
    """分位数口径 = 线性插值（与 scripts/coordinated_stats.py 同款）。"""
    assert percentile([], 0.95) is None
    assert percentile([0.5], 0.95) == 0.5
    assert percentile([0.0, 1.0], 0.5) == 0.5
    values = [float(index) for index in range(101)]
    assert abs(percentile(values, 0.95) - 95.0) < 1e-9


# ── 逐原语指标 ──────────────────────────────────────────────────────────
def test_report_aggregates_per_primitive_success_rate_and_errors() -> None:
    """逐原语成功率 / 位置误差 P95 / 姿态误差 P95 / 样本数。"""
    samples = [
        _sample(
            [
                _primitive_result(
                    "approach", position_error_m=1e-6, orientation_error_rad=2e-6
                ),
                _primitive_result("close"),
            ],
            success=True,
        ),
        _sample(
            [
                _primitive_result(
                    "approach", position_error_m=3e-6, orientation_error_rad=4e-6
                ),
                _primitive_result("close"),
            ],
            success=True,
        ),
    ]
    report = build_task_metrics_report(samples, task_name="demo")

    assert isinstance(report, TaskMetricsReport)
    assert report.task_name == "demo"
    assert report.samples == 2
    assert report.success_rate == 1.0
    assert report.zero_escape_rate == 1.0

    approach = report.primitive("approach")
    assert approach is not None
    assert approach.samples == 2
    assert approach.success_rate == 1.0
    assert approach.position_error_p95_m is not None
    assert 1e-6 <= approach.position_error_p95_m <= 3e-6
    assert approach.orientation_error_p95_rad is not None
    assert 2e-6 <= approach.orientation_error_p95_rad <= 4e-6
    assert approach.zero_escape_rate == 1.0

    # 夹爪类原语没有位置 / 姿态误差 ⇒ None（**不**伪造 0.0）
    close = report.primitive("close")
    assert close is not None
    assert close.position_error_p95_m is None
    assert close.orientation_error_p95_rad is None


def test_failed_primitive_stops_the_chain_and_is_not_counted_later() -> None:
    """失败即停止：失败原语之后的原语**不计入**分母。"""
    samples = [
        _sample(
            [
                _primitive_result("approach"),
                _primitive_result(
                    "pre_grasp",
                    success=False,
                    stage="ik",
                    reason="unreachable",
                ),
            ],
            success=False,
        )
    ]
    report = build_task_metrics_report(samples, task_name="demo")

    assert report.success_rate == 0.0
    assert report.primitive("lift") is None
    pre_grasp = report.primitive("pre_grasp")
    assert pre_grasp is not None
    assert pre_grasp.success_rate == 0.0
    assert pre_grasp.failure_breakdown["ik:unreachable"] == 1
    # 门槛不达标 ⇒ 整体判失败
    assert report.thresholds_met is False
    assert any("pre_grasp" in item for item in report.threshold_failures)


def test_collision_escape_lowers_zero_escape_rate() -> None:
    """碰撞逃逸（reason="escape"）计入零逃逸率。"""
    samples = [
        _sample(
            [
                _primitive_result(
                    "approach", success=False, stage="collision", reason="escape"
                )
            ],
            success=False,
        ),
        _sample([_primitive_result("approach")], success=True),
    ]
    report = build_task_metrics_report(samples, task_name="demo")

    approach = report.primitive("approach")
    assert approach is not None
    assert approach.zero_escape_rate == 0.5
    assert report.zero_escape_rate == 0.5


# ── 验收口径 / 端到端口径分行 ───────────────────────────────────────────
def test_acceptance_and_end_to_end_metrics_are_separate_sections() -> None:
    """验收口径与端到端口径**分行**，端到端内容**不混入**验收段。"""
    samples = [_sample([_primitive_result("approach", position_error_m=1e-9)])]
    report = build_task_metrics_report(
        samples,
        task_name="demo",
        end_to_end_extra={"scene_assembly_s": 0.08},
    )

    acceptance = dict(report.acceptance_metrics)
    end_to_end = dict(report.end_to_end_metrics)
    assert acceptance and end_to_end
    assert "primitives" in acceptance
    assert "thresholds" in acceptance
    assert "per_sample_duration_s" in end_to_end
    assert "scene_assembly_s" in end_to_end
    # 端到端的字段**不**出现在验收段里（口径不许混用）
    assert "per_sample_duration_s" not in acceptance
    assert "scene_assembly_s" not in acceptance
    assert "primitives" not in end_to_end


def test_unverified_metrics_section_is_present_and_canonical() -> None:
    """未验证物理指标段：逐条列出，且取自 canonical 清单。"""
    samples = [_sample([_primitive_result("approach")])]
    report = build_task_metrics_report(samples, task_name="demo")

    assert tuple(report.unverified_metrics) == tuple(UNVERIFIED_TASK_METRICS)
    payload = report.as_dict()
    assert payload["unverified_metrics"] == list(UNVERIFIED_TASK_METRICS)
    assert payload["acceptance_metrics"]["thresholds_met"] is True


# ── 门槛判定 ────────────────────────────────────────────────────────────
def test_default_thresholds_match_the_primitive_tolerances() -> None:
    """默认门槛与 S06-T03 的原语容差**同源**。"""
    from rm75_drake.tasks.primitives import (
        DEFAULT_ORIENTATION_TOLERANCE_RAD,
        DEFAULT_POSITION_TOLERANCE_M,
    )

    assert DEFAULT_PRIMITIVE_POSITION_ERROR_P95_M == DEFAULT_POSITION_TOLERANCE_M
    assert (
        DEFAULT_PRIMITIVE_ORIENTATION_ERROR_P95_RAD == DEFAULT_ORIENTATION_TOLERANCE_RAD
    )
    assert DEFAULT_PRIMITIVE_SUCCESS_RATE_THRESHOLD == 0.95
    assert DEFAULT_ZERO_ESCAPE_RATE_THRESHOLD == 1.0


def test_primitive_metric_flags_each_threshold_violation() -> None:
    """逐项门槛判定：成功率 / 位置误差 / 姿态误差 / 零逃逸率。"""
    ok = PrimitiveMetric(
        name="approach",
        samples=10,
        success_rate=1.0,
        position_error_p95_m=1e-9,
        orientation_error_p95_rad=1e-9,
        zero_escape_rate=1.0,
    )
    assert ok.thresholds_met is True
    assert ok.threshold_failures() == ()

    bad = PrimitiveMetric(
        name="approach",
        samples=10,
        success_rate=0.5,
        position_error_p95_m=1e-3,
        orientation_error_p95_rad=1e-3,
        zero_escape_rate=0.9,
    )
    problems = bad.threshold_failures()
    assert len(problems) == 4
    assert any("成功率" in item for item in problems)
    assert any("位置误差" in item for item in problems)
    assert any("姿态误差" in item for item in problems)
    assert any("零逃逸率" in item for item in problems)
    assert bad.thresholds_met is False


def test_report_fails_when_position_error_exceeds_threshold() -> None:
    """误差超门槛 ⇒ 整体判失败（不只判成功率）。"""
    samples = [
        _sample([_primitive_result("approach", position_error_m=1e-3)]),
    ]
    report = build_task_metrics_report(samples, task_name="demo")

    assert report.success_rate == 1.0
    assert report.thresholds_met is False
    assert any("位置误差" in item for item in report.threshold_failures)
