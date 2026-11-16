"""S07 runbook 覆盖性测试（S07-T07）。

契约
----
``runbooks/s07_engineering_productization.md`` 是 S07「工程化与产品化」的运维口径
交付物，必须**存在**且正文覆盖下列关键锚点：

* 门禁三档（fast / medium / nightly）与 **SKIP 不算通过** 的语义；
* golden 基线（位置 / 生成命令 / ``--check``）+ **基线失效**（三指纹漂移）
  与「禁止直接重录掩盖回归」的处置口径；
* 资产与许可更新流程（registry → NOTICE → verify_env）；
* **运行时**厂商接口审计口径（forbidden_apis vs allowed_apis、
  forbidden_call_count vs non_whitelist_call_count、负例自检）；
* 长稳（8 h 真机执行的硬件门控 SKIP 语义）；
* **未验证**指标清单，且指向两个 canonical 唯一真相源的模块路径。

纪律
----
本测试**只读**：只读 git 跟踪的 runbook 文件与 ``src`` 下的模块路径，
**不读** ``.gsd/`` 等被忽略目录，**不**执行任何子进程，**不**产生副作用。
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNBOOK_PATH = REPO_ROOT / "runbooks" / "s07_engineering_productization.md"

#: 两个 canonical 未验证清单的**模块路径**（唯一真相源）。
CANONICAL_UNVERIFIED_PATHS = (
    "rm75_drake.tasks.metrics.UNVERIFIED_TASK_METRICS",
    "rm75_drake.device.hardware_gate.UNVERIFIED_HARDWARE_METRICS",
)


def _read_runbook() -> str:
    """读取 runbook 正文（要求文件存在）。"""
    assert RUNBOOK_PATH.is_file(), f"runbook 缺失：{RUNBOOK_PATH}"
    return RUNBOOK_PATH.read_text(encoding="utf-8")


def test_runbook_file_exists() -> None:
    """runbook 文件必须落盘。"""
    assert RUNBOOK_PATH.is_file(), f"runbook 缺失：{RUNBOOK_PATH}"
    assert RUNBOOK_PATH.stat().st_size > 0, "runbook 不得为空文件"


def test_runbook_covers_required_anchors() -> None:
    """正文必须覆盖各节的关键锚点。"""
    text = _read_runbook()
    anchors = {
        # 门禁三档
        "门禁三档": "门禁三档",
        "fast 档": "fast",
        "medium 档": "medium",
        "nightly 档": "nightly",
        "SKIP 语义": "SKIP",
        "SKIP 不算通过": "SKIP",
        # golden 基线
        "golden 基线": "golden",
        "golden 基线目录": "tests/data/golden/",
        "golden 生成器": "scripts/gen_golden.py",
        "golden --check": "--check",
        "基线失效": "基线失效",
        "三指纹漂移": "指纹",
        "禁止重录掩盖回归": "禁止",
        # 资产与许可
        "许可登记表": "licenses/registry.yaml",
        "NOTICE 成品": "NOTICE",
        "NOTICE 生成器": "tools/gen_notice.py",
        "许可门禁命令": "tools/verify_env.py --upstream --licenses",
        "sha256 校验": "sha256",
        # 运行时审计
        "运行时审计": "运行时",
        "禁用名单": "forbidden_apis",
        "白名单": "allowed_apis",
        "禁用计数": "forbidden_call_count",
        "非白名单计数": "non_whitelist_call_count",
        "负例自检": "负例",
        # 长稳
        "长稳载体": "scripts/long_run.py",
        "8 h 长稳": "8 h",
        "硬件门控 SKIP": "hardware_unreachable",
        # 未验证
        "未验证": "未验证",
        # 运维四段式
        "健康信号": "健康信号",
        "故障信号": "故障信号",
        "恢复流程": "恢复流程",
        "监控缺口": "监控缺口",
    }
    missing = [name for name, needle in anchors.items() if needle not in text]
    assert not missing, f"runbook 缺少关键锚点：{missing}"


def test_runbook_documents_skip_semantics() -> None:
    """SKIP 必须被写明「不算通过」。"""
    text = _read_runbook()
    assert "SKIP ≠ 通过" in text or "SKIP 不算通过" in text, (
        "runbook 必须写明 SKIP 不等于通过"
    )
    assert "[SKIP]" in text and "[FAIL]" in text and "[PASS]" in text, (
        "runbook 必须写明三档输出标签 [PASS] / [SKIP] / [FAIL]"
    )


def test_runbook_forbids_regenerating_golden_to_hide_regression() -> None:
    """必须写明「禁止直接重录基线掩盖回归」。"""
    text = _read_runbook()
    assert "基线失效" in text
    assert re.search(r"禁止.{0,20}(直接)?重录", text), (
        "runbook 必须写明禁止直接重录基线掩盖回归"
    )


def test_runbook_documents_runtime_audit_counter_semantics() -> None:
    """必须写明两个计数的语义差别与负例自检的必要性。"""
    text = _read_runbook()
    assert "forbidden_call_count" in text
    assert "non_whitelist_call_count" in text
    assert "负例" in text, "runbook 必须写明负例自检为何必须存在"


def test_runbook_documents_hardware_gate_skip_and_unverified_metrics() -> None:
    """必须写明硬件门控 SKIP 语义与「未验证」诚实边界。"""
    text = _read_runbook()
    assert "hardware_unreachable" in text
    assert "退出码仍为 0" in text, "SKIP 必须写明退出码仍为 0（K001 纪律）"
    assert "未验证" in text


def test_runbook_references_canonical_unverified_metric_paths() -> None:
    """必须引用两个 canonical 未验证清单的模块路径。"""
    text = _read_runbook()
    for dotted in CANONICAL_UNVERIFIED_PATHS:
        assert dotted in text, f"runbook 必须引用 canonical 未验证清单：{dotted}"


def test_canonical_unverified_paths_are_importable_modules() -> None:
    """两个模块路径必须**真实存在**（runbook 不得指向不存在的模块）。"""
    import importlib

    for dotted in CANONICAL_UNVERIFIED_PATHS:
        module_path, _, attr = dotted.rpartition(".")
        module = importlib.import_module(module_path)
        assert hasattr(module, attr), f"{dotted} 不存在（runbook 指向了不存在的真相源）"
