"""真机验证**阶梯**（RM75-B / 192.168.1.18）。

这个包把「在真实机械臂上从简单到复杂完成整套验证」拆成 **15 级阶梯**
（L0–L14），每一级都有可机器判定的**判据**、明确的**证据**与**门禁**：

* :mod:`rm75_drake.validation.ladder` —— 阶梯**定义**与判定（纯模型，零硬件）；
* :mod:`rm75_drake.validation.hardware` —— 真机**执行器**（产出指标）。

配套文档：``docs/validation/RM75-B_真机验证阶梯.md``；
入口脚本：``scripts/real_machine_ladder.py``。
"""

from __future__ import annotations

from .ladder import (
    STATUS_BLOCKED,
    STATUS_FAIL,
    STATUS_MANUAL,
    STATUS_PASS,
    STATUS_SKIP,
    TIERS,
    Criterion,
    LadderContext,
    LadderError,
    Stage,
    StageResult,
    build_ladder,
    evaluate_criteria,
    ladder_report,
    run_ladder,
    stage_by_id,
)

__all__ = [
    "STATUS_PASS",
    "STATUS_FAIL",
    "STATUS_SKIP",
    "STATUS_BLOCKED",
    "STATUS_MANUAL",
    "TIERS",
    "Criterion",
    "Stage",
    "StageResult",
    "LadderContext",
    "LadderError",
    "build_ladder",
    "stage_by_id",
    "evaluate_criteria",
    "run_ladder",
    "ladder_report",
]
