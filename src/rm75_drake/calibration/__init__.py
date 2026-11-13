"""标定层（S04）：标定文件 schema + 采样 schema + 纯 numpy 求解器 + 残差报告。

分层
----
* :mod:`rm75_drake.calibration.schema` —— **标定文件**的正式 schema 与严格校验
  （未知键 / 缺字段 / 维度错 / 单位错一律抛
  :class:`rm75_drake.calibration.schema.CalibrationError` 并**点名该键**）。
* :mod:`rm75_drake.calibration.samples` —— **采样点集**的 schema
  （关节零位 +10° 对比记录 / 工具 TCP 点对 / 基座外参尖对尖点对；
  每条采样必须带 ``synthetic`` 与 ``source``）。
* :mod:`rm75_drake.calibration.solvers` —— **三个求解器**（纯 numpy）：
  关节零位偏差、工具 TCP 6D（Kabsch）、双臂基座外参 ``X_LR``（Kabsch）。
* :mod:`rm75_drake.calibration.report` —— **残差 / 重复性 / 第三方交叉验证**报告器；
  ``measured=false`` 时显式声明「未实测」并逐条列出未验证项。

**零 pydrake 依赖**：本子包**只**依赖 numpy + 标准库 + PyYAML，
因此标定内核可离线单测、不被 Drake 版本绑死（模型 FK 由调用方以
``fk_fn`` 注入，生产用 :func:`rm75_drake.kinematics.mdh.fk`）。

**失败不静默**：残差超阈值 → 抛 :class:`CalibrationError`（点名超差量与阈值），
**禁止**返回「看起来正常」的结果。

**本环境无实测数据**（无真机、SDK 未安装）：本子包交付的是**算法内核**，
真实标定精度、真实共同点 < 0.5 mm 等**物理指标一律走硬件门控 SKIP**，
不得用合成数据的残差冒充实测精度。
"""

from __future__ import annotations

from .report import (
    UNVERIFIED_ITEMS,
    CalibrationReport,
    build_report,
    cross_validation_summary,
    repeatability_summary,
)
from .samples import (
    MIN_BASE_EXTRINSIC_SAMPLES,
    MIN_TOOL_TCP_SAMPLES,
    BaseExtrinsicSample,
    JointZeroSample,
    SampleSet,
    ToolTcpSample,
    load_samples_file,
    parse_samples,
)
from .schema import (
    REQUIRED_TOP_LEVEL_KEYS,
    SCHEMA_VERSION,
    TOP_LEVEL_KEYS,
    BaseExtrinsicEntry,
    CalibrationDoc,
    CalibrationError,
    Pose6D,
    ResidualSummary,
    ToolOffsetEntry,
    load_calibration_file,
    matrix_to_rpy,
    parse_calibration,
    rpy_to_matrix,
)
from .solvers import (
    DEFAULT_BASE_EXTRINSIC_THRESHOLD_M,
    DEFAULT_JOINT_ZERO_THRESHOLD_M,
    DEFAULT_TOOL_TCP_THRESHOLD_M,
    BaseExtrinsicSolution,
    JointZeroSolution,
    ResidualReport,
    ToolTcpSolution,
    kabsch,
    solve_base_extrinsic,
    solve_joint_zero_offsets,
    solve_tool_tcp,
)

__all__ = [
    # schema
    "CalibrationError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "REQUIRED_TOP_LEVEL_KEYS",
    "CalibrationDoc",
    "Pose6D",
    "ResidualSummary",
    "BaseExtrinsicEntry",
    "ToolOffsetEntry",
    "parse_calibration",
    "load_calibration_file",
    "rpy_to_matrix",
    "matrix_to_rpy",
    # samples
    "JointZeroSample",
    "ToolTcpSample",
    "BaseExtrinsicSample",
    "SampleSet",
    "parse_samples",
    "load_samples_file",
    "MIN_TOOL_TCP_SAMPLES",
    "MIN_BASE_EXTRINSIC_SAMPLES",
    # solvers
    "ResidualReport",
    "JointZeroSolution",
    "ToolTcpSolution",
    "BaseExtrinsicSolution",
    "solve_joint_zero_offsets",
    "solve_tool_tcp",
    "solve_base_extrinsic",
    "kabsch",
    "DEFAULT_JOINT_ZERO_THRESHOLD_M",
    "DEFAULT_TOOL_TCP_THRESHOLD_M",
    "DEFAULT_BASE_EXTRINSIC_THRESHOLD_M",
    # report
    "CalibrationReport",
    "build_report",
    "repeatability_summary",
    "cross_validation_summary",
    "UNVERIFIED_ITEMS",
]
