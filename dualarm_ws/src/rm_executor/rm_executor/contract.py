"""rm_executor 执行契约（**唯一真值**：轨迹 JSON / 执行配置 / 阈值 / 限值复检）。

本模块把 S03 后续所有任务（时钟、传输、闭环、入口、报告）依赖的**数据契约**
全部冻结成**模块级常量 + 纯函数**：

* 轨迹 JSON 契约（:func:`load_trajectory`）；
* 执行配置唯一来源（:func:`load_execution_config`，只读
  ``workcell/rm_workcell.yaml`` 的 ``execution:`` 段）；
* 反馈闭环阈值（模块级常量，来源可追溯）；
* 时间域限值复检（:func:`check_time_domain_limits`，数据源 = 硬件层的
  ``rm_hw_bridge.limits.load_joint_limits()`` 的 SI 值）。

三条硬约束：

1. **单位内部一律 rad**，只在调 SDK 前用 ``rm_hw_bridge`` 的 ``RAD2DEG``
   **单点**换算（执行层不另开换算处）；
2. **执行参数不落第二处常量源**——``period_s`` / ``follow`` / ``trajectory_mode``
   / ``radio`` / ``pre_position_speed`` 只从工位文件读取，缺字段 → blocked；
3. **不静默**：缺 ``units`` / ``schema_version`` 按 rad 推定并置
   ``schema_inferred=true``；缺 ``verify`` → blocked（除非显式
   ``allow_unverified``，仅供离线假执行）。

**零 rmctl 依赖、零 tesseract 依赖、零 SDK 调用、零时钟**（纯数据契约）。
"""
from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import yaml

from rm_executor import paths

__all__ = [
    "ARRIVAL_HOLD_MS",
    "ARRIVAL_SPEED_DEG_S",
    "ARRIVAL_TOL_DEG",
    "ARM_DOF",
    "EXECUTION_FIELDS",
    "EXECUTION_SECTION",
    "FOLLOW_STOP_DEG",
    "FOLLOW_WARN_DEG",
    "JOINT_GROUP",
    "JOINT_NAMES",
    "PRE_POSITION_HOLD_MS",
    "PRE_POSITION_TOL_DEG",
    "SCHEMA_VERSION",
    "STREAM_STOP_MS",
    "STRICT_REQUIRED_FIELDS",
    "STREAM_WARN_MS",
    "STATUS_BLOCKED",
    "STATUS_ERROR",
    "STATUS_OK",
    "UNITS",
    "check_time_domain_limits",
    "load_execution_config",
    "load_trajectory",
]

# ---------------------------------------------------------------------------
# 状态与退出码（与 rm_hw_bridge.errors 的 status→退出码口径一致：0 / 3 / 4）
# ---------------------------------------------------------------------------

#: 成功
STATUS_OK = "ok"
#: 需要人工 / 外部条件（缺文件、缺校验报告、配置缺字段……）
STATUS_BLOCKED = "blocked"
#: 运行错误（限值越界等硬红线）
STATUS_ERROR = "error"

#: status → 进程退出码（与 rm_hw_bridge.errors.EXIT_CODES 同口径）
EXIT_CODES: Dict[str, int] = {
    STATUS_OK: 0,
    STATUS_BLOCKED: 3,
    STATUS_ERROR: 4,
}

# ---------------------------------------------------------------------------
# 轨迹 JSON 契约
# ---------------------------------------------------------------------------

#: 轨迹 schema 版本（当前唯一版本）
SCHEMA_VERSION = 1

#: 轨迹所属分组（RM75 单臂 7 自由度）
JOINT_GROUP = "rm75_arm"

#: RM75 自由度
ARM_DOF = 7

#: 关节顺序（固定：joint1..joint7）
JOINT_NAMES: Tuple[str, ...] = tuple("joint%d" % i for i in range(1, ARM_DOF + 1))

#: 轨迹单位（**固定**，不接受其它单位）
UNITS: Dict[str, str] = {"times": "s", "points": "rad"}

#: 必需字段（完整契约形态）
#:
#: 注：``units`` / ``schema_version`` 缺失时按 rad 推定（结果置
#: ``schema_inferred=true``，**不静默**），因此**硬性**必需字段见
#: :data:`STRICT_REQUIRED_FIELDS`。
REQUIRED_FIELDS: Tuple[str, ...] = (
    "schema_version",
    "group",
    "joint_names",
    "units",
    "times",
    "points",
)

#: **硬性**必需字段（缺任一 → 直接拒绝，不推定）
STRICT_REQUIRED_FIELDS: Tuple[str, ...] = (
    "group",
    "joint_names",
    "times",
    "points",
)

#: 可选字段
OPTIONAL_FIELDS: Tuple[str, ...] = ("verify",)

# ---------------------------------------------------------------------------
# 执行配置（唯一来源 = workcell/rm_workcell.yaml 的 execution: 段）
# ---------------------------------------------------------------------------

#: 工位文件里的执行段名
EXECUTION_SECTION = "execution"

#: 执行段必需字段（缺任一 → blocked，**不得**在代码里补默认值）
EXECUTION_FIELDS: Tuple[str, ...] = (
    "period_s",
    "follow",
    "trajectory_mode",
    "radio",
    "pre_position_speed",
)

# ---------------------------------------------------------------------------
# 反馈闭环阈值（**冻结常量**，来源 = S03 切片契约 §5 / 设计 §5.7）
# ---------------------------------------------------------------------------

#: 预就位容差（度）
PRE_POSITION_TOL_DEG = 0.5
#: 预就位保持时长（毫秒）
PRE_POSITION_HOLD_MS = 200
#: 跟随误差告警阈值（度）
FOLLOW_WARN_DEG = 2.0
#: 跟随误差停机阈值（度）
FOLLOW_STOP_DEG = 5.0
#: 反馈断流告警阈值（毫秒）
STREAM_WARN_MS = 50
#: 反馈断流停机阈值（毫秒）
STREAM_STOP_MS = 200
#: 到位角误差阈值（度）
ARRIVAL_TOL_DEG = 0.1
#: 到位角速度阈值（度/秒）
ARRIVAL_SPEED_DEG_S = 0.5
#: 到位保持时长（毫秒）
ARRIVAL_HOLD_MS = 300

#: 限值复检的绝对容差（数值比较用）
LIMIT_EPS = 1e-9

#: 可粘贴的下一步：先用 rmctl plan 出轨迹
NEXT_ACTION_PLAN_FIRST = (
    "先用 `python3 -m rmctl.cli plan ... --out <traj.json>` 生成带 verify 报告的轨迹，"
    "再把该文件交给执行器"
)

#: 可粘贴的下一步：把硬件适配层加入 PYTHONPATH
NEXT_ACTION_HW_BRIDGE_PATH = (
    "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH  "
    "# 执行层的限值复检以 rm_hw_bridge.limits 为唯一数据源"
)


# --------------------------------------------------------------------- 工具
def _is_num(value: Any) -> bool:
    """是否为有限数值（bool 不算数值）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _result(status: str, reason: str = "", next_action: str = "",
            evidence: Optional[Iterable[Any]] = None, **extra: Any) -> Dict[str, Any]:
    """构造统一结果字典（与 rm_hw_bridge.errors 同构）。"""
    if status not in EXIT_CODES:
        raise ValueError("未知 status: %r（只允许 %s）" % (status, sorted(EXIT_CODES)))
    out: Dict[str, Any] = {
        "status": status,
        "reason": reason,
        "next_action": next_action,
        "evidence": [e if isinstance(e, str) else str(e) for e in (evidence or [])],
    }
    out.update(extra)
    return out


def _blocked(reason: str, next_action: str, evidence: Optional[Iterable[Any]] = None,
             **extra: Any) -> Dict[str, Any]:
    """被阻塞结果（退出码 3）。"""
    return _result(STATUS_BLOCKED, reason, next_action, evidence, **extra)


def _ok(reason: str = "", next_action: str = "", evidence: Optional[Iterable[Any]] = None,
        **extra: Any) -> Dict[str, Any]:
    """成功结果（退出码 0）。"""
    return _result(STATUS_OK, reason, next_action, evidence, **extra)


def _fmt(value: float, digits: int = 6) -> str:
    return ("%." + str(digits) + "f") % float(value)


# ------------------------------------------------------------- 轨迹契约读取
def load_trajectory(path: str, allow_unverified: bool = False) -> Dict[str, Any]:
    """加载并校验轨迹 JSON。

    校验规则（任一不过 → ``status="blocked"``，**不抛裸异常**）：

    * 文件存在且是合法 JSON、顶层是对象；
    * ``times`` 长度 ≥ 2 且**严格递增**，元素为有限数；
    * ``points`` 行数 == ``len(times)``，每行恰好 7 个有限数；
    * ``joint_names`` 长度 7 且顺序 == ``joint1..joint7``；
    * ``group`` == ``rm75_arm``；
    * ``units`` 固定 ``{"times": "s", "points": "rad"}``；
    * ``schema_version`` == 1；
    * ``verify`` 存在且 ``verify.ok is True``。

    **不静默的兼容**：缺 ``units`` / ``schema_version`` → 按 rad 推定，
    结果里 ``schema_inferred=true``；缺 ``verify`` → blocked（除非显式
    ``allow_unverified=True``，仅供离线假执行，放行会写进 evidence）。

    返回
    ----
    dict
        成功：``{"status": "ok", "trajectory": {...}, "schema_inferred": bool,
        "reason": ..., "next_action": "", "evidence": [...]}``；
        失败：``{"status": "blocked", "trajectory": None, "reason": ...,
        "next_action": ..., "evidence": [...]}``。
    """
    path = os.path.abspath(path)
    evidence: List[str] = ["trajectory: %s" % path]

    # 1) 文件可读
    if not os.path.isfile(path):
        return _blocked(
            "找不到轨迹文件: %s" % path,
            "确认轨迹路径正确；轨迹由 `python3 -m rmctl.cli plan` 生成，"
            "不存在时先出轨迹再交给执行器",
            evidence,
            trajectory=None,
            schema_inferred=False,
        )
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError) as exc:
        return _blocked(
            "轨迹文件无法解析为 JSON（%s）: %s" % (type(exc).__name__, exc),
            "修复 %s 的 JSON 格式后重跑（执行器不猜测轨迹内容）" % path,
            evidence + [repr(exc)],
            trajectory=None,
            schema_inferred=False,
        )
    if not isinstance(data, dict):
        return _blocked(
            "轨迹 JSON 顶层必须是对象（当前是 %s）" % type(data).__name__,
            "用 `python3 -m rmctl.cli plan` 重新导出轨迹后重跑",
            evidence,
            trajectory=None,
            schema_inferred=False,
        )

    # 2) 字段存在性
    missing = [f for f in STRICT_REQUIRED_FIELDS if f not in data]
    if missing:
        return _blocked(
            "轨迹缺少必需字段: %s" % ", ".join(missing),
            "轨迹 JSON 必须包含 %s（units 固定 %s）；先用 rmctl plan 出轨迹"
            % (", ".join(REQUIRED_FIELDS), json.dumps(UNITS)),
            evidence,
            trajectory=None,
            schema_inferred=False,
        )

    # 3) 单位 / 版本（缺 → 推定并标记；不符 → 拒绝）
    schema_inferred = False
    units = data.get("units")
    if units is None:
        units = dict(UNITS)
        schema_inferred = True
        evidence.append("units 缺失 → 按 rad 推定（schema_inferred=true）")
    elif not isinstance(units, dict) or units.get("times") != "s" or units.get("points") != "rad":
        return _blocked(
            "轨迹单位不符：units=%r（只接受 {\"times\": \"s\", \"points\": \"rad\"}）"
            % (units,),
            "把轨迹单位统一转成 s / rad（内部一律 rad）后重跑；"
            "若轨迹是度制，先做一次 deg→rad 换算并写回 units",
            evidence,
            trajectory=None,
            schema_inferred=False,
        )

    version = data.get("schema_version")
    if version is None:
        version = SCHEMA_VERSION
        schema_inferred = True
        evidence.append("schema_version 缺失 → 按 %d 推定（schema_inferred=true）"
                        % SCHEMA_VERSION)
    elif version != SCHEMA_VERSION:
        return _blocked(
            "轨迹 schema_version=%r，当前只支持 %d" % (version, SCHEMA_VERSION),
            "用与本执行器匹配的规划端重新导出轨迹（schema_version=%d）后重跑"
            % SCHEMA_VERSION,
            evidence,
            trajectory=None,
            schema_inferred=False,
        )

    # 4) group
    group = data.get("group")
    if group != JOINT_GROUP:
        return _blocked(
            "轨迹 group=%r，当前只支持 %r" % (group, JOINT_GROUP),
            "用 rmctl 为 %s 规划轨迹后重跑" % JOINT_GROUP,
            evidence,
            trajectory=None,
            schema_inferred=False,
        )

    # 5) joint_names
    joint_names = data.get("joint_names")
    if (not isinstance(joint_names, (list, tuple))
            or len(joint_names) != ARM_DOF
            or list(joint_names) != list(JOINT_NAMES)):
        return _blocked(
            "joint_names 不合法：%r（必须是 %d 个且顺序为 %s）"
            % (joint_names, ARM_DOF, list(JOINT_NAMES)),
            "让轨迹携带固定顺序的 joint1..joint7 关节名后重跑",
            evidence,
            trajectory=None,
            schema_inferred=False,
        )

    # 6) times
    times_raw = data.get("times")
    if not isinstance(times_raw, (list, tuple)):
        return _blocked(
            "times 必须是列表（当前是 %s）" % type(times_raw).__name__,
            "修复轨迹 times 字段后重跑",
            evidence, trajectory=None, schema_inferred=schema_inferred,
        )
    times: List[float] = []
    for idx, value in enumerate(times_raw):
        if not _is_num(value):
            return _blocked(
                "times[%d] 不是有限数: %r" % (idx, value),
                "修复轨迹 times 字段后重跑",
                evidence, trajectory=None, schema_inferred=schema_inferred,
            )
        times.append(float(value))
    if len(times) < 2:
        return _blocked(
            "times 长度必须 ≥ 2（当前 %d）" % len(times),
            "轨迹至少要有起点与终点两个采样点",
            evidence, trajectory=None, schema_inferred=schema_inferred,
        )
    for idx in range(1, len(times)):
        if not times[idx] > times[idx - 1]:
            return _blocked(
                "times 必须严格递增（第 %d 个采样点 %s ≤ 前一个 %s）"
                % (idx, _fmt(times[idx]), _fmt(times[idx - 1])),
                "修复轨迹时间轴（严格递增）后重跑；执行器不做时间轴重排",
                evidence, trajectory=None, schema_inferred=schema_inferred,
            )

    # 7) points
    points_raw = data.get("points")
    if not isinstance(points_raw, (list, tuple)):
        return _blocked(
            "points 必须是列表（当前是 %s）" % type(points_raw).__name__,
            "修复轨迹 points 字段后重跑",
            evidence, trajectory=None, schema_inferred=schema_inferred,
        )
    if len(points_raw) != len(times):
        return _blocked(
            "points 行数 %d 与 times 长度 %d 不一致" % (len(points_raw), len(times)),
            "修复轨迹点位与时间轴的对应关系后重跑",
            evidence, trajectory=None, schema_inferred=schema_inferred,
        )
    points: List[List[float]] = []
    for r, row in enumerate(points_raw):
        if not isinstance(row, (list, tuple)) or len(row) != ARM_DOF:
            return _blocked(
                "points[%d] 必须是 %d 个关节角（当前 %r）"
                % (r, ARM_DOF, row),
                "每行必须是 %d 个关节角（rad）" % ARM_DOF,
                evidence, trajectory=None, schema_inferred=schema_inferred,
            )
        out_row: List[float] = []
        for c, value in enumerate(row):
            if not _is_num(value):
                return _blocked(
                    "points[%d][%d] 不是有限数: %r" % (r, c, value),
                    "修复轨迹点位数值后重跑",
                    evidence, trajectory=None, schema_inferred=schema_inferred,
                )
            out_row.append(float(value))
        points.append(out_row)

    # 8) verify（上游四道校验报告）
    verify = data.get("verify")
    if verify is None:
        if not allow_unverified:
            return _blocked(
                "轨迹未携带 verify 报告（上游四道校验未随轨迹落盘）",
                NEXT_ACTION_PLAN_FIRST
                + "；若只是离线假执行，可显式加 --allow-unverified 放行"
                "（该放行仅供离线验证，不代表真机已校验）",
                evidence, trajectory=None, schema_inferred=schema_inferred,
            )
        evidence.append("verify 缺失，但调用方显式 allow_unverified → 放行"
                        "（仅限离线假执行，不代表真机已校验）")
        verify = None
    else:
        if not isinstance(verify, dict) or "ok" not in verify:
            return _blocked(
                "verify 报告格式不合法（必须是含 ok 布尔字段的对象，当前 %r）" % (verify,),
                NEXT_ACTION_PLAN_FIRST,
                evidence, trajectory=None, schema_inferred=schema_inferred,
            )
        if verify.get("ok") is not True:
            failed = []
            checks = verify.get("checks")
            if isinstance(checks, dict):
                failed = [k for k, v in checks.items()
                          if isinstance(v, dict) and v.get("ok") is False]
            return _blocked(
                "上游四道校验未通过（verify.ok=false%s）"
                % ("，失败项: %s" % ", ".join(failed) if failed else ""),
                NEXT_ACTION_PLAN_FIRST
                + "；修正后重新导出带 verify 报告的轨迹",
                evidence, trajectory=None, schema_inferred=schema_inferred,
            )

    trajectory = {
        "schema_version": SCHEMA_VERSION,
        "group": JOINT_GROUP,
        "joint_names": list(JOINT_NAMES),
        "units": dict(UNITS),
        "times": times,
        "points": points,
        "verify": verify,
    }
    return _ok(
        reason="轨迹契约校验通过（%d 个采样点，%.3f s，schema_inferred=%s）"
               % (len(times), times[-1] - times[0], schema_inferred),
        evidence=evidence,
        trajectory=trajectory,
        schema_inferred=schema_inferred,
    )


# --------------------------------------------------------- 执行配置唯一来源
def load_execution_config(ws: str = None) -> Dict[str, Any]:
    """读取执行配置（**唯一来源**：``workcell/rm_workcell.yaml`` 的 ``execution:`` 段）。

    必需字段：``period_s`` / ``follow`` / ``trajectory_mode`` / ``radio`` /
    ``pre_position_speed``。缺任一字段或类型不对 → ``status="blocked"``，
    并给出可粘贴的下一步（**执行层不补默认值、不新增第二处常量源**）。

    返回
    ----
    dict
        成功：``{"status": "ok", "config": {...}, "reason", "next_action",
        "evidence"}``；失败：``status="blocked"``。
    """
    cfg_path = paths.workcell_config_path(ws)
    evidence: List[str] = ["workcell: %s" % cfg_path]

    if not os.path.isfile(cfg_path):
        return _blocked(
            "找不到工位定义文件: %s" % cfg_path,
            "确认工作区根（含 workcell/rm_workcell.yaml）；"
            "可用 `export RM_WS=<仓库根>` 指定后重跑",
            evidence, config=None,
        )
    try:
        with open(cfg_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001 - YAML 解析异常统一转 blocked
        return _blocked(
            "工位定义文件解析失败（%s）: %s" % (type(exc).__name__, exc),
            "修复 %s 的 YAML 格式后重跑" % cfg_path,
            evidence + [repr(exc)], config=None,
        )
    if not isinstance(data, dict):
        return _blocked(
            "工位定义文件格式错误（顶层必须是映射，当前 %s）" % type(data).__name__,
            "修复 %s 的 YAML 格式后重跑" % cfg_path,
            evidence, config=None,
        )
    section = data.get(EXECUTION_SECTION)
    if not isinstance(section, dict):
        return _blocked(
            "工位定义文件缺少 %s: 段（当前是 %r）" % (EXECUTION_SECTION, section),
            "在 %s 的 %s: 段补齐 %s 后重跑"
            % (cfg_path, EXECUTION_SECTION, ", ".join(EXECUTION_FIELDS)),
            evidence, config=None,
        )
    missing = [f for f in EXECUTION_FIELDS if f not in section]
    if missing:
        return _blocked(
            "执行配置缺字段: %s" % ", ".join(missing),
            "在 %s 的 %s: 段补齐 %s 后重跑（执行层不提供默认值）"
            % (cfg_path, EXECUTION_SECTION, ", ".join(EXECUTION_FIELDS)),
            evidence, config=None,
        )

    period_s = section.get("period_s")
    if not _is_num(period_s) or float(period_s) <= 0:
        return _blocked(
            "execution.period_s 非法: %r（必须是正数，单位 s）" % (period_s,),
            "在 %s 把 execution.period_s 设为正的秒数（如 0.010）后重跑" % cfg_path,
            evidence, config=None,
        )
    follow = section.get("follow")
    if not isinstance(follow, bool):
        return _blocked(
            "execution.follow 非法: %r（必须是布尔）" % (follow,),
            "在 %s 把 execution.follow 设为 true/false 后重跑" % cfg_path,
            evidence, config=None,
        )
    trajectory_mode = section.get("trajectory_mode")
    if not isinstance(trajectory_mode, int) or isinstance(trajectory_mode, bool):
        return _blocked(
            "execution.trajectory_mode 非法: %r（必须是整数）" % (trajectory_mode,),
            "在 %s 把 execution.trajectory_mode 设为整数（1=曲线拟合）后重跑" % cfg_path,
            evidence, config=None,
        )
    radio = section.get("radio")
    if not isinstance(radio, int) or isinstance(radio, bool) or radio <= 0:
        return _blocked(
            "execution.radio 非法: %r（必须是正整数）" % (radio,),
            "在 %s 把 execution.radio 设为正整数（曲线拟合倍率）后重跑" % cfg_path,
            evidence, config=None,
        )
    pre_position_speed = section.get("pre_position_speed")
    if not _is_num(pre_position_speed) or not (0 < float(pre_position_speed) <= 100):
        return _blocked(
            "execution.pre_position_speed 非法: %r（必须是 (0, 100] 的数值）"
            % (pre_position_speed,),
            "在 %s 把 execution.pre_position_speed 设为 (0,100] 的速度百分比后重跑"
            % cfg_path,
            evidence, config=None,
        )

    config = {
        "period_s": float(period_s),
        "follow": bool(follow),
        "trajectory_mode": int(trajectory_mode),
        "radio": int(radio),
        "pre_position_speed": float(pre_position_speed),
    }
    return _ok(
        reason="执行配置来自 %s::execution（唯一来源）" % cfg_path,
        evidence=evidence,
        config=config,
    )


# ------------------------------------------------------------- 限值复检
def _default_joint_limits(ws: Optional[str]) -> Tuple[Optional[Dict[str, Any]], str]:
    """取声明限值（SI）：唯一来源 = ``rm_hw_bridge.limits.load_joint_limits()``。

    返回 ``(limits, error)``；``error`` 非空表示无法取得限值（**不伪造限值**）。
    """
    try:
        from rm_hw_bridge import limits as hw_limits  # 惰性导入：硬件层是唯一限值源
    except Exception as exc:  # noqa: BLE001
        return None, "无法导入 rm_hw_bridge.limits（%s）: %s" % (type(exc).__name__, exc)
    try:
        return hw_limits.load_joint_limits(ws), ""
    except Exception as exc:  # noqa: BLE001
        return None, "读取声明限值失败（%s）: %s" % (type(exc).__name__, exc)


def _diff(values: Sequence[float], times: Sequence[float]) -> List[float]:
    """中心差分（首尾用单侧差分）——数值导数。"""
    n = len(values)
    out: List[float] = []
    for i in range(n):
        if n < 2:
            out.append(0.0)
            continue
        if i == 0:
            dt = times[1] - times[0]
            out.append((values[1] - values[0]) / dt if dt > 0 else 0.0)
        elif i == n - 1:
            dt = times[n - 1] - times[n - 2]
            out.append((values[n - 1] - values[n - 2]) / dt if dt > 0 else 0.0)
        else:
            dt = times[i + 1] - times[i - 1]
            out.append((values[i + 1] - values[i - 1]) / dt if dt > 0 else 0.0)
    return out


def check_time_domain_limits(times: Sequence[float], points: Sequence[Sequence[float]],
                             ws: str = None,
                             limits: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """时间域限值复检（位置 / 速度 / 加速度 / jerk）。

    限值来源 = ``rm_hw_bridge.limits.load_joint_limits()`` 的 **SI 值**
    （rad / rad·s⁻¹ / rad·s⁻² / rad·s⁻³）；本函数**不做任何单位换算**。

    导数口径：中心差分（首尾单侧差分）逐阶求导。**分段线性轨迹在拐点处会
    出现真实的加速度/jerk 尖峰**——这是限值意义上的真实越界，故按越界处理
    （宁可报越界，不静默放过）。

    参数
    ----
    limits
        可选的限值覆盖（测试可注入；``None`` → 从硬件层读取）。

    返回
    ----
    dict
        ``{"status": "ok"|"blocked", "reason", "next_action", "evidence",
        "violations": [...]}``。
    """
    evidence: List[str] = []
    if limits is None:
        limits, error = _default_joint_limits(ws)
        if error:
            return _blocked(
                "时间域限值复检无法进行：%s" % error,
                NEXT_ACTION_HW_BRIDGE_PATH,
                evidence + [error],
                violations=[],
            )
        evidence.append("限值来源: rm_hw_bridge.limits.load_joint_limits()")
    else:
        evidence.append("限值来源: 调用方注入")

    n = len(times)
    if n < 2 or len(points) != n:
        return _blocked(
            "时间域限值复检入参不合法（times=%d 点，points=%d 行）" % (n, len(points)),
            "传入严格递增的 times 与等长 points 后重跑",
            evidence, violations=[],
        )

    violations: List[str] = []
    for j, joint in enumerate(JOINT_NAMES):
        item = limits.get(joint)
        if not isinstance(item, dict):
            return _blocked(
                "限值数据缺关节 %s（limits 结构不合法）" % joint,
                "确认 rm_hw_bridge.limits.load_joint_limits() 返回 7 个关节的完整限值",
                evidence, violations=[],
            )
        series = [float(row[j]) for row in points]
        vel = _diff(series, times)
        acc = _diff(vel, times)
        jerk = _diff(acc, times)

        min_pos = float(item["min_pos"])
        max_pos = float(item["max_pos"])
        max_speed = float(item["max_speed"])
        max_acc = float(item["max_acc"])
        max_jerk = float(item["jerk"])

        for i, value in enumerate(series):
            if value < min_pos - LIMIT_EPS or value > max_pos + LIMIT_EPS:
                violations.append(
                    "%s 位置越界 @t=%.4f s: %s rad 不在 [%s, %s]"
                    % (joint, times[i], _fmt(value), _fmt(min_pos), _fmt(max_pos))
                )
        for i, value in enumerate(vel):
            if abs(value) > max_speed + LIMIT_EPS:
                violations.append(
                    "%s 速度越界 @t=%.4f s: %s rad/s > %s rad/s"
                    % (joint, times[i], _fmt(abs(value)), _fmt(max_speed))
                )
        for i, value in enumerate(acc):
            if abs(value) > max_acc + LIMIT_EPS:
                violations.append(
                    "%s 加速度越界 @t=%.4f s: %s rad/s² > %s rad/s²"
                    % (joint, times[i], _fmt(abs(value)), _fmt(max_acc))
                )
        for i, value in enumerate(jerk):
            if abs(value) > max_jerk + LIMIT_EPS:
                violations.append(
                    "%s jerk 越界 @t=%.4f s: %s rad/s³ > %s rad/s³"
                    % (joint, times[i], _fmt(abs(value)), _fmt(max_jerk))
                )

    if violations:
        head = violations[:20]
        return _blocked(
            "时间域限值复检未通过（%d 项越界）" % len(violations),
            "用 `python3 -m rmctl.cli plan` 重新规划（降低幅度/速度）或先 retime "
            "再导出轨迹；越界明细见 evidence（执行器不会自行放宽限值）",
            evidence + head,
            violations=violations,
        )
    return _ok(
        reason="时间域限值复检通过（位置/速度/加速度/jerk，%d 个采样点）" % n,
        evidence=evidence,
        violations=[],
    )
