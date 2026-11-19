"""限值默认值来源：**measured_limits 基线**（R004「限值下发口径与基线同源且可追溯」）。

本模块是硬件适配层里「限值从哪来」的唯一出处：

* 默认来源 = ``workcell/baselines/measured_limits.yaml``（真机实测基线，git 跟踪）；
* **基线未实测（``status != "measured"``）→ 拒绝下发**（返回 ``status="blocked"``），
  绝不用占位值糊过去；
* 唯一的保守回退：调用方**显式** ``allow_placeholder=True`` 时，才回退到
  ``dualarm_ws/src/rm_description/config/joint_limits.yaml`` 的现有限值，
  且结果必须标注 ``source`` 含 ``"placeholder, 未实测"``。

**只读不写**：conservative（取 min）的回填口径在
``dualarm_ws/src/rm_description/scripts/apply_measured_limits.py`` 里，
本模块**不写任何文件**。

**单位口径（内部一律 SI）**：

===================  ==========  ================================================
量                   单位        来源
===================  ==========  ================================================
``max_speed``        rad/s       ``joint_limits.yaml`` 的 ``velocity``
``max_acc``          rad/s^2     实测 ``acceleration``（占位回退时用 joint_limits）
``jerk``             rad/s^3     实测 ``jerk``（占位回退时用 joint_limits）
``min_pos``          rad         ``joint_limits.yaml`` 的 ``lower``
``max_pos``          rad         ``joint_limits.yaml`` 的 ``upper``
===================  ==========  ================================================

度↔弧度的换算**只在** ``rm_hw_bridge/bridge.py`` 一处发生（``DEG2RAD`` / ``RAD2DEG``），
本模块不做任何换算。

**零 rmctl 依赖**：硬件层不得反向依赖体验层（``contract.yaml: rules``）。
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml

from rm_hw_bridge import paths

__all__ = [
    "ARM_DOF",
    "JOINT_NAMES",
    "MIN_SAMPLES",
    "PLACEHOLDER_SOURCE",
    "STATUS_BLOCKED",
    "STATUS_MEASURED",
    "STATUS_PLACEHOLDER",
    "load_joint_limits",
    "load_limit_defaults",
    "measured_limits_path",
]

#: RM75 自由度（与 snapshot.ARM_DOF 一致）
ARM_DOF = 7

#: 关节名（joint1..joint7）
JOINT_NAMES: tuple = tuple("joint%d" % i for i in range(1, ARM_DOF + 1))

#: 实测样本数下限（与 apply_measured_limits.py 的 MIN_SAMPLES 同口径）
MIN_SAMPLES = 3

#: ``joint_limits.yaml`` 里的分组名
JOINT_LIMITS_GROUP = "rm75_arm"

#: 状态取值
STATUS_MEASURED = "measured"
STATUS_PLACEHOLDER = "placeholder"
STATUS_BLOCKED = "blocked"

#: 占位回退时 ``source`` 必须包含的标注
PLACEHOLDER_SOURCE = "joint_limits.yaml (placeholder, 未实测)"

#: 下发面（4 项 × 7 关节）：驱动级生效的限值族
DOWNLINK_FIELDS = ("max_speed", "max_acc", "min_pos", "max_pos")

#: 阻断时的可粘贴下一步（采集 → 回填基线 → 预览回填限值文件）
_NEXT_ACTION_MEASURE = (
    "取得真机窗口后执行 `python3 -m rmctl.hwcollect ladder_acc_jerk --arm left --trials 3` "
    "采集 acc/ jerk 实测值，回填 %s（status 改 measured，每条补齐 source/method/samples≥3），"
    "再用 `python3 dualarm_ws/src/rm_description/scripts/apply_measured_limits.py --dry-run` "
    "预览回填（流程见 docs/code_doc/真机基线与实测限值归档说明.md 第 3 节）"
)

#: 占位回退的告警下一步
_NEXT_ACTION_PLACEHOLDER = (
    "占位限值未实测，仅用于临时在线降速；请在真机窗口内按上一条流程采集实测值后重跑"
)


# --------------------------------------------------------------------- 工具
def measured_limits_path(ws: Optional[str] = None) -> str:
    """实测限值基线路径（``<ws>/workcell/baselines/measured_limits.yaml``）。"""
    return os.path.join(paths.baselines_dir(ws), "measured_limits.yaml")


def _read_yaml(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _blank(v: Any) -> bool:
    return v is None or (isinstance(v, str) and not v.strip())


def _blocked(reason: str, next_action: str, evidence: List[str],
             source: str = "", values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """构造「拒绝下发」结果（无伪造数值）。"""
    return {
        "status": STATUS_BLOCKED,
        "source": source,
        "values": values or {},
        "current": {},
        "reason": reason,
        "next_action": next_action,
        "evidence": list(evidence),
    }


# ------------------------------------------------------------- 声明限值读取
def load_joint_limits(ws: Optional[str] = None) -> Dict[str, Any]:
    """读取 ``joint_limits.yaml`` 的声明限值（每关节 SI 值）。

    返回 ``{joint: {"max_speed": rad/s, "max_acc": rad/s^2, "jerk": rad/s^3,
    "min_pos": rad, "max_pos": rad}}``；文件缺失 / 格式错误时抛 ``FileNotFoundError``
    / ``ValueError``（调用方转成 blocked 结果，不静默吞）。
    """
    path = paths.joint_limits_path(ws)
    if not os.path.isfile(path):
        raise FileNotFoundError("找不到关节限值文件: %s" % path)
    data = _read_yaml(path)
    if not isinstance(data, dict):
        raise ValueError("关节限值文件格式错误（顶层必须是映射）: %s" % path)
    group = data.get(JOINT_LIMITS_GROUP)
    if not isinstance(group, dict):
        raise ValueError("关节限值文件缺少分组 %s: %s" % (JOINT_LIMITS_GROUP, path))

    out: Dict[str, Any] = {}
    for name in JOINT_NAMES:
        item = group.get(name)
        if not isinstance(item, dict):
            raise ValueError("关节限值文件缺少关节 %s: %s" % (name, path))
        lower, upper = item.get("lower"), item.get("upper")
        velocity, acc, jerk = (
            item.get("velocity"), item.get("acceleration"), item.get("jerk"),
        )
        for label, value in (("lower", lower), ("upper", upper), ("velocity", velocity),
                             ("acceleration", acc), ("jerk", jerk)):
            if not _is_num(value):
                raise ValueError(
                    "关节限值文件 %s.%s 不是数值（%r）: %s" % (name, label, value, path)
                )
        # **规划域 vs 执行域**分开：
        #   * min_pos/max_pos           = 执行域（真机机械范围）→ 驱动级限位下发 +
        #     执行层时间域位置复检（防止下发超出机械范围的指令）；
        #   * plan_min_pos/plan_max_pos = 规划域（规划器 / 四道校验用）。
        # 缺省时执行域 = 规划域（绝大多数关节如此）。
        exec_lower = item.get("execution_lower")
        exec_upper = item.get("execution_upper")
        for label, value in (("execution_lower", exec_lower),
                             ("execution_upper", exec_upper)):
            if value is not None and not _is_num(value):
                raise ValueError(
                    "关节限值文件 %s.%s 不是数值（%r）: %s" % (name, label, value, path)
                )
        min_pos = float(lower) if exec_lower is None else float(exec_lower)
        max_pos = float(upper) if exec_upper is None else float(exec_upper)
        if min_pos > float(lower) or max_pos < float(upper):
            raise ValueError(
                "关节限值文件 %s 的执行域（[%s, %s]）不得窄于规划域（[%s, %s]）: %s"
                % (name, min_pos, max_pos, lower, upper, path)
            )
        out[name] = {
            "max_speed": float(velocity),
            "max_acc": float(acc),
            "jerk": float(jerk),
            "min_pos": min_pos,
            "max_pos": max_pos,
            "plan_min_pos": float(lower),
            "plan_max_pos": float(upper),
        }
    return out


# --------------------------------------------------------------- 实测基线读取
def _measured_joint_values(data: Dict[str, Any], path: str) -> Dict[str, Dict[str, Any]]:
    """从基线里抽每关节 acceleration / jerk（附证据）。

    缺失 / 非数值 / 缺 source|method / samples < 3 → 抛 ``ValueError``
    （与 ``apply_measured_limits.py`` 的三条红线同口径）。
    """
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != ARM_DOF:
        raise ValueError(
            "实测限值基线的 joints 必须是 %d 条（当前 %r）: %s"
            % (ARM_DOF, len(joints) if isinstance(joints, list) else joints, path)
        )
    by_name: Dict[str, Dict[str, Any]] = {}
    for item in joints:
        if not isinstance(item, dict) or not item.get("name"):
            raise ValueError("实测限值基线 joints 缺少 name: %s" % path)
        by_name[str(item["name"])] = item

    out: Dict[str, Dict[str, Any]] = {}
    for name in JOINT_NAMES:
        item = by_name.get(name)
        if item is None:
            raise ValueError("实测限值基线缺少关节 %s: %s" % (name, path))
        out[name] = {}
        for field in ("acceleration", "jerk"):
            rec = item.get(field)
            if not isinstance(rec, dict):
                raise ValueError(
                    "实测限值基线 %s.%s 缺失（基线未实测）: %s" % (name, field, path)
                )
            value = rec.get("value")
            if not _is_num(value):
                raise ValueError(
                    "实测限值基线 %s.%s.value 为空（基线未实测）: %s" % (name, field, path)
                )
            if _blank(rec.get("source")) or _blank(rec.get("method")):
                raise ValueError(
                    "实测限值基线 %s.%s 缺 source/method（不可追溯）: %s" % (name, field, path)
                )
            samples = rec.get("samples")
            if not isinstance(samples, int) or isinstance(samples, bool) or samples < MIN_SAMPLES:
                raise ValueError(
                    "实测限值基线 %s.%s.samples < %d（单次极值不算证据）: %s"
                    % (name, field, MIN_SAMPLES, path)
                )
            out[name][field] = float(value)
    return out


# ----------------------------------------------------------------- 主入口
def load_limit_defaults(
    ws: Optional[str] = None,
    allow_placeholder: bool = False,
) -> Dict[str, Any]:
    """读取限值默认值（**S02 下发限值的默认来源**）。

    参数
    ----
    ws
        工作区根（默认由 ``paths.workspace_root`` 向上查找）。
    allow_placeholder
        显式允许在基线未实测时回退到 ``joint_limits.yaml`` 的现有限值。
        **默认关闭**；开启时结果 ``source`` 必须含 ``"placeholder, 未实测"``。

    返回
    ----
    dict
        ``status``（``measured`` / ``placeholder`` / ``blocked``）、``source``、
        ``values``（逐关节 SI 目标值）、``current``（逐关节 SI 声明值）、
        ``reason``、``next_action``、``evidence``。

    **失败语义**：``status="blocked"`` 表示**拒绝下发**（调用方不得下发），
    绝不返回占位数值冒充实测值。
    """
    ml_path = measured_limits_path(ws)
    evidence: List[str] = ["measured_limits: %s" % ml_path]

    # 1) 声明限值（速度 / 位置限值的唯一来源）
    try:
        declared = load_joint_limits(ws)
    except (FileNotFoundError, ValueError) as exc:
        return _blocked(
            "读取声明限值失败，无法组装下发限值：%s" % exc,
            "确认 %s 存在且格式正确（每关节 lower/upper/velocity/acceleration/jerk），"
            "修复后重跑" % paths.joint_limits_path(ws),
            evidence + ["joint_limits: %s" % paths.joint_limits_path(ws)],
        )

    # 2) 实测基线
    if not os.path.isfile(ml_path):
        return _blocked(
            "找不到实测限值基线 %s（基线未实测，不得用占位值下发）" % ml_path,
            _NEXT_ACTION_MEASURE % ml_path,
            evidence + ["measured_limits 文件不存在"],
        )
    try:
        data = _read_yaml(ml_path)
    except Exception as exc:
        return _blocked(
            "实测限值基线解析失败（%s）：%s" % (ml_path, exc),
            "修复 %s 的 YAML 格式后重跑" % ml_path,
            evidence + [repr(exc)],
        )
    if not isinstance(data, dict):
        return _blocked(
            "实测限值基线格式错误（顶层必须是映射）：%s" % ml_path,
            "修复 %s 的 YAML 格式后重跑" % ml_path,
            evidence,
        )

    base_status = str(data.get("status") or "").strip()
    evidence.append("measured_limits.status=%s version=%s" % (
        base_status or "<空>", data.get("version"),
    ))

    # 3) 基线未实测 → 拒绝下发（除非显式允许占位回退）
    if base_status != STATUS_MEASURED:
        reason = (
            "实测限值基线 status=%s（不是 measured）：基线未实测，不得用占位值下发"
            % (base_status or "<空>")
        )
        if not allow_placeholder:
            return _blocked(
                reason,
                _NEXT_ACTION_MEASURE % ml_path,
                evidence,
                source="measured_limits.yaml (status=%s, 未实测)" % (base_status or "<空>"),
            )
        values = {name: dict(item) for name, item in declared.items()}
        return {
            "status": STATUS_PLACEHOLDER,
            "source": PLACEHOLDER_SOURCE,
            "values": values,
            "current": {name: dict(item) for name, item in declared.items()},
            "reason": reason + "；已按调用方显式要求回退到占位限值",
            "next_action": _NEXT_ACTION_PLACEHOLDER,
            "evidence": evidence + ["allow_placeholder=True"],
        }

    # 4) 基线已实测：acc / jerk 取实测值，速度 / 位置取声明限值
    try:
        measured = _measured_joint_values(data, ml_path)
    except ValueError as exc:
        reason = "实测限值基线不满足红线（%s）：基线未实测，不得用占位值下发" % exc
        if not allow_placeholder:
            return _blocked(
                reason,
                _NEXT_ACTION_MEASURE % ml_path,
                evidence,
                source="measured_limits.yaml (status=measured, 数据不完整)",
            )
        values = {name: dict(item) for name, item in declared.items()}
        return {
            "status": STATUS_PLACEHOLDER,
            "source": PLACEHOLDER_SOURCE,
            "values": values,
            "current": {name: dict(item) for name, item in declared.items()},
            "reason": reason + "；已按调用方显式要求回退到占位限值",
            "next_action": _NEXT_ACTION_PLACEHOLDER,
            "evidence": evidence + ["allow_placeholder=True"],
        }

    values: Dict[str, Any] = {}
    for name in JOINT_NAMES:
        item = dict(declared[name])
        item["max_acc"] = measured[name]["acceleration"]
        item["jerk"] = measured[name]["jerk"]
        values[name] = item

    return {
        "status": STATUS_MEASURED,
        "source": (
            "measured_limits.yaml (status=measured, version=%s) 实测 acc/jerk "
            "+ joint_limits.yaml 声明速度/位置限值"
            % data.get("version")
        ),
        "values": values,
        "current": {name: dict(item) for name, item in declared.items()},
        "reason": (
            "限值默认值来源：实测基线（acc/jerk）+ 声明限值（速度/位置），共 %d 关节"
            % len(values)
        ),
        "next_action": "",
        "evidence": evidence,
    }
