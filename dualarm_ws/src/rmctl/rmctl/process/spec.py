"""工艺任务规格（**声明式 YAML**）的加载与校验 —— **唯一入口**。

设计纪律与 ``rm_hw_bridge.gripper_config.load_gripper_config`` /
``rm_executor.contract.load_execution_config`` 完全对齐：

1. **唯一来源**：工艺「长什么样」只由 ``<ws>/workcell/tasks/*.yaml`` 决定；
   规格**可 diff、可评审、版本化**（git 跟踪），代码里**不写死**段序列。
2. **不补默认值**：必需字段缺失或类型不对 → ``status="blocked"``，
   ``reason`` **逐字段指名**（点名到段序号与字段名），**代码绝不猜**。
3. **不抛裸异常**：所有失败路径（文件缺失 / YAML 语法错 / 结构非法 /
   未知字段）一律转成 blocked 结果，**绝不**让异常逃到调用方。
4. **结果同构**：返回字典与 ``rm_hw_bridge.gripper_config`` 同构——
   ``{"status", "reason", "next_action", "evidence", "spec"}``；成功时
   ``spec`` 非空、失败时 ``spec is None``；``next_action`` 始终**可粘贴**。

两条硬边界（避免第二处真相）：

* **恢复阶梯上限不写进规格**：唯一来源 =
  ``workcell/rm_workcell.yaml::gripper.max_regrasp`` +
  ``rm_hw_bridge.gripper_state.RECOVERY_LADDER``。规格里出现相关字段
  （``max_regrasp`` / ``recovery_ladder`` / ``max_stall_retry`` …）→ blocked。
* **单位换算不在本层**：规格里的关节角用**度**（``home_joints_deg`` /
  ``goal_joints_deg``），加载器只做**值域**校验；度→弧度的换算留给客户端层
  （唯一换算点）。

段类型（``segments[].type``）：

* ``motion`` —— 运动段，目标**二选一**：
  ``goal_joints_deg``（7 个有限数，点到点）或 ``goal_pose``
  （6 个有限数 ``[x,y,z,r,p,y]``，米 + 弧度，笛卡尔目标）。
  **都给或都缺 → blocked**（两种都不允许由代码猜）。
* ``gripper`` —— 夹爪段，``action`` ∈ {``close``, ``open``}；
  ``speed`` / ``force`` 可选（缺省 → spec 里为 ``None``，**由客户端层**
  按夹爪配置取缺省值，本层不写死数值）。
* ``judge`` —— 判定段，``expect`` ∈ 八类判定结果
  （``grasp_ok`` / ``grasp_empty`` / ``stall`` / ``overtemp`` /
  ``overcurrent`` / ``offline`` / ``needs_param`` / ``unknown``）。

本模块**零 SDK、零 I/O、零硬件层依赖**（只依赖 ``yaml`` 与标准库）。
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml

from rmctl import paths

__all__ = [
    "DEFAULT_TASK_NAME",
    "EXPECT_RESULTS",
    "GRIPPER_ACTIONS",
    "MAX_ITERATIONS",
    "SEGMENT_TYPES",
    "TASKS_DIR_NAME",
    "default_task_path",
    "load_task_spec",
]

#: 工位任务目录名（相对 ``<ws>/workcell/``）
TASKS_DIR_NAME = "tasks"

#: 默认工艺任务名
DEFAULT_TASK_NAME = "pick_place"

#: 段类型全集
SEGMENT_TYPES: Tuple[str, ...] = ("motion", "gripper", "judge")

#: 夹爪段允许的动作
GRIPPER_ACTIONS: Tuple[str, ...] = ("close", "open")

#: 判定段 ``expect`` 允许的八类结果（与 ``rm_hw_bridge.gripper_state`` 一致）
EXPECT_RESULTS: Tuple[str, ...] = (
    "grasp_ok",
    "grasp_empty",
    "stall",
    "overtemp",
    "overcurrent",
    "offline",
    "needs_param",
    "unknown",
)

#: 段目标允许的关节数（7 自由度）
JOINT_COUNT = 7

#: 段目标允许的笛卡尔位姿长度（x,y,z,r,p,y）
POSE_COUNT = 6

#: 迭代次数上限（工艺约束：连续 ≤50 圈后强制人工检查）。
#: 本层**不据此 blocked**（规格只声明循环数），但值域非法（<1 / 非整数）
#: 一律 blocked。
MAX_ITERATIONS = 50

#: 顶层允许的字段（**白名单**：出现未知字段 → blocked，避免第二处配置源）
TOP_LEVEL_FIELDS: Tuple[str, ...] = (
    "version",
    "name",
    "description",
    "iterations",
    "home_joints_deg",
    "segments",
)

#: 各段类型允许的字段（白名单）
SEGMENT_FIELDS: Dict[str, Tuple[str, ...]] = {
    "motion": ("name", "type", "goal_joints_deg", "goal_pose"),
    "gripper": ("name", "type", "action", "speed", "force", "expect"),
    "judge": ("name", "type", "expect"),
}

#: **禁止**出现在规格里的字段（恢复阶梯上限的唯一来源在工位文件 / 硬件层）
FORBIDDEN_FIELDS: Tuple[str, ...] = (
    "max_regrasp",
    "max_stall_retry",
    "max_offline_retry",
    "recovery_ladder",
    "recovery_steps",
)

_NEXT_ACTION_WS = "可用 `export RM_WS=<仓库根>` 指定工作区后重跑"


# --------------------------------------------------------------------------
# 内部工具
# --------------------------------------------------------------------------

def _is_num(value: Any) -> bool:
    """是否为有限数值（bool 不算数值）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _ok(reason: str, evidence: List[str], spec: Dict[str, Any]) -> Dict[str, Any]:
    """成功结果（与 ``rm_hw_bridge.errors.ok`` 同构）。"""
    return {
        "status": "ok",
        "reason": reason,
        "next_action": "",
        "evidence": list(evidence),
        "spec": spec,
    }


def _blocked(reason: str, next_action: str, evidence: List[str]) -> Dict[str, Any]:
    """被阻塞结果（与 ``rm_hw_bridge.errors.blocked`` 同构）。"""
    return {
        "status": "blocked",
        "reason": reason,
        "next_action": next_action or (
            "查看上条 reason 与 evidence，修复对应字段后重跑同一条命令"
        ),
        "evidence": list(evidence),
        "spec": None,
    }


def _fix_hint(path: str, field: str, detail: str = "") -> str:
    """可粘贴的下一步：指出改哪个文件的哪个字段。"""
    tail = ("（%s）" % detail) if detail else ""
    return "在 %s 把 %s 改成合法值后重跑%s" % (path, field, tail)


# --------------------------------------------------------------------------
# 路径
# --------------------------------------------------------------------------

def tasks_dir(ws: str = None) -> str:
    """工艺任务目录：``<ws>/workcell/tasks``。"""
    return os.path.join(paths.workcell_dir(ws), TASKS_DIR_NAME)


def default_task_path(ws: str = None) -> str:
    """默认工艺任务规格文件：``<ws>/workcell/tasks/pick_place.yaml``。"""
    return os.path.join(tasks_dir(ws), "%s.yaml" % DEFAULT_TASK_NAME)


# --------------------------------------------------------------------------
# 校验
# --------------------------------------------------------------------------

def _check_number_list(
    value: Any, count: int, field: str, path: str
) -> Tuple[Optional[List[Any]], Optional[Dict[str, Any]]]:
    """校验一个「N 个有限数」的列表；不合法 → blocked（点名字段）。"""
    if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
        return None, _blocked(
            "%s 非法: %r（必须是 %d 个有限数的列表）" % (field, value, count),
            _fix_hint(path, field, "需要恰好 %d 个数" % count),
            ["spec: %s" % path],
        )
    if len(value) != count:
        return None, _blocked(
            "%s 非法: %r（必须恰好 %d 个数，当前 %d 个）"
            % (field, value, count, len(value)),
            _fix_hint(path, field, "需要恰好 %d 个数" % count),
            ["spec: %s" % path],
        )
    for index, item in enumerate(value):
        if not _is_num(item):
            return None, _blocked(
                "%s[%d] 非法: %r（必须是有限数）" % (field, index, item),
                _fix_hint(path, field, "第 %d 项必须是有限数" % index),
                ["spec: %s" % path],
            )
    return list(value), None


def _check_motion_segment(
    seg: Dict[str, Any], index: int, path: str
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """校验 ``motion`` 段：目标**二选一**（关节角 或 笛卡尔位姿）。"""
    prefix = "segments[%d]" % index
    has_joints = "goal_joints_deg" in seg and seg["goal_joints_deg"] is not None
    has_pose = "goal_pose" in seg and seg["goal_pose"] is not None

    if has_joints and has_pose:
        return None, _blocked(
            "%s 同时给了 goal_joints_deg 与 goal_pose（运动目标只能二选一）" % prefix,
            _fix_hint(
                path,
                "%s.goal_joints_deg / %s.goal_pose" % (prefix, prefix),
                "删掉其中一个目标",
            ),
            ["spec: %s" % path],
        )
    if not has_joints and not has_pose:
        return None, _blocked(
            "%s 缺字段: goal_joints_deg 或 goal_pose（运动段必须二选一，代码不猜）"
            % prefix,
            _fix_hint(
                path,
                "%s.goal_joints_deg" % prefix,
                "补 7 个关节角（度）或 6 维位姿 [x,y,z,r,p,y]",
            ),
            ["spec: %s" % path],
        )

    out: Dict[str, Any] = {"name": seg["name"], "type": "motion"}
    if has_joints:
        joints, err = _check_number_list(
            seg["goal_joints_deg"], JOINT_COUNT, "%s.goal_joints_deg" % prefix, path
        )
        if err is not None:
            return None, err
        out["goal_joints_deg"] = joints
        out["goal_pose"] = None
    else:
        pose, err = _check_number_list(
            seg["goal_pose"], POSE_COUNT, "%s.goal_pose" % prefix, path
        )
        if err is not None:
            return None, err
        out["goal_pose"] = pose
        out["goal_joints_deg"] = None
    return out, None


def _check_gripper_segment(
    seg: Dict[str, Any], index: int, path: str
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """校验 ``gripper`` 段：动作合法；``speed``/``force`` 可选（缺省 → None）。"""
    prefix = "segments[%d]" % index
    action = seg.get("action")
    if not isinstance(action, str) or action not in GRIPPER_ACTIONS:
        return None, _blocked(
            "%s.action 非法: %r（只允许 %s）"
            % (prefix, action, " | ".join(GRIPPER_ACTIONS)),
            _fix_hint(path, "%s.action" % prefix,
                      "取值 %s" % " | ".join(GRIPPER_ACTIONS)),
            ["spec: %s" % path],
        )

    out: Dict[str, Any] = {"name": seg["name"], "type": "gripper", "action": action}
    for key in ("speed", "force"):
        raw = seg.get(key)
        if raw is None:
            # 缺省 → None（**不写死数值**，由客户端层按夹爪配置取缺省）
            out[key] = None
            continue
        if not _is_num(raw):
            return None, _blocked(
                "%s.%s 非法: %r（必须是有限数或省略）" % (prefix, key, raw),
                _fix_hint(path, "%s.%s" % (prefix, key), "填实测数值或删掉该字段"),
                ["spec: %s" % path],
            )
        out[key] = raw

    if "expect" in seg and seg["expect"] is not None:
        expect = seg["expect"]
        if not isinstance(expect, str) or expect not in EXPECT_RESULTS:
            return None, _blocked(
                "%s.expect 非法: %r（只允许 %s）"
                % (prefix, expect, " | ".join(EXPECT_RESULTS)),
                _fix_hint(path, "%s.expect" % prefix,
                          "取值 %s" % " | ".join(EXPECT_RESULTS)),
                ["spec: %s" % path],
            )
        out["expect"] = expect
    else:
        out["expect"] = None
    return out, None


def _check_judge_segment(
    seg: Dict[str, Any], index: int, path: str
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """校验 ``judge`` 段：``expect`` 必填且属于八类判定结果。"""
    prefix = "segments[%d]" % index
    if "expect" not in seg or seg["expect"] is None:
        return None, _blocked(
            "%s 缺字段: expect（判定段必须声明期望的判定结果）" % prefix,
            _fix_hint(path, "%s.expect" % prefix,
                      "取值 %s" % " | ".join(EXPECT_RESULTS)),
            ["spec: %s" % path],
        )
    expect = seg["expect"]
    if not isinstance(expect, str) or expect not in EXPECT_RESULTS:
        return None, _blocked(
            "%s.expect 非法: %r（只允许 %s）"
            % (prefix, expect, " | ".join(EXPECT_RESULTS)),
            _fix_hint(path, "%s.expect" % prefix,
                      "取值 %s" % " | ".join(EXPECT_RESULTS)),
            ["spec: %s" % path],
        )
    return {"name": seg["name"], "type": "judge", "expect": expect}, None


def _check_segment(
    seg: Any, index: int, path: str
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """校验单段：类型 / 名字 / 字段白名单 / 类型专属字段。"""
    prefix = "segments[%d]" % index
    if not isinstance(seg, dict):
        return None, _blocked(
            "%s 非法: %r（每段必须是映射）" % (prefix, seg),
            _fix_hint(path, prefix, "每段写成映射（name/type/...）"),
            ["spec: %s" % path],
        )
    seg_type = seg.get("type")
    if not isinstance(seg_type, str) or seg_type not in SEGMENT_TYPES:
        return None, _blocked(
            "%s.type 非法: %r（只允许 %s）"
            % (prefix, seg_type, " | ".join(SEGMENT_TYPES)),
            _fix_hint(path, "%s.type" % prefix,
                      "取值 %s" % " | ".join(SEGMENT_TYPES)),
            ["spec: %s" % path],
        )
    name = seg.get("name")
    if not isinstance(name, str) or not name.strip():
        return None, _blocked(
            "%s 缺字段: name（段名必须是非空字符串）" % prefix,
            _fix_hint(path, "%s.name" % prefix, "填一个唯一段名"),
            ["spec: %s" % path],
        )
    unknown = [
        key for key in seg
        if key not in SEGMENT_FIELDS[seg_type]
    ]
    if unknown:
        return None, _blocked(
            "%s 含未知字段: %s（%s 段只允许 %s）"
            % (prefix, ", ".join(sorted(unknown)), seg_type,
               ", ".join(SEGMENT_FIELDS[seg_type])),
            _fix_hint(path, "%s.%s" % (prefix, sorted(unknown)[0]),
                      "删掉该字段（本层不猜语义）"),
            ["spec: %s" % path],
        )
    if seg_type == "motion":
        return _check_motion_segment(seg, index, path)
    if seg_type == "gripper":
        return _check_gripper_segment(seg, index, path)
    return _check_judge_segment(seg, index, path)


def _validate(data: Dict[str, Any], path: str) -> Tuple[Optional[Dict[str, Any]],
                                                       Optional[Dict[str, Any]]]:
    """顶层校验（逐字段指名）。"""
    evidence = ["spec: %s" % path]

    unknown = [key for key in data if key not in TOP_LEVEL_FIELDS]
    if unknown:
        key = sorted(unknown)[0]
        if key in FORBIDDEN_FIELDS:
            return None, _blocked(
                "规格含禁止字段: %s（恢复阶梯上限的唯一来源是 "
                "workcell/rm_workcell.yaml::gripper.max_regrasp 与 "
                "rm_hw_bridge.gripper_state.RECOVERY_LADDER）" % key,
                "从 %s 删掉 %s 后重跑（不得在规格里另立第二处配置源）"
                % (path, key),
                evidence,
            )
        return None, _blocked(
            "规格含未知字段: %s（顶层只允许 %s）"
            % (key, ", ".join(TOP_LEVEL_FIELDS)),
            "从 %s 删掉 %s 后重跑" % (path, key),
            evidence,
        )

    for key in ("version", "name", "iterations", "home_joints_deg", "segments"):
        if key not in data:
            return None, _blocked(
                "规格缺字段: %s" % key,
                "在 %s 补齐 %s 后重跑（代码不提供默认值）" % (path, key),
                evidence,
            )

    version = data["version"]
    if isinstance(version, bool) or not isinstance(version, int) or version < 1:
        return None, _blocked(
            "version 非法: %r（必须是 >= 1 的整数）" % (version,),
            _fix_hint(path, "version", "填 >= 1 的整数"),
            evidence,
        )
    name = data["name"]
    if not isinstance(name, str) or not name.strip():
        return None, _blocked(
            "name 非法: %r（必须是非空字符串）" % (name,),
            _fix_hint(path, "name", "填工艺名，如 pick_place"),
            evidence,
        )
    iterations = data["iterations"]
    if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations < 1:
        return None, _blocked(
            "iterations 非法: %r（必须是 >= 1 的整数）" % (iterations,),
            _fix_hint(path, "iterations", "填 >= 1 的整数"),
            evidence,
        )

    home, err = _check_number_list(
        data["home_joints_deg"], JOINT_COUNT, "home_joints_deg", path
    )
    if err is not None:
        return None, err

    raw_segments = data["segments"]
    if isinstance(raw_segments, (str, bytes)) or not isinstance(
        raw_segments, (list, tuple)
    ):
        return None, _blocked(
            "segments 非法: %r（必须是列表）" % (raw_segments,),
            _fix_hint(path, "segments", "写成段序列列表"),
            evidence,
        )
    if len(raw_segments) == 0:
        return None, _blocked(
            "segments 非法: 空列表（至少要有一段）",
            _fix_hint(path, "segments", "至少写一段（motion/gripper/judge）"),
            evidence,
        )

    segments: List[Dict[str, Any]] = []
    seen: Dict[str, int] = {}
    for index, raw in enumerate(raw_segments):
        seg, err = _check_segment(raw, index, path)
        if err is not None:
            return None, err
        assert seg is not None
        if seg["name"] in seen:
            return None, _blocked(
                "segments 段名重复: %s（segments[%d] 与 segments[%d] 同名）"
                % (seg["name"], seen[seg["name"]], index),
                _fix_hint(path, "segments[%d].name" % index,
                          "改成未出现过的段名（段名必须唯一）"),
                evidence,
            )
        seen[seg["name"]] = index
        segments.append(seg)

    spec: Dict[str, Any] = {
        "version": version,
        "name": name.strip(),
        "description": data.get("description"),
        "iterations": iterations,
        "home_joints_deg": home,
        "segments": segments,
        "source": path,
    }
    return spec, None


# --------------------------------------------------------------------------
# 加载
# --------------------------------------------------------------------------

def load_task_spec(path: str = None, ws: str = None) -> Dict[str, Any]:
    """加载并校验工艺任务规格（**唯一入口**）。

    :param path: 规格文件路径；缺省取 :func:`default_task_path` 的结果
    :param ws: 工作区根（缺省自动向上查找）
    :return: 与 ``rm_hw_bridge.gripper_config.load_gripper_config`` **同构**的字典：

        * 成功：``{"status": "ok", "reason": ..., "next_action": "",
          "evidence": [...], "spec": {...}}``；
        * 失败：``{"status": "blocked", "reason": "...（逐字段指名）",
          "next_action": "（可粘贴）", "evidence": [...], "spec": None}``。

    **绝不抛裸异常**，**绝不补默认值**。
    """
    spec_path = os.path.abspath(path or default_task_path(ws))
    evidence: List[str] = ["spec: %s" % spec_path]

    if not os.path.isfile(spec_path):
        return _blocked(
            "找不到工艺任务规格文件: %s" % spec_path,
            "确认工作区根（含 workcell/tasks/）；%s" % _NEXT_ACTION_WS,
            evidence,
        )
    try:
        with open(spec_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001 - YAML 解析异常统一转 blocked
        return _blocked(
            "工艺任务规格文件解析失败（%s）: %s" % (type(exc).__name__, exc),
            "修复 %s 的 YAML 格式后重跑" % spec_path,
            evidence + [repr(exc)],
        )
    if not isinstance(data, dict):
        return _blocked(
            "工艺任务规格文件格式错误（顶层必须是映射，当前 %s）"
            % type(data).__name__,
            "修复 %s 的 YAML 格式后重跑" % spec_path,
            evidence,
        )

    spec, err = _validate(data, spec_path)
    if err is not None:
        return err
    assert spec is not None
    return _ok(
        "工艺任务规格来自 %s（version=%s, iterations=%s, 段数=%d）"
        % (spec_path, spec["version"], spec["iterations"], len(spec["segments"])),
        evidence + [
            "段序列: %s" % ", ".join(s["name"] for s in spec["segments"]),
        ],
        spec,
    )
