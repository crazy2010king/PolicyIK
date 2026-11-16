"""规划服务声明式配置的**只读**加载器（设计 §5.5 / D001）。

唯一来源：

* ``<ws>/workcell/planner/pipeline.yaml`` —— 管线名 / 规划器个数 / 单次规划
  时间上限 / 采样范围 / 确定性开关。
* ``<ws>/workcell/planner/profiles.yaml`` —— 服务端硬超时 / 时间缩放上限 /
  碰撞裕度 / 连续碰撞校验开关 / 场景版本号。

纪律与 ``rmctl.process.spec`` / ``rm_executor.contract`` **完全同构**：

1. **唯一来源**：配置只由上面两份 YAML 决定，可 diff、可评审、版本化；
   代码里**不写死**第二份数值。
2. **不补默认值**：必需字段缺失 / 类型不对 / 出现未知字段 → ``status="blocked"``，
   ``reason`` **逐字段指名**。
3. **不抛裸异常**：文件缺失 / YAML 语法错 / 结构非法 / 未知字段一律转成
   blocked 结果，**绝不**让异常逃到调用方。
4. **结果同构**：``{"status", "reason", "next_action", "evidence", "spec"}``；
   成功时 ``spec`` 非空、失败时 ``spec is None``；``next_action`` 始终可粘贴。

本模块**零 SDK、零 I/O、零硬件层依赖**（只依赖 ``yaml`` 与标准库）。
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml

from rmctl import paths

__all__ = [
    "PLANNER_DIR_NAME",
    "PIPELINE_CONFIG_NAME",
    "PROFILES_CONFIG_NAME",
    "planner_dir",
    "pipeline_config_path",
    "profiles_config_path",
    "load_pipeline_config",
    "load_profiles_config",
]

#: 规划配置目录名（相对 ``<ws>/workcell/``）
PLANNER_DIR_NAME = "planner"

#: 管线配置文件名
PIPELINE_CONFIG_NAME = "pipeline.yaml"

#: profile 配置文件名
PROFILES_CONFIG_NAME = "profiles.yaml"

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
    """成功结果（与 ``rmctl.process.spec`` 同构）。"""
    return {
        "status": "ok",
        "reason": reason,
        "next_action": "",
        "evidence": list(evidence),
        "spec": spec,
    }


def _blocked(reason: str, next_action: str, evidence: List[str]) -> Dict[str, Any]:
    """被阻塞结果（与 ``rmctl.process.spec`` 同构）。"""
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

def planner_dir(ws: str = None) -> str:
    """规划配置目录：``<ws>/workcell/planner``。"""
    return os.path.join(paths.workcell_dir(ws), PLANNER_DIR_NAME)


def pipeline_config_path(ws: str = None) -> str:
    """管线配置路径：``<ws>/workcell/planner/pipeline.yaml``。"""
    return os.path.join(planner_dir(ws), PIPELINE_CONFIG_NAME)


def profiles_config_path(ws: str = None) -> str:
    """profile 配置路径：``<ws>/workcell/planner/profiles.yaml``。"""
    return os.path.join(planner_dir(ws), PROFILES_CONFIG_NAME)


# --------------------------------------------------------------------------
# 校验
# --------------------------------------------------------------------------

def _load_yaml(path: str) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """读 YAML：不存在 / 语法错 / 非映射 → blocked（不抛裸异常）。"""
    evidence = ["config: %s" % path]
    if not os.path.isfile(path):
        return None, _blocked(
            "找不到规划配置文件: %s" % path,
            "确认工作区根（含 workcell/planner/）；%s" % _NEXT_ACTION_WS,
            evidence,
        )
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001 - YAML 解析异常统一转 blocked
        return None, _blocked(
            "规划配置文件解析失败（%s）: %s" % (type(exc).__name__, exc),
            "修复 %s 的 YAML 格式后重跑" % path,
            evidence + [repr(exc)],
        )
    if not isinstance(data, dict):
        return None, _blocked(
            "规划配置文件顶层不是映射: %r" % (data,),
            "在 %s 顶层写成键值映射后重跑" % path,
            evidence,
        )
    return data, None


def _check_whitelist(
    data: Dict[str, Any], allowed: Tuple[str, ...], path: str
) -> Optional[Dict[str, Any]]:
    """顶层字段白名单：出现未知字段 → blocked（避免第二处配置源）。"""
    unknown = sorted(key for key in data if key not in allowed)
    if unknown:
        return _blocked(
            "配置含未知字段: %s（只允许 %s）"
            % (", ".join(unknown), ", ".join(allowed)),
            _fix_hint(path, unknown[0], "删掉该字段（本层不猜语义）"),
            ["config: %s" % path],
        )
    return None


def _check_required(
    data: Dict[str, Any], required: Tuple[str, ...], path: str
) -> Optional[Dict[str, Any]]:
    """必需字段缺失 → blocked（逐字段指名，不补默认值）。"""
    for key in required:
        if key not in data or data[key] is None:
            return _blocked(
                "配置缺字段: %s" % key,
                _fix_hint(path, key, "补齐该字段（代码不提供默认值）"),
                ["config: %s" % path],
            )
    return None


def _check_int(
    data: Dict[str, Any], key: str, minimum: int, path: str
) -> Optional[Dict[str, Any]]:
    """整数字段校验（bool 不算整数）。"""
    value = data[key]
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        return _blocked(
            "%s 非法: %r（必须是 >= %d 的整数）" % (key, value, minimum),
            _fix_hint(path, key, "填 >= %d 的整数" % minimum),
            ["config: %s" % path],
        )
    return None


def _check_float(
    data: Dict[str, Any], key: str, minimum: float, exclusive: bool, path: str
) -> Optional[Dict[str, Any]]:
    """有限数校验。"""
    value = data[key]
    if not _is_num(value):
        return _blocked(
            "%s 非法: %r（必须是有限数）" % (key, value),
            _fix_hint(path, key, "填有限数值"),
            ["config: %s" % path],
        )
    number = float(value)
    if exclusive:
        if not number > minimum:
            return _blocked(
                "%s 非法: %r（必须 > %g）" % (key, value, minimum),
                _fix_hint(path, key, "填 > %g 的数值" % minimum),
                ["config: %s" % path],
            )
    elif number < minimum:
        return _blocked(
            "%s 非法: %r（必须 >= %g）" % (key, value, minimum),
            _fix_hint(path, key, "填 >= %g 的数值" % minimum),
            ["config: %s" % path],
        )
    return None


def _check_bool(
    data: Dict[str, Any], key: str, path: str
) -> Optional[Dict[str, Any]]:
    """布尔字段校验。"""
    value = data[key]
    if not isinstance(value, bool):
        return _blocked(
            "%s 非法: %r（必须是 true / false）" % (key, value),
            _fix_hint(path, key, "填 true 或 false"),
            ["config: %s" % path],
        )
    return None


#: 管线配置的顶层字段白名单
PIPELINE_FIELDS: Tuple[str, ...] = (
    "version",
    "pipeline",
    "num_planners",
    "planning_time_s",
    "planner_range",
    "deterministic",
)

#: profile 配置的顶层字段白名单
PROFILES_FIELDS: Tuple[str, ...] = (
    "version",
    "timeout_ms",
    "max_time_scaling",
    "collision_margin_m",
    "verify",
    "scene_version",
)

#: profile 配置里 ``verify`` 子表的字段白名单
VERIFY_FIELDS: Tuple[str, ...] = ("continuous",)


# --------------------------------------------------------------------------
# 加载
# --------------------------------------------------------------------------

def load_pipeline_config(ws: str = None) -> Dict[str, Any]:
    """加载并校验 ``workcell/planner/pipeline.yaml``（**唯一入口**）。

    :param ws: 工作区根（缺省自动向上查找）
    :return: ``{"status", "reason", "next_action", "evidence", "spec"}``；
        成功时 ``spec`` 含 ``version / pipeline / num_planners /
        planning_time_s / planner_range / deterministic / source``。

    **绝不抛裸异常**，**绝不补默认值**。
    """
    path = os.path.abspath(pipeline_config_path(ws))
    evidence = ["config: %s" % path]

    data, err = _load_yaml(path)
    if err is not None:
        return err
    assert data is not None

    err = _check_whitelist(data, PIPELINE_FIELDS, path)
    if err is not None:
        return err
    err = _check_required(data, PIPELINE_FIELDS, path)
    if err is not None:
        return err

    err = _check_int(data, "version", 1, path)
    if err is not None:
        return err

    pipeline = data["pipeline"]
    if not isinstance(pipeline, str) or not pipeline.strip():
        return _blocked(
            "pipeline 非法: %r（必须是非空字符串）" % (pipeline,),
            _fix_hint(path, "pipeline", "填管线名，如 FreespacePipeline"),
            evidence,
        )

    err = _check_int(data, "num_planners", 1, path)
    if err is not None:
        return err
    err = _check_float(data, "planning_time_s", 0.0, True, path)
    if err is not None:
        return err
    err = _check_float(data, "planner_range", 0.0, False, path)
    if err is not None:
        return err
    err = _check_bool(data, "deterministic", path)
    if err is not None:
        return err

    spec = {
        "version": int(data["version"]),
        "pipeline": pipeline.strip(),
        "num_planners": int(data["num_planners"]),
        "planning_time_s": float(data["planning_time_s"]),
        "planner_range": float(data["planner_range"]),
        "deterministic": bool(data["deterministic"]),
        "source": path,
    }
    return _ok("管线配置加载成功: %s" % path, evidence, spec)


def load_profiles_config(ws: str = None) -> Dict[str, Any]:
    """加载并校验 ``workcell/planner/profiles.yaml``（**唯一入口**）。

    :param ws: 工作区根（缺省自动向上查找）
    :return: ``{"status", "reason", "next_action", "evidence", "spec"}``；
        成功时 ``spec`` 含 ``version / timeout_ms / max_time_scaling /
        collision_margin_m / verify / scene_version / source``。

    **绝不抛裸异常**，**绝不补默认值**。
    """
    path = os.path.abspath(profiles_config_path(ws))
    evidence = ["config: %s" % path]

    data, err = _load_yaml(path)
    if err is not None:
        return err
    assert data is not None

    err = _check_whitelist(data, PROFILES_FIELDS, path)
    if err is not None:
        return err
    err = _check_required(data, PROFILES_FIELDS, path)
    if err is not None:
        return err

    err = _check_int(data, "version", 1, path)
    if err is not None:
        return err
    err = _check_int(data, "timeout_ms", 1, path)
    if err is not None:
        return err
    err = _check_float(data, "max_time_scaling", 0.0, True, path)
    if err is not None:
        return err
    err = _check_float(data, "collision_margin_m", 0.0, False, path)
    if err is not None:
        return err
    err = _check_int(data, "scene_version", 0, path)
    if err is not None:
        return err

    verify = data["verify"]
    if not isinstance(verify, dict):
        return _blocked(
            "verify 非法: %r（必须是映射）" % (verify,),
            _fix_hint(path, "verify", "写成映射，如 continuous: true"),
            evidence,
        )
    err = _check_whitelist(verify, VERIFY_FIELDS, path)
    if err is not None:
        return err
    err = _check_required(verify, VERIFY_FIELDS, path)
    if err is not None:
        return err
    err = _check_bool(verify, "continuous", path)
    if err is not None:
        return err

    spec = {
        "version": int(data["version"]),
        "timeout_ms": int(data["timeout_ms"]),
        "max_time_scaling": float(data["max_time_scaling"]),
        "collision_margin_m": float(data["collision_margin_m"]),
        "verify": {"continuous": bool(verify["continuous"])},
        "scene_version": int(data["scene_version"]),
        "source": path,
    }
    return _ok("profile 配置加载成功: %s" % path, evidence, spec)
