"""安全配置读取（**唯一来源**：``workcell/rm_workcell.yaml`` 的 ``safety:`` 段）。

读取范围与口径
--------------

* 只读 ``safety:`` 段，键：

  * ``error_codes`` —— 错误码表路径（**相对 ``workcell/`` 目录**）；
  * ``l1_speed_scale`` —— L1 降速比例；
  * ``enable_l1_slowdown`` —— 是否启用 L1 降速；
  * ``enable_l2_pause_on_stream_lag`` —— 断流告警区间是否升到 L2 暂停；
  * ``sdk_timeout_s`` —— SDK 调用超时（秒）。

* **缺项用默认值**，并在 ``evidence`` 里逐条记「使用默认值」（**不静默**）；
* **类型非法 → blocked**（带可粘贴的下一步，不猜测用户意图）；
* **文件缺失 → blocked**（带可粘贴的下一步）。

本模块**零 SDK、零第三方依赖**（除 pyyaml），且**不 import** 执行层 / 体验层
（安全层不得反向依赖）。
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml

from rm_safety import paths

__all__ = [
    "SAFETY_SECTION",
    "STATUS_BLOCKED",
    "STATUS_OK",
    "DEFAULT_ERROR_CODES_REL",
    "DEFAULT_L1_SPEED_SCALE",
    "DEFAULT_ENABLE_L1_SLOWDOWN",
    "DEFAULT_ENABLE_L2_PAUSE_ON_STREAM_LAG",
    "DEFAULT_SDK_TIMEOUT_S",
    "load_safety_config",
]

#: 工位文件里的安全段名
SAFETY_SECTION = "safety"

#: 成功 / 被阻塞状态
STATUS_OK = "ok"
STATUS_BLOCKED = "blocked"

#: 缺省值（缺项时使用，并写入 evidence）
DEFAULT_ERROR_CODES_REL = "safety/error_codes.yaml"
DEFAULT_L1_SPEED_SCALE = 0.5
DEFAULT_ENABLE_L1_SLOWDOWN = True
DEFAULT_ENABLE_L2_PAUSE_ON_STREAM_LAG = False
DEFAULT_SDK_TIMEOUT_S = 1.0


def _result(status: str, reason: str = "", next_action: str = "",
            evidence: Optional[List[str]] = None, **extra: Any) -> Dict[str, Any]:
    """统一结果字典（与仓库既有层的 ``result_dict`` 同构，**独立实现**）。"""
    out: Dict[str, Any] = {
        "status": status,
        "reason": reason,
        "next_action": next_action,
        "evidence": [e if isinstance(e, str) else str(e) for e in (evidence or [])],
    }
    out.update(extra)
    return out


def _blocked(reason: str, next_action: str,
             evidence: Optional[List[str]] = None, **extra: Any) -> Dict[str, Any]:
    """被阻塞结果（退出码 3）。"""
    return _result(STATUS_BLOCKED, reason, next_action, evidence, **extra)


def _is_num(value: Any) -> bool:
    """是否为有限数值（bool 不算数值）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    number = float(value)
    return number == number and number not in (float("inf"), float("-inf"))


def load_safety_config(ws: str = None) -> Dict[str, Any]:
    """读取安全配置（``workcell/rm_workcell.yaml::safety``）。

    返回
    ----
    dict
        成功：``{"status": "ok", "config": {...}, "reason", "next_action",
        "evidence"}``；失败：``{"status": "blocked", ...}``。

        ``config`` 键：``error_codes``（相对 workcell 的路径）、
        ``error_codes_path``（绝对路径）、``l1_speed_scale``、
        ``enable_l1_slowdown``、``enable_l2_pause_on_stream_lag``、
        ``sdk_timeout_s``。
    """
    cfg_path = paths.workcell_config_path(ws)
    workcell_dir = paths.workcell_dir(ws)
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

    section = data.get(SAFETY_SECTION)
    if section is None:
        section = {}
        evidence.append("缺少 %s: 段 → 全部使用默认值（不静默）" % SAFETY_SECTION)
    elif not isinstance(section, dict):
        return _blocked(
            "%s: 段格式错误（必须是映射，当前 %s）"
            % (SAFETY_SECTION, type(section).__name__),
            "修复 %s 的 %s: 段后重跑" % (cfg_path, SAFETY_SECTION),
            evidence, config=None,
        )

    config: Dict[str, Any] = {}

    # ------------------------------------------------------- error_codes 路径
    raw_codes = section.get("error_codes")
    if raw_codes is None:
        error_codes_rel = DEFAULT_ERROR_CODES_REL
        evidence.append("safety.error_codes 缺失 → 使用默认值 %r（不静默）"
                        % error_codes_rel)
    elif isinstance(raw_codes, str) and raw_codes.strip():
        error_codes_rel = raw_codes.strip()
    else:
        return _blocked(
            "safety.error_codes 非法: %r（必须是非空字符串，相对 workcell 目录）"
            % (raw_codes,),
            "在 %s 把 safety.error_codes 设为相对 workcell 的路径（如 %r）后重跑"
            % (cfg_path, DEFAULT_ERROR_CODES_REL),
            evidence, config=None,
        )
    config["error_codes"] = error_codes_rel
    config["error_codes_path"] = os.path.join(workcell_dir, error_codes_rel)

    # ------------------------------------------------------------ l1_speed_scale
    raw_scale = section.get("l1_speed_scale")
    if raw_scale is None:
        config["l1_speed_scale"] = DEFAULT_L1_SPEED_SCALE
        evidence.append("safety.l1_speed_scale 缺失 → 使用默认值 %.3f（不静默）"
                        % DEFAULT_L1_SPEED_SCALE)
    elif _is_num(raw_scale) and 0.0 < float(raw_scale) <= 1.0:
        config["l1_speed_scale"] = float(raw_scale)
    else:
        return _blocked(
            "safety.l1_speed_scale 非法: %r（必须是 (0, 1] 的数值）" % (raw_scale,),
            "在 %s 把 safety.l1_speed_scale 设为 (0, 1] 的比例（如 0.5）后重跑"
            % cfg_path,
            evidence, config=None,
        )

    # -------------------------------------------------------- enable_l1_slowdown
    raw_l1 = section.get("enable_l1_slowdown")
    if raw_l1 is None:
        config["enable_l1_slowdown"] = DEFAULT_ENABLE_L1_SLOWDOWN
        evidence.append("safety.enable_l1_slowdown 缺失 → 使用默认值 %s（不静默）"
                        % DEFAULT_ENABLE_L1_SLOWDOWN)
    elif isinstance(raw_l1, bool):
        config["enable_l1_slowdown"] = raw_l1
    else:
        return _blocked(
            "safety.enable_l1_slowdown 非法: %r（必须是布尔）" % (raw_l1,),
            "在 %s 把 safety.enable_l1_slowdown 设为 true/false 后重跑" % cfg_path,
            evidence, config=None,
        )

    # -------------------------------------------- enable_l2_pause_on_stream_lag
    raw_lag = section.get("enable_l2_pause_on_stream_lag")
    if raw_lag is None:
        config["enable_l2_pause_on_stream_lag"] = DEFAULT_ENABLE_L2_PAUSE_ON_STREAM_LAG
        evidence.append(
            "safety.enable_l2_pause_on_stream_lag 缺失 → 使用默认值 %s（不静默）"
            % DEFAULT_ENABLE_L2_PAUSE_ON_STREAM_LAG)
    elif isinstance(raw_lag, bool):
        config["enable_l2_pause_on_stream_lag"] = raw_lag
    else:
        return _blocked(
            "safety.enable_l2_pause_on_stream_lag 非法: %r（必须是布尔）" % (raw_lag,),
            "在 %s 把 safety.enable_l2_pause_on_stream_lag 设为 true/false 后重跑"
            % cfg_path,
            evidence, config=None,
        )

    # ------------------------------------------------------------ sdk_timeout_s
    raw_timeout = section.get("sdk_timeout_s")
    if raw_timeout is None:
        config["sdk_timeout_s"] = DEFAULT_SDK_TIMEOUT_S
        evidence.append("safety.sdk_timeout_s 缺失 → 使用默认值 %.3f s（不静默）"
                        % DEFAULT_SDK_TIMEOUT_S)
    elif _is_num(raw_timeout) and float(raw_timeout) > 0:
        config["sdk_timeout_s"] = float(raw_timeout)
    else:
        return _blocked(
            "safety.sdk_timeout_s 非法: %r（必须是正的秒数）" % (raw_timeout,),
            "在 %s 把 safety.sdk_timeout_s 设为正的秒数（如 1.0）后重跑" % cfg_path,
            evidence, config=None,
        )

    return _result(
        STATUS_OK,
        reason="安全配置来自 %s::%s（唯一来源）" % (cfg_path, SAFETY_SECTION),
        next_action="",
        evidence=evidence,
        config=config,
    )
