"""夹爪配置契约（**唯一来源** = ``workcell/rm_workcell.yaml`` 的 ``gripper:`` 段）。

设计纪律与 ``rm_executor/contract.py::load_execution_config`` 完全对齐
（**同构语义、独立实现**——硬件层不得反向依赖执行层）：

1. **唯一来源**：夹爪的可配置项只从工位文件读；换机型 = 换这份文件 +
   重新标定，**不改代码**。
2. **不补默认值**：必需字段缺失或类型不对 → ``status="blocked"``，
   ``reason`` **指名**缺哪个字段，**代码里绝不补默认值**。
3. **不伪造标定**：标定类字段（位置/力度换算系数与阈值）在真机实测之前
   允许为 ``None``——配置**合法**（``status="ok"``），但调用方据此走
   ``NEEDS_PARAM``（判定层不得产出 ``grasp_ok`` / ``grasp_empty``）。
4. **结果同构**：返回字典与 ``rm_executor.contract.result_dict`` 同构
   （``status`` / ``reason`` / ``next_action`` / ``evidence`` / ``config``），
   ``next_action`` 始终**可粘贴**。

模块**零 SDK、零 rmctl / rm_executor 依赖**（只依赖 ``yaml``、标准库与
硬件层自己的 ``paths`` / ``errors``）。
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml

from rm_hw_bridge import errors, paths

__all__ = [
    "CALIBRATION_FIELDS",
    "GRIPPER_SECTION",
    "PATHS",
    "REQUIRED_FIELDS",
    "THRESHOLD_KEYS",
    "GripperConfig",
    "load_gripper_config",
]

#: 工位文件里的夹爪段名
GRIPPER_SECTION = "gripper"

#: 驱动路径取值（``path:`` 字段）
PATHS: Tuple[str, ...] = ("modbus", "gripper_api", "rm_plus")

#: 夹爪段**必需字段**（缺任一或类型不对 → blocked，代码不补默认值）
REQUIRED_FIELDS: Tuple[str, ...] = (
    "model",
    "protocol",
    "path",
    "device_id",
    "baud",
    "timeout_ms",
    "poll_interval_ms",
    "action_timeout_ms",
    "max_regrasp",
)

#: **标定类字段**（真机实测前为 ``None`` 合法，不构成 blocked）
CALIBRATION_FIELDS: Tuple[str, ...] = (
    "position_mm_per_unit",
    "force_scale",
    "force_threshold",
    "grasp_position_window",
)

#: 判定层 ``judge(state, thresholds)`` 需要的阈值键
THRESHOLD_KEYS: Tuple[str, ...] = (
    "force_threshold",
    "grasp_position_window",
    "position_mm_per_unit",
    "force_scale",
)

#: 可粘贴的下一步：指定工作区根
_NEXT_ACTION_WS = "可用 `export RM_WS=<仓库根>` 指定工作区后重跑"


def _is_num(value: Any) -> bool:
    """是否为有限数值（bool 不算数值）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


@dataclass
class GripperConfig:
    """夹爪配置（来自工位文件 ``gripper:`` 段）。

    换算系数与阈值（``position_mm_per_unit`` / ``force_scale`` /
    ``force_threshold`` / ``grasp_position_window``）在真机实测之前为
    ``None``——**不得伪造**。
    """

    model: str
    protocol: str
    path: str
    device_id: int
    baud: int
    timeout_ms: int
    poll_interval_ms: int
    action_timeout_ms: int
    max_regrasp: int
    power: Optional[str] = None
    position_mm_per_unit: Optional[float] = None
    force_scale: Optional[float] = None
    force_threshold: Optional[float] = None
    grasp_position_window: Optional[Tuple[float, float]] = None
    source: str = ""
    calibration_pending: List[str] = field(default_factory=list)

    @property
    def thresholds(self) -> Dict[str, Any]:
        """判定层需要的阈值字典（缺项为 ``None`` → 走 NEEDS_PARAM）。

        键集合与 :data:`THRESHOLD_KEYS` 一致，也与
        ``rm_hw_bridge.gripper_state.REQUIRED_GRASP_THRESHOLDS`` 对齐。
        """
        return {
            "force_threshold": self.force_threshold,
            "grasp_position_window": self.grasp_position_window,
            "position_mm_per_unit": self.position_mm_per_unit,
            "force_scale": self.force_scale,
        }

    def to_dict(self) -> Dict[str, Any]:
        """JSON 友好字典（含 ``thresholds`` 子字典）。"""
        return {
            "model": self.model,
            "power": self.power,
            "protocol": self.protocol,
            "path": self.path,
            "device_id": self.device_id,
            "baud": self.baud,
            "timeout_ms": self.timeout_ms,
            "poll_interval_ms": self.poll_interval_ms,
            "action_timeout_ms": self.action_timeout_ms,
            "max_regrasp": self.max_regrasp,
            "position_mm_per_unit": self.position_mm_per_unit,
            "force_scale": self.force_scale,
            "force_threshold": self.force_threshold,
            "grasp_position_window": (
                list(self.grasp_position_window)
                if self.grasp_position_window is not None
                else None
            ),
            "thresholds": {
                key: (
                    list(value) if isinstance(value, tuple) else value
                )
                for key, value in self.thresholds.items()
            },
            "source": self.source,
            "calibration_pending": list(self.calibration_pending),
        }


def _blocked(reason: str, next_action: str, evidence: List[str]) -> Dict[str, Any]:
    """blocked 结果（与 ``rm_executor.contract.result_dict`` 同构）。"""
    return errors.blocked(reason, next_action, evidence, config=None)


def _parse_calibration_number(
    section: Dict[str, Any], key: str, cfg_path: str, evidence: List[str]
) -> Tuple[Optional[float], Optional[Dict[str, Any]]]:
    """解析标定数值字段（``None`` 合法；类型错 → blocked 结果或 ``None``）。"""
    raw = section.get(key)
    if raw is None:
        return None, None
    if not _is_num(raw):
        return None, _blocked(
            "gripper.%s 非法: %r（必须是数值或 null）" % (key, raw),
            "在 %s 把 gripper.%s 设为实测数值或 null 后重跑"
            "（实测前保持 null，不得写入猜测值）" % (cfg_path, key),
            evidence,
        )
    return float(raw), None


def _parse_window(
    section: Dict[str, Any], cfg_path: str, evidence: List[str]
) -> Tuple[Optional[Tuple[float, float]], Optional[Dict[str, Any]]]:
    """解析夹持位置窗口（``None`` 合法；必须恰为两个有限数且 lo <= hi）。"""
    raw = section.get("grasp_position_window")
    if raw is None:
        return None, None
    if isinstance(raw, (str, bytes)) or not isinstance(raw, (list, tuple)):
        return None, _blocked(
            "gripper.grasp_position_window 非法: %r（必须是 [lo, hi] 或 null）" % (raw,),
            "在 %s 把 gripper.grasp_position_window 设为 [lo, hi]（mm）或 null 后重跑"
            % cfg_path,
            evidence,
        )
    if len(raw) != 2 or not _is_num(raw[0]) or not _is_num(raw[1]):
        return None, _blocked(
            "gripper.grasp_position_window 非法: %r（必须是两个有限数 [lo, hi]）" % (raw,),
            "在 %s 把 gripper.grasp_position_window 设为 [lo, hi]（mm）或 null 后重跑"
            % cfg_path,
            evidence,
        )
    lo, hi = float(raw[0]), float(raw[1])
    if lo > hi:
        return None, _blocked(
            "gripper.grasp_position_window 非法: %r（lo 必须 <= hi）" % (raw,),
            "在 %s 把 gripper.grasp_position_window 的 lo 调小到 <= hi 后重跑"
            % cfg_path,
            evidence,
        )
    return (lo, hi), None


def load_gripper_config(ws: str = None) -> Dict[str, Any]:
    """读取夹爪配置（**唯一来源**：``workcell/rm_workcell.yaml`` 的 ``gripper:`` 段）。

    返回
    ----
    dict
        与 ``rm_executor.contract.result_dict`` **同构**：

        * 成功：``{"status": "ok", "reason": ..., "next_action": "",
          "evidence": [...], "config": {...}}``；``config["thresholds"]``
          给出判定层 ``judge(state, thresholds)`` 需要的四个键（未标定 → ``None``）。
        * 失败：``{"status": "blocked", "reason": "...（指名缺失/非法字段）",
          "next_action": "（可粘贴）", "evidence": [...], "config": None}``。

    blocked 触发条件：工位文件缺失 / YAML 解析失败 / 缺 ``gripper:`` 段 /
    必需字段缺失或类型错 / 标定字段类型错。

    **标定字段为 ``None`` 不构成 blocked**（配置合法），但 ``config``
    里的对应值保持 ``None``，调用方据此走 ``NEEDS_PARAM``。
    """
    cfg_path = paths.workcell_config_path(ws)
    evidence: List[str] = ["workcell: %s" % cfg_path]

    if not os.path.isfile(cfg_path):
        return _blocked(
            "找不到工位定义文件: %s" % cfg_path,
            "确认工作区根（含 workcell/rm_workcell.yaml）；%s" % _NEXT_ACTION_WS,
            evidence,
        )
    try:
        with open(cfg_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:  # noqa: BLE001 - YAML 解析异常统一转 blocked
        return _blocked(
            "工位定义文件解析失败（%s）: %s" % (type(exc).__name__, exc),
            "修复 %s 的 YAML 格式后重跑" % cfg_path,
            evidence + [repr(exc)],
        )
    if not isinstance(data, dict):
        return _blocked(
            "工位定义文件格式错误（顶层必须是映射，当前 %s）" % type(data).__name__,
            "修复 %s 的 YAML 格式后重跑" % cfg_path,
            evidence,
        )

    section = data.get(GRIPPER_SECTION)
    if not isinstance(section, dict):
        return _blocked(
            "工位定义文件缺少 %s: 段（当前是 %r）" % (GRIPPER_SECTION, section),
            "在 %s 的 %s: 段补齐 %s 后重跑"
            % (cfg_path, GRIPPER_SECTION, ", ".join(REQUIRED_FIELDS)),
            evidence,
        )

    missing = [f for f in REQUIRED_FIELDS if f not in section]
    if missing:
        return _blocked(
            "夹爪配置缺字段: %s" % ", ".join(missing),
            "在 %s 的 %s: 段补齐 %s 后重跑（代码不提供默认值）"
            % (cfg_path, GRIPPER_SECTION, ", ".join(REQUIRED_FIELDS)),
            evidence,
        )

    # ---- 必需字段的类型校验（逐个指名） ----
    for key in ("model", "protocol"):
        value = section[key]
        if not isinstance(value, str) or not value.strip():
            return _blocked(
                "gripper.%s 非法: %r（必须是非空字符串）" % (key, value),
                "在 %s 把 gripper.%s 设为非空字符串后重跑" % (cfg_path, key),
                evidence,
            )
    path_value = section["path"]
    if not isinstance(path_value, str) or path_value not in PATHS:
        return _blocked(
            "gripper.path 非法: %r（只允许 %s）" % (path_value, " | ".join(PATHS)),
            "在 %s 把 gripper.path 设为 %s 之一后重跑"
            % (cfg_path, " | ".join(PATHS)),
            evidence,
        )
    for key, minimum in (
        ("device_id", 0),
        ("baud", 1),
        ("timeout_ms", 1),
        ("poll_interval_ms", 1),
        ("action_timeout_ms", 1),
        ("max_regrasp", 0),
    ):
        value = section[key]
        if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
            return _blocked(
                "gripper.%s 非法: %r（必须是 >= %d 的整数）" % (key, value, minimum),
                "在 %s 把 gripper.%s 设为 >= %d 的整数后重跑"
                % (cfg_path, key, minimum),
                evidence,
            )
    power = section.get("power")
    if power is not None and not isinstance(power, str):
        return _blocked(
            "gripper.power 非法: %r（必须是字符串或 null）" % (power,),
            "在 %s 把 gripper.power 设为供电方式字符串（如 external_24v）后重跑"
            % cfg_path,
            evidence,
        )

    # ---- 标定类字段（None 合法；类型错 → blocked） ----
    calibration: Dict[str, Optional[float]] = {}
    for key in ("position_mm_per_unit", "force_scale", "force_threshold"):
        value, err = _parse_calibration_number(section, key, cfg_path, evidence)
        if err is not None:
            return err
        calibration[key] = value
    window, err = _parse_window(section, cfg_path, evidence)
    if err is not None:
        return err

    cfg = GripperConfig(
        model=str(section["model"]).strip(),
        protocol=str(section["protocol"]).strip(),
        path=str(path_value),
        device_id=int(section["device_id"]),
        baud=int(section["baud"]),
        timeout_ms=int(section["timeout_ms"]),
        poll_interval_ms=int(section["poll_interval_ms"]),
        action_timeout_ms=int(section["action_timeout_ms"]),
        max_regrasp=int(section["max_regrasp"]),
        power=power if isinstance(power, str) else None,
        position_mm_per_unit=calibration["position_mm_per_unit"],
        force_scale=calibration["force_scale"],
        force_threshold=calibration["force_threshold"],
        grasp_position_window=window,
        source=cfg_path,
    )
    # 未标定项清单（标定字段仍为 None → 判定层走 NEEDS_PARAM）
    cfg.calibration_pending = [
        key for key in CALIBRATION_FIELDS if cfg.thresholds.get(key) is None
    ]

    return errors.ok(
        reason="夹爪配置来自 %s::%s（唯一来源）" % (cfg_path, GRIPPER_SECTION),
        evidence=evidence + [
            "gripper 段: %s" % ", ".join(sorted(section.keys())),
        ],
        config=cfg.to_dict(),
    )
