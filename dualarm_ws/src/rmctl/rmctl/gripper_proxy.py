"""夹爪**代理阈值**的唯一来源（真机标定未回填时的离线判据）。

设计背景
--------

真机夹爪的四类标定量（``force_threshold`` / ``grasp_position_window`` /
``position_mm_per_unit`` / ``force_scale``）在实测回填前是 ``None``，
而离线（``mode=fake``）的工艺跑批又必须能跑完。历史上这份代理阈值写在
``rmctl/cli.py`` 里，于是任何**新**模块一旦想用代理阈值，就会面临
「再抄一份」的诱惑 —— 那就是**第二处真相**。

本模块把这份代理阈值收敛成**唯一一份**：

* :data:`GRIPPER_PROXY_THRESHOLDS` —— 代理阈值字典（数值逐字沿用历史值）；
* :data:`GRIPPER_PROXY_NOTE` —— 固定声明「代理模型，不代表真机结论」；
* :func:`effective_thresholds` —— **合并逻辑**（工位配置优先，``None`` 的键
  用代理值补齐，并标注来源）。

``rmctl/cli.py`` 通过导入本模块使用同一份常量与同一份合并逻辑
（``rmctl.cli.GRIPPER_PROXY_THRESHOLDS is rmctl.gripper_proxy.GRIPPER_PROXY_THRESHOLDS``
恒成立），因此**全仓库只有这一份代理阈值**。

三条纪律
--------

1. **不补默认值以外的东西**：本模块只做「工位配置 → 代理值」的补齐，
   绝不引入第三种数值来源。
2. **代理 ≠ 真机结论**：凡使用代理阈值产出的判定，输出里必须带
   :data:`GRIPPER_PROXY_NOTE`。
3. **判定真值仍在硬件层**：``rm_hw_bridge.gripper_state.judge`` 是唯一的
   判定实现；本模块**不**实现任何判定逻辑。
"""
from __future__ import annotations

import os
import sys
from typing import Any, Dict, List, Optional, Tuple

__all__ = [
    "GRIPPER_PROXY_NOTE",
    "GRIPPER_PROXY_THRESHOLDS",
    "effective_thresholds",
    "load_effective_thresholds",
]

#: **代理阈值**（真机标定未回填时的离线判据；输出里必须标注「不代表真机结论」）
GRIPPER_PROXY_THRESHOLDS: Dict[str, Any] = {
    "force_threshold": 100.0,
    "grasp_position_window": [10.0, 30.0],
    "position_mm_per_unit": 0.05,
    "force_scale": 0.01,
}

#: 离线代理的固定声明（**必须**出现在使用代理阈值的输出里）
GRIPPER_PROXY_NOTE = "代理模型，不代表真机结论"

#: 阈值来源标签
SOURCE_WORKCELL = "workcell_config"
SOURCE_PROXY = "proxy_default"


def effective_thresholds(cfg: Optional[Dict[str, Any]]
                         ) -> Tuple[Dict[str, Any], List[str], str]:
    """判定阈值：优先用工位配置；缺失项用**代理阈值**补齐（并标注来源）。

    合并逻辑逐字来自 ``rmctl/cli.py::_gripper_thresholds``（历史上的实现），
    现在收敛到本模块，``cli.py`` 反过来调用本函数 —— **两份实现不存在**。

    :param cfg: 夹爪配置字典（``rm_hw_bridge.gripper_config`` 的
        ``config`` 形态；也可直接给 ``{"thresholds": {...}}``）。
    :return: ``(thresholds, proxy_used_keys, source)``：

        * ``thresholds`` —— 判定层 ``judge(state, thresholds)`` 用的四键；
        * ``proxy_used_keys`` —— 实际由代理值补齐的键（按代理表顺序）；
        * ``source`` —— ``"workcell_config"``（全部来自工位配置）或
          ``"proxy_default"``（有键来自代理表）。

    **不补默认值以外的东西**：工位配置里的合法值一律原样保留。
    """
    raw = dict((cfg or {}).get("thresholds") or {})
    effective: Dict[str, Any] = {}
    proxy_used: List[str] = []
    for key, proxy_value in GRIPPER_PROXY_THRESHOLDS.items():
        value = raw.get(key)
        if value is None:
            effective[key] = proxy_value
            proxy_used.append(key)
        else:
            effective[key] = value
    window = effective.get("grasp_position_window")
    if isinstance(window, (list, tuple)):
        effective["grasp_position_window"] = tuple(float(v) for v in window)
    source = SOURCE_PROXY if proxy_used else SOURCE_WORKCELL
    return effective, proxy_used, source


def _import_gripper_config():
    """惰性导入 ``rm_hw_bridge.gripper_config``（附兜底 PYTHONPATH）。

    返回 ``(module, error_reason)``；``module`` 为 ``None`` 时
    ``error_reason`` 说明原因（**不把 ImportError 堆栈甩给操作者**）。
    """
    def _import():
        from rm_hw_bridge import gripper_config as _cfg

        return _cfg

    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001 - 统一降级
        first = repr(exc)
    from . import paths as _paths

    root = _paths.workspace_root()
    pkg_parent = os.path.join(root, "dualarm_ws", "src", "rm_hw_bridge")
    if os.path.isdir(pkg_parent) and pkg_parent not in sys.path:
        sys.path.insert(0, pkg_parent)
    try:
        return _import(), ""
    except Exception as exc:  # noqa: BLE001
        return None, "%s / 兜底路径 %s: %r" % (first, pkg_parent, exc)


def load_effective_thresholds(ws: Optional[str] = None
                              ) -> Tuple[Optional[Dict[str, Any]],
                                         List[str],
                                         str,
                                         Dict[str, Any]]:
    """从工位文件读取夹爪配置，并与代理阈值合并（**唯一来源链**）。

    :param ws: 工作区根（缺省自动查找）。
    :return: ``(thresholds, proxy_used_keys, source, config_result)``：

        * ``thresholds`` —— 工位配置不可用时为 ``None``；
        * ``proxy_used_keys`` —— 由代理值补齐的键；
        * ``source`` —— ``"workcell_config"`` / ``"proxy_default"`` /
          ``"unavailable"``；
        * ``config_result`` —— 原始配置加载结果（``status`` / ``reason`` /
          ``next_action`` / ``evidence``）。

    **工艺层不读工位配置**（那属于编排层的事），本函数供编排层取阈值用。
    """
    module, err = _import_gripper_config()
    if module is None:
        return None, [], "unavailable", {
            "status": "blocked",
            "reason": "硬件适配层 rm_hw_bridge 不可用: %s" % err,
            "next_action": ("export PYTHONPATH=%s:$PYTHONPATH  "
                            "# 让 rmctl 看到硬件适配层"
                            % os.path.join(os.path.dirname(os.path.dirname(
                                os.path.dirname(os.path.abspath(__file__)))),
                                "rm_hw_bridge")),
            "evidence": [err],
        }
    result = module.load_gripper_config(ws)
    if result.get("status") != "ok":
        return None, [], "unavailable", result
    thresholds, proxy_used, source = effective_thresholds(result.get("config"))
    return thresholds, proxy_used, source, result
