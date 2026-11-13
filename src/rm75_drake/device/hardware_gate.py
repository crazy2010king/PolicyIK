"""真机可达性**门控**与「未验证指标」清单（S03-T08）。

为什么要有门控层
----------------
研究 F1 判定真机在当前环境**不可达**（两台开发机都 ping 不到机械臂），
因此物理指标（5 ms 周期 P99.9 抖动 < 0.5 ms、末端跟踪误差 < 3 mm、
急停响应 < 100 ms、100 Hz×30 min 长稳、四类故障注入的物理验证）
**不能**写成 CI 可达成的验收项。

但「不可达」必须被**如实表达**，两头都不能骗：

* 不得伪装成**失败**——否则脚本无法自动执行，CI 永远红；
* 不得伪装成**成功**——那就等于用假数据冒充真机验收。

⇒ 约定（与 ``rm75_drake.cli`` 的 ``exec`` 子命令**同一份**实现）：

* 探测失败 ⇒ 输出 ``SKIP(reason=hardware_unreachable)`` + **逐条**列出
  未验证的真机指标，退出码 **0**；
* 探测成功 ⇒ 真跑测量，达不到阈值就**如实失败**（退出码 1）。

真相源纪律
----------
本模块是「硬件门控 + 未验证指标清单」的**唯一**真相源：
:mod:`rm75_drake.cli` 的 ``exec`` 子命令与 ``scripts/`` 下的三个测量脚本
（``measure_jitter.py`` / ``soak_test.py`` / ``fault_inject.py``）
都从这里取常量，避免出现第二份「未验证指标」清单。

本模块**零 pydrake 依赖**（只依赖标准库 + 本包的配置层），
因此在没有任何硬件、没有 Drake 的环境里也能被直接测。
"""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence

__all__ = [
    "SKIP_REASON_HARDWARE_UNREACHABLE",
    "UNVERIFIED_HARDWARE_METRICS",
    "DEFAULT_CONNECT_TIMEOUT_S",
    "HardwareProbe",
    "probe_hardware",
    "format_skip",
    "unverified_lines",
]


#: 「真机不可达」的**稳定** reason 标签（脚本与 CI 按它判定 SKIP）。
SKIP_REASON_HARDWARE_UNREACHABLE = "hardware_unreachable"

#: 真机不可达时**必须如实列出**的未验证指标（绝不假装通过、绝不省略）。
#: 这份清单是「硬件门控」的语义核心：它把「没测的东西」显式写在输出里。
UNVERIFIED_HARDWARE_METRICS: Sequence[str] = (
    "5 ms 透传周期抖动（P50/P99/P99.9/max）",
    "末端跟踪误差（目标 < 3 mm）",
    "急停响应时间（目标 < 100 ms）",
    "100 Hz × 30 min 长稳",
    "四类故障注入的物理验证（拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位）",
)

#: 探测 TCP 指令通道的默认超时（s）。
DEFAULT_CONNECT_TIMEOUT_S = 1.0


@dataclass(frozen=True)
class HardwareProbe:
    """一次真机可达性探测的结果（**不抛异常**，只如实报告）。

    字段
    ----
    reachable:
        是否可达。
    reason:
        不可达时的**稳定标签**（可达时为空串）。
    detail:
        人类可读的原因（可达时为空串）。
    """

    reachable: bool
    reason: str = ""
    detail: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "reachable": bool(self.reachable),
            "reason": self.reason,
            "detail": self.detail,
        }


def probe_hardware(config: Any, *, timeout: float = DEFAULT_CONNECT_TIMEOUT_S) -> HardwareProbe:
    """探测真机是否可达（**SDK 可导入 → TCP 指令通道可连接**）。

    参数
    ----
    config:
        设备配置（需要 ``arm_ip`` 与 ``tcp_port``）。
    timeout:
        TCP 连接超时（s）。

    返回
    ----
    :class:`HardwareProbe`
        不可达时 ``reason == "hardware_unreachable"``，``detail`` 说明原因。

    说明
    ----
    * SDK 缺失 → 不可达（连不上谈不上）；
    * TCP 连不上 → 不可达（网段 / 上电 / IP 不对）；
    * 两者都通过 ⇒ 判为可达（**只说明链路通**，不代表机械臂已使能）。
    """
    from .sdk import sdk_available

    if not sdk_available():
        return HardwareProbe(
            False,
            SKIP_REASON_HARDWARE_UNREACHABLE,
            "厂商 SDK 未安装（分发名 robotic-arm，导入名 Robotic_Arm），"
            "无法建立真机链路",
        )
    host = str(getattr(config, "arm_ip", ""))
    port = int(getattr(config, "tcp_port", 0))
    try:
        with socket.create_connection((host, port), timeout=max(0.05, float(timeout))):
            pass
    except OSError as exc:
        return HardwareProbe(
            False,
            SKIP_REASON_HARDWARE_UNREACHABLE,
            f"无法连接 TCP 指令通道 {host}:{port}（{type(exc).__name__}）",
        )
    return HardwareProbe(True, "", "")


def unverified_lines(metrics: Iterable[str]) -> List[str]:
    """返回「未验证指标」的逐行文本（供人类阅读 / 测试断言）。"""
    return [f"  - {item}" for item in metrics]


def format_skip(
    probe: HardwareProbe, metrics: Iterable[str] = UNVERIFIED_HARDWARE_METRICS
) -> str:
    """返回 ``SKIP`` 文本块（**含逐条未验证指标**）。

    输出形如::

        SKIP(reason=hardware_unreachable)：无法连接 TCP 指令通道 192.168.1.18:8080
        以下真机指标**未验证**（不得当作通过）：
          - 5 ms 透传周期抖动（P50/P99/P99.9/max）
          - ...
    """
    reason = probe.reason or SKIP_REASON_HARDWARE_UNREACHABLE
    detail = probe.detail or "真机不可达"
    lines = [f"SKIP(reason={reason})：{detail}"]
    lines.append("以下真机指标**未验证**（不得当作通过）：")
    lines.extend(unverified_lines(metrics))
    return "\n".join(lines)
