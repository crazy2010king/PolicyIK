"""真机进程的**安全退出**（绕开解释器清理阶段的段错误）。

为什么需要
----------
真机实测（RM75-B / 厂商 SDK 1.1.7）：SDK 的 UDP 接收线程是**原生线程**，
在解释器退出阶段仍可能回调已释放的 Python 对象 ⇒
``Fatal Python error: Segmentation fault``，且崩溃时机**随机**
（可能发生在报告写盘之前 ⇒ 证据丢失）。

处理方式
--------
1. 回调对象**进程级保活**（见 :mod:`rm75_drake.device.realtime`）；
2. 报告 / 输出全部 ``flush`` 之后，用 :func:`os._exit` 直接退出
   ⇒ 句柄、socket、线程由内核回收，**不会段错误，也不会丢证据**。
"""

from __future__ import annotations

import os
import sys

__all__ = ["safe_exit"]


def safe_exit(code: int) -> None:
    """刷新输出后**直接退出进程**（不跑解释器清理阶段）。

    参数
    ----
    code:
        进程退出码。
    """
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:  # pragma: no cover - 收尾尽力而为
        pass
    os._exit(int(code))
