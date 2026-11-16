"""回归层（S07）：全链路会话产物与 golden-file 基线。

分层与依赖
----------
本包是**回归 / 对照**侧的产物层，与既有的两份产物 schema 并列、互不修改：

* :mod:`rm75_drake.recording`（S02，``RECORDING_SCHEMA_VERSION = 1``）
  —— 关节流 + 夹爪调度（**只读消费**）；
* :mod:`rm75_drake.device.run_log`（S03，``RUN_LOG_SCHEMA_VERSION = 1``）
  —— 设备层消息流（**只读消费**）；
* :mod:`rm75_drake.regression.session`（S07-T01，``SESSION_SCHEMA_VERSION = 1``）
  —— **全链路会话产物**：把上面两份产物内联进**一个自洽文件**，
  并加**顶层校验和**（篡改可检出）。

为什么另立 schema（而不是改既有两份）
--------------------------------------
既有两份 schema 互不相通：``run_log`` 的事件 kind 白名单里没有夹爪动作
⇒ ``Recording.gripper_schedule`` 在 run_log 中**无来源**；两者的
``command.joint_rad`` 与 ``Recording.samples[].q`` 语义重叠但格式不同，
没有任何现成的转换 / 合并代码。改既有 schema 会升版，并击穿本切片自己即将
建立的 golden 基线（仓库既有先例：``run_log.py`` / ``tasks/schema.py`` /
``devices/*.yaml`` 都是「新需求另立 schema + 另立真相源」）。
因此本包**只读消费**既有两份产物，不修改它们任何一行。

零 pydrake 依赖
---------------
本包采用 **PEP 562 惰性导入**（仿 :mod:`rm75_drake.tasks`）：
``import rm75_drake.regression`` **不会**把 pydrake 拉进 ``sys.modules``，
会话 schema 的读写与校验可以离线单测、不被 Drake 版本绑死。
"""

from __future__ import annotations

import importlib
from typing import Any

__all__ = [
    "session",
    "SESSION_SCHEMA_VERSION",
    "SessionError",
    "Session",
    "build_session",
    "write_session",
    "load_session",
    "compute_session_checksum",
    "session_segment_digests",
    "source_schema_versions",
    "assert_source_schemas_unchanged",
]

#: 惰性子模块（PEP 562）：import 本包不拉起任何子模块。
_LAZY_MODULES = ("session",)

#: 从 ``session`` 子模块透出的符号。
_SESSION_EXPORTS = (
    "SESSION_SCHEMA_VERSION",
    "SessionError",
    "Session",
    "build_session",
    "write_session",
    "load_session",
    "compute_session_checksum",
    "session_segment_digests",
    "source_schema_versions",
    "assert_source_schemas_unchanged",
)


def __getattr__(name: str) -> Any:
    """PEP 562 惰性导入 / 符号透出。"""
    if name in _LAZY_MODULES:
        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    if name in _SESSION_EXPORTS:
        module = importlib.import_module(f"{__name__}.session")
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
