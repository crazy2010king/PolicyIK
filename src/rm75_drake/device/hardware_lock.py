"""真机**互斥锁**：同一台机械臂同一时刻只允许一个进程占用。

为什么需要
----------
厂商约束：**同一 IP+port 只能被一个进程占用**（《设计文档》§八.3 已写明）。
真机实测（2026-11-20）踩了这个坑：长稳（soak）在跑时，
另一个进程（轨迹执行）也去连 `192.168.1.18:8080` ⇒ 长稳的
``rm_movej_canfd`` 立刻返回 ``-2``（发送失败）⇒ 整个长稳作废。

⇒ 所有**会碰真机**的入口（CLI ``exec``、阶梯脚本、测量脚本、标定脚本、
故障注入脚本）都先拿一把**按机械臂 IP 命名的文件锁**：
拿不到就**立刻报错退出**（而不是把对方的连接顶掉）。

实现
----
``fcntl.flock``（POSIX advisory lock）：

* 锁文件放在系统临时目录，名字带机械臂 IP ⇒ 跨机器/跨进程都互斥；
* ``LOCK_EX | LOCK_NB`` ⇒ 非阻塞：被占用时立即抛
  :class:`HardwareLockBusy`（消息里写明**是谁**占着、怎么查）；
* 进程退出时内核自动释放 ⇒ 崩溃不会留下死锁。
"""

from __future__ import annotations

import errno
import fcntl
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Iterator, Optional

__all__ = [
    "HardwareLockBusy",
    "lock_path_for",
    "hardware_lock",
]


class HardwareLockBusy(RuntimeError):
    """真机已被另一个进程占用（同一 IP+port 只能一个进程连）。"""


#: 本进程已持有的锁（键 = 机械臂 IP）——**同进程可重入**
#: （厂商约束是「跨进程」互斥；同一进程内多个传输层对象不应互相打架）。
_HELD: Dict[str, Dict[str, Any]] = {}


def lock_path_for(arm_ip: str, *, lock_dir: Optional[Path] = None) -> Path:
    """返回某台机械臂的锁文件路径。"""
    base = Path(lock_dir) if lock_dir is not None else Path(tempfile.gettempdir())
    safe = str(arm_ip).replace(":", "_").replace("/", "_")
    return base / f"rm75_hardware_{safe}.lock"


@contextmanager
def hardware_lock(
    arm_ip: str, *, lock_dir: Optional[Path] = None, holder: str = ""
) -> Iterator[Path]:
    """拿住某台真机的互斥锁（上下文管理器）。

    参数
    ----
    arm_ip:
        机械臂 IP（锁的键）。
    lock_dir:
        锁文件目录；``None`` ⇒ 系统临时目录。
    holder:
        持有者描述（写进锁文件，便于 ``cat`` 排查是谁占着）。

    异常
    ----
    :class:`HardwareLockBusy`
        锁已被别的进程持有。
    """
    key = str(arm_ip)
    held = _HELD.get(key)
    if held is not None:
        # 同进程重入：计数 +1（不重复 flock）。
        held["count"] = int(held["count"]) + 1
        try:
            yield held["path"]
        finally:
            held["count"] = int(held["count"]) - 1
        return

    path = lock_path_for(arm_ip, lock_dir=lock_dir)
    handle = open(path, "a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno not in (errno.EACCES, errno.EAGAIN):
                raise
            handle.seek(0)
            try:
                current = handle.read().strip()
            except Exception:  # pragma: no cover - 读不出来也要报错
                current = ""
            raise HardwareLockBusy(
                f"机械臂 {arm_ip} 已被另一个进程占用（同一 IP+port 只能一个进程连）："
                f"锁文件 {path}"
                + (f"，当前持有者：{current}" if current else "")
                + "；请先结束那个进程（真机上并发跑两个控制进程会互相顶掉连接，"
                "表现为 rm_movej_canfd 返回 -2）"
            ) from exc
        try:
            handle.seek(0)
            handle.truncate()
            handle.write(
                f"pid={os.getpid()} holder={holder or 'unknown'} "
                f"started={__import__('time').strftime('%Y-%m-%dT%H:%M:%S')}"
            )
            handle.flush()
        except Exception:  # pragma: no cover - 写不进去不影响互斥
            pass
        _HELD[key] = {"path": path, "count": 1, "handle": handle}
        try:
            yield path
        finally:
            _HELD.pop(key, None)
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            finally:
                handle.close()
    except HardwareLockBusy:
        handle.close()
        raise
