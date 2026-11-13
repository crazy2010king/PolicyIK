"""安全闸门（**进程外强制停机**通道）。

问题
----

执行器（``rm_executor``）在自己的进程里跑，操作者/看门狗进程必须能在
**不进入执行器内部**的前提下把机械臂停下来。当前执行层没有任何外部停机
入口 —— 这就是本模块要补的那块。

选型
----

用 **Unix domain socket**（``socket.AF_UNIX``）而不是文件轮询或信号：

* 文件轮询有延迟与竞态（写文件与读文件的时序不可控）；
* 信号无法携带**等级**（L1 降速 / L2 暂停 / L3 缓停 / L4 急停）；
* ``socket`` 是标准库（**零新依赖**），且天然是「外部进程 → 执行器」的
  **单向**通道。

协议（一行 JSON 请求 → 一行 JSON 响应）
----------------------------------------

请求::

    {"cmd": "stop", "level": "L4"}     # 强制停机（level 缺省 L4）
    {"cmd": "status"}                   # 查询闸门状态

响应::

    {"status": "ok", "cmd": "stop", "level": "L4", "stop_requested": true, ...}
    {"status": "error", "reason": "请求不是合法 JSON: ...", "next_action": "..."}

**闸门只做停机请求，不做配置下发**：除 ``stop`` / ``status`` 之外的命令一律
拒绝（非法 JSON、非法等级、未知命令都回一行错误 JSON，**不静默忽略**）。

类
--

* :class:`InProcessGate` —— 进程内共享状态（``threading.Lock`` + 请求标志 +
  等级），供执行器每 tick 查询（``stop_requested`` / ``requested_level``）；
* :class:`UnixSocketGate` —— 在 :class:`InProcessGate` 之上加 **socket 服务端**
  （独立线程，不依赖 tick —— 停机请求不能等 tick 才有响应）；
* :class:`GateServer` —— 生命周期门面（``start()`` / ``stop()`` /
  ``socket_path``）；路径被占用或创建失败 → **诚实报错**
  （:class:`GateError`，带可粘贴 ``next_action``），**不假装成功**。

依赖方向
--------

本模块**只依赖标准库** + 本包的 :mod:`rm_safety.levels`（等级唯一真值），
**不 import** ``rm_executor`` / ``rm_hw_bridge`` / ``rmctl`` —— 安全层
不得反向依赖任何上层（层序：hardware < safety < executor < experience）。
"""
from __future__ import annotations

import json
import os
import socket
import threading
from typing import Any, Callable, Dict, List, Optional

from rm_safety import levels as _levels
from rm_safety import paths as _paths

__all__ = [
    "CMD_STATUS",
    "CMD_STOP",
    "DEFAULT_SOCKET_RELPATH",
    "MAX_REQUEST_BYTES",
    "GateError",
    "GateServer",
    "InProcessGate",
    "UnixSocketGate",
    "default_socket_path",
    "handle_gate_request",
]

#: 停机命令
CMD_STOP = "stop"
#: 状态查询命令
CMD_STATUS = "status"

#: 默认 socket 路径（相对工作区根）
DEFAULT_SOCKET_RELPATH = os.path.join("build", "run", "rm_runtime.sock")

#: 单条请求的最大字节数（防呆：闸门只接受一行小 JSON）
MAX_REQUEST_BYTES = 8192

#: ``sockaddr_un.sun_path`` 长度上限（Linux = 108 字节，含结尾 NUL）
_SOCKET_PATH_MAX_BYTES = 107

#: 服务端 accept 超时（秒）—— 让线程能及时退出）
_ACCEPT_TIMEOUT_S = 0.2

#: 单个客户端的读超时（秒）
_CLIENT_TIMEOUT_S = 2.0

#: 停机请求的默认原因（写进证据与事件）
DEFAULT_STOP_REASON = "外部闸门请求"

#: 错误记录保留上限（内存有界：淘汰旧记录，但**计数不受影响**）
MAX_ERROR_RECORDS = 100


def default_socket_path(ws: Optional[str] = None) -> str:
    """默认闸门 socket 路径（工作区根下的 ``build/run/rm_runtime.sock``）。"""
    return os.path.join(_paths.workspace_root(ws), DEFAULT_SOCKET_RELPATH)


class GateError(RuntimeError):
    """闸门启动/运行错误（**诚实报错**：带 reason + 可粘贴 next_action）。"""

    def __init__(self, reason: str, next_action: str = "") -> None:
        super().__init__(reason)
        self.reason = reason
        self.next_action = next_action

    def to_dict(self) -> Dict[str, Any]:
        """自描述字典（报告 / CLI 输出用）。"""
        return {
            "status": "error",
            "reason": self.reason,
            "next_action": self.next_action,
        }


class InProcessGate:
    """进程内共享的停机状态（**唯一真值**：请求标志 + 等级）。

    线程安全：``threading.Lock`` 保护状态；停机回调（handler）在**锁外**
    调用，避免回调里再碰闸门造成死锁。回调异常被记录到
    :attr:`handler_errors`，**不让它掀翻服务端线程**。
    """

    def __init__(self, on_stop: Optional[Callable[..., Any]] = None) -> None:
        self._lock = threading.Lock()
        self._stop_requested = False
        self._level = ""
        self._reason = ""
        self._source = ""
        self._request_count = 0
        self._handlers: List[Callable[..., Any]] = []
        self._handler_errors: List[str] = []
        self._handler_error_count = 0
        if on_stop is not None:
            self.add_handler(on_stop)

    # ------------------------------------------------------------ 状态查询
    @property
    def stop_requested(self) -> bool:
        """是否已收到停机请求（执行器每 tick 查询这个标志）。"""
        with self._lock:
            return self._stop_requested

    @property
    def requested_level(self) -> str:
        """请求的停机等级（``L1``–``L4``；无请求时为空串）。"""
        with self._lock:
            return self._level

    @property
    def reason(self) -> str:
        """最近一次停机请求的原因。"""
        with self._lock:
            return self._reason

    @property
    def source(self) -> str:
        """最近一次停机请求的来源（如 ``socket``）。"""
        with self._lock:
            return self._source

    @property
    def request_count(self) -> int:
        """停机请求累计次数（审计用）。"""
        with self._lock:
            return self._request_count

    @property
    def handler_errors(self) -> List[str]:
        """停机回调抛出的异常记录（留痕，不吞掉）。"""
        with self._lock:
            return list(self._handler_errors)

    @property
    def handler_error_count(self) -> int:
        """停机回调异常**总次数**（**不受保留上限影响**）。"""
        with self._lock:
            return self._handler_error_count

    # ------------------------------------------------------------ 停机回调
    def add_handler(self, handler: Callable[..., Any]) -> None:
        """注册停机回调 ``f(level, reason, source)``（幂等：同一函数只注册一次）。"""
        with self._lock:
            if handler not in self._handlers:
                self._handlers.append(handler)

    def remove_handler(self, handler: Callable[..., Any]) -> None:
        """注销停机回调（幂等）。"""
        with self._lock:
            if handler in self._handlers:
                self._handlers.remove(handler)

    # ------------------------------------------------------------ 停机请求
    def request_stop(self, level: str = _levels.LEVEL_L4,
                     reason: str = DEFAULT_STOP_REASON,
                     source: str = "") -> Dict[str, Any]:
        """登记一次停机请求并通知所有回调。

        * 等级**单调**：已有请求时保留**更重**的等级（L4 不会被 L2 覆盖）；
        * 回调在锁外调用，异常被记录到 :attr:`handler_errors`；
        * 非法等级抛 :class:`ValueError`（**不静默兜底**）。
        """
        level = str(level or "").strip().upper()
        if level not in _levels.LEVEL_ORDER:
            raise ValueError("非法安全等级 %r（只允许 %s）"
                             % (level, ", ".join(_levels.LEVEL_ORDER)))
        with self._lock:
            self._request_count += 1
            if not self._stop_requested:
                self._stop_requested = True
                self._level = level
                self._reason = reason or DEFAULT_STOP_REASON
                self._source = source or ""
            elif _levels.level_rank(level) > _levels.level_rank(self._level):
                # 等级单调：更重的请求覆盖更轻的
                self._level = level
                self._reason = reason or self._reason
                self._source = source or self._source
            handlers = list(self._handlers)
            effective_level = self._level
            effective_reason = self._reason
            count = self._request_count
        for handler in handlers:
            try:
                handler(effective_level, effective_reason, source or "")
            except Exception as exc:  # noqa: BLE001 - 回调异常不得掀翻服务端
                with self._lock:
                    self._handler_error_count += 1
                    self._handler_errors.append(
                        "停机回调异常（%s）: %s" % (type(exc).__name__, exc))
                    if len(self._handler_errors) > MAX_ERROR_RECORDS:
                        del self._handler_errors[0]
        return {
            "status": "ok",
            "cmd": CMD_STOP,
            "level": effective_level,
            "reason": effective_reason,
            "stop_requested": True,
            "request_count": count,
        }

    def clear(self) -> None:
        """复位停机状态（**仅供测试 / 进程内复用**；已发的回调不回收）。"""
        with self._lock:
            self._stop_requested = False
            self._level = ""
            self._reason = ""
            self._source = ""
            self._request_count = 0

    # ------------------------------------------------------------ 自描述
    def to_dict(self) -> Dict[str, Any]:
        """自描述字典（status 响应 / 报告用）。"""
        with self._lock:
            return {
                "status": "ok",
                "cmd": CMD_STATUS,
                "stop_requested": self._stop_requested,
                "requested_level": self._level,
                "reason": self._reason,
                "source": self._source,
                "request_count": self._request_count,
                "pid": os.getpid(),
            }

    def status(self) -> Dict[str, Any]:
        """闸门状态（等价 :meth:`to_dict`）。"""
        return self.to_dict()


class UnixSocketGate(InProcessGate):
    """Unix domain socket 闸门（**服务端**）。

    协议见模块文档。服务线程**独立于 tick** —— 停机请求不能等 tick 才有
    响应（tick 是 10 ms 栅格，而外部停机必须「立即」）。

    参数
    ----
    socket_path
        socket 路径；``None`` → :func:`default_socket_path`。
    backlog
        listen backlog。
    on_stop
        停机回调 ``f(level, reason, source)``（可选）。
    """

    def __init__(self, socket_path: Optional[str] = None, backlog: int = 8,
                 on_stop: Optional[Callable[..., Any]] = None) -> None:
        super().__init__(on_stop=on_stop)
        self._socket_path = os.path.abspath(socket_path or default_socket_path())
        self._backlog = max(1, int(backlog))
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._stop_event = threading.Event()
        self._server_errors: List[str] = []
        self._server_error_count = 0

    # ------------------------------------------------------------ 属性
    @property
    def socket_path(self) -> str:
        """socket 路径（绝对路径）。"""
        return self._socket_path

    @property
    def running(self) -> bool:
        """服务线程是否在跑。"""
        return self._running

    @property
    def server_errors(self) -> List[str]:
        """服务端线程异常记录（留痕，不静默）。"""
        with self._lock:
            return list(self._server_errors)

    @property
    def server_error_count(self) -> int:
        """服务端异常**总次数**（**不受保留上限影响**）。"""
        with self._lock:
            return self._server_error_count

    def _record_server_error(self, message: str) -> None:
        """记一条服务端异常（**内存有界**：记录上限 + 计数）。"""
        with self._lock:
            self._server_error_count += 1
            self._server_errors.append(message)
            if len(self._server_errors) > MAX_ERROR_RECORDS:
                del self._server_errors[0]

    # ------------------------------------------------------------ 生命周期
    def start(self) -> Dict[str, Any]:
        """启动 socket 服务端。

        失败（路径过长 / 目录不可写 / 路径被占用 / bind 失败）→ 抛
        :class:`GateError`（带可粘贴 ``next_action``），**不假装成功**。
        """
        if self._running:
            return {"status": "ok", "socket_path": self._socket_path,
                    "already_running": True}
        path = self._socket_path
        if len(path.encode("utf-8")) > _SOCKET_PATH_MAX_BYTES:
            raise GateError(
                "安全闸门 socket 路径过长（%d 字节 > %d）: %s"
                % (len(path.encode("utf-8")), _SOCKET_PATH_MAX_BYTES, path),
                "用 --gate-socket <更短的路径> 重启 rm_runtime"
                "（例如 --gate-socket /tmp/rm_runtime.sock）")
        parent = os.path.dirname(path) or "."
        try:
            os.makedirs(parent, exist_ok=True)
        except OSError as exc:
            raise GateError(
                "无法创建安全闸门 socket 目录 %s（%s）: %s"
                % (parent, type(exc).__name__, exc),
                "确认目录可写（mkdir -p %s），或用 --gate-socket <可写路径> 重启"
                % parent) from exc
        if os.path.exists(path):
            if _socket_alive(path):
                raise GateError(
                    "安全闸门 socket 已被占用：%s（已有进程在监听）" % path,
                    "停掉占用该路径的 rm_runtime，或用 --gate-socket <其它路径> 重启")
            # 陈旧 socket（上次进程异常退出留下）→ 删除后重建（留痕）
            try:
                os.unlink(path)
            except OSError as exc:
                raise GateError(
                    "无法清理陈旧的安全闸门 socket %s（%s）: %s"
                    % (path, type(exc).__name__, exc),
                    "手动删除该文件后重试：rm -f %s" % path) from exc
            with self._lock:
                self._server_error_count += 1
                self._server_errors.append("已清理陈旧 socket：%s" % path)
                if len(self._server_errors) > MAX_ERROR_RECORDS:
                    del self._server_errors[0]
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.bind(path)
            sock.listen(self._backlog)
            sock.settimeout(_ACCEPT_TIMEOUT_S)
        except OSError as exc:
            sock.close()
            raise GateError(
                "创建安全闸门 socket 失败（%s）: %s" % (path, exc),
                "确认路径可写且未被占用，或用 --gate-socket <其它路径> 重试") from exc
        self._sock = sock
        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._serve_loop,
                                        name="rm-safety-gate", daemon=True)
        self._thread.start()
        return {"status": "ok", "socket_path": path}

    def stop(self) -> Dict[str, Any]:
        """停止服务端并清理 socket 文件（幂等）。"""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:  # pragma: no cover - 关闭失败不影响退出
                pass
        self._running = False
        path = self._socket_path
        if os.path.exists(path):
            try:
                os.unlink(path)
            except OSError:  # pragma: no cover - 清理失败不影响退出
                pass
        return {"status": "ok", "socket_path": path, "stopped": True}

    # ------------------------------------------------------------ 服务循环
    def _serve_loop(self) -> None:
        """accept 循环（独立线程；异常留痕不掀翻进程）。"""
        while not self._stop_event.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                conn, _ = sock.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                if self._stop_event.is_set():
                    break
                self._record_server_error("accept 失败: %s" % exc)
                continue
            try:
                self._handle_conn(conn)
            except Exception as exc:  # noqa: BLE001 - 单个连接异常不影响服务
                self._record_server_error(
                    "连接处理异常（%s）: %s" % (type(exc).__name__, exc))
            finally:
                try:
                    conn.close()
                except OSError:  # pragma: no cover
                    pass

    def _handle_conn(self, conn: socket.socket) -> None:
        """读一行请求 → 处理 → 回一行 JSON 响应。"""
        try:
            conn.settimeout(_CLIENT_TIMEOUT_S)
        except OSError:  # pragma: no cover
            pass
        raw = self._read_line(conn)
        response = handle_gate_request(self, raw)
        payload = (json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            conn.sendall(payload)
        except OSError:
            # 客户端已断开：服务端不视为错误（**不掀翻服务线程**）
            pass

    @staticmethod
    def _read_line(conn: socket.socket) -> str:
        """读一行（到 ``\\n`` / EOF / 超时 / 超长为止）。"""
        buf = b""
        while b"\n" not in buf and len(buf) < MAX_REQUEST_BYTES:
            try:
                chunk = conn.recv(1024)
            except socket.timeout:
                break
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
        return buf.decode("utf-8", errors="replace").strip()

    # ------------------------------------------------------------ 状态
    def to_dict(self) -> Dict[str, Any]:
        """闸门状态（含 socket 路径与服务端异常记录）。"""
        data = super().to_dict()
        data["socket_path"] = self._socket_path
        data["running"] = self._running
        data["server_errors"] = self.server_errors
        data["server_error_count"] = self.server_error_count
        data["handler_errors"] = self.handler_errors
        data["handler_error_count"] = self.handler_error_count
        return data


def handle_gate_request(gate: InProcessGate, raw: Any) -> Dict[str, Any]:
    """处理一行请求文本 → 响应字典（**纯逻辑**，便于离线逐条断言）。

    非法 JSON / 非法等级 / 未知命令 → ``status="error"`` + reason +
    可粘贴 next_action（**不静默忽略**）。
    """
    text = raw if isinstance(raw, str) else str(raw or "")
    text = text.strip()
    if not text:
        return {
            "status": "error",
            "cmd": "",
            "reason": "请求为空（期望一行 JSON）",
            "next_action": ('请求格式：{"cmd": "stop", "level": "L4"} 或 '
                            '{"cmd": "status"}'),
        }
    try:
        payload = json.loads(text)
    except (ValueError, TypeError) as exc:
        return {
            "status": "error",
            "cmd": "",
            "reason": "请求不是合法 JSON: %s" % exc,
            "next_action": ('请求格式（一行 JSON）：{"cmd": "stop", "level": "L4"} '
                            '或 {"cmd": "status"}'),
        }
    if not isinstance(payload, dict):
        return {
            "status": "error",
            "cmd": "",
            "reason": "请求不是 JSON 对象（当前 %s）" % type(payload).__name__,
            "next_action": '请求格式（一行 JSON 对象）：{"cmd": "stop", "level": "L4"}',
        }
    cmd = str(payload.get("cmd") or "").strip().lower()
    if cmd == CMD_STATUS:
        return gate.status()
    if cmd != CMD_STOP:
        return {
            "status": "error",
            "cmd": cmd,
            "reason": "未知命令 %r（闸门只接受 %s / %s）" % (cmd, CMD_STOP, CMD_STATUS),
            "next_action": '只支持 {"cmd": "stop", "level": "L4"} 与 {"cmd": "status"}',
        }
    raw_level = payload.get("level")
    level = _levels.LEVEL_L4 if raw_level in (None, "") else str(raw_level).strip().upper()
    if level not in _levels.LEVEL_ORDER:
        return {
            "status": "error",
            "cmd": CMD_STOP,
            "reason": "非法安全等级 %r（只允许 %s）"
                      % (raw_level, ", ".join(_levels.LEVEL_ORDER)),
            "next_action": "用 L1 / L2 / L3 / L4 之一重发（缺省 L4 急停）",
        }
    reason = str(payload.get("reason") or DEFAULT_STOP_REASON)
    result = gate.request_stop(level=level, reason=reason, source="socket")
    result["socket_path"] = getattr(gate, "socket_path", "")
    return result


class GateServer:
    """闸门生命周期门面（``start()`` / ``stop()`` / ``socket_path``）。

    启动失败 → 抛 :class:`GateError`（**诚实报错** + 可粘贴 next_action）。
    """

    def __init__(self, socket_path: Optional[str] = None, backlog: int = 8,
                 on_stop: Optional[Callable[..., Any]] = None) -> None:
        self._gate = UnixSocketGate(socket_path=socket_path, backlog=backlog,
                                    on_stop=on_stop)

    @property
    def gate(self) -> UnixSocketGate:
        """底层闸门（执行器用它查询 ``stop_requested``）。"""
        return self._gate

    @property
    def socket_path(self) -> str:
        """socket 路径。"""
        return self._gate.socket_path

    @property
    def running(self) -> bool:
        """服务是否在跑。"""
        return self._gate.running

    def start(self) -> Dict[str, Any]:
        """启动服务端（失败抛 :class:`GateError`）。"""
        return self._gate.start()

    def stop(self) -> Dict[str, Any]:
        """停止服务端并清理 socket 文件（幂等）。"""
        return self._gate.stop()

    def status(self) -> Dict[str, Any]:
        """闸门状态。"""
        return self._gate.status()


def _socket_alive(path: str) -> bool:
    """探测 socket 路径是否**真的有人在监听**（区分陈旧文件与占用）。"""
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.settimeout(0.5)
    try:
        probe.connect(path)
        return True
    except OSError:
        return False
    finally:
        try:
            probe.close()
        except OSError:  # pragma: no cover
            pass
