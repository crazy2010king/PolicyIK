"""rm_planner 规划服务的**薄客户端**（Unix socket 控制面）。

这个模块是 ``rmctl plan --via-service`` 的**唯一**客户端实现：把一个请求
字典发给 ``rm_planner`` 服务，拿回一行 JSON 响应。它**不碰厂商规划内核**
（零 SDK），因此规划进程可以独立崩溃 / 重启而不影响客户端以外的链路。

协议（与服务端 :mod:`rmctl.planner_service.server` **逐字一致**）：

* 请求：一行 JSON —— ``{"cmd": "plan", "request": {...}}`` /
  ``{"cmd": "cancel", "plan_id": "..."}`` / ``{"cmd": "status"}``；
* 响应：一行 JSON —— plan 返回契约层的 ``PlanResponse`` 字典，
  cancel / status 返回各自的字典。

纪律（与契约层、服务端一致）：

1. **不抛裸异常**：连不上 socket / 读超时 / 连接中断 / 响应非法 /
   请求非法 —— 一律返回**结构化错误**（``status`` = ``FAILED`` / ``TIMEOUT`` /
   ``INVALID_REQUEST`` + ``failure_reason`` + 可粘贴的 ``metrics.next_action``）。
   调用方想要异常语义时显式调 :func:`raise_for_status`。
2. **客户端只补协议必需的缺省**：`plan_id`（缺省生成）与 ``timeout_ms``
   （缺省取构造参数）。**不补业务缺省**：其余字段缺失就是请求非法。
3. **不静默**：所有失败路径都留下 ``failure_reason``，可粘贴的下一步写在
   ``metrics.next_action``。

单位约定与契约层一致：时间秒（``timeout_ms`` 为毫秒）、角度 rad、长度 m。
"""
from __future__ import annotations

import json
import os
import socket
import uuid
from typing import Any, Dict, Optional, Tuple

from rmctl import paths
from rmctl.planner_service.contract import (
    STATUSES,
    InvalidRequest,
    PlanRequest,
    PlanResponse,
)

__all__ = [
    "DEFAULT_TIMEOUT_MS",
    "DEFAULT_CONNECT_TIMEOUT_S",
    "READ_GRACE_S",
    "PlannerClient",
    "PlannerClientError",
    "default_socket_path",
    "new_plan_id",
    "raise_for_status",
]

#: 请求缺省硬超时（毫秒）—— 与 ``workcell/planner/profiles.yaml::timeout_ms``
#: 同值；服务端以**请求里的** ``timeout_ms`` 为准（这里只是客户端缺省）。
DEFAULT_TIMEOUT_MS = 20000

#: 连接超时（秒）—— 连不上就立刻给结构化错误，不干等。
DEFAULT_CONNECT_TIMEOUT_S = 3.0

#: 读超时余量（秒）：服务端的**启动宽限**（30 s）+ 收尾余量。
#: 客户端读超时必须大于服务端的两个 deadline，否则客户端先超时、
#: 服务端还在规划（会留下一个在飞 worker）。
READ_GRACE_S = 45.0


class PlannerClientError(RuntimeError):
    """客户端侧错误（仅在调用方显式要求抛异常时产生）。"""

    def __init__(self, reason: str, next_action: str = "",
                 status: str = "FAILED") -> None:
        self.reason = reason
        self.next_action = next_action
        self.status = status
        super().__init__(reason)


def default_socket_path(ws: Optional[str] = None) -> str:
    """默认 socket 路径：``<ws>/build/run/rm_planner.sock``（与服务端同一来源）。"""
    from rmctl.planner_service.server import default_socket_path as _server_default

    return _server_default(ws)


def new_plan_id() -> str:
    """生成一个请求标识（客户端缺省时补上）。"""
    return "plan-%s" % uuid.uuid4().hex[:12]


def _error_response(
    plan_id: str,
    status: str,
    reason: str,
    next_action: str = "",
    **metrics: Any,
) -> Dict[str, Any]:
    """构造结构化错误响应（形状与契约层 ``PlanResponse`` 一致）。"""
    extra = dict(metrics)
    if next_action:
        extra["next_action"] = next_action
    return PlanResponse(
        plan_id=plan_id,
        status=status,
        trajectory=None,
        failure_reason=reason,
        metrics=extra,
    ).to_dict()


def raise_for_status(response: Any) -> Dict[str, Any]:
    """按响应状态抛异常（``SUCCESS`` 时原样返回）。

    客户端本身**不抛**异常；需要异常语义的调用方显式调本函数。

    :raises PlannerClientError: 状态不是 ``SUCCESS`` 时
    """
    if not isinstance(response, dict):
        raise PlannerClientError(
            "响应不是映射（%s）" % type(response).__name__,
            next_action="确认 socket 上跑的是 rm_planner 服务",
        )
    status = str(response.get("status") or "")
    if status == "SUCCESS":
        return response
    metrics = response.get("metrics")
    metrics = metrics if isinstance(metrics, dict) else {}
    reason = str(response.get("failure_reason") or "").strip()
    if not reason:
        reason = "规划服务返回状态 %s（无失败原因）" % (status or "未知")
    raise PlannerClientError(
        reason,
        next_action=str(metrics.get("next_action") or ""),
        status=status or "FAILED",
    )


class PlannerClient:
    """rm_planner 规划服务的薄客户端。

    :param socket_path: socket 路径；``None`` → ``<ws>/build/run/rm_planner.sock``
    :param ws: 工作区根（决定默认 socket 路径与契约校验用的配置）
    :param timeout_ms: 请求缺省硬超时（毫秒）；``None`` → 20000
    :param connect_timeout_s: 连接超时（秒）
    :param read_grace_s: 读超时余量（秒），叠在请求 deadline 之上
    """

    def __init__(
        self,
        socket_path: Optional[str] = None,
        ws: Optional[str] = None,
        timeout_ms: Optional[int] = None,
        connect_timeout_s: float = DEFAULT_CONNECT_TIMEOUT_S,
        read_grace_s: float = READ_GRACE_S,
    ) -> None:
        self.ws = os.path.abspath(ws or paths.workspace_root())
        self.socket_path = os.path.abspath(
            socket_path or default_socket_path(self.ws)
        )
        self.timeout_ms = int(timeout_ms) if timeout_ms else DEFAULT_TIMEOUT_MS
        self.connect_timeout_s = float(connect_timeout_s)
        self.read_grace_s = float(read_grace_s)

    # ------------------------------------------------------------ 元信息
    def describe(self) -> Dict[str, Any]:
        """客户端自身的连接参数（诊断用；不含秘密）。"""
        return {
            "socket_path": self.socket_path,
            "ws": self.ws,
            "timeout_ms": self.timeout_ms,
            "connect_timeout_s": self.connect_timeout_s,
        }

    # ------------------------------------------------------------ 命令
    def plan(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """发起一次规划请求 → 响应字典。

        客户端负责补两样东西：**``plan_id``**（缺省生成）与
        **``timeout_ms``**（缺省取构造参数）。其余字段缺失 → 非法请求
        （结构化 ``INVALID_REQUEST``，不抛异常）。

        :param request: 契约层请求字典（字段名与 ``PlanRequest`` 一致）
        :return: 契约层响应字典（``status`` / ``trajectory`` /
            ``verification`` / ``failure_reason`` / ``metrics``）
        """
        data = dict(request or {})
        plan_id = str(data.get("plan_id") or "").strip() or new_plan_id()
        data["plan_id"] = plan_id
        if not data.get("timeout_ms"):
            data["timeout_ms"] = int(self.timeout_ms)

        try:
            PlanRequest.from_dict(data, ws=self.ws)
        except InvalidRequest as exc:
            return _error_response(
                plan_id,
                "INVALID_REQUEST",
                "请求非法（%s）: %s" % (exc.field, exc.reason),
                next_action="按契约修正字段后重试（原因里指名了字段）",
                field=exc.field,
            )
        except Exception as exc:  # noqa: BLE001 - 契约层意外错误也不外泄
            return _error_response(
                plan_id,
                "INVALID_REQUEST",
                "请求构造失败（%s）: %s" % (type(exc).__name__, exc),
                next_action="按契约核对请求字段后重试",
            )

        payload, err = self._roundtrip(
            {"cmd": "plan", "request": data},
            plan_id=plan_id,
            timeout_s=self._read_timeout_s(data.get("timeout_ms")),
        )
        if err is not None:
            return err
        status = str(payload.get("status") or "")
        if status not in STATUSES:
            return _error_response(
                plan_id,
                "FAILED",
                "响应状态非法: %r（只允许 %s）" % (status, " | ".join(STATUSES)),
                next_action="确认 socket 上跑的是 rm_planner 服务"
                            "（rm-planner --ws <仓库根>）",
                error="malformed_response",
            )
        return payload

    def cancel(self, plan_id: str) -> Dict[str, Any]:
        """取消正在执行的 plan → 服务端字典（``status`` = ``ok`` / ``error``）。"""
        pid = str(plan_id or "").strip()
        payload, err = self._roundtrip(
            {"cmd": "cancel", "plan_id": pid},
            plan_id=pid,
            timeout_s=self.connect_timeout_s + 5.0,
        )
        if err is not None:
            return err
        return payload

    def status(self) -> Dict[str, Any]:
        """服务状态（存活信息 / 在飞 worker / 计数器）。"""
        payload, err = self._roundtrip(
            {"cmd": "status"},
            plan_id="",
            timeout_s=self.connect_timeout_s + 5.0,
        )
        if err is not None:
            return err
        return payload

    @staticmethod
    def raise_for_status(response: Any) -> Dict[str, Any]:
        """见模块级 :func:`raise_for_status`（方法形态）。"""
        return raise_for_status(response)

    # ------------------------------------------------------------ 传输
    def _read_timeout_s(self, timeout_ms: Any) -> float:
        """读超时 = 请求 deadline + 启动宽限余量。"""
        try:
            deadline = float(timeout_ms) / 1000.0
        except (TypeError, ValueError):
            deadline = float(self.timeout_ms) / 1000.0
        return deadline + self.read_grace_s

    def _roundtrip(
        self,
        payload: Dict[str, Any],
        plan_id: str = "",
        timeout_s: Optional[float] = None,
    ) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
        """发一行 JSON → 读一行 JSON。

        :return: ``(数据, None)`` 或 ``(None, 结构化错误)``
        """
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            try:
                sock.settimeout(self.connect_timeout_s)
                sock.connect(self.socket_path)
            except socket.timeout:
                return None, _error_response(
                    plan_id,
                    "FAILED",
                    "连接规划服务超时（%.1f s）: %s"
                    % (self.connect_timeout_s, self.socket_path),
                    next_action="确认 rm_planner 已启动："
                                "rm-planner --ws %s --socket %s"
                                % (self.ws, self.socket_path),
                    error="connect_timeout",
                )
            except OSError as exc:
                return None, _error_response(
                    plan_id,
                    "FAILED",
                    "连不上规划服务 socket %s（%s）: %s"
                    % (self.socket_path, type(exc).__name__, exc),
                    next_action="先启动规划服务："
                                "rm-planner --ws %s --socket %s"
                                % (self.ws, self.socket_path),
                    error="connect_failed",
                )
            try:
                sock.settimeout(timeout_s if timeout_s is not None else
                                self.connect_timeout_s + 5.0)
                sock.sendall(
                    (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
                )
                line = _read_line(sock)
            except socket.timeout:
                return None, _error_response(
                    plan_id,
                    "TIMEOUT",
                    "等待规划服务响应超时（%.1f s）: %s"
                    % (timeout_s or 0.0, self.socket_path),
                    next_action="放宽 timeout_ms，或用 "
                                "{\"cmd\": \"status\"} 查看服务是否还在跑",
                    error="read_timeout",
                )
            except OSError as exc:
                return None, _error_response(
                    plan_id,
                    "FAILED",
                    "与规划服务的连接中断（%s）: %s"
                    % (type(exc).__name__, exc),
                    next_action="查看服务日志；服务进程可能已崩溃，"
                                "重启：rm-planner --ws %s" % self.ws,
                    error="connection_lost",
                )
        finally:
            try:
                sock.close()
            except OSError:  # pragma: no cover
                pass

        if not line:
            return None, _error_response(
                plan_id,
                "FAILED",
                "规划服务没有返回数据（连接已关闭）: %s" % self.socket_path,
                next_action="查看服务日志；服务进程可能已崩溃，"
                            "重启：rm-planner --ws %s" % self.ws,
                error="empty_response",
            )
        try:
            data = json.loads(line)
        except Exception as exc:  # noqa: BLE001 - 非法 JSON → 结构化错误
            return None, _error_response(
                plan_id,
                "FAILED",
                "响应不是合法 JSON（%s）: %s" % (type(exc).__name__, exc),
                next_action="确认 socket 上跑的是 rm_planner 服务",
                error="malformed_response",
            )
        if not isinstance(data, dict):
            return None, _error_response(
                plan_id,
                "FAILED",
                "响应不是 JSON 对象（%s）" % type(data).__name__,
                next_action="确认 socket 上跑的是 rm_planner 服务",
                error="malformed_response",
            )
        return data, None


def _read_line(sock: socket.socket) -> str:
    """读一行（到 ``\\n`` / EOF 为止）。"""
    buf = b""
    while b"\n" not in buf:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            raise
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
        if len(buf) > 1024 * 1024:  # 防呆：响应不该这么大
            break
    return buf.decode("utf-8", errors="replace").strip()
