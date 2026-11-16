"""rm_planner 规划服务（**服务端**）：Unix socket + 每请求独占 worker + 硬超时。

进程模型（D004 三进程模型之一）::

    rmctl（薄客户端）──Unix socket──▶ rm_planner（本模块）
                                        │  spawn
                                        ▼
                                    worker 子进程（每请求独占）
                                        │  一行 JSON 请求 / 一行 JSON 响应
                                        ▼
                                    厂商 Task Composer（不可中断）

为什么必须用**子进程**而不是线程：厂商的 ``TaskComposerFuture`` **没有
cancel**（只有 ready/valid/wait/waitFor/context，且 ``waitFor(float)`` 的绑定
本身抛 TypeError），进程内启动的 taskflow 一旦开跑就无法从 Python 打断。
因此「服务端**强制**硬超时」只能靠 **deadline 到点 SIGKILL worker**；同一机制
顺带满足「可中断」（cancel）与「崩溃可替换」（worker 死、服务不死）。

协议（一行 JSON 请求 → 一行 JSON 响应，沿用 ``rm_safety.gate`` 的风格）：

* ``{"cmd": "plan", "request": {...}}`` —— 规划（响应 = 契约层 PlanResponse）
* ``{"cmd": "cancel", "plan_id": "..."}`` —— 取消正在执行的 plan
* ``{"cmd": "status"}`` —— 服务状态（活跃 worker / 计数器）

非法 JSON / 未知 cmd / 缺字段 → 回一行 ``status="INVALID_REQUEST"`` 的响应
（带 reason + 可粘贴 next_action），**不静默忽略**；请求体有字节上限（防呆）。

超时语义（**两个 deadline**，都要诚实）：

* **启动宽限**（``startup_timeout_s``，默认 30 s）：worker 从 spawn 到发出
  ``ready`` 心跳（模型加载完成）的时间。超时 → ``TIMEOUT``（阶段
  ``startup``）。
* **规划 deadline**（请求的 ``timeout_ms``）：从 ``ready`` 心跳起算（= 规划
  真正开始的时间）。到点 → **SIGKILL worker** + ``waitpid`` 回收 → 回
  ``TIMEOUT``，失败原因写明 deadline 与**被中断阶段**（取 worker 最后一条
  心跳）。

纪律：

* worker 崩溃 / 超时 / 被取消 → **服务进程本身继续存活**（下一条请求照常）；
* 每条 plan 请求 **spawn 一个专属 worker**（进程级种子 → 确定性）；
* 所有 worker 都被 ``waitpid`` 回收（不留僵尸），计数可在 ``status`` 里查。
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from queue import Empty, Queue
from typing import Any, Dict, List, Optional

from rmctl import paths
from rmctl.planner_service.contract import InvalidRequest, PlanRequest, PlanResponse

__all__ = [
    "DEFAULT_SOCKET_RELPATH",
    "MAX_REQUEST_BYTES",
    "PlannerServer",
    "PlannerServerError",
    "default_socket_path",
    "main",
]

#: 默认 socket 路径（相对工作区根，与 ``rm_safety.gate`` 同风格）。
DEFAULT_SOCKET_RELPATH = os.path.join("build", "run", "rm_planner.sock")

#: 请求体字节上限（防呆；超长 → INVALID_REQUEST）。
MAX_REQUEST_BYTES = 256 * 1024

#: ``sockaddr_un.sun_path`` 长度上限（Linux = 108 字节，含结尾 NUL）。
_SOCKET_PATH_MAX_BYTES = 107

#: 客户端连接读超时（秒）。
_CLIENT_TIMEOUT_S = 30.0

#: accept 超时（秒，用于检查停止标志）。
_ACCEPT_TIMEOUT_S = 0.5

#: worker 启动宽限（秒）：spawn → ready 心跳。
DEFAULT_STARTUP_TIMEOUT_S = 30.0

#: worker stderr 保留字节数（环形，内存有界）。
_MAX_STDERR_BYTES = 8192

#: worker stdout 结束哨兵（**模块级唯一对象**：读写两侧必须能认同一个）。
_EOF = object()


class PlannerServerError(RuntimeError):
    """服务端启动失败（**带可粘贴 next_action**，不假装成功）。"""

    def __init__(self, reason: str, next_action: str = "") -> None:
        self.reason = reason
        self.next_action = next_action
        super().__init__(reason)


def default_socket_path(ws: Optional[str] = None) -> str:
    """默认 socket 路径：``<ws>/build/run/rm_planner.sock``。"""
    return os.path.join(ws or paths.workspace_root(), DEFAULT_SOCKET_RELPATH)


def _pkg_root() -> str:
    """``rmctl`` 包所在的 ``src`` 目录（worker 子进程的 PYTHONPATH 用）。"""
    here = os.path.dirname(os.path.abspath(__file__))      # rmctl/planner_service
    pkg = os.path.dirname(here)                            # rmctl
    return os.path.dirname(pkg)                            # src（rmctl 的父目录）


class _Worker:
    """一次 plan 请求的 worker 句柄（服务端内部状态）。"""

    def __init__(self, plan_id: str, proc: subprocess.Popen) -> None:
        self.plan_id = plan_id
        self.proc = proc
        self.pid = int(proc.pid)
        self.started_at = time.time()
        self.ready_at: Optional[float] = None
        self.stage = "startup"
        self.cancelled = False
        self.killed = False
        self.reaped = False
        self.exit_code: Optional[int] = None
        self.stderr_tail = ""

    # -- 生命周期 ------------------------------------------------------
    def kill(self) -> None:
        """SIGKILL worker（**硬中断**，不留残留进程）。"""
        if self.proc.poll() is not None:
            return
        self.killed = True
        try:
            os.kill(self.pid, signal.SIGKILL)
        except OSError:
            pass

    def reap(self, timeout: float = 5.0) -> Optional[int]:
        """``waitpid`` 回收（避免僵尸）；返回退出码。"""
        if self.reaped:
            return self.exit_code
        try:
            self.exit_code = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.kill()
            try:
                self.exit_code = self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:  # pragma: no cover - 极端情况
                self.exit_code = None
        self.reaped = True
        return self.exit_code

    def alive(self) -> bool:
        """worker 是否还活着。"""
        return self.proc.poll() is None


class PlannerServer:
    """rm_planner 规划服务（Unix socket 服务端）。

    :param socket_path: socket 路径；``None`` → ``<ws>/build/run/rm_planner.sock``
    :param ws: 工作区根（规划配置 / 模型 / worker 的工作目录）
    :param worker_pool_size: 同时允许的**在飞 worker** 数（保护：避免无限
        spawn 打爆机器；超出即拒绝新请求并说明）
    :param startup_timeout_s: worker 启动宽限（秒）
    """

    def __init__(
        self,
        socket_path: Optional[str] = None,
        ws: Optional[str] = None,
        worker_pool_size: int = 1,
        startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S,
        backlog: int = 8,
    ) -> None:
        self.ws = os.path.abspath(ws or paths.workspace_root())
        self.socket_path = os.path.abspath(
            socket_path or default_socket_path(self.ws)
        )
        self.worker_pool_size = max(1, int(worker_pool_size))
        self.startup_timeout_s = float(startup_timeout_s)
        self._backlog = max(1, int(backlog))
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self._active: Dict[str, _Worker] = {}
        self._counters: Dict[str, int] = {
            "requests_total": 0,
            "success": 0,
            "failed": 0,
            "timeout": 0,
            "cancelled": 0,
            "invalid": 0,
            "workers_spawned": 0,
            "workers_reaped": 0,
            "worker_crashes": 0,
        }
        self._errors: List[str] = []

    # ------------------------------------------------------------ 属性
    @property
    def running(self) -> bool:
        """服务是否在跑。"""
        return self._running

    @property
    def counters(self) -> Dict[str, int]:
        """计数器副本（含 worker 计数，用于断言「无残留 worker」）。"""
        with self._lock:
            return dict(self._counters)

    def worker_pids(self) -> List[int]:
        """当前**在飞** worker 的 pid 列表。"""
        with self._lock:
            return [w.pid for w in self._active.values()]

    def active_workers(self) -> List[Dict[str, Any]]:
        """在飞 worker 的快照（plan_id / pid / 阶段 / 已耗时）。"""
        with self._lock:
            now = time.time()
            return [
                {
                    "plan_id": w.plan_id,
                    "pid": w.pid,
                    "stage": w.stage,
                    "elapsed_s": round(now - w.started_at, 3),
                    "cancelled": bool(w.cancelled),
                }
                for w in self._active.values()
            ]

    def server_errors(self) -> List[str]:
        """服务端异常记录（留痕，不静默）。"""
        with self._lock:
            return list(self._errors)

    def _record_error(self, message: str) -> None:
        with self._lock:
            self._errors.append(message)
            if len(self._errors) > 200:
                del self._errors[0]

    def _bump(self, key: str, delta: int = 1) -> None:
        with self._lock:
            self._counters[key] = self._counters.get(key, 0) + delta

    # ------------------------------------------------------------ 生命周期
    def start(self) -> Dict[str, Any]:
        """启动 socket 服务端。

        失败（路径过长 / 目录不可写 / 路径被占用 / bind 失败）→ 抛
        :class:`PlannerServerError`（带可粘贴 ``next_action``），**不假装成功**。
        """
        if self._running:
            return {"status": "ok", "socket_path": self.socket_path,
                    "already_running": True}
        path = self.socket_path
        if len(path.encode("utf-8")) > _SOCKET_PATH_MAX_BYTES:
            raise PlannerServerError(
                "规划服务 socket 路径过长（%d 字节 > %d）: %s"
                % (len(path.encode("utf-8")), _SOCKET_PATH_MAX_BYTES, path),
                "用 --socket <更短的路径> 重启 rm-planner"
                "（例如 --socket /tmp/rm_planner.sock）")
        parent = os.path.dirname(path) or "."
        try:
            os.makedirs(parent, exist_ok=True)
        except OSError as exc:
            raise PlannerServerError(
                "无法创建规划服务 socket 目录 %s（%s）: %s"
                % (parent, type(exc).__name__, exc),
                "确认目录可写（mkdir -p %s），或用 --socket <可写路径> 重启" % parent
            ) from exc
        if os.path.exists(path):
            if _socket_alive(path):
                raise PlannerServerError(
                    "规划服务 socket 已被占用：%s（已有进程在监听）" % path,
                    "停掉占用该路径的 rm-planner，或用 --socket <其它路径> 重启")
            try:
                os.unlink(path)
            except OSError as exc:
                raise PlannerServerError(
                    "无法清理陈旧的规划服务 socket %s（%s）: %s"
                    % (path, type(exc).__name__, exc),
                    "手动删除该文件后重试：rm -f %s" % path) from exc
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.bind(path)
            sock.listen(self._backlog)
            sock.settimeout(_ACCEPT_TIMEOUT_S)
        except OSError as exc:
            sock.close()
            raise PlannerServerError(
                "创建规划服务 socket 失败（%s）: %s" % (path, exc),
                "确认路径可写且未被占用，或用 --socket <其它路径> 重试") from exc
        self._sock = sock
        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._serve_loop,
                                        name="rm-planner-server", daemon=True)
        self._thread.start()
        return {"status": "ok", "socket_path": path, "ws": self.ws}

    def stop(self) -> Dict[str, Any]:
        """停止服务端并清理 socket 文件（幂等）；在飞 worker 一并回收。"""
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
            except OSError:  # pragma: no cover
                pass
        with self._lock:
            workers = list(self._active.values())
        for worker in workers:
            worker.kill()
            worker.reap()
        with self._lock:
            self._active.clear()
        self._running = False
        if os.path.exists(self.socket_path):
            try:
                os.unlink(self.socket_path)
            except OSError:  # pragma: no cover
                pass
        return {"status": "ok", "socket_path": self.socket_path, "stopped": True}

    # ------------------------------------------------------------ 服务循环
    def _serve_loop(self) -> None:
        """accept 循环（独立线程；单连接异常不影响服务）。"""
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
                self._record_error("accept 失败: %s" % exc)
                continue
            thread = threading.Thread(target=self._handle_conn, args=(conn,),
                                      name="rm-planner-conn", daemon=True)
            thread.start()

    def _handle_conn(self, conn: socket.socket) -> None:
        """读一行请求 → 处理 → 回一行 JSON 响应。"""
        try:
            conn.settimeout(_CLIENT_TIMEOUT_S)
        except OSError:  # pragma: no cover
            pass
        try:
            raw, too_long = self._read_line(conn)
            response = self.handle_request(raw, too_long=too_long)
            payload = (json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8")
            try:
                conn.sendall(payload)
            except OSError:
                pass  # 客户端已断开：不算服务端错误
        except Exception as exc:  # noqa: BLE001 - 单连接异常不影响服务
            self._record_error(
                "连接处理异常（%s）: %s" % (type(exc).__name__, exc))
            try:
                fallback = json.dumps(
                    _invalid_response(
                        "",
                        "服务端处理请求时异常（%s）: %s" % (type(exc).__name__, exc),
                    ),
                    ensure_ascii=False,
                ) + "\n"
                conn.sendall(fallback.encode("utf-8"))
            except OSError:
                pass
        finally:
            try:
                conn.close()
            except OSError:  # pragma: no cover
                pass

    @staticmethod
    def _read_line(conn: socket.socket):
        """读一行（到 ``\\n`` / EOF / 超时 / 超长为止）。"""
        buf = b""
        too_long = False
        while b"\n" not in buf:
            if len(buf) > MAX_REQUEST_BYTES:
                too_long = True
                break
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                break
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
        return buf.decode("utf-8", errors="replace").strip(), too_long

    # ------------------------------------------------------------ 请求分发
    def handle_request(self, raw: Any, too_long: bool = False) -> Dict[str, Any]:
        """处理一行请求文本 → 响应字典（**纯逻辑**，便于离线逐条断言）。"""
        self._bump("requests_total")
        if too_long:
            self._bump("invalid")
            return _invalid_response(
                "",
                "请求体超过上限（%d 字节）" % MAX_REQUEST_BYTES,
                next_action="精简请求体后重试",
            )
        text = raw if isinstance(raw, str) else str(raw or "")
        text = text.strip()
        if not text:
            self._bump("invalid")
            return _invalid_response(
                "",
                "请求为空（期望一行 JSON）",
                next_action='请求格式：{"cmd": "plan", "request": {...}} / '
                           '{"cmd": "cancel", "plan_id": "..."} / {"cmd": "status"}',
            )
        try:
            payload = json.loads(text)
        except Exception as exc:  # noqa: BLE001
            self._bump("invalid")
            return _invalid_response(
                "",
                "请求不是合法 JSON（%s）: %s" % (type(exc).__name__, exc),
                next_action='请求格式（一行 JSON）：{"cmd": "plan", "request": {...}}',
            )
        if not isinstance(payload, dict):
            self._bump("invalid")
            return _invalid_response(
                "",
                "请求不是 JSON 对象（当前 %s）" % type(payload).__name__,
                next_action='请求格式（一行 JSON 对象）：{"cmd": "plan", ...}',
            )
        cmd = str(payload.get("cmd") or "").strip().lower()
        if cmd == "status":
            return self.status()
        if cmd == "cancel":
            return self.handle_cancel(payload)
        if cmd == "plan":
            return self.handle_plan(payload)
        self._bump("invalid")
        return _invalid_response(
            "",
            "未知命令 %r（只接受 plan / cancel / status）" % cmd,
            next_action='只支持 {"cmd": "plan", "request": {...}}、'
                       '{"cmd": "cancel", "plan_id": "..."} 与 {"cmd": "status"}',
        )

    # ------------------------------------------------------------ status
    def status(self) -> Dict[str, Any]:
        """服务状态（活跃 worker / 计数器 / 最近错误）。"""
        with self._lock:
            counters = dict(self._counters)
            errors = list(self._errors[-10:])
        return {
            "cmd": "status",
            "status": "ok",
            "running": bool(self._running),
            "socket_path": self.socket_path,
            "ws": self.ws,
            "worker_pool_size": self.worker_pool_size,
            "active_workers": self.active_workers(),
            "counters": counters,
            "errors": errors,
        }

    # ------------------------------------------------------------ cancel
    def handle_cancel(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """取消正在执行的 plan（**SIGKILL 该 plan 的 worker**）。"""
        plan_id = str(payload.get("plan_id") or "").strip()
        if not plan_id:
            self._bump("invalid")
            return _invalid_response(
                "",
                "cancel 缺少 plan_id",
                next_action='{"cmd": "cancel", "plan_id": "<正在执行的 plan_id>"}',
            )
        with self._lock:
            worker = self._active.get(plan_id)
        if worker is None:
            return {
                "cmd": "cancel",
                "status": "error",
                "plan_id": plan_id,
                "reason": "没有正在执行的 plan_id %r" % plan_id,
                "next_action": "用 {\"cmd\": \"status\"} 查看在飞的 plan_id",
            }
        worker.cancelled = True
        worker.kill()
        return {
            "cmd": "cancel",
            "status": "ok",
            "plan_id": plan_id,
            "worker_pid": worker.pid,
            "reason": "已向 worker 发送 SIGKILL（原请求方将收到 CANCELLED）",
        }

    # ------------------------------------------------------------ plan
    def handle_plan(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """执行一次规划（**spawn 专属 worker + deadline 强制中断**）。"""
        data = payload.get("request")
        if not isinstance(data, dict):
            self._bump("invalid")
            return _invalid_response(
                str(data.get("plan_id") or "") if isinstance(data, dict) else "",
                "plan 请求缺少 request 对象",
                next_action='{"cmd": "plan", "request": {…契约字段…}}',
            )
        plan_id = str(data.get("plan_id") or "")
        try:
            request = PlanRequest.from_dict(data, ws=self.ws)
        except InvalidRequest as exc:
            self._bump("invalid")
            return _invalid_response(
                plan_id,
                "请求非法（%s）: %s" % (exc.field, exc.reason),
                next_action="按契约修正字段后重试（原因里指名了字段）",
                field=exc.field,
            )
        plan_id = request.plan_id

        with self._lock:
            if plan_id in self._active:
                self._bump("invalid")
                return _invalid_response(
                    plan_id,
                    "plan_id %r 已在执行中" % plan_id,
                    next_action="换一个 plan_id，或先 cancel 掉在飞的那个",
                )
            if len(self._active) >= self.worker_pool_size:
                self._bump("invalid")
                return _invalid_response(
                    plan_id,
                    "在飞 worker 数已达上限（%d）" % self.worker_pool_size,
                    next_action="稍后重试，或用 {\"cmd\": \"cancel\"} 取消在飞的 plan",
                )

        worker = self._spawn_worker(request)
        with self._lock:
            self._active[plan_id] = worker
        self._bump("workers_spawned")
        try:
            response = self._await_worker(worker, request)
        finally:
            with self._lock:
                self._active.pop(plan_id, None)
            worker.reap()
            self._bump("workers_reaped")
        status = response.get("status")
        if status == "SUCCESS":
            self._bump("success")
        elif status == "TIMEOUT":
            self._bump("timeout")
        elif status == "CANCELLED":
            self._bump("cancelled")
        elif status == "INVALID_REQUEST":
            self._bump("invalid")
        else:
            self._bump("failed")
        return response

    # -- worker 生命周期 ------------------------------------------------
    def _worker_env(self) -> Dict[str, str]:
        """worker 子进程的环境（PYTHONPATH + LD_LIBRARY_PATH 继承）。"""
        env = os.environ.copy()
        pkg_root = _pkg_root()
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = (
            pkg_root + (os.pathsep + existing if existing else "")
        )
        try:
            from rmctl.vendor_env import ensure_libstdcxx

            lib = ensure_libstdcxx()
        except Exception:  # noqa: BLE001 - 预加载失败不阻断（子进程会自举）
            lib = None
        if lib:
            lib_dir = os.path.dirname(os.path.abspath(lib))
            current = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = (
                lib_dir + (os.pathsep + current if current else "")
            )
        env["RM_WS"] = self.ws
        return env

    def _spawn_worker(self, request: PlanRequest) -> _Worker:
        """spawn 一个**专属** worker 子进程（进程级种子 → 确定性）。"""
        cmd = [
            sys.executable,
            "-m",
            "rmctl.planner_service.worker",
            "--ws",
            self.ws,
            "--seed",
            str(int(request.seed)),
        ]
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=self._worker_env(),
            cwd=self.ws if os.path.isdir(self.ws) else None,
            text=True,
            bufsize=1,
        )
        worker = _Worker(request.plan_id, proc)
        self._stderr_reader(worker)
        return worker

    def _stderr_reader(self, worker: _Worker) -> None:
        """后台抽干 worker 的 stderr（防管道写满阻塞；保留尾部供诊断）。"""

        def _run() -> None:
            stream = worker.proc.stderr
            if stream is None:
                return
            try:
                for chunk in iter(lambda: stream.readline(), ""):
                    if not chunk:
                        break
                    worker.stderr_tail = (worker.stderr_tail + chunk)[-_MAX_STDERR_BYTES:]
            except Exception:  # noqa: BLE001 - 读日志失败不影响主流程
                pass

        thread = threading.Thread(target=_run, name="rm-planner-worker-stderr",
                                  daemon=True)
        thread.start()

    # -- 等待 worker ----------------------------------------------------
    def _await_worker(self, worker: _Worker, request: PlanRequest) -> Dict[str, Any]:
        """等待 worker 响应；**deadline 到点 SIGKILL**。"""
        queue: "Queue[str]" = Queue()
        reader = threading.Thread(target=self._stdout_reader,
                                  args=(worker, queue), daemon=True)
        reader.start()

        # 把请求写给 worker（一行 JSON）。
        try:
            worker.proc.stdin.write(
                json.dumps({"cmd": "plan", "request": request.to_dict()},
                           ensure_ascii=False) + "\n"
            )
            worker.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            worker.kill()
            self._bump("worker_crashes")
            return _failed_response(
                request.plan_id,
                "worker 启动后无法接收请求（%s）: %s" % (type(exc).__name__, exc),
                worker_pid=worker.pid,
            )

        timeout_s = float(request.timeout_ms) / 1000.0
        eof = _EOF
        while True:
            if worker.cancelled:
                worker.kill()
                return _cancelled_response(
                    request.plan_id,
                    "规划被取消：服务端已 SIGKILL worker（阶段 %s）" % worker.stage,
                    worker_pid=worker.pid,
                    stage=worker.stage,
                )
            now = time.time()
            if worker.ready_at is None:
                if now - worker.started_at > self.startup_timeout_s:
                    worker.kill()
                    return _timeout_response(
                        request.plan_id,
                        timeout_ms=request.timeout_ms,
                        stage="startup",
                        worker_pid=worker.pid,
                        elapsed_s=now - worker.started_at,
                        reason=("worker 启动超时（%.1f s 内没有发出 ready 心跳）"
                                % self.startup_timeout_s),
                    )
            else:
                if now - worker.ready_at > timeout_s:
                    worker.kill()
                    return _timeout_response(
                        request.plan_id,
                        timeout_ms=request.timeout_ms,
                        stage=worker.stage,
                        worker_pid=worker.pid,
                        elapsed_s=now - worker.ready_at,
                        reason=("规划超时：服务端在 deadline %d ms 处强制中断 worker"
                                % int(request.timeout_ms)),
                    )
            try:
                line = queue.get(timeout=0.05)
            except Empty:
                if not worker.alive() and queue.empty():
                    if worker.cancelled:
                        # 取消优先于崩溃：worker 是**我们**杀的。
                        return _cancelled_response(
                            request.plan_id,
                            "规划被取消：服务端已 SIGKILL worker（阶段 %s）"
                            % worker.stage,
                            worker_pid=worker.pid,
                            stage=worker.stage,
                        )
                    code = worker.reap()
                    self._bump("worker_crashes")
                    return _failed_response(
                        request.plan_id,
                        "worker 异常退出（退出码 %s）: %s"
                        % (_describe_exit(code), _tail(worker.stderr_tail)),
                        worker_pid=worker.pid,
                        exit_code=code,
                        stage=worker.stage,
                    )
                continue
            if line is eof:
                if worker.cancelled:
                    return _cancelled_response(
                        request.plan_id,
                        "规划被取消：服务端已 SIGKILL worker（阶段 %s）"
                        % worker.stage,
                        worker_pid=worker.pid,
                        stage=worker.stage,
                    )
                code = worker.reap()
                self._bump("worker_crashes")
                return _failed_response(
                    request.plan_id,
                    "worker 异常退出（退出码 %s）: %s"
                    % (_describe_exit(code), _tail(worker.stderr_tail)),
                    worker_pid=worker.pid,
                    exit_code=code,
                    stage=worker.stage,
                )
            kind = _classify(line)
            if kind == "heartbeat":
                stage = str(line.get("stage") or "")
                if stage:
                    worker.stage = stage
                if stage == "ready" and worker.ready_at is None:
                    worker.ready_at = time.time()
                continue
            if kind == "response":
                payload = line.get("response")
                if isinstance(payload, dict):
                    return payload
                self._bump("worker_crashes")
                return _failed_response(
                    request.plan_id,
                    "worker 响应不是对象: %r" % (payload,),
                    worker_pid=worker.pid,
                    stage=worker.stage,
                )
            # 其它行 = worker 日志噪声（不参与协议）→ 留痕后忽略。
            self._record_error("worker 输出了非协议行: %s" % _tail(line))

    @staticmethod
    def _stdout_reader(worker: _Worker, queue: "Queue[Any]") -> None:
        """读 worker 的 stdout（逐行；EOF 时放入哨兵）。"""
        stream = worker.proc.stdout
        if stream is None:  # pragma: no cover
            return
        eof = _EOF
        try:
            for raw in stream:
                text = raw.strip()
                if not text:
                    continue
                try:
                    payload = json.loads(text)
                except Exception:  # noqa: BLE001 - 非 JSON 行 = 日志噪声
                    payload = {"kind": "log", "text": text}
                queue.put(payload if isinstance(payload, dict)
                          else {"kind": "log", "text": text})
        except Exception:  # noqa: BLE001 - 读管道失败：交给调用方按退出码判断
            pass
        queue.put(eof)


# --------------------------------------------------------------------------
# 响应构造（结构化，不抛裸异常）
# --------------------------------------------------------------------------

def _invalid_response(plan_id: str, reason: str, next_action: str = "",
                       field: str = "") -> Dict[str, Any]:
    """构造 ``INVALID_REQUEST`` 响应（reason + 可粘贴 next_action）。"""
    metrics: Dict[str, Any] = {}
    if next_action:
        metrics["next_action"] = next_action
    if field:
        metrics["field"] = field
    return PlanResponse(
        plan_id=plan_id,
        status="INVALID_REQUEST",
        failure_reason=reason,
        metrics=metrics,
    ).to_dict()


def _failed_response(plan_id: str, reason: str, **metrics: Any) -> Dict[str, Any]:
    """构造 ``FAILED`` 响应（worker 崩溃等）。"""
    return PlanResponse(
        plan_id=plan_id,
        status="FAILED",
        failure_reason=reason,
        metrics=dict(metrics),
    ).to_dict()


def _timeout_response(plan_id: str, timeout_ms: int, stage: str,
                      worker_pid: int, elapsed_s: float,
                      reason: str = "") -> Dict[str, Any]:
    """构造 ``TIMEOUT`` 响应（**写明 deadline 与被中断阶段**）。"""
    text = reason or (
        "规划超时：服务端在 deadline %d ms 处强制中断 worker" % int(timeout_ms)
    )
    text = "%s（被中断阶段：%s）" % (text, stage)
    return PlanResponse(
        plan_id=plan_id,
        status="TIMEOUT",
        failure_reason=text,
        metrics={
            "timeout_ms": int(timeout_ms),
            "interrupted_stage": stage,
            "worker_pid": int(worker_pid),
            "killed": True,
            "elapsed_s": round(float(elapsed_s), 3),
            "next_action": "放宽 timeout_ms，或改用更容易的场景 / 更短的规划目标",
        },
    ).to_dict()


def _cancelled_response(plan_id: str, reason: str, worker_pid: int,
                        stage: str) -> Dict[str, Any]:
    """构造 ``CANCELLED`` 响应。"""
    return PlanResponse(
        plan_id=plan_id,
        status="CANCELLED",
        failure_reason=reason,
        metrics={
            "interrupted_stage": stage,
            "worker_pid": int(worker_pid),
            "killed": True,
            "next_action": "如需继续，请重新发起 plan 请求",
        },
    ).to_dict()


def _classify(line: Dict[str, Any]) -> str:
    """判定一行是心跳 / 响应 / 日志噪声。"""
    kind = str(line.get("kind") or "").strip().lower()
    if kind in ("heartbeat", "response"):
        return kind
    if "response" in line and isinstance(line.get("response"), dict):
        return "response"
    return "log"


def _describe_exit(code: Optional[int]) -> str:
    """退出码的人类可读描述（负数 = 被信号杀死）。"""
    if code is None:
        return "未知"
    if code < 0:
        try:
            return "%d (被信号 %d 杀死)" % (code, -code)
        except Exception:  # noqa: BLE001 - pragma: no cover
            return str(code)
    return str(code)


def _tail(text: str, limit: int = 400) -> str:
    """日志尾部（单行化，避免响应里出现裸换行）。"""
    if not text:
        return ""
    flat = " ".join(str(text).split())
    return flat[-limit:]


def _socket_alive(path: str) -> bool:
    """socket 路径上是否已有进程在监听。"""
    if not os.path.exists(path):
        return False
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        probe.settimeout(0.5)
        probe.connect(path)
        return True
    except OSError:
        return False
    finally:
        try:
            probe.close()
        except OSError:  # pragma: no cover
            pass


# --------------------------------------------------------------------------
# 命令行入口
# --------------------------------------------------------------------------

def main(argv: Optional[list] = None) -> int:
    """``rm-planner`` 命令行入口。"""
    parser = argparse.ArgumentParser(
        prog="rm-planner",
        description="rm_planner 规划服务（Unix socket + 每请求独占 worker）",
    )
    parser.add_argument("--socket", default=None,
                        help="socket 路径（默认 <ws>/build/run/rm_planner.sock）")
    parser.add_argument("--ws", default=None,
                        help="工作区根（默认自动探测）")
    parser.add_argument("--worker-pool-size", type=int, default=1,
                        help="同时允许的在飞 worker 数（默认 1）")
    args = parser.parse_args(argv)

    server = PlannerServer(
        socket_path=args.socket,
        ws=args.ws,
        worker_pool_size=args.worker_pool_size,
    )
    try:
        info = server.start()
    except PlannerServerError as exc:
        sys.stderr.write("rm_planner 启动失败: %s\n" % exc.reason)
        if exc.next_action:
            sys.stderr.write("下一步: %s\n" % exc.next_action)
        sys.stderr.flush()
        return 3

    print("rm_planner 已启动")
    print("  socket: %s" % info["socket_path"])
    print("  ws:     %s" % info["ws"])
    print("  用法:   python3 -c \"import socket,json;s=socket.socket(socket.AF_UNIX);"
          "s.connect('%s');s.sendall((json.dumps({'cmd':'status'})+'\\n').encode());"
          "print(s.recv(65536).decode())\"" % info["socket_path"])
    sys.stdout.flush()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("rm_planner 退出中…")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":  # pragma: no cover - 进程入口
    raise SystemExit(main())
