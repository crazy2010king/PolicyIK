"""rm_runtime —— 真机窗口的进程入口（进程角色名 ``rm_runtime``）。

用法（真机窗口里跑的就是这一条）::

    python3 -m rm_executor.runtime --arm left --traj <traj.json> --report <report.json>

与 :mod:`rm_executor.cli` 的关系：

* **共用同一个** :class:`rm_executor.executor.Executor`、**同一份** 报告 schema
  （:mod:`rm_executor.report`）——离线代理与真机结论**字段完全一致**；
* 本入口**强制 live 模式**（没有 ``--mode``），并按 ``rm_hw_bridge.connect``
  的接口建链；
* **诚实降级**：控制器不可达时返回 ``status="error"`` + reason
  「控制器不可达」+ 可粘贴 next_action，**不伪造任何快照数值**
  （沿用 S02 的诚实分层口径）；
* **不接受** ``--allow-unverified``（真机路径只跑带 ``verify`` 报告的轨迹）。

安全闸门（S04/T7）
------------------

进程启动时**先**建安全闸门（Unix domain socket，默认
``build/run/rm_runtime.sock``，可用 ``--gate-socket`` 覆盖），**之后**才允许
跑轨迹：

* 闸门创建失败 → **诚实报错**（退出码 3 + reason + 可粘贴 next_action），
  **不假装成功**、**不带闸门开跑**（没有外部停机通道就不动机械臂）；
* 闸门收到停机请求 → 执行器**立即**执行 L4 急停（``transport.stop()``）+
  置停止标志（tick 下一拍退出）+ 记 ``safety_l4_emergency_stop`` 事件；
* 进程退出时关闭闸门并清理 socket 文件。

方向：闸门是**外部进程 → rm_runtime** 的单向通道（``rmctl safety stop`` 是
客户端）；rm_runtime **不反向依赖** rmctl。

退出码：``0`` ok / ``3`` blocked / ``4`` error（与 ``rm_hw_bridge.errors`` 同口径）。

零 rmctl 依赖、零 tesseract 依赖。
"""
from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from rm_executor import cli as exec_cli
from rm_executor import contract
from rm_executor import report as report_mod

__all__ = ["build_parser", "main"]


def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器（**无 --mode**：本入口强制真机路径**）。"""
    parser = argparse.ArgumentParser(
        prog="rm_executor.runtime",
        description="rm_runtime：10 ms 单时钟执行器的真机窗口入口（强制 live）",
    )
    parser.add_argument("--traj", required=True, help="轨迹 JSON 路径（须带 verify）")
    parser.add_argument("--report", default=None, help="统一报告 JSON 落盘路径")
    parser.add_argument("--arm", choices=["left", "right"], default="left",
                        help="选哪条臂")
    parser.add_argument("--ip", default=None, help="控制器 IP")
    parser.add_argument("--port", type=int, default=None, help="控制器 TCP 端口")
    parser.add_argument("--udp-port", dest="udp_port", type=int, default=None,
                        help="控制器 UDP 主动上报端口")
    parser.add_argument("--duration", type=float, default=None,
                        help="等效运行时长（秒），默认取轨迹时长")
    parser.add_argument("--realtime", action="store_true",
                        help="用真实墙钟（真机路径默认即为真实墙钟）")
    parser.add_argument("--ws", default=None,
                        help="工作区根（默认自动发现或 $RM_WS）")
    parser.add_argument("--gate-socket", dest="gate_socket", default=None,
                        help="安全闸门 Unix socket 路径"
                             "（默认 <工作区根>/build/run/rm_runtime.sock）")
    return parser


def _start_gate(path: str):
    """建安全闸门（返回 ``(server, error_dict)``）。

    失败 → 返回 ``(None, {reason, next_action})``（**诚实报错**，不假装成功）。
    """
    try:
        from rm_safety.gate import GateError, GateServer
    except Exception as exc:  # noqa: BLE001 - 安全层不可用 → blocked
        return None, {
            "reason": "安全层 rm_safety 不可用（无法建安全闸门）: %s: %s"
                      % (type(exc).__name__, exc),
            "next_action": ("export PYTHONPATH=$PWD/dualarm_ws/src/rm_safety:"
                            "$PYTHONPATH  # 让进程看到安全层"),
        }
    server = GateServer(socket_path=path)
    try:
        server.start()
    except GateError as exc:
        return None, {"reason": exc.reason, "next_action": exc.next_action}
    except Exception as exc:  # noqa: BLE001 - 兜底：任何异常都诚实报错
        return None, {
            "reason": "安全闸门启动失败（%s）: %s" % (type(exc).__name__, exc),
            "next_action": "用 --gate-socket <其它可写路径> 重试",
        }
    return server, {}


def main(argv: Optional[List[str]] = None) -> int:
    """主入口；返回进程退出码（0 / 3 / 4）。"""
    args = build_parser().parse_args(argv)

    # -- 安全闸门：先建闸门，再谈运动（没有外部停机通道就不开跑）--------
    try:
        from rm_safety.gate import default_socket_path
        socket_path = args.gate_socket or default_socket_path(args.ws)
    except Exception:  # noqa: BLE001 - 路径解析失败 → 后面 _start_gate 会报错
        socket_path = args.gate_socket or ""
    server, error = _start_gate(socket_path)
    if server is None:
        print("安全闸门启动失败: %s" % error["reason"], file=sys.stderr)
        if error.get("next_action"):
            print("next_action : %s" % error["next_action"], file=sys.stderr)
        return contract.EXIT_CODES.get(contract.STATUS_BLOCKED, 3)
    print("gate socket : %s" % server.socket_path)

    try:
        rep = exec_cli.execute(mode="live",
                               trajectory_path=args.traj,
                               allow_unverified=False,
                               duration_s=args.duration,
                               realtime=True,
                               arm=args.arm,
                               ip=args.ip,
                               port=args.port,
                               udp_port=args.udp_port,
                               ws=args.ws,
                               gate=server.gate)
    finally:
        try:
            server.stop()
        except Exception:  # noqa: BLE001 - 闸门清理失败不影响报告
            pass

    rep.setdefault("evidence", []).append(
        "safety gate: %s（已关闭）" % server.socket_path)
    rc = contract.EXIT_CODES.get(rep.get("status"), 4)
    if args.report:
        try:
            path = report_mod.dump_report(rep, args.report)
            print("report      : %s" % path)
        except OSError as exc:
            print("报告落盘失败: %s" % exc, file=sys.stderr)
            rc = contract.EXIT_CODES[contract.STATUS_ERROR]
    else:
        import json
        print(json.dumps(rep, ensure_ascii=False, indent=2))
    exec_cli._print_summary(rep)
    return rc


if __name__ == "__main__":
    sys.exit(main())
