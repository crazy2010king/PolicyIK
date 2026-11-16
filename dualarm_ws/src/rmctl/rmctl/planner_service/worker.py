"""rm_planner 的 **worker 子进程**（每请求独占一个进程）。

为什么必须是子进程（两条硬约束，都来自厂商内核的现实）：

1. **确定性种子是进程级的**：OMPL 的 RNG 是全局量，且一旦开始采样就**不能
   再改种子**（厂商会打印 ``"Random number generation already started.
   Changing seed now will not lead to deterministic sampling."``）。因此
   「同请求同种子逐点一致」要求**一个进程只服务一个种子** → 每请求一个
   worker 子进程。
2. **进程内无法中断**：厂商的 ``TaskComposerFuture`` 只有
   ready/valid/wait/waitFor/context，**没有 cancel**（且 ``waitFor(float)``
   的绑定本身抛 TypeError）。进程内启动的 taskflow 一旦开跑就无法从 Python
   打断 → 「服务端**强制**硬超时」只能靠 **deadline 到点 SIGKILL worker**。

启动顺序**不可颠倒**（本模块的纪律）::

    RNG_setSeed(seed)   # 进程生存期内第一次设定，必须在任何规划之前
      → 加载 Model
      → 进入请求循环

与父进程的协议（**一行 JSON = 一行 JSON**，以 ``\\n`` 分隔）：

* 请求（父 → 子）：``{"cmd": "plan", "request": {...}}`` / ``{"cmd": "quit"}``
* 响应（子 → 父）：

  * ``{"kind": "heartbeat", "stage": "ready|plan|verify|done", ...}``
    —— **阶段心跳**，供服务端在超时 / 取消响应里说明「在哪个阶段被中断」；
  * ``{"kind": "response", "response": {...}}``
    —— 契约层的 :class:`~rmctl.planner_service.contract.PlanResponse`。

纪律：

* worker **自身不静默退出**：请求非法 / 规划失败都回一行结构化响应，
  除非父进程要求退出（``{"cmd": "quit"}`` 或 stdin EOF）；
* stdout **只**写协议行；厂商内核的日志走 stderr（父进程单独收集，
  绝不与协议行混流）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, Optional

__all__ = [
    "KIND_HEARTBEAT",
    "KIND_RESPONSE",
    "STAGE_DONE",
    "STAGE_PLAN",
    "STAGE_READY",
    "STAGE_VERIFY",
    "emit_line",
    "main",
]

#: 行协议里的行类型标记。
KIND_HEARTBEAT = "heartbeat"
KIND_RESPONSE = "response"

#: 阶段名（心跳）。
STAGE_READY = "ready"
STAGE_PLAN = "plan"
STAGE_VERIFY = "verify"
STAGE_DONE = "done"


def emit_line(stream, payload: Dict[str, Any]) -> None:
    """写一行 JSON（**单行内不含裸换行**）并立刻 flush。"""
    stream.write(json.dumps(payload, ensure_ascii=False) + "\n")
    stream.flush()


def _heartbeat(stage: str, plan_id: str = "", **extra: Any) -> Dict[str, Any]:
    """构造一条心跳行。"""
    payload = {
        "kind": KIND_HEARTBEAT,
        "stage": stage,
        "plan_id": plan_id,
        "pid": os.getpid(),
    }
    payload.update(extra)
    return payload


def _error_response(plan_id: str, reason: str,
                    next_action: str = "") -> Dict[str, Any]:
    """构造一条 ``INVALID_REQUEST`` 响应行（非法输入不抛裸异常）。"""
    from rmctl.planner_service.contract import PlanResponse

    metrics: Dict[str, Any] = {}
    if next_action:
        metrics["next_action"] = next_action
    return {
        "kind": KIND_RESPONSE,
        "response": PlanResponse(
            plan_id=plan_id,
            status="INVALID_REQUEST",
            failure_reason=reason,
            metrics=metrics,
        ).to_dict(),
    }


def _failed_response(plan_id: str, reason: str,
                     **metrics: Any) -> Dict[str, Any]:
    """构造一条 ``FAILED`` 响应行（内部异常 → 结构化失败，不静默退出）。"""
    from rmctl.planner_service.contract import PlanResponse

    return {
        "kind": KIND_RESPONSE,
        "response": PlanResponse(
            plan_id=plan_id,
            status="FAILED",
            failure_reason=reason,
            metrics=dict(metrics),
        ).to_dict(),
    }


def _build_request(data: Any, ws: str):
    """构造契约层请求；非法即抛 ``InvalidRequest``（逐字段指名）。"""
    from rmctl.planner_service.contract import PlanRequest

    return PlanRequest.from_dict(data, ws=ws)


def run_worker(ws: str, seed: int, stdin=None, stdout=None) -> int:
    """worker 主循环（**测试可直接调用**）。

    :param ws: 工作区根（``workcell/planner/*.yaml`` 与 ``build/model`` 所在）
    :param seed: 进程级随机种子（**在任何规划之前**设定）
    :param stdin: 请求输入流（默认 ``sys.stdin``）
    :param stdout: 协议输出流（默认 ``sys.stdout``）
    :return: 进程退出码（0 = 正常退出）
    """
    stdin = stdin if stdin is not None else sys.stdin
    stdout = stdout if stdout is not None else sys.stdout

    # ---- 启动顺序（不可颠倒）：种子 → 模型 → 请求循环 ----
    from rmctl.planner_service import engine

    engine.apply_seed(int(seed))

    from rmctl.model import Model

    try:
        model = Model(ws=ws)
    except Exception as exc:  # noqa: BLE001 - 模型加载失败 → 结构化失败
        emit_line(stdout, _failed_response(
            "", "模型加载失败（%s）: %s" % (type(exc).__name__, exc),
            stage="startup", seed=int(seed),
        ))
        return 4

    emit_line(stdout, _heartbeat(
        STAGE_READY, seed=int(seed),
        joints=list(model.joint_names),
    ))

    while True:
        line = stdin.readline()
        if not line:
            # 父进程关闭 stdin → 正常收工。
            break
        text = line.strip()
        if not text:
            continue
        try:
            payload = json.loads(text)
        except Exception as exc:  # noqa: BLE001 - 非法 JSON → 结构化错误
            emit_line(stdout, _error_response(
                "", "请求不是合法 JSON（%s）: %s" % (type(exc).__name__, exc),
                next_action='请求格式（一行 JSON）：{"cmd": "plan", "request": {...}}',
            ))
            continue
        if not isinstance(payload, dict):
            emit_line(stdout, _error_response(
                "", "请求不是 JSON 对象（%s）" % type(payload).__name__,
                next_action='请求格式（一行 JSON 对象）：{"cmd": "plan", ...}',
            ))
            continue

        cmd = str(payload.get("cmd") or "").strip().lower()
        if cmd in ("quit", "exit", "shutdown"):
            break
        if cmd != "plan":
            emit_line(stdout, _error_response(
                "", "未知命令 %r（worker 只接受 plan / quit）" % cmd,
                next_action='只支持 {"cmd": "plan", "request": {...}}',
            ))
            continue

        data = payload.get("request")
        plan_id = ""
        if isinstance(data, dict):
            plan_id = str(data.get("plan_id") or "")
        try:
            request = _build_request(data, ws)
        except Exception as exc:  # noqa: BLE001 - 契约层非法 → INVALID_REQUEST
            emit_line(stdout, _error_response(
                plan_id, "请求非法: %s" % exc,
                next_action="按契约修正字段后重试（原因里指名了字段）",
            ))
            continue

        plan_id = request.plan_id

        def _on_stage(stage: str, _pid=plan_id) -> None:
            emit_line(stdout, _heartbeat(stage, _pid))

        try:
            response = engine.plan(
                request,
                ws=ws,
                model=model,
                on_stage=_on_stage,
                seed_applied=True,
            )
        except Exception as exc:  # noqa: BLE001 - 内部异常 → FAILED 响应
            from rmctl.planner_service.contract import InvalidRequest

            if isinstance(exc, InvalidRequest):
                emit_line(stdout, _error_response(
                    plan_id, "请求非法: %s" % exc,
                    next_action="按契约修正字段后重试（原因里指名了字段）",
                ))
            else:
                emit_line(stdout, _failed_response(
                    plan_id,
                    "规划异常（%s）: %s" % (type(exc).__name__, exc),
                    stage="plan", seed=int(request.seed),
                ))
            continue

        payload_out = {
            "kind": KIND_RESPONSE,
            "response": response.to_dict(),
        }
        emit_line(stdout, payload_out)
    return 0


def main(argv: Optional[list] = None) -> int:
    """命令行入口（``python3 -m rmctl.planner_service.worker``）。"""
    parser = argparse.ArgumentParser(
        prog="rmctl.planner_service.worker",
        description="rm_planner 的 worker 子进程（每请求独占进程）",
    )
    parser.add_argument("--ws", required=True, help="工作区根")
    parser.add_argument("--seed", type=int, required=True, help="进程级随机种子")
    args = parser.parse_args(argv)

    try:
        return run_worker(args.ws, int(args.seed))
    except Exception as exc:  # noqa: BLE001 - 启动失败 → 明确退出码 + stderr
        sys.stderr.write(
            "worker 启动失败（%s）: %s\n" % (type(exc).__name__, exc)
        )
        sys.stderr.flush()
        return 3


if __name__ == "__main__":  # pragma: no cover - 进程入口
    raise SystemExit(main())
