#!/usr/bin/env python3
"""RM75-B 真机窗口**只读**门禁（window gate）。

为什么需要它
------------
S10 的默认结局是「真机窗口不可用 → blocked 结案」。但此前没有任何一个工具能
一次性回答三件事：

1. **为什么不能跑**（逐项门禁结果，而不是一句"连不上"）；
2. **已经试过哪些动作**（attempted actions，可审计）；
3. **下一步可粘贴什么命令**（next action）。

本工具就是这三件事的机器可读产出（``rmctl.rm75_window_gate/1``）。

硬纪律
------
* **纯只读**：只做 ICMP ping / TCP 连接探测 / 本地 UDP 端口占用检查 /
  SDK 导入检查 / 探针 ``--precheck-only``。**绝不发出任何运动指令**
  （不调用 ``rm_movej`` / ``rm_movej_canfd`` / 不下发轨迹）。
* **不伪造结论**：任一门禁项不通过 → ``status="blocked"`` + 明确 reason +
  可粘贴下一步；退出码 **唯一来源** = ``rm_executor.contract.EXIT_CODES``
  （ok=0 / blocked=3），**不自造第二套 status 字面量**。
* **诚实话语**：工件里固定带 ``honesty`` 声明——本工件只是只读门禁结果，
  **不得据此声称真机长跑或断流已达标**。

用法
----
::

    python3 scripts/rm75_window_gate.py --ip 192.168.1.18 \
        --out build/reports/validation/window_gate.json

退出码：``0`` 全部门禁通过 / ``3`` blocked（外部条件不具备）。

离线可测：``run_gate`` 的每一项探测都是**可注入的可调用对象**
（ping / tcp / udp_free / sdk / probe），测试注入假实现即可。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

# 与 scripts/rm_regression.py 同一套自举方式：裸环境也能 import 到各层包
for _rel in (
    "dualarm_ws/src/rmctl",
    "dualarm_ws/src/rm_executor",
    "dualarm_ws/src/rm_hw_bridge",
    "dualarm_ws/src/rm_safety",
    "dualarm_ws/src/rm_description/scripts",
):
    _p = os.path.join(REPO_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from rm_executor import contract  # noqa: E402

#: 工件 schema
SCHEMA = "rmctl.rm75_window_gate/1"

#: status / 退出码词表 —— **唯一来源** = rm_executor.contract
STATUS_OK = contract.STATUS_OK
STATUS_BLOCKED = contract.STATUS_BLOCKED
STATUS_ERROR = contract.STATUS_ERROR
EXIT_CODES = contract.EXIT_CODES

DEFAULT_IP = "192.168.1.18"
DEFAULT_PORT = 8080
DEFAULT_UDP_PORTS = (8091, 8092)

#: 固定诚实声明（每个工件必带）
HONESTY = ("本工件只记录只读门禁结果；未执行任何运动指令；"
           "不得据此声称真机长跑或断流已达标。")

#: 门禁项名（顺序固定，便于跨次比较）
CHECK_IP = "ip_reachable"
CHECK_TCP = "tcp_reachable"
CHECK_UDP = "udp_ports_free"
CHECK_SDK = "sdk_loadable"
CHECK_PROBE = "probe_precheck"


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


# ------------------------------------------------------------------ 默认探测
def _default_ping(ip: str, count: int = 3, timeout_s: float = 1.0
                  ) -> Tuple[bool, str]:
    """ICMP ping。返回 ``(ok, detail)``。"""
    if shutil.which("ping") is None:
        return False, "本机没有 ping 命令（无法做 ICMP 门禁）"
    cmd = ["ping", "-c", str(int(count)), "-W", str(int(max(1, timeout_s))), str(ip)]
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=float(count) * max(1.0, timeout_s) + 5.0)
    except subprocess.TimeoutExpired:
        return False, "ping 超时（%s）" % " ".join(cmd)
    except OSError as exc:  # pragma: no cover - 环境异常
        return False, "ping 执行失败: %r" % (exc,)
    out = (proc.stdout or b"").decode("utf-8", "replace") if isinstance(
        proc.stdout, bytes) else (proc.stdout or "")
    tail = " / ".join(line.strip() for line in out.strip().splitlines()[-2:])
    if proc.returncode == 0:
        return True, tail or "ping 通"
    return False, tail or "ping 失败（exit=%s）" % proc.returncode


def _default_tcp(ip: str, port: int, timeout_s: float = 2.0) -> Tuple[bool, str]:
    """TCP 可连接性。返回 ``(ok, detail)``。"""
    try:
        with socket.create_connection((str(ip), int(port)), timeout=float(timeout_s)):
            return True, "TCP %s:%d 可连接" % (ip, int(port))
    except OSError as exc:
        return False, "TCP %s:%d 连接失败: %s" % (ip, int(port), exc)


def _default_udp_free(port: int, host: str = "0.0.0.0") -> Tuple[bool, str]:
    """本地 UDP 端口是否空闲（bind 试探）。返回 ``(ok, detail)``。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host, int(port)))
        return True, "本地 UDP %d 空闲" % int(port)
    except OSError as exc:
        return False, "本地 UDP %d 已被占用: %s" % (int(port), exc)
    finally:
        sock.close()


def _default_sdk() -> Tuple[bool, str]:
    """硬件适配层可加载且关键符号存在。返回 ``(ok, detail)``。"""
    try:
        from rm_hw_bridge import bridge as _bridge
    except Exception as exc:
        return False, ("rm_hw_bridge 不可导入: %r（%s）"
                       % (exc, "export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH"))
    missing: List[str] = []
    for name in ("connect",):
        if not hasattr(_bridge, name):
            missing.append("bridge.%s" % name)
    link_cls = getattr(_bridge, "ArmLink", None)
    for name in ("snapshot", "stop", "transmit_joint"):
        if link_cls is None or not hasattr(link_cls, name):
            missing.append("ArmLink.%s" % name)
    if missing:
        return False, "rm_hw_bridge 缺少关键符号: %s" % ", ".join(missing)
    return True, "rm_hw_bridge 可导入且关键符号齐全"


def _probe_module(here: str):
    """取得 rm75_safe_probe 模块（测试缝隙：优先复用 sys.modules 注入的模块）。"""
    injected = sys.modules.get("rm75_safe_probe")
    if injected is not None:
        return injected
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "rm75_safe_probe", os.path.join(here, "rm75_safe_probe.py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _default_probe(arm: str, ip: str, port: int, udp_port: Optional[int],
                   ws: Optional[str]) -> Dict[str, Any]:
    """探针 7 项前置检查（**只读**：``precheck_only=True``）。"""
    probe = _probe_module(HERE)
    return probe.run_probe(arm=arm, ip=ip, port=port, udp_port=udp_port,
                           precheck_only=True, ws=ws)


# ------------------------------------------------------------------ 门禁主体
def _check(name: str, ok: bool, detail: str, evidence: Any = None) -> Dict[str, Any]:
    return {"name": name, "ok": bool(ok), "detail": detail, "evidence": evidence}


def run_gate(arm: str = "left", ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
             udp_ports: Sequence[int] = DEFAULT_UDP_PORTS,
             ws: Optional[str] = None,
             ping: Optional[Callable[..., Tuple[bool, str]]] = None,
             tcp: Optional[Callable[..., Tuple[bool, str]]] = None,
             udp_free: Optional[Callable[..., Tuple[bool, str]]] = None,
             sdk: Optional[Callable[[], Tuple[bool, str]]] = None,
             probe: Optional[Callable[..., Dict[str, Any]]] = None
             ) -> Dict[str, Any]:
    """跑一遍只读门禁，返回工件字典。

    每个探测都是可注入的可调用对象；默认实现是真实的 ping / TCP / 探针。
    """
    ping_fn = ping or _default_ping
    tcp_fn = tcp or _default_tcp
    udp_fn = udp_free or _default_udp_free
    sdk_fn = sdk or _default_sdk
    probe_fn = probe or _default_probe

    ports = [int(p) for p in (udp_ports or ())]
    started = _now_iso()
    checks: List[Dict[str, Any]] = []
    attempted: List[Dict[str, Any]] = []

    def act(action: str, command: str, result: str, ok: bool) -> None:
        attempted.append({"action": action, "command": command, "result": result,
                          "ok": bool(ok)})

    # --- 1) ip 可达（ICMP）
    ping_cmd = "ping -c 3 -W 1 %s" % ip
    try:
        ok, detail = ping_fn(ip)
    except Exception as exc:  # pragma: no cover - 探测自身异常
        ok, detail = False, "ping 探测异常: %r" % (exc,)
    checks.append(_check(CHECK_IP, ok, detail, {"command": ping_cmd}))
    act("icmp_ping", ping_cmd, detail, ok)

    # --- 2) TCP 可连接
    tcp_cmd = "nc -vz %s %s" % (ip, int(port))
    try:
        ok, detail = tcp_fn(ip, int(port))
    except Exception as exc:  # pragma: no cover
        ok, detail = False, "TCP 探测异常: %r" % (exc,)
    checks.append(_check(CHECK_TCP, ok, detail, {"command": tcp_cmd}))
    act("tcp_connect", tcp_cmd, detail, ok)

    # --- 3) 本地 UDP 端口空闲（避免与上次未退干净的客户端抢 ip:port）
    udp_cmd = "ss -ulnp | grep -E '%s'" % "|".join(str(p) for p in ports)
    udp_results = []
    udp_ok = True
    for p in ports:
        try:
            pok, pdetail = udp_fn(p)
        except Exception as exc:  # pragma: no cover
            pok, pdetail = False, "UDP 端口检查异常: %r" % (exc,)
        udp_results.append({"port": p, "ok": bool(pok), "detail": pdetail})
        udp_ok = udp_ok and bool(pok)
    udp_detail = "；".join("%d %s" % (r["port"], r["detail"]) for r in udp_results)
    checks.append(_check(CHECK_UDP, udp_ok, udp_detail,
                         {"command": udp_cmd, "ports": udp_results}))
    act("udp_port_check", udp_cmd, udp_detail, udp_ok)

    # --- 4) SDK 可加载
    sdk_cmd = "python3 -c 'from rm_hw_bridge import bridge'"
    try:
        ok, detail = sdk_fn()
    except Exception as exc:  # pragma: no cover
        ok, detail = False, "SDK 检查异常: %r" % (exc,)
    checks.append(_check(CHECK_SDK, ok, detail, {"command": sdk_cmd}))
    act("sdk_import", sdk_cmd, detail, ok)

    # --- 5) 探针 7 项前置检查（只读；网络不通则跳过，避免无谓连接）
    udp_port = ports[0] if ports else None
    probe_cmd = ("python3 scripts/rm75_safe_probe.py --ip %s --port %s "
                 "--udp-port %s --precheck-only" % (ip, int(port), udp_port))
    net_ok = bool(checks[0]["ok"]) and bool(checks[1]["ok"])
    if net_ok:
        try:
            report = probe_fn(arm=arm, ip=ip, port=int(port), udp_port=udp_port,
                              ws=ws)
        except Exception as exc:  # pragma: no cover
            report = {"status": STATUS_ERROR, "reason": "探针调用异常: %r" % (exc,),
                      "phases": [], "evidence": []}
        probe_status = str(report.get("status") or "")
        probe_ok = probe_status == STATUS_OK
        failed_names = []
        for phase in (report.get("phases") or []):
            if phase.get("name") == "preflight":
                for c in (phase.get("data") or {}).get("checks", []) or []:
                    if not c.get("ok"):
                        failed_names.append(str(c.get("name")))
        detail = "探针前置检查 status=%s" % (probe_status or "(空)")
        if failed_names:
            detail += "（未通过项: %s）" % ", ".join(failed_names)
        checks.append(_check(CHECK_PROBE, probe_ok, detail,
                             {"command": probe_cmd, "report_status": probe_status,
                              "failed_preflight_checks": failed_names}))
        act("probe_precheck", probe_cmd, detail, probe_ok)
    else:
        detail = ("已跳过：ip/TCP 门禁未通过，未建立任何连接（零运动指令）")
        checks.append(_check(CHECK_PROBE, False, detail,
                             {"command": probe_cmd, "skipped": True}))
        act("probe_precheck", probe_cmd, detail, False)

    failed = [c for c in checks if not c["ok"]]
    status = STATUS_OK if not failed else STATUS_BLOCKED
    reason = ("全部门禁通过（可进入真机窗口）" if not failed else
              "门禁未通过：%s" % "；".join("%s（%s）" % (c["name"], c["detail"])
                                        for c in failed))
    next_action = _next_action(checks, ip, port, ports)

    return {
        "schema": SCHEMA,
        "status": status,
        "reason": reason,
        "arm": arm,
        "endpoint": {"ip": ip, "port": int(port), "udp_ports": ports},
        "started_at": started,
        "finished_at": _now_iso(),
        "checks": checks,
        "attempted_actions": attempted,
        "next_action": next_action,
        "honesty": HONESTY,
        "exit_code": int(EXIT_CODES.get(status, EXIT_CODES[STATUS_BLOCKED])),
    }


def _next_action(checks: List[Dict[str, Any]], ip: str, port: int,
                 ports: Sequence[int]) -> List[str]:
    """下一步**可粘贴命令**列表。

    通用四项恒在（与切片契约一致）：示教器重新使能 7 轴 / 清错误码复位 /
    ``ss -ulnp`` 确认 UDP 端口空闲 / 重跑本脚本；再按失败项追加针对性命令。
    """
    failed = {c["name"] for c in checks if not c["ok"]}
    actions: List[str] = []
    if CHECK_IP in failed or CHECK_TCP in failed:
        actions.append("ping -c 3 %s" % ip)
        actions.append("nc -vz %s %d" % (ip, int(port)))
    if CHECK_SDK in failed:
        actions.append("export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PYTHONPATH")
    actions.append("在示教器上重新使能 7 轴（joint_en_flag 全 1）")
    actions.append("在示教器上清错误码复位（arm_err / joint_err_code 全 0）")
    actions.append("ss -ulnp | grep -E '%s'   # 确认本地 UDP 端口未被占用"
                   % "|".join(str(p) for p in ports))
    if CHECK_PROBE in failed:
        actions.append("python3 scripts/rm75_safe_probe.py --ip %s --precheck-only" % ip)
    actions.append("python3 scripts/rm75_window_gate.py --ip %s --out "
                   "build/reports/validation/window_gate.json" % ip)
    # 去重保序
    seen = set()
    unique: List[str] = []
    for a in actions:
        if a not in seen:
            seen.add(a)
            unique.append(a)
    return unique


# ------------------------------------------------------------------ CLI
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rm75_window_gate",
        description="RM75-B 真机窗口只读门禁（纯只读，绝不发运动指令）")
    p.add_argument("--arm", choices=["left", "right"], default="left")
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--udp-ports", dest="udp_ports", default=",".join(
        str(p) for p in DEFAULT_UDP_PORTS),
        help="本地需空闲的 UDP 端口（逗号分隔，默认 %s）"
             % ",".join(str(p) for p in DEFAULT_UDP_PORTS))
    p.add_argument("--ws", default=None)
    p.add_argument("--out", default=None)
    p.add_argument("--json", action="store_true")
    return p


def main(argv: Optional[List[str]] = None,
         gate: Optional[Callable[..., Dict[str, Any]]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        ports = [int(x) for x in str(args.udp_ports).split(",") if x.strip()]
    except ValueError:
        sys.stderr.write("--udp-ports 非法: %r\n" % args.udp_ports)
        return EXIT_CODES[STATUS_ERROR]
    runner = gate or run_gate
    artifact = runner(arm=args.arm, ip=args.ip, port=args.port,
                      udp_ports=ports, ws=args.ws)

    if args.out:
        out = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as fh:
            json.dump(artifact, fh, ensure_ascii=False, indent=2)

    if args.json:
        print(json.dumps(artifact, ensure_ascii=False, indent=2))
    else:
        print("真机窗口门禁 | status=%s | 端点=%s:%s"
              % (artifact.get("status"), args.ip, args.port))
        for c in artifact.get("checks", []):
            print("  [%s] %-18s %s" % ("PASS" if c.get("ok") else "FAIL",
                                       c.get("name"), c.get("detail")))
        if artifact.get("reason"):
            print("  原因: %s" % artifact["reason"])
        for a in artifact.get("next_action", []):
            print("  下一步: %s" % a)
        print("  诚实声明: %s" % artifact.get("honesty"))
        if args.out:
            print("  工件已写入: %s" % args.out)

    status = str(artifact.get("status") or STATUS_ERROR)
    if status not in EXIT_CODES:
        status = STATUS_ERROR
    return int(EXIT_CODES[status])


if __name__ == "__main__":
    sys.exit(main())
