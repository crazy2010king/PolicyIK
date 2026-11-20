#!/usr/bin/env python3
"""S13 真机**只读**采集：控制器/软件版本 + 机型 + 双臂连通性 + 容器镜像 → 证据工件。

零运动红线
----------
本脚本只做三件事，**全部只读**：

1. **SDK 只读读数** —— ``rm_get_arm_software_info``（版本矩阵）+
   ``rm_get_robot_info``（机型 / 控制器代数），经硬件适配层
   ``rm_hw_bridge.bridge.ArmLink`` 调用（support 名单，不计入 core 22）；
2. **网络可达性探测** —— ``ping -c 3 -W 1`` + TCP connect 尝试（超时 ≤2 s），
   **只判定可达 / 不可达**，不据此推断任何其它结论；
3. **记录容器镜像标签** —— 由**宿主机** ``docker inspect ros_dev`` 取，
   通过 ``--container-image`` / ``RM_CONTAINER_IMAGE`` 传入（容器内无 docker CLI）。

**不下发任何运动指令**：不调用 ``rm_movej`` / ``rm_movej_canfd`` / ``rm_set_arm_*`` /
``rm_set_joint_drive_*``；连链时也不下发 UDP 主动上报配置
（``enable_realtime=False``），除了 SDK 侧通信超时（``rm_set_timeout``）之外
没有任何配置写。

用法
----
::

    # 真机窗口（远端主机 172.16.119.199:2224 → 容器 ros_dev 内）：
    python3 scripts/rm75_s13_versions_probe.py --arm left --ip 192.168.1.18 \
        --out build/reports/s13_versions

    # 离线预演（不连任何设备）：
    python3 scripts/rm75_s13_versions_probe.py --dry-run

退出码：``0`` ok / ``3`` blocked（外部条件不具备，如真机窗口不可达）/
``4`` error（参数非法）。
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

SCHEMA = "rmctl.rm75_s13_versions_connectivity/1"

#: 默认控制器地址（左臂）
DEFAULT_IP = "192.168.1.18"
#: 默认右臂地址
DEFAULT_RIGHT_IP = "192.168.1.19"
#: 默认 TCP 端口
DEFAULT_PORT = 8080
#: ping 次数 / 单包等待（秒）
PING_COUNT = 3
PING_WAIT_S = 1
#: TCP 探测超时（秒）——**硬上限 2 s**
TCP_TIMEOUT_S = 2.0
#: 原始输出摘要保留的最大字符数
SUMMARY_MAX_CHARS = 1200

STATUS_OK = "ok"
STATUS_DRY_RUN = "dry_run"
STATUS_BLOCKED = "blocked"
STATUS_ERROR = "error"

EXIT_OK = 0
EXIT_ERROR = 4
EXIT_BLOCKED = 3

#: ``rm_get_arm_software_info`` 的版本字段（顺序 = 工件字段顺序）
VERSION_FIELDS = (
    "product_version",
    "robot_controller_version",
    "algorithm_version",
    "ctrl_version",
    "ctrl_build_time",
    "dynamic_model_version",
    "plan_version",
    "plan_build_time",
    "com_version",
    "program_version",
)

#: ``rm_get_robot_info`` 的字段
ROBOT_INFO_FIELDS = (
    "arm_dof",
    "arm_model",
    "force_type",
    "robot_controller_version",
)


# --------------------------------------------------------------------- 工具
def _now_iso() -> str:
    """当前 UTC 时间（ISO8601）。"""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _trim(text: str) -> str:
    text = (text or "").strip()
    if len(text) > SUMMARY_MAX_CHARS:
        return text[:SUMMARY_MAX_CHARS] + "...[truncated]"
    return text


def _ping(ip: str, count: int = PING_COUNT,
          wait_s: int = PING_WAIT_S) -> Dict[str, Any]:
    """``ping -c N -W w <ip>``（只读探测；失败如实记录原始输出）。"""
    cmd = ["ping", "-c", str(count), "-W", str(wait_s), ip]
    try:
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=count * wait_s + 10,
        )
        out = proc.stdout.decode("utf-8", errors="replace")
        return {
            "command": " ".join(cmd),
            "ok": proc.returncode == 0,
            "returncode": int(proc.returncode),
            "summary": _trim(out),
        }
    except FileNotFoundError as exc:
        return {
            "command": " ".join(cmd),
            "ok": False,
            "returncode": None,
            "summary": "ping 不可用：%s" % exc,
        }
    except subprocess.TimeoutExpired:
        return {
            "command": " ".join(cmd),
            "ok": False,
            "returncode": None,
            "summary": "ping 超时（> %d s）" % (count * wait_s + 10),
        }
    except Exception as exc:  # noqa: BLE001 - 探测失败不得冒泡成崩溃
        return {
            "command": " ".join(cmd),
            "ok": False,
            "returncode": None,
            "summary": "ping 异常：%r" % (exc,),
        }


def _tcp_probe(ip: str, port: int,
               timeout_s: float = TCP_TIMEOUT_S) -> Dict[str, Any]:
    """TCP 连接尝试（**只握手不发数据**，超时 ≤2 s）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout_s)
    try:
        sock.connect((ip, int(port)))
        return {
            "ok": True,
            "timeout_s": float(timeout_s),
            "summary": "TCP 连接成功 %s:%s（仅握手，未发送任何应用层数据）"
                       % (ip, port),
        }
    except Exception as exc:  # noqa: BLE001 - 超时/拒绝都记原始信息
        return {
            "ok": False,
            "timeout_s": float(timeout_s),
            "summary": "TCP 连接失败 %s:%s（超时 %.1f s）：%s"
                       % (ip, port, timeout_s, exc),
        }
    finally:
        try:
            sock.close()
        except Exception:  # pragma: no cover - 关闭失败不影响结论
            pass


def _connectivity(ip: str, port: int) -> Dict[str, Any]:
    """一次连通性结论（ping + TCP），**只判定可达 / 不可达**。"""
    ping = _ping(ip)
    tcp = _tcp_probe(ip, port)
    return {
        "ip": ip,
        "port": int(port),
        "reachable": bool(ping["ok"] or tcp["ok"]),
        "definition": "reachable = (ping 成功 或 TCP 握手成功)；"
                      "只判定网络可达性，不据此推断任何其它结论",
        "ping": ping,
        "tcp": tcp,
    }


def _empty_versions() -> Dict[str, Any]:
    return {name: None for name in VERSION_FIELDS}


def _empty_robot_info() -> Dict[str, Any]:
    return {name: None for name in ROBOT_INFO_FIELDS}


def _honesty(container_image: Optional[str]) -> Dict[str, Any]:
    """诚实性声明（**显式**写明本轮做了什么 / 没做什么）。"""
    return {
        "no_motion_command_issued": True,
        "motion_commands_sent": 0,
        "teachpendant_version_read": False,
        "notes": [
            "本轮未读取示教器版本（示教器无自动读数路径，只能人工读屏）。",
            "本轮未下发任何运动指令（零运动是不可协商的红线）。",
            "连链时未下发 UDP 主动上报配置，仅读取版本 / 机型信息。",
            "SDK 版本串逐字来自 rm_get_arm_software_info 读数；"
            "SDK 返回空串一律记为 null，未做任何编造或推测。",
            "容器镜像标签取自宿主机 docker inspect（容器内无 docker CLI）。",
        ],
    }


# ----------------------------------------------------------------- 采集主体
def collect_versions(ip: str, port: int, arm: str) -> Dict[str, Any]:
    """建**一次**链、全程复用，读版本矩阵 + 机型（只读）。"""
    from rm_hw_bridge import bridge as hw_bridge  # 延迟导入：离线干跑不碰 SDK

    result: Dict[str, Any] = {
        "controller_software": _empty_versions(),
        "robot_info": _empty_robot_info(),
        "link_status": None,
        "link_reason": "",
        "link_next_action": "",
        "evidence": [],
    }
    link = hw_bridge.connect(ip, port=int(port), arm=arm, enable_realtime=False)
    try:
        result["link_status"] = link.status
        result["link_reason"] = link.reason
        result["link_next_action"] = link.next_action
        result["evidence"] = list(link.evidence or [])
        if not link.connected:
            return result

        sw = link.arm_software_info()
        result["evidence"].append("arm_software_info: %s（%s）" % (
            sw.get("status"), sw.get("reason", "")))
        for name in VERSION_FIELDS:
            result["controller_software"][name] = sw.get(name)

        ri = link.arm_robot_info()
        result["evidence"].append("arm_robot_info: %s（%s）" % (
            ri.get("status"), ri.get("reason", "")))
        for name in ROBOT_INFO_FIELDS:
            result["robot_info"][name] = ri.get(name)
    finally:
        link.close()
    return result


def build_artifact(args: argparse.Namespace) -> Dict[str, Any]:
    """组装证据工件（只读采集）。"""
    container_image = (args.container_image
                       or os.environ.get("RM_CONTAINER_IMAGE") or None)
    source = ("远端真机窗口 %s → 容器 %s；脚本 scripts/rm75_s13_versions_probe.py"
              % (args.source_host, args.source_container))
    method = ("rm_hw_bridge.ArmLink 只读读数（rm_get_arm_software_info + "
              "rm_get_robot_info，均在 support 名单内）；连通性用 ping -c %d -W %d "
              "与 TCP 握手（超时 %.1f s）；容器镜像由宿主机 docker inspect 取"
              % (PING_COUNT, PING_WAIT_S, TCP_TIMEOUT_S))

    if args.dry_run:
        artifact: Dict[str, Any] = {
            "schema": SCHEMA,
            "status": STATUS_DRY_RUN,
            "executed_on": _now_iso(),
            "arm": args.arm,
            "controller_software": _empty_versions(),
            "robot_info": _empty_robot_info(),
            "left_connectivity": {
                "ip": args.ip, "port": int(args.port), "reachable": None,
                "definition": "dry-run：未探测（不连任何设备）",
                "ping": {"command": "ping -c %d -W %d %s" % (
                    PING_COUNT, PING_WAIT_S, args.ip),
                    "ok": None, "returncode": None,
                    "summary": "dry-run：未执行 ping"},
                "tcp": {"ok": None, "timeout_s": TCP_TIMEOUT_S,
                        "summary": "dry-run：未执行 TCP 探测"},
            },
            "right_connectivity": {
                "ip": args.right_ip, "port": int(args.port), "reachable": None,
                "definition": "dry-run：未探测（不连任何设备）",
                "ping": {"command": "ping -c %d -W %d %s" % (
                    PING_COUNT, PING_WAIT_S, args.right_ip),
                    "ok": None, "returncode": None,
                    "summary": "dry-run：未执行 ping"},
                "tcp": {"ok": None, "timeout_s": TCP_TIMEOUT_S,
                        "summary": "dry-run：未执行 TCP 探测"},
            },
            "container_image": container_image,
            "source": source,
            "method": method,
            "gates": [],
            "next_action": _remote_command(args),
            "honesty": _honesty(container_image),
            "dry_run": True,
        }
        return artifact

    left = _connectivity(args.ip, int(args.port))
    right = _connectivity(args.right_ip, int(args.port))
    versions = collect_versions(args.ip, int(args.port), args.arm)

    sw = versions["controller_software"]
    ri = versions["robot_info"]
    non_empty = [k for k, v in sw.items() if v]
    read_ok = bool(versions["link_status"] == "ok" and non_empty)

    gates = [
        {"name": "controller_link", "ok": versions["link_status"] == "ok",
         "detail": versions["link_reason"] or "已连接"},
        {"name": "software_version_read", "ok": read_ok,
         "detail": "非空版本字段 %d 个：%s" % (
             len(non_empty), ", ".join(non_empty) or "无")},
        {"name": "robot_info_read",
         "ok": ri.get("arm_dof") is not None,
         "detail": "arm_dof=%s, arm_model=%s, robot_controller_version=%s" % (
             ri.get("arm_dof"), ri.get("arm_model"),
             ri.get("robot_controller_version"))},
        {"name": "left_connectivity_probed", "ok": True,
         "detail": "left %s reachable=%s" % (args.ip, left["reachable"])},
        {"name": "right_connectivity_probed", "ok": True,
         "detail": "right %s reachable=%s" % (args.right_ip, right["reachable"])},
        {"name": "container_image_captured", "ok": bool(container_image),
         "detail": container_image or "未提供（容器内无 docker CLI，"
                                       "需宿主机 docker inspect ros_dev 取）"},
    ]

    if read_ok:
        status = STATUS_OK
        next_action = (
            "把本工件读数回填 workcell/baselines/version_matrix.yaml 的 "
            "controller_firmware 字段（示教器版本仍无自动读数路径，"
            "该表 status 保持 blocked）；"
            "右臂连通性结论回填 workcell/baselines/network_plan.yaml。"
        )
    else:
        status = STATUS_BLOCKED
        next_action = (
            "%s\n（远端窗口不可达 / 控制器不可达时，先在远端容器内执行 "
            "`ping -c 3 -W 1 %s` 与 `python3 scripts/rmctl.py connect --arm %s "
            "--ip %s --json` 复核；仍不通则记录为 blocked，"
            "**不得**用 dry-run 或合成数据顶替真实读数。）"
            % (_remote_command(args), args.ip, args.arm, args.ip)
        )

    return {
        "schema": SCHEMA,
        "status": status,
        "executed_on": _now_iso(),
        "arm": args.arm,
        "controller_software": sw,
        "robot_info": ri,
        "left_connectivity": left,
        "right_connectivity": right,
        "container_image": container_image,
        "source": source,
        "method": method,
        "gates": gates,
        "next_action": next_action,
        "honesty": _honesty(container_image),
        "dry_run": False,
        "link_status": versions["link_status"],
        "link_reason": versions["link_reason"],
        "evidence": versions["evidence"],
    }


def _remote_command(args: argparse.Namespace) -> str:
    """可粘贴的下一步命令（真机窗口）。"""
    return (
        "ssh -p 2224 -o ConnectTimeout=8 -o BatchMode=yes zy@172.16.119.199 "
        "'docker exec -i ros_dev bash -lc \"cd "
        "/home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/"
        ".gsd-worktrees/M001-3kzz40 && source scripts/rmenv.sh && "
        "export RM_API2_LIB=$PWD/../../.deps/rm_api2/libapi_c.so && "
        "python3 scripts/rm75_s13_versions_probe.py --arm %s --ip %s "
        "--out build/reports/s13_versions\"'"
    )


#: **不得**出现在工件里的字段名（任何声称做过运动的字段 —— 零运动红线）
FORBIDDEN_MOTION_FIELDS = frozenset({
    "move", "moves", "moved", "movej", "movej_canfd", "motion",
    "motion_commands", "trajectory", "trajectory_executed", "executed",
    "joint_deg", "joint_degrees", "joint_position", "joint_positions",
    "target_deg", "arrival_error_deg", "preposition",
})


def _walk_keys(node: Any, path: str = "") -> List[str]:
    """递归列出所有字典键路径（用于禁用字段名扫描）。"""
    out: List[str] = []
    if isinstance(node, dict):
        for key, value in node.items():
            out.append(str(key))
            out.extend(_walk_keys(value, path))
    elif isinstance(node, list):
        for item in node:
            out.extend(_walk_keys(item, path))
    return out


def validate_artifact(artifact: Dict[str, Any]) -> List[str]:
    """工件自检：返回问题列表（**空列表 = 合规**）。

    供测试与 CI 复用；检查项与 S13-T2 的验收口径一一对应：

    * schema / status 合法（``ok`` / ``blocked``）；
    * 版本字段「非空字符串 **或** 显式 ``null``」，``ok`` 时至少一个非空；
    * 左 / 右连通性都有**明确布尔结论** + 原始输出摘要；
    * honesty 写明「未读取示教器版本」「未下发任何运动指令」；
    * 工件里**没有**任何声称做过运动的字段名；
    * ``status=blocked`` 时工件仍须**自证**（门禁结果 + 可粘贴 next_action）。
    """
    problems: List[str] = []

    if artifact.get("schema") != SCHEMA:
        problems.append("schema 不是 %s（实为 %r）" % (SCHEMA, artifact.get("schema")))

    status = artifact.get("status")
    if status not in (STATUS_OK, STATUS_BLOCKED):
        problems.append("status 必须是 ok / blocked（实为 %r）" % (status,))

    sw = artifact.get("controller_software")
    if not isinstance(sw, dict):
        problems.append("缺少 controller_software")
    else:
        for name in VERSION_FIELDS:
            if name not in sw:
                problems.append("controller_software 缺字段 %s" % name)
                continue
            value = sw[name]
            if value is None:
                continue
            if not isinstance(value, str) or not value.strip():
                problems.append(
                    "controller_software.%s 必须是非空字符串或 null（实为 %r）"
                    % (name, value))
        if status == STATUS_OK:
            if not any(sw.get(name) for name in VERSION_FIELDS):
                problems.append("status=ok 但没有任何非空版本串")

    ri = artifact.get("robot_info")
    if not isinstance(ri, dict):
        problems.append("缺少 robot_info")
    else:
        for name in ("arm_dof", "arm_model", "robot_controller_version"):
            if name not in ri:
                problems.append("robot_info 缺字段 %s" % name)
            elif ri[name] is not None and not isinstance(ri[name], int):
                problems.append("robot_info.%s 必须是整数或 null（实为 %r）"
                                % (name, ri[name]))

    for side in ("left_connectivity", "right_connectivity"):
        conn = artifact.get(side)
        if not isinstance(conn, dict):
            problems.append("缺少 %s" % side)
            continue
        if not isinstance(conn.get("reachable"), bool):
            problems.append("%s.reachable 必须是明确布尔值（实为 %r）"
                            % (side, conn.get("reachable")))
        for proto in ("ping", "tcp"):
            block = conn.get(proto)
            if not isinstance(block, dict):
                problems.append("%s.%s 缺原始输出摘要" % (side, proto))
                continue
            summary = block.get("summary")
            if not isinstance(summary, str) or not summary.strip():
                problems.append("%s.%s.summary 必须是非空字符串" % (side, proto))

    honesty = artifact.get("honesty")
    if not isinstance(honesty, dict):
        problems.append("缺少 honesty 声明")
    else:
        if honesty.get("no_motion_command_issued") is not True:
            problems.append("honesty.no_motion_command_issued 必须为 true")
        if honesty.get("teachpendant_version_read") is not False:
            problems.append("honesty.teachpendant_version_read 必须为 false")
        notes = " ".join(str(n) for n in (honesty.get("notes") or []))
        if "运动指令" not in notes:
            problems.append("honesty 缺「未下发任何运动指令」语义")
        if "示教器" not in notes:
            problems.append("honesty 缺「未读取示教器版本」语义")

    for key in _walk_keys(artifact):
        if key in FORBIDDEN_MOTION_FIELDS:
            problems.append("工件出现疑似运动字段名 %r（零运动红线）" % key)

    if status == STATUS_BLOCKED:
        gates = artifact.get("gates")
        if not isinstance(gates, list) or not gates:
            problems.append("status=blocked 时工件必须自证：gates 不得为空")
        elif not all(isinstance(g, dict) and "name" in g and "ok" in g
                     for g in gates):
            problems.append("gates 每项必须含 name / ok")
        if not str(artifact.get("next_action") or "").strip():
            problems.append("status=blocked 时必须有可粘贴 next_action")

    return problems


def _resolve_out(out: str, dry_run: bool) -> str:
    """``--out`` 既接受文件路径，也接受路径前缀（``build/reports/x`` → ``x.json``）。"""
    path = out
    if not path.endswith(".json"):
        path = path + ".json"
    if dry_run:
        # dry-run 骨架**不得**覆盖真机工件
        base, ext = os.path.splitext(path)
        path = base + ".dryrun" + ext
    return path


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rm75_s13_versions_probe",
        description="S13 真机只读采集：控制器/软件版本 + 机型 + 双臂连通性 + 容器镜像")
    parser.add_argument("--arm", default="left", choices=("left", "right"),
                        help="要读版本的主臂（默认 left）")
    parser.add_argument("--ip", default=DEFAULT_IP, help="主臂控制器 IP")
    parser.add_argument("--right-ip", default=DEFAULT_RIGHT_IP, help="右臂 IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="控制器 TCP 端口（默认 8080）")
    parser.add_argument("--out", default="build/reports/s13_versions",
                        help="输出 JSON 路径（无 .json 后缀时自动补齐）")
    parser.add_argument("--dry-run", action="store_true",
                        help="离线骨架：不连接任何设备，status=dry_run")
    parser.add_argument("--container-image", default=None,
                        help="容器镜像标签（宿主机 docker inspect 取；"
                             "缺省读环境变量 RM_CONTAINER_IMAGE）")
    parser.add_argument("--source-host", default="172.16.119.199:2224",
                        help="采集窗口来源（写入工件 source 字段）")
    parser.add_argument("--source-container", default="ros_dev",
                        help="采集容器名（写入工件 source 字段）")
    parser.add_argument("--note", action="append", default=None,
                        help="操作者备注（可重复）：写入工件 operator_notes，"
                             "**只作现场说明**，不冒充任何测量值")
    args = parser.parse_args(argv)

    if args.port <= 0 or args.port > 65535:
        print("端口非法：%r" % (args.port,), file=sys.stderr)
        return EXIT_ERROR

    artifact = build_artifact(args)
    notes = list(getattr(args, "note", None) or [])
    if notes:
        artifact["operator_notes"] = notes
    path = _resolve_out(args.out, args.dry_run)
    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(artifact, fh, ensure_ascii=False, indent=2)
        fh.write("\n")

    sw = artifact["controller_software"]
    print("status = %s" % artifact["status"])
    print("工件   = %s" % path)
    print("版本读数（非空）:")
    for name in VERSION_FIELDS:
        if sw.get(name):
            print("  %-26s = %s" % (name, sw[name]))
    print("机型   = %s" % json.dumps(artifact["robot_info"], ensure_ascii=False))
    print("左臂连通 = %s（%s）" % (artifact["left_connectivity"]["ip"],
                                   artifact["left_connectivity"]["reachable"]))
    print("右臂连通 = %s（%s）" % (artifact["right_connectivity"]["ip"],
                                   artifact["right_connectivity"]["reachable"]))
    print("容器镜像 = %s" % artifact.get("container_image"))
    if artifact.get("next_action"):
        print("下一步:\n%s" % artifact["next_action"])

    if artifact["status"] == STATUS_OK:
        return EXIT_OK
    if artifact["status"] == STATUS_DRY_RUN:
        return EXIT_OK
    if artifact["status"] == STATUS_BLOCKED:
        return EXIT_BLOCKED
    return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
