#!/usr/bin/env bash
# RM75-B 真机最小动作验证 —— 一键执行器（安全顺序编排）
#
#   usage: bash scripts/run_rm75_probe.sh [options]
#
# 设计要点（安全优先）：
#   1. **默认不动臂**：不带 --with-motion 时只跑「离线预演 + 只读体检」；
#      任何运动都必须显式加 --with-motion（避免手滑就跑起动作）。
#   2. **顺序不可跳过**：连通性检查 → 离线预演 → 只读体检 → （可选）最小动作。
#      前一步不通过就不进下一步（`set -e` + 显式退出码传递）。
#   3. **运动步骤只做一件事**：单关节微动 + 回程，幅度由 --delta-deg 限制，
#      硬上限 1.0°（超限由 rm75_safe_probe.py 在 0 次 SDK 调用前拒绝）。
#   4. **SDK 找不到就 blocked(3)**，不静默继续，也不伪造结论。
#   5. 退出码与 rm75_safe_probe.py 一致：0 ok / 2 参数非法 / 3 blocked / 4 error。
#
# 可注入变量（测试用，正常使用不需要）：
#   RM75_PROBE_PYTHON   调用探针的解释器（默认 python3）
#   RM75_PROBE_SCRIPT   探针脚本路径（默认 $ROOT/scripts/rm75_safe_probe.py）
#   RM_API2_LIB         RM-API2 动态库路径（缺省时按 .deps/rm_api2/ 顺序自寻）
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IP="192.168.1.18"
PORT="8080"
UDP_PORT="8091"
JOINT="0"
DELTA_DEG="0.3"
SPEED_PCT="2"
CHANNEL="movej"
OUT_DIR="$ROOT/build/reports"
WITH_MOTION="0"
SKIP_PRECHECK="0"
SKIP_NET_CHECK="0"
SDK_LIB="${RM_API2_LIB:-}"
PY="${RM75_PROBE_PYTHON:-python3}"
PROBE="${RM75_PROBE_SCRIPT:-$ROOT/scripts/rm75_safe_probe.py}"

EXIT_USAGE=2
EXIT_BLOCKED=3

usage() {
    sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

选项：
  --ip <addr>          控制器 IP（默认 192.168.1.18）
  --port <n>           TCP 端口（默认 8080）
  --udp-port <n>       UDP 主动上报端口（默认 8091）
  --joint <0-6>        微动关节序号（0 基，默认 0 = J1）
  --delta-deg <deg>    微动角度，硬上限 1.0°（默认 0.3）
  --speed-pct <n>      rm_movej 速度百分比，硬上限 10（默认 2）
  --channel <c>        movej | canfd（默认 movej）
  --out-dir <dir>      报告落盘目录（默认 <repo>/build/reports）
  --with-motion        **执行最小动作**（默认只读；不加此开关臂不会动）
  --skip-precheck      跳过只读体检（不建议；仅调试用）
  --skip-net-check     跳过 ping/TCP 连通性检查（仅离线自检 / CI 用；真机跑不要加）
  --sdk-lib <path>     指定 libapi_c.so（默认按 RM_API2_LIB / .deps/rm_api2/ 自寻）
  -h, --help           显示本帮助

示例：
  # 只读体检（零运动，默认行为）
  bash scripts/run_rm75_probe.sh --ip 192.168.1.18

  # 最小动作验证（0.3° / 2% 速度 / 单关节 / 去程回程）
  bash scripts/run_rm75_probe.sh --ip 192.168.1.18 --with-motion \
       --joint 0 --delta-deg 0.3 --speed-pct 2
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --ip) IP="$2"; shift 2 ;;
        --port) PORT="$2"; shift 2 ;;
        --udp-port) UDP_PORT="$2"; shift 2 ;;
        --joint) JOINT="$2"; shift 2 ;;
        --delta-deg) DELTA_DEG="$2"; shift 2 ;;
        --speed-pct) SPEED_PCT="$2"; shift 2 ;;
        --channel) CHANNEL="$2"; shift 2 ;;
        --out-dir) OUT_DIR="$2"; shift 2 ;;
        --with-motion) WITH_MOTION="1"; shift ;;
        --skip-precheck) SKIP_PRECHECK="1"; shift ;;
        --skip-net-check) SKIP_NET_CHECK="1"; shift ;;
        --sdk-lib) SDK_LIB="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage >&2; exit $EXIT_USAGE ;;
    esac
done

say()  { printf '\n=== %s\n' "$*"; }
warn() { printf '警告: %s\n' "$*" >&2; }
die_blocked() { printf '阻塞: %s\n' "$*" >&2; exit $EXIT_BLOCKED; }

# ---------------------------------------------------------------- 0) SDK 定位
find_sdk() {
    if [ -n "$SDK_LIB" ] && [ -f "$SDK_LIB" ]; then
        printf '%s' "$SDK_LIB"; return 0
    fi
    local cand
    for cand in "$ROOT/.deps/rm_api2/libapi_c.so" \
                "$ROOT/../../.deps/rm_api2/libapi_c.so" \
                "$ROOT/../.deps/rm_api2/libapi_c.so" \
                "$ROOT/.deps/rm_api2"/*.so ; do
        if [ -f "$cand" ]; then printf '%s' "$cand"; return 0; fi
    done
    return 1
}

say "0/4 定位 RM-API2 动态库"
SDK_PATH="$(find_sdk)" || die_blocked \
"未找到 RM-API2（libapi_c.so）。
  已搜索: \$RM_API2_LIB / $ROOT/.deps/rm_api2/ / 上级 checkout 的 .deps/rm_api2/
  → 修复: export RM_API2_LIB=/abs/path/to/libapi_c.so
           或 mkdir -p $ROOT/.deps/rm_api2 && cp /abs/path/to/libapi_c.so $ROOT/.deps/rm_api2/"
export RM_API2_LIB="$SDK_PATH"
echo "  SDK: $RM_API2_LIB"

# ------------------------------------------------------------ 1) 环境自举
export PYTHONPATH="$ROOT/dualarm_ws/src/rm_hw_bridge:$ROOT/dualarm_ws/src/rm_safety:$ROOT/dualarm_ws/src/rmctl:${PYTHONPATH:-}"

# ------------------------------------------------------- 2) 连通性检查
say "1/4 连通性检查（$IP:$PORT）"
if [ "$SKIP_NET_CHECK" = "1" ]; then
    warn "已跳过连通性检查（--skip-net-check）：真机运行不要加这个开关"
elif command -v ping >/dev/null 2>&1; then
    if ping -c 2 -W 2 "$IP" >/dev/null 2>&1; then
        echo "  ping $IP: OK"
    else
        die_blocked "ping $IP 不通（机械臂不可达）：确认臂已上电、网线与 IP 配置"
    fi
else
    warn "本机无 ping 命令，跳过 ICMP 检查"
fi

if [ "$SKIP_NET_CHECK" = "1" ]; then
    :
elif (exec 3<>"/dev/tcp/$IP/$PORT") 2>/dev/null; then
    exec 3>&- 2>/dev/null || true
    echo "  TCP $IP:$PORT: OK"
else
    die_blocked "TCP $IP:$PORT 连不上：确认控制器已上电、端口未被占用（同一 ip:port 只允许一个客户端）"
fi

if command -v ss >/dev/null 2>&1; then
    if ss -ulnp 2>/dev/null | grep -q ":$UDP_PORT "; then
        warn "本机 UDP $UDP_PORT 已被占用：若占用者是上一次未退净的探针进程，"
        warn "请先结束它（否则 SDK 会静默失败）"
    else
        echo "  UDP $UDP_PORT: 空闲"
    fi
fi

# ------------------------------------------------------------ 3) 探针执行
run_probe() {
    local label="$1"; shift
    say "$label"
    echo "  \$ $*"
    "$@"
    local rc=$?
    return $rc
}

PROBE_ARGS=(--ip "$IP" --port "$PORT" --udp-port "$UDP_PORT" --arm left)

if ! "$PY" "$PROBE" "${PROBE_ARGS[@]}" --dry-run; then
    die_blocked "离线预演失败（参数越界或探针不可用）：修好后再跑"
fi

if [ "$SKIP_PRECHECK" = "1" ]; then
    warn "已跳过只读体检（--skip-precheck）：运动前臂的状态未经检查"
else
    if ! "$PY" "$PROBE" "${PROBE_ARGS[@]}" --precheck-only; then
        die_blocked "只读体检未通过：臂不可达 / 有错误码 / 未使能 / 控制器忙 —— 本次不会运动"
    fi
fi

if [ "$WITH_MOTION" != "1" ]; then
    say "完成（只读模式）"
    echo "臂未发生任何运动。要执行最小动作验证，请追加 --with-motion"
    exit 0
fi

say "警告：接下来会让真机做一次关节 $JOINT 的 $DELTA_DEG° 微动（$CHANNEL 通道，${SPEED_PCT}% 速度）并回程"
echo "  请确认：工作半径内无人 / 急停按钮可触 / 示教器在手边"

mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d-%H%M%S)"
REPORT="$OUT_DIR/rm75_safe_probe_${STAMP}.json"

if "$PY" "$PROBE" "${PROBE_ARGS[@]}" \
        --channel "$CHANNEL" --joint "$JOINT" \
        --delta-deg "$DELTA_DEG" --speed-pct "$SPEED_PCT" \
        --out "$REPORT"; then
    say "完成（最小动作验证通过）"
    echo "  报告: $REPORT"
    exit 0
else
    rc=$?
    say "失败（退出码 $rc）"
    [ -f "$REPORT" ] && echo "  报告: $REPORT"
    echo "  请先人工确认臂位姿，再按报告里的 evidence/next_action 处理"
    exit "$rc"
fi
