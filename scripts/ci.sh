#!/usr/bin/env bash
#
# ci.sh —— CI 三档的**唯一实现**（T05 / R010）
#
#   bash scripts/ci.sh fast      # 目标 < 10 min：lint + 骨架单测 + 锁一致性 + 禁用接口扫描
#   bash scripts/ci.sh medium    # fast + 依赖锁一致性 + package:// 解析 + wheel 归档校验
#   bash scripts/ci.sh nightly   # medium + 规划统计与长稳占位（当前为明确标注的 no-op）
#
# 设计约定
# --------
# * **CI 逻辑只在这里**。.github/workflows/ci.yml 只是薄封装（触发器 + 调本脚本），
#   因此用 Actions、内网 cron 还是远程容器 runner 跑，检查项都不会分叉成两套。
# * 退出码语义统一：任一检查失败 → 整体非 0；输出每项 PASS/FAIL 与分档耗时。
# * 解释器优先级：`.venv/bin/python`（存在时）→ 系统 `python3`。
# * 不修改 git 状态、不安装依赖、不访问网络（medium 的 PyPI 元数据比对只在
#   本地缓存存在时进行，否则明确标为 SKIP 而不是伪装成通过）。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ -x "$REPO_ROOT/.venv/bin/python" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON="$(command -v python3)"
else
    echo "ci: 找不到 python 解释器（也没有 .venv）" >&2
    exit 2
fi

TIER="${1:-fast}"
START_TS=$SECONDS
FAILED_CHECKS=()

usage() {
    cat <<'EOF'
用法：bash scripts/ci.sh {fast|medium|nightly}

三档含义与耗时预期
  fast      目标 < 10 min。内容：lint（ruff，缺失时退回 py_compile 兜底）、
            骨架单测（路径层 / 锁 / 禁用名单扫描器）、versions.lock 自洽性、
            厂商解算接口禁用名单静态扫描。
  medium    在 fast 之上追加：constraints ↔ versions.lock 依赖锁一致性、
            package:// 解析测试、wheel 离线归档 sha256 校验（归档缺失时 SKIP）。
  nightly   在 medium 之上追加：规划统计与长稳占位。当前为**明确标注的 no-op**
            ——真实负载在 S02（场景构建）与 S07（golden-file 回归）落地后填充。

退出码
  0  全部检查通过（SKIP 项不算失败）
  1  至少一项检查失败
  2  参数或运行环境错误
EOF
}

if [ "$#" -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    usage
    exit 0
fi

case "$TIER" in
    fast|medium|nightly) ;;
    *)
        echo "ci: 未知档位 '$TIER'（可选 fast|medium|nightly）" >&2
        usage >&2
        exit 2
        ;;
esac

# --------------------------------------------------------------------------- #
# 检查执行框架
# --------------------------------------------------------------------------- #
PASS_COUNT=0
SKIP_COUNT=0
FAIL_COUNT=0

record_pass() { PASS_COUNT=$((PASS_COUNT + 1)); printf '[PASS] %s — %s\n' "$1" "$2"; }
record_skip() { SKIP_COUNT=$((SKIP_COUNT + 1)); printf '[SKIP] %s — %s\n' "$1" "$2"; }
record_fail() {
    FAIL_COUNT=$((FAIL_COUNT + 1))
    FAILED_CHECKS+=("$1")
    printf '[FAIL] %s — %s\n' "$1" "$2"
}

run_check() {
    # run_check <名称> <说明> -- <命令...>
    local name="$1"; shift
    local desc="$1"; shift
    [ "$1" = "--" ] && shift
    local log
    log="$(mktemp)"
    local start=$SECONDS
    "$@" >"$log" 2>&1
    local rc=$?
    local dur=$((SECONDS - start))
    if [ $rc -eq 0 ]; then
        record_pass "$name" "${desc}（${dur}s）"
    else
        record_fail "$name" "${desc}（${dur}s，退出码 $rc）"
        sed -n '1,40p' "$log" | sed 's/^/    | /'
    fi
    rm -f "$log"
    return $rc
}

# 允许"SKIP 语义"的检查：命令退出码 2 视为跳过（例如归档缺失）
run_check_skippable() {
    local name="$1"; shift
    local desc="$1"; shift
    [ "$1" = "--" ] && shift
    local log
    log="$(mktemp)"
    local start=$SECONDS
    "$@" >"$log" 2>&1
    local rc=$?
    local dur=$((SECONDS - start))
    if [ $rc -eq 0 ]; then
        record_pass "$name" "${desc}（${dur}s）"
    elif [ $rc -eq 2 ]; then
        record_skip "$name" "${desc}（${dur}s，环境未就绪：$(head -n 1 "$log")）"
    else
        record_fail "$name" "${desc}（${dur}s，退出码 $rc）"
        sed -n '1,40p' "$log" | sed 's/^/    | /'
    fi
    rm -f "$log"
    return 0
}

# --------------------------------------------------------------------------- #
# fast 档
# --------------------------------------------------------------------------- #
echo "=== ci tier: $TIER（解释器 $PYTHON）==="

# 1) lint：优先 ruff，缺失时用 py_compile 兜底（保证 CI 在任何环境都能跑）
if command -v ruff >/dev/null 2>&1; then
    run_check "lint/ruff" "ruff 静态检查" -- ruff check src tools scripts tests
elif "$PYTHON" -m ruff --version >/dev/null 2>&1; then
    run_check "lint/ruff" "ruff 静态检查（python -m）" -- "$PYTHON" -m ruff check src tools scripts tests
else
    run_check "lint/py_compile" "ruff 不可用，py_compile 兜底语法检查" -- \
        "$PYTHON" -m compileall -q src tools scripts tests
fi

# 2) 骨架单测（不含会递归调用本脚本的 test_ci_script.py）
FAST_TESTS=(
    tests/test_env_lock.py
    tests/test_paths.py
    tests/test_forbidden_apis.py
)
if [ -f tests/test_forbidden_apis.py ]; then
    run_check "tests/skeleton" "骨架单测（${FAST_TESTS[*]}）" -- \
        "$PYTHON" -m pytest -q "${FAST_TESTS[@]}"
else
    run_check "tests/skeleton" "骨架单测（${FAST_TESTS[*]}）" -- \
        "$PYTHON" -m pytest -q tests/test_env_lock.py tests/test_paths.py
fi

# 3) 锁自洽性（versions.lock ↔ constraints，离线可跑）
run_check "lock/consistency" "versions.lock 自洽性" -- \
    "$PYTHON" tools/verify_env.py --lock-only

# 4) 厂商解算接口禁用名单静态扫描（名单唯一真相源 = versions.lock）
run_check "forbidden/apis" "禁用接口静态扫描（src tools scripts）" -- \
    "$PYTHON" scripts/scan_forbidden_apis.py src tools scripts

# --------------------------------------------------------------------------- #
# medium 档
# --------------------------------------------------------------------------- #
if [ "$TIER" = "medium" ] || [ "$TIER" = "nightly" ]; then
    # 5) 依赖锁一致性：constraints ↔ versions.lock。
    #    verify_env --lock-only 已做钉版比对；这里再加 PyPI 元数据比对
    #    （仅在本机缓存存在时进行，缺失则明确 SKIP）。
    if [ -d vendor/.pypi-cache ]; then
        run_check_skippable "lock/constraints" "constraints ↔ versions.lock（PyPI 元数据比对）" -- \
            "$PYTHON" tools/gen_constraints.py --check --offline
    else
        record_skip "lock/constraints" "无本地 PyPI 元数据缓存（vendor/.pypi-cache），跳过元数据比对"
    fi

    # 6) package:// 解析测试
    run_check "tests/package_map" "package:// 解析测试" -- \
        "$PYTHON" -m pytest -q tests/test_package_map.py

    # 7) wheel 离线归档 sha256 校验（归档本机镜像，不入 git；缺失 → SKIP）
    run_check_skippable "wheels/archive" "vendor/wheels 归档 sha256 校验" -- \
        "$PYTHON" tools/verify_env.py --wheels
fi

# --------------------------------------------------------------------------- #
# nightly 档
# --------------------------------------------------------------------------- #
if [ "$TIER" = "nightly" ]; then
    # 8) 规划统计与长稳：S02/S07 才填充真实负载，当前是**明确标注的 no-op**。
    #    刻意不做任何"假装跑过"的事情——占位就是占位。
    record_skip "nightly/plan-stats" "规划统计占位：S02 场景构建后填充真实负载（当前 no-op）"
    record_skip "nightly/soak" "长稳测试占位：S07 golden-file 回归后填充（当前 no-op）"
fi

# --------------------------------------------------------------------------- #
# 汇总
# --------------------------------------------------------------------------- #
ELAPSED=$((SECONDS - START_TS))
echo "--- ci tier '$TIER' 汇总：PASS=$PASS_COUNT SKIP=$SKIP_COUNT FAIL=$FAIL_COUNT，耗时 ${ELAPSED}s ---"
if [ "$FAIL_COUNT" -ne 0 ]; then
    echo "ci: 失败项：${FAILED_CHECKS[*]}" >&2
    exit 1
fi
exit 0
