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
            厂商解算接口禁用名单静态扫描、**模型回归**（碰撞/质量/FK/限位/夹爪，
            模型本体零外部资产依赖 → 恒跑；Drake 不可用时显式 SKIP）、
            **规划回归**（场景 / 轨迹 / CLI / 回放，Drake 不可用时显式 SKIP）、
            **设备层与安全闸（零硬件）**（安全七闸 / 状态机 / 执行环 / 恢复流程 /
            全链路运行日志 / 硬件门控脚本，恒跑——零硬件也能回归）、
            **双臂几何与标定回归（零硬件）**（模型变体 / 双臂单 plant 装配 /
            碰撞过滤矩阵 / 共同点一致性 / 同步执行器 / 标定内核 / 标定 CLI，
            恒跑——零硬件也能回归）、
            **双臂紧协调回归**（RM75 解析 IK / 双臂 14 维限位 / 共享时间轴双轨迹 /
            最小坐标参数化 / 最小坐标凸区域 / 协调规划 / 协调指标脚本 / 协调 CLI，
            恒跑——零硬件也能回归）。
  medium    在 fast 之上追加：constraints ↔ versions.lock 依赖锁一致性、
            package:// 解析测试、wheel 离线归档 sha256 校验（归档缺失时 SKIP）。
  nightly   在 medium 之上追加：**规划统计真实调用**（scripts/plan_stats.py，
            默认 100 组随机起终点，规模/预算可用环境变量覆盖；
            超时按失败处理但打印实测值）、规划链路自检（verify_env --plan）、
            长稳占位（golden-file 回归，属 S07）。

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
        echo "    | ...（日志尾部，用于保留实测值）"
        tail -n 20 "$log" | sed 's/^/    | /'
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

# 5) 模型回归（S01-T06 接入）。
#    模型本体（URDF）零外部资产依赖 → 碰撞/质量/FK/限位断言**恒跑**，不得因
#    上游归档缺失而跳过；只有 Drake 本身不可用时才显式 SKIP（环境未就绪）。
MODEL_TESTS=(
    tests/test_model_load.py
    tests/test_model_regression.py
    tests/test_mdh_fk.py
    tests/test_gripper.py
    tests/test_joint_limits.py
)
if "$PYTHON" -c "import pydrake" >/dev/null 2>&1; then
    run_check "tests/model" "模型回归单测（碰撞/质量/FK/限位/夹爪）" -- \
        "$PYTHON" -m pytest -q "${MODEL_TESTS[@]}"
    run_check "model/verify-env" "模型自检（verify_env --model）" -- \
        "$PYTHON" tools/verify_env.py --model
else
    record_skip "tests/model" "Drake（pydrake）不可用：模型回归断言跳过（环境未就绪）"
    record_skip "model/verify-env" "Drake（pydrake）不可用：模型自检跳过（环境未就绪）"
fi

# 6) 网格工具单测（合成夹具部分与上游归档无关 → 恒跑；归档依赖用例内部 SKIP）
run_check "tests/mesh-tools" "网格工具单测（STL 解析 / OBJ 写出 / 包围盒）" -- \
    "$PYTHON" -m pytest -q tests/test_mesh_tools.py

# 7) 视觉网格回归（**依赖上游归档** → 缺失时显式 SKIP，不伪装成通过）
run_check_skippable "assets/visual-mesh" "官方 STL → OBJ 转换与包围盒复测（需上游归档）" -- \
    "$PYTHON" scripts/convert_meshes.py --print-boxes

# 8) 规划回归（S02-T08 接入）：CLI 族 + 录制/回放 + 场景/轨迹契约。
#    重负载用例（IRIS 区域生成）打 @pytest.mark.slow → fast 档用 -m "not slow" 跳过。
#    pydrake 不可用 → 显式 SKIP（不伪装成通过）。
PLANNING_TESTS=(
    tests/test_scenario.py
    tests/test_planning_scene.py
    tests/test_trajectory.py
    tests/test_cli.py
    tests/test_replay.py
)
if "$PYTHON" -c "import pydrake" >/dev/null 2>&1; then
    run_check "tests/planning" "规划回归单测（场景/轨迹/CLI/回放）" -- \
        "$PYTHON" -m pytest -q "${PLANNING_TESTS[@]}" -m "not slow"
else
    record_skip "tests/planning" "Drake（pydrake）不可用：规划回归断言跳过（环境未就绪）"
fi

# 9) **设备层与安全闸（零硬件）**（S03-T08 接入）。
#    安全七闸 / 状态机 / 执行环 / 恢复流程 / 全链路运行日志 / 硬件门控脚本
#    全部可在**无真机、无 SDK** 的环境里被真实验证 ⇒ **恒跑**，不得 SKIP。
#    新增项失败即整体失败（run_check 的语义）。
DEVICE_TESTS=(
    tests/test_device_config.py
    tests/test_device_sdk.py
    tests/test_device_transport.py
    tests/test_device_safety.py
    tests/test_device_state_source.py
    tests/test_device_state_machine.py
    tests/test_device_exec_loop.py
    tests/test_device_metrics.py
    tests/test_device_recovery.py
    tests/test_device_run_log.py
    tests/test_device_scripts.py
    tests/test_rm_device_driver.py
)
run_check "tests/device" "设备层与安全闸回归（零硬件：七闸 / 状态机 / 执行环 / 恢复 / 运行日志 / 脚本）" -- \
    "$PYTHON" -m pytest -q "${DEVICE_TESTS[@]}"

# 10) **双臂几何与标定回归（零硬件）**（S04-T08 接入）。
#     模型变体生成器 / 双臂单 plant 装配 / 程序化碰撞过滤矩阵 / 共同点一致性判定器 /
#     双臂同一 tick 同步执行器 / 标定内核（schema + 求解器 + 产出物）/ 标定 CLI。
#     全部可在**无真机、无实测数据**的环境里被真实验证 ⇒ **恒跑**，不得 SKIP。
#     新增项失败即整体失败（run_check 的语义，绝不降级成 run_check_skippable）。
BIMANUAL_CALIBRATION_TESTS=(
    tests/test_bimanual_model_variants.py
    tests/test_bimanual_dual_scene.py
    tests/test_bimanual_collision_filter.py
    tests/test_bimanual_consistency.py
    tests/test_bimanual_sync_exec.py
    tests/test_calibration_solvers.py
    tests/test_calibration_schema.py
    tests/test_calibration_artifact.py
    tests/test_calibration_cli.py
)
run_check "tests/bimanual-calibration" "双臂几何与标定回归（零硬件：变体 / 单 plant / 碰撞过滤 / 一致性 / 同步执行 / 标定内核 / 标定 CLI）" -- \
    "$PYTHON" -m pytest -q "${BIMANUAL_CALIBRATION_TESTS[@]}"

# 11) **双臂紧协调回归（零硬件）**（S05-T07 接入）。
#     RM75 解析 IK（numpy-only）/ 双臂 14 维限位 / 共享时间轴双轨迹 /
#     最小坐标参数化（相对位姿构造性满足）/ 最小坐标凸区域生成 + 缓存 /
#     协调规划（参数化 → 区域图 → GCS → 14 维重定时 → 零逃逸复检）/
#     协调指标脚本 / 协调 CLI。
#     全部可在**无真机、无实测数据**的环境里被真实验证 ⇒ **恒跑**，不得 SKIP。
#     新增项失败即整体失败（run_check 的语义，绝不降级成 run_check_skippable）。
BIMANUAL_COORDINATION_TESTS=(
    tests/test_analytic_ik.py
    tests/test_coordinated_limits.py
    tests/test_coordinated_trajectory.py
    tests/test_coordinated_parameterization.py
    tests/test_coordinated_regions.py
    tests/test_coordinated_planner.py
    tests/test_coordinated_stats.py
    tests/test_coordinated_cli.py
)
run_check "tests/bimanual-coordination" "双臂紧协调回归（零硬件：解析 IK / 双臂限位 / 共享时间轴 / 最小坐标参数化 / 凸区域 / 协调规划 / 指标脚本 / 协调 CLI）" -- \
    "$PYTHON" -m pytest -q "${BIMANUAL_COORDINATION_TESTS[@]}"

# --------------------------------------------------------------------------- #
# medium 档
# --------------------------------------------------------------------------- #
if [ "$TIER" = "medium" ] || [ "$TIER" = "nightly" ]; then
    # 8) 依赖锁一致性：constraints ↔ versions.lock。
    #    verify_env --lock-only 已做钉版比对；这里再加 PyPI 元数据比对
    #    （仅在本机缓存存在时进行，缺失则明确 SKIP）。
    if [ -d vendor/.pypi-cache ]; then
        run_check_skippable "lock/constraints" "constraints ↔ versions.lock（PyPI 元数据比对）" -- \
            "$PYTHON" tools/gen_constraints.py --check --offline
    else
        record_skip "lock/constraints" "无本地 PyPI 元数据缓存（vendor/.pypi-cache），跳过元数据比对"
    fi

    # 9) package:// 解析测试
    run_check "tests/package_map" "package:// 解析测试" -- \
        "$PYTHON" -m pytest -q tests/test_package_map.py

    # 10) wheel 离线归档 sha256 校验（归档本机镜像，不入 git；缺失 → SKIP）
    run_check_skippable "wheels/archive" "vendor/wheels 归档 sha256 校验" -- \
        "$PYTHON" tools/verify_env.py --wheels
fi

# --------------------------------------------------------------------------- #
# nightly 档
# --------------------------------------------------------------------------- #
if [ "$TIER" = "nightly" ]; then
    # 11) 规划统计（**真实调用**，不再是 no-op）。
    #     规模与时间预算可用环境变量覆盖：
    #       PLAN_STATS_SAMPLES   采样组数（默认 100）
    #       PLAN_STATS_MAX_SECONDS  时间预算（默认 1500，超出即停并打印实测值）
    #     超时按**失败**处理（退出码非 0），但实测值照样打印（见 run_check 的日志回显）。
    PLAN_STATS_SAMPLES="${PLAN_STATS_SAMPLES:-100}"
    PLAN_STATS_MAX_SECONDS="${PLAN_STATS_MAX_SECONDS:-1500}"
    run_check "nightly/plan-stats" "规划统计（${PLAN_STATS_SAMPLES} 组随机起终点，预算 ${PLAN_STATS_MAX_SECONDS}s）" -- \
        "$PYTHON" scripts/plan_stats.py --samples "$PLAN_STATS_SAMPLES" \
        --max-seconds "$PLAN_STATS_MAX_SECONDS"

    # 12) 规划链路自检（装配场景 + IK + 短规划 + 碰撞复检）
    if "$PYTHON" -c "import pydrake" >/dev/null 2>&1; then
        run_check "nightly/plan-verify" "规划链路自检（verify_env --plan）" -- \
            "$PYTHON" tools/verify_env.py --plan
    else
        record_skip "nightly/plan-verify" "Drake（pydrake）不可用：规划链路自检跳过（环境未就绪）"
    fi

    # 13) 长稳测试占位：S07 golden-file 回归后填充（当前 no-op）。
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
