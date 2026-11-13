#!/usr/bin/env bash
#
# setup_env.sh —— 环境重建的**唯一入口**（T04）
#
#   bash tools/setup_env.sh [--offline] [--venv DIR] [--profile py312|py310]
#                           [--dry-run] [--help]
#
# 目标：兑现 R010 的验收句「新机器 30 分钟内一条命令重建完整环境」。
#
# 流程
#   1. 选择 profile（py312 = baseline 受支持基线 / py310 = fallback 非受支持降级档）
#      与对应的 constraints 文件（constraints/<profile>.txt）；
#   2. 检测 / 创建 venv（默认仓库根 .venv），**不修改系统 site-packages、不污染 --user**；
#   3. 升级 pip（离线模式跳过）；
#   4. 按 constraints 安装完整依赖闭包（--require-hashes）；
#   5. 调用 tools/verify_env.py --check 做逐包版本对照；
#   6. 打印 Drake / robotic_arm 实装版本与 versions.lock 对照结果；
#   7. 输出 wall time，便于对照 30 min 目标；用退出码表达成败。
#
# 退出码：0 成功 / 1 安装或自检失败 / 2 参数或环境错误。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
LOCK_FILE="${LOCK_FILE:-$REPO_ROOT/versions.lock}"
WHEELS_DIR="${WHEELS_DIR:-$REPO_ROOT/vendor/wheels}"

VENV_DIR="$REPO_ROOT/.venv"
PROFILE=""
OFFLINE=0
DRY_RUN=0
START_TS=$SECONDS

usage() {
    cat <<'EOF'
用法：bash tools/setup_env.sh [选项]

选项
  --offline            离线安装（pip install --no-index --find-links vendor/wheels），
                       要求本机已有 vendor/wheels 离线镜像
  --venv DIR           venv 目录（默认 <仓库根>/.venv）；已存在则直接复用
  --profile py312|py310  目标 profile（默认按当前解释器自动选：3.12→py312，3.10→py310）
  --dry-run            只打印将要执行的命令，不做任何改动
  -h, --help           显示本帮助

示例
  bash tools/setup_env.sh --profile py312                 # 联网重建受支持基线
  bash tools/setup_env.sh --offline --venv /tmp/rm75-env  # 用本机镜像离线重建
  bash tools/setup_env.sh --dry-run                       # 只看将要执行什么
EOF
}

die() {
    echo "setup_env: $*" >&2
    exit 2
}

run() {
    # 打印并执行一条命令；--dry-run 时只打印
    printf '+ %s\n' "$*"
    if [ "$DRY_RUN" -eq 0 ]; then
        "$@"
    fi
}

while [ $# -gt 0 ]; do
    case "$1" in
        --offline) OFFLINE=1 ;;
        --venv)
            shift || die "--venv 缺少取值"
            VENV_DIR="$1"
            ;;
        --profile)
            shift || die "--profile 缺少取值"
            PROFILE="$1"
            ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "未知参数: $1" ;;
    esac
    shift
done

command -v "$PYTHON" >/dev/null 2>&1 || die "找不到解释器: $PYTHON"

# ── 1. profile 选择 ────────────────────────────────────────────────────────
profile_to_key() {
    case "$1" in
        py312|baseline) echo "baseline" ;;
        py310|fallback) echo "fallback" ;;
        *) die "未知 profile: $1（可选 py312 / py310）" ;;
    esac
}

if [ -z "$PROFILE" ]; then
    host_py="$("$PYTHON" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    case "$host_py" in
        3.12) PROFILE="py312" ;;
        3.10) PROFILE="py310" ;;
        *) die "当前解释器 Python $host_py 不在受支持档位（3.12 / 3.10）；请显式传 --profile" ;;
    esac
fi
PROFILE_KEY="$(profile_to_key "$PROFILE")"

CONSTRAINTS="$REPO_ROOT/constraints/$PROFILE.txt"
[ -f "$CONSTRAINTS" ] || die "缺少约束文件: $CONSTRAINTS"

# 离线镜像必须先于任何改动存在（提前失败，避免白建一个 venv）
if [ "$OFFLINE" -eq 1 ] && [ ! -d "$WHEELS_DIR" ]; then
    die "离线模式需要本机镜像，但 $WHEELS_DIR 不存在；请先执行 bash tools/lock_deps.sh --download"
fi

# ── 2. 目标解释器（profile 要求特定 Python 小版本）────────────────────────
PROFILE_PY_VERSION="$(awk -v p="$PROFILE" '
    /^# python[[:space:]]*:/ {gsub(/[^0-9. ]/, "", $0); print $4; exit}
' "$CONSTRAINTS")"
[ -n "$PROFILE_PY_VERSION" ] || PROFILE_PY_VERSION="$( [ "$PROFILE_KEY" = "baseline" ] && echo 3.12 || echo 3.10 )"

HOST_PY_VERSION="$("$PYTHON" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
if [ "$HOST_PY_VERSION" != "$PROFILE_PY_VERSION" ] && [ -z "${CONDA_PREFIX:-}" ]; then
    echo "setup_env: 注意：当前解释器 Python $HOST_PY_VERSION 与 profile $PROFILE（需要 $PROFILE_PY_VERSION）不一致" >&2
    echo "setup_env: 将尝试用 python$PROFILE_PY_VERSION 创建 venv；找不到则报错" >&2
fi

# ── 3. venv 检测 / 创建 ───────────────────────────────────────────────────
VENV_PY="$VENV_DIR/bin/python"
if [ ! -x "$VENV_PY" ]; then
    if [ "$DRY_RUN" -eq 1 ]; then
        VENV_PY="<待创建> $VENV_DIR/bin/python"
    else
        mkdir -p "$VENV_DIR"
        echo "setup_env: 创建 venv: $VENV_DIR"
        if ! "$PYTHON" -m venv "$VENV_DIR" 2>"$VENV_DIR/.venv-create.log"; then
            # 常见于 Debian/Ubuntu 未装 python3-venv（ensurepip 缺失）：
            # 退化为 --without-pip，再用宿主 pip 的 --python 能力安装。
            echo "setup_env: venv 创建（带 ensurepip）失败，改用 --without-pip 重建"
            if ! "$PYTHON" -m venv --without-pip --clear "$VENV_DIR" 2>>"$VENV_DIR/.venv-create.log"; then
                die "创建 venv 失败: $VENV_DIR（详见 $VENV_DIR/.venv-create.log）"
            fi
        fi
        VENV_PY="$VENV_DIR/bin/python"
    fi
fi

if [ -x "$VENV_PY" ]; then
    VENV_PY_VERSION="$("$VENV_PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    [ "$VENV_PY_VERSION" = "$PROFILE_PY_VERSION" ] || die \
        "venv 解释器版本 $VENV_PY_VERSION 与 profile $PROFILE（需要 $PROFILE_PY_VERSION）不符；请换 --venv 或重建 venv"
fi

if [ -n "${CONDA_PREFIX:-}" ]; then
    echo "setup_env: 检测到 conda 环境（CONDA_PREFIX=$CONDA_PREFIX）"
    echo "setup_env: 仍在独立 venv（$VENV_DIR）内安装，不修改 conda / 系统 site-packages"
fi

# ── 3b. pip 能力探测（venv 自带 pip，或用宿主 pip --python 兜底）──────────
PIP_CMD=("$VENV_PY" -m pip)
if ! "$VENV_PY" -m pip --version >/dev/null 2>&1; then
    # venv 里没有 pip：宿主 pip 必须 ≥ 22.3（--python 选项）
    host_pip_version="$("$PYTHON" -m pip --version 2>/dev/null | awk '{print $2}')" || host_pip_version=""
    [ -n "$host_pip_version" ] || die "venv 内没有 pip，且宿主 pip 不可用；请手动为 $VENV_DIR 装好 pip"
    host_pip_major="$(echo "$host_pip_version" | cut -d. -f1)"
    host_pip_minor="$(echo "$host_pip_version" | cut -d. -f2)"
    if [ "$host_pip_major" -lt 22 ] || { [ "$host_pip_major" -eq 22 ] && [ "$host_pip_minor" -lt 3 ]; }; then
        die "宿主 pip $host_pip_version 过旧（需 ≥ 22.3 支持 --python），无法向无 pip 的 venv 安装"
    fi
    PIP_CMD=("$PYTHON" -m pip --python "$VENV_PY")
    echo "setup_env: venv 内无 pip，改用宿主 pip $host_pip_version --python 注入"
fi

# ── 4. 依赖清单（唯一真相源：versions.lock）────────────────────────────────
INFO="$("$PYTHON" "$REPO_ROOT/tools/wheels_lock.py" --lock "$LOCK_FILE" profile-info --profile "$PROFILE_KEY")" \
    || die "读取 $LOCK_FILE 失败"
eval "$INFO"
[ -n "${REQUIREMENTS:-}" ] || die "versions.lock 未给出任何依赖（profile=$PROFILE_KEY）"

# 注意：pip 的 --require-hashes 要求**每一条 requirement 自身**带 --hash，
# 只在 constraints 文件里写 hash 会被 pip 判为「hash 缺失」。
# 本项目的 constraints/*.txt 每一行都是 `name==version --hash=sha256:...`，
# 因此它同时就是一份 hash 钉死的 requirements 文件，直接用 -r 传入即可。
PIP_ARGS=(--requirement "$CONSTRAINTS" --require-hashes)
if [ "$OFFLINE" -eq 1 ]; then
    PIP_ARGS+=(--no-index --find-links "$WHEELS_DIR")
fi

DRAKE_REQ=""
for req in "${REQUIREMENTS[@]}"; do
    case "$req" in
        drake==*) DRAKE_REQ="$req" ;;
    esac
done
echo "setup_env: profile=$PROFILE（$PROFILE_KEY，python $PROFILE_PY_VERSION，drake ${DRAKE_REQ:-?}）"
echo "setup_env: venv=$VENV_DIR"
echo "setup_env: constraints=$CONSTRAINTS"
echo "setup_env: 离线模式=$([ "$OFFLINE" -eq 1 ] && echo yes || echo no)"

# ── 5. pip 升级（离线跳过）────────────────────────────────────────────────
if [ "$OFFLINE" -eq 1 ]; then
    echo "setup_env: 离线模式，跳过 pip 升级"
else
    run "${PIP_CMD[@]}" install --upgrade pip
fi

# ── 6. 安装依赖闭包 ───────────────────────────────────────────────────────
# 依赖清单来自 constraints 文件本身（每行已含 name==version + --hash），
# 因此不再额外传 requirement 名单：裸名单没有 hash，会让 --require-hashes 直接失败。
run "${PIP_CMD[@]}" install "${PIP_ARGS[@]}"

# ── 7. 自检 ───────────────────────────────────────────────────────────────
if [ "$DRY_RUN" -eq 1 ]; then
    printf '+ %s %s --check --lock %s\n' "$VENV_PY" "$REPO_ROOT/tools/verify_env.py" "$LOCK_FILE"
    echo "setup_env: (dry-run) 计划执行自检、打印关键包版本对照"
    echo "setup_env: (dry-run) 未做任何改动"
    exit 0
fi

echo "setup_env: 自检 tools/verify_env.py --check"
set +e
"$VENV_PY" "$REPO_ROOT/tools/verify_env.py" --check --lock "$LOCK_FILE"
VERIFY_RC=$?
set -e
if [ "$VERIFY_RC" -ne 0 ]; then
    echo "setup_env: 自检未通过（退出码 $VERIFY_RC），环境与 versions.lock 不一致" >&2
    echo "setup_env: wall time: $(( SECONDS - START_TS ))s" >&2
    exit "$VERIFY_RC"
fi

# ── 8. 版本对照输出 ───────────────────────────────────────────────────────
echo "setup_env: 关键包版本（venv 内实装）"
"$VENV_PY" - <<'PY' || true
import importlib.metadata as md
for name in ("drake", "robotic_arm", "mosek", "numpy", "scipy"):
    try:
        print(f"  {name}=={md.version(name)}")
    except md.PackageNotFoundError:
        print(f"  {name}: (未安装)")
PY

echo "setup_env: OK —— 环境已按 versions.lock 重建"
echo "setup_env: wall time: $(( SECONDS - START_TS ))s（目标：新机 30 min 内完成）"
