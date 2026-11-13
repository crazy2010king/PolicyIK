#!/usr/bin/env bash
#
# lock_deps.sh —— 依赖锁与离线 wheel 镜像归档的唯一入口（T03）
#
#   --download      按 constraints/<profile>.txt 把完整依赖闭包下载到 vendor/wheels/
#   --verify        用 sha256sum -c 全量校验归档与 SHA256SUMS 一致
#   --write-lock    把 SHA256SUMS 的「文件名 → sha256」映射回写 versions.lock:wheels
#
# 设计约束
#   * vendor/wheels/ 是**本机离线镜像**，已被 .gitignore 忽略（vendor/）。
#     入 git 的是锁文件 + 校验清单 + 本脚本，而不是二进制。
#   * 依赖闭包必须**完整钉死**：constraints 里每个包都带 --hash，
#     pip 因此进入 require-hashes 模式，任何未钉版本的传递依赖都会直接报错。
#     传递依赖的实测版本已回写进 versions.lock:packages（T03 实测）。
#   * 目标平台标签来自 versions.lock:platform（platform_tag + extra_platform_tags），
#     脚本内不硬编码第二份版本/平台真相源。
#
# 退出码：0 成功 / 1 校验失败 / 2 运行期错误。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
LOCK_FILE="${LOCK_FILE:-$REPO_ROOT/versions.lock}"
WHEELS_DIR="${WHEELS_DIR:-$REPO_ROOT/vendor/wheels}"
PROFILE_KEY="baseline"
DURATION=0

usage() {
    cat <<'EOF'
用法：bash tools/lock_deps.sh <子命令> [选项]

子命令
  --download            按 constraints 下载完整依赖闭包到 vendor/wheels/（需联网）
  --verify              校验归档与 SHA256SUMS 一致（sha256sum -c / 内核兜底）
  --write-lock          把 SHA256SUMS 回写进 versions.lock 的 wheels 段

选项
  --profile py312|py310 目标 profile（默认 py312 = baseline 受支持基线；
                        py310 = fallback 非受支持降级档）
  --dest DIR            归档目录（默认 vendor/wheels）
  --lock FILE           锁文件路径（默认 versions.lock）
  -h, --help            显示本帮助

示例
  bash tools/lock_deps.sh --download --profile py312
  bash tools/lock_deps.sh --verify
  bash tools/lock_deps.sh --write-lock
EOF
}

die() {
    echo "lock_deps: $*" >&2
    exit 2
}

# profile 名（py312/py310）→ versions.lock 里的段名（baseline/fallback）
profile_to_key() {
    case "$1" in
        py312|baseline) echo "baseline" ;;
        py310|fallback) echo "fallback" ;;
        *) die "未知 profile: $1（可选 py312 / py310）" ;;
    esac
}

require_pip() {
    command -v "$PYTHON" >/dev/null 2>&1 || die "找不到解释器: $PYTHON"
    local version
    version="$("$PYTHON" -m pip --version 2>/dev/null | awk '{print $2}')" || version=""
    [ -n "$version" ] || die "pip 不可用（$PYTHON -m pip --version 失败）"
    local major minor
    major="$(echo "$version" | cut -d. -f1)"
    minor="$(echo "$version" | cut -d. -f2)"
    if [ "$major" -lt 22 ] 2>/dev/null; then
        die "pip ≥ 22 必需（当前 $version）：--platform/--python-version 参数需要 pip ≥ 22"
    fi
    if [ "$major" -eq 22 ] && [ "$minor" -lt 3 ]; then
        die "pip ≥ 22.3 必需（当前 $version）：多 --platform 参数需要 pip ≥ 22.3"
    fi
    echo "$version"
}

cmd_download() {
    local pip_version
    pip_version="$(require_pip)"

    local constraints="$REPO_ROOT/constraints/$PROFILE_NAME.txt"
    [ -f "$constraints" ] || die "缺少约束文件: $constraints"

    # 从 versions.lock 取下载参数（唯一真相源）
    local info
    info="$("$PYTHON" "$REPO_ROOT/tools/wheels_lock.py" --lock "$LOCK_FILE" \
        profile-info --profile "$PROFILE_KEY")" || die "读取 versions.lock 失败"

    local PROFILE PYTHON_VERSION PYTHON_TAG PLATFORMS
    local -a REQUIREMENTS
    eval "$info"
    [ -n "${PROFILE:-}" ] || die "profile-info 输出异常"

    local -a pip_args=()
    for platform in "${PLATFORMS[@]}"; do
        pip_args+=(--platform "$platform")
    done
    [ "${#PLATFORMS[@]}" -gt 0 ] || die "versions.lock:platform 未给出任何平台标签"

    mkdir -p "$WHEELS_DIR"
    echo "lock_deps: 开始下载 profile=$PROFILE (python $PYTHON_VERSION, tag $PYTHON_TAG)"
    echo "lock_deps: 目标平台标签: ${PLATFORMS[*]}"
    echo "lock_deps: 约束文件: $constraints"

    local start=$SECONDS
    "$PYTHON" -m pip download \
        --only-binary :all: \
        "${pip_args[@]}" \
        --python-version "$PYTHON_VERSION" \
        --implementation cp \
        --abi "$PYTHON_TAG" \
        --constraint "$constraints" \
        --dest "$WHEELS_DIR" \
        "${REQUIREMENTS[@]}" || die "pip download 失败（网络/解析/哈希问题，见上方 pip 输出）"
    DURATION=$(( SECONDS - start ))

    "$PYTHON" "$REPO_ROOT/tools/wheels_lock.py" --dir "$WHEELS_DIR" sha256sums \
        || die "生成 SHA256SUMS 失败"

    "$PYTHON" "$REPO_ROOT/tools/wheels_lock.py" --dir "$WHEELS_DIR" --lock "$LOCK_FILE" \
        archive-info --profile "$PROFILE_KEY" \
        --duration "$DURATION" \
        --timestamp "$(date -Is)" \
        --pip-version "$pip_version" \
        --note "本次耗时受 pip HTTP 缓存冷热影响：热缓存下同批发行文件数秒即可落盘，冷缓存才是真实下载耗时。" \
        --note "pip 以 require-hashes 模式解析（constraints 中每个包都带 --hash），任何未钉版本的传递依赖都会直接失败。" \
        || die "生成 ARCHIVE_INFO.md 失败"

    echo "lock_deps: 下载完成，耗时 ${DURATION}s"
    echo "lock_deps: 下一步：bash tools/lock_deps.sh --write-lock"
}

cmd_verify() {
    if [ ! -d "$WHEELS_DIR" ]; then
        die "归档目录不存在: $WHEELS_DIR（先跑 --download；本机镜像不入 git）"
    fi
    if [ ! -f "$WHEELS_DIR/SHA256SUMS" ]; then
        die "缺少校验清单: $WHEELS_DIR/SHA256SUMS"
    fi
    if command -v sha256sum >/dev/null 2>&1; then
        ( cd "$WHEELS_DIR" && sha256sum -c SHA256SUMS ) || exit 1
        echo "lock_deps: 归档校验通过（sha256sum -c）"
        return 0
    fi
    echo "lock_deps: 未找到 sha256sum，改用内核校验" >&2
    "$PYTHON" "$REPO_ROOT/tools/wheels_lock.py" --dir "$WHEELS_DIR" verify
}

cmd_write_lock() {
    "$PYTHON" "$REPO_ROOT/tools/wheels_lock.py" --dir "$WHEELS_DIR" --lock "$LOCK_FILE" write-lock \
        || die "回写 versions.lock 失败"
}

ACTION=""
while [ $# -gt 0 ]; do
    case "$1" in
        --download) ACTION="download" ;;
        --verify) ACTION="verify" ;;
        --write-lock) ACTION="write-lock" ;;
        --profile)
            shift || die "--profile 缺少取值"
            PROFILE_NAME="$1"
            PROFILE_KEY="$(profile_to_key "$1")"
            ;;
        --dest)
            shift || die "--dest 缺少取值"
            WHEELS_DIR="$1"
            ;;
        --lock)
            shift || die "--lock 缺少取值"
            LOCK_FILE="$1"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            die "未知参数: $1"
            ;;
    esac
    shift || true
done

PROFILE_NAME="${PROFILE_NAME:-py312}"
[ -n "$ACTION" ] || { usage >&2; die "缺少子命令"; }

case "$ACTION" in
    download) cmd_download ;;
    verify) cmd_verify ;;
    write-lock) cmd_write_lock ;;
esac
