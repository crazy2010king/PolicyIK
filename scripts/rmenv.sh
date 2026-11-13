#!/usr/bin/env bash
# rmctl 运行环境（零编译路线）
#   usage: source scripts/rmenv.sh
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RM_WS="$ROOT"
# 硬件适配层 rm_hw_bridge（厂商解耦层）、安全层 rm_safety（失效安全停机四级链路）
# 与执行层 rm_executor（10 ms 单时钟执行器）也加入 PYTHONPATH
# （**追加**，不替换 rmctl）
export PYTHONPATH="$ROOT/dualarm_ws/src/rmctl:$ROOT/dualarm_ws/src/rm_hw_bridge:$ROOT/dualarm_ws/src/rm_safety:$ROOT/dualarm_ws/src/rm_executor:${PYTHONPATH:-}"
# nanobind 0.35 wheel 需要 GLIBCXX_3.4.31+（本机系统 libstdc++ 只有 3.4.30）。
# 优先用本 checkout 自带的 .deps/libstdcxx；缺失时按相对路径【只读】引用同一仓库
# 其它 checkout 的 .deps/libstdcxx（worktree 布局为 $ROOT/../../.deps/libstdcxx，
# 其次 $ROOT/../.deps/libstdcxx）。只引用、不拷贝、不修改被引用的文件。
RM_LIBSTDCXX_DIR=""
for _cand in "$ROOT/.deps/libstdcxx" "$ROOT/../../.deps/libstdcxx" "$ROOT/../.deps/libstdcxx"; do
  if [ -d "$_cand" ] && ls "$_cand"/libstdc++.so.6* >/dev/null 2>&1; then
    RM_LIBSTDCXX_DIR="$_cand"
    break
  fi
done
if [ -n "$RM_LIBSTDCXX_DIR" ]; then
  export LD_LIBRARY_PATH="$RM_LIBSTDCXX_DIR:${LD_LIBRARY_PATH:-}"
  if ! strings "$RM_LIBSTDCXX_DIR"/libstdc++.so.6* 2>/dev/null | grep -q 'GLIBCXX_3.4.31'; then
    echo "警告: $RM_LIBSTDCXX_DIR 未导出 GLIBCXX_3.4.31，nanobind 导入可能失败" >&2
    echo "      → 请换用 conda-forge 的 libstdcxx-ng（>= 3.4.31）放到 <repo>/.deps/libstdcxx" >&2
  fi
else
  echo "警告: 未找到可用的 .deps/libstdcxx（候选: \$ROOT/.deps/libstdcxx、" >&2
  echo "      \$ROOT/../../.deps/libstdcxx、\$ROOT/../.deps/libstdcxx）" >&2
  echo "      → nanobind 导入需要 GLIBCXX_3.4.31+，请安装 conda-forge libstdcxx-ng 并放到上述任一路径" >&2
fi
unset _cand
export PATH="$HOME/.local/bin:$PATH"
echo "RM_WS=$RM_WS"
echo "PYTHONPATH=$PYTHONPATH"
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "RM_LIBSTDCXX_DIR=$RM_LIBSTDCXX_DIR"
