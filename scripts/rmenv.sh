#!/usr/bin/env bash
# rmctl 运行环境（零编译路线）
#   usage: source scripts/rmenv.sh
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RM_WS="$ROOT"
export PYTHONPATH="$ROOT/dualarm_ws/src/rmctl:${PYTHONPATH:-}"
# nanobind 0.35 wheel 需要 GLIBCXX_3.4.31+（本地系统是 3.4.30），用随包的 conda-forge libstdc++ 补齐
export LD_LIBRARY_PATH="$ROOT/.deps/libstdcxx:${LD_LIBRARY_PATH:-}"
export PATH="$HOME/.local/bin:$PATH"
echo "RM_WS=$RM_WS"
echo "PYTHONPATH=$PYTHONPATH"
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
