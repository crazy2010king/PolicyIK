"""rmctl —— 双臂工程操作入口（设计 §6）。"""
__version__ = "0.1.0"

# 厂商 nanobind wheel 需要 GLIBCXX_3.4.31+（见 vendor_env 模块说明）。
# 这里在**任何** numpy / 厂商库被 import 之前预加载仓库自带的
# .deps/libstdcxx（只读引用）：裸环境（未 source scripts/rmenv.sh，即没有
# LD_LIBRARY_PATH）也能 import tesseract_robotics；加载失败时静默降级，
# 由调用方沿用「厂商内核不可用 → NEEDS_HUMAN + next_action」路径。
from .vendor_env import ensure_libstdcxx as _ensure_libstdcxx

_ensure_libstdcxx()
