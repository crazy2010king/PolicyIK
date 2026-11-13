"""pytest 全局配置：把 ``src/`` 注入 ``sys.path``。

这样测试无需 ``pip install -e .`` 即可 ``import rm75_drake``（CI 的 fast 档也依赖这一点）。
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = REPO_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
