"""rm_hw_bridge —— 厂商解耦的 RM-API2 硬件适配层。

本包只做「把厂商 C ABI 固化成可校验的机器可读规格，并据此生成 ctypes 绑定」，
**零 rmctl 依赖**（硬件层不得反向依赖体验层），依赖仅 ``pyyaml``（可选 numpy）。
**禁止** import ``tesseract`` / ``Robotic_Arm`` 等厂商封装。

ABI 唯一真值：``rm_hw_bridge/abi_spec.yaml``；
生成物：``rm_hw_bridge/generated/rm_api2_ctypes.py`` + ``layout_golden.json``。
"""
from __future__ import annotations

import os

__all__ = [
    "__version__",
    "ABI_SPEC_PATH",
    "CONTRACT_PATH",
    "GENERATED_DIR",
    "GENERATED_MODULE_PATH",
    "LAYOUT_GOLDEN_PATH",
]

__version__ = "0.1.0"

_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

#: ABI 唯一真值（机器可读规格）
ABI_SPEC_PATH = os.path.join(_PACKAGE_DIR, "abi_spec.yaml")
#: 生成物目录
GENERATED_DIR = os.path.join(_PACKAGE_DIR, "generated")
#: 生成的 ctypes 绑定
GENERATED_MODULE_PATH = os.path.join(GENERATED_DIR, "rm_api2_ctypes.py")
#: 布局黄金表
LAYOUT_GOLDEN_PATH = os.path.join(GENERATED_DIR, "layout_golden.json")
#: 符号契约（调用许可清单：core 22 / support / blacklist 的唯一真值）
CONTRACT_PATH = os.path.join(_PACKAGE_DIR, "contract.yaml")
