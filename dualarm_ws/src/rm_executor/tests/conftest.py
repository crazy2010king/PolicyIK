"""rm_executor 测试共用夹具（pytest）。

只使用 git 跟踪的仓库文件 + pytest 临时目录；
绝不依赖 .gitignore 目录（``build/``、``.deps/``、``.gsd/``）。
"""
from __future__ import annotations

import json
import os
import sys

import pytest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.abspath(os.path.join(TESTS_DIR, ".."))
REPO_ROOT = os.path.abspath(os.path.join(TESTS_DIR, "..", "..", "..", ".."))
HW_BRIDGE_SRC = os.path.abspath(
    os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_hw_bridge")
)
SAFETY_SRC = os.path.abspath(
    os.path.join(REPO_ROOT, "dualarm_ws", "src", "rm_safety")
)

# 执行层包 + 硬件适配层（限值唯一数据源）+ 安全层（唯一动作权威）都在 sys.path 上
for _p in (PKG_SRC, HW_BRIDGE_SRC, SAFETY_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)


@pytest.fixture(scope="session")
def repo_root() -> str:
    """仓库根（workcell/ 与 dualarm_ws/ 都在这一层）。"""
    return REPO_ROOT


@pytest.fixture(scope="session")
def executor_pkg_root() -> str:
    """rm_executor 包根（dualarm_ws/src/rm_executor）。

    **刻意不叫 ``pkg_root``**：与 ``rm_hw_bridge/tests/conftest.py`` 的同名
    fixture 冲突时，session 级同名 fixture 会互相覆盖，导致合并跑两个包的
    pytest 时硬件层用例拿到错误的包根路径。
    """
    return PKG_SRC


@pytest.fixture(scope="session")
def workcell_config_path() -> str:
    """工位定义文件（执行配置唯一来源，git 跟踪）。"""
    return os.path.join(REPO_ROOT, "workcell", "rm_workcell.yaml")


@pytest.fixture(scope="session")
def joint_limits() -> dict:
    """声明限值（SI）—— 直接读 **git 跟踪** 的 joint_limits.yaml（不经 build/）。"""
    import yaml

    path = os.path.join(
        REPO_ROOT, "dualarm_ws", "src", "rm_description", "config",
        "joint_limits.yaml",
    )
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    group = data["rm75_arm"]
    out = {}
    for name in ("joint%d" % i for i in range(1, 8)):
        item = group[name]
        out[name] = {
            "max_speed": float(item["velocity"]),
            "max_acc": float(item["acceleration"]),
            "jerk": float(item["jerk"]),
            "min_pos": float(item["lower"]),
            "max_pos": float(item["upper"]),
        }
    return out


# ------------------------------------------------------------------ 夹具数据
def _smooth_trajectory(n: int = 21, amp: float = 0.1, freq: float = 0.5):
    """平滑的 7 关节小幅正弦轨迹（限值内，时间严格递增）。"""
    import math

    times = [round(i * 0.05, 6) for i in range(n)]
    points = []
    for t in times:
        base = amp * math.sin(2 * math.pi * freq * t)
        points.append([round(base * (1.0 + 0.1 * j), 9) for j in range(7)])
    return times, points


def _base_payload(times, points, **overrides):
    """构造一条合法轨迹 payload（可覆盖字段做负例）。"""
    payload = {
        "schema_version": 1,
        "group": "rm75_arm",
        "joint_names": ["joint%d" % i for i in range(1, 8)],
        "units": {"times": "s", "points": "rad"},
        "times": list(times),
        "points": [list(row) for row in points],
        "verify": {"ok": True, "checks": {}},
    }
    payload.update(overrides)
    return payload


@pytest.fixture()
def smooth_trajectory():
    """限值内的平滑轨迹 (times, points)。"""
    return _smooth_trajectory()


@pytest.fixture()
def base_payload(smooth_trajectory):
    """合法轨迹 payload 字典。"""
    times, points = smooth_trajectory
    return _base_payload(times, points)


@pytest.fixture()
def write_traj(tmp_path):
    """把 payload 写到 pytest 临时目录并返回路径。"""

    def _write(payload, name="traj.json"):
        path = tmp_path / name
        if isinstance(payload, str):
            path.write_text(payload, encoding="utf-8")
        else:
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        return str(path)

    return _write


@pytest.fixture()
def fake_ws(tmp_path):
    """构造一个最小「仓库根」布局的工作区（用于配置读取的正负例）。"""
    ws = tmp_path / "ws"
    (ws / "workcell").mkdir(parents=True)
    (ws / "dualarm_ws" / "src" / "rm_description").mkdir(parents=True)
    return ws
