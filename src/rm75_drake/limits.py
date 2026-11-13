"""关节限位交接层（S01-T06）。

职责边界
--------
* **限位真值**放在 ``models/rm75_drake/joint_limits.yaml``（入 git，是交 S03
  安全闸的交接件）；本模块只做**读取与一致性校验**；
* 硬件限位 = 官方 RM75 参数表（位置/速度）+ 官方 RM75-B URDF 的 ``effort``；
* **软件收紧限位** = 硬件限位每侧再收 4°（0.0698132 rad），是本项目给下游
  （S03 安全闸 / S02 规划）使用的**可执行软限位**。

为什么需要独立一层
------------------
模型真值有**两处**落点（URDF 与 YAML），二者一旦漂移，下游会拿到过期限位。
本模块提供唯一的校验函数 :func:`check_joint_limits`：

* YAML 的 ``hardware`` 必须与 URDF 声明逐条相等（否则交接件过期）；
* YAML 的 ``software`` 必须恰好比 ``hardware`` 每侧窄一个 margin
  （否则"收紧"规则被悄悄改掉）；
* ``software`` 必须严格落在 ``hardware`` 之内（安全闸不允许越界）。

校验失败一律返回**问题列表**（不抛异常、不静默通过），由调用方决定退出码。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from . import paths

__all__ = [
    "SOFTWARE_MARGIN_RAD",
    "SOFTWARE_MARGIN_DEG",
    "JOINT_LIMITS_RELATIVE",
    "LimitsError",
    "joint_limits_path",
    "load_joint_limits",
    "expected_software_limits",
    "check_joint_limits",
]


#: 软件收紧限位的每侧余量（rad）= 4°。
SOFTWARE_MARGIN_RAD = 0.0698132

#: 同一余量的角度表示（deg）。
SOFTWARE_MARGIN_DEG = 4.0

#: 限位交接件相对仓库根的路径（入 git）。
JOINT_LIMITS_RELATIVE = "models/rm75_drake/joint_limits.yaml"

#: 数值比较容差（rad / rad/s / N·m）。
_TOL = 1e-9

#: 期望的关节名顺序（RM75-B = 7 DoF）。
_EXPECTED_JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 8))


class LimitsError(RuntimeError):
    """限位交接件不可读 / 结构非法。"""


def joint_limits_path(path: Optional[Path] = None) -> Path:
    """返回限位交接件路径（默认 ``models/rm75_drake/joint_limits.yaml``）。"""
    if path is not None:
        return Path(path)
    return paths.models_dir / "rm75_drake" / "joint_limits.yaml"


def load_joint_limits(path: Optional[Path] = None) -> Dict[str, Any]:
    """读取限位交接件 YAML。

    文件缺失或 YAML 解析失败抛 :class:`LimitsError`（**不**静默返回空字典）。
    """
    target = joint_limits_path(path)
    if not target.is_file():
        raise LimitsError(f"限位交接件不存在：{target}")
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺依赖
        raise LimitsError("需要 PyYAML（pip install pyyaml）") from exc
    try:
        data = yaml.safe_load(target.read_text(encoding="utf-8"))
    except Exception as exc:
        raise LimitsError(f"限位交接件解析失败：{target}（{exc}）") from exc
    if not isinstance(data, dict):
        raise LimitsError(f"限位交接件顶层必须是映射：{target}")
    return data


def expected_software_limits(
    lower: float,
    upper: float,
    margin: float = SOFTWARE_MARGIN_RAD,
) -> Dict[str, float]:
    """按"每侧收紧 margin"计算软件限位。"""
    return {"position_lower_rad": lower + margin, "position_upper_rad": upper - margin}


def _close(a: Any, b: Any, tol: float = _TOL) -> bool:
    try:
        return abs(float(a) - float(b)) <= tol
    except (TypeError, ValueError):
        return False


def check_joint_limits(
    limits: Dict[str, Any],
    urdf_limits: Optional[Sequence[Dict[str, Any]]] = None,
    margin: float = SOFTWARE_MARGIN_RAD,
) -> List[str]:
    """校验限位交接件，返回问题列表（空列表 = 一致）。

    参数
    ----
    limits:
        :func:`load_joint_limits` 的返回值。
    urdf_limits:
        可选的 URDF 声明限位（``models.joint_limits_from_urdf()`` 的返回值）；
        给出时做**逐条交叉核对**（交接件过期检测）。
    margin:
        软件收紧余量（rad）；默认 :data:`SOFTWARE_MARGIN_RAD`。
    """
    problems: List[str] = []

    if limits.get("schema_version") != 1:
        problems.append(
            f"schema_version 必须是 1，实际是 {limits.get('schema_version')!r}"
        )

    source = limits.get("source")
    if not isinstance(source, dict):
        problems.append("缺少 source 段（必须标注官方参数表与上游 commit）")
    else:
        commit = str(source.get("upstream_commit", ""))
        if len(commit) != 40 or not all(c in "0123456789abcdef" for c in commit):
            problems.append(f"source.upstream_commit 不是 40 位 hex：{commit!r}")

    sw = limits.get("software_limits")
    declared_margin = margin
    if not isinstance(sw, dict):
        problems.append("缺少 software_limits 段")
    else:
        declared_margin = float(sw.get("margin_rad", margin))
        if not _close(declared_margin, margin, 1e-6):
            problems.append(
                f"software_limits.margin_rad={declared_margin} 与项目约定 {margin} 不一致"
            )

    joints = limits.get("joints")
    if not isinstance(joints, list) or not joints:
        problems.append("缺少 joints 段（必须是列表）")
        return problems

    names = [str(j.get("name")) for j in joints if isinstance(j, dict)]
    if tuple(names) != _EXPECTED_JOINT_NAMES:
        problems.append(
            f"joints 必须按顺序包含 {_EXPECTED_JOINT_NAMES}，实际是 {tuple(names)}"
        )

    urdf_by_name: Dict[str, Dict[str, Any]] = {}
    for item in urdf_limits or ():
        urdf_by_name[str(item.get("name"))] = dict(item)

    for joint in joints:
        if not isinstance(joint, dict):
            problems.append(f"joints 里出现非映射元素：{joint!r}")
            continue
        name = str(joint.get("name", "<未命名>"))
        hardware = joint.get("hardware")
        software = joint.get("software")
        if not isinstance(hardware, dict) or not isinstance(software, dict):
            problems.append(f"{name}: 必须同时有 hardware 与 software 段")
            continue

        h_lower = hardware.get("position_lower_rad")
        h_upper = hardware.get("position_upper_rad")
        h_vel = hardware.get("velocity_rad_s")
        h_eff = hardware.get("effort_nm")
        for key, value in (
            ("position_lower_rad", h_lower),
            ("position_upper_rad", h_upper),
            ("velocity_rad_s", h_vel),
            ("effort_nm", h_eff),
        ):
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                problems.append(f"{name}.hardware.{key} 必须是数值，实际是 {value!r}")
        if not isinstance(h_lower, (int, float)) or not isinstance(h_upper, (int, float)):
            continue
        if not float(h_lower) < 0.0 < float(h_upper):
            problems.append(
                f"{name}.hardware 位置限位必须跨过 0：[{h_lower}, {h_upper}]"
            )
        if isinstance(h_vel, (int, float)) and float(h_vel) <= 0.0:
            problems.append(f"{name}.hardware.velocity_rad_s 必须为正：{h_vel!r}")
        if isinstance(h_eff, (int, float)) and float(h_eff) <= 0.0:
            problems.append(f"{name}.hardware.effort_nm 必须为正：{h_eff!r}")

        s_lower = software.get("position_lower_rad")
        s_upper = software.get("position_upper_rad")
        for key, value in (
            ("position_lower_rad", s_lower),
            ("position_upper_rad", s_upper),
        ):
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                problems.append(f"{name}.software.{key} 必须是数值，实际是 {value!r}")
        if not isinstance(s_lower, (int, float)) or not isinstance(s_upper, (int, float)):
            continue

        expected = expected_software_limits(float(h_lower), float(h_upper), declared_margin)
        if not _close(s_lower, expected["position_lower_rad"], 1e-6):
            problems.append(
                f"{name}.software.position_lower_rad={s_lower} 与"
                f"（硬件 {h_lower} + {declared_margin}）={expected['position_lower_rad']} 不一致"
            )
        if not _close(s_upper, expected["position_upper_rad"], 1e-6):
            problems.append(
                f"{name}.software.position_upper_rad={s_upper} 与"
                f"（硬件 {h_upper} - {declared_margin}）={expected['position_upper_rad']} 不一致"
            )
        if not float(h_lower) < float(s_lower) < float(s_upper) < float(h_upper):
            problems.append(
                f"{name}.software 位置限位必须严格落在 hardware 之内："
                f"{[h_lower, h_upper]} vs {[s_lower, s_upper]}"
            )

        s_vel = software.get("velocity_rad_s")
        if isinstance(s_vel, (int, float)) and isinstance(h_vel, (int, float)):
            if float(s_vel) > float(h_vel) + 1e-12:
                problems.append(
                    f"{name}.software.velocity_rad_s={s_vel} 不得大于硬件 {h_vel}"
                )
        s_eff = software.get("effort_nm")
        if isinstance(s_eff, (int, float)) and isinstance(h_eff, (int, float)):
            if float(s_eff) > float(h_eff) + 1e-12:
                problems.append(
                    f"{name}.software.effort_nm={s_eff} 不得大于硬件 {h_eff}"
                )

        # 与 URDF 声明逐条交叉核对（交接件过期检测）
        urdf_item = urdf_by_name.get(name)
        if urdf_item:
            pairs = (
                ("position_lower_rad", urdf_item.get("lower")),
                ("position_upper_rad", urdf_item.get("upper")),
                ("velocity_rad_s", urdf_item.get("velocity")),
                ("effort_nm", urdf_item.get("effort")),
            )
            for key, urdf_value in pairs:
                hw_value = hardware.get(key)
                if urdf_value is None or not isinstance(hw_value, (int, float)):
                    continue
                if not _close(hw_value, urdf_value, 1e-6):
                    problems.append(
                        f"{name}.hardware.{key}={hw_value} 与 URDF 声明 {urdf_value} 不一致"
                    )

    return problems
