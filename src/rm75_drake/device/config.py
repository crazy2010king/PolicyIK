"""设备（部署）配置加载与严格校验（S03-T01）。

为什么**另立**设备配置
----------------------
``scenario.yaml`` 的 schema 在 S02 已经**冻结**（顶层四键 + ``robots[]`` 严格
子键白名单），把真机连接信息（IP / 端口 / UDP 周期 / 零位偏移 / 围栏 / 阈值）
塞进去要么升 ``schema_version``（会击穿 S07 的 golden-file 基线），要么破坏
schema 纪律。研究结论 F8：**设备连接信息是部署配置，不是场景语义** ⇒ 另立
``devices/*.yaml``，由本模块加载与校验。

校验纪律（**沿用** :mod:`rm75_drake.scenario` 的做法）
---------------------------------------------------
* 未知键 → :class:`DeviceConfigError`，消息里**点名该键**；
* 缺必填字段 → :class:`DeviceConfigError`，消息里**点名该字段**；
* 数值越界 / 量纲错 / 维度不对 → :class:`DeviceConfigError`，点名该键；
* **禁止静默默认值**：唯一允许的兜底是 ``accel_limits`` 与
  ``current_thresholds`` 两段**整段缺失**时的保守默认，且必须同时把
  ``accel_limits_measured=False`` / ``current_thresholds_calibrated=False``
  暴露给调用方（下游必须知道这是**未标定**的兜底值，不得当作实测真值）。

单位约定
--------
========  ==========================================================
量纲      约定
========  ==========================================================
角度      **rad（弧度）**。关节零位偏移一律弧度。
加速度    rad/s²。
电流      mA。
长度      m。
时间      s。
========  ==========================================================

UDP ``cycle`` 的**两种读法**（研究 F4，必须现场标定）
--------------------------------------------------
官方 SDK 的 ``rm_realtime_push_config_t.cycle`` 文档写「广播周期，5 ms 的倍数」，
而官方 ``ros2_rm_robot/rm_driver`` 源码证据是 ``config.cycle = udp_cycle / 5``
⇒ **cycle 的单位是 5 ms**（5 ms 上报周期对应 ``cycle=1``）；
另一种读法是「cycle 的单位是毫秒」（本项目设计文档写的是 ``cycle=5`` ⇒ 5 ms）。
两种读法相差 5 倍，直接影响 age 阈值与抖动口径 ⇒
``udp_cycle`` 必须是**配置项**，并且 :meth:`DeviceConfig.udp_period_candidates_s`
同时给出两种读法的候选周期，真机首件事是实测周期分布。
"""

from __future__ import annotations

import ipaddress
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .. import paths
from ..planning.trajectory import DEFAULT_ACCELERATION_LIMITS_RAD_S2, JOINT_COUNT

__all__ = [
    "DeviceConfigError",
    "SCHEMA_VERSION",
    "TOP_LEVEL_KEYS",
    "REQUIRED_TOP_LEVEL_KEYS",
    "ACCEL_LIMIT_KEYS",
    "CURRENT_THRESHOLD_KEYS",
    "FENCE_KEYS",
    "CONSERVATIVE_CURRENT_THRESHOLDS_MA",
    "DeviceConfig",
    "default_device_config_path",
    "load_device_config",
    "device_config_from_dict",
]


class DeviceConfigError(ValueError):
    """设备配置 schema 校验失败（未知键 / 缺字段 / 越界 / 量纲错）。"""


#: 当前设备配置 schema 版本。
SCHEMA_VERSION = 1

#: 顶层键（顺序即 YAML 里的书写顺序）。
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "name",
    "arm_ip",
    "tcp_port",
    "udp_port",
    "udp_cycle",
    "joint_zero_offset",
    "accel_limits",
    "current_thresholds",
    "fence",
)

#: 顶层**必填**键（``accel_limits`` 与 ``current_thresholds`` 两段允许整段缺失：
#: 缺失时使用保守兜底并显式标注 ``measured=False`` / ``calibrated=False``）。
REQUIRED_TOP_LEVEL_KEYS: Tuple[str, ...] = tuple(
    key for key in TOP_LEVEL_KEYS if key not in ("accel_limits", "current_thresholds")
)

#: ``accel_limits`` 段的合法子键。
ACCEL_LIMIT_KEYS: Tuple[str, ...] = ("measured", "values")

#: ``current_thresholds`` 段的合法子键。
CURRENT_THRESHOLD_KEYS: Tuple[str, ...] = ("calibrated", "values")

#: ``fence`` 段的合法子键。
FENCE_KEYS: Tuple[str, ...] = ("table_height_m", "singularity_radius_m")

#: 电流阈值**保守兜底**（mA）——厂商未公布逐轴电流上限，缺数据时用它，
#: 并把 ``current_thresholds_calibrated`` 标成 ``False``（未标定）。
#: 大关节（J1/J2）取 1500 mA、中关节（J3/J4）1200 mA、小关节（J5–J7）1000 mA。
CONSERVATIVE_CURRENT_THRESHOLDS_MA: Tuple[float, ...] = (
    1500.0,
    1500.0,
    1200.0,
    1200.0,
    1000.0,
    1000.0,
    1000.0,
)

#: TCP 端口合法区间（含）。
_PORT_MIN = 1
_PORT_MAX = 65535

#: 桌面高度的合法上界（m）——超过视为量纲写错（填了毫米）。
_TABLE_HEIGHT_LIMIT_M = 2.0


# ── 内部校验工具 ────────────────────────────────────────────────────────
def _require_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise DeviceConfigError(f"{path} 必须是映射（mapping），收到 {type(value).__name__}")
    for key in value:
        if not isinstance(key, str):
            raise DeviceConfigError(f"{path} 的键必须是字符串，收到 {key!r}")
    return value


def _check_keys(
    mapping: Mapping[str, Any],
    path: str,
    allowed: Sequence[str],
    required: Sequence[str],
) -> None:
    """未知键 / 缺必填键都抛 :class:`DeviceConfigError`（点名键名）。"""
    unknown = [k for k in mapping if k not in allowed]
    if unknown:
        raise DeviceConfigError(
            f"{path} 出现未知键：{sorted(unknown)}；该处允许的键只有 {list(allowed)}"
        )
    missing = [k for k in required if k not in mapping]
    if missing:
        raise DeviceConfigError(
            f"{path} 缺少必填字段：{missing}；该处允许的键只有 {list(allowed)}"
        )


def _as_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DeviceConfigError(f"{path} 必须是数值，收到 {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise DeviceConfigError(f"{path} 必须是有限数值，收到 {value!r}")
    return out


def _as_positive_float(value: Any, path: str) -> float:
    out = _as_float(value, path)
    if out <= 0.0:
        raise DeviceConfigError(f"{path} 必须是正数，收到 {value!r}")
    return out


def _as_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DeviceConfigError(f"{path} 必须是整数，收到 {value!r}")
    return int(value)


def _as_bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise DeviceConfigError(f"{path} 必须是布尔值（true/false），收到 {value!r}")
    return value


def _as_float_vector(value: Any, path: str, length: int) -> Tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise DeviceConfigError(f"{path} 必须是 {length} 维序列，收到 {type(value).__name__}")
    if len(value) != length:
        raise DeviceConfigError(
            f"{path} 必须是 {length} 维序列（实际 {len(value)} 维）"
        )
    return tuple(_as_float(v, f"{path}[{i}]") for i, v in enumerate(value))


def _as_positive_float_vector(value: Any, path: str, length: int) -> Tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise DeviceConfigError(f"{path} 必须是 {length} 维序列，收到 {type(value).__name__}")
    if len(value) != length:
        raise DeviceConfigError(
            f"{path} 必须是 {length} 维序列（实际 {len(value)} 维）"
        )
    return tuple(_as_positive_float(v, f"{path}[{i}]") for i, v in enumerate(value))


def _as_port(value: Any, path: str) -> int:
    port = _as_int(value, path)
    if not _PORT_MIN <= port <= _PORT_MAX:
        raise DeviceConfigError(
            f"{path} 必须是 {_PORT_MIN}–{_PORT_MAX} 之间的端口，收到 {port}"
        )
    return port


# ── 数据结构 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class DeviceConfig:
    """一台真机的**部署配置**（全部字段已校验、单位统一）。

    字段
    ----
    name:
        设备名（写进运行日志 / 录制文件）。
    arm_ip:
        机械臂 IP（IPv4）。
    tcp_port:
        指令通道端口（官方默认 8080）。
    udp_port:
        状态主动上报（UDP）的**本机监听端口**。
    udp_cycle:
        UDP 上报周期参数（**两种读法**，见模块 docstring；须真机实测标定）。
    joint_zero_offset:
        关节零位偏移（rad，7 维）。
    accel_limits_rad_s2:
        每轴加速度上限（rad/s²，7 维）。
    accel_limits_measured:
        加速度上限是否**实测标定**过（False = 兜底/未标定）。
    current_thresholds_ma:
        每轴电流阈值（mA，7 维）。
    current_thresholds_calibrated:
        电流阈值是否**实测标定**过（False = 保守兜底）。
    fence_table_height_m:
        桌面高度（m）——围栏判定的下边界。
    fence_singularity_radius_m:
        奇异圆柱半径（m）——TCP 距基座轴线小于该值即判奇异/越界。
    source_path:
        配置文件来源路径（``None`` 表示来自内存字典）。
    """

    name: str
    arm_ip: str
    tcp_port: int
    udp_port: int
    udp_cycle: int
    joint_zero_offset: Tuple[float, ...]
    accel_limits_rad_s2: Tuple[float, ...]
    accel_limits_measured: bool
    current_thresholds_ma: Tuple[float, ...]
    current_thresholds_calibrated: bool
    fence_table_height_m: float
    fence_singularity_radius_m: float
    schema_version: int = SCHEMA_VERSION
    source_path: Optional[Path] = None

    # -- 派生量 ----------------------------------------------------------
    def udp_period_candidates_s(self) -> Tuple[float, float]:
        """返回 ``udp_cycle`` 的**两种读法**对应的上报周期（s）。

        返回
        ----
        ``(cycle * 0.005, cycle * 0.001)``
            第一个元素 = 「单位是 5 ms 的倍数」读法（官方 ``rm_driver`` 证据）；
            第二个元素 = 「单位是毫秒」读法（本项目设计文档读法）。
            两者相差 5 倍，真机首测必须实测周期分布后择一。
        """
        return (self.udp_cycle * 0.005, self.udp_cycle * 0.001)

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典（写进运行日志 / 排障输出）。"""
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "arm_ip": self.arm_ip,
            "tcp_port": self.tcp_port,
            "udp_port": self.udp_port,
            "udp_cycle": self.udp_cycle,
            "joint_zero_offset": list(self.joint_zero_offset),
            "accel_limits": {
                "measured": self.accel_limits_measured,
                "values": list(self.accel_limits_rad_s2),
            },
            "current_thresholds": {
                "calibrated": self.current_thresholds_calibrated,
                "values": list(self.current_thresholds_ma),
            },
            "fence": {
                "table_height_m": self.fence_table_height_m,
                "singularity_radius_m": self.fence_singularity_radius_m,
            },
        }


# ── 加载入口 ────────────────────────────────────────────────────────────
def default_device_config_path() -> Path:
    """返回默认设备配置路径（``devices/single_arm.yaml``）。"""
    return paths.devices_dir / "single_arm.yaml"


def load_device_config(path: Optional[Path] = None) -> DeviceConfig:
    """从 YAML 加载设备配置。

    参数
    ----
    path:
        配置文件路径；``None`` → :func:`default_device_config_path`。

    异常
    ----
    :class:`DeviceConfigError`
        文件不存在 / YAML 解析失败 / schema 非法（消息点名出错键名）。
    """
    target = Path(path) if path is not None else default_device_config_path()
    if not target.is_file():
        raise DeviceConfigError(f"设备配置文件不存在：{target}")
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺依赖
        raise DeviceConfigError("需要 PyYAML（pip install pyyaml）") from exc
    try:
        data = yaml.safe_load(target.read_text(encoding="utf-8"))
    except Exception as exc:
        raise DeviceConfigError(f"设备配置解析失败：{target}（{exc}）") from exc
    if not isinstance(data, Mapping):
        raise DeviceConfigError(f"设备配置顶层必须是映射：{target}")
    config = device_config_from_dict(data)
    return DeviceConfig(**{**config.__dict__, "source_path": target})


def device_config_from_dict(data: Mapping[str, Any]) -> DeviceConfig:
    """从（已解析的）映射构造 :class:`DeviceConfig`，逐项严格校验。"""
    mapping = _require_mapping(data, "设备配置")
    _check_keys(mapping, "设备配置", TOP_LEVEL_KEYS, REQUIRED_TOP_LEVEL_KEYS)

    version = _as_int(mapping["schema_version"], "schema_version")
    if version != SCHEMA_VERSION:
        raise DeviceConfigError(
            f"schema_version 必须是 {SCHEMA_VERSION}，实际是 {version}"
        )

    name = mapping["name"]
    if not isinstance(name, str) or not name.strip():
        raise DeviceConfigError(f"name 必须是非空字符串，收到 {name!r}")

    arm_ip = mapping["arm_ip"]
    if not isinstance(arm_ip, str) or not arm_ip.strip():
        raise DeviceConfigError(f"arm_ip 必须是非空字符串，收到 {arm_ip!r}")
    try:
        ipaddress.IPv4Address(arm_ip.strip())
    except ipaddress.AddressValueError as exc:
        raise DeviceConfigError(f"arm_ip 必须是合法 IPv4 地址，收到 {arm_ip!r}") from exc

    tcp_port = _as_port(mapping["tcp_port"], "tcp_port")
    udp_port = _as_port(mapping["udp_port"], "udp_port")

    udp_cycle = _as_int(mapping["udp_cycle"], "udp_cycle")
    if udp_cycle < 1:
        raise DeviceConfigError(f"udp_cycle 必须 ≥ 1，收到 {udp_cycle}")

    joint_zero_offset = _as_float_vector(
        mapping["joint_zero_offset"], "joint_zero_offset", JOINT_COUNT
    )

    accel_limits, accel_measured = _parse_accel_limits(mapping.get("accel_limits"))
    current_ma, current_calibrated = _parse_current_thresholds(
        mapping.get("current_thresholds")
    )
    table_height, singularity_radius = _parse_fence(mapping["fence"])

    return DeviceConfig(
        name=name.strip(),
        arm_ip=arm_ip.strip(),
        tcp_port=tcp_port,
        udp_port=udp_port,
        udp_cycle=udp_cycle,
        joint_zero_offset=joint_zero_offset,
        accel_limits_rad_s2=accel_limits,
        accel_limits_measured=accel_measured,
        current_thresholds_ma=current_ma,
        current_thresholds_calibrated=current_calibrated,
        fence_table_height_m=table_height,
        fence_singularity_radius_m=singularity_radius,
        schema_version=version,
    )


# ── 分段解析 ────────────────────────────────────────────────────────────
def _parse_accel_limits(section: Any) -> Tuple[Tuple[float, ...], bool]:
    """解析 ``accel_limits`` 段。

    整段缺失 → 用 S02 的自研默认（:data:`DEFAULT_ACCELERATION_LIMITS_RAD_S2`）
    并标 ``measured=False``（**未实测标定**，不得当作真值使用）。
    """
    if section is None:
        return tuple(float(v) for v in DEFAULT_ACCELERATION_LIMITS_RAD_S2), False
    mapping = _require_mapping(section, "accel_limits")
    _check_keys(mapping, "accel_limits", ACCEL_LIMIT_KEYS, ACCEL_LIMIT_KEYS)
    measured = _as_bool(mapping["measured"], "accel_limits.measured")
    values = _as_positive_float_vector(
        mapping["values"], "accel_limits.values", JOINT_COUNT
    )
    return values, measured


def _parse_current_thresholds(section: Any) -> Tuple[Tuple[float, ...], bool]:
    """解析 ``current_thresholds`` 段。

    整段缺失 → 用保守兜底 :data:`CONSERVATIVE_CURRENT_THRESHOLDS_MA`
    并标 ``calibrated=False``（**未标定**）。
    """
    if section is None:
        return CONSERVATIVE_CURRENT_THRESHOLDS_MA, False
    mapping = _require_mapping(section, "current_thresholds")
    _check_keys(
        mapping, "current_thresholds", CURRENT_THRESHOLD_KEYS, CURRENT_THRESHOLD_KEYS
    )
    calibrated = _as_bool(mapping["calibrated"], "current_thresholds.calibrated")
    values = _as_positive_float_vector(
        mapping["values"], "current_thresholds.values", JOINT_COUNT
    )
    return values, calibrated


def _parse_fence(section: Any) -> Tuple[float, float]:
    """解析 ``fence`` 段（桌面高度 + 奇异圆柱半径）。"""
    mapping = _require_mapping(section, "fence")
    _check_keys(mapping, "fence", FENCE_KEYS, FENCE_KEYS)
    table_height = _as_float(mapping["table_height_m"], "fence.table_height_m")
    if not 0.0 <= table_height <= _TABLE_HEIGHT_LIMIT_M:
        raise DeviceConfigError(
            f"fence.table_height_m 必须在 0–{_TABLE_HEIGHT_LIMIT_M} m 之间"
            f"（若填的是毫米请先换算），收到 {table_height}"
        )
    radius = _as_positive_float(
        mapping["singularity_radius_m"], "fence.singularity_radius_m"
    )
    return table_height, radius
