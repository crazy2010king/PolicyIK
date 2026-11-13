"""IRIS 区域生成与凸区域缓存（S02-T04）。

职责
----
把「一个种子位形」变成「一个 C 空间凸区域（:class:`pydrake.geometry.optimization.HPolyhedron`）」，
并提供**可复用**的缓存结构（:class:`RegionCache`）——这是交给 S05（空间 GCS）的交接件。

为什么必须缓存
--------------
本机实测（Drake 1.51.1 / Python，场景 = ``scenarios/single_arm.yaml``）：
C 空间 IRIS（``IrisNp``）单个区域耗时 **8–18 s**（取决于
``num_collision_infeasible_samples`` 与障碍物碰撞认证的难度）。若每次规划都重跑
IRIS，规划链路将不可用。因此本模块的硬性契约是：

1. **同 spec 两次生成必须得到同一区域**（IRIS 确定性 + ``random_seed`` 固定，
   实测两次运行的 ``A``/``b`` 逐位相同）；
2. **命中缓存不重跑 IRIS**（:class:`RegionCache` 惰性缓存 + 计数器
   :attr:`RegionCache.iris_runs` 供测试与观测）；
3. **缓存可落盘、可复用**，但**必须校验模型版本**（URDF 的 sha256），
   版本不符时**拒绝复用并显式报错**（否则会拿旧模型的凸区域去规划新模型的运动）。

种子与配置约定（S02 → S05 的交接契约）
------------------------------------
:class:`RegionSpec` 是**区域的身份**：它完整决定「这个区域长什么样」。

====================  ==========================================================
字段                   含义
====================  ==========================================================
``seed_q``            种子位形（7 个关节角，rad，顺序 joint_1..joint_7）
``random_seed``       IRIS 随机源种子（``IrisOptions.random_seed``）
``configuration_space_margin``  C 空间安全余量（m，``IrisOptions.configuration_space_margin``）
``iteration_limit``   IRIS 迭代上限（``IrisOptions.iteration_limit``）
``num_collision_infeasible_samples``  碰撞不可行认证的随机重启次数（IRIS 认证强度）
``mixing_steps``      IRIS 混合步数（``IrisOptions.mixing_steps``）
====================  ==========================================================

前四个字段是**契约字段**（切片计划指定）；后两个是**认证强度旋钮**，放在 spec 里是
为了让「spec 完整决定区域」——否则缓存键无法区分「认证强度不同」的两个区域。

``generate_region(..., options=...)`` 只允许覆盖 :data:`OVERRIDABLE_OPTIONS` 里
这 6 个旋钮；**其它 IRIS 参数固定为 Drake 默认值**。这样做的理由同上：spec 必须是
区域的**完整**身份，否则缓存可能返回「用另一套参数算出来的区域」。

域（domain）与软限位
--------------------
**实测结论（Drake 1.51.1，本机）**：``IrisNp`` 的 domain 取自 **plant 的关节限位**
（即 URDF 里的**硬件**限位），而不是软件软限位。实测一次 IRIS 返回的多面体里
含有 14 条轴对齐约束，数值恰好是 URDF 的硬件限位（例如 joint_1 = ±3.106 rad），
因此区域**会越出软件软限位**。

本模块的处置（明确记录，避免后人误以为 IRIS 用了软限位）：

1. 域 = **软件软限位盒**（由 :mod:`rm75_drake.limits` 的 ``software`` 段构造的
   :class:`HPolyhedron`，见 :func:`soft_limit_box`）；
2. IRIS 返回的凸区域与该盒做**精确交集**（两者都是 H-多面体，交集 = 约束行堆叠，
   见 :func:`_clip_to_soft_limits`），得到最终区域；
3. 于是最终区域 **⊆ 软限位盒**（测试用 ``ContainedIn`` 断言），同时因为是 IRIS
   区域的**子集**，碰撞自由性依然成立。

**种子位形必须落在软件软限位内**（越界 → 显式报错，见 :func:`_validate_seed`）。

失败可解释
----------
所有失败都抛 :class:`RegionError`，并带稳定 ``reason`` 标签（:data:`FAILURE_REASONS`）：

======================  ====================================================
``invalid_seed``       种子维度不对 / 含 NaN-Inf / 越软件软限位
``invalid_config``     配置非法（margin < 0、迭代上限 ≤ 0、未知旋钮……）
``seed_infeasible``    种子位形本身碰撞/不可行（Drake 1.51.1 实测：
                       "The seed point is in collision; geometry X is in
                       collision with geometry Y"）
``empty_domain``       IRIS 返回空域（或与软限位盒求交后为空）
``scene_not_connected``  plant 没有与 scene graph 连接（Drake:
                       "required InputPort[0] (geometry_query) ... is not connected"）
``iris_error``         其它 Drake 侧错误（原样冒泡，带上下文）
``version_mismatch``   缓存文件的 URDF 版本与当前场景不符 → **拒绝复用**
``cache_corrupt``      缓存文件损坏 / 格式非法
======================  ====================================================

缓存文件格式（自研 YAML + 数值快照，**不是** Drake 的 ``SaveIrisRegionsYamlFile``）
------------------------------------------------------------------------------
Drake 自带的 ``SaveIrisRegionsYamlFile`` / ``LoadIrisRegionsYamlFile`` 面向的是
``IrisInConfigurationSpaceFromCliqueCover`` 的**IRIS 配置**（障碍物、种子、
``IrisOptions`` 及其在配置空间中的多面体表示），它**不**携带我们这套
「种子 + margin + 随机种子 + URDF 版本」的身份信息，也无法表达「按 spec 惰性求值」
的缓存语义。因此本模块采用**自研 YAML + 数值快照**：

.. code-block:: yaml

    schema_version: 1
    model_version: "<sha256(models/rm75_drake/rm75_drake.urdf)>"
    generator: rm75_drake.planning.regions
    regions:
      - id: region-xxxxxxxxxxxx
        spec:
          seed_q: [0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0]
          random_seed: 1234
          configuration_space_margin: 0.01
          iteration_limit: 100
          num_collision_infeasible_samples: 5
          mixing_steps: 10
        A: [[...], ...]      # 多面体 Ax ≤ b 的 A（m×7）
        b: [...]             # 多面体 Ax ≤ b 的 b（m）

文件里同时记录 **spec**（种子 / margin / 随机种子 / 迭代上限）与 **URDF 版本标识**
（URDF 文件内容的 sha256）。载入时版本不符 → 抛
:class:`RegionError`（``reason="version_mismatch"``），**不**静默复用。
"""

from __future__ import annotations

import hashlib
import json
import logging
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .. import limits as _limits

__all__ = [
    "RegionError",
    "RegionSpec",
    "Region",
    "RegionCache",
    "FAILURE_REASONS",
    "REASON_INVALID_SEED",
    "REASON_INVALID_CONFIG",
    "REASON_SEED_INFEASIBLE",
    "REASON_EMPTY_DOMAIN",
    "REASON_SCENE_NOT_CONNECTED",
    "REASON_IRIS_ERROR",
    "REASON_VERSION_MISMATCH",
    "REASON_CACHE_CORRUPT",
    "DEFAULT_RANDOM_SEED",
    "DEFAULT_CONFIGURATION_SPACE_MARGIN",
    "DEFAULT_ITERATION_LIMIT",
    "DEFAULT_NUM_COLLISION_INFEASIBLE_SAMPLES",
    "DEFAULT_MIXING_STEPS",
    "OVERRIDABLE_OPTIONS",
    "CACHE_SCHEMA_VERSION",
    "generate_region",
    "soft_limit_box",
    "software_joint_limits",
    "scene_model_version",
]


LOGGER = logging.getLogger(__name__)

# ── 失败类别（稳定标签，供调用方分类，不依赖自然语言措辞）────────────────
REASON_INVALID_SEED = "invalid_seed"
REASON_INVALID_CONFIG = "invalid_config"
REASON_SEED_INFEASIBLE = "seed_infeasible"
REASON_EMPTY_DOMAIN = "empty_domain"
REASON_SCENE_NOT_CONNECTED = "scene_not_connected"
REASON_IRIS_ERROR = "iris_error"
REASON_VERSION_MISMATCH = "version_mismatch"
REASON_CACHE_CORRUPT = "cache_corrupt"

#: 全部失败类别。
FAILURE_REASONS: Tuple[str, ...] = (
    REASON_INVALID_SEED,
    REASON_INVALID_CONFIG,
    REASON_SEED_INFEASIBLE,
    REASON_EMPTY_DOMAIN,
    REASON_SCENE_NOT_CONNECTED,
    REASON_IRIS_ERROR,
    REASON_VERSION_MISMATCH,
    REASON_CACHE_CORRUPT,
)

#: RM75 的关节数（规划变量维度）。
JOINT_COUNT = 7

#: 默认 IRIS 旋钮（Drake 的默认值，见 ``IrisOptions``）。
DEFAULT_RANDOM_SEED = 1234
DEFAULT_CONFIGURATION_SPACE_MARGIN = 0.01
DEFAULT_ITERATION_LIMIT = 100
DEFAULT_NUM_COLLISION_INFEASIBLE_SAMPLES = 5
DEFAULT_MIXING_STEPS = 10

#: ``options=`` 允许覆盖的旋钮（其余 IRIS 参数固定为 Drake 默认值）。
OVERRIDABLE_OPTIONS: Tuple[str, ...] = (
    "random_seed",
    "configuration_space_margin",
    "iteration_limit",
    "num_collision_infeasible_samples",
    "mixing_steps",
)

#: 缓存文件的 schema 版本。
CACHE_SCHEMA_VERSION = 1

#: 缓存文件的生成者标识。
CACHE_GENERATOR = "rm75_drake.planning.regions"

#: 数值比较容差（限位复核）。
_LIMIT_TOL = 1e-9


class RegionError(RuntimeError):
    """区域生成 / 缓存失败。

    属性
    ----
    reason:
        稳定标签（见 :data:`FAILURE_REASONS`），默认 ``"region_error"``。
    """

    def __init__(self, message: str, *, reason: str = "region_error") -> None:
        super().__init__(message)
        self.reason = reason


def _as_float_vector(values: Any, label: str) -> np.ndarray:
    """把任意序列转成一维 float 数组（非法 → ``invalid_seed``）。"""
    try:
        array = np.asarray(values, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise RegionError(
            f"{label}必须是数值序列：{values!r}（{exc}）",
            reason=REASON_INVALID_SEED,
        ) from exc
    return array


# ── 软限位（真值来自 limits.py，不在本模块复制常量）──────────────────────
def software_joint_limits() -> Tuple[Tuple[str, ...], np.ndarray, np.ndarray]:
    """返回软件软限位 ``(关节名, 下限数组, 上限数组)``。

    真值来自 ``models/rm75_drake/joint_limits.yaml`` 的 ``software`` 段
    （硬件限位每侧再收 4°），**不在本模块复制常量**。
    """
    data = _limits.load_joint_limits()
    joints = data.get("joints")
    if not isinstance(joints, list) or not joints:
        raise RegionError("限位交接件缺少 joints 段", reason=REASON_INVALID_CONFIG)
    names: List[str] = []
    lower: List[float] = []
    upper: List[float] = []
    for joint in joints:
        software = joint.get("software")
        if not isinstance(software, dict):
            raise RegionError(
                f"限位交接件缺少 {joint.get('name')} 的 software 段",
                reason=REASON_INVALID_CONFIG,
            )
        names.append(str(joint["name"]))
        lower.append(float(software["position_lower_rad"]))
        upper.append(float(software["position_upper_rad"]))
    if len(names) != JOINT_COUNT:
        raise RegionError(
            f"软件限位必须是 {JOINT_COUNT} 维，实际 {len(names)}",
            reason=REASON_INVALID_CONFIG,
        )
    return tuple(names), np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)


def soft_limit_box():
    """返回软件软限位盒（:class:`pydrake.geometry.optimization.HPolyhedron`）。

    这是本模块的 **domain**：最终区域保证 ⊆ 该盒（见模块 docstring「域与软限位」）。
    """
    from pydrake.geometry import optimization as go

    _, lower, upper = software_joint_limits()
    return go.HPolyhedron.MakeBox(lower, upper)


# ── RegionSpec ───────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RegionSpec:
    """区域身份（种子 + 配置）——S02 → S05 的交接契约。

    字段
    ----
    seed_q:
        种子位形（7 个关节角，rad）。构造时即校验维度与有限性。
    random_seed:
        IRIS 随机源种子（默认 1234）。
    configuration_space_margin:
        C 空间安全余量（m，默认 0.01）。
    iteration_limit:
        IRIS 迭代上限（默认 100）。
    num_collision_infeasible_samples:
        碰撞不可行认证的随机重启次数（默认 5）。**调小会显著加快 IRIS**
        （本机实测 5 → ~17 s，3 → ~8 s，1 → ~1.5 s），但认证强度随之下降。
    mixing_steps:
        IRIS 混合步数（默认 10）。

    说明
    ----
    **同 spec 两次生成必须得到同一区域**：IRIS 在固定 ``random_seed`` 下是确定性的，
    本机实测两次运行的 ``A``/``b`` 逐位相同。
    """

    seed_q: Tuple[float, ...]
    random_seed: int = DEFAULT_RANDOM_SEED
    configuration_space_margin: float = DEFAULT_CONFIGURATION_SPACE_MARGIN
    iteration_limit: int = DEFAULT_ITERATION_LIMIT
    num_collision_infeasible_samples: int = DEFAULT_NUM_COLLISION_INFEASIBLE_SAMPLES
    mixing_steps: int = DEFAULT_MIXING_STEPS

    def __post_init__(self) -> None:
        seed = _as_float_vector(self.seed_q, "种子位形")
        if seed.size != JOINT_COUNT:
            raise RegionError(
                f"种子位形必须是 {JOINT_COUNT} 个关节角，收到 {seed.size} 个",
                reason=REASON_INVALID_SEED,
            )
        if not np.all(np.isfinite(seed)):
            raise RegionError(
                f"种子位形含 NaN/Inf：{self.seed_q!r}", reason=REASON_INVALID_SEED
            )
        object.__setattr__(self, "seed_q", tuple(float(v) for v in seed))
        _validate_config(
            random_seed=self.random_seed,
            configuration_space_margin=self.configuration_space_margin,
            iteration_limit=self.iteration_limit,
            num_collision_infeasible_samples=self.num_collision_infeasible_samples,
            mixing_steps=self.mixing_steps,
        )

    # ── 序列化 / 身份 ───────────────────────────────────────────────────
    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON/YAML 化的字典（缓存文件与日志用）。"""
        return {
            "seed_q": list(self.seed_q),
            "random_seed": int(self.random_seed),
            "configuration_space_margin": float(self.configuration_space_margin),
            "iteration_limit": int(self.iteration_limit),
            "num_collision_infeasible_samples": int(
                self.num_collision_infeasible_samples
            ),
            "mixing_steps": int(self.mixing_steps),
        }

    @classmethod
    def from_dict(cls, data: Any) -> "RegionSpec":
        """从字典构造（缓存文件读取用）。非法结构 → ``cache_corrupt``。"""
        if not isinstance(data, dict):
            raise RegionError(
                f"spec 必须是映射，收到 {type(data).__name__}",
                reason=REASON_CACHE_CORRUPT,
            )
        try:
            return cls(
                seed_q=tuple(float(v) for v in data["seed_q"]),
                random_seed=int(data["random_seed"]),
                configuration_space_margin=float(
                    data["configuration_space_margin"]
                ),
                iteration_limit=int(data["iteration_limit"]),
                num_collision_infeasible_samples=int(
                    data.get(
                        "num_collision_infeasible_samples",
                        DEFAULT_NUM_COLLISION_INFEASIBLE_SAMPLES,
                    )
                ),
                mixing_steps=int(data.get("mixing_steps", DEFAULT_MIXING_STEPS)),
            )
        except RegionError as exc:
            # spec 本身非法（种子/配置）→ 缓存损坏，而不是"调用方传错"。
            raise RegionError(
                f"缓存里的 spec 非法：{exc}", reason=REASON_CACHE_CORRUPT
            ) from exc
        except (KeyError, TypeError, ValueError) as exc:
            raise RegionError(
                f"缓存里的 spec 缺少字段或类型不对：{exc}", reason=REASON_CACHE_CORRUPT
            ) from exc

    def key(self) -> Tuple[Any, ...]:
        """返回**可哈希**的身份元组（缓存键的一部分）。"""
        return (
            tuple(round(v, 12) for v in self.seed_q),
            int(self.random_seed),
            float(self.configuration_space_margin),
            int(self.iteration_limit),
            int(self.num_collision_infeasible_samples),
            int(self.mixing_steps),
        )


@dataclass(frozen=True)
class Region:
    """一个 C 空间凸区域。

    字段
    ----
    id:
        区域标识（由「模型版本 + spec」派生的稳定短哈希）。
    hpolyhedron:
        区域多面体（:class:`pydrake.geometry.optimization.HPolyhedron`），
        **保证 ⊆ 软件软限位盒**，且是 IRIS 碰撞自由区域的子集。
    seed_q:
        生成该区域所用的种子位形（rad）。
    seed_options:
        生成该区域所用的 :class:`RegionSpec`（区域身份）。
    model_version:
        生成时的 URDF 版本标识（sha256 十六进制）。
    duration_s:
        本次 IRIS 计算耗时（s），用于观测与性能回归。
    """

    id: str
    hpolyhedron: Any
    seed_q: Tuple[float, ...]
    seed_options: RegionSpec
    model_version: str = ""
    duration_s: float = 0.0

    @property
    def ambient_dimension(self) -> int:
        """区域的环境维度（RM75 = 7）。"""
        return int(self.hpolyhedron.ambient_dimension())

    @property
    def is_empty(self) -> bool:
        """区域是否为空。"""
        return bool(self.hpolyhedron.IsEmpty())

    def contains(self, point: Sequence[float]) -> bool:
        """点是否落在区域内。

        用 ``ConvexSet.PointInSet``（Drake 1.51/1.55 的 ``ConvexSet`` **没有**
        ``Contains`` 方法，只有 ``PointInSet``）。
        """
        return bool(
            self.hpolyhedron.PointInSet(np.asarray(point, dtype=float))
        )


# ── 配置校验 ─────────────────────────────────────────────────────────────
def _validate_config(
    *,
    random_seed: int,
    configuration_space_margin: float,
    iteration_limit: int,
    num_collision_infeasible_samples: int,
    mixing_steps: int,
) -> None:
    """校验 IRIS 配置参数（非法 → ``invalid_config``）。"""
    if not isinstance(random_seed, int) or isinstance(random_seed, bool):
        raise RegionError(
            f"random_seed 必须是整数，收到 {random_seed!r}", reason=REASON_INVALID_CONFIG
        )
    if random_seed < 0:
        raise RegionError(
            f"random_seed 必须 ≥ 0，收到 {random_seed}", reason=REASON_INVALID_CONFIG
        )
    if not isinstance(configuration_space_margin, (int, float)) or isinstance(
        configuration_space_margin, bool
    ):
        raise RegionError(
            f"configuration_space_margin 必须是数值，收到 {configuration_space_margin!r}",
            reason=REASON_INVALID_CONFIG,
        )
    if float(configuration_space_margin) < 0.0:
        raise RegionError(
            f"configuration_space_margin 必须 ≥ 0，收到 {configuration_space_margin}",
            reason=REASON_INVALID_CONFIG,
        )
    if not isinstance(iteration_limit, int) or isinstance(iteration_limit, bool):
        raise RegionError(
            f"iteration_limit 必须是整数，收到 {iteration_limit!r}",
            reason=REASON_INVALID_CONFIG,
        )
    if iteration_limit <= 0:
        raise RegionError(
            f"iteration_limit 必须 > 0，收到 {iteration_limit}", reason=REASON_INVALID_CONFIG
        )
    if (
        not isinstance(num_collision_infeasible_samples, int)
        or isinstance(num_collision_infeasible_samples, bool)
        or num_collision_infeasible_samples <= 0
    ):
        raise RegionError(
            "num_collision_infeasible_samples 必须是正整数，收到 "
            f"{num_collision_infeasible_samples!r}",
            reason=REASON_INVALID_CONFIG,
        )
    if (
        not isinstance(mixing_steps, int)
        or isinstance(mixing_steps, bool)
        or mixing_steps < 0
    ):
        raise RegionError(
            f"mixing_steps 必须是非负整数，收到 {mixing_steps!r}",
            reason=REASON_INVALID_CONFIG,
        )


def _apply_options(spec: RegionSpec, options: Any) -> RegionSpec:
    """把 ``options``（dict 或 ``IrisOptions``）覆盖到 spec 上。

    只允许覆盖 :data:`OVERRIDABLE_OPTIONS` 里的旋钮——否则 spec 就不能完整
    决定区域，缓存键会失效（可能返回"用另一套参数算出来的区域"）。
    """
    if options is None:
        return spec
    if isinstance(options, RegionSpec):
        return options
    if isinstance(options, dict):
        values = dict(options)
        unknown = sorted(set(values) - set(OVERRIDABLE_OPTIONS))
        if unknown:
            raise RegionError(
                f"options 含有未知旋钮 {unknown}；只允许 "
                f"{list(OVERRIDABLE_OPTIONS)}（其余 IRIS 参数固定为 Drake 默认值，"
                "以保证 spec 完整决定区域）",
                reason=REASON_INVALID_CONFIG,
            )
    else:
        values = {}
        for key in OVERRIDABLE_OPTIONS:
            if hasattr(options, key):
                values[key] = getattr(options, key)
    if not values:
        return spec
    updated = replace(spec, **values)
    # replace() 会再次触发 __post_init__ 的校验。
    return updated


# ── 模型版本 ─────────────────────────────────────────────────────────────
def scene_model_version(scene: Any) -> str:
    """返回场景的模型版本标识（URDF 文件内容的 sha256，十六进制）。

    * ``scene.urdf_paths``（:class:`rm75_drake.planning.scene.SceneBundle` 提供）
      里每个 URDF 的内容 sha256 按顺序拼接后再哈希；
    * 若场景没有提供 ``urdf_paths``（例如自造的场景对象），返回空串——
      此时缓存文件里的版本标识也是空串，版本比对**不会**拦住复用，
      因此**生产路径必须用** :func:`rm75_drake.planning.scene.build_scene` 装配场景。
    """
    urdf_paths = tuple(getattr(scene, "urdf_paths", ()) or ())
    if not urdf_paths:
        return ""
    digest = hashlib.sha256()
    for path in urdf_paths:
        target = Path(path)
        digest.update(target.name.encode("utf-8"))
        try:
            digest.update(hashlib.sha256(target.read_bytes()).digest())
        except OSError as exc:
            raise RegionError(
                f"无法读取 URDF 以计算模型版本：{target}（{exc}）",
                reason=REASON_INVALID_CONFIG,
            ) from exc
    return digest.hexdigest()


def _region_id(model_version: str, spec: RegionSpec) -> str:
    """返回区域的稳定标识（模型版本 + spec 的短哈希）。"""
    payload = json.dumps(
        {"model_version": model_version, "spec": spec.to_dict()},
        sort_keys=True,
        ensure_ascii=False,
    )
    return "region-" + hashlib.sha256(payload.encode("utf-8")).hexdigest()[:12]


# ── 区域生成 ─────────────────────────────────────────────────────────────
def _validate_seed(seed_q: Sequence[float]) -> List[str]:
    """返回种子位形相对**软件软限位**的越界描述（空列表 = 合法）。"""
    names, lower, upper = software_joint_limits()
    problems: List[str] = []
    for index, name in enumerate(names):
        value = float(seed_q[index])
        low = float(lower[index])
        high = float(upper[index])
        if value < low - _LIMIT_TOL or value > high + _LIMIT_TOL:
            problems.append(
                f"{name}={value:.6f} 超出软件软限位 [{low:.6f}, {high:.6f}] rad"
            )
    return problems


def _clip_to_soft_limits(region, box):
    """把 IRIS 区域与软限位盒求精确交集（两者都是 H-多面体 → 约束行堆叠）。"""
    from pydrake.geometry import optimization as go

    a_region = np.asarray(region.A(), dtype=float)
    b_region = np.asarray(region.b(), dtype=float).reshape(-1)
    a_box = np.asarray(box.A(), dtype=float)
    b_box = np.asarray(box.b(), dtype=float).reshape(-1)
    a_all = np.vstack((a_region, a_box))
    b_all = np.concatenate((b_region, b_box))
    return go.HPolyhedron(a_all, b_all)


def _run_iris(scene: Any, spec: RegionSpec):
    """跑一次 ``IrisNp``，返回原始多面体（未与软限位盒求交）。"""
    from pydrake.geometry import optimization as go

    plant = getattr(scene, "plant", None)
    context = getattr(scene, "plant_context", None)
    if plant is None or context is None:
        raise RegionError(
            "场景缺少 plant / plant_context（请用 build_scene 装配场景）",
            reason=REASON_INVALID_CONFIG,
        )

    options = go.IrisOptions()
    options.configuration_space_margin = float(spec.configuration_space_margin)
    options.iteration_limit = int(spec.iteration_limit)
    options.random_seed = int(spec.random_seed)
    options.num_collision_infeasible_samples = int(
        spec.num_collision_infeasible_samples
    )
    options.mixing_steps = int(spec.mixing_steps)

    original = np.asarray(plant.GetPositions(context), dtype=float).copy()
    try:
        plant.SetPositions(context, np.asarray(spec.seed_q, dtype=float))
        return go.IrisNp(plant, context, options)
    except RuntimeError as exc:
        message = str(exc)
        if "is not connected" in message:
            raise RegionError(
                "IRIS 失败：plant 没有与 scene graph 连接"
                "（Drake: required InputPort[0] (geometry_query) ... is not "
                f"connected）。请用 build_scene 装配场景。原始错误：{message}",
                reason=REASON_SCENE_NOT_CONNECTED,
            ) from exc
        if "infeasible" in message.lower() or "in collision" in message.lower():
            raise RegionError(
                "IRIS 失败：种子位形不可行（seed_q 本身碰撞/不在可行域内），"
                f"seed_q={list(spec.seed_q)}。原始错误：{message}",
                reason=REASON_SEED_INFEASIBLE,
            ) from exc
        raise RegionError(
            f"IRIS 计算失败：{message}", reason=REASON_IRIS_ERROR
        ) from exc
    finally:
        # IRIS 会在上下文里到处设位形；用完必须还原（场景是共享对象）。
        plant.SetPositions(context, original)


def generate_region(
    scene: Any,
    seed_q: Sequence[float],
    *,
    options: Any = None,
    spec: Optional[RegionSpec] = None,
) -> Region:
    """生成一个 C 空间凸区域。

    参数
    ----
    scene:
        :class:`rm75_drake.planning.scene.SceneBundle`（S02-T01 装配产物）。
        **前置条件**：``scene.plant`` 必须已与 scene graph 连接（否则 IRIS 报
        "required InputPort[0] (geometry_query) ... is not connected"，
        本模块会转成 ``reason="scene_not_connected"``）。
    seed_q:
        种子位形（7 个关节角，rad）。必须落在**软件软限位**内。
    options:
        可选：覆盖 IRIS 旋钮。可传 dict 或 ``IrisOptions``；只允许
        :data:`OVERRIDABLE_OPTIONS` 里的键（未知键 → ``invalid_config``）。
        ``None`` → 用 spec / 默认值。
    spec:
        可选：显式给出区域身份 :class:`RegionSpec`。给出时 ``seed_q`` 必须与
        ``spec.seed_q`` 一致（否则 ``invalid_config``）。

    返回
    ----
    :class:`Region`

    异常
    ----
    RegionError
        种子非法（``invalid_seed``）、配置非法（``invalid_config``）、
        种子不可行（``seed_infeasible``）、区域为空（``empty_domain``）、
        场景未连接（``scene_not_connected``）、Drake 侧其它错误（``iris_error``）。
    """
    if spec is not None and not isinstance(spec, RegionSpec):
        raise RegionError(
            f"spec 必须是 RegionSpec，收到 {type(spec).__name__}",
            reason=REASON_INVALID_CONFIG,
        )
    if spec is None:
        base = RegionSpec(seed_q=tuple(float(v) for v in _as_float_vector(seed_q, "种子位形")))
    else:
        base = spec
        given = _as_float_vector(seed_q, "种子位形")
        if given.size != len(base.seed_q) or not np.allclose(
            given, np.asarray(base.seed_q, dtype=float), rtol=0.0, atol=1e-12
        ):
            raise RegionError(
                "seed_q 与 spec.seed_q 不一致："
                f"{list(given)} vs {list(base.seed_q)}",
                reason=REASON_INVALID_CONFIG,
            )
    effective = _apply_options(base, options)

    violations = _validate_seed(effective.seed_q)
    if violations:
        raise RegionError(
            "种子位形越出软件软限位：" + "；".join(violations),
            reason=REASON_INVALID_SEED,
        )

    version = scene_model_version(scene)
    started = time.perf_counter()
    raw = _run_iris(scene, effective)
    duration = time.perf_counter() - started

    if raw.IsEmpty():
        raise RegionError(
            f"IRIS 返回空域（seed_q={list(effective.seed_q)}）",
            reason=REASON_EMPTY_DOMAIN,
        )

    clipped = _clip_to_soft_limits(raw, soft_limit_box())
    if clipped.IsEmpty():
        raise RegionError(
            "IRIS 区域与软件软限位盒求交后为空"
            f"（seed_q={list(effective.seed_q)}）",
            reason=REASON_EMPTY_DOMAIN,
        )

    region = Region(
        id=_region_id(version, effective),
        hpolyhedron=clipped,
        seed_q=tuple(float(v) for v in effective.seed_q),
        seed_options=effective,
        model_version=version,
        duration_s=duration,
    )
    LOGGER.info(
        "IRIS 区域 %s 生成完成：seed=%s，耗时 %.2f s，约束行 %d",
        region.id,
        [round(v, 4) for v in region.seed_q],
        duration,
        int(np.asarray(clipped.A()).shape[0]),
    )
    return region


# ── 缓存 ─────────────────────────────────────────────────────────────────
class RegionCache:
    """IRIS 区域的惰性缓存（``list[RegionSpec] → list[Region]``）。

    语义
    ----
    * 缓存持有一份 **spec 列表**；:meth:`get` / :meth:`regions` 命中已算过的 spec
      时**直接复用**（**不重跑 IRIS**）；
    * 惰性：只有真正被请求的 spec 才会触发 IRIS 计算；
    * :attr:`iris_runs` 记录**实际发生的 IRIS 次数**（测试与观测用）。

    落盘
    ----
    :meth:`save_cache` / :meth:`load_cache` 用**自研 YAML + 数值快照**
    （见模块 docstring「缓存文件格式」），文件里同时记录 spec 与 **URDF 版本标识**；
    版本不符时 :meth:`load_cache` 抛 :class:`RegionError`
    （``reason="version_mismatch"``），**拒绝复用**。
    """

    def __init__(
        self,
        scene: Any,
        specs: Iterable[RegionSpec] = (),
        *,
        model_version: Optional[str] = None,
    ) -> None:
        self._scene = scene
        self._specs: List[RegionSpec] = []
        self._regions: Dict[Tuple[Any, ...], Region] = {}
        self._version = (
            model_version if model_version is not None else scene_model_version(scene)
        )
        self._iris_runs = 0
        for spec in specs:
            self.add_spec(spec)

    # ── 只读属性 ────────────────────────────────────────────────────────
    @property
    def model_version(self) -> str:
        """缓存对应的模型版本标识（URDF 内容 sha256；无则空串）。"""
        return self._version

    @property
    def iris_runs(self) -> int:
        """实际发生的 IRIS 计算次数（命中缓存不计数）。"""
        return self._iris_runs

    @property
    def specs(self) -> Tuple[RegionSpec, ...]:
        """缓存登记的 spec 列表（有序）。"""
        return tuple(self._specs)

    def __len__(self) -> int:
        return len(self._specs)

    def __contains__(self, spec: object) -> bool:
        return isinstance(spec, RegionSpec) and spec.key() in self._regions

    # ── 登记与查询 ──────────────────────────────────────────────────────
    def add_spec(self, spec: RegionSpec) -> RegionSpec:
        """登记一个 spec（重复登记返回原对象，顺序保持首次登记的次序）。"""
        if not isinstance(spec, RegionSpec):
            raise RegionError(
                f"spec 必须是 RegionSpec，收到 {type(spec).__name__}",
                reason=REASON_INVALID_CONFIG,
            )
        for existing in self._specs:
            if existing.key() == spec.key():
                return existing
        self._specs.append(spec)
        return spec

    def get(self, spec: RegionSpec) -> Region:
        """返回 spec 对应的区域（命中缓存则复用，否则计算一次并缓存）。"""
        if not isinstance(spec, RegionSpec):
            raise RegionError(
                f"spec 必须是 RegionSpec，收到 {type(spec).__name__}",
                reason=REASON_INVALID_CONFIG,
            )
        key = spec.key()
        cached = self._regions.get(key)
        if cached is not None:
            return cached
        region = generate_region(self._scene, spec.seed_q, spec=spec)
        self._iris_runs += 1
        self._regions[key] = region
        if not any(existing.key() == key for existing in self._specs):
            self._specs.append(spec)
        return region

    def regions(self) -> List[Region]:
        """返回与 :attr:`specs` 等长、顺序一致的区域列表（惰性计算缺失项）。"""
        return [self.get(spec) for spec in self._specs]

    # ── 落盘 ────────────────────────────────────────────────────────────
    def save_cache(self, path) -> Path:
        """把**已计算**的区域写成 YAML 缓存文件，返回文件路径。

        文件里同时记录 spec（种子 / margin / 随机种子 / 迭代上限 / 认证强度）与
        **URDF 版本标识**（sha256）。见模块 docstring「缓存文件格式」。
        """
        import yaml

        target = Path(path)
        payload: Dict[str, Any] = {
            "schema_version": CACHE_SCHEMA_VERSION,
            "model_version": self._version,
            "generator": CACHE_GENERATOR,
            "regions": [],
        }
        for spec in self._specs:
            region = self._regions.get(spec.key())
            if region is None:
                continue  # 惰性：未计算的 spec 不落盘。
            a = np.asarray(region.hpolyhedron.A(), dtype=float)
            b = np.asarray(region.hpolyhedron.b(), dtype=float).reshape(-1)
            payload["regions"].append(
                {
                    "id": region.id,
                    "spec": spec.to_dict(),
                    "A": [[float(v) for v in row] for row in a],
                    "b": [float(v) for v in b],
                }
            )
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            yaml.safe_dump(payload, sort_keys=False, default_flow_style=False),
            encoding="utf-8",
        )
        LOGGER.info("IRIS 区域缓存已写入 %s（%d 个区域）", target, len(payload["regions"]))
        return target

    @classmethod
    def load_cache(cls, path, scene: Any) -> "RegionCache":
        """从 YAML 缓存文件恢复缓存（**校验模型版本**）。

        版本标识与当前场景不一致 → 抛 :class:`RegionError`
        （``reason="version_mismatch"``），**不**复用旧区域。
        """
        import yaml
        from pydrake.geometry import optimization as go

        target = Path(path)
        if not target.is_file():
            raise RegionError(
                f"缓存文件不存在：{target}", reason=REASON_CACHE_CORRUPT
            )
        try:
            payload = yaml.safe_load(target.read_text(encoding="utf-8"))
        except Exception as exc:
            raise RegionError(
                f"缓存文件解析失败：{target}（{exc}）", reason=REASON_CACHE_CORRUPT
            ) from exc
        if not isinstance(payload, dict):
            raise RegionError(
                f"缓存文件顶层必须是映射：{target}", reason=REASON_CACHE_CORRUPT
            )
        version = int(payload.get("schema_version", -1))
        if version != CACHE_SCHEMA_VERSION:
            raise RegionError(
                f"缓存文件 schema_version 必须是 {CACHE_SCHEMA_VERSION}，实际 {version}",
                reason=REASON_CACHE_CORRUPT,
            )
        current = scene_model_version(scene)
        stored = str(payload.get("model_version", ""))
        if stored != current:
            raise RegionError(
                "缓存文件与当前场景的模型版本不一致，拒绝复用："
                f"缓存 {stored or '<空>'} vs 当前 {current or '<空>'}",
                reason=REASON_VERSION_MISMATCH,
            )
        entries = payload.get("regions")
        if not isinstance(entries, list):
            raise RegionError(
                f"缓存文件缺少 regions 段：{target}", reason=REASON_CACHE_CORRUPT
            )

        cache = cls(scene, model_version=stored)
        for entry in entries:
            if not isinstance(entry, dict):
                raise RegionError(
                    "缓存文件的 regions 段出现非映射元素", reason=REASON_CACHE_CORRUPT
                )
            spec = RegionSpec.from_dict(entry.get("spec"))
            try:
                a = np.asarray(entry["A"], dtype=float)
                b = np.asarray(entry["b"], dtype=float).reshape(-1)
            except (KeyError, TypeError, ValueError) as exc:
                raise RegionError(
                    f"缓存文件里的多面体数据损坏：{exc}", reason=REASON_CACHE_CORRUPT
                ) from exc
            if a.ndim != 2 or a.shape[0] != b.shape[0]:
                raise RegionError(
                    f"缓存文件里的多面体形状非法：A{a.shape} vs b{b.shape}",
                    reason=REASON_CACHE_CORRUPT,
                )
            region = Region(
                id=str(entry.get("id") or _region_id(stored, spec)),
                hpolyhedron=go.HPolyhedron(a, b),
                seed_q=spec.seed_q,
                seed_options=spec,
                model_version=stored,
                duration_s=0.0,
            )
            cache.add_spec(spec)
            cache._regions[spec.key()] = region
        LOGGER.info("IRIS 区域缓存已从 %s 载入（%d 个区域）", target, len(cache._specs))
        return cache
