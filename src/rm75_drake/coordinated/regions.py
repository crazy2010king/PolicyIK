"""**最小坐标（8 维）凸区域生成 + 缓存**（S05-T04）。

为什么必须自研
--------------
Drake 1.51.1 的 ``IrisNp`` / ``Iris`` 绑定的是 ``MultibodyPlant`` 的**完整
关节空间**与 plant 内的障碍几何；它**无法**表达「8 维最小坐标 + 映射回 14 维
再判碰撞」这种自定义空间。因此本切片**不调用 IRIS**，而是用 numpy 自研一个
最小坐标区域生成器：

* **起点**：一个「碰撞自由 + 相对位姿可满足」的种子 ``x0``（8 维）；
* **膨胀**：沿一组确定性方向探测边界（映射回 14 维 → oracle 判碰撞），
  得到支撑不等式 ``aᵢ·x ≤ bᵢ``；
* **切割**：探测到碰撞样本时，沿该方向**收缩**支撑不等式（**保守收缩，
  宁小勿大**），并反复采样验证，直到「区域内取样点全部碰撞自由」；
* **输出**：R⁸ 的 ``HPolyhedron``（``pydrake.geometry.optimization``）
  + ``is_empty`` + ``contains(x)`` + 种子 + 可复现规格（:class:`MinimalRegionSpec`）。

热循环与算法解耦（为 pybind11 移植预留）
----------------------------------------
碰撞 oracle 的**接口是纯 numpy 的**：``oracle(x_batch) -> np.ndarray[bool]``
（输入一批 8 维参数，输出 free/busy 布尔数组），**不出现任何 Drake 类型**。
Drake 只以「注入的后端回调」形式出现（:func:`make_numpy_oracle` 的
``collision_free`` 参数），因此把热循环机械移植到 C++（D031 延期项）时，
**不需要改动算法本身**。

缓存（**另立类名**，不改 S02 冻结的 ``planning/regions.py``）
-----------------------------------------------------------
* :class:`MinimalRegionSpec` —— 区域身份（种子 + 参数 + 维度）；
* :class:`MinimalRegion` —— 区域本体（A/b + 种子 + spec + 版本 + 耗时）；
* :class:`MinimalRegionCache` —— 惰性缓存、YAML 落盘、``model_version``
  **版本门禁**、命中 / 未命中 / 生成计数。

版本门禁语义：缓存文件的 ``model_version`` 与当前场景不一致时，
:func:`MinimalRegionCache.load_cache` **丢弃**陈旧条目（``strict=True`` 时
抛 :class:`MinimalRegionError` / ``reason="version_mismatch"``），随后
:meth:`MinimalRegionCache.get` **明确 miss 并重新生成**——**绝不**复用陈旧区域。

降级链（显式、可解释；**禁止静默降级**）
----------------------------------------
区域生成失败（种子不可行 / 区域为空 / 保守性验证失败）⇒
:func:`generate_region_with_fallback` 走**采样基线**（最小空间 PRM，**同一套
oracle**）+ 局部精修，并在结果里用 ``stage`` / ``reason`` / ``degraded``
**显式标注降级**。

维度纪律
--------
所有区域**恒为 8 维**（:data:`MINIMAL_DIM`），且 :meth:`MinimalRegion.as_graph_entry`
返回 ``(id, HPolyhedron)`` 元组，可直接喂给
:func:`rm75_drake.planning.gcs.build_graph`（S05-T05 建图用）。

零 pydrake 依赖（模块级）
-------------------------
本模块顶层只 import numpy + 标准库；``pydrake``（HPolyhedron）与
:mod:`rm75_drake.bimanual.dual_scene` 的碰撞查询都是**函数内惰性导入**，
因此 ``import rm75_drake.coordinated.regions`` **不会**把 pydrake 拉进
``sys.modules``。
"""

from __future__ import annotations

import hashlib
import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "MINIMAL_DIM",
    "MinimalRegionError",
    "MinimalRegionSpec",
    "MinimalRegion",
    "MinimalRegionResult",
    "MinimalRegionCache",
    "make_numpy_oracle",
    "make_parameterization_oracle",
    "generate_minimal_region",
    "generate_region_with_fallback",
    "sample_baseline_path",
    "minimal_model_version",
    "FAILURE_REASONS",
    "REASON_INVALID_SPEC",
    "REASON_INVALID_ORACLE",
    "REASON_SEED_INFEASIBLE",
    "REASON_EMPTY_REGION",
    "REASON_CONSERVATISM_FAILED",
    "REASON_VERSION_MISMATCH",
    "REASON_CACHE_CORRUPT",
    "REASON_SAMPLING_FAILED",
    "REASON_GOAL_INFEASIBLE",
    "CACHE_SCHEMA_VERSION",
    "CACHE_GENERATOR",
    "STAGE_REGIONS",
    "STAGE_SAMPLE_BASELINE",
]

LOGGER = logging.getLogger(__name__)

#: 最小坐标参数化的维度（leader 7 关节 + follower ψ 1）。**所有区域恒为 8 维**。
MINIMAL_DIM = 8

# ── 失败类别（稳定标签）────────────────────────────────────────────────
REASON_INVALID_SPEC = "invalid_spec"
REASON_INVALID_ORACLE = "invalid_oracle"
REASON_SEED_INFEASIBLE = "seed_infeasible"
REASON_EMPTY_REGION = "empty_region"
REASON_CONSERVATISM_FAILED = "conservatism_failed"
REASON_VERSION_MISMATCH = "version_mismatch"
REASON_CACHE_CORRUPT = "cache_corrupt"
REASON_SAMPLING_FAILED = "sampling_failed"
REASON_GOAL_INFEASIBLE = "goal_infeasible"

#: 全部失败类别。
FAILURE_REASONS: Tuple[str, ...] = (
    REASON_INVALID_SPEC,
    REASON_INVALID_ORACLE,
    REASON_SEED_INFEASIBLE,
    REASON_EMPTY_REGION,
    REASON_CONSERVATISM_FAILED,
    REASON_VERSION_MISMATCH,
    REASON_CACHE_CORRUPT,
    REASON_SAMPLING_FAILED,
    REASON_GOAL_INFEASIBLE,
)

#: 阶段标签（区域阶段 / 采样基线降级阶段）。
STAGE_REGIONS = "regions"
STAGE_SAMPLE_BASELINE = "sample_baseline"

#: 缓存文件的 schema 版本。
CACHE_SCHEMA_VERSION = 1

#: 缓存文件的生成者标识。
CACHE_GENERATOR = "rm75_drake.coordinated.regions"

#: 数值容差（约束判定 / ``contains``）。
_TOL = 1e-9

#: 区域「可用体积」下限：内接球半径小于该值 ⇒ 视为**空区域**
#: （零体积的多面体对 GCS 建图毫无意义 ⇒ 触发降级链）。
MIN_EXTENT = 1e-6


class MinimalRegionError(RuntimeError):
    """最小坐标区域生成 / 缓存失败。

    ``reason`` 是稳定标签（见 :data:`FAILURE_REASONS`），默认 ``"region_error"``。
    """

    def __init__(self, message: str, *, reason: str = "region_error") -> None:
        super().__init__(message)
        self.reason = str(reason)


# ── 规格 ───────────────────────────────────────────────────────────────
def _as_float_vector(values: Any, label: str) -> np.ndarray:
    """把任意序列转成一维 float 数组（非法 → 显式报错）。"""
    try:
        array = np.asarray(values, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise MinimalRegionError(
            f"{label} 必须是数值序列：{exc}", reason=REASON_INVALID_SPEC
        ) from exc
    if not np.all(np.isfinite(array)):
        raise MinimalRegionError(
            f"{label} 含 NaN/Inf：{values!r}", reason=REASON_INVALID_SPEC
        )
    return array


@dataclass(frozen=True)
class MinimalRegionSpec:
    """最小坐标区域的**可复现规格**（区域身份 = 种子 + 参数）。

    字段
    ----
    seed_x:
        种子参数点（**8 维**，leader 7 关节 + follower ψ）。
    dim:
        维度（**恒为 8**；传别的值 → 显式报错并点名期望 / 实际）。
    random_seed:
        采样随机源种子（**固定 ⇒ 同 spec 两次生成逐位相同**）。
    step:
        方向探测步长（rad / 参数单位）。
    max_expand:
        单方向最大探测步数（探测半径 = ``step * max_expand``）。
    margin:
        保守余量（rad）：支撑不等式的可走范围**再收紧**这么多，
        换取「宁小勿大」的保守性。
    num_validation_samples:
        每轮保守性验证的采样点数。
    max_rounds:
        保守性收缩的最大轮数（超过 ⇒ ``conservatism_failed``）。
    include_diagonals:
        是否在坐标轴方向之外加入对角方向（提高覆盖，降低保守性代价）。

    说明
    ----
    **同 spec 两次生成必须得到同一区域**：采样用 ``numpy.random.default_rng``，
    种子固定 ⇒ 采样序列逐位相同；方向集合是确定性的。
    """

    seed_x: Tuple[float, ...]
    dim: int = MINIMAL_DIM
    random_seed: int = 20250516
    step: float = 0.05
    max_expand: int = 8
    margin: float = 0.02
    num_validation_samples: int = 48
    max_rounds: int = 8
    include_diagonals: bool = True

    def __post_init__(self) -> None:
        seed = _as_float_vector(self.seed_x, "种子参数")
        if int(self.dim) != MINIMAL_DIM:
            raise MinimalRegionError(
                f"[dimension] 最小坐标区域必须是 {MINIMAL_DIM} 维，实际 {int(self.dim)} 维",
                reason=REASON_INVALID_SPEC,
            )
        if seed.size != MINIMAL_DIM:
            raise MinimalRegionError(
                f"[dimension] 种子参数必须是 {MINIMAL_DIM} 维，实际 {seed.size} 维",
                reason=REASON_INVALID_SPEC,
            )
        object.__setattr__(self, "seed_x", tuple(float(v) for v in seed))
        if float(self.step) <= 0.0:
            raise MinimalRegionError(
                f"[config] step 必须是正数，实际 {self.step}", reason=REASON_INVALID_SPEC
            )
        if int(self.max_expand) < 1:
            raise MinimalRegionError(
                f"[config] max_expand 必须是正整数，实际 {self.max_expand}",
                reason=REASON_INVALID_SPEC,
            )
        if float(self.margin) < 0.0:
            raise MinimalRegionError(
                f"[config] margin 必须是非负数，实际 {self.margin}",
                reason=REASON_INVALID_SPEC,
            )
        if int(self.num_validation_samples) < 1:
            raise MinimalRegionError(
                f"[config] num_validation_samples 必须是正整数，实际 "
                f"{self.num_validation_samples}",
                reason=REASON_INVALID_SPEC,
            )
        if int(self.max_rounds) < 1:
            raise MinimalRegionError(
                f"[config] max_rounds 必须是正整数，实际 {self.max_rounds}",
                reason=REASON_INVALID_SPEC,
            )

    # ── 序列化 / 身份 ───────────────────────────────────────────────────
    def to_dict(self) -> Dict[str, Any]:
        """返回可 YAML/JSON 化的字典。"""
        return {
            "seed_x": list(self.seed_x),
            "dim": int(self.dim),
            "random_seed": int(self.random_seed),
            "step": float(self.step),
            "max_expand": int(self.max_expand),
            "margin": float(self.margin),
            "num_validation_samples": int(self.num_validation_samples),
            "max_rounds": int(self.max_rounds),
            "include_diagonals": bool(self.include_diagonals),
        }

    @classmethod
    def from_dict(cls, data: Any) -> "MinimalRegionSpec":
        """从字典构造（缓存读取用）；非法结构 → ``cache_corrupt``。"""
        if not isinstance(data, dict):
            raise MinimalRegionError(
                f"spec 必须是映射，收到 {type(data).__name__}",
                reason=REASON_CACHE_CORRUPT,
            )
        try:
            return cls(
                seed_x=tuple(float(v) for v in data["seed_x"]),
                dim=int(data.get("dim", MINIMAL_DIM)),
                random_seed=int(data["random_seed"]),
                step=float(data["step"]),
                max_expand=int(data["max_expand"]),
                margin=float(data["margin"]),
                num_validation_samples=int(data["num_validation_samples"]),
                max_rounds=int(data["max_rounds"]),
                include_diagonals=bool(data.get("include_diagonals", True)),
            )
        except MinimalRegionError as exc:
            raise MinimalRegionError(
                f"缓存里的 spec 非法：{exc}", reason=REASON_CACHE_CORRUPT
            ) from exc
        except (KeyError, TypeError, ValueError) as exc:
            raise MinimalRegionError(
                f"缓存里的 spec 缺少字段或类型不对：{exc}", reason=REASON_CACHE_CORRUPT
            ) from exc

    def key(self) -> Tuple[Any, ...]:
        """返回**可哈希**的身份元组（缓存键）。"""
        return (
            tuple(round(v, 12) for v in self.seed_x),
            int(self.dim),
            int(self.random_seed),
            float(self.step),
            int(self.max_expand),
            float(self.margin),
            int(self.num_validation_samples),
            int(self.max_rounds),
            bool(self.include_diagonals),
        )


# ── 区域 ───────────────────────────────────────────────────────────────
@dataclass
class MinimalRegion:
    """一个 **R⁸** 的最小坐标凸区域。

    字段
    ----
    id:
        区域标识（模型版本 + spec 的短哈希，稳定可复现）。
    a / b:
        支撑不等式 ``A x ≤ b``（``a`` 形状 ``(m, 8)``，``b`` 形状 ``(m,)``）。
    seed_x:
        生成该区域所用的种子（**恒在区域内**）。
    spec:
        生成该区域所用的 :class:`MinimalRegionSpec`（区域身份）。
    model_version:
        生成时的场景模型版本标识（URDF 内容 sha256）。
    duration_s:
        本次生成耗时（s）。
    rounds / cuts:
        保守性收缩轮数 / 追加的支撑不等式条数（观测用）。
    reason:
        失败原因（正常生成时为空串；空区域时是稳定标签）。
    """

    id: str
    a: np.ndarray
    b: np.ndarray
    seed_x: Tuple[float, ...]
    spec: MinimalRegionSpec
    model_version: str = ""
    duration_s: float = 0.0
    rounds: int = 0
    cuts: int = 0
    reason: str = ""
    _poly: Any = field(default=None, repr=False, compare=False)

    # -- 基本属性 --------------------------------------------------------
    @property
    def ambient_dimension(self) -> int:
        """区域的环境维度（**恒为 8**）。"""
        return MINIMAL_DIM

    @property
    def num_constraints(self) -> int:
        """支撑不等式的条数。"""
        return int(np.asarray(self.a).shape[0])

    @property
    def hpolyhedron(self) -> Any:
        """区域的 :class:`pydrake.geometry.optimization.HPolyhedron`（惰性构造）。"""
        if self._poly is None:
            from pydrake.geometry import optimization as go

            a = np.asarray(self.a, dtype=float).reshape(-1, MINIMAL_DIM)
            b = np.asarray(self.b, dtype=float).reshape(-1)
            self._poly = go.HPolyhedron(a, b)
        return self._poly

    @property
    def is_empty(self) -> bool:
        """区域是否为空（用 Drake 的精确判定）。"""
        return bool(self.hpolyhedron.IsEmpty())

    def contains(self, point: Sequence[float]) -> bool:
        """点是否落在区域内（纯 numpy 判定，含容差）。"""
        array = np.asarray(point, dtype=float).reshape(-1)
        if array.size != MINIMAL_DIM:
            raise MinimalRegionError(
                f"[dimension] 判定点必须是 {MINIMAL_DIM} 维，实际 {array.size} 维",
                reason=REASON_INVALID_SPEC,
            )
        a = np.asarray(self.a, dtype=float).reshape(-1, MINIMAL_DIM)
        b = np.asarray(self.b, dtype=float).reshape(-1)
        return bool(np.all(a @ array <= b + _TOL))

    # -- 采样 ------------------------------------------------------------
    def inscribed_radius(self) -> float:
        """返回以种子为中心、**内接于区域**的最大球半径（保守采样用）。"""
        a = np.asarray(self.a, dtype=float).reshape(-1, MINIMAL_DIM)
        b = np.asarray(self.b, dtype=float).reshape(-1)
        seed = np.asarray(self.seed_x, dtype=float)
        if a.size == 0:
            return 0.0
        norms = np.linalg.norm(a, axis=1)
        slack = b - a @ seed
        usable = norms > 1e-12
        if not np.any(usable):
            return 0.0
        return float(max(0.0, np.min(slack[usable] / norms[usable])))

    def sample(self, count: int, *, seed: int = 0) -> np.ndarray:
        """在区域内均匀采样（内接球法；**保证采样点满足所有约束**）。

        退化（内接半径 ≈ 0）时返回只含种子的数组。
        """
        number = int(count)
        if number < 1:
            raise MinimalRegionError(
                f"[config] count 必须是正整数，实际 {number}",
                reason=REASON_INVALID_SPEC,
            )
        radius = self.inscribed_radius()
        seed_vec = np.asarray(self.seed_x, dtype=float)
        if radius <= 1e-12:
            return seed_vec.reshape(1, MINIMAL_DIM).repeat(number, axis=0)
        rng = np.random.default_rng(int(seed))
        out = np.empty((number, MINIMAL_DIM), dtype=float)
        filled = 0
        while filled < number:
            raw = rng.normal(size=(number, MINIMAL_DIM))
            norms = np.linalg.norm(raw, axis=1)
            keep = norms > 1e-12
            raw = raw[keep] / norms[keep, None]
            radii = radius * rng.random(size=raw.shape[0]) ** (1.0 / MINIMAL_DIM)
            candidates = seed_vec[None, :] + radii[:, None] * raw
            a = np.asarray(self.a, dtype=float).reshape(-1, MINIMAL_DIM)
            b = np.asarray(self.b, dtype=float).reshape(-1)
            if a.size:
                inside = np.all(candidates @ a.T <= b + _TOL, axis=1)
            else:
                inside = np.ones(len(candidates), dtype=bool)
            candidates = candidates[inside]
            take = min(number - filled, len(candidates))
            out[filled : filled + take] = candidates[:take]
            filled += take
        return out

    # -- 与 GCS 对接 ------------------------------------------------------
    def as_graph_entry(self) -> Tuple[str, Any]:
        """返回 ``(id, HPolyhedron)`` 元组（直接喂给 ``planning.gcs.build_graph``）。"""
        return (self.id, self.hpolyhedron)

    @classmethod
    def empty(
        cls,
        spec: MinimalRegionSpec,
        *,
        reason: str,
        model_version: str = "",
    ) -> "MinimalRegion":
        """构造一个**空区域**（``is_empty`` 为真，``reason`` 明确）。"""
        a = np.zeros((2, MINIMAL_DIM), dtype=float)
        a[0, 0] = 1.0
        a[1, 0] = -1.0
        b = np.array([-1.0, -1.0], dtype=float)
        return cls(
            id=_region_id(model_version, spec),
            a=a,
            b=b,
            seed_x=tuple(spec.seed_x),
            spec=spec,
            model_version=model_version,
            duration_s=0.0,
            reason=str(reason),
        )

    def to_dict(self) -> Dict[str, Any]:
        """返回可 YAML/JSON 化的字典（缓存落盘用）。"""
        return {
            "id": self.id,
            "spec": self.spec.to_dict(),
            "A": [[float(v) for v in row] for row in np.asarray(self.a, dtype=float)],
            "b": [float(v) for v in np.asarray(self.b, dtype=float).reshape(-1)],
            "reason": self.reason,
            "rounds": int(self.rounds),
            "cuts": int(self.cuts),
        }


# ── 结果对象 ───────────────────────────────────────────────────────────
@dataclass
class MinimalRegionResult:
    """区域生成 / 降级的结果（**降级显式**）。

    字段
    ----
    success:
        是否**拿到了凸区域**（降级到采样基线时为 ``False``）。
    stage:
        阶段标签：``"regions"``（正常）或 ``"sample_baseline"``（**降级**）。
    reason:
        稳定标签（``"ok"`` 或失败类别）。
    degraded:
        是否发生了降级。
    region:
        生成的区域（失败时是**空区域**，``is_empty`` 为真）。
    path:
        降级到采样基线时的路径（8 维参数点序列）；未降级时为空数组。
    duration_s:
        总耗时（s）。
    message:
        人类可读说明（**不作为判据**，判据是 ``reason`` / ``stage``）。
    """

    success: bool
    stage: str
    reason: str
    degraded: bool = False
    region: Optional[MinimalRegion] = None
    path: Optional[np.ndarray] = None
    duration_s: float = 0.0
    message: str = ""

    @property
    def is_empty(self) -> bool:
        """区域是否为空（无区域时为真）。"""
        if self.region is None:
            return True
        return bool(self.region.is_empty)

    def as_dict(self) -> Dict[str, Any]:
        """可 JSON 序列化的字典（进 diagnostics）。"""
        return {
            "success": bool(self.success),
            "stage": self.stage,
            "reason": self.reason,
            "degraded": bool(self.degraded),
            "duration_s": float(self.duration_s),
            "num_constraints": (
                0 if self.region is None else int(self.region.num_constraints)
            ),
            "path_points": 0 if self.path is None else int(len(self.path)),
            "message": self.message,
        }


# ── oracle（纯 numpy 接口）─────────────────────────────────────────────
Oracle = Callable[[np.ndarray], np.ndarray]


def _check_oracle_output(flags: Any, expected: int) -> np.ndarray:
    """校验 oracle 输出（形状 / dtype）；非法 → ``invalid_oracle``。"""
    array = np.asarray(flags).reshape(-1)
    if array.size != expected:
        raise MinimalRegionError(
            f"[oracle] oracle 必须为每个采样点返回一个布尔值：期望 {expected} 个，"
            f"实际 {array.size} 个",
            reason=REASON_INVALID_ORACLE,
        )
    return array.astype(bool)


def make_numpy_oracle(
    mapper: Callable[[np.ndarray], np.ndarray],
    collision_free: Callable[[np.ndarray], np.ndarray],
) -> Oracle:
    """组装**纯 numpy**的碰撞 oracle。

    参数
    ----
    mapper:
        ``(N, 8) → (N, 14)`` 的映射（numpy ⇒ numpy，**不出现 Drake 类型**）。
    collision_free:
        ``(N, 14) → (N,) bool`` 的碰撞判定后端（Drake 实现在调用侧注入）。

    返回
    ----
    ``oracle(x_batch) -> np.ndarray[bool]``：输入一批 8 维参数，输出 free/busy。
    """

    def oracle(x_batch: np.ndarray) -> np.ndarray:
        x = np.asarray(x_batch, dtype=float)
        if x.ndim == 1:
            x = x.reshape(1, -1)
        if x.ndim != 2 or x.shape[1] != MINIMAL_DIM:
            raise MinimalRegionError(
                f"[oracle] 输入必须是 (N, {MINIMAL_DIM}) 的参数批次，实际形状 {x.shape}",
                reason=REASON_INVALID_ORACLE,
            )
        q = np.asarray(mapper(x), dtype=float)
        if q.shape != (len(x), 2 * MINIMAL_DIM - 2):
            raise MinimalRegionError(
                f"[oracle] mapper 必须返回 (N, 14) 的关节配置，实际形状 {q.shape}",
                reason=REASON_INVALID_ORACLE,
            )
        return _check_oracle_output(collision_free(q), len(x))

    return oracle


def make_parameterization_oracle(
    parameterization: Any,
    *,
    enforce_limits: bool = True,
) -> Oracle:
    """由 :class:`~rm75_drake.coordinated.parameterization.CoordinatedParameterization`
    组装最小坐标 oracle（**同一个 plant**）。

    * follower 无解（``unreachable`` / ``branch_exhausted`` / ``limit_violation`` /
      ``wrist_singular``）⇒ 该点视为 **busy**（不是「未知」）；
    * ``enforce_limits=True`` 时，超软件限位的位形也算 busy
      （可行域 = 限位内 ∩ 碰撞自由 ∩ 相对位姿可满足）。
    """

    def oracle(x_batch: np.ndarray) -> np.ndarray:
        from .parameterization import CoordinatedParameterizationError

        x = np.asarray(x_batch, dtype=float)
        if x.ndim == 1:
            x = x.reshape(1, -1)
        if x.ndim != 2 or x.shape[1] != MINIMAL_DIM:
            raise MinimalRegionError(
                f"[oracle] 输入必须是 (N, {MINIMAL_DIM}) 的参数批次，实际形状 {x.shape}",
                reason=REASON_INVALID_ORACLE,
            )
        flags = np.zeros(len(x), dtype=bool)
        for index in range(len(x)):
            try:
                q14 = parameterization.to_joint_config(x[index])
            except CoordinatedParameterizationError:
                continue  # follower 无解 ⇒ busy。
            if enforce_limits and not parameterization.within_limits(q14):
                continue
            if not parameterization.is_collision_free(q14):
                continue
            flags[index] = True
        return flags

    return oracle


# ── 方向集合与几何小工具 ──────────────────────────────────────────────
def _direction_set(spec: MinimalRegionSpec) -> List[np.ndarray]:
    """返回**确定性**的探测方向集合（单位向量）。"""
    dirs: List[np.ndarray] = []
    for index in range(MINIMAL_DIM):
        axis = np.zeros(MINIMAL_DIM, dtype=float)
        axis[index] = 1.0
        dirs.append(axis)
        dirs.append(-axis)
    if spec.include_diagonals:
        for bits in range(16):
            diag = np.zeros(MINIMAL_DIM, dtype=float)
            for bit in range(4):
                diag[bit] = 1.0 if (bits >> bit) & 1 else -1.0
                diag[MINIMAL_DIM - 1 - bit] = 1.0 if (bits >> bit) & 1 else -1.0
            norm = float(np.linalg.norm(diag))
            dirs.append(diag / norm)
    return dirs


def _extent_along(
    oracle: Oracle,
    x0: np.ndarray,
    direction: np.ndarray,
    *,
    step: float,
    max_expand: int,
) -> float:
    """沿 ``direction`` 探测边界，返回**可行步长** ``t``（0 = 该方向不可走）。

    判据是**整段路径可行**（保守）：一旦某一步不可行即停止。
    """
    ts = np.arange(1, int(max_expand) + 1, dtype=float) * float(step)
    points = x0[None, :] + ts[:, None] * direction[None, :]
    flags = _check_oracle_output(oracle(points), len(ts))
    extent = 0.0
    for t, flag in zip(ts, flags):
        if not flag:
            break
        extent = float(t)
    return extent


def _sample_inside(
    rng: np.random.Generator,
    x0: np.ndarray,
    a_rows: np.ndarray,
    b_rows: np.ndarray,
    count: int,
) -> np.ndarray:
    """在「以种子为中心的内接球」里采样（**保证满足全部约束**）。"""
    if a_rows.size == 0:
        radius = 0.0
    else:
        norms = np.linalg.norm(a_rows, axis=1)
        slack = b_rows - a_rows @ x0
        usable = norms > 1e-12
        radius = (
            float(max(0.0, np.min(slack[usable] / norms[usable]))) if np.any(usable) else 0.0
        )
    if radius <= 1e-12:
        return x0.reshape(1, MINIMAL_DIM)
    out = np.empty((int(count), MINIMAL_DIM), dtype=float)
    filled = 0
    while filled < count:
        raw = rng.normal(size=(count, MINIMAL_DIM))
        norms = np.linalg.norm(raw, axis=1)
        keep = norms > 1e-12
        raw = raw[keep] / norms[keep, None]
        radii = radius * rng.random(size=len(raw)) ** (1.0 / MINIMAL_DIM)
        candidates = x0[None, :] + radii[:, None] * raw
        inside = np.all(candidates @ a_rows.T <= b_rows + _TOL, axis=1)
        candidates = candidates[inside]
        take = min(count - filled, len(candidates))
        out[filled : filled + take] = candidates[:take]
        filled += take
    return out


def _inscribed_radius(x0: np.ndarray, a_rows: np.ndarray, b_rows: np.ndarray) -> float:
    """以 ``x0`` 为中心、**内接于多面体**的最大球半径。"""
    if a_rows.size == 0:
        return 0.0
    norms = np.linalg.norm(a_rows, axis=1)
    slack = b_rows - a_rows @ x0
    usable = norms > 1e-12
    if not np.any(usable):
        return 0.0
    return float(max(0.0, np.min(slack[usable] / norms[usable])))


def _region_id(model_version: str, spec: MinimalRegionSpec) -> str:
    """区域的稳定标识（模型版本 + spec 的短哈希）。"""
    payload = json.dumps(
        {"model_version": model_version, "spec": spec.to_dict()},
        sort_keys=True,
        ensure_ascii=False,
    )
    return "minimal-region-" + hashlib.sha256(payload.encode("utf-8")).hexdigest()[:12]


# ── 区域生成 ───────────────────────────────────────────────────────────
def generate_minimal_region(
    spec: MinimalRegionSpec,
    oracle: Oracle,
    *,
    model_version: str = "",
    directions: Optional[Sequence[Sequence[float]]] = None,
) -> MinimalRegion:
    """生成一个 **R⁸** 的最小坐标凸区域（自研，**不调用 IRIS**）。

    算法（**显式可测**）
    --------------------
    1. **起点**：种子 ``x0`` 必须**碰撞自由**（oracle 判定），否则
       ``reason="seed_infeasible"``；
    2. **膨胀**：沿确定性方向集合探测边界，得到支撑不等式 ``aᵢ·x ≤ bᵢ``
       （可走范围再收紧 ``margin``）；
    3. **切割**：在区域内采样 → oracle 判定；发现 busy 样本 ⇒ 沿该方向
       **重新探测**边界并追加一条**更紧**的支撑不等式（**保守收缩**）；
    4. **验证**：反复采样，直到「区域内取样点全部 busy-free」；
       超过 ``max_rounds`` 仍失败 ⇒ ``reason="conservatism_failed"``。

    失败
    ----
    :class:`MinimalRegionError`
        ``reason`` ∈ ``invalid_spec`` / ``invalid_oracle`` /
        ``seed_infeasible`` / ``conservatism_failed``。
    """
    if not isinstance(spec, MinimalRegionSpec):
        raise MinimalRegionError(
            f"spec 必须是 MinimalRegionSpec，收到 {type(spec).__name__}",
            reason=REASON_INVALID_SPEC,
        )
    if not callable(oracle):
        raise MinimalRegionError(
            f"oracle 必须可调用，收到 {type(oracle).__name__}",
            reason=REASON_INVALID_ORACLE,
        )
    started = time.perf_counter()
    x0 = np.asarray(spec.seed_x, dtype=float).reshape(MINIMAL_DIM)

    seed_flags = _check_oracle_output(oracle(x0.reshape(1, MINIMAL_DIM)), 1)
    if not bool(seed_flags[0]):
        raise MinimalRegionError(
            "[seed] 种子参数点不可行（碰撞 / 限位 / follower 无解）："
            f"{list(spec.seed_x)}",
            reason=REASON_SEED_INFEASIBLE,
        )

    if directions is None:
        dirs = _direction_set(spec)
    else:
        dirs = []
        for item in directions:
            vector = np.asarray(item, dtype=float).reshape(-1)
            if vector.size != MINIMAL_DIM:
                raise MinimalRegionError(
                    f"[dimension] 探测方向必须是 {MINIMAL_DIM} 维，实际 {vector.size} 维",
                    reason=REASON_INVALID_SPEC,
                )
            norm = float(np.linalg.norm(vector))
            if norm <= 1e-12:
                raise MinimalRegionError(
                    "[config] 探测方向不能是零向量", reason=REASON_INVALID_SPEC
                )
            dirs.append(vector / norm)

    a_rows: List[np.ndarray] = []
    b_rows: List[float] = []
    for direction in dirs:
        extent = _extent_along(
            oracle,
            x0,
            direction,
            step=spec.step,
            max_expand=spec.max_expand,
        )
        reach = max(0.0, extent - float(spec.margin))
        a_rows.append(direction)
        b_rows.append(float(direction @ x0) + reach)

    rng = np.random.default_rng(int(spec.random_seed))
    a_matrix = np.asarray(a_rows, dtype=float).reshape(-1, MINIMAL_DIM)
    b_vector = np.asarray(b_rows, dtype=float).reshape(-1)

    if _inscribed_radius(x0, a_matrix, b_vector) <= MIN_EXTENT:
        raise MinimalRegionError(
            "[empty] 膨胀后区域没有可用体积（内接球半径 ≈ 0）",
            reason=REASON_EMPTY_REGION,
        )

    rounds = 0
    busy_total = 0
    converged = False
    for rounds in range(1, int(spec.max_rounds) + 1):
        samples = _sample_inside(
            rng, x0, a_matrix, b_vector, int(spec.num_validation_samples)
        )
        flags = _check_oracle_output(oracle(samples), len(samples))
        busy = samples[~flags]
        busy_total += int(len(busy))
        if len(busy) == 0:
            converged = True
            break
        for point in busy:
            delta = point - x0
            norm = float(np.linalg.norm(delta))
            if norm <= 1e-12:
                continue
            direction = delta / norm
            extent = _extent_along(
                oracle,
                x0,
                direction,
                step=spec.step,
                max_expand=spec.max_expand,
            )
            reach = max(0.0, extent - float(spec.margin))
            a_matrix = np.vstack((a_matrix, direction.reshape(1, MINIMAL_DIM)))
            b_vector = np.concatenate((b_vector, np.array([float(direction @ x0) + reach])))
            b_vector[-1] = float(direction @ x0) + reach

    if not converged:
        raise MinimalRegionError(
            f"[conservatism] {spec.max_rounds} 轮收缩后区域内仍有碰撞样本"
            f"（累计 {busy_total} 个），保守性无法保证",
            reason=REASON_CONSERVATISM_FAILED,
        )

    # 独立复核：用**另一个**随机流再验一次（避免过拟合到验证样本）。
    check_rng = np.random.default_rng(int(spec.random_seed) + 1)
    check_samples = _sample_inside(
        check_rng, x0, a_matrix, b_vector, int(spec.num_validation_samples)
    )
    check_flags = _check_oracle_output(oracle(check_samples), len(check_samples))
    if not np.all(check_flags):
        raise MinimalRegionError(
            "[conservatism] 独立复核采样发现碰撞样本，保守性无法保证",
            reason=REASON_CONSERVATISM_FAILED,
        )

    if _inscribed_radius(x0, a_matrix, b_vector) <= MIN_EXTENT:
        raise MinimalRegionError(
            "[empty] 收缩后区域没有可用体积（内接球半径 ≈ 0）",
            reason=REASON_EMPTY_REGION,
        )

    region = MinimalRegion(
        id=_region_id(model_version, spec),
        a=a_matrix,
        b=b_vector,
        seed_x=tuple(spec.seed_x),
        spec=spec,
        model_version=model_version,
        duration_s=time.perf_counter() - started,
        rounds=int(rounds),
        cuts=int(a_matrix.shape[0] - len(dirs)),
    )
    if not region.contains(x0):  # pragma: no cover - 结构性不变量
        raise MinimalRegionError(
            "[invariant] 生成区域必须包含种子", reason=REASON_CONSERVATISM_FAILED
        )
    return region


# ── 采样基线（降级路径）────────────────────────────────────────────────
def sample_baseline_path(
    seed_x: Sequence[float],
    goal_x: Sequence[float],
    oracle: Oracle,
    *,
    random_seed: int = 20250516,
    num_samples: int = 64,
    radius: float = 0.4,
    connect_tolerance: float = 1e-9,
) -> np.ndarray:
    """最小空间 **PRM** 采样基线（**同一套 oracle**）。

    返回从 ``seed_x`` 到 ``goal_x`` 的 8 维参数路径（形状 ``(M, 8)``）。

    失败
    ----
    :class:`MinimalRegionError`
        ``reason`` ∈ ``seed_infeasible`` / ``goal_infeasible`` /
        ``sampling_failed``。
    """
    start = np.asarray(seed_x, dtype=float).reshape(-1)
    goal = np.asarray(goal_x, dtype=float).reshape(-1)
    if start.size != MINIMAL_DIM or goal.size != MINIMAL_DIM:
        raise MinimalRegionError(
            f"[dimension] 起点 / 终点必须是 {MINIMAL_DIM} 维，实际 "
            f"{start.size} / {goal.size} 维",
            reason=REASON_INVALID_SPEC,
        )
    start_flags = _check_oracle_output(oracle(start.reshape(1, MINIMAL_DIM)), 1)
    if not bool(start_flags[0]):
        raise MinimalRegionError(
            "[seed] 起点不可行（采样基线无法起步）", reason=REASON_SEED_INFEASIBLE
        )
    goal_flags = _check_oracle_output(oracle(goal.reshape(1, MINIMAL_DIM)), 1)
    if not bool(goal_flags[0]):
        raise MinimalRegionError(
            "[goal] 终点不可行（采样基线无法到达）", reason=REASON_GOAL_INFEASIBLE
        )

    rng = np.random.default_rng(int(random_seed))
    nodes = [start, goal]
    center = 0.5 * (start + goal)
    attempts = 0
    while len(nodes) < int(num_samples) and attempts < 20 * int(num_samples):
        attempts += 1
        raw = rng.normal(size=MINIMAL_DIM)
        norm = float(np.linalg.norm(raw))
        if norm <= 1e-12:
            continue
        candidate = center + float(radius) * (raw / norm)
        flag = _check_oracle_output(oracle(candidate.reshape(1, MINIMAL_DIM)), 1)
        if bool(flag[0]):
            nodes.append(candidate)
    node_array = np.asarray(nodes, dtype=float)

    # 边：两端可行且中点可行。
    midpoints = 0.5 * (node_array[:, None, :] + node_array[None, :, :])
    flat = midpoints.reshape(-1, MINIMAL_DIM)
    mid_flags = _check_oracle_output(oracle(flat), len(flat)).reshape(
        len(node_array), len(node_array)
    )
    adjacency = [
        [j for j in range(len(node_array)) if j != i and bool(mid_flags[i, j])]
        for i in range(len(node_array))
    ]

    # BFS：0 = 起点，1 = 终点。
    previous = {0: None}
    queue = [0]
    found = False
    while queue:
        current = queue.pop(0)
        if current == 1:
            found = True
            break
        for neighbour in adjacency[current]:
            if neighbour not in previous:
                previous[neighbour] = current
                queue.append(neighbour)
    if not found:
        raise MinimalRegionError(
            "[sampling] 采样基线未能连通起点与终点", reason=REASON_SAMPLING_FAILED
        )

    path: List[np.ndarray] = []
    node = 1
    while node is not None:
        path.append(node_array[node])
        node = previous[node]
    path.reverse()
    out = np.asarray(path, dtype=float)
    if out.shape[0] < 2 or float(
        np.linalg.norm(out[0] - start)
    ) > 1e-6 or float(np.linalg.norm(out[-1] - goal)) > 1e-6:
        raise MinimalRegionError(
            "[sampling] 采样基线路径端点不符", reason=REASON_SAMPLING_FAILED
        )
    _ = connect_tolerance
    return _refine_path(out, oracle)


def _refine_path(path: np.ndarray, oracle: Oracle, *, max_passes: int = 4) -> np.ndarray:
    """**局部精修**：剪掉可以直连的中间节点（shortcut 平滑）。

    只在「直连段的中点可行」时裁剪 ⇒ 裁剪后路径仍然**保守可行**；
    端点保持不变。
    """
    current = [np.asarray(row, dtype=float) for row in np.asarray(path, dtype=float)]
    for _ in range(int(max_passes)):
        changed = False
        index = 0
        while index + 2 < len(current):
            head = current[index]
            tail = current[index + 2]
            midpoint = 0.5 * (head + tail)
            flag = _check_oracle_output(oracle(midpoint.reshape(1, MINIMAL_DIM)), 1)
            if bool(flag[0]):
                del current[index + 1]
                changed = True
            else:
                index += 1
        if not changed:
            break
    return np.asarray(current, dtype=float)


# ── 降级链入口 ─────────────────────────────────────────────────────────
def generate_region_with_fallback(
    spec: MinimalRegionSpec,
    oracle: Oracle,
    *,
    goal_x: Optional[Sequence[float]] = None,
    model_version: str = "",
) -> MinimalRegionResult:
    """区域生成 + **显式降级链**。

    * 正常：``stage="regions"`` / ``reason="ok"`` / ``degraded=False``；
    * 降级：区域生成失败 ⇒ 走**采样基线**（同一套 oracle），
      ``stage="sample_baseline"`` / ``degraded=True`` /
      ``reason`` = 区域失败原因（采样基线再失败 ⇒ ``reason`` 更新为采样失败原因，
      ``message`` 里保留完整链路）。

    **不静默降级**：降级时 ``region`` 是**空区域**（``is_empty`` 为真）。
    """
    started = time.perf_counter()
    try:
        region = generate_minimal_region(spec, oracle, model_version=model_version)
    except MinimalRegionError as exc:
        if exc.reason in (REASON_INVALID_SPEC, REASON_INVALID_ORACLE):
            # 调用方 bug（非法 spec / oracle）⇒ **不降级**，直接点名抛错。
            raise
        base_reason = exc.reason
        empty = MinimalRegion.empty(spec, reason=base_reason, model_version=model_version)
        if goal_x is None:
            return MinimalRegionResult(
                success=False,
                stage=STAGE_SAMPLE_BASELINE,
                reason=base_reason,
                degraded=True,
                region=empty,
                path=None,
                duration_s=time.perf_counter() - started,
                message=(
                    f"区域生成失败（{base_reason}）且未提供目标点，无法走采样基线：{exc}"
                ),
            )
        try:
            path = sample_baseline_path(
                spec.seed_x,
                goal_x,
                oracle,
                random_seed=int(spec.random_seed),
            )
        except MinimalRegionError as fallback_exc:
            return MinimalRegionResult(
                success=False,
                stage=STAGE_SAMPLE_BASELINE,
                reason=fallback_exc.reason,
                degraded=True,
                region=empty,
                path=None,
                duration_s=time.perf_counter() - started,
                message=(
                    f"区域生成失败（{base_reason}），采样基线亦失败"
                    f"（{fallback_exc.reason}）：{fallback_exc}"
                ),
            )
        return MinimalRegionResult(
            success=False,
            stage=STAGE_SAMPLE_BASELINE,
            reason=base_reason,
            degraded=True,
            region=empty,
            path=path,
            duration_s=time.perf_counter() - started,
            message=f"区域生成失败（{base_reason}），已降级到采样基线：{exc}",
        )
    return MinimalRegionResult(
        success=True,
        stage=STAGE_REGIONS,
        reason="ok",
        degraded=False,
        region=region,
        path=None,
        duration_s=time.perf_counter() - started,
        message="",
    )


# ── 模型版本 ───────────────────────────────────────────────────────────
def minimal_model_version(bundle: Any) -> str:
    """返回双臂场景的**模型版本标识**（URDF 内容 sha256）。

    覆盖机械臂 URDF 与夹爪 URDF（S04 的重命名变体）；任一文件读不到 ⇒
    抛 :class:`MinimalRegionError`（``reason="invalid_spec"``），
    **不静默返回空串**（空串会让版本门禁失效）。
    """
    paths = tuple(getattr(bundle, "urdf_paths", ()) or ()) + tuple(
        getattr(bundle, "gripper_urdf_paths", ()) or ()
    )
    if not paths:
        raise MinimalRegionError(
            "[version] 场景没有提供 urdf_paths，无法计算模型版本",
            reason=REASON_INVALID_SPEC,
        )
    digest = hashlib.sha256()
    for item in paths:
        target = Path(item)
        digest.update(target.name.encode("utf-8"))
        try:
            digest.update(hashlib.sha256(target.read_bytes()).digest())
        except OSError as exc:
            raise MinimalRegionError(
                f"[version] 无法读取 URDF 以计算模型版本：{target}（{exc}）",
                reason=REASON_INVALID_SPEC,
            ) from exc
    return digest.hexdigest()


# ── 缓存 ───────────────────────────────────────────────────────────────
class MinimalRegionCache:
    """最小坐标区域的惰性缓存（``list[MinimalRegionSpec] → list[MinimalRegion]``）。

    语义
    ----
    * **惰性**：只有真正被请求的 spec 才会触发生成；
    * :attr:`hit_count` / :attr:`miss_count` / :attr:`generation_count` 记录
      命中、未命中与实际生成次数（观测用）；
    * **版本门禁**：缓存文件的 ``model_version`` 与当前场景不一致 ⇒
      陈旧条目被**丢弃**（:attr:`stale_entries_dropped`），
      随后的 :meth:`get` **明确 miss 并重新生成**——**绝不复用陈旧区域**。

    落盘
    ----
    :meth:`save_cache` / :meth:`load_cache` 用 YAML（自研格式，见
    ``schema_version`` / ``model_version`` / ``generator`` / ``regions``）。
    """

    def __init__(
        self,
        *,
        model_version: str = "",
        specs: Iterable[MinimalRegionSpec] = (),
        regions: Iterable[MinimalRegion] = (),
    ) -> None:
        self._version = str(model_version)
        self._specs: List[MinimalRegionSpec] = []
        self._regions: Dict[Tuple[Any, ...], MinimalRegion] = {}
        self._hits = 0
        self._misses = 0
        self._generations = 0
        self._stale_dropped = 0
        self._version_mismatch = False
        for spec in specs:
            self.add_spec(spec)
        for region in regions:
            self._regions[region.spec.key()] = region
            self.add_spec(region.spec)

    # ── 只读属性 ────────────────────────────────────────────────────────
    @property
    def model_version(self) -> str:
        """缓存对应的模型版本标识。"""
        return self._version

    @property
    def hit_count(self) -> int:
        """命中次数。"""
        return self._hits

    @property
    def miss_count(self) -> int:
        """未命中次数。"""
        return self._misses

    @property
    def generation_count(self) -> int:
        """实际发生的区域生成次数（命中不计数）。"""
        return self._generations

    @property
    def stale_entries_dropped(self) -> int:
        """因版本门禁被丢弃的陈旧条目数。"""
        return self._stale_dropped

    @property
    def version_mismatch(self) -> bool:
        """是否发生过版本不匹配（版本门禁是否生效过）。"""
        return self._version_mismatch

    @property
    def specs(self) -> Tuple[MinimalRegionSpec, ...]:
        """已登记的 spec 列表（有序）。"""
        return tuple(self._specs)

    def __len__(self) -> int:
        return len(self._specs)

    def __contains__(self, spec: object) -> bool:
        return isinstance(spec, MinimalRegionSpec) and spec.key() in self._regions

    # ── 登记与查询 ──────────────────────────────────────────────────────
    def add_spec(self, spec: MinimalRegionSpec) -> MinimalRegionSpec:
        """登记一个 spec（重复登记返回原对象）。"""
        if not isinstance(spec, MinimalRegionSpec):
            raise MinimalRegionError(
                f"spec 必须是 MinimalRegionSpec，收到 {type(spec).__name__}",
                reason=REASON_INVALID_SPEC,
            )
        for existing in self._specs:
            if existing.key() == spec.key():
                return existing
        self._specs.append(spec)
        return spec

    def get(
        self,
        spec: MinimalRegionSpec,
        generator: Optional[Callable[[MinimalRegionSpec], MinimalRegion]] = None,
    ) -> MinimalRegion:
        """返回 spec 对应的区域（命中则复用，否则生成一次并缓存）。"""
        if not isinstance(spec, MinimalRegionSpec):
            raise MinimalRegionError(
                f"spec 必须是 MinimalRegionSpec，收到 {type(spec).__name__}",
                reason=REASON_INVALID_SPEC,
            )
        key = spec.key()
        cached = self._regions.get(key)
        if cached is not None:
            self._hits += 1
            return cached
        self._misses += 1
        if generator is None:
            raise MinimalRegionError(
                "[cache] 未命中且未提供生成器（generator=None）",
                reason=REASON_INVALID_SPEC,
            )
        region = generator(spec)
        self._generations += 1
        self._regions[key] = region
        self.add_spec(spec)
        return region

    def regions(self) -> List[MinimalRegion]:
        """返回已缓存的区域列表（顺序 = spec 登记顺序）。"""
        out: List[MinimalRegion] = []
        for spec in self._specs:
            region = self._regions.get(spec.key())
            if region is not None:
                out.append(region)
        return out

    # ── 落盘 ────────────────────────────────────────────────────────────
    def save_cache(self, path) -> Path:
        """把**已生成**的区域写成 YAML 缓存文件，返回文件路径。"""
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
                continue  # 惰性：未生成的 spec 不落盘。
            payload["regions"].append(region.to_dict())
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            yaml.safe_dump(payload, sort_keys=False, default_flow_style=False),
            encoding="utf-8",
        )
        LOGGER.info(
            "最小坐标区域缓存已写入 %s（%d 个区域）", target, len(payload["regions"])
        )
        return target

    @classmethod
    def load_cache(
        cls,
        path,
        *,
        model_version: Optional[str] = None,
        strict: bool = False,
    ) -> "MinimalRegionCache":
        """从 YAML 缓存文件恢复缓存（**版本门禁**）。

        * ``model_version`` 不一致 ⇒ **丢弃陈旧条目**（``stale_entries_dropped``
          记录条数，``version_mismatch`` 为真），随后 :meth:`get` 明确 miss
          并重新生成；``strict=True`` 时改为抛
          :class:`MinimalRegionError`（``reason="version_mismatch"``）。
        * 文件缺失 / 解析失败 / schema 不符 ⇒ ``reason="cache_corrupt"``。
        """
        import yaml

        target = Path(path)
        if not target.is_file():
            raise MinimalRegionError(
                f"缓存文件不存在：{target}", reason=REASON_CACHE_CORRUPT
            )
        try:
            payload = yaml.safe_load(target.read_text(encoding="utf-8"))
        except Exception as exc:
            raise MinimalRegionError(
                f"缓存文件解析失败：{target}（{exc}）", reason=REASON_CACHE_CORRUPT
            ) from exc
        if not isinstance(payload, dict):
            raise MinimalRegionError(
                f"缓存文件顶层必须是映射：{target}", reason=REASON_CACHE_CORRUPT
            )
        version = int(payload.get("schema_version", -1))
        if version != CACHE_SCHEMA_VERSION:
            raise MinimalRegionError(
                f"缓存文件 schema_version 必须是 {CACHE_SCHEMA_VERSION}，实际 {version}",
                reason=REASON_CACHE_CORRUPT,
            )
        stored = str(payload.get("model_version", ""))
        current = stored if model_version is None else str(model_version)
        if stored != current:
            if strict:
                raise MinimalRegionError(
                    "缓存文件与当前场景的模型版本不一致，拒绝复用："
                    f"缓存 {stored or '<空>'} vs 当前 {current or '<空>'}",
                    reason=REASON_VERSION_MISMATCH,
                )
            entries = payload.get("regions")
            dropped = len(entries) if isinstance(entries, list) else 0
            LOGGER.warning(
                "最小坐标区域缓存版本不匹配，丢弃 %d 个陈旧条目：缓存 %s vs 当前 %s",
                dropped,
                stored or "<空>",
                current or "<空>",
            )
            cache = cls(model_version=current)
            cache._stale_dropped = dropped
            cache._version_mismatch = True
            return cache

        entries = payload.get("regions")
        if not isinstance(entries, list):
            raise MinimalRegionError(
                f"缓存文件缺少 regions 段：{target}", reason=REASON_CACHE_CORRUPT
            )

        cache = cls(model_version=stored)
        for entry in entries:
            if not isinstance(entry, dict):
                raise MinimalRegionError(
                    "缓存文件的 regions 段出现非映射元素", reason=REASON_CACHE_CORRUPT
                )
            spec = MinimalRegionSpec.from_dict(entry.get("spec"))
            try:
                a = np.asarray(entry["A"], dtype=float)
                b = np.asarray(entry["b"], dtype=float).reshape(-1)
            except (KeyError, TypeError, ValueError) as exc:
                raise MinimalRegionError(
                    f"缓存文件里的多面体数据损坏：{exc}", reason=REASON_CACHE_CORRUPT
                ) from exc
            if a.ndim != 2 or a.shape[1] != MINIMAL_DIM or a.shape[0] != b.shape[0]:
                raise MinimalRegionError(
                    f"缓存文件里的多面体形状非法：A{a.shape} vs b{b.shape}"
                    f"（维度必须是 {MINIMAL_DIM}）",
                    reason=REASON_CACHE_CORRUPT,
                )
            region = MinimalRegion(
                id=str(entry.get("id") or _region_id(stored, spec)),
                a=a,
                b=b,
                seed_x=tuple(spec.seed_x),
                spec=spec,
                model_version=stored,
                duration_s=0.0,
                rounds=int(entry.get("rounds", 0)),
                cuts=int(entry.get("cuts", 0)),
                reason=str(entry.get("reason", "")),
            )
            cache.add_spec(spec)
            cache._regions[spec.key()] = region
        LOGGER.info(
            "最小坐标区域缓存已从 %s 载入（%d 个区域）", target, len(cache._specs)
        )
        return cache
