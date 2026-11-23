"""规划服务的内部 IDL（设计 §4.3）—— **契约层唯一真相**。

设计 §4.3 给出的 ``PlanRequest`` / ``PlanResponse`` 在这里被冻结成**可执行、
可校验**的 Python 契约：引擎 / 服务端 / 客户端 / 验收四层共用这一份，不再各长
一份隐式真相。

纪律（与 ``rmctl.process.spec`` / ``rm_executor.contract`` 一致）：

1. **逐字段指名**：字段缺失 / 类型错 / 非有限数 / 未知 goal 类型 →
   :class:`InvalidRequest`（带 ``field`` 与 ``reason``），**不静默忽略、
   不补默认值**。
2. **有限数纪律**：``NaN`` / ``Inf`` 一律拒绝（请求与响应皆然），
   ``to_dict()`` 的输出永远可 JSON 序列化。
3. **不抛裸异常**：**解析**方向（``parse_response_line``）失败时返回
   ``status="INVALID_REQUEST"`` 的响应对象，而不是把异常抛给调用方；
   **构造**方向（``from_dict``）失败时抛 :class:`InvalidRequest`（这是调用方的
   输入错误，必须指名到字段）。
4. **确定性**：`trajectory_digest()` 对「时间 + 点位」做规范化 JSON 摘要，
   供「同请求同种子结果逐点一致」的验收比对（不依赖 dict 顺序）。

单位约定：**时间**用秒（``timeout_ms`` 除外，它是毫秒）、**角度**用 rad、
**长度**用 m。
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

__all__ = [
    "STATUSES",
    "GOAL_TYPES",
    "CONSTRAINT_KEYS",
    "SCENE_OBSTACLE_KEYS",
    "VERIFICATION_KEYS",
    "JOINT_COUNT",
    "POSE_COUNT",
    "InvalidRequest",
    "PlanRequest",
    "PlanResponse",
    "parse_response_line",
    "trajectory_digest",
]

#: 响应状态全集（与设计 §4.3 逐字一致）。
STATUSES: Tuple[str, ...] = (
    "SUCCESS",
    "FAILED",
    "TIMEOUT",
    "CANCELLED",
    "INVALID_REQUEST",
)

#: 支持的目标类型（本层只冻结两种；笛卡尔路径 / 相对约束由后续切片扩展）。
GOAL_TYPES: Tuple[str, ...] = ("joint", "pose")

#: 允许的约束键（时间缩放 / 碰撞裕度 + 速度加速度 jerk 缩放）。
CONSTRAINT_KEYS: Tuple[str, ...] = (
    "time_scaling",
    "collision_margin",
    "velocity_scaling",
    "acceleration_scaling",
    "jerk_scaling",
)

#: 场景里单个障碍物的字段白名单。
SCENE_OBSTACLE_KEYS: Tuple[str, ...] = ("name", "box_size", "position")

#: 校验报告的必备键（四道校验逐项结论 + 总判定 + 问题列表）。
VERIFICATION_KEYS: Tuple[str, ...] = (
    "limits",
    "discrete_collision",
    "continuous_collision",
    "ok",
    "issues",
)

#: 关节数（7 自由度）。
JOINT_COUNT = 7

#: 笛卡尔位姿维数（x, y, z, r, p, y）。
POSE_COUNT = 6


class InvalidRequest(ValueError):
    """非法规划请求：**逐字段指名**。

    :param field: 出错字段名（如 ``"start_state"`` / ``"goal.type"``）
    :param reason: 人类可读的失败原因（含字段名）
    """

    def __init__(self, field: str, reason: str) -> None:
        self.field = field
        self.reason = reason
        super().__init__("%s: %s" % (field, reason))


# --------------------------------------------------------------------------
# 内部校验工具
# --------------------------------------------------------------------------

def _is_num(value: Any) -> bool:
    """是否为有限数值（bool 不算数值）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _require_str(value: Any, field: str) -> str:
    """非空字符串。"""
    if not isinstance(value, str) or not value.strip():
        raise InvalidRequest(field, "必须是非空字符串，当前是 %r" % (value,))
    return value.strip()


def _require_int(value: Any, field: str, minimum: int) -> int:
    """整数（bool 不算整数）。"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidRequest(field, "必须是整数，当前是 %r" % (value,))
    if value < minimum:
        raise InvalidRequest(field, "必须 >= %d，当前是 %r" % (minimum, value))
    return int(value)


def _require_bool(value: Any, field: str) -> bool:
    """布尔。"""
    if not isinstance(value, bool):
        raise InvalidRequest(field, "必须是 true / false，当前是 %r" % (value,))
    return value


def _require_finite_list(value: Any, count: int, field: str) -> List[float]:
    """恰好 ``count`` 个有限数（``NaN`` / ``Inf`` 一律拒绝）。"""
    if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
        raise InvalidRequest(
            field, "必须是 %d 个有限数的列表，当前是 %r" % (count, value)
        )
    if len(value) != count:
        raise InvalidRequest(
            field,
            "必须恰好 %d 个有限数，当前是 %d 个" % (count, len(value)),
        )
    out: List[float] = []
    for index, item in enumerate(value):
        if not _is_num(item):
            raise InvalidRequest(
                "%s[%d]" % (field, index),
                "必须是有限数（NaN / Inf 不接受），当前是 %r" % (item,),
            )
        out.append(float(item))
    return out


# --------------------------------------------------------------------------
# PlanRequest
# --------------------------------------------------------------------------

@dataclass
class PlanRequest:
    """规划请求（设计 §4.3）。

    字段语义：

    * ``plan_id`` —— 请求标识（幂等 / 关联响应用）。
    * ``profile`` —— 规划 profile 名（``FREESPACE`` / ``CARTESIAN`` /
      ``DUAL_RELATIVE``；本层只校验是非空字符串，**不猜**语义）。
    * ``group`` —— 规划组（``left_arm`` / ``right_arm`` / ``dual_arm``）。
    * ``start_state`` —— 起始关节角，**7 个有限数，单位 rad**。
    * ``goal`` —— 目标：``{"type": "joint", "values": [7 个有限数]}`` 或
      ``{"type": "pose", "values": [x, y, z, r, p, y]}``。
    * ``constraints`` —— 约束：``time_scaling``（≤ profile 的
      ``max_time_scaling``）、``collision_margin``（m）等。
    * ``seed`` —— 可复现随机种子（``>= 0``）。
    * ``timeout_ms`` —— **服务端强制中断**的硬超时（毫秒，``> 0``）。
    * ``scene_version`` —— 世界模型版本号（版本不符服务端直接拒绝）。
    * ``scene`` —— 可选场景：``{"obstacles": [{"name", "box_size", "position"}]}``，
      语义 = 在 ``base_link`` 上固定一个盒子障碍物。
    * ``deterministic`` —— 确定性开关（缺省 ``True``）。
    """

    plan_id: str
    profile: str
    group: str
    start_state: List[float]
    goal: Dict[str, Any]
    constraints: Dict[str, Any]
    seed: int
    timeout_ms: int
    scene_version: int
    scene: Optional[Dict[str, Any]] = None
    deterministic: bool = True
    #: 时间缩放上限（**不参与线协议**）：``None`` = 用 ``profiles.yaml`` 的
    #: ``max_time_scaling``；显式给出时用于离线校验 / 测试。
    max_time_scaling: Optional[float] = field(default=None, repr=False)

    def __post_init__(self) -> None:
        self._validate()

    # -- 校验 ------------------------------------------------------------

    def _validate(self) -> None:
        self.plan_id = _require_str(self.plan_id, "plan_id")
        self.profile = _require_str(self.profile, "profile")
        self.group = _require_str(self.group, "group")
        self.start_state = _require_finite_list(
            self.start_state, JOINT_COUNT, "start_state"
        )
        self.goal = self._validate_goal(self.goal)
        self.constraints = self._validate_constraints(self.constraints)
        self.seed = _require_int(self.seed, "seed", 0)
        self.timeout_ms = _require_int(self.timeout_ms, "timeout_ms", 1)
        self.scene_version = _require_int(self.scene_version, "scene_version", 0)
        self.deterministic = _require_bool(self.deterministic, "deterministic")
        if self.scene is not None:
            self.scene = self._validate_scene(self.scene)

    @staticmethod
    def _validate_goal(goal: Any) -> Dict[str, Any]:
        """目标校验：类型必须是 ``joint`` / ``pose``，值必须是有限数。"""
        if not isinstance(goal, dict):
            raise InvalidRequest("goal", "必须是映射，当前是 %r" % (goal,))
        unknown = sorted(key for key in goal if key not in ("type", "values"))
        if unknown:
            raise InvalidRequest(
                "goal.%s" % unknown[0],
                "未知字段（goal 只允许 type / values）",
            )
        goal_type = goal.get("type")
        if goal_type not in GOAL_TYPES:
            raise InvalidRequest(
                "goal.type",
                "未知目标类型 %r（只允许 %s）" % (goal_type, " | ".join(GOAL_TYPES)),
            )
        count = JOINT_COUNT if goal_type == "joint" else POSE_COUNT
        values = _require_finite_list(goal.get("values"), count, "goal.values")
        return {"type": goal_type, "values": values}

    def _validate_constraints(self, constraints: Any) -> Dict[str, Any]:
        """约束校验：白名单 + 有限数 + ``time_scaling`` 上限。"""
        if constraints is None:
            constraints = {}
        if not isinstance(constraints, dict):
            raise InvalidRequest(
                "constraints", "必须是映射，当前是 %r" % (constraints,)
            )
        unknown = sorted(key for key in constraints if key not in CONSTRAINT_KEYS)
        if unknown:
            raise InvalidRequest(
                "constraints.%s" % unknown[0],
                "未知约束（只允许 %s）" % ", ".join(CONSTRAINT_KEYS),
            )
        out: Dict[str, Any] = {}
        for key, value in constraints.items():
            if not _is_num(value):
                raise InvalidRequest(
                    "constraints.%s" % key,
                    "必须是有限数（NaN / Inf 不接受），当前是 %r" % (value,),
                )
            out[key] = float(value)
        if "time_scaling" in out:
            if out["time_scaling"] <= 0.0:
                raise InvalidRequest(
                    "constraints.time_scaling",
                    "必须 > 0，当前是 %r" % (out["time_scaling"],),
                )
            limit = self.max_time_scaling
            if limit is None:
                limit = self._profile_max_time_scaling()
            if limit is not None and out["time_scaling"] > float(limit) + 1e-12:
                raise InvalidRequest(
                    "constraints.time_scaling",
                    "超过 profile 允许的上限 %g（当前是 %r）"
                    % (float(limit), out["time_scaling"]),
                )
        if "collision_margin" in out and out["collision_margin"] < 0.0:
            raise InvalidRequest(
                "constraints.collision_margin",
                "必须 >= 0（米），当前是 %r" % (out["collision_margin"],),
            )
        return out

    @staticmethod
    def _profile_max_time_scaling() -> Optional[float]:
        """从 ``profiles.yaml`` 取 ``max_time_scaling``（**唯一来源**）。

        取不到（文件缺失 / 结构非法）时返回 ``None`` = **不检查上限**
        （配置加载器会另行给出 blocked 证据；本层不补默认值、也不猜上限）。
        """
        try:
            from rmctl.planner_service import config as _config

            result = _config.load_profiles_config()
        except Exception:  # noqa: BLE001 - 配置不可得 ≠ 请求非法
            return None
        if not isinstance(result, dict) or result.get("status") != "ok":
            return None
        spec = result.get("spec") or {}
        value = spec.get("max_time_scaling")
        if not _is_num(value):
            return None
        return float(value)

    @staticmethod
    def _validate_scene(scene: Any) -> Dict[str, Any]:
        """场景校验：障碍物为盒子（尺寸 + 位置，均为有限数）。"""
        if not isinstance(scene, dict):
            raise InvalidRequest("scene", "必须是映射，当前是 %r" % (scene,))
        unknown = sorted(key for key in scene if key != "obstacles")
        if unknown:
            raise InvalidRequest(
                "scene.%s" % unknown[0], "未知字段（scene 只允许 obstacles）"
            )
        obstacles = scene.get("obstacles")
        if obstacles is None:
            obstacles = []
        if not isinstance(obstacles, list):
            raise InvalidRequest(
                "scene.obstacles", "必须是列表，当前是 %r" % (obstacles,)
            )
        out: List[Dict[str, Any]] = []
        for index, raw in enumerate(obstacles):
            prefix = "scene.obstacles[%d]" % index
            if not isinstance(raw, dict):
                raise InvalidRequest(prefix, "必须是映射，当前是 %r" % (raw,))
            extra = sorted(key for key in raw if key not in SCENE_OBSTACLE_KEYS)
            if extra:
                raise InvalidRequest(
                    "%s.%s" % (prefix, extra[0]),
                    "未知字段（障碍物只允许 %s）" % ", ".join(SCENE_OBSTACLE_KEYS),
                )
            name = _require_str(raw.get("name"), "%s.name" % prefix)
            box_size = _require_finite_list(
                raw.get("box_size"), 3, "%s.box_size" % prefix
            )
            position = _require_finite_list(
                raw.get("position"), 3, "%s.position" % prefix
            )
            for axis, size in enumerate(box_size):
                if size <= 0.0:
                    raise InvalidRequest(
                        "%s.box_size[%d]" % (prefix, axis),
                        "必须是正数（米），当前是 %r" % (size,),
                    )
            out.append(
                {"name": name, "box_size": box_size, "position": position}
            )
        return {"obstacles": out}

    # -- 序列化 ----------------------------------------------------------

    @classmethod
    def from_dict(
        cls,
        data: Any,
        max_time_scaling: Optional[float] = None,
        ws: Optional[str] = None,
    ) -> "PlanRequest":
        """从字典构造请求；非法即抛 :class:`InvalidRequest`（逐字段指名）。

        :param data: 请求字典（**字段名与设计 §4.3 一致**）
        :param max_time_scaling: 时间缩放上限；``None`` = 用
            ``profiles.yaml`` 的 ``max_time_scaling``（唯一来源）
        :param ws: 工作区根（用于查 ``profiles.yaml``）
        """
        if not isinstance(data, dict):
            raise InvalidRequest("<request>", "请求必须是映射，当前是 %r" % (data,))
        allowed = (
            "plan_id",
            "profile",
            "group",
            "start_state",
            "goal",
            "constraints",
            "seed",
            "timeout_ms",
            "scene_version",
            "scene",
            "deterministic",
        )
        unknown = sorted(key for key in data if key not in allowed)
        if unknown:
            raise InvalidRequest(
                unknown[0], "未知请求字段（不静默忽略）"
            )
        for key in ("plan_id", "profile", "group", "start_state", "goal",
                    "seed", "timeout_ms", "scene_version"):
            if key not in data:
                raise InvalidRequest(
                    key, "必填字段缺失: %s（代码不补默认值）" % key
                )
        if "constraints" not in data:
            raise InvalidRequest("constraints", "必填字段缺失（代码不补默认值）")
        limit = max_time_scaling
        if limit is None and ws is not None:
            limit = _max_time_scaling_from_ws(ws)
        request = cls(
            plan_id=data["plan_id"],
            profile=data["profile"],
            group=data["group"],
            start_state=data["start_state"],
            goal=data["goal"],
            constraints=data["constraints"],
            seed=data["seed"],
            timeout_ms=data["timeout_ms"],
            scene_version=data["scene_version"],
            scene=data.get("scene"),
            deterministic=data.get("deterministic", True),
            max_time_scaling=limit,
        )
        return request

    def to_dict(self) -> Dict[str, Any]:
        """稳定可 JSON 序列化的请求字典（键序固定）。"""
        return {
            "plan_id": self.plan_id,
            "profile": self.profile,
            "group": self.group,
            "start_state": [float(v) for v in self.start_state],
            "goal": {
                "type": self.goal["type"],
                "values": [float(v) for v in self.goal["values"]],
            },
            "constraints": {
                key: float(value) for key, value in self.constraints.items()
            },
            "seed": int(self.seed),
            "timeout_ms": int(self.timeout_ms),
            "scene_version": int(self.scene_version),
            "scene": None if self.scene is None else {
                "obstacles": [
                    {
                        "name": item["name"],
                        "box_size": [float(v) for v in item["box_size"]],
                        "position": [float(v) for v in item["position"]],
                    }
                    for item in self.scene.get("obstacles", [])
                ]
            },
            "deterministic": bool(self.deterministic),
        }

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, PlanRequest):
            return NotImplemented
        return self.to_dict() == other.to_dict()


def _max_time_scaling_from_ws(ws: str) -> Optional[float]:
    """从指定工作区的 ``profiles.yaml`` 取 ``max_time_scaling``。"""
    try:
        from rmctl.planner_service import config as _config

        result = _config.load_profiles_config(ws)
    except Exception:  # noqa: BLE001 - 配置不可得 ≠ 请求非法
        return None
    if not isinstance(result, dict) or result.get("status") != "ok":
        return None
    value = (result.get("spec") or {}).get("max_time_scaling")
    if not _is_num(value):
        return None
    return float(value)


# --------------------------------------------------------------------------
# PlanResponse
# --------------------------------------------------------------------------

def _default_verification() -> Dict[str, Any]:
    """四道校验报告的默认骨架（键齐全，逐项结论为空）。"""
    return {
        "limits": {"ok": False, "issues": []},
        "discrete_collision": {"ok": False, "issues": []},
        "continuous_collision": {"ok": False, "issues": []},
        "ok": False,
        "issues": [],
    }


@dataclass
class PlanResponse:
    """规划响应（设计 §4.3）。

    * ``verification`` —— 四道校验（限位 / 离散碰撞 / 连续碰撞 / 时间可实现性）
      的逐项结论；**``SUCCESS`` 的语义是「四道全过」**，不是「规划器说 OK」。
    * ``failure_reason`` —— 可解释的失败原因（哪一节点失败 / 哪一约束不满足）。
    * ``metrics`` —— 各节点耗时、种子、规划器个数、总耗时、复检与重定时缩放。
    """

    plan_id: str = ""
    status: str = "INVALID_REQUEST"
    trajectory: Optional[Dict[str, Any]] = None
    verification: Dict[str, Any] = field(default_factory=_default_verification)
    failure_reason: str = ""
    metrics: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.status not in STATUSES:
            raise ValueError(
                "未知响应状态 %r（只允许 %s）" % (self.status, " | ".join(STATUSES))
            )
        if self.verification is None:
            self.verification = _default_verification()
        missing = [key for key in VERIFICATION_KEYS if key not in self.verification]
        if missing:
            raise ValueError(
                "verification 缺字段: %s" % ", ".join(missing)
            )

    def to_dict(self) -> Dict[str, Any]:
        """稳定可 JSON 序列化的响应字典（键序固定）。"""
        return {
            "plan_id": self.plan_id,
            "status": self.status,
            "trajectory": self.trajectory,
            "verification": self.verification,
            "failure_reason": self.failure_reason,
            "metrics": self.metrics,
        }


def parse_response_line(line: str) -> PlanResponse:
    """解析一行 JSON 响应 → :class:`PlanResponse`。

    **解析失败不抛裸异常**：返回 ``status="INVALID_REQUEST"`` 的响应对象，
    ``failure_reason`` 里说明原因（调用方据此走错误路径，不崩客户端）。
    """
    if not isinstance(line, str):
        return PlanResponse(
            plan_id="",
            status="INVALID_REQUEST",
            failure_reason="响应行不是字符串（%s）" % type(line).__name__,
        )
    text = line.strip()
    if not text:
        return PlanResponse(
            plan_id="",
            status="INVALID_REQUEST",
            failure_reason="响应行为空",
        )
    try:
        data = json.loads(text)
    except Exception as exc:  # noqa: BLE001 - JSON 解析异常统一转响应对象
        return PlanResponse(
            plan_id="",
            status="INVALID_REQUEST",
            failure_reason="响应行不是合法 JSON（%s）: %s"
            % (type(exc).__name__, exc),
        )
    if not isinstance(data, dict):
        return PlanResponse(
            plan_id="",
            status="INVALID_REQUEST",
            failure_reason="响应不是 JSON 对象（%s）" % type(data).__name__,
        )
    status = data.get("status")
    if status not in STATUSES:
        return PlanResponse(
            plan_id=str(data.get("plan_id") or ""),
            status="INVALID_REQUEST",
            failure_reason="响应状态非法: %r（只允许 %s）"
            % (status, " | ".join(STATUSES)),
        )
    verification = data.get("verification")
    if not isinstance(verification, dict):
        verification = _default_verification()
    else:
        merged = _default_verification()
        merged.update(verification)
        verification = merged
    metrics = data.get("metrics")
    if not isinstance(metrics, dict):
        metrics = {}
    trajectory = data.get("trajectory")
    if trajectory is not None and not isinstance(trajectory, dict):
        trajectory = None
    return PlanResponse(
        plan_id=str(data.get("plan_id") or ""),
        status=status,
        trajectory=trajectory,
        verification=verification,
        failure_reason=str(data.get("failure_reason") or ""),
        metrics=metrics,
    )


# --------------------------------------------------------------------------
# 确定性摘要
# --------------------------------------------------------------------------

def _canonical_trajectory(trajectory: Any) -> str:
    """轨迹的规范化文本（**不依赖 dict 顺序**，float 用 ``repr``）。

    支持两种点位表达：

    * ``{"points": [{"t": 0.0, "positions": [...]}, ...]}``（推荐）；
    * ``{"points": [[...], ...], "times": [...]}``。
    """
    if trajectory is None:
        return "times:;points:"
    if not isinstance(trajectory, dict):
        raise ValueError("trajectory 必须是映射（当前 %s）" % type(trajectory).__name__)
    points = trajectory.get("points") or []
    if not isinstance(points, list):
        raise ValueError("trajectory.points 必须是列表")
    times: List[float] = []
    rows: List[List[float]] = []
    for index, item in enumerate(points):
        if isinstance(item, dict):
            raw_t = item.get("t", item.get("time"))
            raw_q = item.get("positions", item.get("q", item.get("joints")))
            if not _is_num(raw_t):
                raise ValueError("trajectory.points[%d] 缺少合法时间戳" % index)
            if not isinstance(raw_q, (list, tuple)):
                raise ValueError("trajectory.points[%d] 缺少合法点位" % index)
            times.append(float(raw_t))
            rows.append([float(v) for v in raw_q])
        elif isinstance(item, (list, tuple)):
            raw_times = trajectory.get("times") or []
            if not isinstance(raw_times, list) or index >= len(raw_times):
                raise ValueError(
                    "trajectory.points[%d] 是裸点位但缺少对应时间戳" % index
                )
            times.append(float(raw_times[index]))
            rows.append([float(v) for v in item])
        else:
            raise ValueError("trajectory.points[%d] 类型非法" % index)
    for row in rows:
        for value in row:
            if not math.isfinite(value):
                raise ValueError("trajectory 点位含非有限数")
    for value in times:
        if not math.isfinite(value):
            raise ValueError("trajectory 时间戳含非有限数")
    return "times:%s;points:%s" % (
        ",".join(repr(t) for t in times),
        "|".join(",".join(repr(v) for v in row) for row in rows),
    )


def trajectory_digest(trajectory: Any) -> str:
    """轨迹摘要（sha256 十六进制）—— 供「同请求同种子逐点一致」比对。

    规范化：时间 + 点位 → ``repr(float)`` 拼成的稳定文本 → sha256。
    **不依赖 dict 顺序**；``NaN`` / ``Inf`` 直接报错（不产生误导性的摘要）。
    """
    canonical = _canonical_trajectory(trajectory)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
