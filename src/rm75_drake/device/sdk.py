"""厂商 SDK 的**懒加载**封装与 rad↔deg **唯一转换点**（S03-T01）。

为什么懒加载
------------
官方 SDK 的 PyPI **分发名**是 ``robotic-arm``，但**导入名**是 ``Robotic_Arm``
（大写 R/A）——这是个经典陷阱。本机 CI 环境**没有安装**该 SDK
（离线 wheel 只在 ``vendor/wheels/``），因此设备层**不得**在 import 时硬依赖它：
所有 SDK 访问都走 :func:`load_sdk` 的懒加载路径，缺失时抛**显式**
:class:`NotAvailableError`（沿用 S02 :mod:`rm75_drake.viz` 的约定：消息必须说明
**为什么**不可用），**绝不**抛裸 :class:`ImportError`、更不假装"已连接"。

为什么 rad↔deg 转换只此一处
--------------------------
SDK 全线用**度**（``rm_movej_canfd`` 的关节角、``rm_joint_status_t.joint_position``
都是 °），而本项目全线用**弧度**（scenario / trajectory / limits / recording 一律 rad）。
把度当弧度下发 ⇒ 关节瞬间飞到 57 倍位置（180° 被当成 180 rad）。
⇒ 转换**只允许**在本模块实现一次，其他模块一律调用 :func:`deg_to_rad` /
:func:`rad_to_deg`；:func:`assert_single_conversion_point` 会扫描整个包的源码，
发现第二份实现即报错（测试据此守住不变量）。

运行时调用审计（R008 的运行时那半）
--------------------------------
:func:`load_sdk` 返回的 SDK 命名空间被 :class:`SdkProxy` 包装：每次调用都会记进
:class:`SdkAudit`，**非白名单符号**的调用次数可被断言为 0
（静态扫描器 :mod:`scripts.scan_forbidden_apis` 是另一半证据）。

禁用名单（厂商解算接口）
----------------------
:func:`load_forbidden_apis` 把 ``versions.lock:forbidden_apis`` 读进运行时
（**唯一真相源**；模块级缓存，不把文件 I/O 放进 5 ms 热路径）。
:class:`SdkAudit` 因此多出 :attr:`SdkAudit.forbidden_calls` /
:attr:`SdkAudit.forbidden_call_count` 两个计数——**命中禁用名单（含变体名）
的调用数必须为 0**，这正是「静态检查 + 运行时审计」双证里的运行时那一证。
"""

from __future__ import annotations

import importlib
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..viz import NotAvailableError

__all__ = [
    "NotAvailableError",
    "SDK_IMPORT_NAME",
    "SDK_DIST_NAME",
    "SDK_UNAVAILABLE_REASON",
    "SDK_CALL_WHITELIST",
    "CONVERSION_FUNCTION_NAMES",
    "deg_to_rad",
    "rad_to_deg",
    "sdk_available",
    "load_sdk",
    "load_rm_interface",
    "SdkCapabilities",
    "probe_capabilities",
    "SdkAudit",
    "SdkProxy",
    "ForbiddenApisError",
    "load_forbidden_apis",
    "clear_forbidden_apis_cache",
    "is_forbidden_api_name",
    "audit_snapshot",
    "assert_single_conversion_point",
]


#: SDK 的**导入名**（大写 R/A；PyPI 分发名是 ``robotic-arm``）。
SDK_IMPORT_NAME = "Robotic_Arm"

#: SDK 的 **PyPI 分发名**（``pip install robotic-arm==1.1.7``）。
SDK_DIST_NAME = "robotic-arm"

#: SDK 不可用时的**标准原因说明**（写进异常消息，绝不隐瞒）。
SDK_UNAVAILABLE_REASON = (
    "睿尔曼官方 SDK 不可用：(1) 本机未安装（分发名 robotic-arm，导入名 Robotic_Arm，"
    "大写 R/A）；(2) 离线 wheel 只在 vendor/wheels/ 里，未 pip 安装；"
    "(3) 即使安装了也必须在真机所在网段才能建立 TCP/UDP 连接。"
    "设备层以懒加载方式使用它：缺失时抛 NotAvailableError，绝不静默降级。"
)

#: 设备层允许调用的 SDK 符号白名单（研究 F6：全部在 ``.so`` 导出符号里核过，
#: 且与 ``versions.lock:forbidden_apis`` 不相交）。
SDK_CALL_WHITELIST: frozenset = frozenset(
    {
        # 透传下发（唯一运动通道）
        "rm_movej_canfd",
        # 使能 / 去使能
        "rm_set_joint_en_state",
        # 错误码读取 / 清除
        "rm_get_joint_err_flag",
        "rm_set_joint_clear_err",
        "rm_clear_system_err",
        "rm_clear_system_runtime",
        # 急停 / 缓停 / 暂停 / 继续
        "rm_set_arm_emergency_stop",
        "rm_set_arm_slow_stop",
        "rm_set_arm_pause",
        "rm_set_arm_continue",
        "rm_set_arm_stop",
        # 限位放宽 / 恢复
        "rm_set_joint_min_pos",
        "rm_set_joint_max_pos",
        # 状态读取
        "rm_get_current_joint_current",
        "rm_get_current_arm_state",
        # 夹爪
        "rm_set_gripper_position",
        "rm_get_gripper_state",
        # UDP 主动上报（三线程模式）
        "rm_set_realtime_push",
        "rm_realtime_arm_state_call_back",
    }
)

#: rad↔deg 转换函数的**唯一**实现名（扫描用）。
CONVERSION_FUNCTION_NAMES: Tuple[str, ...] = ("deg_to_rad", "rad_to_deg")

#: 每弧度的度数（内部常量；外部**不得**另建换算）。
_DEG_PER_RAD = 180.0 / math.pi


# ── 唯一转换点 ──────────────────────────────────────────────────────────
def deg_to_rad(deg: float) -> float:
    """度 → 弧度（**全项目唯一实现**，见模块 docstring）。

    参数
    ----
    deg:
        角度（度）。

    返回
    ----
    角度（弧度）。
    """
    return float(deg) / _DEG_PER_RAD


def rad_to_deg(rad: float) -> float:
    """弧度 → 度（**全项目唯一实现**，见模块 docstring）。

    参数
    ----
    rad:
        角度（弧度）。

    返回
    ----
    角度（度）。
    """
    return float(rad) * _DEG_PER_RAD


#: 转换函数的源码扫描模式（``def deg_to_rad(`` 之类）。
_DEF_PATTERN = re.compile(
    r"^\s*def\s+(" + "|".join(CONVERSION_FUNCTION_NAMES) + r")\s*\(", re.MULTILINE
)
#: 其它「手写换算」的特征：``math.radians`` / ``math.degrees`` / ``np.deg2rad`` /
#: ``np.rad2deg`` / ``180.0 / math.pi`` / ``math.pi / 180``。
_INLINE_PATTERN = re.compile(
    r"math\.(radians|degrees)\s*\(|np\.(deg2rad|rad2deg)\s*\(|"
    r"180(?:\.0)?\s*/\s*math\.pi|math\.pi\s*/\s*180"
)

#: 字符串字面量（扫描前先剥掉：文档/提示语里的换算说明**不是**第二份实现）。
_STRING_LITERAL_PATTERN = re.compile(r"'(?:[^'\\]|\\.)*'|\"(?:[^\"\\]|\\.)*\"")


def _strip_string_literals(line: str) -> str:
    """剥掉一行里的字符串字面量（避免把提示语误判成转换实现）。"""
    return _STRING_LITERAL_PATTERN.sub('""', line)


def assert_single_conversion_point(root: Optional[Path] = None) -> List[str]:
    """扫描包源码，确认 rad↔deg 转换**只有本模块**一处实现。

    参数
    ----
    root:
        扫描根目录；``None`` → 本包目录（``src/rm75_drake``）。

    返回
    ----
    违规位置列表（形如 ``"path:line"``）；**空列表 = 合规**。

    异常
    ----
    AssertionError
        发现第二份转换实现（调用方可用 ``assert not offenders`` 表达契约）。
    """
    package_root = Path(root) if root is not None else Path(__file__).resolve().parent.parent
    this_file = Path(__file__).resolve()
    offenders: List[str] = []
    for path in sorted(package_root.rglob("*.py")):
        resolved = path.resolve()
        if resolved == this_file:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), start=1):
            code = _strip_string_literals(line)
            if _DEF_PATTERN.match(code) or _INLINE_PATTERN.search(code):
                offenders.append(f"{path}:{lineno}: {line.strip()}")
    if offenders:
        raise AssertionError(
            "rad↔deg 转换必须只在 device/sdk.py 实现一次，发现其它实现：\n"
            + "\n".join(offenders)
        )
    return offenders


# ── SDK 懒加载 ──────────────────────────────────────────────────────────
def sdk_available() -> bool:
    """返回 SDK 是否**当前可导入**（不抛异常）。

    探测失败的原因**不吞**：需要原因时用 :func:`load_sdk`（会抛
    :class:`NotAvailableError` 并写明原因）。
    """
    try:
        importlib.import_module(SDK_IMPORT_NAME)
    except Exception:
        return False
    return True


def load_sdk() -> Any:
    """懒加载并返回 ``Robotic_Arm`` 顶层模块。

    异常
    ----
    :class:`NotAvailableError`
        SDK 未安装 / 导入失败（消息含 :data:`SDK_UNAVAILABLE_REASON` 与原始异常）。
    """
    try:
        return importlib.import_module(SDK_IMPORT_NAME)
    except Exception as exc:
        raise NotAvailableError(
            f"{SDK_UNAVAILABLE_REASON}（导入名 {SDK_IMPORT_NAME!r}，"
            f"分发名 {SDK_DIST_NAME!r}；原始异常：{exc!r}）"
        ) from exc


def load_rm_interface() -> Any:
    """懒加载并返回 ``Robotic_Arm.rm_robot_interface`` 模块（方法与 docstring 在此）。"""
    try:
        return importlib.import_module(f"{SDK_IMPORT_NAME}.rm_robot_interface")
    except Exception as exc:
        raise NotAvailableError(
            f"{SDK_UNAVAILABLE_REASON}（需要 {SDK_IMPORT_NAME}.rm_robot_interface；"
            f"原始异常：{exc!r}）"
        ) from exc


# ── 能力探测 ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class SdkCapabilities:
    """SDK / 控制器的能力探测结果。

    字段
    ----
    arm_dof:
        机械臂自由度（探测不到时为 ``None``，**不猜**）。
    controller_generation:
        控制器代次（探测不到时为 ``None``——开放问题 1，真机首测确认）。
    triple_mode_supported:
        是否支持三线程模式（UDP 主动上报的前提）。
    detail:
        人类可读的说明（探测到了什么、没探测到什么）。
    """

    arm_dof: Optional[int]
    controller_generation: Optional[int]
    triple_mode_supported: bool
    detail: str


#: 控制器代次可能出现的属性名（SDK 未公开统一接口，逐个尝试）。
_GENERATION_ATTRS: Tuple[str, ...] = (
    "controller_type",
    "arm_controller_type",
    "controller_generation",
)


def probe_capabilities(arm: Any = None) -> SdkCapabilities:
    """探测 SDK / 控制器能力（控制器代次、臂自由度、三线程模式支持）。

    参数
    ----
    arm:
        已初始化的 ``RoboticArm`` 实例；``None`` → 只做静态探测。

    返回
    ----
    :class:`SdkCapabilities`（探测不到的字段一律 ``None`` / ``False``，**不猜**）。
    """
    arm_dof: Optional[int] = None
    generation: Optional[int] = None
    triple = False
    notes: List[str] = []

    if arm is not None:
        raw_dof = getattr(arm, "arm_dof", None)
        if isinstance(raw_dof, int) and raw_dof > 0:
            arm_dof = raw_dof
            notes.append(f"arm_dof={arm_dof}（来自 arm.arm_dof）")
        else:
            notes.append("arm_dof 未知（SDK 实例未暴露 arm_dof 属性）")
        for attr in _GENERATION_ATTRS:
            value = getattr(arm, attr, None)
            if isinstance(value, int):
                generation = value
                notes.append(f"controller_generation={generation}（来自 arm.{attr}）")
                break
        else:
            notes.append("controller_generation 未知（开放问题 1，真机首测确认）")
        triple = bool(getattr(arm, "triple_mode", False)) or arm_dof is not None
        notes.append(
            "三线程模式：UDP 主动上报只在三线程模式可用"
            f"（triple_mode={triple}）"
        )
    else:
        notes.append("未提供 arm 实例，只做静态探测")

    return SdkCapabilities(
        arm_dof=arm_dof,
        controller_generation=generation,
        triple_mode_supported=triple,
        detail="；".join(notes),
    )


# ── 禁用名单（厂商解算接口）──────────────────────────────────────────────
class ForbiddenApisError(RuntimeError):
    """禁用名单读取失败（锁文件缺失 / 解析失败 / 段落非法）。"""


#: 禁用名单缓存：键 = (锁文件绝对路径, mtime_ns, size)，值 = 冻结名单。
#: 模块级缓存 ⇒ **不把文件 I/O 放进 5 ms 热路径**；键里带 mtime/size ⇒ 锁文件
#: 改动后自动失效（不会拿旧名单当新名单用）。
_FORBIDDEN_APIS_CACHE: Dict[Tuple[str, int, int], frozenset] = {}


def load_forbidden_apis(lock_path: Optional[Path] = None) -> frozenset:
    """读取 ``versions.lock:forbidden_apis``（厂商解算接口禁用名单）。

    参数
    ----
    lock_path:
        锁文件路径；``None`` → :data:`rm75_drake.paths.versions_lock_path`
        （**唯一真相源**：任何模块都不得另立一份名单）。

    返回
    ----
    ``frozenset``：禁用接口名（与锁文件 ``forbidden_apis`` 段逐项一致）。

    异常
    ----
    ForbiddenApisError
        锁文件不存在 / 不可读 / 解析失败 / ``forbidden_apis`` 段缺失或为空
        （**绝不**静默返回空名单——那会让运行时审计变成假绿灯）。
    """
    from ..paths import versions_lock_path

    path = Path(lock_path) if lock_path is not None else Path(versions_lock_path)
    try:
        stat = path.stat()
    except OSError as exc:
        raise ForbiddenApisError(
            f"读取禁用名单失败（锁文件不可读）：{path}（{exc}）"
        ) from exc
    key = (str(path.resolve()), int(stat.st_mtime_ns), int(stat.st_size))
    cached = _FORBIDDEN_APIS_CACHE.get(key)
    if cached is not None:
        return cached

    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺依赖
        raise ForbiddenApisError(
            "读取禁用名单需要 PyYAML（pip install pyyaml）"
        ) from exc
    try:
        data: Any = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ForbiddenApisError(f"锁文件解析失败：{path}（{exc}）") from exc
    if not isinstance(data, dict):
        raise ForbiddenApisError(f"锁文件结构非法（应为映射）：{path}")
    names = data.get("forbidden_apis")
    if not isinstance(names, list):
        raise ForbiddenApisError(f"{path}:forbidden_apis 缺失或为空")
    frozen = frozenset(str(name).strip() for name in names if str(name).strip())
    if not frozen:
        raise ForbiddenApisError(f"{path}:forbidden_apis 缺失或为空")
    _FORBIDDEN_APIS_CACHE[key] = frozen
    return frozen


def clear_forbidden_apis_cache() -> None:
    """清空禁用名单缓存（测试用：改过锁文件后强制重读）。"""
    _FORBIDDEN_APIS_CACHE.clear()


#: 变体名后缀的形态：只允许标识符字符（字母 / 数字 / 下划线）。
_VARIANT_SUFFIX_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")


def is_forbidden_api_name(
    name: str,
    *,
    forbidden: Optional[frozenset] = None,
    whitelist: Optional[frozenset] = None,
) -> bool:
    """判断一个 SDK 符号名是否命中禁用名单（含**变体名**）。

    判定规则
    --------
    1. **精确命中**禁用名单 ⇒ 命中；
    2. **变体名**：以某个禁用名为前缀、后接只含标识符字符的后缀
       （例如 ``rm_algo_inverse_kinematics_v2``）⇒ 命中
       —— 名单未列变体时**也不漏检**；
    3. **白名单优先**：白名单里的名字一律不命中
       （``rm_movej_canfd`` 与禁用名 ``rm_movej`` 前缀相近，但它是允许的
       透传接口 ⇒ 不得前缀误伤）。

    参数
    ----
    name:
        待判定的符号名。
    forbidden:
        禁用名单；``None`` → :func:`load_forbidden_apis`（锁文件唯一真相源）。
    whitelist:
        白名单；``None`` → :data:`SDK_CALL_WHITELIST`。
    """
    forbidden_set = forbidden if forbidden is not None else load_forbidden_apis()
    allowed = whitelist if whitelist is not None else SDK_CALL_WHITELIST
    if name in allowed:
        return False
    if name in forbidden_set:
        return True
    for base in forbidden_set:
        if not name.startswith(base):
            continue
        suffix = name[len(base) :]
        if suffix and _VARIANT_SUFFIX_PATTERN.match(suffix):
            return True
    return False


# ── 运行时调用审计 ──────────────────────────────────────────────────────
@dataclass
class SdkAudit:
    """SDK 调用审计计数器（R008 的运行时那半）。

    字段
    ----
    whitelist:
        允许调用的符号集合。
    calls:
        全部调用记录（``(符号名, 参数摘要)`` 顺序）。
    non_whitelist_calls:
        **非白名单**调用记录（必须为空 —— 否则说明设备层越界调用了厂商接口）。
    forbidden:
        **禁用名单**（厂商解算接口；默认来自 ``versions.lock:forbidden_apis``）。
    forbidden_calls:
        **命中禁用名单**的调用记录（必须为空 —— 否则说明主链路调用了厂商解算接口）。
    """

    whitelist: frozenset = field(default_factory=lambda: SDK_CALL_WHITELIST)
    calls: List[Tuple[str, str]] = field(default_factory=list)
    non_whitelist_calls: List[Tuple[str, str]] = field(default_factory=list)
    forbidden: frozenset = field(default_factory=load_forbidden_apis)
    forbidden_calls: List[Tuple[str, str]] = field(default_factory=list)

    def record(self, name: str, args_repr: str = "") -> None:
        """记录一次调用（内部使用）。

        既有的 ``calls`` / ``non_whitelist_calls`` 语义**一字不改**；
        额外追加**禁用名单命中**记录（含变体名）。
        """
        self.calls.append((name, args_repr))
        if name not in self.whitelist:
            self.non_whitelist_calls.append((name, args_repr))
        if is_forbidden_api_name(
            name, forbidden=self.forbidden, whitelist=self.whitelist
        ):
            self.forbidden_calls.append((name, args_repr))

    @property
    def total_call_count(self) -> int:
        """总调用次数。"""
        return len(self.calls)

    @property
    def non_whitelist_call_count(self) -> int:
        """非白名单调用次数（**必须为 0**）。"""
        return len(self.non_whitelist_calls)

    @property
    def forbidden_call_count(self) -> int:
        """命中禁用名单的调用次数（**必须为 0**）。"""
        return len(self.forbidden_calls)

    def called(self, name: str) -> int:
        """返回某个符号被调用的次数。"""
        return sum(1 for call, _ in self.calls if call == name)

    def reset(self) -> None:
        """清空计数（用于分段统计）。"""
        self.calls.clear()
        self.non_whitelist_calls.clear()
        self.forbidden_calls.clear()


def audit_snapshot(audit: Optional[SdkAudit] = None) -> Dict[str, Any]:
    """把审计计数器压成**可 JSON 序列化**的字典（CLI / 脚本输出用）。

    参数
    ----
    audit:
        审计计数器；``None``（例如离线替身传输层，没有 SDK 调用可审）
        ⇒ 计数全为 0。

    字段：``total_call_count`` / ``non_whitelist_call_count`` /
    ``forbidden_call_count`` / ``forbidden_calls``。
    """
    if audit is None:
        return {
            "total_call_count": 0,
            "non_whitelist_call_count": 0,
            "forbidden_call_count": 0,
            "forbidden_calls": [],
        }
    return {
        "total_call_count": audit.total_call_count,
        "non_whitelist_call_count": audit.non_whitelist_call_count,
        "forbidden_call_count": audit.forbidden_call_count,
        "forbidden_calls": [[name, args] for name, args in audit.forbidden_calls],
    }


class SdkProxy:
    """SDK 命名空间的**调用代理**（自动审计每次调用）。

    用法::

        proxy = SdkProxy(load_sdk(), audit)
        proxy.rm_movej_canfd([0.0] * 7, follow=True, trajectory_mode=0)

    * 属性访问被代理成「可调用对象」，调用会先记账再转发；
    * 目标对象上不存在的属性 → :class:`AttributeError`（不静默返回空函数）。
    """

    def __init__(self, target: Any, audit: Optional[SdkAudit] = None) -> None:
        self._target = target
        self.audit = audit if audit is not None else SdkAudit()
        #: 取子模块的记录（属性访问；**不**计入 calls / 非白名单 / 禁用计数）。
        self.submodule_accesses: List[str] = []

    @property
    def target(self) -> Any:
        """被代理的原始对象。"""
        return self._target

    def get_submodule(self, name: str) -> Any:
        """取**真实子模块**（例如 ``rm_robot_interface``）。

        为什么单独开这个口子
        ------------------
        SDK 的子模块不是「函数调用」，只是一次属性访问；把它塞进
        :meth:`__getattr__` 会被当成一次 SDK 调用记账（虚增计数）。
        ⇒ 这里记录一次**属性访问**（:attr:`submodule_accesses`），
        **不**计入 ``calls`` / 非白名单 / 禁用名单计数。
        """
        if name.startswith("_"):
            raise AttributeError(name)
        self.submodule_accesses.append(name)
        return getattr(self._target, name)

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        attr = getattr(self._target, name)

        def _wrapper(*args: Any, **kwargs: Any) -> Any:
            self.audit.record(name, _args_repr(args, kwargs))
            return attr(*args, **kwargs)

        _wrapper.__name__ = name
        return _wrapper

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return f"SdkProxy(target={self._target!r}, audit={self.audit!r})"


def _args_repr(args: Sequence[Any], kwargs: Dict[str, Any]) -> str:
    """把调用参数压成一行摘要（审计记录用，避免持有了大对象）。"""
    parts = [repr(a) for a in args]
    parts.extend(f"{k}={v!r}" for k, v in kwargs.items())
    text = ", ".join(parts)
    return text if len(text) <= 200 else text[:197] + "..."
