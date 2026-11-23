"""RM-API2 装载层：``.so`` 查找 → 符号绑定（**只绑清单内**）→ 句柄生命周期。

设计要点（R003 厂商解耦的代码层强制）：

* **唯一真值**：符号许可清单来自 ``rm_hw_bridge/contract.yaml``；本模块**不硬编码**
  任何白/黑名单符号名。
* **只绑清单内符号**：黑名单符号**连 ``getattr`` 都不做**（防止误用）；
  绑定对象 ``SymbolBinding`` 只暴露清单符号，其它名字一律 ``AttributeError``。
* **缺符号不抛裸异常**：缺失清单以 ``missing_symbols`` 列表给出。
* **可注入**：允许传入已加载的 ``lib`` 对象（测试用假 SDK），
  测试因此不依赖 ``.deps/``（gitignored）里的真实 ``.so``。
* **零 rmctl 依赖**：硬件层不得反向依赖体验层。

``.so`` 查找优先级（与 ``rmctl/hwcollect.py:sdk_candidates()`` 同语义，S01 已验证）：

1. 环境变量 ``RM_API2_LIB``
2. ``<repo>/.deps/rm_api2/libapi_c.so``
3. ``<repo>/.deps/rm_api2/`` 目录内任意 ``.so``
"""
from __future__ import annotations

import ctypes
import os
from typing import Any, Dict, Iterable, List, Optional

import yaml

__all__ = [
    "CONTRACT_PATH",
    "ENV_SDK_LIB",
    "MissingSymbolsError",
    "RmBridge",
    "RmBridgeError",
    "RobotArm",
    "SymbolBinding",
    "blacklist_names",
    "blacklist_prefixes",
    "contract_symbols",
    "core_symbols",
    "find_library",
    "load_contract",
    "repo_root",
    "sdk_candidates",
    "support_symbols",
    "validate_contract",
]

_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

#: 符号契约（唯一真值）
CONTRACT_PATH = os.path.join(_PACKAGE_DIR, "contract.yaml")

#: 指定 RM-API2 动态库的环境变量
ENV_SDK_LIB = "RM_API2_LIB"

#: 仓库内物化 SDK 的相对路径（.deps/ 是 gitignored，不入库）
DEPS_REL = os.path.join(".deps", "rm_api2", "libapi_c.so")

#: RM-API2 三线程模式（不开三线程收不到 UDP 主动上报）。
#: 取值来自 ABI 唯一真值（rm_define.h 的 rm_thread_mode_e 枚举）：
#: SINGLE=0 / DUAL=1 / **TRIPLE=2**——与 abi_spec.yaml + 生成物一致。
RM_TRIPLE_MODE_E = 2


# --------------------------------------------------------------------- 契约
def load_contract(path: Optional[str] = None) -> Dict[str, Any]:
    """读取符号契约（``contract.yaml``）。

    契约是机器可读的唯一真值：装载器与审计器都只读它。
    """
    contract_path = path or CONTRACT_PATH
    with open(contract_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError("契约文件格式错误（顶层必须是映射）: %s" % contract_path)
    return data


def _entry_names(entries: Iterable[Any]) -> List[str]:
    out = []
    for item in entries or []:
        if isinstance(item, dict):
            name = item.get("name")
        else:
            name = item
        if not name:
            continue
        out.append(str(name))
    return out


def contract_symbols(contract: Optional[Dict[str, Any]] = None) -> List[str]:
    """清单内**全部**符号（core + support），保持顺序、去重。"""
    contract = contract or load_contract()
    names = _entry_names(contract.get("core")) + _entry_names(contract.get("support"))
    seen = set()
    out = []
    for name in names:
        if name not in seen:
            seen.add(name)
            out.append(name)
    return out


def core_symbols(contract: Optional[Dict[str, Any]] = None) -> List[str]:
    """核心符号（计入 ``limits.core_count``，恰好 22）。"""
    contract = contract or load_contract()
    return _entry_names(contract.get("core"))


def support_symbols(contract: Optional[Dict[str, Any]] = None) -> List[str]:
    """支持层符号（**不计入 22**）。"""
    contract = contract or load_contract()
    return _entry_names(contract.get("support"))


def blacklist_names(contract: Optional[Dict[str, Any]] = None) -> List[str]:
    """黑名单符号（全名，不含前缀族）。"""
    contract = contract or load_contract()
    bl = contract.get("blacklist") or {}
    return _entry_names(bl.get("symbols"))


def blacklist_prefixes(contract: Optional[Dict[str, Any]] = None) -> List[str]:
    """黑名单前缀族（如 ``rm_algo_``）。"""
    contract = contract or load_contract()
    bl = contract.get("blacklist") or {}
    return _entry_names(bl.get("prefixes"))


def validate_contract(contract: Optional[Dict[str, Any]] = None) -> List[str]:
    """契约自校验：返回问题列表（空 = 契约完好）。

    校验项：
    * ``limits.core_count`` 与 ``len(core)`` 一致；
    * core / support / blacklist 无重复、无交叉；
    * core 符号全部出现在 ABI 规格的符号表里（避免清单写了但 ABI 没有）。
    """
    contract = contract or load_contract()
    problems: List[str] = []
    core = core_symbols(contract)
    support = support_symbols(contract)
    black = blacklist_names(contract)

    expected = (contract.get("limits") or {}).get("core_count")
    if expected is not None and int(expected) != len(core):
        problems.append(
            "limits.core_count=%s 与 core 实际数量 %d 不一致" % (expected, len(core))
        )
    dup_core = sorted({n for n in core if core.count(n) > 1})
    if dup_core:
        problems.append("core 存在重复符号: %s" % ", ".join(dup_core))
    dup_support = sorted({n for n in support if support.count(n) > 1})
    if dup_support:
        problems.append("support 存在重复符号: %s" % ", ".join(dup_support))
    overlap = sorted(set(core) & set(support))
    if overlap:
        problems.append("core 与 support 符号交叉: %s" % ", ".join(overlap))
    black_overlap = sorted((set(core) | set(support)) & set(black))
    if black_overlap:
        problems.append("白名单与黑名单交叉: %s" % ", ".join(black_overlap))

    try:
        from rm_hw_bridge.generated import rm_api2_ctypes as _gen
    except Exception as exc:  # pragma: no cover - 生成物缺失属环境问题
        problems.append("无法导入生成的 ctypes 绑定: %s" % exc)
    else:
        unknown = [n for n in core + support if n not in _gen.SYMBOLS]
        if unknown:
            problems.append("契约符号不在 ABI 规格符号表中: %s" % ", ".join(unknown))
    return problems


# --------------------------------------------------------------------- 句柄 id
def _handle_id(handle: Any) -> Optional[int]:
    """读句柄 id。

    RM-API2 的句柄是 ``rm_robot_handle*``（结构体只有一个 ``int id``），
    因此真实指针的 id 在 ``handle.contents.id``；兼容注入的假 SDK（直接暴露
    ``handle.id``）。返回 ``None`` 表示读不出 id（调用方应视为可疑）。

    **NULL 指针必须安全降级**：ctypes 对空指针取 ``.contents`` 会抛
    ``ValueError: NULL pointer access``（不是 AttributeError，``getattr`` 的
    默认值兜不住）——这正是"控制器客户端连接数达上限、rm_create_robot_arm 返回
    NULL"时的真实表现。这里统一吞掉并返回 ``None``，让上层给出可粘贴的错误，
    而不是把裸 ctypes 异常抛给调用方。
    """
    if handle is None:
        return None
    try:
        contents = getattr(handle, "contents", None)
    except (ValueError, AttributeError):
        return None
    if contents is not None:
        value = getattr(contents, "id", None)
        if value is not None:
            return int(value)
    value = getattr(handle, "id", None)
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


# ------------------------------------------------------------------ 库查找
def _is_repo_root(d: str) -> bool:
    return os.path.isdir(os.path.join(d, "dualarm_ws", "src", "rm_description")) and os.path.isdir(
        os.path.join(d, "workcell")
    )


def repo_root(start: Optional[str] = None) -> str:
    """向上查找仓库根（**不依赖 rmctl.paths**，硬件层零 rmctl 依赖）。"""
    here = os.path.abspath(start or _PACKAGE_DIR)
    cur = here
    while True:
        if _is_repo_root(cur):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    # 回退：包根上溯三层（dualarm_ws/src/rm_hw_bridge/rm_hw_bridge → 仓库根）
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def sdk_candidates(ws: Optional[str] = None, env: Optional[Dict[str, str]] = None) -> List[str]:
    """SDK 候选路径（按优先级）：环境变量 → ``.deps/rm_api2/libapi_c.so`` → 目录内任意 ``.so``。"""
    env = os.environ if env is None else env
    out: List[str] = []
    var = env.get(ENV_SDK_LIB)
    if var:
        out.append(var)
    root = os.path.abspath(ws or repo_root())
    out.append(os.path.join(root, DEPS_REL))
    dep_dir = os.path.join(root, ".deps", "rm_api2")
    if os.path.isdir(dep_dir):
        for name in sorted(os.listdir(dep_dir)):
            if ".so" in name:
                out.append(os.path.join(dep_dir, name))
    return out


def find_library(ws: Optional[str] = None, env: Optional[Dict[str, str]] = None) -> Optional[str]:
    """返回第一个真实存在的候选 ``.so`` 路径（找不到返回 ``None``）。"""
    for cand in sdk_candidates(ws=ws, env=env):
        if cand and os.path.isfile(cand):
            return os.path.abspath(cand)
    return None


# ------------------------------------------------------------------ 符号绑定
class MissingSymbolsError(RuntimeError):
    """库缺少清单符号（**携带缺失清单**，不抛裸异常）。"""

    def __init__(self, missing: Iterable[str]):
        self.missing_symbols = sorted(missing)
        super().__init__("RM-API2 库缺少清单符号: %s" % ", ".join(self.missing_symbols))


class SymbolBinding:
    """只暴露**清单内**符号的绑定对象。

    * 清单内符号 → 绑好 argtypes / restype 的 ctypes 函数（或注入的假符号）；
    * 清单外符号（含黑名单）→ ``AttributeError``（黑名单符号**连 getattr 都不做**）。
    """

    def __init__(self, lib: Any, contract: Optional[Dict[str, Any]] = None):
        self._lib = lib
        self._contract = contract or load_contract()
        self._fns: Dict[str, Any] = {}
        self._missing: List[str] = []
        self._bind()

    # -- 内部
    def _bind(self) -> None:
        from rm_hw_bridge.generated import rm_api2_ctypes as _gen

        for name in contract_symbols(self._contract):
            fn = getattr(self._lib, name, None)
            if fn is None:
                self._missing.append(name)
                continue
            sig = _gen.SYMBOLS.get(name)
            if sig is not None:
                try:
                    fn.argtypes = list(sig["argtypes"])
                    fn.restype = sig["restype"]
                except AttributeError:
                    # 注入的假符号不可写属性时保留原样（测试用）
                    pass
            self._fns[name] = fn

    # -- 公开
    @property
    def missing_symbols(self) -> List[str]:
        """缺失的清单符号（空 = 全部就位）。"""
        return sorted(self._missing)

    @property
    def bound_symbols(self) -> List[str]:
        """实际绑定成功的清单符号。"""
        return sorted(self._fns)

    @property
    def contract(self) -> Dict[str, Any]:
        return self._contract

    def has(self, name: str) -> bool:
        return name in self._fns

    def __getattr__(self, name: str) -> Any:
        # 只允许清单内符号；其余（含黑名单）一律 AttributeError
        if name.startswith("_"):
            raise AttributeError(name)
        fns = self.__dict__.get("_fns") or {}
        if name in fns:
            return fns[name]
        raise AttributeError(
            "符号 %r 不在契约清单内（黑名单与未授权符号一律不可取用）" % name
        )

    def __dir__(self):
        return sorted(set(list(self.__dict__.keys()) + list(self._fns)))

    def __repr__(self) -> str:
        return "<SymbolBinding bound=%d missing=%d>" % (len(self._fns), len(self._missing))


# ------------------------------------------------------------------ 句柄封装
class RobotArm:
    """机械臂句柄包装（``close()`` 调 ``rm_delete_robot_arm``）。"""

    def __init__(self, bridge: "RmBridge", handle: Any):
        self._bridge = bridge
        self._handle = handle
        self._closed = False

    @property
    def handle(self) -> Any:
        return self._handle

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> int:
        """释放句柄（幂等）。返回 ``rm_delete_robot_arm`` 的返回码。"""
        if self._closed:
            return 0
        rc = self._bridge.binding.rm_delete_robot_arm(self._handle)
        self._closed = True
        return int(rc or 0)

    def __enter__(self) -> "RobotArm":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __repr__(self) -> str:
        return "<RobotArm closed=%s>" % self._closed


class RmBridge:
    """硬件适配层入口：装载 + 绑定 + 句柄生命周期。

    用法::

        bridge = RmBridge()                 # 自动找 .so
        arm = bridge.create("192.168.1.100", 80)
        ...
        arm.close()
        bridge.close()
    """

    def __init__(
        self,
        lib: Any = None,
        contract: Optional[Dict[str, Any]] = None,
        ws: Optional[str] = None,
        env: Optional[Dict[str, str]] = None,
        strict: bool = False,
        silence_logs: bool = True,
    ):
        self._contract = contract or load_contract()
        self._strict = strict
        self._lib_path: Optional[str] = None
        self._arms: List[RobotArm] = []
        #: **必须持有**日志回调对象的引用（ctypes 回调被 GC 后，
        #: SDK 持有的是悬空函数指针 → 第一次打日志就段错误）。
        self._log_callbacks: List[Any] = []

        if lib is None:
            path = find_library(ws=ws, env=env)
            if not path:
                raise RuntimeError(
                    "找不到 RM-API2 动态库（候选: %s）；"
                    "请设置 %s 或把 SDK 物化到 <repo>/.deps/rm_api2/"
                    % (", ".join(sdk_candidates(ws=ws, env=env)), ENV_SDK_LIB)
                )
            self._lib_path = path
            lib = ctypes.CDLL(path)

        self._lib = lib
        self.binding = SymbolBinding(lib, self._contract)
        if self.binding.missing_symbols and strict:
            raise MissingSymbolsError(self.binding.missing_symbols)

        if silence_logs:
            self.silence_sdk_logs()

    # -- 属性
    @property
    def lib(self) -> Any:
        return self._lib

    @property
    def lib_path(self) -> Optional[str]:
        return self._lib_path

    @property
    def missing_symbols(self) -> List[str]:
        return self.binding.missing_symbols

    @property
    def contract(self) -> Dict[str, Any]:
        return self._contract

    # -- SDK 日志接管
    def silence_sdk_logs(self, level: int = 0) -> bool:
        """用支持层 ``rm_set_log_call_back`` 静音 SDK 的 printf 噪声。

        失败（符号缺失 / 库不支持）时**安静返回 False**，不抛异常——
        日志接管是尽力而为，不应阻断主流程。

        **回调对象的引用必须被持有**：ctypes 回调若被 GC，SDK 里就剩一个
        悬空函数指针，下次 SDK 打日志会直接段错误（真机路径实测过）。
        """
        fn = getattr(self.binding, "rm_set_log_call_back", None)
        if fn is None:
            return False
        try:
            from rm_hw_bridge.generated import rm_api2_ctypes as _gen

            cb_t = getattr(_gen, "rm_log_callback_ptr", None)
            if cb_t is None:  # pragma: no cover - 生成物异常
                return False
            cb = cb_t(lambda *args: None)
            self._log_callbacks.append(cb)   # 关键：防止被 GC
            fn(cb, level)
            return True
        except Exception:
            return False

    # -- 生命周期
    def init_sdk(self, mode: int = RM_TRIPLE_MODE_E) -> int:
        """初始化 SDK（默认三线程模式，否则收不到 UDP 主动上报）。"""
        return int(self.binding.rm_init(mode))

    def create(self, ip: str, port: int) -> RobotArm:
        """创建机械臂实例。

        失败时抛 :class:`RmBridgeError`（携带原因），**不返回半可用句柄**。
        """
        try:
            handle = self.binding.rm_create_robot_arm(ip.encode("utf-8"), int(port))
        except Exception as exc:
            raise RmBridgeError("rm_create_robot_arm 调用异常: %s" % exc) from exc
        if handle is None:
            raise RmBridgeError(
                "rm_create_robot_arm 返回 NULL（连接数达上限 5 或参数非法）: %s:%s" % (ip, port)
            )
        # RM-API2 失败时仍返回非空指针，但 id == -1
        handle_id = _handle_id(handle)
        if handle_id is not None and handle_id < 0:
            raise RmBridgeError(
                "rm_create_robot_arm 返回无效句柄 id=%s（控制器不可达？）: %s:%s"
                % (handle_id, ip, port)
            )
        arm = RobotArm(self, handle)
        self._arms.append(arm)
        return arm

    def close(self) -> None:
        """释放所有句柄（幂等）。"""
        for arm in list(self._arms):
            try:
                arm.close()
            except Exception:
                pass
        self._arms = []

    def __enter__(self) -> "RmBridge":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __repr__(self) -> str:
        return "<RmBridge lib=%s bound=%d missing=%d>" % (
            self._lib_path,
            len(self.binding.bound_symbols),
            len(self.binding.missing_symbols),
        )


class RmBridgeError(RuntimeError):
    """硬件适配层的可诊断失败（带原因，不返回伪造数据）。"""
