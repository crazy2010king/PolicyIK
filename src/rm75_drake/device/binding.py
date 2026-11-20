"""真机 **SDK 绑定**：连接 + 调用路由 + 运行时审计（真机 bring-up 的唯一入口）。

为什么要有这一层
----------------
睿尔曼官方 SDK（``robotic-arm`` 1.1.7 / ``Robotic_Arm.rm_robot_interface``）的
真实调用形态是**实例方法**，而不是模块级函数：

.. code-block:: python

    arm = RoboticArm(RM_TRIPLE_MODE_E)      # 1) 三线程模式（UDP 主动上报的前提）
    arm.rm_create_robot_arm(ip, port)        # 2) **建立连接**（不调用它就没有句柄）
    arm.rm_movej_canfd(joint_deg, True, 0, 0, 0)   # 3) 实例方法，参数是**度**

而模块层的同名符号是 ctypes 原始函数（形如 ``rm_movej_canfd(handle, config)``），
签名与实例方法**完全不同**。⇒ 只按「模块级函数」调用，真机上必然失败。

本模块把「怎么跟真机说话」收敛成**一处**：

* :class:`ArmBinding` 负责：加载 SDK → 建实例 → **建连接** → 校验句柄 →
  把白名单符号调用**路由**到实例方法（实例没有该符号时回落到模块级函数，
  以兼容测试替身与旧版 SDK 布局）→ 记账到 :class:`~rm75_drake.device.sdk.SdkAudit`。

真机实测结论（RM75-B / 192.168.1.18 / 控制器代次 3）
---------------------------------------------------
* ``RM_TRIPLE_MODE_E = 2``（``RM_SINGLE=0 / RM_DUAL=1 / RM_TRIPLE=2``）；
  传 3 会被 SDK 打印 ``Invalid thread mode`` 并且连接拿不到句柄；
* ``rm_create_robot_arm`` 返回的句柄 ``id == -1`` 表示**连接失败**；
* 逐关节接口（使能 / 清错 / 限位）都是**逐轴**调用（``joint_num`` + 值），
  不是整臂一次调用。

这些结论都是**真机实测**得到的（证据见 ``docs/evidence/rm75-b-real-arm-ladder.md``），
不是文档抄录。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional

from .config import DeviceConfig
from .sdk import SdkAudit, SdkProxy, load_sdk

__all__ = [
    "RM_TRIPLE_MODE_E",
    "RM_SINGLE_MODE_E",
    "RM_DUAL_MODE_E",
    "SdkCallTimeout",
    "ConnectReport",
    "ArmBinding",
]


#: 睿尔曼 SDK 的线程模式枚举（``rm_thread_mode_e``）。
#:
#: ⚠️ 真机实测：传 ``3`` 会被 SDK 判为 ``Invalid thread mode``，
#: 且 ``rm_create_robot_arm`` 拿不到有效句柄（``id == -1``）。
RM_SINGLE_MODE_E = 0
RM_DUAL_MODE_E = 1
RM_TRIPLE_MODE_E = 2


class SdkCallTimeout(RuntimeError):
    """SDK 调用**超时**（厂商 ctypes 调用没有内置超时，可能永久阻塞）。

    真机实测（RM75-B / 控制器代次 3）：
    ``rm_set_arm_emergency_stop(True)`` 返回 ``-4``（控制器超时拒绝）之后，
    紧接着的 ``rm_set_realtime_push(enable=False)`` **不再返回**（阻塞 > 8 s），
    而同一连接上的 ``rm_get_current_arm_state`` 仍然正常（0.01 s）。
    ⇒ 结论：厂商 ctypes 调用**必须**由我方加超时保护，否则一次异常就能
    把 5 ms 执行环 / 收尾流程挂死。
    """


@dataclass(frozen=True)
class ConnectReport:
    """一次真机连接的结果（**如实**报告，不猜）。"""

    connected: bool
    arm_dof: Optional[int]
    controller_version: Optional[int]
    detail: str

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "connected": bool(self.connected),
            "arm_dof": self.arm_dof,
            "controller_version": self.controller_version,
            "detail": self.detail,
        }


class ArmBinding:
    """真机 SDK 绑定（连接 + 调用路由 + 审计）。

    参数
    ----
    config:
        设备配置（IP / 端口）。
    sdk:
        已加载的 SDK 命名空间；``None`` → :meth:`open` 时懒加载
        （缺失 → :class:`~rm75_drake.device.sdk.NotAvailableError`）。
    audit:
        调用审计计数器（运行时白名单审计，R008）。

    用法::

        binding = ArmBinding(device_config)
        report = binding.open()
        binding.call("rm_movej_canfd", [0.0] * 7, follow=True, trajectory_mode=0)
        binding.close()
    """

    def __init__(
        self,
        config: DeviceConfig,
        *,
        sdk: Any = None,
        audit: Optional[SdkAudit] = None,
    ) -> None:
        self.config = config
        self.audit = audit if audit is not None else SdkAudit()
        self._sdk = sdk
        self._interface: Any = None
        self._arm: Any = None
        self._proxy: Optional[SdkProxy] = None
        self._report: Optional[ConnectReport] = None

    # -- 属性 ------------------------------------------------------------
    @property
    def arm(self) -> Any:
        """已连接（或待连接）的 ``RoboticArm`` 实例；未 :meth:`open` 时是 ``None``。"""
        return self._arm

    @property
    def interface(self) -> Any:
        """``Robotic_Arm.rm_robot_interface`` 模块（类型定义、回调注册都在这里）。"""
        if self._interface is None:
            raise RuntimeError("ArmBinding 尚未 open()")
        return self._interface

    @property
    def report(self) -> Optional[ConnectReport]:
        """最近一次 :meth:`open` 的连接报告。"""
        return self._report

    @property
    def is_open(self) -> bool:
        """是否已成功连接。"""
        return self._report is not None and self._report.connected

    # -- 生命周期 --------------------------------------------------------
    def open(self) -> ConnectReport:
        """建立连接（幂等）。

        步骤
        ----
        1. 懒加载 SDK（缺失 → :class:`~rm75_drake.device.sdk.NotAvailableError`）；
        2. ``RoboticArm(RM_TRIPLE_MODE_E)``（UDP 主动上报的前提）；
        3. ``rm_create_robot_arm(ip, port)`` **建立连接**；
        4. 校验句柄（``id == -1`` ⇒ 连接失败）。

        返回
        ----
        :class:`ConnectReport`（成功时 ``connected=True``）。

        异常
        ----
        :class:`~rm75_drake.device.sdk.NotAvailableError`
            SDK 未安装。
        RuntimeError
            SDK 结构不符合预期（找不到 ``RoboticArm`` / 连接失败）。
        """
        if self._report is not None and self._report.connected:
            return self._report
        if self._sdk is None:
            # ⚠️ 官方 SDK 的顶层包 ``Robotic_Arm/__init__.py`` **是空的**：
            #    子模块 ``rm_robot_interface`` 必须**显式导入**才有属性
            #    （否则 ``Robotic_Arm.rm_robot_interface`` 直接 AttributeError）。
            from .sdk import load_rm_interface

            self._interface = load_rm_interface()
        else:
            # 显式注入的 SDK（测试替身）：优先用它的 rm_robot_interface 子模块。
            self._interface = getattr(self._sdk, "rm_robot_interface", self._sdk)
        self._proxy = SdkProxy(self._interface, self.audit)

        arm_cls = getattr(self._interface, "RoboticArm", None)
        if arm_cls is None:
            raise RuntimeError(
                "SDK 里找不到 RoboticArm（Robotic_Arm.rm_robot_interface.RoboticArm）"
            )
        self._arm = arm_cls(RM_TRIPLE_MODE_E)

        connect = getattr(self._arm, "rm_create_robot_arm", None)
        if connect is None:
            # 测试替身 / 旧版 SDK：没有建连接的方法 ⇒ 明确记录「未连接」，
            # **不假装已连上**（调用方据此决定是否允许真机动作）。
            self._report = ConnectReport(
                connected=False,
                arm_dof=getattr(self._arm, "arm_dof", None),
                controller_version=getattr(
                    self._arm, "robot_controller_version", None
                ),
                detail=(
                    "SDK 实例未暴露 rm_create_robot_arm（测试替身或旧版 SDK 布局）；"
                    "未建立真机连接"
                ),
            )
            return self._report

        handle = connect(str(self.config.arm_ip), int(self.config.tcp_port))
        handle_id = getattr(handle, "id", None)
        if handle_id is not None and int(handle_id) < 0:
            self._report = ConnectReport(
                connected=False,
                arm_dof=getattr(self._arm, "arm_dof", None),
                controller_version=getattr(
                    self._arm, "robot_controller_version", None
                ),
                detail=(
                    f"rm_create_robot_arm({self.config.arm_ip}:{self.config.tcp_port}) "
                    f"返回无效句柄 id={handle_id}（连接失败）"
                ),
            )
            return self._report

        self._report = ConnectReport(
            connected=True,
            arm_dof=getattr(self._arm, "arm_dof", None),
            controller_version=getattr(self._arm, "robot_controller_version", None),
            detail=(
                f"已连接 {self.config.arm_ip}:{self.config.tcp_port}"
                f"（handle id={handle_id}）"
            ),
        )
        return self._report

    def close(self) -> None:
        """关闭连接（幂等；失败不抛异常——收尾阶段不应掩盖主错误）。

        ⚠️ 真机实测（RM75-B / 控制器代次 3）：**不调用** ``rm_delete_robot_arm``。
        删除句柄时 SDK 的 UDP 接收线程仍在跑 ⇒ 进程退出阶段会**段错误**
        （``Fatal Python error: Segmentation fault``）。
        ⇒ 收尾只停主动上报，句柄交给进程退出释放（socket 由内核回收）。
        需要显式删句柄时请调用 :meth:`delete_handle`（并自行承担风险）。
        """
        arm = self._arm
        self._arm = None
        self._report = None
        del arm

    def delete_handle(self) -> bool:
        """显式删除 SDK 句柄（**默认不做**：真机实测会段错误）。

        返回
        ----
        ``True``
            删除成功；``False``
            句柄不存在 / 删除失败（**不抛异常**）。
        """
        arm = self._arm
        if arm is None:
            return False
        destroy = getattr(arm, "rm_delete_robot_arm", None)
        if destroy is None:
            return False
        try:
            destroy()
        except Exception:  # pragma: no cover - 收尾尽力而为
            return False
        self._arm = None
        self._report = None
        return True

    # -- 调用 ------------------------------------------------------------
    def call(
        self,
        name: str,
        /,
        *args: Any,
        timeout_s: Optional[float] = None,
        **kwargs: Any,
    ) -> Any:
        """调用一个 SDK 符号（**实例方法优先**，模块级函数兜底）。

        路由规则
        --------
        1. 实例上有同名可调用属性 → 调**实例方法**（真 SDK 的真实形态）；
        2. 否则 → 调模块级同名函数（测试替身 / 旧布局）；
        3. 都没有 → :class:`AttributeError`（点名符号，不静默返回空函数）。

        超时保护
        --------
        ``timeout_s`` 给定时，调用在**守护线程**里执行并等待至多该时长；
        超时则抛 :class:`SdkCallTimeout`（底层线程仍在跑，但**不阻塞我方流程**）。
        厂商 ctypes 调用没有内置超时 ⇒ 非实时路径上的调用**必须**给超时。

        每次调用都会记进 :attr:`audit`（白名单 / 禁用名单审计）。
        """
        if self._proxy is None:
            raise RuntimeError("ArmBinding 尚未 open()")
        arm = self._arm
        target = getattr(arm, name, None) if arm is not None else None
        if callable(target):
            fn = target
        else:
            module_fn = getattr(self._proxy.target, name, None)
            if not callable(module_fn):
                raise AttributeError(
                    f"SDK 里既没有实例方法也没有模块级函数：{name}"
                    f"（真机 SDK 的调用形态是实例方法）"
                )
            fn = module_fn
        if timeout_s is None:
            result = fn(*args, **kwargs)
        else:
            result = _call_with_timeout(fn, args, kwargs, float(timeout_s), name)
        self.audit.record(name, _args_repr(args, kwargs))
        return result

    def has(self, name: str) -> bool:
        """返回该符号在实例或模块上是否可用。"""
        if self._arm is not None and callable(getattr(self._arm, name, None)):
            return True
        if self._proxy is None:
            return False
        return callable(getattr(self._proxy.target, name, None))


def _call_with_timeout(
    fn: Any,
    args: Any,
    kwargs: Any,
    timeout_s: float,
    name: str,
) -> Any:
    """在守护线程里执行 SDK 调用，最多等 ``timeout_s`` 秒。"""
    box: Dict[str, Any] = {}
    done = threading.Event()

    def _run() -> None:
        try:
            box["result"] = fn(*args, **kwargs)
        except Exception as exc:  # pragma: no cover - 由调用方看到
            box["error"] = exc
        finally:
            done.set()

    thread = threading.Thread(target=_run, daemon=True, name=f"rm75-sdk-{name}")
    thread.start()
    if not done.wait(max(0.001, float(timeout_s))):
        raise SdkCallTimeout(
            f"SDK 调用超时（{timeout_s:.2f} s）：{name}"
            f"（厂商 ctypes 调用无内置超时；真机实测：一次被控制器拒绝的调用"
            f"（返回 -4）会让后续同连接请求不再返回）"
        )
    if "error" in box:
        raise box["error"]
    return box.get("result")


def _args_repr(args: Any, kwargs: Any) -> str:
    """把调用参数压成一行摘要（审计记录用）。"""
    parts = [repr(a) for a in args]
    parts.extend(f"{k}={v!r}" for k, v in kwargs.items())
    text = ", ".join(parts)
    return text if len(text) <= 200 else text[:197] + "..."
