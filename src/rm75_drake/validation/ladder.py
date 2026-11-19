"""真机验证**阶梯**（RM75-B / 192.168.1.18）。

为什么要有「阶梯」
------------------
设计文档 §七 给出了 P0–P7 的**实施**路线，§九 给出了**验证体系**（分层手段 +
门槛），但两者都没有回答真机现场的三个问题：

1. **先做哪一步**？——现场第一件事不是「规划一条轨迹」，而是「确认链路通、
   状态回读正常、机械臂已使能、围栏参数与现场一致」。
2. **每一步的通过判据是什么**？——没有判据的现场操作会退化成「看着还行」。
3. **哪一步没过就不能做下一步**？——设计文档 §七 的核心纪律是「不达标不进
   下一阶段」，但没有落到可执行的**门禁**上。

⇒ 本模块把真机验证拆成 **15 级阶梯（L0–L14）**，从「只读、不动」到
「双臂搬运 + 长稳 + 回放回归」，每一级都有：

* **判据**（:class:`Criterion`，可机器判定）；
* **证据**（这一级要留下什么文件 / 数字）；
* **门禁**（前一级 FAIL ⇒ 后续级一律 BLOCKED）；
* **人工项**（需要人拍物理急停、量末端误差、拔网线的那部分）——
  明确标成 ``manual``，由操作者按证据文件回填，**不假装自动可测**。

设计纪律
--------
* 本模块**零 pydrake 依赖**、**零硬件依赖**（纯模型 + 判定），
  因此可以在任何环境里被单测；
* 真机动作（会动的阶段）必须显式 ``allow_motion``，否则一律 BLOCKED；
* 真机不可达时输出 ``SKIP`` 并**逐条列出未验证指标**（沿用
  :mod:`rm75_drake.device.hardware_gate` 的诚实纪律）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

__all__ = [
    "STATUS_PASS",
    "STATUS_FAIL",
    "STATUS_SKIP",
    "STATUS_BLOCKED",
    "STATUS_MANUAL",
    "TIERS",
    "Criterion",
    "Stage",
    "StageResult",
    "LadderContext",
    "LadderError",
    "build_ladder",
    "stage_by_id",
    "evaluate_criteria",
    "run_ladder",
    "ladder_report",
]


#: 阶段状态。
STATUS_PASS = "pass"
STATUS_FAIL = "fail"
STATUS_SKIP = "skip"
STATUS_BLOCKED = "blocked"
STATUS_MANUAL = "manual"

#: 阶梯分层（复杂度从低到高）。
TIERS: Tuple[str, ...] = ("A", "B", "C", "D", "E")

#: 阶梯报告 schema 版本。
LADDER_SCHEMA_VERSION = 1


class LadderError(ValueError):
    """阶梯定义 / 参数非法。"""


# ── 判据 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Criterion:
    """一条**可机器判定**的通过判据。

    参数
    ----
    key:
        指标键名（也用于人工证据回填：``{"L8": {"tcp_tracking_error_mm": 2.1}}``）。
    description:
        人话描述（写进报告与操作手册）。
    op:
        比较符：``<=`` / ``>=`` / ``==`` / ``!=`` / ``in`` / ``is_true`` /
        ``is_false``。
    threshold:
        阈值（数值或集合，语义随 ``op``）。
    unit:
        单位（仅展示用）。
    """

    key: str
    description: str
    op: str
    threshold: Any = None
    unit: str = ""

    def evaluate(self, value: Any) -> Tuple[bool, str]:
        """返回 ``(是否通过, 说明)``；**缺指标**算不通过（不猜、不放过）。"""
        if self.op == "is_true":
            ok = bool(value) is True
            return ok, f"{self.key}={value!r}（要求为真）"
        if self.op == "is_false":
            ok = bool(value) is False
            return ok, f"{self.key}={value!r}（要求为假）"
        if value is None:
            return False, f"{self.key} **缺失**（要求 {self.op} {self.threshold}）"
        if self.op == "in":
            try:
                ok = value in self.threshold
            except TypeError:
                ok = False
            return ok, f"{self.key}={value!r}（要求属于 {self.threshold}）"
        if self.op == "not_in":
            try:
                ok = value not in self.threshold
            except TypeError:
                ok = False
            return ok, f"{self.key}={value!r}（要求不属于 {self.threshold}）"
        if isinstance(self.threshold, (int, float)) and not isinstance(
            self.threshold, bool
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return False, f"{self.key}={value!r}（要求数值）"
            if self.op == "<=":
                ok = float(value) <= float(self.threshold)
            elif self.op == ">=":
                ok = float(value) >= float(self.threshold)
            elif self.op == "==":
                ok = float(value) == float(self.threshold)
            elif self.op == "!=":
                ok = float(value) != float(self.threshold)
            else:
                raise LadderError(f"未知比较符：{self.op!r}")
            unit = f" {self.unit}" if self.unit else ""
            return ok, f"{self.key}={value}{unit}（要求 {self.op} {self.threshold}{unit}）"
        ok = value == self.threshold
        return ok, f"{self.key}={value!r}（要求 {self.op} {self.threshold!r}）"


# ── 阶段 ────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Stage:
    """一级阶梯。

    字段
    ----
    id:
        阶段号（``L0``–``L14``）。
    title:
        标题。
    tier:
        复杂度层级（见 :data:`TIERS`）。
    goal:
        这一级要证明什么（一句话）。
    motion:
        是否**会让机械臂动**（True ⇒ 必须显式授权）。
    manual:
        是否**必须人工介入**（物理急停 / 量末端误差 / 拔网线 …）。
    criteria:
        通过判据。
    prerequisites:
        前置条件（人话 + 阶段号）。
    depends_on:
        **依赖的阶段 id**（门禁依据）：只要依赖里有任何一级不是 ``pass``，
        本级一律 ``blocked``。
    evidence:
        这一级要留下的证据。
    commands:
        建议命令（``manual`` 级为委托执行的命令）。
    design_refs:
        对应设计文档章节。
    milestone_refs:
        对应 M001 切片。
    failure_actions:
        不通过时的处置动作。
    executor:
        执行器键（``None`` ⇒ 委托 / 人工级）。
    """

    id: str
    title: str
    tier: str
    goal: str
    motion: bool
    manual: bool
    criteria: Tuple[Criterion, ...]
    prerequisites: Tuple[str, ...] = ()
    depends_on: Tuple[str, ...] = ()
    evidence: Tuple[str, ...] = ()
    commands: Tuple[str, ...] = ()
    design_refs: Tuple[str, ...] = ()
    milestone_refs: Tuple[str, ...] = ()
    failure_actions: Tuple[str, ...] = ()
    executor: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典（阶段定义，报告里带一份）。"""
        return {
            "id": self.id,
            "title": self.title,
            "tier": self.tier,
            "goal": self.goal,
            "motion": bool(self.motion),
            "manual": bool(self.manual),
            "executor": self.executor,
            "prerequisites": list(self.prerequisites),
            "depends_on": list(self.depends_on),
            "evidence": list(self.evidence),
            "commands": list(self.commands),
            "design_refs": list(self.design_refs),
            "milestone_refs": list(self.milestone_refs),
            "failure_actions": list(self.failure_actions),
            "criteria": [
                {
                    "key": c.key,
                    "description": c.description,
                    "op": c.op,
                    "threshold": c.threshold,
                    "unit": c.unit,
                }
                for c in self.criteria
            ],
        }


@dataclass(frozen=True)
class StageResult:
    """一级阶梯的执行结果。"""

    stage_id: str
    title: str
    tier: str
    status: str
    metrics: Dict[str, Any] = field(default_factory=dict)
    failures: Tuple[str, ...] = ()
    evidence: Tuple[str, ...] = ()
    notes: Tuple[str, ...] = ()
    duration_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """返回可 JSON 序列化的字典。"""
        return {
            "stage_id": self.stage_id,
            "title": self.title,
            "tier": self.tier,
            "status": self.status,
            "metrics": dict(self.metrics),
            "failures": list(self.failures),
            "evidence": list(self.evidence),
            "notes": list(self.notes),
            "duration_s": float(self.duration_s),
        }


@dataclass
class LadderContext:
    """阶梯执行上下文（跨阶段共享的运行时状态）。"""

    device_config: Any
    dry_run: bool = False
    allow_motion: bool = False
    options: Dict[str, Any] = field(default_factory=dict)
    state: Dict[str, Any] = field(default_factory=dict)

    def option(self, key: str, default: Any = None) -> Any:
        """读取一个运行选项。"""
        return self.options.get(key, default)


#: 执行器签名：``(stage, context) -> metrics``（异常 ⇒ 该级 FAIL）。
Executor = Callable[[Stage, LadderContext], Mapping[str, Any]]


# ── 阶梯定义 ────────────────────────────────────────────────────────────
def build_ladder() -> Tuple[Stage, ...]:
    """返回**完整的阶梯定义**（L0–L14，按执行顺序）。

    这是阶梯的**唯一真相源**（文档、脚本、单测都从这里取）。
    """
    stages: List[Stage] = [
        Stage(
            id="L0",
            title="链路与网关可达（只读，零 SDK）",
            tier="A",
            goal="确认开发机与机械臂在同一网段、TCP 指令通道与厂商网页可达。",
            motion=False,
            manual=False,
            executor="network",
            prerequisites=["机械臂已上电", "网线已连接", "急停手柄在手边"],
            evidence=["ping / TCP 连接耗时", "厂商网页 HTTP 状态码"],
            commands=[
                "ping -c 3 192.168.1.18",
                "nc -vz 192.168.1.18 8080",
                "curl -s -o /dev/null -w '%{http_code}\\n' http://192.168.1.18/",
            ],
            design_refs=["§九 通信层", "§八.3 通信与驱动"],
            milestone_refs=["S03"],
            failure_actions=[
                "确认机械臂上电与网线；确认本机在 192.168.1.0/24 网段",
                "确认没有第二个进程占用 192.168.1.18:8080（同一 IP+port 只能一个进程）",
            ],
            criteria=(
                Criterion("tcp_connected", "TCP 指令通道可连接", "is_true"),
                Criterion("http_status", "厂商网页可达", "in", {200, 301, 302, 403}),
            ),
        ),
        Stage(
            id="L1",
            depends_on=("L0",),
            title="厂商 SDK 与控制器能力（只读）",
            tier="A",
            goal="确认 SDK 可导入、版本与锁文件一致、控制器代次与自由度符合预期。",
            motion=False,
            manual=False,
            executor="sdk",
            prerequisites=["L0 通过"],
            evidence=["SDK 版本", "API 版本", "arm_dof", "控制器代次"],
            commands=["python3 -c 'import Robotic_Arm; print(Robotic_Arm.__file__)'"],
            design_refs=["§二.2 通信选型", "§八.3"],
            milestone_refs=["S03"],
            failure_actions=[
                "安装 SDK：pip install robotic-arm==1.1.7（导入名是 Robotic_Arm，大写 R/A）",
            ],
            criteria=(
                Criterion("sdk_available", "SDK 可导入", "is_true"),
                Criterion("arm_dof", "机械臂自由度", "==", 7),
                Criterion("controller_version", "控制器代次已知", "is_true"),
            ),
        ),
        Stage(
            id="L2",
            depends_on=("L1",),
            title="真机连接与只读状态快照（只读）",
            tier="A",
            goal="建立 SDK 连接，读回关节角 / 位姿 / 逐轴电流 / 错误码。",
            motion=False,
            manual=False,
            executor="connect_state",
            prerequisites=["L1 通过", "机械臂处于可查询状态（未急停）"],
            evidence=["关节角（deg 与 rad）", "TCP 位姿", "逐轴电流（mA）", "逐轴错误码"],
            commands=["python3 scripts/real_machine_ladder.py --stage L2 --json"],
            design_refs=["§五.3 状态机", "§八.3"],
            milestone_refs=["S03-T06"],
            failure_actions=[
                "连接失败：确认 IP/端口、确认只有一个进程占用通道",
                "错误码非 0：按厂商流程清错（放宽限位 → 清错 → 上使能 → 回位）",
            ],
            criteria=(
                Criterion("connected", "SDK 连接建立", "is_true"),
                Criterion("state_query_ok", "状态查询返回 0", "is_true"),
                Criterion("joint_error_count", "逐轴错误码个数", "==", 0),
            ),
        ),
        Stage(
            id="L3",
            depends_on=("L2",),
            title="UDP 主动上报接入与周期标定（只读）",
            tier="A",
            goal="打通唯一状态回读通道，并**实测**上报周期（定死 cycle 读法）。",
            motion=False,
            manual=False,
            executor="realtime",
            prerequisites=["L2 通过", "上报目标 IP 已填成本机在 192.168.1.x 上的地址"],
            evidence=["实测周期 P50/P99/P99.9/max", "cycle 读法结论", "不完整帧计数"],
            commands=["python3 scripts/real_machine_ladder.py --stage L3 --json"],
            design_refs=["§二.2", "§八.3 状态回读滞后", "§九 通信层"],
            milestone_refs=["S03-T03", "S03-T08"],
            failure_actions=[
                "收不到帧：确认三线程模式（RM_TRIPLE_MODE_E=2）、上报目标 IP 是本机 IP",
                "周期偏差 5 倍：cycle 单位是 5 ms 的倍数（实测 cycle=1 ⇒ 5 ms）",
            ],
            criteria=(
                Criterion("frames", "收到的上报帧数", ">=", 100),
                Criterion("period_match", "实测周期与期望周期一致（≤10% 偏差）", "is_true"),
                Criterion("incomplete_frames", "不完整帧（errCode=-3）个数", "==", 0),
            ),
        ),
        Stage(
            id="L4",
            depends_on=("L3",),
            title="使能标志与就绪判定（只读）",
            tier="A",
            goal="确认**全部 7 轴已使能**（TCP 连上 ≠ 可以动）。",
            motion=False,
            manual=False,
            executor="enable_check",
            prerequisites=["L3 通过（UDP 帧里有使能标志）"],
            evidence=["逐轴使能标志", "控制器状态", "机械臂状态枚举"],
            commands=["python3 scripts/real_machine_ladder.py --stage L4 --json"],
            design_refs=["§五.3 状态机 WAIT_ENABLE", "§八.3 一上电发指令就报错"],
            milestone_refs=["S03-T04"],
            failure_actions=[
                "未全轴使能：用厂商网页 / 示教器逐轴上使能，或本项目 rm_set_joint_en_state",
                "使能后仍报错：先清错再上使能（官方顺序）",
                "机械臂状态 = 9（急停）/ 10（缓停）：先在网页或示教器**释放急停**，"
                "本级不过 ⇒ 后续所有运动级一律 BLOCKED",
            ],
            criteria=(
                Criterion("en_flags_received", "收到使能标志", "is_true"),
                Criterion("all_axes_enabled", "7 轴全部使能", "is_true"),
                Criterion("joint_error_count", "逐轴错误码个数", "==", 0),
                Criterion(
                    "arm_current_status",
                    "机械臂状态不是急停(9)/缓停(10)",
                    "not_in",
                    {9, 10},
                ),
            ),
        ),
        Stage(
            id="L5",
            depends_on=("L2",),
            title="运动前静态安全自检（围栏 / 软限位，只读）",
            tier="A",
            goal="确认当前姿态落在软件软限位内、TCP 在桌面之上且在奇异圆柱之外。",
            motion=False,
            manual=False,
            executor="static_safety",
            prerequisites=["L2 通过（需要 TCP 位姿）"],
            evidence=["TCP 高度 / 水平距离", "围栏参数", "越界项列表"],
            commands=["python3 scripts/real_machine_ladder.py --stage L5 --json"],
            design_refs=["§五.2 安全七闸 G6 几何围栏", "§八.5"],
            milestone_refs=["S01", "S03"],
            failure_actions=[
                "TCP 低于桌面：抬高机械臂或修正 devices/*.yaml 的 table_height_m",
                "TCP 靠近基座轴线：移动到奇异圆柱之外，或修正 singularity_radius_m",
            ],
            criteria=(
                Criterion("joints_within_soft_limits", "关节角在软件软限位内", "is_true"),
                Criterion("fence_violations", "围栏违规项数", "==", 0),
            ),
        ),
        Stage(
            id="L6",
            depends_on=("L4", "L5",),
            title="单轴小幅点动与回零（首次运动）",
            tier="B",
            goal="第一次让机械臂动：单轴 5° 点动 → 反馈校验 → 回原位。",
            motion=True,
            manual=False,
            executor="jog",
            prerequisites=[
                "L4 通过（已使能）",
                "L5 通过（围栏无违规）",
                "工作范围内无人，物理急停手柄在手边",
            ],
            evidence=["指令关节角", "反馈关节角", "点动误差", "回零误差"],
            commands=["python3 scripts/real_machine_ladder.py --stage L6 --allow-motion --json"],
            design_refs=["§五.1 唯一指令通道", "§五.2 安全七闸"],
            milestone_refs=["S03-T05", "S03-T06"],
            failure_actions=[
                "点动误差大：检查反馈是否为 UDP 实测值（不是命令回读）",
                "点动后不回落：立即拍物理急停，并检查围栏与限位",
            ],
            criteria=(
                Criterion("jog_error_deg", "点动到位误差（度）", "<=", 1.0),
                Criterion("return_error_deg", "回零误差（度）", "<=", 1.0),
                Criterion("joint_error_count", "点动后逐轴错误码个数", "==", 0),
            ),
        ),
        Stage(
            id="L7",
            depends_on=("L6",),
            title="5 ms 透传保持位姿与周期抖动（运动）",
            tier="B",
            goal="验证 5 ms 透传节拍与抖动口径（P99.9 < 0.5 ms）。",
            motion=True,
            manual=False,
            executor="jitter",
            prerequisites=["L6 通过", "已显式给出保持位姿（--joint-rad）"],
            evidence=["抖动 P50/P99/P99.9/max", "安全违规数", "传输错误数"],
            commands=[
                "python3 scripts/measure_jitter.py --joint-rad \"0 -0.6 0 0.9 0 0.5 0\" --duration-s 60 --json",
            ],
            design_refs=["§二.1 5 ms 周期预算", "§六 主张 3", "§九 通信层"],
            milestone_refs=["S03-T08"],
            failure_actions=[
                "抖动超标：绑核 + SCHED_FIFO；确认不在透传环里做日志 I/O",
                "高频跟随报错：把周期从 200 Hz 降到 100 Hz",
            ],
            criteria=(
                Criterion("jitter_p999_ms", "抖动 P99.9（ms）", "<=", 0.5),
                Criterion("num_violations", "安全七闸违规数", "==", 0),
                Criterion("num_errors", "传输层错误数", "==", 0),
            ),
        ),
        Stage(
            id="L8",
            depends_on=("L7",),
            title="规划轨迹执行与末端跟踪（运动 + 外部测量）",
            tier="C",
            goal="让 Drake 规划的轨迹在真机跑完，并量出末端跟踪误差。",
            motion=True,
            manual=True,
            executor=None,
            prerequisites=["L7 通过", "场景 YAML 与模型已就位", "已准备外部测量手段（标定板 / 相机）"],
            evidence=["执行退出码", "运行日志", "末端跟踪误差（mm）", "逐点指令-反馈偏差"],
            commands=[
                "python3 -m rm75_drake.cli exec --device-config devices/single_arm.yaml "
                "--recording <plan.json> --log-out run_logs/$(date +%Y%m%d_%H%M%S).json",
            ],
            design_refs=["§四 规划层", "§六 主张 8", "§九 HIL 层"],
            milestone_refs=["S02", "S03-T06"],
            failure_actions=[
                "轨迹执行中途停机：看运行日志里的安全闸 reason（限位 / 围栏 / 电流 / 状态 age）",
                "跟踪误差大：先怀疑零位偏移与基座外参，再怀疑规划本身",
            ],
            criteria=(
                Criterion("exec_exit_code", "轨迹执行退出码", "==", 0),
                Criterion("tcp_tracking_error_mm", "末端跟踪误差（mm）", "<=", 3.0),
            ),
        ),
        Stage(
            id="L9",
            depends_on=("L4",),
            title="夹爪供电 / 协议 / 开合与夹持反馈（运动）",
            tier="B",
            goal="确认夹爪在线、开合指令生效、回读位置与指令一致。",
            motion=True,
            manual=False,
            executor="gripper",
            prerequisites=["L4 通过", "夹爪已独立供电并共地（末端口最大 1.5 A）"],
            evidence=["夹爪在线标志", "指令位置", "回读 actpos", "夹爪模式"],
            commands=["python3 scripts/real_machine_ladder.py --stage L9 --allow-motion --json"],
            design_refs=["§八.3 夹爪四坑", "§二 L0 夹爪"],
            milestone_refs=["S01", "S03"],
            failure_actions=[
                "指令成功但无动作：rm_set_tool_voltage(3) + 配置末端协议/波特率",
                "夹爪掉电：独立 24 V 电源并共地",
            ],
            criteria=(
                Criterion("gripper_online", "夹爪在线", "is_true"),
                Criterion("open_error", "张开位回读误差（0–1000）", "<=", 20),
                Criterion("close_error", "闭合位回读误差（0–1000）", "<=", 20),
            ),
        ),
        Stage(
            id="L10",
            depends_on=("L2",),
            title="安全七闸逐闸验证（离线注入 + 真机围栏拒绝）",
            tier="C",
            goal="证明七闸**真的会拦**：离线替身逐闸注入 + 真机越界目标被拒。",
            motion=False,
            manual=False,
            executor="safety_gates",
            prerequisites=["L5 通过"],
            evidence=["逐闸注入结果", "真机越界目标被拒的记录"],
            commands=["python3 scripts/real_machine_ladder.py --stage L10 --json"],
            design_refs=["§五.2 安全七闸", "§八 巨坑清单", "§九 故障注入"],
            milestone_refs=["S03-T02"],
            failure_actions=["任何一闸没拦住 ⇒ 该闸视为未实现，回退到 device/safety.py 修"],
            criteria=(
                Criterion("gates_checked", "被逐闸验证的闸数", ">=", 7),
                Criterion("gates_blocked", "被正确拦截的闸数", "==", 7),
                Criterion("real_fence_rejected", "真机越界目标被安全层拒绝", "is_true"),
            ),
        ),
        Stage(
            id="L11",
            depends_on=("L7",),
            title="故障注入与恢复（五类，人工编排）",
            tier="D",
            goal="五类物理故障都能进入安全态并按官方顺序恢复。",
            motion=False,
            manual=True,
            executor=None,
            prerequisites=["L7 通过", "现场安全规程已就位", "急停手柄可用"],
            evidence=["每类故障的 reason / 安全态 / 恢复耗时"],
            commands=["python3 scripts/fault_inject.py --hardware --json"],
            design_refs=["§九 故障注入", "§八.3"],
            milestone_refs=["S03-T07", "S08"],
            failure_actions=[
                "恢复卡在 resync_q：帧不完整 / 错误码未清 / 未全轴使能，逐项排查",
                "恢复卡在 check_gripper：夹爪仍离线，先处理夹爪",
            ],
            criteria=(
                Criterion("faults_injected", "完成注入的故障类别数", ">=", 5),
                Criterion("faults_recovered", "成功恢复到 READY 的故障类别数", ">=", 5),
            ),
        ),
        Stage(
            id="L12",
            depends_on=("L7",),
            title="长稳：100 Hz × 30 min → 8 h（运动）",
            tier="D",
            goal="证明长时间透传无异常停机、无安全违规累积。",
            motion=True,
            manual=True,
            executor=None,
            prerequisites=["L7 通过", "先跑 5 min 冒烟再跑满"],
            evidence=["长稳时长", "抖动分位", "违规数", "最终状态"],
            commands=[
                "python3 scripts/soak_test.py --joint-rad \"0 -0.6 0 0.9 0 0.5 0\" --hz 100 --duration-s 1800 --json",
                "python3 scripts/long_run.py --duration-s 28800 --json",
            ],
            design_refs=["§七 P2 验收门槛", "§九 长稳"],
            milestone_refs=["S03-T08", "S07"],
            failure_actions=["中途停机：查运行日志里的 stop_reason 与违规 reason"],
            criteria=(
                Criterion("soak_minutes", "长稳时长（min）", ">=", 30),
                Criterion("final_state", "执行环最终状态", "==", "completed"),
                Criterion("num_violations", "安全违规数", "==", 0),
            ),
        ),
        Stage(
            id="L13",
            depends_on=("L12",),
            title="双臂接入、标定与相对位姿搬运",
            tier="E",
            goal="第二台臂接入后，两臂共同点偏差 < 0.5 mm、相对位姿搬运成功率 ≥ 95%。",
            motion=True,
            manual=True,
            executor="dual_arm",
            prerequisites=["单臂阶梯全绿", "第二台臂可达（出厂默认同为 192.168.1.18，须先改 IP）"],
            evidence=["两臂共同点偏差", "搬运成功率", "标定产物"],
            commands=[
                "python3 scripts/calibrate_dual_arm.py --scenario scenarios/dual_arm.yaml",
                "python3 scripts/coordinated_stats.py --samples 100",
            ],
            design_refs=["§七 P4/P5", "§六 主张 1/4"],
            milestone_refs=["S04", "S05"],
            failure_actions=[
                "共同点偏差大：先修基座外参（尖对尖多姿态最小二乘），再谈规划",
                "双臂 IP 冲突：逐台改 .18 / .19 并独立网段",
            ],
            criteria=(
                Criterion("dual_arm_reachable", "第二台臂可达", "is_true"),
                Criterion("common_point_error_mm", "共同点偏差（mm）", "<=", 0.5),
                Criterion("handover_success_rate", "相对位姿搬运成功率", ">=", 0.95),
            ),
        ),
        Stage(
            id="L14",
            depends_on=("L8",),
            title="录制回放与 golden-file 回归（离线）",
            tier="E",
            goal="证明同一条录制离线重放逐点一致（可复算回归）。",
            motion=False,
            manual=True,
            executor=None,
            prerequisites=["L8 至少跑过一次并留下录制"],
            evidence=["golden 比对结果", "录制文件校验和"],
            commands=["python3 scripts/gen_golden.py --check"],
            design_refs=["§六 主张 6", "§九 可验证性"],
            milestone_refs=["S07"],
            failure_actions=["逐点不一致：先查规划版本漂移，再查时间参数化"],
            criteria=(
                Criterion("golden_exit_code", "golden 比对退出码", "==", 0),
            ),
        ),
    ]
    return tuple(stages)


def stage_by_id(stages: Sequence[Stage], stage_id: str) -> Stage:
    """按 id 取阶段；找不到抛 :class:`LadderError`。"""
    for stage in stages:
        if stage.id == stage_id:
            return stage
    raise LadderError(
        f"未知阶段 {stage_id!r}；可选：{[s.id for s in stages]}"
    )


def evaluate_criteria(
    stage: Stage, metrics: Mapping[str, Any]
) -> Tuple[List[str], List[str]]:
    """用指标判定阶段是否通过。

    返回
    ----
    ``(通过项说明, 失败项说明)``。
    """
    passed: List[str] = []
    failed: List[str] = []
    for criterion in stage.criteria:
        value = metrics.get(criterion.key)
        ok, message = criterion.evaluate(value)
        if ok:
            passed.append(message)
        else:
            failed.append(message)
    return passed, failed


# ── 运行 ────────────────────────────────────────────────────────────────
def run_ladder(
    stages: Sequence[Stage],
    executor: Optional[Executor],
    *,
    context: LadderContext,
    manual_evidence: Optional[Mapping[str, Mapping[str, Any]]] = None,
    stage_ids: Optional[Sequence[str]] = None,
) -> List[StageResult]:
    """按顺序执行阶梯，返回逐级结果。

    门禁规则
    --------
    * 依赖的阶段（:attr:`Stage.depends_on`）里只要有一级不是 ``pass``，
      本级一律 ``blocked``（设计文档 §七「不达标不进下一阶段」）；
    * ``motion=True`` 的阶段在没有 ``allow_motion`` 时一律 ``blocked``；
    * ``manual=True`` 且没有人工证据时返回 ``manual``（附上要做什么）；
    * ``executor=None`` 的阶段（委托 / 人工）用人工证据判定；
    * 执行器抛异常 ⇒ 该级 ``fail``，异常消息进 ``failures``（**不吞**）。

    参数
    ----
    stages:
        阶梯定义（:func:`build_ladder` 的结果）。
    executor:
        执行器；``None`` ⇒ 全部阶段只做门禁与人工证据判定。
    context:
        执行上下文。
    manual_evidence:
        人工证据：``{阶段 id: {判据 key: 值}}``。
    stage_ids:
        只跑这些阶段（``None`` ⇒ 全部）。
    """
    selected = list(stages)
    if stage_ids:
        wanted = {str(sid).upper() for sid in stage_ids}
        selected = [s for s in selected if s.id.upper() in wanted]
    evidence_map = dict(manual_evidence or {})

    results: List[StageResult] = []
    status_by_id: Dict[str, str] = {}
    for stage in selected:
        started = time.monotonic()
        unmet = [
            dep
            for dep in stage.depends_on
            if status_by_id.get(dep, STATUS_PASS) != STATUS_PASS
        ]
        if unmet:
            results.append(
                StageResult(
                    stage_id=stage.id,
                    title=stage.title,
                    tier=stage.tier,
                    status=STATUS_BLOCKED,
                    notes=(
                        f"依赖级别 {unmet} 未通过（状态："
                        f"{ {dep: status_by_id.get(dep) for dep in unmet} }）"
                        f" ⇒ 按门禁规则不得进入本级",
                    ),
                    duration_s=0.0,
                )
            )
            status_by_id[stage.id] = STATUS_BLOCKED
            continue
        if stage.motion and not context.allow_motion:
            results.append(
                StageResult(
                    stage_id=stage.id,
                    title=stage.title,
                    tier=stage.tier,
                    status=STATUS_BLOCKED,
                    notes=(
                        "本级会让机械臂运动；未显式授权（--allow-motion）⇒ 不执行",
                    ),
                    evidence=tuple(stage.commands),
                    duration_s=0.0,
                )
            )
            continue

        metrics: Dict[str, Any] = {}
        notes: List[str] = []
        failures: List[str] = []
        manual_values = dict(evidence_map.get(stage.id, {}))

        if stage.executor is not None and executor is not None:
            try:
                produced = executor(stage, context)
                metrics.update(dict(produced or {}))
            except Exception as exc:  # 执行器失败 ⇒ 该级 FAIL（不吞异常）
                failures.append(f"执行器异常：{type(exc).__name__}: {exc}")
        metrics.update(manual_values)

        missing = [c.key for c in stage.criteria if c.key not in metrics]
        if not failures and stage.manual and missing:
            # 人工级：指标不全 ⇒ ``manual``（**不判失败**，等证据回填）。
            status = STATUS_MANUAL
            notes.append(
                "本级需要人工介入：按 commands 完成后把实测值写进证据文件"
                f"（格式：{{\"{stage.id}\": {{\"<判据 key>\": <值>}} }}）；"
                f"还缺：{missing}"
            )
        else:
            if not failures:
                passed, failed = evaluate_criteria(stage, metrics)
                failures.extend(failed)
                notes.extend(passed)
            if failures:
                status = STATUS_FAIL
            else:
                status = STATUS_PASS
        status_by_id[stage.id] = status

        results.append(
            StageResult(
                stage_id=stage.id,
                title=stage.title,
                tier=stage.tier,
                status=status,
                metrics=metrics,
                failures=tuple(failures),
                evidence=tuple(stage.evidence),
                notes=tuple(notes),
                duration_s=time.monotonic() - started,
            )
        )
    return results


def ladder_report(
    results: Sequence[StageResult],
    *,
    context: LadderContext,
    device: Optional[Mapping[str, Any]] = None,
    generated_at: Optional[str] = None,
) -> Dict[str, Any]:
    """把逐级结果整理成**可归档**的报告字典。"""
    summary = {
        STATUS_PASS: 0,
        STATUS_FAIL: 0,
        STATUS_SKIP: 0,
        STATUS_BLOCKED: 0,
        STATUS_MANUAL: 0,
    }
    for result in results:
        summary[result.status] = summary.get(result.status, 0) + 1
    return {
        "schema_version": LADDER_SCHEMA_VERSION,
        "generated_at": generated_at or time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "mode": "dry_run" if context.dry_run else "hardware",
        "allow_motion": bool(context.allow_motion),
        "device": dict(device or {}),
        "stages": [result.to_dict() for result in results],
        "summary": summary,
    }
