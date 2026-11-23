"""状态快照数据结构（R004 全字段）与 JSON schema。

快照字段与 R004 一一对应：

===========================  ==========================================
字段                          数据来源（全部白名单内）
===========================  ==========================================
``joint_deg``                ``rm_get_joint_degree``（°）
``joint_current_mA``         ``rm_get_arm_all_state``（mA）
``joint_temperature_c``      ``rm_get_arm_all_state``（℃）
``joint_voltage_v``          ``rm_get_arm_all_state``（V）
``joint_en_flag``            ``rm_get_arm_all_state``（1 使能 / 0 掉使能）
``joint_err_code``           ``rm_get_arm_all_state``（关节错误码）
``arm_err``                  ``rm_get_current_arm_state.err``（错误码列表）
``brake_state``              ``rm_get_joint_err_flag``（err_flag / brake_state）
``controller_trajectory_type``  ``rm_get_arm_current_trajectory``（规划类型 + 枚举名）
``arm_current_status``       UDP 实时上报回调（``rm_udp_arm_current_status_e``）
``realtime``                 实时上报统计（frames / last_ts / period_ms）
``controller_pose``          厂商运动学输出（**仅交叉校验**）
===========================  ==========================================

**红线**：``controller_pose`` 必须携带 ``controller_pose_scope="cross_check_only"``——
厂商运动学输出**只**用于 FK 交叉校验，**不得**作为规划真值（R003）。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

__all__ = [
    "ARM_DOF",
    "SNAPSHOT_SCHEMA",
    "StateSnapshot",
    "TRAJECTORY_ENUM_NAMES",
    "UDP_STATUS_ENUM_NAMES",
    "controller_pose_scope",
]

#: 机械臂自由度（与 ABI 常量 ARM_DOF 一致）
ARM_DOF = 7

#: 厂商运动学输出的作用域标记（**只用于交叉校验**）
controller_pose_scope = "cross_check_only"

#: ``rm_arm_current_trajectory_e`` → 枚举名（来自 ABI 规格枚举表）
TRAJECTORY_ENUM_NAMES: Dict[int, str] = {
    0: "RM_NO_PLANNING_E",
    1: "RM_JOINT_SPACE_PLANNING_E",
    2: "RM_CARTESIAN_LINEAR_PLANNING_E",
    3: "RM_CARTESIAN_ARC_PLANNING_E",
    4: "RM_SPLINE_CURVE_MOTION_PLANNING_E",
    5: "RM_TRAJECTORY_REPLAY_PLANNING_E",
    6: "RM_BLEND_TRAJECTORY_PLANNING_E",
}

#: ``rm_udp_arm_current_status_e`` → 枚举名
UDP_STATUS_ENUM_NAMES: Dict[int, str] = {
    0: "RM_IDLE_E",
    1: "RM_MOVE_L_E",
    2: "RM_MOVE_J_E",
    3: "RM_MOVE_C_E",
    4: "RM_MOVE_S_E",
    5: "RM_MOVE_THROUGH_JOINT_E",
    6: "RM_MOVE_THROUGH_POSE_E",
    7: "RM_MOVE_THROUGH_FORCE_POSE_E",
    8: "RM_MOVE_THROUGH_CURRENT_E",
    9: "RM_STOP_E",
    10: "RM_SLOW_STOP_E",
    11: "RM_PAUSE_E",
    12: "RM_CURRENT_DRAG_E",
    13: "RM_SENSOR_DRAG_E",
    14: "RM_TECH_DEMONSTRATION_E",
    15: "RM_TRAJECTORY_REPRODUCTON_E",
    16: "RM_MOVE_INIT_POSITION_E",
    17: "RM_MOVEV_CANFD_E",
}

#: 抱闸分组：J1–J4 硬抱闸，J5–J7 软抱闸（硬件事实，随快照一并输出，便于判读）
BRAKE_HARD_JOINTS = (1, 2, 3, 4)
BRAKE_SOFT_JOINTS = (5, 6, 7)


@dataclass
class StateSnapshot:
    """一次可信状态快照（R004 全字段）。

    所有数值字段都**必须**来自真实 SDK 返回；任何一步失败都不构造本对象
    （调用方改为返回 ``status="error"`` 的结果字典，**不产生任何伪造数值**）。
    """

    # -- 关节角（°）
    joint_deg: List[float] = field(default_factory=list)
    # -- 关节电流（mA）
    joint_current_mA: List[float] = field(default_factory=list)
    # -- 关节温度（℃）
    joint_temperature_c: List[float] = field(default_factory=list)
    # -- 关节电压（V）
    joint_voltage_v: List[float] = field(default_factory=list)
    # -- 关节使能位（1 使能 / 0 掉使能）
    joint_en_flag: List[int] = field(default_factory=list)
    # -- 关节错误码
    joint_err_code: List[int] = field(default_factory=list)
    # -- 臂级错误码列表（``rm_current_arm_state_t.err``）
    arm_err: List[int] = field(default_factory=list)
    # -- 抱闸状态：{"err_flag": int, "brake_state": int, "brake_open": [bool×7],
    #    "hard_brake_joints": [1..4], "soft_brake_joints": [5..7]}
    brake_state: Dict[str, Any] = field(default_factory=dict)
    # -- 控制器规划类型（int）+ 枚举名
    controller_trajectory_type: Optional[int] = None
    controller_trajectory_type_name: Optional[str] = None
    # -- 控制器状态机（UDP 实时上报）
    arm_current_status: Optional[int] = None
    arm_current_status_name: Optional[str] = None
    # -- 实时上报统计：{"frames": int, "last_ts": float|None, "period_ms": int}
    realtime: Dict[str, Any] = field(default_factory=dict)
    # -- 厂商运动学输出（**仅交叉校验**）
    controller_pose: Dict[str, Any] = field(default_factory=dict)
    controller_pose_scope: str = controller_pose_scope

    def to_dict(self) -> Dict[str, Any]:
        """转成 JSON 友好的字典（键序稳定）。"""
        return asdict(self)

    @property
    def joint_count(self) -> int:
        return len(self.joint_deg)


#: 快照 JSON schema（机器可读，供下游（S03 状态镜像契约）校验）
SNAPSHOT_SCHEMA: Dict[str, Any] = {
    "schema": "rm_hw_bridge.snapshot/1",
    "type": "object",
    "required": [
        "joint_deg",
        "joint_current_mA",
        "joint_temperature_c",
        "joint_voltage_v",
        "joint_en_flag",
        "joint_err_code",
        "arm_err",
        "brake_state",
        "controller_trajectory_type",
        "controller_trajectory_type_name",
        "arm_current_status",
        "arm_current_status_name",
        "realtime",
        "controller_pose",
        "controller_pose_scope",
    ],
    "properties": {
        "joint_deg": {
            "type": "array",
            "items": {"type": "number"},
            "minItems": ARM_DOF,
            "maxItems": ARM_DOF,
            "unit": "deg",
        },
        "joint_current_mA": {
            "type": "array", "items": {"type": "number"},
            "minItems": ARM_DOF, "maxItems": ARM_DOF, "unit": "mA",
        },
        "joint_temperature_c": {
            "type": "array", "items": {"type": "number"},
            "minItems": ARM_DOF, "maxItems": ARM_DOF, "unit": "degC",
        },
        "joint_voltage_v": {
            "type": "array", "items": {"type": "number"},
            "minItems": ARM_DOF, "maxItems": ARM_DOF, "unit": "V",
        },
        "joint_en_flag": {
            "type": "array", "items": {"type": "integer"},
            "minItems": ARM_DOF, "maxItems": ARM_DOF,
        },
        "joint_err_code": {
            "type": "array", "items": {"type": "integer"},
            "minItems": ARM_DOF, "maxItems": ARM_DOF,
        },
        "arm_err": {"type": "array", "items": {"type": "integer"}},
        "brake_state": {
            "type": "object",
            "required": ["err_flag", "brake_state", "brake_open"],
            "properties": {
                "err_flag": {"type": "integer"},
                "brake_state": {"type": "integer"},
                "brake_open": {
                    "type": "array", "items": {"type": "boolean"},
                    "minItems": ARM_DOF, "maxItems": ARM_DOF,
                },
                "hard_brake_joints": {"type": "array", "items": {"type": "integer"}},
                "soft_brake_joints": {"type": "array", "items": {"type": "integer"}},
            },
        },
        "controller_trajectory_type": {"type": ["integer", "null"]},
        "controller_trajectory_type_name": {"type": ["string", "null"]},
        "arm_current_status": {"type": ["integer", "null"]},
        "arm_current_status_name": {"type": ["string", "null"]},
        "realtime": {
            "type": "object",
            "required": ["frames", "last_ts", "period_ms"],
            "properties": {
                "frames": {"type": "integer", "minimum": 0},
                "last_ts": {"type": ["number", "null"]},
                "period_ms": {"type": ["integer", "null"]},
            },
        },
        "controller_pose": {"type": "object"},
        "controller_pose_scope": {
            "type": "string",
            "const": controller_pose_scope,
            "description": "厂商运动学输出仅用于 FK 交叉校验，不得作为规划真值（R003）",
        },
    },
}
