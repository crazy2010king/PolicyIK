"""RM-API2 ctypes 绑定（**自动生成，请勿手改**）。

源头: rm_hw_bridge/abi_spec.yaml
abi_spec.yaml sha256: 3c93a05cf713a69aaf5ff24bcbcd32387767d36e5d1d67377b95a61d30a14b2d
SDK: RM-API2 1.1.7

重新生成: python3 -m rm_hw_bridge.codegen --spec <abi_spec.yaml> --out <dir>
幂等校验: python3 -m rm_hw_bridge.codegen --spec <abi_spec.yaml> --out <dir> --check

ABI 关键规则:
* rm_create_robot_arm 失败时仍返回非空指针但 handle.id == -1；必须读 id 判失败。
* rm_set_realtime_push / rm_movej_canfd 的结构体按值传递（非指针）。
* rm_movej_canfd_mode_t.joint 是 float*；调用方必须让缓冲存活到调用结束。
* rm_realtime_arm_state_call_back 是进程级全局注册；同进程无法同时收两路臂。
* rm_realtime_push_config_t.ip 是 char[28]（ctypes.c_char * 28），赋值必须用 bytes。
* 单位：关节角 °；rm_set_joint_drive_max_speed °/s；_max_acc °/s²；_min_pos/_max_pos °。
"""
from __future__ import annotations

import ctypes

SPEC_SHA256 = "3c93a05cf713a69aaf5ff24bcbcd32387767d36e5d1d67377b95a61d30a14b2d"
SDK_NAME = "RM-API2"
SDK_VERSION = "1.1.7"

# ---------------------------------------------------------------- 常量
# 机械臂自由度（关节数）
ARM_DOF = 7

# ---------------------------------------------------------------- 枚举
# 当前机械臂规划类型
RM_BLEND_TRAJECTORY_PLANNING_E = 6
RM_CARTESIAN_ARC_PLANNING_E = 3
RM_CARTESIAN_LINEAR_PLANNING_E = 2
RM_JOINT_SPACE_PLANNING_E = 1
RM_NO_PLANNING_E = 0
RM_SPLINE_CURVE_MOTION_PLANNING_E = 4
RM_TRAJECTORY_REPLAY_PLANNING_E = 5

# 末端力传感器版本
RM_MODEL_RM_BV_E = 4
RM_MODEL_RM_B_E = 0
RM_MODEL_RM_ISFV_E = 5
RM_MODEL_RM_ISF_E = 3
RM_MODEL_RM_SF_E = 2
RM_MODEL_RM_ZF_E = 1

# 机械臂型号
RM_MODEL_ECO_62_E = 6
RM_MODEL_ECO_63_E = 8
RM_MODEL_ECO_65_E = 5
RM_MODEL_GEN_72_E = 7
RM_MODEL_RM_63_III_E = 4
RM_MODEL_RM_63_II_E = 3
RM_MODEL_RM_63_I_E = 2
RM_MODEL_RM_65_E = 0
RM_MODEL_RM_75_E = 1
RM_MODEL_RXL75II_E = 16
RM_MODEL_RXL75T_E = 18
RM_MODEL_RXL75_E = 12
RM_MODEL_RXR75II_E = 17
RM_MODEL_RXR75T_E = 19
RM_MODEL_RXR75_E = 13
RM_MODEL_UNIVERSAL_E = 9
RM_MODEL_ZM7L_E = 10
RM_MODEL_ZM7R_E = 11
RM_MODEL_ZPFL74_E = 14
RM_MODEL_ZPFR74_E = 15

# SDK 线程模式
RM_DUAL_MODE_E = 1
RM_SINGLE_MODE_E = 0
RM_TRIPLE_MODE_E = 2

# UDP 主动上报中的机械臂状态
RM_CURRENT_DRAG_E = 12
RM_IDLE_E = 0
RM_MOVEV_CANFD_E = 17
RM_MOVE_C_E = 3
RM_MOVE_INIT_POSITION_E = 16
RM_MOVE_J_E = 2
RM_MOVE_L_E = 1
RM_MOVE_S_E = 4
RM_MOVE_THROUGH_CURRENT_E = 8
RM_MOVE_THROUGH_FORCE_POSE_E = 7
RM_MOVE_THROUGH_JOINT_E = 5
RM_MOVE_THROUGH_POSE_E = 6
RM_PAUSE_E = 11
RM_SENSOR_DRAG_E = 13
RM_SLOW_STOP_E = 10
RM_STOP_E = 9
RM_TECH_DEMONSTRATION_E = 14
RM_TRAJECTORY_REPRODUCTON_E = 15

# 枚举在 C ABI 中是 int 尺寸（不改变内存布局）

# ---------------------------------------------------------------- 结构体

class rm_robot_handle(ctypes.Structure):
    """机械臂控制句柄"""

    _fields_ = [
        ("id", ctypes.c_int),  # 句柄 id，连接成功 >0，连接失败 -1
    ]

class rm_position_t(ctypes.Structure):
    """位置坐标（单位 m）"""

    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]

class rm_quat_t(ctypes.Structure):
    """四元数"""

    _fields_ = [
        ("w", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]

class rm_euler_t(ctypes.Structure):
    """欧拉角（单位 rad）"""

    _fields_ = [
        ("rx", ctypes.c_float),
        ("ry", ctypes.c_float),
        ("rz", ctypes.c_float),
    ]

class rm_pose_t(ctypes.Structure):
    """位置姿态"""

    _fields_ = [
        ("position", rm_position_t),  # 位置，单位 m
        ("quaternion", rm_quat_t),
        ("euler", rm_euler_t),  # 欧拉角，单位 rad
    ]

class rm_err_t(ctypes.Structure):
    """错误代码结构体"""

    _fields_ = [
        ("err_len", ctypes.c_uint8),  # 错误代码个数
        ("err", (ctypes.c_int * 24)),  # 错误代码数组
    ]

class rm_current_arm_state_t(ctypes.Structure):
    """机械臂当前状态"""

    _fields_ = [
        ("pose", rm_pose_t),  # 机械臂当前位姿
        ("joint", (ctypes.c_float * 7)),  # 机械臂当前关节角度，单位 °
        ("err", rm_err_t),
    ]

class rm_joint_status_t(ctypes.Structure):
    """机械臂关节状态参数"""

    _fields_ = [
        ("joint_current", (ctypes.c_float * 7)),  # 关节电流，单位 mA
        ("joint_en_flag", (ctypes.c_bool * 7)),  # 关节使能状态
        ("joint_err_code", (ctypes.c_uint16 * 7)),  # 关节错误码
        ("joint_position", (ctypes.c_float * 7)),  # 关节角度，单位 °
        ("joint_temperature", (ctypes.c_float * 7)),  # 关节温度，单位 ℃
        ("joint_voltage", (ctypes.c_float * 7)),  # 关节电压，单位 V
        ("joint_speed", (ctypes.c_float * 7)),  # 关节速度，单位 °/s
    ]

class rm_arm_all_state_t(ctypes.Structure):
    """机械臂所有状态参数"""

    _fields_ = [
        ("joint_current", (ctypes.c_float * 7)),  # 关节电流，单位 mA
        ("joint_en_flag", (ctypes.c_int * 7)),  # 关节使能状态
        ("joint_temperature", (ctypes.c_float * 7)),  # 关节温度，单位 ℃
        ("joint_voltage", (ctypes.c_float * 7)),  # 关节电压，单位 V
        ("joint_err_code", (ctypes.c_int * 7)),  # 关节错误码
        ("err", rm_err_t),
    ]

class rm_udp_custom_config_t(ctypes.Structure):
    """UDP 主动上报自定义项（每个字段：1 上报 / 0 关闭 / -1 保持原状）"""

    _fields_ = [
        ("joint_speed", ctypes.c_int),
        ("lift_state", ctypes.c_int),
        ("expand_state", ctypes.c_int),
        ("hand_state", ctypes.c_int),
        ("arm_current_status", ctypes.c_int),
        ("aloha_state", ctypes.c_int),
        ("plus_base", ctypes.c_int),
        ("plus_state", ctypes.c_int),
    ]

class rm_realtime_push_config_t(ctypes.Structure):
    """机械臂主动上报接口配置"""

    _fields_ = [
        ("cycle", ctypes.c_int),  # 广播周期，5ms 的倍数
        ("enable", ctypes.c_bool),  # 是否主动上报
        ("port", ctypes.c_int),  # 广播端口号
        ("force_coordinate", ctypes.c_int),  # 系统外受力数据坐标系
        ("ip", (ctypes.c_char * 28)),  # 上报目标 IP（bytes 赋值）
        ("custom_config", rm_udp_custom_config_t),
    ]

class rm_movej_canfd_mode_t(ctypes.Structure):
    """角度透传模式结构体（按值传入 rm_movej_canfd）"""

    _fields_ = [
        ("joint", ctypes.POINTER(ctypes.c_float)),  # 关节角度指针，单位 °（缓冲须在调用期间存活）
        ("expand", ctypes.c_float),  # 扩展关节角度
        ("follow", ctypes.c_bool),  # true 高跟随 / false 低跟随
        ("trajectory_mode", ctypes.c_int),  # 0 完全透传 / 1 曲线拟合 / 2 滤波
        ("radio", ctypes.c_int),  # 平滑系数 0-1000
    ]

class rm_force_sensor_t(ctypes.Structure):
    """UDP 主动上报力传感器数据"""

    _fields_ = [
        ("force", (ctypes.c_float * 6)),  # 原始力，0.001N / 0.001Nm
        ("zero_force", (ctypes.c_float * 6)),  # 系统外受力，0.001N / 0.001Nm
        ("coordinate", ctypes.c_int),  # 受力坐标系
    ]

class rm_udp_lift_state_t(ctypes.Structure):
    """升降机构状态"""

    _fields_ = [
        ("height", ctypes.c_int),  # 高度，单位 mm
        ("pos", ctypes.c_float),  # 角度，单位 °
        ("current", ctypes.c_int),  # 驱动电流，单位 mA
        ("err_flag", ctypes.c_int),
        ("en_flag", ctypes.c_int),
    ]

class rm_udp_expand_state_t(ctypes.Structure):
    """扩展关节状态"""

    _fields_ = [
        ("pos", ctypes.c_float),  # 角度，单位 °
        ("current", ctypes.c_int),  # 驱动电流，单位 mA
        ("err_flag", ctypes.c_int),
        ("en_flag", ctypes.c_int),
        ("joint_id", ctypes.c_int),
        ("mode", ctypes.c_int),
    ]

class rm_udp_hand_state_t(ctypes.Structure):
    """灵巧手状态"""

    _fields_ = [
        ("hand_pos", (ctypes.c_int * 6)),
        ("hand_angle", (ctypes.c_int * 6)),
        ("hand_force", (ctypes.c_int * 6)),
        ("hand_state", (ctypes.c_int * 6)),
        ("hand_err", ctypes.c_int),
    ]

class rm_udp_aloha_state_t(ctypes.Structure):
    """aloha 主臂状态"""

    _fields_ = [
        ("io1_state", ctypes.c_int),
        ("io2_state", ctypes.c_int),
    ]

class rm_plus_base_info_t(ctypes.Structure):
    """末端设备基础信息"""

    _fields_ = [
        ("manu", (ctypes.c_char * 10)),
        ("type", ctypes.c_int),
        ("hv", (ctypes.c_char * 10)),
        ("sv", (ctypes.c_char * 10)),
        ("bv", (ctypes.c_char * 10)),
        ("id", ctypes.c_int),
        ("dof", ctypes.c_int),
        ("check", ctypes.c_int),
        ("bee", ctypes.c_int),
        ("force", ctypes.c_bool),
        ("touch", ctypes.c_bool),
        ("touch_num", ctypes.c_int),
        ("touch_sw", ctypes.c_int),
        ("hand", ctypes.c_int),
        ("pos_up", (ctypes.c_int * 6)),
        ("pos_low", (ctypes.c_int * 6)),
        ("angle_up", (ctypes.c_int * 6)),
        ("angle_low", (ctypes.c_int * 6)),
        ("speed_up", (ctypes.c_int * 6)),
        ("speed_low", (ctypes.c_int * 6)),
        ("force_up", (ctypes.c_int * 6)),
        ("force_low", (ctypes.c_int * 6)),
    ]

class rm_plus_state_info_t(ctypes.Structure):
    """末端设备实时信息"""

    _fields_ = [
        ("sys_state", ctypes.c_int),
        ("dof_state", (ctypes.c_int * 6)),
        ("dof_err", (ctypes.c_int * 6)),
        ("pos", (ctypes.c_int * 6)),
        ("speed", (ctypes.c_int * 6)),
        ("angle", (ctypes.c_int * 6)),
        ("current", (ctypes.c_int * 6)),
        ("normal_force", (ctypes.c_int * 18)),
        ("tangential_force", (ctypes.c_int * 18)),
        ("tangential_force_dir", (ctypes.c_int * 18)),
        ("tsa", (ctypes.c_uint32 * 12)),
        ("tma", (ctypes.c_uint32 * 12)),
        ("touch_data", (ctypes.c_int * 18)),
        ("force", (ctypes.c_int * 6)),
    ]

class rm_realtime_arm_joint_state_t(ctypes.Structure):
    """UDP 主动上报机械臂信息（回调参数，**按值** 传递）"""

    _fields_ = [
        ("errCode", ctypes.c_int),  # 数据解析错误码
        ("arm_ip", (ctypes.c_char * 16)),
        ("arm_port", ctypes.c_int),
        ("joint_status", rm_joint_status_t),
        ("force_sensor", rm_force_sensor_t),
        ("err", rm_err_t),
        ("waypoint", rm_pose_t),
        ("liftState", rm_udp_lift_state_t),
        ("expandState", rm_udp_expand_state_t),
        ("handState", rm_udp_hand_state_t),
        ("arm_current_status", ctypes.c_int),
        ("aloha_state", rm_udp_aloha_state_t),
        ("rm_plus_state", ctypes.c_int),
        ("plus_base_info", rm_plus_base_info_t),
        ("plus_state_info", rm_plus_state_info_t),
    ]

class rm_robot_info_t(ctypes.Structure):
    """机械臂基本信息"""

    _fields_ = [
        ("arm_dof", ctypes.c_uint8),
        ("arm_model", ctypes.c_int),
        ("force_type", ctypes.c_int),
        ("robot_controller_version", ctypes.c_uint8),
    ]

# ---------------------------------------------------------------- 回调类型
# 日志回调（va_list 在 x86-64 SysV 上以寄存器保存，ctypes 侧按 void* 近似传递）
rm_log_callback_ptr = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_void_p)
# UDP 主动上报回调（**进程级全局注册**，同进程只能收一路臂）
rm_realtime_arm_state_callback_ptr = ctypes.CFUNCTYPE(None, rm_realtime_arm_joint_state_t)

# ---------------------------------------------------------------- 符号表
#: 符号名 → {"argtypes": [...], "restype": ...}（可直接喂给 ctypes）
SYMBOLS = {
    "rm_create_robot_arm": {
        "argtypes": [ctypes.c_char_p, ctypes.c_int],
        "restype": ctypes.POINTER(rm_robot_handle),
    },
    "rm_delete_robot_arm": {
        "argtypes": [ctypes.POINTER(rm_robot_handle)],
        "restype": ctypes.c_int,
    },
    "rm_get_arm_all_state": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(rm_arm_all_state_t)],
        "restype": ctypes.c_int,
    },
    "rm_get_arm_current_trajectory": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_float)],
        "restype": ctypes.c_int,
    },
    "rm_get_current_arm_state": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(rm_current_arm_state_t)],
        "restype": ctypes.c_int,
    },
    "rm_get_joint_degree": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(ctypes.c_float)],
        "restype": ctypes.c_int,
    },
    "rm_get_joint_err_flag": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint16)],
        "restype": ctypes.c_int,
    },
    "rm_get_realtime_push": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(rm_realtime_push_config_t)],
        "restype": ctypes.c_int,
    },
    "rm_get_robot_info": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(rm_robot_info_t)],
        "restype": ctypes.c_int,
    },
    "rm_init": {
        "argtypes": [ctypes.c_int],
        "restype": ctypes.c_int,
    },
    "rm_movej": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int],
        "restype": ctypes.c_int,
    },
    "rm_movej_canfd": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), rm_movej_canfd_mode_t],
        "restype": ctypes.c_int,
        "by_value": ['config'],
    },
    "rm_realtime_arm_state_call_back": {
        "argtypes": [rm_realtime_arm_state_callback_ptr],
        "restype": None,
    },
    "rm_set_arm_continue": {
        "argtypes": [ctypes.POINTER(rm_robot_handle)],
        "restype": ctypes.c_int,
    },
    "rm_set_arm_pause": {
        "argtypes": [ctypes.POINTER(rm_robot_handle)],
        "restype": ctypes.c_int,
    },
    "rm_set_arm_slow_stop": {
        "argtypes": [ctypes.POINTER(rm_robot_handle)],
        "restype": ctypes.c_int,
    },
    "rm_set_arm_stop": {
        "argtypes": [ctypes.POINTER(rm_robot_handle)],
        "restype": ctypes.c_int,
    },
    "rm_set_collision_state": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.c_int],
        "restype": ctypes.c_int,
    },
    "rm_set_delete_current_trajectory": {
        "argtypes": [ctypes.POINTER(rm_robot_handle)],
        "restype": ctypes.c_int,
    },
    "rm_set_joint_drive_max_acc": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.c_int, ctypes.c_float],
        "restype": ctypes.c_int,
    },
    "rm_set_joint_drive_max_pos": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.c_int, ctypes.c_float],
        "restype": ctypes.c_int,
    },
    "rm_set_joint_drive_max_speed": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.c_int, ctypes.c_float],
        "restype": ctypes.c_int,
    },
    "rm_set_joint_drive_min_pos": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.c_int, ctypes.c_float],
        "restype": ctypes.c_int,
    },
    "rm_set_log_call_back": {
        "argtypes": [rm_log_callback_ptr, ctypes.c_int],
        "restype": None,
    },
    "rm_set_realtime_push": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), rm_realtime_push_config_t],
        "restype": ctypes.c_int,
        "by_value": ['config'],
    },
    "rm_set_self_collision_enable": {
        "argtypes": [ctypes.POINTER(rm_robot_handle), ctypes.c_bool],
        "restype": ctypes.c_int,
    },
    "rm_set_timeout": {
        "argtypes": [ctypes.c_int],
        "restype": None,
    },
}

#: 全部符号名（白名单基线；装载层另加黑名单静态审计）
SYMBOL_NAMES = tuple(sorted(SYMBOLS))


def bind_library(lib, strict: bool = True):
    """把 argtypes / restype 绑定到已加载的 CDLL 上。

    strict=True 时，库中缺少任何规格符号都会抛 RuntimeError（避免静默半可用）。
    """
    missing = []
    for name, sig in SYMBOLS.items():
        fn = getattr(lib, name, None)
        if fn is None:
            missing.append(name)
            continue
        fn.argtypes = list(sig["argtypes"])
        fn.restype = sig["restype"]
    if missing and strict:
        raise RuntimeError(
            "RM-API2 库缺少规格符号: %s" % ", ".join(sorted(missing)))
    if missing:
        return lib, missing
    return lib
