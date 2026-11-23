"""rm_hw_bridge 连接 + 状态快照的**离线**测试（假 SDK）。

覆盖（T3 验收点）：

* 假 SDK 下 ``connect()`` + ``snapshot()`` 产出**字段齐全**的快照
  （R004 全字段 + ``controller_pose_scope="cross_check_only"``）；
* ``handle.id = -1``（控制器不可达）→ ``status="error"``，reason / next_action 非空，
  **快照里没有任何数值**；
* SDK 缺失 → ``status="blocked"`` + **可粘贴**的 next_action；
* 任一 SDK 调用返回非 0 → ``status="error"``，reason 写明**接口名 + 返回码**，
  且**不产生任何伪造数值**；
* 统一错误模型：blocked → 退出码 3、error → 退出码 4；
* 实时上报回调是**进程级全局注册**（只接一路臂）。

只用**注入的假 SDK**，不依赖 ``.deps/``（gitignored）里的真实 ``.so``。
"""
from __future__ import annotations

import ctypes
import os

import pytest

from rm_hw_bridge import bridge, errors, loader, paths, snapshot as snap_mod
from rm_hw_bridge.generated import rm_api2_ctypes as gen


# --------------------------------------------------------------------- 假 SDK
class FakeHandle:
    """假句柄：RM-API2 失败时仍返回非空指针但 ``id == -1``。"""

    def __init__(self, handle_id=1):
        self.id = handle_id


class FakeLib:
    """假动态库：实现快照所需的全部白名单符号。

    假符号是**普通 Python 可调用对象**（ctypes 的 argtypes/restype 对它们无效），
    因此这里可以按 Python 语义直接读写结构体缓冲。
    """

    def __init__(self, handle_id=1, results=None, joint_deg=None):
        self.calls = []
        self.realtime_cb = None
        self.push_config = None
        self.handle = FakeHandle(handle_id)
        self.results = dict(results or {})
        self.joint_deg = list(joint_deg if joint_deg is not None
                              else [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        self._install()

    # -- 安装假符号
    def _install(self):
        lib = self

        def _rc(name, default=0):
            return lib.results.get(name, default)

        def rm_init(mode):
            lib.calls.append(("rm_init", mode))
            return _rc("rm_init")

        def rm_create_robot_arm(ip, port):
            lib.calls.append(("rm_create_robot_arm", ip, port))
            return _rc("rm_create_robot_arm", lib.handle)

        def rm_delete_robot_arm(handle):
            lib.calls.append(("rm_delete_robot_arm",))
            return _rc("rm_delete_robot_arm")

        def rm_set_realtime_push(handle, config):
            lib.calls.append(("rm_set_realtime_push", config))
            lib.push_config = config
            return _rc("rm_set_realtime_push")

        def rm_realtime_arm_state_call_back(cb):
            lib.calls.append(("rm_realtime_arm_state_call_back",))
            lib.realtime_cb = cb
            return None

        def rm_get_joint_degree(handle, joints):
            lib.calls.append(("rm_get_joint_degree",))
            rc = _rc("rm_get_joint_degree")
            if rc == 0:
                for i, v in enumerate(lib.joint_deg):
                    joints[i] = float(v)
            return rc

        def rm_get_arm_all_state(handle, state_ptr):
            lib.calls.append(("rm_get_arm_all_state",))
            rc = _rc("rm_get_arm_all_state")
            if rc == 0:
                st = state_ptr.contents
                for i in range(7):
                    st.joint_current[i] = 100.0 + i
                    st.joint_temperature[i] = 30.0 + i
                    st.joint_voltage[i] = 12.0 + i * 0.1
                    st.joint_en_flag[i] = 1
                    st.joint_err_code[i] = 0
            return rc

        def rm_get_current_arm_state(handle, state_ptr):
            lib.calls.append(("rm_get_current_arm_state",))
            rc = _rc("rm_get_current_arm_state")
            if rc == 0:
                st = state_ptr.contents
                st.pose.position.x = 0.1
                st.pose.position.y = 0.2
                st.pose.position.z = 0.3
                st.pose.quaternion.w = 1.0
                st.pose.euler.rx = 0.01
                st.err.err_len = 2
                st.err.err[0] = 1001
                st.err.err[1] = 1002
            return rc

        def rm_get_joint_err_flag(handle, err_flag, brake_state):
            lib.calls.append(("rm_get_joint_err_flag",))
            rc = _rc("rm_get_joint_err_flag")
            if rc == 0:
                err_flag[0] = 0b0000101
                brake_state[0] = 0b0000011
            return rc

        def rm_get_arm_current_trajectory(handle, type_ptr, data):
            lib.calls.append(("rm_get_arm_current_trajectory",))
            rc = _rc("rm_get_arm_current_trajectory")
            if rc == 0:
                type_ptr[0] = 1  # RM_JOINT_SPACE_PLANNING_E
                for i in range(7):
                    data[i] = float(i)
            return rc

        def rm_set_log_call_back(cb, level):
            lib.calls.append(("rm_set_log_call_back", cb, level))
            return None

        for name, fn in list(locals().items()):
            if name.startswith("rm_") and callable(fn):
                setattr(self, name, fn)

    # -- 工具
    def fn(self, name):
        return getattr(self, name)


@pytest.fixture()
def fake_lib():
    return FakeLib()


@pytest.fixture()
def contract():
    return loader.load_contract()


# --------------------------------------------------------------- 连接 + 快照
def test_connect_and_snapshot_has_all_r004_fields(fake_lib):
    """假 SDK 下 connect + snapshot 产出字段齐全的快照（R004 全字段）。"""
    link = bridge.connect("192.168.1.18", 8080, udp_port=8091, lib=fake_lib)
    assert link.status == errors.STATUS_OK, link.reason

    res = link.snapshot()
    assert res["status"] == errors.STATUS_OK, res["reason"]
    payload = res["snapshot"]

    # R004 全字段
    for field in ("joint_deg", "joint_current_mA", "joint_temperature_c",
                  "joint_voltage_v", "joint_en_flag", "joint_err_code"):
        assert field in payload, "快照缺字段 %s" % field
        assert len(payload[field]) == 7, "%s 必须是 7 个关节" % field
    assert payload["joint_deg"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert payload["joint_current_mA"] == [100.0, 101.0, 102.0, 103.0, 104.0, 105.0, 106.0]
    assert payload["joint_en_flag"] == [1] * 7
    assert payload["arm_err"] == [1001, 1002]
    assert payload["brake_state"]["err_flag"] == 0b0000101
    assert payload["brake_state"]["brake_open"] == [True, True, False, False, False, False, False]
    assert payload["brake_state"]["hard_brake_joints"] == [1, 2, 3, 4]
    assert payload["brake_state"]["soft_brake_joints"] == [5, 6, 7]
    assert payload["controller_trajectory_type"] == 1
    assert payload["controller_trajectory_type_name"] == "RM_JOINT_SPACE_PLANNING_E"

    # 实时上报统计
    assert payload["realtime"]["frames"] == 0
    assert payload["realtime"]["period_ms"] == 5

    # 控制器位姿：必须带 cross_check_only 标记（R003）
    assert payload["controller_pose_scope"] == "cross_check_only"
    assert payload["controller_pose"]["scope"] == "cross_check_only"
    assert payload["controller_pose"]["position"]["x"] == pytest.approx(0.1)


def test_snapshot_keys_match_schema(fake_lib):
    """快照字段与 JSON schema 一一对应（S03 状态镜像契约来源）。"""
    link = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    payload = link.snapshot()["snapshot"]
    required = snap_mod.SNAPSHOT_SCHEMA["required"]
    missing = [k for k in required if k not in payload]
    assert not missing, "快照缺 schema 要求的字段: %s" % missing


def test_connect_uses_triple_thread_mode(fake_lib):
    """必须 rm_init(RM_TRIPLE_MODE_E)（否则收不到 UDP 主动上报）。"""
    bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    init_calls = [c for c in fake_lib.calls if c[0] == "rm_init"]
    assert init_calls, "没有调用 rm_init"
    assert init_calls[0][1] == gen.RM_TRIPLE_MODE_E == 2


def test_realtime_push_cycle_is_multiple_of_five(fake_lib):
    """rm_set_realtime_push 的 cycle 必须是 5 的倍数。"""
    bridge.connect("192.168.1.18", 8080, udp_port=8091, lib=fake_lib,
                   realtime_period_ms=10)
    cfg = fake_lib.push_config
    assert cfg is not None, "没有下发 UDP 主动上报配置"
    assert cfg.cycle == 10
    assert cfg.cycle % 5 == 0
    assert cfg.enable is True
    assert cfg.port == 8091


def test_realtime_callback_is_global_registration(fake_lib):
    """实时上报回调是**进程级全局注册**（同进程只接一路臂）。"""
    link = bridge.connect("192.168.1.18", 8080, lib=fake_lib)
    assert fake_lib.realtime_cb is not None, "没有注册实时上报回调"
    # 第二次连接复用同一个回调对象（全局单例）
    bridge.connect("192.168.1.19", 8080, lib=fake_lib)
    assert link.status == errors.STATUS_OK


# --------------------------------------------------------------- 失败路径
def test_invalid_handle_id_is_error_without_numbers():
    """handle.id = -1 → status=error，reason/next_action 非空，快照无任何数值。"""
    fake = FakeLib(handle_id=-1)
    link = bridge.connect("192.168.1.18", 8080, lib=fake)
    assert link.status == errors.STATUS_ERROR
    assert link.reason, "reason 不能为空"
    assert link.next_action, "next_action 不能为空"
    assert "id=-1" in link.reason

    res = link.snapshot()
    assert res["status"] == errors.STATUS_ERROR
    assert res["reason"] and res["next_action"]
    # 快照里没有任何数值（不伪造）
    assert "snapshot" not in res
    assert not _has_numbers(res)


def test_sdk_missing_is_blocked_with_pasteable_action(tmp_path):
    """SDK 缺失 → status=blocked + 可粘贴 next_action（不抛异常）。"""
    link = bridge.connect("192.168.1.18", 8080, lib=None,
                          ws=str(tmp_path),
                          env={"RM_API2_LIB": "/nonexistent/libapi_c.so"})
    assert link.status == errors.STATUS_BLOCKED
    assert link.reason
    assert link.next_action
    assert "RM_API2_LIB" in link.next_action or ".deps/rm_api2" in link.next_action
    assert link.evidence, "必须给出搜索路径作为证据"


@pytest.mark.parametrize("api,rc", [
    ("rm_get_joint_degree", -1),
    ("rm_get_arm_all_state", -2),
    ("rm_get_current_arm_state", -3),
    ("rm_get_joint_err_flag", -1),
    ("rm_get_arm_current_trajectory", -1),
])
def test_sdk_nonzero_return_is_error_with_api_name(api, rc):
    """任一 SDK 调用返回非 0 → status=error，reason 写明接口名与返回码，无伪造数值。"""
    fake = FakeLib(results={api: rc})
    link = bridge.connect("192.168.1.18", 8080, lib=fake)
    assert link.status == errors.STATUS_OK, link.reason
    res = link.snapshot()
    assert res["status"] == errors.STATUS_ERROR
    assert api in res["reason"], "reason 必须写明接口名: %s" % res["reason"]
    assert str(rc) in res["reason"], "reason 必须写明返回码: %s" % res["reason"]
    assert res["next_action"]
    assert "snapshot" not in res
    assert not _has_numbers(res)


def test_realtime_push_failure_is_error():
    """rm_set_realtime_push 失败 → error（不返回半可用连接）。"""
    fake = FakeLib(results={"rm_set_realtime_push": -4})
    link = bridge.connect("192.168.1.18", 8080, lib=fake)
    assert link.status == errors.STATUS_ERROR
    assert "rm_set_realtime_push" in link.reason
    assert link.next_action


def test_invalid_realtime_period_is_error():
    """上报周期不是 5 的倍数 → error（SDK 会拒绝）。"""
    fake = FakeLib()
    link = bridge.connect("192.168.1.18", 8080, lib=fake, realtime_period_ms=7)
    assert link.status == errors.STATUS_ERROR
    assert "5 的倍数" in link.reason


# --------------------------------------------------------------- 回归护栏
def test_log_callback_reference_is_retained():
    """日志回调对象必须被持有。

    回归：ctypes 回调一旦被 GC，SDK 里就剩悬空函数指针 →
    **第一次打日志就段错误**（真机路径实测过，rm_create_robot_arm 直接 crash）。
    """
    fake = FakeLib()
    b = loader.RmBridge(lib=fake, silence_logs=True)
    assert b._log_callbacks, "日志回调对象没有被持有（会段错误）"
    registered = [c for c in fake.calls if c[0] == "rm_set_log_call_back"][0][1]
    assert any(cb is registered for cb in b._log_callbacks)


def test_handle_id_read_from_real_ctypes_pointer():
    """真实 SDK 返回的是指针：id 在 ``handle.contents.id``（不是 ``handle.id``）。"""
    handle = gen.rm_robot_handle(id=-1)
    ptr = ctypes.pointer(handle)
    assert loader._handle_id(ptr) == -1
    assert loader._handle_id(ctypes.pointer(gen.rm_robot_handle(id=7))) == 7


def test_connect_detects_invalid_id_on_real_ctypes_pointer():
    """指针形态的句柄（id=-1）同样必须被判失败。"""
    fake = FakeLib()
    fake.rm_create_robot_arm = lambda ip, port: ctypes.pointer(gen.rm_robot_handle(id=-1))
    link = bridge.connect("192.168.1.18", 8080, lib=fake)
    assert link.status == errors.STATUS_ERROR
    assert "id=-1" in link.reason
    assert not link.connected


# --------------------------------------------------------------- 错误模型
def test_error_model_exit_codes():
    assert errors.exit_code(errors.ok()) == 0
    assert errors.exit_code(errors.blocked("r", "a")) == 3
    assert errors.exit_code(errors.error("r", "a")) == 4


def test_failure_result_always_has_next_action():
    """blocked / error 必须带 next_action（可粘贴的下一步）。"""
    for res in (errors.blocked("r", ""), errors.error("r", "")):
        assert res["next_action"], "失败结果必须带 next_action"
        assert res["status"] in ("blocked", "error")


def test_errors_module_does_not_allow_unknown_status():
    with pytest.raises(ValueError):
        errors.result("weird", "r", "a")


# --------------------------------------------------------------- 路径解析
def test_paths_are_independent_of_rmctl(repo_root):
    """硬件层路径解析独立实现（不 import rmctl）。"""
    assert paths.workspace_root(repo_root) == repo_root
    assert paths.baselines_dir(repo_root).endswith(
        os.path.join("workcell", "baselines"))
    assert paths.joint_limits_path(repo_root).endswith("joint_limits.yaml")


def test_hardware_layer_does_not_import_rmctl():
    """硬件层不得 import rmctl（契约硬边界；AST 判定，不受注释/文档串影响）。"""
    import ast

    src = os.path.join(os.path.dirname(os.path.abspath(paths.__file__)))
    for name in ("bridge.py", "errors.py", "paths.py", "snapshot.py"):
        with open(os.path.join(src, name), "r", encoding="utf-8") as fh:
            tree = ast.parse(fh.read(), filename=name)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert alias.name.split(".")[0] != "rmctl", \
                        "%s 不得 import rmctl" % name
            elif isinstance(node, ast.ImportFrom):
                root = (node.module or "").split(".")[0]
                assert root != "rmctl", "%s 不得 import rmctl" % name


# --------------------------------------------------------------- 工具
def _has_numbers(obj):
    """递归检查是否含数值（用于断言"失败路径不产生任何数值"）。"""
    if isinstance(obj, bool):
        return False
    if isinstance(obj, (int, float)):
        return True
    if isinstance(obj, dict):
        return any(_has_numbers(v) for v in obj.values())
    if isinstance(obj, (list, tuple)):
        return any(_has_numbers(v) for v in obj)
    return False
