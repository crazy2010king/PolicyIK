"""限值默认值来源（measured_limits）+ 限值/安全开关下发（T4 验收点）。

覆盖：

* **负例**：基线 ``status=blocked`` → 下发被拒，**0 次 SDK 下发调用**，
  结果带 reason 与可粘贴 next_action；
* 占位回退必须**显式** ``allow_placeholder=True`` 才发生，
  且结果带 ``placeholder, 未实测`` 标注；
* **正例**：pytest 临时目录构造 ``status=measured`` 的 ``measured_limits.yaml``
  （含 source/method/samples）→ 下发 7 关节 × 4 项，参数经单位换算后与期望一致
  （°/s、°/s²、°）；
* ``dry_run=True`` → 0 次 SDK 调用但输出完整 diff；
* 非法 ``collision_stage``（9 / -1 / 非整数）→ 拒绝且 0 次调用；
* SDK 返回非 0 → ``status="error"`` + reason 写明**接口名 + 返回码**；
* 硬件层**零 rmctl 依赖**、单位换算**单点收敛**（只在 bridge.py）。

只用**注入的假 SDK** + pytest 临时目录，不依赖 ``.deps/``（gitignored）里的真 SDK。
"""
from __future__ import annotations

import math
import os
import re

import pytest
import yaml

from rm_hw_bridge import bridge, errors, limits as limits_mod

# --------------------------------------------------------------- 假 SDK
DOWNLINK_SYMBOLS = (
    "rm_set_joint_drive_max_speed",
    "rm_set_joint_drive_max_acc",
    "rm_set_joint_drive_min_pos",
    "rm_set_joint_drive_max_pos",
    "rm_set_collision_state",
    "rm_set_self_collision_enable",
)


class FakeHandle:
    """假句柄（``id`` 直接暴露在属性上）。"""

    def __init__(self, handle_id: int = 1):
        self.id = handle_id


class FakeLib:
    """假动态库：实现连接 + 限值下发所需的全部白名单符号。"""

    def __init__(self, handle_id: int = 1, results=None):
        self.calls = []
        self.results = dict(results or {})
        self.handle = FakeHandle(handle_id)

    # -- 计数
    def downlink_calls(self):
        """下发类调用（限值族 + 安全开关）。"""
        return [c for c in self.calls if c[0] in DOWNLINK_SYMBOLS]

    def _rc(self, name, default=0):
        return self.results.get(name, default)

    # -- 符号
    def rm_init(self, mode):
        self.calls.append(("rm_init", mode))
        return self._rc("rm_init")

    def rm_create_robot_arm(self, ip, port):
        self.calls.append(("rm_create_robot_arm", ip, port))
        return self._rc("rm_create_robot_arm", self.handle)

    def rm_delete_robot_arm(self, handle):
        self.calls.append(("rm_delete_robot_arm",))
        return self._rc("rm_delete_robot_arm")

    def rm_set_joint_drive_max_speed(self, handle, joint_num, value):
        self.calls.append(("rm_set_joint_drive_max_speed", joint_num, value))
        return self._rc("rm_set_joint_drive_max_speed")

    def rm_set_joint_drive_max_acc(self, handle, joint_num, value):
        self.calls.append(("rm_set_joint_drive_max_acc", joint_num, value))
        return self._rc("rm_set_joint_drive_max_acc")

    def rm_set_joint_drive_min_pos(self, handle, joint_num, value):
        self.calls.append(("rm_set_joint_drive_min_pos", joint_num, value))
        return self._rc("rm_set_joint_drive_min_pos")

    def rm_set_joint_drive_max_pos(self, handle, joint_num, value):
        self.calls.append(("rm_set_joint_drive_max_pos", joint_num, value))
        return self._rc("rm_set_joint_drive_max_pos")

    def rm_set_collision_state(self, handle, stage):
        self.calls.append(("rm_set_collision_state", stage))
        return self._rc("rm_set_collision_state")

    def rm_set_self_collision_enable(self, handle, state):
        self.calls.append(("rm_set_self_collision_enable", state))
        return self._rc("rm_set_self_collision_enable")


# --------------------------------------------------------------- 临时工作区
JOINT_LIMITS_YAML = """\
# 测试用声明限值（单位 SI）
rm75_arm:
  joint1: {lower: -3.106,  upper: 3.106,  velocity: 3.1416, acceleration: 5.2359, jerk: 30.0}
  joint2: {lower: -2.2689, upper: 2.2689, velocity: 3.1416, acceleration: 5.2359, jerk: 30.0}
  joint3: {lower: -3.106,  upper: 3.106,  velocity: 3.9270, acceleration: 5.2359, jerk: 30.0}
  joint4: {lower: -2.356,  upper: 2.356,  velocity: 3.9270, acceleration: 5.2359, jerk: 30.0}
  joint5: {lower: -3.106,  upper: 3.106,  velocity: 3.9270, acceleration: 5.2359, jerk: 30.0}
  joint6: {lower: -2.234,  upper: 2.234,  velocity: 3.9270, acceleration: 5.2359, jerk: 30.0}
  joint7: {lower: -3.14159, upper: 3.14159, velocity: 3.9270, acceleration: 5.2359, jerk: 30.0}
"""


def _measured_baseline(status="measured", with_evidence=True, samples=3):
    """构造一份 measured_limits.yaml 内容。"""
    joints = []
    for i in range(1, 8):
        rec = {}
        for field in ("acceleration", "jerk"):
            value = 5.0 if field == "acceleration" else 30.0
            item = {
                "value": value,
                "unit": "rad/s^2" if field == "acceleration" else "rad/s^3",
            }
            if with_evidence:
                item["source"] = "hwcollect ladder_acc_jerk 试验 %d" % i
                item["method"] = "阶梯速度包络"
                item["samples"] = samples
            rec[field] = item
        joints.append({"name": "joint%d" % i, **rec})
    return {
        "kind": "measured_limits",
        "schema_version": 1,
        "version": 3,
        "status": status,
        "captured_at": "2026-01-01T00:00:00",
        "arm": "left",
        "udp_period_ms": 5,
        "joints": joints,
        "tcp_repeatability": {
            "value_mm": 0.05,
            "unit": "mm",
            "source": "tcp_repeatability",
            "method": "固定目标 30 次",
            "samples": 30,
        },
    }


def _write_workspace(tmp_path, baseline, joint_limits=JOINT_LIMITS_YAML):
    """构造一个临时「仓库根」（含 workcell/ 与 dualarm_ws/src/rm_description）。"""
    ws = tmp_path / "ws"
    (ws / "workcell" / "baselines").mkdir(parents=True)
    (ws / "dualarm_ws" / "src" / "rm_description" / "config").mkdir(parents=True)
    (ws / "workcell" / "baselines" / "measured_limits.yaml").write_text(
        yaml.safe_dump(baseline, allow_unicode=True), encoding="utf-8"
    )
    (ws / "dualarm_ws" / "src" / "rm_description" / "config" / "joint_limits.yaml").write_text(
        joint_limits, encoding="utf-8"
    )
    return str(ws)


@pytest.fixture(autouse=True)
def _no_env_ws(monkeypatch):
    """清掉 ``RM_WS``，避免环境变量覆盖临时工作区。"""
    monkeypatch.delenv("RM_WS", raising=False)


@pytest.fixture
def link_factory():
    """构造已连接的 :class:`ArmLink`（假 SDK）。"""

    def _make(results=None, handle_id=1):
        lib = FakeLib(handle_id=handle_id, results=results)
        link = bridge.connect("192.168.1.18", 8080, lib=lib, enable_realtime=False)
        assert link.connected, link.reason
        return link, lib

    return _make


# ---------------------------------------------------------------- 负例：基线 blocked
def test_blocked_baseline_rejects_downlink(tmp_path, link_factory):
    """基线 status=blocked → 拒绝下发，0 次 SDK 调用，reason/next_action 齐备。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="blocked"))
    defaults = limits_mod.load_limit_defaults(ws=ws)

    assert defaults["status"] == limits_mod.STATUS_BLOCKED
    assert "未实测" in defaults["reason"]
    assert defaults["next_action"], "失败结果必须带可粘贴下一步"
    assert "hwcollect" in defaults["next_action"]
    assert defaults["values"] == {}, "拒绝下发时不得给出任何数值"

    link, lib = link_factory()
    res = link.set_joint_limits(defaults, dry_run=False)

    assert res["status"] == errors.STATUS_BLOCKED
    assert errors.exit_code(res) == 3
    assert res["reason"] and res["next_action"]
    assert lib.downlink_calls() == [], "被拒时不得产生任何下发调用"


def test_repo_baseline_is_blocked_by_default(repo_root):
    """仓库内真实基线（status=blocked）默认拒绝下发。"""
    defaults = limits_mod.load_limit_defaults(ws=repo_root)
    assert defaults["status"] == limits_mod.STATUS_BLOCKED
    assert defaults["values"] == {}


# ------------------------------------------------- 占位回退：必须显式 opt-in
def test_placeholder_fallback_requires_explicit_opt_in(tmp_path, link_factory):
    """占位回退只在 ``allow_placeholder=True`` 时发生，且必须带未实测标注。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="blocked"))

    strict = limits_mod.load_limit_defaults(ws=ws)
    assert strict["status"] == limits_mod.STATUS_BLOCKED

    loose = limits_mod.load_limit_defaults(ws=ws, allow_placeholder=True)
    assert loose["status"] == limits_mod.STATUS_PLACEHOLDER
    assert "placeholder" in loose["source"]
    assert "未实测" in loose["source"]
    assert loose["values"], "占位回退必须给出数值（来自 joint_limits.yaml）"

    link, lib = link_factory()
    res = link.set_joint_limits(loose, dry_run=False)

    assert res["status"] == errors.STATUS_OK
    assert res["source"] == limits_mod.PLACEHOLDER_SOURCE
    assert len(lib.downlink_calls()) == 7 * 4


def test_placeholder_fallback_never_used_by_default(tmp_path):
    """默认（allow_placeholder=False）下占位值绝不出现在结果里。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="blocked"))
    defaults = limits_mod.load_limit_defaults(ws=ws)
    assert defaults["status"] != limits_mod.STATUS_PLACEHOLDER
    assert defaults["source"] != limits_mod.PLACEHOLDER_SOURCE


# ---------------------------------------------------------------- 正例：实测基线
def test_measured_downlink_units(tmp_path, link_factory):
    """实测基线 → 7 关节 × 4 项下发，单位换算后与期望一致。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="measured"))
    defaults = limits_mod.load_limit_defaults(ws=ws)

    assert defaults["status"] == limits_mod.STATUS_MEASURED
    assert len(defaults["values"]) == 7

    link, lib = link_factory()
    res = link.set_joint_limits(defaults, dry_run=False)

    assert res["status"] == errors.STATUS_OK
    calls = lib.downlink_calls()
    assert len(calls) == 7 * 4, "应下发 7 关节 × 4 项"

    by_sym = {}
    for name, joint_num, value in calls:
        by_sym.setdefault(name, []).append((joint_num, value))

    for sym in ("rm_set_joint_drive_max_speed", "rm_set_joint_drive_max_acc",
                "rm_set_joint_drive_min_pos", "rm_set_joint_drive_max_pos"):
        assert len(by_sym[sym]) == 7
        assert sorted(j for j, _ in by_sym[sym]) == [1, 2, 3, 4, 5, 6, 7]

    # 速度：3.1416 rad/s → °/s
    speed = dict(by_sym["rm_set_joint_drive_max_speed"])
    assert speed[1] == pytest.approx(3.1416 * 180.0 / math.pi, abs=1e-3)
    assert speed[3] == pytest.approx(3.9270 * 180.0 / math.pi, abs=1e-3)
    # 加速度：实测 5.0 rad/s^2 → °/s^2
    acc = dict(by_sym["rm_set_joint_drive_max_acc"])
    assert acc[1] == pytest.approx(5.0 * 180.0 / math.pi, abs=1e-3)
    # 位置：-3.106 rad → °
    min_pos = dict(by_sym["rm_set_joint_drive_min_pos"])
    max_pos = dict(by_sym["rm_set_joint_drive_max_pos"])
    assert min_pos[1] == pytest.approx(-3.106 * 180.0 / math.pi, abs=1e-3)
    assert max_pos[7] == pytest.approx(3.14159 * 180.0 / math.pi, abs=1e-3)


def test_measured_baseline_missing_evidence_is_blocked(tmp_path):
    """status=measured 但缺 source/method/samples → 仍按「未实测」拒绝。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="measured", with_evidence=False))
    defaults = limits_mod.load_limit_defaults(ws=ws)
    assert defaults["status"] == limits_mod.STATUS_BLOCKED
    assert defaults["values"] == {}

    ws2 = _write_workspace(
        tmp_path / "b", _measured_baseline(status="measured", samples=1)
    )
    defaults2 = limits_mod.load_limit_defaults(ws=ws2)
    assert defaults2["status"] == limits_mod.STATUS_BLOCKED


def test_dry_run_makes_zero_sdk_calls(tmp_path, link_factory, capsys):
    """dry_run=True → 0 次 SDK 调用，但输出完整 diff（含单位）。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="measured"))
    defaults = limits_mod.load_limit_defaults(ws=ws)

    link, lib = link_factory()
    res = link.set_joint_limits(defaults, dry_run=True)

    assert res["status"] == errors.STATUS_OK
    assert res["dry_run"] is True
    assert res["sdk_calls"] == 0
    assert lib.downlink_calls() == []
    assert res["applied"] == []
    # 7 关节 × (4 下发项 + jerk 记录)
    assert len(res["diff"]) == 7 * 5
    for entry in res["diff"]:
        assert entry["current_unit"]
        assert entry["target_unit"]
    out = capsys.readouterr().out
    assert "限值 diff" in out
    assert "rad/s" in out


def test_set_joint_limits_runs_in_dry_run_by_default(tmp_path, link_factory):
    """默认就是 dry-run（不下发）。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="measured"))
    defaults = limits_mod.load_limit_defaults(ws=ws)
    link, lib = link_factory()
    res = link.set_joint_limits(defaults)
    assert res["dry_run"] is True
    assert lib.downlink_calls() == []


# ------------------------------------------------------------ 安全开关下发
def test_safety_downlink_ok(link_factory):
    """碰撞等级 + 自碰撞使能下发成功。"""
    link, lib = link_factory()
    res = link.set_safety(collision_stage=5, self_collision_enable=True)
    assert res["status"] == errors.STATUS_OK
    calls = lib.downlink_calls()
    assert [c[0] for c in calls] == [
        "rm_set_collision_state", "rm_set_self_collision_enable",
    ]
    assert calls[0][1] == 5
    assert calls[1][1] is True


@pytest.mark.parametrize("stage", [9, -1, 100])
def test_invalid_collision_stage_rejected(link_factory, stage):
    """越界等级 → 拒绝，且 0 次调用。"""
    link, lib = link_factory()
    res = link.set_safety(collision_stage=stage)
    assert res["status"] == errors.STATUS_ERROR
    assert errors.exit_code(res) == 4
    assert "越界" in res["reason"]
    assert lib.downlink_calls() == []


def test_invalid_collision_stage_type_rejected(link_factory):
    """非整数等级 → 拒绝，0 次调用。"""
    link, lib = link_factory()
    res = link.set_safety(collision_stage="5")
    assert res["status"] == errors.STATUS_ERROR
    assert lib.downlink_calls() == []


def test_invalid_self_collision_type_rejected(link_factory):
    """非 bool 的自碰撞使能 → 拒绝，0 次调用。"""
    link, lib = link_factory()
    res = link.set_safety(self_collision_enable=1)
    assert res["status"] == errors.STATUS_ERROR
    assert lib.downlink_calls() == []


def test_safety_without_any_switch_rejected(link_factory):
    """两个开关都没给 → 拒绝。"""
    link, lib = link_factory()
    res = link.set_safety()
    assert res["status"] == errors.STATUS_ERROR
    assert lib.downlink_calls() == []


def test_safety_rc_nonzero_reports_symbol_and_code(link_factory):
    """SDK 返回非 0 → error + 接口名 + 返回码。"""
    link, lib = link_factory(results={"rm_set_collision_state": -1})
    res = link.set_safety(collision_stage=3)
    assert res["status"] == errors.STATUS_ERROR
    assert "rm_set_collision_state" in res["reason"]
    assert "-1" in res["reason"]
    assert res["next_action"]


def test_limits_rc_nonzero_reports_symbol_and_code(tmp_path, link_factory):
    """限值下发中途返回非 0 → error + 接口名 + 返回码，且不静默吞掉。"""
    ws = _write_workspace(tmp_path, _measured_baseline(status="measured"))
    defaults = limits_mod.load_limit_defaults(ws=ws)
    link, lib = link_factory(results={"rm_set_joint_drive_max_acc": -3})
    res = link.set_joint_limits(defaults, dry_run=False)
    assert res["status"] == errors.STATUS_ERROR
    assert "rm_set_joint_drive_max_acc" in res["reason"]
    assert "-3" in res["reason"]


# ------------------------------------------------------------ 结构约束自检
def test_limits_module_has_no_rmctl_dependency(pkg_root):
    """硬件层不得反向依赖体验层。"""
    src = open(
        os.path.join(pkg_root, "rm_hw_bridge", "limits.py"), "r", encoding="utf-8"
    ).read()
    assert "import rmctl" not in src
    assert "from rmctl" not in src


def test_unit_conversion_is_single_point(pkg_root):
    """度↔弧度换算只在 bridge.py 一处（limits.py 不做换算）。"""
    lim = open(
        os.path.join(pkg_root, "rm_hw_bridge", "limits.py"), "r", encoding="utf-8"
    ).read()
    assert "math.pi" not in lim
    brg = open(
        os.path.join(pkg_root, "rm_hw_bridge", "bridge.py"), "r", encoding="utf-8"
    ).read()
    assert len(re.findall(r"180\.0 / math\.pi", brg)) == 1
    assert bridge.RAD2DEG == pytest.approx(180.0 / math.pi)
    assert bridge.DEG2RAD == pytest.approx(math.pi / 180.0)


def test_downlink_uses_only_whitelisted_symbols(pkg_root):
    """下发用到的符号必须在契约白名单里。"""
    from rm_hw_bridge import loader

    allowed = set(loader.contract_symbols())
    for sym in DOWNLINK_SYMBOLS:
        assert sym in allowed
