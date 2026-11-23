"""`rmctl connect` 子命令的 CLI 契约测试（T3）。

覆盖：

* ``--dry-run --json`` → 退出码 0，输出是**单个可 json.loads 的对象**；
* 无 SDK → 退出码 3（blocked），带**可粘贴**的 next_action；
* 非法 ``--ip`` → 退出码 2；
* ``--json`` 输出**不被 SDK 的 C 层 printf 噪声污染**；
* 假 SDK 下 ``--json`` 输出含 R004 全字段快照 + ``controller_pose_scope``；
* 基线 ``status=blocked`` 时仍用规划值，但输出**标注来源且不声称已实测连通**。

只用 git 跟踪的仓库文件 + pytest 临时目录 + **注入的假 SDK**。
"""
from __future__ import annotations

import json
import os

import pytest

from rmctl import cli


@pytest.fixture()
def repo(repo_root):
    return repo_root


def _run(capsys, argv):
    """跑一次 CLI，返回 (exit_code, stdout, stderr)。"""
    code = cli.main(argv)
    cap = capsys.readouterr()
    return code, cap.out, cap.err


# ------------------------------------------------------------------ dry-run
def test_dry_run_json_exit_zero_and_parseable(capsys, repo):
    """--dry-run --json → 退出码 0，输出可 json.loads。"""
    code, out, err = _run(capsys, ["--ws", repo, "connect", "--arm", "left",
                                   "--dry-run", "--json"])
    assert code == 0, (out, err)
    data = json.loads(out)          # 必须是**单个** JSON 对象
    assert data["status"] == "ok"
    assert data["mode"] == "dry_run"
    assert data["arm"] == "left"
    assert data["ip"] == "192.168.1.18"      # 来自 network_plan 基线
    assert data["port"] == 8080
    assert data["udp_port"] == 8091
    assert data["realtime_period_ms"] == 5
    assert "未连接真机" in data["reason"]


def test_dry_run_json_has_no_snapshot_values(capsys, repo):
    """dry-run 不得产生任何实测数值（不伪造）。"""
    code, out, _ = _run(capsys, ["--ws", repo, "connect", "--dry-run", "--json"])
    assert code == 0
    data = json.loads(out)
    assert "snapshot" not in data
    assert "joint_deg" not in out


def test_dry_run_marks_baseline_source_as_unmeasured(capsys, repo):
    """基线 blocked 时仍用规划值，但必须标注来源且不声称已实测连通。"""
    code, out, _ = _run(capsys, ["--ws", repo, "connect", "--dry-run", "--json"])
    assert code == 0
    data = json.loads(out)
    assert "network_plan.yaml" in data["ip_source"]
    assert "未实测连通" in data["ip_source"]
    assert data["baseline_status"] == "blocked"


def test_dry_run_right_arm_uses_right_baseline(capsys, repo):
    """--arm right 取右臂的规划 IP/UDP 端口。"""
    code, out, _ = _run(capsys, ["--ws", repo, "connect", "--arm", "right",
                                 "--dry-run", "--json"])
    assert code == 0
    data = json.loads(out)
    assert data["ip"] == "192.168.1.19"
    assert data["udp_port"] == 8092


# ------------------------------------------------------------------ 参数非法
def test_invalid_ip_exits_two(capsys, repo):
    """非法 --ip → 退出码 2（输入非法）。"""
    code, out, err = _run(capsys, ["--ws", repo, "connect", "--ip", "not-an-ip",
                                   "--json"])
    assert code == 2, (out, err)
    assert "非法 IP" in err


def test_invalid_port_exits_two(capsys, repo):
    """非法端口 → 退出码 2。"""
    code, out, err = _run(capsys, ["--ws", repo, "connect", "--ip", "192.168.1.18",
                                   "--port", "99999", "--json"])
    assert code == 2, (out, err)


def test_invalid_arm_exits_two(capsys, repo):
    """--arm 只允许 left/right。"""
    with pytest.raises(SystemExit) as exc:
        cli.main(["--ws", repo, "connect", "--arm", "middle"])
    assert exc.value.code == 2


# ------------------------------------------------------------------ SDK 缺失
def test_missing_sdk_exits_three(capsys, repo, monkeypatch):
    """无 SDK → 退出码 3（blocked）+ 可粘贴 next_action。"""
    import rm_hw_bridge.bridge as hwbridge
    import rm_hw_bridge.loader as hwloader

    monkeypatch.setattr(hwloader, "find_library", lambda *a, **k: None)
    monkeypatch.setattr(hwloader, "sdk_candidates", lambda *a, **k: ["/nonexistent/libapi_c.so"])

    code, out, err = _run(capsys, ["--ws", repo, "connect", "--json"])
    assert code == 3, (out, err)
    data = json.loads(out)
    assert data["status"] == "blocked"
    assert data["reason"]
    assert data["next_action"]
    assert "RM_API2_LIB" in data["next_action"] or ".deps/rm_api2" in data["next_action"]
    assert hwbridge is not None


# ------------------------------------------------------------------ 假 SDK
class _FakeLink:
    """假连接对象：CLI 只用 status/to_dict/snapshot/close。"""

    def __init__(self, snapshot_result):
        self.status = "ok"
        self._snap = snapshot_result

    def to_dict(self):
        return {"status": "ok", "reason": "已连接", "next_action": "", "evidence": []}

    def snapshot(self):
        return self._snap

    def close(self):
        pass


def _fake_snapshot():
    return {
        "status": "ok",
        "reason": "状态快照采集成功",
        "next_action": "",
        "evidence": [],
        "snapshot": {
            "joint_deg": [1.0] * 7,
            "joint_current_mA": [100.0] * 7,
            "joint_temperature_c": [30.0] * 7,
            "joint_voltage_v": [12.0] * 7,
            "joint_en_flag": [1] * 7,
            "joint_err_code": [0] * 7,
            "arm_err": [],
            "brake_state": {"err_flag": 0, "brake_state": 0, "brake_open": [False] * 7,
                            "hard_brake_joints": [1, 2, 3, 4],
                            "soft_brake_joints": [5, 6, 7]},
            "controller_trajectory_type": 0,
            "controller_trajectory_type_name": "RM_NO_PLANNING_E",
            "arm_current_status": 0,
            "arm_current_status_name": "RM_IDLE_E",
            "realtime": {"frames": 0, "last_ts": None, "period_ms": 5},
            "controller_pose": {"scope": "cross_check_only"},
            "controller_pose_scope": "cross_check_only",
        },
    }


def test_json_output_not_polluted_by_sdk_noise(capsys, repo, monkeypatch):
    """SDK 的 C 层 printf 噪声不得污染 --json 输出。"""
    import rm_hw_bridge.bridge as hwbridge

    def noisy_connect(*a, **k):
        # 模拟 SDK 的 C 层噪声：直接写 fd 1（Python 层 redirect_stdout 拦不住）
        os.write(1, b"[rm_create_robot_arm] socket connect err!\n")
        return _FakeLink(_fake_snapshot())

    monkeypatch.setattr(hwbridge, "connect", noisy_connect)

    code, out, err = _run(capsys, ["--ws", repo, "connect", "--json"])
    assert code == 0, (out, err)
    # 输出必须是单个可解析 JSON 对象（噪声被隔离）
    data = json.loads(out)
    assert data["status"] == "ok"
    assert data["snapshot"]["controller_pose_scope"] == "cross_check_only"


def test_json_output_contains_full_snapshot(capsys, repo, monkeypatch):
    """假 SDK 下 --json 输出含 R004 全字段快照。"""
    import rm_hw_bridge.bridge as hwbridge

    monkeypatch.setattr(hwbridge, "connect",
                        lambda *a, **k: _FakeLink(_fake_snapshot()))
    code, out, _ = _run(capsys, ["--ws", repo, "connect", "--json"])
    assert code == 0
    data = json.loads(out)
    snap = data["snapshot"]
    for field in ("joint_deg", "joint_current_mA", "joint_temperature_c",
                  "joint_voltage_v", "joint_en_flag", "joint_err_code",
                  "arm_err", "brake_state", "controller_trajectory_type",
                  "arm_current_status", "realtime", "controller_pose",
                  "controller_pose_scope"):
        assert field in snap, "快照缺字段 %s" % field
    assert snap["controller_pose_scope"] == "cross_check_only"
    assert data["arm"] == "left"
    assert data["ip"] == "192.168.1.18"


def test_arm_unreachable_exits_four(capsys, repo, monkeypatch):
    """臂不可达（SDK 返回非 0 / 句柄无效）→ 退出码 4（error）。"""
    import rm_hw_bridge.bridge as hwbridge

    class _ErrLink:
        status = "error"

        def to_dict(self):
            return {"status": "error",
                    "reason": "rm_create_robot_arm 返回无效句柄 id=-1（控制器不可达）",
                    "next_action": "ping 192.168.1.18 确认控制器在线后重跑",
                    "evidence": ["rm_create_robot_arm handle.id=-1"]}

        def snapshot(self):
            return {"status": "error", "reason": "x", "next_action": "y", "evidence": []}

        def close(self):
            pass

    monkeypatch.setattr(hwbridge, "connect", lambda *a, **k: _ErrLink())
    code, out, err = _run(capsys, ["--ws", repo, "connect", "--json"])
    assert code == 4, (out, err)
    data = json.loads(out)
    assert data["status"] == "error"
    assert data["next_action"]
    assert data["snapshot"] is None
