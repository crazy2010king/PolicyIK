"""真机采集脚本骨架（rmctl.hwcollect）的测试。

覆盖：

* 六类实验在 ``--dry-run`` 下各自产出合法 JSON（status/evidence/summary 必填字段）；
* SDK 缺失时真机路径返回 NEEDS_HUMAN，**不抛异常、不产生数值**；
* 正则扫描断言源码中不出现黑名单符号的调用形态（零位标定只能走示教器）；
* 量级约束：UDP 全量帧不落内存（只保留聚合统计 + 抽样帧）。

只用 git 跟踪的文件（源 URDF、workcell/baselines/）+ pytest 临时目录。
"""
import json
import os
import re

import pytest

from rmctl import hwcollect

REQUIRED_KEYS = ("schema", "experiment", "mode", "status", "arm", "started_at",
                 "finished_at", "params", "samples", "summary", "evidence")

EXPERIMENT_NAMES = ("udp_stability", "ladder_acc_jerk", "tcp_repeatability",
                    "fk_cross", "estop_drill", "zero_capture")


@pytest.fixture(scope="module")
def urdf(repo_root):
    """git 跟踪的源 URDF（绝对路径），不依赖 build/model（.gitignore）。"""
    p = os.path.join(repo_root, "dualarm_ws", "src", "rm_description", "urdf",
                     "rm75_clean.urdf")
    assert os.path.exists(p), p
    return p


def _run(name, dry_run=True, **kwargs):
    return hwcollect.run_experiment(name, dry_run=dry_run, **kwargs)


def _no_sdk(monkeypatch):
    """模拟"RM-API2 SDK 不存在"（本仓库确实没有 .deps/rm_api2）。"""
    monkeypatch.setattr(hwcollect, "sdk_candidates", lambda *a, **k: [])
    monkeypatch.setenv(hwcollect.ENV_SDK_LIB, "/nonexistent/libapi_c.so")


# ---------------------------------------------------------------- dry-run 正例
@pytest.mark.parametrize("name", EXPERIMENT_NAMES)
def test_dry_run_produces_valid_json(name, urdf):
    """六类实验在 dry-run 下各自产出合法 JSON（必填字段齐全、无 NaN/Inf）。"""
    kwargs = {"urdf_path": urdf} if name == "fk_cross" else {}
    res = _run(name, dry_run=True, **kwargs)
    for key in REQUIRED_KEYS:
        assert key in res, "缺字段 %s" % key
    assert res["status"] == hwcollect.STATUS_DRY_RUN
    assert res["mode"] == "dry_run"
    assert res["experiment"] == name
    assert isinstance(res["samples"], list)
    assert isinstance(res["summary"], dict)
    assert isinstance(res["evidence"], list) and res["evidence"]
    # NaN/Inf 不可 JSON 化（allow_nan=False 会抛）
    text = json.dumps(res, ensure_ascii=False, allow_nan=False)
    assert json.loads(text)["status"] == hwcollect.STATUS_DRY_RUN


@pytest.mark.parametrize("name", EXPERIMENT_NAMES)
def test_dry_run_marks_evidence_as_synthetic(name, urdf):
    """dry-run 证据必须标明"合成数据"，防止被误当成真机读数回填基线。"""
    kwargs = {"urdf_path": urdf} if name == "fk_cross" else {}
    res = _run(name, dry_run=True, **kwargs)
    kinds = {e.get("kind") for e in res["evidence"]}
    assert "synthetic" in kinds
    blob = json.dumps(res, ensure_ascii=False)
    assert "不是真机" in blob


def test_dry_run_writes_to_out_dir_without_touching_baselines(tmp_path, repo_root):
    """--out 指定目录落盘；输出**不得**写进 workcell/baselines/。"""
    baselines = os.path.join(repo_root, "workcell", "baselines")
    before = sorted(os.listdir(baselines))
    res = hwcollect.run_experiment("zero_capture", dry_run=True)
    path = hwcollect.write_result(res, str(tmp_path), name="zero_capture")
    assert os.path.exists(path)
    assert json.load(open(path, encoding="utf-8"))["status"] == hwcollect.STATUS_DRY_RUN
    assert sorted(os.listdir(baselines)) == before


def test_write_result_default_is_temp_dir_not_repo():
    """缺省输出目录是临时目录（不污染 git 跟踪目录）。"""
    res = hwcollect.run_experiment("zero_capture", dry_run=True)
    path = hwcollect.write_result(res)
    try:
        assert os.path.exists(path)
        assert "rmctl-hwcollect-" in path
    finally:
        os.remove(path)
        os.rmdir(os.path.dirname(path))


# ---------------------------------------------------------------- 缺 SDK 降级
@pytest.mark.parametrize("name", EXPERIMENT_NAMES)
def test_missing_sdk_returns_needs_human_without_raising(name, monkeypatch, urdf):
    """SDK 缺失时：返回 NEEDS_HUMAN，不抛异常，且**不产生任何数值**。"""
    _no_sdk(monkeypatch)
    kwargs = {"urdf_path": urdf} if name == "fk_cross" else {}
    res = hwcollect.run_experiment(name, dry_run=False, **kwargs)
    assert res["status"] == hwcollect.STATUS_NEEDS_HUMAN
    assert res["samples"] == []
    assert res["summary"] == {}
    assert res["reason"]
    assert res["next_action"]
    assert res["evidence"][0]["kind"] == "blocker"
    assert json.dumps(res["summary"], ensure_ascii=False) == "{}"


def test_missing_sdk_message_points_at_deps_dir(monkeypatch):
    """SDK 缺失信息必须给出可粘贴的下一步（放入 .deps/rm_api2/）。"""
    monkeypatch.setattr(hwcollect, "sdk_candidates", lambda *a, **k: [])
    with pytest.raises(hwcollect.SdkMissing) as exc:
        hwcollect.load_sdk()
    text = str(exc.value)
    assert ".deps/rm_api2" in text
    assert hwcollect.ENV_SDK_LIB in text


def test_sdk_candidates_prefers_env_var(monkeypatch, tmp_path):
    """环境变量优先于默认路径（可粘贴的定位顺序）。"""
    lib = str(tmp_path / "libapi_c.so")
    monkeypatch.setenv(hwcollect.ENV_SDK_LIB, lib)
    cands = hwcollect.sdk_candidates(ws=str(tmp_path), env=os.environ)
    assert cands[0] == lib
    assert any(c.endswith(os.path.join("rm_api2", "libapi_c.so")) for c in cands)


# ---------------------------------------------------------------- 黑名单
def test_blacklist_contains_zero_set_symbol():
    """黑名单必须含零位标定符号（只能走示教器受控流程）。"""
    assert "rm_set_joint_zero_pos" in hwcollect.BLACKLIST


def test_source_has_no_blacklist_call():
    """正则扫描：源码中不出现黑名单符号的调用形态。"""
    src = hwcollect._module_source()
    assert src
    for name in hwcollect.BLACKLIST:
        assert re.search(r"%s\s*\(" % re.escape(name), src) is None, \
            "源码出现黑名单符号调用: %s" % name
    assert hwcollect.blacklist_scan() == []
    assert hwcollect.blacklist_scan(src) == []


def test_blacklist_symbol_not_in_whitelist():
    assert not (set(hwcollect.WHITELIST) & set(hwcollect.BLACKLIST))


def test_whitelist_covers_minimal_readonly_set():
    """白名单必须覆盖约定的最小只读集 + 停机族。"""
    required = {"rm_init", "rm_create_robot_arm", "rm_set_realtime_push",
                "rm_get_joint_degree", "rm_get_current_arm_state",
                "rm_get_arm_all_state", "rm_movej", "rm_set_arm_stop",
                "rm_set_arm_slow_stop", "rm_set_arm_pause", "rm_set_arm_continue"}
    assert required <= set(hwcollect.WHITELIST)


def test_threat_notes_cover_q3_surfaces():
    """Q3 威胁面（三线程模式 / 单客户端 / 黑名单 / 位姿用途）必须显式提示。"""
    ids = {n["id"] for n in hwcollect.THREAT_NOTES}
    assert {"blacklist_symbol", "controller_pose_scope", "triple_thread_mode",
            "single_client_per_endpoint"} <= ids


# ---------------------------------------------------------------- 各实验口径
def test_udp_stability_reports_dropout_and_period_multiple():
    """UDP 稳定性：给出断流次数、周期倍数判定、聚合统计。"""
    res = _run("udp_stability", dry_run=True, duration_s=10, period_ms=5)
    s = res["summary"]
    assert s["period_ms"] == 5
    assert s["period_multiple_ok"] is True
    assert s["dropout_count"] >= 1        # 合成数据里刻意挖了断流
    assert s["frames"] > 100
    assert s["expected_frames"] == 2000


def test_udp_stability_does_not_buffer_all_frames():
    """量级约束：600 s @5 ms ≈ 12 万帧，只保留聚合 + 抽样帧（内存可控）。"""
    res = _run("udp_stability", dry_run=True, duration_s=600, period_ms=5)
    s = res["summary"]
    assert s["frames"] > 100000
    assert len(res["samples"]) <= 200      # 抽样上限
    assert s["sampled_frames"] == len(res["samples"])


def test_ladder_uses_envelope_not_single_peak():
    """阶梯实验取多次采样的包络（保守下界），不是单次极值。"""
    res = _run("ladder_acc_jerk", dry_run=True, trials=3)
    s = res["summary"]
    assert s["trials"] == 3
    assert s["derivative_method"] == "rmctl.retime.derivatives"
    assert s["unit_acceleration"] == "rad/s^2"
    assert s["unit_jerk"] == "rad/s^3"
    for j in range(1, 8):
        key = "joint%d" % j
        assert s["acceleration_rad_s2"][key] <= s["acceleration_observed_max_rad_s2"][key]
        assert s["jerk_rad_s3"][key] <= s["jerk_observed_max_rad_s3"][key]
        assert s["acceleration_rad_s2"][key] > 0.0


def test_ladder_values_are_plausible_radians_per_second_squared():
    """单位口径：弧度制 acc 峰值应在 rad/s^2 的合理量级（不是度制的 57 倍）。"""
    res = _run("ladder_acc_jerk", dry_run=True, trials=3)
    accs = list(res["summary"]["acceleration_rad_s2"].values())
    assert all(0.1 < a < 500.0 for a in accs), accs


def test_tcp_repeatability_reports_spread_and_distinguishes_accuracy():
    """TCP 重复性：给出散布统计，并显式区分重复性 ≠ 绝对精度。"""
    res = _run("tcp_repeatability", dry_run=True, repeats=30)
    s = res["summary"]
    assert s["repeats"] == 30
    assert s["nominal_repeatability_mm"] == hwcollect.TCP_NOMINAL_MM
    assert "重复性" in s["scope"] and "绝对精度" in s["scope"]
    assert s["max_deviation_mm"] >= 0.0
    assert len(res["samples"]) == 30


def test_fk_cross_dry_run_passes_and_uses_cross_check(urdf):
    """FK 交叉：dry-run 走 fk_cross.cross_check，零偏移应通过。"""
    res = hwcollect.run_experiment("fk_cross", dry_run=True, n=8, urdf_path=urdf)
    s = res["summary"]
    assert s["ok"] is True
    assert s["samples"] == 8
    assert s["max_pos_dev_mm"] < 1e-6
    assert "仅用于 FK 交叉校验" in s["controller_pose_usage"]
    assert len(res["samples"]) == 8


def test_estop_drill_reports_recovery_and_no_auto_clear():
    """急停演练：记录恢复路径，且**不自动清错**。"""
    res = _run("estop_drill", dry_run=True)
    s = res["summary"]
    assert "soft_stop" in s and "emergency_stop" in s
    assert res["params"]["auto_clear_error"] is False
    assert s["recovery"]
    assert len(res["params"]["safety_checklist"]) >= 3


def test_zero_capture_does_not_fabricate_zero_deg():
    """零位采集：脚本只读，zero_deg 必须留 null 等示教器确认。"""
    res = _run("zero_capture", dry_run=True, photos=["photos/j1.jpg"])
    assert res["summary"]["zero_deg"] is None
    assert res["summary"]["joints"] == {}
    assert res["params"]["photos"] == ["photos/j1.jpg"]


# ---------------------------------------------------------------- CLI
def test_cli_dry_run_exits_zero_and_writes_file(tmp_path):
    """CLI：dry-run 退出码 0 并落盘。"""
    code = hwcollect.main(["udp_stability", "--dry-run", "--out", str(tmp_path),
                            "--duration-s", "2"])
    assert code == 0
    files = [f for f in os.listdir(str(tmp_path)) if f.endswith(".json")]
    assert files
    data = json.load(open(os.path.join(str(tmp_path), files[0]), encoding="utf-8"))
    assert data["status"] == hwcollect.STATUS_DRY_RUN


def test_cli_needs_human_exit_code_3(tmp_path, monkeypatch):
    """CLI：缺 SDK 时退出码 3，并写出 NEEDS_HUMAN 结果（含修复动作）。"""
    _no_sdk(monkeypatch)
    code = hwcollect.main(["zero_capture", "--out", str(tmp_path)])
    assert code == 3
    files = [f for f in os.listdir(str(tmp_path)) if f.endswith(".json")]
    assert files
    data = json.load(open(os.path.join(str(tmp_path), files[0]), encoding="utf-8"))
    assert data["status"] == hwcollect.STATUS_NEEDS_HUMAN
    assert data["next_action"]


def test_cli_rejects_bad_target(tmp_path):
    """负例：--target 不是 7 个数字 → 退出码 2。"""
    assert hwcollect.main(["tcp_repeatability", "--dry-run", "--target", "1,2",
                           "--out", str(tmp_path)]) == 2


def test_cli_rejects_unknown_experiment():
    """负例：未知实验名 → argparse 直接报错退出（非零）。"""
    with pytest.raises(SystemExit) as exc:
        hwcollect.main(["no_such_experiment"])
    assert exc.value.code != 0


def test_run_experiment_rejects_unknown_name():
    """负例：run_experiment 未知名字抛 HwCollectError（可读错误）。"""
    with pytest.raises(hwcollect.HwCollectError):
        hwcollect.run_experiment("no_such_experiment")
