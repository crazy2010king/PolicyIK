"""``rmctl.process.spec`` 的加载与校验测试。

覆盖：
* 正常加载（段序列 / iterations / home 与 goal 原样返回）；
* **逐字段 blocked 语义**：缺文件 / YAML 语法错 / 顶层非映射 / 缺字段 /
  段类型非法 / 段名重复 / 运动目标二选一违规 / 数值个数不对 /
  夹爪动作非法 / 判定期望非法；
* 默认路径解析落在 ``workcell/tasks/pick_place.yaml``；
* 规格里**不出现恢复阶梯上限字段**（防止第二处配置源）；
* 结果与 ``rm_hw_bridge.gripper_config.load_gripper_config`` **同构**。

只用 git 跟踪的仓库文件 + pytest 临时目录（``tmp_path``），绝不依赖
.gitignore 目录。
"""
import os

import pytest

from rmctl.process import spec as spec_mod

#: 一份**合法**的最小规格（各负例在此基础上改写）
VALID_SPEC = """\
version: 1
name: pick_place
iterations: 2
home_joints_deg: [0, 0, 0, 0, 0, 0, 0]
segments:
  - name: home
    type: motion
    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]
"""


@pytest.fixture
def spec_ws(tmp_path):
    """一个含 ``workcell/tasks/`` 的临时工作区根。"""
    wsdir = tmp_path / "ws"
    (wsdir / "workcell" / "tasks").mkdir(parents=True)
    return str(wsdir)


def write_spec(spec_ws, text, name="pick_place.yaml"):
    """把规格文本写进临时工作区并返回路径。"""
    path = os.path.join(spec_ws, "workcell", "tasks", name)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path


def assert_blocked(res, field):
    """断言 blocked 语义：状态、指名字段、next_action 非空、spec 为 None。"""
    assert res["status"] == "blocked", res
    assert res["spec"] is None, res
    assert field in res["reason"], res
    assert res["next_action"], res
    assert res["evidence"], res


# --------------------------------------------------------------------------
# 正常路径
# --------------------------------------------------------------------------

def test_default_task_path_is_workcell_tasks_pick_place(spec_ws):
    """默认路径落在 ``<ws>/workcell/tasks/pick_place.yaml``。"""
    path = spec_mod.default_task_path(spec_ws)
    assert path == os.path.join(spec_ws, "workcell", "tasks", "pick_place.yaml")


def test_default_task_path_matches_repo_layout(repo_root):
    """仓库里的默认规格文件确实在 workcell/tasks/ 下。"""
    path = spec_mod.default_task_path(repo_root)
    assert path == os.path.join(repo_root, "workcell", "tasks", "pick_place.yaml")
    assert os.path.isfile(path)


def test_repo_pick_place_loads_ok(repo_root):
    """仓库自带的 pick_place.yaml 能被加载器成功加载。"""
    res = spec_mod.load_task_spec(ws=repo_root)
    assert res["status"] == "ok", res
    assert res["spec"] is not None
    assert res["next_action"] == ""
    assert res["evidence"]


def test_repo_spec_segments_and_values(repo_root):
    """段序列 / iterations / home 与 goal 原样返回。"""
    res = spec_mod.load_task_spec(ws=repo_root)
    spec = res["spec"]
    assert spec["version"] == 1
    assert spec["name"] == "pick_place"
    assert spec["iterations"] == 50
    assert spec["home_joints_deg"] == [0, 0, 0, 0, 0, 0, 0]
    names = [s["name"] for s in spec["segments"]]
    assert names == [
        "approach_pick", "grasp", "lift", "place_approach",
        "release", "retreat", "home",
    ]
    types = [s["type"] for s in spec["segments"]]
    assert types == [
        "motion", "gripper", "motion", "motion", "gripper", "motion", "motion",
    ]
    approach = spec["segments"][0]
    assert approach["goal_joints_deg"] == [0, 20, 30, 0, 0, 0, 0]
    assert approach["goal_pose"] is None
    grasp = spec["segments"][1]
    assert grasp["action"] == "close"
    assert grasp["speed"] == 100
    assert grasp["force"] == 100
    assert grasp["expect"] == "grasp_ok"
    release = spec["segments"][4]
    assert release["action"] == "open"
    assert release["expect"] == "grasp_empty"


def test_repo_spec_ends_at_home(repo_root):
    """每圈最后一段必须回到 home（保证下一圈起点一致）。"""
    res = spec_mod.load_task_spec(ws=repo_root)
    last = res["spec"]["segments"][-1]
    assert last["name"] == "home"
    assert last["goal_joints_deg"] == [0, 0, 0, 0, 0, 0, 0]


def test_minimal_valid_spec_loads(spec_ws):
    """最小合法规格能加载成功。"""
    path = write_spec(spec_ws, VALID_SPEC)
    res = spec_mod.load_task_spec(path)
    assert res["status"] == "ok", res
    assert res["spec"]["segments"][0]["name"] == "home"


def test_motion_segment_with_pose_only_is_ok(spec_ws):
    """笛卡尔目标（goal_pose）单独给出是合法的另一种运动目标。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_pose: [0.3, 0.0, 0.4, 0.0, 3.14, 0.0]\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert res["status"] == "ok", res
    seg = res["spec"]["segments"][0]
    assert seg["goal_pose"] == [0.3, 0.0, 0.4, 0.0, 3.14, 0.0]
    assert seg["goal_joints_deg"] is None


def test_judge_segment_with_expect_is_ok(spec_ws):
    """judge 段的 expect 属于八类判定结果时合法。"""
    text = VALID_SPEC + """\
  - name: check
    type: judge
    expect: grasp_ok
"""
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert res["status"] == "ok", res
    assert res["spec"]["segments"][-1]["expect"] == "grasp_ok"


# --------------------------------------------------------------------------
# 负例：逐字段 blocked
# --------------------------------------------------------------------------

def test_missing_file_blocked(spec_ws):
    """文件缺失 → blocked（不抛异常）。"""
    res = spec_mod.load_task_spec(
        os.path.join(spec_ws, "workcell", "tasks", "nope.yaml"))
    assert res["status"] == "blocked"
    assert res["spec"] is None
    assert res["next_action"]
    assert "nope.yaml" in res["reason"]


def test_yaml_syntax_error_blocked(spec_ws):
    """YAML 语法错 → blocked（解析异常不外泄）。"""
    path = write_spec(spec_ws, "version: 1\nname: [oops\n")
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "解析失败")


def test_top_level_not_mapping_blocked(spec_ws):
    """顶层非映射 → blocked。"""
    path = write_spec(spec_ws, "- just\n- a\n- list\n")
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "顶层必须是映射")


def test_missing_version_blocked(spec_ws):
    """缺 version → blocked（reason 点名字段）。"""
    text = VALID_SPEC.replace("version: 1\n", "")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "version")


def test_missing_name_blocked(spec_ws):
    """缺 name → blocked。"""
    text = VALID_SPEC.replace("name: pick_place\n", "")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "name")


def test_missing_iterations_blocked(spec_ws):
    """缺 iterations → blocked。"""
    text = VALID_SPEC.replace("iterations: 2\n", "")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "iterations")


def test_invalid_iterations_blocked(spec_ws):
    """iterations 非整数 / < 1 → blocked。"""
    text = VALID_SPEC.replace("iterations: 2", "iterations: 0")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "iterations")


def test_missing_home_joints_blocked(spec_ws):
    """缺 home_joints_deg → blocked。"""
    text = VALID_SPEC.replace("home_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n", "")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "home_joints_deg")


def test_home_joints_not_seven_numbers_blocked(spec_ws):
    """home_joints_deg 不是 7 个有限数 → blocked。"""
    text = VALID_SPEC.replace(
        "home_joints_deg: [0, 0, 0, 0, 0, 0, 0]", "home_joints_deg: [0, 0, 0]")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "home_joints_deg")


def test_missing_segments_blocked(spec_ws):
    """缺 segments → blocked。"""
    text = VALID_SPEC.split("segments:")[0]
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "segments")


def test_empty_segments_blocked(spec_ws):
    """空 segments → blocked。"""
    text = VALID_SPEC.split("segments:")[0] + "segments: []\n"
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "segments")


def test_segment_type_invalid_blocked(spec_ws):
    """段 type 不在 {motion, gripper, judge} → blocked。"""
    text = VALID_SPEC.replace("type: motion", "type: dance")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "type")


def test_segment_missing_name_blocked(spec_ws):
    """段缺 name → blocked。"""
    text = VALID_SPEC.replace("  - name: home\n", "  - \n")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "name")


def test_duplicate_segment_names_blocked(spec_ws):
    """段名重复 → blocked。"""
    text = VALID_SPEC + """\
  - name: home
    type: motion
    goal_joints_deg: [1, 1, 1, 0, 0, 0, 0]
"""
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "段名重复")


def test_motion_segment_without_any_target_blocked(spec_ws):
    """motion 段既无 goal_joints_deg 也无 goal_pose → blocked。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n", "")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "goal_joints_deg")
    assert "goal_pose" in res["reason"]


def test_motion_segment_with_both_targets_blocked(spec_ws):
    """motion 段同时给 joints 与 pose → blocked（代码不猜）。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n"
        "    goal_pose: [0.3, 0.0, 0.4, 0.0, 3.14, 0.0]\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "goal_joints_deg")
    assert "goal_pose" in res["reason"]


def test_goal_joints_not_seven_numbers_blocked(spec_ws):
    """goal_joints_deg 不是 7 个数 → blocked。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]", "    goal_joints_deg: [0, 0]")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "goal_joints_deg")


def test_goal_joints_with_non_number_blocked(spec_ws):
    """goal_joints_deg 里出现非数值 → blocked。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]",
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, oops]",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "goal_joints_deg")


def test_goal_pose_not_six_numbers_blocked(spec_ws):
    """goal_pose 不是 6 个数 → blocked。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_pose: [0.3, 0.0, 0.4]\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "goal_pose")


def test_gripper_action_invalid_blocked(spec_ws):
    """gripper 段 action 非法 → blocked。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n"
        "  - name: grab\n"
        "    type: gripper\n"
        "    action: squeeze\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "action")


def test_judge_expect_invalid_blocked(spec_ws):
    """judge 段 expect 不在八类结果集 → blocked。"""
    text = VALID_SPEC + """\
  - name: check
    type: judge
    expect: grasp_maybe
"""
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "expect")


def test_judge_segment_missing_expect_blocked(spec_ws):
    """judge 段缺 expect → blocked。"""
    text = VALID_SPEC + """\
  - name: check
    type: judge
"""
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "expect")


def test_gripper_expect_invalid_blocked(spec_ws):
    """gripper 段的 expect 非法 → blocked。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n"
        "  - name: grab\n"
        "    type: gripper\n"
        "    action: close\n"
        "    expect: nope\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "expect")


def test_unknown_segment_field_blocked(spec_ws):
    """段里出现未知字段 → blocked（本层不猜语义）。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n"
        "    speed: 10\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "speed")


# --------------------------------------------------------------------------
# 第二处配置源的防线
# --------------------------------------------------------------------------

def test_spec_contains_no_recovery_ladder_limit(repo_root):
    """规格里不出现恢复阶梯上限字段（防止第二处配置源）。"""
    res = spec_mod.load_task_spec(ws=repo_root)
    spec = res["spec"]
    flat = set(spec.keys())
    for seg in spec["segments"]:
        flat.update(seg.keys())
    for forbidden in ("max_regrasp", "max_stall_retry", "max_offline_retry",
                      "recovery_ladder", "recovery_steps"):
        assert forbidden not in flat


def test_forbidden_recovery_field_in_spec_blocked(spec_ws):
    """规格里写恢复阶梯上限 → blocked（唯一来源在工位文件 / 硬件层）。"""
    text = VALID_SPEC.replace(
        "iterations: 2", "iterations: 2\nmax_regrasp: 5")
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert_blocked(res, "max_regrasp")


def test_gripper_speed_and_force_default_to_none(spec_ws):
    """gripper 段 speed/force 缺省 → None（不写死数值）。"""
    text = VALID_SPEC.replace(
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n",
        "    goal_joints_deg: [0, 0, 0, 0, 0, 0, 0]\n"
        "  - name: grab\n"
        "    type: gripper\n"
        "    action: close\n",
    )
    path = write_spec(spec_ws, text)
    res = spec_mod.load_task_spec(path)
    assert res["status"] == "ok", res
    seg = res["spec"]["segments"][1]
    assert seg["speed"] is None
    assert seg["force"] is None


# --------------------------------------------------------------------------
# 与夹爪配置加载器同构
# --------------------------------------------------------------------------

def test_result_shape_is_isomorphic_to_gripper_config(repo_root):
    """结果字典与 ``rm_hw_bridge.gripper_config`` 同构（键集合一致）。"""
    from rm_hw_bridge.gripper_config import load_gripper_config

    grip = load_gripper_config(ws=repo_root)
    task = spec_mod.load_task_spec(ws=repo_root)
    assert grip["status"] == "ok", grip
    assert task["status"] == "ok", task
    common = {"status", "reason", "next_action", "evidence"}
    # ``gripper_declared`` 是结构化的「工位是否声明夹爪」布尔字段
    # （``rm_hw_bridge.gripper_config.DECLARED_KEY``），恒定存在。
    assert set(grip.keys()) - common == {"config", "gripper_declared"}
    assert set(task.keys()) - common == {"spec"}
    assert common <= set(grip.keys())
    assert common <= set(task.keys())
    # 失败时同样是「spec 为 None + next_action 非空」
    bad = spec_mod.load_task_spec(os.path.join(repo_root, "workcell", "nope.yaml"))
    assert bad["spec"] is None
    assert bad["next_action"]
