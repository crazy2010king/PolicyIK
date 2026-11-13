#!/usr/bin/env python3
"""环境自检工具（T04）：把 versions.lock 的声明与"实装 reality"逐项对照。

设计要点
--------
* **唯一真相源**是仓库根的 ``versions.lock``；本脚本不硬编码任何版本号/名单，
  profile 的解析、包的解析全部复用 ``tools/gen_constraints.py``（T01）与
  ``tools/wheels_lock.py``（T03）的既有内核，避免出现第二份真相源。
* 本工具是 **CLI 可读**的：每个子检查输出一段人类可读的对照表，
  并以**退出码**表达一致性（0 = 通过，1 = 检查失败，2 = 运行期错误）。
* 可被 pytest（``tests/test_verify_env.py``）与 CI（``scripts/ci.sh``）直接复用，
  因此锁文件不会静默漂移。

子检查
------
    --lock-only     只校验 versions.lock 自洽性（必填字段 / constraints 钉版一致 /
                    wheels 映射格式），**不依赖已安装包**（新机没装依赖也能跑）。
    --check         逐包比对 versions.lock 声明版本 vs importlib.metadata 实装版本；
                    按当前解释器版本自动选择 profile（3.12 → baseline，3.10 → fallback）。
    --wheels        校验 vendor/wheels 的 SHA256SUMS；归档缺失时给出明确提示（SKIP），
                    不崩溃。
    --forbidden     确认 versions.lock:forbidden_apis 名单存在、覆盖核心禁用接口、
                    与 allowed_apis 不冲突，且（若扫描脚本已存在）扫描脚本确实以锁为名单来源。
    --package-map   PackageMap 解析冒烟：复用 rm75_drake.package_map.build_package_map()。
    --model         模型自检（S01-T06）：复用 rm75_drake.models 加载改造版 RM75-B，
                    输出 DoF / 总质量 / 零位末端高度 / 7 条限位摘要，
                    并校验 models/rm75_drake/joint_limits.yaml 与 URDF 一致。
                    该子检查**依赖 Drake**，因此不进入"未指定子检查时的默认全跑"集合，
                    需显式指定（CI 由 scripts/ci.sh fast 档调用）。
    --plan          规划链路自检（S02-T08）：装配场景 → 一次 IK → 一段短规划
                    → 一次碰撞复检；打印 ``[PASS]/[FAIL] plan — ...``。
                    该子检查**依赖 Drake**，同样需显式指定。

退出码：0 通过 / 1 检查失败 / 2 运行期错误（锁文件不可读、包映射构建失败、
Drake 不可用等）。
"""

from __future__ import annotations

import argparse
import importlib.metadata as importlib_metadata
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = Path(__file__).resolve().parent
DEFAULT_LOCK = REPO_ROOT / "versions.lock"
DEFAULT_CONSTRAINTS_DIR = REPO_ROOT / "constraints"
DEFAULT_WHEELS_DIR = REPO_ROOT / "vendor" / "wheels"
DEFAULT_REGISTRY = REPO_ROOT / "licenses" / "registry.yaml"
SCANNER = REPO_ROOT / "scripts" / "scan_forbidden_apis.py"

if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import gen_constraints  # noqa: E402  （复用 T01 的锁解析内核）
from gen_constraints import LockError, load_lock, resolve_packages, validate_lock  # noqa: E402

#: 禁用名单里必须存在的核心厂商解算接口（T01 已定，T04 复核，防止名单被削减）。
REQUIRED_FORBIDDEN_APIS = (
    "rm_movej",
    "rm_movep",
    "rm_movec",
    "rm_moves",
    "rm_algo_inverse_kinematics",
)

SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
CONSTRAINT_PIN_RE = re.compile(r"^(?P<name>[A-Za-z0-9._-]+)==(?P<version>[^\s]+)")


class VerifyError(RuntimeError):
    """运行期错误（锁文件不可读 / PackageMap 构建失败等）。"""


# --------------------------------------------------------------------------- #
# profile 选择
# --------------------------------------------------------------------------- #
def current_python_tag() -> str:
    """当前解释器的 ``major.minor``（如 ``"3.10"``）。"""
    return f"{sys.version_info.major}.{sys.version_info.minor}"


def select_profile(lock: Dict[str, Any], python_version: Optional[str] = None) -> Tuple[str, Dict[str, Any]]:
    """按当前（或指定的）解释器版本选择 profile 段。

    返回 ``(profile_key, profile_block)``；解释器版本不在锁里声明的两个档位中时抛
    :class:`VerifyError`（显式失败，绝不静默退回某一档）。
    """
    version = python_version or current_python_tag()
    for key in ("baseline", "fallback"):
        block = lock.get(key)
        if not isinstance(block, dict):
            continue
        if str(block.get("python")) == version:
            return key, block
    known = ", ".join(
        f"{key}={lock[key].get('python')}" for key in ("baseline", "fallback") if key in lock
    )
    raise VerifyError(
        f"当前解释器 Python {version} 不在 versions.lock 声明的档位中（{known}）。"
        "请改用受支持的解释器，或在 versions.lock 中新增对应档位。"
    )


# --------------------------------------------------------------------------- #
# 锁文件自洽性（--lock-only）
# --------------------------------------------------------------------------- #
def parse_constraint_pins(path: Path) -> Dict[str, str]:
    """解析 constraints 文件里的 ``name==version`` 钉版（忽略注释与 --hash）。"""
    pins: Dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        match = CONSTRAINT_PIN_RE.match(line)
        if not match:
            continue
        pins[match.group("name").lower()] = match.group("version")
    return pins


def check_lock_only(
    lock: Dict[str, Any],
    constraints_dir: Path = DEFAULT_CONSTRAINTS_DIR,
) -> List[str]:
    """校验 versions.lock 自洽性，返回问题列表（空列表 = 自洽）。"""
    problems: List[str] = []
    try:
        validate_lock(lock)
    except LockError as exc:
        return [str(exc)]

    # constraints 与 lock 的钉版必须一致（否则 pip 装出来的栈与锁不符）
    for key in ("baseline", "fallback"):
        block = lock.get(key)
        if not isinstance(block, dict):
            continue
        target = constraints_dir / f"{block.get('profile', key)}.txt"
        if not target.is_file():
            problems.append(f"缺少约束文件: {target}")
            continue
        pins = parse_constraint_pins(target)
        try:
            expected = resolve_packages(lock, key)
        except LockError as exc:
            problems.append(str(exc))
            continue
        for name, version in expected.items():
            pinned = pins.get(name.lower())
            if pinned is None:
                problems.append(f"{target.name} 未钉住 {name}（lock 声明 {version}）")
            elif pinned != version:
                problems.append(
                    f"{target.name} 中 {name} 钉版 {pinned} 与 lock 声明 {version} 不一致"
                )

    # wheels 映射格式
    wheels = lock.get("wheels")
    if isinstance(wheels, dict):
        for name, digest in wheels.items():
            if not SHA256_RE.match(str(digest)):
                problems.append(f"versions.lock:wheels 中 {name} 的 sha256 格式非法: {digest}")

    # 上游 commit 必须是 40 位 hex（validate_lock 已覆盖，这里补一条可读性更好的信息）
    upstream = lock.get("upstream")
    if isinstance(upstream, dict):
        for name, spec in upstream.items():
            commit = str((spec or {}).get("commit", ""))
            if len(commit) != 40 or not all(c in "0123456789abcdef" for c in commit):
                problems.append(f"versions.lock:upstream.{name}.commit 不是 40 位 hex: {commit!r}")

    return problems


# --------------------------------------------------------------------------- #
# 实装版本对照（--check）
# --------------------------------------------------------------------------- #
def installed_version(distribution: str) -> Optional[str]:
    """取已安装发行包的版本；未安装返回 ``None``。"""
    try:
        return importlib_metadata.version(distribution)
    except importlib_metadata.PackageNotFoundError:
        return None
    except Exception:  # pragma: no cover - 平台相关异常
        return None


def check_installed(
    lock: Dict[str, Any],
    profile_key: str,
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """逐包比对声明版本与实装版本。

    返回 ``(rows, problems)``：``rows`` 是对照表数据，``problems`` 是问题描述。
    """
    rows: List[Dict[str, Any]] = []
    problems: List[str] = []
    try:
        expected = resolve_packages(lock, profile_key)
    except LockError as exc:
        return ([], [str(exc)])

    for name in sorted(expected):
        declared = str(expected[name])
        actual = installed_version(name)
        if actual is None:
            status = "MISSING"
        elif actual == declared:
            status = "OK"
        else:
            status = "MISMATCH"
        rows.append(
            {"package": name, "declared": declared, "installed": actual, "status": status}
        )
        if status == "MISSING":
            problems.append(f"{name}: 未安装（lock 声明 {declared}）")
        elif status == "MISMATCH":
            problems.append(f"{name}: 实装 {actual} ≠ 声明 {declared}")
    return rows, problems


# --------------------------------------------------------------------------- #
# 禁用名单（--forbidden）
# --------------------------------------------------------------------------- #
def check_forbidden(
    lock: Dict[str, Any],
    registry_path: Path = DEFAULT_REGISTRY,
    scanner_path: Path = SCANNER,
) -> List[str]:
    """校验禁用名单的唯一真相源状态，返回问题列表。"""
    problems: List[str] = []
    forbidden = lock.get("forbidden_apis")
    allowed = lock.get("allowed_apis")

    if not isinstance(forbidden, list) or not forbidden:
        problems.append("versions.lock:forbidden_apis 缺失或为空")
        forbidden = []
    if not isinstance(allowed, list) or not allowed:
        problems.append("versions.lock:allowed_apis 缺失或为空")
        allowed = []

    missing = [name for name in REQUIRED_FORBIDDEN_APIS if name not in forbidden]
    if missing:
        problems.append("versions.lock:forbidden_apis 缺少核心禁用接口: " + ", ".join(missing))

    overlap = sorted(set(forbidden) & set(allowed))
    if overlap:
        problems.append("同一接口同时出现在 forbidden_apis 与 allowed_apis: " + ", ".join(overlap))

    # 许可登记表不得自带第二份禁用名单（若存在则必须与锁一致）
    if registry_path.is_file():
        try:
            registry = yaml_load(registry_path)
        except Exception as exc:
            problems.append(f"许可登记表解析失败: {registry_path}（{exc}）")
            registry = {}
        if isinstance(registry, dict) and "forbidden_apis" in registry:
            if list(registry["forbidden_apis"]) != list(forbidden):
                problems.append(
                    "licenses/registry.yaml:forbidden_apis 与 versions.lock 不一致（名单只能有一份）"
                )

    # 扫描脚本若已存在，必须以锁为名单来源（不得硬编码第二份名单）
    if scanner_path.is_file():
        text = scanner_path.read_text(encoding="utf-8")
        if "forbidden_apis" not in text:
            problems.append(
                f"{scanner_path.name} 未引用 versions.lock:forbidden_apis（名单来源必须是锁文件）"
            )
        hardcoded = [name for name in REQUIRED_FORBIDDEN_APIS if name in text]
        if hardcoded:
            problems.append(
                f"{scanner_path.name} 疑似硬编码禁用名单: " + ", ".join(hardcoded)
            )
    return problems


def yaml_load(path: Path) -> Any:
    """读取 YAML（延迟导入 PyYAML，缺失时给出明确错误）。"""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - 环境缺依赖
        raise VerifyError("需要 PyYAML（pip install pyyaml）") from exc
    return yaml.safe_load(path.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------- #
# 归档校验（--wheels）
# --------------------------------------------------------------------------- #
def check_wheels(wheels_dir: Path = DEFAULT_WHEELS_DIR) -> Tuple[str, List[str]]:
    """校验 vendor/wheels 归档。

    返回 ``(status, problems)``，status ∈ {"PASS", "FAIL", "SKIP"}。
    归档目录不存在 → ``SKIP`` + 明确提示（本机镜像不入 git，属正常状态）。
    """
    if not wheels_dir.is_dir():
        return (
            "SKIP",
            [
                f"本机无离线归档 {wheels_dir}（vendor/ 不入 git）："
                "重建前请先执行 bash tools/lock_deps.sh --download"
            ],
        )
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    try:
        from tools import wheels_lock  # noqa: WPS433（延迟导入）
    except Exception as exc:  # pragma: no cover - 导入失败
        return ("FAIL", [f"无法导入 tools.wheels_lock: {exc}"])
    try:
        problems = wheels_lock.verify_archive(wheels_dir)
    except Exception as exc:
        return ("FAIL", [f"归档校验失败: {exc}"])
    if problems:
        return ("FAIL", problems)
    return ("PASS", [])


# --------------------------------------------------------------------------- #
# PackageMap 冒烟（--package-map）
# --------------------------------------------------------------------------- #
def check_package_map() -> Tuple[str, List[str]]:
    """PackageMap 解析冒烟，返回 ``(status, problems)``。"""
    src_dir = REPO_ROOT / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))
    try:
        from rm75_drake import package_map as pm
    except Exception as exc:
        return ("FAIL", [f"无法导入 rm75_drake.package_map: {exc}"])
    try:
        pkg_map = pm.build_package_map()
        names = pm.package_names(pkg_map)
        if "rm75_drake" not in names:
            return ("FAIL", [f"PackageMap 中没有 rm75_drake 包（已注册: {names}）"])
        url = "package://rm75_drake/placeholder.urdf"
        resolved = pm.resolve_package_url(pkg_map, url)
    except Exception as exc:
        return ("FAIL", [f"PackageMap 解析失败: {exc}"])
    return ("PASS", [f"package://rm75_drake/placeholder.urdf → {resolved}"])


# --------------------------------------------------------------------------- #
# 模型自检（--model）
# --------------------------------------------------------------------------- #
#: 零位末端高度的容差（m）——官方 URDF 的 rpy=1.5708 舍入引入的残差为 ~2.24e-6 m，
#: 故高度用 1e-6、横向用 1e-5。
TIP_HEIGHT_TOL_M = 1e-6
TIP_LATERAL_TOL_M = 1e-5
MASS_TOL_KG = 1e-6
LIMIT_TOL = 1e-6


def _load_model_facts() -> Dict[str, Any]:
    """加载改造版 RM75-B 并提取自检事实。

    失败语义：Drake 不可用 → :class:`ImportError`（环境错误，退出码 2）；
    模型本身有问题 → 其它异常（检查失败，退出码 1）。
    """
    src_dir = REPO_ROOT / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))
    from rm75_drake import models as m
    from rm75_drake.kinematics import mdh

    plant, _model_index, _parser = m.load_rm75()
    plant.Finalize()
    context = plant.CreateDefaultContext()
    pose = m.eval_link_pose_in_world(plant, context, m.RM75_TIP_LINK)
    translation = pose.translation()
    return {
        "dof": int(plant.num_positions()),
        "expected_dof": int(m.RM75_DOF),
        "total_mass_kg": float(plant.CalcTotalMass(context)),
        "expected_total_mass_kg": float(m.RM75_TOTAL_MASS_KG),
        "tip_position_m": [float(v) for v in translation],
        "expected_tip_height_m": float(mdh.MDH_ZERO_HEIGHT_M),
        "joint_limits": m.joint_limits_from_plant(plant),
        "urdf_limits": m.joint_limits_from_urdf(),
    }


def check_model(
    limits_path: Optional[Path] = None,
    loader: Optional[Any] = None,
) -> Tuple[str, List[str], Dict[str, Any]]:
    """模型自检，返回 ``(status, problems, summary)``。

    status ∈ {"PASS", "FAIL", "ERROR"}：

    * ``PASS``：DoF / 总质量 / 零位末端高度 / 7 条限位 / 限位交接件全部一致；
    * ``FAIL``：模型加载成功但数值或交接件不一致；
    * ``ERROR``：Drake 不可用等**环境**问题（调用方应返回退出码 2）。

    ``loader`` 可注入（测试用）：默认 :func:`_load_model_facts`。
    """
    src_dir = REPO_ROOT / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))

    try:
        facts = (loader or _load_model_facts)()
    except ImportError as exc:
        return ("ERROR", [f"模型自检需要 Drake（pydrake）不可用：{exc}"], {})
    except Exception as exc:  # 模型加载失败 = 检查失败
        return ("FAIL", [f"模型加载/自检失败：{exc}"], {})

    problems: List[str] = []

    dof = facts.get("dof")
    expected_dof = facts.get("expected_dof")
    if dof != expected_dof:
        problems.append(f"DoF={dof}，期望 {expected_dof}")

    mass = facts.get("total_mass_kg")
    expected_mass = facts.get("expected_total_mass_kg")
    if mass is None or expected_mass is None or abs(mass - expected_mass) > MASS_TOL_KG:
        problems.append(f"总质量={mass} kg，期望 {expected_mass} kg（容差 {MASS_TOL_KG}）")

    tip = facts.get("tip_position_m") or [0.0, 0.0, 0.0]
    expected_height = facts.get("expected_tip_height_m")
    if abs(tip[2] - expected_height) > TIP_HEIGHT_TOL_M:
        problems.append(
            f"零位末端高度={tip[2]} m，期望 {expected_height} m（容差 {TIP_HEIGHT_TOL_M}）"
        )
    if abs(tip[0]) > TIP_LATERAL_TOL_M:
        problems.append(f"零位末端 x={tip[0]} m 偏离 0 超过 {TIP_LATERAL_TOL_M}")
    if abs(tip[1]) > TIP_LATERAL_TOL_M:
        problems.append(f"零位末端 y={tip[1]} m 偏离 0 超过 {TIP_LATERAL_TOL_M}")

    urdf_by_name = {str(item["name"]): item for item in facts.get("urdf_limits") or ()}
    limits_rows = []
    for item in facts.get("joint_limits") or ():
        name = str(item.get("name"))
        urdf_item = urdf_by_name.get(name)
        if urdf_item is None:
            problems.append(f"Drake 解析出关节 {name}，但 URDF 里没有对应关节")
            continue
        for key, urdf_key in (
            ("lower", "lower"),
            ("upper", "upper"),
            ("velocity", "velocity"),
        ):
            if abs(float(item[key]) - float(urdf_item[urdf_key])) > LIMIT_TOL:
                problems.append(
                    f"{name}.{key}：Drake 解析 {item[key]} ≠ URDF 声明 {urdf_item[urdf_key]}"
                )
        limits_rows.append(
            {
                "name": name,
                "lower": float(item["lower"]),
                "upper": float(item["upper"]),
                "velocity": float(item["velocity"]),
                "effort": float(urdf_item.get("effort", 0.0)),
            }
        )
    if len(limits_rows) != 7:
        problems.append(f"关节限位摘要应有 7 条，实际 {len(limits_rows)} 条")

    # 限位交接件（models/rm75_drake/joint_limits.yaml）与 URDF 的一致性
    limits_summary: Dict[str, Any] = {}
    try:
        from rm75_drake import limits as limits_mod

        limits_data = limits_mod.load_joint_limits(limits_path)
        limit_problems = limits_mod.check_joint_limits(
            limits_data, urdf_limits=facts.get("urdf_limits")
        )
        problems.extend(limit_problems)
        limits_summary = {
            "path": str(limits_mod.joint_limits_path(limits_path)),
            "software_margin_rad": limits_data.get("software_limits", {}).get("margin_rad"),
        }
    except Exception as exc:
        problems.append(f"限位交接件校验失败：{exc}")

    summary = {
        "dof": dof,
        "total_mass_kg": mass,
        "tip_position_m": tip,
        "expected_tip_height_m": expected_height,
        "joint_limits": limits_rows,
        "joint_limits_handoff": limits_summary,
    }
    return ("FAIL" if problems else "PASS", problems, summary)


# --------------------------------------------------------------------------- #
# 规划链路自检（--plan）
# --------------------------------------------------------------------------- #
#: 自检用的起点位形（抬肘名义姿态，碰撞自由）。
PLAN_START_Q = (0.0, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)
#: 自检用的目标位形（沿 joint_1 偏移 0.2 rad，直线段碰撞自由）。
PLAN_TARGET_Q = (0.2, -0.6, 0.0, 0.9, 0.0, 0.5, 0.0)


def check_plan(
    scenario_path: Optional[Path] = None,
) -> Tuple[str, List[str], Dict[str, Any]]:
    """规划链路自检，返回 ``(status, problems, summary)``。

    流程：**装配场景 → 一次 IK → 一段短规划 → 一次碰撞复检**。

    status ∈ {"PASS", "FAIL", "ERROR"}：

    * ``PASS``：四步全部成功，且碰撞复检逃逸数 = 0；
    * ``FAIL``：链路跑通但结果不满足（IK 失败 / 规划失败 / 有逃逸）；
    * ``ERROR``：Drake 不可用等**环境**问题（调用方应返回退出码 2）。
    """
    src_dir = REPO_ROOT / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))
    try:
        import numpy as np
        from pydrake.math import RigidTransform

        from rm75_drake.planner import Planner, PlanOptions
        from rm75_drake.planning.gcs import collision_recheck
        from rm75_drake.planning.ik import solve_ik
        from rm75_drake.planning.scene import build_scene
        from rm75_drake.scenario import load_scenario
    except ImportError as exc:
        return ("ERROR", [f"规划自检需要 Drake（pydrake）不可用：{exc}"], {})

    scenario_file = scenario_path or (REPO_ROOT / "scenarios" / "single_arm.yaml")
    try:
        scenario = load_scenario(scenario_file)
        scene = build_scene(scenario)
    except Exception as exc:
        return ("FAIL", [f"场景装配失败：{exc}"], {})

    problems: List[str] = []
    summary: Dict[str, Any] = {"scenario": str(scenario_file)}

    # ── 一次 IK（目标 = 起点位形的 FK 位姿）───────────────────────────
    try:
        context = scene.plant.CreateDefaultContext()
        scene.plant.SetPositions(context, np.asarray(PLAN_START_Q, dtype=float))
        pose = scene.plant.GetFrameByName("link_7").CalcPoseInWorld(context)
        target = RigidTransform(pose.rotation(), pose.translation())
        ik_result = solve_ik(scene, target)
        summary["ik_success"] = bool(ik_result.success)
        summary["ik_reason"] = ik_result.reason
        summary["ik_message"] = ik_result.message
        if not ik_result.success:
            problems.append(f"IK 失败（{ik_result.reason}）：{ik_result.message}")
    except Exception as exc:
        problems.append(f"IK 自检异常：{exc}")

    # ── 一段短规划 + 一次碰撞复检 ──────────────────────────────────────
    try:
        planner = Planner(
            scenario,
            scene=scene,
            options=PlanOptions(
                start_configuration=PLAN_START_Q,
                region_spacing=0.5,
                region_options={"num_collision_infeasible_samples": 3},
            ),
        )
        result = planner.plan_joint_target(PLAN_TARGET_Q)
        summary["plan_success"] = bool(result.success)
        summary["plan_message"] = result.message
        summary["stage_durations_s"] = {
            stage: (result.diagnostics.get(stage) or {}).get("duration_s")
            for stage in ("ik", "regions", "gcs", "retime", "collision")
        }
        if not result.success:
            problems.append(f"规划失败（{result.stage}/{result.reason}）：{result.message}")
        else:
            trajectory = result.trajectory
            recheck = collision_recheck(scene, trajectory.q_matrix())
            summary["num_samples"] = int(recheck.num_samples)
            summary["num_escapes"] = int(recheck.num_escapes)
            if recheck.num_escapes != 0:
                problems.append(f"碰撞复检发现 {recheck.num_escapes} 处逃逸")
    except Exception as exc:
        problems.append(f"规划自检异常：{exc}")

    return ("FAIL" if problems else "PASS", problems, summary)


# --------------------------------------------------------------------------- #
# 输出
# --------------------------------------------------------------------------- #
def _table(rows: Sequence[Dict[str, Any]]) -> str:
    header = ("package", "declared", "installed", "status")
    widths = [len(h) for h in header]
    for row in rows:
        for index, key in enumerate(header):
            widths[index] = max(widths[index], len(str(row.get(key, ""))))
    lines = ["  ".join(h.ljust(widths[i]) for i, h in enumerate(header))]
    lines.append("  ".join("-" * widths[i] for i in range(len(header))))
    for row in rows:
        lines.append(
            "  ".join(str(row.get(key, "")).ljust(widths[i]) for i, key in enumerate(header))
        )
    return "\n".join(lines)


def _print_model_summary(summary: Dict[str, Any]) -> None:
    """打印模型自检摘要（人类可读）。"""
    if not summary:
        return
    print(
        f"  DoF={summary.get('dof')}  总质量={summary.get('total_mass_kg')} kg  "
        f"零位末端={[round(v, 9) for v in summary.get('tip_position_m', [])]} m  "
        f"期望高度={summary.get('expected_tip_height_m')} m"
    )
    rows = summary.get("joint_limits") or []
    if rows:
        header = f"  {'joint':<10}{'lower(rad)':>14}{'upper(rad)':>16}{'vel(rad/s)':>14}{'effort(N·m)':>14}"
        print(header)
        print("  " + "-" * (len(header) - 2))
        for row in rows:
            print(
                f"  {row['name']:<10}{row['lower']:>14.7f}{row['upper']:>16.7f}"
                f"{row['velocity']:>14.4f}{row['effort']:>14.1f}"
            )
    handoff = summary.get("joint_limits_handoff") or {}
    if handoff:
        print(
            f"  限位交接件：{handoff.get('path')}（软限位收紧 {handoff.get('software_margin_rad')} rad）"
        )


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="环境自检：versions.lock 声明 vs 实装 reality"
    )
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK, help="锁文件路径")
    parser.add_argument("--constraints-dir", type=Path, default=DEFAULT_CONSTRAINTS_DIR)
    parser.add_argument("--wheels-dir", type=Path, default=DEFAULT_WHEELS_DIR)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--profile", default="auto", choices=("auto", "baseline", "fallback"))
    parser.add_argument("--check", action="store_true", help="逐包比对实装版本")
    parser.add_argument("--lock-only", action="store_true", help="只校验锁文件自洽性")
    parser.add_argument("--wheels", action="store_true", help="校验 vendor/wheels 归档")
    parser.add_argument("--forbidden", action="store_true", help="校验禁用名单")
    parser.add_argument("--package-map", action="store_true", help="PackageMap 解析冒烟")
    parser.add_argument(
        "--model", action="store_true", help="改造版 RM75-B 模型自检（DoF/质量/零位高度/限位）"
    )
    parser.add_argument(
        "--plan",
        action="store_true",
        help="规划链路自检（装配场景 + 一次 IK + 一段短规划 + 一次碰撞复检）",
    )
    parser.add_argument(
        "--joint-limits",
        type=Path,
        default=None,
        help="限位交接件路径（默认 models/rm75_drake/joint_limits.yaml）",
    )
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结果")
    args = parser.parse_args(list(argv) if argv is not None else None)

    selected = [
        args.check,
        args.lock_only,
        args.wheels,
        args.forbidden,
        args.package_map,
        args.model,
        args.plan,
    ]
    if not any(selected):
        # 未指定子检查时全部执行（--model 需要 Drake，故不在默认集合里）
        args.check = args.lock_only = args.wheels = args.forbidden = args.package_map = True
        if not args.json:
            print("（--model 需显式指定：它依赖 Drake；CI 由 scripts/ci.sh fast 档调用）")

    report: Dict[str, Any] = {"checks": {}, "ok": True}
    env_error = False

    def emit(
        name: str,
        status: str,
        problems: Sequence[str],
        detail: str = "",
        data: Optional[Dict[str, Any]] = None,
    ) -> None:
        entry: Dict[str, Any] = {"status": status, "problems": list(problems)}
        if data is not None:
            entry["summary"] = data
        report["checks"][name] = entry
        if status == "FAIL":
            report["ok"] = False
        if args.json:
            return
        print(f"[{status}] {name}" + (f" — {detail}" if detail else ""))
        for problem in problems:
            print(f"    - {problem}")

    # ── 锁文件 ────────────────────────────────────────────────────────────
    lock: Optional[Dict[str, Any]] = None
    if args.lock_only or args.check or args.forbidden:
        try:
            lock = load_lock(args.lock)
        except LockError as exc:
            print(f"verify_env: 锁文件不可用: {exc}", file=sys.stderr)
            return 2

    if args.lock_only:
        problems = check_lock_only(lock, args.constraints_dir)
        emit(
            "lock-only",
            "PASS" if not problems else "FAIL",
            problems,
            f"锁文件自洽性（{args.lock}）",
        )
        if not args.json and not problems:
            print("    锁文件结构、constraints 钉版、wheels 映射均一致")

    if args.check:
        try:
            if args.profile == "auto":
                profile_key, block = select_profile(lock)
            else:
                profile_key = args.profile
                block = lock[profile_key]
        except (VerifyError, KeyError) as exc:
            print(f"verify_env: profile 选择失败: {exc}", file=sys.stderr)
            return 2
        rows, problems = check_installed(lock, profile_key)
        detail = (
            f"profile={profile_key}（python {block.get('python')}，"
            f"drake {block.get('drake')}，supported={bool(block.get('supported', False))}）"
        )
        if not args.json:
            print(f"profile: {profile_key}（解释器 Python {current_python_tag()}）")
            print(_table(rows))
        emit("check", "PASS" if not problems else "FAIL", problems, detail)

    if args.wheels:
        status, problems = check_wheels(args.wheels_dir)
        emit("wheels", status, problems, f"归档校验（{args.wheels_dir}）")

    if args.forbidden:
        problems = check_forbidden(lock, args.registry, SCANNER)
        emit(
            "forbidden",
            "PASS" if not problems else "FAIL",
            problems,
            "禁用名单唯一真相源校验",
        )

    if args.package_map:
        status, problems = check_package_map()
        emit("package-map", status, problems, "PackageMap 解析冒烟")

    if args.model:
        status, problems, summary = check_model(args.joint_limits)
        if not args.json:
            _print_model_summary(summary)
        emit(
            "model",
            status,
            problems,
            "改造版 RM75-B 模型自检（DoF / 总质量 / 零位末端高度 / 7 条限位）",
            data=summary,
        )
        if status == "ERROR":
            env_error = True

    if args.plan:
        status, problems, summary = check_plan()
        emit(
            "plan",
            status,
            problems,
            "规划链路自检（场景装配 + IK + 短规划 + 碰撞复检）",
            data=summary,
        )
        if status == "ERROR":
            env_error = True

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))

    if env_error:
        print("verify_env: 模型自检环境不可用（Drake 缺失），属环境错误（退出码 2）", file=sys.stderr)
        return 2
    if not report["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
