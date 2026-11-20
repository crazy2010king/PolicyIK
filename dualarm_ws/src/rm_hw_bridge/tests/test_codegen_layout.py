"""rm_hw_bridge ABI 真相校验测试。

覆盖四轨：
  (a) 永远可跑：运行期 ctypes 布局 vs ``layout_golden.json`` **逐字段**比对；
  (b) 黄金数值表：与任务书给定（gcc 实测）的数值表逐项比对；
  (c) 可选增强：找到 RM-API2 头文件时用 gcc 编译 C 参考程序打印
      ``sizeof`` / ``offsetof`` 与黄金表比对；找不到头文件或 gcc 时
      **显式 skip 并写明原因**（绝不静默通过）；
  (d) 生成幂等：``codegen --check`` 无差异。

外加负例（见 ``## 负例`` 注释块）：畸形规格、漂移检测、符号缺失必须报错。
"""
from __future__ import annotations

import ctypes
import json
import os
import shutil
import subprocess
import sys

import pytest

import rm_hw_bridge.codegen as codegen

# ---------------------------------------------------------------------------
# (b) 黄金数值表：任务书给定的 gcc 实测值（RM-API2 1.1.7, x86-64 SysV）
#     结构体名 → {"size": int, "offsets": {字段: 偏移}}
# ---------------------------------------------------------------------------
GOLDEN_NUMBERS = {
    "rm_robot_handle": {"size": 4, "offsets": {}},
    "rm_arm_all_state_t": {
        "size": 240,
        "offsets": {
            "joint_current": 0,
            "joint_en_flag": 28,
            "joint_temperature": 56,
            "joint_voltage": 84,
            "joint_err_code": 112,
            "err": 140,
        },
    },
    "rm_current_arm_state_t": {
        "size": 168,
        "offsets": {"pose": 0, "joint": 40, "err": 68},
    },
    "rm_realtime_push_config_t": {
        "size": 76,
        "offsets": {
            "cycle": 0,
            "enable": 4,
            "port": 8,
            "ip": 16,
            "custom_config": 44,
        },
    },
    "rm_movej_canfd_mode_t": {
        "size": 24,
        "offsets": {
            "joint": 0,
            "expand": 8,
            "follow": 12,
            "trajectory_mode": 16,
            "radio": 20,
        },
    },
    "rm_joint_status_t": {
        "size": 164,
        "offsets": {"joint_current": 0, "joint_position": 52},
    },
    "rm_realtime_arm_joint_state_t": {
        "size": 1368,
        "offsets": {"joint_status": 24, "arm_current_status": 524},
    },
    "rm_err_t": {"size": 100, "offsets": {}},
    "rm_pose_t": {"size": 40, "offsets": {}},
    "rm_position_t": {"size": 12, "offsets": {}},
    "rm_quat_t": {"size": 16, "offsets": {}},
    "rm_euler_t": {"size": 12, "offsets": {}},
    "rm_udp_custom_config_t": {"size": 32, "offsets": {}},
    "rm_robot_info_t": {"size": 16, "offsets": {}},
}

#: 22 个核心符号（白名单基线，任务书给定）
CORE_SYMBOLS = (
    "rm_init",
    "rm_create_robot_arm",
    "rm_delete_robot_arm",
    "rm_set_realtime_push",
    "rm_get_joint_degree",
    "rm_get_current_arm_state",
    "rm_get_arm_all_state",
    "rm_get_joint_err_flag",
    "rm_get_arm_current_trajectory",
    "rm_movej",
    "rm_movej_canfd",
    "rm_set_delete_current_trajectory",
    "rm_set_arm_stop",
    "rm_set_arm_slow_stop",
    "rm_set_arm_pause",
    "rm_set_arm_continue",
    "rm_set_joint_drive_max_speed",
    "rm_set_joint_drive_max_acc",
    "rm_set_joint_drive_min_pos",
    "rm_set_joint_drive_max_pos",
    "rm_set_collision_state",
    "rm_set_self_collision_enable",
)

#: 支持层符号（22 个，不计入 22）
SUPPORT_SYMBOLS = (
    "rm_set_log_call_back",
    "rm_set_timeout",
    "rm_get_robot_info",
    "rm_get_arm_software_info",
    "rm_get_realtime_push",
    "rm_realtime_arm_state_call_back",
    # --- 夹爪快路径
    "rm_set_gripper_route",
    "rm_set_gripper_release",
    "rm_set_gripper_pick",
    "rm_set_gripper_pick_on",
    "rm_set_gripper_position",
    "rm_get_gripper_state",
    # --- Modbus-RTU 主路径
    "rm_set_modbus_mode",
    "rm_close_modbus_mode",
    "rm_read_holding_registers",
    "rm_read_input_registers",
    "rm_write_registers",
    "rm_write_single_register",
    # --- RM Plus 末端生态协议
    "rm_set_rm_plus_mode",
    "rm_get_rm_plus_mode",
    # --- 工具端电压
    "rm_set_tool_voltage",
    "rm_get_tool_voltage",
)


# ---------------------------------------------------------------------------
# (a) 运行期 ctypes 布局 vs 黄金表（永远可跑）
# ---------------------------------------------------------------------------


def test_runtime_layout_matches_golden_json(generated_module, layout_golden):
    """运行期 ctypes 的 sizeof / 字段 offset 必须与黄金表逐字段一致。"""
    structs = layout_golden["structs"]
    assert structs, "布局黄金表为空"
    for sname, golden in structs.items():
        cls = getattr(generated_module, sname)
        assert ctypes.sizeof(cls) == golden["size"], "%s size 不一致" % sname
        for fname, offset in golden["fields"].items():
            actual = getattr(cls, fname).offset
            assert actual == offset, "%s.%s offset 不一致：%s != %s" % (
                sname, fname, actual, offset)


def test_golden_json_covers_every_spec_struct(spec, layout_golden):
    """规格里的每个结构体都必须出现在黄金表中（漏一个 = 漏校验）。"""
    assert set(layout_golden["structs"]) == set(spec["structs"])


def test_generated_module_records_spec_hash_and_sdk_version(spec_path, generated_module):
    """生成物头部必须注明源头（abi_spec.yaml 的 sha256 + SDK 版本）。"""
    assert generated_module.SPEC_SHA256 == codegen.sha256_file(spec_path)
    assert generated_module.SDK_VERSION == "1.1.7"
    header = open(
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "rm_hw_bridge", "generated", "rm_api2_ctypes.py",
        ),
        "r", encoding="utf-8",
    ).read()
    assert generated_module.SPEC_SHA256 in header
    assert "RM-API2 1.1.7" in header


# ---------------------------------------------------------------------------
# (b) 黄金数值表（任务书给定）
# ---------------------------------------------------------------------------


def test_golden_numbers_match_task_table(layout_golden):
    """黄金表必须与任务书给定的 gcc 实测数值表逐项一致。"""
    structs = layout_golden["structs"]
    for sname, expected in GOLDEN_NUMBERS.items():
        assert sname in structs, "黄金表缺少结构体 %s" % sname
        actual = structs[sname]
        assert actual["size"] == expected["size"], "%s size 不一致" % sname
        for fname, offset in expected["offsets"].items():
            assert actual["fields"][fname] == offset, "%s.%s offset 不一致" % (
                sname, fname)


def test_generated_layout_matches_golden_numbers_via_runtime(generated_module):
    """运行期 ctypes 也必须复现任务书数值表（双保险，不依赖 JSON）。"""
    for sname, expected in GOLDEN_NUMBERS.items():
        cls = getattr(generated_module, sname)
        assert ctypes.sizeof(cls) == expected["size"], "%s size 不一致" % sname
        for fname, offset in expected["offsets"].items():
            assert getattr(cls, fname).offset == offset


# ---------------------------------------------------------------------------
# (c) C 参考编译校验（可选增强，找不到必须显式 skip）
# ---------------------------------------------------------------------------


def _find_rm_api2_include(repo_root, vendor_include_dir):
    """按任务书给定的候选顺序找 RM-API2 头文件目录。"""
    candidates = []
    env_dir = os.environ.get("RM_API2_INCLUDE")
    if env_dir:
        candidates.append(env_dir)
    candidates.append(os.path.join(repo_root, ".deps", "rm_api2", "include"))
    candidates.append(vendor_include_dir)
    # 机器上其它 checkout 的 SDK 源码目录（尽力而为）
    candidates.extend(
        [
            "/tmp/rmrepo/rm_driver/include/rm_driver",
            "/tmp/rmros/rm_driver/include/rm_driver",
            "/tmp/rc/ros2_rm_robot-humble/rm_driver/include/rm_driver",
        ]
    )
    for path in candidates:
        if path and os.path.isfile(os.path.join(path, "rm_interface.h")):
            return path
    return None


def _write_c_probe(spec, path):
    """按规格生成 C 参考程序（打印 sizeof / offsetof）。"""
    lines = [
        "#include <stdio.h>",
        "#include <stddef.h>",
        '#include "rm_interface.h"',
        "int main(void) {",
    ]
    for sname in codegen.struct_order(spec):
        lines.append('  printf("SIZE %s %%zu\\n", sizeof(%s));' % (sname, sname))
        for field in spec["structs"][sname].get("fields") or []:
            lines.append(
                '  printf("OFF %s.%s %%zu\\n", offsetof(%s, %s));'
                % (sname, field["name"], sname, field["name"])
            )
    lines.append("  return 0;")
    lines.append("}")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def test_c_reference_layout_matches_golden(spec, layout_golden, tmp_path,
                                           repo_root, vendor_include_dir):
    """用 gcc 编译 C 参考程序，把 sizeof/offsetof 与黄金表比对。

    找不到头文件或没有 gcc 时**显式 skip 并写明原因**，绝不静默通过。
    """
    include_dir = _find_rm_api2_include(repo_root, vendor_include_dir)
    if include_dir is None:
        pytest.skip(
            "未找到 RM-API2 头文件（候选顺序：RM_API2_INCLUDE → .deps/rm_api2/include → "
            "vendor/rm_api2/include → 机器上其它 SDK checkout）；C 参考编译校验已跳过"
        )
    compiler = shutil.which("gcc") or shutil.which("cc")
    if compiler is None:
        pytest.skip("未找到 gcc/cc，C 参考编译校验已跳过（include=%s）" % include_dir)

    src = os.path.join(str(tmp_path), "abi_probe.c")
    binary = os.path.join(str(tmp_path), "abi_probe")
    _write_c_probe(spec, src)
    compile_proc = subprocess.run(
        [compiler, "-I", include_dir, "-o", binary, src],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    assert compile_proc.returncode == 0, "C 参考编译失败：\n%s" % compile_proc.stdout
    run_proc = subprocess.run([binary], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True)
    assert run_proc.returncode == 0, "C 参考程序运行失败：\n%s" % run_proc.stdout

    sizes = {}
    offsets = {}
    for line in run_proc.stdout.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        kind, key, value = parts
        if kind == "SIZE":
            sizes[key] = int(value)
        elif kind == "OFF":
            offsets[key] = int(value)

    golden = layout_golden["structs"]
    assert set(sizes) == set(golden), "C 参考程序与黄金表覆盖的结构体不一致"
    for sname, size in sizes.items():
        assert size == golden[sname]["size"], "%s size 与 C 参考不一致" % sname
    for key, offset in offsets.items():
        sname, fname = key.split(".", 1)
        assert offset == golden[sname]["fields"][fname], "%s 与 C 参考不一致" % key


# ---------------------------------------------------------------------------
# (d) 生成幂等
# ---------------------------------------------------------------------------


def test_codegen_check_is_idempotent(spec_path, pkg_root):
    """`codegen --check` 必须无差异（生成幂等）。"""
    out_dir = os.path.join(pkg_root, "rm_hw_bridge", "generated")
    proc = subprocess.run(
        [sys.executable, "-m", "rm_hw_bridge.codegen", "--spec", spec_path,
         "--out", out_dir, "--check"],
        cwd=pkg_root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=dict(os.environ, PYTHONPATH=pkg_root),
    )
    assert proc.returncode == 0, "codegen --check 失败：\n%s" % proc.stdout


def test_codegen_regenerate_is_byte_identical(spec_path, pkg_root, tmp_path):
    """重新生成到临时目录必须与磁盘上的生成物逐字节一致。"""
    out_dir = os.path.join(str(tmp_path), "generated")
    proc = subprocess.run(
        [sys.executable, "-m", "rm_hw_bridge.codegen", "--spec", spec_path,
         "--out", out_dir],
        cwd=pkg_root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=dict(os.environ, PYTHONPATH=pkg_root),
    )
    assert proc.returncode == 0, "codegen 生成失败：\n%s" % proc.stdout
    committed = os.path.join(pkg_root, "rm_hw_bridge", "generated")
    for name in ("rm_api2_ctypes.py", "layout_golden.json"):
        with open(os.path.join(committed, name), "rb") as fh:
            want = fh.read()
        with open(os.path.join(out_dir, name), "rb") as fh:
            got = fh.read()
        assert got == want, "%s 与仓库内生成物不一致" % name


# ---------------------------------------------------------------------------
# 符号表
# ---------------------------------------------------------------------------


def test_symbol_table_contains_core_and_support_symbols(generated_module):
    """22 核心符号 + 22 支持符号必须全部在符号表里。"""
    symbols = generated_module.SYMBOLS
    for name in CORE_SYMBOLS:
        assert name in symbols, "符号表缺少核心符号 %s" % name
    for name in SUPPORT_SYMBOLS:
        assert name in symbols, "符号表缺少支持层符号 %s" % name
    assert len(CORE_SYMBOLS) == 22


def test_by_value_structs_are_passed_by_value(generated_module):
    """rm_set_realtime_push / rm_movej_canfd 的结构体必须按值传递。"""
    for name in ("rm_set_realtime_push", "rm_movej_canfd"):
        sig = generated_module.SYMBOLS[name]
        assert sig["by_value"], "%s 未标记按值传参" % name
    # 按值传参 ⇒ argtypes 里是结构体类本身，不是指针
    assert generated_module.SYMBOLS["rm_set_realtime_push"]["argtypes"][-1] is \
        generated_module.rm_realtime_push_config_t
    assert generated_module.SYMBOLS["rm_movej_canfd"]["argtypes"][-1] is \
        generated_module.rm_movej_canfd_mode_t


def test_handle_based_api_shapes(generated_module):
    """handle-based ABI：create 返回指针，其余接口第一参是句柄指针。"""
    symbols = generated_module.SYMBOLS
    assert symbols["rm_create_robot_arm"]["restype"] is \
        ctypes.POINTER(generated_module.rm_robot_handle)
    assert symbols["rm_get_joint_degree"]["argtypes"][0] is \
        ctypes.POINTER(generated_module.rm_robot_handle)
    assert symbols["rm_get_joint_degree"]["argtypes"][1] is \
        ctypes.POINTER(ctypes.c_float)
    # 关节角是 float*，不是 64 位指针被截断成 int
    assert symbols["rm_movej"]["argtypes"][1] is ctypes.POINTER(ctypes.c_float)
    assert symbols["rm_movej_canfd"]["argtypes"][-1] is \
        generated_module.rm_movej_canfd_mode_t
    assert generated_module.rm_movej_canfd_mode_t.joint is not None


def test_ip_field_is_char_array(generated_module):
    """rm_realtime_push_config_t.ip 是 char[28]（bytes 赋值）。"""
    ip_field = dict(generated_module.rm_realtime_push_config_t._fields_)["ip"]
    assert ctypes.sizeof(ip_field) == 28
    assert ctypes.sizeof(generated_module.rm_realtime_push_config_t) == 76


# ---------------------------------------------------------------------------
# 负例（Negative tests）
# ---------------------------------------------------------------------------


def test_negative_unknown_type_is_rejected(spec):
    """畸形规格：未知类型必须抛 SpecError（不能静默生成错布局）。"""
    broken = json.loads(json.dumps(spec))
    broken["structs"]["rm_robot_handle"]["fields"][0]["type"] = "rm_not_exist_t"
    with pytest.raises(codegen.SpecError):
        codegen.resolve_type("rm_not_exist_t", broken)
    with pytest.raises(codegen.SpecError):
        codegen.resolve_type("rm_not_exist_t*", broken)


def test_negative_cyclic_struct_dependency_is_rejected():
    """畸形规格：结构体循环依赖必须抛 SpecError。"""
    broken = {
        "structs": {
            "a_t": {"fields": [{"name": "b", "type": "b_t"}]},
            "b_t": {"fields": [{"name": "a", "type": "a_t"}]},
        },
        "functions": {},
    }
    with pytest.raises(codegen.SpecError):
        codegen.struct_order(broken)


def test_negative_empty_struct_is_rejected():
    """畸形规格：没有字段的结构体必须被拒。"""
    broken = {"structs": {"empty_t": {"fields": []}}, "functions": {}}
    with pytest.raises(codegen.SpecError):
        codegen.render_ctypes(broken, "deadbeef")


def test_negative_check_detects_drift(spec_path, pkg_root, tmp_path):
    """负例：磁盘上的生成物被篡改时，`codegen --check` 必须非 0 退出。"""
    src_dir = os.path.join(pkg_root, "rm_hw_bridge", "generated")
    out_dir = os.path.join(str(tmp_path), "generated")
    shutil.copytree(src_dir, out_dir)
    with open(os.path.join(out_dir, "rm_api2_ctypes.py"), "a", encoding="utf-8") as fh:
        fh.write("\n# tampered\n")
    proc = subprocess.run(
        [sys.executable, "-m", "rm_hw_bridge.codegen", "--spec", spec_path,
         "--out", out_dir, "--check"],
        cwd=pkg_root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env=dict(os.environ, PYTHONPATH=pkg_root),
    )
    assert proc.returncode != 0, "篡改后 --check 竟然通过了"
    assert "漂移" in proc.stdout or "不一致" in proc.stdout


def test_negative_layout_mismatch_is_detected(spec, layout_golden):
    """负例：规格写错（例如 err 数组长度写成 23）时，布局必须与黄金表不一致。"""
    broken = json.loads(json.dumps(spec))
    for field in broken["structs"]["rm_err_t"]["fields"]:
        if field["name"] == "err":
            field["count"] = 23
    assert broken["structs"]["rm_err_t"]["fields"]
    # 用错误规格重新渲染会得到不同的 rm_err_t 布局（黄金表能抓住它）
    rendered = codegen.render_ctypes(broken, "0" * 64)
    assert "ctypes.c_int * 23" in rendered
    assert "ctypes.c_int * 23" not in open(
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "rm_hw_bridge", "generated", "rm_api2_ctypes.py",
        ),
        "r", encoding="utf-8",
    ).read()


def test_negative_no_vendor_or_rmctl_imports(pkg_root):
    """负例：硬件层不得 import 厂商封装（tesseract/Robotic_Arm）或体验层 rmctl。"""
    forbidden = ("tesseract", "Robotic_Arm", "rmctl")
    offenders = []
    for root, _dirs, files in os.walk(os.path.join(pkg_root, "rm_hw_bridge")):
        for fname in files:
            if not fname.endswith(".py"):
                continue
            path = os.path.join(root, fname)
            with open(path, "r", encoding="utf-8") as fh:
                for lineno, line in enumerate(fh, 1):
                    stripped = line.strip()
                    if not stripped.startswith(("import ", "from ")):
                        continue
                    for token in forbidden:
                        if token in stripped:
                            offenders.append("%s:%d: %s" % (path, lineno, stripped))
    assert not offenders, "检测到禁止的导入：\n%s" % "\n".join(offenders)


def test_negative_bind_library_reports_missing_symbols(generated_module):
    """负例：库缺少规格符号时必须报错（不能半可用静默通过）。"""

    class FakeLib:
        pass

    with pytest.raises(RuntimeError) as excinfo:
        generated_module.bind_library(FakeLib(), strict=True)
    assert "rm_init" in str(excinfo.value)

    lib, missing = generated_module.bind_library(FakeLib(), strict=False)
    assert lib is not None
    assert "rm_create_robot_arm" in missing
