# S09 证据 · 官方网格解包链路与 CI fast 视觉网格项转 PASS（G8）

- 任务：S09-T02（切片 S09「证据闭合与需求口径校准」，里程碑 M001-acm1jg）
- 生成时间：2026-11-17 09:05 +0800
- 结论：**归档物化后官方 RM75-B STL 解包链路跑通，`bash scripts/ci.sh fast` 的
  `assets/visual-mesh` 项由 `[SKIP]` 转为 `[PASS]`**，整档 **PASS=17 SKIP=0 FAIL=0，退出码 0**。
- 本机环境（**fallback 档，非受支持基线**）：Ubuntu 22.04 / Python 3.10.12 / Drake 1.51.1；
  受支持基线为 Python 3.12 + Drake 1.55.0（见 `versions.lock`）。本文档中的 fast 档数字
  是 fallback 档实测值，**不得表述为基线一致**。

---

## 1. 物化链路（本任务实测，全离线可用）

| # | 命令 | 退出码 | 耗时 | 关键输出 |
|---|------|--------|------|----------|
| 1 | `python3 tools/fetch_upstream.py` | **0** | 796.6 s | `rm_models` 归档 253.3 s（17 文件 / 3 312 746 B / sha256=`fb7ab533ce42…eb202b`）；`two_finger_gripper` 归档 543.3 s（480 文件 / 60 747 B / sha256=`975b5474afba…d154a5`）；收尾「校验通过：2 个资产」 |
| 2 | `python3 scripts/extract_upstream_assets.py` | **0** | 0.12 s | `rm_models-cff49bb40f5e.tar.gz → third_party/.work/rm75b（17 个文件，前缀 RM75/urdf/RM75-B/）`；`STL 网格 8 个：['base_link','link_1'…'link_7']` |
| 3 | `python3 scripts/convert_meshes.py --print-boxes` | **0** | 0.32 s | 8 个 STL → 8 个 OBJ（`models/rm75_drake/meshes/*.obj`）+ 包围盒表 + 体积/凸包统计表 |
| 4 | `python3 tools/verify_env.py --upstream --licenses` | **0** | 0.07 s | `[PASS] upstream`（MANIFEST ↔ versions.lock ↔ registry 三方交叉）+ `[PASS] licenses`（包集覆盖 / 必填字段 / 上游 commit 声明） |
| 5 | `bash scripts/ci.sh fast`（物化后） | **0** | 55 s | `PASS=17 SKIP=0 FAIL=0`；`assets/visual-mesh` = `[PASS]` |
| 6 | `bash scripts/ci.sh fast`（物化前对照，见 §3） | **0** | 55 s | `PASS=16 SKIP=1 FAIL=0`；`assets/visual-mesh` = `[SKIP]` |

> 说明：命令 1 是 S09-T01 的产物在**本工作树**的再次物化（工作树初始不含
> `third_party/archives/`、不含 `third_party/.work/`）。两条归档 sha256 与 T01 记录
> **逐字一致**，再次证明 `versions.lock:upstream` 的指纹可复现。

## 2. 官方网格包围盒（`convert_meshes.py --print-boxes`，退出码 0）

轴对齐包围盒（m）：

| link | min(x,y,z) | max(x,y,z) | extent(x,y,z) | center(x,y,z) |
|------|-----------|-----------|---------------|---------------|
| base_link | (-0.059300, -0.055000, -0.000000) | (0.055000, 0.055000, 0.138500) | (0.114300, 0.110000, 0.138500) | (-0.002150, 0.000000, 0.069250) |
| link_1 | (-0.041000, -0.068028, -0.102000) | (0.041000, 0.041000, 0.040500) | (0.082000, 0.109028, 0.142500) | (0.000000, -0.013514, -0.030750) |
| link_2 | (-0.040500, -0.152500, -0.035000) | (0.040500, 0.040500, 0.047000) | (0.081000, 0.193000, 0.082000) | (0.000000, -0.056000, 0.006000) |
| link_3 | (-0.034995, -0.067000, -0.103500) | (0.035000, 0.035000, 0.034000) | (0.069995, 0.102000, 0.137500) | (0.000002, -0.016000, -0.034750) |
| link_4 | (-0.034007, -0.128500, -0.029000) | (0.034005, 0.034000, 0.045000) | (0.068012, 0.162500, 0.074000) | (-0.000001, -0.047250, 0.008000) |
| link_5 | (-0.029000, -0.054000, -0.081500) | (0.029000, 0.029000, 0.028500) | (0.058000, 0.083000, 0.110000) | (0.000000, -0.012500, -0.026500) |
| link_6 | (-0.028607, -0.115000, -0.028500) | (0.028605, 0.028475, 0.040000) | (0.057211, 0.143475, 0.068500) | (-0.000001, -0.043263, 0.005750) |
| link_7 | (-0.028983, -0.036653, -0.029000) | (0.039008, 0.036631, 0.000000) | (0.067991, 0.073284, 0.029000) | (0.005013, -0.000011, -0.014500) |

体积 / 凸包：

| link | verts | faces | volume(m³) | hull_vtx | hull_vol(m³) |
|------|-------|-------|-----------|----------|--------------|
| base_link | 22134 | 7378 | 0.00083804 | 244 | 0.00105582 |
| link_1 | 11376 | 3792 | 0.00059175 | 505 | 0.00088005 |
| link_2 | 11418 | 3806 | 0.00043109 | 691 | 0.00069446 |
| link_3 | 10686 | 3562 | 0.00043005 | 485 | 0.00065457 |
| link_4 | 11142 | 3714 | 0.00028828 | 753 | 0.00043396 |
| link_5 | 11046 | 3682 | 0.00023870 | 532 | 0.00035645 |
| link_6 | 11742 | 3914 | 0.00021800 | 755 | 0.00033439 |
| link_7 | 10296 | 3432 | 0.00006493 | 380 | 0.00009052 |

## 3. CI fast 逐项结果：物化前 vs 物化后

**物化前**（等价于「官方 STL 未解包」的状态：把 `third_party/.work/rm75b` 临时移走，
其余一切不变，跑**最终版本**的 `scripts/ci.sh`（与下文的「物化后」同一份脚本），随后原样恢复）：

```
[SKIP] assets/visual-mesh — 官方 STL → OBJ 转换与包围盒复测（需上游归档）（0s，环境未就绪：
       [convert_meshes][ERROR] STL 源目录不存在：…/third_party/.work/rm75b/RM75/urdf/RM75-B/meshes）
--- ci tier 'fast' 汇总：PASS=16 SKIP=1 FAIL=0，耗时 55s ---
（整档退出码 0）
```

**物化后**（同一份 `scripts/ci.sh`，逐字输出）：

```
[PASS] lint/py_compile — ruff 不可用，py_compile 兜底语法检查（0s）
[PASS] tests/skeleton — 骨架单测（tests/test_env_lock.py tests/test_paths.py tests/test_forbidden_apis.py）（1s）
[PASS] lock/consistency — versions.lock 自洽性（0s）
[PASS] forbidden/apis — 禁用接口静态扫描（src tools scripts）（0s）
[PASS] tests/model — 模型回归单测（碰撞/质量/FK/限位/夹爪）（1s）
[PASS] model/verify-env — 模型自检（verify_env --model）（0s）
[PASS] tests/verify-env-contract — verify_env 退出码契约（参数错误 2 / 内容漂移 1）（0s）
[PASS] tests/mesh-tools — 网格工具单测（STL 解析 / OBJ 写出 / 包围盒 / 视觉网格链路契约）（1s）
[PASS] assets/visual-mesh — 官方 STL → OBJ 转换与包围盒复测（需上游归档）（0s）
[PASS] tests/planning — 规划回归单测（场景/轨迹/CLI/回放）（2s）
[PASS] tests/device — 设备层与安全闸回归（零硬件：七闸 / 状态机 / 执行环 / 恢复 / 运行日志 / 脚本）（3s）
[PASS] tests/bimanual-calibration — 双臂几何与标定回归（零硬件：变体 / 单 plant / 碰撞过滤 / 一致性 / 同步执行 / 标定内核 / 标定 CLI）（2s）
[PASS] tests/bimanual-coordination — 双臂紧协调回归（零硬件：解析 IK / 双臂限位 / 共享时间轴 / 最小坐标参数化 / 凸区域 / 协调规划 / 指标脚本 / 协调 CLI）（27s）
[PASS] tests/tasks — 任务原语与感知回归（零硬件：schema / 原语 / 状态机 / 双臂 / 感知 / 指标 / CLI）（8s）
[PASS] tests/regression — 会话录制回归（零硬件：session 产物 / golden 逐点一致）（3s）
[PASS] assets/lock — 上游资产三方交叉校验 + 许可门禁 + NOTICE 一致性（verify_env --upstream --licenses；gen_notice --check）（0s）
[PASS] tests/upstream — 上游获取与许可门禁回归（零硬件：fetch_upstream / asset_license_gate）（6s）
--- ci tier 'fast' 汇总：PASS=17 SKIP=0 FAIL=0，耗时 55s ---
CI_EXIT=0
```

**SKIP 差异对照（唯一差异项）**：

| 项 | 物化前 | 物化后 | 说明 |
|----|--------|--------|------|
| `assets/visual-mesh` | `[SKIP]`（退出码 2 = 环境未就绪） | `[PASS]`（退出码 0） | 官方 STL 源目录 `third_party/.work/rm75b/RM75/urdf/RM75-B/meshes` 由不存在变为存在（8 个 STL） |
| fast 档汇总 | PASS=16 / SKIP=1 / FAIL=0 | **PASS=17 / SKIP=0 / FAIL=0** | 整档退出码均为 0 |

其余 16 项在物化前后完全一致（同一命令、同一结果、同一耗时量级）。
**没有为让该项 PASS 而修改 `scripts/ci.sh` 的 SKIP 语义**：`assets/visual-mesh` 仍然是
`run_check_skippable`（退出码 2 → SKIP，退出码 0 → PASS，其他非 0 → FAIL），
该语义被 `tests/test_visual_mesh_chain.py::test_ci_visual_mesh_item_keeps_skippable_semantics`
逐字守护。

## 4. 上游三方交叉与许可门禁（`verify_env --upstream --licenses`，退出码 0）

```
[PASS] upstream — 上游资产三方交叉校验（MANIFEST ↔ versions.lock ↔ registry）
    registry 条目 22 条；lock 包 17 个；允许再分发 19 个；受限禁止再分发 3 个；
    commit 未锁定例外 2 个：['constrained-bimanual-planning-example', 'manipulation']
[PASS] licenses — 许可门禁（包集覆盖 / 必填字段 / 上游 commit 声明）
```

**既有语义（必须写清，不得误读为不一致）**：

- `--upstream` 校验的是 `third_party/MANIFEST.yaml` ↔ `versions.lock:upstream` ↔
  `licenses/registry.yaml` 三者的 **commit / sha256 / file_count / total_bytes / license 字段**，
  **不要求归档实体（tar.gz）存在**；归档缺失只写 notes，不判不一致
  （见 `tests/test_asset_license_gate.py::test_upstream_check_does_not_require_archive_entity`）。
- **归档字节级存在性由另一条命令负责**：`python3 tools/fetch_upstream.py --verify`
  （本次退出码 0，见 `docs/evidence/S09-upstream-archive.md`）。
- `--licenses` 校验包集覆盖 / 必填字段 / 上游资产的 commit 声明，与归档实体无关。

## 5. 顺带修复的真实缺陷（`scripts/convert_meshes.py`）

- **缺陷**：转换成功后打印 `out.relative_to(REPO_ROOT)`；当 `--dst` 指向**仓库外**
  目录（如 tmp_path）时抛 `ValueError: '…' is not in the subpath of '…'`，
  把一次**本已成功**的转换变成失败退出码（traceback 覆盖真实结果）。
- **修复**：新增模块级 `rel_to_repo(path)`（与 `tools/fetch_upstream.py` 同一约定）：
  仓库内路径渲染为仓库相对路径，仓库外路径用 `os.path.relpath` 渲染成 `../…` 形式，
  **不再抛异常**；`[convert_meshes] … -> …` 日志改走该函数。
- **回归用例**：`tests/test_visual_mesh_chain.py::test_rel_to_repo_handles_outside_path`
  （仓库内 / 仓库外两种路径形态）。

## 6. 失败模式（外部依赖与失败路径）

| 依赖 | 失败形态 | 实测行为 | 覆盖位置 |
|------|----------|----------|----------|
| 归档文件 `third_party/archives/*.tar.gz` | 缺失 | `extract_upstream_assets.py` 退出码 **1**，stderr 提示先运行 fetch | `tests/test_asset_license_gate.py::test_extract_missing_archive_still_exit_one` |
| 归档字节被篡改 | sha256 不符 | 解包前 sha256 门禁拦下，退出码 **1**（先校验后解包） | `tests/test_asset_license_gate.py::test_extract_rejects_tampered_archive`、`tests/test_fetch_upstream.py::test_verify_detects_tampered_archive` |
| `versions.lock` 缺 `archive.sha256` | 元数据不完整 | 退出码 **2**（环境/参数错误），拒绝解包 | `scripts/extract_upstream_assets.py`（`expected_sha` 分支） |
| STL 源目录不存在 | 解包未做 / 目录被移走 | `convert_meshes.py` 退出码 **2** → CI 记 `[SKIP]` | `tests/test_visual_mesh_chain.py::test_convert_meshes_missing_src_dir_exits_two` |
| STL 目录存在但无 `.STL` | 空目录 | 退出码 **2**，不得把「没有网格」当成功 | `tests/test_visual_mesh_chain.py::test_convert_meshes_empty_src_dir_exits_two` |
| STL 内容损坏 | 解析失败 | `convert_meshes.py` 退出码 **1**（业务失败） | `tests/test_mesh_tools.py::test_garbage_bytes_raise` / `test_truncated_binary_stl_raises` |
| `--dst` 在仓库外 | 日志渲染 | 已修复：不再抛 ValueError | `tests/test_visual_mesh_chain.py::test_rel_to_repo_handles_outside_path` |
| 网络（仅 fetch 阶段） | 抓取失败 / 断流 | fetch 非 0 退出并打印失败资产；**下游解包链路不触网**，物化失败时 `assets/visual-mesh` 只能是 SKIP（不会假 PASS） | 本文档 §3 物化前对照行 |
| 归档条目路径穿越（`../`、绝对路径） | 恶意归档 | `extract_upstream_assets.py` 退出码 **1** 并拒绝解包 | `scripts/extract_upstream_assets.py`（`rel.is_absolute() or ".." in rel.parts` 分支） |

**结论**：无一条失败路径会被伪装成 PASS——缺输入一律退出码 2（→ SKIP），
字节/解析类失败一律退出码 1（→ FAIL）。

## 7. 负载画像（10× 断点）

- 本任务链路的资源瓶颈**不在解包与网格转换**：`extract` 0.12 s、`convert` 0.32 s、
  `verify_env --upstream --licenses` 0.07 s、CI 的 `assets/visual-mesh` 项 0 s。
- 真正的 10× 断点是**归档抓取（网络）**：冷抓取 `rm_models` 253.3 s + `two_finger_gripper`
  543.3 s ≈ 13.3 min；10× 规模（≈10 个归档）时约 2.2 h，远超任何 CI 档预算。
  **保护措施**：`tools/fetch_upstream.py` 复用 `third_party/.work/` 浅克隆缓存
  （二次抓取只需重新 `git archive`），且 fetch **不在 CI fast/medium/nightly 任何档内**
  ——CI 只跑「拉取工具自身的单测」（`tests/upstream` 项，零网络）。
- 磁盘：归档 759 328 B + 14 196 B；解包产物 17 文件 / 3.3 MB；OBJ 派生 8 个文件。
  全部落 `.gitignore` 排除目录（`third_party/archives/`、`third_party/.work/`、
  `models/**/meshes/`），仓库体积不受影响。
- CPU/内存：网格转换是纯 numpy 小规模运算（最大网格 22 134 顶点），内存占用 < 100 MB。

## 8. 负向测试（防假绿灯）

| 负向场景 | 断言 | 用例 |
|----------|------|------|
| STL 源目录缺失 | 退出码 **2**（不是 0，也不是 1），stderr 含「STL 源目录不存在」 | `tests/test_visual_mesh_chain.py::test_convert_meshes_missing_src_dir_exits_two` |
| STL 目录为空 | 退出码 **2**，stderr 含「没有找到任何 .STL」 | `tests/test_visual_mesh_chain.py::test_convert_meshes_empty_src_dir_exits_two` |
| CI 该项被改成无条件 PASS | 断言 ci.sh 仍为 `run_check_skippable "assets/visual-mesh"`，且**不存在** `run_check "assets/visual-mesh"` | `tests/test_visual_mesh_chain.py::test_ci_visual_mesh_item_keeps_skippable_semantics` |
| 归档被篡改 | 解包前 sha256 门禁拦下（退出码 1） | `tests/test_asset_license_gate.py::test_extract_rejects_tampered_archive` |
| 归档缺失 | 退出码 1 且提示先物化 | `tests/test_asset_license_gate.py::test_extract_missing_archive_still_exit_one` |
| 归档实体缺失时上游检查仍成立 | `--upstream` 不要求归档实体存在 | `tests/test_asset_license_gate.py::test_upstream_check_does_not_require_archive_entity` |
| 损坏 / 截断 / 非法 STL | 解析抛错，不产出静默坏网格 | `tests/test_mesh_tools.py::test_garbage_bytes_raise`、`test_truncated_binary_stl_raises`、`test_ascii_without_solid_header_raises` |
| 仓库外 `--dst` | 不抛 ValueError，渲染为 `../…` | `tests/test_visual_mesh_chain.py::test_rel_to_repo_handles_outside_path` |

新增测试文件 `tests/test_visual_mesh_chain.py`（5 个用例，耗时 < 1 s，无 Drake 依赖）已并入
CI fast 的 `tests/mesh-tools` 项（`scripts/ci.sh`），因此本档回归常驻。

## 9. 复算方式（可直接粘贴）

```bash
# 1) 物化归档（需要网络；约 13 min，复用 third_party/.work/ 缓存更快）
python3 tools/fetch_upstream.py

# 2) 字节级校验（退出码 0 + 「校验通过：2 个资产」）
python3 tools/fetch_upstream.py --verify

# 3) 解包官方 STL 到 third_party/.work/rm75b/（退出码 0）
python3 scripts/extract_upstream_assets.py

# 4) STL → OBJ + 包围盒表（退出码 0）
python3 scripts/convert_meshes.py --print-boxes

# 5) 上游三方交叉 + 许可门禁（退出码 0）
python3 tools/verify_env.py --upstream --licenses

# 6) CI fast 档（退出码 0；物化后 assets/visual-mesh = PASS）
bash scripts/ci.sh fast

# 7) 视觉网格链路正/反向契约测试（退出码 0）
python3 -m pytest -q tests/test_visual_mesh_chain.py
```

## 10. 边界声明

- 归档本体、浅克隆缓存、派生 OBJ 网格**均不入 git**（`.gitignore`：`third_party/archives/`、
  `third_party/.work/`、`models/**/meshes/`）；仓库内只保留本证据文档。
- `third_party/MANIFEST.yaml` 的 `generated_at` / `fetched_at` 三个时间戳字段因本次
  重新抓取而更新；**sha256 / file_count / total_bytes / commit 等实质字段逐字未变**
  （`git diff` 只显示 3 行时间戳差异）。
- `scripts/ci.sh` 的改动仅限：把 `tests/test_visual_mesh_chain.py` 并入既有
  `tests/mesh-tools` 项的 pytest 列表并更新该项说明文字；**SKIP/PASS/FAIL 语义未改**。
