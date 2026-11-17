# S02 证据 · runtime-executable 全量复跑与「旧口径 → 现状口径」漂移对照（R-4）

- 任务：S09-T03（切片 S09「证据闭合与需求口径校准」，里程碑 M001-acm1jg）
- 生成时间：2026-11-17 09:35 +0800（证据 JSON 的 `environment.timestamp` = `2026-11-17T09:35:59+0800`）
- 唯一证据收集器：`scripts/collect_evidence.py`（声明式检查清单 → 命令 / 退出码 / 耗时 / 关键输出摘要）
- 机器可读产物：`docs/evidence/s02-rerun.json`（`schema_version=1`，`generator=scripts/collect_evidence.py`）
- 复算命令（可直接粘贴执行）：

  ```bash
  python3 scripts/collect_evidence.py --output docs/evidence/s02-rerun.json
  # 只跑某一组：--checks unit / env / static / cli / plan / ci
  # 机器可读：--json ；清单：--list
  ```

- 本机环境（**fallback 档，非受支持基线**）：Ubuntu 22.04 / Linux 5.15.0 / Python **3.10.12** /
  Drake **1.51.1**；受支持基线为 Python 3.12 + Drake 1.55.0（见 `versions.lock`）。
  **本文档中的一切数字都是 fallback 档实测值，不得表述为与受支持基线一致**，
  也**不得**把 Drake 1.51.1 的行为差异说成基线结论。

---

## 1. 结论

一轮全量复跑 **8/8 检查 PASS，SKIP=0，FAIL=0，收集器退出码 0**（总耗时 ≈ 570 s）。
S02 的 UAT 文本（`01-02-UAT.md`）确已过期：其中「CI fast PASS=9 SKIP=0」「扫描 34 个文件、名单 11 项」
「bimanual 未实现、退出码 2」等口径与现状不符，详见 §4 漂移对照表。

复跑同时暴露了**一处旧 UAT 文本未记录的现状缺口**：`plan_stats` / `verify_env --plan` / CLI 规划链路的
GCS 阶段在本机**恒为凸松弛**（日志恒有
`WARNING:rm75_drake.planning.gcs:整数版 GCS 不可用，回退凸松弛：GraphOfConvexSets: There is no solver
available that can solve the mixed-integer version of this problem.`），即本机**没有整数版 GCS 求解器**，
GCS 数字必须按**凸松弛**口径解读。

## 2. 逐条证据

命令均为收集器实际执行的那一条（`PYTHONPATH=src` 由收集器按检查项注入），
耗时为收集器实测（`duration_s`，秒）。

| # | 检查 id | 组 | 命令 | 退出码 | 耗时(s) | 判定 | 关键输出摘要 |
|---|---------|----|------|--------|---------|------|--------------|
| 1 | `pytest-not-slow` | unit | `python3 -m pytest -q -m "not slow"` | **0** | 389.3 | PASS | `1369 passed, 3 skipped, 44 deselected, 19 warnings in 388.88s` |
| 2 | `verify-env-model` | env | `python3 tools/verify_env.py --model` | **0** | 0.2 | PASS | `[PASS] model — 改造版 RM75-B 模型自检（DoF / 总质量 / 零位末端高度 / 7 条限位）` |
| 3 | `verify-env-plan` | env | `python3 tools/verify_env.py --plan` | **0** | 14.9 | PASS | `[PASS] plan — 规划链路自检（场景装配 + IK + 短规划 + 碰撞复检）` |
| 4 | `forbidden-apis-scan` | static | `python3 scripts/scan_forbidden_apis.py src tools scripts` | **0** | 0.2 | PASS | `scan_forbidden_apis: 未命中禁用接口（扫描 90 个代码文件，名单 13 项）` |
| 5 | `cli-single-replay` | cli | ① `python3 -m rm75_drake.cli single --scenario scenarios/single_arm.yaml --target-joints 0.1 -0.5 0.0 0.8 0.0 0.4 0.0 --region-spacing 0.25 --output <tmp>/recording.json --json` ② `python3 -m rm75_drake.cli replay --recording <tmp>/recording.json --scenario scenarios/single_arm.yaml` | ① **0** ② **0** | 84.9 / 0.2 | PASS | ①`"regions": 84.36`，`"gcs": 0.0026`，`"retime": 0.2347`，`"collision": 0.0022`，`"num_escapes": 0`；②`复检位形数：155`、`逃逸数：0`、`最大单步关节变化：0.012166 rad` |
| 6 | `cli-bimanual` | cli | `python3 -m rm75_drake.cli bimanual --json` | **0** | 1.4 | PASS | `"success": true`、`协调规划成功：29 个采样点（0.005 s 网格），时长 0.140 s，区域 3 个，GCS 模式 convex_relaxation，碰撞复检逃逸 0，相对位姿残差 max 1.040e-08 m / 3.541e-08 rad`；诊断 `"relative_pose_residual_max_position_m": 1.0395e-08`、`"num_escapes": 0` |
| 7 | `plan-stats` | plan | `python3 scripts/plan_stats.py --samples 2 --seed 0 --json` | **0** | 21.8 | PASS | `"samples_completed": 2`、`"truncated": false`、`"success_rate": 1.0`、端到端 `"p95": 10.799` s、GCS `"p95": 8.876e-4` s、`"gcs_p95_target_met": true`（目标 1.0 s）、`"iris_per_region_mean_s": 5.324`、`"total_wall_time_s": 21.538` |
| 8 | `ci-fast` | ci | `bash scripts/ci.sh fast` | **0** | 55.4 | PASS | `--- ci tier 'fast' 汇总：PASS=17 SKIP=0 FAIL=0，耗时 55s ---` |

**汇总**：`{'passed': 8, 'skipped': 0, 'failed': 0, 'total': 8}`，收集器退出码 **0**。

### 2.1 采集器判定了「失败」的一次真实失败（不隐藏）

首轮收集器运行中，检查 #5 被判为 **FAIL**（退出码 2，耗时 0.1 s，
原因 `exit_code_mismatch`）：首次清单把 7 个关节目标写成**单个** argv 元素
（`"--target-joints", "0.1 -0.5 0.0 0.8 0.0 0.4 0.0"`），而 CLI 的 `--target-joints` 是
`nargs=7`（`src/rm75_drake/cli.py`），故 argparse 直接以退出码 2 拒绝。
修正为 7 个独立 argv 元素后复跑通过。

这条失败被**如实保留在本文档中**，因为它正是「收集器不把失败伪装成通过」的直接证据：
同一份清单，第一次被判 FAIL 并以退出码 1 结束（`COLLECT_EXIT=1`），
修正后被判 PASS 并以退出码 0 结束（`COLLECT_EXIT=0`）。

### 2.2 GCS 口径：凸松弛，不是整数 GCS

检查 #3 / #5 / #6 / #7 的 stderr 中**恒有**同一条 WARNING：

```
WARNING:rm75_drake.planning.gcs:整数版 GCS 不可用，回退凸松弛：GraphOfConvexSets:
There is no solver available that can solve the mixed-integer version of this problem.
```

含义（必须随数字一起引用）：

- 本机**没有整数版 GCS 求解器**（无 Mosek / Gurobi / SCIP 混合整数求解器），
  `GraphOfConvexSets` 只能走 `convex_relaxation=true` 的 Clp 凸松弛；
- 因此 §2 第 7 行的 **GCS P95 = 8.876e-4 s** 是**凸松弛口径**，不是整数 GCS 口径；
- 碰撞复检零逃逸（#5 的 `逃逸数：0`、#6 的 `num_escapes: 0`）是当前**唯一**的兜底安全性证据；
- 凸松弛解点退化 ⇒ 几何路径由 IRIS 区域链种子插值得到（与 S02 旧 UAT 的已知限制第 3 条一致）。

### 2.3 禁用接口扫描的文件数（90，不是 89）

检查 #4 报「扫描 **90** 个代码文件，名单 13 项」。旧 UAT 写「34 个文件、名单 11 项」。
当前 90 与本切片 T02 文档记录的 89 相差 1 个文件，原因是**本任务新增了
`scripts/collect_evidence.py`**（扫描范围 = `src tools scripts`，新增 1 个脚本文件即 +1）。
**名单仍是 13 项、命中仍是 0**；文件数随代码量增长属正常，引用时请以实测值为准。

## 3. 复算方式与可复现性

- 收集器把每条检查的**完整命令**（含 `PYTHONPATH=src` 注入与 `--target-joints` 的 7 个独立参数）
  写进 `docs/evidence/s02-rerun.json` 的 `checks[].steps[].command`，
  配合 `exit_code` / `duration_s` / `summary` / `pattern_matched` 可逐条重算；
- 判定逻辑集中在收集器内（`run_step` 的判定链：命令不存在 → SKIP；超时 → FAIL；
  退出码不符 → FAIL；判定正则不命中 → FAIL；否则 PASS），**任何一项都不会被静默跳过**：
  环境缺失记为 `SKIP` 并计入汇总（`passed` / `skipped` / `failed` 三项分开计数）；
- 收集器自身的退出码语义：`0` = 无 FAIL（可能存在 SKIP）、`1` = 存在 FAIL、`2` = 参数或环境错误
  （未知检查组 / id、输出路径不可写）；
- 判定逻辑单测：`tests/test_collect_evidence.py`（假可执行脚本驱动 **通过 / 失败 / 超时 / 缺命令**
  四种分支 + 退出码语义），**不跑任何重负载规划**。

## 4. 「S02 旧口径 → 现状口径」漂移对照表

旧口径取自 `.gsd/phases/01-acm1jg-phase/01-02-UAT.md`（S02 的 UAT 文本，**该文本本身尚未更新**）；
现状口径取自本轮复跑（`docs/evidence/s02-rerun.json`）。

| # | 口径项 | S02 旧口径（`01-02-UAT.md`） | 现状口径（本轮实测） | 差异性质 |
|---|--------|------------------------------|----------------------|----------|
| 1 | CI fast 汇总 | `PASS=9 SKIP=0 FAIL=0` | **`PASS=17 SKIP=0 FAIL=0`**（耗时 55 s，退出码 0） | 旧文本**漏记**了 S03–S08 追加的常驻检查项（设备层 / 双臂 / 任务 / 录制 / 上游资产等），并漏记 `assets/visual-mesh` 由 SKIP 转 PASS（归档物化后） |
| 2 | 禁用接口扫描 | 「扫描 34 个文件、名单 11 项」，0 命中 | 「扫描 **90** 个代码文件、名单 **13** 项」，**0 命中**（退出码 0） | 名单从 11 扩到 13 项（`versions.lock` 为唯一真相源），扫描面随 `src tools scripts` 增长；**命中数始终为 0** |
| 3 | CLI `bimanual` | 「明确报『双臂紧协调属 S05，未实现』并返回**非 0**」 | **已实现，退出码 0**：`"success": true`、29 采样点 / 3 区域、**14 维零逃逸**（`num_escapes: 0`）、**相对位姿残差 max 1.0395e-08 m / 3.541e-08 rad** | 旧文本描述的是 S05 之前的占位行为；现状由 S05 交付。**残差量级 ~1e-8 m**（不是 1e-6 级别）说明相对位姿约束是**构造性满足**的 |
| 4 | CLI `bimanual` 的物理指标 | 旧文本未区分仿真与真机 | 现状输出仍带 `unverified_metrics`（真机双臂成功率 ≥95%、协调 P95 <3 s、共持漂移 <2 mm/10 s、真机相对位姿保持、真机 14 维零逃逸 —— **全部未验证**） | 现状**没有**把仿真结果冒充真机达标；旧文本根本没提这条门控 |
| 5 | `plan_stats` GCS 数字口径 | 未写明是整数 GCS 还是凸松弛 | 恒为**凸松弛**（`整数版 GCS 不可用，回退凸松弛` WARNING 每次必现）：GCS **P95 = 8.876e-4 s**（目标 1.0 s，`gcs_p95_target_met: true`）、端到端 **P95 = 10.799 s**、成功率 1.0（2 组）、IRIS 单区域 5.324 s | 旧文本只说「实测约 0.92 ms」，**未标注求解器口径**；引用 GCS 数字时必须带「凸松弛」限定语 |
| 6 | FK 门槛 | 旧文本/旧计划写「位置误差 **< 1e-6 m**」 | 当前测试层已放宽为 **< 5e-6 m**（官方 RM75-B URDF 的 `rpy=1.5708` 是四舍五入值 ⇒ 端点 y 有 **~2.2407e-6 m** 固有残差）；精确 ±π/2 语义变体仍以 < 1e-6 m 判定 | **两条门槛不是同一条**；旧口径（<1e-6 m）对官方 URDF 不成立。本轮复跑**不改变**该阈值；口径收敛与正式声明由 **S09-T04** 承接（`docs/architecture/fk-tolerance.md`） |
| 7 | CI 汇总口径 vs 单项门槛 | 未区分 | `ci.sh fast` 只统计**项级** PASS / SKIP / FAIL（是否命令成功退出），**不校验数值门槛**（FK 5e-6 m、GCS P95 1 s 等都不在 fast 档判定内） | 「CI fast 全绿」**不等于**「数值门槛达标」；引用时不得混用 |
| 8 | 规划链单测规模 | 旧文本按 S02 当时的三五个用例描述 | 全量 `pytest -m "not slow"`：**1369 passed / 3 skipped / 44 deselected**，389 s | 旧文本未给全量口径数字 |
| 9 | CI fast 单项粒度 | 未逐项列出 | 17 项全部 `[PASS]`，**SKIP=0**（`assets/visual-mesh` 因上游归档已物化而 PASS，见 `docs/evidence/S09-ci-fast.md`） | 归档物化后无 SKIP 项 |

## 5. 已知限制（如实记录，不判失败）

1. **fallback 档**：Python 3.10.12 + Drake 1.51.1，**非受支持基线**（3.12 + 1.55.0）。
   数字不得表述为基线一致；基线复跑仍待补。
2. **无整数 GCS 求解器**：GCS 数字为凸松弛口径（§2.2），几何路径靠 IRIS 区域链插值 +
   碰撞复检零逃逸兜底。
3. **`plan_stats` 只采了 2 组**（`--samples 2 --seed 0`，有界采样）：成功率 1.0 / P95 等
   是 **2 组样本**的统计量，样本量小，**不能**当作大规模统计结论。
4. **端到端 P95 10.8 s 由 IRIS 主导**（单区域 5.3 s），生产依赖 `RegionCache`；
   本轮未测缓存命中路径。
5. **无真机**：`bimanual` 的真机指标（成功率 / P95 / 共持漂移 / 零逃逸）全部 `unverified_metrics`，
   本文档不提供任何真机结论。
6. **meshcat 不可用**：可视化走 2D 兜底，本轮未涉及三维浏览器可视化。
7. 本轮复跑**未修改**任何被测行为（只新增了收集器脚本与单测），故旧结论（模型真值、
   场景 schema、录制回放契约等）仍然成立。

## 6. 产物清单

| 文件 | 说明 |
|------|------|
| `scripts/collect_evidence.py` | 唯一证据收集器（声明式清单 / `--checks` / `--json` / `--output` / `--list`；退出码 0/1/2） |
| `tests/test_collect_evidence.py` | 收集器判定逻辑与退出码语义单测（假命令，不跑重负载规划） |
| `docs/evidence/s02-rerun.json` | 本轮机器可读证据（8 条检查，含命令 / 退出码 / 耗时 / 摘要 / 环境指纹） |
| `docs/evidence/S02-rerun.md` | 本文档（人类可读逐条证据 + 旧→新口径漂移对照表 + 已知限制） |
