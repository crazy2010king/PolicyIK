# 证据索引（docs/evidence/）

- 任务：S09-T06（切片 S09「证据闭合与需求口径校准」，里程碑 M001-acm1jg）
- 生成时间：2026-11-17（+0800）
- 作用：本目录是**入库的证据面**（替代只能留在本机 `.gsd/` 里的证据）。每份文档都给出
  覆盖的需求 ID、复算命令（可直接粘贴执行）、关键数字与诚实边界。
- 本机环境（**fallback 档，非受支持基线**）：Ubuntu 22.04 / Linux 5.15.0 /
  Python **3.10.12** / Drake **1.51.1** / git 2.34.1；
  受支持基线为 **Python 3.12 + Drake 1.55.0**（见 `versions.lock`）。
  本目录内的一切数字都是 fallback 档实测值，**不得表述为与受支持基线一致**。
- 唯一证据收集器：`scripts/collect_evidence.py`（声明式检查清单 → 命令 / 退出码 / 耗时 /
  关键输出摘要；退出码语义 `0` = 无 FAIL、`1` = 有 FAIL、`2` = 参数或环境错误）。

---

## 1. 文档清单

| # | 文档 | 覆盖需求 | 内容摘要 | 复算命令（可直接粘贴） |
|---|------|---------|---------|----------------------|
| 1 | `README.md` | 全部（索引） | 本文件：证据索引、环境口径、复算入口 | — |
| 2 | `REQUIREMENTS-PROOF.md` | **R002 / R007 / R011 / R012 / R014 / R019**（另含 R017/R006/R010 标注规则说明） | 6 条需求的可回填「目标 Proof（validation）」与「目标 Notes」措辞，逐条绑定证据小节与关键数字 | `python3 -m pytest -q tests/test_evidence_docs.py` |
| 3 | `S02-rerun.md` | R006 / R007 / R011 / R012 / R017（间接） | S02 全量复跑（8/8 PASS）与「旧口径 → 现状口径」漂移对照表（CI fast PASS=17、扫描 90 文件/13 项 0 命中、bimanual exit 0、GCS 凸松弛口径） | `python3 scripts/collect_evidence.py --output docs/evidence/s02-rerun.json` |
| 4 | `S09-ci-fast.md` | R019（间接：许可/资产链路）、R014（CI 门禁面） | 上游归档物化后官方网格解包链路跑通；`assets/visual-mesh` 由 `[SKIP]` 转 `[PASS]`，fast 档 **PASS=17 SKIP=0 FAIL=0** | `bash scripts/ci.sh fast` |
| 5 | `S09-upstream-archive.md` | **R019** | 上游归档物化与**字节级校验**闭合：`--verify` 退出码 0、两条 sha256 与 MANIFEST / versions.lock 三方逐字一致；归档**不入 git**、入库凭据与**可复现配方** | `python3 tools/fetch_upstream.py --verify` |
| 6 | `rm75-b-real-arm-ladder.md` | 真机验证阶梯（L0–L14）首轮实测 | 现场 RM75-B（192.168.1.18）**两轮**实测：只读级别 L0–L5/L10 与运动级别 L6/L7/L8/L9/L12/L14 全部 PASS；L4 因机械臂处于**急停态**（`arm_current_status=9`）判 FAIL ⇒ 运动级被门禁拦截；`cycle` 读法实测定为 **5 ms 的倍数**（P50=4.998 ms）；**末端跟踪误差 2.56 mm**（加速度上限须收紧到 1.0/1.2 rad/s²）；**修掉 14 个真机缺陷**；未验证指标逐条列出 | `python3 scripts/real_machine_ladder.py --to L5 --device-config devices/rm75_b_192_168_1_18.yaml --json` |
| 7 | `rm75-b-ladder-2026-11-19.json` | 真机验证阶梯第一轮**原始报告** | L0–L10 的逐级 PASS/FAIL/BLOCKED、判据实测值与未验证指标清单（机器可读） | `python3 scripts/real_machine_ladder.py --to L10 --device-config devices/rm75_b_192_168_1_18.yaml --json` |
| 8 | `rm75-b-real-exec-runlog.json` / `rm75-b-real-exec-recording.json` / `rm75-b-real-exec-compare.json` | **一次真机运行**的全链路运行日志 + 其录制 + 逐点一致性比较 | 真机 131 点实际下发序列与离线录制**逐点一致**（max diff 0.0 rad）；同一条录制离线重放：131 点、逃逸 0、最大单步 0.001521 rad ⇒ SC3 真机口径闭合 | `python3 scripts/compare_exec_vs_recording.py --run-log docs/evidence/rm75-b-real-exec-runlog.json --recording docs/evidence/rm75-b-real-exec-recording.json --json` |
| 9 | `rm75-b-final-round-2026-11-20.md` | **R003 / R008 / R017**（真机口径）、R001（关节空间透传） | **第三轮真机最终回归的机器可读纪要**：全阶梯 **L0–L10 PASS=11 / FAIL=0**（`mode=hardware`、`allow_motion=true`）、末端误差 **2.6620 mm**（阈值 3）、100 Hz × 30 min 长稳 **180000 拍零违规 / 抖动 P99.9 0.3145 ms**、运行时 `forbidden_call_count=0`；并列出单臂 / 五类物理故障 / 急停 / 围栏余量 4 mm 等诚实边界 | `python3 scripts/real_machine_ladder.py --to L10 --device-config devices/rm75_b_192_168_1_18.yaml --allow-motion --json` |
| 10 | `rm75-b-ladder-final-2026-11-20.json` | 真机阶梯第三轮**最终报告** | L0–L10 逐级 PASS/FAIL、判据实测值、`summary={pass:11,fail:0}`、`unverified_metrics` 清单（机器可读） | 同上 |
| 11 | `rm75-b-ladder-after-recovery-2026-11-20.json` | 网卡掉线**恢复后**回归 | 开发机→机械臂网卡恢复后的只读级回归（L0–L5/L10 pass；L6–L9 未允许运动 ⇒ blocked） | `python3 scripts/real_machine_ladder.py --to L5 --device-config devices/rm75_b_192_168_1_18.yaml --json` |
| 12 | `rm75-b-soak-30min-final-2026-11-20.json` | **R003**（抖动/时延）、**R008**（运行时零调用） | 100 Hz × 1800 s：`ticks=180000`、`commands_sent=180000`、violations=0、errors=0、抖动 P99.9 **0.3145 ms**（阈值 0.5）、命令→执行延迟 P99.9 0.2901 ms、`sdk_audit.forbidden_call_count=0`、标定后加速度上限 1.0/1.2 | `python3 scripts/soak_test.py --device-config devices/rm75_b_192_168_1_18.yaml --duration-s 1800 --hz 100 --json` |
| 13 | `rm75-b-tracking-final-2026-11-20.json` | **R003**（末端跟踪误差 < 3 mm） | 131 点配对：`tcp_error_mm` p50=1.4416 / p95=2.4467 / **max=2.6620**（阈值 3.0）⇒ `status=pass` | `python3 scripts/eval_tracking.py --run-log docs/evidence/rm75-b-runlog-final-2026-11-20.json --json` |
| 14 | `rm75-b-runlog-final-2026-11-20.json` / `rm75-b-plan-final-2026-11-20.json` | **R007**（录制/回放同源） | 最终一次真机 `cli exec` 的全链路运行日志（`mode=hardware`、265 events、131 samples、violations=0、jitter p999 0.2964 ms）与其下发的规划轨迹（recording schema v1、131 点、dt=0.005） | `python3 scripts/compare_exec_vs_recording.py --run-log docs/evidence/rm75-b-runlog-final-2026-11-20.json --recording docs/evidence/rm75-b-plan-final-2026-11-20.json --json` |
| 15 | `rm75-b-current-calibration-2026-11-20.json` | **R015**（标定链） | 电流阈值标定 `status=ok`、`calibrated=true`、逐轴 `[6379.5, 3240.0, 1762.5, 2133.0, 600.0, 718.5, 600.0]` mA | `python3 scripts/calibrate_currents.py --device-config devices/rm75_b_192_168_1_18.yaml --json` |
| 16 | `rm75-b-evidence-summary-2026-11-20.json` | 汇总口径 | `{"L8": {"exec_exit_code": 0, "tcp_tracking_error_mm": 2.6619684756546693}}` | — |
| 17 | `rm75-b-soak-8h-failed-2026-11-20.json` | **失败记录**（8 h 长稳未达标） | 8 h × 100 Hz：ticks/commands_sent 均 2 880 000，但抖动直方图 `dropped=879 999`、P99.9 **9.9276 ms**（阈值 0.5）、max 35 833 ms ⇒ `status=failed`（SDK 发送阻塞，与提交 `4cfab73` 的超时保护同根因）；`forbidden_call_count=0`。**只能作失败证据与缺陷溯源，不得当作达标证据** | `python3 scripts/long_run.py --duration-s 28800 --hz 100 --json` |

机器可读产物（与 `S02-rerun.md` 配套）：`s02-rerun.json`
（`schema_version=1`，`generator=scripts/collect_evidence.py`，逐项命令 / 退出码 / 耗时 / 摘要）。

## 2. 关键数字速查（fallback 档实测）

| 口径项 | 现状数字 | 出处 |
|-------|---------|------|
| 上游归档字节级校验 | `--verify` 退出码 **0**，「校验通过：2 个资产」，2 条 sha256 三方逐字一致 | `S09-upstream-archive.md` §1–§3 |
| CI fast 档 | **PASS=17 / SKIP=0 / FAIL=0**，退出码 0，耗时 55 s | `S09-ci-fast.md` §1 |
| 禁用接口扫描 | 扫描 **90** 个代码文件、名单 **13** 项、**0 命中** | `S02-rerun.md` §2 / §2.3 |
| 全量非慢回归 | `pytest -m "not slow"`：**1369 passed / 3 skipped / 44 deselected** | `S02-rerun.md` §2 |
| CLI `bimanual` | 退出码 **0**、29 采样点、3 区域、逃逸 0、相对位姿残差 max 1.0395e-08 m / 3.541e-08 rad | `S02-rerun.md` §2 |
| `plan_stats`（2 样本、seed 0） | 成功率 1.0、端到端 P95 10.799 s、GCS P95 8.876e-4 s（**凸松弛**口径） | `S02-rerun.md` §2 / §2.2 |
| FK 门槛 | 真实官方 URDF **< 5e-6 m**（固有残差 ≈ 2.2407e-6 m）；精确 ±π/2 变体 < 1e-6 m；鉴别力 > 1e-3 m | `../architecture/fk-tolerance.md` |
| 真机末端跟踪误差（第三轮，131 点） | **max 2.6620 mm**（阈值 3 mm），p95 2.4467 mm | `rm75-b-tracking-final-2026-11-20.json` |
| 真机 100 Hz × 30 min 长稳 | ticks **180000**、commands_sent 180000、violations **0**、errors 0、抖动 P99.9 **0.3145 ms**（阈值 0.5）、延迟 P99.9 0.2901 ms | `rm75-b-soak-30min-final-2026-11-20.json` |
| 真机运行时禁用接口调用 | `sdk_audit.forbidden_call_count=0`（18 万次真实调用） | `rm75-b-soak-30min-final-2026-11-20.json` |
| 8 h 长稳 | **未达标（失败记录已入库）**：ticks 2 880 000 全部发出，但 `dropped=879 999`、抖动 P99.9 9.9276 ms（阈值 0.5）⇒ `status=failed` | `rm75-b-soak-8h-failed-2026-11-20.json` |
| 真机阶梯最终回归 | **PASS=11 / FAIL=0**（L0–L10，`mode=hardware`、`allow_motion=true`） | `rm75-b-ladder-final-2026-11-20.json` |
| CLI `task` 双臂缺口 | **仅支持单臂**；多臂场景退出码 **2** + reason `task_single_arm_only` | `../../runbooks/s06_task_primitives.md` §7 |

## 3. 复算入口

```bash
# 全量证据复跑（唯一收集器，机器可读）
python3 scripts/collect_evidence.py --output docs/evidence/s02-rerun.json

# 单组复跑（unit / env / static / cli / plan / ci）
python3 scripts/collect_evidence.py --checks cli --json

# 上游归档字节级校验（R019）
python3 tools/fetch_upstream.py --verify

# CI fast 全档
bash scripts/ci.sh fast

# 需求 Proof 口径守护用例（本文档与 REQUIREMENTS-PROOF.md 的不回退守卫）
python3 -m pytest -q tests/test_evidence_docs.py
```

## 4. 诚实边界（统一口径）

1. **fallback 档**：本机为 Python 3.10.12 + Drake 1.51.1，**非受支持基线**（3.12 + 1.55.0）；
   本目录数字**不得**被表述为基线一致结论。
2. **GCS 口径**：本机无整数版 GCS 求解器，所有 GCS 数字均为**凸松弛**口径
   （日志恒有「整数版 GCS 不可用，回退凸松弛」WARNING）。
3. **真机未验证**：真机物理指标（抓放成功率、夹持误差、共持漂移、感知延迟等）一律标注为
   未验证，逐条列在各自的 `UNVERIFIED_*_METRICS` 唯一真相源中。
4. **归档不入 git**：上游归档实体受许可限制不随仓库分发，仓库内只保留指纹与复现配方。
5. **证据口径回退守卫**：`tests/test_evidence_docs.py` 守护「6 条需求的 Proof 必须存在且可复算」
   与「README 索引必须覆盖本目录全部文档」两条口径。
