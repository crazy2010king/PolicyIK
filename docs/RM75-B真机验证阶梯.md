# RM75-B 真机验证阶梯（V0 → V9）

> 目标：把 M001-3kzz40 的验收条款 **CC1/CC2/CC4/CC6/CC7** 从"没有真机证据"变成
> **有据可查的阶梯式验证**；以手上**单臂**睿尔曼 RM75-B（`192.168.1.18:8080`）
> 作为唯一验证环境，**从简单到复杂**逐级取证，每级都有判据、留档与回滚口径。
>
> 工具：`scripts/rm75_validation_ladder.py`　　探针：`scripts/rm75_safe_probe.py`
> 一键脚本：`scripts/run_rm75_probe.sh`　　报告速览：`scripts/rm75_probe_report.py`
> 首轮实跑：2026-11-19，远端主机 `172.16.119.199`（容器 `--network host`）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 |
|---|---|
| 只读体检（V0+V1，臂不动） | `python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --stages V0,V1` |
| 跑到执行层闭环（V0..V5） | `python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --max-stage V5` |
| 跑到重复性（V0..V7） | `python3 scripts/rm75_validation_ladder.py --ip 192.168.1.18 --max-stage V7` |
| 高风险阶段（现场看护） | `... --stages V8,V9 --allow-risky` |
| 离线预演（零 SDK 调用） | `... --dry-run` |
| 看报告 | `python3 scripts/rm75_probe_report.py build/reports/validation/ladder_report.json` |

---

## 1. 阶梯设计（越靠前越安全、越靠后越需要现场看护）

| 阶段 | 名称 | 动作 | 风险 | 动臂 | M001 切片 | 验收条款 / 类别 |
|---|---|---|---|---|---|---|
| **V0** | 契约体检 | `rmctl doctor` 硬件面检查 + `rm_hw_bridge.audit --repo` | 无 | 否 | S02 / S09 | CC5 / Contract |
| **V1** | 连接与状态快照 | 建链 + 全字段快照 + **7 项前置检查** | 无 | 否 | S02 | CC6 / Integration |
| **V2** | UDP 上报质量 | 主动上报帧数 / **实测周期** / 断流 | 无 | 否 | S01 / S02 | CC4 / Operational |
| **V3** | 最小运动·规划通道 | `rm_movej` 单关节 **0.3°** 去程-回程 | 低 | 是 | S03 / S04 | CC2 / Operational |
| **V4** | 关节角透传 | `rm_movej_canfd` 单关节 0.3° 去程-回程 | 低 | 是 | S03 | CC6 / Integration |
| **V5** | 执行层 live 闭环 | `rm_executor --mode live`：10 ms 栅格 + 反馈闭环 + 到位判定 | 低 | 是 | S03 / S04 | CC2 / CC6 / Integration |
| **V6** | FK 交叉校验 | 自研 FK vs 控制器位姿（20 样本） | 无 | 否 | S01 | CC5 / Contract |
| **V7** | TCP 重复性 | 多次回到同一点（默认目标=**当前位姿**） | 中 | 是 | S01 | CC4 / Operational |
| **V8** | 阶梯 acc/jerk | 单关节 **10°/档** 逐档提速 | **高** | 是 | S01 / S04 | CC4 / Operational |
| **V9** | 急停演练 | 软停 + 急停留档 | **高** | 是 | S04 | CC7 / Operational |

**不在本阶梯覆盖的条款**（诚实标注，不留空）：

| 条款 | 为什么不在本阶梯 |
|---|---|
| CC1（取放循环真机 50 次） | 需要**夹爪 + 工艺场景**，本阶梯只覆盖单臂运动链路 |
| CC3（规划性能 / 人工 MTTR） | 规划内核需 tesseract 环境；MTTR 属现场人工指标 |
| CC8（切片交付线） | 由 9 个切片的 SUMMARY/ASSESSMENT 承担 |

---

## 2. 阶梯纪律（写在代码里）

| # | 纪律 | 实现 |
|---|---|---|
| 1 | **顺序不可跳过** | 前一阶段 `blocked`/`error` → 后续阶段标记 `skipped`（`--continue-on-failure` 可例外） |
| 2 | **质量失败不阻断、但必须标红** | V2 周期/断流不合格 → `failed`（整体状态降级），**不**阻断后续阶段；0 帧才是硬阻断 |
| 3 | **高风险阶段需显式授权** | V8 / V9 必须 `--allow-risky`，否则 `skipped` |
| 4 | **运动幅度由探针封顶** | 单关节、≤1.0°、低速、去程-回程对称、异常即 `rm_set_arm_stop` 急停 |
| 5 | **一轮真机工作 = 一条连接** | 控制器只接纳 ≈5 个客户端（见 §4）→ 全阶梯**共享一条句柄** |
| 6 | **不伪造数值** | 失败即 `failed`/`blocked`/`error` + 证据；报告里没有占位数字 |

---

## 3. 怎么跑（远端）

```bash
ssh -p 2224 zy@172.16.119.199
cd /home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.gsd-worktrees/M001-3kzz40

# 环境：SDK + PYTHONPATH（探针/阶梯只依赖硬件适配层，stdlib + pyyaml）
export RM_API2_LIB=/home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.deps/rm_api2/libapi_c.so
export PYTHONPATH=$PWD/dualarm_ws/src/rmctl:$PWD/dualarm_ws/src/rm_hw_bridge:$PWD/dualarm_ws/src/rm_safety:$PWD/dualarm_ws/src/rm_executor
export RM_WS=$PWD

# 建议在容器里跑（带 numpy/tesseract），并用 --network host 才能收到臂的 UDP 上报
docker run --rm --network host -v /home:/home -w "$PWD" \
  172.16.119.20:5000/rwthika/ros2-cuda:humble-desktop-full-v25.08_conda_sonnic_20261110_1124 \
  bash -lc "export PYTHONPATH=$PYTHONPATH RM_API2_LIB=$RM_API2_LIB RM_WS=$RM_WS && \
            /opt/conda/envs/drake_dev/bin/python3 scripts/rm75_validation_ladder.py \
            --ip 192.168.1.18 --max-stage V7 --ws \$PWD --udp-duration-s 60"
```

产物：`build/reports/validation/ladder_report.{json,md}` + 每阶段子目录 `V*/`。

---

## 4. 真机窗口的第一条硬规矩：连接预算

**RM75-B 控制器对同一客户端的连接数有上限（实测 ≈5）**，超过后
`rm_create_robot_arm` 返回 NULL / `id=-1`，且**释放很慢（5~10 min 量级）**。

| 现象 | 真相 |
|---|---|
| 第 6 次建链失败（即使换新进程） | 控制器侧限额耗尽，**不是网络问题** |
| 历史基线里的 "ping 100% 丢包 / handle is NULL" | 部分是**连接额度被前面的脚本耗尽**造成的假象 |

因此：

1. **一轮真机工作建一次链、全程复用**（阶梯已实现：探针 `link_factory` + `close_link=False`、
   `hwcollect.*(link=...)`、`rm_executor.execute(link=...)`）；
2. 看到 "id=-1 / NULL" 先**等释放**（或重启控制器），不要反复重试刷额度；
3. 详见 `docs/rm_hw_bridge硬件适配层说明.md §9`。

---

## 5. 2026-11-19 实跑结果（V0 → V9 全部执行完毕）

**证据工件（可入库）**：`workcell/baselines/evidence/rm75_validation_ladder_20261119.json`
（由 `scripts/rm75_evidence_merge.py` 合并 5 份真机报告生成）。

### 5.1 阶段结果总览（10 个阶段：9 pass / 1 failed）

| 阶段 | 结论（真机实测值） |
|---|---|
| V0 契约体检 | `hw:sdk` / `hw:symbols`（22 核心 + 21 支持）/ `hw:contract`（155 个 .py 零违规）全 PASS |
| V1 连接与快照 | 建链成功；7 项前置检查 7/7；控制器 `RM_NO_PLANNING_E`；7 轴全使能、错误码全 0、温度 30~32 ℃ |
| V2 UDP 上报质量 | **600 s 长跑**：24002 帧、**实测周期 24.998 ms**（配置 5 ms 的 5 倍）、**断流窗口 0**、最大空窗 0.0 ms → PASS（"到达率 20%" 的根因是控制器按 25 ms 上报，**不是随机丢包**） |
| V3 最小运动（`rm_movej`） | J1 +0.3°（2% 速度）：到位误差 **0.005°**、其它关节漂移 **0.002°**、回程 **0.038°** |
| V4 关节角透传（canfd） | 到位误差 **0.163°**、漂移 **0.002°**、回程 **0.162°** |
| V5 执行层 live 闭环 | **501 拍 / 5.53 s**：到位误差 **0.0030°**、跟随误差 RMS **0.0506°**、tick 抖动 p99 **0.201 ms** → CC2 三条门限（<0.1° / ≤0.3° / <2 ms）全达标 |
| V6 FK 交叉校验 | **failed（诚实）**：最大位置偏差 **75.03 mm**（阈值 1 mm），归因 `systematic_translation`（偏差方向一致 → 整体平移），且 TCP 未标定 → **该偏差不能当作模型精度结论** |
| V7 TCP 重复性 | 固定目标 30 次：最大散布 **0.0341 mm**、平均散布 0.0127 mm（标称 ±0.05 mm，within_nominal=true） |
| V8 阶梯 acc/jerk（高风险） | joint1 3 档（10°/档、速度 5/8/11%、每档测完回原位）：包络 acc **1.7557 rad/s²**、jerk **81.099 rad/s³** |
| V9 急停演练（高风险） | 软停 + 急停 rc=0、`drill_performed=true`、状态快照留档 → **臂现在处于停止态，需在示教器清错并重新使能** |

### 5.2 真机跑出来并修掉的 **7 个缺陷**

| # | 现象（真机） | 根因 | 修复 |
|---|---|---|---|
| 1 | 建链抛 `ValueError: NULL pointer access` | 控制器连接额度耗尽；NULL 指针读 `.contents` 抛裸 ctypes 异常 | 结构化报错 + TCP 可达性探测分类（`blocked`=额度满 / `error`=不可达）；`_handle_id` 对 NULL 安全降级 |
| 2 | 时间域限值复检 **501 项越界** | 声明限位把**规划域 ±180°** 当**执行域**用；真机 J7 = **-200.5°** 在规划域外、执行域内 | `joint_limits.yaml` 拆出 `execution_lower/upper`（J7 执行域 ±360°）；执行层用执行域、规划器仍用规划域 |
| 3 | 误判「控制器抢轨迹」（`RM_JOINT_SPACE_PLANNING_E`） | 看门狗在**阶段 B 预就位**就绪，而预就位走 `rm_movej`（控制器必然规划） | 看门狗改为**阶段 C 起流前**才启动 |
| 4 | 误判「控制器抢轨迹」（`RM_BLEND_…` / `RM_SPLINE_…` / `RM_CARTESIAN_ARC_…`） | 透传时控制器轨迹类型**随 `trajectory_mode` 与运动阶段变化** | 期望值改成**集合**（透传族）+ **连续 3 次**才判抢轨迹（抗瞬态）；别人的关节空间/回放轨迹仍 FATAL + 停机 |
| 5 | `arm_err=[0]` 被当未知错误码 → L3 缓停，轨迹只跑 2 拍 | 厂商约定 0 = 无错误，但被当"未知码"保守升级 | 安全层排除 0（真机保护/碰撞仍照常升级） |
| 6 | **普通停止态**被当「保护停」→ L4 急停（ticks=2 即停） | `RM_STOP_E(9)` 是**普通停止态**（空闲/预就位刚结束/透传刚下发都会短暂出现），单看状态就升级 L4 | 保护停判据加**佐证**：有非零错误码 → 立即 L4；或停止态**持续 >100 ms** 且正在下发指令 → L4；否则只记 advisory |
| 7 | 阶梯实验 `joints` 参数**被忽略** → 7 个关节各走 10° 且不回原位 | 真机路径硬编码 `range(7)` | 真机路径**只动指定关节**（1 基关节号），每档测完**自动回原位**（净位移 0） |

外加**阶梯自身的两处判据错误**（本轮自查发现并修掉）：

* V6 只看采集层 `status`（"ok"）→ 把 FK 交叉校验的 **FAIL 误报成通过**；改为判 `summary.ok` + 归因解释；
* 报告里"只跑部分阶段"时阶段号与名称**张冠李戴**（拿全局表 zip 部分结果）；改为按位置配对。

### 5.3 仍未达标 / 仍未做

| 项 | 状态 | 归属 |
|---|---|---|
| V6 FK 交叉校验 | failed（**真不合格**）：75.03 mm 系统性平移 | 需先**标定 TCP**（`calibration.tcp` 现为 null），再复测；零位也未取证 |
| CC1（取放 50 次真机）/ CC3（性能+人工 MTTR）/ CC8 | 不在本阶梯范围（见 §1） | — |
| V8 的 joint2..joint7 | 未测（只测了 joint1，3 档） | 基线 `measured_limits` 仍是 blocked：需 7 个关节都测齐才能把 status 改 measured |
| 基线回填 | 四张基线仍是 `blocked` | 已把实测值写进各基线的 `partial_observations` + 证据工件；正式值字段待全表测齐 |
| 右臂 192.168.1.19 | 未接触 | — |
| 控制器固件 / 示教器版本 | 未读取 | 需示教器界面 |

---

## 6. 与 M001 里程碑验证的关系

`.gsd/phases/01-3kzz40-phase/01-VALIDATION.md` 的判定是 `needs-attention`，
**唯一原因**是"真机窗口不可达 → CC1/CC2/CC4/CC6/CC7 与 Operational/UAT 计划项缺少
真机与现场人证据"。本阶梯的定位就是**逐条补上这些证据**：

| 验收条款 | 本阶梯给的证据（2026-11-19 真机） | 还差什么 |
|---|---|---|
| CC2 | V3/V5：真机微动 + **501 拍 10 ms 透传闭环**，到位误差 0.0030°、跟随误差 RMS 0.0506°、抖动 p99 0.201 ms（**三条门限全达标**） | 600 s **无保护停**长跑（当前只做了 600 s 上报长跑，不是执行长跑） |
| CC4 | V2（600 s 上报长跑，零断流）/ V7（重复性 0.0341 mm）/ V8（joint1 acc/jerk 3 档） | joint2..joint7 的 acc/jerk；供电/接地等现场项 |
| CC5 | V0（契约+符号+审计）/ V6（FK 交叉**已跑但不合格**：75.03 mm 系统性平移） | V6 要先标定 TCP 才能达标 |
| CC6 | V1/V4/V5：建链→快照→透传→反馈闭环→到位判定（**全链路真机通过**） | 断流 200 ms 内停机的**真机复现**（离线注入已有） |
| CC7 | V9：急停演练留档（软停+急停 rc=0、状态快照） | 「≤50 圈强制人工检查」流程需写入工艺文件 |

> 本文件**不替代**里程碑验证流程：证据产出后仍需按 `docs/code_doc/真机基线与实测限值归档说明.md`
> 回填 `workcell/baselines/*.yaml`，再重跑 `/gsd validate-milestone`。

---

## 7. 离线自检（不碰真机）

```bash
python3 -m pytest dualarm_ws/src/rmctl/tests/test_rm75_validation_ladder.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_safe_probe.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_probe_runner.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_probe_report.py \
                 dualarm_ws/src/rm_executor/tests/test_feedback_events.py \
                 dualarm_ws/src/rm_safety/tests/test_levels_policy.py -q
```

证据工件（入库用）：

```bash
python3 scripts/rm75_evidence_merge.py \
    --out workcell/baselines/evidence/rm75_validation_ladder_20261119.json \
    build/reports/validation/ladder_report.json \
    build/reports/validation_rerun/ladder_report.json \
    build/reports/validation_udp600/ladder_report.json \
    build/reports/validation_v8/ladder_report.json \
    build/reports/validation_v9/ladder_report.json
```

阶梯编排器的离线用例覆盖：阶段注册表与风险标注、高风险阶段授权闸门、
前序失败 → 后续 skipped、质量失败不阻断但整体降级、共享句柄只建一次、
探针阶段不关闭共享句柄、连不上 → blocked、验收条款覆盖表（含未覆盖原因）、
微动轨迹生成（时间严格递增、位移=设定值、首点=当前位姿、其它关节零位移）。
