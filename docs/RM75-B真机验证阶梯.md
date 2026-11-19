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

## 5. 2026-11-19 首轮实跑结果

### 5.1 通过的阶段

| 阶段 | 结论（实测值） |
|---|---|
| V0 | `hw:sdk` / `hw:symbols`（22 核心 + 21 支持）/ `hw:contract`（155 个 .py 零违规）全 PASS |
| V1 | 建链成功；7 项前置检查 7/7 通过；控制器 `RM_NO_PLANNING_E` / `RM_IDLE_E`；7 轴全部使能、错误码全 0、温度 30~32 ℃ |
| V2 | 60 s 上报 **2400 帧**，**实测周期 25.0 ms**（配置 5 ms 的 5×），无断流窗口；到达率 20% → **质量不合格**（不阻断） |
| V3 | J1 +0.3°（2% 速度）：到位误差 **0.016~0.054°**，其它关节漂移 **≤0.004°**，回程 **0.027~0.051°** |
| V4 | canfd 透传：到位误差 **0.152~0.186°**，漂移 **≤0.004°**，回程 **0.138~0.165°** |

### 5.2 V5（执行层 live）暴露并修掉的 **4 个真机缺陷**

| # | 现象（真机） | 根因 | 修复 |
|---|---|---|---|
| 1 | 建链抛 `ValueError: NULL pointer access` | 控制器连接额度耗尽；且 NULL 指针读 `.contents` 抛裸 ctypes 异常 | 本层安全降级 + TCP 可达性探测分类（`blocked` vs `error`） |
| 2 | 时间域限值复检 **501 项越界** | 声明限位把 **规划域 ±180°** 当成 **执行域** 用；真机 J7 = **-200.5°** 落在规划域外、执行域内 | `joint_limits.yaml` 拆出 `execution_lower/upper`（J7 ±360°），执行层用执行域、规划器仍用规划域 |
| 3 | 误判「控制器抢轨迹」（`RM_JOINT_SPACE_PLANNING_E`） | 看门狗在**阶段 B 预就位**就绪，而预就位本来就走 `rm_movej`（控制器规划） | 看门狗改为**阶段 C 起流前**才启动 |
| 4 | 误判「控制器抢轨迹」（`RM_BLEND_…` / `RM_SPLINE_…`） | 透传时控制器的轨迹类型**随 `trajectory_mode` 变化**（0→纯透传 / 1→样条 / 滤波→混合），不是恒为纯透传 | 期望值改成**集合**：{纯透传, 混合, 样条}；真正的抢轨迹（关节空间/笛卡尔/回放）仍然 FATAL + 停机 |

外加一处**安全层误升级**：`arm_err=[0]`（0 = 无错误）被当成"未知错误码"→ L3 缓停，
导致轨迹只跑 2 拍就被停掉 —— 已修正为**排除 0**（并补回归用例）。

### 5.3 仍未达标 / 仍未做（**会话结束时的真实状态**）

| 项 | 状态 | 归属 / 下一步 |
|---|---|---|
| UDP 上报到达率 ~20%、实测周期 25 ms（≠ 配置 5 ms） | 质量不合格（V2 `failed`，不阻断后续阶段） | `network_plan.yaml` 仍 `blocked`；需 600 s 长跑 + 主机侧负载排查 |
| V5 执行层 live 闭环 | 4 个真机缺陷已定位并修复（含回归用例）；**修完后的确认复跑未完成** | 见 §5.4 的现场待办；复跑命令见 §3 |
| V6 FK 交叉校验、V7 TCP 重复性 | 被 V5 的失败挡住（阶梯纪律）→ V5 通过后可跑 | CC5 / CC4 |
| V8 阶梯 acc/jerk（10°/档）、V9 急停演练 | **未跑**（需 `--allow-risky` + 现场看护 + 示教器在手边） | CC4 / CC7 |
| CC1 / CC3 / CC8 | 不在本阶梯范围（见 §1） | — |

### 5.4 现场待办（**会话结束时臂不可达**）

会话末尾（约 17:45 起）`ping 192.168.1.18` **持续 100% 丢包**（持续 >10 min），
控制器整体不可达（TCP 8080 亦连不上）。因此 V5 的确认复跑、V6/V7、V8/V9 都没能做完。

| 待办 | 说明 |
|---|---|
| ① 现场确认臂的**供电与网线**（或控制器是否重启/挂死） | 这是**环境条件**，不是代码问题；阶梯此时会如实报 `blocked`（"控制器不可达"），不会伪造结论 |
| ② 恢复后按 §3 重跑 `--max-stage V7` | 阶梯会重跑 V0..V7 并覆盖 `build/reports/validation/ladder_report.json` |
| ③ V8 / V9 单独安排现场窗口 | `--allow-risky` + 急停按钮在手 + 示教器在手（急停后需人工清错并重新使能） |
| ④ 补 600 s UDP 长跑 | 把 `network_plan.yaml` 的上报质量三项真正测出来（到达率 / 周期 / 抖动） |

> **可能性提示（不下定论）**：会话中出现过"连接额度耗尽 + 反复重试"（本轮多次触发
> `id=-1`），控制器随后整体不可达。**无法确认**二者是否相关，但**连接风暴确实是
> 本轮暴露出的最需要收敛的行为** —— 现在全链路都改成"一轮真机工作一条连接"，
> 且连接额度耗尽会立刻以 `blocked` + 可粘贴处置动作报出来（不再刷重试）。

---

## 6. 与 M001 里程碑验证的关系

`.gsd/phases/01-3kzz40-phase/01-VALIDATION.md` 的判定是 `needs-attention`，
**唯一原因**是"真机窗口不可达 → CC1/CC2/CC4/CC6/CC7 与 Operational/UAT 计划项缺少
真机与现场人证据"。本阶梯的定位就是**逐条补上这些证据**：

| 验收条款 | 本阶梯能给的证据 | 还差什么 |
|---|---|---|
| CC2 | V3/V5：真机微动 + 10 ms 透传闭环 + 到位误差 | 600 s 无保护停长跑、tick p99、跟随误差 RMS |
| CC4 | V2/V7/V8：上报质量、重复性、acc/jerk 实测 | V8 需现场看护；供电/接地等现场项 |
| CC5 | V0/V6：契约、符号、FK 交叉 | — |
| CC6 | V1/V4/V5：建链→快照→透传→反馈闭环 | 断流 200 ms 内停机的真机复现 |
| CC7 | V9：急停演练留档 | V9 未跑；「≤50 圈强制人工检查」流程需写入工艺文件 |

> 本文件**不替代**里程碑验证流程：证据产出后仍需按 `docs/code_doc/真机基线与实测限值归档说明.md`
> 回填 `workcell/baselines/*.yaml`，再重跑 `/gsd validate-milestone`。

---

## 7. 离线自检（不碰真机）

```bash
python3 -m pytest dualarm_ws/src/rmctl/tests/test_rm75_validation_ladder.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_safe_probe.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_probe_runner.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_probe_report.py -q
```

阶梯编排器的离线用例覆盖：阶段注册表与风险标注、高风险阶段授权闸门、
前序失败 → 后续 skipped、质量失败不阻断但整体降级、共享句柄只建一次、
探针阶段不关闭共享句柄、连不上 → blocked、验收条款覆盖表（含未覆盖原因）、
微动轨迹生成（时间严格递增、位移=设定值、首点=当前位姿、其它关节零位移）。
