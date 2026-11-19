# RM75-B 真机「最小动作」安全探针（rm75_safe_probe）

> 目的：在**不产生任何危险动作**的前提下，验证本项目的控制接口
> （`rm_hw_bridge` 句柄语义 → `rm_movej` / `rm_movej_canfd` 关节角通道 →
> `rm_set_arm_stop` 停机族）是否真的能控制真机。
>
> 工具：`scripts/rm75_safe_probe.py`　　测试：`dualarm_ws/src/rmctl/tests/test_rm75_safe_probe.py`
> 首次实跑：2026-11-19，远端主机 `172.16.119.199`，控制器 `192.168.1.18:8080`（左臂 RM75-B）。

---

## 0. 一分钟索引

| 我要做的事 | 命令 |
|---|---|
| 离线预演（零 SDK 调用） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --dry-run` |
| **只读体检**（零运动） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only` |
| 最小动作验证（0.3°） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej --joint 0 --delta-deg 0.3 --speed-pct 2` |
| 透传通道验证（0.3°） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel canfd --joint 0 --delta-deg 0.3` |
| 落盘报告 | 加 `--out build/reports/rm75_safe_probe.json` |
| 离线单测 | `python3 -m pytest dualarm_ws/src/rmctl/tests/test_rm75_safe_probe.py -q` |

---

## 1. 安全设计（硬约束写在代码里，不靠"人记得"）

| # | 约束 | 实现位置 | 越界后果 |
|---|---|---|---|
| 1 | **单关节、单次、亚度级** | `--delta-deg` 硬上限 `1.0°`（默认 `0.3°`），只动 1 个关节 | 超过上限 → **0 次 SDK 调用**即拒绝（`envelope` 阶段失败） |
| 2 | **低速** | `rm_movej` 速度百分比硬上限 `10%`（默认 `2%` ≈ 3.6°/s） | 同上，参数层直接拒绝 |
| 3 | **先只读后运动** | `--precheck-only` 模式；先跑只读，确认臂在线且空闲再运动 | 只读阶段不匹配就退出码 4，不会运动 |
| 4 | **安全开关先武装** | 运动前必须成功下发 `rm_set_collision_state(8)` + `rm_set_self_collision_enable(True)` | 武装失败 → 中止，**不动** |
| 5 | **只走关节角通道** | `rm_movej` / `rm_movej_canfd`；位姿类接口在 `contract.yaml` 黑名单里 | 静态审计（`hw:contract`）保证不越线 |
| 6 | **去程-回程对称** | 去程 `+δ` → 回读校验 → 回程 `-δ` → 回读校验 | 结束时臂回到初始位姿 |
| 7 | **异常即急停** | 任一步失败/超时/回读超差 → `rm_set_arm_stop`（L4 急停） | 报告里留 `estop` 阶段证据 |
| 8 | **不伪造数值** | 读不到就 `blocked` / `error` + 证据 | 报告里没有占位数值 |

判定容差：到位 `0.2°`、其它关节漂移 `0.1°`、回程归位 `0.2°`。

---

## 2. 环境准备（远端主机）

### 2.1 RM-API2 动态库

契约要求 **RM-API2 1.1.7**（x86-64 SysV ABI）。二选一：

```bash
# ① 环境变量
export RM_API2_LIB=/abs/path/to/libapi_c.so
# ② 物化到仓库（.deps/ 是 gitignored，不入库）
mkdir -p <repo>/.deps/rm_api2 && cp /abs/path/to/libapi_c.so <repo>/.deps/rm_api2/
```

> 本轮实测使用 `robotic_arm-1.1.7` wheel 里的 `Robotic_Arm/libs/linux_x86/libapi_c.so`
> （sha256 `396913cc1a839e948ca58e02525f9098e86026e5b0f62d2561acc3de53105aac`），
> 与 `dualarm_ws/src/rm_hw_bridge/vendor/rm_api2/PROVENANCE.md` 记录的 ABI 口径一致。
>
> **放在 worktree 的 `.deps/` 里会破坏一条既有单测**
> （`test_gripper_probe_real_path_blocked_without_sdk` 断言"没有 SDK → blocked"，
> 而 `.deps/rm_api2/` 一旦存在，探针就会真的去连真机 → 退出码变成 4）。
> 因此本轮把动态库放在**主 checkout** 的 `.deps/`（同样是 gitignored），并用环境变量指过去：
>
> ```bash
> export RM_API2_LIB=<主 checkout>/.deps/rm_api2/libapi_c.so
> ```

### 2.2 PYTHONPATH

探针只依赖硬件适配层（stdlib + pyyaml），**不需要 tesseract**：

```bash
cd <repo>
export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PWD/dualarm_ws/src/rm_safety:$PWD/dualarm_ws/src/rmctl
```

### 2.3 单客户端约束

同一 `ip:port` 只允许一个客户端：探针跑之前确认 8080 没被别的进程占着
（`ss -ulnp | grep 8091` 看 UDP 主动上报端口是否空闲）。

---

## 3. 执行顺序（**必须按这个顺序**）

```bash
# ① 离线预演：只验证参数与计划，不碰真机
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --dry-run

# ② 只读体检：连上真机、读状态、过 7 项前置检查，**不发任何运动指令**
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only

# ③ 最小动作：单关节 0.3°、2% 速度、去程-回程
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej \
    --joint 0 --delta-deg 0.3 --speed-pct 2 --out build/reports/rm75_safe_probe.json

# ④ 透传通道（可选）：验证 10 ms 单时钟主通道 rm_movej_canfd
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel canfd \
    --joint 0 --delta-deg 0.3 --out build/reports/rm75_safe_probe_canfd.json
```

### 前置检查（7 项，任一不过 → **不运动**）

| 检查项 | 判据 |
|---|---|
| `arm_err_empty` | 臂级错误码全 0（厂商 `err[]` 带长度，值为 0 = 无错误） |
| `joint_err_code_zero` | 7 个关节错误码全 0 |
| `all_joints_enabled` | 7 个关节使能位全 1 |
| `controller_idle` | 控制器规划类型 = `RM_NO_PLANNING_E`（没在跑别的轨迹） |
| `joints_readable` | 读到 7 个关节角 |
| `within_limits` | 当前位姿离限位 ≥ 1° |
| `target_within_limits` | 目标位姿离限位 ≥ 1° |

---

## 4. 退出码与失败处理

| 退出码 | 含义 | 处理 |
|---|---|---|
| `0` | 通过（`ok`）或离线预演（`dry_run`） | 进入下一阶段（扩大基线采集） |
| `2` | 参数非法 | 按提示改参数 |
| `3` | `blocked`（SDK 缺失 / 控制器不可达 / 外部条件不具备） | 按报告里的 `next_action` 处理 |
| `4` | `error`（控制链路或判定未通过） | **先人工确认臂位姿**，再查报告里的 `estop` 阶段 |

报告（`--out`）结构：`schema=rmctl.rm75_safe_probe/1`，`phases[]` 逐段给
`status/detail/data`，`evidence[]` 给失败项与可粘贴的 `next_action`。

---

## 5. 2026-11-19 首轮实测结果（远端主机 172.16.119.199）

### 5.1 连通性

| 项 | 结果 |
|---|---|
| `ping 192.168.1.18` | 0% 丢包，rtt 0.456 / 0.516 ms |
| `TCP 192.168.1.18:8080` | 连接成功 |
| `rmctl doctor` 的 `hw:sdk` / `hw:symbols` / `hw:contract` | 三项全 PASS（22 核心 + 21 支持符号全部可解析，静态审计 155 个 .py 零违规） |
| `rmctl connect --arm left --ip 192.168.1.18 --json` | `status=ok`，R004 全字段快照可读 |

### 5.2 最小动作验证

真机初始位姿（快照）：
`J1=-0.587° J2=42.126° J3=19.493° J4=-22.689° J5=-1.241° J6=5.320° J7=-200.538°`
（控制器 `RM_IDLE_E` / `RM_NO_PLANNING_E`，7 轴全部使能，关节错误码全 0，温度 30~32 ℃）。

| 通道 | 去程到位误差 | 其它关节最大漂移 | 回程偏差 | 判定 |
|---|---|---|---|---|
| `movej`（`rm_movej`，2% 速度） | **0.046°** | **0.002°** | **0.042°** | PASS |
| `canfd`（`rm_movej_canfd` 透传） | **0.167°** | **0.004°** | **0.159°** | PASS |

结论：**本项目的控制接口可以正常控制该 RM75-B**（连接、状态采集、安全开关、关节角
规划运动、关节角透传、停机族全部可用），且两轮结束时臂均回到初始位姿。

### 5.3 尚未覆盖（**不得**据此声称已实测）

| 项 | 状态 |
|---|---|
| UDP 主动上报质量（周期 / 丢包 / 抖动） | 只取到个位数帧，未做 10 分钟长跑（`rmctl.hwcollect udp_stability --duration-s 600`） |
| 加速度 / jerk 实测上限（`ladder_acc_jerk`） | 未执行 |
| TCP 重复性（`tcp_repeatability`） | 未执行 |
| 零位取证 / FK 一致性（`zero_capture` / `fk_cross`） | 未执行 |
| 控制器固件 / 示教器版本 | 未读取 |
| 右臂 `192.168.1.19` | 本轮完全未接触 |

因此 `workcell/baselines/*.yaml` 四张基线**仍为 `status=blocked`**（修订号升到 3，
并把"不可达"改成"连通性已恢复、采集项未做"）。

---

## 6. 常见问题

| 现象 | 查什么 |
|---|---|
| `blocked`：找不到 `libapi_c.so` | `RM_API2_LIB` 或 `<repo>/.deps/rm_api2/`；`file <so>` 确认架构是 x86-64 |
| `error`：前置检查 `controller_idle` 不过 | 控制器正在跑别的轨迹（示教器上操作过）→ 等它停稳再跑 |
| `error`：前置检查 `all_joints_enabled` 不过 | 有轴掉使能 → 现场复位后再跑，**不要**强行运动 |
| `error`：`rm_movej_canfd` 返回非 0 | 同一 `ip:port` 被别的客户端占用（上一进程没退干净） |
| 去程到位误差偏大（canfd 通道） | 控制器按自身速度曲线执行，透传通道精度低于规划通道属正常；容差内即 PASS |
