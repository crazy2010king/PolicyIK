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
| **一键跑整套（默认只读，臂不动）** | `bash scripts/run_rm75_probe.sh --ip 192.168.1.18` |
| **一键跑整套（含最小动作）** | `bash scripts/run_rm75_probe.sh --ip 192.168.1.18 --with-motion --joint 0 --delta-deg 0.3 --speed-pct 2` |
| 离线预演（零 SDK 调用） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --dry-run` |
| **只读体检**（零运动） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only` |
| 最小动作验证（0.3°） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej --joint 0 --delta-deg 0.3 --speed-pct 2` |
| 透传通道验证（0.3°） | `python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel canfd --joint 0 --delta-deg 0.3` |
| 落盘报告 | 加 `--out build/reports/rm75_safe_probe.json` |
| **报告速览**（只读文件） | `python3 scripts/rm75_probe_report.py build/reports/rm75_safe_probe*.json` |
| 离线单测 | `python3 -m pytest dualarm_ws/src/rmctl/tests/test_rm75_safe_probe.py dualarm_ws/src/rmctl/tests/test_rm75_probe_runner.py dualarm_ws/src/rmctl/tests/test_rm75_probe_report.py -q` |

> **单独运行的完整操作手册**（环境准备、四种测试方法、判读、应急）见 §7；
> **一键脚本**的用法见 §8。

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
| `movej`（`rm_movej`，2% 速度） | **0.042° ~ 0.046°**（3 轮复跑） | **0.002° ~ 0.004°** | **0.042° ~ 0.045°** | PASS |
| `canfd`（`rm_movej_canfd` 透传） | **0.167° ~ 0.178°**（2 轮复跑） | **0.004°** | **0.147° ~ 0.159°** | PASS |

结论：**本项目的控制接口可以正常控制该 RM75-B**（连接、状态采集、安全开关、关节角
规划运动、关节角透传、停机族全部可用），且每轮结束时臂均回到初始位姿。

> 透传通道（`canfd`）的误差明显大于规划通道（`movej`）—— 透传由控制器按自己的速度
> 曲线执行，规划通道则由 `rm_movej` 走控制器内部插值；两者都在 0.2° 容差内。

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

---

## 7. 单独运行指南（完整操作手册）

### 7.1 先明确「在哪里跑」

机械臂 `192.168.1.18` 在**远端主机 `172.16.119.199` 的 192.168.1.0/24 网段**上
（本地开发机 ping 不到它；2026-11-19 实测：远端 ping 0% 丢包 / rtt 0.46 ms，本地不可达）。

本地与远端是**同一份 NFS 文件**，所以脚本只需要在远端执行，代码不用同步：

```bash
ssh -p 2224 zy@172.16.119.199
cd /home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.gsd-worktrees/M001-3kzz40
```

> 若在 `ros_dev` 容器里跑：TCP 靠 docker NAT 可以通，但 **UDP 主动上报（8091）到不了容器**
> （报文发给宿主机的 192.168.1.198），所以**推荐直接在宿主机上跑**。

### 7.2 环境准备（每次新开 SSH 都要重跑）

```bash
# 1) RM-API2 1.1.7 动态库（x86-64）
export RM_API2_LIB=/home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.deps/rm_api2/libapi_c.so

# 2) PYTHONPATH（探针只依赖硬件适配层，stdlib + pyyaml，**不需要 tesseract**）
export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PWD/dualarm_ws/src/rm_safety:$PWD/dualarm_ws/src/rmctl

# 3) 三项自检
python3 -c "import ctypes;print('SDK:',ctypes.CDLL('$RM_API2_LIB').rm_api_version())"
ping -c 2 192.168.1.18          # 期望 0% 丢包
ss -ulnp | grep 8091            # 期望无输出（UDP 上报端口空闲；同一 ip:port 只允许一个客户端）
```

> **为什么 SDK 放主 checkout 的 `.deps/` 而不是 worktree 的 `.deps/`**：
> worktree 里一旦存在 `.deps/rm_api2/libapi_c.so`，既有单测
> `test_gripper_probe_real_path_blocked_without_sdk`（断言"无 SDK → blocked(3)"）
> 就会变成"真去连臂 → error(4)"而变红。用 `RM_API2_LIB` 指过去最干净。
> §8 的一键脚本会自动按 `$ROOT/.deps/rm_api2/` → `../../.deps/rm_api2/` → `../.deps/rm_api2/` 顺序自寻。

### 7.3 四种测试方法（安全度从低到高，**必须按顺序**）

#### 方法 A｜离线预演 `--dry-run` — 零 SDK 调用，零风险

只校验参数与生成运动计划，**完全不连真机**。改参数后先跑它。

```bash
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --dry-run
```
```
RM75-B 安全探针 | 状态=dry_run | 臂=left | 端点=192.168.1.18:8080
  [PASS] envelope     安全包络校验通过（未发出任何 SDK 调用）
  [PASS] plan         计划：关节1 位移 +0.300°，通道 movej（速度 2%），去程-回程对称
```
**判据**：`status=dry_run` + 退出码 `0`；若报 `envelope` 失败 → 参数越界（见 §7.5）。

#### 方法 B｜只读体检 `--precheck-only` — 连真机，**零运动**

验证"连得上、读得懂、臂是空闲且健康的"，**一条运动指令都不发**。

```bash
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only
```

7 项前置检查（任一不过 → **不运动**，退出码 4）：

| 检查项 | 判据 | 不过时的含义与处理 |
|---|---|---|
| `arm_err_empty` | 臂级错误码全 0 | 控制器报臂级错误 → 示教器清错后重跑 |
| `joint_err_code_zero` | 7 轴错误码全 0 | 某轴报错 → 现场处理，**不要**硬跑 |
| `all_joints_enabled` | 7 轴使能位全 1 | 有轴掉使能 → 重新使能 |
| `controller_idle` | 规划类型 = `RM_NO_PLANNING_E` | 控制器正跑别的轨迹（示教器操作过）→ 等它停稳 |
| `joints_readable` | 读到 7 个关节角 | 读不到 → 检查使能与网络 |
| `within_limits` | 当前位姿离限位 ≥1° | 贴近限位 → 不要往该方向动 |
| `target_within_limits` | 目标位姿离限位 ≥1° | 同上 |

**判据**：`7/7 通过` + `status=ok` + 退出码 `0`；报告里有 `motion` 阶段写着"未发出任何运动指令"。

#### 方法 C｜最小动作验证 `--channel movej` — 关节空间规划运动

验证 `rm_movej` 通道能真正把臂带到目标角。

```bash
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej \
        --joint 0 --delta-deg 0.3 --speed-pct 2 \
        --out build/reports/rm75_safe_probe.json
```

流程：`连接 → 只读快照 → 7 项前置检查 → 武装安全开关（碰撞等级 8 + 自碰撞使能）
→ 去程 +0.3° → 回读 → 回程 → 回读 → 关闭句柄`。

**判据**（实测口径）：

| 指标 | 容差 | 2026-11-19 实测 |
|---|---|---|
| 去程到位误差 | ≤0.2° | **0.045°** |
| 其它关节最大漂移 | ≤0.1° | **0.004°** |
| 回程归位偏差 | ≤0.2° | **0.045°** |

#### 方法 D｜关节角透传验证 `--channel canfd` — 项目 10 ms 单时钟主通道

验证 `rm_movej_canfd`（纯关节角透传、不做逆解、不在黑名单）这条主通道。

```bash
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel canfd \
        --joint 0 --delta-deg 0.3 \
        --out build/reports/rm75_safe_probe_canfd.json
```

与 C 的差别：透传是**非阻塞**的，探针改为**轮询回读**判定到位（默认上限 30 s）。
透传通道由控制器按自身速度曲线执行，精度低于规划通道属正常 —— 实测到位误差 **0.167°**、
回程 **0.159°**，仍在容差内 → PASS。

### 7.4 参数速查

| 参数 | 默认 | 含义 | 安全上限 |
|---|---|---|---|
| `--ip` | 192.168.1.18 | 控制器 IP | — |
| `--port` / `--udp-port` | 8080 / 8091 | TCP / UDP 上报端口 | — |
| `--joint` | 0 | 要微动的关节（0 基，0=J1 … 6=J7） | 只能 1 个 |
| `--delta-deg` | 0.3 | 微动角度（可负） | **≤1.0°**，越界 0 次 SDK 调用即拒 |
| `--speed-pct` | 2 | `rm_movej` 速度百分比 | **≤10** |
| `--channel` | movej | `movej` / `canfd` | — |
| `--precheck-only` | 关 | 只读模式 | — |
| `--dry-run` | 关 | 离线预演 | — |
| `--no-realtime` | 开→可关 | 关掉 UDP 主动上报 | — |
| `--out` / `--json` | — | 报告落盘 / stdout 只出 JSON | — |

### 7.5 判读与证据落盘

报告 JSON（`schema=rmctl.rm75_safe_probe/1`）：

```
phases[]   : snapshot → preflight → arm_safety → move_out → verify_out → move_back → verify_back
             （异常时额外出现 estop 阶段），每段含 status / detail / data
evidence[] : 失败项 + 可粘贴的 next_action
params     : 本次用的关节/角度/速度/通道/容差
```

取关键数字（用速览小工具，只读文件、不碰硬件）：

```bash
python3 scripts/rm75_probe_report.py build/reports/rm75_safe_probe.json
```

它会把各阶段的关键判定（到位误差 / 漂移 / 回程偏差 / 急停记录）打成一览表。

| 退出码 | 含义 | 该做什么 |
|---|---|---|
| 0 | `ok` / `dry_run` | 进入下一阶段 |
| 2 | 参数非法 | 改参数 |
| 3 | `blocked`（SDK 缺失 / 臂不可达） | 按 `next_action` 处理后重跑 |
| 4 | `error`（检查未过 / 运动失败 / 回读超差） | **先人工确认臂位姿**，再看 `estop` 阶段 |

### 7.6 异常与人工急停

- **自动急停**：探针任何一步失败/超时/回读超差 → 立即调用 `rm_set_arm_stop`（L4），报告里出现 `estop` 阶段。
- **人工急停**：以**示教器/物理急停按钮**为准。项目里的 `rmctl safety stop --level L4`
  只有在 `rm_executor` runtime 起着 socket（`build/run/rm_runtime.sock`）时才有服务端可连，
  **探针本身不启闸门**，不要指望它。
- **急停后**：需在**示教器上清错并重新使能**，项目不自动清错、不自动续跑。

### 7.7 其它「相关控制接口」测试方法（项目已有，按风险排序）

| 命令 | 是否运动 | 风险 | 说明 |
|---|---|---|---|
| `python3 -m rmctl doctor --json` | 否 | 无 | 体检；看 `hw:sdk` / `hw:symbols` / `hw:contract`（需容器 drake_dev 环境） |
| `python3 -m rmctl connect --arm left --ip 192.168.1.18 --json` | 否 | 无 | R004 全字段状态快照（含控制器位姿，仅作交叉校验） |
| `python3 -m rmctl.hwcollect udp_stability --arm left --duration-s 600 --out <dir>` | 否 | 无 | UDP 上报周期/丢包/抖动（10 分钟长跑，**补齐 network_plan 基线就靠它**） |
| `python3 -m rmctl.hwcollect fk_cross --arm left --samples 20 --out <dir>` | 否 | 无 | 自研 FK vs 控制器位姿交叉校验 |
| `python3 -m rmctl.hwcollect tcp_repeatability --arm left --repeats 30 --out <dir>` | **是** | 中 | ⚠️ **默认目标点是全零位 `[0,0,0,0,0,0,0]`**，从当前位姿（如 J7=-200°）回零是大角度运动 —— 跑之前必须清场 + 人工看护，必要时用 `--target` 指定接近当前位姿的角度 |
| `python3 -m rmctl.hwcollect ladder_acc_jerk --arm left --trials 3 --out <dir>` | **是** | **高** | ⚠️ 单关节阶梯位移 **10°/档**、逐档提速 —— 属于「大角度 + 大力矩」，**必须人工现场看护**，建议等安全探针与基线齐备后再做 |
| `python3 -m rmctl.hwcollect estop_drill --arm left --out <dir>` | **是（急停）** | **高** | ⚠️ 主动触发缓停 + 急停；跑之前必须过项目自带的 4 条 checklist（半径内无人、急停可触、示教器在手边、恢复流程已确认） |
| `python3 -m rmctl.hwcollect zero_capture --arm left --photo <照片> --out <dir>` | 否 | 无 | 零位读数 + 人工照片取证 |

> 所有 `hwcollect` 实验都支持 `--dry-run`（合成数据，报告里会显式标注「不是真机读数，禁止回填基线」）。
> **纪律**：hwcollect 的输出单独落盘，**不要直接覆盖 `workcell/baselines/*.yaml`**；
> 基线回填要走 `docs/code_doc/真机基线与实测限值归档说明.md` 规定的窗口流程。

### 7.8 完整「一次安全验证」复制粘贴块

```bash
ssh -p 2224 zy@172.16.119.199
cd /home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.gsd-worktrees/M001-3kzz40
export RM_API2_LIB=/home/zy/nfs/test_cc_dev/git_source/groot_nvidia/PolicyIK_roi/.deps/rm_api2/libapi_c.so
export PYTHONPATH=$PWD/dualarm_ws/src/rm_hw_bridge:$PWD/dualarm_ws/src/rm_safety:$PWD/dualarm_ws/src/rmctl

python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --dry-run                       # A
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --precheck-only                  # B
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel movej --joint 0 \
        --delta-deg 0.3 --speed-pct 2 --out build/reports/rm75_safe_probe.json        # C
python3 scripts/rm75_safe_probe.py --ip 192.168.1.18 --channel canfd --joint 0 \
        --delta-deg 0.3 --out build/reports/rm75_safe_probe_canfd.json               # D
```

离线单测（不碰真机，任何机器都能跑）：

```bash
python3 -m pytest dualarm_ws/src/rmctl/tests/test_rm75_safe_probe.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_probe_runner.py \
                 dualarm_ws/src/rmctl/tests/test_rm75_probe_report.py -q
```

---

## 8. 一键脚本 `scripts/run_rm75_probe.sh`

把 §7 的整套流程（定位 SDK → 连通性检查 → 离线预演 → 只读体检 → 可选最小动作）编排成一条命令。

### 8.1 用法

```bash
# 默认：只读（臂不会动）
bash scripts/run_rm75_probe.sh --ip 192.168.1.18

# 含最小动作
bash scripts/run_rm75_probe.sh --ip 192.168.1.18 --with-motion \
        --joint 0 --delta-deg 0.3 --speed-pct 2

# 透传通道
bash scripts/run_rm75_probe.sh --ip 192.168.1.18 --with-motion --channel canfd
```

### 8.2 脚本保证的安全性质

| # | 性质 | 实现 |
|---|---|---|
| 1 | **默认不动臂** | 不带 `--with-motion` 时只跑预演 + 只读体检；运动必须显式开 |
| 2 | **顺序不可跳过** | 连通性 → 预演 → 体检 → （可选）运动；前一步失败即退出，退出码如实透传 |
| 3 | **SDK 缺失即 blocked(3)** | 找不到 `libapi_c.so` → 退出码 3 + 可粘贴修复动作，**一步探针都不跑** |
| 4 | **参数越界由探针拒绝** | 位移 >1.0° / 速度 >10% 在 0 次 SDK 调用前被拒 |
| 5 | **报告留痕** | 运动步骤自动落盘 `build/reports/rm75_safe_probe_<时间戳>.json` |

### 8.3 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `--ip` / `--port` / `--udp-port` | 192.168.1.18 / 8080 / 8091 | 控制器地址与端口 |
| `--joint` / `--delta-deg` / `--speed-pct` | 0 / 0.3 / 2 | 微动关节、角度（≤1.0°）、速度（≤10） |
| `--channel` | movej | `movej` / `canfd` |
| `--out-dir` | `<repo>/build/reports` | 报告落盘目录 |
| `--with-motion` | 关 | **执行最小动作**（不加则臂不动） |
| `--skip-precheck` | 关 | 跳过只读体检（不建议；仅调试） |
| `--skip-net-check` | 关 | 跳过 ping/TCP 检查（**仅离线自检 / CI**，真机跑不要加） |
| `--sdk-lib` | 自寻 | 指定 `libapi_c.so` |

自寻顺序：`$RM_API2_LIB` → `$ROOT/.deps/rm_api2/libapi_c.so` → `$ROOT/../../.deps/rm_api2/` → `$ROOT/../.deps/rm_api2/`。

### 8.4 实测（2026-11-19 远端）

| 模式 | 结果 |
|---|---|
| 默认（只读） | SDK 自寻命中 `../../.deps/rm_api2/libapi_c.so`；ping OK / TCP OK / UDP 8091 空闲；预演 dry_run；体检 7/7 通过；**臂未运动**；退出码 0 |
| `--with-motion` | 到位误差 **0.046°**、漂移 **0.004°**、回程 **0.045°** → PASS；报告落盘 `build/reports/rm75_safe_probe_20261119-164103.json`；退出码 0 |

### 8.5 脚本的离线契约测试

`dualarm_ws/src/rmctl/tests/test_rm75_probe_runner.py`（7 条，用假解释器断言调用序列）：
默认不运动、参数顺序、SDK 缺失即 blocked 且零调用、体检失败不进运动、失败退出码透传、未知参数退出码 2。

### 8.6 报告速览小工具 `scripts/rm75_probe_report.py`

把探针报告里各阶段的关键判定数字打成一览表（纯读文件，不碰硬件）：

```bash
python3 scripts/rm75_probe_report.py build/reports/rm75_safe_probe*.json
```

退出码：0 全部 ok / 3 存在 blocked / 4 存在 error 或报告不可读。

