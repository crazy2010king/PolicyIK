# S03 真机 bring-up 手册（RM75-D 单臂）

> **适用范围**：把本仓库的设备层（`src/rm75_drake/device/`）从「零硬件可验证」
> 推进到「真机可执行」的**逐步操作手册**。
>
> **诚实前提**：本手册里的物理指标（5 ms 周期抖动、末端跟踪误差、急停响应、
> 30 min 长稳）在**没有真机**的环境里**无法验证**。所有脚本在真机不可达时
> 会输出 `SKIP(reason=hardware_unreachable)` 并**逐条列出未验证指标**——
> 绝不拿假数据冒充通过。

> ⚠️ **安全第一**：机械臂通电前必须确认 (1) 工作范围内无人；(2) 物理急停手柄
> 在手边且可拍；(3) 关节无卡死、无干涉；(4) 桌面与围栏参数已按现场标定。
> 软件急停（`rm_set_arm_emergency_stop`）**不能**替代物理急停。

---

## 0. 术语与真相源

| 内容 | 真相源 |
|---|---|
| 部署配置（IP / 端口 / 周期 / 零位 / 阈值 / 围栏） | `devices/single_arm.yaml` |
| 场景语义（起点 / 目标 / 障碍） | `scenarios/single_arm.yaml`（schema 已冻结） |
| 软件软限位 / 速度上限 | `models/rm75_drake/joint_limits.yaml` |
| SDK 版本与接口白名单 | `versions.lock`（`packages.robotic_arm` / `allowed_apis`） |
| 安全七闸阈值装配 | `src/rm75_drake/device/safety.py::safety_limits_from_sources` |
| 恢复流程（官方顺序） | `src/rm75_drake/device/recovery.py::RECOVERY_STEPS` |

---

## 1. 上电顺序（严格按序）

1. **机械臂基座上电**（控制器指示灯正常）。
2. **夹爪模块上电**（两指夹爪供电；若夹爪不在线 → `gripper_offline` 故障）。
3. **网线连接**：机械臂 ↔ 开发机（同一网段；官方默认机械臂 `192.168.1.18`，
   上报目标 IP 也是 `192.168.1.18`）。
4. **开发机网络**：`ip addr` 确认本机有 `192.168.1.x/24` 的地址。
5. **确认连通**：

   ```bash
   ping -c 3 192.168.1.18
   # TCP 指令通道（官方默认 8080）
   nc -vz 192.168.1.18 8080
   ```

6. **安装 SDK**（可选依赖，分发名 `robotic-arm`，**导入名 `Robotic_Arm` 大写 R/A**）：

   ```bash
   pip install -e '.[device]'      # 等价于 robotic-arm==1.1.7
   python3 -c "import Robotic_Arm; print('SDK ok')"
   ```

   装不上也不影响 CI：设备层是**懒加载**的，缺失时抛显式
   `NotAvailableError`，绝不静默降级。

---

## 2. 改 IP（`.18` / `.19`）

机械臂出厂默认 `192.168.1.18`；若现场用两台臂（`.18` / `.19`），逐台改：

| 项 | 改哪里 | 说明 |
|---|---|---|
| 机械臂自身 IP | 厂商调试工具 / 控制器面板 | 上位机连接的是**机械臂自身**的 IP |
| SDK 上报目标 IP | `rm_set_realtime_push` 的目标 IP 参数 | 必须是**开发机**的 IP，不是机械臂的 |
| 本项目配置 | `devices/single_arm.yaml` 的 `arm_ip` | 与机械臂自身 IP 一致 |

> ⚠️ 常见坑：把「上报目标 IP」填成机械臂 IP ⇒ 收不到 UDP 状态帧 ⇒
> 状态 age 超时 ⇒ 安全闸 G7 停机。

改完后**必须**回写 `devices/single_arm.yaml`（部署配置的唯一真相源），
并把 `name` 改成可区分的名字（如 `rm75_single_arm_b`）。

---

## 3. 三线程模式与 UDP 周期实测标定

### 3.1 为什么必须三线程模式

UDP **主动上报**（`rm_set_realtime_push` + `rm_realtime_arm_state_call_back`）
只在**三线程模式**下可用（`RM_TRIPLE_MODE_E`）。设备层在
`src/rm75_drake/device/transport.py::SdkTransport._init_arm` 里以三线程模式
初始化 `RoboticArm`；老版本 SDK 常量名不同时代码会**显式回退**并记录，
不会假装已生效。

### 3.2 `cycle` 的两种读法（必须实测，不采信二手数字）

官方 SDK 文档写「广播周期，5 ms 的倍数」，而官方 `ros2_rm_robot/rm_driver`
源码是 `config.cycle = udp_cycle / 5` ⇒ **cycle 的单位是 5 ms**
（5 ms 上报周期对应 `cycle = 1`）；另一种读法是「cycle 的单位是毫秒」
（`cycle = 5` 表示 5 ms）。两种读法**相差 5 倍**，直接决定 age 阈值与
抖动口径。

`DeviceConfig.udp_period_candidates_s()` 同时给出两种读法的候选周期。
**标定步骤**：

1. 在 `devices/single_arm.yaml` 里设 `udp_cycle`（先用读法 A：`1`）。
2. 真机透传运行一次，同时抓 UDP 帧到达时刻（可用 `tcpdump -i <iface> udp`，
   或在设备层日志里看状态帧的到达时间戳）。
3. 统计帧间隔分布（P50 / P99 / P99.9 / max）：
   * 若实测 ≈ **5 ms** ⇒ 读法 A 成立（`cycle=1`）；
   * 若实测 ≈ **25 ms** ⇒ 读法 B 成立（`cycle=1` ⇒ 1 ms？不 ⇒ 改 `udp_cycle=5`）。
4. 把结论写回 `devices/single_arm.yaml` 的注释里（**口径必须留痕**）。

> 帧完整性：`errCode == -3` 表示「推送数据不完整」⇒ 该帧**必须丢弃**，
> 不得进入状态机（见 `state_source.py`）。

---

## 4. 抖动测量（5 ms 透传周期）

```bash
# 真机：保持位姿 60 s，统计节拍抖动
python3 scripts/measure_jitter.py \
    --joint-rad "0 -0.6 0 0.9 0 0.5 0" \
    --duration-s 60 --period-ms 5 --json
```

* `--joint-rad` **必须显式给出**（7 个关节角，rad）——脚本绝不自己编造下发目标。
* 判定阈值：抖动 **P99.9 < 0.5 ms**（超了 → 退出码 1）。
* 无真机时：输出 `SKIP(reason=hardware_unreachable)` + 未验证指标清单，退出码 0。
* 离线演练（**不代表真机指标**）：

  ```bash
  python3 scripts/measure_jitter.py --dry-run --duration-s 5
  ```

---

## 5. 长稳（100 Hz × 30 min）

```bash
python3 scripts/soak_test.py \
    --joint-rad "0 -0.6 0 0.9 0 0.5 0" \
    --hz 100 --duration-s 1800 --json
```

* 关注：中途停机（`final_state != completed`）、安全七闸违规数、传输错误数、
  抖动 P99.9；
* 时长可参数化（现场先跑 5 min 冒烟，再跑满 30 min）；
* 离线演练默认 5 s（`--dry-run`），**不代表**真机长稳。

---

## 6. 故障注入与恢复（人工编排）

```bash
# 离线流程演练（替身）：五类故障逐条「注入 → 安全态 → 恢复 → READY」
python3 scripts/fault_inject.py --json

# 真机：按脚本打印的步骤人工注入，脚本检测 → 安全态 → 恢复 → 校验 READY
python3 scripts/fault_inject.py --hardware
```

五类故障与期望安全态：

| 故障 | 物理操作 | 期望 reason | 期望安全态 |
|---|---|---|---|
| 拔网线 | 拔掉机械臂网线 | `link_down` | `FAULT` |
| 物体掉落 | 夹爪上的物体掉落 | `gripper_offline` | `HOLDING` |
| 夹爪夹空 | 空夹 / 未夹到 | `gripper_not_at_target` | `HOLDING` |
| 关节超限位 | 手动撞限位 / 关节报错 | `joint_error` | `FAULT` |
| 供电跌落 | 断开供电 / 制造欠压（真机） | `power_supply_drop` | `FAULT` |

恢复流程（**官方顺序**，每一步都走传输层）：

```
verify_link → check_gripper → clear_error → relax_limits → enable
            → homing → restore_limits → resync_q → reset_ready
```

对应 SDK 符号（全部在 `versions.lock:allowed_apis` 白名单内）：
`rm_set_joint_clear_err` → `rm_clear_system_err` → `rm_set_joint_min_pos` /
`rm_set_joint_max_pos` → `rm_set_joint_en_state` → `rm_movej_canfd`（唯一运动通道）
→ `rm_set_joint_min_pos` / `rm_set_joint_max_pos` → 重同步 `q` → 复位 `READY`。

> `rm_clear_system_err` 清除的是**系统级**错误码（含供电跌落这类帧级系统错误），
> 逐轴错误码由 `rm_set_joint_clear_err` 清除——**两者都要清**，恢复才不会停在 `resync_q`。

任一步失败都会给出**可执行诊断**（失败符号 + 当前逐关节错误码 + 建议动作），
后续步骤显式标记 `skipped`——绝不静默跳过。

### 6.1 供电跌落（真机复验步骤）

供电跌落是**帧级系统错误码**承载的故障：状态帧 `errCode` 非 0（非 `-3`）时，
本项目判定为 `power_supply_drop`，进入 `FAULT` 安全态。

1. 确认急停手柄可用、周围无人、机械臂处于可控姿态；确认现场安全规程已就位。
2. 断开机械臂供电或制造欠压（**按现场安全规程操作**）。
3. 观察脚本 / CLI 是否检测到 `power_supply_drop` 并进入 `FAULT` 安全态
   （离线可先用 `python3 scripts/fault_inject.py --only power_supply_drop --json`
   确认判定与恢复链在替身上闭环）。
4. 恢复供电后按官方恢复顺序执行：
   清错 `rm_set_joint_clear_err` → `rm_clear_system_err` → 放宽限位 → 上使能 →
   回位 → 恢复限位 → 重同步 `q` → 复位 `READY`。
5. 记录断电时长与恢复耗时。

> **如实标注**：厂商帧级系统错误码（`errCode`）↔ 供电跌落的对应关系在硬件可达前
> **未核实**。当前实现以「帧级系统错误码非 0」承载该故障，
> `power_supply_drop` 是本项目自定的**稳定 reason 标签**，**不代表厂商错误码表**。
> 真机复验后必须回写实际错误码与判定规则（并同步 `src/rm75_drake/device/recovery.py`
> 与 `scripts/fault_inject.py` 中的判定）。真机物理验证在完成之前仍属「未验证指标」。

---

## 7. 急停响应与末端跟踪误差

| 指标 | 目标 | 测量入口 |
|---|---|---|
| 急停响应时间 | < 100 ms | 人工拍物理急停 + 时间戳录像；软件急停仅作兜底 |
| 末端跟踪误差 | < 3 mm | 真机执行 `python3 -m rm75_drake.cli exec --recording <file>`，比对末端轨迹 |

> 这两项当前**没有**自动化脚本入口（需要外部测量设备），
> 因此在 `versions.lock` 允许清单与运行日志里都被列为「未验证指标」，
> **不得**在验收报告里写成「通过」。

---

## 8. 一次完整的真机执行

```bash
# 规划 → rm_device 驱动 → 5 ms 执行环 → 全链路运行日志
python3 -m rm75_drake.cli exec \
    --device-config devices/single_arm.yaml \
    --recording /path/to/plan.json \
    --log-out run_logs/s03_exec_$(date +%Y%m%d_%H%M%S).json

# 离线先跑一遍（零硬件）
python3 -m rm75_drake.cli exec --recording /path/to/plan.json --dry-run
```

退出码：`0` 成功**或**真机不可达时的 `SKIP`；`1` 安全态触发 / 传输层失败；
`2` 用法错误。

---

## 9. 标定回写清单（真机首测后必须回写）

| 文件 | 字段 | 依据 |
|---|---|---|
| `devices/single_arm.yaml` | `arm_ip` / `udp_cycle` | 现场网络 + 实测周期分布 |
| `devices/single_arm.yaml` | `accel_limits.measured` | 实测逐轴加速度上限（当前 `false`） |
| `devices/single_arm.yaml` | `current_thresholds.calibrated` | 实测逐轴电流阈值（当前 `false`） |
| `devices/single_arm.yaml` | `fence.table_height_m` / `singularity_radius_m` | 现场桌面高度 / 奇异圆柱半径 |
| `models/rm75_drake/joint_limits.yaml` | `software` 段 | 实测软件软限位（若与出厂不一致） |

回写后**必须**跑一遍零硬件回归，确认没把七闸阈值改坏：

```bash
bash scripts/ci.sh fast
```

---

## 10. 常见故障速查

| 现象 | 可能原因 | 处置 |
|---|---|---|
| `NotAvailableError` | SDK 未安装 / 导入名写成小写 | `pip install -e '.[device]'`；导入名是 `Robotic_Arm` |
| `SKIP(reason=hardware_unreachable)` | SDK 缺失或 TCP 不通 | 看脚本给出的 detail；先 `ping` / `nc` |
| 状态 age 持续超限 | UDP 没收到帧 / 上报目标 IP 填错 | 回看 §2 / §3.2 |
| 关节「飞到 57 倍位置」 | 度当弧度下发 | 检查 rad↔deg 唯一转换点（`device/sdk.py`） |
| `gripper_offline` | 夹爪未上电 / 通信异常 | 检查夹爪供电与 `rm_plus_state` |
| 恢复停在 `check_gripper` | 夹爪仍离线 / 未到位 | 按诊断里的建议动作处理后重跑 |
| 恢复停在 `resync_q` | 帧不完整 / 错误码未清零 / 未全轴使能 | 按诊断里的建议动作处理后重跑 |
| `power_supply_drop`（或状态帧 `errCode` 非 0） | 供电跌落 / 欠压 / 保险丝熔断 / 电源模块故障 | 确认供电与保险丝/电源模块；恢复供电后按 §6 的恢复顺序清错（`rm_clear_system_err`）并重同步 `q`；恢复停在 `resync_q` 时检查帧是否仍带系统错误码 |

---

## 11. 真机实测结论（2026-11-19，现场 RM75-B / 192.168.1.18）

> 完整证据见 `docs/evidence/rm75-b-real-arm-ladder.md`；
> 逐级验证流程见 `docs/validation/RM75-B_真机验证阶梯.md`。
> 本节把**实测**结论回写到本手册，避免下一个人重复踩坑。

### 11.1 SDK 事实（与本手册 §3 的差异已按实测更正）

| 事项 | 本手册原写法 | **实测结论** |
|---|---|---|
| 线程模式常量 | — | `RM_TRIPLE_MODE_E = 2`（`RM_SINGLE=0 / RM_DUAL=1 / RM_TRIPLE=2`）；传 **3** 会被 SDK 打印 `Invalid thread mode` 且**拿不到句柄** |
| 建连接 | 只说 `rm_init` 三线程 | **必须**显式 `rm_create_robot_arm(ip, port)`；句柄 `id == -1` 即连接失败 |
| 调用形态 | 未写 | 厂商 API 是**实例方法**（`arm.rm_movej_canfd(...)`）；模块级同名符号是 ctypes 原函数，**签名不同**，直接调用必失败 |
| 逐关节接口 | 未写 | 使能 / 清错 / 限位都是**逐轴**调用（`joint_num` + 值）；夹爪是 `rm_set_gripper_position(pos, block, timeout)`；急停是 `rm_set_arm_emergency_stop(state)` |
| `cycle` 单位 | §3.2 写「两种读法待标定」 | **读法 A 成立**：`cycle` 单位是 **5 ms 的倍数**（`cycle=1` ⇒ 实测 P50 **4.998 ms**；`cycle=5` ⇒ 实测 24.954 ms） |
| 收尾 | 未写 | 删句柄 / 回调对象被 GC 后，SDK 的 UDP 线程会回调野指针 ⇒ **随机段错误**；回调对象须进程级保活，退出用 `os._exit` |
| 逐关节接口的轴号 | 未写 | **`joint_num` 是 1-based（1~7）**：传 0 ⇒ 控制器返回 1（失败）；我方曾按 0-based 调用 ⇒ 恢复流程的限位/使能/清错**全部打错轴**（假绿灯） |
| SDK 返回码 | 未写 | 控制器拒绝请求时返回 1（SDK 打印 `...: false`）且**不抛异常** ⇒ 不检查返回码就会把失败当成功 |
| 控制器出厂关节软限位 | 未写 | 实测：J1 ±125.998°、J2 ±130.0°、J3 ±130.988°、J4 ±135.0°、J5 ±123.998°、J6 ±128.0°、J7 ±360.0°；**J1/J3/J5/J6 比模型软限位更紧** ⇒ 有效限位必须取**交集**（`devices/*.yaml` 的 `controller_joint_limits` 段） |
| 写限位的操作纪律 | 未写 | 必须「**先读 → 改 → 写 → 读回校验**」四步；写限位前先 `rm_get_joint_min_pos/max_pos`，写完必须读回核对（我方曾把 J1 下限误钉在当前位形） |

### 11.2 现场这台臂的当前状态（2026-11-19）

| 事实 | 数值 | 影响 |
|---|---|---|
| 控制器代次 | **3**（三代替代机） | 与《设计文档》的开放问题 1 对上了 |
| 逐轴使能 | 7/7 已使能 | 但 `arm_current_status = 9`（**急停**）⇒ **能连上 ≠ 能动** |
| 静态保持电流 | J2 **-1824 mA**（另一时刻 -1413 mA） | 兜底阈值 1500 mA ⇒ **G5 会误触发**，必须先标定 |
| TCP 高度 / 桌面阈值 | 0.754 m / 0.75 m（占位值） | 余量仅 ~4 mm ⇒ 动之前先按现场标定 `fence` 段 |

### 11.3 与本手册的关系

本手册讲「**怎么把设备层接到真机上**」；
「**怎么在真机上按顺序把整套验证做完**」由
`docs/validation/RM75-B_真机验证阶梯.md`（L0–L14）与
`scripts/real_machine_ladder.py` 承担。
