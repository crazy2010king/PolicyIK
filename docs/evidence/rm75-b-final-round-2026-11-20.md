# 真机证据（机器可读）：RM75-B 2026-11-20 第三轮最终回归与长稳

> 采集时间：2026-11-20 13:12（+0800）／长稳 30 min 同期
> 对象：睿尔曼 RM75-B（7-DoF），现场 IP `192.168.1.18`，控制器代次 3；夹爪 WHJ03
> 执行环境：远端工作站 `172.16.119.199`（宿主网卡 `192.168.1.198/24`；规划在 `docker ros_dev`
> 的 `conda drake_dev`，硬件执行在宿主 venv），Python 3.12 + `robotic-arm==1.1.7`
> 采集工具：`scripts/real_machine_ladder.py` / `scripts/soak_test.py` / `scripts/eval_tracking.py` /
> `rm75_drake.cli exec` / `scripts/calibrate_currents.py`
> 性质：**机器可读原始产物**（本目录此前只有首轮 JSON 与叙述性纪要，第三轮的原始 JSON 未入库）

---

## 1. 本目录新增文件与各自证明什么

| 文件 | 证明什么 | 关键数字 |
|---|---|---|
| `rm75-b-ladder-final-2026-11-20.json` | **全阶梯 L0–L10 最终回归**（`mode=hardware`、`allow_motion=true`） | `summary = {pass: 11, fail: 0, skip: 0, blocked: 0}` |
| `rm75-b-ladder-after-recovery-2026-11-20.json` | 开发机到机械臂网卡掉线**恢复后**的只读级回归 | L0–L5 / L10 pass；L6–L9 因未允许运动判 blocked |
| `rm75-b-soak-30min-final-2026-11-20.json` | **100 Hz × 30 min 真机长稳** + 运行时 SDK 审计 | ticks=180000、commands_sent=180000、violations=0、errors=0、jitter P99.9=**0.3145 ms**（阈值 0.5）、command_latency P99.9=**0.2901 ms**、`sdk_audit.forbidden_call_count=0` |
| `rm75-b-tracking-final-2026-11-20.json` | **末端跟踪误差**评估（131 点配对） | `tcp_error_mm` p50=1.4416 / p95=2.4467 / **max=2.6620**（阈值 3.0）⇒ `status=pass`；关节误差 max 0.4915°（J1），其余轴 ≤ 0.006° |
| `rm75-b-runlog-final-2026-11-20.json` | 最终一次真机 `cli exec` 的**全链路运行日志** | `mode=hardware`、`dry_run=false`、events=265、samples=131、violations=0、jitter p999=0.2964 ms |
| `rm75-b-plan-final-2026-11-20.json` | 上述那次真机执行所下发的**规划轨迹**（recording schema v1） | 131 采样点、`dt=0.005` |
| `rm75-b-current-calibration-2026-11-20.json` | 电流阈值标定结果 | `status=ok`，`current_thresholds.calibrated=true`，逐轴值 `[6379.5, 3240.0, 1762.5, 2133.0, 600.0, 718.5, 600.0]` mA |
| `rm75-b-evidence-summary-2026-11-20.json` | 上述结果的**汇总口径** | `{"L8": {"exec_exit_code": 0, "tcp_tracking_error_mm": 2.6619684756546693}}` |

## 2. 逐级数字（`rm75-b-ladder-final-2026-11-20.json`）

| 级别 | 判定 | 关键实测值 |
|---|---|---|
| L0 链路与网关 | pass | TCP 连接 0.603 ms；示教器网页 HTTP 200 |
| L1 SDK 与控制器 | pass | SDK 1.1.7；`arm_dof=7`；控制器代次 3；句柄 id=1 |
| L2 连接与只读状态 | pass | 错误关节数 0；TCP 高 0.74759 m、半径 0.3263 m |
| L3 UDP 主动上报 | pass | 667 帧；P50 5.0027 ms / P99 5.3766 ms / P99.9 5.5 ms；不完整帧 0、非法帧 0 |
| L4 使能与就绪 | pass | 7 轴全使能；`arm_current_status=0`（急停已释放）；夹爪在线 |
| L5 运动前静态自检 | pass | 关节全在软限位内；围栏零违规；高度余量 **0.00462 m**（现场余量仅 ~4 mm，见 §4） |
| L6 单轴点动与回零 | pass | 点动误差 0.631°；回零误差 0.425°；错误码 0 |
| L7 5 ms 透传抖动 | pass | P50 0.0080 ms / P99 0.1348 ms / **P99.9 0.1952 ms** / max 0.2246 ms；违规 0 |
| L8 轨迹执行 + 末端跟踪 | pass | `exec_exit_code=0`；**末端误差 2.6620 mm**（阈值 3） |
| L9 夹爪 | pass | 张开 目标 1000 / 回读 1002；闭合 目标 0 / 回读 -2（误差 2） |
| L10 安全七闸 | pass | 7/7 闸拦下；真机越界目标被安全层拒绝（未下发） |

## 3. 长稳与运行时审计（`rm75-b-soak-30min-final-2026-11-20.json`）

- `mode=hardware`、`hz=100.0`、`requested_duration_s=1800.0`；
- `ticks=180000`、`commands_sent=180000`、`final_state=completed`、`stop_reason=completed`；
- `num_violations=0`、`num_errors=0`、`errors=[]`、`failures=[]`、`status=ok`；
- 抖动直方图：`count=179999`、`dropped=0`、P50 0.0230 ms、P99 0.2221 ms、**P99.9 0.3145 ms**、max 1.0066 ms（阈值 `threshold_p999_ms=0.5`）；
- 命令→执行延迟：`count=180000`、P99.9 **0.2901 ms**、max 1.7224 ms；
- **运行时 SDK 审计**：`total_call_count=180000`、`non_whitelist_call_count=0`、**`forbidden_call_count=0`**、`forbidden_calls=[]`
  ⇒ 厂商解算接口在 18 万次真实调用中**一次都没有被调用**（R008 运行时那半的硬证据）；
- 本轮生效的标定值：`accel_limits.measured=true`（`[1.0, 1.0, 1.2, 1.2, 1.2, 1.2, 1.2]`）、
  `current_thresholds.calibrated=true`（`[3200.0, 3154.5, 1576.5, 2005.5, 600.0, 681.0, 600.0]`）。

## 4. 诚实边界（必须随引用一起转述）

1. **单臂**：本轮全部为**单台** RM75-B；双臂级别（L13）与「两臂共同搬运」指标**未验证**（现场无第二台臂）。
2. **五类物理故障注入未做**：拔网线 / 物体掉落 / 夹爪夹空 / 关节超限位 / 供电跌落的**物理**注入未执行；
   替身层五类全链已通（见 `scripts/fault_inject.py`），真机仅「夹爪未到位链」在第一轮闭合。
3. **急停响应未验证**：厂商 `rm_set_arm_emergency_stop` 返回 `-4`，**软件急停无效**，必须物理急停；
   因此「急停响应 < 100 ms」在本轮 `unverified_metrics` 内。
4. **围栏高度余量仅 ~4 mm**（L5：0.00462 m）：现场围栏参数偏紧，属于**现场标定待办**，不是本次回归的失败项。
5. **加速度上限是标定后的收紧值**（1.0 / 1.2 rad/s²），与仓库默认（6.0 / 8.0）不同；引用末端误差时必须用同一份标定配置。
6. 本目录中的**离线类**数字（CI fast、禁用接口扫描、pytest 计数）属 fallback 档（Python 3.10.12 / Drake 1.51.1），
   与本文件的**真机类**数字（远端 Python 3.12 / Drake 1.55.0）**分属不同环境**，不可混说。

## 5. 复算入口

```bash
# 真机（需现场：物理急停在手、单臂、显式 --allow-motion）
python3 scripts/real_machine_ladder.py --to L10 \
    --device-config devices/rm75_b_192_168_1_18.yaml --allow-motion --json
python3 scripts/soak_test.py --device-config devices/rm75_b_192_168_1_18.yaml \
    --duration-s 1800 --hz 100 --json
python3 scripts/eval_tracking.py --run-log docs/evidence/rm75-b-runlog-final-2026-11-20.json --json

# 离线可复算（对照用）
python3 scripts/scan_forbidden_apis.py src tools scripts
bash scripts/ci.sh fast
```
