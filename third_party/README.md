# third_party/ —— 上游资产离线归档区

本目录承载**上游官方资产**（睿尔曼 rm_models、Two-Finger-Gripper 夹爪）的
commit 锁定与离线归档。核心约定：**入 git 的只有清单，不是二进制**。

---

## 1. 目录结构

```
third_party/
├── README.md        ← 本文件
├── MANIFEST.yaml    ← 入 git：每个资产的 commit / 归档文件名 / sha256 / 文件数 / 体积 / 许可
├── archives/        ← 不入 git：归档本体（本机镜像，gitignored）
│   ├── rm_models-<commit12>.tar.gz            （RM75/urdf/RM75-B/ 子集）
│   └── two_finger_gripper-<commit12>-metadata.tar.gz （文件清单 + 许可文件）
└── .work/           ← 不入 git：git 浅克隆临时工作目录（gitignored）
```

| 路径 | 入 git？ | 原因 |
|------|----------|------|
| `third_party/MANIFEST.yaml` | ✅ 是 | 只是 commit + sha256 文本清单，是「可复现凭据」 |
| `third_party/archives/**` | ❌ 否 | 上游未明示许可（rm_models）/ 官方网格，不得再分发 |
| `third_party/.work/**` | ❌ 否 | 临时克隆，可随时重建 |
| `models/**/meshes/**` | ❌ 否 | 官方 STL 网格不入 git；改造后的模型由 S01 自产后入库 |

---

## 2. 资产与许可约束

| 资产 | commit（锁定于 `versions.lock:upstream`） | 许可 | 可再分发 | 本仓处理 |
|------|-------------------------------------------|------|----------|----------|
| rm_models（RM75-B URDF + STL） | `cff49bb40f5ed0728505c03bff0e521a0dc21acd` | 未明示 | ❌ 否 | 只归档 `RM75/urdf/RM75-B/` 子集；官方网格不入 git，S01 改造（STL→OBJ、凸包化）后自产 |
| Two-Finger-Gripper | `91f587b1d4efaab235f3d227ee614b28f35060c9` | OpenAtom Open Hardware License 1.0 | ✅ 是 | 只归档元数据（文件清单 + 许可正文），不归档固件/PCB/CAD 等大体积二进制 |

相关但不属于本目录资产的许可（详见 `licenses/registry.yaml`）：

- **Mosek 11.1.2** —— 商业许可，限制再分发 → wheel 绝不入 git，只落 `vendor/wheels/` 本机镜像。
- **robotic_arm / RM_API2** —— 上游未明示许可 → 按最保守口径，只作设备层驱动依赖安装，不随产品再分发。
- **drake（BSD-3）** —— 自身可再分发，但其 wheel 内含商业求解器依赖 Mosek，故 drake wheel 也不入 git。

---

## 3. 复现命令

```bash
# 抓取全部资产（解析 commit → 稀疏拉取 → 产归档 → 写 MANIFEST → 回写 versions.lock）
python3 tools/fetch_upstream.py

# 只解析 ref → commit，不下载
python3 tools/fetch_upstream.py --list-remote

# 指定其它 ref（例如某个 tag）
python3 tools/fetch_upstream.py --ref v1.0

# 只处理一个资产
python3 tools/fetch_upstream.py --asset rm_models

# 按 MANIFEST 重算 sha256 校验（归档缺失或不匹配 → 退出码 1）
python3 tools/fetch_upstream.py --verify
```

归档可复现性：归档由 `git archive --format=tar.gz` 产生（mtime 取 commit 时间、gzip 无时间戳），
**同一 commit 反复抓取得到字节一致的归档**，因此 sha256 可以长期写进 `versions.lock`。

`versions.lock:upstream.*.archive.sha256` 与 `third_party/MANIFEST.yaml` 的 sha256 必须一致，
由 `tests/test_fetch_upstream.py` 断言。

---

## 4. 为什么不整仓归档

- rm_models 整仓 zip ≈ 248 MB（ECO62/ECO63/ECO65/RM65/RM75 全系列 + STEP + PDF），
  本项目只关心 RM75-B 一个机型。
- Two-Finger-Gripper 整仓 ≈ 44 MB，内含固件 `.exe`、Gerber、Keil 工程等与本项目无关的二进制。

整仓归档既无必要（只会撑爆仓库），也不合规（rm_models 许可未明示）。
因此策略是 **commit 锁定 + 子集归档 + sha256 清单**：
任何人拿到 commit 都能在几分钟内重建出同一批资产。

---

## 5. 与其它切片的关系

- **S01（模型）**：从 `third_party/archives/rm_models-*.tar.gz` 解出 RM75-B 的 URDF/网格，
  做定点改造（STL→OBJ、凸包化碰撞几何），产物入 `models/` 并在 `MANIFEST.yaml` 记录来源 commit。
- **S02（场景）**：夹爪参考信息（URDF 命名/关节语义）来自 Two-Finger-Gripper 的元数据清单，
  不复制固件与二进制。
