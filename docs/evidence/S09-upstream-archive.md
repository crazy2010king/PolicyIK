# S09 证据 · 上游归档物化与字节级校验闭合（R-3 / R019）

- 任务：S09-T01（切片 S09「证据闭合与需求口径校准」，里程碑 M001-acm1jg）
- 生成时间：2026-11-16 21:18:28 +0800
- 结论：**字节级校验成立**（`--verify` 退出码 0，两条归档 sha256 与
  `third_party/MANIFEST.yaml`、`versions.lock:upstream` 三方逐字一致）。**未走豁免口径。**
- 本机环境（fallback 档，非受支持基线）：Python 3.10.12 / git 2.34.1 / Ubuntu 22.04；
  Drake 侧基线信息见 `versions.lock`（baseline = py312 + Drake 1.55.0，fallback = py310 + Drake 1.51.1）。
  归档抓取与校验**不依赖 Drake**，故本条证据与 Drake 版本无关。

---

## 1. 物化结果（两条归档）

| 资产 | 文件名 | sha256 | file_count | total_bytes（解包未压缩） | 归档文件大小（tar.gz 落盘） | commit | 许可 |
|------|--------|--------|------------|--------------------------|----------------------------|--------|------|
| `rm_models` | `rm_models-cff49bb40f5e.tar.gz` | `fb7ab533ce42a146d676e95a58d115e2695909fe325e403bddc8f84313eb202b` | 17 | 3 312 746 B | 759 328 B | `cff49bb40f5ed0728505c03bff0e521a0dc21acd` | 未明示（不可再分发） |
| `two_finger_gripper` | `two_finger_gripper-91f587b1d4ef-metadata.tar.gz` | `975b5474afbac9de17bf2ea2601d6eacd0267ecdc54c0f46640115ac64d154a5` | 480 | 60 747 B | 14 196 B | `91f587b1d4efaab235f3d227ee614b28f35060c9` | OpenAtom Open Hardware License 1.0 |

落盘位置：`third_party/archives/`（已被 `.gitignore` 排除，第 11 行 `third_party/archives/`）。
浅克隆缓存：`third_party/.work/`（第 12 行 `third_party/.work/`），保留以便下次复用（复用后抓取耗时大幅下降）。

独立复核（不依赖工具自身，直接 `sha256sum`）：

```
$ sha256sum third_party/archives/*.tar.gz
fb7ab533ce42a146d676e95a58d115e2695909fe325e403bddc8f84313eb202b  third_party/archives/rm_models-cff49bb40f5e.tar.gz
975b5474afbac9de17bf2ea2601d6eacd0267ecdc54c0f46640115ac64d154a5  third_party/archives/two_finger_gripper-91f587b1d4ef-metadata.tar.gz
```

## 2. 逐字一致性结论（MANIFEST ↔ versions.lock ↔ 归档实际字节）

三方逐字一致，**sha256 / filename / file_count / total_bytes / commit** 五项全部相同：

| 字段 | `third_party/MANIFEST.yaml` | `versions.lock:upstream` | 归档实际字节 | 一致 |
|------|----------------------------|--------------------------|--------------|------|
| `rm_models.sha256` | `fb7ab533ce42…eb202b` | `fb7ab533ce42…eb202b` | `fb7ab533ce42…eb202b` | ✅ |
| `rm_models.archive.filename` | `rm_models-cff49bb40f5e.tar.gz` | 同左 | 落盘同名 | ✅ |
| `rm_models.archive.file_count` | 17 | 17 | `git archive` 条目 17 | ✅ |
| `rm_models.archive.total_bytes` | 3312746 | 3312746 | 3312746 | ✅ |
| `rm_models.commit` | `cff49bb4…acd` | `cff49bb4…acd` | `git rev-parse` 解析所得 | ✅ |
| `two_finger_gripper.sha256` | `975b5474afba…d154a5` | `975b5474afba…d154a5` | `975b5474afba…d154a5` | ✅ |
| `two_finger_gripper.archive.filename` | `two_finger_gripper-91f587b1d4ef-metadata.tar.gz` | 同左 | 落盘同名 | ✅ |
| `two_finger_gripper.archive.file_count` | 480 | 480 | `git ls-tree` 条目 479 + 许可正文 = 480 | ✅ |
| `two_finger_gripper.archive.total_bytes` | 60747 | 60747 | 60747 | ✅ |
| `two_finger_gripper.commit` | `91f587b1…60c9` | `91f587b1…60c9` | `git rev-parse` 解析所得 | ✅ |

**可复现性证明**：本次抓取在**全新 workdir**（`third_party/.work/` 此前不存在）上冷抓取完成，
产出 sha256 与仓库中既有记录（2026-11-13 由其他机器写入 `MANIFEST.yaml` 的值）**逐字节相同**，
说明 `git archive --format=tar.gz` 的产物在本机 git 2.34.1 上可复现 ⇒ 写进 `versions.lock` 的
sha256 可长期有效。

## 3. 物化与校验的命令 / 退出码 / 耗时

| # | 命令 | 退出码 | 耗时 |
|---|------|--------|------|
| 1 | `python3 tools/fetch_upstream.py` | **0** | 782 s（21:05:26 → 21:18:28）<br>· `rm_models`：浅克隆 + 归档 245.0 s<br>· `two_finger_gripper`：浅克隆 + 归档 537.3 s |
| 2 | `python3 tools/fetch_upstream.py --verify` | **0** | 0.04 s |
| 3 | `sha256sum third_party/archives/*.tar.gz` | 0 | < 0.1 s |
| 4 | `python3 -m pytest -q tests/test_fetch_upstream.py` | **0** | 5.28 s（19 passed / 0 failed / 0 skipped） |

`--verify` 输出（逐字）：

```
[fetch_upstream] OK  rm_models: rm_models-cff49bb40f5e.tar.gz sha256=fb7ab533ce42a146d676e95a58d115e2695909fe325e403bddc8f84313eb202b
[fetch_upstream] OK  two_finger_gripper: two_finger_gripper-91f587b1d4ef-metadata.tar.gz sha256=975b5474afbac9de17bf2ea2601d6eacd0267ecdc54c0f46640115ac64d154a5
[fetch_upstream] 校验通过：2 个资产
```

抓取端到端日志（关键行）：

```
[fetch_upstream] rm_models: commit=cff49bb40f5ed0728505c03bff0e521a0dc21acd
[fetch_upstream] rm_models: 归档 rm_models-cff49bb40f5e.tar.gz (17 文件 / 3312746 B / sha256=fb7ab533ce42… / 245.0s)
[fetch_upstream] two_finger_gripper: commit=91f587b1d4efaab235f3d227ee614b28f35060c9
[fetch_upstream] two_finger_gripper: 归档 two_finger_gripper-91f587b1d4ef-metadata.tar.gz (480 文件 / 60747 B / sha256=975b5474afba… / 537.3s)
[fetch_upstream] 已写清单 third_party/MANIFEST.yaml
[fetch_upstream] 已回写 versions.lock:upstream.*.archive
```

抓取后 `versions.lock` 的 `upstream.*.archive` 段**逐字未变**（`git diff versions.lock` 为空），
`third_party/MANIFEST.yaml` 仅 `generated_at` / `fetched_at` 三个时间戳字段更新（sha256 等实质字段未变）。

## 4. 入库凭据与复现配方（重要）

- **归档本体不入 git（上游许可限制）**：`rm_models` 上游许可未明示、官方网格不可再分发，
  因此 `third_party/archives/`（以及 `third_party/.work/`）被 `.gitignore` 排除，
  仓库中**只有凭据、没有二进制**。
- **入库凭据 = `third_party/MANIFEST.yaml` + `versions.lock:upstream`**：两者都记录
  commit、归档文件名、sha256、file_count、total_bytes，构成可核对的字节级指纹。
- **可复现配方 = `python3 tools/fetch_upstream.py`**：在有网机器上重跑即可重建同样的归档字节
  （本机已实测：冷抓取产出的 sha256 与既有记录逐字一致）。
- **校验配方 = `python3 tools/fetch_upstream.py --verify`**：按清单重算 sha256 并比对
  `versions.lock`，缺失 / 不匹配即退出码 1。

## 5. 顺带修复的真实缺陷（`tools/fetch_upstream.py`）

- **缺陷**：`cmd_fetch()` 末尾用 `manifest_path.relative_to(REPO_ROOT)` 渲染日志；
  当 `--manifest`（或 `--lock`）指向**仓库外**路径时，`Path.relative_to` 抛
  `ValueError: '…' is not in the subpath of '…'`，把一次本已成功的抓取变成失败。
- **修复**：新增模块级 `rel_to_repo(path)` —— 仓库内路径渲染为仓库相对路径，
  仓库外路径用 `os.path.relpath` 渲染成 `../…` 形式，**不再抛异常**；
  `已写清单` / `已回写` 两处日志统一走该函数。
- **回归用例**（`tests/test_fetch_upstream.py`）：
  - `test_fetch_with_manifest_outside_repo_does_not_raise`：用**本地 git 仓库**充当上游
    （commit 直接写成 40 位 hex ⇒ 全程不触网），`--manifest` 指向仓库外路径，
    断言退出码 0、清单被写出、日志里不含 `not in the subpath of`；
  - `test_rel_to_repo_handles_outside_path`：单元级断言仓库内/外两种路径的渲染形态。

## 6. R019 目标 Proof / Notes 措辞草案（供切片关闭时回填需求库）

- **目标 Proof（validation 文本）**：
  「按 `versions.lock:upstream` 锁定上游 commit（`rm_models` = `cff49bb40f5ed0728505c03bff0e521a0dc21acd`、
  `two_finger_gripper` = `91f587b1d4efaab235f3d227ee614b28f35060c9`）；在本机物化归档后
  `python3 tools/fetch_upstream.py --verify` 退出码 0 且输出「校验通过：2 个资产」，
  两条归档 sha256 逐字一致（`rm_models` = `fb7ab533ce42a146d676e95a58d115e2695909fe325e403bddc8f84313eb202b`，
  `two_finger_gripper` = `975b5474afbac9de17bf2ea2601d6eacd0267ecdc54c0f46640115ac64d154a5`），
  与 `third_party/MANIFEST.yaml`、`versions.lock:upstream` 三方交叉一致；
  归档本体不入 git（上游许可限制），入库凭据 = `third_party/MANIFEST.yaml` + `versions.lock:upstream`，
  可复现配方 = `python3 tools/fetch_upstream.py`。证据：docs/evidence/S09-upstream-archive.md。」
- **目标 Notes**：
  「字节级校验在本机已成立（2026-11-16 冷抓取产出 sha256 与既有记录逐字一致，见
  docs/evidence/S09-upstream-archive.md）；此前「本机未物化」的状态已于 S09 消除。
  `rm_models` 上游许可未明示、官方网格不可再分发，故归档与浅克隆目录（`third_party/archives/`、
  `third_party/.work/`）不入 git，只保留清单与 lock 指纹。」

## 7. 复算方式（可直接粘贴）

```bash
# 1) 物化（需要网络；复用 third_party/.work/ 缓存会显著更快）
python3 tools/fetch_upstream.py

# 2) 字节级校验（退出码 0 + 「校验通过：2 个资产」）
python3 tools/fetch_upstream.py --verify

# 3) 独立复核 sha256
sha256sum third_party/archives/*.tar.gz

# 4) 回归用例（含仓库外 --manifest 回归）
python3 -m pytest -q tests/test_fetch_upstream.py
```
