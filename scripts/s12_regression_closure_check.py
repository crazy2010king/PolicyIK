"""S12-T7 结案检查（端到端）：跑一键回归 → 校验进程退出码 ∈ {0,3} → 校验报告汇总段语义。

- 进程退出码断言：本脚本退出码 0 当且仅当「回归进程退出码 ∈ {0,3} 且与报告 exit_code
  一致 且 汇总段只含 fail/blocked 且每段含 id/reason/next_action 且顶层 reason 指名问题段」。
- 绝不改段结论、绝不改 status 去凑 0：回归段 verdict 保持原样。
- 离线：不连真机、不下发电机运动指令。

用法：python3 scripts/s12_regression_closure_check.py
"""
import json
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPORT = os.path.join("build", "reports", "s12_regression.json")
SUMMARY_KEY = "problem_segments"

# 1) 跑一键回归（退出码必须 ∈ {0,3}；4 = 真回归，不接受）
proc = subprocess.run(
    [sys.executable, "scripts/rm_regression.py", "--out", REPORT],
    cwd=REPO,
    capture_output=True,
    text=True,
)
rc = proc.returncode
print("regression_process_exit_code=%d" % rc)
assert rc in (0, 3), "回归退出码 %d 不在 {0,3}（4 表示真回归）" % rc

# 2) 报告必须存在且可解析（缺失 → 直接抛错，不伪造通过）
report_path = os.path.join(REPO, REPORT)
assert os.path.exists(report_path), "回归报告未落盘: %s" % REPORT
report = json.load(open(report_path, encoding="utf-8"))

# 3) 报告 exit_code 必须与进程退出码一致
assert report["exit_code"] == rc, (
    "报告 exit_code=%s 与进程退出码 %s 不一致" % (report["exit_code"], rc)
)

# 4) 顶层必须带汇总键，且**只含** fail / blocked 段
assert SUMMARY_KEY in report, "顶层缺少 %s 汇总键" % SUMMARY_KEY
summary = report[SUMMARY_KEY]
for item in summary:
    assert item["verdict"] in ("fail", "blocked"), (
        "汇总段混入非 fail/blocked 段: %s" % item
    )
    for field in ("id", "reason", "next_action"):
        assert item.get(field), "汇总段 %s 缺字段 %s" % (item.get("id"), field)

# 5) 汇总段集合 == 实际 fail/blocked 段集合（不得漏报、不得多报）
verdicts = {s["id"]: s["verdict"] for s in report["segments"]}
expected = {sid for sid, v in verdicts.items() if v in ("fail", "blocked")}
actual = {item["id"] for item in summary}
assert actual == expected, "汇总段集合与实际 fail/blocked 段不一致: %s != %s" % (
    sorted(actual),
    sorted(expected),
)

# 6) 顶层 status 与段 verdict 一致
if "fail" in verdicts.values():
    assert report["status"] == "error" and rc == 4
elif "blocked" in verdicts.values():
    assert report["status"] == "blocked" and rc == 3
else:
    assert report["status"] == "ok" and rc == 0

# 7) 顶层 reason 必须指名每个问题段 id
for item in summary:
    assert item["id"] in report["reason"], (
        "顶层 reason 未指名问题段 %s: %s" % (item["id"], report["reason"])
    )

print("segments=%d %s" % (len(report["segments"]), json.dumps(verdicts, ensure_ascii=False)))
print("summary_only_fail_blocked=True ids=%s" % sorted(actual))
print("top status=%s exit_code=%s reason=%s"
      % (report["status"], report["exit_code"], report["reason"]))
print("CLOSURE_CHECK_OK")
