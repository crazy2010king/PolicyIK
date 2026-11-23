"""S12-T7 结案断言：校验一键回归报告的汇总段语义（只含 fail/blocked、每段含
id/reason/next_action、顶层 status/exit_code 与段 verdict 一致）。

离线自检脚本，不连真机、不改动任何被测代码。
"""
import json
import sys

REPORT = "build/reports/s12_regression.json"
SUMMARY_KEY = "problem_segments"

report = json.load(open(REPORT, encoding="utf-8"))

# 1) 顶层必须带汇总键
assert SUMMARY_KEY in report, "顶层缺少 %s 汇总键" % SUMMARY_KEY
summary = report[SUMMARY_KEY]

# 2) 汇总段**只含** fail / blocked
for item in summary:
    assert item["verdict"] in ("fail", "blocked"), (
        "汇总段混入了非 fail/blocked 段: %s" % item
    )

# 3) 每段必须含 id / reason / next_action
for item in summary:
    for field in ("id", "reason", "next_action"):
        assert item.get(field), "汇总段 %s 缺字段 %s" % (item.get("id"), field)

# 4) 段级 verdict 清单
segments = report["segments"]
verdicts = {s["id"]: s["verdict"] for s in segments}
problem_ids = {item["id"] for item in summary}
expected_problem_ids = {
    sid for sid, v in verdicts.items() if v in ("fail", "blocked")
}
assert problem_ids == expected_problem_ids, (
    "汇总段集合与实际 fail/blocked 段不一致: %s != %s"
    % (sorted(problem_ids), sorted(expected_problem_ids))
)

# 5) 顶层 status / exit_code 与段 verdict 一致
if "fail" in verdicts.values():
    assert report["status"] == "error", report["status"]
    assert report["exit_code"] == 4, report["exit_code"]
else:
    if "blocked" in verdicts.values():
        assert report["status"] == "blocked", report["status"]
        assert report["exit_code"] == 3, report["exit_code"]
    else:
        assert report["status"] == "ok", report["status"]
        assert report["exit_code"] == 0, report["exit_code"]

# 6) 顶层 reason 必须**指名**问题段 id（不得含糊）
for item in summary:
    assert item["id"] in report["reason"], (
        "顶层 reason 未指名问题段 %s: %s" % (item["id"], report["reason"])
    )

# 7) 退出码必须 ∈ {0, 3}（本次复跑不得出现 4）
assert report["exit_code"] in (0, 3), report["exit_code"]

print("segments=%d %s" % (len(segments), json.dumps(verdicts, ensure_ascii=False)))
print("summary_only_fail_blocked=%s ids=%s"
      % (True, sorted(problem_ids)))
print("top status=%s exit_code=%s reason=%s"
      % (report["status"], report["exit_code"], report["reason"]))
print("ASSERTIONS_OK")
