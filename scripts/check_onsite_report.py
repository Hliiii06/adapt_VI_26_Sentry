#!/usr/bin/env python3
"""校验 onsite_inspect 生成的 report.json（现场/回归用，失败返回非零）。

用法示例：

    python3 scripts/check_onsite_report.py --report log/onsite/<ts>/report.json \\
        --require-topic /cloud_registered --require-topic /Odometry_transformed \\
        --require-tf-static --require-tf "base_link -> base_footprint"

判据：
  * 每个 `--require-topic` 的实测消息数 ≥ `--min-count`（默认 1）；
  * `--require-tf-static`：`/tf_static` 至少收到一条变换；
  * `--require-tf`：给定 `parent -> child` 出现在 `/tf` 或 `/tf_static` 里。
"""

import argparse
import json
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    parser.add_argument("--require-topic", action="append", default=[])
    parser.add_argument("--require-tf", action="append", default=[])
    parser.add_argument("--require-tf-static", action="store_true")
    parser.add_argument("--min-count", type=int, default=1)
    args = parser.parse_args(argv)

    try:
        with open(args.report) as handle:
            report = json.load(handle)
    except Exception as exc:
        print("FAIL: 无法读取报告 %s: %s" % (args.report, exc))
        return 2

    channels = {c["topic"]: c for c in report.get("channels", [])}
    tf_all = set(report.get("tf", {})) | set(report.get("tf_static", {}))
    failures = []

    for topic in args.require_topic:
        channel = channels.get(topic)
        if channel is None:
            failures.append("%s: 报告里没有这个条目" % topic)
            continue
        if channel.get("count", 0) < args.min_count:
            failures.append("%s: 只收到 %d 条消息（要求 ≥ %d）"
                            % (topic, channel.get("count", 0), args.min_count))
    for pair in args.require_tf:
        if pair not in tf_all:
            failures.append("TF 缺失: %s" % pair)
    if args.require_tf_static and not report.get("tf_static"):
        failures.append("/tf_static 一条都没收到（采集器后加入时必须用 transient_local 订阅）")

    if failures:
        print("FAIL:")
        for item in failures:
            print("  - %s" % item)
        return 2
    print("PASS: 报告满足要求（topics=%d, tf=%d, tf_static=%d）"
          % (len(channels), len(report.get("tf", {})), len(report.get("tf_static", {}))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
