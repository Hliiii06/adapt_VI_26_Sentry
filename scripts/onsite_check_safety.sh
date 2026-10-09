#!/usr/bin/env bash
# 现场安全自检：影子在跑的时候，确认它没有碰真实控制入口（失败返回非零）。
#
# 用法：
#   bash scripts/onsite_check_safety.sh idle      # 空转：影子输出应恒为零
#   bash scripts/onsite_check_safety.sh motion    # 已发目标：应能看到候选速度流过影子话题
#
# 检查项：
#   1. 影子图隔离：必需节点在跑；**影子命名空间内**没有任何 /cmd_vel、/cmd_vel_remap、
#      /sentry_scan/cmd_vel 发布者；外部（Nav2）发布者只统计、不判失败；
#   2. 影子速度的 angular.z / linear.z 恒为 0（不透传 yaw 与未定义分量）；
#   3. idle 模式下 /sentry_scan/cmd_vel_shadow 必须全零；motion 模式必须出现 ≥0.2 m/s；
#   4. 控制话题拓扑：用 onsite_control_audit.py 精确区分**发布者/订阅者**
#      （不要用 grep `ros2 topic info -v`，它会把订阅者如 uart_node 显示成发布者）。
#
# 不健康时先跑：bash scripts/onsite_diagnose.sh（打印每通道原因与拒绝日志）。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
MODE="${1:-idle}"
DURATION="${2:-6}"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "未找到 install/setup.bash，请先 source 好现场环境（或本仓库 install）" >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-${REPO_ROOT}/log/ros}"

RC=0
EXTRA=()
case "${MODE}" in
  idle)   EXTRA=(--expect-zero) ;;
  motion) EXTRA=(--expect-motion) ;;
  *) echo "用法: $0 [idle|motion] [duration]" >&2; exit 1 ;;
esac

echo "== 1/4 影子图与门控（mode=${MODE}）"
python3 scripts/check_shadow_graph.py --duration "${DURATION}" --expect-healthy "${EXTRA[@]}" || RC=$?

echo
echo "== 2/4 控制话题拓扑（发布者 vs 订阅者，精确区分）"
python3 scripts/onsite_control_audit.py || RC=2

echo
echo "== 3/4 影子命名空间是否出现在控制话题发布者名单（必须为 0）"
if python3 scripts/onsite_control_audit.py > /dev/null 2>&1; then
  echo "   OK：影子没有出现在任何控制话题发布者名单"
else
  echo "   FAIL: 影子节点出现在了真实控制话题上（详见上面 2/4）" >&2
  RC=2
fi

echo
echo "== 4/4 影子速度的未定义分量（angular.z / linear.z 必须为 0）"
python3 scripts/check_shadow_graph.py --duration 3 --expect-healthy "${EXTRA[@]}" \
  | grep -E "max\|wz\||shadow samples" || true

if [[ "${RC}" -ne 0 ]]; then
  echo
  echo "== FAIL (rc=${RC})：先不要继续。"
  if [[ "${MODE}" == "motion" ]]; then
    echo "   注意：motion 模式需要**先发目标**（python3 scripts/onsite_send_goal.py --frame odom --x 2 --y 0）。"
  fi
  echo "   下一步先跑：bash scripts/onsite_diagnose.sh（会打印 health 每通道原因、适配器拒绝日志、门控原因、控制话题拓扑）"
  echo "   把诊断输出 + log/onsite/<时间戳>/report.txt 一起回传。"
else
  echo
  echo "== PASS：影子只读、未接入真实控制入口（mode=${MODE}）"
fi
exit "${RC}"
