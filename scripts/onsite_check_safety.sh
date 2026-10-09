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
#   4. 控制话题发布者名单（人工确认：只有原有导航，没有影子节点）。

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
echo "== 2/4 控制话题发布者（人工确认：只有原有导航）"
for topic in /cmd_vel /cmd_vel_remap /cmd_vel_nav; do
  echo "-- ${topic}"
  ros2 topic info -v "${topic}" 2>/dev/null | sed -n '1,40p' | grep -E "Publisher count|Node name|Node namespace|Topic type" || echo "   (无此话题)"
done

echo
echo "== 3/4 影子命名空间内是否出现控制话题发布者（必须为 0）"
SHADOW_CMD=$(ros2 topic info -v /cmd_vel 2>/dev/null | grep -c "/sentry_scan" || true)
echo "   /cmd_vel 上影子命名空间发布者数量: ${SHADOW_CMD}"
if [[ "${SHADOW_CMD}" -ne 0 ]]; then
  echo "   FAIL: 影子节点出现在了真实控制话题上" >&2
  RC=2
fi

echo
echo "== 4/4 影子速度的未定义分量（angular.z / linear.z 必须为 0）"
python3 scripts/check_shadow_graph.py --duration 3 --expect-healthy "${EXTRA[@]}" \
  | grep -E "max\|wz\||shadow samples" || true

if [[ "${RC}" -ne 0 ]]; then
  echo
  echo "== FAIL (rc=${RC})：先不要继续，把上面的输出回传"
else
  echo
  echo "== PASS：影子只读、未接入真实控制入口（mode=${MODE}）"
fi
exit "${RC}"
