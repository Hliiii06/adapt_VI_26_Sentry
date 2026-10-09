#!/usr/bin/env bash
# 可选：录制本次影子调试所需的话题与 TF（只记录，不发布）。
#
# 用法：
#   bash scripts/onsite_record.sh                 # 默认录 120 s
#   bash scripts/onsite_record.sh 300             # 录 300 s
#   TOPICS="/Odometry_transformed /cloud_registered" bash scripts/onsite_record.sh
#
# 产物：log/onsite/<时间戳>/bag/（log/ 已被 .gitignore 排除）
# 录包用于事后复核：坐标/时间戳/TF/候选速度是否一致；不用于驱动任何东西。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
DURATION="${1:-120}"

DEFAULT_TOPICS=(
  /Odometry_transformed
  /LIVO2/imu_propagate
  /cloud_registered
  /tf
  /tf_static
  /sentry_scan/body_pose
  /sentry_scan/sensor_pose
  /sentry_scan/cloud
  /sentry_scan/health
  /sentry_scan/health_ok
  /sentry_scan/planning/bspline
  /sentry_scan/cmd_vel_candidate
  /sentry_scan/cmd_vel_shadow
)
# 只读取实际存在的话题，避免 ros2 bag record 因缺失话题报错。
set +u
source /opt/ros/humble/setup.bash
[[ -f "${REPO_ROOT}/install/setup.bash" ]] && source "${REPO_ROOT}/install/setup.bash"
set -u

if [[ -n "${TOPICS:-}" ]]; then
  read -r -a CANDIDATES <<<"${TOPICS}"
else
  CANDIDATES=("${DEFAULT_TOPICS[@]}")
fi

AVAILABLE="$(ros2 topic list 2>/dev/null || true)"
RECORD=()
for topic in "${CANDIDATES[@]}"; do
  if grep -qx "${topic}" <<<"${AVAILABLE}"; then RECORD+=("${topic}"); fi
done
if (( ${#RECORD[@]} == 0 )); then
  echo "没有可录制的话题；先启动实车系统或影子入口。" >&2
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${REPO_ROOT}/log/onsite/${STAMP}"
mkdir -p "${OUT_DIR}"
echo "录制 ${DURATION}s -> ${OUT_DIR}/bag"
printf '  %s\n' "${RECORD[@]}"

timeout "${DURATION}" ros2 bag record -o "${OUT_DIR}/bag" "${RECORD[@]}" || true
echo "录制结束：${OUT_DIR}/bag"
echo "回传建议：${OUT_DIR}/bag 整个目录 + 同一时间段的 log/shadow/ 目录"
