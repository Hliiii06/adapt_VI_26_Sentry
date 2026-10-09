#!/usr/bin/env bash
# 可选：录制本次影子调试所需的话题与 TF（只记录，不发布）。
#
# 用法：
#   bash scripts/onsite_record.sh                 # 默认录 120 s
#   bash scripts/onsite_record.sh 300             # 录 300 s
#   TOPICS="/Odometry_transformed /cloud_registered" bash scripts/onsite_record.sh
#
# 产物：log/onsite/<时间戳>/bag/（log/ 已被 .gitignore 排除）
# 退出码：0 = 到时正常结束且 bag 里有消息；非 0 = 录制异常/未落盘/0 条消息（不要当作已有证据）。

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

set +u
source /opt/ros/humble/setup.bash
[[ -f "${REPO_ROOT}/install/setup.bash" ]] && source "${REPO_ROOT}/install/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-${REPO_ROOT}/log/ros}"

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
  echo "FAIL: 没有可录制的话题；先启动实车系统或影子入口。" >&2
  exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${REPO_ROOT}/log/onsite/${STAMP}"
mkdir -p "${OUT_DIR}"
echo "录制 ${DURATION}s -> ${OUT_DIR}/bag"
printf '  %s\n' "${RECORD[@]}"

# 用 SIGINT 让 rosbag2 正常收尾（SIGTERM 可能留下不完整的 bag）；
# 严格区分"到时正常结束"与"异常退出"，并校验落盘结果。
timeout --signal=INT --kill-after=20 "${DURATION}" \
  ros2 bag record -o "${OUT_DIR}/bag" "${RECORD[@]}"
RC=$?
if [[ "${RC}" -ne 0 && "${RC}" -ne 124 && "${RC}" -ne 130 ]]; then
  echo "FAIL: ros2 bag record 异常退出（rc=${RC}）；录制未完成，不要当作已有证据。" >&2
  exit 2
fi

META="${OUT_DIR}/bag/metadata.yaml"
if [[ ! -f "${META}" ]]; then
  echo "FAIL: 没有生成 ${META}（录制没有落盘；检查磁盘空间与 rosbag2 报错）。" >&2
  exit 2
fi

COUNT="$(METADATA="${META}" python3 -c '
import os, yaml
with open(os.environ["METADATA"]) as handle:
    data = yaml.safe_load(handle) or {}
info = data.get("rosbag2_bagfile_information", {})
print(int(info.get("message_count", 0) or 0))
')"
if [[ "${COUNT:-0}" -le 0 ]]; then
  echo "FAIL: bag 里 0 条消息（话题存在但没数据？）；不作为证据。" >&2
  exit 2
fi

echo "录制结束（rc=${RC}，到时正常结束）：${OUT_DIR}/bag，共 ${COUNT} 条消息"
ros2 bag info "${OUT_DIR}/bag" 2>/dev/null | sed -n '1,40p' || true
echo "回传建议：${OUT_DIR}/bag 整个目录 + 同一时间段的 log/shadow/ 目录"
