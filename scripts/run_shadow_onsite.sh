#!/usr/bin/env bash
# 实车在线只读影子入口（现场启动脚本）：先自检，再启动 /sentry_scan。
#
# 边界（脚本只做检查与启动，不改任何实车节点）：
#   * 不启动 / 不重启 LIO、TF、Nav2、串口、电控；它们保持原样运行；
#   * 不启动运动模拟器或 open_loop_controller；不发布替代真实定位的 TF；
#   * 只订阅实车话题与 TF，候选速度只出现在 /sentry_scan/cmd_vel_shadow；
#   * 影子 launch 没有任何"下发真实命令"的开关。
#
# 用法：
#   bash scripts/run_shadow_onsite.sh                      # 默认参数（见下方 env）
#   bash scripts/run_shadow_onsite.sh start_rviz:=false    # 透传给 launch 的参数
#   bash scripts/run_shadow_onsite.sh preflight_only       # 只做检查，不启动
#
# 可用环境变量覆盖：
#   ODOM_TOPIC     默认 /Odometry_transformed
#   VELOCITY_TOPIC 默认 /LIVO2/imu_propagate
#   CLOUD_TOPIC    默认 /cloud_registered
#   PLANNING_FRAME 默认 odom
#   ROBOT_HEIGHT / ROBOT_RADIUS / SAFETY_MARGIN 见 docs/interfaces/shadow_input_contract.md
#   2026-10-09 用户确认的实车包络：半径 0.26 m、高 0.15 m 的圆柱（下方默认值即此）
#   SHADOW_LOG_DIR 默认 log/shadow

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

ODOM_TOPIC="${ODOM_TOPIC:-/Odometry_transformed}"
VELOCITY_TOPIC="${VELOCITY_TOPIC:-/LIVO2/imu_propagate}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered}"
PLANNING_FRAME="${PLANNING_FRAME:-odom}"
# 已确认的实车包络（2026-10-09，用户实测/确认）：半径 0.26 m、高 0.15 m 的圆柱。
# 这是**真实尺寸**，不是为通过检查而缩小；safety_margin 仍为 0（用户未给额外余量）。
ROBOT_HEIGHT="${ROBOT_HEIGHT:-0.15}"
ROBOT_RADIUS="${ROBOT_RADIUS:-0.26}"
SAFETY_MARGIN="${SAFETY_MARGIN:-0.0}"
SHADOW_LOG_DIR="${SHADOW_LOG_DIR:-log/shadow}"

PREFLIGHT_ONLY=0
LAUNCH_ARGS=()
for arg in "$@"; do
  if [[ "${arg}" == "preflight_only" ]]; then PREFLIGHT_ONLY=1; else LAUNCH_ARGS+=("${arg}"); fi
done

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-${REPO_ROOT}/log/ros}"
mkdir -p "${SHADOW_LOG_DIR}"

echo "================================================================"
echo " SCAN 影子入口（只读）——不向底盘输出任何命令"
echo " 规划系=${PLANNING_FRAME}  车高=${ROBOT_HEIGHT}  半径=${ROBOT_RADIUS}  余量=${SAFETY_MARGIN}"
echo " 输入: ${ODOM_TOPIC} | ${VELOCITY_TOPIC} | ${CLOUD_TOPIC}"
echo " 输出: /sentry_scan/*（cmd_vel_candidate → cmd_vel_shadow，仅记录/显示）"
echo " 退出: 在本终端 Ctrl-C —— 只结束影子入口，不影响原有系统"
echo "================================================================"

echo
echo "== 预检 1/3：实车输入话题是否存在"
TOPICS="$(ros2 topic list 2>/dev/null || true)"
MISSING=()
for topic in "${ODOM_TOPIC}" "${VELOCITY_TOPIC}" "${CLOUD_TOPIC}"; do
  if ! grep -qx "${topic}" <<<"${TOPICS}"; then MISSING+=("${topic}"); fi
done
if (( ${#MISSING[@]} )); then
  echo "   FAIL: 以下话题当前不存在："
  for topic in "${MISSING[@]}"; do echo "     - ${topic}"; done
  echo "   先运行采集脚本确认实际话题名/命名空间，再用 *_TOPIC 覆盖后重试："
  echo "     python3 scripts/onsite_inspect.py --duration 20"
  exit 1
fi
echo "   OK"

echo
echo "== 预检 2/3：影子命名空间是否已在运行（避免重复实例）"
if ros2 node list 2>/dev/null | grep -q "^/sentry_scan/"; then
  echo "   FAIL: /sentry_scan 下已有节点在运行；先 Ctrl-C 退出旧实例。" >&2
  ros2 node list | grep "^/sentry_scan/" | sed 's/^/     /'
  exit 1
fi
echo "   OK"

echo
echo "== 预检 3/3：真实控制入口当前发布者（仅供确认，影子不会接管）"
for topic in /cmd_vel /cmd_vel_remap /cmd_vel_nav; do
  publishers="$(ros2 topic info -v "${topic}" 2>/dev/null \
    | grep -E "Node name|Node namespace" | paste -sd' ' - || true)"
  printf "   %-18s %s\n" "${topic}" "${publishers:-无发布者/无此话题}"
done

if [[ "${PREFLIGHT_ONLY}" -eq 1 ]]; then
  echo
  echo "== preflight_only：检查完成，未启动影子入口"
  exit 0
fi

echo
echo "== 启动影子入口（本终端保持前台运行）"
exec ros2 launch sentry_scan_adapter sentry_scan_shadow.launch.py \
  planning_frame:="${PLANNING_FRAME}" \
  odom_topic:="${ODOM_TOPIC}" \
  velocity_topic:="${VELOCITY_TOPIC}" \
  cloud_topic:="${CLOUD_TOPIC}" \
  robot_height:="${ROBOT_HEIGHT}" \
  robot_radius:="${ROBOT_RADIUS}" \
  safety_margin:="${SAFETY_MARGIN}" \
  shadow_log_dir:="${SHADOW_LOG_DIR}" \
  "${LAUNCH_ARGS[@]}"
