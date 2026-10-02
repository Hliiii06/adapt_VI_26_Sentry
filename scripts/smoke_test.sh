#!/usr/bin/env bash
# 仿真冒烟测试：在同一个 shell 内启动、检查、停止（沙箱下后台进程不跨调用存活）。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
export ROS_HOME="${REPO_ROOT}/log/ros"
mkdir -p "${ROS_HOME}" "${REPO_ROOT}/log/scenarios"

LOG="${REPO_ROOT}/log/scenarios/smoke_launch.log"
MODE="${1:-1}"

# setsid：仿真放进**独立进程组**，清理时只杀本组，不会误杀同一用户的其它实验
setsid ./scripts/run_sentry_sim.sh start_rviz:=false "navi_mode:=${MODE}" > "${LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 1
LAUNCH_PGID="$(ps -o pgid= -p ${LAUNCH_PID} 2>/dev/null | tr -d ' ')"
cleanup() {
  if [[ -n "${LAUNCH_PGID}" ]]; then
    kill -INT -"${LAUNCH_PGID}" 2>/dev/null || true
    sleep 2
    kill -KILL -"${LAUNCH_PGID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "启动 navi_mode=${MODE}，PID=${LAUNCH_PID}，PGID=${LAUNCH_PGID}；等待节点就绪..."
sleep 25

echo "=== 节点 ==="
timeout 15 ros2 node list || true

echo "=== 话题 ==="
timeout 15 ros2 topic list | sort || true

echo "=== body_pose 一帧 ==="
timeout 15 ros2 topic echo --once /sentry_sim/body_pose 2>&1 | head -25 || true

echo "=== 关键话题频率（各 5 s） ==="
for t in /sentry_sim/cloud /sentry_sim/body_pose /sentry_sim/cmd_vel /sentry_sim/grid_map/occupancy_inflate; do
  echo "--- ${t}"
  timeout 6 ros2 topic hz "${t}" 2>&1 | head -3 || true
done

echo "=== 启动日志中的关键行 ==="
grep -iE "envelope|extrinsic|height filter|tracker ready|simulator ready|error|warn" "${LOG}" | head -30 || true

echo "=== 停止（只杀本次进程组 PGID=${LAUNCH_PGID}）==="
cleanup
LAUNCH_PGID=""
echo "完成"
