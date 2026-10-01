#!/usr/bin/env bash
# 隔离构建：产物只落在本仓库的 build/ 与 install/，日志落在 log/ros/。
# 不 source 任何既有 overlay（旧 install 可能来自其它提交）。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

# ROS 的 setup.bash 会引用未定义变量，source 期间必须关闭 -u。
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_HOME="${REPO_ROOT}/log/ros"
mkdir -p "${ROS_HOME}"

CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}" colcon build \
  --base-paths src \
  --build-base build \
  --install-base install \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "构建完成。启动仿真：scripts/run_sentry_sim.sh navi_mode:=1"
