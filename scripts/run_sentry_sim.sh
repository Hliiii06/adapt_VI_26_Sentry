#!/usr/bin/env bash
# 启动全向哨兵 PCD/RViz 闭环仿真。
#
# 用法：
#   scripts/run_sentry_sim.sh navi_mode:=1                      # RViz 2D Goal Pose
#   scripts/run_sentry_sim.sh navi_mode:=2 \
#       keypoints_file:=$(pwd)/install/scan_planner/share/scan_planner/config/sentry_waypoints.yaml
#   scripts/run_sentry_sim.sh navi_mode:=3 \
#       reference_path_file:=$(pwd)/install/scan_planner/share/scan_planner/config/sentry_reference_path.yaml
#   scripts/run_sentry_sim.sh start_rviz:=false                 # 无界面
#   scripts/run_sentry_sim.sh yaw_mode:=spin spin_rate:=0.6     # 边转边走
#
# 所有节点在 /sentry_sim 命名空间内；cmd_vel 不会离开该命名空间。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "未找到 install/setup.bash，请先运行 scripts/build.sh" >&2
  exit 1
fi

# ROS 的 setup.bash 会引用未定义变量，source 期间必须关闭 -u。
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

# 海康 MVS SDK 在 ~/.bashrc 里把 /opt/MVS/lib/64 放在 LD_LIBRARY_PATH 最前面，
# 其中的 libusb-1.0.so.0 缺少 libusb_set_option 符号，会让 PCL 的 libpcl_io
# 在运行期解析失败（map_pub / pcl_render_node 启动即退出）。本仿真不需要 MVS，
# 这里剔除其路径，使 libusb 解析到系统库。
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH}" | tr ':' '\n' \
    | grep -v '^/opt/MVS/' | paste -sd: -)"
  export LD_LIBRARY_PATH
fi

# ROS 日志留在仓库内，便于收集可复现证据（log/ 已被 .gitignore 排除）。
export ROS_HOME="${ROS_HOME:-${REPO_ROOT}/log/ros}"
mkdir -p "${ROS_HOME}"

exec ros2 launch scan_planner sentry_sim.launch.py "$@"
