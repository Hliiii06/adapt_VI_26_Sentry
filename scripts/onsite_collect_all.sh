#!/usr/bin/env bash
# 现场一次性采集包（全部只读）：把所有"只有实车在跑才能确定"的事实收进一个目录并打包。
#
# 用法（机器人静止即可，不要停任何实车节点）：
#   bash scripts/onsite_collect_all.sh
#   bash scripts/onsite_collect_all.sh --record 20     # 额外录 20s bag（可选）
#
# 产物：log/onsite/collect_<时间戳>/ 与同名 .tar.gz
# 回传：把 .tar.gz 发回即可（**不要**提交到公开仓库；log/ 已被 .gitignore 排除）。
#
# 只读保证：只做 list/info/echo/hz/tf2_echo/view_frames/param dump/订阅采样 + 可选录包，
# 不创建任何发布者，不调用服务/action，不启动或停止任何实车节点。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

RECORD_SECONDS=0
if [[ "${1:-}" == "--record" ]]; then RECORD_SECONDS="${2:-20}"; fi

set +u
source /opt/ros/humble/setup.bash
[[ -f "${REPO_ROOT}/install/setup.bash" ]] && source "${REPO_ROOT}/install/setup.bash"
set -u
export ROS_HOME="${ROS_HOME:-${REPO_ROOT}/log/ros}"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${REPO_ROOT}/log/onsite/collect_${STAMP}"
mkdir -p "${OUT}"
echo "采集到: ${OUT}"
echo "注意：本采集全程只读；不影响 LIO / 雷达 / Nav2 / 串口。"

save() {  # save <文件名> <命令...>
  local name="$1"; shift
  { echo "\$ $*"; echo; "$@" 2>&1; } > "${OUT}/${name}.txt" || true
  echo "  -> ${name}.txt"
}

echo
echo "== 1/9 环境与图 =="
save basic bash -c 'date; echo; uname -a; echo; hostname; echo; printenv | grep -E "^(ROS_|RMW_)" | sort'
save nodes ros2 node list
save topics ros2 topic list -t
save topic_info_all bash -c 'for t in $(ros2 topic list); do echo "===== $t"; ros2 topic info -v "$t"; done'
{ for n in $(ros2 node list 2>/dev/null); do echo "===== $n"; ros2 node info "$n" 2>&1; done; } \
  > "${OUT}/node_info.txt" 2>&1 || true
echo "  -> node_info.txt"

echo
echo "== 2/9 关键话题采集（onsite_inspect，20s）=="
python3 scripts/onsite_inspect.py --duration 20 --out-dir "${OUT}/inspect" > "${OUT}/inspect_stdout.txt" 2>&1 || true
echo "  -> inspect/report.txt"

echo
echo "== 3/9 影子健康原因（若影子在跑）=="
python3 scripts/onsite_health_dump.py --duration 4 > "${OUT}/shadow_health.txt" 2>&1 || true
save shadow_control_audit python3 scripts/onsite_control_audit.py

echo
echo "== 4/9 TF 链 =="
( cd "${OUT}" && timeout 10 ros2 run tf2_tools view_frames > view_frames_stdout.txt 2>&1 || true )
for pair in "odom base_link" "odom base_footprint" "odom lidar_link" "odom camera_init" \
            "base_link base_footprint" "base_footprint lidar_link" "camera_init base_link" \
            "map odom" "odom world" "world odom"; do
  name="tf_echo_$(echo "${pair}" | tr ' ' '_')"
  { echo "\$ ros2 run tf2_ros tf2_echo ${pair}"; timeout 5 ros2 run tf2_ros tf2_echo ${pair} 2>&1; } \
    > "${OUT}/${name}.txt" || true
done
save tf_static_once timeout 5 ros2 topic echo --once /tf_static
echo "  -> view_frames.pdf / tf_echo_*.txt / tf_static_once.txt"

echo
echo "== 5/9 频率与时间戳 =="
for topic in /Odometry_transformed /Odometry /LIVO2/imu_propagate /cloud_registered /pointcloud /tf /tf_static; do
  name="hz_$(echo "${topic}" | tr '/' '_' | sed 's/^_//')"
  { echo "\$ ros2 topic hz ${topic}"; timeout 6 ros2 topic hz "${topic}" 2>&1; } > "${OUT}/${name}.txt" || true
done
for topic in /Odometry_transformed /LIVO2/imu_propagate /cloud_registered; do
  name="echo_header_$(echo "${topic}" | tr '/' '_' | sed 's/^_//')"
  { echo "\$ ros2 topic echo --once ${topic} --field header"; timeout 5 ros2 topic echo --once "${topic}" --field header 2>&1; } \
    > "${OUT}/${name}.txt" || true
done
save echo_velocity_twist timeout 5 ros2 topic echo --once /LIVO2/imu_propagate --field twist
save echo_odom_pose timeout 5 ros2 topic echo --once /Odometry_transformed --field pose
echo "  -> hz_*.txt / echo_*.txt"

echo
echo "== 6/9 点云特征（布局/尺度/是否含自身）=="
python3 scripts/onsite_cloud_stats.py --duration 8 > "${OUT}/cloud_stats.txt" 2>&1 || true
cat "${OUT}/cloud_stats.txt"
echo "  -> cloud_stats.txt"

echo
echo "== 7/9 运行参数快照（含 use_sim_time、串口话题、控制话题）=="
mkdir -p "${OUT}/params"
for n in $(ros2 node list 2>/dev/null); do
  safe="$(echo "${n}" | tr '/' '_' | sed 's/^_//')"
  ros2 param dump "${n}" > "${OUT}/params/${safe}.yaml" 2>/dev/null || true
done
ls "${OUT}/params" | head -20
echo "  -> params/*.yaml"

echo
echo "== 8/9 可选录包 =="
if [[ "${RECORD_SECONDS}" -gt 0 ]]; then
  timeout --signal=INT --kill-after=15 "${RECORD_SECONDS}" \
    ros2 bag record -o "${OUT}/bag" /Odometry_transformed /LIVO2/imu_propagate \
    /cloud_registered /tf /tf_static > "${OUT}/record_stdout.txt" 2>&1 || true
  ls -la "${OUT}/bag" 2>/dev/null | head -5
else
  echo "  跳过（需要时加：bash scripts/onsite_collect_all.sh --record 20）"
fi

echo
echo "== 9/9 打包 =="
tar czf "${OUT}.tar.gz" -C "$(dirname "${OUT}")" "$(basename "${OUT}")" 2>/dev/null || true
du -h "${OUT}.tar.gz" 2>/dev/null || true
echo
echo "回传这个文件即可："
echo "  ${OUT}.tar.gz"
echo "注意：包里可能含串口设备名/内网地址等现场信息，请**不要**提交到公开仓库（log/ 已被 .gitignore 排除）。"
