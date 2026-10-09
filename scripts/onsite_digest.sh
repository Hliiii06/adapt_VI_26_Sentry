#!/usr/bin/env bash
# 把现场采集目录压成"可粘贴的摘要"（终端里几十行），并可选打包。
#
# 用法：
#   bash scripts/onsite_digest.sh ~/下载/collect_20261009_205335
#   bash scripts/onsite_digest.sh ~/下载/collect_20261009_205335 --tar   # 顺带打包成 .tar.gz
#
# 为什么需要：采集目录在实车电脑上，助手看不到；摘要打印到终端后可以直接复制粘贴。
# 它只读文件，不启动/不连接任何 ROS 节点。

set -uo pipefail

DIR="${1:-}"
MODE="${2:-}"
if [[ -z "${DIR}" || ! -d "${DIR}" ]]; then
  echo "用法: bash scripts/onsite_digest.sh <采集目录> [--tar]" >&2
  echo "例如: bash scripts/onsite_digest.sh ~/下载/collect_20261009_205335" >&2
  exit 1
fi

head_or() {  # head_or <文件> <行数>
  local file="$1" lines="$2"
  if [[ -f "${file}" ]]; then head -n "${lines}" "${file}"; else echo "（缺 ${file##*/}）"; fi
}

echo "################ 采集目录: ${DIR} ################"
echo
echo "########## 1) 节点（判断哪些属于原系统） ##########"
head_or "${DIR}/nodes.txt" 60

echo
echo "########## 2) 话题与类型 ##########"
head_or "${DIR}/topics.txt" 80

echo
echo "########## 3) 采集报告：关键话题（类型/频率/年龄/frame） ##########"
REPORT="$(find "${DIR}/inspect" -name report.txt 2>/dev/null | head -1 || true)"
if [[ -n "${REPORT}" ]]; then
  sed -n '/== 关键话题 ==/,/== TF 动态/p' "${REPORT}" | head -60
  echo "----- TF（动态 + 静态）-----"
  sed -n '/== TF 动态 ==/,/== 控制话题发布者/p' "${REPORT}" | head -40
  echo "----- 控制话题发布者 / FAIL / WARN -----"
  sed -n '/== 控制话题发布者/,$p' "${REPORT}" | head -40
else
  echo "（缺 inspect/report.txt）"
fi

echo
echo "########## 4) 影子健康原因（health_ok=False 的原因） ##########"
if [[ -f "${DIR}/shadow_health.txt" ]]; then
  grep -vE "^\s*$" "${DIR}/shadow_health.txt" | head -45
else
  echo "（缺 shadow_health.txt：影子入口当时没在跑？）"
fi

echo
echo "########## 5) 控制话题拓扑（发布者 vs 订阅者） ##########"
head_or "${DIR}/shadow_control_audit.txt" 40

echo
echo "########## 6) 点云特征（布局/尺度/是否含自身） ##########"
head_or "${DIR}/cloud_stats.txt" 40

echo
echo "########## 7) TF 关键对（只取第一段变换） ##########"
for pair in odom_base_link odom_camera_init base_link_base_footprint base_footprint_lidar_link odom_world camera_init_base_link; do
  file="${DIR}/tf_echo_${pair}.txt"
  if [[ -f "${file}" ]]; then
    echo "----- ${pair} -----"
    grep -A3 -m1 -E "Translation|Rotation" "${file}" | head -8
  fi
done
if [[ -f "${DIR}/tf_static_once.txt" ]]; then
  echo "----- /tf_static（父子帧一览） -----"
  grep -E "frame_id|child_frame_id" "${DIR}/tf_static_once.txt" | head -30
fi

echo
echo "########## 8) 频率（每个话题最后一行） ##########"
for file in "${DIR}"/hz_*.txt; do
  [[ -f "${file}" ]] || continue
  printf "%-42s %s\n" "$(basename "${file}" .txt)" "$(grep -E "average rate|no new messages|does not appear" "${file}" | tail -1)"
done

echo
echo "########## 9) 时间戳/frame（header 与 twist 采样） ##########"
for file in "${DIR}"/echo_header_*.txt "${DIR}/echo_velocity_twist.txt" "${DIR}/echo_odom_pose.txt"; do
  [[ -f "${file}" ]] || continue
  echo "----- $(basename "${file}" .txt) -----"
  grep -E "sec:|nanosec:|frame_id:|x:|y:|z:|w:" "${file}" | head -14
done

echo
echo "########## 10) 运行参数里的关键项 ##########"
if [[ -d "${DIR}/params" ]]; then
  grep -rhiE "use_sim_time|twist_topic|cmd_vel|odom_topic|cloud|frame_id|serial|baud|port" \
    "${DIR}/params" 2>/dev/null | sed 's/^[[:space:]]*//' | sort -u | head -40
  echo "（参数文件：$(ls "${DIR}/params" 2>/dev/null | wc -l) 个）"
else
  echo "（缺 params/：第 7 步可能被跳过或超时）"
fi

echo
echo "########## 11) 该采集目录里有什么 ##########"
find "${DIR}" -maxdepth 2 -type f | sed "s#^${DIR}/##" | sort | head -60

if [[ "${MODE}" == "--tar" ]]; then
  OUT_TAR="${DIR%/}.tar.gz"
  tar czf "${OUT_TAR}" -C "$(dirname "${DIR}")" "$(basename "${DIR}")" && \
    echo && echo "已打包: ${OUT_TAR}（$(du -h "${OUT_TAR}" | cut -f1)）"
fi
echo
echo "把以上输出整段复制回传给助手即可；若想发原文件，用上面的 --tar 打包后发送。"
