#!/usr/bin/env bash
# 从场景 launch 日志中提取可审查的要点，避免把上千条重复告警塞进仓库。
#
# 输出 docs/testing/evidence/<场景>_key_log.txt：
#   - 生效配置（演示地图删点、碰撞包络、雷达外参）
#   - FSM 状态转换序列（去重保序）
#   - 告警/错误按"归一化后的类型"计数（数字被抹掉）
#   - 收到的轨迹条数
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
SRC_DIR="${REPO_ROOT}/log/scenarios"
DST_DIR="${REPO_ROOT}/docs/testing/evidence"
mkdir -p "${DST_DIR}"

for log in "${SRC_DIR}"/*_launch.log; do
  [[ -e "${log}" ]] || continue
  name="$(basename "${log}" _launch.log)"
  out="${DST_DIR}/${name}_key_log.txt"
  {
    echo "# 场景 ${name} 关键日志摘要"
    echo "# 由 scripts/summarize_launch_log.sh 从 log/scenarios/${name}_launch.log 提取"
    echo
    echo "## 生效配置"
    grep -hoE "\[(map_pub|scan_planner_node|pcl_render_node|closed_loop_controller|go2_kinematic_sim)-[0-9]+\] .*(DEMO MAP|Collision envelope|Lidar extrinsic|tracker ready|simulator ready).*" "${log}" \
      | sed 's/^\[[a-z_]*-[0-9]*\] //' | sort -u || true
    echo
    echo "## FSM 状态转换（去重保序）"
    grep -hoE "\[[A-Z_]+\]: from [A-Z_]+ to [A-Z_]+" "${log}" | awk '!seen[$0]++' || true
    echo
    echo "## 告警/错误类型计数（数字已归一化）"
    grep -hoE "\[(WARN|ERROR)\] .*" "${log}" \
      | sed -E 's/\[(WARN|ERROR)\] \[[0-9.]+\] \[[a-z_]+\]: //; s/[0-9]+\.[0-9]+/N/g; s/[0-9]+/N/g' \
      | sort | uniq -c | sort -rn | head -20 || true
    echo
    echo "## 关键事件"
    echo "收到轨迹条数: $(grep -c 'Received trajectory' "${log}" || true)"
    echo "A-star 失败: $(grep -c 'A-star failed' "${log}" || true)"
    echo "动力学可行性失败: $(grep -c 'Dynamic feasibility failed' "${log}" || true)"
    echo "重规划连续失败告警: $(grep -c 'Replan failed' "${log}" || true)"
    echo "任务取消: $(grep -c 'Task cancelled' "${log}" || true)"
    echo "取消闩锁: $(grep -c 'Cancel latched' "${log}" || true)"
    echo "拒绝轨迹: $(grep -c 'Rejecting' "${log}" || true)"
    echo "到达航点规划: $(grep -c 'Planning to waypoint' "${log}" || true)"
  } > "${out}"
  echo "生成 ${out} ($(wc -l < "${out}") 行)"
done
