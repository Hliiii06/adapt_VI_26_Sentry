#!/usr/bin/env bash
# S3 场景编排：在同一个 shell 内启动仿真、记录证据、停止。
#
# 用法: scripts/scenario.sh <场景>
#   mode1_lateral   Mode 1 + yaw_mode=hold，沿 +Y 横移到目标（车头不变）
#   mode1_align     Mode 1 + yaw_mode=align，对比朝向策略
#   mode2_waypoints Mode 2 参数航点
#   mode3_path      Mode 3 参考路线
#   spin            Mode 2 + yaw_mode=spin（边转边走）
#   cancel          Mode 1 后发 planning/reset 取消
#   collision_block 目标落在障碍内，检查是否拒绝/不撞
#   odom_loss       Mode 2 途中杀掉里程计，检查是否停车
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
export ROS_HOME="${REPO_ROOT}/log/ros"
mkdir -p "${ROS_HOME}" "${REPO_ROOT}/log/scenarios"

CFG="${REPO_ROOT}/install/scan_planner/share/scan_planner/config"
SCENARIO="${1:?用法: scripts/scenario.sh <场景>}"
OUT="${REPO_ROOT}/log/scenarios/${SCENARIO}"
LAUNCH_LOG="${OUT}_launch.log"
mkdir -p "$(dirname "${OUT}")"

case "${SCENARIO}" in
  mode1_lateral)   MODE=1; EXTRA=(yaw_mode:=hold);  DUR=30; GOAL_X=-6.0; GOAL_Y=7.5 ;;
  mode1_align)     MODE=1; EXTRA=(yaw_mode:=align); DUR=30; GOAL_X=-6.0; GOAL_Y=7.5 ;;
  collision_block) MODE=1; EXTRA=(yaw_mode:=hold);  DUR=20; GOAL_X=-6.98; GOAL_Y=0.58 ;;
  tight_pass)      MODE=1; EXTRA=(yaw_mode:=hold);  DUR=25; GOAL_X=-4.25; GOAL_Y=2.25 ;;
  cancel)          MODE=1; EXTRA=(yaw_mode:=hold);  DUR=25; GOAL_X=-6.0; GOAL_Y=7.5 ;;
  mode2_waypoints) MODE=2; EXTRA=(keypoints_file:=${CFG}/sentry_waypoints.yaml); DUR=35 ;;
  mode3_path)      MODE=3; EXTRA=(reference_path_file:=${CFG}/sentry_reference_path.yaml); DUR=35 ;;
  spin)            MODE=2; EXTRA=(keypoints_file:=${CFG}/sentry_waypoints.yaml yaw_mode:=spin spin_rate:=0.5); DUR=35 ;;
  odom_loss)       MODE=2; EXTRA=(keypoints_file:=${CFG}/sentry_waypoints.yaml); DUR=30 ;;
  *) echo "未知场景: ${SCENARIO}" >&2; exit 2 ;;
esac

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

cleanup() {
  pkill -f sentry_sim.launch.py 2>/dev/null || true
  pkill -f scan_planner_node 2>/dev/null || true
  pkill -f pcl_render_node 2>/dev/null || true
  pkill -f go2_kinematic_sim 2>/dev/null || true
  pkill -f closed_loop_controller 2>/dev/null || true
  pkill -f map_pub 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT

echo "### 场景 ${SCENARIO}: navi_mode=${MODE} ${EXTRA[*]:-}"

if [[ "${SCENARIO}" == "odom_loss" ]]; then
  ./scripts/run_sentry_sim.sh start_rviz:=false "navi_mode:=${MODE}" "${EXTRA[@]}" \
    > "${LAUNCH_LOG}" 2>&1 &
  # 全程记录 cmd_vel；机器人运动途中切断里程计，再检查命令是否归零。
  python3 scripts/scenario_test.py --scenario odom_loss --duration 12 --out "${OUT}" \
    > "${OUT}_recorder.log" 2>&1 &
  REC_PID=$!
  sleep 8
  echo "[odom_loss] 切断运动模拟器（body_pose 停止发布）"
  pkill -f go2_kinematic_sim 2>/dev/null || true
  wait "${REC_PID}" || true

  python3 - "${OUT}_cmdvel.csv" <<'PY'
import csv, sys
rows = [(float(r["t"]), float(r["vx"]), float(r["vy"]), float(r["wz"]))
        for r in csv.DictReader(open(sys.argv[1]))]
moving = [r for r in rows if abs(r[1]) > 0.01 or abs(r[2]) > 0.01 or abs(r[3]) > 0.01]
print("cmd_vel 样本数: %d，其中非零: %d" % (len(rows), len(moving)))
if not moving:
    print("结论: 失败 —— 全程没有非零命令，本场景无效")
    sys.exit(0)
last = moving[-1][0]
after = [r for r in rows if r[0] > last + 0.05]
stuck = [r for r in after if abs(r[1]) > 0.01 or abs(r[2]) > 0.01 or abs(r[3]) > 0.01]
print("最后一次非零命令: t=%.2fs" % last)
print("其后样本数: %d，其中仍非零: %d" % (len(after), len(stuck)))
print("结论: %s" % ("通过 —— 里程计断流后命令保持为 0" if not stuck
                    else "失败 —— 断流后仍有非零命令 %d 个" % len(stuck)))
PY
else
  # 记录器先启动并等 body_pose 上线，避免漏掉 Mode 2/3 的自动起步阶段。
  python3 scripts/scenario_test.py --scenario "${SCENARIO}" --duration "${DUR}" \
    --goal-x "${GOAL_X:--6.0}" --goal-y "${GOAL_Y:-7.5}" --out "${OUT}" \
    > "${OUT}_recorder.log" 2>&1 &
  REC_PID=$!
  sleep 1
  ./scripts/run_sentry_sim.sh start_rviz:=false "navi_mode:=${MODE}" "${EXTRA[@]}" \
    > "${LAUNCH_LOG}" 2>&1 &
  wait "${REC_PID}" || true
  tail -40 "${OUT}_recorder.log"
fi

echo "### 关键日志"
grep -iE "collision envelope|extrinsic|height filter|tracker ready|simulator ready|Reached|fail|emergency|ERROR" \
  "${LAUNCH_LOG}" | head -15 || true
