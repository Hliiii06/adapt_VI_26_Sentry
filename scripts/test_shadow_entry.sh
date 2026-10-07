#!/usr/bin/env bash
# 影子接入场景测试（I1/I2 契约验收；失败返回非零）。
#
# 用法：
#   bash scripts/test_shadow_entry.sh adapter_math        # 适配层数学单测（不需要 ROS 图）
#   bash scripts/test_shadow_entry.sh no_inputs           # 无输入：不健康且影子输出为零
#   bash scripts/test_shadow_entry.sh healthy_static      # 合成输入、无任务：健康但无运动
#   bash scripts/test_shadow_entry.sh mode1_goal          # Mode 1 目标：候选速度流过门控
#   bash scripts/test_shadow_entry.sh mode2_waypoints     # Mode 2 航点
#   bash scripts/test_shadow_entry.sh mode3_path          # Mode 3 参考路线
#   bash scripts/test_shadow_entry.sh cloud_stop          # 云中断
#   bash scripts/test_shadow_entry.sh odom_stop           # odom / TF 中断
#   bash scripts/test_shadow_entry.sh tf_stop             # 仅动态 TF 中断
#   bash scripts/test_shadow_entry.sh stale_stamp         # 旧 stamp 重发
#   bash scripts/test_shadow_entry.sh stamp_backwards     # 时间倒退
#   bash scripts/test_shadow_entry.sh localization_jump   # map→odom 跳变
#   bash scripts/test_shadow_entry.sh pairing_mismatch    # GridMap 严格配对拒绝
#   bash scripts/test_shadow_entry.sh cancel              # 取消后不重新运动
#
# 进程管理：影子 launch 与合成 RM 输入都用 setsid 放进各自进程组，只 kill 本次启动的
# 进程组，不使用宽泛的 pkill -f。记录与日志落在 log/shadow/<场景>.<时间戳>/。

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

SCENARIO="${1:-no_inputs}"
DOMAIN="${SHADOW_DOMAIN_ID:-73}"
STARTUP_WAIT="${SHADOW_STARTUP_WAIT:-10}"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "未找到 install/setup.bash，请先运行 scripts/build.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${DOMAIN}"
export ROS_HOME="${REPO_ROOT}/log/ros"

# 不需要 ROS 图的场景直接执行。
if [[ "${SCENARIO}" == "adapter_math" ]]; then
  python3 scripts/test_shadow_adapter_math.py
  exit $?
fi

# 回放安全门：不满足隔离/时钟条件时 launch 必须拒绝启动。
if [[ "${SCENARIO}" == "replay_guard" ]]; then
  FAIL=0
  out1="$(ROS_DOMAIN_ID=0 timeout 40 ros2 launch sentry_scan_adapter sentry_scan_shadow.launch.py \
    replay:=true use_sim_time:=true start_rviz:=false 2>&1 | head -20)"
  grep -q "isolated ROS_DOMAIN_ID" <<<"${out1}" || {
    echo "FAIL: replay:=true 在非隔离 domain 下没有被拒绝" >&2; FAIL=2; }
  out2="$(timeout 40 ros2 launch sentry_scan_adapter sentry_scan_shadow.launch.py \
    use_sim_time:=true start_rviz:=false 2>&1 | head -20)"
  grep -q "only allowed with replay" <<<"${out2}" || {
    echo "FAIL: use_sim_time:=true 在非 replay 下没有被拒绝" >&2; FAIL=2; }
  if [[ "${FAIL}" -eq 0 ]]; then
    echo "== PASS replay_guard: 回放门（隔离 domain + /clock）拒绝非法组合"
  fi
  exit "${FAIL}"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_ROOT}/log/shadow/${SCENARIO}.${STAMP}"
mkdir -p "${LOG_DIR}"

SHARE="${REPO_ROOT}/install/sentry_scan_adapter/share/sentry_scan_adapter/config"
LAUNCH_ARGS=()
case "${SCENARIO}" in
  mode2_waypoints) LAUNCH_ARGS=(navi_mode:=2 "keypoints_file:=${SHARE}/shadow_test_waypoints.yaml") ;;
  mode3_path)      LAUNCH_ARGS=(navi_mode:=3 "reference_path_file:=${SHARE}/shadow_test_reference_path.yaml") ;;
esac

LAUNCH_PGID=""
FAKE_PID=""
cleanup() {
  [[ -n "${FAKE_PID}" ]] && kill "${FAKE_PID}" 2>/dev/null
  [[ -n "${LAUNCH_PGID}" ]] && kill -- "-${LAUNCH_PGID}" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

echo "== scenario=${SCENARIO} domain=${DOMAIN} log=${LOG_DIR}"

setsid ros2 launch sentry_scan_adapter sentry_scan_shadow.launch.py \
  start_rviz:=false "${LAUNCH_ARGS[@]}" > "${LOG_DIR}/launch.log" 2>&1 &
LAUNCH_PGID=$!
sleep "${STARTUP_WAIT}"

RC=0
run_fake() {  # run_fake <额外参数...>
  setsid python3 scripts/fake_rm_inputs.py --domain-note "${SCENARIO}" "$@" \
    > "${LOG_DIR}/fake_inputs.log" 2>&1 &
  FAKE_PID=$!
}

case "${SCENARIO}" in
  no_inputs)
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero || RC=$?
    ;;
  healthy_static)
    run_fake --duration 40
    sleep 6
    python3 scripts/check_shadow_graph.py --duration 6 --expect-healthy --expect-zero || RC=$?
    ros2 run sentry_scan_adapter check_inputs --duration 6 --planning-frame odom || RC=$?
    ;;
  mode1_goal)
    run_fake --duration 60 --send-goal
    sleep 6
    python3 scripts/check_shadow_graph.py --duration 10 --expect-healthy --expect-motion || RC=$?
    ;;
  mode2_waypoints|mode3_path)
    run_fake --duration 60
    sleep 8
    python3 scripts/check_shadow_graph.py --duration 10 --expect-healthy --expect-motion || RC=$?
    ;;
  cloud_stop)
    run_fake --duration 60 --stop-cloud-after 8
    sleep 12
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero || RC=$?
    ;;
  odom_stop)
    run_fake --duration 60 --stop-odom-after 8
    sleep 12
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero || RC=$?
    ;;
  tf_stop)
    run_fake --duration 60 --stop-tf-after 8
    sleep 12
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero || RC=$?
    ;;
  stale_stamp)
    run_fake --duration 60 --repeat-old-stamp
    sleep 6
    python3 scripts/check_shadow_graph.py --duration 8 --expect-unhealthy --expect-zero || RC=$?
    ;;
  stamp_backwards)
    run_fake --duration 60 --stamp-backwards-after 6
    sleep 10
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero || RC=$?
    ;;
  localization_jump)
    run_fake --duration 60 --jump-after 8
    sleep 12
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero || RC=$?
    ;;
  pairing_mismatch)
    run_fake --duration 40 --mismatch-pairing
    sleep 6
    python3 scripts/check_shadow_graph.py --duration 6 --expect-healthy --expect-zero || RC=$?
    if ! grep -q "strict pairing" "${LOG_DIR}/launch.log"; then
      echo "FAIL: GridMap 未记录严格配对拒绝（sensor_pose/cloud 时间戳不一致应被拒绝）" >&2
      RC=2
    fi
    ;;
  cancel)
    run_fake --duration 60 --send-goal --cancel-after 14
    sleep 20
    python3 scripts/check_shadow_graph.py --duration 6 --expect-healthy --expect-zero || RC=$?
    ;;
  *)
    echo "未知场景: ${SCENARIO}" >&2
    RC=1
    ;;
esac

if [[ "${RC}" -ne 0 ]]; then
  echo "== FAIL ${SCENARIO} (rc=${RC}); 日志: ${LOG_DIR}"
  grep -E "ERROR|WARN|rejected|unhealthy|reset|jump" "${LOG_DIR}/launch.log" | tail -20 || true
else
  echo "== PASS ${SCENARIO}; 日志: ${LOG_DIR}"
fi
exit "${RC}"
