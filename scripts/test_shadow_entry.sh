#!/usr/bin/env bash
# 影子接入场景测试（I1/I2 契约验收；失败返回非零）。
#
# 用法：
#   bash scripts/test_shadow_entry.sh no_inputs        # 无输入：必须不健康且影子输出为零
#   bash scripts/test_shadow_entry.sh healthy_static   # 合成输入、无任务：健康但无运动
#   bash scripts/test_shadow_entry.sh mode1_goal       # 合成输入 + Mode 1 目标：候选速度流过门控
#   bash scripts/test_shadow_entry.sh stale_stamp      # 旧 stamp 重发：必须拒绝并撤销任务
#   bash scripts/test_shadow_entry.sh stamp_backwards  # 时间倒退：必须拒绝
#   bash scripts/test_shadow_entry.sh cloud_gap        # 云中断：必须进入不健康、影子输出为零
#   bash scripts/test_shadow_entry.sh localization_jump# map→odom 跳变：撤销任务并要求重新接手
#   bash scripts/test_shadow_entry.sh cancel           # 取消后不重新运动
#
# 进程管理：影子 launch 与合成的 RM 输入都用 setsid 放进各自的进程组，
# 只 kill 本次启动的进程组，不使用宽泛的 pkill -f。
# 记录与日志落在 log/shadow/<场景>.<时间戳>/（log/ 已被 .gitignore 排除）。

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

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_ROOT}/log/shadow/${SCENARIO}.${STAMP}"
mkdir -p "${LOG_DIR}"

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
  start_rviz:=false > "${LOG_DIR}/launch.log" 2>&1 &
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
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy --expect-zero \
      || RC=$?
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
  cloud_stop)
    run_fake --duration 60 --stop-cloud-after 8
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
