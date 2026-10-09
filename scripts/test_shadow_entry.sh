#!/usr/bin/env bash
# 影子接入场景测试（I1/I2 契约验收；失败返回非零）。
#
# 用法：
#   bash scripts/test_shadow_entry.sh adapter_math        # 适配层数学/点云布局单测（不需要 ROS 图）
#   bash scripts/test_shadow_entry.sh guard_logic         # 影子门控逻辑单测（不需要 ROS 图）
#   bash scripts/test_shadow_entry.sh no_inputs           # 无输入：不健康且影子输出为零
#   bash scripts/test_shadow_entry.sh healthy_static      # 合成输入、无任务：健康但无运动
#   bash scripts/test_shadow_entry.sh mode1_goal          # Mode 1 目标：候选速度流过门控
#   bash scripts/test_shadow_entry.sh mode2_waypoints     # Mode 2 航点
#   bash scripts/test_shadow_entry.sh mode3_path          # Mode 3 参考路线
#   bash scripts/test_shadow_entry.sh task_frame_transform # 非单位 map→odom 与任务坐标转换
#   bash scripts/test_shadow_entry.sh padded_cloud         # 行填充点云：重排为密集布局后走全链
#   bash scripts/test_shadow_entry.sh onsite_tools         # 现场工具彩排：采集/安全自检/发目标/录包
#   bash scripts/test_shadow_entry.sh cloud_stop          # 云中断
#   bash scripts/test_shadow_entry.sh odom_stop           # odom / TF 中断
#   bash scripts/test_shadow_entry.sh tf_stop             # 仅动态 TF 中断
#   bash scripts/test_shadow_entry.sh stale_stamp         # 旧 stamp 重发
#   bash scripts/test_shadow_entry.sh stamp_backwards     # 时间倒退
#   bash scripts/test_shadow_entry.sh localization_jump   # map→odom 跳变
#   bash scripts/test_shadow_entry.sh pairing_mismatch    # GridMap 严格配对拒绝
#   bash scripts/test_shadow_entry.sh map_gate            # 地图停更 -> 撤销+锁止+归零
#   bash scripts/test_shadow_entry.sh map_relatch         # 心跳恢复但无新任务：持续为零
#   bash scripts/test_shadow_entry.sh map_relatch_newtask # 新任务授权后才解除锁止
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
if [[ "${SCENARIO}" == "guard_logic" ]]; then
  python3 scripts/test_shadow_guard_logic.py
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
  map_gate|map_relatch|map_relatch_newtask)
                   LAUNCH_ARGS=(map_update_topic:=test/map_heartbeat) ;;
esac

LAUNCH_PGID=""
FAKE_PID=""
SPOOF_PID=""
cleanup() {
  [[ -n "${SPOOF_PID}" ]] && kill "${SPOOF_PID}" 2>/dev/null
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

# 失效/取消类场景统一用"事件前有运动 + 限时归零 + 整窗为零"的判据。
stop_check() {  # stop_check <fallback-fault-at> <duration> [额外参数...]
  # 故障时刻优先取注入方在 /sentry_scan/test/fault_marker 上发布的标记（实际事件时间），
  # 因此不再手工推算 zero-from —— 由判据按实际故障时刻 + stop-deadline 计算。
  local fault="$1"; shift
  local duration="$1"; shift
  python3 scripts/check_shadow_stop.py --duration "${duration}" --fault-at "${fault}" \
    --stop-deadline 3.0 --min-motion 0.2 "$@" || RC=$?
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
  task_frame_transform)
    # 非单位 map->odom + 目标/路线都在 map 下：验证坐标变换与未知 frame 拒绝。
    run_fake --duration 60 --map-odom-offset 1.0,1.0,0.3
    sleep 6
    python3 scripts/check_task_adapter.py || RC=$?
    if ! grep -q "empty header.frame_id" "${LOG_DIR}/launch.log" \
       || ! grep -q "cannot transform 'no_such_frame'" "${LOG_DIR}/launch.log"; then
      echo "FAIL: 未看到空 frame / 未知 frame 的明确拒绝日志" >&2
      RC=2
    fi
    ;;
  padded_cloud)
    # 带行填充的点云必须被适配层先重排为密集布局，再走 TF → GridMap → 规划。
    run_fake --duration 60 --send-goal --padded-cloud
    sleep 6
    python3 scripts/check_shadow_graph.py --duration 10 --expect-healthy --expect-motion || RC=$?
    if [[ "${RC}" -eq 0 ]] && ! grep -q "repacked to a dense layout" "${LOG_DIR}/launch.log"; then
      echo "FAIL: 带行填充的点云没有被适配层重排为密集布局" >&2
      RC=2
    fi
    ;;
  onsite_tools)
    # 现场工具的本地彩排（合成输入）：采集 → 安全自检(idle) → 单独发目标 → 安全自检(motion) → 录包
    run_fake --duration 120
    sleep 8
    echo "-- onsite_inspect"
    python3 scripts/onsite_inspect.py --duration 8 --out-dir "${LOG_DIR}/onsite" || RC=$?
    echo "-- onsite_check_safety idle"
    bash scripts/onsite_check_safety.sh idle 6 || RC=$?
    echo "-- onsite_send_goal (只发影子入口)"
    python3 scripts/onsite_send_goal.py --frame odom --x 2.0 --y 0.0 --wait 10 || RC=$?
    sleep 4
    echo "-- onsite_check_safety motion"
    bash scripts/onsite_check_safety.sh motion 8 || RC=$?
    echo "-- onsite_record 5s"
    timeout 30 bash scripts/onsite_record.sh 5 || RC=$?
    ;;
  nav2_coexist)
    # 旧导航（外部命名空间）在 /cmd_vel 上发布：影子仍应放行候选速度。
    setsid python3 scripts/spoof_nav2_cmdvel.py --duration 60 \
      > "${LOG_DIR}/spoof_nav2.log" 2>&1 &
    SPOOF_PID=$!
    run_fake --duration 60 --send-goal
    sleep 6
    python3 scripts/check_shadow_graph.py --duration 10 --expect-healthy --expect-motion || RC=$?
    kill "${SPOOF_PID}" 2>/dev/null
    ;;
  cloud_stop)
    run_fake --duration 60 --send-goal --stop-cloud-after 12
    stop_check 11 30
    ;;
  invalid_cloud)
    run_fake --duration 60 --send-goal --nan-cloud-after 12
    stop_check 11 30
    if [[ "${RC}" -eq 0 ]] && ! grep -q "finite xyz" "${LOG_DIR}/launch.log"; then
      echo "FAIL: 全 NaN 点云没有被适配层按有效点检查拒绝" >&2
      RC=2
    fi
    ;;
  map_gate)
    # 门控行为测试：心跳来源在 11 s 被切断，适配器仍健康、候选仍在流，
    # guard 必须因"地图未更新"撤销任务、锁止并归零。
    run_fake --duration 60 --send-goal --fake-map-heartbeat-until 11
    stop_check 11 30 --expect-healthy-after 15
    if [[ "${RC}" -eq 0 ]] && ! grep -q "LATCHING" "${LOG_DIR}/launch.log"; then
      echo "FAIL: 地图停更没有触发撤销+锁止日志" >&2
      RC=2
    fi
    ;;
  map_relatch)
    # 地图停更 -> 撤销并锁止；心跳在 20 s 恢复但**不发新任务**：必须持续为零。
    run_fake --duration 60 --send-goal --fake-map-heartbeat-until 11 \
      --fake-map-heartbeat-resume 20
    stop_check 11 30 --expect-healthy-after 22
    if [[ "${RC}" -eq 0 ]] && grep -q "map latch cleared" "${LOG_DIR}/launch.log"; then
      echo "FAIL: 没有新任务却解除了地图锁止" >&2
      RC=2
    fi
    ;;
  map_relatch_newtask)
    # 同上，但 24 s 发新目标：锁止应在新任务授权（task_id 更大）且地图已恢复后解除。
    run_fake --duration 60 --send-goal --fake-map-heartbeat-until 11 \
      --fake-map-heartbeat-resume 20 --second-goal-after 24
    stop_check 11 36 --zero-until 23 --expect-healthy-after 22 \
      --expect-motion-after 26 --resume-at 23
    ;;
  odom_stop)
    run_fake --duration 60 --send-goal --stop-odom-after 12
    stop_check 11 30
    ;;
  tf_stop)
    run_fake --duration 60 --send-goal --stop-tf-after 12
    stop_check 11 30
    ;;
  stale_stamp)
    # 先正常运行产生运动，11 s 后冻结时间戳（旧 stamp 重发）。
    run_fake --duration 60 --send-goal --freeze-stamp-after 11
    stop_check 11 30
    ;;
  stamp_backwards)
    run_fake --duration 60 --send-goal --stamp-backwards-after 6
    stop_check 11 30
    ;;
  localization_jump)
    run_fake --duration 60 --send-goal --jump-after 12
    stop_check 11 30
    ;;
  recovery_no_resume)
    # 输入失效 -> 停车 -> 输入恢复：健康恢复但旧任务不得复活。
    run_fake --duration 60 --send-goal --stop-cloud-after 12 --resume-cloud-after 20
    stop_check 11 34 --expect-healthy-after 22
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
    run_fake --duration 60 --send-goal --cancel-after 12
    stop_check 11 30 --expect-healthy-after 15
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
