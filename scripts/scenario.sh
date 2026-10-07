#!/usr/bin/env bash
# S3 场景编排：在同一个 shell 内启动仿真、记录证据、判定通过/失败、停止。
#
# 进程管理：仿真用 setsid 放进**独立进程组**，清理时只杀该进程组，
# 不会影响同一用户启动的其它同名 ROS 进程。
#
# 用法: scripts/scenario.sh <场景> [更多 launch 参数...]
#   mode1_lateral     Mode 1 + yaw_mode=hold，沿 +Y 横移到目标（车头不变）
#   mode1_align       Mode 1 + yaw_mode=align，对比朝向策略
#   mode2_waypoints   Mode 2 参数航点
#   mode3_path        Mode 3 参考路线
#   spin              Mode 2 + yaw_mode=spin（边转边走）
#   tight_pass        用户 PCD，贴障通过
#   cancel            Mode 1 后发 planning/reset 取消（判据：时限内停车且持续静止）
#   collision_block   目标落在障碍内
#   odom_loss         Mode 2 途中杀掉里程计（判据：断流后时限内停车且持续静止）
#   gap_wide          合成地图 0.80 m 通道（预期通过）
#   gap_narrow        合成地图 0.30 m 通道（预期拒绝）
#   gap_edge          合成地图 0.44 m 通道：中心线不碰、车体边缘会碰（预期拒绝）
#   low_obstacle      合成地图 0.15 m 矮障碍，不删点（预期拦停）
#   low_obstacle_cut  同一地图套用演示删点（预期被穿过 = 复现已知缺陷）
#   terrain_ramp10/20/30  合成 10/20/30° 上坡
#   terrain_down     同一 20° 坡反向（下坡）
#   terrain_crest    上坡->平台->下坡（跨越坡顶）
#   goal_out_of_grid 目标在已知地面网格之外：必须拒绝规划且机器人不动
#   cancel_race      规划期间取消 + 注入延迟旧授权与新时间戳轨迹，必须保持静止
#   terrain_lateral  横向坡面上绕障，检验轨迹高度与实际执行高度一致
#   terrain_tunnel_high  可控洞口：高洞应通过
#   terrain_tunnel_low   可控洞口：低洞应拒绝
#   terrain_field    真实场地 + 地形分离，Mode 3，z 跟随真实地面（起伏约 0.14 m）
#   terrain_field_mode1  同上但走 Mode 1（RViz 2D Goal Pose 链路）
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"
export ROS_HOME="${REPO_ROOT}/log/ros"
mkdir -p "${ROS_HOME}" "${REPO_ROOT}/log/scenarios"

CFG="${REPO_ROOT}/install/scan_planner/share/scan_planner/config"
MAPS="${REPO_ROOT}/docs/testing/maps"
SCENARIO="${1:?用法: scripts/scenario.sh <场景>}"
shift || true
OUT="${REPO_ROOT}/log/scenarios/${SCENARIO}"
LAUNCH_LOG="${OUT}_launch.log"

# 用户 PCD 的默认演示预处理
DEMO_MAP_ARGS=(pcd_map_file:="${HOME}/pcd_map/rmuc2026_field.pcd"
               map_offset_z:=-0.30 keep_z_min:=0.0 keep_z_max:=1.5)
# 合成测试地图：不平移、不删点（墙面 z∈[0,1]，矮箱 z∈[0,0.15] 必须保留）
SYN_MAP_ARGS=()

FAILURES=0
SEND_GOAL="${SEND_GOAL:-true}"
CANCEL_AFTER="${CANCEL_AFTER:--1}"

case "${SCENARIO}" in
  mode1_lateral)   MODE=1; EXTRA=(yaw_mode:=hold);  DUR=30; GOAL_X=-6.0; GOAL_Y=7.5; CHECK=goal ;;
  mode1_align)     MODE=1; EXTRA=(yaw_mode:=align); DUR=30; GOAL_X=-6.0; GOAL_Y=7.5; CHECK=goal ;;
  tight_pass)      MODE=1; EXTRA=(yaw_mode:=hold);  DUR=25; GOAL_X=-4.25; GOAL_Y=2.25; CHECK=goal ;;
  collision_block) MODE=1; EXTRA=(yaw_mode:=hold);  DUR=20; GOAL_X=-6.98; GOAL_Y=0.58; CHECK=none ;;
  # 目标在已知地面网格之外：必须**拒绝规划**且机器人不动（不把未知区域当可行驶地面）
  goal_out_of_grid)
                   MODE=1; EXTRA=(yaw_mode:=hold); DUR=25; GOAL_X=5.0; GOAL_Y=5.0; SEND_GOAL=true
                   CHECK=no_motion; STOP_DEADLINE=0.20; OBSERVE=8.0
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/tunnel/tunnel_high.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/tunnel/tunnel_high_ground.pcd
                                 ground_grid_file:=${MAPS}/tunnel/tunnel_high_ground.txt)
                   INIT_X=0.0; INIT_Y=-2.0 ;;
  cancel)          MODE=1; EXTRA=(yaw_mode:=hold);  DUR=25; GOAL_X=-6.0; GOAL_Y=7.5; CHECK=stop; STOP_DEADLINE=1.5; CANCEL_AFTER=4.0
                   OBSERVE=6.0 ;;
  # 规划计算期间取消 + 注入延迟到达的旧授权与"新时间戳"轨迹：必须全程保持静止
  cancel_race)     MODE=1; EXTRA=(yaw_mode:=hold); DUR=25; GOAL_X=-6.0; GOAL_Y=7.5; CHECK=stop
                   STOP_DEADLINE=0.20; CANCEL_AFTER=0.15; OBSERVE=6.0; INJECT_STALE=1 ;;
  mode2_waypoints) MODE=2; EXTRA=(keypoints_file:=${CFG}/sentry_waypoints.yaml); DUR=40; CHECK=goal; SEND_GOAL=false
                   GOAL_X=-6.0; GOAL_Y=7.5; TOL=0.30 ;;
  mode3_path)      MODE=3; EXTRA=(reference_path_file:=${CFG}/sentry_reference_path.yaml); DUR=40; CHECK=goal; SEND_GOAL=false
                   GOAL_X=-6.0; GOAL_Y=7.5; TOL=0.30 ;;
  spin)            MODE=2; EXTRA=(keypoints_file:=${CFG}/sentry_waypoints.yaml yaw_mode:=spin spin_rate:=0.5); DUR=40
                   CHECK=goal; SEND_GOAL=false; GOAL_X=-6.0; GOAL_Y=7.5; TOL=0.30; MIN_YAW=3.0 ;;
  odom_loss)       MODE=2; EXTRA=(keypoints_file:=${CFG}/sentry_waypoints.yaml); DUR=14; CHECK=stop; STOP_DEADLINE=1.2; SEND_GOAL=false ;;
  gap_wide)        MODE=1; EXTRA=(yaw_mode:=hold); DUR=30; GOAL_X=0.0; GOAL_Y=2.5; CHECK=passage; GATE_Y=1.5; EXPECT=pass
                   INIT_X=0.0; INIT_Y=-1.0; SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/gap_0.80.pcd map_offset_z:=0.0 keep_z_min:=0.0 keep_z_max:=1.2 publish_raw_cloud:=false) ;;
  gap_narrow)      MODE=1; EXTRA=(yaw_mode:=hold); DUR=30; GOAL_X=0.0; GOAL_Y=2.5; CHECK=passage; GATE_Y=0.5; EXPECT=refuse
                   INIT_X=0.0; INIT_Y=-1.0; SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/gap_0.30.pcd map_offset_z:=0.0 keep_z_min:=0.0 keep_z_max:=1.2 publish_raw_cloud:=false) ;;
  gap_edge)        MODE=1; EXTRA=(yaw_mode:=hold); DUR=30; GOAL_X=0.0; GOAL_Y=2.5; CHECK=passage; GATE_Y=0.5; EXPECT=refuse
                   INIT_X=0.0; INIT_Y=-1.0; SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/gap_0.44.pcd map_offset_z:=0.0 keep_z_min:=0.0 keep_z_max:=1.2 publish_raw_cloud:=false) ;;
  low_obstacle)    MODE=1; EXTRA=(yaw_mode:=hold); DUR=35; GOAL_X=0.0; GOAL_Y=2.5; CHECK=passage; GATE_Y=0.5; EXPECT=refuse
                   INIT_X=0.0; INIT_Y=-1.0; SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/low_obstacle.pcd map_offset_z:=0.0 keep_z_min:=0.0 keep_z_max:=1.2 publish_raw_cloud:=false) ;;
  # 同一张地图，但套用演示地图的绝对高度删点：矮墙被删掉，机器人会直接穿过去。
  # 该场景"通过"的含义是"已知缺陷被复现"，不是适配通过。
  low_obstacle_cut) MODE=1; EXTRA=(yaw_mode:=hold); DUR=35; GOAL_X=0.0; GOAL_Y=2.5; CHECK=passage; GATE_Y=0.5; EXPECT=pass
                   INIT_X=0.0; INIT_Y=-1.0; SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/low_obstacle.pcd map_offset_z:=-0.30 keep_z_min:=0.0 keep_z_max:=1.2 publish_raw_cloud:=false) ;;
  # 真实场地 + 地形分离：障碍云喂 SCAN，地形表面仅供 RViz 显示，z 跟随真实地面
  terrain_field)   MODE=3; EXTRA=(reference_path_file:=${MAPS}/field/field_slope_mode3.yaml)
                   DUR=45; CHECK=terrain; SEND_GOAL=false
                   TERRAIN_EXPECT=uphill; GOAL_X=-11.00; GOAL_Y=-1.75; TOL=0.60
                   MIN_RISE=0.10
                   INIT_X=-11.75; INIT_Y=-7.50; GROUND_GRID="${MAPS}/field/rmuc2026_ground.txt"
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/field/rmuc2026_obstacles.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.0 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/field/rmuc2026_surface.pcd
                                 ground_grid_file:=${MAPS}/field/rmuc2026_ground.txt) ;;
  # 真实场地 + 地形，Mode 1（RViz 2D Goal Pose 走的就是这条链路）
  terrain_field_mode1) MODE=1; EXTRA=(yaw_mode:=hold); DUR=40
                   GOAL_X=-11.0; GOAL_Y=-2.0; CHECK=goal
                   INIT_X=-11.50; INIT_Y=-7.00
                   GROUND_GRID="${MAPS}/field/rmuc2026_ground.txt"
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/field/rmuc2026_obstacles.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.0 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/field/rmuc2026_surface.pcd
                                 ground_grid_file:=${MAPS}/field/rmuc2026_ground.txt) ;;
  # 横向高度变化 + 绕障：检验"规划检查的高度"与"实际执行的高度"是否一致
  terrain_lateral) L=lateral_slope
                   MODE=3; EXTRA=(reference_path_file:=${MAPS}/lateral/${L}_mode3.yaml)
                   DUR=45; CHECK=tracked_height; SEND_GOAL=false
                   INIT_X=0.0; INIT_Y=-1.5
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/lateral/${L}.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.0 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/lateral/${L}_ground.pcd
                                 ground_grid_file:=${MAPS}/lateral/${L}_ground.txt) ;;
  # 可控洞口：高洞（洞顶离平台 0.45 m）应当通过
  terrain_tunnel_high)
                   T=tunnel_high
                   MODE=3; EXTRA=(reference_path_file:=${MAPS}/tunnel/tunnel_mode3.yaml)
                   DUR=55; CHECK=terrain; SEND_GOAL=false
                   TERRAIN_EXPECT=crest; GOAL_X=0.0; GOAL_Y=7.0; TOL=0.60
                   MIN_RISE=0.45; MIN_DROP=0.45
                   GROUND_GRID="${MAPS}/tunnel/${T}_ground.txt"
                   INIT_X=0.0; INIT_Y=-2.0
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/tunnel/${T}.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/tunnel/${T}_ground.pcd
                                 ground_grid_file:=${MAPS}/tunnel/${T}_ground.txt) ;;
  # 可控洞口：低洞（洞顶离平台 0.20 m，侵入机体高度带）必须拒绝并停在洞前
  terrain_tunnel_low)
                   T=tunnel_low
                   MODE=3; EXTRA=(reference_path_file:=${MAPS}/tunnel/tunnel_low_mode3.yaml)
                   DUR=45; CHECK=passage; SEND_GOAL=false; GATE_Y=2.20; EXPECT=refuse
                   INIT_X=0.0; INIT_Y=-2.0
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/tunnel/${T}.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/tunnel/${T}_ground.pcd
                                 ground_grid_file:=${MAPS}/tunnel/${T}_ground.txt) ;;
  # 下坡：同一条 20° 坡，路线反向
  terrain_down)    TMAP=ramp_20deg
                   MODE=3; EXTRA=(reference_path_file:=${MAPS}/terrain/${TMAP}_down_mode3.yaml)
                   DUR=45; CHECK=terrain; SEND_GOAL=false
                   TERRAIN_EXPECT=downhill; GOAL_X=0.0; GOAL_Y=-2.0; TOL=0.60; MIN_DROP=0.80
                   GROUND_GRID="${MAPS}/terrain/${TMAP}_ground.txt"
                   INIT_X=0.0; INIT_Y=5.0
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/terrain/${TMAP}.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/terrain/${TMAP}_ground.pcd
                                 ground_grid_file:=${MAPS}/terrain/${TMAP}_ground.txt) ;;
  # 跨越坡顶：上坡 -> 平台 -> 下坡
  terrain_crest)   TMAP=hill_20deg
                   MODE=3; EXTRA=(reference_path_file:=${MAPS}/terrain/${TMAP}_mode3.yaml)
                   DUR=55; CHECK=terrain; SEND_GOAL=false
                   TERRAIN_EXPECT=crest; GOAL_X=0.0; GOAL_Y=6.5; TOL=0.60
                   MIN_RISE=0.55; MIN_DROP=0.55
                   GROUND_GRID="${MAPS}/terrain/${TMAP}_ground.txt"
                   INIT_X=0.0; INIT_Y=-2.0
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/terrain/${TMAP}.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/terrain/${TMAP}_ground.pcd
                                 ground_grid_file:=${MAPS}/terrain/${TMAP}_ground.txt) ;;
  # 合成坡度地形：验证高度跟随。Mode 3 路线由地形网格生成（z = 地面，SCAN 再加 body_height）
  terrain_ramp10|terrain_ramp20|terrain_ramp30)
                   ANGLE="${SCENARIO#terrain_ramp}"; TMAP="ramp_${ANGLE}deg"
                   MODE=3; EXTRA=(reference_path_file:=${MAPS}/terrain/${TMAP}_mode3.yaml)
                   DUR=45; CHECK=terrain; SEND_GOAL=false
                   TERRAIN_EXPECT=uphill; GOAL_X=0.0; GOAL_Y=5.0; TOL=0.60
                   MIN_RISE="$(python3 -c "import math;print('%.3f'%(math.tan(math.radians(${ANGLE}))*3.0*0.8))")"
                   GROUND_GRID="${MAPS}/terrain/${TMAP}_ground.txt"
                   INIT_X=0.0; INIT_Y=-2.0
                   SYN_MAP_ARGS=(pcd_map_file:=${MAPS}/terrain/${TMAP}.pcd map_offset_z:=0.0
                                 keep_z_min:=-1.0 keep_z_max:=2.5 publish_raw_cloud:=false
                                 ground_file:=${MAPS}/terrain/${TMAP}_ground.pcd
                                 ground_grid_file:=${MAPS}/terrain/${TMAP}_ground.txt) ;;
  *) echo "未知场景: ${SCENARIO}" >&2; exit 2 ;;
esac

INIT_X="${INIT_X:--6.0}"; INIT_Y="${INIT_Y:-1.25}"
if [[ ${#SYN_MAP_ARGS[@]} -gt 0 ]]; then MAP_ARGS=("${SYN_MAP_ARGS[@]}"); else MAP_ARGS=("${DEMO_MAP_ARGS[@]}"); fi

set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/install/setup.bash"
set -u

LAUNCH_PGID=""
cleanup() {
  if [[ -n "${LAUNCH_PGID}" ]]; then
    kill -INT -"${LAUNCH_PGID}" 2>/dev/null || true
    sleep 2
    kill -KILL -"${LAUNCH_PGID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

start_sim() {
  setsid ./scripts/run_sentry_sim.sh start_rviz:=false "navi_mode:=${MODE}" \
    "init_x:=${INIT_X}" "init_y:=${INIT_Y}" "${EXTRA[@]}" "${MAP_ARGS[@]}" "$@" \
    > "${LAUNCH_LOG}" 2>&1 &
  sleep 1
  # setsid 建立了新会话，取其进程组号即本次启动的专属组
  LAUNCH_PGID="$(ps -o pgid= -p $! 2>/dev/null | tr -d ' ')"
  echo "[harness] 仿真进程组 PGID=${LAUNCH_PGID}"
}

echo "### 场景 ${SCENARIO}: navi_mode=${MODE} init=(${INIT_X}, ${INIT_Y}) ${EXTRA[*]:-}"

if [[ "${SCENARIO}" == "odom_loss" ]]; then
  start_sim "$@"
  python3 scripts/scenario_test.py --scenario odom_loss --duration "${DUR}" --out "${OUT}" \
    --send-goal "${SEND_GOAL}" --cancel-after "${CANCEL_AFTER}" \
    ${INJECT_STALE:+--inject-stale} > "${OUT}_recorder.log" 2>&1 &
  REC_PID=$!
  sleep 8
  EVENT_EPOCH="$(date +%s.%N)"
  echo "[odom_loss] 切断运动模拟器（body_pose 停止发布）t=${EVENT_EPOCH}"
  pkill -g "${LAUNCH_PGID}" -f go2_kinematic_sim 2>/dev/null || true
  wait "${REC_PID}" || true
  tail -20 "${OUT}_recorder.log"

  T0_EPOCH="$(grep -oP 't0_epoch=\K[0-9.]+' "${OUT}_meta.txt" 2>/dev/null || echo "")"
  if [[ -z "${T0_EPOCH}" ]]; then
    echo "失败：缺少 meta 中的 t0_epoch"; FAILURES=$((FAILURES+1))
  else
    EVENT_T="$(python3 -c "print(f'{${EVENT_EPOCH} - ${T0_EPOCH}:.4f}')")"
    if ! python3 scripts/check_stop.py --csv "${OUT}_cmdvel.csv" --event-t "${EVENT_T}" \
         --deadline "${STOP_DEADLINE}" --observe "${OBSERVE:-4.0}" --label "里程计断流"; then
      FAILURES=$((FAILURES+1))
    fi
  fi
else
  # 记录器先启动并等 body_pose 上线，避免漏掉 Mode 2/3 的自动起步阶段。
  python3 scripts/scenario_test.py --scenario "${SCENARIO}" --duration "${DUR}" \
    --goal-x "${GOAL_X:--6.0}" --goal-y "${GOAL_Y:-7.5}" --out "${OUT}" \
    --send-goal "${SEND_GOAL}" --cancel-after "${CANCEL_AFTER}" \
    ${INJECT_STALE:+--inject-stale} > "${OUT}_recorder.log" 2>&1 &
  REC_PID=$!
  sleep 1
  start_sim "$@"
  wait "${REC_PID}" || true
  tail -30 "${OUT}_recorder.log"

  if [[ "${CHECK}" == "stop" ]]; then
    CANCEL_T="$(grep -oP 'cancel_t=\K[0-9.]+' "${OUT}_meta.txt" 2>/dev/null || echo "")"
    if [[ -z "${CANCEL_T}" ]]; then
      echo "失败：缺少 meta 中的 cancel_t"; FAILURES=$((FAILURES+1))
    elif ! python3 scripts/check_stop.py --csv "${OUT}_cmdvel.csv" --event-t "${CANCEL_T}" \
           --deadline "${STOP_DEADLINE}" --observe "${OBSERVE:-6.0}" --label "取消"; then
      FAILURES=$((FAILURES+1))
    fi
  fi

  if [[ "${CHECK}" == "no_motion" ]]; then
    # 既要求机器人不动，也要求日志里确实出现了越界拒绝——否则"没动"可能只是没收到目标。
    if ! python3 scripts/check_no_motion.py --csv "${OUT}_cmdvel.csv" \
         --duration "${OBSERVE:-8.0}" --label "越界拒绝"; then
      FAILURES=$((FAILURES+1))
    fi
    if ! grep -q "outside the known ground grid" "${OUT}_launch.log"; then
      echo "失败：日志中没有出现越界拒绝，无法区分拒绝规划与没收到目标"
      FAILURES=$((FAILURES+1))
    fi
  fi

  if [[ "${CHECK}" == "passage" ]]; then
    if ! python3 scripts/check_passage.py --csv "${OUT}.csv" --gate-y "${GATE_Y}" \
         --expect "${EXPECT}"; then
      FAILURES=$((FAILURES+1))
    fi
  fi

  if [[ "${CHECK}" == "terrain" ]]; then
    if ! python3 scripts/check_terrain.py --csv "${OUT}.csv" --ground-grid "${GROUND_GRID}" \
         --body-height 0.125 --expect "${TERRAIN_EXPECT}" \
         --min-rise "${MIN_RISE:-0.10}" --min-drop "${MIN_DROP:-0.10}" \
         --goal-x "${GOAL_X}" --goal-y "${GOAL_Y}" --goal-tolerance "${TOL:-0.60}"; then
      FAILURES=$((FAILURES+1))
    fi
  fi

  if [[ "${CHECK}" == "tracked_height" ]]; then
    if ! python3 scripts/check_tracked_height.py --csv "${OUT}.csv" \
         --ground-grid "${MAPS}/lateral/lateral_slope_ground.txt" \
         --obstacles "${MAPS}/lateral/lateral_slope.pcd" \
         --body-height 0.125 --radius 0.26 --planned "${OUT}_planned.csv" \
         --min-lateral 0.30 --min-height-change 0.05 --require-counterfactual; then
      FAILURES=$((FAILURES+1))
    fi
  fi

  if [[ "${CHECK}" == "goal" ]]; then
    YAWARG=""
    [[ -n "${MIN_YAW:-}" ]] && YAWARG="--min-yaw-travel ${MIN_YAW}"
    if ! python3 scripts/check_passage.py --csv "${OUT}.csv" --gate-y 0.0 \
         --expect reach --goal-x "${GOAL_X}" --goal-y "${GOAL_Y}" --tolerance "${TOL:-0.20}" ${YAWARG}; then
      FAILURES=$((FAILURES+1))
    fi
  fi
fi

echo "### 关键日志"
grep -iE "DEMO MAP|Collision envelope|Lidar extrinsic|Task cancelled|Cancel latched|Rejecting|tracker ready|ERROR" \
  "${LAUNCH_LOG}" | head -12 || true

if [[ ${FAILURES} -gt 0 ]]; then
  echo "### 场景 ${SCENARIO} 失败项: ${FAILURES}"
  exit 1
fi
echo "### 场景 ${SCENARIO} 通过"
