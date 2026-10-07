#!/usr/bin/env bash
# Diagnostic runner, not an acceptance test. Never overwrites previous evidence.
# Usage: bash scripts/probe_navigation.sh NAME SECONDS [launch arguments...]
# SEND_GOAL=true GOAL_X=... GOAL_Y=... for Mode 1; default Mode 3 (no goal).
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
NAME=${1:?case name required}; DURATION=${2:?duration required}; shift 2
[[ "$NAME" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_HOME="$PWD/log/ros"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-71}
export ROS_LOCALHOST_ONLY=1
OUT="$PWD/log/probes/$NAME"
[[ ! -e "$OUT" ]] || { echo "Evidence already exists: $OUT"; exit 2; }
mkdir -p "$OUT"
LAUNCH_PID= REC_PID=
cleanup() {
  for pid in "$REC_PID" "$LAUNCH_PID"; do
    [[ -n "$pid" ]] && kill -INT -- "-$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "$REC_PID" "$LAUNCH_PID"; do
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
printf '%q ' "$@" > "$OUT/launch_args.txt"
REC_ARGS=()
if [[ "${ODOM_ONLY:-false}" == true ]]; then REC_ARGS+=(--odom-only); fi
setsid timeout --signal=INT --kill-after=3s "$((DURATION + 40))" \
  python3 scripts/scenario_test.py --scenario "$NAME" --duration "$DURATION" \
  --send-goal "${SEND_GOAL:-false}" --goal-x "${GOAL_X:-0}" --goal-y "${GOAL_Y:-7}" \
  --out "$OUT/run" "${REC_ARGS[@]}" > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
setsid bash scripts/run_sentry_sim.sh start_rviz:=false "$@" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
if [[ -n "${SNAPSHOT_AT:-}" ]]; then
  sleep "$SNAPSHOT_AT"
  python3 scripts/dump_cloud.py --topic /sentry_sim/grid_map/occupancy \
    --out "$OUT/occupancy.txt" --timeout 5
  python3 scripts/dump_cloud.py --topic /sentry_sim/grid_map/occupancy_inflate \
    --out "$OUT/inflate.txt" --timeout 5
fi
wait "$REC_PID"
cat "$OUT/run.txt"
