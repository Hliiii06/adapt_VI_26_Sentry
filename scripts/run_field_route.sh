#!/usr/bin/env bash
# Known-corridor RViz entry. Does not send a goal, UART or real-robot commands.
# Usage: bash scripts/run_field_route.sh large_ramp [launch arguments...]
#        bash scripts/run_field_route.sh small_tunnel robot_height:=0.10
#        bash scripts/run_field_route.sh south_tunnel robot_height:=0.10
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
CASE=${1:?choose large_ramp, small_tunnel or south_tunnel}; shift
PCD=${PCD_MAP_FILE:-${HOME}/pcd_map/rmuc2026_field.pcd}
BACKGROUND="$PWD/docs/testing/maps/field/rmuc2026_local_obstacles.pcd"
[[ -f "$PCD" && -f "$BACKGROUND" ]] || { echo 'Missing field PCD/background obstacles' >&2; exit 2; }
# Coordinates and the background classification below belong to this exact map.
# A relocated copy is fine; a changed map needs a new corridor assessment.
read -r PCD_SHA _ < <(sha256sum "$PCD")
read -r BG_SHA _ < <(sha256sum "$BACKGROUND")
if [[ "$PCD_SHA" != ebc0add17e2a229abed880926f4f01baf89ca9a8fe7a34d80b8b2b07b986adc7 ||
      "$BG_SHA" != 40525d6f0fbee1392c495a35f32dda23d6d7b0ebde35dfe08fe87912249d0c80 ]]; then
  echo 'Map identity changed; these fixed corridors/background must be reassessed.' >&2
  exit 2
fi
case "$CASE" in
  large_ramp)
    PREP=(--axis x --start 6 --end 10 --cross-min 3.8 --cross-max 4.9 --seed-z .06)
    INIT=(init_x:=6.5 init_y:=4.2); GOAL='(9.0, 4.2)';;
  small_tunnel)
    PREP=(--axis y --start 3.6 --end 6.6 --cross-min 4.3 --cross-max 5.1 --seed-z .06)
    INIT=(init_x:=4.7 init_y:=4.1); GOAL='(4.7, 6.1)';;
  south_tunnel)
    PREP=(--axis x --start 2.2 --end -2 --cross-min -6.3 --cross-max -5.7 --seed-z .02)
    INIT=(init_x:=1.6 init_y:=-6.0); GOAL='(-1.5, -6.0)';;
  *) echo "Unknown corridor: $CASE" >&2; exit 2;;
esac
mkdir -p artifacts/field_routes
RUN_DIR=$(mktemp -d "$PWD/artifacts/field_routes/${CASE}.XXXXXX")
python3 scripts/prepare_route_terrain.py --input "$PCD" --background-obstacles "$BACKGROUND" \
  --out-prefix "$RUN_DIR/map" "${PREP[@]}"
echo "Generated bounded corridor: $RUN_DIR"
echo "RViz: select 2D Goal Pose near $GOAL. Default robot height remains 0.25 m."
echo 'Tunnel feasibility demo: explicitly set robot_height:=0.10; this is NOT the real vehicle size.'
exec bash scripts/run_sentry_sim.sh navi_mode:=1 "${INIT[@]}" \
  pcd_map_file:="$RUN_DIR/map_obstacles.pcd" ground_file:="$RUN_DIR/map_surface.pcd" \
  ground_grid_file:="$RUN_DIR/map_ground.txt" map_offset_z:=0 keep_z_min:=-1 keep_z_max:=2.5 \
  publish_raw_cloud:=false "$@"
