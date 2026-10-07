#!/usr/bin/env bash
# Raw-map, grid-free Mode 2 diagnostic. No terrain preparation or hardware I/O.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
PCD=${PCD_MAP_FILE:-${HOME}/pcd_map/rmuc2026_field.pcd}
[[ -f "$PCD" ]] || { echo "Missing raw PCD: $PCD" >&2; exit 2; }
read -r PCD_SHA _ < <(sha256sum "$PCD")
[[ "$PCD_SHA" == ebc0add17e2a229abed880926f4f01baf89ca9a8fe7a34d80b8b2b07b986adc7 ]] || {
  echo 'These fixed waypoints belong to the original rmuc2026 field PCD.' >&2; exit 2;
}
echo 'Mode 2: six XYZ goals, RAW map, NO ground/support grid.'
echo 'Diagnostic H=0.10 m cylinder has clearance above the floor; NOT a wheel-contact simulation.'
echo 'Direct spline odometry, not cmd_vel tracking. Automatic start. Stop with Ctrl-C.'
exec bash scripts/run_sentry_sim.sh \
  navi_mode:=2 execution_mode:=waypoint_z_preview \
  init_x:=1.9 init_y:=-6 init_z:=0.17 robot_height:=0.10 \
  keypoints_file:="$PWD/docs/testing/maps/field/south_tunnel_waypoint_z_mode2.yaml" \
  pcd_map_file:="$PCD" map_offset_z:=0 keep_z_min:=-10000 keep_z_max:=10000 \
  publish_raw_cloud:=true "$@"
