#!/usr/bin/env python3
"""Check actual goal arrival, height change and cylinder clearance of a probe.

Uses all recorded odometry frames, not planner success messages. Obstacle checks
are independent of SCAN's voxel map, but still conditional on the input terrain
classification; this does not certify traction, support or real-vehicle safety.
"""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from prepare_terrain_map import read_pcd_xyz


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--obstacles', required=True)
    ap.add_argument('--goal', type=float, nargs=2, required=True)
    ap.add_argument('--height', type=float, required=True)
    ap.add_argument('--radius', type=float, default=.26)
    ap.add_argument('--tolerance', type=float, default=.15)
    ap.add_argument('--min-rise', type=float, default=0.)
    ap.add_argument('--min-drop', type=float, default=0.)
    ap.add_argument('--json-out')
    a = ap.parse_args()
    if not all(math.isfinite(v) for v in [*a.goal, a.height, a.radius, a.tolerance,
                                         a.min_rise, a.min_drop]) or min(a.height, a.radius) <= 0:
        ap.error('finite coordinates and positive robot dimensions required')
    if min(a.tolerance, a.min_rise, a.min_drop) < 0:
        ap.error('tolerance and required height changes must be nonnegative')
    with open(a.csv) as f:
        rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]
    if len(rows) < 100 or not all(math.isfinite(v) for r in rows for v in r.values()):
        ap.error('missing/nonfinite odometry')
    buckets = defaultdict(list)
    cell = a.radius
    for p in read_pcd_xyz(a.obstacles):
        buckets[(math.floor(p[0]/cell), math.floor(p[1]/cell))].append(p)
    collisions = 0
    first = None
    for r in rows:
        x,y,z = r['x'],r['y'],r['z']
        ix,iy = math.floor(x/cell),math.floor(y/cell)
        hit = False
        for j in range(iy-1,iy+2):
            for i in range(ix-1,ix+2):
                for px,py,pz in buckets.get((i,j), []):
                    if abs(pz-z) <= a.height/2+1e-6 and (px-x)**2+(py-y)**2 < a.radius**2:
                        hit = True
                        if first is None:
                            first = dict(t=r['t'], position=[x,y,z], obstacle=[px,py,pz])
                        break
        collisions += hit
    goal_error = math.dist([rows[-1]['x'],rows[-1]['y']],a.goal)
    zs = [r['z'] for r in rows]
    rise,drop = max(zs)-zs[0],max(zs)-zs[-1]
    duration = rows[-1]['t']-rows[0]['t']
    ok = (goal_error <= a.tolerance and rise >= a.min_rise and drop >= a.min_drop
          and collisions == 0 and duration >= 5)
    report = dict(passed=ok, frames=len(rows), duration=duration, goal_error=goal_error,
                  rise=rise,drop=drop,collision_frames=collisions,first_collision=first,
                  height=a.height,radius=a.radius,
                  limitation='clearance conditional on supplied obstacle classification; no dynamics')
    text = json.dumps(report,ensure_ascii=False,indent=2)
    print(text)
    if a.json_out:
        Path(a.json_out).write_text(text+'\n')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
