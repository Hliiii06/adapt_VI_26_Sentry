#!/usr/bin/env python3
"""Extract one entrance-connected support layer in an axis-aligned corridor.

Offline, explicitly bounded diagnostic map, NOT a whole-field terrain planner.
Dependencies: Python standard library and existing repository PCD/grid helpers.
The source PCD is never modified. Missing/unreachable support rejects export;
no guessed heights fill holes. The height grid ends at the corridor border.
"""
import argparse
import json
import math
from pathlib import Path

from prepare_terrain_map import read_pcd_xyz, write_pcd_xyz


def support_candidates(points, layer_gap, max_thickness):
    """Thin z clusters with horizontal coverage; a vertical column is not ground."""
    columns = {}
    for p in points:
        columns.setdefault((round(p[0], 4), round(p[1], 4)), []).append(p)
    # A vertical face contributes many z samples at the same XY. Do not allow
    # these columns to join the floor, ramp and roof into one thick 'layer'.
    points = []
    for column in columns.values():
        runs = []
        for p in sorted(column, key=lambda p: p[2]):
            if not runs or p[2] - runs[-1][-1][2] > .045:
                runs.append([])
            runs[-1].append(p)
        for run in runs:
            if len({round(v[2], 4) for v in run}) >= 6:
                # A face's interior is not support. Its top/bottom edges can
                # join adjacent surfaces, but cannot be jumped onto from below.
                points.extend((run[0], run[-1]))
            else:
                points.extend(run)
    groups = []
    for p in sorted(points, key=lambda p: p[2]):
        if not groups or p[2] - groups[-1][-1][2] > layer_gap:
            groups.append([])
        groups[-1].append(p)
    result = []
    for group in groups:
        if group[-1][2] - group[0][2] > max_thickness:
            continue
        xy = {(round(p[0], 4), round(p[1], 4)) for p in group}
        if len(xy) < 3:
            continue
        # Top of a thin sampled support band; never the top of a tall column.
        result.append(group[-1][2])
    return result


def trace_lane(cells, seed_z, max_delta, layer_gap, max_thickness):
    previous = seed_z
    result = []
    alive = True
    for points in cells:
        candidates = support_candidates(points, layer_gap, max_thickness)
        reachable = [z for z in candidates if abs(z - previous) <= max_delta + 1e-6]
        if alive and reachable:
            # Follow the upper reachable support instead of the persistent floor
            # underneath a ramp. A separated roof cannot be jumped onto.
            previous = max(reachable)
            result.append(previous)
        else:
            alive = False
            result.append(None)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', required=True)
    ap.add_argument('--out-prefix', required=True)
    ap.add_argument('--background-obstacles',
                    help='existing obstacle classification outside corridor; never used inside it')
    ap.add_argument('--axis', choices=['x', 'y'], required=True)
    ap.add_argument('--start', type=float, required=True, help='entrance coordinate on axis')
    ap.add_argument('--end', type=float, required=True)
    ap.add_argument('--cross-min', type=float, required=True)
    ap.add_argument('--cross-max', type=float, required=True)
    ap.add_argument('--seed-z', type=float, required=True, help='known entrance support height')
    ap.add_argument('--cell', type=float, default=.10)
    ap.add_argument('--max-slope', type=float, default=30., help='diagnostic continuity bound, degrees')
    ap.add_argument('--quantization', type=float, default=.025, help='PCD vertical quantization allowance')
    ap.add_argument('--layer-gap', type=float, default=.012,
                    help='cluster noise within a surface, less than this PCD\'s 4 cm z spacing')
    ap.add_argument('--max-thickness', type=float, default=.081)
    ap.add_argument('--surface-tolerance', type=float, default=.041,
                    help='one 4 cm source-PCD voxel; not a vehicle obstacle-climbing allowance')
    a = ap.parse_args()
    numeric = (a.start, a.end, a.cross_min, a.cross_max, a.seed_z, a.cell,
               a.max_slope, a.quantization, a.layer_gap, a.max_thickness, a.surface_tolerance)
    if not all(math.isfinite(v) for v in numeric):
        ap.error('all geometric parameters must be finite')
    if a.cell <= 0 or a.start == a.end or a.cross_min >= a.cross_max or not 0 <= a.max_slope < 60:
        ap.error('invalid corridor or continuity parameters')
    if min(a.quantization, a.surface_tolerance) < 0 or min(a.layer_gap, a.max_thickness) <= 0:
        ap.error('invalid surface tolerances')
    for span in (abs(a.end-a.start), a.cross_max-a.cross_min):
        cells_in_span = span/a.cell
        if cells_in_span < 1 or abs(cells_in_span-round(cells_in_span)) > 1e-6:
            ap.error('corridor spans must be positive whole multiples of cell size')
    prefix = Path(a.out_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    paths = [str(prefix) + suffix for suffix in ('_ground.txt', '_surface.pcd', '_obstacles.pcd', '_report.json')]
    if any(Path(p).exists() for p in paths):
        ap.error('output exists; choose a new prefix to preserve evidence')
    amin, amax = sorted((a.start, a.end))
    bounds = (amin, amax, a.cross_min, a.cross_max) if a.axis == 'x' else (a.cross_min, a.cross_max, amin, amax)
    xmin, xmax, ymin, ymax = bounds
    nx, ny = round((xmax-xmin)/a.cell)+1, round((ymax-ymin)/a.cell)+1
    cells = [[[] for _ in range(nx)] for _ in range(ny)]
    raw = read_pcd_xyz(a.input)
    background = set(read_pcd_xyz(a.background_obstacles)) if a.background_obstacles else None

    def index(x, y):
        i, j = math.floor((x-xmin)/a.cell+.5), math.floor((y-ymin)/a.cell+.5)
        return (i, j) if 0 <= i < nx and 0 <= j < ny else None

    for p in raw:
        ij = index(p[0], p[1])
        if ij is not None:
            i, j = ij
            cells[j][i].append(p)
    support = [[None] * nx for _ in range(ny)]
    max_delta = math.tan(math.radians(a.max_slope))*a.cell+a.quantization
    order = list(range(nx if a.axis == 'x' else ny))
    if a.end < a.start:
        order.reverse()
    for lane in range(ny if a.axis == 'x' else nx):
        ids = [(s,lane) if a.axis == 'x' else (lane,s) for s in order]
        zs = trace_lane([cells[j][i] for i,j in ids], a.seed_z, max_delta, a.layer_gap, a.max_thickness)
        for (i,j), z in zip(ids,zs):
            support[j][i] = z

    missing = [(round(xmin+i*a.cell,3), round(ymin+j*a.cell,3))
               for j in range(ny) for i in range(nx) if support[j][i] is None]
    if missing:
        ap.error(f'{len(missing)} unsupported cells; refusing to invent ground. First: {missing[:8]}')
    grid = support
    with open(paths[0], 'w') as f:
        f.write('# entrance-connected route support; all samples observed, bounded corridor only\n')
        f.write(f'# cell {a.cell:.6f} x0 {xmin:.6f} y0 {ymin:.6f} nx {nx} ny {ny}\n')
        for row in grid:
            f.write(' '.join(f'{z:.5f}' for z in row)+'\n')
    # Use precisely the same interpolation as C++ for classification.
    from check_tracked_height import make_lookup
    height_at = make_lookup(paths[0])
    obstacles = []
    removed = 0
    for p in raw:
        ij = index(p[0],p[1])
        if ij is None or not (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax):
            if background is None or p in background:
                obstacles.append(p)
            continue
        i,j = ij
        # Require all four interpolation neighbours to have known support.
        fi, fj = (p[0]-xmin)/a.cell, (p[1]-ymin)/a.cell
        i0,j0 = math.floor(fi),math.floor(fj)
        known = all(support[jj][ii] is not None
                    for jj in (j0,min(j0+1,ny-1)) for ii in (i0,min(i0+1,nx-1)))
        if known and p[2] <= height_at(p[0],p[1])+a.surface_tolerance:
            removed += 1
        else:
            obstacles.append(p)
    write_pcd_xyz(paths[2], obstacles)
    surface = [(xmin+i*a.cell,ymin+j*a.cell,z) for j,row in enumerate(support)
               for i,z in enumerate(row) if z is not None]
    write_pcd_xyz(paths[1], surface)
    report = dict(vars(a), source_points=len(raw), obstacle_points=len(obstacles),
                  support_or_below_removed=removed, supported_cells=len(surface),
                  total_cells=nx*ny, max_neighbor_delta=max_delta,
                  support_z_range=[min(p[2] for p in surface),max(p[2] for p in surface)] if surface else [],
                  scope='Only this entrance-connected corridor; no dynamics/whole-field certification')
    Path(paths[3]).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if surface else 1


if __name__ == '__main__':
    raise SystemExit(main())
