#!/usr/bin/env python3
"""Small, deterministic regressions; standard library only, no ROS required."""
import tempfile
import unittest
from pathlib import Path
from prepare_route_terrain import trace_lane
from make_terrain_route import Ground


def surfaces(*heights):
    return [(x,y,z) for x in (0.,.04,.08) for y in (0.,.04,.08) for z in heights]


class TerrainTests(unittest.TestCase):
    def trace(self, cells):
        return trace_lane(cells,0.,.083,.012,.081)

    def test_follow_ramp_above_persistent_floor(self):
        zs = [0.,.04,.08,.12,.16,.20]
        self.assertEqual(self.trace([surfaces(0.,z) for z in zs]),zs)

    def test_roof_not_selected_and_short_obstacle_not_ground(self):
        self.assertEqual(self.trace([surfaces(0.),surfaces(0.,.15),surfaces(0.,.40)]),[0.]*3)

    def test_missing_support_does_not_resume_behind_gap(self):
        self.assertEqual(self.trace([surfaces(0.),[],surfaces(0.)]),[0.,None,None])

    def test_cannot_climb_vertical_face(self):
        wall = surfaces(*[i*.04 for i in range(11)])
        self.assertEqual(self.trace([surfaces(0.),wall,surfaces(.4)]),[0.,0.,None])

    def test_roof_and_ramp_remain_separate_at_face(self):
        cells = [surfaces(0.),surfaces(0.,.04,.4),surfaces(0.,.08,.4),surfaces(0.,.12,.4)]
        self.assertEqual(self.trace(cells),[0.,.04,.08,.12])

    def test_grid_sample_origin_and_boundary_match_cpp(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'ground.txt'
            p.write_text('# cell 1 x0 0 y0 0 nx 2 ny 2\n0 1\n0 1\n')
            g=Ground(p)
            self.assertEqual(g.at(0,0),0.)
            self.assertEqual(g.at(1,0),1.)
            self.assertEqual(g.at(.5,.5),.5)
            self.assertEqual(g.at(-.5,.5),0.)


if __name__ == '__main__':
    unittest.main()
