import unittest
from dataclasses import replace

import numpy as np

from jev_cbf.geometry.adaptive import FitLimits, adaptive_covers, box_belief, validate_cover


class AdaptiveTests(unittest.TestCase):
    def test_every_whole_cell_is_enclosed_and_counts_emerge_from_geometry(self):
        limits = FitLimits(budget_s=2., max_split_checks=150)
        cells, side = box_belief([0., 0., 0.], [.05, .05, .4], limits)
        candidates, report = adaptive_covers(cells, side, limits=limits)
        self.assertLessEqual(len(candidates), 3)
        self.assertEqual(candidates[0]['primitive_count'], 1)
        self.assertGreater(candidates[-1]['primitive_count'], 4)
        self.assertLessEqual(candidates[-1]['primitive_count'], 10)
        for candidate in candidates:
            validate_cover(cells, side, candidate['centers_object_m'], candidate['radii_m'], limits.padding_m)
        self.assertLess(candidates[-1]['excess_volume_m3'], candidates[0]['excess_volume_m3'])
        self.assertEqual(report['completed_counts'], list(range(1, len(report['completed_counts'])+1)))
        with self.assertRaises(ValueError):
            validate_cover(cells, side, candidates[-1]['centers_object_m'], np.array(candidates[-1]['radii_m'])*.5)

    def test_work_and_cell_caps_preserve_a_completed_cover(self):
        limits = FitLimits(max_cells=100, max_split_checks=1, budget_s=2.)
        cells, side = box_belief([0., 0., 0.], [.2, .2, 1.], limits)
        self.assertLessEqual(len(cells), 100)
        self.assertGreaterEqual(cells[:, 2].max()+side/2, 1.)
        candidates, report = adaptive_covers(cells, side, limits=limits)
        self.assertEqual(report['split_checks'], 1)
        self.assertLessEqual(len(candidates[-1]['radii_m']), 2)
        for c in candidates:
            validate_cover(cells, side, c['centers_object_m'], c['radii_m'], limits.padding_m)
        with self.assertRaises(ValueError):
            FitLimits(max_spheres=0)
        with self.assertRaises(ValueError):
            adaptive_covers(np.full((3, 3), np.nan), side)
        with self.assertRaises(ValueError):
            adaptive_covers(np.zeros((2,3)),side)


if __name__ == '__main__':
    unittest.main()
