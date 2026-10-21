"""Independent boundary/cell checks for new optional geometry candidates."""

from itertools import product
import unittest
import numpy as np

from geometry import make_shape_covers, make_sphere_covers, evaluate_covers


class ShapeCoverTests(unittest.TestCase):
    def test_original_candidates_preserved_and_ids_unique(self):
        for shape, half in (("box", [.04]*3), ("cylinder", [.035, .035, .15])):
            covers = make_shape_covers(half, shape=shape)
            self.assertEqual(covers[:3], make_sphere_covers(half))
            self.assertEqual(len({c.candidate_id for c in covers}), len(covers))
            self.assertTrue(all(len(c.radii_m) <= 4 for c in covers))

    def test_cylinder_section_boundary_rings_inside_their_own_spheres(self):
        half = np.array([.035, .035, .15])
        angles = np.linspace(0, 2*np.pi, 513)
        for cover in make_shape_covers(half, shape="cylinder")[3:]:
            count = len(cover.radii_m)
            for center, radius in zip(cover.centers_m, cover.radii_m, strict=True):
                for z in (center[2]-half[2]/count, center[2]+half[2]/count):
                    ring = np.c_[half[0]*np.cos(angles), half[0]*np.sin(angles), np.full(len(angles), z)]
                    self.assertLessEqual((np.linalg.norm(ring-center, axis=1)-radius).max(), 1e-15)

    def test_box_grid_section_corners_and_independent_volume_diagnostics(self):
        half = np.array([.04]*3)
        cover = make_shape_covers(half, shape="box")[-1]
        self.assertEqual(cover.candidate_id, "box-grid-4")
        for center, radius in zip(cover.centers_m, cover.radii_m, strict=True):
            corners = np.array(list(product((-1, 1), repeat=3)))*[.02, .02, .04]+center
            self.assertLessEqual((np.linalg.norm(corners-center, axis=1)-radius).max(), 1e-15)
        for shape, dims in (("box", half), ("cylinder", [.035, .035, .15])):
            result = evaluate_covers(dims, make_shape_covers(dims, shape=shape), shape=shape, samples=100_000, seed=4)
            self.assertTrue(all(c["uncovered_object_samples"] == 0 for c in result["candidates"]))

    def test_invalid_shape_and_non_circular_cylinder_rejected(self):
        for shape, half in (("ellipsoid", [.1]*3), ("cylinder", [.1, .2, .3])):
            with self.assertRaises(ValueError):
                make_shape_covers(half, shape=shape)

    def test_richer_counts_and_box_grids_have_full_cell_enclosures(self):
        counts=(1,2,3,4,5,6,8)
        for shape in ("box","cylinder"):
            covers=make_shape_covers([.04,.04,.15],shape=shape,counts=counts)
            self.assertEqual(len({c.candidate_id for c in covers}),len(covers))
            self.assertTrue(any(len(c.radii_m)==8 for c in covers))
            result=evaluate_covers([.04,.04,.15],covers,shape=shape,samples=100_000,seed=15)
            self.assertTrue(all(c["uncovered_object_samples"]==0 for c in result["candidates"]))
        cover=next(c for c in make_shape_covers([.04]*3,shape="box",counts=counts) if c.candidate_id=="box-grid-8")
        for center,radius in zip(cover.centers_m,cover.radii_m,strict=True):
            corners=np.array(list(product((-1,1),repeat=3)))*.02+center
            self.assertLessEqual((np.linalg.norm(corners-center,axis=1)-radius).max(),1e-15)
