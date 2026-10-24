"""Geometry checks, not robot safety or perception validation."""

from dataclasses import replace
import io
from itertools import product
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from jev_cbf.geometry.covers import evaluate_covers, make_sphere_covers, sphere_union_contains
import jev_cbf.reporting.inspection as inspect_geometry


class SphereCoverTests(unittest.TestCase):
    def test_single_sphere_and_stable_ids_without_input_mutation(self):
        extents = np.array([0.035, 0.035, 0.15])
        original = extents.copy()
        covers = make_sphere_covers(extents)
        self.assertEqual([c.candidate_id for c in covers], ["box-split-1", "box-split-2", "box-split-4"])
        self.assertEqual(covers[0].centers_m, ((0.0, 0.0, 0.0),))
        self.assertAlmostEqual(covers[0].radii_m[0], np.linalg.norm(extents))
        np.testing.assert_array_equal(extents, original)
        self.assertEqual(covers, make_sphere_covers(extents))

    def test_every_section_corner_is_inside_its_own_sphere(self):
        # Independent corner tests in addition to the construction argument:
        # exercise all split axes, boundaries, aspect ratios, and non-power-of-2 counts.
        rng = np.random.default_rng(42)
        signs = np.array(list(product((-1, 1), repeat=3)))
        for extents in rng.uniform(0.001, 0.5, size=(100, 3)):
            for cover in make_sphere_covers(extents, counts=(1, 2, 3, 4, 7)):
                axis = int(np.argmax(extents))
                self.assertEqual(cover.split_axis, axis)
                edges = np.linspace(-extents[axis], extents[axis], len(cover.radii_m) + 1)
                for i, (center, radius) in enumerate(zip(cover.centers_m, cover.radii_m)):
                    corners = signs * extents
                    corners[:, axis] = np.where(signs[:, axis] < 0, edges[i], edges[i + 1])
                    distance = np.linalg.norm(corners - center, axis=1)
                    self.assertTrue(np.all(distance <= radius * (1 + 1e-14)))
                    self.assertTrue(sphere_union_contains(corners, cover).all())
                points = rng.uniform(-extents, extents, size=(100, 3))
                self.assertTrue(sphere_union_contains(points, cover).all())

    def test_axis_tie_is_deterministic(self):
        self.assertEqual(make_sphere_covers((0.1, 0.1, 0.1))[0].split_axis, 0)

    def test_invalid_dimensions_and_counts_are_rejected(self):
        for values in ((0, 1, 1), (-1, 1, 1), (np.nan, 1, 1), (np.inf, 1, 1), (1, 2)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                make_sphere_covers(values)
        for counts in ((), (0,), (-1,), (1.5,), (True,), (1, 1)):
            with self.subTest(counts=counts), self.assertRaises(ValueError):
                make_sphere_covers((0.1, 0.1, 0.2), counts)

    def test_membership_includes_boundary_and_rejects_bad_points(self):
        cover = make_sphere_covers((0.1, 0.1, 0.1), counts=(1,))[0]
        radius = cover.radii_m[0]
        np.testing.assert_array_equal(
            sphere_union_contains([[0, 0, 0], [radius, 0, 0], [2 * radius, 0, 0]], cover),
            [True, True, False],
        )
        self.assertEqual(sphere_union_contains(np.empty((0, 3)), cover).shape, (0,))
        for points in ([0, 0, 0], [[np.nan, 0, 0]], [[1, 2]]):
            with self.assertRaises(ValueError):
                sphere_union_contains(points, cover)


class VolumeTests(unittest.TestCase):
    def test_shared_samples_are_reproducible_and_overlap_is_not_double_counted(self):
        extents = (0.04, 0.04, 0.04)
        cover = make_sphere_covers(extents, counts=(1,))[0]
        duplicate = replace(
            cover, candidate_id="identical-two-spheres",
            centers_m=cover.centers_m * 2, radii_m=cover.radii_m * 2,
        )
        report = evaluate_covers(extents, (cover, duplicate), samples=10_000)
        self.assertEqual(report, evaluate_covers(extents, (cover, duplicate), samples=10_000))
        first, second = report["candidates"]
        self.assertEqual(first["union_volume_m3"], second["union_volume_m3"])
        self.assertEqual(first["excess_volume_m3"], second["excess_volume_m3"])
        self.assertEqual(first["uncovered_object_samples"], 0)

    def test_volume_estimates_match_independent_one_and_two_sphere_formulas(self):
        extents = (0.035, 0.035, 0.15)
        covers = make_sphere_covers(extents, counts=(1, 2))
        report = evaluate_covers(extents, covers, shape="cylinder", samples=100_000, seed=8)
        for cover, result in zip(covers, report["candidates"]):
            radius = cover.radii_m[0]
            exact = len(cover.radii_m) * 4 / 3 * np.pi * radius ** 3
            if len(cover.radii_m) == 2:
                distance = np.linalg.norm(np.subtract(*cover.centers_m))
                exact -= np.pi * (4 * radius + distance) * (2 * radius - distance) ** 2 / 12
            self.assertLess(
                abs(result["union_volume_m3"] - exact),
                6 * result["union_volume_standard_error_m3"],
            )
            self.assertLess(
                abs(result["excess_volume_m3"] - (exact - report["object_volume_m3"])),
                6 * result["excess_volume_standard_error_m3"],
            )
            self.assertEqual(result["uncovered_object_samples"], 0)

    def test_sampling_domain_encloses_all_spheres_and_reference_volumes_are_exact(self):
        extents = (0.035, 0.035, 0.15)
        covers = make_sphere_covers(extents)
        for shape, exact in (
            ("box", 8 * np.prod(extents)),
            ("cylinder", np.pi * extents[0] ** 2 * 2 * extents[2]),
        ):
            report = evaluate_covers(extents, covers, shape=shape, samples=1000)
            self.assertAlmostEqual(report["object_volume_m3"], exact)
            lower, upper = np.array(report["sampling_aabb_m"])
            for cover in covers:
                centers = np.array(cover.centers_m)
                radii = np.array(cover.radii_m)[:, None]
                self.assertTrue(np.all(centers - radii >= lower))
                self.assertTrue(np.all(centers + radii <= upper))
            self.assertGreater(upper[2], covers[0].radii_m[0])

    def test_invalid_reference_or_sampling_parameters_are_rejected(self):
        covers = make_sphere_covers((0.1, 0.1, 0.2))
        for kwargs in ({"samples": 0}, {"samples": True}, {"shape": "mesh"}, {"seed": -1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                evaluate_covers((0.1, 0.1, 0.2), covers, **kwargs)
        with self.assertRaises(ValueError):
            evaluate_covers((0.1, 0.2, 0.3), covers, shape="cylinder")
        with self.assertRaises(ValueError):
            evaluate_covers((0.1, 0.1, 0.2), ())


class InspectionTests(unittest.TestCase):
    def test_headless_cli_records_geometry_and_parameters_without_opening_viewer(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            with patch.object(inspect_geometry, "show_covers") as viewer, patch("sys.stdout", new_callable=io.StringIO):
                self.assertEqual(inspect_geometry.main([
                    "--no-view", "--samples", "1000", "--report", str(path),
                ]), 0)
            viewer.assert_not_called()
            report = json.loads(path.read_text())
        self.assertEqual(report["samples"], 1000)
        self.assertEqual(report["reference_shape"], "cylinder")
        self.assertEqual(len(report["candidates"]), 3)
        self.assertIn("union", report["coverage_basis"])

    def test_cli_rejects_invalid_geometry_before_viewer(self):
        for args in (["--size", "0", "1", "1"], ["--seconds", "nan"], ["--counts", "1", "1"]):
            with patch("sys.stderr", new_callable=io.StringIO), self.assertRaises(SystemExit) as error:
                inspect_geometry.main(args)
            self.assertEqual(error.exception.code, 2)

    def test_viewer_scene_compiles_and_preserves_local_dimensions(self):
        import mujoco

        extents = np.array([0.035, 0.035, 0.15])
        covers = make_sphere_covers(extents)
        for shape in ("box", "cylinder"):
            xml, _, _ = inspect_geometry.comparison_xml(extents, covers, shape)
            model = mujoco.MjModel.from_xml_string(xml)
            self.assertEqual(model.ngeom, 3 + 1 + 2 + 4)
            self.assertTrue(np.all(model.geom_contype == 0))
            for cover in covers:
                body = model.body(cover.candidate_id)
                for i, (center, radius) in enumerate(zip(cover.centers_m, cover.radii_m)):
                    geom = model.geom(f"{cover.candidate_id}-sphere-{i}")
                    self.assertEqual(geom.bodyid, body.id)
                    np.testing.assert_allclose(geom.pos, center)
                    self.assertAlmostEqual(geom.size[0], radius)


if __name__ == "__main__":
    unittest.main()
