"""Offline display checks with fabricated answers, not live Jev evidence."""

from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from jev_cbf.reporting.rendering import add_scene_labels
from jev_cbf.paths import ROOT
from tests.fixtures.selection import fixture
import jev_cbf.legacy.view_selection as view_selection




class SceneTests(unittest.TestCase):
    def setUp(self):
        self.report = fixture()

    def test_recorded_geometry_and_world_frames_preserved(self):
        before = deepcopy(self.report)
        # Viewing must not call a model or regenerate today's candidate defaults.
        with patch("jev_cbf.legacy.selection.ask_jev", side_effect=AssertionError("network")), patch(
            "jev_cbf.geometry.covers.make_sphere_covers", side_effect=AssertionError("regeneration")
        ):
            scene = view_selection.selection_scene(self.report)
        self.assertEqual(self.report, before)
        model = mujoco.MjModel.from_xml_string(scene["xml"])
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertEqual((model.ngeom, model.nq), (16, 0))
        self.assertTrue(np.all(model.geom_contype == 0))
        self.assertTrue(np.all(model.geom_conaffinity == 0))
        for obj in self.report["request"]["state"]["objects"]:
            object_geom = model.geom(f"{obj['id']}/object")
            np.testing.assert_allclose(data.geom_xpos[object_geom.id], obj["center_m"])
            if obj["id"] == "far_cube":
                self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "far_cube/cover/0"), -1)
                continue
            candidate = next(c for c in obj["candidates"] if c["id"] == "box-split-4")
            for index, (position, radius) in enumerate(zip(candidate["centers_object_m"], candidate["radii_m"])):
                geom = model.geom(f"{obj['id']}/cover/{index}")
                np.testing.assert_allclose(data.geom_xpos[geom.id], np.array(obj["center_m"]) + position)
                self.assertEqual(geom.size[0], radius)
                self.assertEqual(geom.group, 1)
        self.assertIn("offline-fabricated-viewer-fixture", scene["caption"])
        self.assertIn("geom conf 0.50", scene["caption"])
        self.assertIn("NOT filtered", scene["caption"])

    def test_sweep_uses_actual_proxy_radius_and_recorded_endpoints(self):
        scene = view_selection.selection_scene(self.report)
        model = mujoco.MjModel.from_xml_string(scene["xml"])
        proxy = self.report["request"]["state"]["robot_proxy"]
        start, goal = np.array(proxy["start_m"]), np.array(proxy["goal_m"])
        sweep = model.geom("nominal/sweep")
        np.testing.assert_allclose(sweep.pos, (start + goal) / 2)
        np.testing.assert_allclose(sweep.size[:2], [proxy["radius_m"], np.linalg.norm(goal - start) / 2])
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, sweep.quat)
        axis = rotation.reshape(3, 3)[:, 2] * sweep.size[1]
        endpoints = np.array([sweep.pos - axis, sweep.pos + axis])
        self.assertTrue(np.allclose(endpoints, [start, goal]) or np.allclose(endpoints, [goal, start]))
        for name, position in (("start", start), ("goal", goal)):
            geom = model.geom(f"proxy/{name}")
            np.testing.assert_allclose(geom.pos, position)
            self.assertEqual(geom.size[0], proxy["radius_m"])
            self.assertEqual(geom.group, 3)
        self.assertEqual(model.geom("table/display-only").group, 4)

    def test_stationary_proxy_has_no_degenerate_capsule(self):
        proxy = self.report["request"]["state"]["robot_proxy"]
        proxy["goal_m"] = proxy["start_m"][:]
        scene = view_selection.selection_scene(self.report)
        model = mujoco.MjModel.from_xml_string(scene["xml"])
        self.assertEqual(model.ngeom, 14)
        self.assertNotIn('name="nominal/sweep"', scene["xml"])

    def test_previews_failures_and_disagreeing_cache_rejected(self):
        variants = [{"request": self.report["request"]}]
        for key in ("error", "validation_error"):
            variants.append({**self.report, key: "test failure"})
        invalid = deepcopy(self.report)
        invalid["proposals"][0]["role"] = "omit"
        variants.append(invalid)
        for report in variants:
            with self.subTest(report_keys=list(report)), self.assertRaises(ValueError):
                view_selection.selection_scene(report)

    def test_invalid_selected_geometry_rejected(self):
        for invalid_value in ([], [-1], [float("nan")]):
            report = deepcopy(self.report)
            report["request"]["state"]["objects"][0]["candidates"][2]["radii_m"] = invalid_value
            with self.subTest(radii=invalid_value), self.assertRaises(ValueError):
                view_selection.selection_scene(report)

    def test_world_labels_have_expected_positions(self):
        scene = view_selection.selection_scene(self.report)
        model = mujoco.MjModel.from_xml_string(scene["xml"])
        visual_scene = mujoco.MjvScene(model, maxgeom=20)
        add_scene_labels(visual_scene, scene["labels"])
        self.assertEqual(visual_scene.ngeom, 5)
        for index, (text, position) in enumerate(scene["labels"]):
            geom = visual_scene.geoms[index]
            self.assertEqual(geom.type, mujoco.mjtGeom.mjGEOM_LABEL)
            self.assertEqual(geom.label, text)
            np.testing.assert_allclose(geom.pos, position)
        first_label = visual_scene.geoms[0].label
        add_scene_labels(visual_scene, [("appended label", [1, 2, 3])], append=True)
        self.assertEqual(visual_scene.ngeom, 6)
        self.assertEqual(visual_scene.geoms[0].label, first_label)
        self.assertEqual(visual_scene.geoms[5].label, "appended label")
        with self.assertRaises(ValueError):
            add_scene_labels(mujoco.MjvScene(model, maxgeom=1), scene["labels"])

    def test_check_cli_does_not_open_viewer(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fabricated.json"
            path.write_text(json.dumps(self.report))
            with patch("jev_cbf.legacy.view_selection.show_scene", side_effect=AssertionError("GUI")), patch(
                "sys.stdout", new_callable=io.StringIO
            ) as output:
                self.assertEqual(view_selection.main(["--report", str(path), "--check"]), 0)
            self.assertIn("16 display geoms", output.getvalue())

    def test_cli_rejects_missing_report_and_invalid_duration(self):
        for args in (["--report", "/nonexistent/selection.json", "--check"], ["--seconds", "0"], ["--seconds", "nan"]):
            with self.subTest(args=args), patch("sys.stderr", new_callable=io.StringIO), self.assertRaises(SystemExit) as error:
                view_selection.main(args)
            self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
