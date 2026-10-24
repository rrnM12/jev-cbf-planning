"""Static adapter checks; fabricated selections are not model evidence."""

from copy import deepcopy
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from jev_cbf.paths import ROOT
from jev_cbf.upstream import bindings, load_upstream
from jev_cbf.control.simulation import collision_diagnostics, obstacle_spheres, scene_spec
from tests.fixtures.selection import fixture


class StaticAdapterTests(unittest.TestCase):
    def setUp(self):
        self.report = fixture()

    def test_replay_translates_only_selected_recorded_covers(self):
        before = deepcopy(self.report)
        centers, radii, included = obstacle_spheres(self.report, "replay")
        self.assertEqual((centers.shape, radii.shape), ((8, 3), (8,)))
        self.assertEqual([item["object_id"] for item in included], ["bottle", "cube"])
        obj = self.report["request"]["state"]["objects"][0]
        candidate = obj["candidates"][2]
        np.testing.assert_allclose(centers[:4], np.array(candidate["centers_object_m"]) + obj["center_m"])
        np.testing.assert_array_equal(radii[:4], candidate["radii_m"])
        self.assertEqual(before, self.report)

    def test_all_includes_omitted_objects_without_changing_selected_geometry(self):
        replay_centers, replay_radii, _ = obstacle_spheres(self.report, "replay")
        all_centers, all_radii, included = obstacle_spheres(self.report, "all")
        np.testing.assert_array_equal(all_centers[:8], replay_centers)
        np.testing.assert_array_equal(all_radii[:8], replay_radii)
        self.assertEqual(len(included), 3)
        self.assertEqual(len(all_radii), 12)

    def test_off_has_correct_empty_shapes_and_allows_explicit_baseline(self):
        centers, radii, included = obstacle_spheres(self.report, "off")
        self.assertEqual((centers.shape, radii.shape, included), ((0, 3), (0,), []))
        centers, radii, included = obstacle_spheres(self.report, "all", "box-split-1")
        self.assertEqual((centers.shape, radii.shape), ((3, 3), (3,)))
        self.assertTrue(all(item["candidate_id"] == "box-split-1" for item in included))

    def test_review_is_not_silently_treated_as_omit(self):
        report = self.report
        report.pop("proposals")
        answer = report["response"]["answers"]["role/bottle"]
        answer["choice"] = "review"
        answer["probabilities"] = {key: float(key == "review") for key in answer["probabilities"]}
        with self.assertRaisesRegex(ValueError, "unresolved review"):
            obstacle_spheres(report, "replay")

    def test_invalid_mode_and_missing_all_on_candidate_rejected(self):
        for mode, candidate in (("invalid", "conditional"), ("all", "missing")):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                obstacle_spheres(self.report, mode, candidate)

    def test_temporary_bindings_restore_on_failure(self):
        module = SimpleNamespace(value="original")
        with self.assertRaises(RuntimeError):
            with bindings(module, value="temporary"):
                self.assertEqual(module.value, "temporary")
                raise RuntimeError("test")
        self.assertEqual(module.value, "original")

    def test_scene_keeps_physical_objects_in_every_mode_and_covers_nonphysical(self):
        state = self.report["request"]["state"]
        frax_dir = ROOT.parent / "frax"
        if not (frax_dir / "examples/xml/scene.xml").exists():
            self.skipTest("read-only sibling FRAX checkout unavailable")
        for mode in ("off", "all", "replay"):
            with self.subTest(mode=mode):
                centers, radii, _ = obstacle_spheres(self.report, mode)
                model = scene_spec(mujoco, frax_dir, state, centers, radii).compile()
                self.assertEqual(model.geom("floor").type, mujoco.mjtGeom.mjGEOM_PLANE)
                for obj in state["objects"]:
                    geom = model.geom(f"scene/{obj['id']}")
                    np.testing.assert_allclose(geom.pos, obj["center_m"])
                    self.assertEqual(geom.contype, 1)
                    self.assertEqual(geom.conaffinity, 1)
                for index, radius in enumerate(radii):
                    geom = model.geom(f"cover/{index}")
                    self.assertEqual((geom.contype, geom.conaffinity), (0, 0))
                    self.assertEqual(geom.size[0], radius)

    def test_independent_collision_check_detects_unconstrained_object_penetration(self):
        model = mujoco.MjModel.from_xml_string('''
          <mujoco><worldbody>
            <geom name="scene/object" type="box" size="0.2 0.2 0.2"/>
            <body name="target" mocap="true" pos="3 0 0">
              <geom type="sphere" size="0.5" contype="0" conaffinity="0"/>
            </body>
            <body name="robot"><freejoint/>
              <geom type="sphere" size="0.1" mass="1"/>
              <geom type="sphere" size="2" contype="0" conaffinity="0"/>
            </body>
          </worldbody></mujoco>''')
        positions = np.array([[2, 0, 0, 1, 0, 0, 0], [.25, 0, 0, 1, 0, 0, 0]])
        result, distances = collision_diagnostics(mujoco, model, positions, np.zeros((2, 6)), ["object"])
        np.testing.assert_allclose(distances[:, 0], [1.7, -.05], atol=1e-6)
        self.assertTrue(result["any_sampled_object_penetration"])
        self.assertEqual(result["robot_collision_geom_count"], 1)
        self.assertEqual(result["penetrating_sample_count_by_object"]["object"], 1)
        self.assertEqual(result["contact_sample_count_by_object"]["object"], 1)
        self.assertTrue(result["any_sampled_scene_violation"])

    def test_moving_scene_bodies_are_not_robot_geometry_and_pose_history_is_independent(self):
        model = mujoco.MjModel.from_xml_string('''
          <mujoco><worldbody>
            <body name="target" mocap="true" pos="9 0 0"/>
            <body name="object" mocap="true"><geom name="scene/object" type="box" size=".2 .2 .2"/></body>
            <body name="robot" pos="2 0 0"><joint type="slide" axis="1 0 0"/>
              <geom type="sphere" size=".1" mass="1"/></body>
          </worldbody></mujoco>''')
        positions = np.array([[[0, 0, 0]], [[2.25, 0, 0]]])
        before = model.geom_pos.copy()
        result, distances = collision_diagnostics(mujoco, model, np.zeros((2, 1)), np.zeros((2, 1)),
                                                  ["object"], positions)
        self.assertEqual(result["robot_collision_geom_count"], 1)
        np.testing.assert_allclose(distances[:, 0], [1.7, -.05], atol=1e-6)
        np.testing.assert_array_equal(model.geom_pos, before)
        self.assertTrue(result["any_sampled_scene_violation"])

    def test_real_upstream_protection_parity_and_joint_frames(self):
        frax_dir, cbfpy_dir = ROOT.parent / "frax", ROOT.parent / "cbfpy"
        if not (frax_dir / "examples/oscbf_demo.py").exists() or not (cbfpy_dir / "cbfpy").exists():
            self.skipTest("read-only sibling sources unavailable")
        demo, _ = load_upstream(frax_dir, cbfpy_dir)
        from frax.robots.franka_panda import default_q
        robot = demo.load_panda()
        qs = np.array([default_q, default_q + .01])
        zs = np.concatenate([qs, np.zeros_like(qs)], axis=1)
        original = demo.DemoOSCBFConfig(robot, np.array([[.5, .5, .5]]), np.array([.3]))
        original_h = np.asarray(demo.jax.jit(demo.jax.vmap(original.h_2))(zs))
        protected = np.concatenate([original_h[:, :15], original_h[:, -28:]], axis=1)
        for mode in ("off", "all", "replay"):
            centers, radii, _ = obstacle_spheres(self.report, mode)
            config = demo.DemoOSCBFConfig(robot, centers, radii)
            actual = np.asarray(demo.jax.jit(demo.jax.vmap(config.h_2))(zs))
            np.testing.assert_array_equal(np.concatenate([actual[:, :15], actual[:, -28:]], axis=1), protected)
        spec = scene_spec(mujoco, frax_dir, self.report["request"]["state"], centers, radii)
        spec.attach(mujoco.MjSpec.from_file(str(frax_dir / "examples/xml/panda.xml")),
                    prefix="", frame=spec.worldbody.add_frame())
        model = spec.compile()
        data = mujoco.MjData(model)
        for q in qs:
            data.qpos[:7] = q
            mujoco.mj_forward(model, data)
            transforms = np.asarray(robot.joint_to_world_transforms(q))
            for index in range(7):
                np.testing.assert_allclose(transforms[index, :3, 3], data.xpos[model.body(f"link{index + 1}").id], atol=1e-9)
        # Positive control for the independent checker: deliberately place the
        # omitted object inside a physical robot geom, without running physics.
        physical = next(i for i in range(model.ngeom) if model.geom_bodyid[i] != 0
                        and (model.geom_contype[i] or model.geom_conaffinity[i]))
        model.geom("scene/far_cube").pos[:] = data.geom_xpos[physical]
        result, _ = collision_diagnostics(mujoco, model, data.qpos[None].copy(), data.qvel[None].copy(), ["far_cube"])
        self.assertEqual(result["nonpositive_distance_sample_count_by_object"]["far_cube"], 1)
        self.assertTrue(result["any_sampled_scene_violation"])
        # Recompile a moving-link overlap so the static broadphase is valid.
        moving = next(i for i in range(model.ngeom) if model.body_dofnum[model.geom_bodyid[i]] > 0
                      and (model.geom_contype[i] or model.geom_conaffinity[i]))
        spec.geom("scene/far_cube").pos = data.geom_xpos[moving].tolist()
        overlapping = spec.compile()
        result, _ = collision_diagnostics(mujoco, overlapping, data.qpos[None].copy(), data.qvel[None].copy(), ["far_cube"])
        self.assertEqual(result["penetrating_sample_count_by_object"]["far_cube"], 1)
        self.assertEqual(result["contact_sample_count_by_object"]["far_cube"], 1)


if __name__ == "__main__":
    unittest.main()
