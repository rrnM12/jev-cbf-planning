"""Offline robot-context feature checks; no live model or robot rollout."""

from copy import deepcopy
import json
import unittest

import numpy as np

from inspect_selection import ROOT, prepare_request, scene_motion
from robot_selection import add_motion_features, robot_context, scene_input
from run_static import obstacle_spheres


class FakeRobot:
    joint_names = ("test_joint",)
    collision_radii = ((.1, .1),)

    def link_collision_data(self, q):
        return np.array([[0, 0, 1], [0, 1, 1]]), np.array([.1, .1])


class RobotRequestTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.nominal = {"qpos": np.zeros((3, 9)), "qvel": np.zeros((3, 9)),
                        "ee_m": np.array([[0, 0, 1], [.5, 0, 1], [1, 0, 1]]), "time_s": np.array([0, 1, 2])}
        scene = {"id": "hidden_suite_id", "title": "EXPECTED AVOID: must not leak", "objects": [
            {"id": "object_a", "shape": "box", "size_m": [.2, .2, .2], "center_m": [.5, 0, 1], "expected_role": "avoid"}]}
        context = robot_context(self.robot, self.nominal)
        self.facts = scene_input(scene, [1, 0, 1], [0, 0, 1])
        template = json.loads((ROOT / "prompts/robot_selection.json").read_text())
        self.request = prepare_request(self.facts, template, robot_context=context, samples=1000)

    def test_robot_request_has_no_proxy_or_expected_scene_labels(self):
        state = self.request["state"]
        self.assertNotIn("robot_proxy", state)
        self.assertIn("robot_context", state)
        self.assertNotIn("EXPECTED", json.dumps(self.request))
        self.assertNotIn("hidden_suite_id", json.dumps(self.request))
        self.assertNotIn("expected_role", json.dumps(self.request))
        start, goal = scene_motion(state)
        np.testing.assert_array_equal(start, [0, 0, 1])
        np.testing.assert_array_equal(goal, [1, 0, 1])
        self.assertNotIn("nominal_clearance_m", state["objects"][0]["candidates"][0])

    def test_features_measure_all_robot_spheres_and_keep_choice_facts_equal(self):
        sphere_path = np.array([[[0, 0, 1], [0, 1, 1]], [[.5, 0, 1], [0, 1, 1]], [[1, 0, 1], [0, 1, 1]]])
        request = add_motion_features(self.request, self.robot, self.nominal, sphere_path,
                                      np.array([[.3], [-.1], [.3]]),
                                      {"nonpositive_distance_sample_count_by_object": {"object_a": 1},
                                       "closest_robot_body_by_object": {"object_a": "test_link"},
                                       "contact_sample_count_by_object": {"object_a": 1}})
        obj = request["state"]["objects"][0]
        self.assertTrue(obj["nominal_native_collision_features"]["contact_or_nonpositive_distance"])
        for c in obj["candidates"]:
            self.assertLess(c["nominal_path_robot_sphere_clearance_m"], 0)
            self.assertGreater(c["initial_robot_sphere_clearance_m"], 0)
            self.assertEqual(c["robot_primitive_constraint_pairs"], 2*c["primitive_count"])
            self.assertEqual(c, request["questions"]["geometry/object_a"]["criteria"][c["id"]])

    def test_baselines_do_not_need_fake_model_answers(self):
        report = {"mode": "robot_request_preview", "request": self.request}
        c, r, _ = obstacle_spheres(report, "off")
        self.assertEqual((c.shape, r.shape), ((0, 3), (0,)))
        c, r, _ = obstacle_spheres(report, "all", "box-split-4")
        self.assertEqual(len(r), 4)
        with self.assertRaises(ValueError):
            obstacle_spheres(report, "jev")


if __name__ == "__main__":
    unittest.main()
