"""Offline online-loop checks. All model answers here are fabricated."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from inspect_selection import ROOT, prepare_request
from online import ConstraintRuntime, motion_request, object_motion, query_saved, FaultInjectedJev
from robot_selection import robot_context, scene_input
from run_static import load_upstream, run_case
from run_suite import read


def fabricated(request, roles=None):
    answers = {}
    for name, question in request["questions"].items():
        choice = (roles or {}).get(name.split("/")[1], "avoid") if name.startswith("role/") else "box-split-4"
        answers[name] = {"type": "choice", "choice": choice, "confidence": 1.,
                         "probabilities": {key: float(key == choice) for key in question["criteria"]}}
    return {"model": "FABRICATED OFFLINE TEST", "answers": answers}


class OnlineTests(unittest.TestCase):
    def test_injected_unavailable_calls_are_not_sent_or_charged(self):
        called=[]
        with tempfile.TemporaryDirectory() as tmp:
            adapter=FaultInjectedJev(lambda request:called.append(request) or {'model':'FABRICATED OFFLINE TEST'},
                                     {'fail_calls':[2,3]},Path(tmp))
            request={'state':{}}
            adapter(request)
            for _ in range(2):
                with self.assertRaisesRegex(ValueError,'INJECTED'):adapter(request)
            adapter(request)
            self.assertEqual(len(called),2)
            self.assertEqual([e['logical_call'] for e in read(Path(tmp)/'model_fault_events.json')],[1,2,3,4])

    @classmethod
    def setUpClass(cls):
        cls.demo, cls.utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
        cls.robot = cls.demo.load_panda()
        cls.config = read(ROOT/"examples/dynamic_scenes.json")
        cls.scene = cls.config["scenes"][0]
        q = np.asarray(cls.robot.default_configuration)
        context = robot_context(cls.robot, {"qpos": q[None], "qvel": np.zeros((1, 7))})
        facts = scene_input(cls.scene, cls.config["goal_m"], np.asarray(cls.robot.ee_transform(q))[:3, 3])
        cls.request = prepare_request(facts, read(ROOT/"prompts/dynamic_selection.json"),
                                      robot_context=context, samples=1000)

    def test_motion_is_continuous_with_explicit_segment_velocities(self):
        for at, y, vy in ((0., -.28, 0.), (2., -.28, .06), (4., -.16, .06),
                          (6., -.04, 0.), (7., -.04, -.06), (11., -.28, 0.)):
            positions, velocities = object_motion(self.scene["objects"], at)
            self.assertAlmostEqual(positions[0, 1], y)
            self.assertAlmostEqual(velocities[0, 1], vy)
        objects = deepcopy(self.scene["objects"])
        objects[0]["motion"][1]["start_s"] = 3.
        with self.assertRaises(ValueError):
            object_motion(objects, 4.)

    def test_request_uses_current_measurements_without_future_script_or_scene_title(self):
        q = np.asarray(self.robot.default_configuration)
        positions, velocities = object_motion(self.scene["objects"], 4.)
        predict = self.demo.jax.jit(self.demo.jax.vmap(self.robot.link_collision_positions))
        request = motion_request(self.request, self.robot, q, np.zeros(7), positions, velocities,
                                 4., q+.01, 3., predict)
        self.assertEqual(request["state"]["snapshot_time_s"], 4.)
        self.assertEqual(request["state"]["objects"][0]["velocity_m_s"], [0., .06, 0.])
        self.assertNotIn("motion", request["state"]["objects"][0])
        self.assertNotIn("title", request["state"])
        for obj in request["state"]["objects"]:
            for c in obj["candidates"]:
                self.assertEqual(c, request["questions"][f"geometry/{obj['id']}"]["criteria"][c["id"]])

    def test_query_records_before_send_and_does_not_repeat(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            calls = []
            def ask(request):
                self.assertEqual(read(folder/"attempt.json")["status"], "started")
                calls.append(request)
                return fabricated(request)
            first = query_saved(folder, self.request, ask=ask)
            self.assertEqual(query_saved(folder, self.request, ask=ask), first)
            self.assertEqual(len(calls), 1)
            other = deepcopy(self.request); other["model"] = "changed"
            with self.assertRaises(ValueError):
                query_saved(folder, other, ask=ask)

    def test_failed_query_is_preserved_without_retry_or_fake_answer(self):
        with tempfile.TemporaryDirectory() as temporary:
            def fail(_):
                raise ValueError("fabricated failure")
            result = query_saved(Path(temporary), self.request, ask=fail)
            self.assertNotIn("response", result)
            self.assertIn("error", result)
            self.assertEqual(query_saved(Path(temporary), self.request, ask=lambda _: self.fail("retried")), result)

    def test_install_gate_retains_previous_set_on_review_failure_or_overlap(self):
        q = np.asarray(self.robot.default_configuration)
        with tempfile.TemporaryDirectory() as temporary:
            runtime = ConstraintRuntime(deepcopy(self.scene), self.config, self.request, None, q,
                                        "once", Path(temporary))
            runtime.setup(self.demo, self.robot, .05)
            report = {"request": self.request, "response": fabricated(self.request, {"object_a": "review", "object_b": "omit"})}
            runtime.install(report, q, np.zeros(7), 0., source="FABRICATED")
            self.assertEqual(runtime.events[-1]["status"], "accepted")
            self.assertEqual(runtime.policy, {"object_a": "box-split-4", "object_b": None})
            # A later omission removes existing rows, not just the initial
            # all-on fallback. This is a mechanism test, not live Jev evidence.
            omit = {"request": self.request,
                    "response": fabricated(self.request, {"object_a": "omit", "object_b": "omit"})}
            runtime.install(omit, q, np.zeros(7), 1., source="FABRICATED LATE REMOVAL")
            self.assertEqual(runtime.events[-1]["status"], "accepted")
            self.assertFalse(runtime.parameters(1.)[3].any())
            runtime.install(report, q, np.zeros(7), 1.1, source="FABRICATED REVIEW RETAINS OMIT")
            self.assertIsNone(runtime.policy["object_a"])
            restore = {"request": self.request,
                       "response": fabricated(self.request, {"object_a": "avoid", "object_b": "omit"})}
            runtime.install(restore, q, np.zeros(7), 1.2, source="FABRICATED RE-ADD")
            self.assertEqual(runtime.events[-1]["status"], "accepted")
            self.assertEqual(runtime.parameters(1.2)[3].sum(), 4)
            previous = runtime.policy.copy()
            runtime.install({"error": "FABRICATED"}, q, np.zeros(7), 0., source="FABRICATED")
            self.assertEqual(runtime.policy, previous)
            runtime.scene["objects"][0]["center_m"] = np.asarray(self.robot.link_collision_positions(q))[10].tolist()
            report["response"] = fabricated(self.request)
            runtime.install(report, q, np.zeros(7), 0., source="FABRICATED")
            self.assertEqual(runtime.events[-1]["status"], "rejected")
            self.assertEqual(runtime.policy, previous)
            # A warmed JIT receives new arrays rather than stale captured fields.
            h0 = np.asarray(runtime.h2_fn(np.r_[q, np.zeros(7), 0.], *runtime.parameters(0.)))
            h4 = np.asarray(runtime.h2_fn(np.r_[q, np.zeros(7), 4.], *runtime.parameters(4.)))
            self.assertGreater(np.max(abs(h0-h4)), .05)

    def test_bounded_real_physics_uses_runtime_arrays_and_preserves_robot_protections(self):
        q = np.asarray(self.robot.default_configuration)
        with tempfile.TemporaryDirectory() as temporary:
            runtime = ConstraintRuntime(self.scene, self.config, self.request, None, q,
                                        "all", Path(temporary))
            result, trajectory = run_case(self.demo, self.utils, {"request": self.request}, "off",
                                          frax_dir=ROOT.parent/"frax", steps=30,
                                          model_profile="frax", scene_guard=.05, runtime=runtime)
            self.assertTrue(result["complete"])
            self.assertEqual(result["collision"]["robot_collision_geom_count"], 23)
            self.assertFalse(result["collision"]["any_sampled_scene_violation"])
            self.assertGreater(result["minimum_joint_limit_barrier_rad"], 0.)
            self.assertGreater(result["minimum_floor_barrier_m"], 0.)
            self.assertEqual(trajectory["cover_centers_m"].shape, (31, 8, 3))
            self.assertEqual(result["allocated_scene_pair_count"], 168)


if __name__ == "__main__":
    unittest.main()
