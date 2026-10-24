"""Real native contact physics; no model calls or fabricated pickup attachments."""

from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from jev_cbf.paths import ROOT
from jev_cbf.control.pickup import DEFAULTS, PickupRuntime, finger_envelope_spheres
from jev_cbf.legacy.requests import pickup_request
from jev_cbf.evaluation import pickup_collision_diagnostics
from jev_cbf.upstream import load_upstream
from jev_cbf.control.simulation import run_case
from jev_cbf.io import read
from jev_cbf.control.model import panda_spec


class PickupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.demo, cls.utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
        cls.robot = cls.demo.load_panda()
        cls.scene = deepcopy(read(ROOT/"configs/legacy/scenes/pickup_scenes.json")["scenes"][0])
        cls.scene["objects"] = cls.scene["objects"][:1]
        cls.request = pickup_request(cls.scene, DEFAULTS, cls.robot, read(ROOT/"configs/legacy/prompts/pickup_selection.json"), samples=1000)

    def test_articulated_profile_preserves_reference_mass_without_double_counting(self):
        mj, robot = self.utils.mujoco, self.robot
        model = panda_spec(mj, ROOT.parent/"frax", profile="frax_gripper", robot=robot).compile()
        self.assertEqual((model.nq, model.nv, model.nu), (9, 9, 8))
        self.assertEqual(model.neq, 1)  # Original finger coupling, NOT an object weld.
        data = mj.MjData(model)
        for q in (robot.default_configuration, robot.default_configuration+.01):
            data.qpos[:7], data.qpos[7:9] = q, .02
            mj.mj_forward(model, data)
            mass = np.empty((model.nv, model.nv)); mj.mj_fullM(model, data, mass)
            np.testing.assert_allclose(mass[:7, :7], np.asarray(robot.mass_matrix(q)), atol=1e-9)
            np.testing.assert_allclose(data.qfrc_bias[:7], np.asarray(robot.gravity_vector(q)), atol=1e-9)

    def test_task_rejects_missing_target_and_independently_moving_clutter(self):
        for kind in ("target", "motion"):
            scene = deepcopy(self.scene)
            if kind == "target":
                scene["target_id"] = "missing"
            else:
                scene["objects"][0]["motion"] = [{"start_s": 0, "end_s": 1, "velocity_m_s": [1, 0, 0]}]
            with self.assertRaises(ValueError):
                pickup_request(scene, DEFAULTS, self.robot, read(ROOT/"configs/legacy/prompts/pickup_selection.json"))

    def test_finger_envelopes_cover_native_hulls_at_intermediate_apertures(self):
        from itertools import product
        mj, robot = self.utils.mujoco, self.robot
        model = panda_spec(mj, ROOT.parent/"frax", profile="frax_gripper", robot=robot).compile()
        centers, radii = finger_envelope_spheres(mj, model, robot)
        data = mj.MjData(model)
        for aperture, offset in ((0., 0.), (.013, .01), (.027, -.01), (.04, .02)):
            data.qpos[:7], data.qpos[7:9] = robot.default_configuration+offset, aperture
            mj.mj_forward(model, data)
            tf = np.asarray(robot.ee_transform(data.qpos[:7]))
            world_centers = centers @ tf[:3, :3].T + tf[:3, 3]
            for g in range(model.ngeom):
                name = model.body(int(model.geom_bodyid[g])).name
                if name not in ("left_finger", "right_finger") or not model.geom_contype[g]:
                    continue
                if model.geom_type[g] == mj.mjtGeom.mjGEOM_BOX:
                    points = np.array(list(product((-1, 1), repeat=3)))*model.geom_size[g]
                else:
                    mesh = model.geom_dataid[g]
                    a, n = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                    points = model.mesh_vert[a:a+n]
                points = points @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
                clearance = np.linalg.norm(points[:, None]-world_centers[None], axis=-1)-radii
                self.assertLessEqual(clearance.min(axis=1).max(), 1e-9)

    def test_real_pickup_stops_at_verified_lift_and_distinguishes_allowed_contact(self):
        with tempfile.TemporaryDirectory() as tmp:
            runtime = PickupRuntime(self.scene, DEFAULTS, self.request, None, "all", Path(tmp))
            result, trace = run_case(self.demo, self.utils, {"request": self.request}, "off", frax_dir=ROOT.parent/"frax",
                                     steps=4000, runtime=runtime, scene_guard=.05, control_dt=.002)
            self.assertTrue(result["complete"])
            self.assertTrue(result["grasp_verified"])
            self.assertTrue(result["task_success"])
            self.assertLess(result["steps"], 4000)
            self.assertGreaterEqual(result["final_target_bottom_m"], DEFAULTS["pickup_bottom_lift_m"])
            self.assertTrue(trace["bilateral_finger_contact"][-1])
            self.assertTrue(np.all(trace["bilateral_finger_contact"][-120:]))
            self.assertLess(np.max(np.linalg.norm(np.diff(trace["target_center_m"], axis=0), axis=1)), .002)
            self.assertGreater(trace["qpos"][0, 7], trace["qpos"][-1, 7])
            self.assertTrue(result["collision"]["any_sampled_scene_contact"])
            self.assertFalse(result["collision"]["any_sampled_scene_violation"])
            self.assertEqual(trace["cover_enabled"].shape[1:], (29, 4))
            self.assertTrue(trace["cover_enabled"][:, :16].all())
            self.assertFalse(trace["cover_enabled"][:, 16:].any())
            # Native negative control: placing the free target in the palm must
            # not gain the finger contact exemption, regardless of selection.
            env = runtime.env
            q, v = trace["qpos"][-1].copy(), trace["qvel"][-1].copy()
            q[9:12] = env.data.xpos[env.model.body("hand").id]
            summary, _, _ = pickup_collision_diagnostics(self.utils.mujoco, env.model, q[None], v[None], self.scene["objects"], self.scene["target_id"])
            self.assertTrue(summary["any_forbidden_robot_target_contact"])
            self.assertTrue(summary["any_sampled_scene_violation"])

    def test_geometry_ablation_uses_conditional_ids_without_fake_role_decisions(self):
        scene = deepcopy(read(ROOT/"configs/legacy/scenes/pickup_scenes.json")["scenes"][0])
        request = pickup_request(scene, DEFAULTS, self.robot, read(ROOT/"configs/legacy/prompts/pickup_selection.json"), samples=1000, shape_aware=True)
        answers = {}
        for key, question in request["questions"].items():
            choice = "omit" if key.startswith("role/") else "box-split-1"
            answers[key] = {"type":"choice","choice":choice,"confidence":1.,"probabilities":{c:float(c==choice) for c in question["criteria"]}}
        report = {"request":request,"response":{"model":"FABRICATED OFFLINE CONTRACT TEST","answers":answers}}
        with tempfile.TemporaryDirectory() as tmp:
            runtime = PickupRuntime(scene, DEFAULTS, request, report, "geometry", Path(tmp))
            result, _ = run_case(self.demo,self.utils,{"request":request},"off",frax_dir=ROOT.parent/"frax",steps=20,runtime=runtime,scene_guard=.05)
            self.assertEqual(runtime.policy["bottle_a"], "box-split-1")
            self.assertEqual(runtime.policy["box_b"], "box-split-1")
            self.assertEqual(runtime.policy["target_object"], "box-split-4")
            self.assertEqual(result["selection_events"][0]["status"], "accepted")
            self.assertIn("role decisions ignored", result["selection_source"])

    def test_task_alignment_gates_and_no_lift_without_bilateral_contact(self):
        """Fabricated EE measurements isolate sequencing, not physical safety."""
        mj = self.utils.mujoco
        with tempfile.TemporaryDirectory() as tmp:
            runtime = PickupRuntime(self.scene,DEFAULTS,self.request,None,"all",Path(tmp))
            spec = runtime.scene_spec(mj,ROOT.parent/"frax",self.request["state"],[],[])
            spec.attach(runtime.robot_spec(mj,ROOT.parent/"frax"),prefix="",frame=spec.worldbody.add_frame())
            model=spec.compile();data=mj.MjData(model);data.qpos[:7]=self.robot.default_configuration
            env=SimpleNamespace(model=model,data=data,real_time=False,des_rot=np.diag([1.,-1.,-1.]))
            runtime.initialize(env);runtime.robot=self.robot
            pose=np.eye(4);pose[:3,:3]=env.des_rot
            twist=np.zeros(6)
            runtime.ee_kinematics=lambda q,v:(pose,twist)
            runtime.phase="descend";pose[:3,3]=runtime.waypoint()+[.012,0,0]
            runtime.after_step(env)
            self.assertEqual(runtime.phase,"pregrasp")
            runtime.apply_auxiliary_control(env)
            self.assertEqual(data.ctrl[7],255.)
            pose[:3,3]=runtime.waypoint();twist[0]=.1
            runtime.after_step(env)
            self.assertEqual(runtime.phase,"pregrasp")
            twist[:]=0
            runtime.after_step(env)
            self.assertEqual(runtime.phase,"descend")
            runtime.phase="close";runtime.phase_began=0;data.time=2.
            runtime.after_step(env)
            self.assertEqual(runtime.phase,"close")

    def test_eight_slot_runtime_and_terminal_features_use_real_robot_geometry(self):
        scene=deepcopy(read(ROOT/"configs/legacy/scenes/pickup_scenes.json")["scenes"][0])
        request=pickup_request(scene,DEFAULTS,self.robot,read(ROOT/"configs/legacy/prompts/pickup_terminal.json"),
                               counts=(1,2,3,4,5,6,8),shape_aware=True,terminal_features=True,samples=1000)
        request["state"]["motion"]["phase_joint_reference_rad"]={"descend":np.asarray(self.robot.default_configuration).tolist()}
        with tempfile.TemporaryDirectory() as tmp:
            runtime=PickupRuntime(scene,DEFAULTS,request,None,"all_fine",Path(tmp))
            result,_=run_case(self.demo,self.utils,{"request":request},"off",frax_dir=ROOT.parent/"frax",steps=2,runtime=runtime,scene_guard=.05)
            self.assertIn('no API',result['selection_source'])
            self.assertEqual(runtime.object_capacity,8)
            self.assertEqual(result["allocated_scene_pair_count"],29*24)
            policy={o["id"]:"box-split-8" for o in scene["objects"]}
            policy[scene["target_id"]]="box-split-4"
            params=runtime.parameters(0.,policy)
            self.assertEqual(int(params[3].sum()),64+29*16)
            updated=runtime.make_request(np.asarray(self.robot.default_configuration),np.zeros(7),0.)
            for obj in updated["state"]["objects"]:
                if obj["id"]==scene["target_id"]:continue
                for candidate in obj["candidates"]:
                    self.assertAlmostEqual(candidate["terminal_reference_clearance_m"]["descend"],candidate["current_robot_sphere_clearance_m"])
                    self.assertEqual(updated["questions"]["geometry/"+obj["id"]]["criteria"][candidate["id"]],candidate)

    def test_startup_selects_admissible_all_object_cover_and_keeps_target_guards(self):
        scene=deepcopy(read(ROOT/'configs/legacy/scenes/pickup_challenge_scenes.json')['scenes'][0])
        request=pickup_request(scene,DEFAULTS,self.robot,read(ROOT/'configs/legacy/prompts/pickup_terminal.json'),
                               counts=(1,2,3,4,5,6,8),shape_aware=True,samples=1000)
        with tempfile.TemporaryDirectory() as tmp:
            # Explicit offline failure tests fallback, not a fabricated Jev answer.
            runtime=PickupRuntime(scene,DEFAULTS,request,{'error':'OFFLINE simulated unavailable response'},'once',Path(tmp))
            result,trace=run_case(self.demo,self.utils,{'request':request},'off',frax_dir=ROOT.parent/'frax',
                                  steps=2,runtime=runtime,scene_guard=.05)
            startup=read(Path(tmp)/'startup.json')
            self.assertEqual(startup['status'],'accepted')
            self.assertTrue(all(startup['policy'].values()))
            self.assertNotEqual(startup['policy']['bottle_a'],'box-split-4')
            self.assertGreaterEqual(trace['scene_h2_m'].min(),0.)
            self.assertTrue(trace['cover_enabled'][:,:16,:4].all())
            self.assertTrue(result['complete'])

    def test_unsupported_startup_is_recorded_before_any_actuation(self):
        scene=deepcopy(read(ROOT/'configs/legacy/scenes/pickup_scenes.json')['scenes'][0])
        request=pickup_request(scene,DEFAULTS,self.robot,read(ROOT/'configs/legacy/prompts/pickup_terminal.json'),samples=1000)
        for obj in request['state']['objects']:
            if obj['id']==scene['target_id']:continue
            for candidate in obj['candidates']:candidate['radii_m']=[1000.]*len(candidate['radii_m'])
        with tempfile.TemporaryDirectory() as tmp:
            runtime=PickupRuntime(scene,DEFAULTS,request,None,'rule',Path(tmp))
            with patch.object(self.demo.ManipulatorEnv,'apply_control') as apply:
                with self.assertRaisesRegex(RuntimeError,'no controls executed.*startup rejected'):
                    run_case(self.demo,self.utils,{'request':request},'off',frax_dir=ROOT.parent/'frax',
                             steps=2,runtime=runtime,scene_guard=.05)
                apply.assert_not_called()
            startup=read(Path(tmp)/'startup.json')
            self.assertEqual(startup['status'],'rejected_before_actuation')
            self.assertFalse(runtime.startup_checked)
            self.assertTrue(startup['candidate_checks'])
            self.assertFalse(any(c['admissible'] for c in startup['candidate_checks']))

    def test_fixed_four_baseline_does_not_silently_change_representation(self):
        scene=deepcopy(read(ROOT/'configs/legacy/scenes/pickup_challenge_scenes.json')['scenes'][0])
        request=pickup_request(scene,DEFAULTS,self.robot,read(ROOT/'configs/legacy/prompts/pickup_terminal.json'),samples=1000)
        with tempfile.TemporaryDirectory() as tmp:
            runtime=PickupRuntime(scene,DEFAULTS,request,None,'all',Path(tmp))
            with self.assertRaisesRegex(RuntimeError,'no controls executed.*startup rejected'):
                run_case(self.demo,self.utils,{'request':request},'off',frax_dir=ROOT.parent/'frax',
                         steps=2,runtime=runtime,scene_guard=.05)
            self.assertTrue(all(v=='box-split-4' for v in runtime.policy.values()))


if __name__ == "__main__":
    unittest.main()
