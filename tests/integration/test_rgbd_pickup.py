from jev_cbf.paths import ROOT
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from jev_cbf.geometry.adaptive import FitLimits
from jev_cbf.sensing.camera_io import CameraFrame, Intrinsics
from jev_cbf.control.rgbd import CompletionSchedule, RGBDPickupRuntime
from jev_cbf.sensing.rgbd import SceneEstimator, components, detect


class RGBDTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary_root=ROOT/'artifacts'
        cls.temporary_root.mkdir(exist_ok=True)

    def test_completion_driven_400_and_700_ms_schedules(self):
        schedule = CompletionSchedule()
        schedule.start(0.)
        self.assertFalse(schedule.ready(100.))
        schedule.complete()
        self.assertFalse(schedule.ready(.4))
        self.assertTrue(schedule.ready(.5))
        schedule.start(.5)
        with self.assertRaises(ValueError):
            schedule.start(.7)
        schedule.complete()
        self.assertTrue(schedule.ready(1.2))  # 700 ms request: no extra wait.
        schedule.start(1.2)
        with self.assertRaises(ValueError):
            CompletionSchedule(.49)

    def test_default_difficulty_counts_exclude_the_pickup_target(self):
        from jev_cbf.paths import ROOT
        from jev_cbf.experiments.rgbd import configured_clutter_counts
        from jev_cbf.io import read
        config = read(ROOT/'configs/scenes/rgbd_pickup.json')
        self.assertEqual(configured_clutter_counts(config), dict(easy=1, medium=2, hard=3))
        self.assertEqual(config['clutter_counts'], dict(easy=1, medium=2, hard=3))

    def test_incorrect_declared_obstacle_count_refuses_before_perception(self):
        from unittest.mock import patch
        from jev_cbf.experiments.rgbd import prepare
        from jev_cbf.io import save
        config = dict(clutter_counts=dict(easy=1), scenes=[dict(id='easy', target_id='target', objects=[dict(id='target')])])
        with tempfile.TemporaryDirectory(dir=self.temporary_root) as temp:
            root = Path(temp)
            save(root/'config.json', config)
            with patch('jev_cbf.experiments.rgbd.load_upstream') as load:
                with self.assertRaisesRegex(ValueError, 'clutter counts'):
                    prepare(root/'run', root/'config.json')
                load.assert_not_called()
            self.assertFalse((root/'run').exists())

    def test_rgb_component_inventory_and_target_missing_are_not_oracle_filled(self):
        rgb = np.zeros((30, 40, 3), np.uint8)
        rgb[8:19, 10:19] = [200, 150, 55]  # highlight remains orange
        rgb[8:22, 26:35] = [40, 170, 190]
        intr = Intrinsics(40, 30, 300., 300., 20., 15.)
        frame = CameraFrame('rgb', 0, 0., 'test', rgb, np.full((30, 40), .12), intr, 'optical')
        T = np.eye(4); T[:3, 3] = [.45, 0., 0.]
        estimator = SceneEstimator(limits=FitLimits(budget_s=1.))
        estimate = estimator.initialize([(frame, T)])
        self.assertEqual(len(estimate['objects']), 2)
        self.assertEqual(estimate['target_id'], 'observed_target')
        self.assertNotIn('target_object', str(estimate))
        self.assertTrue(all(o['shape'] == 'box' for o in estimate['objects']))
        empty = CameraFrame('rgb', 1, .1, 'test', np.zeros_like(rgb), frame.depth_m, intr, 'optical')
        missing = estimator.track_target([(empty, T)])
        self.assertFalse(missing['fresh'])
        with self.assertRaises(ValueError):
            SceneEstimator().initialize([(empty, T)])

    def test_rgb_connected_pixels_separate_at_a_metric_depth_edge(self):
        mask = np.ones((4, 8), bool)
        points = np.zeros((4, 8, 3)); points[:, 4:, 2] = .2
        groups = components(mask, minimum=4, points=points)
        self.assertEqual(sorted(map(len, groups)), [16, 16])

    def test_late_model_response_is_retained_in_the_saved_event_log(self):
        from concurrent.futures import Future
        from unittest.mock import Mock
        from jev_cbf.io import read
        with tempfile.TemporaryDirectory(dir=self.temporary_root) as temp:
            runtime=RGBDPickupRuntime.__new__(RGBDPickupRuntime)
            runtime.camera_executor=None;runtime.model_executor=Mock()
            runtime.model_future=Future();runtime.model_future.set_result(({},'queries/001/selection.json',{'latency':.7}))
            runtime.request_timings=[];runtime.events=[];runtime.output=Path(temp)
            runtime.close()
            self.assertEqual(read(Path(temp)/'events.json')[0]['status'],'not applied after rollout')
            self.assertEqual(len(runtime.request_timings),1)

    def test_five_conditions_share_target_guards_and_apply_independent_ablations(self):
        from unittest.mock import patch
        from jev_cbf.control.rgbd import observed_request
        # Fabricated responses are used only in this contract test.
        obj = dict(id='observed_target', shape='box', center_m=[.5, 0., .06], size_m=[.04, .04, .12],
                   candidates=[dict(id='adaptive-1', centers_object_m=[[0., 0., 0.]], radii_m=[.08]),
                               dict(id='adaptive-3', centers_object_m=[[0., 0., -.04],[0.,0.,0.],[0.,0.,.04]],radii_m=[.04]*3)])
        other = deepcopy(obj); other['id']='observed_00'; other['center_m']=[.4,.2,.1]
        estimate=dict(objects=[obj,other],target_id=obj['id'],support_z_m=0.)
        state=dict(objects=deepcopy(estimate['objects']),motion=dict(goal_m=[.5,0.,.095]))
        request=dict(state=state,questions={},model='jev-1.13.0')
        supplied=dict(request=dict(state=dict(objects=[other])))
        proposal=[dict(object_id=other['id'],role='avoid',candidate_id='adaptive-3',conditional_candidate_id='adaptive-3')]
        with tempfile.TemporaryDirectory(dir=self.temporary_root) as temp:
            for method, expected in [('all_jev','adaptive-3'),('jev_single','adaptive-1'),('jev_adaptive','adaptive-3')]:
                runtime=RGBDPickupRuntime(dict(objects=[],target_id='native'),dict(query_period_s=2.,scene_pair_cap=1160),request,estimate,method,Path(temp)/method)
                runtime.admit=lambda policy,*args: (setattr(runtime,'policy',policy) or dict(status='accepted',policy=policy))
                with patch('jev_cbf.control.rgbd.validate_selection',return_value=proposal):
                    runtime.install(supplied,np.zeros(7),np.zeros(7),0.,source='TEST ONLY')
                self.assertEqual(runtime.policy[other['id']],expected)
                self.assertEqual(runtime.policy[obj['id']],'adaptive-1')
                proposal[0]['role']='omit';proposal[0]['candidate_id']=None
                with patch('jev_cbf.control.rgbd.validate_selection',return_value=proposal):
                    runtime.install(supplied,np.zeros(7),np.zeros(7),0.,source='TEST ONLY')
                self.assertEqual(runtime.policy[other['id']], 'adaptive-3' if method=='all_jev' else None)
                proposal[0]['role']='avoid';proposal[0]['candidate_id']='adaptive-3'

    def test_native_rgbd_plant_truth_mutation_cannot_change_controller_geometry(self):
        from jev_cbf.experiments.rgbd import prepare, SETTINGS
        from jev_cbf.io import save, read
        from jev_cbf.paths import ROOT
        from jev_cbf.upstream import load_upstream
        from jev_cbf.control.simulation import run_case
        scene = deepcopy(read(ROOT/'configs/legacy/scenes/pickup_scenes.json')['scenes'][0])
        scene['id'] = 'easy'
        # Outputs intentionally live within the repository in this workflow.
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as temp:
            root = Path(temp)
            save(root/'config.json', dict(scenes=[scene]))
            folder = root/'prepared'
            prepare(folder, root/'config.json')
            request, estimate = read(folder/'easy/request.json'), read(folder/'easy/estimate.json')
            self.assertEqual(len(estimate['objects']), 3)
            initial = read(folder/'easy/initial/request.json')
            self.assertNotIn('observed_target', [o['id'] for o in initial['state']['objects']])
            self.assertNotIn('target_object', str(initial))
            runtime = RGBDPickupRuntime(scene, SETTINGS, request, estimate, 'off', root/'run')
            before, waypoint = runtime.parameters(0.), runtime.waypoint()
            runtime.physical_scene['objects'][0]['center_m'] = [9., 9., 9.]
            runtime.physical_scene['objects'][0]['size_m'] = [9., 9., 9.]
            np.testing.assert_array_equal(runtime.parameters(0.)[0], before[0])
            np.testing.assert_array_equal(runtime.waypoint(), waypoint)
            # Native integration uses the unmutated plant and an actual 1 ms
            # FRAX/CBFpy control step, not a fabricated policy/filter.
            runtime.physical_scene = scene
            demo, utils = load_upstream(ROOT.parent/'frax', ROOT.parent/'cbfpy')
            result, trace = run_case(demo, utils, {'request': request}, 'off', frax_dir=ROOT.parent/'frax',
                                     steps=2, control_dt=.001, runtime=runtime, scene_guard=.05)
            self.assertEqual(result['simulation_seconds'], .002)
            self.assertFalse(result['unwanted_collision'])
            self.assertTrue(np.isfinite(trace['torques_Nm']).all())
            self.assertLess(result['oscbf_nominal_first_step_max_difference_Nm'],1e-7)


if __name__ == '__main__':
    unittest.main()
