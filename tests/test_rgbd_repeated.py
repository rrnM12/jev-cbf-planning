"""Offline workflow contracts; fake answers here are never model evidence."""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from inspect_selection import ROOT
from rgbd_pickup import METHODS
import rgbd_repeated as repeated
from run_rgbd_pickup import verify_inputs
from run_suite import digest, read, save


class RepeatedStudyTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'artifacts').mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT/'artifacts')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.reference, self.output = self.root/'startup', self.root/'repeated'
        case = self.reference/'easy'
        for name in ('scenes.json', 'settings.json', 'questions.json',
                     'easy/scene.json', 'easy/estimate.json', 'easy/request.json',
                     'easy/observation/calibration.json', 'easy/qualification/adaptive/result.json'):
            save(self.reference/name, {})
        save(case/'qualification.json', dict(qualified=True))
        save(case/'initial/request.json', dict(TEST_ONLY=True))
        save(case/'initial/selection.json', dict(request=dict(TEST_ONLY=True)))
        for method in METHODS:
            path = case/method
            path.mkdir()
            (path/'trajectory.npz').write_bytes(b'TEST FIXTURE, NOT A PHYSICAL TRAJECTORY')
            save(path/'result.json', dict(trajectory_sha256=digest(path/'trajectory.npz'),
                unwanted_collision=False, pickup_success=True, safe_pickup=True,
                simulation_seconds=2., cbf_counts=dict(clutter_pairs_mean=145.), warm_loop={}))
        frozen = {str(p.relative_to(self.reference)): digest(p) for p in self.reference.rglob('*.json')}
        save(self.reference/'manifest.json', dict(scene_ids=['easy'], methods=METHODS, frozen_files=frozen,
                                               source_sha256={}, query_schedule='startup_only'))

    def prepare(self):
        with patch.object(repeated, 'usage', return_value=dict(TEST_ONLY=True)):
            repeated.prepare(self.output, self.reference)

    def test_exact_inputs_answers_baselines_and_reference_remain_bound(self):
        self.prepare()
        manifest = verify_inputs(self.output)
        self.assertEqual(manifest['query_schedule'], 'completion_driven_repeated')
        for relative in ('easy/estimate.json', 'easy/initial/selection.json', 'easy/off/trajectory.npz'):
            self.assertEqual(digest(self.reference/relative), digest(self.output/relative))
        self.assertFalse((self.output/'easy/jev_adaptive/result.json').exists())
        repeated.check_reference(self.output)
        save(self.output/'easy/initial/selection.json', dict(changed=True))
        with self.assertRaisesRegex(ValueError, 'frozen input changed'):
            repeated.check_reference(self.output)

    def test_reference_result_change_is_rejected_and_output_cannot_be_overwritten(self):
        self.prepare()
        with self.assertRaisesRegex(ValueError, 'fresh directory'):
            self.prepare()
        save(self.reference/'easy/jev_adaptive/result.json', dict(changed=True))
        with self.assertRaisesRegex(ValueError, 'startup reference changed'):
            repeated.check_reference(self.output)

    def test_every_jev_ablation_runs_repeated_and_completed_runs_make_no_calls(self):
        self.prepare()
        def execute(case, method, **kwargs):
            self.assertTrue(kwargs['repeated'])
            self.assertEqual(kwargs['initial'], read(case/'initial/selection.json'))
            save(case/method/'result.json', dict(TEST_ONLY=True))
        with patch.object(repeated, 'execute', side_effect=execute) as run, \
             patch.object(repeated, 'usage', return_value=dict(TEST_ONLY=True)), \
             patch.object(repeated, 'report', return_value={}):
            with self.assertRaisesRegex(ValueError, '--live'):
                repeated.run(self.output)
            run.assert_not_called()
            repeated.run(self.output, live=True)
            self.assertEqual([call.args[1] for call in run.call_args_list], list(repeated.JEV_METHODS))
            run.reset_mock()
            repeated.run(self.output, live=False)
            run.assert_not_called()

    def test_report_checks_actual_request_overlap_and_retains_late_responses(self):
        self.prepare()
        for method in repeated.JEV_METHODS:
            path = self.output/'easy'/method
            result = read(self.reference/'easy'/method/'result.json')
            events = [dict(kind='selection', source='initial/selection.json', status='accepted', previous_policy={}, policy={'o':'a'}),
                      dict(kind='selection', source='queries/001/selection.json', status='accepted', previous_policy={'o':'a'}, policy={'o':'a'}),
                      dict(kind='late_response', source='queries/002/selection.json', status='not applied after rollout')]
            timings = [dict(request_start_monotonic_s=start, completion_monotonic_s=start+.3, snapshot_at_s=i)
                       for i,start in enumerate((10., 10.6))]
            result.update(selection_events=events, model_request_timings=timings)
            save(path/'result.json', result); save(path/'events.json', events)
            for index in (1, 2):
                answer = dict(response=dict(usage=dict(input_tokens=100)))
                if index == 2:
                    answer['error'] = 'TEST ONLY invalid late choice; not a model result'
                save(path/'queries'/f'{index:03d}'/'selection.json', answer)
        report = repeated.report(self.output)
        self.assertEqual(report['new_logical_requests'], 6)
        self.assertTrue(all(row['late_responses']==1 and row['changes_after_startup']==0 for row in report['rows']))
        self.assertTrue(all(len(row['invalid_responses'])==1 for row in report['rows']))
        path = self.output/'easy/all_jev/result.json'
        result = read(path)
        result['model_request_timings'][0]['completion_monotonic_s'] = 10.8
        save(path, result)
        with self.assertRaisesRegex(ValueError, 'one-in-flight'):
            repeated.report(self.output)


if __name__ == '__main__':
    unittest.main()
