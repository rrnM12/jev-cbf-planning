"""No hardware/SDK required: optical frames, records and delayed fake sources."""

from pathlib import Path
import tempfile
from threading import Event, get_ident
import time
import unittest

import numpy as np

from jev_cbf.sensing.camera_io import CameraFrame, CameraWorker, FrameRecorder, Intrinsics, ReplaySource, deproject, masked_clouds, rigid_transform, robot_transform


def frame(**kwargs):
    values=dict(camera_id='scene', frame_number=1, capture_time_s=2., clock_domain='synthetic',
                rgb=np.zeros((2, 3, 3), dtype=np.uint8), depth_m=np.full((2, 3), 2.),
                intrinsics=Intrinsics(3, 2, 2., 4., 1., .5), optical_frame='scene/optical')
    return CameraFrame(**dict(values, **kwargs))


class CameraTests(unittest.TestCase):
    def test_optical_projection_uses_intrinsics_and_integer_pixel_centers(self):
        f=frame();points=deproject(f)
        np.testing.assert_allclose(points[0, 0], [-1., -.25, 2.])
        np.testing.assert_allclose(points[1, 2], [1., .25, 2.])
        with self.assertRaises(ValueError):f.depth_m[0, 0]=0
        depth=np.array([[0., np.nan, -1.], [2., 3., 4.]])
        self.assertTrue(np.isnan(deproject(frame(depth_m=depth))[0]).all())

    def test_distortion_requires_actual_deprojection_not_a_pinhole_guess(self):
        intr=Intrinsics(3, 2, 2., 4., 1., .5, 'brown_conrady', (.1, 0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, 'SDK-derived'):deproject(frame(intrinsics=intr))
        points=deproject(frame())
        np.testing.assert_array_equal(deproject(frame(intrinsics=intr, points_optical_m=points)), points)
        with self.assertRaises(ValueError):frame(points_optical_m=np.zeros((2, 3, 3)))

    def test_fixed_and_capture_time_wrist_transform_require_provenance(self):
        f=frame();matrix=np.eye(4);matrix[:3, 3]=[1., 2., 3.]
        calibration={'scene':dict(mount='fixed', optical_frame=f.optical_frame, calibration_id='SYNTHETIC', T_robot_optical=matrix)}
        np.testing.assert_array_equal(robot_transform(f, calibration), matrix)
        cloud=masked_clouds(f, {'object':np.ones((2, 3), dtype=bool)}, matrix)['object']
        np.testing.assert_allclose(cloud[0], [0., 1.75, 5.])
        with self.assertRaises(ValueError):robot_transform(f, {})
        calibration['scene']['device_serial']='OTHER DEVICE'
        with self.assertRaisesRegex(ValueError,'serial'):robot_transform(f, calibration)
        calibration['scene'].pop('device_serial')
        calibration['scene']=dict(mount='wrist', optical_frame=f.optical_frame, calibration_id='SYNTHETIC', T_ee_optical=matrix)
        pose=lambda _:dict(T_robot_ee=matrix, capture_time_s=2., clock_domain='synthetic')
        np.testing.assert_array_equal(robot_transform(f, calibration, pose_provider=pose), matrix@matrix)
        for clock, at in [('unknown_robot_clock', 2.), ('synthetic', 2.01)]:
            with self.assertRaises(ValueError):
                robot_transform(f, calibration, pose_provider=lambda _:dict(T_robot_ee=matrix, capture_time_s=at, clock_domain=clock))
        with self.assertRaises(ValueError):rigid_transform(np.diag([1., 1., -1., 1.]))
        with self.assertRaises(ValueError):masked_clouds(f, {'bad':np.ones((2, 3), dtype=int)}, matrix)

    def test_record_replay_preserves_calibration_masks_and_detects_tampering(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'recording';recorder=FrameRecorder(folder)
            f=frame(points_optical_m=deproject(frame()));mask=np.ones((2, 3), dtype=bool)
            recorder.write(f, {'box':mask}, identity_source='synthetic mask, NOT detector')
            replay=ReplaySource(folder);copy=replay.read(.1)
            np.testing.assert_array_equal(copy.rgb, f.rgb)
            np.testing.assert_array_equal(copy.depth_m, f.depth_m)
            np.testing.assert_array_equal(deproject(copy), deproject(f))
            self.assertEqual(copy.capture_time_s, f.capture_time_s)
            self.assertEqual(copy.clock_domain, f.clock_domain)
            self.assertTrue(copy.metadata['replay'])
            self.assertEqual(copy.metadata['recorded_receipt_monotonic_s'], f.received_monotonic_s)
            masks, source=replay.annotations(copy)
            np.testing.assert_array_equal(masks['box'], mask);self.assertIn('synthetic', source)
            with self.assertRaises(StopIteration):replay.read(.1)
            with self.assertRaises(ValueError):FrameRecorder(folder)
            (folder/'000000.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'hash'):ReplaySource(folder).read(.1)

    def test_raw_recordings_do_not_fabricate_masks(self):
        with tempfile.TemporaryDirectory() as temp:
            rec=FrameRecorder(temp);rec.write(frame())
            replay=ReplaySource(temp);f=replay.read(.1)
            with self.assertRaisesRegex(ValueError, 'no recorded masks'):replay.annotations(f)

    def test_configured_recording_limit_does_not_overcapture_or_publish_after_close(self):
        done=Event();counts=[]
        class Source:
            n=0
            def read(self, timeout_s):
                self.n+=1;counts.append(self.n);return frame(frame_number=self.n)
            def close(self):done.set()
        worker=CameraWorker(Source,frame_limit=2).start()
        self.assertTrue(done.wait(2.));self.assertEqual(counts,[1,2])
        self.assertEqual(worker.latest().sequence,2);self.assertEqual(worker.health()['status'],'eof')
        worker.close()

    def test_background_source_processing_and_shutdown_never_wait_in_latest(self):
        entered, release, done=Event(), Event(), Event();owners=[]
        class Source:
            def __init__(self):owners.append(get_ident());self.count=0
            def read(self, timeout_s):
                entered.set();release.wait(2.)
                if self.count:raise OSError('SYNTHETIC disconnect')
                self.count+=1;return frame()
            def close(self):owners.append(get_ident());done.set()
        def process(f, source):owners.append(get_ident());return {'frame_number':f.frame_number}
        worker=CameraWorker(Source, process).start()
        try:
            self.assertTrue(entered.wait(2.))
            start=time.monotonic()
            for _ in range(1000):self.assertIsNone(worker.latest())
            self.assertLess(time.monotonic()-start, .2)
            self.assertFalse(worker.close(.001))  # Honest unresponsive-source status.
            release.set();self.assertTrue(done.wait(2.));self.assertTrue(worker.close(2.))
            self.assertTrue(all(owner!=get_ident() for owner in owners))
        finally:release.set();worker.close(2.)

    def test_eof_and_disconnect_retain_last_snapshot_with_explicit_health(self):
        for error in (StopIteration(), OSError('SYNTHETIC disconnect')):
            done=Event()
            class Source:
                n=0
                def read(self, timeout_s):
                    self.n+=1
                    if self.n>3:raise error
                    return frame(frame_number=self.n)
                def close(self):done.set()
            worker=CameraWorker(Source).start()
            self.assertTrue(done.wait(2.))
            self.assertEqual(worker.latest().value.frame_number, 3)
            self.assertEqual(worker.health()['published'], 3)
            self.assertEqual(worker.health()['status'], 'eof' if isinstance(error, StopIteration) else 'error')
            self.assertGreaterEqual(worker.health()['snapshot_age_host_s'], 0.)
            worker.close(2.)


if __name__=='__main__':unittest.main()
