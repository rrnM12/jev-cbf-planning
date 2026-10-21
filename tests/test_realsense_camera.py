"""Strict SDK doubles validate call/format logic, NOT physical RealSense support."""

from dataclasses import asdict
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from camera_io import CameraFrame, Intrinsics, deproject
from realsense_camera import RealSenseSource


def fake_sdk(serials=('SYNTHETIC-1',), *, scale=.002):
    state=SimpleNamespace(stops=0, calls=[], aligned=0, scale=scale, available=True, missing=False)
    def intr(fx):return SimpleNamespace(width=3, height=2, fx=fx, fy=4., ppx=1., ppy=.5,
                                       model='distortion.none', coeffs=[0.]*5)
    class Video:
        def __init__(self, data, fx):
            self.data=data;self.intr=intr(fx);self.profile=self
        def as_video_stream_profile(self):return self
        def get_intrinsics(self):return self.intr
        def get_extrinsics_to(self, other):
            rotation=np.eye(3) if self is state.aligned_depth else np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
            return SimpleNamespace(rotation=rotation.ravel(order='F').tolist(),translation=[0.,0.,0.] if self is state.aligned_depth else [.02,0.,0.])
        def get_data(self):return self.data
        def get_timestamp(self):return 1000.
        def get_frame_timestamp_domain(self):return 'timestamp_domain.hardware_clock'
        def get_frame_number(self):return 42
    state.depth=Video(np.full((2,3),1000,dtype=np.uint16),4.)
    state.color=Video(np.zeros((2,3,3),dtype=np.uint8),2.)
    state.aligned_depth=Video(state.depth.data,2.)
    class Frames:
        def __init__(self, depth):self.depth=depth
        def get_depth_frame(self):return None if state.missing else self.depth
        def get_color_frame(self):return state.color
    device=SimpleNamespace(first_depth_sensor=lambda:SimpleNamespace(get_depth_scale=lambda:state.scale))
    class Config:
        def enable_device(self, serial):state.calls.append(('device',serial))
        def disable_all_streams(self):state.calls.append(('disable_all',))
        def enable_stream(self, kind, width, height, fmt, fps):state.calls.append(('stream',kind,width,height,fmt,fps))
    class Pipeline:
        def __init__(self, ctx):pass
        def start(self, config):return SimpleNamespace(get_device=lambda:device)
        def try_wait_for_frames(self, milliseconds):
            state.calls.append(('wait_ms', milliseconds))
            if state.available=='error':raise OSError('SYNTHETIC unplug')
            return (True,Frames(state.depth)) if state.available else (False,None)
        def stop(self):state.stops+=1
    class Align:
        def __init__(self, stream):assert stream=='color'
        def process(self, frames):state.aligned+=1;return Frames(state.aligned_depth)
    class PointCloud:
        def calculate(self, depth):
            i=depth.intr
            f=CameraFrame('synthetic',0,0.,'synthetic',state.color.data,depth.data.astype(float)*state.scale,
                          Intrinsics(i.width,i.height,i.fx,i.fy,i.ppx,i.ppy),'synthetic/optical')
            points=deproject(f).astype(np.float32)
            vertices=points.reshape(-1,3).copy().view(dtype=[('x','f4'),('y','f4'),('z','f4')]).reshape(-1)
            return SimpleNamespace(get_vertices=lambda:vertices)
    sdk=SimpleNamespace(context=lambda:SimpleNamespace(query_devices=lambda:[SimpleNamespace(get_info=lambda key,s=s:s) for s in serials]),
                        camera_info=SimpleNamespace(serial_number='serial_number'), config=Config, pipeline=Pipeline,
                        stream=SimpleNamespace(depth='depth',color='color'), format=SimpleNamespace(z16='z16',rgb8='rgb8'),
                        align=Align, pointcloud=PointCloud)
    return sdk,state


class RealSenseTests(unittest.TestCase):
    def test_metric_depth_aligned_intrinsics_clock_and_owned_arrays(self):
        sdk,state=fake_sdk();camera=RealSenseSource({'camera_id':'wrist'},sdk=sdk)
        f=camera.read(.125)
        self.assertEqual(f.camera_id,'wrist');self.assertEqual(f.optical_frame,'wrist/color_optical')
        self.assertEqual(f.capture_time_s,1.)
        self.assertIn('hardware_clock',f.clock_domain)
        self.assertEqual(f.intrinsics.fx,2.)  # AFTER alignment, not native depth's 4.
        np.testing.assert_array_equal(f.depth_m,2.)
        np.testing.assert_allclose(deproject(f)[0,0],[-1.,-.25,2.])
        self.assertIn(('wait_ms',125),state.calls)
        self.assertIn(('stream','color',0,0,'rgb8',0),state.calls)
        state.depth.data[:]=0;state.color.data[:]=255
        np.testing.assert_array_equal(f.depth_m,2.);np.testing.assert_array_equal(f.rgb,0)
        camera.close();camera.close();self.assertEqual(state.stops,1)
        with self.assertRaises(RuntimeError):camera.read(.1)

    def test_unaligned_depth_preserves_its_own_profile(self):
        sdk,state=fake_sdk();camera=RealSenseSource({'align_depth_to_color':False,
            'depth':{'width':3,'height':2,'fps':15}},sdk=sdk)
        f=camera.read(.1)
        self.assertEqual(f.intrinsics.fx,4.);self.assertEqual(state.aligned,0)
        self.assertTrue(f.optical_frame.endswith('/depth_optical'))
        self.assertEqual(f.metadata['color_intrinsics']['fx'],2.)
        np.testing.assert_allclose(np.array(f.metadata['T_color_depth'])[:3,:3],[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        self.assertEqual(f.metadata['T_color_depth'][0][3],.02)
        camera.close()

    def test_selection_timeout_missing_frames_and_disconnect_are_not_fake_data(self):
        for serials in ((),('SYNTHETIC-1','SYNTHETIC-2')):
            sdk,_=fake_sdk(serials)
            with self.assertRaisesRegex(ValueError,'serial'):RealSenseSource({},sdk=sdk)
        sdk,state=fake_sdk(('SYNTHETIC-1','SYNTHETIC-2'))
        camera=RealSenseSource({'serial':'SYNTHETIC-2'},sdk=sdk)
        state.available=False;self.assertIsNone(camera.read(.01))
        state.available=True;state.missing=True;self.assertIsNone(camera.read(.01))
        state.available='error'
        with self.assertRaises(OSError):camera.read(.01)
        camera.close();self.assertEqual(state.stops,1)

    def test_startup_scale_and_profile_errors_cleanup_or_fail_before_capture(self):
        sdk,state=fake_sdk(scale=0.)
        with self.assertRaises(ValueError):RealSenseSource({},sdk=sdk)
        self.assertEqual(state.stops,1)
        sdk,state=fake_sdk()
        with self.assertRaises(ValueError):RealSenseSource({'depth':{'width':640}},sdk=sdk)
        self.assertEqual(state.stops,0)
        with patch('realsense_camera.importlib.import_module',side_effect=ImportError):
            with self.assertRaisesRegex(RuntimeError,'optional'):RealSenseSource({})


if __name__=='__main__':unittest.main()
