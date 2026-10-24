"""Tiny saved-frame inspection tests; no device, renderer or SDK is needed."""

from html.parser import HTMLParser
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from jev_cbf.sensing.camera_io import CameraFrame, FrameRecorder, Intrinsics
from jev_cbf.reporting.camera import inspect_recording


class InspectionTests(unittest.TestCase):
    def test_uncalibrated_unsegmented_frames_stay_preview_only(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            frame=CameraFrame('unknown',0,3.,'synthetic',np.zeros((4,4,3),dtype=np.uint8),np.ones((4,4)),
                              Intrinsics(4,4,4.,4.,1.5,1.5),'unknown/optical')
            recorder=FrameRecorder(folder/'recording');recorder.write(frame)
            rows=inspect_recording(folder/'recording',folder/'preview')
            self.assertIsNone(rows[0]['geometry']);self.assertIsNone(rows[0]['identity_source'])
            self.assertIn('disabled',rows[0]['error'])
            self.assertNotIn('pyrealsense2',__import__('sys').modules)
            with self.assertRaises(ValueError):inspect_recording(folder/'recording',folder/'preview')

    def test_calibrated_supplied_masks_feed_geometry_and_local_page_links_resolve(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            frame=CameraFrame('camera',0,0.,'synthetic',np.zeros((4,4,3),dtype=np.uint8),np.ones((4,4)),
                              Intrinsics(4,4,4.,4.,1.5,1.5),'camera/optical')
            recorder=FrameRecorder(folder/'recording')
            recorder.write(frame,{'object':np.ones((4,4),dtype=bool)},identity_source='SYNTHETIC test mask')
            calibration={'camera':dict(mount='fixed',calibration_id='SYNTHETIC',optical_frame=frame.optical_frame,
                                       T_robot_optical=np.eye(4).tolist())}
            rows=inspect_recording(folder/'recording',folder/'preview',calibration=calibration,support_z=0.)
            self.assertEqual(rows[0]['geometry']['status'],'ready')
            self.assertTrue(rows[0]['geometry']['estimated_objects'][0]['candidates'])
            saved=json.loads((folder/'preview/inspection.json').read_text())
            self.assertEqual(saved['calibration_used'],calibration)
            class Links(HTMLParser):
                def handle_starttag(inner,tag,attrs):
                    for key,value in attrs:
                        if key in ('href','src'):self.assertTrue((folder/'preview'/value).is_file(),value)
            Links().feed((folder/'preview/index.html').read_text())


if __name__=='__main__':unittest.main()
