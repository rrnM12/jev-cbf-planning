"""Playback reports reflect enabled robot/object pairs and physical lift."""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from jev_cbf.legacy.pickup_gallery import mode_payload, verify_review
from jev_cbf.io import save
from jev_cbf.reporting.images import write_apng


class PickupGalleryTests(unittest.TestCase):
    def test_review_check_detects_broken_links_without_claiming_visual_qa(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp);base=root/'artifacts/pickup';page=base/'review/index.html'
            page.parent.mkdir(parents=True)
            page.write_text('<a href="missing.json">missing</a>')
            with patch('jev_cbf.legacy.pickup_gallery.ROOT',root),self.assertRaises(ValueError):verify_review(base)
            from jev_cbf.io import read
            result=read(base/'review_checks.json')
            self.assertFalse(result['all_checks_pass'])
            self.assertEqual(len(result['broken_links']),1)
            self.assertIn('Not full-browser',result['scope'])

    def test_review_check_checks_real_atlas_bounds_before_mocked_js(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp);base=root/'artifacts/pickup';folder=base/'review';folder.mkdir(parents=True)
            payload={'off':{'times':[0.,1.],'image_index':[0,1]}}
            import json
            (folder/'index.html').write_text('<input id="scrub" max="1"><script id="data" type="application/json">'+json.dumps(payload)+'</script><script>// deliberately mocked JS</script>')
            write_apng(folder/'off_atlas.png',[np.zeros((360,3840,3),dtype=np.uint8)],frame_count=1,fps=1)
            save(folder/'off_atlas.json',{'frames':2})
            with patch('jev_cbf.legacy.pickup_gallery.ROOT',root),patch('subprocess.run',return_value=SimpleNamespace(stdout='{"modes":1,"controls":true,"draws":2}')):
                checked=verify_review(base)
            self.assertTrue(checked['all_checks_pass'])
            save(folder/'off_atlas.json',{'frames':1})
            with patch('jev_cbf.legacy.pickup_gallery.ROOT',root),self.assertRaises(AssertionError):verify_review(base)

    def test_pairwise_masks_and_lengths_are_not_confused_with_primitive_count(self):
        masks = np.zeros((3, 29, 12), dtype=bool)
        masks[:, :16, :4] = True  # Mandatory arm-target guards: 64 pairs.
        masks[:, :, 4:6] = True  # Clutter: two primitives, 58 pairs.
        trace = {
            "cover_enabled": masks, "time_s": np.array([0., .5, 1.]),
            "phase": np.array(["approach", "close", "lift"]),
            "target_bottom_m": np.array([0., 0., .06]),
            "bilateral_finger_contact": np.array([False, True, True]),
            "enabled_scene_pairs": masks.sum(axis=(1, 2)),
            "object_distance_m": np.array([[-.001, .12, .3]]*3),
            "scene_h2_m": np.array([.1, .05, .01]),
        }
        result = {"simulation_seconds": 1., "grasp_verified": True}
        objects = [{"id": name} for name in ("target", "a", "b")]
        result = mode_payload(trace, result, np.array([0, 2]), "target", objects)
        self.assertEqual(result["enabled"], [[4, 2, 0], [4, 2, 0]])
        self.assertEqual(result["pairs"], [122, 122])
        self.assertEqual(result["bottom_mm"], [0., 60.])
        self.assertEqual(result["clutter_mm"], [120., 120.])
        self.assertEqual(result["scene_barrier_mm"], [100., 10.])
        self.assertEqual(result["times"], [0., 1.])
        trace["cover_enabled"]=np.pad(masks,((0,0),(0,0),(0,12)))
        expanded=mode_payload(trace,{"simulation_seconds":1.,"grasp_verified":True},np.array([0,2]),"target",objects)
        self.assertEqual(expanded["enabled"],[[6,0,0],[6,0,0]])
