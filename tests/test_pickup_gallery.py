"""Playback reports reflect enabled robot/object pairs and physical lift."""

import unittest
import numpy as np

from pickup_gallery import mode_payload


class PickupGalleryTests(unittest.TestCase):
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
