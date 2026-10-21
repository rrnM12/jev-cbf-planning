"""Geometry summaries measure unions and actual applied time weights."""

import unittest
from unittest.mock import patch
import importlib.metadata
import numpy as np

from analyze_pickup import footprint
from run_pickup import runtime_versions


class FootprintTests(unittest.TestCase):
    def test_independent_truth_outside_perceived_domain_is_not_ignored(self):
        request={'state':{'target_id':'target','objects':[
            {'id':'target','shape':'box','size_m':[.1]*3,'center_m':[0.,0.,0.],'candidates':[]},
            {'id':'a','shape':'box','size_m':[.02]*3,'center_m':[0.,0.,0.],
             'candidates':[{'centers_object_m':[[0.,0.,0.]],'radii_m':[.02]}]}]}}
        trace={'cover_radii_m':np.full((2,8),.02),'cover_centers_m':np.zeros((2,8,3)),
               'cover_enabled':np.zeros((2,29,8),dtype=bool)}
        truth=[{'id':'target','shape':'box','size_m':[.1]*3,'center_m':[0.,0.,0.]},
               {'id':'a','shape':'box','size_m':[.1]*3,'center_m':[1.,0.,0.]}]
        result=footprint(request,trace,truth_objects=truth,samples=300_000)
        self.assertAlmostEqual(result['mean_unmodeled_clutter_m3'],.001,delta=.00005)
        self.assertEqual(result['mean_uncovered_constrained_clutter_m3'],0.)  # Omitted is not automatically a bad cover.
        trace['cover_enabled'][:,:,4]=True
        represented=footprint(request,trace,truth_objects=truth,samples=300_000)
        self.assertAlmostEqual(represented['mean_uncovered_constrained_clutter_m3'],.001,delta=.00005)

    def test_optional_sdk_metadata_does_not_block_offline_preparation(self):
        def version(name):
            if name == "typesafe-sdk":
                raise importlib.metadata.PackageNotFoundError(name)
            return "FABRICATED VERSION"
        with patch("run_pickup.importlib.metadata.version",side_effect=version):
            self.assertIsNone(runtime_versions()["typesafe-sdk"])

    def fixture(self):
        obj={"shape":"box","size_m":[.1,.1,.1],"center_m":[0,0,0],
             "candidates":[{"centers_object_m":[[0,0,0]],"radii_m":[.1]}]}
        request={"state":{"target_id":"target","objects":[dict(obj,id="target"),dict(obj,id="a"),dict(obj,id="b")]}}
        enabled=np.zeros((3,29,12),dtype=bool);enabled[:,:,4]=True;enabled[:,:,8]=True
        radii=np.full((3,12),.01);radii[:,[4,8]]=.1
        trace={"cover_enabled":enabled,"cover_centers_m":np.zeros((3,12,3)),"cover_radii_m":radii}
        return request,trace

    def test_overlapping_object_covers_count_once_and_target_is_excluded(self):
        request,trace=self.fixture()
        both=footprint(request,trace,samples=100_000,seed=3)
        trace["cover_enabled"][:,:,8]=False
        one=footprint(request,trace,samples=100_000,seed=3)
        self.assertEqual(both["mean_excess_m3"],one["mean_excess_m3"])
        self.assertAlmostEqual(one["mean_excess_m3"],4*np.pi*.1**3/3-.1**3,delta=.00008)
        self.assertEqual(one["mean_unmodeled_clutter_m3"],0.)
        trace["cover_enabled"][:]=False
        off=footprint(request,trace,samples=100_000,seed=3)
        self.assertEqual(off["mean_excess_m3"],0.)
        self.assertAlmostEqual(off["mean_unmodeled_clutter_m3"],.001,delta=.00005)

    def test_time_average_uses_control_intervals_not_final_display_frame(self):
        request,trace=self.fixture()
        full=footprint(request,trace,samples=100_000,seed=3)
        trace["cover_enabled"][1:]=False
        half=footprint(request,trace,samples=100_000,seed=3)
        self.assertEqual(half["mean_excess_m3"],full["mean_excess_m3"]/2)
        self.assertEqual(half["unique_applied_geometry_sets"],2)
