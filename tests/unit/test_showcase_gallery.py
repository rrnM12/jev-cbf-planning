"""Outcome reports must derive claims from data, not desired results."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from jev_cbf.io import save, digest
from jev_cbf.legacy.showcase_gallery import comparison_data, observed_results, selection_changes


class ShowcaseReportTests(unittest.TestCase):
    def test_late_add_remove_geometry_changes_exclude_initial_fallback(self):
        def event(at, previous, current):
            return {"kind": "selection", "status": "accepted", "applied_at_s": at,
                    "previous_policy": previous, "policy": current}
        events = [event(0, {"a": "four"}, {"a": None}),
                  event(2, {"a": None}, {"a": "four"}),
                  event(4, {"a": "four"}, {"a": "two"}),
                  event(6, {"a": "two"}, {"a": None})]
        self.assertEqual([x["kind"] for x in selection_changes(events)], ["added", "geometry changed", "removed"])

    def test_failed_or_missing_rollouts_never_report_success(self):
        self.assertIn("No repeated", observed_results({"test": {"runs": []}}))
        run = {"mode": "repeated", "task_success": False,
               "collision": {"any_sampled_scene_violation": True}, "selection_events": []}
        description = observed_results({"test": {"runs": [run]}})
        self.assertIn("0/1 pass", description)
        self.assertIn("0/1 have no sampled", description)
        self.assertIn("dropping is not demonstrated", description)

    def test_explicit_collision_audit_is_hash_bound_and_does_not_overwrite_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            original = {"runs": [{"mode": "all", "complete": True, "within_2cm_at_end": True,
                                  "any_sampled_negative_h2": False, "task_success": False,
                                  "collision": {"any_sampled_scene_violation": True}}]}
            save(folder/"comparison.json", original)
            save(folder/"all.npz", {"not": "a real trajectory; hash-only test"})
            save(folder/"collision_recheck.json", {"all": {"source_trajectory_sha256": digest(folder/"all.npz"),
                                                          "collision": {"any_sampled_scene_violation": False}}})
            report = comparison_data(folder)
            self.assertTrue(report["runs"][0]["task_success"])
            from jev_cbf.io import read
            self.assertEqual(read(folder/"comparison.json"), original)
            save(folder/"all.npz", {"changed": True})
            with self.assertRaises(ValueError):
                comparison_data(folder)
