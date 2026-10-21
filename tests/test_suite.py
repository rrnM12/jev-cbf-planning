"""Offline freeze/one-shot guards with explicitly fabricated model responses."""

from contextlib import redirect_stdout
from copy import deepcopy
import io
from pathlib import Path
import tempfile
import unittest

from run_suite import check_manifest, digest, one_shot, read, save
from test_view_selection import fixture


class SuiteTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)
        report = fixture()
        self.request, self.response = report["request"], report["response"]
        save(self.folder / "case/request.json", self.request)
        self.manifest = {"ready_for_live": True, "sdk_invocation_budget": 1,
                         "cases": [{"id": "case"}], "frozen_files": {"case/request.json": digest(self.folder / "case/request.json")}}
        save(self.folder / "manifest.json", self.manifest)

    def test_record_before_call_and_never_repeat_a_scene(self):
        calls = []
        def fabricated(request):
            self.assertEqual(read(self.folder / "attempts.json")["case"]["status"], "started")
            calls.append(request)
            return self.response
        with redirect_stdout(io.StringIO()):
            one_shot(self.folder, "case", self.request, ask=fabricated)
            one_shot(self.folder, "case", self.request, ask=fabricated)
        self.assertEqual(len(calls), 1)
        result = read(self.folder / "case/selection.json")
        self.assertEqual(result["response"]["model"], "offline-fabricated-viewer-fixture")
        self.assertEqual(read(self.folder / "attempts.json")["case"]["status"], "success")

    def test_failed_request_is_preserved_without_fallback_or_retry(self):
        def fabricated_failure(_request):
            raise ValueError("fabricated SDK failure")
        with redirect_stdout(io.StringIO()):
            one_shot(self.folder, "case", self.request, ask=fabricated_failure)
            one_shot(self.folder, "case", self.request, ask=lambda _: self.fail("retried"))
        result = read(self.folder / "case/selection.json")
        self.assertIn("error", result)
        self.assertNotIn("proposals", result)

    def test_frozen_inputs_and_changed_argument_rejected(self):
        changed = deepcopy(self.request)
        changed["state"]["task"] = "different"
        with self.assertRaises(ValueError):
            one_shot(self.folder, "case", changed, ask=lambda _: self.fail("network"))
        save(self.folder / "case/request.json", changed)
        with self.assertRaises(ValueError):
            check_manifest(self.folder)

    def test_failed_qualification_prevents_any_attempt(self):
        self.manifest["ready_for_live"] = False
        save(self.folder / "manifest.json", self.manifest)
        with self.assertRaises(ValueError):
            one_shot(self.folder, "case", self.request, ask=lambda _: self.fail("network"))
        self.assertFalse((self.folder / "attempts.json").exists())


if __name__ == "__main__":
    unittest.main()
