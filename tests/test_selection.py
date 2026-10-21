"""Offline request/contract checks. Fabricated responses are NOT Jev evidence."""

from copy import deepcopy
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from geometry import make_sphere_covers
import inspect_selection as selection


def fixture():
    scene = json.loads((selection.ROOT / "examples/selection_scene.json").read_text())
    template = json.loads((selection.ROOT / "prompts/selection.json").read_text())
    return scene, template


def fake_response(request):
    """Deliberately hand-written test labels; no model has produced these."""
    answers = {}
    for question_id, question in request["questions"].items():
        choice = "avoid" if question_id.startswith("role/") else "box-split-1"
        answers[question_id] = {
            "type": "choice", "choice": choice, "confidence": 1.0,
            "probabilities": {key: float(key == choice) for key in question["criteria"]},
        }
    return {
        "model": "jev-offline-test-fixture-not-a-live-model",
        "answers": answers,
        "usage": {"input_tokens": 100, "output_tokens": 20},
    }


class RequestTests(unittest.TestCase):
    def test_request_reuses_covers_and_preserves_inputs(self):
        scene, template = fixture()
        before = deepcopy((scene, template))
        request = selection.prepare_request(scene, template, samples=1000)
        self.assertEqual((scene, template), before)
        self.assertEqual(request["model"], "jev-latest")
        self.assertEqual(len(request["questions"]), 6)
        for obj, original in zip(request["state"]["objects"], scene["objects"]):
            covers = make_sphere_covers(np.array(original["size_m"]) / 2)
            for candidate, cover in zip(obj["candidates"], covers):
                self.assertEqual(candidate["id"], cover.candidate_id)
                np.testing.assert_array_equal(candidate["centers_object_m"], cover.centers_m)
                np.testing.assert_array_equal(candidate["radii_m"], cover.radii_m)
            role = request["questions"][f"role/{obj['id']}"]
            geometry = request["questions"][f"geometry/{obj['id']}"]
            # TypeSafe doesn't show question IDs to the model: name the object
            # in instructions, and phrase geometry independently/conditionally.
            self.assertIn(obj["id"], role["instructions"])
            self.assertIn("Assuming", geometry["instructions"])
            self.assertEqual(set(geometry["criteria"]), {c.candidate_id for c in covers})

    def test_custom_task_template_and_counts_change_request(self):
        scene, template = fixture()
        scene["task"] = "Inspect this different instruction without authorizing contact."
        template["role"]["instructions"] = "New wording for {object_id}."
        request = selection.prepare_request(scene, template, counts=(1, 3), samples=1000, model="explicit-model")
        self.assertEqual(request["state"]["task"], scene["task"])
        self.assertEqual(request["model"], "explicit-model")
        self.assertEqual(request["questions"]["role/bottle"]["instructions"], "New wording for bottle.")
        self.assertEqual(set(request["questions"]["geometry/bottle"]["criteria"]), {"box-split-1", "box-split-3"})

    def test_fixture_does_not_inject_oracle_decisions(self):
        scene, template = fixture()
        scene["expected_answers"] = {"bottle": "avoid"}
        scene["objects"][0]["expected_role"] = "avoid"
        request = selection.prepare_request(scene, template, samples=1000)
        serialized = json.dumps(request)
        self.assertNotIn("expected_answers", serialized)
        self.assertNotIn("expected_role", serialized)

    def test_invalid_scene_inputs_rejected(self):
        scene, template = fixture()
        variants = []
        for task in ("", None):
            invalid = deepcopy(scene)
            invalid["task"] = task
            variants.append(invalid)
        invalid = deepcopy(scene)
        invalid["objects"][1]["id"] = invalid["objects"][0]["id"]
        variants.append(invalid)
        invalid = deepcopy(scene)
        invalid["objects"][0]["rotation"] = [0, 0, 0]
        variants.append(invalid)
        for radius in (0, -1, True, float("nan")):
            invalid = deepcopy(scene)
            invalid["robot_proxy"]["radius_m"] = radius
            variants.append(invalid)
        for invalid in variants:
            with self.assertRaises(ValueError):
                selection.prepare_request(invalid, template, samples=100)

    def test_swept_sphere_clearance_handles_interior_endpoints_and_stationary_proxy(self):
        cover = make_sphere_covers((0.1, 0.1, 0.1), counts=(1,))[0]
        radius = cover.radii_m[0]
        start, goal = np.array([-1., 0, 0]), np.array([1., 0, 0])
        for center, distance in (([0, 1, 0], 1), ([2, 0, 0], 1), ([-2, 0, 0], 1)):
            self.assertAlmostEqual(
                selection.nominal_clearance(cover, center, start, goal, 0.02), distance - radius - 0.02,
            )
        self.assertAlmostEqual(
            selection.nominal_clearance(cover, [0, 0, 0], start, start, 0.02), 1 - radius - 0.02,
        )


class ResponseTests(unittest.TestCase):
    def setUp(self):
        self.request = selection.prepare_request(*fixture(), samples=1000)

    def test_omit_and_review_do_not_produce_selected_geometry(self):
        response = fake_response(self.request)
        for object_id, role in (("cube", "review"), ("far_cube", "omit")):
            answer = response["answers"][f"role/{object_id}"]
            answer["choice"] = role
            answer["probabilities"] = {key: float(key == role) for key in answer["probabilities"]}
        proposals = selection.decode_response(self.request, response)
        self.assertEqual(proposals[0]["candidate_id"], "box-split-1")
        for proposal in proposals[1:]:
            self.assertIsNone(proposal["candidate_id"])
            self.assertEqual(proposal["conditional_candidate_id"], "box-split-1")

    def test_bad_ids_missing_answers_and_invalid_probabilities_rejected(self):
        response = fake_response(self.request)
        variants = []
        missing = deepcopy(response)
        del missing["answers"]["role/bottle"]
        variants.append(missing)
        unknown = deepcopy(response)
        unknown["answers"]["geometry/bottle"]["choice"] = "invented-geometry"
        variants.append(unknown)
        for value in (float("nan"), -0.1, 1.1, True):
            invalid = deepcopy(response)
            invalid["answers"]["role/bottle"]["confidence"] = value
            variants.append(invalid)
        inconsistent = deepcopy(response)
        inconsistent["answers"]["role/bottle"]["choice"] = "omit"
        variants.append(inconsistent)
        unnormalized = deepcopy(response)
        unnormalized["answers"]["role/bottle"]["probabilities"]["review"] = 0.5
        variants.append(unnormalized)
        for invalid in variants:
            with self.assertRaises(ValueError):
                selection.decode_response(self.request, invalid)

    def test_provider_rounded_probability_mass_is_not_a_model_failure(self):
        for values in ((.33,.33,.33),(.34,.34,.33)):
            response = fake_response(self.request)
            answer = response["answers"]["geometry/bottle"]
            answer["probabilities"] = dict(zip(answer["probabilities"],values,strict=True))
            before = deepcopy(response)
            proposals = selection.decode_response(self.request,response)
            self.assertEqual(proposals[0]["candidate_id"],"box-split-1")
            self.assertEqual(response,before)  # No renormalization/fabrication.


class CliTests(unittest.TestCase):
    def test_default_preview_never_calls_model_and_records_no_fake_answers(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "preview.json"
            with patch.object(selection, "ask_jev") as model, patch("sys.stdout", new_callable=io.StringIO):
                status = selection.main(["--samples", "1000", "--output", str(output)])
            model.assert_not_called()
            report = json.loads(output.read_text())
        self.assertEqual(status, 0)
        self.assertEqual(report["mode"], "request_preview")
        self.assertTrue(report["proposal_only"])
        self.assertNotIn("response", report)
        self.assertNotIn("proposals", report)

    def test_missing_key_is_explicit_and_report_does_not_claim_live_success(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "failed.json"
            with patch.dict(os.environ, {}, clear=True), patch("sys.stdout", new_callable=io.StringIO), patch("sys.stderr", new_callable=io.StringIO):
                status = selection.main(["--live", "--samples", "1000", "--output", str(output)])
            report = json.loads(output.read_text())
        self.assertEqual(status, 2)
        self.assertEqual(report["mode"], "live_requested")
        self.assertIn("TYPESAFE_API_KEY is missing", report["error"])
        self.assertNotIn("response", report)

    def test_invalid_model_response_is_preserved_without_validated_proposals(self):
        response = fake_response(selection.prepare_request(*fixture(), samples=1000))
        response["answers"]["geometry/bottle"]["choice"] = "invented-geometry"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "invalid.json"
            with patch.object(selection, "ask_jev", return_value=response), patch("sys.stdout", new_callable=io.StringIO), patch("sys.stderr", new_callable=io.StringIO):
                status = selection.main(["--live", "--samples", "1000", "--output", str(output)])
            report = json.loads(output.read_text())
        self.assertEqual(status, 2)
        self.assertEqual(report["response"], response)
        self.assertIn("validation_error", report)
        self.assertNotIn("proposals", report)


@unittest.skipUnless(importlib.util.find_spec("typesafe_sdk"), "optional Jev SDK not installed")
class SdkTransportTests(unittest.TestCase):
    def test_real_sdk_serializes_request_and_parses_response_without_network(self):
        import httpx2
        from typesafe_sdk import TypeSafeClient

        request = selection.prepare_request(*fixture(), samples=1000)
        fabricated = fake_response(request)
        received = []

        def handler(wire):
            self.assertEqual(str(wire.url), "https://api.typesafe.ai/v1/systemone")
            received.append(json.loads(wire.content))
            return httpx2.Response(200, json=fabricated)

        client = TypeSafeClient(api_key="offline-test-not-a-real-key", transport=httpx2.MockTransport(handler), compression=False)
        with patch.dict(os.environ, {"TYPESAFE_API_KEY": "offline-test-not-a-real-key"}), patch("typesafe_sdk.TypeSafeClient", return_value=client):
            response = selection.ask_jev(request)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0]["model"], request["model"])
        self.assertEqual(received[0]["state"], request["state"])
        self.assertEqual(received[0]["questions"], request["questions"])
        self.assertEqual(response["model"], fabricated["model"])
        self.assertEqual(len(selection.decode_response(request, response)), 3)

    def test_sdk_http_failure_produces_no_fallback_and_does_not_log_error_body(self):
        import httpx2
        from typesafe_sdk import TypeSafeClient

        request = selection.prepare_request(*fixture(), samples=1000)
        client = TypeSafeClient(
            api_key="offline-test-not-a-real-key", compression=False,
            transport=httpx2.MockTransport(lambda wire: httpx2.Response(400, json={"error": "private-error-body"})),
        )
        with patch.dict(os.environ, {"TYPESAFE_API_KEY": "offline-test-not-a-real-key"}), patch("typesafe_sdk.TypeSafeClient", return_value=client):
            with self.assertRaises(ValueError) as error:
                selection.ask_jev(request)
        self.assertNotIn("private-error-body", str(error.exception))
        self.assertNotIn("offline-test-not-a-real-key", str(error.exception))


if __name__ == "__main__":
    unittest.main()
