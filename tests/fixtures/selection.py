"""Fabricated selection report shared by saved-view/static-suite tests."""

import json
from jev_cbf.paths import ROOT
from jev_cbf.legacy.selection import prepare_request
from jev_cbf.selection.validation import decode_response

def fixture():
    scene = json.loads((ROOT / "configs/legacy/scenes/selection_scene.json").read_text())
    questions = json.loads((ROOT / "configs/legacy/prompts/selection.json").read_text())
    request = prepare_request(scene, questions, samples=1000)
    answers = {}
    for question_id, question in request["questions"].items():
        if question_id.startswith("role/"):
            choice = "omit" if question_id.endswith("far_cube") else "avoid"
        else:
            choice = "box-split-4"
        answers[question_id] = {
            "type": "choice", "choice": choice, "confidence": 0.5,
            "probabilities": {key: float(key == choice) for key in question["criteria"]},
        }
    response = {"model": "offline-fabricated-viewer-fixture", "answers": answers}
    return {
        "mode": "offline_fabricated_test", "request": request, "response": response,
        "proposals": decode_response(request, response),
    }
