import math
import numpy as np

def _vector(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain three finite coordinates in metres")
    return vector

def scene_motion(state):
    """Shared world-metre endpoints; robot motion is not a sphere proxy."""
    motion = state["motion"] if "robot_context" in state else state["robot_proxy"]
    return _vector(motion["start_m"], "start_m"), _vector(motion["goal_m"], "goal_m")

def validate_scene(state):
    """Validate geometry independently of whether model answers exist yet."""
    scene_motion(state)
    ids = set()
    if not state.get("objects"):
        raise ValueError("scene needs objects")
    for obj in state["objects"]:
        if not isinstance(obj["id"], str) or not obj["id"] or obj["id"] in ids:
            raise ValueError("object IDs must be unique nonempty strings")
        ids.add(obj["id"])
        _vector(obj["center_m"], "object center")
        size = _vector(obj["size_m"], "object dimensions")
        if np.any(size <= 0) or obj["shape"] not in ("box", "cylinder"):
            raise ValueError("expected positive box/cylinder dimensions")
        if obj["shape"] == "cylinder" and size[0] != size[1]:
            raise ValueError("circular Z-cylinder requires equal X/Y dimensions")
        if "rotation" in obj or "quaternion" in obj:
            raise ValueError("rotated objects are not supported")

def validate_selection(report):
    """Decode a successful saved answer without coupling control to a viewer."""
    if report.get("error") or report.get("validation_error") or "response" not in report:
        raise ValueError("need a successful saved selection response, not a preview or failed run")
    validate_scene(report["request"]["state"])
    proposals = decode_response(report["request"], report["response"])
    if "proposals" in report and report["proposals"] != proposals:
        raise ValueError("cached proposals disagree with the saved model answers")
    return proposals

def decode_response(request, response):
    """Validate IDs/distributions and keep role and conditional geometry separate.

    This validates a response contract, NOT the correctness or safety of a decision.
    Never silently replace the model's answer with an oracle or fallback choice.
    """
    answers = response["answers"]
    if set(answers) != set(request["questions"]):
        raise ValueError("response question IDs do not match the request")
    if not isinstance(response.get("model"), str) or not response["model"]:
        raise ValueError("response must identify the model used")
    for question_id, question in request["questions"].items():
        answer = answers[question_id]
        criteria = question["criteria"]
        probabilities = answer["probabilities"]
        if answer.get("type") != "choice" or answer.get("choice") not in criteria or set(probabilities) != set(criteria):
            raise ValueError(f"{question_id}: invalid Choice answer or candidate IDs")
        values = [answer["confidence"], *probabilities.values()]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ValueError(f"{question_id}: probabilities/confidence must be finite numbers in [0, 1]")
        # The provider's Choice schema says APPROXIMATELY one. Actual responses
        # round probabilities to hundredths, so K entries can accumulate up to
        # K*0.005 rounding error. Keep the raw values and chosen ID unchanged;
        # this is wire-format tolerance, not a confidence/safety threshold.
        if not math.isclose(sum(probabilities.values()), 1, abs_tol=len(criteria)*.005+1e-8):
            raise ValueError(f"{question_id}: probabilities differ from one beyond hundredth-rounding allowance")
        if probabilities[answer["choice"]] < max(probabilities.values()):
            raise ValueError(f"{question_id}: choice must have maximal probability")
    proposals = []
    for obj in request["state"]["objects"]:
        role = answers[f"role/{obj['id']}"]["choice"]
        candidate_id = answers[f"geometry/{obj['id']}"]["choice"]
        proposals.append({
            "object_id": obj["id"],
            "role": role,
            "candidate_id": candidate_id if role == "avoid" else None,
            "conditional_candidate_id": candidate_id,
        })
    return proposals
