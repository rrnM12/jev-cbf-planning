"""Inspect a text-only Jev proposal; never construct or deploy robot constraints.

Default execution previews the request without an SDK, API key, or network call.
Use --live explicitly to send the generated state/questions through TypeSafe's SDK.
"""

from jev_cbf.paths import ROOT
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from jev_cbf.geometry.covers import evaluate_covers, make_sphere_covers, make_shape_covers
from jev_cbf.selection.validation import _vector
from jev_cbf.selection.validation import scene_motion
from jev_cbf.selection.validation import validate_scene
from jev_cbf.selection.validation import validate_selection
from jev_cbf.selection.validation import decode_response
from jev_cbf.selection.client import ask_jev

def nominal_clearance(cover, object_center, start, goal, robot_radius):
    """Exact sphere-union/straight swept-sphere clearance, not whole-arm risk.

    Negative means the two modeled volumes overlap. This scalar is a descriptive
    feature only, not a min-aggregated CBF or a planned path safety certificate.
    """
    centers = np.asarray(cover.centers_m) + object_center
    segment = goal - start
    length_squared = float(segment @ segment)
    fractions = (
        np.clip((centers - start) @ segment / length_squared, 0, 1)
        if length_squared > 0 else np.zeros(len(centers))
    )
    closest = start + fractions[:, None] * segment
    return float(np.min(
        np.linalg.norm(centers - closest, axis=1) - np.asarray(cover.radii_m) - robot_radius
    ))

def prepare_request(scene, template, *, model="jev-latest", counts=(1, 2, 4), samples=100_000, seed=0, robot_context=None, shape_aware=False):
    """Reuse inspected covers; attach measured features and editable Choice questions.

    The example's frame is fixed and all objects are axis-aligned. Changing text
    does not update numeric start/goal/geometry. No oracle avoidance labels or
    expected model answers are generated or passed in this experiment.
    """
    if not isinstance(scene.get("task"), str) or not scene["task"].strip():
        raise ValueError("scene.task must be nonempty text")
    if not isinstance(scene.get("frame"), str) or not scene["frame"].strip():
        raise ValueError("scene.frame must state the coordinate/length conventions")
    if robot_context is None:
        proxy = scene["robot_proxy"]
        start = _vector(proxy["start_m"], "start_m")
        goal = _vector(proxy["goal_m"], "goal_m")
        radius = proxy["radius_m"]
        if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0:
            raise ValueError("robot_proxy.radius_m must be finite and positive")
    else:
        _vector(scene["motion"]["start_m"], "start_m")
        _vector(scene["motion"]["goal_m"], "goal_m")
    if not scene.get("objects"):
        raise ValueError("scene.objects must not be empty")
    state = {
        "description": scene.get("description", ""),
        "frame": scene["frame"],
        "task": scene["task"],
        "feature_notes": (
            "Candidate sphere centres are object-local; object center_m translates "
            "them into world coordinates. Objects have identity rotation. "
            "nominal_clearance_m is minimum sphere-cover clearance from the "
            "straight swept proxy, including proxy radius and NO extra margin. "
            "It is not whole-arm collision risk or a CBF. Excess volumes are "
            "Monte Carlo estimates, with one estimated standard error."
        ),
        "objects": [],
    }
    if robot_context is None:
        state["robot_proxy"] = deepcopy(proxy)
    else:
        state["robot_context"] = deepcopy(robot_context)
        state["motion"] = deepcopy(scene["motion"])
        state["feature_notes"] = (
            "Ground-truth axis-aligned object bounds; sphere centers object-local, "
            "translated by center_m. Covers are unions covering the full box. "
            "Robot-aware motion features are attached by the robot request builder, "
            "not estimated by a single end-effector sphere. No added physical margin."
        )
    questions = {}
    object_ids = set()
    for obj in scene["objects"]:
        object_id = obj["id"]
        if not isinstance(object_id, str) or not object_id.strip() or object_id in object_ids:
            raise ValueError("object IDs must be unique, nonempty strings")
        object_ids.add(object_id)
        if "rotation" in obj or "quaternion" in obj:
            raise ValueError("rotated objects are not supported by this axis-aligned fixture")
        center = _vector(obj["center_m"], f"{object_id}.center_m")
        size = _vector(obj["size_m"], f"{object_id}.size_m")
        covers = make_shape_covers(size/2, shape=obj["shape"], counts=counts) if shape_aware else make_sphere_covers(size / 2, counts)
        measurement = evaluate_covers(size / 2, covers, shape=obj["shape"], samples=samples, seed=seed)
        candidates = []
        for cover, result in zip(covers, measurement["candidates"], strict=True):
            if result["uncovered_object_samples"]:
                raise ValueError(f"{object_id}/{cover.candidate_id}: sampled coverage diagnostic failed")
            candidates.append({
                "id": cover.candidate_id,
                "centers_object_m": cover.centers_m,
                "radii_m": cover.radii_m,
                "primitive_count": len(cover.radii_m),
                "excess_volume_m3": result["excess_volume_m3"],
                "excess_volume_standard_error_m3": result["excess_volume_standard_error_m3"],
            })
            if robot_context is None:
                candidates[-1]["nominal_clearance_m"] = nominal_clearance(cover, center, start, goal, radius)
        state["objects"].append({
            "id": object_id,
            "description": obj.get("description", ""),
            "shape": obj["shape"],
            "size_m": size.tolist(),
            "center_m": center.tolist(),
            "candidates": candidates,
        })
        role = deepcopy(template["role"])
        role["type"] = "choice"
        role["instructions"] = role["instructions"].format(object_id=object_id)
        if set(role["criteria"]) != {"avoid", "omit", "review"}:
            raise ValueError("role criteria must be avoid/omit/review; update the decoder if experimenting with other roles")
        questions[f"role/{object_id}"] = role
        questions[f"geometry/{object_id}"] = {
            "type": "choice",
            "instructions": template["geometry"]["instructions"].format(object_id=object_id),
            "criteria": {c["id"]: c for c in candidates},
        }
    if shape_aware:
        if robot_context is not None:
            state["feature_notes"] = (
                "Ground-truth axis-aligned object shapes; sphere centers are object-local and translated by center_m. "
                "Every union covers the full reference shape. Box-section candidates cover the bounding box; "
                "shape-aware cylinder candidates cover the cylinder itself. Robot-aware motion features come "
                "from the request builder, not a single end-effector proxy. Excess-volume errors are Monte Carlo "
                "sampling errors, not perception uncertainty. No physical margin or safety guarantee is implied."
            )
        state["feature_notes"] += " Shape-aware cylinder candidates cover the cylinder, not every corner of its bounding box. All candidates cover the full stated reference shape; grid candidates tile two box axes. No calibrated uncertainty margin."
    # JSON-native values make preview serialization, SDK input, and recorded input
    # identical. Reject NaN/Infinity before any request or output is written.
    return json.loads(json.dumps({"model": model, "state": state, "questions": questions}, allow_nan=False))

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=ROOT / "configs/legacy/scenes/selection_scene.json")
    parser.add_argument("--questions", type=Path, default=ROOT / "configs/legacy/prompts/selection.json")
    parser.add_argument("--task", help="override only task text; numeric scene/goal stay unchanged")
    parser.add_argument("--model", default="jev-latest", help="Jev model ID or alias; actual returned ID is recorded")
    parser.add_argument("--counts", type=int, nargs="+", default=(1, 2, 4))
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--live", action="store_true", help="send one SDK request (normal SDK retries apply)")
    parser.add_argument("--output", type=Path, help="save request and, if live, raw response/proposals as JSON")
    args = parser.parse_args(argv)
    report = None
    status = 0
    try:
        scene = json.loads(args.scene.read_text())
        template = json.loads(args.questions.read_text())
        if args.task is not None:
            scene["task"] = args.task
        request = prepare_request(scene, template, model=args.model, counts=args.counts, samples=args.samples, seed=args.seed)
        report = {
            "mode": "live_requested" if args.live else "request_preview",
            "proposal_only": True,
            "geometry_sampling": {"samples": args.samples, "seed": args.seed},
            "request": request,
        }
        print("Selection inspection only: no robot, CBF deployment, or contact permission.")
        print("Task:", request["state"]["task"])
        for obj in request["state"]["objects"]:
            print(f"\n{obj['id']} at {obj['center_m']} m")
            for candidate in obj["candidates"]:
                print(f"  {candidate['id']}: {candidate['primitive_count']} spheres; "
                      f"excess {candidate['excess_volume_m3'] * 1000:.3f} L; "
                      f"nominal clearance {candidate['nominal_clearance_m'] * 1000:.1f} mm")
        if args.live:
            start = time.perf_counter()
            response = ask_jev(request)
            report.update(mode="live_jev", elapsed_seconds=time.perf_counter() - start, response=response)
            # Preserve the raw response for inspection even if decoding fails.
            try:
                report["proposals"] = decode_response(request, response)
            except (KeyError, TypeError, ValueError) as error:
                report["validation_error"] = str(error)
                raise
            print("\nJev proposals:", json.dumps(report["proposals"], indent=2))
            for question_id, answer in response["answers"].items():
                print(f"{question_id}: model confidence {answer['confidence']:.3f}; "
                      f"probabilities {json.dumps(answer['probabilities'])}")
            print(f"Actual model: {response['model']}; SDK call: {report['elapsed_seconds']:.3f} s")
        else:
            print("\nREQUEST PREVIEW ONLY: no model answers, API key, or network call.")
    except (OSError, KeyError, TypeError, ValueError) as error:
        if report is not None:
            report["error"] = str(error)
        print(f"Selection inspection failed: {error}", file=sys.stderr)
        status = 2
    if args.output and report is not None:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
            print(f"Report: {args.output}")
        except OSError as error:
            parser.exit(2, f"Could not write selection report: {error}\n")
    return status

if __name__ == "__main__":
    raise SystemExit(main())
