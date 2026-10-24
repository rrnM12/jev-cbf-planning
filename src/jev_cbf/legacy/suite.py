"""Reusable, phased robot-context selection -> static simulation -> gallery workflow.

prepare/baselines are offline. select/run require --live for the approved one-shot
calls. Saved attempts prevent accidental repeat calls on resume. No online loop.
"""

import argparse
from datetime import datetime, timezone
import html
import importlib.metadata
import json
from pathlib import Path
import re
import subprocess
import time
import numpy as np
from jev_cbf.paths import ROOT
from jev_cbf.selection.client import ask_jev
from jev_cbf.selection.validation import decode_response
from jev_cbf.legacy.selection import prepare_request
from jev_cbf.legacy.robot_selection import nominal_sphere_path, prepare_robot_request, scene_input
from jev_cbf.selection.context import robot_context
from jev_cbf.upstream import load_upstream
from jev_cbf.control.simulation import run_case
from jev_cbf.io import save
from jev_cbf.io import digest
from jev_cbf.io import read
from jev_cbf.provenance import check_manifest

def prepare(folder, scenes_path, questions_path, frax_dir, cbfpy_dir, *, samples=100_000, seed=0, model="jev-latest"):
    if folder.exists() and any(folder.iterdir()):
        raise ValueError("prepare requires a new/empty output directory")
    config, template = read(scenes_path), read(questions_path)
    ids = [scene["id"] for scene in config["scenes"]]
    if not 1 <= len(ids) <= 6 or len(set(ids)) != len(ids) or any(not re.fullmatch(r"[a-z0-9_]+", name) for name in ids):
        raise ValueError("need one to six unique scene IDs containing lowercase letters/digits/underscores")
    if config["steps"] < 2 or any("goal_m" in scene for scene in config["scenes"]):
        raise ValueError("need positive rollout length and one shared suite goal (per-scene goals are not supported yet)")
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "scenes.json").write_bytes(scenes_path.read_bytes())
    (folder / "questions.json").write_bytes(questions_path.read_bytes())
    demo, utils = load_upstream(frax_dir, cbfpy_dir)
    robot = demo.load_panda()
    q = np.asarray(robot.default_configuration)
    initial = {"qpos": np.r_[q, 0., 0.][None], "qvel": np.zeros((1, 9))}
    context = robot_context(robot, initial)
    first = dict(config["scenes"][0], goal_m=config["goal_m"])
    facts = scene_input(first, config["goal_m"], np.asarray(robot.ee_transform(q))[:3, 3])
    seed_request = prepare_request(facts, template, robot_context=context, model=model, samples=samples, seed=seed)
    nominal_result, nominal = run_case(
        demo, utils, {"request": seed_request}, "off", frax_dir=frax_dir,
        steps=config["steps"], nominal_free_space=True,
    )
    if not nominal_result["complete"] or not nominal_result["within_2cm_at_end"]:
        raise ValueError("original controller does not reach this goal in free space; no model calls allowed")
    save(folder / "nominal_prediction.json", nominal_result)
    np.savez_compressed(folder / "nominal_prediction.npz", **nominal)
    sphere_path = nominal_sphere_path(robot, nominal)
    cases = []
    for scene in config["scenes"]:
        scene = dict(scene, goal_m=config["goal_m"])
        request, qualification = prepare_robot_request(
            scene, template, robot, nominal, sphere_path, utils.mujoco, frax_dir,
            samples=samples, seed=seed, model=model,
        )
        case_dir = folder / scene["id"]
        save(case_dir / "request.json", request)
        save(case_dir / "qualification.json", qualification)
        cases.append({"id": scene["id"], "title": scene["title"], "qualification": qualification})
        print(f"Prepared {scene['id']}: initial native clear={qualification['native_initial_clear']}, "
              f"nominal end clear={qualification['native_predicted_end_clear']}, "
              f"all-four initial admissible={qualification['all_four_initial_model_clear']}", flush=True)
    frozen = ["scenes.json", "questions.json", "nominal_prediction.npz"] + [f"{name}/request.json" for name in ids]
    manifest = {
        "kind": "robot-aware static suite; six variations of one no-contact reaching task, not six independent tasks",
        "created_utc": datetime.now(timezone.utc).isoformat(), "cases": cases,
        "steps": config["steps"], "all_geometry": config["all_geometry"],
        "model_alias": model, "sdk_invocation_budget": len(ids),
        "sampling": {"samples": samples, "seed": seed},
        "ready_for_live": all(c["qualification"][key] for c in cases
                              for key in ("native_initial_clear", "native_predicted_end_clear", "all_four_initial_model_clear")),
        "frozen_files": {name: digest(folder / name) for name in frozen},
        "upstream_revisions": {name: subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
                               for name, path in (("frax", frax_dir), ("cbfpy", cbfpy_dir))},
        "runtime_versions": {name: importlib.metadata.version(name) for name in ("jax", "mujoco", "numpy", "elastiqp")},
    }
    save(folder / "manifest.json", manifest)
    print(f"Frozen suite: {folder}; ready for live={manifest['ready_for_live']}. No model calls made.", flush=True)
    return manifest

def one_shot(folder, case_id, request, *, ask=ask_jev):
    """Record an attempt BEFORE calling; failure never becomes a guessed answer."""
    manifest = check_manifest(folder)
    if not manifest["ready_for_live"]:
        raise ValueError("scene qualification failed; no model request sent")
    if case_id not in [case["id"] for case in manifest["cases"]]:
        raise ValueError("unknown scene")
    frozen = read(folder / case_id / "request.json")
    if request != frozen:
        raise ValueError("request differs from frozen input")
    ledger_path = folder / "attempts.json"
    attempts = read(ledger_path) if ledger_path.exists() else {}
    if case_id in attempts:
        print(f"Skipping {case_id}: already attempted; no repeat SDK invocation.", flush=True)
        return
    if len(attempts) >= manifest["sdk_invocation_budget"]:
        raise ValueError("approved SDK invocation budget exhausted")
    attempts[case_id] = {"started_utc": datetime.now(timezone.utc).isoformat(), "status": "started"}
    save(ledger_path, attempts)
    report = {"request": request, "proposal_only": True, "mode": "live_requested_robot"}
    began = time.perf_counter()
    try:
        report["response"] = ask(request)
        report["elapsed_seconds"] = time.perf_counter() - began
        report["proposals"] = decode_response(request, report["response"])
        report["mode"] = "live_jev_robot"
        attempts[case_id]["status"] = "success"
    except (ValueError, KeyError, TypeError) as error:
        report["error"] = str(error)
        attempts[case_id]["status"] = "failed"
    save(folder / case_id / "selection.json", report)
    save(ledger_path, attempts)
    print(f"Selection {case_id}: {attempts[case_id]['status']}", flush=True)

def evaluate(folder, frax_dir, cbfpy_dir, modes):
    manifest = check_manifest(folder)
    demo, utils = load_upstream(frax_dir, cbfpy_dir)
    for case in manifest["cases"]:
        case_dir = folder / case["id"]
        summary_path = case_dir / "comparison.json"
        summary = read(summary_path) if summary_path.exists() else {
            "runs": [], "failures": {}, "all_geometry": manifest["all_geometry"],
            "selection_context": "actual Panda/nominal whole-arm context; one-shot ground-truth selection",
            "source_sha256": digest(case_dir / "request.json"), "robot_aware": True,
        }
        for mode in modes:
            if mode in [r["mode"] for r in summary["runs"]] or mode in summary["failures"]:
                continue
            request = read(case_dir / "request.json")
            report = {"request": request, "mode": "robot_request_preview"}
            if mode == "jev":
                if not (case_dir / "selection.json").exists():
                    continue
                report = read(case_dir / "selection.json")
                if report.get("request") != request:
                    summary["failures"][mode] = "saved response input differs from the frozen robot request"
                    save(summary_path, summary)
                    continue
                # Baselines never require a model response. Once available,
                # retain the original response for provenance/rendering.
                save(case_dir / "selection_source.json", report)
                summary["source_model"] = report.get("response", {}).get("model", "failed request")
                summary["source_report_mode"] = report["mode"]
                summary["selection_proposals"] = report.get("proposals", [])
                summary["model_answers"] = report.get("response", {}).get("answers", {})
            elif not (case_dir / "selection_source.json").exists():
                save(case_dir / "selection_source.json", report)
            try:
                result, trajectory = run_case(demo, utils, report, mode, frax_dir=frax_dir,
                                              steps=manifest["steps"], all_geometry=manifest["all_geometry"])
                result["estimated_sum_object_local_excess_m3"] = sum(
                    obj["estimated_object_local_excess_m3"] for obj in result["included_objects"]
                )
                np.savez_compressed(case_dir / f"{mode}.npz", **trajectory)
                summary["runs"].append(result)
                print(f"{case['id']} / {mode}: success={result['task_success']}, "
                      f"collision/contact={result['collision']['any_sampled_scene_violation']}, "
                      f"final error={result['final_position_error_m']:.4f} m, pairs={result['scene_pair_constraint_count']}", flush=True)
            except (RuntimeError, ValueError, KeyError, TypeError) as error:
                summary["failures"][mode] = str(error)
                print(f"{case['id']} / {mode}: FAILED: {error}", flush=True)
            save(summary_path, summary)

def gallery(folder, frax_dir, *, html_only=False):
    from jev_cbf.legacy.static_gallery import main as render_comparison
    manifest = check_manifest(folder)
    rows = []
    results = []
    for case in manifest["cases"]:
        case_dir = folder / case["id"]
        path = case_dir / "comparison.json"
        if not path.exists():
            continue
        summary = read(path)
        if summary["runs"]:
            render_comparison(["--run-dir", str(case_dir), "--frax-dir", str(frax_dir),
                               "--width", "480", "--height", "360", "--fps", "8"] + (["--html-only"] if html_only else []))
        for mode in ("off", "all", "jev"):
            matches = [r for r in summary["runs"] if r["mode"] == mode]
            if matches:
                r = matches[0]
                minimum = min(r["collision"]["minimum_distance_m_by_object"].values())
                barrier = r["minimum_selected_scene_sphere_barrier_m"]
                barrier_text = "not enabled" if barrier is None else f"{barrier*1000:.3f}"
                rows.append(f"<tr><td><a href='{case['id']}/index.html'>{html.escape(case['title'])}</a></td>"
                            f"<td>{mode}</td><td>{r['task_success']}</td><td>{r['collision']['any_sampled_scene_violation']}</td>"
                            f"<td>{r['final_position_error_m']*1000:.1f}</td><td>{r['scene_pair_constraint_count']}</td>"
                            f"<td>{minimum*1000:.1f}</td><td>{barrier_text}</td><td>{r['estimated_sum_object_local_excess_m3']*1000:.2f}</td></tr>")
                results.append({"scene_id": case["id"], **r})
            else:
                reason = summary["failures"].get(mode, "Not run / no model answer yet")
                rows.append(f"<tr><td>{html.escape(case['title'])}</td><td>{mode}</td><td colspan='7'>FAILED / NOT RUN: {html.escape(reason)}</td></tr>")
    counts = {mode: {"completed": sum(r["complete"] for r in results if r["mode"] == mode),
                     "task_successes": sum(r["task_success"] for r in results if r["mode"] == mode),
                     "modeled_barrier_violations": sum(r["any_sampled_negative_h2"] for r in results if r["mode"] == mode),
                     "sampled_scene_violations": sum(r["collision"]["any_sampled_scene_violation"] for r in results if r["mode"] == mode)}
              for mode in ("off", "all", "jev")}
    save(folder / "suite_results.json", {"counts": counts, "runs": results, "scene_count": len(manifest["cases"]),
                                         "scope": "Descriptive counts over six hand-designed variations of one task; no statistical confidence or general safety/latency claim."})
    document = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
      <title>Robot-aware static suite</title><style>body{font:16px system-ui;background:#111821;color:#e5edf5;margin:30px;line-height:1.5}
      a{color:#8ad6ff}table{border-collapse:collapse;width:100%}td,th{text-align:left;border-bottom:1px solid #405064;padding:10px}
      main{max-width:1500px;margin:auto}pre{white-space:pre-wrap}</style><main><h1>Robot-aware static suite</h1>
      <p>Six variations of one fixed, no-contact Panda reaching task. Click scene names to watch the recorded comparisons.
      Inputs are oracle geometry and measured nominal whole-arm motion, not vision. Scenes and questions were frozen before model answers.</p>
      <p>OFF: only scene CBFs off. ALL: all objects with fixed four-sphere covers, independent of Jev.
      JEV: one-shot selected objects and sphere covers. Every mode keeps the original controller and robot protections.
      This compares combined role/geometry choice, not an isolated role effect. A CBF filter is not a path planner.</p>
      <p>Contact checks include every object and all native robot collision geometry, including gripper/base. Zero distance is flagged.
      These are sampled checks with an elastic QP, not a continuous-time or hardware safety guarantee. Failures are retained.</p>
      <p>Task success means a complete rollout, final EE error below 20 mm, no sampled scene collision/contact,
      and no negative modeled h_2. This unchanged exploratory tolerance is not a physical safety margin.</p>
      <p>Negative modeled barriers are reported separately from physical contact. No sampled contact does not establish that every CBF model constraint held.</p>
      <table><thead><tr><th>Scene / animation</th><th>Mode</th><th>Task success</th><th>Collision/contact flag</th><th>Final error (mm)</th>
      <th>Scene CBF pairs</th><th>Minimum distance (mm)</th><th>Minimum modeled scene h_2 (mm)</th><th>Sum local excess (L)</th></tr></thead><tbody>''' + "".join(rows) + '''</tbody></table>
      <p>Excess is the sum of per-object Monte Carlo excess-union estimates, not a global obstacle union or robot-expanded volume.</p>
      <h2>Descriptive counts (not a statistical benchmark)</h2><pre>''' + html.escape(json.dumps(counts, indent=2)) + '''</pre>
      <p><a href="manifest.json">Frozen protocol/provenance</a> · <a href="suite_results.json">Full results</a> · <a href="attempts.json">Model call ledger</a></p></main></html>'''
    (folder / "index.html").write_text(document)
    print(f"Suite gallery: {folder / 'index.html'}", flush=True)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "baselines", "select", "evaluate", "gallery", "run", "status"))
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/suite/panda_static_v2")
    parser.add_argument("--scenes", type=Path, default=ROOT / "configs/legacy/scenes/robot_scenes.json")
    parser.add_argument("--questions", type=Path, default=ROOT / "configs/legacy/prompts/robot_selection.json")
    parser.add_argument("--model", default="jev-latest")
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--live", action="store_true", help="authorize one SDK invocation per unattempted frozen scene, up to the six-scene budget")
    parser.add_argument("--html-only", action="store_true", help="refresh gallery tables without rerendering animations")
    parser.add_argument("--frax-dir", type=Path, default=ROOT.parent / "frax")
    parser.add_argument("--cbfpy-dir", type=Path, default=ROOT.parent / "cbfpy")
    args = parser.parse_args(argv)
    folder, frax_dir, cbfpy_dir = args.output.resolve(), args.frax_dir.resolve(), args.cbfpy_dir.resolve()
    if args.action == "select" and not args.live:
        parser.error("select requires explicit --live; no model call sent")
    try:
        if args.action == "prepare" or (args.action == "run" and not (folder / "manifest.json").exists()):
            prepare(folder, args.scenes, args.questions, frax_dir, cbfpy_dir, samples=args.samples, seed=args.seed, model=args.model)
        if args.action in ("baselines", "run"):
            evaluate(folder, frax_dir, cbfpy_dir, ("off", "all"))
        if args.action == "select" or (args.action == "run" and args.live):
            manifest = check_manifest(folder)
            for case in manifest["cases"]:
                one_shot(folder, case["id"], read(folder / case["id"] / "request.json"))
        if args.action == "evaluate" or (args.action == "run" and args.live):
            evaluate(folder, frax_dir, cbfpy_dir, ("jev",))
        if args.action in ("gallery", "run"):
            gallery(folder, frax_dir, html_only=args.html_only)
        if args.action == "status":
            manifest = check_manifest(folder)
            print(json.dumps({"ready_for_live": manifest["ready_for_live"], "scenes": [case["id"] for case in manifest["cases"]],
                              "attempts": read(folder / "attempts.json") if (folder / "attempts.json").exists() else {}}, indent=2))
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        parser.exit(2, f"Suite failed: {error}\n")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
