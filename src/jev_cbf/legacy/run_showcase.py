"""Prepare/run/render the dynamic simulation proof of concept.

prepare and baselines are offline. run --live makes bounded, recorded Jev calls
for the frozen fixtures, sharing each initial answer with once/repeated modes.
"""

import argparse
import importlib.metadata
import json
from pathlib import Path
import subprocess
import numpy as np
from jev_cbf.paths import ROOT
from jev_cbf.legacy.selection import prepare_request
from jev_cbf.selection.runtime import ConstraintRuntime, motion_request, object_motion, query_saved
from jev_cbf.selection.context import robot_context
from jev_cbf.legacy.robot_selection import scene_input
from jev_cbf.control.simulation import collision_diagnostics, run_case, scene_spec
from jev_cbf.upstream import load_upstream
from jev_cbf.provenance import check_manifest
from jev_cbf.io import digest, read, save
from jev_cbf.control.model import dynamics_comparison, panda_spec
from jev_cbf.provenance import source_files

def prepare(folder, scenes_path, questions_path):
    if folder.exists() and any(folder.iterdir()):
        raise ValueError("prepare needs a new/empty output directory")
    config, template = read(scenes_path), read(questions_path)
    demo, utils = load_upstream(ROOT.parent / "frax", ROOT.parent / "cbfpy")
    robot = demo.load_panda()
    folder.mkdir(parents=True, exist_ok=True)
    save(folder / "scenes.json", config)
    save(folder / "questions.json", template)
    q, v = np.asarray(robot.default_configuration), np.zeros(7)
    context = robot_context(robot, {"qpos": q[None], "qvel": v[None]})
    start = np.asarray(robot.ee_transform(q))[:3, 3]
    first = scene_input(config["scenes"][0], config["goal_m"], start)
    base = prepare_request(first, template, robot_context=context)
    result, trace = run_case(demo, utils, {"request": base}, "off", frax_dir=ROOT.parent / "frax",
                             steps=3000, nominal_free_space=True, model_profile=config["model_profile"],
                             scene_guard=config["scene_guard_m_s2"])
    if not result["task_success"]:
        raise ValueError("free-space original policy did not pass; no model calls allowed")
    np.savez_compressed(folder / "nominal_goal.npz", **trace)
    save(folder / "nominal_goal.json", result)
    goal_q = trace["qpos"][-1, :7]
    native = panda_spec(utils.mujoco, ROOT.parent / "frax", profile="frax", robot=robot).compile()
    parity = [dynamics_comparison(utils.mujoco, native, robot, q+delta, np.arange(7)*.03, np.arange(7))
              for delta in (0., .01, -.02)]
    if max(p["max_acceleration_difference_rad_s2"] for p in parity) > 1e-8:
        raise ValueError("native/FRAX dynamics parity failed")
    save(folder / "dynamics_parity.json", parity)
    predict = demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
    frozen = ["scenes.json", "questions.json", "nominal_goal.npz"]
    for scene in config["scenes"]:
        facts = scene_input(scene, config["goal_m"], start)
        request = prepare_request(facts, template, robot_context=context)
        positions, velocities = object_motion(scene["objects"], 0.)
        request = motion_request(request, robot, q, v, positions, velocities, 0., goal_q,
                                 config["forecast_horizon_s"], predict)
        path = folder / scene["id"] / "initial/request.json"
        save(path, request)
        frozen.append(str(path.relative_to(folder)))
    save(folder / "manifest.json", {
        "scope": "Two hand-designed dynamic fixtures, oracle poses/velocities, no contact; no statistical or hardware guarantee.",
        "cases": [{"id": s["id"], "title": s["title"]} for s in config["scenes"]],
        "frozen_files": {name: digest(folder/name) for name in frozen},
        "maximum_logical_sdk_invocations": len(config["scenes"])*int(np.ceil(config["duration_s"]/config["query_period_s"])),
        "model": "jev-latest", "original_static_results": "artifacts/suite/panda_static_v2",
        "correction_audit": "artifacts/diagnosis/static_corrected_v1",
        "upstream_revisions": {name: subprocess.check_output(["git", "-C", str(ROOT.parent/name), "rev-parse", "HEAD"], text=True).strip()
                               for name in ("frax", "oscbf", "cbfpy")},
        "runtime_versions": {name: importlib.metadata.version(name) for name in ("jax", "mujoco", "elastiqp", "numpy")},
        "runtime_source_sha256_before_live": {name: digest(ROOT/name) for name in source_files(legacy=True)},
        "decisions": {
            "plant": "In-memory FRAX inertias; rigid closed gripper; no damping/armature; independent native parity checked. Not calibrated Panda physics.",
            "sampled_guard": "0.05 m/s² scene inequality tightening; raw geometry/gains unchanged; empirical allowance, not formal robust-CBF bound.",
            "motion": "Prescribed mocap object paths, piecewise constant velocity. Explicit time state accounts for velocity in RD2 derivatives; acceleration jumps not formally certified.",
            "selection": "Initial answer shared across once/repeated. Repeated queries every 2 simulated seconds in a wall-paced rollout; actual response/apply latency recorded. No confidence threshold.",
            "failure": "Keep previous policy on failed/review/inadmissible updates; initial conservative all-on until accepted. Not a certified safe stop/fallback.",
            "capacity": "Four slots per object; masks preserve QP shape. Enabled row counts do not mean smaller allocated QPs.",
        },
    })
    print(f"Prepared {folder}; no model calls made.", flush=True)

def run_modes(folder, modes, *, live=False):
    check_manifest(folder)
    config = read(folder/"scenes.json")
    with np.load(folder/"nominal_goal.npz") as trace:
        goal_q = trace["qpos"][-1, :7]
    demo, utils = load_upstream(ROOT.parent / "frax", ROOT.parent / "cbfpy")
    for scene in config["scenes"]:
        case_dir = folder/scene["id"]
        summary_path = case_dir/"comparison.json"
        summary = read(summary_path) if summary_path.exists() else {"runs": [], "failures": {}}
        request = read(case_dir/"initial/request.json")
        for mode in modes:
            if any(r["mode"] == mode for r in summary["runs"]) or mode in summary["failures"]:
                continue
            initial = None
            if mode in ("once", "repeated"):
                if not live:
                    raise ValueError("new model-selected rollouts require --live; use saved artifacts to view")
                initial = query_saved(case_dir/"initial", request)
            runtime = ConstraintRuntime(scene, config, request, initial, goal_q, mode, case_dir/mode, live=live)
            try:
                result, trace = run_case(demo, utils, {"request": request}, "off", frax_dir=ROOT.parent/"frax",
                                         steps=round(config["duration_s"]/config["control_dt_s"]),
                                         model_profile=config["model_profile"], scene_guard=config["scene_guard_m_s2"],
                                         control_dt=config["control_dt_s"], runtime=runtime)
                np.savez_compressed(case_dir/f"{mode}.npz", **trace)
                summary["runs"].append(result)
                print(f"{scene['id']}/{mode}: complete={result['complete']}, success={result['task_success']}, "
                      f"contact={result['collision']['any_sampled_scene_violation']}, "
                      f"scene_h2={result['minimum_selected_scene_sphere_barrier_m']}, "
                      f"error={result['final_position_error_m']:.4f}, enabled pairs={result['enabled_scene_pair_counts']}", flush=True)
            except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
                summary["failures"][mode] = str(error)
                if runtime.executor:
                    runtime.executor.shutdown(wait=True)
                print(f"{scene['id']}/{mode}: FAILED: {error}", flush=True)
            save(summary_path, summary)

def audit_collisions(folder):
    """Recheck saved states independently; retain original reports/trajectories."""
    check_manifest(folder)
    config = read(folder/"scenes.json")
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    for scene in config["scenes"]:
        case = folder/scene["id"]
        if not (case/"comparison.json").exists():
            continue
        request = read(case/"initial/request.json")
        spec = scene_spec(utils.mujoco, ROOT.parent/"frax", request["state"], np.empty((0, 3)), [], moving_objects=True)
        spec.attach(panda_spec(utils.mujoco, ROOT.parent/"frax", profile="frax", robot=demo.load_panda()), prefix="", frame=spec.worldbody.add_frame())
        model = spec.compile()
        path = case/"collision_recheck.json"
        audit = read(path) if path.exists() else {}
        for result in read(case/"comparison.json")["runs"]:
            mode = result["mode"]
            trajectory_path = case/f"{mode}.npz"
            distance_path = case/f"{mode}_collision_audit.npz"
            source_hash = digest(trajectory_path)
            if mode in audit and audit[mode]["source_trajectory_sha256"] == source_hash and distance_path.exists():
                continue
            with np.load(trajectory_path) as trajectory:
                collision, distances = collision_diagnostics(utils.mujoco, model, trajectory["qpos"], trajectory["qvel"],
                    [obj["id"] for obj in scene["objects"]], trajectory["object_positions_m"])
            np.savez_compressed(distance_path, object_distance_m=distances)
            audit[mode] = {"source_trajectory_sha256": source_hash, "collision": collision,
                           "distance_trace": distance_path.name,
                           "evaluator": "All native robot collision geoms; exclude every scene mocap geom from robot set; independent MjData with measured per-state object poses."}
            save(path, audit)
            print(f"Audit {scene['id']}/{mode}: contact={collision['any_sampled_scene_violation']}", flush=True)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "baselines", "run", "audit", "gallery", "status"))
    parser.add_argument("--output", type=Path, default=ROOT/"artifacts/showcase/dynamic_v1")
    parser.add_argument("--scenes", type=Path, default=ROOT/"configs/legacy/scenes/dynamic_scenes.json")
    parser.add_argument("--questions", type=Path, default=ROOT/"configs/legacy/prompts/dynamic_selection.json")
    parser.add_argument("--live", action="store_true")
    args = parser.parse_args(argv)
    folder = args.output.resolve()
    try:
        if args.action == "prepare" or (args.action == "run" and not (folder/"manifest.json").exists()):
            prepare(folder, args.scenes, args.questions)
        if args.action in ("baselines", "run"):
            run_modes(folder, ("off", "all"))
        if args.action == "run" and args.live:
            run_modes(folder, ("once", "repeated"), live=True)
        if args.action in ("audit", "run"):
            audit_collisions(folder)
        if args.action in ("gallery", "run"):
            from jev_cbf.legacy.showcase_gallery import render_gallery
            render_gallery(folder)
        if args.action == "status":
            manifest = check_manifest(folder)
            calls = [dict(path=str(p.relative_to(folder)), **read(p)) for p in folder.rglob("attempt.json")]
            print(json.dumps({"manifest": manifest, "model_attempts": calls}, indent=2))
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        parser.exit(2, f"Showcase failed: {error}\n")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
