"""Prepare, run, replay and inspect the single static-clutter pickup task.

prepare/baselines/gallery/status are offline. run --live uses the approved
persistent dollar budget. Initial decisions are shared by once/repeated.
"""

import argparse
from copy import deepcopy
import importlib.metadata
from pathlib import Path
import re
import shutil
import subprocess
from types import SimpleNamespace

import numpy as np

from api_budget import JevBudget
from inspect_selection import ROOT
from online import query_saved,FaultInjectedJev
from pickup import DEFAULTS, PickupRuntime, pickup_request
from run_static import load_upstream, run_case
from run_suite import check_manifest, digest, read, save


SOURCES = ("pickup.py", "run_pickup.py", "run_static.py", "runtime_cbf.py", "sim_model.py", "online.py", "inspect_selection.py", "api_budget.py", "geometry.py", "robot_selection.py", "study_pickup.py", "perception.py", "pickup_layouts.py")
BUDGET_PATH = ROOT/"artifacts/usage/pickup_research_v1.json"


def runtime_type(scene):
    if 'observation_config' in scene:
        from perception import PerceivedPickupRuntime
        return PerceivedPickupRuntime
    return PickupRuntime


def runtime_versions():
    versions={}
    for name in ("mujoco","jax","elastiqp","typesafe-sdk"):
        try:
            versions[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            if name != "typesafe-sdk":
                raise
            versions[name]=None  # Optional for preparation/offline baselines.
    return versions


def execute(folder, scene, settings, request, mode, demo, utils, *, initial=None, live=False, budget=None):
    output = folder/mode
    if (output/"result.json").exists():
        return read(output/"result.json")
    if (output/"attempt.json").exists():
        raise ValueError(f"interrupted rollout retained: {output}; choose a new run directory")
    save(output/"attempt.json", {"status": "started", "mode": mode})
    runtime = runtime_type(scene)(scene, settings, request, initial, mode, output, live=live)
    if budget is not None:
        runtime.query_options = {"ask": budget}
    try:
        result, trajectory = run_case(demo, utils, {"request": request}, "off", frax_dir=ROOT.parent/"frax",
                                      steps=round(settings["duration_s"]/settings["control_dt_s"]),
                                      model_profile="frax_gripper", scene_guard=settings["scene_guard_m_s2"],
                                      control_dt=settings["control_dt_s"], runtime=runtime)
    except (ValueError, RuntimeError):
        startup=read(output/'startup.json') if (output/'startup.json').exists() else {}
        save(output/'attempt.json',dict(status=startup.get('status','failed'),mode=mode,
                                        note='No successful result fabricated; inspect retained startup/diagnostic files and use a new run directory.'))
        raise
    np.savez_compressed(output/"trajectory.npz", **trajectory)
    result["trajectory_sha256"] = digest(output/"trajectory.npz")
    save(output/"result.json", result)
    save(output/"attempt.json", {"status": "finished", "task_success": result["task_success"]})
    print(f"{scene['id']}/{mode}: pickup={result['grasp_verified']} success={result['task_success']} "
          f"unwanted_contact={result['collision']['any_sampled_scene_violation']} "
          f"bottom={result['final_target_bottom_m']:.4f}m steps={result['steps']}", flush=True)
    return result


def prepare(folder, scenes_path, template_path):
    if folder.exists() and any(folder.iterdir()):
        raise ValueError("prepare requires a new/empty output directory")
    config, template = read(scenes_path), read(template_path)
    if config.get('observation') is not None:
        for scene in config['scenes']:scene['observation_config']=deepcopy(config['observation'])
    if config.get('model_faults') is not None:
        for scene in config['scenes']:scene['model_faults']=deepcopy(config['model_faults'])
    settings = dict(DEFAULTS, **config.get("settings", {}))
    geometry_options = dict(shape_aware=config.get("shape_aware_geometry",False),
                            counts=tuple(config.get("geometry_counts",(1,2,4))),
                            terminal_features=config.get("terminal_clearance_features",False))
    if set(settings) != set(DEFAULTS) or any(not np.isfinite(v) or v <= 0 for v in settings.values()):
        raise ValueError("settings must use known names and finite positive values")
    ids = [s["id"] for s in config["scenes"]]
    if not ids or len(set(ids)) != len(ids) or any(not re.fullmatch(r"[a-z0-9_]+", i) for i in ids):
        raise ValueError("need unique lowercase scene IDs")
    save(folder/"scenes.json", config)
    save(folder/"questions.json", template)
    save(folder/"settings.json", settings)
    source_hashes={name:digest(ROOT/name) for name in SOURCES}
    snapshot=folder/"source_snapshot";snapshot.mkdir()
    for name in SOURCES:
        shutil.copyfile(ROOT/name,snapshot/name)
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    robot, mj = demo.load_panda(), utils.mujoco
    frozen = ["scenes.json", "questions.json", "settings.json"]
    for scene in config["scenes"]:
        case = folder/scene["id"]
        # No clutter forces/barriers in this reference. Real finger/target
        # physics remain; it is labeled a prediction, not the evaluated off run.
        unobstructed = deepcopy(scene)
        unobstructed.pop('observation_config',None)
        unobstructed["objects"] = [o for o in scene["objects"] if o["id"] == scene["target_id"]]
        seed = pickup_request(unobstructed, settings, robot, template, **geometry_options)
        execute(case/"nominal_reference", unobstructed, settings, seed, "off", demo, utils)
        nominal_result = read(case/"nominal_reference/off/result.json")
        with np.load(case/"nominal_reference/off/trajectory.npz") as trace:
            references = {}
            for phase in ("approach", "pregrasp", "descend", "close", "lift"):
                indices = np.flatnonzero(trace["phase"] == phase)
                if not len(indices):
                    raise ValueError(f"{scene['id']}: nominal pickup did not reach {phase}; no model calls")
                references[phase] = trace["qpos"][indices[-1], :7].tolist()
        if not nominal_result["grasp_verified"]:
            raise ValueError(f"{scene['id']}: unobstructed physical pickup failed; no model calls")
        observed_scene=scene
        if 'observation_config' in scene:
            from perception import observe_initial
            observed_scene=observe_initial(scene,settings,robot,template,geometry_options,case/'initial_camera',scene['observation_config'])
        base = pickup_request(observed_scene, settings, robot, template, **geometry_options)
        if 'observation_config' in scene:base['state']['geometry_counts']=list(geometry_options['counts'])
        base["state"]["motion"]["phase_joint_reference_rad"] = references
        base["state"]["motion"]["reference_source"] = "Recorded original-controller pickup with clutter absent, same target and native finger/target physics. Reference is not a clutter-safe trajectory."
        # Measure the actual initial state in an independent native model.
        runtime = runtime_type(scene)(scene, settings, base, None, "all", case/"qualification")
        spec = runtime.scene_spec(mj, ROOT.parent/"frax", base["state"], [], [])
        spec.attach(runtime.robot_spec(mj, ROOT.parent/"frax"), prefix="", frame=spec.worldbody.add_frame())
        model = spec.compile(); data = mj.MjData(model)
        data.qpos[:7] = robot.default_configuration
        env = SimpleNamespace(model=model, data=data, real_time=False)
        runtime.initialize(env)
        runtime.robot, runtime.sphere_predict = robot, demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
        request = runtime.make_request(np.asarray(robot.default_configuration), np.zeros(7), 0.)
        if hasattr(runtime,'sensor'):runtime.sensor.close()
        save(case/"initial/request.json", request)
        save(case/"scene.json", scene)
        frozen += [f"{scene['id']}/initial/request.json", f"{scene['id']}/scene.json"]
    if any(digest(ROOT/name)!=expected for name,expected in source_hashes.items()):
        raise ValueError("source changed during preparation; retained drafts need a new run")
    save(folder/"manifest.json", {
        "task": "static_clutter_pickup", "model": "jev-1.13.0", "scene_ids": ids,
        "frozen_files": {name: digest(folder/name) for name in frozen},
        "source_sha256": source_hashes,
        "upstream_revisions": {name: subprocess.check_output(["git", "-C", str(ROOT.parent/name), "rev-parse", "HEAD"], text=True).strip() for name in ("frax", "oscbf", "cbfpy")},
        "versions": runtime_versions(),
        "budget_ledger": str(BUDGET_PATH), "budget_cap_usd": "7.50",
        "baseline_modes": config.get('baseline_modes',["off", "all", "rule"]), "model_modes": config.get('model_modes',["geometry", "once", "repeated"]),
        "notes": "Frozen before new model answers. Native articulated gripper + free target; seven-DOF FRAX contact dynamics remain approximate. Optional depth-derived clutter uses oracle instance labels and known target. No hardware or independently moving obstacles.",
    })


def run(folder, *, live=False, baselines=False):
    manifest = check_manifest(folder)
    settings, config = read(folder/"settings.json"), read(folder/"scenes.json")
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    budget = JevBudget(BUDGET_PATH)
    for scene in config["scenes"]:
        case = folder/scene["id"]
        request = read(case/"initial/request.json")
        modes = manifest.get("baseline_modes", ("off", "all")) if baselines else manifest.get("model_modes", ("once", "repeated"))
        if all((case/mode/"result.json").exists() for mode in modes):
            continue  # Completed data replay needs neither current physics nor API.
        for name, expected in manifest["source_sha256"].items():
            if digest(ROOT/name) != expected:
                raise ValueError(f"source changed since preparation: {name}; freeze a new run")
        if baselines:
            for mode in modes:
                execute(case, scene, settings, request, mode, demo, utils)
        else:
            if not live:
                raise ValueError("once/repeated comparison requires --live; gallery/status/replay make no requests")
            # Physics source edits after preparation need an explicitly new run;
            # changing only gallery code does not invalidate saved decisions.
            ask=FaultInjectedJev(budget,scene['model_faults'],case) if scene.get('model_faults') else budget
            initial = query_saved(case/"initial", request, ask=ask)
            for mode in modes:
                execute(case, scene, settings, request, mode, demo, utils, initial=initial, live=mode == "repeated", budget=ask)


def reuse_baselines(folder, source):
    """Explicitly reuse identical non-model runs across prompt comparisons.

    Full input state, settings, and producer source hashes must match. Reused
    results are labeled, never counted as fresh independent physics experiments.
    """
    current, previous = check_manifest(folder), check_manifest(source)
    if read(folder/"settings.json") != read(source/"settings.json"):
        raise ValueError("cannot reuse baselines with different task settings")
    for name, expected in current["source_sha256"].items():
        if previous["source_sha256"].get(name) != expected:
            raise ValueError(f"baseline implementation changed: {name}")
    for case in current["scene_ids"]:
        if read(folder/case/"initial/request.json")["state"] != read(source/case/"initial/request.json")["state"]:
            raise ValueError("baseline scene/geometry/motion context differs")
        for mode in current.get('baseline_modes',("off","all","rule")):
            origin, destination = source/case/mode, folder/case/mode
            if destination.exists():
                raise ValueError("destination already contains baseline data")
            result = read(origin/"result.json")
            if digest(origin/"trajectory.npz") != result["trajectory_sha256"]:
                raise ValueError("source baseline trajectory hash mismatch")
            shutil.copytree(origin, destination)
            result["baseline_reuse"] = {"source":str(origin.resolve()),"source_result_sha256":digest(origin/"result.json"),
                                       "note":"Identical-state/source non-model baseline replay, not a fresh independent rollout."}
            save(destination/"result.json",result)


def generate_layouts(folder, *, count=12, seed=31):
    """Offline fixture construction, not a learned selector or random benchmark.

    Search fixed cylinders that obstruct an unobstructed nominal path while
    leaving its start and grasp/lift configurations clear. Retain the search
    evidence; if none is found, keep an explicit clear-clutter control instead.
    No model answers are consulted and no expected labels enter model requests.
    """
    if folder.exists() and any(folder.iterdir()):
        raise ValueError("layout generation requires a new output directory")
    if not 1 <= count <= 24:
        raise ValueError("choose between one and 24 lightweight fixtures")
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    robot, mj = demo.load_panda(), utils.mujoco
    rng, template = np.random.default_rng(seed), read(ROOT/"prompts/pickup_selection.json")
    predict = demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
    ee_predict = demo.jax.jit(demo.jax.vmap(robot.ee_transform))
    records, scenes = [], []
    for number in range(count):
        xy = [float(rng.uniform(.45, .58)), float(rng.uniform(-.16, .16))]
        target = {"id": "target_object", "description": "Designated cylindrical pickup target.", "shape": "cylinder",
                  "size_m": [.044, .044, .12], "center_m": [*xy, .06], "mass_kg": .05}
        name = f"layout_{number:02d}"
        single = {"id": name, "target_id": target["id"], "objects": [target]}
        request = pickup_request(single, DEFAULTS, robot, template)
        execute(folder/name, single, DEFAULTS, request, "off", demo, utils)
        with np.load(folder/name/"off/trajectory.npz") as trace:
            result = read(folder/name/"off/result.json")
            if not result["grasp_verified"]:
                raise ValueError(f"{name}: unobstructed pickup fails; retain it and diagnose before model calls")
            indices = np.unique(np.r_[np.linspace(0, len(trace["qpos"])-1, 60, dtype=int),
                                      np.flatnonzero(trace["phase"] == "close")[0]])
            q = trace["qpos"][indices, :7]
            original = np.asarray(predict(q))
            tf = np.asarray(ee_predict(q))
            tool = np.einsum("tij,kj->tki", tf[:, :3, :3], trace["tool_centers_ee_m"]) + tf[:, None, :3, 3]
            path = np.concatenate([original, tool], axis=1)
            robot_radii = np.r_[np.asarray(robot.link_collision_data(q[0])[1]), trace["tool_radii_m"]]
            late = np.isin(trace["phase"][indices], ["close", "lift"])
            probes = []
            for _ in range(1200):
                x, y = rng.uniform(.25, .66), rng.uniform(-.25, .25)
                height = float(rng.choice([.14, .20, .26, .32, .38]))
                if np.linalg.norm(np.array([x,y])-xy) < .10:
                    continue
                centers = np.c_[np.full(4,x), np.full(4,y), height*(np.arange(4)+.5)/4]
                radius = np.sqrt(2*.025**2+(height/8)**2)
                d = np.linalg.norm(path[:, :, None]-centers[None, None], axis=-1)-robot_radii[None,:,None]-radius
                if d[0].min() <= .01 or d[late].min() <= .005:
                    continue
                probes.append((float(d.min()), [float(x),float(y),height/2], height))
            probes.sort(key=lambda item:item[0])
            runtime = PickupRuntime(single, DEFAULTS, request, None, "off", folder/name/"unused")
            spec = runtime.scene_spec(mj, ROOT.parent/"frax", request["state"], [], [])
            heights = (.14, .20, .26, .32, .38)
            for i, height in enumerate(heights):
                body = spec.worldbody.add_body(name=f"probe_body_{i}", mocap=True, pos=[5,5,5])
                body.add_geom(name=f"probe_{i}", type=mj.mjtGeom.mjGEOM_CYLINDER, size=[.025,height/2,0])
            spec.attach(runtime.robot_spec(mj, ROOT.parent/"frax"), prefix="", frame=spec.worldbody.add_frame())
            model = spec.compile(); data = mj.MjData(model)
            probe_geoms = {height:model.geom(f"probe_{i}").id for i,height in enumerate(heights)}
            probe_mocaps = [model.body(f"probe_body_{i}").mocapid[0] for i in range(len(heights))]
            target_geom = model.geom("scene/target_object").id
            robot_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] != 0
                           and g not in {target_geom, *probe_geoms.values()} and model.geom_bodyid[g] != model.body("target").id
                           and (model.geom_contype[g] or model.geom_conaffinity[g])]
            best, best_native = None, float("inf")
            native_attempts = []
            for approximate, center, height in probes[:40]:
                # Prescribed evaluation-only bodies avoid mutating static model
                # geometry/cached world poses. These are candidate placements,
                # not moving obstacles in an evaluated manipulation scene.
                geom = probe_geoms[height]
                data.mocap_pos[probe_mocaps] = [5,5,5]
                data.mocap_pos[model.body_mocapid[model.geom_bodyid[geom]]] = center
                values = []
                for index in indices:
                    data.qpos[:], data.qvel[:] = trace["qpos"][index], trace["qvel"][index]
                    mj.mj_forward(model, data)
                    if not np.allclose(data.geom_xpos[geom], center, atol=1e-12, rtol=0):
                        raise ValueError("native probe pose differs from candidate geometry")
                    values.append(min(mj.mj_geomDistance(model,data,geom,g,1000.,None) for g in robot_geoms))
                values = np.array(values)
                eligible = values[0] > .01 and values[late].min() > .01
                native_attempts.append({"center_m":center,"height_m":height,"initial_m":float(values[0]),"grasp_lift_min_m":float(values[late].min()),"path_min_m":float(values.min()),"eligible":bool(eligible)})
                if eligible and values.min() < best_native:
                    best, best_native = (center,height), float(values.min())
                if eligible and values.min() < -.002:
                    break
        if best is None:
            best = ([.34, -.25 if xy[1] >= 0 else .25, .12], .24)
        bottle = {"id":"bottle_a","description":"Stationary tabletop cylinder.","shape":"cylinder","size_m":[.05,.05,best[1]],"center_m":best[0]}
        box = {"id":"box_b","description":"Stationary tabletop box.","shape":"box","size_m":[.08,.08,.08],"center_m":[.65, -.28 if xy[1] >= 0 else .28,.04]}
        scenes.append({"id":name,"target_id":target["id"],"objects":[target,bottle,box]})
        records.append({"id":name,"seed":seed,"approximate_qualified_candidates":len(probes),"native_attempts":native_attempts,
                        "selected_native_path_min_m":None if best_native == float("inf") else best_native,
                        "clear_control_fallback":not bool(native_attempts and any(a['eligible'] for a in native_attempts))})
        print(f"Generated {name}: native nominal path min={best_native:.4f} m", flush=True)
    save(folder/"generated_scenes.json", {"settings":{},"shape_aware_geometry":True,"scenes":scenes})
    save(folder/"generation_evidence.json", {"seed":seed,"count":count,"records":records,
         "scope":"Designed static fixtures, not a random population benchmark. Geometry search uses no model answers. Labels/search results are not sent to Jev."})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "baselines", "run", "gallery", "status", "layouts"))
    parser.add_argument("--output", type=Path, default=ROOT/"artifacts/pickup/showcase_v1")
    parser.add_argument("--scenes", type=Path, default=ROOT/"examples/pickup_scenes.json")
    parser.add_argument("--questions", type=Path, default=ROOT/"prompts/pickup_selection.json")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--count", type=int, default=12, help="number of offline generated layout variations")
    parser.add_argument("--seed", type=int, default=31, help="offline layout generator seed")
    args = parser.parse_args(argv)
    if args.action == "layouts":
        generate_layouts(args.output, count=args.count, seed=args.seed)
    elif args.action == "prepare":
        prepare(args.output, args.scenes, args.questions)
    elif args.action in ("baselines", "run"):
        run(args.output, live=args.live, baselines=args.action == "baselines")
    elif args.action == "gallery":
        from pickup_gallery import render_gallery
        render_gallery(args.output)
    else:
        check_manifest(args.output)
        for file in sorted(args.output.glob("*/*/result.json")):
            result = read(file)
            print(file.parent.parent.name, result["mode"], "success", result["task_success"], "pickup", result["grasp_verified"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
