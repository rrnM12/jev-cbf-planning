"""Offline audit, measurements and summaries of saved pickup experiments.

No model calls. ``replay`` runs new physics with recorded choices and reports
the context mismatch explicitly. ``profile`` measures
warmed CBF calls on a recorded state, not a real-time control-loop guarantee.
"""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time
from copy import deepcopy

import numpy as np

from inspect_selection import ROOT, decode_response
from pickup import DEFAULTS, PickupRuntime, pickup_collision_diagnostics
from run_static import load_upstream, run_case
from run_suite import check_manifest, digest, read, save
from runtime_cbf import guarded_cbf, sphere_runtime_config


def results(folder):
    check_manifest(folder)
    return [(p, read(p)) for p in sorted(folder.glob("*/*/result.json"))]


def footprint(request, trace, *, samples=100_000, seed=0):
    """Scene-union geometry cost from ACTUALLY applied clutter primitives.

    Excludes the interaction target and robot-radius expansion. Omitted real
    clutter is reported as unmodeled volume, not automatically an unsafe error.
    Common points support mode comparisons; standard errors are MC noise only.
    """
    objects = request["state"]["objects"]
    clutter = [(i,o) for i,o in enumerate(objects) if o["id"] != request["state"]["target_id"]]
    if not clutter:
        return {"mean_excess_m3":0.,"mean_unmodeled_clutter_m3":0.,"mc_samples":samples}
    all_centers, all_radii, columns = [], [], []
    for i,obj in clutter:
        columns.extend(range(i*4,i*4+4))
        for candidate in obj["candidates"]:
            all_centers.extend(np.asarray(candidate["centers_object_m"])+obj["center_m"])
            all_radii.extend(candidate["radii_m"])
    centers, radii = np.asarray(all_centers), np.asarray(all_radii)[:,None]
    lower,upper=(centers-radii).min(axis=0),(centers+radii).max(axis=0)
    volume=float(np.prod(upper-lower))
    points=np.random.default_rng(seed).uniform(lower,upper,size=(samples,3))
    truth=np.zeros(samples,dtype=bool)
    for _,obj in clutter:
        relative=points-np.asarray(obj["center_m"]);half=np.asarray(obj["size_m"])/2
        truth |= (np.all(abs(relative)<=half,axis=1) if obj["shape"]=="box" else
                  (np.linalg.norm(relative[:,:2],axis=1)<=half[0]) & (abs(relative[:,2])<=half[2]))
    enabled=np.any(trace["cover_enabled"][:-1],axis=1)[:,columns]
    positions=trace["cover_centers_m"][:-1,columns]
    sizes=trace["cover_radii_m"][:-1,columns]
    signatures=np.c_[enabled.astype(float),sizes,positions.reshape(len(positions),-1)]
    _,indices,counts=np.unique(signatures,axis=0,return_index=True,return_counts=True)
    excess,missing=np.zeros(samples),np.zeros(samples)
    for index,count in zip(indices,counts,strict=True):
        covered=np.zeros(samples,dtype=bool)
        for center,radius in zip(positions[index][enabled[index]],sizes[index][enabled[index]],strict=True):
            covered |= np.sum((points-center)**2,axis=1)<=radius**2
        weight=count/len(positions)
        excess += weight*(covered & ~truth)
        missing += weight*(truth & ~covered)
    return {"mean_excess_m3":float(excess.mean()*volume),
            "excess_mc_standard_error_m3":float(excess.std(ddof=1)*volume/np.sqrt(samples)),
            "mean_unmodeled_clutter_m3":float(missing.mean()*volume),
            "unmodeled_mc_standard_error_m3":float(missing.std(ddof=1)*volume/np.sqrt(samples)),
            "mc_samples":samples,"seed":seed,"unique_applied_geometry_sets":len(indices),
            "scope":"Time-averaged spatial UNION of applied clutter covers, minus actual clutter union. Excludes target/robot-radius inflation; unmodeled clutter may be intentionally irrelevant. MC errors are not perception uncertainty or safety probabilities."}


def report(folder):
    rows, methods = [], {}
    for path, result in results(folder):
        row = {"scene":path.parent.parent.name,"mode":result["mode"],"pickup":result["grasp_verified"],
               "safe_completion":result["task_success"],"unwanted_contact":result["collision"]["any_sampled_scene_violation"],
               "negative_modeled_barriers":result["any_sampled_negative_h2"],
               "mean_enabled_pairs":result["enabled_scene_pair_counts"]["mean"],
               "allocated_pairs":result["allocated_scene_pair_count"],"duration_s":result["simulation_seconds"],
               "reused_baseline":bool(result.get("baseline_reuse"))}
        request=read(path.parent.parent/"initial/request.json")
        with np.load(path.parent/"trajectory.npz") as trace:
            row["geometry_footprint"]=footprint(request,trace,seed=int(hashlib.sha256(row["scene"].encode()).hexdigest()[:8],16))
        rows.append(row)
    for mode in sorted({r["mode"] for r in rows}):
        group = [r for r in rows if r["mode"] == mode]
        methods[mode] = {"cases":len(group),"pickups":sum(r["pickup"] for r in group),
                         "safe_completions":sum(r["safe_completion"] for r in group),
                         "contact_cases":sum(r["unwanted_contact"] for r in group),
                         "negative_barrier_cases":sum(r["negative_modeled_barriers"] for r in group),
                         "mean_enabled_pairs_equal_layout_weight":float(np.mean([r["mean_enabled_pairs"] for r in group])),
                         "mean_duration_s":float(np.mean([r["duration_s"] for r in group])),
                         "mean_clutter_excess_m3":float(np.mean([r["geometry_footprint"]["mean_excess_m3"] for r in group])),
                         "mean_unmodeled_clutter_m3":float(np.mean([r["geometry_footprint"]["mean_unmodeled_clutter_m3"] for r in group]))}
    output = {"methods":methods,"rows":rows,
              "scope":"Designed static-clutter pickup fixtures, not a statistical population benchmark. Enabled scene pairs include 64 mandatory arm-target pairs in constrained modes; allocated pairs stay fixed. Omission is not automatically an error; native contact checks evaluate every object. No hardware/perception or formal contact/continuous-time safety proof."}
    save(folder/"research_report.json", output)
    print(json.dumps(methods, indent=2))
    return output


def audit(folder):
    manifest = check_manifest(folder)
    config, settings = read(folder/"scenes.json"), read(folder/"settings.json")
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    mj, checks, queries = utils.mujoco, [], []
    for scene in config["scenes"]:
        case = folder/scene["id"]
        request = read(case/"initial/request.json")
        runtime = PickupRuntime(scene,settings,request,None,"off",case/"audit_unused")
        spec = runtime.scene_spec(mj, ROOT.parent/"frax", request["state"], [], [])
        spec.attach(runtime.robot_spec(mj,ROOT.parent/"frax"),prefix="",frame=spec.worldbody.add_frame())
        model = spec.compile()
        assert model.body("pickup_object").mocapid[0] < 0
        assert model.joint("pickup_free").type[0] == mj.mjtJoint.mjJNT_FREE
        assert all(t != mj.mjtEq.mjEQ_WELD for t in model.eq_type)
        initial_state = None
        for mode in ("off","all","rule","geometry","once","repeated"):
            path = case/mode/"result.json"
            if not path.exists():
                continue
            result = read(path)
            source = path.parent/"trajectory.npz"
            with np.load(source) as trace:
                check = {"scene":scene["id"],"mode":mode,
                         "trajectory_hash":digest(source)==result["trajectory_sha256"],
                         "finite_state_and_control":all(np.isfinite(trace[k]).all() for k in ("qpos","qvel","torques_Nm","cover_centers_m","cover_radii_m")),
                         "length":len(trace["qpos"])==result["steps"]+1,
                         "free_body_trace":bool(np.array_equal(trace["qpos"][:,9:12],trace["target_center_m"])),
                         "maximum_native_finger_soft_limit_overshoot_m":float(np.maximum(
                             np.maximum(-trace["qpos"][:,7:9],trace["qpos"][:,7:9]-.04),0).max())}
                start = np.r_[trace["qpos"][0],trace["qvel"][0]]
                if initial_state is None:
                    initial_state = start.copy()
                check["identical_initial_state"] = bool(np.array_equal(initial_state,start))
                for i, obj in enumerate(scene["objects"]):
                    if obj["id"] != scene["target_id"]:
                        check[f"static_{obj['id']}"] = bool(np.all(trace["object_positions_m"][:,i] == obj["center_m"]))
                native, distances, contact = pickup_collision_diagnostics(mj,model,trace["qpos"],trace["qvel"],scene["objects"],scene["target_id"])
                check["native_contact_result_reproduced"] = native["any_sampled_scene_violation"] == result["collision"]["any_sampled_scene_violation"]
                check["native_distances_reproduced"] = bool(np.allclose(distances,trace["object_distance_m"],atol=1e-10,rtol=0))
                check["bilateral_contacts_reproduced"] = bool(np.array_equal(contact["bilateral_finger_contact"],trace["bilateral_finger_contact"]))
                if result["grasp_verified"]:
                    window = trace["time_s"] >= trace["time_s"][-1]-settings["pickup_confirmation_s"]-1e-9
                    check["confirmation_window"] = bool(np.all(contact["bilateral_finger_contact"][window]) and
                        np.all(trace["target_bottom_m"][window] >= settings["pickup_bottom_lift_m"]) and
                        trace["time_s"][-1]-trace["time_s"][np.flatnonzero(window)[0]] >= settings["pickup_confirmation_s"]-1e-9)
                if mode == "repeated":
                    for p in [case/"initial/request.json",*sorted(case.glob("queries/*/request.json"))]:
                        snapshot = read(p)["state"]
                        at = snapshot.get("snapshot_time_s",0.)
                        index = int(np.argmin(abs(trace["time_s"]-at)))
                        context = snapshot["robot_context"]
                        queries.append({"file":str(p.relative_to(folder)),"qv_matches_recorded_snapshot":bool(
                            np.array_equal(context["initial_joint_positions_rad"],trace["qpos"][index,:7]) and
                            np.array_equal(context["initial_joint_velocities_rad_s"],trace["qvel"][index,:7]))})
                checks.append(check)
                save(path.parent/"native_audit.json",{"source_trajectory_sha256":digest(source),"collision":native,"checks":check})
                print("Audited",scene["id"],mode,flush=True)
    source_snapshot = folder/"source_snapshot"
    source_matches = {name:digest(source_snapshot/name)==expected if (source_snapshot/name).exists() else None
                      for name,expected in manifest["source_sha256"].items()}
    required_modes = list(manifest.get("baseline_modes",("off","all")))+list(manifest.get("model_modes",("once","repeated")))
    complete = all((folder/case/mode/"result.json").exists() for case in manifest["scene_ids"] for mode in required_modes)
    output = {"rollouts":checks,"query_snapshots":queries,"preserved_source_hash_matches":source_matches,
              "all_required_methods_complete":complete,
              "all_checks_pass":complete and all(all(v for v in c.values() if isinstance(v,bool)) for c in checks+queries) and all(v is not False for v in source_matches.values()),
              "scope":"Independent native contact re-evaluation at recorded states, no new rollout. Finger envelopes assume specified 0–4 cm travel; native soft-limit overshoot is reported, not a proof of that assumption. Not a continuous-time safety or contact-dynamics certificate."}
    save(folder/"verification.json",output)
    if not output["all_checks_pass"]:
        raise ValueError("audit discrepancy retained in verification.json")
    return output


def profile(folder):
    demo, _ = load_upstream(ROOT.parent/"frax",ROOT.parent/"cbfpy")
    robot = demo.load_panda()
    first = read(folder/"manifest.json")["scene_ids"][0]
    scene=read(folder/first/"scene.json")
    target_index=next(i for i,o in enumerate(scene["objects"]) if o["id"]==scene["target_id"])
    others=[i for i in range(len(scene["objects"])) if i!=target_index]
    with np.load(folder/first/"all/trajectory.npz") as trace:
        index = int(np.argmin(trace["scene_h2_m"][:-1]))
        tool = (trace["tool_centers_ee_m"],trace["tool_radii_m"])
        capacity = trace["cover_radii_m"].shape[1]
        config = sphere_runtime_config(demo,robot,capacity,pairwise=True,tool_spheres=tool)
        cbf = guarded_cbf(demo,config,scene_guard=read(folder/"settings.json")["scene_guard_m_s2"])
        z = np.r_[trace["qpos"][index,:7],trace["qvel"][index,:7],trace["time_s"][index]]
        reference = trace["torques_Nm"][index]
        base = (trace["cover_centers_m"][index],trace["cover_radii_m"][index],np.zeros((capacity,3)),trace["cover_enabled"][index],z[-1])
        records = []
        for label,indices in (("target_only",[target_index]),("target_and_one_clutter",[target_index,*others[:1]]),("all",list(range(len(scene["objects"]))))):
            active = base[3].copy()
            for obj in range(capacity//4):
                if obj not in indices:
                    active[:,4*obj:4*obj+4] = False
            args = (*base[:3],active,base[4])
            cbf.safety_filter(z,reference,*args).block_until_ready()
            timings=[]
            for _ in range(300):
                started=time.perf_counter();cbf.safety_filter(z,reference,*args).block_until_ready()
                timings.append((time.perf_counter()-started)*1000)
            records.append({"label":label,"enabled_scene_pairs":int(active.sum()),"allocated_scene_pairs":config.scene_pair_count,
                            "median_ms":float(np.median(timings[10:])),"p95_ms":float(np.percentile(timings[10:],95))})
    output = {"warm_filter_calls":records,"scope":"One recorded near-boundary state and already-filtered reference torque; not an end-to-end controller, original nominal-torque workload, throughput/real-time certification, or proof that dropping rows saves runtime. All masks retain identical allocated QP shape."}
    save(folder/"cpu_profile.json",output)
    print(json.dumps(output,indent=2))
    return output


def replay(folder, destination, case_id, *, task_protocol="current"):
    """Controlled decoder-correction replay, not a fresh Jev evaluation.

    Apply every recorded proposal at its recorded apply time, including raw
    replies previously rejected by the decoder. Never edit the original report
    or re-query. Current RD2 admission still decides whether to install a choice.
    """
    if destination.exists() and any(destination.iterdir()):
        raise ValueError("replay destination must be new/empty")
    manifest=check_manifest(folder)
    case = folder/case_id
    scene, settings = read(case/"scene.json"), dict(DEFAULTS,**read(folder/"settings.json"))
    base_class=PickupRuntime
    if task_protocol == "archived":
        archive=folder/"source_snapshot/pickup.py"
        if digest(archive) != manifest["source_sha256"]["pickup.py"]:
            raise ValueError("archived task source hash mismatch")
        spec=importlib.util.spec_from_file_location("archived_pickup_task",archive)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        base_class=module.PickupRuntime
        settings=read(folder/"settings.json")
    initial = read(case/"initial/selection.json")
    events = read(case/"repeated/result.json")["selection_events"]
    scheduled = [e for e in events if e.get("kind")=="selection" and e.get("source","").startswith("queries/")]

    def redecode(path):
        original = read(path)
        report = deepcopy(original)
        if "response" in report:
            try:
                report["proposals"] = decode_response(report["request"],report["response"])
                if "error" in report:
                    report["original_decoder_error"] = report.pop("error")
                report["mode"] = "recorded_raw_response_redecoded_for_diagnostic"
            except (ValueError,KeyError,TypeError):
                pass
        return report

    class RecordedRuntime(base_class):
        def __init__(self):
            super().__init__(scene,settings,initial["request"],redecode(case/"initial/selection.json"),"repeated",destination)
            self.replay_index = 0

        def before_control(self, env):
            super().before_control(env)
            at = float(env.data.time)
            while self.replay_index < len(scheduled) and at+1e-9 >= scheduled[self.replay_index]["applied_at_s"]:
                event = scheduled[self.replay_index]
                self.install(redecode(case/event["source"]),env.data.qpos[:7],env.data.qvel[:7],at,source=event["source"])
                self.events[-1]["replay_note"] = "Recorded raw response at original apply time; original file unchanged; current admission gate retained."
                self.replay_index += 1

    demo, utils = load_upstream(ROOT.parent/"frax",ROOT.parent/"cbfpy")
    runtime = RecordedRuntime()
    result, trace = run_case(demo,utils,{"request":initial["request"]},"off",frax_dir=ROOT.parent/"frax",
                             steps=round(settings["duration_s"]/settings["control_dt_s"]),control_dt=settings["control_dt_s"],
                             model_profile="frax_gripper",scene_guard=settings["scene_guard_m_s2"],runtime=runtime)
    destination.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(destination/"trajectory.npz",**trace)
    result.update(mode="decoder_correction_replay",selection_source="Recorded raw Jev answers at original apply times, revalidated; no model calls",
                  original_case=str(case.resolve()),trajectory_sha256=digest(destination/"trajectory.npz"),
                  task_protocol=task_protocol,
                  replay_scope="Diagnostic physics replay, NOT fresh model performance. Inputs after the first changed control may no longer match original query snapshots. The source data/failed reports are retained.")
    with np.load(case/"repeated/trajectory.npz") as original:
        first = scheduled[0]["applied_at_s"]
        count = int(np.searchsorted(trace["time_s"],first))
        result["max_q_difference_before_first_changed_update"] = float(np.max(abs(trace["qpos"][:count]-original["qpos"][:count])))
    save(destination/"result.json",result)
    print({k:result[k] for k in ("task_success","grasp_verified","max_q_difference_before_first_changed_update","simulation_seconds")})
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=("report","audit","profile","replay"))
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts/pickup/heldout_v1")
    parser.add_argument("--destination",type=Path,help="new directory for a diagnostic replay")
    parser.add_argument("--case",default="layout_02")
    parser.add_argument("--task-protocol",choices=("current","archived"),default="current",help="archived task sequence with current decoder, or the current task sequence")
    args=parser.parse_args(argv)
    if args.action == "replay":
        if args.destination is None:
            parser.error("replay needs a new --destination")
        replay(args.output,args.destination,args.case,task_protocol=args.task_protocol)
    else:
        globals()[args.action](args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
