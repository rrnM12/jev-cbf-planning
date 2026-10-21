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

ANALYSIS_SOURCE_BYTES=Path(__file__).read_bytes()
ANALYSIS_SOURCE_SHA256=hashlib.sha256(ANALYSIS_SOURCE_BYTES).hexdigest()


def results(folder):
    check_manifest(folder)
    return [(p, read(p)) for p in sorted(folder.glob("*/*/result.json"))]


def footprint(request, trace, *, samples=100_000, seed=0, truth_objects=None):
    """Scene-union geometry cost from ACTUALLY applied clutter primitives.

    Excludes the interaction target and robot-radius expansion. Omitted real
    clutter is reported as unmodeled volume, not automatically an unsafe error.
    Common points support mode comparisons; standard errors are MC noise only.
    """
    objects = request["state"]["objects"]
    physical={o['id']:o for o in (objects if truth_objects is None else truth_objects)}
    clutter = [(i,o) for i,o in enumerate(objects) if o["id"] != request["state"]["target_id"]]
    if not clutter:
        return {"mean_excess_m3":0.,"mean_unmodeled_clutter_m3":0.,"mc_samples":samples}
    all_centers, all_radii, columns = [], [], []
    object_capacity=trace["cover_radii_m"].shape[1]//len(objects)
    for i,obj in clutter:
        columns.extend(range(i*object_capacity,(i+1)*object_capacity))
        for candidate in obj["candidates"]:
            all_centers.extend(np.asarray(candidate["centers_object_m"])+obj["center_m"])
            all_radii.extend(candidate["radii_m"])
    centers, radii = np.asarray(all_centers), np.asarray(all_radii)[:,None]
    lower,upper=(centers-radii).min(axis=0),(centers+radii).max(axis=0)
    # Observation fits may miss real surfaces or grow later. The evaluation
    # domain must enclose actual applied covers AND independent physical truth.
    active=np.any(trace['cover_enabled'],axis=1)[:,columns]
    applied=trace['cover_centers_m'][:,columns][active]
    applied_radii=trace['cover_radii_m'][:,columns][active,None]
    if len(applied):
        lower=np.minimum(lower,(applied-applied_radii).min(axis=0))
        upper=np.maximum(upper,(applied+applied_radii).max(axis=0))
    for _,observed in clutter:
        obj=physical[observed['id']];half=np.array(obj['size_m'])/2
        lower=np.minimum(lower,np.array(obj['center_m'])-half)
        upper=np.maximum(upper,np.array(obj['center_m'])+half)
    volume=float(np.prod(upper-lower))
    points=np.random.default_rng(seed).uniform(lower,upper,size=(samples,3))
    truth=np.zeros(samples,dtype=bool);object_truth=[]
    for _,observed in clutter:
        obj=physical[observed['id']]
        relative=points-np.asarray(obj["center_m"]);half=np.asarray(obj["size_m"])/2
        mask=(np.all(abs(relative)<=half,axis=1) if obj["shape"]=="box" else
              (np.linalg.norm(relative[:,:2],axis=1)<=half[0]) & (abs(relative[:,2])<=half[2]))
        truth |= mask;object_truth.append(mask)
    enabled=np.any(trace["cover_enabled"][:-1],axis=1)[:,columns]
    positions=trace["cover_centers_m"][:-1,columns]
    sizes=trace["cover_radii_m"][:-1,columns]
    signatures=np.c_[enabled.astype(float),sizes,positions.reshape(len(positions),-1)]
    _,indices,counts=np.unique(signatures,axis=0,return_index=True,return_counts=True)
    excess,missing,constrained_missing=np.zeros(samples),np.zeros(samples),np.zeros(samples)
    for index,count in zip(indices,counts,strict=True):
        covered=np.zeros(samples,dtype=bool)
        for center,radius in zip(positions[index][enabled[index]],sizes[index][enabled[index]],strict=True):
            covered |= np.sum((points-center)**2,axis=1)<=radius**2
        weight=count/len(positions)
        excess += weight*(covered & ~truth)
        missing += weight*(truth & ~covered)
        represented=np.zeros(samples,dtype=bool)
        for j,mask in enumerate(object_truth):
            if enabled[index,j*object_capacity:(j+1)*object_capacity].any():represented |= mask
        constrained_missing += weight*(represented & ~covered)
    return {"mean_excess_m3":float(excess.mean()*volume),
            "excess_mc_standard_error_m3":float(excess.std(ddof=1)*volume/np.sqrt(samples)),
            "mean_unmodeled_clutter_m3":float(missing.mean()*volume),
            "mean_uncovered_constrained_clutter_m3":float(constrained_missing.mean()*volume),
            "uncovered_constrained_mc_standard_error_m3":float(constrained_missing.std(ddof=1)*volume/np.sqrt(samples)),
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
            row["geometry_footprint"]=footprint(request,trace,seed=int(hashlib.sha256(row["scene"].encode()).hexdigest()[:8],16),
                                               truth_objects=read(path.parent.parent/'scene.json')['objects'])
            row['initial_scene_barrier_m']=float(trace['scene_h2_m'][0])
            row['initially_outside_modeled_scene_safe_set']=bool(trace['scene_h2_m'][0]<0)
            row['later_scene_crossing_from_nonnegative_start']=bool(trace['scene_h2_m'][0]>=0 and np.any(trace['scene_h2_m']<0))
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
        methods[mode]['mean_uncovered_constrained_clutter_m3']=float(np.mean([r['geometry_footprint'].get('mean_uncovered_constrained_clutter_m3',0.) for r in group]))
        methods[mode]['initial_model_inadmissible_cases']=sum(r['initially_outside_modeled_scene_safe_set'] for r in group)
        methods[mode]['later_scene_crossing_cases']=sum(r['later_scene_crossing_from_nonnegative_start'] for r in group)
    output = {"methods":methods,"rows":rows,
              'analysis_source_sha256':ANALYSIS_SOURCE_SHA256,
              "scope":"Designed static-clutter pickup fixtures, not a statistical population benchmark. Enabled scene pairs include 64 mandatory arm-target pairs in constrained modes; allocated pairs stay fixed. Omission is not automatically an error; native contact/volume evaluation uses independent physical scene truth, including in camera-derived clutter runs. Camera conditions keep oracle instance labels and a known target. No hardware or formal contact/continuous-time safety proof."}
    save(folder/"research_report.json", output)
    archive=folder/'analysis_snapshot'/f'{ANALYSIS_SOURCE_SHA256}.py'
    archive.parent.mkdir(exist_ok=True)
    if not archive.exists():archive.write_bytes(ANALYSIS_SOURCE_BYTES)
    print(json.dumps(methods, indent=2))
    return output


def compare(folder):
    """Compare frozen conditions; classification-only repeats are NOT rerolls."""
    protocol=read(folder/'protocol.json');conditions={};repeatability=[]
    for source in sorted((folder/'inputs').glob('*.json')):
        name=source.stem;run=folder/name
        if not (run/'manifest.json').exists():
            conditions[name]={'complete':False};continue
        manifest=check_manifest(run)
        modes=manifest.get('baseline_modes',[])+manifest.get('model_modes',[])
        complete=all((run/c/m/'result.json').exists() for c in manifest['scene_ids'] for m in modes)
        summary=read(run/'research_report.json')['methods'] if (run/'research_report.json').exists() else {}
        conditions[name]={'complete':complete,'methods':summary,'scene_count':len(manifest['scene_ids'])}
        repeats_complete=True
        for case in manifest['scene_ids']:
            files=[run/case/'initial/selection.json',*sorted((run/case/'classification_repeats').glob('*/selection.json'))]
            responses=[read(p) for p in files if p.exists()]
            repeats_complete &= len(responses)>=protocol.get('classification_repetitions',1)
            valid=[r for r in responses if 'error' not in r and 'response' in r]
            if not responses:continue
            keys=list(valid[0]['request']['questions']) if valid else []
            changes={key:len({r['response']['answers'][key]['choice'] for r in valid})>1 for key in keys}
            policies=[]
            for response in valid:
                target=response['request']['state'].get('target_id')
                proposals=decode_response(response['request'],response['response'])
                policies.append(tuple((p['object_id'],'retain_previous' if p['role']=='review' else p['candidate_id'])
                                      for p in proposals if p['object_id']!=target))
            repeatability.append({'condition':name,'case':case,'attempts':len(responses),'valid':len(valid),
                 'questions_with_disagreement':[k for k,v in changes.items() if v],
                 'proposed_clutter_policy_disagreement':len(set(policies))>1,
                 'control_answer_is_first_only':True})
        conditions[name]['classification_repeats_complete']=bool(repeats_complete)
    output={'protocol_sha256':digest(folder/'protocol.json'),'conditions':conditions,'classification_repeatability':repeatability,
            'all_conditions_complete':all(c['complete'] and c.get('classification_repeats_complete',False) for c in conditions.values()),
            'scope':'Frozen designed-condition comparison, not a population or causal significance claim. Primary task success is unchanged. Additional initial requests are predeclared classification-only repeats, never substituted into a physical rollout. Independent physical truth underlies coverage/collision metrics.'}
    save(folder/'comparison.json',output)
    print(json.dumps({name:dict(complete=c['complete'],safe={m:r['safe_completions'] for m,r in c.get('methods',{}).items()}) for name,c in conditions.items()},indent=2))
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
        physical_state=deepcopy(request['state']);physical_state['objects']=deepcopy(scene['objects'])
        spec = runtime.scene_spec(mj, ROOT.parent/"frax", physical_state, [], [])
        spec.attach(runtime.robot_spec(mj,ROOT.parent/"frax"),prefix="",frame=spec.worldbody.add_frame())
        model = spec.compile()
        assert model.body("pickup_object").mocapid[0] < 0
        assert model.joint("pickup_free").type[0] == mj.mjtJoint.mjJNT_FREE
        assert all(t != mj.mjtEq.mjEQ_WELD for t in model.eq_type)
        initial_state = None
        for mode in ("off","all","rule","geometry","once","repeated","all_fine"):
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
    source=folder/first/('all_fine' if (folder/first/'all_fine/trajectory.npz').exists() else 'all')/'trajectory.npz'
    with np.load(source) as trace:
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
            for obj in range(len(scene['objects'])):
                if obj not in indices:
                    slots=capacity//len(scene["objects"])
                    active[:,slots*obj:slots*(obj+1)] = False
            args = (*base[:3],active,base[4])
            cbf.safety_filter(z,reference,*args).block_until_ready()
            timings=[]
            for _ in range(300):
                started=time.perf_counter();cbf.safety_filter(z,reference,*args).block_until_ready()
                timings.append((time.perf_counter()-started)*1000)
            records.append({"label":label,"enabled_scene_pairs":int(active.sum()),"allocated_scene_pairs":config.scene_pair_count,
                            "median_ms":float(np.median(timings[10:])),"p95_ms":float(np.percentile(timings[10:],95))})
    output = {"source_trajectory":str(source),"source_trajectory_sha256":digest(source),
              "warm_filter_calls":records,"scope":"One recorded near-boundary state and already-filtered reference torque; not an end-to-end controller, original nominal-torque workload, throughput/real-time certification, or proof that dropping rows saves runtime. All masks retain identical allocated QP shape. Different candidate families may change geometry and allocation, so cross-family timings do not isolate a single cause."}
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


def validate_startup(source, destination):
    """Targeted correction checks on retained failures, NOT another unseen study.

    Reuses frozen scene/request inputs; no Jev answers or provider calls. Unsupported
    inputs produce startup diagnostics, not fabricated zero-control trajectories.
    """
    import shutil
    from run_pickup import SOURCES, execute
    if destination.exists() and any(destination.iterdir()):
        raise ValueError('startup validation requires a new/empty directory')
    plan=(('small_cover', 'unseen_selection_v1/small_terminal', 'unseen_00', 'all_fine', False),
          ('rich_cover', 'unseen_selection_v1/rich_terminal', 'unseen_00', 'all_fine', False),
          ('clean_depth', 'camera_control_v1/two_views', 'layout_02', 'all_fine', False),
          ('noisy_depth_02', 'camera_control_v1/sensor_noise', 'layout_02', 'all_fine', True),
          ('noisy_depth_03', 'camera_control_v1/sensor_noise', 'layout_03', 'all_fine', True),
          ('fixed_four', 'unseen_selection_v1/small_terminal', 'unseen_00', 'all', True))
    hashes={name:digest(ROOT/name) for name in SOURCES}
    save(destination/'protocol.json',{'cases':plan,'source_sha256':hashes,'analysis_source_sha256':ANALYSIS_SOURCE_SHA256,
         'scope':'Offline startup-correction diagnostics on already-seen retained inputs, not new model-performance/unseen evidence. Same full physical task; no margin shrinking, object omission or provider calls.'})
    shutil.copyfile(Path(__file__),destination/'analysis_source.py')
    demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');rows=[]
    for name,relative,case_id,mode,expected_rejection in plan:
        origin=source/relative;folder=destination/name;case=folder/case_id
        original=check_manifest(origin);scene=read(origin/case_id/'scene.json');settings=read(origin/'settings.json')
        request=read(origin/case_id/'initial/request.json')
        for file,value in (('scenes.json',{'scenes':[scene]}),('settings.json',settings),('questions.json',read(origin/'questions.json')),
                           (f'{case_id}/scene.json',scene),(f'{case_id}/initial/request.json',request)):
            save(folder/file,value)
        snapshot=folder/'source_snapshot';snapshot.mkdir()
        for file in SOURCES:shutil.copyfile(ROOT/file,snapshot/file)
        if (origin/case_id/'initial_camera').exists():shutil.copytree(origin/case_id/'initial_camera',case/'initial_camera')
        frozen=('scenes.json','settings.json','questions.json',f'{case_id}/scene.json',f'{case_id}/initial/request.json')
        save(folder/'manifest.json',dict(task='static_clutter_pickup_startup_diagnostic',scene_ids=[case_id],
             frozen_files={file:digest(folder/file) for file in frozen},source_sha256=hashes,
             baseline_modes=[mode],model_modes=[],upstream_revisions=original['upstream_revisions'],
             diagnostic_input_source=str(origin/case_id)))
        if any(digest(ROOT/file)!=expected for file,expected in hashes.items()):raise ValueError('source changed during validation')
        try:
            result=execute(case,scene,settings,request,mode,demo,utils)
            audit(folder)
            row=dict(name=name,case=case_id,mode=mode,rejected_before_actuation=False,safe_completion=result['task_success'],
                     expected_outcome_verified=bool(not expected_rejection and result['task_success']))
        except (ValueError,RuntimeError):
            path=case/mode/'startup.json'
            if not path.exists() or read(path)['status']!='rejected_before_actuation':raise
            assert not (case/mode/'trajectory.npz').exists() and not (case/mode/'result.json').exists()
            row=dict(name=name,case=case_id,mode=mode,rejected_before_actuation=True,safe_completion=False,
                     expected_outcome_verified=expected_rejection,error=read(path)['error'])
        rows.append(row)
        save(destination/'validation.json',{'rows':rows,'complete':len(rows)==len(plan),
             'all_expected_outcomes_verified':len(rows)==len(plan) and all(r['expected_outcome_verified'] for r in rows),
             'scope':'Three supported full-physics checks and three expected startup refusals; refusals are not safe pickups, trajectories or hardware safe stops. All retained earlier experimental results remain unchanged.'})
        print('Startup diagnostic:',row,flush=True)
    return read(destination/'validation.json')


def startup_report(folder):
    """Check the actual startup contract separately from predicted outcomes."""
    original=read(folder/'validation.json');rows=[]
    for r in original['rows']:
        path=folder/r['name']/r['case']/r['mode'];startup=read(path/'startup.json')
        if r['rejected_before_actuation']:
            checks={'explicit_refusal':startup['status']=='rejected_before_actuation',
                    'no_fabricated_result':not (path/'result.json').exists(),
                    'no_fabricated_trajectory':not (path/'trajectory.npz').exists()}
        else:
            result=read(path/'result.json');audit_result=read(path.parent.parent/'verification.json')
            checks={'all_objects_retained_at_start':all(startup['policy'].values()),
                    'initial_h2_nonnegative':startup['minimum_h2_before_install']>=0,
                    'initial_transformed_h1_nonnegative':startup['minimum_h1_before_install']>=0,
                    'safe_pickup':result['task_success'],'native_audit':audit_result['all_checks_pass']}
        rows.append(dict(r,checks=checks,startup_contract_verified=all(checks.values()),
                         source_startup_sha256=digest(path/'startup.json')))
    output={'complete':original['complete'],'rows':rows,
            'all_startup_contracts_verified':original['complete'] and all(r['startup_contract_verified'] for r in rows),
            'scope':'Offline post-run startup-contract check, separate from pre-run outcome predictions. A valid cover may exist even when refusal was predicted: finer sphere covers are not pointwise nested. The raw protocol/predictions are unchanged. Refusals are not safe pickups or hardware stops.'}
    save(folder/'startup_report.json',output)
    print(json.dumps({'accepted_pickups':sum(r['safe_completion'] for r in rows),
                      'refusals':sum(r['rejected_before_actuation'] for r in rows),
                      'all_startup_contracts_verified':output['all_startup_contracts_verified']},indent=2))
    return output


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=("report","audit","profile","replay","compare","startup","startup-report"))
    parser.add_argument("--output",type=Path,default=ROOT/"artifacts/pickup/heldout_v1")
    parser.add_argument("--destination",type=Path,help="new directory for a diagnostic replay")
    parser.add_argument("--case",default="layout_02")
    parser.add_argument("--task-protocol",choices=("current","archived"),default="current",help="archived task sequence with current decoder, or the current task sequence")
    args=parser.parse_args(argv)
    if args.action == "replay":
        if args.destination is None:
            parser.error("replay needs a new --destination")
        replay(args.output,args.destination,args.case,task_protocol=args.task_protocol)
    elif args.action == 'startup':
        if args.destination is None:
            parser.error('startup needs --output SOURCE_PICKUP_ROOT and a new --destination')
        validate_startup(args.output,args.destination)
    elif args.action == 'startup-report':
        startup_report(args.output)
    else:
        globals()[args.action](args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
