"""Render saved physical pickup trajectories; no API calls or new rollouts."""

import html
import hashlib
import inspect
import json
import os

import numpy as np

from inspect_geometry import add_scene_labels
from inspect_selection import ROOT
from pickup import PickupRuntime
from run_static import load_upstream
from run_suite import check_manifest, digest, read, save
from showcase_gallery import STYLE
from static_gallery import write_apng


TITLES = {"off": "Scene CBFs off", "all": "All clutter / fixed four", "once": "Jev once", "repeated": "Jev repeated", "rule": "Geometric rule", "geometry": "All objects / Jev geometry", "all_fine":"All clutter / least excess"}


def mode_payload(trace, result, indices, target_id, objects):
    clutter_columns = [i for i, o in enumerate(objects) if o["id"] != target_id]
    enabled = trace["cover_enabled"][indices]
    per_object = np.any(enabled, axis=1).reshape(len(indices), len(objects), -1).sum(axis=2)
    clutter_distance = trace["object_distance_m"][indices][:, clutter_columns]
    unwanted=np.any(clutter_distance<=0,axis=1) if clutter_columns else np.zeros(len(indices),dtype=bool)
    if "forbidden_target_distance_m" in trace:
        unwanted |= trace["forbidden_target_distance_m"][indices] <= 0
    if "target_clutter_contact" in trace:
        unwanted |= trace["target_clutter_contact"][indices]
    return {
        "times": trace["time_s"][indices].tolist(), "phase": trace["phase"][indices].tolist(),
        "bottom_mm": (trace["target_bottom_m"][indices]*1000).tolist(),
        "finger_contact": trace["bilateral_finger_contact"][indices].tolist(),
        "pairs": trace["enabled_scene_pairs"][indices].tolist(), "enabled": per_object.tolist(),
        "clutter_mm": (clutter_distance.min(axis=1)*1000).tolist() if clutter_columns else None,
        "scene_barrier_mm": (trace["scene_h2_m"][indices]*1000).tolist(),
        "unwanted_contact":unwanted.tolist(),
        "finished_at_s": result["simulation_seconds"], "pickup_verified": result["grasp_verified"],
        "objects": [o["id"] for o in objects],
    }


def render_frames(mj, model, trace, indices, objects, target_id, *, width=480, height=360, camera_options=None, object_labels=None):
    data = mj.MjData(model)
    option = mj.MjvOption(); option.geomgroup[3] = 0; option.geomgroup[4] = 0
    camera = mj.MjvCamera(); camera.lookat[:] = [.35, 0., .25]
    camera.distance, camera.azimuth, camera.elevation = 1.35, 135, -35
    for key, value in (camera_options or {}).items():
        if key == 'lookat':
            camera.lookat[:] = value
        elif key in ('distance', 'azimuth', 'elevation'):
            setattr(camera, key, value)
        else:
            raise ValueError('unknown playback camera option: '+key)
    model.geom("target_geom").rgba[3] = 0
    with mj.Renderer(model, width=width, height=height) as renderer:
        for index in indices:
            data.qpos[:], data.qvel[:] = trace["qpos"][index], trace["qvel"][index]
            mj.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            scene = renderer.scene
            enabled = np.any(trace["cover_enabled"][index], axis=0)
            for point, radius in zip(trace["cover_centers_m"][index][enabled], trace["cover_radii_m"][index][enabled], strict=True):
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0, 0]),
                               point, np.eye(3).ravel(), np.array([.95, .65, .12, .13]))
                scene.ngeom += 1
            labels = []
            for obj in objects:
                geom = model.geom(f"scene/{obj['id']}").id
                label = (object_labels or {}).get(obj['id'], "TARGET" if obj["id"] == target_id else obj["id"])
                labels.append((label, data.geom_xpos[geom]+[0, 0, obj["size_m"][2]/2+.025]))
            add_scene_labels(scene, labels, append=True)
            for point in trace["ee_m"][np.linspace(0, index, min(index+1, 70), dtype=int)]:
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE,
                               np.array([.003, 0, 0]), point, np.eye(3).ravel(), np.array([.9, .2, .8, 1.]))
                scene.ngeom += 1
            yield renderer.render().copy()


def render_case(folder, scene, settings, demo, mj):
    request = read(folder/"initial/request.json")
    runtime = PickupRuntime(scene, settings, request, None, "off", folder/"render_unused")
    physical=dict(request['state'],objects=scene['objects'])
    spec = runtime.scene_spec(mj, ROOT.parent/"frax", physical, [], [])
    spec.attach(runtime.robot_spec(mj, ROOT.parent/"frax"), prefix="", frame=spec.worldbody.add_frame())
    model = spec.compile()
    results = {mode: read(folder/mode/"result.json") for mode in TITLES if (folder/mode/"result.json").exists()}
    if not results:
        rejections=[(mode,read(folder/mode/'startup.json')) for mode in TITLES
                    if (folder/mode/'startup.json').exists() and read(folder/mode/'startup.json')['status']=='rejected_before_actuation']
        if rejections:
            cards=''.join(f"<article><h2>{TITLES[mode]}</h2><p>REJECTED BEFORE ACTUATION</p><p>{html.escape(event['error'])}</p><a href='{mode}/startup.json'>Startup diagnostics</a> · <a href='{mode}/attempt.json'>Recorded attempt</a></article>" for mode,event in rejections)
            (folder/'index.html').write_text(f'<!doctype html><html lang="en"><meta charset="utf-8"><title>Unsupported startup</title><style>{STYLE}</style><main><a href="../index.html">← Overview</a><h1>{html.escape(scene["id"])}: unsupported starting model</h1><p>No control trajectory or successful pickup is fabricated for a rejected setup. This is a simulator startup refusal, not a hardware safe stop.</p>{cards}<a href="initial/request.json">Frozen request</a></main></html>')
        return results
    duration = max(r["simulation_seconds"] for r in results.values())
    fps, columns, width, height = 6, 8, 480, 360
    times = np.arange(int(np.ceil(duration*fps))+1)/fps
    payload, cards = {}, []
    for mode, result in results.items():
        source = folder/mode/"trajectory.npz"
        if digest(source) != result["trajectory_sha256"]:
            raise ValueError("trajectory hash differs from saved result")
        with np.load(source) as trace:
            initial_barrier = float(trace['scene_h2_m'][0])
            indices = np.minimum(np.searchsorted(trace["time_s"], times), len(trace["time_s"])-1)
            payload[mode] = mode_payload(trace, result, indices, scene["target_id"], scene["objects"])
            unique_indices, image_index = np.unique(indices, return_inverse=True)
            payload[mode]["image_index"] = image_index.tolist()
            atlas_path, meta_path = folder/f"{mode}_atlas.png", folder/f"{mode}_atlas.json"
            renderer_source = inspect.getsource(render_frames)+inspect.getsource(PickupRuntime.robot_spec)+inspect.getsource(PickupRuntime.scene_spec)
            fingerprint = {"trajectory_sha256": digest(source), "renderer_sha256": hashlib.sha256(renderer_source.encode()).hexdigest(),
                           "model_builder_sha256": digest(ROOT/"sim_model.py"), "frames": len(unique_indices),
                           "indices_sha256":hashlib.sha256(unique_indices.tobytes()).hexdigest()}
            if not atlas_path.exists() or not meta_path.exists() or read(meta_path) != fingerprint:
                atlas = np.zeros((int(np.ceil(len(unique_indices)/columns))*height, columns*width, 3), dtype=np.uint8)
                for n, frame in enumerate(render_frames(mj, model, trace, unique_indices, scene["objects"], scene["target_id"])):
                    y, x = n//columns*height, n%columns*width
                    atlas[y:y+height, x:x+width] = frame
                write_apng(atlas_path, [atlas], frame_count=1, fps=1)
                y, x = (len(unique_indices)-1)//columns*height, (len(unique_indices)-1)%columns*width
                write_apng(folder/f"{mode}_final.png", [atlas[y:y+height, x:x+width]], frame_count=1, fps=1)
                save(meta_path, fingerprint)
                print(f"Rendered {scene['id']}/{mode}: {len(unique_indices)} unique frames / {len(indices)} timeline ticks", flush=True)
        status = "PASS" if result["task_success"] else "FAIL"
        barrier=result["minimum_selected_scene_sphere_barrier_m"]
        barrier_text="not enabled" if barrier is None else f"{barrier*1000:.3f} mm"
        cards.append(f'''<article id="card_{mode}"><h2>{TITLES[mode]}</h2><canvas id="{mode}" width="480" height="360"></canvas>
          <p id="{mode}_live"></p><hr><p class="{'good' if result['task_success'] else 'bad'}">{status} · verified physical pickup: {result['grasp_verified']}</p>
          <p>Unwanted contact: {result['collision']['any_sampled_scene_violation']}<br>Lowest target point at end: {result['final_target_bottom_m']*1000:.1f} mm<br>
          Minimum modeled scene barrier: {barrier_text}<br>
          Initial modeled scene barrier: {initial_barrier*1000:.3f} mm{' — INVALID MODELED START' if initial_barrier < 0 else ''}<br>
          Mean enabled pairs: {result['enabled_scene_pair_counts']['mean']:.1f} / {result['allocated_scene_pair_count']} allocated</p>
          <a href="{mode}/result.json">Results</a> · <a href="{mode}/trajectory.npz">Trajectory</a> · <a href="{mode}_final.png">Final frame</a></article>''')
    events = []
    for mode, result in results.items():
        for event in result["selection_events"]:
            if event.get("kind") == "selection":
                source = event.get("source", "")
                link = f'<a href="{html.escape(source)}">Raw answer</a>' if (folder/source).is_file() else html.escape(source)
                events.append(f"<tr><td>{mode}</td><td>{event.get('snapshot_at_s')}</td><td>{event.get('applied_at_s')}</td><td>{html.escape(event['status'])}</td><td>{html.escape(str(event.get('policy')))}</td><td>{link}</td></tr>")
    save(folder/"playback.json", payload)
    encoded = json.dumps(payload, allow_nan=False).replace("</", "<\\/")
    toggles = " ".join(f'<label><input id="show_{mode}" type="checkbox" checked> {TITLES[mode]}</label>' for mode in results)
    camera_section=''
    if 'observation_config' in scene:
        views=[]
        for camera in ('scene_camera','wrist_camera'):
            for suffix in ('','_depth'):
                path=folder/'initial_camera'/f'{camera}{suffix}.png'
                if path.exists():views.append(f'<figure><img width="320" src="initial_camera/{path.name}" alt="{camera}{suffix}"><figcaption>{camera}{suffix}</figcaption></figure>')
        camera_section=f'''<details><summary>Camera inputs and perception assumptions</summary><p>Clutter geometry is depth-fitted, with oracle simulator instance labels. Target geometry/pose and robot/camera kinematics remain known. Padding is experimental, not calibrated uncertainty. Native evaluation uses the physical objects, not fitted boxes.</p><div class="cards">{''.join(views)}</div><p><a href="initial_camera/fit.json">Initial fit</a> · Per-method observation events and raw frames are recorded under each method directory.</p></details>'''
    initial_answer_link='<a href="initial/selection.json">Initial answer/probabilities</a> · ' if (folder/'initial/selection.json').exists() else ''
    document = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Pickup: {html.escape(scene['id'])}</title><style>{STYLE}</style><main><a href="../index.html">← Overview</a>
    <h1>Static-clutter pickup: {html.escape(scene['id'])}</h1><p>Orange solid object = pickup target; cyan objects = fixed clutter; transparent spheres = enabled object covers; magenta = actual hand path.</p>
    <p>One task: maneuver, grasp, lift enough to verify pickup, then stop. An ended panel holds its last recorded frame while other methods continue.</p>
    {camera_section}
    <div class="controls"><button id="play">Pause</button><button id="restart">Restart</button><input id="scrub" aria-label="Simulation time" type="range" min="0" max="{len(times)-1}" value="0" step="1"><span id="clock"></span>
    <select id="speed" aria-label="Playback speed"><option value="1">1×</option><option value=".5">0.5×</option><option value=".25">0.25×</option></select></div>
    <p>Show methods: {toggles}</p><div class="cards">{''.join(cards)}</div><h2>Recorded selection updates</h2><table><tr><th>Mode</th><th>Snapshot s</th><th>Applied s</th><th>Status</th><th>Effective choices</th><th>Evidence</th></tr>{''.join(events)}</table>
    <p>{initial_answer_link}<a href="initial/request.json">Initial request</a> · <a href="playback.json">Playback data</a></p>
    <p class="notice">Only finger-target contact is intended. Palm/arm-target and robot/target-clutter collisions fail evaluation, including omitted objects. No target weld, teleportation, or scripted lift.
    The FRAX arm model does not include finger/contact/payload dynamics: these runs are empirical, not certified contact safety. Full-aperture finger envelopes are conservative. Fixed QP slots mean fewer enabled rows, not measured solver speedup.</p>
    <script id="data" type="application/json">{encoded}</script><script>
    const data=JSON.parse(document.getElementById('data').textContent), images={{}}, scrub=document.getElementById('scrub');
    let playing=true, frame=0, previous=0, carry=0;
    function draw(){{document.getElementById('clock').textContent=(frame/{fps}).toFixed(2)+' s';
      for(const [mode,d] of Object.entries(data)){{const i=Math.min(frame,d.times.length-1),image=images[mode];
        const cell=d.image_index?d.image_index[i]:i;
        if(image&&image.complete&&image.naturalWidth)document.getElementById(mode).getContext('2d').drawImage(image,(cell%{columns})*480,Math.floor(cell/{columns})*360,480,360,0,0,480,360);
        const ended=frame/{fps}>=d.finished_at_s?' · ENDED':'';
        const clearance=d.clutter_mm?' · closest clutter '+d.clutter_mm[i].toFixed(1)+' mm':'';
        document.getElementById(mode+'_live').textContent='t='+d.times[i].toFixed(2)+' s · '+d.phase[i]+ended+' · lowest target point '+d.bottom_mm[i].toFixed(1)+' mm · both fingers touching: '+d.finger_contact[i]+clearance+' · unwanted contact here: '+d.unwanted_contact[i]+' · '+d.pairs[i]+' pairs — '+d.objects.map((o,j)=>o+': '+d.enabled[i][j]+' primitives').join(' · ');
      }} }}
    for(const mode of Object.keys(data)){{images[mode]=new Image();images[mode].onload=draw;images[mode].src=mode+'_atlas.png';}}
    for(const mode of Object.keys(data))document.getElementById('show_'+mode).onchange=e=>document.getElementById('card_'+mode).style.display=e.target.checked?'':'none';
    function button(){{document.getElementById('play').textContent=playing?'Pause':'Play';}}
    document.getElementById('play').onclick=()=>{{playing=!playing;button();}};
    document.getElementById('restart').onclick=()=>{{frame=0;scrub.value=0;carry=0;draw();}};
    scrub.oninput=()=>{{frame=Number(scrub.value);playing=false;button();draw();}};
    function tick(now){{const elapsed=previous?Math.min(now-previous,250):0;previous=now;
      if(playing){{carry+=elapsed*Number(document.getElementById('speed').value);while(carry>=1000/{fps}){{carry-=1000/{fps};frame=(frame+1)%(Number(scrub.max)+1);}}scrub.value=frame;draw();}}requestAnimationFrame(tick);}}
    draw();requestAnimationFrame(tick);</script></main></html>'''
    (folder/"index.html").write_text(document)
    return results


def render_gallery(folder):
    manifest = check_manifest(folder)
    config, settings = read(folder/"scenes.json"), read(folder/"settings.json")
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    summaries, cards = {}, []
    for scene in config["scenes"]:
        results = render_case(folder/scene["id"], scene, settings, demo, utils.mujoco)
        summaries[scene["id"]] = results
        rows = "".join(f"<tr><td>{TITLES[m]}</td><td>{r['grasp_verified']}</td><td>{r['collision']['any_sampled_scene_violation']}</td><td>{r['task_success']}</td><td>{r['enabled_scene_pair_counts']['mean']:.1f}</td></tr>" for m,r in results.items())
        cards.append(f"<article><h2><a href='{scene['id']}/index.html'>{html.escape(scene['id'])} →</a></h2>{'<p>Startup rejected before actuation; no physical rollout.</p>' if not results else ''}<table><tr><th>Method</th><th>Pickup</th><th>Unwanted contact</th><th>Pass</th><th>Mean pairs</th></tr>{rows}</table></article>")
    from run_pickup import BUDGET_PATH
    ledger = read(BUDGET_PATH) if BUDGET_PATH.exists() else {"cap_usd": "7.50", "attempts": []}
    reported = sum(float(a.get("reported_usage_estimated_usd", 0)) for a in ledger["attempts"])
    reserved = sum(float(a["reserved_usd"]) for a in ledger["attempts"])
    attempts = list(folder.glob("*/initial/attempt.json"))+list(folder.glob("*/queries/*/attempt.json"))
    classification_repeats=list(folder.glob('*/classification_repeats/*/attempt.json'))
    save(folder/"review_results.json", {"cases": summaries, "manifest": manifest, "dataset_logical_api_attempts":len(attempts),
                                      "classification_only_repeat_attempts":len(classification_repeats),
                                      "usage_ledger_scope":"All experiments in the approved autonomous work, not just this dataset", "usage_ledger": ledger})
    extra_links = " · ".join(f'<a href="{name}">{label}</a>' for name,label in (("verification.json","Independent verification"),("test_output.txt","Offline tests"),("research_report.json","Research analysis")) if (folder/name).exists())
    (folder/"index.html").write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Static-clutter pickup gallery</title>
    <style>{STYLE}</style><main><h1>Jev-selected CBFs<br><span class="muted">Static-clutter pickup</span></h1>
    <p>One task family: maneuver through fixed clutter, pick up the designated object, then stop. Different layouts; same original FRAX nominal torque policy and task logic. CBFpy backend, read-only upstream repositories.</p>
    <div class="notice">Open a layout for synchronized playback. Check actual lift, bilateral finger contact, unwanted collisions, and chosen covers. A plausible animation alone is not success.</div>
    <div class="cards" style="margin-top:24px">{''.join(cards)}</div>
    <h2>Scope</h2><p>{'Depth-fitted clutter with oracle instance identities and a known target/robot/camera model.' if any('observation_config' in s for s in config['scenes']) else 'Ground-truth object geometry/kinematics.'} No hardware. Native articulated fingers and free-body target. Only finger-target contact is allowed. Original arm protections stay on in every method; OFF disables scene-object CBFs only. The arm model omits finger/contact/payload dynamics, and sampled diagnostics are not a continuous-time safety proof.</p>
    <p>This dataset: {len(attempts)} control-selection attempts and {len(classification_repeats)} classification-only repeats. Injected unavailable attempts are not provider requests. Overall approved-work budget (all experiments): ${float(ledger['cap_usd']):.2f}; {len(ledger['attempts'])} logical provider requests; reported token-cost estimate ${reported:.6f}; conservative reservations including possible retries ${reserved:.6f}. Estimates use <a href="https://docs.typesafe.ai/models">official direct-API pricing</a>, not an invoice.</p>
    <p><a href="manifest.json">Frozen inputs/provenance</a> · <a href="review_results.json">Full results and usage ledger</a> · {extra_links}</p></main></html>''')
    print(f"Pickup gallery: {folder/'index.html'}", flush=True)


def render_research_hub(base):
    """One entry point for current evidence and explicitly retained history."""
    if (base/'unseen_selection_v1/comparison.json').exists():
        return render_study_review(base)
    destination=base/"review";destination.mkdir(parents=True,exist_ok=True)
    sections=[]
    for name,title in (("corrected_replication_v2","Corrected replication — same 12 layouts"),
                       ("heldout_v1","Original held-out evaluation — before engineering corrections")):
        path=base/name/"research_report.json"
        if not path.exists():
            continue
        data=read(path);rows=[]
        for mode in ("off","all","rule","geometry","once","repeated"):
            if mode not in data["methods"]:
                continue
            result=data["methods"][mode]
            rows.append(f"<tr><td>{TITLES[mode]}</td><td>{result['safe_completions']}/{result['cases']}</td>"
                        f"<td>{result['contact_cases']}</td><td>{result['mean_enabled_pairs_equal_layout_weight']:.1f}</td>"
                        f"<td>{result.get('mean_clutter_excess_m3',0)*1000:.2f} L</td></tr>")
        sections.append(f"<section><h2><a href='../{name}/index.html'>{title} →</a></h2><table><tr><th>Method</th><th>Safe pickups</th><th>Contact cases</th><th>Mean enabled pairs</th><th>Mean clutter-cover excess</th></tr>{''.join(rows)}</table><p><a href='../{name}/research_report.json'>Analysis/data</a> · <a href='../{name}/verification.json'>Independent native audit</a></p></section>")
    geometry=[]
    for name in ("cylinder","box"):
        data=read(base/"geometry_study_v1"/f"{name}.json")
        for candidate in data["candidates"]:
            geometry.append(f"<tr><td>{name}</td><td>{candidate['candidate_id']}</td><td>{candidate['primitive_count']}</td><td>{candidate['excess_volume_m3']*1000:.3f} ± {candidate['excess_volume_standard_error_m3']*1000:.3f} L</td><td>{candidate['uncovered_object_samples']}</td></tr>")
    ledger=read(ROOT/"artifacts/usage/pickup_research_v1.json")
    estimated=sum(float(a.get("reported_usage_estimated_usd",0)) for a in ledger["attempts"])
    reserved=sum(float(a["reserved_usd"]) for a in ledger["attempts"])
    links=" · ".join(f'<a href="{os.path.relpath(ROOT/name,destination)}">{name}</a>' for name in ("README.md","PROGRESS.md","ARCHITECTURE.md"))
    (destination/"index.html").write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Static-clutter pickup — research review</title><style>{STYLE}section{{margin:32px 0}}.tablewrap{{overflow:auto}}</style><main>
    <h1>Static-clutter pickup<br><span class="muted">Working system and research review</span></h1>
    <p>One task family: maneuver the arm/gripper through fixed clutter, pick up the designated object, then stop after physical lift verification.</p>
    <div class="notice">Start with the corrected replication below. Each layout has synchronized, scrubbable playback and method visibility toggles. Orange solid object = target, cyan = static clutter, transparent spheres = applied covers. Read safety/completion diagnostics, not just the animation.</div>
    {''.join(sections)}
    <h2>How to interpret these results</h2>
    <p>Safe pickup requires verified lift/bilateral finger contact, no unwanted native contacts across the whole run, finite completion, and nonnegative sampled modeled barriers. OFF disables only scene CBFs; intrinsic robot protections remain. The geometry-only ablation retains clutter and uses the shared initial answer's conditional geometries. The geometric rule uses no model.</p>
    <p>Enabled-pair counts include 64 mandatory arm-target pairs in constrained modes. Allocated scene pairs remain 348: these are not smaller QPs or measured optimization speedups. Cover excess is a time-averaged spatial union metric excluding the target and robot-radius inflation; Monte Carlo error is sampling noise, not perception uncertainty. Intentionally omitted clutter is not automatically an unsafe missed barrier.</p>
    <p>The original 12-layout evaluation was frozen before model answers. The corrected replication reuses those same layouts after decoder/grasp-skill fixes, so it is not a new unseen benchmark. All-on and the deterministic rule are important baselines; the data do not establish Jev superiority or general safety.</p>
    <h2>What changed after diagnosis</h2>
    <p>The provider returns approximately normalized probabilities; rejecting rounded totals of 0.99 lost valid updates. The decoder now tolerates hundredth rounding without changing raw answers. A controlled decoder-only replay still failed. The grasp skill now aligns above the target, checks settling, backs off lateral descent drift, and requires bilateral contact before lifting. The original FRAX nominal torque equations/gains are unchanged.</p>
    <p><a href='../decoder_correction_replay_v1/result.json'>Decoder-only replay (failed)</a> · <a href='../alignment_correction_replay_v1/result.json'>Alignment replay (successful, not fresh model performance)</a> · <a href='../corrected_replication_protocol_v2.json'>Correction protocol</a></p>
    <h2>Geometry component in isolation</h2><p>Every candidate conservatively encloses its known reference shape by a cell/slab construction. Sampled misses below are diagnostics, not that proof. Global volume is not the same as obstruction near a task's desired motion.</p>
    <table><tr><th>Reference</th><th>Candidate</th><th>Spheres</th><th>Excess ± MC SE</th><th>Sampled misses</th></tr>{''.join(geometry)}</table>
    <p><a href='../geometry_study_v1/cylinder.json'>Cylinder raw measurements</a> · <a href='../geometry_study_v1/box.json'>Box raw measurements</a></p>
    <h2>Prompt experiments and CPU measurements</h2>
    <p>The revised prompt did not improve the three development-layout completion/contact outcomes and used more enabled pairs; the original was retained before the held-out model calls. This small development comparison is not proof of an optimal prompt. A major CPU bottleneck was eager per-step FK; a compiled wrapper removed it. Warm CBF calls still have fixed allocated dimensions, so compact-QP engineering was not justified by this small setup.</p>
    <p><a href='../development_shape_prompt1_v3/index.html'>Development prompt 1</a> · <a href='../development_shape_prompt2_v2/index.html'>Development prompt 2</a> · <a href='../development_prompt_comparison.json'>Comparison data</a> · <a href='../showcase_v1/fk_profile.json'>FK timing</a> · <a href='../corrected_replication_v2/cpu_profile.json'>Final CPU microbenchmark</a></p>
    <h2>Limits and next research questions</h2>
    <p>Ground-truth shapes/poses, one upright cylindrical target family, idealized FRAX-matched arm inertias, and native articulated fingers. The seven-DOF CBF model omits finger/contact/payload dynamics. Target interaction masks are task-defined, not learned body selection; only finger-target contact is allowed by the evaluator. Finite sampled tests are not a continuous-time/contact-safety or hardware guarantee. No perception, learned policy, global motion planner, placement, pushing, or moving-clutter experiments.</p>
    <p>Initial coarse covers can obstruct the desired grasp despite reducing constraint count. Explicit candidate clearance at the future grasp/lift reference poses is a promising next input to study. Under oracle geometry, the simple geometric rule is strong; semantic/perception value has not been established here.</p>
    <h2>Budget and reproducibility</h2><p>{len(ledger['attempts'])} new logical SDK requests across the approved work. Reported token-cost estimate ${estimated:.6f}; conservative reservations ${reserved:.6f}; cap $7.50. <a href='https://docs.typesafe.ai/models'>Direct-provider pricing</a>, not a billing invoice. No new model calls are needed to view, audit, or analyze saved results.</p>
    <p><a href='../showcase_v1/index.html'>First four-item pickup gallery</a> · {links}</p></main></html>''')
    print(f"Research review: {destination/'index.html'}",flush=True)


def render_study_review(base):
    """Current research package, not a hand-picked success-only showcase."""
    destination=base/'review';destination.mkdir(parents=True,exist_ok=True)
    groups=(('selection_development_v1','Development: counts × terminal context'),
            ('unseen_selection_v1','Frozen unseen geometry-qualified layouts'),
            ('camera_control_v1','Depth-derived clutter and controlled failures'))
    sections=[]
    for name,title in groups:
        root=base/name
        if not(root/'protocol.json').exists():continue
        comparison=read(root/'comparison.json') if (root/'comparison.json').exists() else {}
        details=[]
        for config in sorted((root/'inputs').glob('*.json')):
            condition=config.stem;folder=root/condition;path=folder/'research_report.json'
            methods=read(path)['methods'] if path.exists() else {}
            rows=[]
            for mode in ('off','all','all_fine','rule','geometry','once','repeated'):
                if mode not in methods:continue
                r=methods[mode];gap=r.get('mean_uncovered_constrained_clutter_m3')
                rows.append(f"<tr><td>{TITLES[mode]}</td><td>{r['safe_completions']}/{r['cases']}</td><td>{r['contact_cases']}</td><td>{r.get('initial_model_inadmissible_cases','pending')}</td><td>{r.get('later_scene_crossing_cases','pending')}</td><td>{r['mean_enabled_pairs_equal_layout_weight']:.1f}</td><td>{r['mean_clutter_excess_m3']*1000:.3f}</td><td>{'pending' if gap is None else f'{gap*1000:.4f}'}</td></tr>")
            link=f"<a href='../{name}/{condition}/index.html'>Synchronized gallery →</a>" if (folder/'index.html').exists() else 'Playback not yet rendered'
            links=' · '.join(f"<a href='../{name}/{condition}/{file}'>{label}</a>" for file,label in (('research_report.json','Analysis'),('verification.json','Native audit'),('manifest.json','Frozen provenance')) if (folder/file).exists())
            details.append(f"<details><summary>{html.escape(condition)} — {link}</summary><div class='tablewrap'><table><tr><th>Method</th><th>Observed safe pickups</th><th>Contact cases</th><th>Invalid modeled starts</th><th>Scene barrier crossings after nonnegative start</th><th>Mean enabled pairs</th><th>Excess L</th><th>Constrained coverage gap L</th></tr>{''.join(rows)}</table></div><p>{links}</p></details>")
        repeated=[r for r in comparison.get('classification_repeatability',[]) if r['valid']>=2]
        consistency=f"Initial classifications disagreed in {sum(bool(r['questions_with_disagreement']) for r in repeated)}/{len(repeated)} scene/condition groups with multiple valid answers; proposed clutter-policy disagreement occurred in {sum(r.get('proposed_clutter_policy_disagreement',False) for r in repeated)}/{len(repeated)}. Conditional geometry for omitted objects does not affect the proposed policy. These are classification-only repeats, not physical rollout repetitions or applied-policy safety tests." if repeated else ''
        status='COMPLETE' if comparison.get('all_conditions_complete') else 'IN PROGRESS — not a final result'
        sections.append(f"<section><h2>{title}</h2><p>{status}. <a href='../{name}/protocol.json'>Protocol</a>"+
                        (f" · <a href='../{name}/comparison.json'>Full comparison</a>" if (root/'comparison.json').exists() else '')+f"</p>{''.join(details)}<p>{consistency}</p></section>")
    ledger=read(ROOT/'artifacts/usage/pickup_research_v1.json')
    cost=sum(float(a.get('reported_usage_estimated_usd',0)) for a in ledger['attempts'])
    reserved=sum(float(a['reserved_usd']) for a in ledger['attempts'])
    camera=''
    frames=base/'camera_inspection_v3/00'
    if frames.exists():
        camera='<div class="cards">'+''.join(f"<figure><img width='320' src='../camera_inspection_v3/00/{name}.png' alt='{name}'><figcaption>{name}</figcaption></figure>" for name in ('scene_camera','wrist_camera','scene_camera_depth','wrist_camera_depth'))+'</div>'
    bundle_link="<a href='../review_bundle.zip'>Download local portable review bundle</a>" if (base/'review_bundle.zip').exists() else 'Portable bundle is not yet generated.'
    findings=''
    unseen=base/'unseen_selection_v1/comparison.json'
    if unseen.exists() and read(unseen).get('all_conditions_complete'):
        conditions=read(unseen)['conditions']
        small,rich=(conditions[k]['methods'] for k in ('small_terminal','rich_terminal'))
        findings=f'''<section><h2>Main findings in these fixtures</h2><p>Richer covers matter: one-shot and repeated Jev each improve from {small['once']['safe_completions']}/12 to {rich['once']['safe_completions']}/12 observed safe pickups. The all-object/least-excess baseline also improves from {small['all_fine']['safe_completions']}/12 to {rich['all_fine']['safe_completions']}/12, so geometry alone explains an important part of the gain.</p>
        <p>Repeated querying does not improve completion over once on this new set. The rich geometric rule also finishes {rich['rule']['safe_completions']}/12 with {rich['rule']['mean_enabled_pairs_equal_layout_weight']:.1f} mean enabled pairs versus {rich['once']['mean_enabled_pairs_equal_layout_weight']:.1f} for one-shot Jev. These experiments do not establish Jev superiority. OFF has three unwanted-contact cases; fixed-four is often too conservative to finish.</p></section>'''
    startup=''
    validation=base/'startup_validation_v1/validation.json'
    if validation.exists():
        post=validation.with_name('startup_report.json')
        data=read(post if post.exists() else validation)
        rows=''.join(f"<tr><td><a href='../startup_validation_v1/{r['name']}/index.html'>{html.escape(r['name'])}</a></td><td>{'REFUSED BEFORE ACTUATION' if r['rejected_before_actuation'] else 'Physical rollout'}</td><td>{r['safe_completion']}</td><td>{r.get('startup_contract_verified','pending')}</td></tr>" for r in data['rows'])
        startup=f'''<section><h2>Startup correction: separate validation</h2><p>{'COMPLETE' if data['complete'] else 'IN PROGRESS'}. A currently inadmissible model is no longer silently used to actuate. Normal selection modes initialize with an admissible cover for every object; fixed-four retains its exact baseline meaning and refuses an invalid start. If no supplied full-shape candidate is admissible, the setup is rejected. Estimated-shape containment is not true-object coverage.</p><table><tr><th>Retained diagnostic input</th><th>Outcome</th><th>Safe pickup</th><th>Startup contract verified</th></tr>{rows}</table><p>Four physical pickups and two refusals. One refusal prediction was wrong: noisy layout 03 has an admissible two-sphere cover even though its four/eight-sphere choices overlap the initial arm. Covers are not pointwise nested; the raw prediction is retained, not rewritten.</p><p><a href='../startup_validation_v1/validation.json'>Raw validation/predictions</a> · <a href='../startup_validation_v1/startup_report.json'>Actual startup-contract checks</a> · <a href='../startup_validation_v1/protocol.json'>Correction protocol</a>. Already-seen diagnostics, not another unseen benchmark. Refusals are not safe pickups or hardware safe stops. The original frozen study above is unchanged.</p></section>'''
    (destination/'index.html').write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Jev CBF research review</title><style>{STYLE}section{{margin:30px 0}}details{{margin:14px 0;padding:12px;border:1px solid #718096;border-radius:8px}}summary{{cursor:pointer}}.tablewrap{{overflow:auto}}figure{{margin:10px}}img{{max-width:100%;height:auto}}</style><main>
    <h1>Static-clutter pickup<br><span class="muted">Richer covers, selection, and camera-derived constraints</span></h1>
    <p>One task: maneuver through stationary clutter, pick up the designated object, and stop after physical lift verification. Original FRAX nominal arm controller and CBFpy backend; upstream repositories unchanged.</p>
    <div class="notice">Inspect all outcomes, not just plausible animations. Development layouts were already seen. The new unseen fixtures were geometry-qualified and frozen before model answers. Camera/error studies use two seen diagnostic layouts. Expand a condition, open its gallery, then scrub or hide methods to compare.</div>
    {findings}
    {''.join(sections)}
    {startup}
    <section><h2>What the geometry experiment tests</h2><p>Small family: 1/2/4 spheres. Rich family: 1/2/3/4/5/6/8, with cylinder-specific slabs and useful box grids. The development comparison holds the prompt fixed and varies counts and explicit terminal clearance. A negative terminal clearance says a cover blocks a reference posture, not that the physical object collides or no alternate route exists.</p>
    <p>All-on/fixed-four is the historical coarse baseline. All-on/least-excess keeps every clutter object and chooses from the same candidate family. The rule is non-model and can omit objects. More options can improve shape fit while increasing constraints or making selection harder; the tables, not an assumed Jev advantage, determine the outcome.</p></section>
    <section><h2>What the cameras actually provide</h2>{camera}<p>Native scene and hand-mounted wrist RGB/depth. Metric depth is backprojected using known calibration; clutter dimensions/poses are fitted from visible points. Instance identities come from oracle simulator segmentation, and the designated target geometry/pose remains known. Jev receives structured measurements, not raw RGBD. This is not learned detection or complete unknown-scene perception.</p>
    <p>Experimental 5 mm padding is not calibrated uncertainty. Visible-point boxes can miss unseen surfaces; calibration errors can leave true obstacle volume outside a cover even with positive modeled barriers. Constrained coverage gap separates that from intentional omission. Native contact evaluation always checks the true scene.</p>
    <p>Only Jev requests run in a background worker. Camera capture/fitting and evidence logging are periodic synchronous before-control work and can delay a step. No hard-real-time sensor isolation or active viewpoint planner is implemented.</p>
    <p><a href='../camera_inspection_v3/report.json'>Camera geometry inspection</a> · <a href='../camera_inspection_v3/sensor_study.json'>Controlled raw-depth study</a> · <a href='../perception_pilot_v1/rule/result.json'>First closed-loop depth-clutter pilot</a> · <a href='../camera_control_v1/fault_diagnostics.json'>Recorded outage/delay/freshness diagnostics</a></p></section>
    <section><h2>Safety and interpretation limits</h2><p>Observed safe pickup requires verified lift/bilateral finger contact, no unwanted native object contacts, finite completion and nonnegative sampled modeled barriers. Also inspect initial admissibility: a coarse/noisy cover can contain the initial robot pose, so CBF invariance preconditions do not hold. Do not mistake that for a certified filter starting inside its safe set. The seven-DOF arm model omits finger/contact/payload dynamics. No hardware, formal contact/continuous-time safety, global planning, or hard real-time guarantee.</p>
    <p>Enabled pairs include mandatory arm-target guards. Small/rich families allocate different fixed slot counts; disabling rows does not shrink a given QP. Coverage and excess are sampled scene-union diagnostics, not safety probabilities. Injected model outages make no HTTP request; delayed/late/rejected updates retain previous choices, not a certified safe fallback.</p>
    <p><a href='../selection_study_v1/terminal_diagnosis.json'>Original stall diagnosis</a> · <a href='../challenge_selection_v1.json'>New layout selection/provenance</a> · <a href='../d18_analysis_protocol_v1.json'>Analysis definitions</a> · <a href='../corrected_replication_v2/research_report.json'>Prior study summary</a></p></section>
    <section><h2>Budget and reproduction</h2><p>{len(ledger['attempts'])} logical provider requests across the cumulative approved work. Estimated reported token cost ${cost:.6f}; conservative reservations ${reserved:.6f}; total cap $7.50. <a href='https://docs.typesafe.ai/models'>Provider pricing</a>, not a billing invoice. View, analyze, and audit saved runs without new model calls.</p>
    <p>{bundle_link}</p><p><a href='../../../README.md'>Usage</a> · <a href='../../../PROGRESS.md'>Progress/decisions</a> · <a href='../../../ARCHITECTURE.md'>Architecture</a></p></section></main></html>''')
    print('Extended research review:',destination/'index.html',flush=True)


def build_bundle(base,destination):
    """Local, self-contained viewing bundle; no upload, credentials or upstream copies."""
    import subprocess
    import zipfile
    if destination.exists():raise ValueError('bundle already exists; retain it and choose a new output')
    key=os.environ.get('TYPESAFE_API_KEY','').encode()
    if not key:raise ValueError('load the configured key for an exact exclusion scan, without printing it')
    files=set()
    for raw in subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT).split(b'\0'):
        if raw:
            path=ROOT/os.fsdecode(raw)
            if path.is_file():files.add(path)
    for name in ('selection_development_v1','unseen_selection_v1','camera_control_v1','camera_inspection_v3','perception_pilot_v1','selection_study_v1','startup_validation_v1','review'):
        files.update(p for p in (base/name).rglob('*') if p.is_file())
    for name in ('challenge_selection_v1.json','d18_analysis_protocol_v1.json','corrected_replication_v2/research_report.json'):
        files.add(base/name)
    for name in ('review_checks.json','regression_d18.json','resume_check_d18.json','handoff_checks_d18.json','security_checks_d18.json','provenance_checks_d18.json'):
        if (base/name).exists():files.add(base/name)
    ledger=ROOT/'artifacts/usage/pickup_research_v1.json'
    if ledger.exists():files.add(ledger)
    for path in files:
        relative=path.relative_to(ROOT)
        if any(part in ('.git','.venv','.uv-cache','__pycache__') or part.startswith('.env') for part in relative.parts):
            raise ValueError('private/cache path in proposed bundle')
        if key in path.read_bytes():raise ValueError('configured credential found in proposed bundle: '+str(relative))
    records={str(p.relative_to(ROOT)):digest(p) for p in sorted(files)}
    save(base/'bundle_manifest.json',{'files_sha256':records,'scope':'Local portable viewing/source/result bundle. No credentials or upstream repositories; no upload. Physics reruns require the documented sibling sources/environment. Raw native/SDK data retained; known-target/oracle-instance perception limits apply.'})
    files.add(base/'bundle_manifest.json');destination.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(destination,'x') as archive:
        for path in sorted(files):
            compression=zipfile.ZIP_STORED if path.suffix in ('.png','.npz') else zipfile.ZIP_DEFLATED
            archive.write(path,str(path.relative_to(ROOT)),compress_type=compression)
    print(f'Portable bundle: {destination} ({destination.stat().st_size/1_000_000:.1f} MB, {len(files)} files)',flush=True)
    return destination


def verify_review(base):
    """Check saved links, atlas mappings and actual playback JS without a browser.

    A mocked DOM exercises controls; native rendered frames still need inspection.
    No network, new physics, model requests, or credential access.
    """
    from html.parser import HTMLParser
    from pathlib import Path
    from urllib.parse import unquote, urlsplit
    import re
    import struct
    import subprocess
    roots=[base/name for name in ('selection_development_v1','unseen_selection_v1','camera_control_v1','startup_validation_v1','review')]
    pages=sorted({p for root in roots for p in root.rglob('*.html')})
    broken=[];references=0;playback=[]
    class Links(HTMLParser):
        def handle_starttag(self,tag,attrs):
            nonlocal references
            for key,value in attrs:
                if key not in ('href','src') or not value:continue
                url=urlsplit(value)
                if url.scheme or url.netloc or not url.path:continue
                references+=1
                if not (page.parent/unquote(url.path)).resolve().exists():
                    broken.append({'page':str(page.relative_to(ROOT)),'reference':value})
    harness=r'''
const vm=require('node:vm'), assert=require('node:assert/strict');
let input='';process.stdin.on('data',s=>input+=s);process.stdin.on('end',()=>{
 const {script,data,max,atlases}=JSON.parse(input), elements={}, draws=[];
 for(const [mode,d] of Object.entries(data)){
   elements[mode]={getContext:()=>({drawImage:(image,x,y,w,h)=>{
     const a=atlases[mode];assert(x>=0&&y>=0&&x+w<=a[0]&&y+h<=a[1]);draws.push(mode);
   }})};elements[mode+'_live']={textContent:''};elements['show_'+mode]={checked:true};
   elements['card_'+mode]={style:{}};
 }
 for(const id of ['clock','play','restart','scrub','speed'])elements[id]={textContent:'',value:'0'};
 elements.data={textContent:JSON.stringify(data)};elements.scrub.max=String(max);elements.speed.value='1';
 class Image{constructor(){this.complete=true;this.naturalWidth=3840;}set src(s){if(this.onload)this.onload();}}
 const context={document:{getElementById:id=>{assert(id in elements,id);return elements[id];}},Image,
   requestAnimationFrame:()=>{}};
 vm.runInNewContext(script,context,{timeout:3000});
 assert(draws.length>0);elements.play.onclick();assert.equal(elements.play.textContent,'Play');
 elements.play.onclick();assert.equal(elements.play.textContent,'Pause');
 elements.scrub.value=String(max);elements.scrub.oninput();assert.equal(elements.play.textContent,'Play');
 for(const [mode,d] of Object.entries(data)){
   assert(elements[mode+'_live'].textContent.includes('ENDED'));
   elements['show_'+mode].onchange({target:{checked:false}});assert.equal(elements['card_'+mode].style.display,'none');
   elements['show_'+mode].onchange({target:{checked:true}});assert.equal(elements['card_'+mode].style.display,'');
 }
 elements.restart.onclick();assert.equal(elements.scrub.value,0);elements.play.onclick();
 vm.runInNewContext('tick(100);tick(350);',context,{timeout:3000});
 process.stdout.write(JSON.stringify({modes:Object.keys(data).length,controls:true,draws:draws.length}));
});'''
    for page in pages:
        document=page.read_text();Links().feed(document)
        match=re.search(r'<script id="data" type="application/json">(.*?)</script><script>(.*?)</script>',document,re.S)
        if not match:continue
        data=json.loads(match[1]);maximum=int(re.search(r'id="scrub"[^>]*max="(\d+)"',document)[1]);atlases={}
        for mode,d in data.items():
            assert len(d['times'])==maximum+1
            with (page.parent/f'{mode}_atlas.png').open('rb') as source:header=source.read(24)
            assert header[:8]==b'\x89PNG\r\n\x1a\n'
            width,height=struct.unpack('>II',header[16:24]);atlases[mode]=[width,height]
            frames=read(page.parent/f'{mode}_atlas.json')['frames']
            assert all(0<=index<frames for index in d['image_index'])
            assert width==8*480 and height>=360*int(np.ceil(frames/8))
        result=subprocess.run(['node','-e',harness],input=json.dumps({'script':match[2],'data':data,'max':maximum,'atlases':atlases}),
                              text=True,capture_output=True,check=True)
        playback.append(dict(page=str(page.relative_to(ROOT)),**json.loads(result.stdout)))
    output={'html_pages':len(pages),'local_references':references,'broken_links':broken,'playback':playback,
            'all_checks_pass':bool(pages and playback) and not broken,
            'scope':'Saved local links, atlas/frame indexing and playback JS in a mocked DOM. Not full-browser visual certification or independent physical safety validation.'}
    save(base/'review_checks.json',output)
    if not output['all_checks_pass']:raise ValueError('review check failed; see review_checks.json')
    print(f'Review checks passed: {len(pages)} pages, {references} local references, {len(playback)} playback pages',flush=True)
    return output


if __name__=='__main__':
    import argparse
    from pathlib import Path
    parser=argparse.ArgumentParser(description='Review or locally bundle saved pickup research; no new physics/API calls.')
    parser.add_argument('action',choices=('review','bundle','verify'))
    parser.add_argument('--base',type=Path,default=ROOT/'artifacts/pickup')
    parser.add_argument('--output',type=Path,help='new zip file for bundle')
    args=parser.parse_args()
    if args.action=='review':render_research_hub(args.base)
    elif args.action=='bundle':build_bundle(args.base,args.output or args.base/'review_bundle.zip')
    else:verify_review(args.base)
