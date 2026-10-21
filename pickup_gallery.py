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


TITLES = {"off": "Scene CBFs off", "all": "All clutter constrained", "once": "Jev once", "repeated": "Jev repeated", "rule": "Geometric rule", "geometry": "All objects / Jev geometry"}


def mode_payload(trace, result, indices, target_id, objects):
    clutter_columns = [i for i, o in enumerate(objects) if o["id"] != target_id]
    enabled = trace["cover_enabled"][indices]
    per_object = np.any(enabled, axis=1).reshape(len(indices), len(objects), 4).sum(axis=2)
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


def render_frames(mj, model, trace, indices, objects, target_id, *, width=480, height=360):
    data = mj.MjData(model)
    option = mj.MjvOption(); option.geomgroup[3] = 0; option.geomgroup[4] = 0
    camera = mj.MjvCamera(); camera.lookat[:] = [.35, 0., .25]
    camera.distance, camera.azimuth, camera.elevation = 1.35, 135, -35
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
                labels.append(("TARGET" if obj["id"] == target_id else obj["id"], data.geom_xpos[geom]+[0, 0, obj["size_m"][2]/2+.025]))
            add_scene_labels(scene, labels, append=True)
            for point in trace["ee_m"][np.linspace(0, index, min(index+1, 70), dtype=int)]:
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE,
                               np.array([.003, 0, 0]), point, np.eye(3).ravel(), np.array([.9, .2, .8, 1.]))
                scene.ngeom += 1
            yield renderer.render().copy()


def render_case(folder, scene, settings, demo, mj):
    request = read(folder/"initial/request.json")
    runtime = PickupRuntime(scene, settings, request, None, "off", folder/"render_unused")
    spec = runtime.scene_spec(mj, ROOT.parent/"frax", request["state"], [], [])
    spec.attach(runtime.robot_spec(mj, ROOT.parent/"frax"), prefix="", frame=spec.worldbody.add_frame())
    model = spec.compile()
    results = {mode: read(folder/mode/"result.json") for mode in TITLES if (folder/mode/"result.json").exists()}
    if not results:
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
    document = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Pickup: {html.escape(scene['id'])}</title><style>{STYLE}</style><main><a href="../index.html">← Overview</a>
    <h1>Static-clutter pickup: {html.escape(scene['id'])}</h1><p>Orange solid object = pickup target; cyan objects = fixed clutter; transparent spheres = enabled object covers; magenta = actual hand path.</p>
    <p>One task: maneuver, grasp, lift enough to verify pickup, then stop. An ended panel holds its last recorded frame while other methods continue.</p>
    <div class="controls"><button id="play">Pause</button><button id="restart">Restart</button><input id="scrub" aria-label="Simulation time" type="range" min="0" max="{len(times)-1}" value="0" step="1"><span id="clock"></span>
    <select id="speed" aria-label="Playback speed"><option value="1">1×</option><option value=".5">0.5×</option><option value=".25">0.25×</option></select></div>
    <p>Show methods: {toggles}</p><div class="cards">{''.join(cards)}</div><h2>Recorded selection updates</h2><table><tr><th>Mode</th><th>Snapshot s</th><th>Applied s</th><th>Status</th><th>Effective choices</th><th>Evidence</th></tr>{''.join(events)}</table>
    <p><a href="initial/selection.json">Initial answer/probabilities (when queried)</a> · <a href="initial/request.json">Initial request</a> · <a href="playback.json">Playback data</a></p>
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
        cards.append(f"<article><h2><a href='{scene['id']}/index.html'>{html.escape(scene['id'])} →</a></h2><table><tr><th>Method</th><th>Pickup</th><th>Unwanted contact</th><th>Pass</th><th>Mean pairs</th></tr>{rows}</table></article>")
    from run_pickup import BUDGET_PATH
    ledger = read(BUDGET_PATH) if BUDGET_PATH.exists() else {"cap_usd": "7.50", "attempts": []}
    reported = sum(float(a.get("reported_usage_estimated_usd", 0)) for a in ledger["attempts"])
    reserved = sum(float(a["reserved_usd"]) for a in ledger["attempts"])
    attempts = list(folder.glob("*/initial/attempt.json"))+list(folder.glob("*/queries/*/attempt.json"))
    save(folder/"review_results.json", {"cases": summaries, "manifest": manifest, "dataset_logical_api_attempts":len(attempts),
                                      "usage_ledger_scope":"All experiments in the approved autonomous work, not just this dataset", "usage_ledger": ledger})
    extra_links = " · ".join(f'<a href="{name}">{label}</a>' for name,label in (("verification.json","Independent verification"),("test_output.txt","Offline tests"),("research_report.json","Research analysis")) if (folder/name).exists())
    (folder/"index.html").write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Static-clutter pickup gallery</title>
    <style>{STYLE}</style><main><h1>Jev-selected CBFs<br><span class="muted">Static-clutter pickup</span></h1>
    <p>One task family: maneuver through fixed clutter, pick up the designated object, then stop. Different layouts; same original FRAX nominal torque policy and task logic. CBFpy backend, read-only upstream repositories.</p>
    <div class="notice">Open a layout for synchronized playback. Check actual lift, bilateral finger contact, unwanted collisions, and chosen covers. A plausible animation alone is not success.</div>
    <div class="cards" style="margin-top:24px">{''.join(cards)}</div>
    <h2>Scope</h2><p>Ground-truth object geometry/kinematics; no perception or hardware. Native articulated fingers and free-body target. Only finger-target contact is allowed. Original arm protections stay on in every method; OFF disables scene-object CBFs only. The arm model omits finger/contact/payload dynamics, and sampled diagnostics are not a continuous-time safety proof.</p>
    <p>This dataset: {len(attempts)} logical model attempts. Overall approved-work budget (all experiments): ${float(ledger['cap_usd']):.2f}; {len(ledger['attempts'])} logical requests; reported token-cost estimate ${reported:.6f}; conservative reservations including possible retries ${reserved:.6f}. Estimates use <a href="https://docs.typesafe.ai/models">official direct-API pricing</a>, not an invoice.</p>
    <p><a href="manifest.json">Frozen inputs/provenance</a> · <a href="review_results.json">Full results and usage ledger</a> · {extra_links}</p></main></html>''')
    print(f"Pickup gallery: {folder/'index.html'}", flush=True)


def render_research_hub(base):
    """One entry point for current evidence and explicitly retained history."""
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
