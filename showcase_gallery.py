"""Synchronized, scrubbable gallery of recorded dynamic simulations only."""

import html
import json
import os

import numpy as np

from inspect_geometry import add_scene_labels
from inspect_selection import ROOT
from run_static import load_upstream, scene_spec
from run_suite import check_manifest, digest, read, save
from sim_model import panda_spec
from static_gallery import write_apng


STYLE = """body{margin:0;background:#101722;color:#e7eef7;font:16px system-ui;line-height:1.5}
main{max-width:1500px;margin:32px auto;padding:0 24px}h1{line-height:1.2}a{color:#91d9ff}
.muted,small{color:#b3c0d0}.notice{border-left:4px solid #ffbd66;padding:12px 18px;background:#24303e}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:18px}
article{background:#1b2838;padding:18px;border-radius:12px}canvas,img{width:100%;border-radius:8px}
table{border-collapse:collapse;width:100%;margin:20px 0}td,th{padding:10px;text-align:left;border-bottom:1px solid #435266}
.controls{position:sticky;top:0;background:#101722ef;padding:16px 0;z-index:2;display:flex;align-items:center;gap:12px}
input[type=range]{flex:1;min-width:100px}button,select{font:inherit;padding:8px 14px;cursor:pointer}
.good{color:#9ce0af}.bad{color:#ffb7a6}pre{white-space:pre-wrap;font-size:13px}details{margin:18px 0}
"""


def selection_changes(events):
    """Late semantic additions/removals, excluding initial fallback replacement."""
    changes = []
    for event in events:
        if event.get("kind") != "selection" or event.get("status") != "accepted" or event["applied_at_s"] <= 0:
            continue
        for name, selected in event["policy"].items():
            previous = event["previous_policy"][name]
            if selected != previous:
                changes.append({"object": name, "at_s": event["applied_at_s"],
                                "kind": "removed" if selected is None else "added" if previous is None else "geometry changed"})
    return changes


def observed_results(summaries):
    """Describe recorded results, never hard-code preferred experiment outcomes."""
    repeated = [r for s in summaries.values() for r in s["runs"] if r["mode"] == "repeated"]
    if not repeated:
        return "No repeated-selection rollout has been recorded yet."
    successes = sum(r["task_success"] for r in repeated)
    no_contact = sum(not r["collision"]["any_sampled_scene_violation"] for r in repeated)
    descriptions = [f"Repeated selection: {successes}/{len(repeated)} pass the task criterion; {no_contact}/{len(repeated)} have no sampled scene contact."]
    drops = []
    for name, summary in summaries.items():
        result = next((r for r in summary["runs"] if r["mode"] == "repeated"), None)
        if result is None:
            continue
        changes = selection_changes(result["selection_events"])
        drops.extend(c for c in changes if c["kind"] == "removed")
        detail = ", ".join(f"{c['object']} {c['kind']} at {c['at_s']:.2f} s" for c in changes) or "no late role/geometry change"
        once = next((r for r in summary["runs"] if r["mode"] == "once"), None)
        comparison = (f" One-shot contact: {once['collision']['any_sampled_scene_violation']}; repeated contact: {result['collision']['any_sampled_scene_violation']}." if once else "")
        descriptions.append(f"{name}: {detail}.{comparison}")
    if not drops:
        descriptions.append("Jev did not remove previously selected obstacles later in these runs. Runtime removal is supported, but effective late-stage dropping is not demonstrated; conservatism remains a limitation.")
    return " ".join(descriptions)


def comparison_data(folder):
    """Apply an explicit hash-bound audit without overwriting original results."""
    summary = read(folder/"comparison.json")
    audit_path = folder/"collision_recheck.json"
    audit = read(audit_path) if audit_path.exists() else {}
    for result in summary["runs"]:
        mode = result["mode"]
        if mode in audit:
            if digest(folder/f"{mode}.npz") != audit[mode]["source_trajectory_sha256"]:
                raise ValueError("collision audit trajectory changed")
            result["collision"] = audit[mode]["collision"]
            result["task_success"] = bool(result["complete"] and result["within_2cm_at_end"]
                                          and not result["collision"]["any_sampled_scene_violation"]
                                          and not result["any_sampled_negative_h2"])
            result["collision_audit"] = "collision_recheck.json (corrected exclusion of scene mocap bodies from robot geometry)"
        if mode == "off":
            result["minimum_selected_scene_sphere_barrier_m"] = None
    return summary


def frames(mj, model, trajectory, result, indices, objects, *, width=480, height=360):
    data = mj.MjData(model)
    target = model.body("target").mocapid[0]
    object_mocaps = [model.body(f"object/{o['id']}").mocapid[0] for o in objects]
    option = mj.MjvOption(); option.geomgroup[3] = 0; option.geomgroup[4] = 0
    camera = mj.MjvCamera(); camera.lookat[:] = [.35, 0., .28]
    camera.distance, camera.azimuth, camera.elevation = 1.5, 135, -45
    with mj.Renderer(model, width=width, height=height) as renderer:
        for index in indices:
            data.qpos[:], data.qvel[:] = trajectory["qpos"][index], trajectory["qvel"][index]
            data.mocap_pos[target] = result["goal_m"]
            data.mocap_pos[object_mocaps] = trajectory["object_positions_m"][index]
            mj.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            scene = renderer.scene
            enabled = trajectory["cover_enabled"][index]
            for point, radius in zip(trajectory["cover_centers_m"][index][enabled],
                                     trajectory["cover_radii_m"][index][enabled], strict=True):
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE,
                               np.array([radius, 0, 0]), point, np.eye(3).ravel(), np.array([1., .5, .12, .20]))
                scene.ngeom += 1
            add_scene_labels(scene, [(obj["id"], pos + [0, 0, obj["size_m"][2]/2+.035])
                                     for obj,pos in zip(objects, trajectory["object_positions_m"][index], strict=True)], append=True)
            for point in trajectory["ee_m"][np.linspace(0, index, min(index+1, 100), dtype=int)]:
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE,
                               np.array([.003, 0, 0]), point, np.eye(3).ravel(), np.array([.9, .2, .8, 1.]))
                scene.ngeom += 1
            yield renderer.render().copy()


def render_case(folder, scene, settings, demo, mj):
    summary = comparison_data(folder)
    request = read(folder/"initial/request.json")
    spec = scene_spec(mj, ROOT.parent/"frax", request["state"], np.empty((0, 3)), [], moving_objects=True)
    spec.attach(panda_spec(mj, ROOT.parent/"frax", profile="frax", robot=demo.load_panda()), prefix="", frame=spec.worldbody.add_frame())
    model = spec.compile()
    width, height, columns, fps = 480, 360, 8, 8
    cards, payload = [], {}
    robot = demo.load_panda()
    predict = demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
    robot_radii = np.asarray(robot.link_collision_data(robot.default_configuration)[1])
    for result in summary["runs"]:
        mode = result["mode"]
        with np.load(folder/f"{mode}.npz") as trajectory:
            indices = np.unique(np.linspace(0, len(trajectory["qpos"])-1,
                                           round(settings["duration_s"]*fps)+1, dtype=int))
            atlas_path = folder/f"{mode}_atlas.png"
            if not atlas_path.exists():
                atlas = np.zeros((int(np.ceil(len(indices)/columns))*height, columns*width, 3), dtype=np.uint8)
                for number, frame in enumerate(frames(mj, model, trajectory, result, indices, scene["objects"])):
                    y, x = number//columns*height, number%columns*width
                    atlas[y:y+height, x:x+width] = frame
                write_apng(atlas_path, [atlas], frame_count=1, fps=1)
                last = len(indices)-1; y,x=last//columns*height,last%columns*width
                write_apng(folder/f"{mode}_final.png", [atlas[y:y+height,x:x+width]], frame_count=1, fps=1)
                print(f"Rendered {scene['id']}/{mode}: {len(indices)} synchronized frames", flush=True)
            physical = trajectory["object_distance_m"][indices].min(axis=1)
            if (folder/f"{mode}_collision_audit.npz").exists():
                with np.load(folder/f"{mode}_collision_audit.npz") as audit:
                    physical = audit["object_distance_m"][indices].min(axis=1)
            if "scene_h2_m" in trajectory.files:
                scene_barriers = trajectory["scene_h2_m"][indices]
            else:
                # Recover raw sphere clearances for early baseline traces that
                # predate h2 trace logging; no new controller/physics rollout.
                link_centers = np.asarray(predict(trajectory["qpos"][indices, :7]))
                clearance = (np.linalg.norm(link_centers[:, :, None]-trajectory["cover_centers_m"][indices, None], axis=-1)
                             - robot_radii[None, :, None]-trajectory["cover_radii_m"][indices, None])
                clearance = np.where(trajectory["cover_enabled"][indices, None], clearance, 1.)
                scene_barriers = clearance.min(axis=(1, 2))
            payload[mode] = {
                "times": trajectory["time_s"][indices].tolist(),
                "counts": trajectory["enabled_scene_pairs"][indices].astype(int).tolist(),
                "enabled": trajectory["cover_enabled"][indices].reshape(len(indices), len(scene["objects"]), 4).sum(axis=2).tolist(),
                "distance_mm": None if physical is None else (physical*1000).tolist(),
                "h2_mm": (scene_barriers*1000).tolist(),
                "error_mm": (np.linalg.norm(trajectory["ee_m"][indices]-settings["goal_m"], axis=1)*1000).tolist(),
                "objects": [o["id"] for o in scene["objects"]],
            }
        collision = result["collision"]["any_sampled_scene_violation"]
        minimum = min(result["collision"]["minimum_distance_m_by_object"].values())*1000
        barrier = result["minimum_selected_scene_sphere_barrier_m"]
        barrier_text = "not enabled" if barrier is None else f"{barrier*1000:.3f} mm"
        title = {"off":"Scene CBFs off", "all":"All objects", "once":"Jev once", "repeated":"Jev repeated"}[mode]
        mean = result["enabled_scene_pair_counts"]["mean"]
        cards.append(f'''<article><h2>{title}</h2><canvas id="{mode}" width="480" height="360"></canvas>
          <div id="{mode}_live"></div><hr>
          <p class="{'bad' if collision else 'good'}">Whole-run physical contact: {'DETECTED' if collision else 'none detected'}</p>
          <p>Final goal error: {result['final_position_error_m']*1000:.2f} mm<br>
          Minimum physical clearance: {minimum:.2f} mm<br>Minimum enabled scene barrier: {barrier_text}<br>
          Mean enabled scene pairs: {mean:.1f} / {result['allocated_scene_pair_count']} allocated<br>
          Task criterion passed: {result['task_success']}</p>
          <a href="{mode}.npz">Recorded trajectory</a> · <a href="{mode}_final.png">Final frame</a></article>''')
    events = next((r["selection_events"] for r in summary["runs"] if r["mode"] == "repeated"), [])
    case_observation = observed_results({scene["id"]: summary})
    rows = []
    for e in events:
        if e["kind"] != "selection":
            continue
        policy = ", ".join(f"{k}: {v or 'omit'}" for k,v in e.get("policy", {}).items())
        snapshot = e.get("snapshot_at_s")
        rows.append(f"<tr><td>{snapshot}</td><td>{e['applied_at_s']:.3f}</td><td>{e.get('elapsed_sdk_s')}</td>"
                    f"<td>{html.escape(e['status'])}</td><td>{html.escape(policy)}</td><td><a href='{html.escape(e['source'])}'>input/output</a></td></tr>")
    event_table = "".join(rows)
    save(folder/"playback.json", payload)
    document = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
      <title>{html.escape(scene['title'])}</title><style>{STYLE}</style><main>
      <a href="../index.html">← Showcase overview</a><h1>{html.escape(scene['title'])}</h1>
      <p>Cyan = physical objects · orange = enabled keep-out covers · red = goal · magenta = recorded hand path.</p>
      <p>Every panel keeps the original nominal policy and robot protections. Only scene-object selection differs.
      ONCE still tracks known object poses/velocities; it freezes only Jev's role/geometry choices. REPEATED makes fresh asynchronous choices.</p>
      <div class="controls"><button id="play">Pause</button><button id="restart">Restart</button>
      <input id="scrub" type="range" min="0" max="{round(settings['duration_s']*fps)}" value="0" step="1" aria-label="Simulation time">
      <span id="clock"></span><select id="speed" aria-label="Playback speed"><option value="1">1×</option><option value=".5">0.5×</option><option value=".25">0.25×</option></select></div>
      <div class="cards">{''.join(cards)}</div>
      <h2>Repeated Jev updates</h2><p>Initial answer is shared with ONCE. New choices are applied only after an RD2 admissibility check.
      Errors/review keep previous choices. No confidence cutoff or oracle repair. SDK latency is wall time; apply time is simulation time.</p>
      <p class="notice">{html.escape(case_observation)}</p>
      <table><tr><th>Snapshot (s)</th><th>Applied (s)</th><th>SDK seconds</th><th>Status</th><th>Resulting choices</th><th>Evidence</th></tr>{event_table}</table>
      <details><summary>Important limitations and raw results</summary><p>The plant is idealized and matched to FRAX, not calibrated hardware.
      Object motion is prescribed, piecewise constant velocity. The CBF includes obstacle velocity, but acceleration jumps, perception error,
      arbitrary latency, and infeasibility have no formal safety guarantee. A small empirical inequality guard addresses sampled-control error.
      Fixed slots mean fewer enabled rows, NOT smaller allocated QPs. These are two selected fixtures, not a statistical benchmark.</p>
      <p><a href="comparison.json">Original results</a> · <a href="review_results.json">Results with explicit collision audit</a> ·
      <a href="playback.json">Playback data</a></p></details>
      <script id="data" type="application/json">{json.dumps(payload, allow_nan=False).replace('</', '<\\/')}</script>
      <script>
      const data=JSON.parse(document.getElementById('data').textContent), images={{}}, scrub=document.getElementById('scrub');
      let playing=true, frame=0, previous=0, carry=0;
      function draw(){{
        document.getElementById('clock').textContent=(frame/{fps}).toFixed(2)+' s';
        for(const [mode,d] of Object.entries(data)){{
          const image=images[mode], canvas=document.getElementById(mode), i=Math.min(frame,d.times.length-1);
          if(image&&image.complete&&image.naturalWidth) canvas.getContext('2d').drawImage(image,(i%{columns})*480,Math.floor(i/{columns})*360,480,360,0,0,480,360);
          const objects=d.objects.map((name,j)=>name+': '+(d.enabled[i][j] ? d.enabled[i][j]+' spheres' : 'unconstrained')).join(' · ');
          const distance=d.distance_mm?' · physical clearance '+d.distance_mm[i].toFixed(1)+' mm':'';
          const barrier=d.counts[i]?' · scene barrier '+d.h2_mm[i].toFixed(2)+' mm':' · scene CBFs off';
          document.getElementById(mode+'_live').textContent='t='+d.times[i].toFixed(2)+' s · goal error '+d.error_mm[i].toFixed(1)+' mm'+distance+barrier+' · '+d.counts[i]+' enabled pairs — '+objects;
        }}
      }}
      for(const mode of Object.keys(data)){{images[mode]=new Image();images[mode].onload=draw;images[mode].src=mode+'_atlas.png';}}
      function button(){{document.getElementById('play').textContent=playing?'Pause':'Play';}}
      document.getElementById('play').onclick=()=>{{playing=!playing;button();}};
      document.getElementById('restart').onclick=()=>{{frame=0;scrub.value=0;carry=0;draw();}};
      scrub.oninput=()=>{{frame=Number(scrub.value);playing=false;button();draw();}};
      function tick(now){{
        const elapsed=previous?Math.min(now-previous,250):0;previous=now;
        if(playing){{carry+=elapsed*Number(document.getElementById('speed').value);while(carry>=1000/{fps}){{carry-=1000/{fps};frame=(frame+1)%(Number(scrub.max)+1);}}scrub.value=frame;draw();}}
        requestAnimationFrame(tick);
      }}
      draw();requestAnimationFrame(tick);
      </script></main></html>'''
    (folder/"index.html").write_text(document)
    save(folder/"review_results.json", summary)
    return summary


def render_gallery(folder):
    manifest = check_manifest(folder)
    settings = read(folder/"scenes.json")
    demo, utils = load_upstream(ROOT.parent/"frax", ROOT.parent/"cbfpy")
    summaries, sections = {}, []
    for scene in settings["scenes"]:
        case = folder/scene["id"]
        if not (case/"comparison.json").exists():
            continue
        summary = render_case(case, scene, settings, demo, utils.mujoco)
        summaries[scene["id"]] = summary
        rows = []
        for result in summary["runs"]:
            rows.append(f"<tr><td>{result['mode']}</td><td>{result['collision']['any_sampled_scene_violation']}</td>"
                        f"<td>{result['task_success']}</td><td>{result['final_position_error_m']*1000:.2f}</td>"
                        f"<td>{result['enabled_scene_pair_counts']['mean']:.1f}</td></tr>")
        sections.append(f"<article><h2><a href='{scene['id']}/index.html'>{html.escape(scene['title'])} →</a></h2>"
                        f"<p>Open synchronized playback; pause, scrub time, or slow it down.</p><table><tr><th>Mode</th><th>Contact</th><th>Pass</th><th>Final mm</th><th>Mean enabled pairs</th></tr>{''.join(rows)}</table></article>")
    calls = [dict(path=str(p.relative_to(folder)), **read(p)) for p in folder.rglob("attempt.json")]
    save(folder/"showcase_results.json", {"cases": summaries, "model_calls": calls, "manifest": manifest})
    history_links = []
    for target, label in ((ROOT/"artifacts/suite/panda_static_v2/index.html", "Original unmodified static results"),
                          (ROOT/"artifacts/diagnosis/static_corrected_v1/hand_path/comparison.json", "Corrected static hand-path audit")):
        if target.exists():
            history_links.append(f'<a href="{os.path.relpath(target, folder)}">{label}</a>')
    verification_links = [f'<a href="{name}">{label}</a>' for name,label in
                          (("verification.json", "Verification checks/scope"), ("test_output.txt", "Offline test output"))
                          if (folder/name).exists()]
    (folder/"index.html").write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
      <title>Jev-selected CBFs — simulation showcase</title><style>{STYLE}</style><main>
      <h1>Jev-selected CBFs<br><span class="muted">Dynamic simulation showcase</span></h1>
      <p>Task-conditioned sphere-cover selection, live asynchronous Jev updates, and CBFpy filtering around the original FRAX nominal policy.
      Ground-truth objects and kinematics; no cameras or hardware.</p>
      <div class="notice">Start with either scene below. All four panels share the same simulation clock.
      Watch an initially distant object approach: does repeated selection add protection before contact, then drop it as the object leaves?
      Contact and barrier diagnostics—not visual appearance alone—determine the reported outcome.</div>
      <div class="cards" style="margin-top:24px">{''.join(sections)}</div>
      <h2>What was corrected</h2><p>The original XML and FRAX had different tool inertia, damping, and rotor inertia.
      The new idealized plant uses FRAX parameters in memory; native dynamics parity is checked independently.
      Small remaining sampled-control error is addressed by a 0.05 m/s² scene inequality guard, without changing cover geometry or CBF gains.
      The six-scene static correction audit has 12 constrained runs with nonnegative sampled barriers; blocked goals remain visible failures.</p>
      <p><a href="dynamics_parity.json">Native dynamics parity</a> · {' · '.join(history_links)}</p>
      <h2>Scope and evidence</h2><p>Model SDK attempts: {len(calls)} / {manifest['maximum_logical_sdk_invocations']} planned maximum.
      Every initial answer is shared by once/repeated. Requests, failures, probabilities, apply times, and trajectories are retained.
      No scene/prompt tuning after model answers. Previous policy is retained on failed/review/inadmissible updates; this is not a certified fallback.</p>
      <p><strong>Observed results:</strong> {html.escape(observed_results(summaries))}</p>
      <p>These two fixtures are a software proof of concept, not evidence of general safety or statistical superiority.
      Fewer enabled constraints do not currently shrink the fixed-capacity QP. No real-time, perception-uncertainty, or hardware guarantee.</p>
      <p><a href="manifest.json">Frozen setup and design choices</a> · <a href="showcase_results.json">Full review results and call ledger</a></p>
      <p>{' · '.join(verification_links)}</p>
      </main></html>''')
    print(f"Showcase: {folder/'index.html'}", flush=True)
