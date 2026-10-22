"""Synchronized five-method RGBD pickup inspection; saved evidence only."""

import html
import base64
import hashlib
import inspect
import json
import os
from pathlib import Path

import numpy as np

from adaptive_geometry import contains, validate_cover
from camera_io import ReplaySource, deproject
from inspect_camera import depth_image
from inspect_selection import ROOT
from pickup_gallery import render_frames
from rgbd_pickup import METHODS
from run_rgbd_pickup import plant_model, verify_inputs
from run_static import load_upstream
from run_suite import digest, read, save
from showcase_gallery import STYLE
from static_gallery import write_apng


def independent_coverage(case):
    """Native geometry is used here only, after inputs/decisions are frozen."""
    scene, estimate = read(case/'scene.json'), read(case/'estimate.json')
    rng, rows = np.random.default_rng(17), []
    with np.load(case/'observation/beliefs.npz') as beliefs:
        for obj in estimate['objects']:
            native = min(scene['objects'], key=lambda o: np.linalg.norm(np.array(o['center_m'])-obj['center_m']))
            half = np.array(native['size_m'])/2
            points = rng.uniform(-half, half, (40_000, 3))
            if native['shape'] == 'cylinder':
                points = points[np.linalg.norm(points[:, :2], axis=1) <= half[0]]
            points += native['center_m']
            cells = beliefs[obj['id']+'_cells_m']
            cloud = beliefs[obj['id']+'_points_m']
            side = float(beliefs[obj['id']+'_side_m'])
            fits = []
            for candidate in obj['candidates']:
                centers = np.array(candidate['centers_object_m'])+obj['center_m']
                radii = np.array(candidate['radii_m'])
                slack = validate_cover(cells, side, centers, radii, estimate['fit_limits']['padding_m'])
                fits.append(dict(candidate_id=candidate['id'], spheres=len(radii), whole_belief_enclosed=True,
                                 minimum_cell_slack_m=slack, observed_point_fraction_covered=float(contains(cloud, centers, radii).mean()),
                                 independently_sampled_true_volume_fraction_covered=float(contains(points, centers, radii).mean()), native_samples=len(points)))
            rows.append(dict(observed_id=obj['id'], evaluation_only_native_id=native['id'], candidates=fits))
    result = dict(objects=rows, scope='Observed-point fit, validated estimated-cell enclosure and independently sampled true-object volume coverage are distinct. Sampling is not proof of full native coverage. Evaluation matching/geometry never enters control or candidate generation.')
    save(case/'coverage.json', result)
    return result


def camera_inspection(case):
    estimate = read(case/'estimate.json')
    calibration = read(case/'observation/calibration.json')
    source = ReplaySource(case/'observation/recording')
    figures = []
    try:
        while True:
            try:
                frame = source.read(.1)
            except StopIteration:
                break
            name = frame.camera_id
            write_apng(case/f'{name}_rgb.png', [frame.rgb], frame_count=1, fps=1)
            write_apng(case/f'{name}_depth.png', [depth_image(frame.depth_m)], frame_count=1, fps=1)
            masks, _ = source.annotations(frame)
            mask_image = frame.rgb.copy()
            for key, mask in masks.items():
                color = np.array([255,110,70] if key.startswith('target') else [70,225,215])
                mask_image[mask] = (.5*mask_image[mask]+.5*color).astype(np.uint8)
            write_apng(case/f'{name}_masks.png', [mask_image], frame_count=1, fps=1)
            T = np.array(calibration[name]['T_robot_optical'])
            # Project 3D great-circle samples, never screen-space guessed circles.
            lines = []
            angles = np.linspace(0., 2*np.pi, 65)
            for obj in estimate['objects']:
                candidate = obj['candidates'][-1]
                for offset, radius in zip(candidate['centers_object_m'], candidate['radii_m'], strict=True):
                    center = np.array(offset)+obj['center_m']
                    for axes in ((0, 1), (0, 2), (1, 2)):
                        world = np.tile(center, (len(angles), 1))
                        world[:, axes[0]] += radius*np.cos(angles)
                        world[:, axes[1]] += radius*np.sin(angles)
                        optical = (world-T[:3, 3])@T[:3, :3]
                        if np.any(optical[:, 2] <= .01):
                            continue
                        u = frame.intrinsics.fx*optical[:, 0]/optical[:, 2]+frame.intrinsics.cx
                        v = frame.intrinsics.fy*optical[:, 1]/optical[:, 2]+frame.intrinsics.cy
                        coordinates = ' '.join(f'{x:.2f},{y:.2f}' for x, y in zip(u, v, strict=True))
                        lines.append(f'<polyline points="{coordinates}" fill="none" stroke="#ffd166" stroke-width=".6"/>')
            width, height = frame.intrinsics.width, frame.intrinsics.height
            embedded = base64.b64encode((case/f'{name}_rgb.png').read_bytes()).decode()
            (case/f'{name}_overlay.svg').write_text(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}"><image href="data:image/png;base64,{embedded}" width="{width}" height="{height}"/>{"".join(lines)}</svg>')
            figures.append(f'<article><h3>{html.escape(name)}</h3><img src="{name}_overlay.svg" alt="Calibrated 3D sphere projections on RGB"><img src="{name}_masks.png" alt="RGB-estimated component masks"><img src="{name}_depth.png" alt="Metric depth, display scaled"></article>')
    finally:
        source.close()
    return ''.join(figures)


def cost(case):
    path = case/'initial/selection.json'
    if not path.exists():
        return dict(calls=0, reported_estimated_usd=0., new_calls=0, new_estimated_usd=0., per_method={})
    report = read(path)
    per_method = {}
    for method in METHODS:
        queries = sorted((case/method/'queries').glob('*/selection.json'))
        per_method[method] = dict(calls=len(queries), estimated_usd=sum(read(p).get('response', {}).get('usage', {}).get('input_tokens', 0)*.042/1_000_000 for p in queries))
    return dict(calls=1, reported_estimated_usd=report.get('response', {}).get('usage', {}).get('input_tokens', 0)*.042/1_000_000,
                new_calls=sum(p['calls'] for p in per_method.values()), new_estimated_usd=sum(p['estimated_usd'] for p in per_method.values()),
                per_method=per_method, elapsed_s=report.get('elapsed_seconds'), note='One startup response shared across three ablations; do not sum it three times. Per-method repeated costs are additional.')


def schedule_table(folder, *, scene=None):
    path = folder/'schedule_comparison.json'
    if not path.exists():
        return ''
    data = read(path)
    rows = []
    for row in data['rows']:
        if scene is not None and row['scene'] != scene:
            continue
        old, new = row['startup_only'], row['repeated']
        rows.append(f'<tr><td>{html.escape(row["scene"])}</td><td>{METHODS[row["method"]]}</td><td>{old["safe_pickup"]} → {new["safe_pickup"]}</td><td>{old["unwanted_collision"]} → {new["unwanted_collision"]}</td><td>{old["cbf_counts"]["clutter_pairs_mean"]:.1f} → {new["cbf_counts"]["clutter_pairs_mean"]:.1f}</td><td>{row["new_logical_requests"]}</td><td>{row["changes_after_startup"]}</td><td>{row["selection_statuses"].get("rejected",0)}</td><td>{len(row.get("invalid_responses",[]))}</td><td>{row["late_responses"]}</td></tr>')
    return '<h2>Startup-only → repeated Jev</h2><table><tr><th>Scene</th><th>Condition</th><th>Safe pickup</th><th>Collision</th><th>Mean clutter pairs</th><th>New requests</th><th>Accepted policy changes after startup</th><th>Rejected selections</th><th>Invalid responses, including late</th><th>Late responses</th></tr>'+''.join(rows)+'</table><p>One run per schedule and condition. Camera/worker timing varies; unchanged installed policies do not establish a causal benefit from querying again. Reused non-Jev baselines are not new trials. Invalid and late answers are retained without re-querying; detailed errors are in the paired comparison evidence.</p>'


def render_case(case, robot, mj, settings):
    repeated = read(case.parent/'manifest.json').get('query_schedule') == 'completion_driven_repeated'
    scene = read(case/'scene.json')
    clutter_count = sum(obj['id'] != scene['target_id'] for obj in scene['objects'])
    initial_estimate = read(case/'estimate.json')
    object_labels = {}
    for native in scene['objects']:
        observed = min(initial_estimate['objects'],key=lambda o:np.linalg.norm(np.array(o['center_m'])-native['center_m']))
        object_labels[native['id']] = 'T' if observed['id']=='observed_target' else 'C'+str(int(observed['id'].split('_')[-1])+1)
    model = plant_model(scene, robot, mj)
    results = {m: read(case/m/'result.json') for m in METHODS if (case/m/'result.json').exists()}
    duration = max((r['simulation_seconds'] for r in results.values()), default=0.)
    fps, width, height, columns = 6, 400, 300, 8
    camera_options = dict(lookat=[.41,.08,.23],distance=1.65,azimuth=135,elevation=-35)
    times = np.arange(int(np.ceil(duration*fps))+1)/fps
    cards, payload, rows = [], {}, []
    api = cost(case)
    for method, title in METHODS.items():
        if repeated:
            title += ' · reused baseline' if method in ('off', 'all_single') else ' · repeated Jev'
        if method not in results:
            attempt = read(case/method/'attempt.json') if (case/method/'attempt.json').exists() else dict(status='pending')
            cards.append(f'<article><h2>{title}</h2><p class="bad">No completed rollout: {html.escape(str(attempt))}</p></article>')
            continue
        result, path = results[method], case/method/'trajectory.npz'
        if result.get('startup_refusal'):
            rows.append(f'<tr><td>{title}</td><td>Unmeasured</td><td>False</td><td>False</td><td>No actuation</td><td>No rollout / no additional call</td></tr>')
            cards.append(f'<article><h2>{title}</h2><p class="bad">Rejected before actuation</p><p>Pickup failed. No collision-free outcome or successful trajectory is claimed.</p><p>{html.escape(result["startup"].get("error", ""))}</p><a href="{method}/startup.json">Startup check</a> · <a href="{method}/result.json">Outcome</a></article>')
            continue
        if digest(path) != result['trajectory_sha256']:
            raise ValueError('trajectory fingerprint mismatch')
        with np.load(path) as trace:
            indices = np.minimum(np.searchsorted(trace['time_s'], times), len(trace['time_s'])-1)
            unique, inverse = np.unique(indices, return_inverse=True)
            enabled = np.any(trace['cover_enabled'][indices], axis=1)
            payload[method] = dict(image_index=inverse.tolist(), phase=trace['phase'][indices].tolist(),
                                   pairs=trace['enabled_scene_pairs'][indices].tolist(), spheres=enabled.sum(1).tolist(),
                                   bottom_mm=(trace['target_bottom_m'][indices]*1000).tolist(),
                                   estimated_bottom_mm=(trace['estimated_target_bottom_m'][indices]*1000).tolist(),
                                   bilateral=trace['bilateral_finger_contact'][indices].tolist())
            fingerprint = dict(trajectory_sha256=result['trajectory_sha256'], renderer_sha256=digest(ROOT/'rgbd_gallery.py'),
                               shared_renderer_sha256=hashlib.sha256(inspect.getsource(render_frames).encode()).hexdigest(),
                               model_builder_sha256=digest(ROOT/'sim_model.py'),camera_options=camera_options,
                               object_labels=object_labels,
                               frames=len(unique), width=width, height=height)
            atlas_path, meta = case/f'{method}_atlas.png', case/f'{method}_atlas.json'
            if not atlas_path.exists() or not meta.exists() or read(meta) != fingerprint:
                atlas = np.zeros((int(np.ceil(len(unique)/columns))*height, columns*width, 3), np.uint8)
                for n, image in enumerate(render_frames(mj, model, trace, unique, scene['objects'], scene['target_id'], width=width, height=height, camera_options=camera_options, object_labels=object_labels)):
                    y, x = n//columns*height, n%columns*width
                    atlas[y:y+height, x:x+width] = image
                write_apng(atlas_path, [atlas], frame_count=1, fps=1)
                write_apng(case/f'{method}_final.png', [image], frame_count=1, fps=1)
                save(meta, fingerprint)
                print('Rendered', case.name, method, len(unique), 'frames', flush=True)
        count = result['cbf_counts']
        shared_api = api['reported_estimated_usd'] if method not in ('off', 'all_single') else 0.
        additional = api['per_method'].get(method, dict(calls=0, estimated_usd=0.))
        provider_text = f'${shared_api:.6f} shared startup + ${additional["estimated_usd"]:.6f} repeated ({additional["calls"]} new requests)'
        rows.append(f'<tr><td>{title}</td><td>{result["unwanted_collision"]}</td><td>{result["pickup_success"]}</td><td>{result["safe_pickup"]}</td><td>{count["clutter_pairs_mean"]:.0f} / {count["clutter_pairs_peak"]}</td><td>{provider_text} + {result["rollout_wall_seconds_including_initialization"]:.1f}s</td></tr>')
        p95 = result['control_wall_ms_after_first']['p95']
        elapsed = result['rollout_wall_seconds_including_initialization']-result['initialization_wall_seconds']
        achieved = result['steps']/max(elapsed, .0001)
        cards.append(f'''<article id="card_{method}"><h2>{title}</h2><canvas id="{method}" width="{width}" height="{height}"></canvas><p id="{method}_live"></p>
          <p class="{'good' if result['safe_pickup'] else 'bad'}">Unwanted collision: {result['unwanted_collision']} · Pickup: {result['pickup_success']} · Safe pickup: {result['safe_pickup']}</p>
          <p>Clutter objects mean/peak: {count['clutter_objects_mean']:.1f}/{count['clutter_objects_peak']}<br>Clutter spheres mean/peak: {count['clutter_spheres_mean']:.1f}/{count['clutter_spheres_peak']}<br>Clutter CBF pairs mean/peak: {count['clutter_pairs_mean']:.0f}/{count['clutter_pairs_peak']}<br>Target pairs: {count['mandatory_target_pairs']} · intrinsic protections: {count['intrinsic_barriers']}<br>Allocated scene rows: {count['allocated_scene_pairs']} · total barrier rows: {count['allocated_total_barriers']}<br>Allocated CBFpy QP inequality rows: {count['allocated_total_barriers']+14} (includes 14 actuator bounds; before elastic backend bookkeeping).</p>
          <p>1 ms simulation step; achieved overall warm loop: {achieved:.0f} Hz wall time.<br>Measured step p95: {result['warm_loop']['p95_ms']:.2f} ms; {result['warm_loop']['fraction_above_1ms']*100:.1f}% exceed 1 ms.<br>Control-only p95: {p95:.2f} ms. Sensor worker mean: {result['sensor_processing_s']['mean'] or 0:.3f} s.<br>Runtime incl. initialization: {result['rollout_wall_seconds_including_initialization']:.1f} s.<br>Provider estimate: {provider_text}.<br>Model proposal accepted: {result['model_selection_accepted'] if method not in ('off','all_single') else 'not applicable'}.</p>
          <a href="{method}/result.json">Metrics</a> · <a href="{method}/trajectory.npz">Trajectory</a> · <a href="{method}_final.png">Final frame</a></article>''')
    inspection = camera_inspection(case)
    coverage = independent_coverage(case)
    geometry_rows = ''.join(f'<tr><td>{o["observed_id"]}</td><td>{c["candidate_id"]}</td><td>{100*c["observed_point_fraction_covered"]:.2f}%</td><td>{c["whole_belief_enclosed"]}</td><td>{100*c["independently_sampled_true_volume_fraction_covered"]:.2f}%</td></tr>' for o in coverage['objects'] for c in o['candidates'])
    estimate = read(case/'estimate.json')
    fit_total = sum(o['fitting']['elapsed_s'] for o in estimate['objects'])
    fit_overruns = sum(o['fitting']['budget_overrun'] for o in estimate['objects'])
    save(case/'playback.json', dict(times_s=times.tolist(), methods=payload))
    encoded = json.dumps(payload, allow_nan=False).replace('</', '<\\/')
    events = []
    for method, r in results.items():
        for e in r['selection_events']:
            if e.get('kind') == 'selection':
                events.append(f'<tr><td>{method}</td><td>{html.escape(e["status"])}</td><td>{html.escape(str(e.get("policy")))}</td><td>{html.escape(str(e.get("error", "")))}</td></tr>')
    (case/'index.html').write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{case.name} RGBD pickup</title><style>{STYLE}</style><main>
    <a href="../index.html">← Three-scene gallery</a><h1>{case.name.capitalize()} · {clutter_count} clutter obstacle{'s' if clutter_count != 1 else ''} + pickup target</h1><p>Configured obstacle counts exclude the target. RGBD estimated {len(initial_estimate['objects'])-1} clutter tracks; configured counts are shown for inspection and are not passed into perception/control. Orange T = target; cyan C1/C2/C3 = observed_00/01/02 clutter tracks; translucent gold = enabled covers; magenta = hand path. Display matching to the native plant is offline evaluation only. Frames share one simulation clock; ended methods hold their last frame.</p>
    <div class="controls"><button id="play">Pause</button><button id="restart">Restart</button><input id="scrub" aria-label="Simulation time" type="range" min="0" max="{len(times)-1}" value="0"><span id="clock"></span><select id="speed" aria-label="Playback speed"><option value="1">1×</option><option value=".5">0.5×</option><option value=".25">0.25×</option></select></div>
    <div class="cards">{''.join(cards)}</div><table><tr><th>Method</th><th>Collision</th><th>Pickup</th><th>Safe pickup</th><th>Clutter pairs mean/peak</th><th>Cost: provider + runtime</th></tr>{''.join(rows)}</table>
    <p>Pickup and collision are independently scored from native simulation after control. The observer's stop flag is separate. A collision-free stall is a failed pickup. Negative modeled barriers remain failures in detailed task_success even if the physical safe-pickup conjunction passes. QP allocation stays fixed.</p>
    {schedule_table(case.parent, scene=case.name)}
    <details><summary>RGBD, calibrated projected covers and geometry inspection</summary><p>Shared initial survey/estimation: {estimate['elapsed_s']*1000:.1f} ms; sphere fitting total: {fit_total*1000:.1f} ms; per-object 75 ms cooperative-budget overruns: {fit_overruns}. These cached costs are shared by all methods, not recomputed per control step.</p><div class="cards">{inspection}</div><table><tr><th>Observed ID</th><th>Candidate</th><th>Observed points covered</th><th>Every whole belief cell enclosed</th><th>Independent native-volume samples covered</th></tr>{geometry_rows}</table><p>Native-volume percentages are sampled diagnostics, not proof of full-object containment.</p><p><a href="estimate.json">Observed inventory, candidates and fit timings</a> · <a href="coverage.json">Observed fit / estimated-cell enclosure / independent true-volume coverage</a> · <a href="observation/recording/recording.json">Raw RGBD recording</a> · <a href="observation/calibration.json">Idealized calibration</a></p><p>Classical color masks are estimated from RGB. Static clutter uses the initial survey; no native inventory, instance labels, dimensions or target pose enters control. Convex-footprint extrusion, 3 mm padding and visible-extent target tracking are provisional. Missed objects and unseen regions are unknown. This designed demo does not validate hardware or general perception.</p></details>
    <h2>Selection admission</h2><table><tr><th>Method</th><th>Status</th><th>Installed/retained policy</th><th>Reason</th></tr>{''.join(events)}</table><p><a href="qualification.json">Pre-model scene qualification</a> · <a href="initial/request.json">Frozen model input</a> · <a href="initial/selection.json">Raw answer/probabilities</a></p>
    <script>const data={encoded},images={{}}, columns={columns}, w={width},h={height},fps={fps};let frame=0,playing=true,previous=0,carry=0;
    const scrub=document.getElementById('scrub');function draw(){{document.getElementById('clock').textContent=(frame/fps).toFixed(2)+' s';for(const [mode,d] of Object.entries(data)){{let n=d.image_index[frame],c=document.getElementById(mode).getContext('2d');if(images[mode].complete)c.drawImage(images[mode],n%columns*w,Math.floor(n/columns)*h,w,h,0,0,w,h);document.getElementById(mode+'_live').textContent=d.phase[frame]+' · '+d.pairs[frame]+' pairs / '+d.spheres[frame]+' spheres · true bottom '+d.bottom_mm[frame].toFixed(1)+' mm · estimated '+d.estimated_bottom_mm[frame].toFixed(1)+' mm · bilateral '+d.bilateral[frame];}}}}
    for(const mode of Object.keys(data)){{images[mode]=new Image();images[mode].onload=draw;images[mode].src=mode+'_atlas.png';}}
    document.getElementById('play').onclick=()=>{{playing=!playing;document.getElementById('play').textContent=playing?'Pause':'Play';}};document.getElementById('restart').onclick=()=>{{frame=0;carry=0;scrub.value=0;draw();}};
    scrub.oninput=()=>{{frame=Number(scrub.value);playing=false;document.getElementById('play').textContent='Play';draw();}};
    function tick(now){{let elapsed=previous?Math.min(now-previous,250):0;previous=now;if(playing){{carry+=elapsed*Number(document.getElementById('speed').value);while(carry>=1000/fps){{carry-=1000/fps;frame=(frame+1)%(Number(scrub.max)+1);}}scrub.value=frame;draw();}}requestAnimationFrame(tick);}}requestAnimationFrame(tick);</script></main></html>''')
    return results, api


def gallery(folder):
    manifest = verify_inputs(folder, source=False)
    settings = read(folder/'settings.json')
    demo, utils = load_upstream(ROOT.parent/'frax', ROOT.parent/'cbfpy')
    robot, cards, summary = demo.load_panda(), [], {}
    total_cost = 0.
    new_cost, new_calls = 0., 0
    for name in manifest['scene_ids']:
        results, api = render_case(folder/name, robot, utils.mujoco, settings)
        summary[name] = results; total_cost += api['reported_estimated_usd']
        new_cost += api['new_estimated_usd']; new_calls += api['new_calls']
        rows = ''.join(f'<tr><td>{METHODS[m]}</td><td>{r["unwanted_collision"] if r["unwanted_collision"] is not None else "Unmeasured"}</td><td>{r["pickup_success"]}</td><td>{r["safe_pickup"]}</td><td>{str(round(r["cbf_counts"]["clutter_pairs_mean"])) if "cbf_counts" in r else "Refused"}</td></tr>' for m, r in results.items())
        scene = read(folder/name/'scene.json')
        clutter_count = sum(obj['id'] != scene['target_id'] for obj in scene['objects'])
        cards.append(f'<article><h2><a href="{name}/index.html">{name.capitalize()} · {clutter_count} clutter obstacle{"s" if clutter_count != 1 else ""} + pickup target → synchronized playback</a></h2><table><tr><th>Method</th><th>Collision</th><th>Pickup</th><th>Safe pickup</th><th>Clutter CBFs</th></tr>{rows}</table></article>')
    save(folder/'review_results.json', dict(cases=summary, manifest=manifest, provider_estimated_usd=total_cost+new_cost,
         startup_provider_estimated_usd=total_cost, additional_repeated_provider_estimated_usd=new_cost,
         additional_repeated_calls=new_calls))
    (folder/'index.html').write_text(f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RGBD adaptive-cover pickup</title><style>{STYLE}</style><main><h1>RGBD pickup through static clutter</h1><p>Easy / medium / hard · five agreed methods · original FRAX nominal torque policy and CBFpy filter.</p><div class="notice">Designed simulation demonstration. Each scene was qualified by a real off-mode collision and an offline adaptive pickup witness before new Jev answers. No assumed winner. Read the failed outcomes alongside successful pickups.</div><div class="cards">{''.join(cards)}</div><p>All methods share calibrated RGBD observations, estimated target/grasp pose, task logic and contact assumptions. Off disables only scene-object CBFs. Only finger-target contact and ordinary support contact are intended. Intrinsic robot protections remain active. Fixed allocated QP rows do not shrink when fewer pairs are enabled.</p><p>One actual startup answer is shared by the three Jev conditions per scene: {len(manifest['scene_ids'])} logical requests, estimated ${total_cost:.6f} input-token cost. Runtime, local fitting, control timings and provider estimates are shown separately; there is no combined cost score or real-time certification. Repeated scheduling is a separate diagnostic.</p><p><a href="manifest.json">Frozen provenance</a> · <a href="review_results.json">All results</a> · <a href="verification.json">Independent verification</a> · <a href="../development.html">Retained development attempts</a></p></main></html>''')
    checks = ' · '.join(f'<a href="{name}">{label}</a>' for name,label in (
        ('research_summary.json','Comparison and timing summary'),('test_output.txt','Regression log'),
        ('playback_checks.json','Offline playback checks'),('handoff_checks.json','Credential/resume/upstream checks'),
        ('usage_after.json','Cumulative usage summary')) if (folder/name).exists())
    repeated_path = folder/'easy/repeated_diagnostic/result.json'
    repeated_text = ''
    if repeated_path.exists():
        repeated = read(repeated_path)
        repeated_text = f'<p>Separate easy-scene repeated diagnostic: pickup={repeated["pickup_success"]}, unwanted collision={repeated["unwanted_collision"]}, {len(repeated["model_request_timings"])} logical requests. <a href="easy/repeated_diagnostic/result.json">Timing and results</a> · <a href="easy/repeated_diagnostic/events.json">Acceptance/late-response events</a>. This is outside the five-method startup matrix.</p>'
    page = folder/'index.html'
    if manifest.get('query_schedule') == 'completion_driven_repeated':
        reference = ROOT/manifest['startup_reference']['directory']/'index.html'
        relative = html.escape(os.path.relpath(reference, folder), quote=True)
        page.write_text(page.read_text().replace('Repeated scheduling is a separate diagnostic.',
            f'This gallery runs repeated updates for all three Jev conditions in all three scenes: {new_calls} new logical requests, estimated ${new_cost:.6f} additional input-token cost. Startup answers and the six non-Jev baselines are reused, not charged or counted again. Starts are at least 500 ms apart, with one request in flight. <a href="{relative}">Original startup-only gallery</a> · <a href="schedule_comparison.json">Paired comparison evidence</a>.').replace('</main>', schedule_table(folder)+'</main>'))
    page.write_text(page.read_text().replace('</main>',f'{repeated_text}<p>{checks}</p><p>Deterministic all-adaptive qualification witnesses also achieved 3/3 safe pickups. These designed scenes show geometry/selection tradeoffs, not unique Jev superiority or unseen generalization.</p></main>'))
    print('Gallery:', folder/'index.html', flush=True)
