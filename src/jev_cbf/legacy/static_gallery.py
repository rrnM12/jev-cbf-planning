"""Render recorded static-comparison rollouts into a local HTML/APNG gallery.

Renders simulation data, not screenshots of another application. No physics
rollout, model request, browser dependency, or additional Python package needed.
"""

import argparse
import html
import json
from pathlib import Path
import numpy as np
from jev_cbf.paths import ROOT
from jev_cbf.reporting.rendering import add_scene_labels
from jev_cbf.control.simulation import obstacle_spheres, scene_spec
from jev_cbf.control.model import panda_spec
from jev_cbf.reporting.images import _chunk
from jev_cbf.reporting.images import write_apng

def render_frames(mujoco, model, trajectory, result, indices, *, width, height, azimuth=135, elevation=-45):
    data = mujoco.MjData(model)
    data.mocap_pos[model.body("target").mocapid[0]] = result["goal_m"]
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # Robot collision hulls are not the visual robot.
    option.geomgroup[4] = 1  # Transparent selected covers; never physical geoms.
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.30, .05, .30]
    camera.distance, camera.azimuth, camera.elevation = 1.5, azimuth, elevation
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for index in indices:
            data.qpos[:] = trajectory["qpos"][index]
            data.qvel[:] = trajectory["qvel"][index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            labels = []
            for geom_id in range(model.ngeom):
                name = model.geom(geom_id).name
                if name and name.startswith("scene/"):
                    height = model.geom_size[geom_id, 2 if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX else 1]
                    labels.append((name.removeprefix("scene/"), data.geom_xpos[geom_id] + [0, 0, height + .025]))
            add_scene_labels(renderer.scene, labels, append=True)
            # A sparse history of the actual end-effector path, not nominal/safe
            # waypoints. Display-only markers cannot affect the saved trajectory.
            for point in trajectory["ee_m"][np.linspace(0, index, min(index + 1, 100), dtype=int)]:
                scene = renderer.scene
                if scene.ngeom >= scene.maxgeom:
                    break
                mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                   np.array([.004, 0, 0]), point, np.eye(3).ravel(),
                                   np.array([.9, .2, .8, 1]))
                scene.ngeom += 1
            yield renderer.render().copy()

def gallery_html(summary):
    cards = []
    failure_banner = ("<p><strong>Incomplete comparison: " + html.escape(str(summary["error"])) + "</strong></p>") if summary.get("error") else ""
    if summary.get("failures"):
        failure_banner += "<p><strong>Failed modes (no fallback): " + html.escape(str(summary["failures"])) + "</strong></p>"
    recheck_link = ' · <a href="collision_recheck.json">Collision recheck</a>' if summary.get("collision_recheck_available") else ""
    robot_aware = summary.get("robot_aware", False)
    context_label = ("One-shot actual Panda/whole-arm context — ground-truth geometry, not perception."
                     if robot_aware else "Saved proxy-decision replay — not a fresh whole-arm Jev assessment.")
    decisions = []
    for proposal in summary.get("selection_proposals", []):
        name = proposal["object_id"]
        role = summary.get("model_answers", {}).get(f"role/{name}", {})
        geometry = summary.get("model_answers", {}).get(f"geometry/{name}", {})
        probability = role.get("probabilities", {}).get(proposal["role"])
        geometry_probability = geometry.get("probabilities", {}).get(proposal["conditional_candidate_id"])
        decisions.append(f"<tr><td>{html.escape(name)}</td><td>{html.escape(proposal['role'])}</td>"
                         f"<td>{html.escape(str(proposal['candidate_id']))}</td><td>{probability}</td>"
                         f"<td>{geometry_probability}</td></tr>")
    decisions_table = ("<h2>Recorded Jev proposals</h2><p>Conditional geometry probabilities are retained even for omitted objects; no confidence threshold or oracle repair was applied.</p>"
                       "<table><tr><th>Object</th><th>Role</th><th>Selected cover</th><th>P(role)</th><th>P(conditional cover)</th></tr>"
                       + "".join(decisions) + "</table>") if decisions else ""
    for result in summary["runs"]:
        mode = result["mode"]
        minimum = min(result["collision"]["minimum_distance_m_by_object"].values())
        violation = result["collision"].get("any_sampled_scene_violation", result["collision"]["any_sampled_object_penetration"])
        barrier = result.get("minimum_selected_scene_sphere_barrier_m")
        barrier_text = "not enabled" if barrier is None else f"{barrier*1000:.3f} mm"
        cards.append(f'''<article><h2>{html.escape(mode.upper())}</h2>
          <img class="rollout" src="{html.escape(mode)}.png" alt="Recorded {html.escape(mode)} rollout">
          <p>Final EE error: <strong>{result['final_position_error_m']*1000:.1f} mm</strong><br>
          Rollout complete: {result['complete']} · Task criterion passed: {result['task_success']}<br>
          Scene CBF pairs: {result['scene_pair_constraint_count']}<br>
          Minimum sampled robot/object distance: {minimum*1000:.1f} mm<br>
          Minimum modeled scene barrier: <strong>{barrier_text}</strong><br>
          Sampled collision/contact flag: <strong>{'YES' if violation else 'none detected'}</strong><br>
          Controller host-time median / p95: {result['control_wall_ms_after_first']['median']:.2f} /
          {result['control_wall_ms_after_first']['p95']:.2f} ms</p>
          <p><a href="{html.escape(mode)}_final.png">Final frame</a> ·
          <a href="{html.escape(mode)}.npz">Recorded trajectory</a></p></article>''')
    return '''<!doctype html><html lang="en"><meta charset="utf-8">
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>Static Panda comparison</title>
      <style>body{font:16px system-ui;background:#111821;color:#e5edf5;margin:24px;line-height:1.5}
      main{max-width:1500px;margin:auto}section{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:18px}
      article{background:#1c2835;border-radius:12px;padding:16px}img{width:100%;border-radius:8px}
      a{color:#8ad6ff}button{padding:10px 16px;font:inherit;cursor:pointer}small{color:#b4c4d4}</style>
      <main><h1>Static Panda comparison</h1>''' + failure_banner + '''
      <p><small>Recorded selection source: ''' + html.escape(summary.get("source_report_mode", "see saved source report")) + " / " + html.escape(summary.get("source_model", "unspecified")) + '''</small></p>
      <p><strong>''' + html.escape(context_label) + '''</strong>
      All modes use the same default Panda start, saved objects/goal, original controller,
      gains, elastic solver, and robot protections. Only scene-object CBFs differ.</p>
      <p>OFF: no scene-object CBFs. ALL: every object constrained using ''' + html.escape(summary["all_geometry"]) + '''
      geometry. JEV/REPLAY: only saved “avoid” objects, with their selected geometry.</p>
      <p>Cyan: physical objects. Orange: modeled keep-out sphere covers. Magenta dots:
      recorded EE path. Red sphere: goal. Objects are fixed, physically collidable;
      orange covers and path markers are display-only. Floor Z=0 is a tabletop proxy,
      not a modeled raised table. A CBF filter is not a path planner.</p>
      <button onclick="document.querySelectorAll('.rollout').forEach(i=>{i.src=i.src.split('?')[0]+'?restart='+Date.now()})">Restart animations</button>
      <p><small>Animations loop at approximately simulated speed; separate images are not
      frame-synchronized. Distances use all MuJoCo robot collision geoms against every
      object at recorded states, including omitted objects. Zero distances are flagged
      rather than considered clear; contact depths supplement distance queries. No continuous-time safety
      guarantee; the elastic QP can relax inequalities. One scene/run per mode is not
      a safety-rate, model-quality, or real-time benchmark.</small></p>
      <p>Task criterion: complete rollout, final EE error &lt;20 mm, no sampled collision/contact or negative modeled h_2.</p>
      <p>Physical contact checks and modeled barriers are different diagnostics. A negative modeled barrier is a failure even when physical collision shapes remain separated.</p>
      <section>''' + "\n".join(cards) + '''</section>
      ''' + decisions_table + '''
      <p><a href="comparison.json">Full diagnostics</a>''' + recheck_link + ''' · <a href="selection_source.json">Saved model input/output</a></p>
      </main></html>'''

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=ROOT / "artifacts/static/initial")
    parser.add_argument("--frax-dir", type=Path, default=ROOT.parent / "frax")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--azimuth", type=float, default=135, help="rendering camera angle, degrees")
    parser.add_argument("--elevation", type=float, default=-45, help="rendering camera angle, degrees")
    parser.add_argument("--html-only", action="store_true", help="refresh report text using existing rendered frames")
    args = parser.parse_args(argv)
    if args.width < 1 or args.height < 1 or not 1 <= args.fps <= 65535 or not np.isfinite([args.azimuth, args.elevation]).all():
        parser.error("need positive dimensions, finite camera angles, and integer fps in [1, 65535]")
    try:
        import mujoco
        summary = json.loads((args.run_dir / "comparison.json").read_text())
        report = json.loads((args.run_dir / "selection_source.json").read_text())
        summary.setdefault("source_report_mode", report.get("mode", "unspecified"))
        summary.setdefault("source_model", report.get("response", {}).get("model", "no model response; baselines only"))
        summary.setdefault("selection_proposals", report.get("proposals", []))
        summary.setdefault("model_answers", report.get("response", {}).get("answers", {}))
        recheck_path = args.run_dir / "collision_recheck.json"
        if recheck_path.exists():
            recheck = json.loads(recheck_path.read_text())
            if recheck["source_sha256"] != summary["source_sha256"]:
                raise ValueError("collision recheck source differs from the rollout")
            summary["collision_recheck_available"] = True
            for result in summary["runs"]:
                result["collision"] = recheck["runs"][result["mode"]]
        if args.html_only and any(not (args.run_dir / f"{run['mode']}.png").exists() for run in summary["runs"]):
            raise ValueError("render animations before requesting --html-only")
        for result in ([] if args.html_only else summary["runs"]):
            mode = result["mode"]
            centers, radii, _ = obstacle_spheres(report, mode, summary["all_geometry"])
            spec = scene_spec(mujoco, args.frax_dir, report["request"]["state"], centers, radii)
            frame = spec.worldbody.add_frame(pos=[0, 0, 0])
            profile = result.get("model_profile", "upstream")
            robot = None
            if profile == "frax":
                from jev_cbf.upstream import load_upstream
                demo, _ = load_upstream(args.frax_dir, ROOT.parent / "cbfpy")
                robot = demo.load_panda()
            spec.attach(panda_spec(mujoco, args.frax_dir, profile=profile, robot=robot), prefix="", frame=frame)
            model = spec.compile()
            # Original scene offscreen defaults are only 640x480; change the
            # framebuffer allocation for rendering, not rollout physics.
            model.vis.global_.offwidth = args.width
            model.vis.global_.offheight = args.height
            with np.load(args.run_dir / f"{mode}.npz", allow_pickle=False) as trajectory:
                last = len(trajectory["qpos"]) - 1
                count = max(2, int(np.ceil(result["simulation_seconds"] * args.fps)) + 1)
                indices = np.unique(np.linspace(0, last, count, dtype=int))
                frames = render_frames(mujoco, model, trajectory, result, indices, width=args.width, height=args.height,
                                       azimuth=args.azimuth, elevation=args.elevation)
                write_apng(args.run_dir / f"{mode}.png", frames, frame_count=len(indices), fps=args.fps)
                final_frame = render_frames(mujoco, model, trajectory, result, [last], width=args.width, height=args.height,
                                            azimuth=args.azimuth, elevation=args.elevation)
                write_apng(args.run_dir / f"{mode}_final.png", final_frame, frame_count=1, fps=1)
            print(f"Rendered {mode}: {len(indices)} frames", flush=True)
        (args.run_dir / "index.html").write_text(gallery_html(summary))
        print(f"Gallery: {args.run_dir / 'index.html'}", flush=True)
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        parser.exit(2, f"Gallery failed: {error}\n")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
