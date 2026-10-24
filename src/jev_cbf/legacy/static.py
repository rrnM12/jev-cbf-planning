import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import numpy as np
from jev_cbf.paths import ROOT
from jev_cbf.control.simulation import MODES, obstacle_spheres
from jev_cbf.upstream import load_upstream
from jev_cbf.control.simulation import run_case

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=ROOT / "artifacts/selection/live_initial.json")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/static/initial")
    parser.add_argument("--modes", nargs="+", choices=MODES, default=("off", "all", "replay"))
    parser.add_argument("--steps", type=int, default=3000, help="physics steps per mode (default: 6 simulated seconds)")
    parser.add_argument("--all-geometry", default="conditional", help="all-on baseline: conditional answers or an explicit candidate ID")
    parser.add_argument("--view", action="store_true", help="show physics live; use mjpython on macOS")
    parser.add_argument("--model-profile", choices=("upstream", "frax"), default="upstream")
    parser.add_argument("--scene-guard", type=float, default=0., help="scene RD2 inequality tightening, m/s²; not a geometry margin")
    parser.add_argument("--control-dt", type=float, help="override control/physics step, seconds")
    parser.add_argument("--frax-dir", type=Path, default=ROOT.parent / "frax")
    parser.add_argument("--cbfpy-dir", type=Path, default=ROOT.parent / "cbfpy")
    args = parser.parse_args(argv)
    if args.steps < 2 or len(set(args.modes)) != len(args.modes):
        parser.error("need at least two steps and unique modes")
    try:
        raw = args.report.read_bytes()
        report = json.loads(raw)
        for mode in args.modes:
            obstacle_spheres(report, mode, args.all_geometry)
        if (args.output / "comparison.json").exists() or any((args.output / f"{mode}.npz").exists() for mode in args.modes):
            raise ValueError("output already contains a run; choose a new --output directory")
        demo, utils = load_upstream(args.frax_dir.resolve(), args.cbfpy_dir.resolve())
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "selection_source.json").write_bytes(raw)
        revisions = {name: subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
                     for name, path in (("frax", args.frax_dir), ("cbfpy", args.cbfpy_dir))}
        summary = {"source_sha256": hashlib.sha256(raw).hexdigest(), "all_geometry": args.all_geometry,
                   "upstream_revisions": revisions,
                   "runtime_versions": {name: importlib.metadata.version(name) for name in ("jax", "jaxlib", "mujoco", "elastiqp", "numpy")},
                   "source_model": report.get("response", {}).get("model", "no model response; baselines only"),
                   "source_report_mode": report.get("mode", "unspecified"),
                   "robot_aware": "robot_context" in report["request"]["state"],
                   "selection_context": "actual Panda context" if "robot_context" in report["request"]["state"] else "Proxy-decision replay with default Panda start, not a new whole-arm Jev decision.",
                   "runs": []}
        for mode in args.modes:
            print(f"\n=== {mode.upper()} ===", flush=True)
            try:
                result, trajectory = run_case(demo, utils, report, mode, frax_dir=args.frax_dir.resolve(),
                                              steps=args.steps, view=args.view, all_geometry=args.all_geometry,
                                              model_profile=args.model_profile, scene_guard=args.scene_guard,
                                              control_dt=args.control_dt)
            except (RuntimeError, ValueError) as error:
                summary["failed_mode"] = mode
                summary["error"] = str(error)
                (args.output / "comparison.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
                raise
            np.savez_compressed(args.output / f"{mode}.npz", **trajectory)
            summary["runs"].append(result)
            (args.output / "comparison.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
            print(f"{mode}: complete={result['complete']}, final error={result['final_position_error_m']:.4f} m, "
                  f"sampled scene collision/contact flag={result['collision']['any_sampled_scene_violation']}, "
                  f"scene pairs={result['scene_pair_constraint_count']}", flush=True)
        print(f"Saved comparison: {args.output / 'comparison.json'}", flush=True)
        return 0 if all(r["complete"] for r in summary["runs"]) else 1
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        parser.exit(2, f"Static comparison failed: {error}\n")

if __name__ == "__main__":
    raise SystemExit(main())
