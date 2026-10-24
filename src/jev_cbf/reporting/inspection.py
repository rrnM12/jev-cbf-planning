"""Inspect conservative sphere covers without a robot, Jev, or a controller.

On macOS use mjpython for the viewer; regular python works with --no-view.
"""

import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from jev_cbf.geometry.covers import evaluate_covers, make_sphere_covers, make_shape_covers
from jev_cbf.reporting.rendering import add_scene_labels

def comparison_xml(half_extents, covers, shape):
    """Side-by-side copies: opaque reference object, transparent cover spheres.

    Display translations separate candidates only; the stored geometry stays in
    the object frame. All geoms are static, with contacts disabled. No physics or
    collision evaluation is implied by this inspection scene.
    """
    root = ET.Element("mujoco", model="Sphere-cover inspection")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "rgba", haze="0 0 0 1")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 3", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    span = max(
        float(np.max(np.abs(cover.centers_m) + np.asarray(cover.radii_m)[:, None]))
        for cover in covers
    )
    spacing = 2.6 * span
    ET.SubElement(root, "statistic", center="0 0 0", extent=str(spacing * len(covers)))
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", contype="0", conaffinity="0")
    colors = ("0.20 0.55 0.95 0.22", "0.15 0.80 0.55 0.22", "0.95 0.65 0.15 0.22")
    for i, cover in enumerate(covers):
        offset = (i - (len(covers) - 1) / 2) * spacing
        body = ET.SubElement(world, "body", name=cover.candidate_id, pos=f"{offset} 0 0")
        size = half_extents if shape == "box" else (half_extents[0], half_extents[2])
        ET.SubElement(
            body, "geom", name=f"{cover.candidate_id}-object", type=shape,
            size=" ".join(map(str, size)), rgba="0.60 0.62 0.67 1", group="0",
        )
        for j, (center, radius) in enumerate(zip(cover.centers_m, cover.radii_m)):
            ET.SubElement(
                body, "geom", name=f"{cover.candidate_id}-sphere-{j}", type="sphere",
                pos=" ".join(map(str, center)), size=str(radius),
                rgba=colors[i % len(colors)], group="1",
            )
    return ET.tostring(root, encoding="unicode"), span, spacing

def show_covers(half_extents, covers, shape, seconds=None):
    xml, span, spacing = comparison_xml(half_extents, covers, shape)
    print("Viewer: left to right = " + ", ".join(c.candidate_id for c in covers), flush=True)
    print("Grey = reference object; coloured = sphere union. Orbit/zoom to inspect.", flush=True)
    print("In Rendering > Geom groups: group 0 = objects; group 1 = covers.", flush=True)
    show_scene(xml, lookat=(0, 0, 0), distance=max(spacing * len(covers), 4 * span), seconds=seconds)

def show_scene(xml, *, lookat, distance, azimuth=-90, elevation=-15, labels=(), caption="", seconds=None):
    """Shared static inspection viewer. No integration step or controller runs."""
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = lookat
            viewer.cam.distance = distance
            viewer.cam.azimuth = azimuth
            viewer.cam.elevation = elevation
            add_scene_labels(viewer.user_scn, labels)
        # Text overlays are available in the installed MuJoCo. Older versions
        # still get the printed legend and world labels, without changing data.
        if caption and hasattr(viewer, "set_texts"):
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT, caption, ""))
        start = time.monotonic()
        while viewer.is_running() and (seconds is None or time.monotonic() - start < seconds):
            viewer.sync()
            time.sleep(1 / 30)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shape", choices=("box", "cylinder"), default="cylinder")
    parser.add_argument(
        "--size", type=float, nargs=3, default=(0.07, 0.07, 0.30),
        metavar=("X", "Y", "Z"), help="full bounding-box dimensions in metres",
    )
    parser.add_argument("--counts", type=int, nargs="+", default=(1, 2, 4))
    parser.add_argument("--shape-aware", action="store_true", help="also compare exact-cylinder slab covers or a four-cell box grid")
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-view", action="store_true", help="numeric experiment only")
    parser.add_argument("--seconds", type=float, help="close viewer after this many seconds")
    parser.add_argument("--report", type=Path, help="save geometry and measurements as JSON")
    args = parser.parse_args(argv)
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    half_extents = np.asarray(args.size) / 2
    try:
        covers = make_shape_covers(half_extents, shape=args.shape, counts=args.counts) if args.shape_aware else make_sphere_covers(half_extents, args.counts)
        report = evaluate_covers(
            half_extents, covers, shape=args.shape, samples=args.samples, seed=args.seed
        )
    except ValueError as error:
        parser.error(str(error))
    report["coverage_basis"] = (
        "Each box section is contained in its circumscribing sphere; their union "
        "covers the bounding box and any object contained in it. Analytic in exact "
        "arithmetic; sampled misses below are diagnostics, not the proof."
    )
    if args.shape_aware:
        report["coverage_basis"] += " Cylinder slabs or box-grid cells are each enclosed by their sphere; these unions cover the full stated shape, not necessarily the cylinder's bounding-box corners."
    print(f"Reference: {args.shape}, full dimensions {list(args.size)} m")
    print(f"Shared Monte Carlo points: {args.samples}; seed: {args.seed}")
    print(f"Reference object volume: {report['object_volume_m3'] * 1000:.4f} litres")
    print("Candidate        Spheres    Union (L)    Excess (L) +/- MC SE    Sampled misses")
    for result in report["candidates"]:
        print(
            f"{result['candidate_id']:<16} {result['primitive_count']:>7}"
            f" {result['union_volume_m3'] * 1000:>12.4f}"
            f" {result['excess_volume_m3'] * 1000:>11.4f}"
            f" +/- {result['excess_volume_standard_error_m3'] * 1000:.4f}"
            f" {result['uncovered_object_samples']:>14}"
        )
    print("Sphere count is a cost proxy, not measured CBF/QP runtime. No margin added.")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"Report: {args.report}")
    if not args.no_view:
        show_covers(half_extents, covers, args.shape, args.seconds)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
