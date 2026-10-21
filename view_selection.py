"""View a saved model proposal in its complete synthetic scene, without API calls.

On macOS launch with mjpython. This is not a robot rollout or collision/safety
evaluation. Covers and poses come from the recorded request, not today's defaults.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from inspect_geometry import show_scene
from inspect_selection import ROOT, decode_response


def _numbers(values):
    return " ".join(str(float(value)) for value in values)


def _vector(values, name, *, positive=False):
    values = np.asarray(values, dtype=float)
    if values.shape != (3,) or not np.isfinite(values).all() or (positive and np.any(values <= 0)):
        raise ValueError(f"{name} must contain three finite {'positive lengths' if positive else 'coordinates'}")
    return values


def selection_scene(report):
    """Validate saved answers, then render only the selected recorded covers.

    Object and sphere centres are kept in their recorded object/world frames.
    The table is a display-only Z=0 reference. All contacts are disabled. A path
    capsule has the proxy's actual radius; the thinner centreline is just ink.
    """
    if report.get("error") or report.get("validation_error") or "response" not in report:
        raise ValueError("need a successful saved selection response, not a preview or failed run")
    request, response = report["request"], report["response"]
    proposals = decode_response(request, response)
    if "proposals" in report and report["proposals"] != proposals:
        raise ValueError("cached proposals disagree with the saved model answers")
    state = request["state"]
    proxy = state["robot_proxy"]
    start = _vector(proxy["start_m"], "start_m")
    goal = _vector(proxy["goal_m"], "goal_m")
    radius = float(proxy["radius_m"])
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("proxy radius must be finite and positive")

    root = ET.Element("mujoco", model="Saved Jev selection - inspection only")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", contype="0", conaffinity="0")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "rgba", haze="0 0 0 1")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0.3 -0.4 1.8", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    lower = np.minimum(start, goal) - radius
    upper = np.maximum(start, goal) + radius
    labels = []
    summary = []
    for obj, proposal in zip(state["objects"], proposals, strict=True):
        object_id = obj["id"]
        center = _vector(obj["center_m"], f"{object_id}.center_m")
        half_extents = _vector(obj["size_m"], f"{object_id}.size_m", positive=True) / 2
        shape = obj["shape"]
        if shape not in ("box", "cylinder") or (shape == "cylinder" and half_extents[0] != half_extents[1]):
            raise ValueError(f"{object_id}: expected an axis-aligned box or circular Z-cylinder")
        body = ET.SubElement(world, "body", name=object_id, pos=_numbers(center))
        color = "0.70 0.72 0.76 1" if proposal["role"] != "review" else "1 0.85 0.15 1"
        size = half_extents if shape == "box" else half_extents[[0, 2]]
        ET.SubElement(body, "geom", name=f"{object_id}/object", type=shape,
                      size=_numbers(size), rgba=color, group="0")
        object_lower, object_upper = center - half_extents, center + half_extents
        count = 0
        if proposal["role"] == "avoid":
            matches = [c for c in obj["candidates"] if c["id"] == proposal["candidate_id"]]
            if len(matches) != 1:
                raise ValueError(f"{object_id}: selected candidate is missing or duplicated")
            candidate = matches[0]
            centers = np.asarray(candidate["centers_object_m"], dtype=float)
            radii = np.asarray(candidate["radii_m"], dtype=float)
            if (radii.ndim != 1 or len(radii) == 0 or centers.shape != (len(radii), 3)
                    or not np.isfinite(centers).all() or not np.isfinite(radii).all() or np.any(radii <= 0)):
                raise ValueError(f"{object_id}: invalid recorded sphere geometry")
            count = len(radii)
            for index, (position, sphere_radius) in enumerate(zip(centers, radii, strict=True)):
                ET.SubElement(body, "geom", name=f"{object_id}/cover/{index}", type="sphere",
                              pos=_numbers(position), size=str(float(sphere_radius)),
                              rgba="1 0.48 0.12 0.23", group="1")
            object_lower = np.minimum(object_lower, np.min(centers + center - radii[:, None], axis=0))
            object_upper = np.maximum(object_upper, np.max(centers + center + radii[:, None], axis=0))
        lower, upper = np.minimum(lower, object_lower), np.maximum(upper, object_upper)
        geometry_answer = response["answers"][f"geometry/{object_id}"]
        text = f"{object_id}: {proposal['role']}"
        if count:
            text += f" / {count} spheres / geom conf {geometry_answer['confidence']:.2f}"
        labels.append((text, [*center[:2], object_upper[2] + 0.04]))
        summary.append(text)

    for name, position, color in (
        ("start", start, "0.15 0.50 1 1"), ("goal", goal, "0.75 0.30 1 1"),
    ):
        ET.SubElement(world, "geom", name=f"proxy/{name}", type="sphere", pos=_numbers(position),
                      size=str(radius), rgba=color, group="3")
        labels.append((f"{name} proxy", position + [0, 0, radius + 0.025]))
    if np.linalg.norm(goal - start) > 0:
        for name, width, color in (
            ("sweep", radius, "0.15 0.80 1 0.12"),
            ("centerline", radius * 0.10, "0.15 0.80 1 1"),
        ):
            ET.SubElement(world, "geom", name=f"nominal/{name}", type="capsule",
                          fromto=_numbers(np.r_[start, goal]), size=str(width), rgba=color, group="2")
    lookat = (lower + upper) / 2
    span = float(np.max(upper - lower))
    ET.SubElement(world, "geom", name="table/display-only", type="plane",
                  pos=_numbers([lookat[0], lookat[1], 0]), size=_numbers([span, span, 0.01]),
                  rgba="0.14 0.16 0.19 1", group="4")
    ET.SubElement(root, "statistic", center=_numbers(lookat), extent=str(span))
    caption = (
        f"SAVED PROPOSAL ONLY - no physics or controller\n"
        f"Source: {report.get('mode', 'unspecified')} / {response['model']}\n"
        "Grey: objects | Orange: selected covers\n"
        "Cyan: nominal sweep | Blue: start | Purple: goal\n"
        "Overlap is possible: the nominal path is NOT filtered.\n"
        + "\n".join(summary)
    )
    return {
        "xml": ET.tostring(root, encoding="unicode"),
        "lookat": lookat,
        "distance": 2.5 * span,
        "labels": labels,
        "caption": caption,
        "proposals": proposals,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=ROOT / "artifacts/selection/live_initial.json")
    parser.add_argument("--top", action="store_true", help="start with a top-down camera")
    parser.add_argument("--seconds", type=float, help="close viewer after a bounded preview")
    parser.add_argument("--check", action="store_true", help="validate/compile the scene without a window")
    args = parser.parse_args(argv)
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    try:
        scene = selection_scene(json.loads(args.report.read_text()))
        print(scene["caption"], flush=True)
        print("Geom groups: 0 objects, 1 covers, 2 nominal sweep, 3 start/goal proxies, 4 display table.", flush=True)
        print("The table is only a Z=0 visual reference; it was not part of Jev's constraints.", flush=True)
        if args.check:
            import mujoco
            model = mujoco.MjModel.from_xml_string(scene["xml"])
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.geom_xpos).all():
                raise ValueError("nonfinite rendered geometry positions")
            print(f"Scene compiled: {model.ngeom} display geoms. No physics steps or API calls.")
        else:
            show_scene(
                scene["xml"], lookat=scene["lookat"], distance=scene["distance"],
                azimuth=-75, elevation=-90 if args.top else -40,
                labels=scene["labels"], caption=scene["caption"], seconds=args.seconds,
            )
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.exit(2, f"Selection visualization failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
