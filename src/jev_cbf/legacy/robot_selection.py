"""Ground-truth Panda/motion context for the existing selection API.

This module measures features and prepares requests; it never chooses Jev's
answers, changes controller math, or fits perception geometry.
"""

from copy import deepcopy
import json
import numpy as np
from jev_cbf.legacy.selection import prepare_request
from jev_cbf.control.simulation import collision_diagnostics, scene_spec
from jev_cbf.selection.context import robot_context

def scene_input(scene, goal, start):
    """Exclude suite titles/expected labels from model-visible scene facts."""
    objects = []
    for obj in scene["objects"]:
        objects.append({key: deepcopy(obj[key]) for key in ("id", "shape", "size_m", "center_m")})
    return {
        "description": "Ground-truth static, axis-aligned objects on a Z=0 floor-level tabletop proxy. No perception. Objects are fixed and physically collidable in evaluated runs.",
        "frame": "robot base/world; lengths in metres; Z points up; floor surface Z=0; identity object rotations",
        "task": "Move the actual Panda end effector from motion.start_m to motion.goal_m with fixed down-facing orientation, without physical contact with any object. The controller may deviate from its nominal motion. No pushing, grasping, or interaction exemption is authorized.",
        "motion": {"start_m": np.asarray(start).tolist(), "goal_m": list(goal)},
        "objects": objects,
    }

def nominal_sphere_path(robot, nominal):
    import jax
    q = np.asarray(nominal["qpos"])[:, :7]
    return np.asarray(jax.jit(jax.vmap(robot.link_collision_positions))(q))

def add_motion_features(request, robot, nominal, sphere_path, native_distances, native_summary):
    """Extrema use every nominal sample; sparse snapshots are explanatory only."""
    radii = np.asarray(robot.link_collision_data(np.asarray(nominal["qpos"])[0, :7])[1])
    if sphere_path.shape != (len(nominal["qpos"]), len(radii), 3):
        raise ValueError("nominal sphere path shape mismatch")
    state = request["state"]
    last = len(sphere_path) - 1
    times = np.asarray(nominal["time_s"])
    early = int(np.searchsorted(times, min(2.0, times[-1])))
    snapshots = np.unique(np.r_[np.linspace(0, early, 9, dtype=int), last])
    state["motion"]["nominal_prediction"] = {
        "kind": "Counterfactual original-controller rollout with scene-object CBFs AND scene contact forces disabled; existing robot protections retained. Not an evaluated off-baseline and not a safe path.",
        "physics_samples": len(sphere_path), "time_horizon_s": float(times[-1]),
        "snapshot_time_s": times[snapshots].round(6).tolist(),
        "snapshot_joint_positions_rad": np.asarray(nominal["qpos"])[snapshots, :7].round(6).tolist(),
        "snapshot_ee_m": np.asarray(nominal["ee_m"])[snapshots].round(6).tolist(),
        "snapshot_robot_sphere_centers_m": sphere_path[snapshots].round(6).tolist(),
        "notes": "Snapshot coordinates rounded only for model context; extrema below use full precision at every recorded state. Discrete prediction, not a continuous-time or altered-motion safety guarantee.",
    }
    for column, obj in enumerate(state["objects"]):
        obj["nominal_native_collision_features"] = {
            "initial_distance_m": float(native_distances[0, column]),
            "minimum_distance_m": float(native_distances[:, column].min()),
            "predicted_end_distance_m": float(native_distances[-1, column]),
            "closest_robot_body": native_summary["closest_robot_body_by_object"][obj["id"]],
            "contact_or_nonpositive_distance": bool(native_summary["nonpositive_distance_sample_count_by_object"][obj["id"]]
                                                       or native_summary["contact_sample_count_by_object"][obj["id"]]),
            "scope": "All native robot collision geometry versus the actual object primitive, not its sphere cover; zero distance is flagged as potentially touching/overlapping.",
        }
        for candidate in obj["candidates"]:
            centers = np.asarray(candidate["centers_object_m"]) + obj["center_m"]
            distances = (np.linalg.norm(sphere_path[:, :, None] - centers[None, None], axis=-1)
                         - radii[None, :, None] - np.asarray(candidate["radii_m"])[None, None])
            where = np.unravel_index(np.argmin(distances), distances.shape)
            candidate.update(
                initial_robot_sphere_clearance_m=float(distances[0].min()),
                nominal_path_robot_sphere_clearance_m=float(distances.min()),
                predicted_end_robot_sphere_clearance_m=float(distances[-1].min()),
                closest_nominal_time_s=float(times[where[0]]), closest_robot_sphere_index=int(where[1]),
                robot_primitive_constraint_pairs=int(len(radii) * len(centers)),
            )
            # Choice criteria contain copies after JSON conversion. Keep their
            # descriptions identical to the candidate facts in the shared state.
            request["questions"][f"geometry/{obj['id']}"]["criteria"][candidate["id"]] = deepcopy(candidate)
    return json.loads(json.dumps(request, allow_nan=False))

def prepare_robot_request(scene, template, robot, nominal, sphere_path, mujoco, frax_dir, *, model="jev-latest", samples=100_000, seed=0):
    context = robot_context(robot, nominal)
    facts = scene_input(scene, scene["goal_m"], np.asarray(nominal["ee_m"])[0])
    request = prepare_request(facts, template, model=model, samples=samples, seed=seed, robot_context=context)
    spec = scene_spec(mujoco, frax_dir, request["state"], np.empty((0, 3)), np.empty(0))
    spec.attach(mujoco.MjSpec.from_file(str(frax_dir / "examples/xml/panda.xml")), prefix="", frame=spec.worldbody.add_frame())
    native_summary, native_distances = collision_diagnostics(
        mujoco, spec.compile(), nominal["qpos"], nominal["qvel"], [obj["id"] for obj in facts["objects"]]
    )
    request = add_motion_features(request, robot, nominal, sphere_path, native_distances, native_summary)
    qualification = {
        "nominal_collision": native_summary,
        "native_initial_clear": bool(np.all(native_distances[0] > 0)),
        "native_predicted_end_clear": bool(np.all(native_distances[-1] > 0)),
        "goal_check_scope": "Native collision-free endpoint of nominal rollout, not proof of a continuous feasible path or of every cover's goal feasibility.",
        "all_four_initial_model_clear": all(
            next(c for c in obj["candidates"] if c["id"] == "box-split-4")["initial_robot_sphere_clearance_m"] >= 0
            for obj in request["state"]["objects"]
        ),
    }
    return request, qualification
