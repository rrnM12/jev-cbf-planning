"""Static Panda runs using the unchanged FRAX demo and recorded scene selections.

Consumes proxy-replay or actual robot-context reports without making API calls.
Default runs are headless; use mjpython with --view on macOS.
"""

from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
from jev_cbf.selection.validation import scene_motion, validate_scene, validate_selection
from jev_cbf.control.model import panda_spec
from jev_cbf.control.cbf import guarded_cbf
from jev_cbf.upstream import bindings
from jev_cbf.upstream import Overlay

MODES = ("off", "all", "replay", "jev")

def obstacle_spheres(report, mode, all_geometry="conditional"):
    """Translate recorded covers into world coordinates, with no regeneration.

    All-on uses each object's conditional answer by default, so role selection
    is compared without also changing the selected objects' representations.
    A fixed candidate ID can instead be requested explicitly for that baseline.
    """
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    state = report["request"]["state"]
    validate_scene(state)
    if mode == "jev" and "robot_context" not in state:
        raise ValueError("jev mode requires a robot-context request; use replay for old proxy answers")
    needs_answers = mode in ("replay", "jev") or (mode == "all" and all_geometry == "conditional")
    proposals = validate_selection(report) if needs_answers else [None] * len(state["objects"])
    centers, radii, included = [], [], []
    for obj, proposal in zip(state["objects"], proposals, strict=True):
        if mode == "off":
            continue
        if mode in ("replay", "jev"):
            if proposal["role"] == "review":
                raise ValueError(f"{obj['id']}: unresolved review answer; no automatic omission")
            if proposal["role"] != "avoid":
                continue
            candidate_id = proposal["candidate_id"]
        else:
            candidate_id = proposal["conditional_candidate_id"] if all_geometry == "conditional" else all_geometry
        matches = [c for c in obj["candidates"] if c["id"] == candidate_id]
        if len(matches) != 1:
            raise ValueError(f"{obj['id']}: candidate {candidate_id!r} missing or duplicated")
        candidate = matches[0]
        points = np.asarray(candidate["centers_object_m"], dtype=float)
        sizes = np.asarray(candidate["radii_m"], dtype=float)
        if (sizes.ndim != 1 or len(sizes) == 0 or points.shape != (len(sizes), 3)
                or not np.isfinite(points).all() or not np.isfinite(sizes).all() or np.any(sizes <= 0)):
            raise ValueError(f"{obj['id']}: invalid recorded sphere geometry")
        world = points + np.asarray(obj["center_m"])
        centers.extend(world.tolist())
        radii.extend(sizes.tolist())
        included.append({"object_id": obj["id"], "candidate_id": candidate_id,
                         "sphere_count": len(sizes),
                         "estimated_object_local_excess_m3": candidate["excess_volume_m3"]})
    return np.asarray(centers, dtype=float).reshape(-1, 3), np.asarray(radii), included

def scene_spec(mujoco, frax_dir, state, centers, radii, *, physical_objects=True, moving_objects=False):
    """Extend the original empty scene in memory; keep its floor and assets."""
    spec = mujoco.MjSpec.from_file(str(frax_dir / "examples/xml/scene.xml"))
    for obj in state["objects"]:
        half = np.asarray(obj["size_m"]) / 2
        is_box = obj["shape"] == "box"
        parent = (spec.worldbody.add_body(name=f"object/{obj['id']}", mocap=True, pos=obj["center_m"])
                  if moving_objects else spec.worldbody)
        parent.add_geom(
            name=f"scene/{obj['id']}",
            type=mujoco.mjtGeom.mjGEOM_BOX if is_box else mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=[0, 0, 0] if moving_objects else obj["center_m"], size=half if is_box else [half[0], half[2], 0],
            rgba=[0.25, 0.65, 0.75, 1], contype=int(physical_objects), conaffinity=int(physical_objects), group=0,
        )
    for index, (center, radius) in enumerate(zip(centers, radii, strict=True)):
        spec.worldbody.add_geom(
            name=f"cover/{index}", type=mujoco.mjtGeom.mjGEOM_SPHERE,
            pos=center, size=[radius, 0, 0], rgba=[1, 0.50, 0.12, 0.16],
            contype=0, conaffinity=0, group=4,
        )
    return spec

class HeadlessViewer:
    """No GUI; the original main loop still runs the real torque physics."""

    def is_running(self):
        return True

    def sync(self):
        pass

    def close(self):
        pass

def collision_diagnostics(mujoco, model, qpos, qvel, object_ids, object_positions=None):
    """Check all objects against MuJoCo robot collision geoms, not CBF spheres.

    Mesh distances use MuJoCo collision geometry (including convex hulls), not
    visual-mesh truth. Sampled states do not certify continuous-time clearance.
    Use separate MjData so evaluation cannot change the simulated trajectory.
    """
    objects = {name: model.geom(f"scene/{name}").id for name in object_ids}
    robot_geoms = [i for i in range(model.ngeom)
                   if model.geom_bodyid[i] != 0 and model.geom_bodyid[i] != model.body("target").id
                   and i not in objects.values()
                   and (model.geom_contype[i] or model.geom_conaffinity[i])]
    if not robot_geoms:
        raise ValueError("no physical robot collision geometry found")
    evaluation = mujoco.MjData(model)
    minima = {name: float("inf") for name in objects}
    collision_states = {name: 0 for name in objects}
    nonpositive_states = {name: 0 for name in objects}
    contact_states = {name: 0 for name in objects}
    closest_bodies, closest_samples = {}, {}
    distances = np.empty((len(qpos), len(objects)))
    if object_positions is not None and np.shape(object_positions) != (len(qpos), len(objects), 3):
        raise ValueError("object pose history must match states and object IDs")
    mocap_ids = model.body_mocapid[model.geom_bodyid[list(objects.values())]]
    if object_positions is not None and np.any(mocap_ids < 0):
        raise ValueError("pose histories require prescribed mocap object bodies")
    for index, (position, velocity) in enumerate(zip(qpos, qvel, strict=True)):
        if object_positions is not None:
            evaluation.mocap_pos[mocap_ids] = object_positions[index]
        evaluation.qpos[:] = position
        evaluation.qvel[:] = velocity
        mujoco.mj_forward(model, evaluation)
        for column, (name, geom) in enumerate(objects.items()):
            contacts = [contact for contact in evaluation.contact
                        if (int(contact.geom1) == geom and int(contact.geom2) in robot_geoms)
                        or (int(contact.geom2) == geom and int(contact.geom1) in robot_geoms)]
            distance, closest_geom = min((mujoco.mj_geomDistance(model, evaluation, geom, robot_geom, 1000., None), robot_geom)
                                         for robot_geom in robot_geoms)
            # In a positive control, native mesh distance returned exactly zero
            # for an overlap. Never interpret zero as clear. Contact distances
            # can supply negative depth when the distance-only query does not.
            if contacts:
                contact = min(contacts, key=lambda item: item.dist)
                if contact.dist < distance:
                    distance = float(contact.dist)
                    closest_geom = int(contact.geom2) if int(contact.geom1) == geom else int(contact.geom1)
            if not np.isfinite(distance):
                raise ValueError("nonfinite independent collision distance")
            distances[index, column] = distance
            if distance < minima[name]:
                minima[name] = distance
                closest_bodies[name] = model.body(int(model.geom_bodyid[closest_geom])).name
                closest_samples[name] = index
            collision_states[name] += int(distance < 0)
            nonpositive_states[name] += int(distance <= 0)
            contact_states[name] += int(bool(contacts))
    return {
        "robot_collision_geom_count": len(robot_geoms),
        "minimum_distance_m_by_object": minima,
        "closest_robot_body_by_object": closest_bodies,
        "closest_sample_index_by_object": closest_samples,
        "penetrating_sample_count_by_object": collision_states,
        "nonpositive_distance_sample_count_by_object": nonpositive_states,
        "contact_sample_count_by_object": contact_states,
        "any_sampled_object_penetration": bool(np.any(distances < 0)),
        "any_sampled_nonpositive_distance": bool(np.any(distances <= 0)),
        "any_sampled_scene_contact": bool(any(contact_states.values())),
        "any_sampled_scene_violation": bool(np.any(distances <= 0) or any(contact_states.values())),
        "scope": "All scene objects versus MuJoCo robot collision geoms at recorded states; "
                 "includes gripper/base, not just FRAX moving-link spheres. Minimum of distance query and contact distances. "
                 "Zero distance is flagged, not certified clear. No continuous-time proof.",
    }, distances

def run_case(demo, utils, report, mode, *, frax_dir, steps, view=False, all_geometry="conditional", nominal_free_space=False, model_profile="upstream", scene_guard=0., control_dt=None, runtime=None):
    """Inject scene/constraint data while executing the original main/controller."""
    mujoco = utils.mujoco
    centers, radii, included = obstacle_spheres(report, mode, all_geometry)
    state = report["request"]["state"]
    expected_start, goal = scene_motion(state)
    robot_aware = "robot_context" in state
    if nominal_free_space and mode != "off":
        raise ValueError("counterfactual free-space prediction is only allowed with scene CBFs off")
    captured = {}
    samples, velocities, torques, control_seconds = [], [], [], []
    began = time.perf_counter()

    class Finished(Exception):
        pass

    class StaticEnv(demo.ManipulatorEnv):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            captured["env"] = self
            self.data.mocap_pos[self.mocap_id] = goal
            if runtime is not None:
                runtime.initialize(self)
            mujoco.mj_forward(self.model, self.data)
            samples.append(self.data.qpos.copy())
            velocities.append(self.data.qvel.copy())
            if view:
                with self.viewer.lock():
                    self.viewer.cam.lookat[:] = [0.3, 0.05, 0.3]
                    self.viewer.cam.distance = 1.5
                    self.viewer.cam.azimuth = 135
                    self.viewer.cam.elevation = -30
                    self.viewer.opt.geomgroup[4] = 1
                if hasattr(self.viewer, "set_texts"):
                    self.viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,
                                           mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                           f"{mode.upper()} / {'actual Panda context' if robot_aware else 'proxy-decision replay'}\nDo not drag target during comparison", ""))

        def get_joint_state(self):
            if runtime is not None:
                runtime.before_control(self)
            z = super().get_joint_state()
            captured.setdefault("initialization_wall_seconds", time.perf_counter() - began)
            captured["control_started"] = time.perf_counter()
            return z

        def get_desired_ee_state(self):
            if runtime is not None and hasattr(runtime, "desired_ee_state"):
                return runtime.desired_ee_state(self)
            # Keep every mode's target fixed, even if a GUI user drags its marker.
            self.data.mocap_pos[self.mocap_id] = goal
            return super().get_desired_ee_state()

        def apply_control(self, u):
            if runtime is not None:
                u = runtime.filter(super().get_joint_state(), u, self.data.time)
            u = np.asarray(u)  # Synchronize the JAX result before recording time.
            control_seconds.append(time.perf_counter() - captured["control_started"])
            if not np.isfinite(u).all():
                raise RuntimeError("nonfinite control; aborting without substitute torque")
            torques.append(u.copy())
            super().apply_control(u)
            if runtime is not None and hasattr(runtime, "apply_auxiliary_control"):
                runtime.apply_auxiliary_control(self)

        def step(self):
            super().step()
            if runtime is not None:
                runtime.after_step(self)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise RuntimeError("nonfinite state; aborting simulation")
            samples.append(self.data.qpos.copy())
            velocities.append(self.data.qvel.copy())
            if self.step_count >= steps or getattr(runtime, "task_done", False):
                raise Finished

    config_class = demo.DemoOSCBFConfig
    cbf_class = demo.CBF

    def config_factory(robot, *_args, **_kwargs):
        config = config_class(robot, centers, radii)
        captured["config"] = config
        initial = np.asarray(config.h_2(utils.ManipulatorEnv.get_joint_state(captured["env"])))
        if robot_aware:
            actual_q = captured["env"].data.qpos[:7]
            if not np.allclose(actual_q, state["robot_context"]["initial_joint_positions_rad"], atol=1e-12, rtol=0):
                raise ValueError("request initial joints differ from the robot's actual initial state")
            if not np.allclose(captured["env"].data.qvel[:7], state["robot_context"]["initial_joint_velocities_rad_s"], atol=1e-12, rtol=0):
                raise ValueError("request initial velocity differs from the robot's actual initial state")
            if not np.allclose(np.asarray(robot.ee_transform(actual_q))[:3, 3], expected_start, atol=1e-9, rtol=0):
                raise ValueError("request EE start differs from the robot's actual start")
        if not np.isfinite(initial).all() or np.any(initial < 0):
            raise ValueError("initial state violates the modeled safe set; no rollout started")
        return config

    def cbf_factory(config):
        if runtime is not None:
            runtime.setup(Overlay(demo, CBF=cbf_class), config.robot, scene_guard)
            captured["config"], captured["cbf"] = runtime.config, runtime.cbf
            # The original nested policy still computes its exact nominal torque.
            # Filtering moves outside that JIT so runtime arrays are arguments,
            # not captured Python attributes that silently stay stale in JAX.
            return SimpleNamespace(safety_filter=lambda _z, nominal: nominal)
        cbf = guarded_cbf(Overlay(demo, CBF=cbf_class), config, scene_guard=scene_guard)
        captured["cbf"] = cbf
        return cbf

    def load_spec(path):
        if Path(path).resolve() == (frax_dir / "examples/xml/obstacle_scene.xml").resolve():
            captured["scene_hook_used"] = True
            if runtime is not None and hasattr(runtime, "scene_spec"):
                spec = runtime.scene_spec(mujoco, frax_dir, state, centers, radii)
            else:
                spec = scene_spec(mujoco, frax_dir, state, centers, radii, physical_objects=not nominal_free_space,
                                  moving_objects=runtime is not None)
            if control_dt is not None:
                if control_dt <= 0:
                    raise ValueError("control_dt must be positive")
                spec.option.timestep = control_dt
            return spec
        if Path(path).resolve() == (frax_dir / "examples/xml/panda.xml").resolve():
            if runtime is not None and hasattr(runtime, "robot_spec"):
                return runtime.robot_spec(mujoco, frax_dir)
            return panda_spec(mujoco, frax_dir, profile=model_profile,
                              robot=demo.load_panda() if model_profile == "frax" else None)
        return mujoco.MjSpec.from_file(path)

    viewer = mujoco.viewer if view else Overlay(mujoco.viewer, launch_passive=lambda *_args, **_kwargs: HeadlessViewer())
    adapted_mujoco = Overlay(mujoco, MjSpec=SimpleNamespace(from_file=load_spec), viewer=viewer)
    failure = None
    with bindings(utils, mujoco=adapted_mujoco), bindings(
        demo, ManipulatorEnv=StaticEnv, DemoOSCBFConfig=config_factory,
        CBF=SimpleNamespace(from_config=cbf_factory),
        STARTUP_MSG=f"Initializing static {mode} comparison (original FRAX controller).",
        INFO_MSG=f"Running {mode}: scene spheres={len(radii)}; existing robot protections remain active.",
    ):
        try:
            demo.main("panda")
        except Finished:
            pass
        except (RuntimeError, ValueError) as error:
            failure = f"{type(error).__name__}: {error}"
            if "env" in captured:
                captured["env"].close()
            if runtime is not None and hasattr(runtime, 'close'):
                runtime.close()
    rollout_wall = time.perf_counter() - began
    if not captured.get("scene_hook_used") or any(key not in captured for key in ("config", "cbf", "env")):
        raise RuntimeError(f"upstream integration did not reach expected hooks: {failure}")
    if not torques:
        raise RuntimeError(f"no controls executed: {failure}")
    env, config, cbf = captured["env"], captured["config"], captured["cbf"]
    robot = config.robot
    qpos, qvel = np.asarray(samples), np.asarray(velocities)
    zs = np.concatenate([qpos[:, :7], qvel[:, :7]], axis=1)
    if runtime is None:
        h2 = np.asarray(demo.jax.jit(demo.jax.vmap(config.h_2))(zs))
        h1 = np.asarray(demo.jax.jit(demo.jax.vmap(cbf.h))(zs))
    else:
        h2, h1, residual = runtime.diagnostics(zs, np.asarray(torques), env.dt)
    ee = np.asarray(demo.jax.jit(demo.jax.vmap(lambda q: robot.ee_transform(q)[:3, 3]))(zs[:, :7]))
    if not all(np.isfinite(array).all() for array in (h2, h1, ee)):
        raise RuntimeError("nonfinite barrier/kinematic diagnostics")
    errors = np.linalg.norm(ee - goal, axis=1)
    sphere_count = getattr(config, "scene_robot_sphere_count", len(robot.link_collision_data(zs[0, :7])[1]))
    obstacle_start = 2 * robot.nv + 1
    number_of_slots = runtime.capacity if runtime is not None else len(radii)
    obstacle_end = obstacle_start + sphere_count * number_of_slots
    # This upstream G/h implementation does not depend on nominal torque. Compute
    # its inequality violation with the actual requested controls, without copying
    # a solver or assuming that the elastic QP enforces every inequality exactly.
    @demo.jax.jit
    def violation(z, u):
        return cbf.G_qp(z, u) @ u - cbf.h_qp(z, u)
    if runtime is None:
        residual = np.asarray(demo.jax.jit(demo.jax.vmap(violation))(zs[:len(torques)], np.asarray(torques)))
    if not np.isfinite(residual).all():
        raise RuntimeError("nonfinite QP inequality diagnostics")
    if runtime is not None and hasattr(runtime, "collision_diagnostics"):
        collision, distances = runtime.collision_diagnostics(mujoco, env.model, qpos, qvel)
    else:
        object_positions = runtime.object_history(np.arange(len(qpos))*env.dt) if runtime is not None else None
        collision, distances = collision_diagnostics(mujoco, env.model, qpos, qvel,
                                                   [o["id"] for o in state["objects"]], object_positions)
    times = np.asarray(control_seconds)
    post_first = times[1:] if len(times) > 1 else times
    torque_excess = np.maximum(np.abs(torques) - np.asarray(robot.actuated_joint_max_forces), 0)
    warnings = {str(mujoco.mjtWarning(i).name): int(warning.number)
                for i, warning in enumerate(env.data.warning) if warning.number}
    result = {
        "mode": mode,
        "model_profile": model_profile,
        "scene_acceleration_guard_m_s2": scene_guard,
        "selection_source": ("saved robot-context selection" if mode in ("jev", "replay") else "independent baseline; no model decisions used")
                            if robot_aware else "saved proxy-decision replay; no new robot-context Jev request",
        "nominal_free_space_prediction": nominal_free_space,
        "complete": (len(torques) == steps or getattr(runtime, "task_done", False)) and failure is None and not warnings,
        "failure": failure, "mujoco_warnings": warnings,
        "steps": len(torques), "requested_steps": steps,
        "simulation_seconds": float(env.data.time), "timestep_seconds": env.dt,
        "goal_m": goal.tolist(), "actual_start_m": ee[0].tolist(),
        "requested_start_m": expected_start.tolist(),
        "start_context_changed": not bool(np.allclose(ee[0], expected_start, atol=1e-6)),
        "robot_body_context_changed": not robot_aware,
        "initial_position_error_m": float(errors[0]), "final_position_error_m": float(errors[-1]),
        "minimum_position_error_m": float(errors.min()),
        "within_2cm_at_end": bool(errors[-1] < 0.02),
        "success_definition": "Complete rollout, final EE error < 0.02 m, no sampled nonpositive scene distance/contact or negative modeled h_2; exploratory threshold, not tuned.",
        "included_objects": included, "scene_sphere_count": len(radii),
        "robot_sphere_count": sphere_count, "scene_pair_constraint_count": sphere_count * len(radii),
        "total_barrier_count": int(h2.shape[1]),
        "minimum_joint_limit_barrier_rad": float(h2[:, :2 * robot.nv].min()),
        "minimum_singularity_barrier": float(h2[:, 2 * robot.nv].min()),
        "minimum_selected_scene_sphere_barrier_m": float(h2[:, obstacle_start:obstacle_end].min()) if number_of_slots else None,
        "minimum_self_collision_barrier_m": float(h2[:, obstacle_end:-sphere_count].min()),
        "minimum_floor_barrier_m": float(h2[:, -sphere_count:].min()),
        "minimum_rd1_transformed_barrier": float(h1.min()),
        "any_sampled_negative_h2": bool(np.any(h2 < 0)),
        "maximum_cbf_inequality_violation": float(np.maximum(residual[:, :config.num_cbf], 0).max()),
        "maximum_control_inequality_violation_Nm": float(np.maximum(residual[:, config.num_cbf:], 0).max()),
        "maximum_requested_torque_limit_excess_Nm": float(torque_excess.max()),
        "qp_relaxed": bool(config.relax_qp), "solver_backend": config.backend,
        "scene_near_binding_pair_counts": {
            "threshold": "abs(G u - h) <= solver_tol; descriptive, not the solver's active set",
            "solver_tol": config.solver_tol,
            "median": float(np.median(np.sum(np.abs(residual[:, obstacle_start:obstacle_end]) <= config.solver_tol, axis=1))),
            "maximum": int(np.max(np.sum(np.abs(residual[:, obstacle_start:obstacle_end]) <= config.solver_tol, axis=1))),
        },
        "initialization_wall_seconds": captured["initialization_wall_seconds"],
        "rollout_wall_seconds_including_initialization": rollout_wall,
        "control_wall_ms_after_first": {"median": float(np.median(post_first)*1000),
                                         "p95": float(np.percentile(post_first, 95)*1000),
                                         "max": float(post_first.max()*1000)},
        "control_timing_scope": "Host time from get_joint_state through synchronized returned torque; includes desired-state handling, not pure solver time. First step excluded; single run, not real-time certification.",
        "collision": collision,
        "scope": "Static floor-level tabletop proxies, fixed objects, no contact authorized. Original floor/joint/self/singularity constraints and elastic solver/gains unchanged. Ground-truth geometry, no calibrated uncertainty margin.",
    }
    result["task_success"] = bool(result["complete"] and result["within_2cm_at_end"]
                                  and not collision["any_sampled_scene_violation"] and not result["any_sampled_negative_h2"])
    trajectory = {"qpos": qpos, "qvel": qvel, "ee_m": ee, "torques_Nm": np.asarray(torques),
                  "object_distance_m": distances, "time_s": np.arange(len(qpos))*env.dt}
    if runtime is not None:
        runtime.finish(result, trajectory)
    return result, trajectory
