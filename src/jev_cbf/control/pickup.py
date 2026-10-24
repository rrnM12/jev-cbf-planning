"""Static-clutter pickup using FRAX's original nominal torque controller.

The new code owns task targets, gripper actuation, scene construction, and
contact-aware evaluation, not an alternative arm controller. The target is a
free rigid body: no weld, attachment, teleportation, or scripted target motion.
"""

from jev_cbf.paths import ROOT
from copy import deepcopy
import json
import numpy as np
from jev_cbf.selection.validation import validate_selection
from jev_cbf.geometry.covers import make_sphere_covers
from jev_cbf.selection.runtime import ConstraintRuntime
from jev_cbf.selection.context import robot_context
from jev_cbf.control.simulation import scene_spec
from jev_cbf.io import save
from jev_cbf.control.cbf import guarded_cbf, sphere_runtime_config
from jev_cbf.control.model import panda_spec
from jev_cbf.evaluation import pickup_collision_diagnostics

DEFAULTS = {
    "duration_s": 20., "control_dt_s": .002,
    "approach_height_m": .25, "grasp_height_m": .095,
    "lift_distance_m": .08, "waypoint_tolerance_m": .012,
    "close_duration_s": 1., "pickup_bottom_lift_m": .05,
    "pickup_confirmation_s": .25, "query_period_s": 2.,
    "forecast_horizon_s": 3., "scene_guard_m_s2": .05,
    "pregrasp_clearance_m": .04, "alignment_tolerance_m": .005,
    "alignment_rotation_tolerance_rad": .08, "alignment_speed_m_s": .02,
}

def target_body(mj, robot_spec, target):
    """Append AFTER the arm/fingers so the original first-seven-joints API holds."""
    body = robot_spec.worldbody.add_body(name="pickup_object", pos=target["center_m"])
    body.add_freejoint(name="pickup_free")
    half = np.asarray(target["size_m"])/2
    box = target["shape"] == "box"
    body.add_geom(name=f"scene/{target['id']}", type=mj.mjtGeom.mjGEOM_BOX if box else mj.mjtGeom.mjGEOM_CYLINDER,
                  size=half if box else [half[0], half[2], 0.], mass=target.get("mass_kg", .05),
                  rgba=[.95, .48, .12, 1.], friction=[1., .005, .0001], condim=4)
    return robot_spec

def finger_envelope_spheres(mj, model, robot):
    """Bound native finger collision hulls over their whole 0–4 cm joint travel.

    In the EE frame each finger translates linearly. Bounds of box corners and
    mesh vertices at both travel endpoints bound the swept convex geometry.
    """
    from itertools import product
    data = mj.MjData(model)
    data.qpos[:7] = robot.default_configuration
    transform = np.asarray(robot.ee_transform(robot.default_configuration))
    clouds = {name: [] for name in ("left_finger", "right_finger")}
    for aperture in (0., .04):
        data.qpos[7:9] = aperture
        mj.mj_forward(model, data)
        for geom in range(model.ngeom):
            name = model.body(int(model.geom_bodyid[geom])).name
            if name not in clouds or not model.geom_contype[geom]:
                continue
            if model.geom_type[geom] == mj.mjtGeom.mjGEOM_BOX:
                points = np.array(list(product((-1, 1), repeat=3)))*model.geom_size[geom]
            elif model.geom_type[geom] == mj.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[geom]
                start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                points = model.mesh_vert[start:start+count]
            else:
                raise ValueError("unhandled native finger collision primitive")
            world = points @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
            clouds[name].append((world-transform[:3, 3]) @ transform[:3, :3])
    centers, radii = [], []
    for cloud in clouds.values():
        points = np.concatenate(cloud)
        lower, upper = points.min(axis=0), points.max(axis=0)
        cover = make_sphere_covers((upper-lower)/2, (4,))[0]
        centers.extend(np.asarray(cover.centers_m)+(upper+lower)/2)
        radii.extend(cover.radii_m)
    return np.array(centers), np.array(radii)

class PickupRuntime(ConstraintRuntime):
    """Task hooks layered over the existing selection worker and CBF adapters."""

    def __init__(self, scene, settings, request, initial, mode, output, *, live=False):
        super().__init__(scene, settings, request, initial, np.zeros(7), "all" if mode in ("rule", "geometry", "all_fine") else mode, output, live=live)
        self.mode = mode
        self.target_id = scene["target_id"]
        self.target = next(o for o in scene["objects"] if o["id"] == self.target_id)
        self.phase, self.phase_began, self.task_done = "approach", 0., False
        self.phase_events, self.phase_trace, self.target_trace, self.goal_trace = [], [], [], []
        self.bottom_trace = []
        self.verified_since = None
        self.next_rule_at = 0.
        self.startup_checked = False
        self.startup_trials = []
        # The task contract, not model authority, controls intended contact.
        self.policy[self.target_id] = None if mode == "off" else "box-split-4"

    def robot_spec(self, mj, frax_dir):
        from jev_cbf.upstream import load_upstream
        from jev_cbf.paths import ROOT
        demo, _ = load_upstream(frax_dir, ROOT.parent/"cbfpy")
        self.robot_model = demo.load_panda()
        return target_body(mj, panda_spec(mj, frax_dir, profile="frax_gripper", robot=self.robot_model), self.target)

    def scene_spec(self, mj, frax_dir, state, centers, radii):
        clutter = deepcopy(state)
        clutter["objects"] = [o for o in self.scene["objects"] if o["id"] != self.target_id]
        spec = scene_spec(mj, frax_dir, clutter, centers, radii)
        spec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST
        # The upstream plane is the support surface; the base is mounted on it.
        spec.geom("target_geom").rgba = [1., 0., 0., .15]
        return spec

    def initialize(self, env):
        self.env = env
        self.free_q = int(env.model.joint("pickup_free").qposadr[0])
        self.free_v = int(env.model.joint("pickup_free").dofadr[0])
        if self.free_q != 9 or env.model.nu != 8:
            raise ValueError("pickup requires seven arm joints, two fingers, then the free target")
        env.data.qpos[7:9] = .04
        env.data.ctrl[7] = 255.
        self.target_body_id = env.model.body("pickup_object").id
        self.finger_bodies = {env.model.body(n).id for n in ("left_finger", "right_finger")}
        self.initial_target_center = np.array(self.target["center_m"])
        import jax
        import jax.numpy as jnp
        self.tool_spheres = finger_envelope_spheres(__import__("mujoco"), env.model, self.robot_model)
        self.robot_sphere_count = 21+len(self.tool_spheres[1])
        robot, points = self.robot_model, jnp.asarray(self.tool_spheres[0])
        self.tool_predict = jax.jit(jax.vmap(lambda q: points @ robot.ee_transform(q)[:3, :3].T + robot.ee_transform(q)[:3, 3]))
        self.after_step(env)
        if self.mode == "repeated" and self.live:
            env.real_time = True

    def setup(self, demo, robot, guard):
        self.demo, self.robot = demo, robot
        self.config = sphere_runtime_config(demo, robot, self.capacity, pairwise=True, tool_spheres=self.tool_spheres)
        self.cbf = guarded_cbf(demo, self.config, scene_guard=guard)
        self.h2_fn, self.h1_fn = demo.jax.jit(self.config.h_2), demo.jax.jit(self.cbf.h)
        self.ee_kinematics = demo.jax.jit(lambda q,v: (robot.ee_transform(q),robot.ee_jacobian(q)@v))
        self.sphere_predict = demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
        z = np.r_[robot.default_configuration, np.zeros(7), 0.]
        args = self.parameters(0.)
        self.cbf.safety_filter(z, np.zeros(7), *args).block_until_ready()
        self.h2_fn(z, *args).block_until_ready()
        self.h1_fn(z, *args).block_until_ready()
        self.ee_kinematics(robot.default_configuration,np.zeros(7))[0].block_until_ready()

    def positions(self):
        positions = np.array([o["center_m"] for o in self.scene["objects"]], dtype=float)
        index = [o["id"] for o in self.scene["objects"]].index(self.target_id)
        positions[index] = self.env.data.qpos[self.free_q:self.free_q+3]
        return positions

    def parameters(self, at, policy=None):
        # Reuse cover packing. Clutter is static; target sphere poses/velocities
        # follow measured native rigid-body kinematics, without moving the body.
        policy = self.policy if policy is None else policy
        args = list(super().parameters(at, policy))
        enabled = np.broadcast_to(args[3], (getattr(self, "robot_sphere_count", 29), self.capacity)).copy()
        index = [o["id"] for o in self.base["state"]["objects"]].index(self.target_id)
        slots = slice(index*self.object_capacity, index*self.object_capacity+4)
        candidate = next(c for c in self.base["state"]["objects"][index]["candidates"] if c["id"] == "box-split-4")
        rotation = self.env.data.xmat[self.target_body_id].reshape(3, 3)
        offsets = np.asarray(candidate["centers_object_m"]) @ rotation.T
        args[0][slots] = offsets + self.positions()[index]
        args[1][slots] = candidate["radii_m"]
        import mujoco as mj
        velocity = np.zeros(6)
        mj.mj_objectVelocity(self.env.model, self.env.data, mj.mjtObj.mjOBJ_BODY,
                             self.target_body_id, velocity, 0)
        args[2][slots] = velocity[3:] + np.cross(velocity[:3], offsets)
        enabled[:, slots] = False
        if self.mode != "off":
            enabled[:16, slots] = True
        args[3] = enabled
        return tuple(args)

    def install(self, report, q, v, at, *, source):
        super().install(report, q, v, at, source=source)
        self.policy[self.target_id] = "box-split-4"
        self.events[-1]["policy"] = self.policy.copy()
        self.events[-1]["target_contact_policy"] = "Task-designated target: arm-only barriers throughout pickup; intended tool contact monitored natively. Jev cannot grant contact with other objects."

    def admissible_cover_policy(self, q, v, at):
        """All-object initialization, not semantic selection or route planning.

        Constraints separate by obstacle, so check each candidate with mandatory
        target/intrinsic protections, then admit the combined set. Every candidate
        covers its supplied shape; perception does not guarantee the true shape.
        """
        policy = dict.fromkeys(self.policy)
        policy[self.target_id] = "box-split-4"
        z = np.r_[q, v, at]
        trials = []
        self.startup_trials = trials
        for obj in self.base['state']['objects']:
            if obj['id'] == self.target_id:
                continue
            candidates = sorted(obj['candidates'], key=lambda c: (c['excess_volume_m3'], c['primitive_count'], c['id']))
            chosen = None
            for candidate in candidates:
                single = dict.fromkeys(policy)
                single[self.target_id], single[obj['id']] = "box-split-4", candidate['id']
                args = self.parameters(at, single)
                h2, h1 = np.asarray(self.h2_fn(z, *args)), np.asarray(self.h1_fn(z, *args))
                finite = bool(np.isfinite(h2).all() and np.isfinite(h1).all())
                valid = finite and h2.min() >= 0 and h1.min() >= 0
                trials.append(dict(object_id=obj['id'], candidate_id=candidate['id'], admissible=bool(valid),
                                   minimum_h2=float(h2.min()) if finite else None,
                                   minimum_h1=float(h1.min()) if finite else None))
                if valid:
                    chosen = candidate['id']
                    break
            if chosen is None:
                raise ValueError(f"no currently RD2-admissible full-shape candidate for {obj['id']}")
            policy[obj['id']] = chosen
        return policy, trials

    def check_startup(self, q, v, at):
        """Reject unsupported initial models before executing any arm control."""
        event = dict(at_s=float(at), mode=self.mode,
                     scope='Initial modeled h2 and transformed h1 check, not a feasibility, perception-coverage or hardware safe-stop certificate.')
        try:
            if self.mode in ('off', 'all'):
                policy, trials = self.policy.copy(), []  # Preserve fixed-four baseline semantics.
            else:
                policy, trials = self.admissible_cover_policy(q, v, at)
            event.update(self.admit(policy, q, v, at), candidate_checks=trials)
            self.startup_checked = True
        except ValueError as error:
            event.update(status='rejected_before_actuation', error=str(error), policy=self.policy.copy(),
                         candidate_checks=self.startup_trials)
            save(self.output/'startup.json', event)
            raise ValueError('pickup startup rejected before actuation; see startup.json') from error
        save(self.output/'startup.json', event)

    def close(self):
        """Release a selector worker on an aborted run, without substitute control."""
        if self.executor is not None:
            self.executor.shutdown(wait=True, cancel_futures=True)

    def before_control(self, env):
        if not self.startup_checked:
            self.check_startup(env.data.qpos[:7], env.data.qvel[:7], float(env.data.time))
        super().before_control(env)
        at = float(env.data.time)
        if self.mode == "geometry" and not self.initialized_selection:
            event = {"kind":"selection","source":"initial/selection.json","snapshot_at_s":0.,"applied_at_s":at,
                     "note":"All objects retained; only recorded conditional geometry is used. This isolates geometry from role selection."}
            try:
                proposals = validate_selection(self.initial_report)
                if not self.geometry_context_matches(self.initial_report):
                    raise ValueError("geometry report context differs from the frozen scene")
                policy = {p["object_id"]:p["conditional_candidate_id"] for p in proposals}
                policy[self.target_id] = "box-split-4"
                event.update(self.admit(policy, env.data.qpos[:7], env.data.qvel[:7], at))
            except (ValueError, KeyError, TypeError) as error:
                event.update(status="rejected",error=str(error),policy=self.policy.copy())
            self.events.append(event)
            save(self.output/"events.json",self.events)
            self.initialized_selection = True
        if self.mode not in ("rule","all_fine") or at+1e-9 < self.next_rule_at:
            return
        request = self.make_request(env.data.qpos[:7], env.data.qvel[:7], at)
        policy = {}
        for obj in request["state"]["objects"]:
            # Explicit geometric baseline: retain objects for which any supplied
            # conservative cover intersects the sampled forecast. Choose the
            # least global excess-volume candidate, then the smaller count.
            best = min(obj["candidates"], key=lambda c: (c["excess_volume_m3"], c["primitive_count"], c["id"]))
            risk = min(c["forecast_robot_sphere_clearance_m"] for c in obj["candidates"]) <= 0
            policy[obj["id"]] = best["id"] if risk or self.mode=="all_fine" else None
        policy[self.target_id] = "box-split-4"
        event = {"kind": "selection", "source": "deterministic geometry rule; no model response" if self.mode=="rule" else "all objects / least-excess geometry; no model response",
                 "applied_at_s": at, "snapshot_at_s": at}
        try:
            event.update(self.admit(policy, env.data.qpos[:7], env.data.qvel[:7], at))
        except ValueError as error:
            event.update(status="rejected", error=str(error), policy=self.policy.copy())
        self.events.append(event)
        save(self.output/"events.json", self.events)
        self.next_rule_at += self.settings["query_period_s"]

    def geometry_context_matches(self,report):
        return report['request']['state']==self.base['state']

    def waypoint(self):
        goal = np.array([*self.initial_target_center[:2], self.settings["grasp_height_m"]])
        if self.phase == "approach":
            goal[2] = self.settings["approach_height_m"]
        elif self.phase == "pregrasp":
            goal[2] = self.initial_target_center[2]+self.target["size_m"][2]/2+self.settings["pregrasp_clearance_m"]
        elif self.phase == "lift":
            goal[2] += self.settings["lift_distance_m"]
        return goal

    def desired_ee_state(self, env):
        goal = self.waypoint()
        env.data.mocap_pos[env.mocap_id] = goal
        return np.r_[goal, env.des_rot.ravel(), np.zeros(6)]

    def apply_auxiliary_control(self, env):
        env.data.ctrl[7] = 255. if self.phase in ("approach", "pregrasp", "descend") else 0.

    def after_step(self, env):
        import mujoco as mj
        mj.mj_forward(env.model, env.data)
        at = float(env.data.time)
        target_center = env.data.qpos[self.free_q:self.free_q+3].copy()
        rotation = env.data.xmat[self.target_body_id].reshape(3, 3)
        half = np.asarray(self.target["size_m"])/2
        support = (np.abs(rotation[2]) @ half if self.target["shape"] == "box"
                   else half[0]*np.linalg.norm(rotation[2, :2]) + half[2]*abs(rotation[2, 2]))
        bottom = float(target_center[2]-support)
        if hasattr(self, "robot"):
            transform, twist = map(np.asarray,self.ee_kinematics(env.data.qpos[:7],env.data.qvel[:7]))
            ee = transform[:3,3]
            close = np.linalg.norm(ee-self.waypoint()) <= self.settings["waypoint_tolerance_m"]
            angle = np.arccos(np.clip((np.trace(env.des_rot.T@transform[:3,:3])-1)/2,-1,1))
            aligned = (np.linalg.norm(ee-self.waypoint()) <= self.settings["alignment_tolerance_m"]
                       and np.linalg.norm(twist[:3]) <= self.settings["alignment_speed_m_s"]
                       and angle <= self.settings["alignment_rotation_tolerance_rad"])
            target_geom = env.model.geom(f"scene/{self.target_id}").id
            fingers = set()
            for contact in env.data.contact:
                a, b = int(contact.geom1), int(contact.geom2)
                if target_geom in (a,b) and contact.dist <= 0:
                    body = int(env.model.geom_bodyid[b if a==target_geom else a])
                    if body in self.finger_bodies:
                        fingers.add(body)
            next_phase = None
            if self.phase == "approach" and close:
                next_phase = "pregrasp"
            elif self.phase == "pregrasp" and aligned:
                next_phase = "descend"
            elif self.phase == "descend" and aligned:
                next_phase = "close"
            elif (self.phase == "descend" and np.linalg.norm(ee[:2]-self.initial_target_center[:2]) > 2*self.settings["alignment_tolerance_m"]
                  and ee[2] < self.initial_target_center[2]+self.target["size_m"][2]/2+self.settings["pregrasp_clearance_m"]-self.settings["alignment_tolerance_m"]):
                # Back off above the target if a CBF deflects the open gripper
                # laterally. Do not poke a cylinder while trying to center it.
                next_phase = "pregrasp"
            elif (self.phase == "close" and at-self.phase_began >= self.settings["close_duration_s"]
                  and fingers == self.finger_bodies):
                next_phase = "lift"
            if next_phase:
                self.phase_events.append({"at_s": at, "from": self.phase, "to": next_phase})
                self.phase, self.phase_began = next_phase, at
            # Require both fingers touching a genuinely lifted target for a short
            # verification window. This is not a transport/placement phase.
            lifted = bottom >= self.settings["pickup_bottom_lift_m"]
            if self.phase == "lift" and lifted and fingers == self.finger_bodies:
                if self.verified_since is None:
                    self.verified_since = at
                self.task_done = at-self.verified_since >= self.settings["pickup_confirmation_s"]
            else:
                self.verified_since = None
        self.phase_trace.append(self.phase)
        self.target_trace.append(target_center)
        self.bottom_trace.append(bottom)
        self.goal_trace.append(self.waypoint())

    def collision_diagnostics(self, mj, model, qpos, qvel):
        summary, distances, self.contact_trace = pickup_collision_diagnostics(mj, model, qpos, qvel, self.scene["objects"], self.target_id)
        return summary, distances

    def filter(self, z, nominal, at):
        args = self.parameters(float(at))
        self.params_trace.append(args)
        self.counts.append(int(np.count_nonzero(args[3])))
        return self.cbf.safety_filter(np.r_[z, at], nominal, *args)

    def finish(self, result, trajectory):
        # The parent packs all parameter/event traces and joins any model worker.
        super().finish(result, trajectory)
        lifted = np.asarray(self.target_trace)[:, 2] - self.initial_target_center[2]
        result.update(moving_objects=False, task="static_clutter_pickup", target_id=self.target_id,
                      scene_sphere_count=int(np.any(self.params_trace[-1][3], axis=0).sum()),
                      tool_envelope_sphere_count=len(self.tool_spheres[1]),
                      robot_sphere_count=self.robot_sphere_count,
                      grasp_verified=bool(self.task_done), phase_events=self.phase_events,
                      final_target_center_lift_m=float(lifted[-1]),
                      maximum_target_center_lift_m=float(lifted.max()),
                      final_target_bottom_m=float(self.bottom_trace[-1]),
                      scope="One static-clutter pickup task. Native articulated fingers and free-body frictional target, no weld/teleport. Original FRAX arm nominal torque policy/protections. Finger/contact/payload dynamics differ from the 7-DOF CBF model: empirical simulation evidence, not a contact-safety proof.",
                      success_definition="Verified lift with bilateral finger contact for the configured confirmation window; no unwanted robot-clutter, arm/palm-target, or target-clutter contact; finite complete run and nonnegative modeled barriers.")
        if self.mode == "rule":
            result["selection_source"] = "deterministic geometric forecast/excess-volume baseline; no API or fabricated model answers"
        elif self.mode == "geometry":
            result["selection_source"] = "all-object baseline with recorded conditional Jev geometry; role decisions ignored; shared initial answer, no additional model call"
        elif self.mode == "all_fine":
            result['selection_source']='all clutter retained with least estimated excess-volume candidate; no API or fabricated model answer'
        result["task_success"] = bool(result["complete"] and self.task_done and not result["collision"]["any_sampled_scene_violation"] and not result["any_sampled_negative_h2"])
        trajectory.update(target_center_m=np.asarray(self.target_trace), phase=np.asarray(self.phase_trace),
                          tool_centers_ee_m=self.tool_spheres[0], tool_radii_m=self.tool_spheres[1],
                          target_bottom_m=np.asarray(self.bottom_trace),
                          ee_goal_m=np.asarray(self.goal_trace), **self.contact_trace)
        save(self.output/"task_events.json", self.phase_events)

    def object_history(self, times):
        positions = np.tile(np.array([o["center_m"] for o in self.scene["objects"]]), (len(times), 1, 1))
        index = [o["id"] for o in self.scene["objects"]].index(self.target_id)
        positions[:, index] = self.target_trace
        return positions

    def make_request(self, q, v, at):
        request = deepcopy(self.base)
        request["state"].update(snapshot_time_s=float(at), phase=self.phase)
        request["state"]["motion"].update(start_m=np.asarray(self.robot.ee_transform(q))[:3, 3].tolist(),
                                            current_waypoint_m=self.waypoint().tolist())
        current = robot_context(self.robot, {"qpos": q[None], "qvel": v[None]})
        current["limitations"] = self.base["state"]["robot_context"]["limitations"]
        current["scene_pairing"] = self.base["state"]["robot_context"]["scene_pairing"]
        current["scene_pairing"] += " Eight added EE-frame sphere envelopes conservatively cover both fingers over their full travel and protect against clutter; only their target pairs are exempt."
        tool_positions = np.asarray(self.tool_predict(np.asarray(q)[None]))[0]
        current["moving_link_sphere_centers_world_m"].extend(tool_positions.tolist())
        current["moving_link_sphere_radii_m"].extend(self.tool_spheres[1].tolist())
        current["sphere_groups"].append({"joint": "finger_full_travel_envelopes", "sphere_indices": list(range(21, self.robot_sphere_count))})
        request["state"]["robot_context"] = current
        reference = request["state"]["motion"].get("phase_joint_reference_rad", {})
        qgoal = np.array(reference.get(self.phase, q.tolist()))
        fractions = np.linspace(0., 1., 16)
        qs = q[None]*(1-fractions[:, None]) + qgoal[None]*fractions[:, None]
        path = np.concatenate([np.asarray(self.sphere_predict(qs)), np.asarray(self.tool_predict(qs))], axis=1)
        robot_radii = np.r_[np.asarray(self.robot.link_collision_data(q)[1]), self.tool_spheres[1]]
        request["state"]["motion"]["forecast"] = {
            "kind": "Heuristic sampled joint interpolation toward this phase's unobstructed nominal reference; not an obstacle-aware plan, actual policy rollout, or safety guarantee.",
            "samples": 16, "joint_goal_rad": qgoal.tolist(),
        }
        terminal = {}
        if request["state"].get("terminal_clearance_features"):
            phases = ("pregrasp","descend","close","lift")
            for phase in phases:
                if phase in reference:
                    state_q = np.asarray(reference[phase])[None]
                    terminal[phase] = np.concatenate([np.asarray(self.sphere_predict(state_q)),
                                                      np.asarray(self.tool_predict(state_q))],axis=1)[0]
            request["state"]["motion"]["terminal_clearance_scope"] = (
                "Per-candidate clearance at independently recorded unobstructed pregrasp/grasp/lift joint references. "
                "Negative means this COVER blocks that reference posture, not that the physical object collides "
                "or that no alternate posture/route exists. All finger-travel envelopes included for clutter.")
        for obj, position in zip(request["state"]["objects"], self.positions(), strict=True):
            obj["center_m"] = position.tolist()
            for candidate in obj["candidates"]:
                centers = np.asarray(candidate["centers_object_m"]) + position
                if obj["id"] == self.target_id:
                    centers = np.asarray(candidate["centers_object_m"]) @ self.env.data.xmat[self.target_body_id].reshape(3, 3).T + position
                    obj["measured_rotation_matrix"] = self.env.data.xmat[self.target_body_id].reshape(3, 3).tolist()
                distances = np.linalg.norm(path[:, :, None]-centers[None, None], axis=-1)-robot_radii[None, :, None]-np.asarray(candidate["radii_m"])[None, None]
                candidate.update(current_robot_sphere_clearance_m=float(distances[0].min()),
                                 forecast_robot_sphere_clearance_m=float(distances.min()),
                                 robot_primitive_constraint_pairs=int(len(robot_radii)*len(centers)))
                if terminal and obj["id"] != self.target_id:
                    candidate["terminal_reference_clearance_m"] = {
                        phase:float((np.linalg.norm(points[:,None]-centers[None],axis=-1)
                                     -robot_radii[:,None]-np.asarray(candidate["radii_m"])[None]).min())
                        for phase,points in terminal.items()}
                request["questions"][f"geometry/{obj['id']}"]["criteria"][candidate["id"]] = deepcopy(candidate)
        return json.loads(json.dumps(request, allow_nan=False))
