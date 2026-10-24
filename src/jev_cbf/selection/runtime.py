"""Ground-truth online selection and runtime CBF updates for the Panda demo.

The policy stays in FRAX. The model runs in one background thread; physics never
waits for it. Runtime arrays have fixed capacity to avoid recompiling on updates.
This prototype changes enabled rows, not the allocated QP dimension.
"""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import time
import numpy as np
from jev_cbf.selection.client import ask_jev
from jev_cbf.selection.validation import decode_response, validate_selection
from jev_cbf.selection.context import robot_context
from jev_cbf.io import read, save
from jev_cbf.control.cbf import guarded_cbf, sphere_runtime_config

class FaultInjectedJev:
    """Explicit robustness wrapper; injected failures make NO HTTP call."""
    def __init__(self,ask,config,output):
        self.ask,self.config,self.output=ask,config,output
        self.index=0;self.events=[]
        delay=config.get('delay_s',0.)
        if not np.isfinite(delay) or not 0<=delay<=30:raise ValueError('fault-study delay must be 0–30 s')

    def __call__(self,request):
        self.index+=1
        event={'logical_call':self.index,'snapshot_at_s':request['state'].get('snapshot_time_s',0.)}
        if self.index in self.config.get('fail_calls',[]):
            event['status']='injected unavailable response; no provider call or budget charge'
            self.events.append(event);save(self.output/'model_fault_events.json',self.events)
            raise ValueError('INJECTED model-unavailable fault; no provider request was sent')
        delay=self.config.get('delay_s',0.) if self.index in self.config.get('delay_calls',[]) else 0.
        if delay:time.sleep(delay)
        event.update(status='provider adapter called',injected_delay_s=delay)
        self.events.append(event);save(self.output/'model_fault_events.json',self.events)
        return self.ask(request)

def object_motion(objects, at):
    """Continuous positions and right-hand velocities of a scripted fixture."""
    positions, velocities = [], []
    for obj in objects:
        position = np.array(obj["center_m"], dtype=float)
        velocity = np.zeros(3)
        previous_end = 0.
        for segment in obj.get("motion", []):
            start, end = segment["start_s"], segment["end_s"]
            speed = np.asarray(segment["velocity_m_s"], dtype=float)
            if start < previous_end or end <= start or speed.shape != (3,) or not np.isfinite(speed).all():
                raise ValueError("motion segments must be finite, ordered, and non-overlapping")
            position += speed * np.clip(at - start, 0., end - start)
            if start <= at < end:
                velocity = speed
            previous_end = end
        positions.append(position)
        velocities.append(velocity)
    return np.asarray(positions), np.asarray(velocities)

def query_saved(folder, request, *, ask=ask_jev):
    """One logical SDK attempt per snapshot, recorded before sending.

    Failed/interrupted attempts are not repeated. Reusing an answer requires
    exact request equality; no fabricated answer or confidence cutoff.
    """
    response_path, attempt_path = folder / "selection.json", folder / "attempt.json"
    if response_path.exists():
        report = read(response_path)
        if report["request"] != request:
            raise ValueError("saved model input differs from this snapshot")
        return report
    if attempt_path.exists():
        raise ValueError("snapshot already attempted; interrupted call will not be repeated")
    save(folder / "request.json", request)
    began = time.time()
    save(attempt_path, {"status": "started", "started_unix_s": began})
    report = {"mode": "live_requested_dynamic", "request": request}
    try:
        report["response"] = ask(request)
        report["proposals"] = decode_response(request, report["response"])
        report["mode"] = "live_jev_dynamic"
    except (ValueError, KeyError, TypeError) as error:
        report["error"] = str(error)
    report["elapsed_seconds"] = time.time() - began
    save(response_path, report)
    save(attempt_path, {"status": "failed" if "error" in report else "success",
                        "started_unix_s": began, "elapsed_seconds": report["elapsed_seconds"]})
    return report

def motion_request(base, robot, q, v, positions, velocities, at, goal_q, horizon, sphere_predict):
    """Measured current state + explicitly heuristic kinematic intent forecast.

    The future scene script is NOT passed in. Obstacle velocities are held
    constant over this short forecast; robot joints interpolate toward a
    free-space goal configuration. This is not a simulated policy prediction.
    """
    request = deepcopy(base)
    state = request["state"]
    state["snapshot_time_s"] = float(at)
    state["description"] = "Ground-truth current object poses and velocities in a dynamic no-contact reaching simulation. No camera/perception. Future scripted changes are not known to the selector."
    state["robot_context"] = robot_context(robot, {"qpos": np.asarray(q)[None], "qvel": np.asarray(v)[None]})
    state["motion"]["start_m"] = np.asarray(robot.ee_transform(q))[:3, 3].tolist()
    times = np.linspace(0., horizon, 16)
    fractions = times / horizon
    qs = q[None] * (1-fractions[:, None]) + goal_q[None] * fractions[:, None]
    path = np.asarray(sphere_predict(qs))
    robot_radii = np.asarray(robot.link_collision_data(q)[1])
    state["motion"]["forecast"] = {
        "kind": "Heuristic joint interpolation toward the original controller's free-space goal configuration, not a policy rollout or safety guarantee. Object velocity extrapolation is constant, with unknown future acceleration/changes.",
        "horizon_s": horizon, "joint_goal_rad": goal_q.tolist(),
        "sample_offsets_s": times.tolist(), "sample_ee_goal_m": state["motion"]["goal_m"],
    }
    for obj, position, velocity in zip(state["objects"], positions, velocities, strict=True):
        obj["center_m"], obj["velocity_m_s"] = position.tolist(), velocity.tolist()
        for candidate in obj["candidates"]:
            centers = (np.asarray(candidate["centers_object_m"])[None]
                       + position[None, None] + times[:, None, None]*velocity[None, None])
            clearance = (np.linalg.norm(path[:, :, None] - centers[:, None], axis=-1)
                         - robot_radii[None, :, None] - np.asarray(candidate["radii_m"])[None, None])
            candidate.update(initial_robot_sphere_clearance_m=float(clearance[0].min()),
                             forecast_robot_sphere_clearance_m=float(clearance.min()),
                             forecast_end_robot_sphere_clearance_m=float(clearance[-1].min()),
                             robot_primitive_constraint_pairs=int(len(robot_radii)*len(candidate["radii_m"])))
            request["questions"][f"geometry/{obj['id']}"]["criteria"][candidate["id"]] = deepcopy(candidate)
    return json.loads(json.dumps(request, allow_nan=False))

class ConstraintRuntime:
    """Hook object used by run_static.run_case, not a second robot controller."""

    def __init__(self, scene, config, base_request, initial_report, goal_q, mode, output, *, live=False):
        if mode not in ("off", "all", "once", "repeated"):
            raise ValueError("unknown online comparison mode")
        self.scene, self.settings, self.base = scene, config, base_request
        self.mode, self.output, self.goal_q = mode, output, np.asarray(goal_q)
        self.object_capacity = max(4,max(len(c["radii_m"]) for o in base_request["state"]["objects"] for c in o["candidates"]))
        self.capacity = self.object_capacity*len(scene["objects"])
        self.policy = {obj["id"]: "box-split-4" for obj in base_request["state"]["objects"]}
        if mode == "off":
            self.policy = dict.fromkeys(self.policy)
        self.initial_report, self.live = initial_report, live
        self.initialized_selection = False
        self.params_trace, self.events, self.counts = [], [], []
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="jev-selection") if live and mode == "repeated" else None
        self.future = None
        self.next_query = config["query_period_s"]
        self.query_index = 1

    def initialize(self, env):
        self.env = env
        self.object_geom_ids = [env.model.geom(f"scene/{o['id']}").id for o in self.scene["objects"]]
        self.object_mocap_ids = env.model.body_mocapid[env.model.geom_bodyid[self.object_geom_ids]]
        if np.any(self.object_mocap_ids < 0):
            raise ValueError("online fixtures require mocap object bodies")
        self.after_step(env)
        if self.mode == "repeated" and self.live:
            # Pace this one rollout to wall time so measured SDK latency really
            # elapses while physics keeps stepping. Other modes need no pacing.
            env.real_time = True

    def setup(self, demo, robot, guard):
        self.demo, self.robot = demo, robot
        self.config = sphere_runtime_config(demo, robot, self.capacity)
        self.cbf = guarded_cbf(demo, self.config, scene_guard=guard)
        self.sphere_predict = demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
        self.h2_fn = demo.jax.jit(self.config.h_2)
        self.h1_fn = demo.jax.jit(self.cbf.h)
        args = self.parameters(0.)
        z = np.r_[robot.default_configuration, np.zeros(7), 0.]
        self.cbf.safety_filter(z, np.zeros(7), *args).block_until_ready()
        self.h2_fn(z, *args).block_until_ready()
        self.h1_fn(z, *args).block_until_ready()
        self.sphere_predict(np.tile(robot.default_configuration, (16, 1))).block_until_ready()

    def parameters(self, at, policy=None):
        policy = self.policy if policy is None else policy
        positions, velocities = object_motion(self.scene["objects"], at)
        centers, radii, speeds = np.full((self.capacity, 3), 10.), np.full(self.capacity, .01), np.zeros((self.capacity, 3))
        enabled = np.zeros(self.capacity, dtype=bool)
        for index, obj in enumerate(self.base["state"]["objects"]):
            candidate_id = policy[obj["id"]]
            if candidate_id is None:
                continue
            candidate = next(c for c in obj["candidates"] if c["id"] == candidate_id)
            n = len(candidate["radii_m"])
            if n > self.object_capacity:
                raise ValueError("candidate exceeds the experiment's frozen object capacity")
            slots = slice(self.object_capacity*index, self.object_capacity*index+n)
            centers[slots] = np.asarray(candidate["centers_object_m"]) + positions[index]
            radii[slots], speeds[slots], enabled[slots] = candidate["radii_m"], velocities[index], True
        return centers, radii, speeds, enabled, float(at)

    def install(self, report, q, v, at, *, source):
        event = {"kind": "selection", "source": source, "applied_at_s": float(at),
                 "snapshot_at_s": report.get("request", {}).get("state", {}).get("snapshot_time_s"),
                 "elapsed_sdk_s": report.get("elapsed_seconds")}
        try:
            proposals = validate_selection(report)
            current_objects = self.base["state"]["objects"]
            supplied = report["request"]["state"]["objects"]
            if report["request"]["state"]["motion"]["goal_m"] != self.base["state"]["motion"]["goal_m"]:
                raise ValueError("task goal changed; old answer not applicable")
            if [(o['id'],o['shape'],o['size_m']) for o in current_objects] != [(o['id'],o['shape'],o['size_m']) for o in supplied]:
                raise ValueError("object identity/geometry changed; old answer not applicable")
            policy = self.policy.copy()
            for proposal in proposals:
                if proposal["role"] == "review":
                    continue  # Explicitly retain previous protection, never turn review into omission.
                policy[proposal["object_id"]] = proposal["candidate_id"]
            event.update(self.admit(policy, q, v, at), proposals=proposals)
        except (ValueError, KeyError, TypeError, StopIteration) as error:
            event.update(status="rejected", error=str(error), policy=self.policy.copy())
        self.events.append(event)
        save(self.output / "events.json", self.events)

    def admit(self, policy, q, v, at):
        """Shared admission gate for model proposals or explicitly labeled rules."""
        if set(policy) != set(self.policy):
            raise ValueError("policy object IDs differ from the current scene")
        for obj in self.base["state"]["objects"]:
            if policy[obj["id"]] is not None and policy[obj["id"]] not in {c["id"] for c in obj["candidates"]}:
                raise ValueError("policy contains an unknown candidate ID")
        args, z = self.parameters(at, policy), np.r_[q, v, at]
        h2, h1 = np.asarray(self.h2_fn(z, *args)), np.asarray(self.h1_fn(z, *args))
        if not np.isfinite(h2).all() or not np.isfinite(h1).all() or h2.min() < 0 or h1.min() < 0:
            raise ValueError("proposed cover set is not currently RD2-admissible; previous set retained")
        event = dict(status="accepted", previous_policy=self.policy.copy(), policy=policy.copy(),
                     minimum_h2_before_install=float(h2.min()), minimum_h1_before_install=float(h1.min()))
        self.policy = dict(policy)
        return event

    def before_control(self, env):
        at = float(env.data.time)
        # Round the scheduler only, not positions: accumulated binary step error
        # must not dispatch just before a piecewise velocity change.
        at = round(at, 9)
        if self.mode in ("once", "repeated") and not self.initialized_selection:
            self.install(self.initial_report, env.data.qpos[:7], env.data.qvel[:7], at, source="initial/selection.json")
            self.initialized_selection = True
        if self.future is not None and self.future.done():
            try:
                report, source = self.future.result()
            except Exception as error:
                # Unexpected background failures must not escape into control,
                # and private error bodies/credentials must not enter reports.
                report, source = {"error": f"background worker failed ({type(error).__name__})"}, "background failure"
            self.future = None
            self.install(report, env.data.qpos[:7], env.data.qvel[:7], at, source=source)
        if self.executor and self.future is None and at + 1e-9 >= self.next_query and at < self.settings["duration_s"] - .01:
            q, v = env.data.qpos[:7].copy(), env.data.qvel[:7].copy()
            request = self.make_request(q, v, at)
            folder = self.output.parent / "queries" / f"{self.query_index:02d}"
            source = f"queries/{self.query_index:02d}/selection.json"
            self.query_index += 1
            self.next_query += self.settings["query_period_s"]

            def work():
                return query_saved(folder, request, **getattr(self, "query_options", {})), source

            self.future = self.executor.submit(work)
            self.events.append({"kind": "request_submitted", "at_s": at, "source": source})

    def make_request(self, q, v, at):
        positions, velocities = object_motion(self.scene["objects"], at)
        return motion_request(self.base, self.robot, q, v, positions, velocities, at,
                              self.goal_q, self.settings["forecast_horizon_s"], self.sphere_predict)

    def filter(self, z, nominal, at):
        args = self.parameters(round(float(at), 9))
        self.params_trace.append(args)
        self.counts.append(int(np.count_nonzero(args[3]))*21)
        return self.cbf.safety_filter(np.r_[z, at], nominal, *args)

    def after_step(self, env):
        positions, _ = object_motion(self.scene["objects"], round(float(env.data.time), 9))
        env.data.mocap_pos[self.object_mocap_ids] = positions
        # Recompute native geometry/contact data at the recorded endpoint.
        env_mj = __import__("mujoco")
        env_mj.mj_forward(env.model, env.data)

    def object_history(self, times):
        return np.asarray([object_motion(self.scene["objects"], round(float(t), 9))[0] for t in times])

    def diagnostics(self, zs, torques, dt):
        times = np.arange(len(torques))*dt
        pre = np.c_[zs[:-1], times]
        post = np.c_[zs[1:], times+dt]
        packed = [np.asarray([p[i] for p in self.params_trace]) for i in range(5)]
        vmap = self.demo.jax.jit(self.demo.jax.vmap(self.config.h_2))
        h2 = np.concatenate([np.asarray(vmap(pre, *packed)), np.asarray(vmap(post, *packed))])
        pair_count = getattr(self.config, "scene_pair_count", 21*self.capacity)
        self.scene_h2_trace = np.r_[h2[:len(times), 15:15+pair_count].min(axis=1),
                                   h2[-1, 15:15+pair_count].min()]
        vmap = self.demo.jax.jit(self.demo.jax.vmap(self.cbf.h))
        h1 = np.concatenate([np.asarray(vmap(pre, *packed)), np.asarray(vmap(post, *packed))])

        def residual(z, u, *args):
            return self.cbf.G_qp(z, u, *args) @ u - self.cbf.h_qp(z, u, *args)

        excess = np.asarray(self.demo.jax.jit(self.demo.jax.vmap(residual))(pre, torques, *packed))
        return h2, h1, excess

    def finish(self, result, trajectory):
        if self.future is not None:
            try:
                report, source = self.future.result()
            except Exception as error:
                report, source = {"error": f"background worker failed ({type(error).__name__})"}, "background failure"
            self.events.append({"kind": "late_response", "source": source,
                                "status": "not applied after rollout", "error": report.get("error")})
        if self.executor:
            self.executor.shutdown(wait=True)
        save(self.output / "events.json", self.events)
        result.update(mode=self.mode, selection_source="none" if self.mode in ("off", "all") else "recorded live Jev",
                      moving_objects=True,
                      scope="Dynamic scripted oracle poses/velocities; FRAX-matched ideal plant, original nominal policy and robot protections; constant-velocity RD2 obstacle model, finite sampling, elastic QP. No hardware or formal safety guarantee.",
                      included_objects=[{"object_id": k, "candidate_id": v} for k,v in self.policy.items() if v],
                      scene_sphere_count=int(self.counts[-1]//21),
                      scene_pair_constraint_count=int(self.counts[-1]),
                      enabled_scene_pair_counts={"minimum": min(self.counts), "mean": float(np.mean(self.counts)), "maximum": max(self.counts)},
                      allocated_scene_pair_count=getattr(self.config, "scene_pair_count", 21*self.capacity),
                      allocated_qp_note="Dormant constant-positive rows keep JAX/QP shape fixed; enabled counts are NOT a reduced allocated QP dimension or measured speedup.",
                      selection_events=self.events,
                      numerical_guard_note="0.05 m/s² scene inequality tightening is empirical sampled-control protection, not a perception margin or certified uncertainty bound.")
        if self.mode == "off":
            result["minimum_selected_scene_sphere_barrier_m"] = None
        trajectory.update(object_positions_m=self.object_history(trajectory["time_s"]),
                          cover_centers_m=np.asarray([p[0] for p in self.params_trace]+[self.parameters(trajectory["time_s"][-1])[0]]),
                          cover_radii_m=np.asarray([p[1] for p in self.params_trace]+[self.parameters(trajectory["time_s"][-1])[1]]),
                          cover_enabled=np.asarray([p[3] for p in self.params_trace]+[self.parameters(trajectory["time_s"][-1])[3]]),
                          enabled_scene_pairs=np.r_[self.counts, self.counts[-1]])
        trajectory["scene_h2_m"] = self.scene_h2_trace
