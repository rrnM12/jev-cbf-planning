"""Observation-only pickup task adapter around the unchanged FRAX/CBFpy path.

Plant truth is confined to construction, camera rendering, and offline scoring.
The fast task path consumes completed camera estimates and measured joint state.
"""

from jev_cbf.paths import ROOT
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import time
import numpy as np
from jev_cbf.selection.validation import validate_selection
from jev_cbf.selection.runtime import ConstraintRuntime, query_saved
from jev_cbf.control.pickup import PickupRuntime, finger_envelope_spheres, target_body
from jev_cbf.evaluation import pickup_collision_diagnostics
from jev_cbf.selection.context import robot_context
from jev_cbf.control.simulation import scene_spec
from jev_cbf.io import save
from jev_cbf.control.model import panda_spec
from jev_cbf.sensing.rgbd import SimulatedRGBD, SceneEstimator

METHODS = {
    'off': 'Scene CBFs off',
    'all_single': 'All clutter / single sphere',
    'all_jev': 'All clutter / Jev geometry',
    'jev_single': 'Jev objects / single sphere',
    'jev_adaptive': 'Jev objects / Jev geometry',
}

class CompletionSchedule:
    """Wall-clock start ceiling, no post-completion delay or queued snapshots."""
    def __init__(self, period_s=.5):
        if not np.isfinite(period_s) or period_s < .5:
            raise ValueError('request starts must be at least 500 ms apart')
        self.period, self.last_start, self.inflight = period_s, None, False

    def ready(self, now):
        return not self.inflight and (self.last_start is None or now >= self.last_start+self.period)

    def start(self, now):
        if not self.ready(now):
            raise ValueError('request would overlap or exceed the 2 Hz start ceiling')
        self.last_start, self.inflight = now, True

    def complete(self):
        self.inflight = False

def observed_request(estimate, settings, robot, template):
    q = np.asarray(robot.default_configuration)
    target = estimate['objects'][0]
    state = dict(description=estimate['provenance'],
                 frame='Robot base/world metres, Z up; idealized simulated camera calibration, explicitly calibrated support Z=0.',
                 task='Pick up the orange object through static clutter, then stop. Only finger-target contact and ordinary support contact are intended. No arm/palm-target or robot/target-clutter contact.',
                 target_id=estimate['target_id'], objects=deepcopy(estimate['objects']),
                 motion=dict(start_m=np.asarray(robot.ee_transform(q))[:3, 3].tolist(),
                             goal_m=[*target['center_m'][:2], estimate['grasp_height_m']]),
                 robot_context=robot_context(robot, {'qpos': q[None], 'qvel': np.zeros((1, 7))}),
                 feature_notes='Candidates enclose every cell of the estimated supported-envelope belief plus padding. Hidden geometry is unknown. Union excess is estimated with common Monte Carlo samples, not overlap-summed sphere volume or calibrated uncertainty.',
                 observation_limits=estimate['limitations'], phase='approach')
    state['robot_context']['limitations'] = 'Seven-DOF FRAX model omits native finger/contact/payload dynamics. No hardware/contact or continuous-time safety certification.'
    state['robot_context']['scene_pairing'] = 'Clutter pairs with 21 original spheres and 8 finger-travel envelope spheres. Mandatory target protection pairs with 16 arm spheres; tool-target overlap is monitored independently and does not authorize palm contact.'
    questions = {}
    for obj in state['objects']:
        if obj['id'] == estimate['target_id']:
            continue
        role = deepcopy(template['role'])
        role.update(type='choice', instructions=role['instructions'].format(object_id=obj['id']))
        questions['role/'+obj['id']] = role
        questions['geometry/'+obj['id']] = dict(type='choice', instructions=template['geometry']['instructions'].format(object_id=obj['id']),
                                               criteria={c['id']: deepcopy(c) for c in obj['candidates']})
    return dict(model='jev-1.13.0', state=state, questions=questions)

class RGBDPickupRuntime(PickupRuntime):
    def __init__(self, physical_scene, settings, request, estimate, method, output, *, initial=None, repeated=False, ask=None):
        if method not in METHODS:
            raise ValueError('unknown five-method condition')
        observed_scene = dict(target_id=estimate['target_id'], objects=deepcopy(estimate['objects']))
        super().__init__(observed_scene, settings, deepcopy(request), None, 'off' if method == 'off' else 'all', output)
        self.physical_scene, self.estimate, self.method = deepcopy(physical_scene), deepcopy(estimate), method
        self.target_id = estimate['target_id']
        self.target = deepcopy(estimate['objects'][0])
        self.initial_target_center = np.array(self.target['center_m'])
        self.target_state = dict(center_m=self.target['center_m'], velocity_m_s=[0., 0., 0.], bottom_m=estimate['support_z_m'], capture_time_s=0., fresh=True)
        self.guard_target_state = deepcopy(self.target_state)
        # Common rotation-invariant guard for the observed target envelope.
        # Geometry ablations concern clutter, never target-contact authority.
        self.target_cover = 'adaptive-1'
        self.policy = {obj['id']: ('adaptive-1' if method != 'off' else None) for obj in self.base['state']['objects']}
        if method != 'off':
            self.policy[self.target_id] = self.target_cover
        if self.capacity*29 > settings['scene_pair_cap']:
            raise ValueError('observed inventory exceeds allocated scene-pair cap; no object is dropped')
        self.initial_report, self.repeated, self.ask = initial, repeated, ask
        self.camera_executor = self.camera_future = self.camera_sensor = None
        self.camera_events, self.sensor_seconds, self.acceptance_seconds = [], [], []
        self.last_camera_submit = -1.
        self.observed_trace, self.width_trace = [], []
        self.loop_seconds = []
        self.schedule = CompletionSchedule()
        self.model_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='rgbd-jev') if repeated else None
        self.model_future = None
        self.request_timings, self.query_index = [], 0

    def robot_spec(self, mj, frax_dir):
        from jev_cbf.upstream import load_upstream
        from jev_cbf.paths import ROOT
        demo, _ = load_upstream(frax_dir, ROOT.parent/'cbfpy')
        self.robot_model = demo.load_panda()
        physical_target = next(o for o in self.physical_scene['objects'] if o['id'] == self.physical_scene['target_id'])
        arm = target_body(mj, panda_spec(mj, frax_dir, profile='frax_gripper', robot=self.robot_model), physical_target)
        arm.body('hand').add_camera(name='wrist_camera', pos=[.055, 0., .03], xyaxes=[1., 0., 0., 0., -1., 0.], fovy=90.)
        return arm

    def scene_spec(self, mj, frax_dir, state, centers, radii):
        physical = dict(state, objects=[o for o in self.physical_scene['objects'] if o['id'] != self.physical_scene['target_id']])
        spec = scene_spec(mj, frax_dir, physical, centers, radii)
        spec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST
        spec.geom('target_geom').rgba = [1., 0., 0., 0.]
        from jev_cbf.sensing.calibration import camera_axes
        for name, eye in [('scene_camera', [.85, -.75, .85]), ('opposite_camera', [.85, .75, .85])]:
            spec.worldbody.add_camera(name=name, pos=eye, xyaxes=camera_axes(eye, [.45, 0., .12]), fovy=58.)
        return spec

    def initialize(self, env):
        # No native free-body state, target geometry, object IDs or contacts are
        # read here. The sensor owns its independent MjData for rendering only.
        self.env = env
        env.data.qpos[7:9], env.data.ctrl[7] = .04, 255.
        import jax
        import jax.numpy as jnp
        self.tool_spheres = finger_envelope_spheres(__import__('mujoco'), env.model, self.robot_model)
        self.robot_sphere_count = 21+len(self.tool_spheres[1])
        robot, points = self.robot_model, jnp.asarray(self.tool_spheres[0])
        self.tool_predict = jax.jit(jax.vmap(lambda q: points@robot.ee_transform(q)[:3, :3].T+robot.ee_transform(q)[:3, 3]))
        self.camera_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='rgbd-render-estimate')
        self.after_step(env)

    def parameters(self, at, policy=None):
        policy = self.policy if policy is None else policy
        centers, radii = np.full((self.capacity, 3), 10.), np.full(self.capacity, .01)
        velocities = np.zeros((self.capacity, 3))
        enabled = np.zeros((29, self.capacity), bool)
        for index, obj in enumerate(self.base['state']['objects']):
            if policy[obj['id']] is None:
                continue
            candidate = next(c for c in obj['candidates'] if c['id'] == policy[obj['id']])
            n = len(candidate['radii_m'])
            if n > self.object_capacity:
                raise ValueError('completed fit exceeds frozen capacity')
            slots = slice(index*self.object_capacity, index*self.object_capacity+n)
            position = self.guard_target_state['center_m'] if obj['id'] == self.target_id else obj['center_m']
            centers[slots] = np.array(candidate['centers_object_m'])+position
            radii[slots] = candidate['radii_m']
            enabled[:16 if obj['id'] == self.target_id else 29, slots] = True
            if obj['id'] == self.target_id:
                velocities[slots] = self.guard_target_state.get('velocity_m_s', [0., 0., 0.])
        return centers, radii, velocities, enabled, float(at)

    def setup(self, demo, robot, guard):
        super().setup(demo, robot, guard)
        # Reuse OSCBF's source controller as an independent numerical parity
        # check; the FRAX demo still supplies the actual nominal torque.
        import importlib.util
        from jev_cbf.paths import ROOT
        spec = importlib.util.spec_from_file_location('read_only_oscbf_controllers', ROOT.parent/'oscbf/oscbf/core/controllers.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        controller = module.PoseTaskTorqueController(7, np.r_[np.full(3, 50.), np.full(3, 20.)],
            np.r_[np.full(3, 20.), np.full(3, 10.)], np.full(7, 10.), np.full(7, 5.), None, None)
        def nominal(z, desired):
            q, v = z[:7], z[7:]
            M, inverse, g, c, J, pose = robot.torque_control_matrices(q, v)
            return controller(q, v, pose[:3, 3], pose[:3, :3], desired[:3], desired[3:12].reshape(3, 3),
                desired[12:15], desired[15:18], demo.jnp.zeros(3), demo.jnp.zeros(3),
                demo.jnp.asarray(robot.default_configuration), demo.jnp.zeros(7), J, M, inverse, g, c)
        self.oscbf_nominal = demo.jax.jit(nominal)
        self.oscbf_nominal(np.r_[robot.default_configuration, np.zeros(7)], self.desired_ee_state(self.env)).block_until_ready()
        self.oscbf_parity_error = None

    def filter(self, z, nominal, at):
        if self.oscbf_parity_error is None:
            expected = np.asarray(self.oscbf_nominal(z, self.desired_ee_state(self.env)))
            self.oscbf_parity_error = float(np.max(abs(expected-np.asarray(nominal))))
            if self.oscbf_parity_error > 1e-7:
                raise ValueError('read-only OSCBF/FRAX nominal interface mismatch')
        return super().filter(z, nominal, at)

    def check_startup(self, q, v, at):
        event = dict(kind='startup', at_s=at, method=self.method)
        try:
            if self.method in ('all_jev', 'jev_adaptive') or getattr(self, 'qualification_mode', False):
                policy = dict.fromkeys(self.policy); policy[self.target_id] = self.target_cover
                trials = []
                for obj in self.base['state']['objects'][1:]:
                    selected = None
                    for candidate in sorted(obj['candidates'], key=lambda c: (c['excess_volume_m3'], len(c['radii_m']))):
                        single = dict.fromkeys(policy); single[self.target_id] = self.target_cover; single[obj['id']] = candidate['id']
                        args, z = self.parameters(at, single), np.r_[q, v, at]
                        h2, h1 = np.asarray(self.h2_fn(z, *args)), np.asarray(self.h1_fn(z, *args))
                        valid = np.isfinite(h2).all() and np.isfinite(h1).all() and min(h2.min(), h1.min()) >= 0
                        trials.append(dict(object_id=obj['id'], candidate_id=candidate['id'], valid=bool(valid), minimum_h2=float(h2.min()), minimum_h1=float(h1.min())))
                        if valid:
                            selected = candidate['id']; break
                    if selected is None:
                        event['candidate_checks'] = trials
                        raise ValueError('no initially admissible completed cover for '+obj['id'])
                    policy[obj['id']] = selected
                self.policy = policy
                event['candidate_checks'] = trials
            event.update(self.admit(self.policy, q, v, at))
            self.startup_checked = True
        except ValueError as error:
            event.update(status='rejected_before_actuation', error=str(error), policy=self.policy.copy())
            save(self.output/'startup.json', event)
            raise
        save(self.output/'startup.json', event)

    def install(self, report, q, v, at, *, source):
        began = time.perf_counter()
        event = dict(kind='selection', source=source, applied_at_s=at,
                     snapshot_at_s=report.get('request', {}).get('state', {}).get('snapshot_time_s', 0.))
        try:
            proposals = validate_selection(report)
            expected = [o for o in self.base['state']['objects'] if o['id'] != self.target_id]
            supplied = report['request']['state']['objects']
            # Features change with measured robot state; geometry must match.
            if len(expected) != len(supplied):
                raise ValueError('response belongs to changed inventory')
            for old, new in zip(expected, supplied, strict=True):
                if old['id'] != new['id'] or [(c['id'], c['centers_object_m'], c['radii_m']) for c in old['candidates']] != [(c['id'], c['centers_object_m'], c['radii_m']) for c in new['candidates']]:
                    raise ValueError('response belongs to changed geometry')
            policy = self.policy.copy()
            for p in proposals:
                if self.method == 'all_jev':
                    policy[p['object_id']] = p['conditional_candidate_id']
                elif p['role'] != 'review':
                    policy[p['object_id']] = None if p['role'] == 'omit' else ('adaptive-1' if self.method == 'jev_single' else p['conditional_candidate_id'])
            if self.method != 'off':
                policy[self.target_id] = self.target_cover
            event.update(self.admit(policy, q, v, at), proposals=proposals)
        except (ValueError, KeyError, TypeError, StopIteration) as error:
            event.update(status='rejected', error=str(error), policy=self.policy.copy())
        self.events.append(event)
        self.acceptance_seconds.append(time.perf_counter()-began)

    def before_control(self, env):
        self.loop_started = time.perf_counter()
        at, q, v = float(env.data.time), env.data.qpos[:7], env.data.qvel[:7]
        if self.method == 'jev_single' and not self.initialized_selection:
            self.install(self.initial_report, q, v, at, source='initial/selection.json')
            self.initialized_selection = True
        if not self.startup_checked:
            self.check_startup(q, v, at)
        if self.method not in ('off', 'all_single') and not self.initialized_selection:
            self.install(self.initial_report, q, v, at, source='initial/selection.json')
            self.initialized_selection = True
        if self.camera_future is not None and self.camera_future.done():
            try:
                observation, elapsed = self.camera_future.result()
                self.sensor_seconds.append(elapsed)
                self.camera_events.append(observation)
                if observation['fresh']:
                    # Completed target pose only; fitting/static geometry never
                    # executes in this hook. Retain prior belief on missing data.
                    self.target_state = observation
                    previous = self.guard_target_state
                    self.guard_target_state = deepcopy(observation)
                    if self.method != 'off':
                        began = time.perf_counter()
                        try:
                            observation['guard_admission'] = self.admit(self.policy, q, v, at)['status']
                        except ValueError:
                            self.guard_target_state = previous
                            observation['guard_admission'] = 'rejected; previous guard retained, not certified for changed target pose'
                        observation['guard_acceptance_host_s'] = time.perf_counter()-began
            except Exception as error:
                self.camera_events.append(dict(status='worker_failure', error_type=type(error).__name__, at_s=at))
            self.camera_future = None
        # Camera preparation and native GL run on one owner worker. A busy worker
        # receives no queue of stale states; the next submission uses latest data.
        if self.camera_future is None and at-self.last_camera_submit >= self.settings['camera_period_s']-1e-9:
            self.last_camera_submit = at
            snapshot = (env.data.qpos.copy(), env.data.qvel.copy(), at)
            supported = self.phase in ('approach', 'pregrasp', 'descend')
            self.camera_future = self.camera_executor.submit(self.observe, snapshot, supported)
        if self.model_future is not None and self.model_future.done():
            report, source, timing = self.model_future.result()
            self.request_timings.append(timing)
            self.model_future = None
            self.schedule.last_start = timing['request_start_monotonic_s']
            self.schedule.complete()
            self.install(report, q, v, at, source=source)
        now = time.monotonic()
        if self.model_executor and self.schedule.ready(now) and not self.task_done:
            self.schedule.start(now)
            # Immutable measured state, prepared into a request on the worker.
            measured = (q.copy(), v.copy(), at, self.phase, deepcopy(self.target_state))
            self.query_index += 1
            index = self.query_index
            def work():
                began = time.monotonic()
                request = self.make_request(*measured[:3], phase=measured[3], target_state=measured[4])
                prepared = time.monotonic()
                report = query_saved(self.output/'queries'/f'{index:03d}', request, ask=self.ask)
                return report, f'queries/{index:03d}/selection.json', dict(start_monotonic_s=began, request_start_monotonic_s=prepared, completion_monotonic_s=time.monotonic(), preparation_s=prepared-began, snapshot_at_s=measured[2])
            self.model_future = self.model_executor.submit(work)

    def observe(self, snapshot, supported):
        import mujoco as mj
        start = time.perf_counter()
        if self.camera_sensor is None:
            self.camera_sensor = SimulatedRGBD(mj, self.env.model)
            self.camera_data = mj.MjData(self.env.model)
            self.estimator = SceneEstimator(support_z=self.estimate['support_z_m'])
            self.estimator.initial = deepcopy(self.estimate)
            self.estimator.last_target = np.array(self.target['center_m'])
            self.estimator.target_top = self.target['center_m'][2]+self.estimate['target_height_m']/2-self.estimate['fit_limits']['padding_m']
        self.camera_data.qpos[:], self.camera_data.qvel[:], self.camera_data.time = snapshot
        mj.mj_forward(self.env.model, self.camera_data)
        records = self.camera_sensor.capture(self.camera_data)
        observation = self.estimator.track_target(records, supported=supported)
        return observation, time.perf_counter()-start

    def waypoint(self):
        goal = np.array([*self.initial_target_center[:2], self.estimate['grasp_height_m']])
        if self.phase == 'approach':
            goal[2] = max(self.settings['approach_height_m'], self.target['center_m'][2]+self.estimate['target_height_m']/2+.10)
        elif self.phase == 'pregrasp':
            goal[2] = self.target['center_m'][2]+self.estimate['target_height_m']/2+self.settings['pregrasp_clearance_m']
        elif self.phase == 'lift':
            goal[2] += self.settings['lift_distance_m']
        return goal

    def after_step(self, env):
        at = float(env.data.time)
        width = float(env.data.qpos[7]+env.data.qpos[8])
        if hasattr(self, 'robot'):
            transform, twist = map(np.asarray, self.ee_kinematics(env.data.qpos[:7], env.data.qvel[:7]))
            ee, goal = transform[:3, 3], self.waypoint()
            distance = np.linalg.norm(ee-goal)
            angle = np.arccos(np.clip((np.trace(env.des_rot.T@transform[:3, :3])-1)/2, -1, 1))
            aligned = distance <= self.settings['alignment_tolerance_m'] and np.linalg.norm(twist[:3]) <= self.settings['alignment_speed_m_s'] and angle <= self.settings['alignment_rotation_tolerance_rad']
            # Encoder aperture is hardware available. This is an acquisition
            # hypothesis, NOT bilateral native target-contact truth.
            acquired = .012 < width < .07 and self.phase in ('close', 'lift')
            next_phase = None
            if self.phase == 'approach' and distance <= self.settings['waypoint_tolerance_m']:
                next_phase = 'pregrasp'
            elif self.phase == 'pregrasp' and aligned:
                next_phase = 'descend'
            elif self.phase == 'descend' and aligned:
                next_phase = 'close'
            elif self.phase == 'descend' and np.linalg.norm(ee[:2]-goal[:2]) > 2*self.settings['alignment_tolerance_m'] and ee[2] < self.target['center_m'][2]+self.estimate['target_height_m']/2+self.settings['pregrasp_clearance_m']-self.settings['alignment_tolerance_m']:
                next_phase = 'pregrasp'
            elif self.phase == 'close' and at-self.phase_began >= self.settings['close_duration_s'] and acquired:
                next_phase = 'lift'
            if next_phase:
                self.phase_events.append(dict(at_s=at, **{'from': self.phase, 'to': next_phase}))
                self.phase, self.phase_began = next_phase, at
            fresh = at-self.target_state['capture_time_s'] < self.settings['max_target_age_s']
            lifted = self.target_state['bottom_m'] >= self.estimate['support_z_m']+self.settings['pickup_bottom_lift_m']
            if self.phase == 'lift' and fresh and lifted and acquired:
                if self.verified_since is None:
                    self.verified_since = at
                self.task_done = at-self.verified_since >= self.settings['pickup_confirmation_s']
            else:
                self.verified_since = None
        self.phase_trace.append(self.phase)
        self.target_trace.append(self.target_state['center_m'])
        self.bottom_trace.append(self.target_state['bottom_m'])
        self.goal_trace.append(self.waypoint())
        self.width_trace.append(width)
        self.observed_trace.append(self.target_state['capture_time_s'])
        if hasattr(self, 'loop_started'):
            self.loop_seconds.append(time.perf_counter()-self.loop_started)

    def make_request(self, q, v, at, *, phase=None, target_state=None):
        request = deepcopy(self.base)
        state = request['state']
        phase = self.phase if phase is None else phase
        state.update(snapshot_time_s=float(at), phase=phase)
        state['estimated_target'] = dict(self.target, current_measurement=deepcopy(self.target_state if target_state is None else target_state),
                                         grasp_height_m=self.estimate['grasp_height_m'])
        state['objects'] = [o for o in state['objects'] if o['id'] != self.target_id]
        current = robot_context(self.robot_model, {'qpos': np.asarray(q)[None], 'qvel': np.asarray(v)[None]})
        current.update({k: self.base['state']['robot_context'][k] for k in ('limitations', 'scene_pairing')})
        state['robot_context'] = current
        references = state['motion'].get('phase_joint_reference_rad', {})
        qgoal = np.array(references.get(phase, q))
        fractions = np.linspace(0., 1., 16)
        qs = np.asarray(q)[None]*(1-fractions[:, None])+qgoal[None]*fractions[:, None]
        path = np.concatenate([np.asarray(self.sphere_predict(qs)), np.asarray(self.tool_predict(qs))], axis=1)
        radii = np.r_[np.asarray(self.robot_model.link_collision_data(q)[1]), self.tool_spheres[1]]
        state['motion'].update(start_m=np.asarray(self.robot_model.ee_transform(q))[:3, 3].tolist(), forecast='16 sampled joint interpolation states toward an observation-derived kinematic reference; not a collision-free route or policy rollout.')
        terminal = {p: np.concatenate([np.asarray(self.sphere_predict(np.array(ref)[None])), np.asarray(self.tool_predict(np.array(ref)[None]))], axis=1)[0] for p, ref in references.items()}
        for obj in state['objects']:
            for candidate in obj['candidates']:
                centers = np.array(candidate['centers_object_m'])+obj['center_m']
                clear = np.linalg.norm(path[:, :, None]-centers[None, None], axis=-1)-radii[None, :, None]-np.array(candidate['radii_m'])[None, None]
                candidate.update(current_robot_sphere_clearance_m=float(clear[0].min()), forecast_robot_sphere_clearance_m=float(clear.min()),
                                 robot_primitive_constraint_pairs=29*len(centers),
                                 terminal_reference_clearance_m={p: float((np.linalg.norm(points[:, None]-centers[None], axis=-1)-radii[:, None]-candidate['radii_m']).min()) for p, points in terminal.items()})
                request['questions']['geometry/'+obj['id']]['criteria'][candidate['id']] = deepcopy(candidate)
        return request

    def collision_diagnostics(self, mj, model, qpos, qvel):
        summary, distances, self.contact_trace = pickup_collision_diagnostics(mj, model, qpos, qvel, self.physical_scene['objects'], self.physical_scene['target_id'])
        return summary, distances

    def diagnostics(self, zs, torques, dt):
        # Bound post-run derivative batches rather than materialize a full
        # trajectory's dense QP matrices at once. Controller equations reused.
        import jax
        times = np.arange(len(torques))*dt
        packed = [np.asarray([p[i] for p in self.params_trace]) for i in range(5)]
        h2_fn, h1_fn = jax.jit(jax.vmap(self.config.h_2)), jax.jit(jax.vmap(self.cbf.h))
        residual = jax.jit(jax.vmap(lambda z, u, *args: self.cbf.G_qp(z, u, *args)@u-self.cbf.h_qp(z, u, *args)))
        h2_pre, h2_post, h1_pre, h1_post, excess = [], [], [], [], []
        for start in range(0, len(times), 256):
            end = min(start+256, len(times))
            args = [p[start:end] for p in packed]
            pre, post = np.c_[zs[start:end], times[start:end]], np.c_[zs[start+1:end+1], times[start:end]+dt]
            h2_pre.append(np.asarray(h2_fn(pre, *args))); h2_post.append(np.asarray(h2_fn(post, *args)))
            h1_pre.append(np.asarray(h1_fn(pre, *args))); h1_post.append(np.asarray(h1_fn(post, *args)))
            excess.append(np.asarray(residual(pre, torques[start:end], *args)))
        first, last = np.concatenate(h2_pre), np.concatenate(h2_post)
        pairs = self.config.scene_pair_count
        self.scene_h2_trace = np.r_[first[:, 15:15+pairs].min(1), last[-1, 15:15+pairs].min()]
        return np.r_[first, last], np.r_[np.concatenate(h1_pre), np.concatenate(h1_post)], np.concatenate(excess)

    def object_history(self, times):
        return np.tile(np.array([o['center_m'] for o in self.base['state']['objects']]), (len(times), 1, 1))

    def finish(self, result, trajectory):
        ConstraintRuntime.finish(self, result, trajectory)
        self.close()
        # Independent evaluator reads hidden target truth AFTER control ends.
        import mujoco as mj
        model, data = self.env.model, mj.MjData(self.env.model)
        target = next(o for o in self.physical_scene['objects'] if o['id'] == self.physical_scene['target_id'])
        geom = model.geom('scene/'+target['id']).id
        bottom, centers = [], []
        for q, v in zip(trajectory['qpos'], trajectory['qvel'], strict=True):
            data.qpos[:], data.qvel[:] = q, v
            mj.mj_forward(model, data)
            rotation, half = data.geom_xmat[geom].reshape(3, 3), np.array(target['size_m'])/2
            support = abs(rotation[2])@half if target['shape'] == 'box' else half[0]*np.linalg.norm(rotation[2, :2])+half[2]*abs(rotation[2, 2])
            centers.append(data.geom_xpos[geom].copy()); bottom.append(data.geom_xpos[geom][2]-support)
        bottom, centers = np.array(bottom), np.array(centers)
        valid = (bottom >= self.settings['pickup_bottom_lift_m']) & self.contact_trace['bilateral_finger_contact']
        window = round(self.settings['pickup_confirmation_s']/self.settings['control_dt_s'])
        verified = bool(len(valid) >= window and np.convolve(valid.astype(int), np.ones(window, int), 'valid').max() == window)
        unwanted = result['collision']['any_sampled_scene_violation']
        active = np.array([np.any(p[3], axis=0).reshape(len(self.base['state']['objects']), -1).sum(1) for p in self.params_trace])
        clutter_active = active[:, 1:]
        warm = np.array(self.loop_seconds[1:])
        result.update(method=self.method, moving_objects=False, task='rgbd_static_clutter_pickup', phase_events=self.phase_events,
                      scene_sphere_count=int(active[-1].sum()), observed_inventory=[o['id'] for o in self.base['state']['objects']],
                      native_evaluation_object_ids=[o['id'] for o in self.physical_scene['objects']],
                      model_selection_accepted=any(e.get('kind')=='selection' and e.get('status')=='accepted' for e in self.events),
                      grasp_verified=verified, pickup_success=verified, observer_task_done=bool(self.task_done), unwanted_collision=unwanted,
                      safe_pickup=bool(verified and not unwanted), task_success=bool(verified and not unwanted and result['complete'] and not result['any_sampled_negative_h2']),
                      final_target_bottom_m=float(bottom[-1]), maximum_target_center_lift_m=float((centers[:, 2]-centers[0, 2]).max()),
                      cbf_counts=dict(clutter_objects_mean=float((clutter_active > 0).sum(1).mean()), clutter_objects_peak=int((clutter_active > 0).sum(1).max()),
                                      clutter_spheres_mean=float(clutter_active.sum(1).mean()), clutter_spheres_peak=int(clutter_active.sum(1).max()),
                                      clutter_pairs_mean=float(clutter_active.sum(1).mean()*29), clutter_pairs_peak=int(clutter_active.sum(1).max()*29),
                                      mandatory_target_pairs=int(active[:, 0].max()*16), intrinsic_barriers=int(result['total_barrier_count']-self.config.scene_pair_count),
                                      allocated_scene_pairs=self.config.scene_pair_count, allocated_total_barriers=result['total_barrier_count']),
                      sensor_processing_s=dict(count=len(self.sensor_seconds), mean=float(np.mean(self.sensor_seconds)) if self.sensor_seconds else None,
                                               maximum=max(self.sensor_seconds, default=None)),
                      acceptance_host_s=self.acceptance_seconds, model_request_timings=self.request_timings,
                      oscbf_nominal_first_step_max_difference_Nm=self.oscbf_parity_error,
                      warm_loop=dict(mean_ms=float(warm.mean()*1000), p95_ms=float(np.percentile(warm,95)*1000),
                                     maximum_ms=float(warm.max()*1000), achieved_hz=float(1/warm.mean()),
                                     fraction_above_1ms=float(np.mean(warm>.001)),
                                     scope='Before-control hook through original nominal/filter, physics and task after-step; first step excluded. Host scheduling/record-copy overhead can add time. Empirical wall timing, not hard real time.'),
                      maximum_target_measurement_age_sim_s=float(np.max(trajectory['time_s']-np.array(self.observed_trace))),
                      control_target_hz=1000, scope='RGBD color components + supported envelope + upright target estimate; idealized calibration and measurable joint/aperture inputs. Physical truth only in plant/rendering and independent post-run scoring. Read-only FRAX/CBFpy; seven-DOF dynamics omit finger/contact/payload effects. No hardware or real-time safety certification.')
        trajectory.update(target_center_m=centers, target_bottom_m=bottom, estimated_target_center_m=np.array(self.target_trace),
                          observed_object_ids=np.array([o['id'] for o in self.base['state']['objects']]),
                          estimated_target_bottom_m=np.array(self.bottom_trace), target_capture_time_s=np.array(self.observed_trace),
                          gripper_aperture_m=np.array(self.width_trace), phase=np.array(self.phase_trace), ee_goal_m=np.array(self.goal_trace),
                          tool_centers_ee_m=self.tool_spheres[0], tool_radii_m=self.tool_spheres[1], **self.contact_trace)
        save(self.output/'camera_events.json', self.camera_events)
        save(self.output/'task_events.json', self.phase_events)

    def close(self):
        if self.camera_executor is not None:
            def close_sensor():
                if self.camera_sensor is not None:
                    self.camera_sensor.close()
            self.camera_executor.submit(close_sensor).result()
            self.camera_executor.shutdown(wait=True)
            self.camera_executor = None
        if self.model_executor is not None:
            if self.model_future is not None:
                report, source, timing = self.model_future.result()
                self.request_timings.append(timing)
                self.events.append(dict(kind='late_response', source=source, status='not applied after rollout'))
            self.model_executor.shutdown(wait=True)
            self.model_executor = None
            save(self.output/'events.json', self.events)
