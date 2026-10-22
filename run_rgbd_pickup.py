"""Revision workflow: RGBD preparation, qualifications, five methods, gallery.

All output stays ignored inside this repository. prepare/qualify are offline;
run --live uses the existing cumulative Jev ledger. No completed run is replaced.
"""

import argparse
from copy import deepcopy
import re
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np

from adaptive_geometry import FitLimits
from api_budget import JevBudget
from inspect_selection import ROOT
from online import query_saved
from pickup import DEFAULTS, finger_envelope_spheres, target_body
from rgbd_pickup import METHODS, RGBDPickupRuntime, observed_request
from rgbd_scene import SceneEstimator, SimulatedRGBD, add_rgbd_cameras
from run_pickup import BUDGET_PATH, runtime_versions
from run_static import load_upstream, run_case, scene_spec
from run_suite import read, save, digest
from sim_model import panda_spec


SOURCES = ('adaptive_geometry.py', 'rgbd_scene.py', 'rgbd_pickup.py', 'run_rgbd_pickup.py', 'rgbd_gallery.py',
           'pickup.py', 'online.py', 'run_static.py', 'runtime_cbf.py', 'sim_model.py', 'camera_io.py',
           'geometry.py', 'perception.py', 'api_budget.py', 'inspect_selection.py', 'robot_selection.py')
SETTINGS = dict(DEFAULTS, control_dt_s=.001, camera_period_s=.1, max_target_age_s=.5, scene_pair_cap=1160)


def plant_model(scene, robot, mj):
    """Truth used for plant/rendering only; no request/candidate constructor."""
    target = next(o for o in scene['objects'] if o['id'] == scene['target_id'])
    spec = scene_spec(mj, ROOT.parent/'frax', dict(objects=[o for o in scene['objects'] if o != target]), [], [])
    spec.option.integrator, spec.option.timestep = mj.mjtIntegrator.mjINT_IMPLICITFAST, .001
    spec.geom('target_geom').rgba = [1., 0., 0., 0.]
    arm = target_body(mj, panda_spec(mj, ROOT.parent/'frax', profile='frax_gripper', robot=robot), target)
    add_rgbd_cameras(spec, arm)
    spec.attach(arm, prefix='', frame=spec.worldbody.add_frame())
    return spec.compile()


def kinematic_references(robot, estimate, settings, demo):
    """Damped kinematic interpolation targets, not a controller or route plan."""
    transform = demo.jax.jit(robot.ee_transform)
    jacobian = demo.jax.jit(robot.ee_jacobian)
    target = estimate['objects'][0]
    grasp = estimate['grasp_height_m']
    heights = dict(approach=max(settings['approach_height_m'], estimate['support_z_m']+estimate['target_height_m']+.10),
                   pregrasp=estimate['support_z_m']+estimate['target_height_m']+settings['pregrasp_clearance_m'],
                   descend=grasp, close=grasp, lift=grasp+settings['lift_distance_m'])
    q = np.array(robot.default_configuration)
    desired = np.diag([1., -1., -1.])
    references = {}
    for phase, height in heights.items():
        goal = np.array([*target['center_m'][:2], height])
        for _ in range(120):
            pose, J = np.array(transform(q)), np.array(jacobian(q))
            rotation_error = .5*sum((np.cross(pose[:3, i], desired[:, i]) for i in range(3)), start=np.zeros(3))
            error = np.r_[goal-pose[:3, 3], rotation_error]
            if np.linalg.norm(error) < .0001:
                break
            delta = J.T@np.linalg.solve(J@J.T+.002*np.eye(6), error)
            q += np.clip(delta, -.08, .08)
        references[phase] = q.tolist()
    return references


def prepare(folder, config_path):
    if folder.exists() and any(folder.iterdir()):
        raise ValueError('prepare requires a fresh directory')
    config = read(config_path)
    ids = [scene['id'] for scene in config['scenes']]
    if not ids or len(set(ids))!=len(ids) or any(not re.fullmatch(r'[a-z][a-z0-9_]*', name) for name in ids):
        raise ValueError('scene IDs must be unique lowercase names, not paths')
    source_hashes = {name: digest(ROOT/name) for name in SOURCES}
    settings = dict(SETTINGS, **config.get('settings', {}))
    if set(settings) != set(SETTINGS) or any(not np.isfinite(v) or v <= 0 for v in settings.values()):
        raise ValueError('settings require known names and positive finite values')
    if settings['control_dt_s'] != .001:
        raise ValueError('this revision targets a 1 ms simulation step')
    save(folder/'scenes.json', config); save(folder/'settings.json', settings)
    template = read(ROOT/'prompts/rgbd_pickup_selection.json'); save(folder/'questions.json', template)
    demo, utils = load_upstream(ROOT.parent/'frax', ROOT.parent/'cbfpy')
    robot, mj = demo.load_panda(), utils.mujoco
    limits = FitLimits(**config.get('fit_limits', {}))
    for scene in config['scenes']:
        case = folder/scene['id']
        model = plant_model(scene, robot, mj)
        data = mj.MjData(model)
        data.qpos[:7], data.qpos[7:9] = robot.default_configuration, .04
        mj.mj_forward(model, data)
        sensor = SimulatedRGBD(mj, model)
        try:
            records = sensor.capture(data)
        finally:
            sensor.close()
        estimate = SceneEstimator(support_z=0., limits=limits).initialize(records, case/'observation')
        save(case/'estimate.json', estimate); save(case/'scene.json', scene)
        request = observed_request(estimate, settings, robot, template)
        request['state']['motion']['phase_joint_reference_rad'] = kinematic_references(robot, estimate, settings, demo)
        request['state']['motion']['reference_source'] = 'Damped IK using known robot kinematics and RGBD-estimated target/grasp height, no native scene pose or trajectory.'
        runtime = RGBDPickupRuntime(scene, settings, request, estimate, 'off', case/'unused')
        runtime.robot_model = robot
        runtime.tool_spheres = finger_envelope_spheres(mj, model, robot)
        runtime.sphere_predict = demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
        points = demo.jnp.asarray(runtime.tool_spheres[0])
        runtime.tool_predict = demo.jax.jit(demo.jax.vmap(lambda q: points@robot.ee_transform(q)[:3, :3].T+robot.ee_transform(q)[:3, 3]))
        model_request = runtime.make_request(np.array(robot.default_configuration), np.zeros(7), 0.)
        save(case/'request.json', request); save(case/'initial/request.json', model_request)
        print(scene['id'], 'observed', [(o['id'], o['size_m'], o['fitting']['completed_counts'], o['fitting']['elapsed_s']) for o in estimate['objects']], flush=True)
    frozen = ['scenes.json', 'settings.json', 'questions.json']+[f"{s['id']}/{name}" for s in config['scenes'] for name in ('request.json', 'initial/request.json', 'estimate.json', 'scene.json')]
    snapshot = folder/'source_snapshot'; snapshot.mkdir()
    for name in SOURCES:
        if (ROOT/name).exists():
            shutil.copyfile(ROOT/name, snapshot/name)
    if any(digest(ROOT/name) != expected for name, expected in source_hashes.items()):
        raise ValueError('source changed during preparation; retain this draft and prepare a new run')
    save(folder/'manifest.json', dict(scene_ids=[s['id'] for s in config['scenes']], methods=METHODS,
        schedule='Once at startup; one actual response shared by the three Jev conditions. Repeated scheduler is a separate diagnostic, not a crossed comparison matrix.',
        frozen_files={name: digest(folder/name) for name in frozen},
        source_sha256=source_hashes,
        upstream_revisions={name: subprocess.check_output(['git', '-C', str(ROOT.parent/name), 'rev-parse', 'HEAD'], text=True).strip() for name in ('frax', 'oscbf', 'cbfpy')},
        versions=runtime_versions(), budget_ledger=str(BUDGET_PATH), budget_cap_usd='7.50',
        qualification='Actual off unwanted contact + valid nonpenetrating initial physical scene + offline all-adaptive pickup witness. Freeze before inspecting comparative Jev answers.',
        scope='Designed RGBD demo, no learned/generalization or hardware claim. Estimated whole-cell enclosure is separate from observed fit and true-object coverage.'))


def verify_inputs(folder, *, source=True):
    manifest = read(folder/'manifest.json')
    for name, expected in manifest['frozen_files'].items():
        if digest(folder/name) != expected:
            raise ValueError('frozen input changed: '+name)
    if source:
        for name, expected in manifest['source_sha256'].items():
            if digest(ROOT/name) != expected:
                raise ValueError('source changed; use a fresh run: '+name)
    return manifest


def execute(case, method, *, output=None, initial=None, qualification=False, repeated=False, ask=None):
    folder = case.parent
    output = case/method if output is None else output
    if (output/'result.json').exists():
        return read(output/'result.json')
    if (output/'attempt.json').exists():
        raise ValueError('interrupted attempt retained; choose a fresh directory')
    verify_inputs(folder)
    scene, request, estimate, settings = read(case/'scene.json'), read(case/'request.json'), read(case/'estimate.json'), read(folder/'settings.json')
    demo, utils = load_upstream(ROOT.parent/'frax', ROOT.parent/'cbfpy')
    runtime = RGBDPickupRuntime(scene, settings, request, estimate, method, output, initial=initial, repeated=repeated, ask=ask)
    # Qualification is an explicitly non-model feasibility witness: all observed
    # clutter retained, tightest completed fit. No fabricated Jev response.
    if qualification:
        runtime.qualification_mode = True
        runtime.method = 'all_single'
        runtime.policy = {o['id']: o['candidates'][-1]['id'] for o in request['state']['objects']}
        runtime.policy[runtime.target_id] = runtime.target_cover
    save(output/'attempt.json', dict(status='started', method=method, qualification=qualification))
    start = time.perf_counter()
    try:
        result, trace = run_case(demo, utils, {'request': request}, 'off', frax_dir=ROOT.parent/'frax',
                                 steps=round(settings['duration_s']/settings['control_dt_s']), model_profile='frax_gripper',
                                 scene_guard=settings['scene_guard_m_s2'], control_dt=settings['control_dt_s'], runtime=runtime)
        result['wall_seconds_including_evaluation'] = time.perf_counter()-start
        np.savez_compressed(output/'trajectory.npz', **trace)
        result['trajectory_sha256'] = digest(output/'trajectory.npz')
        result['qualification_only'] = qualification
        save(output/'result.json', result)
        save(output/'attempt.json', dict(status='finished', method=method))
        print(scene['id'], method, 'collision', result['unwanted_collision'], 'pickup', result['pickup_success'], 'observer_done', result['observer_task_done'], 'bottom', result['final_target_bottom_m'], 'time', result['simulation_seconds'], flush=True)
        return result
    except Exception as error:
        runtime.close()
        save(output/'attempt.json', dict(status='failed', error_type=type(error).__name__, error=str(error), qualification=qualification))
        startup = read(output/'startup.json') if (output/'startup.json').exists() else {}
        if startup.get('status') == 'rejected_before_actuation':
            attempted = runtime.parameters(0.)[3]
            result = dict(method=method, startup_refusal=True, steps=0, simulation_seconds=0., complete=False,
                          pickup_success=False, safe_pickup=False, task_success=False, unwanted_collision=None,
                          requested_scene_pairs=int(attempted.sum()), requested_scene_spheres=int(np.any(attempted,axis=0).sum()),
                          allocated_scene_pairs=runtime.config.scene_pair_count,
                          startup_wall_seconds=time.perf_counter()-start,
                          qualification_only=qualification, startup=startup,
                          note='No actuation/trajectory. Collision outcome is unmeasured, not a collision-free success; this condition failed to initialize its representation.')
            save(output/'result.json', result)
            print(scene['id'], method, 'startup refusal retained', flush=True)
            return result
        print(scene['id'], method, 'FAILED', type(error).__name__, str(error), flush=True)
        raise


def qualify(folder):
    verify_inputs(folder)
    for name in read(folder/'manifest.json')['scene_ids']:
        case = folder/name
        off = execute(case, 'off')
        witness = execute(case, 'all_single', output=case/'qualification/adaptive', qualification=True)
        scene = read(case/'scene.json')
        demo, utils = load_upstream(ROOT.parent/'frax', ROOT.parent/'cbfpy')
        mj = utils.mujoco
        model = plant_model(scene, demo.load_panda(), mj); data = mj.MjData(model)
        data.qpos[:7] = demo.load_panda().default_configuration; data.qpos[7:9] = .04
        mj.mj_forward(model, data)
        geoms = [model.geom('scene/'+o['id']).id for o in scene['objects']]
        object_pairs = [float(mj.mj_geomDistance(model,data,a,b,1000.,None)) for i,a in enumerate(geoms) for b in geoms[i+1:]]
        objects_clear = all(d>0 for d in object_pairs)
        with np.load(case/'off/trajectory.npz') as trace:
            initial_clear = bool(np.all(trace['object_distance_m'][0] > 0))
        qualified = bool(initial_clear and objects_clear and off['unwanted_collision'] and witness['safe_pickup'] and not witness.get('any_sampled_negative_h2', True))
        save(case/'qualification.json', dict(qualified=qualified, initial_robot_scene_clear=initial_clear,
             initial_object_pairs_clear=objects_clear, minimum_initial_object_pair_distance_m=min(object_pairs),
             off_unwanted_collision=off['unwanted_collision'], adaptive_feasibility_witness=witness['safe_pickup'],
             witness_minimum_h2=witness.get('minimum_selected_scene_sphere_barrier_m'),
             note='Designed scenes fixed without model answers. Witness is a deterministic all-adaptive feasibility check, not a sixth gallery comparison or evidence of Jev performance.'))
        print(name, 'qualified', qualified, flush=True)


def run(folder, *, live=False):
    manifest = verify_inputs(folder, source=False)
    for name in manifest['scene_ids']:
        case = folder/name
        if all((case/m/'result.json').exists() for m in METHODS):
            continue
        verify_inputs(folder)
        if not read(case/'qualification.json')['qualified']:
            raise ValueError('scene not qualified; no provider call: '+name)
        execute(case, 'all_single')
        if not live:
            raise ValueError('model conditions require --live; offline results remain retained')
        if not BUDGET_PATH.is_file():
            raise ValueError('existing cumulative usage ledger is missing; do not reset it')
        initial = query_saved(case/'initial', read(case/'initial/request.json'), ask=JevBudget(BUDGET_PATH))
        if 'error' in initial:
            raise ValueError('recorded model failure; no fabricated response or re-query')
        for method in ('all_jev', 'jev_single', 'jev_adaptive'):
            execute(case, method, initial=initial)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=('prepare', 'qualify', 'run', 'gallery', 'status', 'repeat'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--scenes', type=Path, default=ROOT/'examples/rgbd_pickup_scenes.json')
    p.add_argument('--live', action='store_true')
    p.add_argument('--case', default='easy')
    a = p.parse_args()
    folder = a.output.resolve()
    if not folder.is_relative_to(ROOT):
        p.error('all outputs must stay inside this repository')
    if a.command == 'prepare':
        prepare(folder, a.scenes)
    elif a.command == 'qualify':
        qualify(folder)
    elif a.command == 'run':
        run(folder, live=a.live)
    elif a.command == 'repeat':
        if not a.live:
            p.error('repeat requires --live and uses the cumulative ledger')
        if a.case not in verify_inputs(folder,source=False)['scene_ids']:
            p.error('repeat case must belong to the frozen scene list')
        case = folder/a.case
        if not (case/'repeated_diagnostic/result.json').exists() and not BUDGET_PATH.is_file():
            raise ValueError('existing cumulative usage ledger is missing; do not reset it')
        initial = read(case/'initial/selection.json')
        execute(case, 'jev_adaptive', output=case/'repeated_diagnostic', initial=initial, repeated=True, ask=JevBudget(BUDGET_PATH))
    elif a.command == 'gallery':
        from rgbd_gallery import gallery
        gallery(folder)
    else:
        for name in verify_inputs(folder, source=False)['scene_ids']:
            print(name, {m: read(folder/name/m/'result.json')['safe_pickup'] if (folder/name/m/'result.json').exists() else 'pending' for m in METHODS})


if __name__ == '__main__':
    main()
