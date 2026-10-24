"""Evidence-driven pickup studies using the existing runner, not a new controller.

Development is a frozen 2x2: candidate counts versus terminal-clearance features.
Only --live makes new Jev requests; diagnosis, preparation and reporting do not.
"""

import argparse
from copy import deepcopy
from pathlib import Path
import numpy as np
from jev_cbf.legacy.analyze_pickup import report
from jev_cbf.geometry.covers import make_shape_covers, evaluate_covers
from jev_cbf.paths import ROOT
from jev_cbf.legacy.pickup import prepare, run
from jev_cbf.paths import BUDGET_PATH
from jev_cbf.selection.budget import JevBudget
from jev_cbf.selection.runtime import query_saved
from jev_cbf.upstream import load_upstream
from jev_cbf.io import read, save, digest

RICH_COUNTS=(1,2,3,4,5,6,8)

def diagnose(source,output):
    """Compare saved one-shot covers against required reference postures."""
    demo,_=load_upstream(ROOT.parent/"frax",ROOT.parent/"cbfpy")
    robot=demo.load_panda()
    predict=demo.jax.jit(demo.jax.vmap(robot.link_collision_positions))
    transforms=demo.jax.jit(demo.jax.vmap(robot.ee_transform))
    rows=[]
    for case in read(source/"manifest.json")["scene_ids"]:
        folder=source/case
        request=read(folder/"initial/request.json")
        result=read(folder/"once/result.json")
        with np.load(folder/"nominal_reference/off/trajectory.npz") as trace:
            phases=("pregrasp","descend","close","lift")
            indices=[np.flatnonzero(trace["phase"]==p)[-1] for p in phases]
            q=trace["qpos"][indices,:7];tf=np.asarray(transforms(q))
            fingers=np.einsum('tij,kj->tki',tf[:,:3,:3],trace['tool_centers_ee_m'])+tf[:,None,:3,3]
            points=np.concatenate([np.asarray(predict(q)),fingers],axis=1)
            robot_radii=np.r_[np.asarray(robot.link_collision_data(q[0])[1]),trace['tool_radii_m']]
        selected={o['object_id']:o['candidate_id'] for o in result['included_objects']}
        for obj in request['state']['objects']:
            if obj['id']==request['state']['target_id']:continue
            covers=make_shape_covers(np.array(obj['size_m'])/2,shape=obj['shape'],counts=RICH_COUNTS)
            volume=evaluate_covers(np.array(obj['size_m'])/2,covers,shape=obj['shape'],samples=100_000,seed=72)
            options=[]
            for cover,measurement in zip(covers,volume['candidates'],strict=True):
                centers=np.array(cover.centers_m)+obj['center_m']
                clear=(np.linalg.norm(points[:,:,None]-centers[None,None],axis=-1)
                       -robot_radii[None,:,None]-np.array(cover.radii_m)[None,None])
                options.append({'id':cover.candidate_id,'count':len(cover.radii_m),
                                'terminal_clearance_m':dict(zip(phases,clear.min(axis=(1,2)).tolist(),strict=True)),
                                'minimum_terminal_m':float(clear.min()),'excess_m3':measurement['excess_volume_m3']})
            chosen=next((c for c in options if c['id']==selected.get(obj['id'])),None)
            rows.append({'case':case,'object_id':obj['id'],'once_success':result['task_success'],
                         'chosen':chosen,'candidates':options})
    save(output,{'rows':rows,'scope':'Offline cover obstruction diagnosis at saved unobstructed reference postures. Not a new Jev run, a physical-collision assertion or proof that another route exists. All richer candidates cover the known shape.'})
    print('Saved diagnosis:',output,flush=True)
    for row in rows:
        if not row['once_success'] and row['chosen']:
            chosen=row['chosen']
            print(row['case'],row['object_id'],chosen['id'],round(chosen['minimum_terminal_m']*1000,2),'mm; best',
                  max(row['candidates'],key=lambda c:c['minimum_terminal_m'])['id'],flush=True)

def development(folder,*,live=False):
    source=ROOT/'artifacts/pickup/corrected_replication_v2'
    ids=('layout_00','layout_02','layout_03','layout_11')
    if not (folder/'protocol.json').exists():
        if folder.exists() and any(folder.iterdir()):raise ValueError('new study needs an empty directory')
        config=read(source/'scenes.json')
        scenes=[s for s in config['scenes'] if s['id'] in ids]
        save(folder/'protocol.json',{'kind':'development_2x2','seen_cases':list(ids),'source':str(source),
             'factors':{'counts':[[1,2,4],list(RICH_COUNTS)],'terminal_features':[False,True]},
             'same_prompt':'configs/legacy/prompts/pickup_terminal.json','template_sha256':digest(ROOT/'configs/legacy/prompts/pickup_terminal.json'),
             'note':'Cases selected from already observed successes/stalls, not unseen evidence. Freeze all four conditions/baselines before new answers; no retries for preferred outputs.'})
        for rich in (False,True):
            for terminal in (False,True):
                name=f"{'rich' if rich else 'small'}_{'terminal' if terminal else 'phase'}"
                config={'scenes':deepcopy(scenes),'settings':{},'shape_aware_geometry':True,
                        'geometry_counts':list(RICH_COUNTS) if rich else [1,2,4],
                        'terminal_clearance_features':terminal}
                save(folder/'inputs'/f'{name}.json',config)
    names=('small_phase','small_terminal','rich_phase','rich_terminal')
    # Preparation across every condition precedes every model call.
    for name in names:
        if not (folder/name/'manifest.json').exists():
            prepare(folder/name,folder/'inputs'/f'{name}.json',ROOT/'configs/legacy/prompts/pickup_terminal.json')
    for name in names:run(folder/name,baselines=True)
    if live:
        for name in names:
            run(folder/name,live=True)
            report(folder/name)
    else:
        print('All conditions frozen and baselines complete; --live explicitly enables model comparisons.',flush=True)

def run_group(folder,names,*,live=False,classification_repetitions=1):
    for name in names:
        if not (folder/name/'manifest.json').exists():
            prepare(folder/name,folder/'inputs'/f'{name}.json',ROOT/'configs/legacy/prompts/pickup_terminal.json')
    for name in names:run(folder/name,baselines=True)
    if not live:
        print('Frozen inputs and offline baselines ready; no new model requests.',flush=True);return
    for name in names:
        run(folder/name,live=True);report(folder/name)
    # These repeats are classification-only, never retried to replace the
    # original decision that actually drove a rollout. All were predeclared.
    budget=JevBudget(BUDGET_PATH)
    for name in names:
        for case in read(folder/name/'manifest.json')['scene_ids']:
            request=read(folder/name/case/'initial/request.json')
            for repeat in range(1,classification_repetitions):
                query_saved(folder/name/case/'classification_repeats'/f'{repeat:02d}',request,ask=budget)

def unseen(folder,scenes_path,*,live=False):
    names=('small_terminal','rich_terminal')
    if not (folder/'protocol.json').exists():
        if folder.exists() and any(folder.iterdir()):raise ValueError('new unseen study needs an empty directory')
        config=read(scenes_path)
        save(folder/'protocol.json',{'kind':'new_unseen_designed_layouts','source_geometry_config':str(scenes_path),
            'source_sha256':digest(scenes_path),'conditions':list(names),'same_prompt_sha256':digest(ROOT/'configs/legacy/prompts/pickup_terminal.json'),
            'classification_repetitions':3,'note':'All layouts/conditions frozen before new answers. Additional initial repeats measure classification consistency only; they never replace the original control decision. Both counts get the same terminal context and strong least-excess all-on/rule baselines. No population/general-safety claim.'})
        for name in names:
            condition=deepcopy(config);condition['geometry_counts']=list(RICH_COUNTS) if name.startswith('rich') else [1,2,4]
            condition['terminal_clearance_features']=True
            condition['baseline_modes']=['off','all','all_fine','rule']
            save(folder/'inputs'/f'{name}.json',condition)
    run_group(folder,names,live=live,classification_repetitions=3)

def camera_study(folder,*,live=False):
    conditions={'oracle':{},'two_views':{},'scene_only':{'cameras':['scene_camera']},
        'sensor_noise':{'depth_noise_m':.003,'pixel_dropout':.5},
        'calibration_error':{'calibration_bias_m':[.012,-.008,0.]},
        'large_calibration_error':{'calibration_bias_m':[.025,-.015,0.]},
        'stale_observations':{'delay_s':3.,'missing_frames':[1,2,3]},
        'model_unavailable':{},'model_delay':{}}
    names=tuple(conditions)
    if not (folder/'protocol.json').exists():
        if folder.exists() and any(folder.iterdir()):raise ValueError('new camera study needs an empty directory')
        source=ROOT/'artifacts/pickup/corrected_replication_v2/scenes.json'
        scenes=[s for s in read(source)['scenes'] if s['id'] in ('layout_02','layout_03')]
        save(folder/'protocol.json',{'kind':'camera_geometry_and_controlled_fault_development','cases':['layout_02','layout_03'],
            'conditions':conditions,'source_sha256':digest(source),'template_sha256':digest(ROOT/'configs/legacy/prompts/pickup_terminal.json'),
            'note':'Seen diagnostic layouts; not unseen model-performance evidence. Simulator segmentation identities and target geometry/pose remain known. Only clutter dimensions/poses come from depth. Fault settings stay in scene/runtime configuration, not model-visible state. No oracle fill-in of missing clutter. Both views use known calibration; assumed 5 mm padding is not calibrated uncertainty.'})
        for name,observation in conditions.items():
            config={'scenes':deepcopy(scenes),'settings':{},'shape_aware_geometry':True,
                'geometry_counts':list(RICH_COUNTS),'terminal_clearance_features':True,
                'baseline_modes':['off','all_fine','rule'],'model_modes':['once','repeated']}
            if name!='oracle':config['observation']=dict(margin_m=.005,period_s=1.,seed=91,**observation)
            if name=='model_unavailable':config['model_faults']={'fail_calls':[2,3]}
            if name=='model_delay':config['model_faults']={'delay_calls':[2],'delay_s':3.5}
            save(folder/'inputs'/f'{name}.json',config)
    run_group(folder,names,live=live)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('diagnose','development','unseen','camera'))
    p.add_argument('--output',type=Path,default=ROOT/'artifacts/pickup/selection_study_v1')
    p.add_argument('--source',type=Path,default=ROOT/'artifacts/pickup/corrected_replication_v2')
    p.add_argument('--live',action='store_true')
    p.add_argument('--scenes',type=Path,help='new geometry-qualified layouts for unseen evaluation')
    args=p.parse_args(argv)
    if args.action=='diagnose':diagnose(args.source,args.output/'terminal_diagnosis.json')
    elif args.action=='development':development(args.output,live=args.live)
    elif args.action=='camera':camera_study(args.output,live=args.live)
    else:
        if args.scenes is None:p.error('unseen needs --scenes')
        unseen(args.output,args.scenes,live=args.live)
    return 0

if __name__=='__main__':raise SystemExit(main())
