"""Generate new static pickup layouts before seeing any model answers.

Designed strata distinguish native path hazards, representation obstruction and
near misses. Eligibility is checked in native geometry; search labels are never
sent to the selector. This is not a random-population benchmark or path planner.
"""

import argparse
from pathlib import Path
import shutil
import numpy as np
from jev_cbf.paths import ROOT
from jev_cbf.control.pickup import DEFAULTS, PickupRuntime
from jev_cbf.legacy.requests import pickup_request
from jev_cbf.legacy.pickup import execute
from jev_cbf.upstream import load_upstream
from jev_cbf.io import read, save, digest

def generate(folder,*,count=12,seed=731,proposals=5000):
    if folder.exists() and any(folder.iterdir()):raise ValueError('new layout generation directory required')
    demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');mj=utils.mujoco;robot=demo.load_panda()
    rng=np.random.default_rng(seed);template=read(ROOT/'configs/legacy/prompts/pickup_terminal.json')
    folder.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(ROOT/'src/jev_cbf/legacy/pickup_layouts.py',folder/'generator_source.py')
    predict=demo.jax.jit(demo.jax.vmap(robot.link_collision_positions));transforms=demo.jax.jit(demo.jax.vmap(robot.ee_transform))
    scenes=[];records=[]
    sizes=[(r,h) for r in (.015,.022,.03,.04) for h in (.10,.14,.18,.22,.26,.32,.40)]
    save(folder/'protocol.json',{'seed':seed,'count':count,'proposals_per_layout':proposals,'source_sha256':digest(ROOT/'src/jev_cbf/legacy/pickup_layouts.py'),
          'strata':['cover_tradeoff','path_hazard','near_miss'],'before_model_answers':True,
          'note':'Designed geometry-qualified fixtures. Required/achieved strata and search failures retained; not a population sample. No model is consulted.'})
    for number in range(count):
        case=f'challenge_{number:02d}';kind=('cover_tradeoff','path_hazard','near_miss')[number%3]
        xy=rng.uniform([.46,-.14],[.57,.14])
        target={'id':'target_object','shape':'cylinder','size_m':[.044,.044,.12],'center_m':[*map(float,xy),.06],'mass_kg':.05}
        single={'id':case,'target_id':'target_object','objects':[target]}
        request=pickup_request(single,DEFAULTS,robot,template)
        execute(folder/case,single,DEFAULTS,request,'off',demo,utils)
        result=read(folder/case/'off/result.json')
        if not result['grasp_verified']:raise ValueError(f'{case}: unobstructed pickup failed; retained before any queries')
        with np.load(folder/case/'off/trajectory.npz') as trace:
            phase_indices=[int(np.flatnonzero(trace['phase']==p)[-1]) for p in ('pregrasp','descend','close','lift')]
            indices=np.unique(np.r_[np.linspace(0,len(trace['qpos'])-1,48,dtype=int),phase_indices])
            q=trace['qpos'][indices,:7];tf=np.asarray(transforms(q))
            points=np.concatenate([np.asarray(predict(q)),np.einsum('tij,kj->tki',tf[:,:3,:3],trace['tool_centers_ee_m'])+tf[:,None,:3,3]],axis=1)
            rr=np.r_[np.asarray(robot.link_collision_data(q[0])[1]),trace['tool_radii_m']]
            late=np.isin(indices,phase_indices[1:])
            centers=rng.uniform([.22,-.31],[.68,.31],(proposals,2))
            which=rng.integers(0,len(sizes),proposals);radii=np.array([sizes[i][0] for i in which]);heights=np.array([sizes[i][1] for i in which])
            scored=[]
            for start in range(0,proposals,64):
                stop=min(start+64,proposals);xybatch=centers[start:stop];rad=radii[start:stop];height=heights[start:stop]
                clear={}
                for n in (4,8):
                    sphere_centers=np.empty((len(rad),n,3));sphere_centers[:,:,:2]=xybatch[:,None]
                    sphere_centers[:,:,2]=height[:,None]*(np.arange(n)+.5)/n
                    sr=np.sqrt(rad**2+(height/(2*n))**2)
                    values=np.linalg.norm(points[None,:,:,None]-sphere_centers[:,None,None],axis=-1)-rr[None,None,:,None]-sr[:,None,None,None]
                    clear[n]=values.min(axis=(2,3))
                for local,index in enumerate(range(start,stop)):
                    initial=float(clear[4][local,0]);fine=float(clear[8][local].min());goal4=float(clear[4][local,late].min());goal8=float(clear[8][local,late].min())
                    if initial<.006 or np.linalg.norm(centers[index]-xy)<radii[index]+.035:continue
                    if kind=='cover_tradeoff':eligible=fine>.003 and goal4<-.002;score=-goal4
                    elif kind=='path_hazard':eligible=fine<-.006 and goal8>.004;score=-fine
                    else:eligible=.003<fine<.025 and goal8>.003;score=-abs(fine-.01)
                    if eligible:scored.append((score,index,initial,fine,goal4,goal8))
            scored.sort(reverse=True)
            runtime=PickupRuntime(single,DEFAULTS,request,None,'off',folder/case/'probe_unused')
            spec=runtime.scene_spec(mj,ROOT.parent/'frax',request['state'],[],[])
            for i,(r,h) in enumerate(sizes):
                body=spec.worldbody.add_body(name=f'probe_body_{i}',mocap=True,pos=[5.,5.,5.])
                body.add_geom(name=f'probe_{i}',type=mj.mjtGeom.mjGEOM_CYLINDER,size=[r,h/2,0.],contype=0,conaffinity=0)
            spec.attach(runtime.robot_spec(mj,ROOT.parent/'frax'),prefix='',frame=spec.worldbody.add_frame())
            model=spec.compile();data=mj.MjData(model);probe_geoms=[model.geom(f'probe_{i}').id for i in range(len(sizes))]
            target_geom=model.geom('scene/target_object').id
            robot_geoms=[g for g in range(model.ngeom) if model.geom_bodyid[g]!=0 and g not in {target_geom,*probe_geoms}
                         and model.geom_bodyid[g]!=model.body('target').id and (model.geom_contype[g] or model.geom_conaffinity[g])]
            attempts=[];chosen=None
            for score,index,initial,fine,goal4,goal8 in scored[:60]:
                gid=probe_geoms[which[index]];center=np.r_[centers[index],heights[index]/2]
                data.mocap_pos[:]=[5.,5.,5.];data.mocap_pos[model.body_mocapid[model.geom_bodyid[gid]]]=center
                native=[];target_clear=[]
                for j in indices:
                    data.qpos[:],data.qvel[:]=trace['qpos'][j],trace['qvel'][j];mj.mj_forward(model,data)
                    if not np.allclose(data.geom_xpos[gid],center,atol=1e-12,rtol=0):raise ValueError('probe pose mismatch')
                    native.append(min(mj.mj_geomDistance(model,data,gid,g,1000.,None) for g in robot_geoms))
                    target_clear.append(mj.mj_geomDistance(model,data,gid,target_geom,1000.,None))
                native=np.array(native)
                eligible=native[0]>.008 and native[late].min()>.008 and min(target_clear)>.008
                eligible &= native.min()<-.002 if kind=='path_hazard' else native.min()>.005
                attempts.append({'index':index,'center_m':center.tolist(),'radius_m':float(radii[index]),'height_m':float(heights[index]),
                     'native_path_min_m':float(native.min()),'coarse_terminal_m':goal4,'fine_terminal_m':goal8,'fine_path_m':fine,'eligible':bool(eligible)})
                if eligible:chosen=attempts[-1];break
        # Retain a clearly labeled negative result, not a secretly changed stratum.
        if chosen is None:
            chosen={'center_m':[.31,-.25,.12],'radius_m':.025,'height_m':.24,'eligible':False}
        bottle={'id':'bottle_a','shape':'cylinder','size_m':[2*chosen['radius_m']]*2+[chosen['height_m']],
                'center_m':chosen['center_m'],'description':'Stationary tabletop clutter.'}
        box={'id':'box_b','shape':'box','size_m':[.08]*3,'center_m':[.66,-.28 if xy[1]>=0 else .28,.04],
             'description':'Stationary tabletop clutter.'}
        scenes.append({'id':case,'target_id':'target_object','objects':[target,bottle,box]})
        records.append({'case':case,'required_stratum':kind,'qualified':chosen['eligible'],'approximate_candidates':len(scored),'native_attempts':attempts,'chosen':chosen})
        save(folder/'generation_evidence.json',{'records':records})
        print(case,kind,'qualified=',chosen['eligible'],'native attempts',len(attempts),flush=True)
    save(folder/'generated_scenes.json',{'settings':{},'shape_aware_geometry':True,'geometry_counts':[1,2,3,4,5,6,8],
         'terminal_clearance_features':True,'scenes':scenes})
    return records

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--count',type=int,default=12);p.add_argument('--seed',type=int,default=731)
    a=p.parse_args();generate(a.output,count=a.count,seed=a.seed)
