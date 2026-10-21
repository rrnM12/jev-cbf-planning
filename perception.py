"""Calibrated simulated RGB/depth observations, separate from evaluation truth.

Instance identities come from MuJoCo segmentation initially (oracle labels, NOT
learned detection). Object dimensions/poses are fitted only from depth points.
Visible-point/table-supported AABBs do not certify unseen full-object coverage.
"""

import argparse
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np

from geometry import make_shape_covers,sphere_union_contains,evaluate_covers
from inspect_selection import ROOT
from pickup import PickupRuntime,pickup_request
from run_static import load_upstream
from run_suite import read,save
from static_gallery import write_apng


def camera_axes(eye,lookat):
    back=np.asarray(eye,dtype=float)-lookat;back/=np.linalg.norm(back)
    right=np.cross([0.,0.,1.],back);right/=np.linalg.norm(right)
    return np.r_[right,np.cross(back,right)]


def add_cameras(scene_spec=None,robot_spec=None):
    """Known extrinsics: fixed scene camera plus a hand-mounted wrist camera."""
    eye=np.array([.85,-.75,.85])
    if scene_spec is not None:
        scene_spec.worldbody.add_camera(name='scene_camera',pos=eye,xyaxes=camera_axes(eye,[.4,0.,.12]),fovy=58.)
    if robot_spec is not None:
        robot_spec.body('hand').add_camera(name='wrist_camera',pos=[.055,0.,.03],
                                          xyaxes=[1.,0.,0.,0.,-1.,0.],fovy=90.)


def backproject(depth,position,rotation,fovy):
    """Metric optical-axis depth; MuJoCo camera looks along local -Z, Y up."""
    depth=np.asarray(depth,dtype=float)
    if depth.ndim!=2 or not np.isfinite(fovy) or not 0<fovy<180:
        raise ValueError('need HxW metric depth and a perspective vertical FOV')
    height,width=depth.shape;focal=.5*height/np.tan(np.deg2rad(fovy)/2)
    row,col=np.indices(depth.shape)
    local=np.stack([(col+.5-width/2)*depth/focal,(height/2-row-.5)*depth/focal,-depth],axis=-1)
    return local@np.asarray(rotation).reshape(3,3).T+position


def fit_objects(clouds,*,margin_m=.005,support_z=0.):
    """Fit table-supported boxes WITHOUT shape, size or pose oracle inputs.

    The margin is an experimental parameter, not calibrated uncertainty. Fusing
    views can reduce missing surfaces; the table assumption only supplies Zmin.
    """
    if not np.isfinite(margin_m) or margin_m<0:raise ValueError('invalid metric margin')
    objects=[]
    for name,points in clouds.items():
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        if len(points)<8:continue
        lower,upper=points.min(axis=0)-margin_m,points.max(axis=0)+margin_m
        lower[2]=support_z-margin_m
        if np.any(upper<=lower):continue
        objects.append({'id':name,'shape':'box','center_m':((lower+upper)/2).tolist(),
                        'size_m':(upper-lower).tolist(),'description':'Depth-fitted visible-surface/table-supported AABB; unseen coverage unproven.',
                        'observed_points':len(points),'margin_m':margin_m})
    return objects


class RGBD:
    def __init__(self,mj,model,object_ids,*,width=320,height=240):
        self.mj,self.model=mj,model
        self.geoms={name:model.geom('scene/'+name).id for name in object_ids}
        model.geom('target_geom').rgba[3]=0  # Hide the upstream control marker, not the pickup object.
        self.renderer=mj.Renderer(model,width=width,height=height)
        self.option=mj.MjvOption();self.option.geomgroup[3]=0;self.option.geomgroup[4]=0

    def capture(self,data,folder=None,*,depth_noise_m=0.,dropout=0.,bias_m=(0.,0.,0.),seed=0,
                cameras=('scene_camera','wrist_camera')):
        clouds={name:[] for name in self.geoms};frames=[]
        rng=np.random.default_rng(seed)
        if not cameras or not set(cameras)<= {'scene_camera','wrist_camera'}:raise ValueError('invalid camera selection')
        for name in cameras:
            camera=self.model.camera(name).id
            self.renderer.update_scene(data,camera=name,scene_option=self.option)
            self.renderer.disable_depth_rendering();self.renderer.disable_segmentation_rendering()
            rgb=self.renderer.render().copy()
            self.renderer.enable_depth_rendering();depth=self.renderer.render().copy()
            self.renderer.disable_depth_rendering();self.renderer.enable_segmentation_rendering()
            segmentation=self.renderer.render().copy();self.renderer.disable_segmentation_rendering()
            measured=depth+rng.normal(0.,depth_noise_m,depth.shape)
            world=backproject(measured,data.cam_xpos[camera],data.cam_xmat[camera],self.model.cam_fovy[camera])
            world+=np.asarray(bias_m)
            retained=(rng.random(depth.shape)>=dropout)&(measured>0)&(measured<3.)
            for obj,geom in self.geoms.items():
                mask=(segmentation[:,:,0]==geom)&(segmentation[:,:,1]==self.mj.mjtObj.mjOBJ_GEOM)&retained
                clouds[obj].append(world[mask])
            frames.append({'camera':name,'camera_position_world_m':data.cam_xpos[camera].tolist(),
                           'camera_rotation_world':data.cam_xmat[camera].reshape(3,3).tolist(),
                           'vertical_fov_deg':float(self.model.cam_fovy[camera]),'image_shape':list(depth.shape)})
            if folder is not None:
                folder.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(folder/(name+'.npz'),rgb=rgb,depth_m=depth,measured_depth_m=measured,segmentation=segmentation,
                                    camera_position_world_m=data.cam_xpos[camera],camera_rotation_world=data.cam_xmat[camera],fovy=self.model.cam_fovy[camera])
                write_apng(folder/(name+'.png'),[rgb],frame_count=1,fps=1)
                scaled=np.clip(depth/1.8,0.,1.)
                image=np.stack([1-scaled,1-abs(2*scaled-1),scaled],axis=-1)
                image[depth>=3.]=0
                write_apng(folder/(name+'_depth.png'),[(image*255).astype(np.uint8)],frame_count=1,fps=1)
        clouds={name:np.concatenate(points) for name,points in clouds.items()}
        metadata={'time_s':float(data.time),'cameras':frames,'visible_points':{k:len(v) for k,v in clouds.items()},
                  'instance_geometry_ids':self.geoms,
                  'depth_noise_m':depth_noise_m,'pixel_dropout':dropout,'world_calibration_bias_m':list(bias_m),
                  'identity_source':'Oracle simulator instance segmentation only. No true object dimensions, shape or center is used by backprojection/fitting.'}
        if folder is not None:save(folder/'observation.json',metadata)
        return clouds,metadata

    def close(self):self.renderer.close()


def observed_candidates(obj,counts):
    covers=make_shape_covers(np.array(obj['size_m'])/2,shape='box',counts=counts)
    measures=evaluate_covers(np.array(obj['size_m'])/2,covers,shape='box',samples=10_000,seed=0)
    return [{'id':c.candidate_id,'centers_object_m':[list(p) for p in c.centers_m],
             'radii_m':list(c.radii_m),'primitive_count':len(c.radii_m),
             'excess_volume_m3':m['excess_volume_m3'],
             'excess_volume_standard_error_m3':m['excess_volume_standard_error_m3']}
            for c,m in zip(covers,measures['candidates'],strict=True)]


def observe_initial(scene,settings,robot,template,options,output,config):
    """Generate measured clutter inputs; keep only the designated target known."""
    demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');mj=utils.mujoco
    seed=pickup_request(scene,settings,robot,template,**options)
    runtime=PickupRuntime(scene,settings,seed,None,'off',output/'unused')
    spec=runtime.scene_spec(mj,ROOT.parent/'frax',seed['state'],[],[])
    arm=runtime.robot_spec(mj,ROOT.parent/'frax');add_cameras(spec,arm)
    spec.attach(arm,prefix='',frame=spec.worldbody.add_frame())
    model=spec.compile();data=mj.MjData(model)
    data.qpos[:7]=robot.default_configuration;data.qpos[7:9]=.04;mj.mj_forward(model,data)
    sensor=RGBD(mj,model,[o['id'] for o in scene['objects']])
    try:
        clouds,metadata=sensor.capture(data,output,depth_noise_m=config.get('depth_noise_m',0.),
            dropout=config.get('pixel_dropout',0.),bias_m=config.get('calibration_bias_m',[0.,0.,0.]),seed=config.get('seed',0))
    finally:sensor.close()
    estimates=fit_objects(clouds,margin_m=config.get('margin_m',.005))
    missing=[o['id'] for o in scene['objects'] if o['id']!=scene['target_id'] and o['id'] not in {e['id'] for e in estimates}]
    if missing:raise ValueError('initial camera observation misses known clutter instances; no oracle geometry fill-in: '+','.join(missing))
    fitted=deepcopy(scene)
    fitted['objects']=[deepcopy(o) if o['id']==scene['target_id'] else
                       next(e for e in estimates if e['id']==o['id']) for o in scene['objects']]
    save(output/'fit.json',{'observation':metadata,'estimated_scene':fitted,
                          'scope':'Clutter geometry fitted from camera points; designated target geometry/pose remains known. Missing initial instances cause preparation failure, not oracle fill-in.'})
    return fitted


class PerceivedPickupRuntime(PickupRuntime):
    """Depth-derived CLUTTER barriers; target/robot kinematics still known.

    This deliberate first perception boundary isolates obstacle modeling from
    grasp-pose estimation. Simulator instance identities remain oracle scaffolding.
    Physical scene truth is never replaced by the fitted boxes.
    """
    def __init__(self,*args,**kwargs):
        args=list(args);args[2]=deepcopy(args[2])  # Never mutate a raw report's request.
        super().__init__(*args,**kwargs)
        legacy=self.base['state'].pop('observation_config',None)
        self.observation=deepcopy(self.scene.get('observation_config',legacy))
        if self.observation is None:raise ValueError('perception runtime needs observation configuration')
        self.clouds={o['id']:[] for o in self.scene['objects'] if o['id']!=self.target_id}
        self.last_seen={name:None for name in self.clouds}
        self.applied_seen=self.last_seen.copy()
        self.next_observation=0.;self.observation_index=0;self.pending=[];self.observation_events=[]

    def scene_spec(self,mj,frax_dir,state,centers,radii):
        physical=deepcopy(state);physical['objects']=deepcopy(self.scene['objects'])
        spec=super().scene_spec(mj,frax_dir,physical,centers,radii)
        add_cameras(scene_spec=spec)
        return spec

    def robot_spec(self,mj,frax_dir):
        robot=super().robot_spec(mj,frax_dir)
        add_cameras(robot_spec=robot)
        return robot

    def initialize(self,env):
        super().initialize(env)
        import mujoco as mj
        self.sensor=RGBD(mj,env.model,[o['id'] for o in self.scene['objects']])
        self.observe(initial=True)

    def observe(self,*,initial=False):
        at=float(self.env.data.time);config=self.observation
        index=self.observation_index;self.observation_index+=1
        missing=index in config.get('missing_frames',[])
        start=time.perf_counter()
        clouds,metadata=self.sensor.capture(self.env.data,self.output/'observations'/f'{index:03d}',
            depth_noise_m=config.get('depth_noise_m',0.),dropout=1. if missing else config.get('pixel_dropout',0.),
            bias_m=config.get('calibration_bias_m',[0.,0.,0.]),seed=config.get('seed',0)+index,
            cameras=tuple(config.get('cameras',('scene_camera','wrist_camera'))))
        for name in self.clouds:
            if len(clouds[name]):
                self.clouds[name].append(clouds[name]);self.last_seen[name]=at
        fused={name:np.concatenate(points) if points else np.empty((0,3)) for name,points in self.clouds.items()}
        estimates=fit_objects(fused,margin_m=config.get('margin_m',.005))
        event={'snapshot_at_s':at,'observation_index':index,'visible_points':metadata['visible_points'],
               'measurement_latency_host_s':time.perf_counter()-start,'missing_frame_injected':missing,
               'estimated_objects':estimates,'identity_source':metadata['identity_source'],
               'last_seen_by_object':self.last_seen.copy()}
        self.pending.append((at+(0. if initial else config.get('delay_s',0.)),estimates,event))
        self.next_observation=at+config.get('period_s',1.)
        self.apply_observations(at,initial=initial)

    def apply_observations(self,at,*,initial=False):
        while self.pending and self.pending[0][0]<=at+1e-9:
            _,estimates,event=self.pending.pop(0)
            previous=deepcopy(self.base)
            counts=tuple(self.base['state'].get('geometry_counts',(1,2,3,4,5,6,8)))
            try:
                for old in self.base['state']['objects']:
                    if old['id']==self.target_id:continue
                    estimate=next(o for o in estimates if o['id']==old['id'])
                    old.update(deepcopy(estimate));old['candidates']=observed_candidates(estimate,counts)
                if not initial:
                    event.update(self.admit(self.policy,self.env.data.qpos[:7],self.env.data.qvel[:7],at))
                event.update(status='accepted',applied_at_s=float(at))
                self.applied_seen=event['last_seen_by_object'].copy()
            except (ValueError,StopIteration) as error:
                self.base=previous
                event.update(status='rejected_previous_belief_retained',applied_at_s=float(at),
                             error='missing known object' if isinstance(error,StopIteration) else str(error))
                if initial:raise ValueError('initial camera observation cannot model every known clutter instance; no oracle fill-in') from error
            self.observation_events.append(event)
            save(self.output/'observation_events.json',self.observation_events)

    def positions(self):
        positions=super().positions()
        for index,obj in enumerate(self.base['state']['objects']):
            if obj['id']!=self.target_id:positions[index]=obj['center_m']
        return positions

    def parameters(self,at,policy=None):
        args=list(super().parameters(at,policy));policy=self.policy if policy is None else policy
        for index,obj in enumerate(self.base['state']['objects']):
            if obj['id']==self.target_id or policy[obj['id']] is None:continue
            candidate=next(c for c in obj['candidates'] if c['id']==policy[obj['id']])
            slots=slice(index*self.object_capacity,index*self.object_capacity+len(candidate['radii_m']))
            args[0][slots]=np.array(candidate['centers_object_m'])+obj['center_m']
        return tuple(args)

    def before_control(self,env):
        at=float(env.data.time)
        self.apply_observations(at)
        if at+1e-9>=self.next_observation:self.observe()
        super().before_control(env)

    def make_request(self,q,v,at):
        request=super().make_request(q,v,at)
        request['state']['description']='Static clutter modeled from calibrated scene/wrist depth and oracle instance labels. Clutter dimensions/poses are fitted, not ground truth. The designated target and robot kinematics remain known.'
        request['state']['feature_notes']='Each sphere union covers its current fitted box, not necessarily unseen true surfaces. Camera errors, missing pixels and stale measurements can invalidate coverage. Volume errors are Monte Carlo sampling errors, not uncertainty bounds. No formal perception safety guarantee.'
        request['state']['observation_summary']={'source':'depth geometry; oracle instance identity',
            'geometry_age_s_by_object':{name:None if seen is None else float(at-seen) for name,seen in self.applied_seen.items()},
            'assumed_padding_m':self.observation.get('margin_m',.005)}
        return request

    def geometry_context_matches(self,report):
        def geometry(state):
            return [(o['id'],o['shape'],o['size_m'],o['center_m'],
                     [(c['id'],c['centers_object_m'],c['radii_m']) for c in o['candidates']]) for o in state['objects']]
        return (report['request']['state']['motion']['goal_m']==self.base['state']['motion']['goal_m']
                and geometry(report['request']['state'])==geometry(self.base['state']))

    def finish(self,result,trajectory):
        super().finish(result,trajectory)
        self.sensor.close()
        result.update(observation_config=self.observation,observation_events=self.observation_events,
                      perception_scope='Clutter geometry comes from two calibrated RGB/depth cameras with oracle instance labels. Target geometry/pose and robot/camera kinematics remain known. Physical truth is used only for scene construction/evaluation, not clutter barrier parameters. Retained stale/rejected estimates are not a certified safety fallback.')

    def close(self):
        super().close()
        if hasattr(self,'sensor'):
            self.sensor.close()


def coverage_truth(objects,estimates,*,counts=(1,2,3,4,5,6,8),samples=30_000,seed=0):
    """Independent oracle evaluation, never fed back into observation fitting."""
    rng=np.random.default_rng(seed);rows=[]
    for obj in objects:
        estimate=next((o for o in estimates if o['id']==obj['id']),None)
        if estimate is None:
            rows.append({'object_id':obj['id'],'observed':False});continue
        half=np.array(obj['size_m'])/2
        points=rng.uniform(-half,half,(samples*2,3))
        if obj['shape']=='cylinder':points=points[np.linalg.norm(points[:,:2],axis=1)<=half[0]]
        points=points[:samples]+obj['center_m']-np.array(estimate['center_m'])
        covers=make_shape_covers(np.array(estimate['size_m'])/2,shape='box',counts=counts)
        candidate=next((c for c in covers if c.candidate_id=='box-grid-8'),
                       next(c for c in covers if c.candidate_id=='box-split-8'))
        inside=sphere_union_contains(points,candidate)
        rows.append({'object_id':obj['id'],'observed':True,'estimated_center_error_m':float(np.linalg.norm(np.array(estimate['center_m'])-obj['center_m'])),
                     'aabb_true_volume_fraction_missing':float(np.mean(np.any(abs(points)>np.array(estimate['size_m'])/2,axis=1))),
                     'cover_id':candidate.candidate_id,'cover_true_volume_fraction_missing':float(np.mean(~inside)),
                     'reference_points':len(points)})
    return rows


def inspect_saved(source,case,output):
    if output.exists() and any(output.iterdir()):raise ValueError('use a new observation output directory')
    scene=read(source/case/'scene.json');request=read(source/case/'initial/request.json');settings=read(source/'settings.json')
    demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');mj=utils.mujoco
    runtime=PickupRuntime(scene,settings,request,None,'off',output/'unused')
    spec=runtime.scene_spec(mj,ROOT.parent/'frax',request['state'],[],[])
    robot=runtime.robot_spec(mj,ROOT.parent/'frax');add_cameras(spec,robot)
    spec.attach(robot,prefix='',frame=spec.worldbody.add_frame())
    model=spec.compile();data=mj.MjData(model);sensor=RGBD(mj,model,[o['id'] for o in scene['objects']])
    merged={o['id']:[] for o in scene['objects']};records=[]
    try:
        with np.load(source/case/'all/trajectory.npz') as trace:
            indices=np.unique([0,*[int(np.flatnonzero(trace['phase']==p)[0]) for p in ('pregrasp','descend','close')]])
            for n,index in enumerate(indices):
                data.qpos[:],data.qvel[:],data.time=trace['qpos'][index],trace['qvel'][index],trace['time_s'][index]
                mj.mj_forward(model,data)
                clouds,metadata=sensor.capture(data,output/f'{n:02d}')
                for name,points in clouds.items():merged[name].append(points)
                fused={name:np.concatenate(points) for name,points in merged.items()}
                estimates=fit_objects(fused)
                records.append({'observation':metadata,'estimated_objects':estimates,'independent_truth_evaluation':coverage_truth(scene['objects'],estimates)})
    finally:sensor.close()
    save(output/'report.json',{'source':str(source),'case':case,'observations':records,
                              'scope':'Rendered wrist + scene RGB/depth, oracle instance labels; depth-only AABB fitting. Multi-view geometry diagnostics, not a closed-loop perceived-geometry controller yet. Visible coverage is not full-object certification.'})
    print('Camera observations:',output,flush=True)


def sensor_study(source,output):
    """Controlled raw-depth corruptions, no policy/model calls or oracle fitting."""
    original=read(source/'report.json');rows=[]
    conditions={
        'two_views_clean':{},'scene_only':{'cameras':['scene_camera']},
        'scene_only_no_padding':{'cameras':['scene_camera'],'margin_m':0.},
        'noise_and_dropout':{'depth_noise_m':.003,'pixel_dropout':.5},
        'calibration_error':{'bias_m':[.012,-.008,0.]},
        'large_calibration_error':{'bias_m':[.025,-.015,0.]},
    }
    scene=read(Path(original['source'])/original['case']/'scene.json')
    for label,condition in conditions.items():
        fused={o['id']:[] for o in scene['objects']}
        rng=np.random.default_rng(91)
        for i,observation in enumerate(original['observations']):
            metadata=observation['observation'];mapping=metadata['instance_geometry_ids']
            for camera in condition.get('cameras',('scene_camera','wrist_camera')):
                with np.load(source/f'{i:02d}'/(camera+'.npz')) as frame:
                    depth=frame['depth_m']+rng.normal(0.,condition.get('depth_noise_m',0.),frame['depth_m'].shape)
                    points=backproject(depth,frame['camera_position_world_m'],frame['camera_rotation_world'],float(frame['fovy']))
                    points+=condition.get('bias_m',[0.,0.,0.])
                    keep=(rng.random(depth.shape)>=condition.get('pixel_dropout',0.))&(depth>0)&(depth<3)
                    segmentation=frame['segmentation']
                    for name,geom in mapping.items():
                        mask=(segmentation[:,:,0]==geom)&(segmentation[:,:,1]==5)&keep
                        fused[name].append(points[mask])
            estimates=fit_objects({name:np.concatenate(p) for name,p in fused.items()},margin_m=condition.get('margin_m',.005))
            rows.append({'condition':label,'frame':i,'estimated_objects':estimates,
                         'independent_truth_evaluation':coverage_truth(scene['objects'],estimates,seed=91)})
    save(output,{'conditions':conditions,'rows':rows,
         'scope':'Offline same-raw-depth controlled sensor study. Oracle instance labels, no oracle geometry in fitting. Margins are parameters, not calibrated uncertainty. Bias is injected in world calibration, noise in optical depth. No new physics or model calls.'})
    print('Sensor study:',output,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=ROOT/'artifacts/pickup/corrected_replication_v2')
    p.add_argument('--case',default='layout_03');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();inspect_saved(a.source,a.case,a.output)
