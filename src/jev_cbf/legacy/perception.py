"""Calibrated simulated RGB/depth observations, separate from evaluation truth.

Instance identities come from MuJoCo segmentation initially (oracle labels, NOT
learned detection). Object dimensions/poses are fitted only from depth points.
Visible-point/table-supported AABBs do not certify unseen full-object coverage.
"""

import argparse
from copy import deepcopy
from pathlib import Path
import time
import numpy as np
from jev_cbf.geometry.covers import make_shape_covers, sphere_union_contains
from jev_cbf.paths import ROOT
from jev_cbf.control.pickup import PickupRuntime
from jev_cbf.legacy.requests import pickup_request
from jev_cbf.upstream import load_upstream
from jev_cbf.io import read, save
from jev_cbf.reporting.images import write_apng
from jev_cbf.sensing.camera_io import CameraFrame, Intrinsics
from jev_cbf.sensing.calibration import camera_axes
from jev_cbf.sensing.calibration import add_cameras
from jev_cbf.sensing.calibration import backproject
from jev_cbf.sensing.geometry import fit_objects
from jev_cbf.sensing.geometry import observed_candidates
from jev_cbf.sensing.geometry import FrameGeometry

class RGBD:
    def __init__(self,mj,model,object_ids,*,width=320,height=240):
        self.mj,self.model=mj,model
        self.width,self.height=width,height
        self.geoms={name:model.geom('scene/'+name).id for name in object_ids}
        model.geom('target_geom').rgba[3]=0  # Hide the upstream control marker, not the pickup object.
        self.renderer=mj.Renderer(model,width=width,height=height)
        self.option=mj.MjvOption();self.option.geomgroup[3]=0;self.option.geomgroup[4]=0

    def render_camera(self,data,name):
        self.renderer.update_scene(data,camera=name,scene_option=self.option)
        self.renderer.disable_depth_rendering();self.renderer.disable_segmentation_rendering()
        rgb=self.renderer.render().copy()
        self.renderer.enable_depth_rendering();depth=self.renderer.render().copy()
        self.renderer.disable_depth_rendering();self.renderer.enable_segmentation_rendering()
        segmentation=self.renderer.render().copy();self.renderer.disable_segmentation_rendering()
        return rgb,depth,segmentation

    def capture_frames(self,data,*,cameras=('scene_camera','wrist_camera')):
        """Common frame boundary + EXPLICIT oracle annotations, no control.

        Native GL remains on its owner thread. RealSense/replay use their own
        source/worker; no background thread touches this simulator's live MjData.
        """
        records=[]
        for name in cameras:
            camera=self.model.camera(name).id
            rgb,depth,segmentation=self.render_camera(data,name)
            focal=.5*self.height/np.tan(np.deg2rad(self.model.cam_fovy[camera])/2)
            matrix=np.eye(4)
            matrix[:3,:3]=data.cam_xmat[camera].reshape(3,3)@np.diag([1.,-1.,-1.])
            matrix[:3,3]=data.cam_xpos[camera]
            frame=CameraFrame(name,int(round(data.time/self.model.opt.timestep)),float(data.time),'simulation',
                rgb,depth,Intrinsics(self.width,self.height,focal,focal,(self.width-1)/2,(self.height-1)/2),
                name+'/optical',metadata={'source':'mujoco','synthetic_calibration':True,'rgb_depth_registered':True})
            masks={obj:(segmentation[:,:,0]==geom)&(segmentation[:,:,1]==self.mj.mjtObj.mjOBJ_GEOM)
                   for obj,geom in self.geoms.items()}
            calibration=dict(mount='fixed',optical_frame=frame.optical_frame,T_robot_optical=matrix.tolist(),
                             calibration_id='SYNTHETIC native pose at this capture; NOT hardware calibration',
                             valid_capture_time_s=frame.capture_time_s,clock_domain=frame.clock_domain)
            records.append((frame,masks,calibration))
        return records

    def capture(self,data,folder=None,*,depth_noise_m=0.,dropout=0.,bias_m=(0.,0.,0.),seed=0,
                cameras=('scene_camera','wrist_camera')):
        clouds={name:[] for name in self.geoms};frames=[]
        rng=np.random.default_rng(seed)
        if not cameras or not set(cameras)<= {'scene_camera','wrist_camera'}:raise ValueError('invalid camera selection')
        for name in cameras:
            camera=self.model.camera(name).id
            rgb,depth,segmentation=self.render_camera(data,name)
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
    def __init__(self,*args,observation_source=None,**kwargs):
        args=list(args);args[2]=deepcopy(args[2])  # Never mutate a raw report's request.
        super().__init__(*args,**kwargs)
        legacy=self.base['state'].pop('observation_config',None)
        self.observation=deepcopy(self.scene.get('observation_config',legacy))
        if self.observation is None and observation_source is not None:self.observation={}
        if self.observation is None:raise ValueError('perception runtime needs observation configuration')
        self.clouds={o['id']:[] for o in self.scene['objects'] if o['id']!=self.target_id}
        self.last_seen={name:None for name in self.clouds}
        self.applied_seen=self.last_seen.copy()
        self.next_observation=0.;self.observation_index=0;self.pending=[];self.observation_events=[]
        # Caller owns/starts/stops the worker. Control only reads its snapshots.
        self.observation_source=observation_source
        self.external_sequence=0

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
        if self.observation_source is not None:
            self.poll_external(initial=True)
            return
        import mujoco as mj
        self.sensor=RGBD(mj,env.model,[o['id'] for o in self.scene['objects']])
        self.observe(initial=True)

    def poll_external(self,*,initial=False):
        snapshot=self.observation_source.latest()
        if snapshot is None:
            if initial:raise ValueError('external camera worker has no prepared geometry snapshot; no actuation')
            return
        if snapshot.sequence==self.external_sequence:return
        self.external_sequence=snapshot.sequence
        observation=snapshot.value;at=float(self.env.data.time)
        if not isinstance(observation,dict):
            raise ValueError('external control input needs prepared geometry, not a raw camera frame')
        event=dict(snapshot_at_s=None,applied_at_s=at,external_camera=True,
                   capture_time_s=observation.get('capture_time_s'),clock_domain=observation.get('clock_domain'),
                   views=deepcopy(observation.get('views',{})),worker_health=self.observation_source.health())
        expected={o['id'] for o in self.scene['objects'] if o['id']!=self.target_id}
        observed={o['id'] for o in observation.get('estimated_objects',[])}
        if observation.get('status')!='ready' or not expected<=observed or event['worker_health']['status']=='error':
            event.update(status='rejected_previous_belief_retained',error='external geometry not ready, missing known clutter, or source error')
            self.observation_events.append(event)
            if initial:raise ValueError('external camera geometry cannot initialize known clutter; no actuation')
            return
        estimates=deepcopy(observation['estimated_objects'])
        # No SDK, projection, fitting, candidate generation or disk I/O in this hook.
        self.pending.append((at,estimates,event))
        self.apply_observations(at,initial=initial)

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
                    if self.observation_source is not None and any(len(c['radii_m'])>self.object_capacity for c in estimate.get('candidates',[])):
                        raise ValueError('external candidate exceeds frozen per-object capacity')
                    old.update(deepcopy(estimate))
                    if self.observation_source is None:old['candidates']=observed_candidates(estimate,counts)
                    elif not estimate.get('candidates'):raise ValueError('external worker did not prepare candidates')
                if not initial:
                    event.update(self.admit(self.policy,self.env.data.qpos[:7],self.env.data.qvel[:7],at))
                event.update(status='accepted',applied_at_s=float(at))
                if 'last_seen_by_object' in event:self.applied_seen=event['last_seen_by_object'].copy()
            except (ValueError,StopIteration) as error:
                self.base=previous
                event.update(status='rejected_previous_belief_retained',applied_at_s=float(at),
                             error='missing known object' if isinstance(error,StopIteration) else str(error))
                if initial:raise ValueError('initial camera observation cannot model every known clutter instance; no oracle fill-in') from error
            self.observation_events.append(event)
            if self.observation_source is None:save(self.output/'observation_events.json',self.observation_events)

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
        if self.observation_source is None:
            self.apply_observations(at)
            if at+1e-9>=self.next_observation:self.observe()
        else:self.poll_external()
        super().before_control(env)

    def make_request(self,q,v,at):
        request=super().make_request(q,v,at)
        request['state']['description']='Static clutter modeled from calibrated scene/wrist depth and oracle instance labels. Clutter dimensions/poses are fitted, not ground truth. The designated target and robot kinematics remain known.'
        request['state']['feature_notes']='Each sphere union covers its current fitted box, not necessarily unseen true surfaces. Camera errors, missing pixels and stale measurements can invalidate coverage. Volume errors are Monte Carlo sampling errors, not uncertainty bounds. No formal perception safety guarantee.'
        request['state']['observation_summary']={'source':'depth geometry; oracle instance identity',
            'geometry_age_s_by_object':{name:None if seen is None else float(at-seen) for name,seen in self.applied_seen.items()},
            'assumed_padding_m':self.observation.get('margin_m',.005)}
        if self.observation_source is not None:
            request['state']['description']='Static clutter from a prepared camera-worker geometry snapshot; mask identity/calibration provenance is explicit. Target geometry/pose and robot kinematics remain known.'
            request['state']['observation_summary']={'source':'external camera-worker snapshot',
                'worker_health':self.observation_source.health(),
                'accepted_observation':deepcopy(next((e for e in reversed(self.observation_events) if e.get('status')=='accepted'),None)),
                'note':'Host snapshot age is not exposure age or robot-clock synchronization. Missing observations never authorize object removal.'}
        return request

    def geometry_context_matches(self,report):
        def geometry(state):
            return [(o['id'],o['shape'],o['size_m'],o['center_m'],
                     [(c['id'],c['centers_object_m'],c['radii_m']) for c in o['candidates']]) for o in state['objects']]
        return (report['request']['state']['motion']['goal_m']==self.base['state']['motion']['goal_m']
                and geometry(report['request']['state'])==geometry(self.base['state']))

    def finish(self,result,trajectory):
        super().finish(result,trajectory)
        if hasattr(self,'sensor'):self.sensor.close()
        save(self.output/'observation_events.json',self.observation_events)
        result.update(observation_config=self.observation,observation_events=self.observation_events,
                      perception_scope='Clutter geometry comes from two calibrated RGB/depth cameras with oracle instance labels. Target geometry/pose and robot/camera kinematics remain known. Physical truth is used only for scene construction/evaluation, not clutter barrier parameters. Retained stale/rejected estimates are not a certified safety fallback.')
        if self.observation_source is not None:
            result['perception_scope']='Simulation consumed external prepared camera-worker snapshots. Capture/masks/calibration/fitting/recording are outside control. Target/robot remain known; caller owns the worker. No hardware capture, hard-real-time or physical-robot safety validation.'
            result['camera_worker_health']=self.observation_source.health()

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
