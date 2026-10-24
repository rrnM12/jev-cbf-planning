"""Depth geometry tests do not use true object dimensions in the estimator."""

import unittest
from copy import deepcopy
from pathlib import Path
import tempfile
import numpy as np
from jev_cbf.sensing.calibration import backproject, camera_axes
from jev_cbf.sensing.geometry import fit_objects


class PerceptionTests(unittest.TestCase):
    def test_backprojection_camera_frame_and_pixel_centers(self):
        depth=np.full((2,2),2.)
        cloud=backproject(depth,[1.,2.,3.],np.eye(3),90.)
        np.testing.assert_allclose(cloud[0,0],[0.,3.,1.],atol=1e-14)
        np.testing.assert_allclose(cloud[1,1],[2.,1.,1.],atol=1e-14)
        axes=camera_axes(np.array([0.,-1.,1.]),[0.,0.,0.]).reshape(2,3)
        self.assertAlmostEqual(float(axes[0]@axes[1]),0.)
        np.testing.assert_allclose(np.linalg.norm(axes,axis=1),1.)

    def test_fit_uses_visible_points_not_oracle_shape_and_retains_missing(self):
        cloud=np.array([[x,y,z] for x in (.2,.3) for y in (-.1,.1) for z in (.05,.2)])
        result=fit_objects({'observed':cloud,'hidden':np.empty((0,3))},margin_m=.01)
        self.assertEqual([o['id'] for o in result],['observed'])
        obj=result[0]
        self.assertEqual(obj['shape'],'box')
        np.testing.assert_allclose(obj['center_m'],[.25,0.,.1])
        np.testing.assert_allclose(obj['size_m'],[.12,.22,.22])
        self.assertIn('unseen coverage unproven',obj['description'])
        with self.assertRaises(ValueError):fit_objects({'a':cloud},margin_m=-1.)

    def test_native_camera_clutter_reaches_cbf_without_changing_physics_truth(self):
        from jev_cbf.legacy.perception import observe_initial, PerceivedPickupRuntime
        from jev_cbf.control.pickup import DEFAULTS
        from jev_cbf.legacy.requests import pickup_request
        from jev_cbf.paths import ROOT
        from jev_cbf.upstream import load_upstream
        from jev_cbf.control.simulation import run_case
        from jev_cbf.io import read
        demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');robot=demo.load_panda()
        scene=deepcopy(read(ROOT/'configs/legacy/scenes/pickup_scenes.json')['scenes'][0])
        template=read(ROOT/'configs/legacy/prompts/pickup_terminal.json')
        options=dict(shape_aware=True,counts=(1,2,3,4,5,6,8),terminal_features=True)
        observation={'margin_m':.005,'period_s':1.,'seed':0}
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            fitted=observe_initial(scene,DEFAULTS,robot,template,options,folder/'initial',observation)
            self.assertNotEqual(fitted['objects'][1]['size_m'],scene['objects'][1]['size_m'])
            self.assertEqual(fitted['objects'][1]['shape'],'box')
            request=pickup_request(fitted,DEFAULTS,robot,template,**options)
            request['state'].update(observation_config=observation,geometry_counts=list(options['counts']))
            before=deepcopy(request)
            runtime=PerceivedPickupRuntime(scene,DEFAULTS,request,None,'all',folder/'run')
            result,_=run_case(demo,utils,{'request':request},'off',frax_dir=ROOT.parent/'frax',steps=2,runtime=runtime,scene_guard=.05)
            self.assertEqual(request,before)  # Raw request is not mutated by belief updates.
            physical=runtime.env.model.geom('scene/bottle_a')
            self.assertEqual(int(physical.type[0]),int(utils.mujoco.mjtGeom.mjGEOM_CYLINDER))
            params=runtime.parameters(0.)
            runtime.scene['objects'][1]['center_m']=[9.,9.,9.]
            np.testing.assert_array_equal(runtime.parameters(0.)[0],params[0])
            self.assertTrue(result['observation_events'])
            self.assertIn('target geometry/pose',result['perception_scope'].lower())
            visible=runtime.make_request(np.asarray(robot.default_configuration),np.zeros(7),2.)['state']
            self.assertNotIn('observation_config',visible)
            self.assertNotIn('calibration_bias_m',visible)
            self.assertIn('geometry_age_s_by_object',visible['observation_summary'])

    def test_common_simulator_frame_projection_matches_legacy_and_worker_drives_sim_filter(self):
        from jev_cbf.sensing.camera_io import CameraWorker, FrameRecorder, ReplaySource, deproject, robot_transform
        from jev_cbf.sensing.geometry import FrameGeometry
        from jev_cbf.legacy.perception import RGBD, PerceivedPickupRuntime
        from jev_cbf.control.pickup import DEFAULTS, PickupRuntime
        from jev_cbf.legacy.requests import pickup_request
        from jev_cbf.paths import ROOT
        from jev_cbf.upstream import load_upstream
        from jev_cbf.control.simulation import run_case
        from jev_cbf.io import read
        from unittest.mock import patch
        demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');robot=demo.load_panda();mj=utils.mujoco
        scene=deepcopy(read(ROOT/'configs/legacy/scenes/pickup_scenes.json')['scenes'][0])
        template=read(ROOT/'configs/legacy/prompts/pickup_terminal.json');options=dict(shape_aware=True,counts=(1,2,3,4,5,6,8))
        seed=pickup_request(scene,DEFAULTS,robot,template,**options)
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);builder=PickupRuntime(scene,DEFAULTS,seed,None,'off',folder/'unused')
            spec=builder.scene_spec(mj,ROOT.parent/'frax',seed['state'],[],[])
            from jev_cbf.sensing.calibration import add_cameras
            arm=builder.robot_spec(mj,ROOT.parent/'frax');add_cameras(spec,arm)
            spec.attach(arm,prefix='',frame=spec.worldbody.add_frame())
            model=spec.compile();data=mj.MjData(model);data.qpos[:7]=robot.default_configuration;data.qpos[7:9]=.04;mj.mj_forward(model,data)
            sensor=RGBD(mj,model,[o['id'] for o in scene['objects']],width=160,height=120)
            try:records=sensor.capture_frames(data,cameras=('scene_camera',))
            finally:sensor.close()
            f,masks,entry=records[0];matrix=robot_transform(f,{f.camera_id:entry})
            common=deproject(f)@matrix[:3,:3].T+matrix[:3,3]
            camera=model.camera(f.camera_id).id
            legacy=backproject(f.depth_m,data.cam_xpos[camera],data.cam_xmat[camera],model.cam_fovy[camera])
            np.testing.assert_allclose(common,legacy,rtol=1e-12,atol=1e-10)
            recorder=FrameRecorder(folder/'recording');recorder.write(f,masks,identity_source='ORACLE simulator masks')
            geometry=FrameGeometry({f.camera_id:entry},lambda f,s:s.annotations(f),support_z=0.,
                                   excluded_ids=[scene['target_id']],expected_ids=['bottle_a','box_b'])
            worker=CameraWorker(lambda:ReplaySource(folder/'recording'),geometry).start();worker.thread.join(5.)
            self.assertFalse(worker.thread.is_alive());self.assertEqual(worker.health()['status'],'eof')
            snapshot=worker.latest().value;self.assertEqual(snapshot['status'],'ready')
            observed=deepcopy(scene);observed['objects']=[scene['objects'][0],*snapshot['estimated_objects']]
            request=pickup_request(observed,DEFAULTS,robot,template,**options)
            runtime=PerceivedPickupRuntime(scene,DEFAULTS,request,None,'all_fine',folder/'run',observation_source=worker)
            with (patch.object(runtime,'observe',side_effect=AssertionError('synchronous camera call in external control path')),
                  patch('jev_cbf.legacy.perception.observed_candidates',side_effect=AssertionError('candidate generation in control path'))):
                result,trace=run_case(demo,utils,{'request':request},'off',frax_dir=ROOT.parent/'frax',steps=4,
                                      runtime=runtime,scene_guard=.05,control_dt=.002)
            self.assertTrue(result['complete']);self.assertGreaterEqual(trace['scene_h2_m'].min(),0.)
            self.assertEqual(runtime.observation_events[0]['status'],'accepted')
            self.assertFalse(hasattr(runtime,'sensor'))
            self.assertEqual(int(runtime.env.model.geom('scene/bottle_a').type[0]),int(mj.mjtGeom.mjGEOM_CYLINDER))
            self.assertIn('external',result['perception_scope'])
            worker.close()

    def test_frame_geometry_missing_masks_calibration_and_freshness_are_explicit(self):
        from jev_cbf.sensing.camera_io import CameraFrame, Intrinsics
        from jev_cbf.sensing.geometry import FrameGeometry
        f=CameraFrame('scene',0,0.,'synthetic',np.zeros((4,4,3),dtype=np.uint8),np.ones((4,4)),
                      Intrinsics(4,4,4.,4.,1.5,1.5),'scene/optical')
        entry=dict(mount='fixed',optical_frame=f.optical_frame,calibration_id='SYNTHETIC',T_robot_optical=np.eye(4))
        geometry=FrameGeometry({'scene':entry},None,support_z=0.)
        with self.assertRaisesRegex(ValueError,'masks'):geometry(f)
        geometry.mask_provider=lambda f,s:({'box':np.ones((4,4),dtype=bool)},'SYNTHETIC masks')
        first=geometry(f);self.assertEqual(first['status'],'ready')
        geometry.mask_provider=lambda f,s:({'box':np.zeros((4,4),dtype=bool)},'SYNTHETIC missing mask')
        from dataclasses import replace
        second=geometry(replace(f,frame_number=1,capture_time_s=1.))
        self.assertEqual(second['views']['scene']['seen']['box']['capture_time_s'],0.)
        self.assertEqual(first['views']['scene']['seen']['box']['capture_time_s'],0.)
        geometry.calibration={}
        with self.assertRaisesRegex(ValueError,'calibration'):geometry(f)
