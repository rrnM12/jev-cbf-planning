"""Depth geometry tests do not use true object dimensions in the estimator."""

import unittest
from copy import deepcopy
from pathlib import Path
import tempfile
import numpy as np
from perception import backproject,fit_objects,camera_axes


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
        from perception import observe_initial,PerceivedPickupRuntime
        from pickup import DEFAULTS,pickup_request
        from inspect_selection import ROOT
        from run_static import load_upstream,run_case
        from run_suite import read
        demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');robot=demo.load_panda()
        scene=deepcopy(read(ROOT/'examples/pickup_scenes.json')['scenes'][0])
        template=read(ROOT/'prompts/pickup_terminal.json')
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
