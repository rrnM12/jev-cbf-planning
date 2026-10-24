from jev_cbf.paths import ROOT
from jev_cbf.control.pickup import target_body
from jev_cbf.sensing.rgbd import add_rgbd_cameras
from jev_cbf.control.simulation import scene_spec
from jev_cbf.control.model import panda_spec

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
