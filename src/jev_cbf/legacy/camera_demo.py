from jev_cbf.sensing.camera_io import FrameRecorder
from jev_cbf.paths import ROOT
from jev_cbf.legacy.perception import RGBD
from jev_cbf.sensing.calibration import add_cameras
from jev_cbf.io import read, save
from jev_cbf.reporting.camera import inspect_recording

def demo(output):
    """Two native camera snapshots, no arm controls, SDK or model calls."""
    from jev_cbf.control.pickup import DEFAULTS, PickupRuntime
    from jev_cbf.legacy.requests import pickup_request
    from jev_cbf.upstream import load_upstream
    if output.exists() and any(output.iterdir()):raise ValueError('demo needs a new/empty directory')
    demo,utils=load_upstream(ROOT.parent/'frax',ROOT.parent/'cbfpy');mj=utils.mujoco
    robot=demo.load_panda();scene=read(ROOT/'configs/legacy/scenes/pickup_scenes.json')['scenes'][0]
    request=pickup_request(scene,DEFAULTS,robot,read(ROOT/'configs/legacy/prompts/pickup_terminal.json'))
    runtime=PickupRuntime(scene,DEFAULTS,request,None,'off',output/'unused')
    spec=runtime.scene_spec(mj,ROOT.parent/'frax',request['state'],[],[])
    arm=runtime.robot_spec(mj,ROOT.parent/'frax');add_cameras(spec,arm)
    spec.attach(arm,prefix='',frame=spec.worldbody.add_frame())
    model=spec.compile();data=mj.MjData(model);data.qpos[:7]=robot.default_configuration;data.qpos[7:9]=.04
    mj.mj_forward(model,data)
    sensor=RGBD(mj,model,[o['id'] for o in scene['objects']]);recorder=FrameRecorder(output/'recording');calibration={}
    try:
        for frame,masks,entry in sensor.capture_frames(data):
            recorder.write(frame,masks,identity_source='ORACLE MuJoCo instance masks, NOT learned detection')
            calibration[frame.camera_id]=entry
    finally:sensor.close()
    config=dict(cameras=calibration,support_z_m=0.,margin_m=.005,excluded_ids=[scene['target_id']],
                scope='SYNTHETIC capture-only fixture calibration; not reusable wrist or hardware calibration')
    save(output/'calibration.json',config)
    inspect_recording(output/'recording',output/'inspection',calibration=calibration,support_z=0.,excluded_ids=[scene['target_id']])
    return output
