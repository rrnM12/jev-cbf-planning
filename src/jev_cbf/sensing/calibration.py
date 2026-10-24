import numpy as np

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
