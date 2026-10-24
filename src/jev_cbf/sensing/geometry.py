from copy import deepcopy
import numpy as np
from jev_cbf.geometry.covers import make_shape_covers, evaluate_covers
from jev_cbf.sensing.camera_io import masked_clouds, robot_transform

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

def observed_candidates(obj,counts):
    covers=make_shape_covers(np.array(obj['size_m'])/2,shape='box',counts=counts)
    measures=evaluate_covers(np.array(obj['size_m'])/2,covers,shape='box',samples=10_000,seed=0)
    return [{'id':c.candidate_id,'centers_object_m':[list(p) for p in c.centers_m],
             'radii_m':list(c.radii_m),'primitive_count':len(c.radii_m),
             'excess_volume_m3':m['excess_volume_m3'],
             'excess_volume_standard_error_m3':m['excess_volume_standard_error_m3']}
            for c,m in zip(covers,measures['candidates'],strict=True)]

class FrameGeometry:
    """Replaceable masks + calibrated frames -> existing fitting/candidate code.

    One latest cloud per camera for STATIC clutter, not a moving-object tracker.
    No data-source/robot communication is hidden inside geometry processing.
    """
    def __init__(self,calibration,mask_provider,*,support_z,margin_m=.005,expected_ids=(),
                 excluded_ids=(),pose_provider=None,counts=(1,2,3,4,5,6,8)):
        if support_z is None or not np.isfinite(support_z):
            raise ValueError('geometry needs an explicit robot-frame table support height')
        self.calibration,self.mask_provider=deepcopy(calibration),mask_provider
        self.support_z,self.margin=float(support_z),margin_m
        self.expected,self.excluded=set(expected_ids),set(excluded_ids)
        self.pose_provider,self.counts=pose_provider,counts
        self.views={}

    def __call__(self,frame,source=None):
        matrix=robot_transform(frame,self.calibration,pose_provider=self.pose_provider)
        if self.mask_provider is None:
            raise ValueError('object masks/identities are required; raw RGBD is not an empty obstacle scene')
        masks,identity=self.mask_provider(frame,source)
        if not identity:
            raise ValueError('mask identity provenance is required')
        clouds=masked_clouds(frame,{k:v for k,v in masks.items() if k not in self.excluded},matrix)
        # An empty/newly missing mask does NOT refresh that object's last-seen time.
        previous=self.views.get(frame.camera_id,{}).get('clouds',{})
        seen=self.views.get(frame.camera_id,{}).get('seen',{}).copy()
        retained={k:v for k,v in previous.items()}
        for name,points in clouds.items():
            if len(points):
                retained[name]=points
                seen[name]=dict(capture_time_s=frame.capture_time_s,clock_domain=frame.clock_domain,
                                received_monotonic_s=frame.received_monotonic_s)
        self.views[frame.camera_id]=dict(clouds=retained,seen=seen,identity_source=identity,
                                        calibration_id=self.calibration[frame.camera_id]['calibration_id'])
        names=set().union(*(v['clouds'] for v in self.views.values()))
        fused={name:np.concatenate([v['clouds'][name] for v in self.views.values() if name in v['clouds']]) for name in sorted(names)}
        estimates=fit_objects(fused,margin_m=self.margin,support_z=self.support_z)
        for obj in estimates:obj['candidates']=observed_candidates(obj,self.counts)
        observed={o['id'] for o in estimates};missing=sorted(self.expected-observed-self.excluded)
        return dict(status='missing_known_objects' if missing else ('ready' if estimates else 'no_observed_objects'),estimated_objects=estimates,
                    missing_known_objects=missing,camera_id=frame.camera_id,capture_time_s=frame.capture_time_s,
                    clock_domain=frame.clock_domain,received_monotonic_s=frame.received_monotonic_s,
                    views={camera:{k:deepcopy(v[k]) for k in ('seen','identity_source','calibration_id')} for camera,v in self.views.items()},
                    support_z_m=self.support_z,assumed_padding_m=self.margin,
                    scope='Static clutter from calibrated frame/mask providers. Retained missing views are stale, not verified coverage. Estimated bounds/clearance are not true-object or hardware safety certificates.')
