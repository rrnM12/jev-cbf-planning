"""RGBD-only scene inventory, color components, and provisional grasp estimates.

This deliberately small classical estimator is for saturated, separated demo
objects. It has no inventory, geom IDs, segmentation render, shape labels, native
poses or dimensions. Robot/camera calibration is idealized simulation input.
"""

from copy import deepcopy
from dataclasses import asdict
import time
import numpy as np
from jev_cbf.geometry.adaptive import FitLimits, adaptive_covers, box_belief
from jev_cbf.sensing.camera_io import CameraFrame, FrameRecorder, Intrinsics, deproject
from jev_cbf.sensing.calibration import camera_axes
from jev_cbf.io import save

def convex_footprint_cells(points, cells, side, padding_m=.003):
    """Supported convex extrusion hypothesis inferred only from observed XY.

    Keep every voxel intersecting the measured convex footprint's half-spaces.
    This is an explicit solid completion model, not a true-shape certificate.
    """
    xy = sorted(set(map(tuple, np.asarray(points)[:, :2])))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    halves = []
    for sequence in (xy, xy[::-1]):
        half = []
        for point in sequence:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    hull = np.array(halves[0]+halves[1])
    if len(hull) < 3:
        raise ValueError('observed footprint is degenerate')
    edges = np.roll(hull, -1, axis=0)-hull
    relative = cells[:, None, :2]-hull[None]
    signed = edges[None, :, 0]*relative[:, :, 1]-edges[None, :, 1]*relative[:, :, 0]
    keep = np.all(signed >= -np.linalg.norm(edges, axis=1)[None]*(np.sqrt(2)*side/2+padding_m), axis=1)
    return cells[keep]

def add_rgbd_cameras(scene, arm):
    for name, eye in [('scene_camera', [.85, -.75, .85]), ('opposite_camera', [.85, .75, .85])]:
        scene.worldbody.add_camera(name=name, pos=eye, xyaxes=camera_axes(eye, [.45, 0., .12]), fovy=58.)
    arm.body('hand').add_camera(name='wrist_camera', pos=[.055, 0., .03], xyaxes=[1., 0., 0., 0., -1., 0.], fovy=90.)

class SimulatedRGBD:
    """Plant-facing renderer; its ONLY output is calibrated RGBD frames."""
    def __init__(self, mj, model, *, width=320, height=240):
        self.mj, self.model, self.width, self.height = mj, model, width, height
        self.renderer = mj.Renderer(model, width=width, height=height)
        self.option = mj.MjvOption()
        self.option.geomgroup[3] = self.option.geomgroup[4] = 0

    def capture(self, data, *, cameras=('scene_camera', 'opposite_camera', 'wrist_camera')):
        records = []
        for name in cameras:
            index = self.model.camera(name).id
            self.renderer.update_scene(data, camera=name, scene_option=self.option)
            rgb = self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            depth = self.renderer.render().copy()
            self.renderer.disable_depth_rendering()
            focal = .5*self.height/np.tan(np.deg2rad(self.model.cam_fovy[index])/2)
            transform = np.eye(4)
            transform[:3, :3] = data.cam_xmat[index].reshape(3, 3)@np.diag([1., -1., -1.])
            transform[:3, 3] = data.cam_xpos[index]
            frame = CameraFrame(name, int(round(data.time/self.model.opt.timestep)), float(data.time), 'simulation', rgb, depth,
                                Intrinsics(self.width, self.height, focal, focal, (self.width-1)/2, (self.height-1)/2), name+'/optical',
                                metadata={'source': 'simulated RGBD; no segmentation', 'idealized_calibration': True, 'rgb_depth_registered': True})
            records.append((frame, transform))
        return records

    def close(self):
        self.renderer.close()

def components(mask, minimum=10, *, points=None, maximum_neighbor_gap_m=.025):
    """4-connected image components; no extra vision dependency."""
    remaining = mask.copy()
    height, width = mask.shape
    found = []
    for y, x in zip(*np.nonzero(mask), strict=True):
        if not remaining[y, x]:
            continue
        remaining[y, x] = False
        stack, pixels = [(int(y), int(x))], []
        while stack:
            row, col = stack.pop()
            pixels.append((row, col))
            for a, b in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
                if 0 <= a < height and 0 <= b < width and remaining[a, b]:
                    if points is not None and np.linalg.norm(points[a, b]-points[row, col]) > maximum_neighbor_gap_m:
                        continue
                    remaining[a, b] = False
                    stack.append((a, b))
        if len(pixels) >= minimum:
            found.append(np.array(pixels))
    return found

def detect(records, *, support_z=0.):
    groups, views = [], []
    for frame, transform in records:
        rgb = frame.rgb.astype(float)
        red, green, blue = np.moveaxis(rgb, -1, 0)
        saturated = (rgb.max(2)-rgb.min(2) > 35) & (rgb.max(2) > 65)
        orange = saturated & (red > .85*green) & (green > 1.15*blue) & (red > 1.25*blue)
        # The textual task supplies "orange object" only. Other saturated
        # components are clutter; colors do not encode native instance IDs.
        world = deproject(frame)@transform[:3, :3].T+transform[:3, 3]
        valid = (frame.depth_m > .05) & (frame.depth_m < 2.) & (world[:, :, 2] > support_z+.008)
        valid &= (world[:, :, 0] > .15) & (world[:, :, 0] < .85) & (abs(world[:, :, 1]) < .5)
        masks = {}
        for kind, mask in [('target', orange & valid), ('clutter', saturated & ~orange & valid)]:
            for n, pixels in enumerate(components(mask, points=world)):
                cloud = world[pixels[:, 0], pixels[:, 1]]
                if len(cloud) < 12:
                    continue
                lower, upper = cloud.min(0), cloud.max(0)
                center = (lower+upper)/2
                # Fuse components across views by observed XY overlap/proximity.
                match = next((g for g in groups if g['kind'] == kind and frame.camera_id not in g['cameras']
                              and np.linalg.norm(g['center'][:2]-center[:2]) < .065), None)
                if match is None:
                    groups.append(dict(kind=kind, clouds=[cloud], cameras={frame.camera_id}, center=center))
                else:
                    match['clouds'].append(cloud); match['cameras'].add(frame.camera_id)
                    fused = np.concatenate(match['clouds'])
                    match['center'] = (fused.min(0)+fused.max(0))/2
                image_mask = np.zeros(mask.shape, bool)
                image_mask[pixels[:, 0], pixels[:, 1]] = True
                masks[f'{kind}_{n}'] = image_mask
        views.append(dict(camera=frame.camera_id, masks=masks, capture_time_s=frame.capture_time_s))
    return [dict(kind=g['kind'], points=np.concatenate(g['clouds']), cameras=sorted(g['cameras'])) for g in groups], views

class SceneEstimator:
    def __init__(self, *, support_z=0., limits=FitLimits()):
        if not np.isfinite(support_z):
            raise ValueError('explicit calibrated support height required')
        self.support_z, self.limits = float(support_z), limits
        self.initial = None

    def initialize(self, records, output=None):
        start = time.perf_counter()
        groups, views = detect(records, support_z=self.support_z)
        targets = [g for g in groups if g['kind'] == 'target']
        if len(targets) != 1:
            raise ValueError(f'orange target is missing/ambiguous ({len(targets)} components); no oracle fill-in')
        clutter = sorted([g for g in groups if g['kind'] == 'clutter'], key=lambda g: tuple(g['points'].mean(0)))
        objects, beliefs = [], {}
        for name, group in [('observed_target', targets[0]), *[(f'observed_{i:02d}', g) for i, g in enumerate(clutter)]]:
            points = group['points']
            lower, upper = points.min(0), points.max(0)
            lower[2] = self.support_z
            # Explicit hypothesis for missing back surfaces: supported observed
            # envelope with 3 mm padding, not automatic free space or truth.
            lower[:2] -= self.limits.padding_m; upper += self.limits.padding_m
            center = (lower+upper)/2
            cells, side = box_belief(lower, upper, self.limits)
            cells = convex_footprint_cells(points, cells, side, self.limits.padding_m)
            candidates, fitting = adaptive_covers(cells, side, origin=center, limits=self.limits)
            obj = dict(id=name, shape='box', center_m=center.tolist(), size_m=(upper-lower).tolist(),
                       description='RGB color component + depth convex-footprint supported extrusion; native shape unknown.',
                       candidates=candidates, fitting=fitting, observed_points=len(points), observed_cameras=group['cameras'])
            objects.append(obj)
            beliefs[name] = dict(cells_m=cells, side_m=side, points_m=points)
        target = objects[0]
        height = target['size_m'][2]
        if not .04 <= height <= .25 or max(target['size_m'][:2]) > .075:
            raise ValueError('observed target outside this parallel-jaw prototype grasp envelope')
        self.initial = dict(status='ready', target_id='observed_target', objects=objects,
                            support_z_m=self.support_z, target_height_m=height,
                            initial_bottom_visibility_offset_m=float(targets[0]['points'][:, 2].min()-self.support_z),
                            grasp_height_m=float(targets[0]['points'][:, 2].max())-.025,
                            elapsed_s=time.perf_counter()-start, fit_limits=asdict(self.limits),
                            provenance='RGBD color components, spatial view association, supported-envelope belief; no oracle inventory/masks/shape/pose/grasp inputs.',
                            limitations='Saturated separated objects only. Missed/occluded objects remain unknown. Estimated envelope completion and upright translation tracking are provisional, not full-object coverage or calibrated sensor uncertainty.')
        self.beliefs = beliefs
        self.target_reference = beliefs['observed_target']['points_m']
        self.last_target = np.array(target['center_m'])
        self.target_top = float(self.target_reference[:, 2].max())
        if output is not None:
            recorder = FrameRecorder(output/'recording')
            calibration = {}
            for (frame, transform), view in zip(records, views, strict=True):
                recorder.write(frame, view['masks'], identity_source='Classical RGB color connected components, NOT simulator segmentation; view-local IDs')
                calibration[frame.camera_id] = dict(mount='fixed', optical_frame=frame.optical_frame,
                    T_robot_optical=transform.tolist(), calibration_id='IDEALIZED simulation capture calibration',
                    valid_capture_time_s=frame.capture_time_s, clock_domain=frame.clock_domain)
            save(output/'calibration.json', calibration)
            save(output/'estimate.json', self.initial)
            np.savez_compressed(output/'beliefs.npz', **{name+'_'+key: value for name, belief in beliefs.items() for key, value in belief.items()})
        return deepcopy(self.initial)

    def track_target(self, records, *, supported=False):
        groups, _ = detect(records, support_z=self.support_z)
        targets = [g for g in groups if g['kind'] == 'target']
        at = records[0][0].capture_time_s
        if len(targets) != 1:
            return dict(status='missing_or_ambiguous', capture_time_s=at, center_m=self.last_target.tolist(), fresh=False)
        points = targets[0]['points']
        initial_center = np.array(self.initial['objects'][0]['center_m'])
        # Top height is a direct observed quantity, with a frozen initial height
        # hypothesis for the unseen bottom. Keep that distinction in reports.
        lower, upper = points.min(0), points.max(0)
        center = (lower+upper)/2
        center[2] = (lower[2]+upper[2])/2
        if supported:
            center[2] = initial_center[2]
        velocity = (center-self.last_target)/max(at-getattr(self, 'last_at', 0.), .001)
        self.last_target, self.last_at = center, at
        # Correct the visible lower extent using the initial supported-view
        # visibility gap and padding. This is an estimator, not a lower bound on
        # hidden true geometry. Multi-view native evaluation can falsify it.
        bottom = self.support_z if supported else lower[2]-self.initial['initial_bottom_visibility_offset_m']-self.limits.padding_m
        return dict(status='observed', capture_time_s=at, center_m=center.tolist(), velocity_m_s=velocity.tolist(),
                    bottom_m=float(bottom), fresh=True, observed_lower_m=lower.tolist(), observed_upper_m=upper.tolist(),
                    observed_points=len(points), cameras=targets[0]['cameras'],
                    scope='Visible-bounds center, orientation unknown; bottom from visible lower extent minus initial visibility offset/padding, not hidden native pose or certified full-object bounds.')
