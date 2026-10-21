"""RGBD frames, explicit optical coordinates, portable recordings and worker I/O.

No camera SDK, MuJoCo, robot driver or model service is imported here. Sources
implement read(timeout_s) -> CameraFrame | None (timeout), raising StopIteration
at EOF, and close(). Acquisition/processing/recording belongs to the owning worker.
"""

from dataclasses import asdict, dataclass, field
import hashlib
import json
from pathlib import Path
from threading import Event, Lock, Thread
import time

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: str = 'none'
    coefficients: tuple = ()

    def __post_init__(self):
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or self.width <= 0 or self.height <= 0
                or not np.isfinite([self.fx, self.fy, self.cx, self.cy, *self.coefficients]).all()
                or min(self.fx, self.fy) <= 0):
            raise ValueError('invalid camera intrinsics')
        object.__setattr__(self, 'coefficients', tuple(self.coefficients))


def readonly(value, dtype=None):
    array = np.array(value, dtype=dtype, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    frame_number: int
    capture_time_s: float
    clock_domain: str
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: Intrinsics
    optical_frame: str
    # This is receipt time, NOT exposure time in the robot clock.
    received_monotonic_s: float = field(default_factory=time.monotonic)
    points_optical_m: np.ndarray | None = None
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        if (not self.camera_id or not self.optical_frame or not self.clock_domain
                or not isinstance(self.frame_number, int) or self.frame_number < 0
                or not np.isfinite([self.capture_time_s, self.received_monotonic_s]).all()):
            raise ValueError('invalid frame identity/timestamps')
        rgb, depth = readonly(self.rgb), readonly(self.depth_m, float)
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('RGB must be HxWx3 uint8, RGB channel order')
        if depth.shape != (self.intrinsics.height, self.intrinsics.width):
            raise ValueError('depth dimensions do not match depth intrinsics')
        object.__setattr__(self, 'rgb', rgb)
        object.__setattr__(self, 'depth_m', depth)
        if self.points_optical_m is not None:
            points = readonly(self.points_optical_m, float)
            if points.shape != (*depth.shape, 3):
                raise ValueError('organized point cloud must match depth pixels')
            valid = np.isfinite(depth) & (depth > 0)
            if not np.isfinite(points[valid]).all() or not np.allclose(points[..., 2][valid], depth[valid], atol=1e-5, rtol=1e-4):
                raise ValueError('point cloud is not consistent with optical-axis metric depth')
            object.__setattr__(self, 'points_optical_m', points)
        # Metadata must be portable, independent of mutable SDK buffers.
        object.__setattr__(self, 'metadata', json.loads(json.dumps(self.metadata, allow_nan=False)))


def deproject(frame):
    """Organized points: optical +X right, +Y down, +Z forward; metres.

    SDK-derived points preserve non-pinhole calibration during SDK-free replay.
    Unsupported distorted intrinsics never silently fall back to ideal pinhole.
    """
    depth, intr = frame.depth_m, frame.intrinsics
    if frame.points_optical_m is not None:
        points = np.array(frame.points_optical_m)
    else:
        if (intr.distortion not in ('none', 'brown_conrady', 'inverse_brown_conrady', 'modified_brown_conrady')
                or any(c != 0 for c in intr.coefficients)):
            raise ValueError('distorted stream needs SDK-derived points or explicit rectification')
        row, col = np.indices(depth.shape)
        points = np.stack([(col-intr.cx)*depth/intr.fx, (row-intr.cy)*depth/intr.fy, depth], axis=-1)
    points[~(np.isfinite(depth) & (depth > 0))] = np.nan
    return points


def rigid_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-7, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-7, rtol=0)):
        raise ValueError('need a finite rigid source-to-destination transform, metres')
    return matrix.copy()


def robot_transform(frame, calibration, *, pose_provider=None):
    """Resolve T_robot_optical. Never assume identity for missing calibration.

    Wrist pose_provider(frame) returns T_robot_ee at the SAME timestamp/domain.
    Clock conversion/interpolation belongs to that provider, not a guessed offset.
    """
    entry = calibration.get(frame.camera_id)
    if not entry or entry.get('optical_frame') != frame.optical_frame or not entry.get('calibration_id'):
        raise ValueError('missing matching optical-frame calibration and provenance')
    if entry.get('device_serial') is not None and entry['device_serial'] != frame.metadata.get('serial'):
        raise ValueError('calibration device serial does not match the frame producer')
    if 'valid_capture_time_s' in entry and (entry['valid_capture_time_s'] != frame.capture_time_s
            or entry.get('clock_domain') != frame.clock_domain):
        raise ValueError('capture-only fixture calibration is not valid for this frame')
    if entry.get('mount') == 'fixed':
        return rigid_transform(entry.get('T_robot_optical'))
    if entry.get('mount') != 'wrist' or pose_provider is None:
        raise ValueError('wrist camera needs calibrated offset and capture-time robot pose provider')
    pose = pose_provider(frame)
    if pose['clock_domain'] != frame.clock_domain or pose['capture_time_s'] != frame.capture_time_s:
        raise ValueError('robot pose does not match camera capture time/clock domain')
    return rigid_transform(pose['T_robot_ee'])@rigid_transform(entry.get('T_ee_optical'))


def masked_clouds(frame, masks, transform):
    points = deproject(frame)
    matrix = rigid_transform(transform)
    world = points@matrix[:3, :3].T+matrix[:3, 3]
    valid = np.isfinite(world).all(axis=-1)
    clouds = {}
    for name, mask in masks.items():
        mask = np.asarray(mask)
        if not isinstance(name, str) or not name or mask.dtype != bool or mask.shape != frame.depth_m.shape:
            raise ValueError('object masks must be named boolean arrays on the depth pixel grid')
        clouds[name] = world[mask & valid]
    return clouds


class FrameRecorder:
    """One writer; append-only frame files + hashes. No SDK-specific bag required."""

    def __init__(self, folder):
        self.folder = Path(folder)
        if self.folder.exists() and any(self.folder.iterdir()):
            raise ValueError('recording needs a new/empty directory')
        self.folder.mkdir(parents=True, exist_ok=True)
        self.records = []

    def write(self, frame, masks=None, *, identity_source=None):
        if masks is not None and not identity_source:
            raise ValueError('recorded masks need explicit identity provenance')
        arrays = {'rgb': frame.rgb, 'depth_m': frame.depth_m}
        if frame.points_optical_m is not None:
            arrays['points_optical_m'] = frame.points_optical_m
        names = []
        for name, mask in (masks or {}).items():
            if np.asarray(mask).dtype != bool or np.shape(mask) != frame.depth_m.shape:
                raise ValueError('recorded mask must use the depth pixel grid')
            names.append(name); arrays[f'mask_{len(names)-1}'] = mask
        metadata = dict(camera_id=frame.camera_id, frame_number=frame.frame_number,
                        capture_time_s=frame.capture_time_s, clock_domain=frame.clock_domain,
                        received_monotonic_s=frame.received_monotonic_s, optical_frame=frame.optical_frame,
                        intrinsics=asdict(frame.intrinsics), metadata=frame.metadata,
                        mask_names=names, identity_source=identity_source)
        stem = f'{len(self.records):06d}'
        path, meta = self.folder/(stem+'.npz'), self.folder/(stem+'.json')
        if path.exists() or meta.exists():
            raise ValueError('refusing to overwrite an existing frame')
        np.savez_compressed(path, **arrays)
        meta.write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
        self.records.append(dict(data=path.name, metadata=meta.name,
                                 data_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                 metadata_sha256=hashlib.sha256(meta.read_bytes()).hexdigest()))
        self.close()

    def close(self):
        (self.folder/'recording.json').write_text(json.dumps(dict(version=1, records=self.records), indent=2)+'\n')


class ReplaySource:
    """Read recorded frames; host receipt time is refreshed, capture clock is not."""

    def __init__(self, folder):
        self.folder = Path(folder)
        manifest = json.loads((self.folder/'recording.json').read_text())
        if manifest['version'] != 1:
            raise ValueError('unsupported recording version')
        self.records = iter(manifest['records']); self.last = None
        self.closed = False

    def read(self, timeout_s):
        if self.closed:
            raise RuntimeError('replay closed')
        record = next(self.records)
        paths = []
        for key in ('data', 'metadata'):
            path = self.folder/record[key]
            if (path.parent != self.folder or path.is_symlink() or path.resolve().parent != self.folder.resolve()
                    or hashlib.sha256(path.read_bytes()).hexdigest() != record[key+'_sha256']):
                raise ValueError('recording path/hash mismatch')
            paths.append(path)
        metadata = json.loads(paths[1].read_text())
        with np.load(paths[0], allow_pickle=False) as arrays:
            frame = CameraFrame(**{k: metadata[k] for k in ('camera_id', 'frame_number', 'capture_time_s', 'clock_domain', 'optical_frame')},
                                rgb=arrays['rgb'], depth_m=arrays['depth_m'], intrinsics=Intrinsics(**metadata['intrinsics']),
                                points_optical_m=arrays['points_optical_m'] if 'points_optical_m' in arrays else None,
                                metadata=dict(metadata['metadata'], recorded_receipt_monotonic_s=metadata['received_monotonic_s'], replay=True))
            masks = {name: readonly(arrays[f'mask_{i}']) for i, name in enumerate(metadata['mask_names'])}
        self.last = (frame, masks, metadata['identity_source'])
        return frame

    def annotations(self, frame):
        if self.last is None or self.last[0] is not frame or self.last[2] is None:
            raise ValueError('no recorded masks/provenance for this exact frame')
        return self.last[1], self.last[2]

    def close(self):
        self.closed = True


@dataclass(frozen=True)
class Snapshot:
    sequence: int
    value: object
    published_monotonic_s: float


class CameraWorker:
    """Source factory/process/recording run off the consumer's control thread.

    latest() only reads a completed snapshot, never waits for camera I/O. One
    slot bounds consumer backlog; this is not a real-time or safe-fallback claim.
    The process callback may perform masks, calibration, fitting and recording.
    """

    def __init__(self, source_factory, process=None, *, read_timeout_s=.1, frame_limit=None):
        if not np.isfinite(read_timeout_s) or read_timeout_s <= 0:
            raise ValueError('read timeout must be finite and positive')
        self.factory, self.process, self.timeout = source_factory, process, read_timeout_s
        if frame_limit is not None and (type(frame_limit) is not int or frame_limit<=0):
            raise ValueError('frame limit must be a positive integer or None')
        self.frame_limit=frame_limit
        self.lock, self.stop_event = Lock(), Event()
        self.snapshot = None; self.state = {'status': 'not_started', 'published': 0}
        self.thread = Thread(target=self._run, name='camera-input', daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _status(self, **values):
        with self.lock:
            self.state = dict(self.state, **values)

    def _run(self):
        source = None
        try:
            self._status(status='starting')
            source = self.factory()
            self._status(status='running')
            while not self.stop_event.is_set():
                frame = source.read(self.timeout)
                if self.stop_event.is_set():
                    break
                if frame is None:
                    self._status(status='waiting_for_frame')
                    self.stop_event.wait(min(self.timeout, .01))
                    continue
                value = self.process(frame, source) if self.process else frame
                if self.stop_event.is_set():
                    break
                with self.lock:
                    n = self.state['published']+1
                    self.snapshot = Snapshot(n, value, time.monotonic())
                    self.state = dict(self.state, status='running', published=n)
                if self.frame_limit is not None and n>=self.frame_limit:
                    self._status(status='eof')
                    break
        except StopIteration:
            self._status(status='eof')
        except Exception as error:
            self._status(status='error', error_type=type(error).__name__, error=str(error))
        finally:
            if source is not None:
                try:
                    source.close()
                except Exception as error:
                    self._status(status='error', error_type=type(error).__name__, error=str(error))
            if self.stop_event.is_set() and self.health()['status'] != 'error':
                self._status(status='stopped')

    def latest(self):
        with self.lock:
            return self.snapshot

    def health(self):
        with self.lock:
            state = self.state.copy()
            snapshot = self.snapshot
        state['snapshot_age_host_s'] = None if snapshot is None else time.monotonic()-snapshot.published_monotonic_s
        return state

    def close(self, timeout_s=1.):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout_s)
        return not self.thread.is_alive()  # False means an unresponsive source, not a safe stop.
