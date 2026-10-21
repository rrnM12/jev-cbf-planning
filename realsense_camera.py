"""Optional RealSense SDK boundary. Hardware/SDK build compatibility UNVERIFIED.

Importing this module does not import the SDK, enumerate devices or start capture.
Only constructing RealSenseSource does that. SDK transport is mock-injectable.
API references: librealsense's align-depth2color.py, opencv_pointcloud_viewer.py,
and https://realsenseai.github.io/librealsense/python_docs/ .
"""

from dataclasses import asdict
import importlib
import time

import numpy as np

from camera_io import CameraFrame, Intrinsics, rigid_transform


class RealSenseSource:
    def __init__(self, config, *, sdk=None):
        if sdk is None:
            try:
                sdk = importlib.import_module('pyrealsense2')
            except ImportError as error:
                raise RuntimeError('RealSense SDK is optional and not installed on this host; use replay. Configure/verify it on the actual camera host.') from error
        self.rs = sdk
        self.pipeline = None; self.started = False
        context = sdk.context()
        devices = list(context.query_devices())
        serials = [d.get_info(sdk.camera_info.serial_number) for d in devices]
        serial = config.get('serial')
        if serial is None:
            if len(serials) != 1:
                raise ValueError('select a camera serial explicitly when zero or multiple devices are present')
            serial = serials[0]
        if serial not in serials:
            raise ValueError('configured RealSense serial was not found')
        self.camera_id = config.get('camera_id', serial)
        if not isinstance(self.camera_id,str) or not self.camera_id:
            raise ValueError('camera_id must be a nonempty logical name')
        self.align_color = config.get('align_depth_to_color', True)
        if not isinstance(self.align_color, bool):
            raise ValueError('align_depth_to_color must be boolean')
        options = sdk.config(); options.enable_device(serial)
        options.disable_all_streams()
        for name, kind, pixel_format in (('depth', sdk.stream.depth, sdk.format.z16),
                                         ('color', sdk.stream.color, sdk.format.rgb8)):
            profile = config.get(name, {})
            width, height, fps = (profile.get(k, 0) for k in ('width', 'height', 'fps'))
            if (any(type(v) is not int or v < 0 for v in (width, height, fps))
                    or bool(width) != bool(height)):
                raise ValueError('stream width/height must be paired; dimensions/fps are nonnegative integers (0 lets SDK resolve)')
            options.enable_stream(kind, width, height, pixel_format, fps)
        self.pipeline = sdk.pipeline(context)
        try:
            profile = self.pipeline.start(options); self.started = True
            self.depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
            if not np.isfinite(self.depth_scale) or self.depth_scale <= 0:
                raise ValueError('device returned invalid metres-per-depth-unit')
            self.aligner = sdk.align(sdk.stream.color) if self.align_color else None
            self.pointcloud = sdk.pointcloud()
            self.serial = serial
        except Exception:
            self.close()
            raise

    @staticmethod
    def _intrinsics(video):
        intr = video.profile.as_video_stream_profile().get_intrinsics()
        return Intrinsics(int(intr.width), int(intr.height), float(intr.fx), float(intr.fy),
                          float(intr.ppx), float(intr.ppy), str(intr.model).split('.')[-1], tuple(intr.coeffs))

    def read(self, timeout_s):
        if not self.started:
            raise RuntimeError('camera source closed')
        if not np.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('read timeout must be finite and positive')
        available, frames = self.pipeline.try_wait_for_frames(max(1, int(timeout_s*1000)))
        if not available:
            return None
        if self.aligner is not None:
            frames = self.aligner.process(frames)
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            return None
        raw = np.asanyarray(depth.get_data())
        if raw.dtype != np.uint16:
            raise ValueError('expected requested Z16 depth, not an unknown depth/disparity format')
        # Query the profile AFTER alignment; never reuse the unaligned intrinsics.
        intr = self._intrinsics(depth)
        points = self.pointcloud.calculate(depth)
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(intr.height, intr.width, 3)
        color_intr = self._intrinsics(color)
        extrinsics=depth.profile.get_extrinsics_to(color.profile)
        color_from_depth=np.eye(4)
        # SDK storage is column-major; our JSON transforms are ordinary rows.
        color_from_depth[:3,:3]=np.asarray(extrinsics.rotation).reshape(3,3,order='F')
        color_from_depth[:3,3]=extrinsics.translation
        color_from_depth=rigid_transform(color_from_depth)
        optical = self.camera_id+('/color_optical' if self.align_color else '/depth_optical')
        return CameraFrame(camera_id=self.camera_id, frame_number=int(depth.get_frame_number()),
                           capture_time_s=float(depth.get_timestamp())/1000.,
                           clock_domain=str(depth.get_frame_timestamp_domain()),
                           rgb=np.asarray(color.get_data()), depth_m=raw.astype(float)*self.depth_scale,
                           intrinsics=intr, optical_frame=optical, received_monotonic_s=time.monotonic(),
                           points_optical_m=vertices,
                           metadata=dict(source='realsense_sdk_hardware_unverified', serial=self.serial,
                                         depth_scale_m_per_unit=self.depth_scale, aligned_depth_to_color=self.align_color,
                                         rgb_depth_registered=self.align_color,
                                         color_intrinsics=asdict(color_intr),
                                         T_color_depth=color_from_depth.tolist(),
                                         color_capture_time_s=float(color.get_timestamp())/1000.,
                                         color_clock_domain=str(color.get_frame_timestamp_domain()),
                                         timestamp_source='SDK get_timestamp; physical exposure/robot-clock alignment unverified',
                                         synchronization_verified=False,
                                         rgb_masks_require_depth_grid_alignment=True))

    def close(self):
        if self.started:
            self.started = False
            self.pipeline.stop()
