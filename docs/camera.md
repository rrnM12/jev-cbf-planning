# Camera and replay boundary

Use `uv run jev-cbf inspect camera --help` for the current commands. Replay is
explicitly offline; live capture is a separate hardware-unverified camera-only
command. No physical robot command is implemented or validated here.

```bash
uv run jev-cbf inspect camera replay --recording artifacts/camera/RECORDING --output artifacts/camera/INSPECTION --calibration artifacts/camera/CAPTURE/calibration.json
```

The example is a schema/scaffolding reference, not calibrated hardware settings.
RealSense SDK import is lazy in `sensing/realsense.py`. No SDK/driver installation,
connection or hardware test was performed during the reorganization.

`CameraFrame` carries metric depth, intrinsics, capture timestamp, clock domain,
frame ID and transform provenance. Raw frames and externally supplied masks are
recorded separately; raw capture does not fabricate detections. Fixed-camera
extrinsics or capture-time wrist transforms need explicit calibration. Distortion
must have its actual deprojection model; pinhole guessing is rejected.

`CameraWorker` separates acquisition, optional geometry processing and recording
from the latest nonblocking snapshot consumed by a simulated controller. EOF,
disconnects, missing masks, calibration and age are explicit health states.
Simulator GL capture stays on its owning thread. The earlier oracle mask demo is
isolated in `legacy/camera_demo.py`; it is not the current RGBD detector.

Missing physical calibration, device QA, learned/general perception, timestamp
synchronization and a validated physical safe-stop policy remain unimplemented.
See [architecture](architecture.md) for the observation/native-truth boundary.
