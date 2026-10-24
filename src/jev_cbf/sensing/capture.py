import time
from jev_cbf.sensing.camera_io import CameraWorker, FrameRecorder
from jev_cbf.io import save

def record_live(config,output,*,frames):
    """Explicit future camera-only entry point; not invoked by offline tests/demo."""
    from jev_cbf.sensing.realsense import RealSenseSource
    recorder=FrameRecorder(output);worker=CameraWorker(lambda:RealSenseSource(config),
        lambda frame,source:(recorder.write(frame),frame)[1],frame_limit=frames).start()
    try:
        while worker.health()['published']<frames and worker.health()['status'] not in ('error','eof','stopped'):
            time.sleep(.05)
    except KeyboardInterrupt:pass
    finally:
        stopped=worker.close()
        save(output/'capture_status.json',dict(health=worker.health(),worker_stopped=stopped,
             recorded_frames=len(recorder.records),requested_frames=frames,complete=len(recorder.records)==frames,
             scope='Camera capture only; adapter/device compatibility and calibration are NOT verified by software mocks. No robot commands.'))
    if not stopped or worker.health()['status']=='error':raise RuntimeError('camera capture failed or did not stop; inspect capture_status.json')
