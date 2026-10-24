import numpy as np
import struct
import zlib

def depth_image(depth):
    valid=np.isfinite(depth)&(depth>0)
    maximum=float(np.percentile(depth[valid],95)) if valid.any() else 1.
    scaled=np.clip(np.nan_to_num(depth,nan=0.,posinf=0.)/max(maximum,1e-6),0.,1.)
    rgb=np.stack([1-scaled,1-abs(2*scaled-1),scaled],axis=-1)
    rgb[~valid]=0
    return (rgb*255).astype(np.uint8)

def _chunk(kind, payload):
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))

def write_apng(path, frames, *, frame_count, fps):
    """Stream RGB frames as standard animated PNG, without a video dependency."""
    if frame_count < 1 or not 1 <= fps <= 65535:
        raise ValueError("need frames and an integer PNG frame rate in [1, 65535]")
    sequence = 0
    dimensions = None
    written = 0
    with path.open("wb") as stream:
        stream.write(b"\x89PNG\r\n\x1a\n")
        for index, frame in enumerate(frames):
            frame = np.asarray(frame)
            if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
                raise ValueError("frames must be uint8 RGB arrays")
            height, width, _ = frame.shape
            if index == 0:
                dimensions = (height, width)
                stream.write(_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
                stream.write(_chunk(b"acTL", struct.pack(">II", frame_count, 0)))
            elif dimensions != (height, width):
                raise ValueError("animation frame dimensions changed")
            stream.write(_chunk(b"fcTL", struct.pack(">IIIIIHHBB", sequence, width, height, 0, 0, 1, fps, 0, 0)))
            sequence += 1
            pixels = b"".join(b"\0" + row.tobytes() for row in frame)
            compressed = zlib.compress(pixels, level=3)
            if index == 0:
                stream.write(_chunk(b"IDAT", compressed))
            else:
                stream.write(_chunk(b"fdAT", struct.pack(">I", sequence) + compressed))
                sequence += 1
            written += 1
        if written != frame_count:
            raise ValueError(f"expected {frame_count} frames, received {written}")
        stream.write(_chunk(b"IEND", b""))
