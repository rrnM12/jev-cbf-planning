"""PNG serialization/report tests; no renderer, robot, or network required."""

import struct
from pathlib import Path
import tempfile
import unittest
import zlib

import numpy as np

from jev_cbf.legacy.static_gallery import gallery_html
from jev_cbf.reporting.images import write_apng


def chunks(path):
    raw = path.read_bytes()
    assert raw[:8] == b"\x89PNG\r\n\x1a\n"
    result = []
    position = 8
    while position < len(raw):
        size = struct.unpack(">I", raw[position:position + 4])[0]
        kind = raw[position + 4:position + 8]
        payload = raw[position + 8:position + 8 + size]
        checksum = struct.unpack(">I", raw[position + 8 + size:position + 12 + size])[0]
        assert checksum == zlib.crc32(kind + payload)
        result.append((kind, payload))
        position += size + 12
    return result


class GalleryTests(unittest.TestCase):
    def test_animation_chunks_sequence_and_pixel_data_roundtrip(self):
        frames = [np.full((3, 4, 3), color, dtype=np.uint8) for color in (0, 255)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.png"
            write_apng(path, iter(frames), frame_count=2, fps=10)
            parts = chunks(path)
        self.assertEqual([kind for kind, _ in parts], [b"IHDR", b"acTL", b"fcTL", b"IDAT", b"fcTL", b"fdAT", b"IEND"])
        self.assertEqual(struct.unpack(">II", parts[1][1]), (2, 0))
        self.assertEqual([struct.unpack(">I", parts[index][1][:4])[0] for index in (2, 4, 5)], [0, 1, 2])
        for compressed, frame in ((parts[3][1], frames[0]), (parts[5][1][4:], frames[1])):
            self.assertEqual(zlib.decompress(compressed), b"".join(b"\0" + row.tobytes() for row in frame))

    def test_invalid_frame_types_and_count_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.png"
            for frames, count in (([np.zeros((2, 2, 3))], 1), ([], 1), ([np.zeros((2, 2, 3), dtype=np.uint8)], 2)):
                with self.subTest(count=count), self.assertRaises(ValueError):
                    write_apng(path, frames, frame_count=count, fps=10)
            with self.assertRaises(ValueError):
                write_apng(path, [], frame_count=0, fps=0)

    def test_report_labels_replay_and_does_not_claim_model_or_safety_benefit(self):
        result = {
            "mode": "replay", "final_position_error_m": .01, "scene_pair_constraint_count": 168,
            "complete": True, "task_success": True,
            "collision": {"minimum_distance_m_by_object": {"object": .03}, "any_sampled_object_penetration": False},
            "control_wall_ms_after_first": {"median": .2, "p95": .3},
        }
        output = gallery_html({"runs": [result], "all_geometry": "conditional"})
        self.assertIn("not a fresh whole-arm Jev assessment", output)
        self.assertIn("elastic QP can relax", output)
        self.assertIn('src="replay.png"', output)
        self.assertIn("every", output)
        self.assertIn("10.0 mm", output)
        output = gallery_html({"runs": [result], "all_geometry": "conditional", "error": "test <failure>"})
        self.assertIn("Incomplete comparison: test &lt;failure&gt;", output)
        output = gallery_html({"runs": [result], "all_geometry": "box-split-4", "robot_aware": True})
        self.assertIn("actual Panda/whole-arm context", output)
        self.assertNotIn("Saved proxy-decision replay", output)


if __name__ == "__main__":
    unittest.main()
