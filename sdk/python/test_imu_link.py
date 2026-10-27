"""Offline tests for sdk/python/imu_link.py.

    cd sdk/python && python -m unittest -v

No serial port is touched: every case feeds bytes to the parser directly, and
the last case compares the decoder against the shared golden capture that the C
and Rust SDKs must also reproduce byte for byte.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import imu_link  # noqa: E402

TESTDATA = Path(__file__).resolve().parent.parent / "testdata"


class FrameBuilders(unittest.TestCase):
    def test_doc_examples(self):
        self.assertEqual(imu_link.feetech_ping().hex(" ").upper(), "FF FF C8 02 01 34")
        self.assertEqual(imu_link.feetech_read(56, 15).hex(" ").upper(),
                         "FF FF C8 04 02 38 0F EA")
        self.assertEqual(imu_link.dxl_ping().hex(" ").upper(),
                         "FF FF FD 00 C8 03 00 01 3B FA")
        self.assertEqual(imu_link.dxl_read(124, 12).hex(" ").upper(),
                         "FF FF FD 00 C8 07 00 02 7C 00 0C 00 AB D7")


class Decoding(unittest.TestCase):
    def test_half_precision(self):
        self.assertEqual(imu_link.half_to_float(0x0000), 0.0)
        self.assertEqual(imu_link.half_to_float(0x3C00), 1.0)
        self.assertEqual(imu_link.half_to_float(0xB800), -0.5)

    def test_known_block(self):
        # gyro -1, 509, 0 ; quaternion x = 0.5, y = z = 0
        block = bytes([0xFF, 0xFF, 0xFD, 0x01, 0, 0, 0x00, 0x38, 0, 0, 0, 0])
        gyro, quat, euler = imu_link.decode_pose(block)
        self.assertAlmostEqual(gyro[0], -1 / 16.4, places=12)
        self.assertAlmostEqual(gyro[1], 509 / 16.4, places=12)
        self.assertAlmostEqual(quat[0], 0.8660254037844386, places=12)
        self.assertAlmostEqual(quat[1], 0.5, places=12)
        self.assertAlmostEqual(euler[0], 60.0, places=9)
        self.assertAlmostEqual(euler[1], 0.0, places=12)

    def test_json_shape(self):
        sample = imu_link.Sample("feetech", 42, 0,
                                 (-0.0610, 31.0366, 0.0),
                                 (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(
            sample.to_json(),
            '{"proto":"feetech","seq":42,"status":0,"gyro_dps":[-0.0610,31.0366,0.0000],'
            '"quat_wxyz":[1.0000,0.0000,0.0000,0.0000],"euler_deg":[0.0000,0.0000,0.0000]}')


class Parsing(unittest.TestCase):
    def setUp(self):
        self.capture = (TESTDATA / "capture.bin").read_bytes()

    def test_whole_capture(self):
        parser = imu_link.Parser()
        samples = parser.feed(self.capture)
        self.assertEqual([s.proto for s in samples], ["feetech", "feetech", "dxl", "dxl"])
        self.assertEqual([s.seq for s in samples], [42, 43, None, None])

    def test_byte_at_a_time(self):
        parser = imu_link.Parser()
        samples = []
        for byte in self.capture:
            samples.extend(parser.feed(bytes([byte])))
        self.assertEqual(len(samples), 4)

    def test_corrupt_frame_is_skipped(self):
        # the capture contains one frame with a broken checksum; four samples
        # in, five frames out, is what proves the resynchronisation works
        parser = imu_link.Parser()
        self.assertEqual(len(parser.feed(self.capture)), 4)
        self.assertEqual(parser.seen, "dxl")

    def test_dxl_status_is_sticky(self):
        # address 143 is read once a second; the pose frames after it carry the
        # last value seen, which is what the shared JSON shape reports
        parser = imu_link.Parser()
        samples = parser.feed(self.capture)
        self.assertEqual(samples[2].status, 0)
        self.assertEqual(samples[3].status, 0)


class Golden(unittest.TestCase):
    def test_matches_expected_ndjson(self):
        expected = (TESTDATA / "expected.ndjson").read_text().splitlines()
        parser = imu_link.Parser()
        got = [s.to_json() for s in parser.feed((TESTDATA / "capture.bin").read_bytes())]
        self.assertEqual(got, expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
