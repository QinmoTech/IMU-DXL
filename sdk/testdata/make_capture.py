#!/usr/bin/env python3
"""Build testdata/capture.bin — a synthetic bus capture.

The frame builders here are deliberately independent of sdk/python/imu_link.py:
if both shared a bug, the SDK tests would still pass. Checksums and the CRC are
spelled out from docs/PROTOCOL.md.

The capture mixes both protocols, byte noise before a frame, and one corrupted
frame, so a decoder has to resynchronise rather than trust the first bytes.
"""

from __future__ import annotations

import struct
from pathlib import Path

ID = 200
OUT = Path(__file__).resolve().parent / "capture.bin"


def feetech_frame(data: bytes, error: int = 0) -> bytes:
    body = bytes([ID, len(data) + 2, error]) + data
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def dxl_crc(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def dxl_frame(data: bytes, error: int = 0) -> bytes:
    body = bytes([0x55, error]) + data
    stuffed = bytearray()
    for byte in body:
        stuffed.append(byte)
        if len(stuffed) >= 3 and stuffed[-3:] == b"\xff\xff\xfd":
            stuffed.append(0xFD)
    frame = (b"\xff\xff\xfd\x00" + bytes([ID]) +
             struct.pack("<H", len(stuffed) + 2) + bytes(stuffed))
    return frame + struct.pack("<H", dxl_crc(frame))


def pose(gyro, quat_half) -> bytes:
    """gyro: 3 ints (raw counts); quat_half: 3 uint16 (binary16 x,y,z)."""
    return struct.pack("<hhhHHH", *gyro, *quat_half)


# half-precision literals used below: 0x3800 = 0.5, 0x0000 = 0
FT1 = pose((-1, 509, 0), (0x3800, 0, 0)) + bytes([42, 0, 0])
FT2 = pose((1640, 0, 0), (0, 0, 0)) + bytes([43, 1, 0])
DX1 = pose((-1, 509, 0), (0x3800, 0, 0))
DX2 = pose((0, 0, 0), (0, 0x3800, 0))

capture = bytearray()
capture += feetech_frame(FT1)                  # sample 1
capture += b"\x00\x00"                          # line noise
capture += feetech_frame(FT2)                  # sample 2 (status bit0 = 1)
bad = bytearray(feetech_frame(b"\x00" * 15))    # corrupted checksum, must be skipped
bad[-1] ^= 0xFF
capture += bad
capture += dxl_frame(b"\x00")                  # DYNAMIXEL status byte (143)
capture += dxl_frame(DX1)                      # sample 3
capture += dxl_frame(DX2)                      # sample 4

OUT.write_bytes(bytes(capture))
print(f"wrote {OUT} ({len(capture)} bytes)")
