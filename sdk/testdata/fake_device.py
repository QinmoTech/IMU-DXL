#!/usr/bin/env python3
"""Emulate the IMU-DXL device on a Linux PTY, to exercise the live serial path.

    python fake_device.py --protocol feetech
    /dev/pts/5                     <- first line on stdout; pass it to read_imu

Frames are built here rather than by the SDK, so an SDK bug cannot hide behind a
shared builder. The pose it serves is the first sample of testdata/capture.bin,
so the expected JSON line is testdata/expected.ndjson line 1 (feetech) or 3
(dxl). Needs a POSIX pty, i.e. Linux/macOS, not Windows.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys

ID = 200
POSE = struct.pack("<hhhHHH", -1, 509, 0, 0x3800, 0, 0)
STATUS_BYTE = 0x00


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
    frame = b"\xff\xff\xfd\x00" + bytes([ID]) + struct.pack("<H", len(stuffed) + 2) + bytes(stuffed)
    return frame + struct.pack("<H", dxl_crc(frame))


def answer_feetech(addr: int, length: int) -> bytes:
    if addr == 56:
        return feetech_frame(POSE + bytes([42, 0, 0]))
    return feetech_frame(bytes(length))


def answer_dxl(addr: int, length: int) -> bytes:
    if addr == 124:
        return dxl_frame(POSE)
    if addr == 143:
        return dxl_frame(bytes([STATUS_BYTE]))
    return dxl_frame(bytes(length))


def take_feetech(buf: bytes):
    """Return (reply or None, consumed) for one request, or (None, 0) if incomplete."""
    start = buf.find(b"\xff\xff")
    if start < 0:
        return None, max(0, len(buf) - 1)
    if start:
        return None, start
    if len(buf) < 4:
        return None, 0
    total = buf[3] + 4
    if len(buf) < total:
        return None, 0
    frame, rest = buf[:total], total
    instruction = frame[4]
    if instruction == 0x01:
        return feetech_frame(b""), rest
    if instruction == 0x02:
        return answer_feetech(frame[5], frame[6]), rest
    return None, rest


def take_dxl(buf: bytes):
    start = buf.find(b"\xff\xff\xfd\x00")
    if start < 0:
        return None, max(0, len(buf) - 3)
    if start:
        return None, start
    if len(buf) < 7:
        return None, 0
    total = 7 + (buf[5] | (buf[6] << 8))
    if len(buf) < total:
        return None, 0
    body = buf[7:total - 2]
    instruction = body[0]
    if instruction == 0x01:
        return dxl_frame(struct.pack("<H", 1030) + bytes([3])), total
    if instruction == 0x02:
        addr = body[1] | (body[2] << 8) if len(body) >= 3 else 0
        length = body[3] | (body[4] << 8) if len(body) >= 5 else 0
        return answer_dxl(addr, length), total
    return None, total


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--protocol", choices=("feetech", "dxl"), required=True)
    args = ap.parse_args()

    try:
        import pty
    except ImportError:
        print("fake_device needs a POSIX pty: run it on Linux or macOS", file=sys.stderr)
        return 2

    master, slave = pty.openpty()
    print(os.ttyname(slave), flush=True)

    take = take_feetech if args.protocol == "feetech" else take_dxl
    buf = b""
    while True:
        ready, _, _ = select.select([master], [], [], 0.5)
        if not ready:
            continue
        try:
            chunk = os.read(master, 256)
        except OSError:
            # the reader closed its end (or a slave is not open yet): keep serving,
            # a new slave can attach later and the buffer is meaningless now
            buf = b""
            continue
        if not chunk:
            buf = b""
            continue
        buf += chunk
        while True:
            reply, consumed = take(buf)
            if not consumed:
                break
            buf = buf[consumed:]
            if reply:
                try:
                    os.write(master, reply)
                except OSError:
                    break


if __name__ == "__main__":
    sys.exit(main())
