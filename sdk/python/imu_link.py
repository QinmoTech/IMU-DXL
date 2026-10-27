"""IMU-DXL servo-bus reader: framing, decoding, and a tiny serial helper.

The device answers as a read-only virtual servo, ID 200, on a 1 Mbps 8N1
single-wire bus. Two wire formats reach the same 12-byte core block:

    Feetech SCS/STS : FF FF ID LEN INSTRUCTION|ERROR DATA... CHECKSUM
                      pose block at address 56, 15 bytes
    DYNAMIXEL 2.0   : FF FF FD 00 ID LEN_L LEN_H ... CRC_L CRC_H
                      pose block at address 124, 12 bytes
                      status byte at address 143, read separately

    0..5   int16 LE x3      gyro raw counts, /16.4 = deg/s
    6..11  binary16 LE x3   quaternion x, y, z; w = sqrt(max(0, 1-x^2-y^2-z^2))
    (Feetech only) 12 = sample counter, 13 = status, 14 = reserved (0)

The parser finds frame boundaries by itself, so a capture that mixes both
formats decodes without being told which one it is.

This module has no third-party dependency; only `open_serial()` imports
pyserial, and only when a port is actually opened.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

ID = 200
BAUD = 1_000_000

FEETECH_POSE_ADDR, FEETECH_POSE_LEN = 56, 15
DXL_POSE_ADDR, DXL_POSE_LEN = 124, 12
DXL_STATUS_ADDR, DXL_STATUS_LEN = 143, 1

GYRO_LSB_PER_DPS = 16.4
DXL_HEADER = b"\xff\xff\xfd\x00"


# --------------------------------------------------------------------------- #
# frame construction
# --------------------------------------------------------------------------- #

def feetech_checksum(payload: bytes) -> int:
    return (~sum(payload)) & 0xFF


def feetech_packet(instruction: int, params: bytes = b"") -> bytes:
    body = bytes([ID, len(params) + 2, instruction]) + params
    return b"\xff\xff" + body + bytes([feetech_checksum(body)])


def feetech_ping() -> bytes:
    return feetech_packet(0x01)


def feetech_read(addr: int, length: int) -> bytes:
    return feetech_packet(0x02, bytes([addr & 0xFF, length & 0xFF]))


def dxl_crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def dxl_stuff(body: bytes) -> bytes:
    out = bytearray()
    for byte in body:
        out.append(byte)
        if len(out) >= 3 and out[-3:] == b"\xff\xff\xfd":
            out.append(0xFD)
    return bytes(out)


def dxl_unstuff(body: bytes) -> bytes:
    out = bytearray()
    for byte in body:
        if len(out) >= 3 and byte == 0xFD and out[-3:] == b"\xff\xff\xfd":
            continue
        out.append(byte)
    return bytes(out)


def dxl_packet(instruction: int, params: bytes = b"") -> bytes:
    body = dxl_stuff(bytes([instruction]) + params)
    frame = DXL_HEADER + bytes([ID, (len(body) + 2) & 0xFF, (len(body) + 2) >> 8]) + body
    crc = dxl_crc16(frame)
    return frame + bytes([crc & 0xFF, crc >> 8])


def dxl_ping() -> bytes:
    return dxl_packet(0x01)


def dxl_read(addr: int, length: int) -> bytes:
    return dxl_packet(0x02, bytes([addr & 0xFF, addr >> 8, length & 0xFF, length >> 8]))


# --------------------------------------------------------------------------- #
# decoding
# --------------------------------------------------------------------------- #

def half_to_float(bits: int) -> float:
    sign = -1.0 if bits & 0x8000 else 1.0
    exp = (bits >> 10) & 0x1F
    frac = bits & 0x3FF
    if exp == 0:
        return sign * frac * 2.0 ** -24
    if exp == 0x1F:
        return math.inf if frac == 0 else math.nan
    return sign * (1.0 + frac / 1024.0) * 2.0 ** (exp - 15)


@dataclass
class Sample:
    proto: str                  # "feetech" or "dxl"
    seq: int | None             # Feetech counter; None on DYNAMIXEL
    status: int | None          # Feetech status byte / DYNAMIXEL address 143
    gyro_dps: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    euler_deg: tuple[float, float, float]

    def to_json(self) -> str:
        def f(x: float) -> str:
            if x == 0.0:
                x = 0.0                     # normalise -0.0
            return f"{x:.4f}"
        seq = "null" if self.seq is None else str(self.seq)
        status = "null" if self.status is None else str(self.status)
        return (
            f'{{"proto":"{self.proto}","seq":{seq},"status":{status},'
            f'"gyro_dps":[{",".join(f(v) for v in self.gyro_dps)}],'
            f'"quat_wxyz":[{",".join(f(v) for v in self.quat_wxyz)}],'
            f'"euler_deg":[{",".join(f(v) for v in self.euler_deg)}]}}'
        )


def decode_pose(block: bytes) -> tuple[tuple[float, float, float],
                                       tuple[float, float, float, float],
                                       tuple[float, float, float]]:
    """12-byte core block -> (gyro deg/s, quaternion wxyz, ZYX euler deg)."""
    if len(block) < 12:
        raise ValueError("pose block needs 12 bytes")
    gyro = tuple(
        int.from_bytes(block[i:i + 2], "little", signed=True) / GYRO_LSB_PER_DPS
        for i in (0, 2, 4)
    )
    x = half_to_float(int.from_bytes(block[6:8], "little"))
    y = half_to_float(int.from_bytes(block[8:10], "little"))
    z = half_to_float(int.from_bytes(block[10:12], "little"))
    w = math.sqrt(max(0.0, 1.0 - x * x - y * y - z * z))
    n = math.hypot(w, x, y, z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n

    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    euler = tuple(math.degrees(v) for v in (roll, pitch, yaw))
    return gyro, (w, x, y, z), euler


# --------------------------------------------------------------------------- #
# stream parser
# --------------------------------------------------------------------------- #

class Parser:
    """Feed bytes, get samples. Frame type is detected per frame."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.status: int | None = None
        self.seen: str | None = None       # last valid frame's format

    def feed(self, data: bytes):
        self.buf += data
        samples = []
        while True:
            result = self._parse_one()
            if result is None:
                break
            if isinstance(result, Sample):
                samples.append(result)
        return samples

    def _parse_one(self):
        buf = self.buf
        if len(buf) < 2:
            return None
        start = buf.find(b"\xff\xff")
        if start < 0:
            del buf[:max(0, len(buf) - 1)]
            return None
        if start:
            del buf[:start]
        if len(buf) < 4:
            return None
        if len(buf) >= 4 and bytes(buf[:4]) == DXL_HEADER:
            return self._dxl()
        return self._feetech()

    def _feetech(self):
        buf = self.buf
        total = buf[3] + 4
        if total < 6 or total > 140:
            del buf[:1]
            return False
        if len(buf) < total:
            return None
        frame = bytes(buf[:total])
        if (sum(frame[2:]) & 0xFF) != 0xFF:      # checksum makes the sum 0xFF
            del buf[:1]
            return False
        del buf[:total]
        if frame[2] != ID:
            return False
        self.seen = "feetech"
        data = frame[5:-1]
        if len(data) == 15:
            gyro, quat, euler = decode_pose(data)
            return Sample("feetech", data[12], data[13], gyro, quat, euler)
        return False

    def _dxl(self):
        buf = self.buf
        if len(buf) < 7:
            return None
        total = 7 + (buf[5] | (buf[6] << 8))
        if total < 10 or total > 260:
            del buf[:1]
            return False
        if len(buf) < total:
            return None
        frame = bytes(buf[:total])
        if dxl_crc16(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
            del buf[:1]
            return False
        del buf[:total]
        if frame[4] != ID:
            return False
        self.seen = "dxl"
        body = dxl_unstuff(frame[7:-2])
        if len(body) < 2 or body[0] != 0x55:
            return False
        data = body[2:]
        if len(data) == 12:
            gyro, quat, euler = decode_pose(data)
            return Sample("dxl", None, self.status, gyro, quat, euler)
        if len(data) == 1:
            self.status = data[0]
        return False


# --------------------------------------------------------------------------- #
# live serial (pyserial is imported only here)
# --------------------------------------------------------------------------- #

def open_serial(port: str, timeout: float = 0.05):
    try:
        import serial  # type: ignore
    except ImportError as exc:            # pragma: no cover - environment specific
        raise SystemExit("pyserial is required for live reading: pip install pyserial") from exc
    return serial.Serial(port, BAUD, timeout=timeout)


def probe(link, deadline: float = 0.3) -> str:
    """Return "feetech" or "dxl" by pinging, or raise if neither answers."""
    import time

    parser = Parser()
    for proto, ping in (("feetech", feetech_ping()), ("dxl", dxl_ping())):
        parser.seen = None
        link.reset_input_buffer()
        link.write(ping)
        end = time.monotonic() + deadline
        while time.monotonic() < end:
            chunk = link.read(64)
            if chunk:
                parser.feed(chunk)
                if parser.seen == proto:
                    return proto
    raise SystemExit("no reply from ID 200: check wiring, ground, baud rate and protocol")
