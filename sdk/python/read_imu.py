#!/usr/bin/env python3
"""Read IMU-DXL pose frames and print one JSON line per sample.

    python read_imu.py --port COM5
    python read_imu.py --port /dev/ttyUSB0 --protocol dxl --count 20
    python read_imu.py --replay ../testdata/capture.bin

Every line has the same shape, whichever protocol carried it:

    {"proto":"feetech","seq":42,"status":0,"gyro_dps":[...],
     "quat_wxyz":[...],"euler_deg":[...]}

`proto` is the wire format that frame arrived in, `seq`/`status` come from the
Feetech block (DYNAMIXEL has no counter, and its status byte is the separate
read at address 143, so `seq` is null there). Angles are degrees, ZYX order.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import imu_link  # noqa: E402


def replay(path: str, count: int) -> int:
    data = Path(path).read_bytes()
    parser = imu_link.Parser()
    printed = 0
    for sample in parser.feed(data):
        print(sample.to_json(), flush=True)
        printed += 1
        if count and printed >= count:
            return 0
    if printed == 0:
        print("no valid frames in the capture", file=sys.stderr)
        return 1
    return 0


def live(port: str, protocol: str, count: int) -> int:
    link = imu_link.open_serial(port)
    chosen = imu_link.probe(link) if protocol == "auto" else protocol
    print(f"# {port} @ {imu_link.BAUD} 8N1 · protocol {chosen}", file=sys.stderr)

    parser = imu_link.Parser()
    printed = 0
    last_status = -1e9          # read address 143 once before the first pose
    try:
        while True:
            now = time.monotonic()
            if chosen == "dxl" and now - last_status > 1.0:
                link.write(imu_link.dxl_read(imu_link.DXL_STATUS_ADDR, imu_link.DXL_STATUS_LEN))
                last_status = now
            request = (
                imu_link.feetech_read(imu_link.FEETECH_POSE_ADDR, imu_link.FEETECH_POSE_LEN)
                if chosen == "feetech"
                else imu_link.dxl_read(imu_link.DXL_POSE_ADDR, imu_link.DXL_POSE_LEN)
            )
            link.write(request)
            chunk = link.read(256)
            if chunk:
                for sample in parser.feed(chunk):
                    print(sample.to_json(), flush=True)
                    printed += 1
                    if count and printed >= count:
                        return 0
            time.sleep(0.02)
    except KeyboardInterrupt:
        return 0


def list_serial_ports() -> int:
    """Print the ports this OS knows about (pyserial enumerates Windows and Linux)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is required for --list: pip install pyserial", file=sys.stderr)
        return 1
    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    if not ports:
        print("no serial ports found", file=sys.stderr)
        return 1
    for port in ports:
        print(f"{port.device}\t{port.description}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="read IMU-DXL pose data over the servo bus")
    ap.add_argument("--port", "-p", help="serial port: COM5 (Windows) or /dev/ttyUSB0 (Linux)")
    ap.add_argument("--protocol", choices=("auto", "feetech", "dxl"), default="auto")
    ap.add_argument("--count", "-n", type=int, default=0, help="stop after N samples (0 = forever)")
    ap.add_argument("--replay", help="decode a capture file instead of opening a port")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        return list_serial_ports()
    if args.replay:
        return replay(args.replay, args.count)
    if not args.port:
        ap.error("either --port or --replay is required")
    return live(args.port, args.protocol, args.count)


if __name__ == "__main__":
    sys.exit(main())
