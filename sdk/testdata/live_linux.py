#!/usr/bin/env python3
"""End-to-end test of the live serial path, on Linux, over a PTY.

    python testdata/live_linux.py
    python testdata/live_linux.py --c-binary sdk/c/read_imu

Starts fake_device.py, points the reader at the PTY it prints, and compares the
first JSON line against testdata/expected.ndjson. This is the only test that
exercises termios, the request/response poll loop and the protocol probe; it
needs a POSIX pty, so it runs on Linux/macOS but not on Windows.
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
PYTHON_READER = HERE.parent / "python" / "read_imu.py"
EXPECTED = (HERE / "expected.ndjson").read_text().splitlines()


def run_case(protocol: str, want: str, c_binary: str | None) -> bool:
    device = subprocess.Popen(
        [sys.executable, str(HERE / "fake_device.py"), "--protocol", protocol],
        stdout=subprocess.PIPE, text=True,
    )
    try:
        port = device.stdout.readline().strip()
        if not port.startswith("/dev/"):
            print(f"  [FAIL] fake device did not report a pty: {port!r}")
            return False

        readers = [("python", [sys.executable, str(PYTHON_READER)])]
        if c_binary:
            readers.append(("c", [c_binary]))

        ok = True
        for name, command in readers:
            result = subprocess.run(
                command + ["--port", port, "--protocol", protocol, "--count", "1"],
                capture_output=True, text=True, timeout=30,
            )
            lines = [line for line in result.stdout.splitlines() if line.startswith("{")]
            good = lines == [want]
            ok = ok and good
            print(f"  [{'PASS' if good else 'FAIL'}] {protocol} via {name}"
                  f" -> {lines or result.stderr.strip()}")
        return ok
    finally:
        device.terminate()
        try:
            device.wait(timeout=5)
        except subprocess.TimeoutExpired:
            device.kill()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--c-binary", help="optional path to the built read_imu")
    args = ap.parse_args()

    print("live serial path over a PTY")
    ok = run_case("feetech", EXPECTED[0], args.c_binary)
    ok = run_case("dxl", EXPECTED[2], args.c_binary) and ok
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
