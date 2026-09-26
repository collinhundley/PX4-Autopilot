#!/usr/bin/env python3
"""Capture the real SITL MSP driver on a PTY; no hardware or third-party modules.

Build px4_sitl_test (or px4_sitl_default), then run:
  python3 src/drivers/osd/msp_osd/test_displayport.py build/px4_sitl_test

Checks MSPv1 checksums, DisplayPort frame order, complete altitude strings,
selected OSD_SYMBOLS bits, and disabled RC-stick forwarding.
"""

import argparse
import collections
import os
from pathlib import Path
import pty
import select
import subprocess
import tempfile
import time


def capture(fd, seconds, data):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.1)
        if ready:
            data.extend(os.read(fd, 65536))
    packets = []
    while len(data) >= 6:
        assert data[:3] == b"$M>", f"Invalid MSP header: {data[:6]!r}"
        size, command = data[3:5]
        if len(data) < size + 6:
            break
        checksum = 0
        for value in data[3:size + 6]:
            checksum ^= value
        assert checksum == 0, "Invalid MSP checksum"
        packets.append((command, bytes(data[5:size + 5])))
        del data[:size + 6]
    return packets


def check_frames(packets, gps_enabled):
    commands = collections.Counter(command for command, _ in packets)
    assert commands[2] and commands[101] and commands[130], commands
    assert not commands[105], "RC stick forwarding must be disabled"
    frames = []
    current = []
    for command, payload in packets:
        if command != 182:
            continue
        assert payload
        current.append(payload)
        if payload == b"\x04":
            frames.append(current)
            current = []
    assert len(frames) >= 5, f"Only {len(frames)} complete frames"
    # The first captured frame may begin partway through the transmission.
    for frame in frames[1:]:
        assert frame[0] == b"\x00", "Missing DisplayPort heartbeat"
        assert frame[1] == b"\x02", "Missing DisplayPort clear"
        assert frame[-1] == b"\x04", "Missing DisplayPort draw"
        writes = frame[2:-1]
        assert all(payload[0] == 3 for payload in writes)
        altitude = [payload for payload in writes if payload[1:3] == bytes([6, 2])]
        assert len(altitude) == 1
        # Regression: sizeof(msp_altitude_t) sent only 10 of these 13 bytes.
        assert len(altitude[0]) == 13, altitude[0]
        text = altitude[0][5:].split(b"\0", 1)[0].decode("ascii")
        assert len(text.rsplit(".", 1)[-1]) == 1, text
        assert float(text) == float(text), "Altitude must not be NaN"
        latitude = [payload for payload in writes if payload[1:3] == bytes([10, 41])]
        assert bool(latitude) == gps_enabled
    print(f"PASS: {len(frames)} frames; GPS enabled={gps_enabled}; MSP counts={dict(commands)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_dir", type=Path)
    parser.add_argument("--instance", type=int, default=28)
    args = parser.parse_args()
    build = args.build_dir.resolve()
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    pending = bytearray()
    env = os.environ | {
        "PX4_SYS_AUTOSTART": "10043",
        "PX4_SIM_MODEL": "sihsim_standard_vtol",
        "PATH": str(build / "bin") + os.pathsep + os.environ["PATH"],
    }

    def cli(command, *arguments, check=True):
        result = subprocess.run(
            [str(build / "bin" / ("px4-" + command)), "--instance", str(args.instance), *arguments],
            env=env, capture_output=True, text=True, timeout=10,
        )
        if check:
            assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout + result.stderr

    with tempfile.TemporaryDirectory(prefix="px4-msp-displayport-") as directory:
        root = Path(directory)
        with (root / "px4.log").open("w+") as log:
            process = subprocess.Popen(
                [str(build / "bin/px4"), "-d", "-i", str(args.instance), "-w", str(root), str(build / "etc")],
                env=env, cwd=root, stdout=log, stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 30
                while "Startup script returned successfully" not in (root / "px4.log").read_text():
                    assert process.poll() is None, (root / "px4.log").read_text()
                    assert time.monotonic() < deadline, (root / "px4.log").read_text()
                    time.sleep(0.2)
                cli("param", "set", "OSD_RC_STICK", "0")
                cli("param", "set", "OSD_SYMBOLS", "16383")
                cli("msp_osd", "start", "-d", device)
                check_frames(capture(master, 2, pending), gps_enabled=True)
                cli("param", "set", "OSD_SYMBOLS", "4096")  # Altitude only among selectable fields.
                capture(master, 2, pending)  # Parameter subscription is rate limited to 1 Hz.
                check_frames(capture(master, 2, pending), gps_enabled=False)
                status = cli("msp_osd", "status")
                assert "initialized: 1" in status, status
                assert "unsuccessful sends: 0" in status, status
                print(status)
                cli("msp_osd", "stop")
            except BaseException:
                print((root / "px4.log").read_text())
                raise
            finally:
                if process.poll() is None:
                    cli("shutdown", check=False)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                os.close(master)
                os.close(slave)


if __name__ == "__main__":
    main()
