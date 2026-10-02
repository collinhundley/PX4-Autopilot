#!/usr/bin/env python3
"""Exercise the real SITL MSP driver on a PTY without flight hardware.

Build px4_sitl_test (or px4_sitl_default), then run:
  python3 src/drivers/osd/msp_osd/test_displayport.py build/px4_sitl_test

Reconstructs DisplayPort grids, checks all requested fields, metric/imperial
units, live item positioning, canvas negotiation/overrides, disabled/re-enabled OSD, malformed receive
traffic, checksums, frame bandwidth, and disabled RC-stick forwarding. Numeric
values may be unavailable in disarmed SITL; semantic values are unit-tested.
"""

import argparse
import collections
import os
from pathlib import Path
import pty
import re
import select
import subprocess
import tempfile
import time


FULL_MASK = ((1 << 30) - 1) & ~(1 << 15)  # ESC temperature remains unsupported.
ALTITUDE_MASK = 1 << 12
# Literal defaults are shared by every canvas; smaller canvases clip fields.
POSITIONS = {
    "mode": (1, 1, 16), "arm": (18, 1, 4), "time": (42, 1, 10),
    "home_arrow": (23, 1), "home": (25, 1, 12),
    "ground_speed": (11, 9, 8), "airspeed": (11, 6, 8),
    "altitude": (34, 6, 12), "throttle": (1, 19, 9),
    "voltage": (21, 17, 5), "cell_voltage": (27, 17, 5),
    "compensated_voltage": (20, 19, 7), "compensated_cell_voltage": (28, 19, 5),
    "percentage": (16, 18, 6), "capacity": (24, 18, 8), "current": (32, 18, 8),
    "horizon": (26, 7),
}


def capture(fd, seconds, data):
    """Retain partial MSP packets between capture windows."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], min(0.1, max(0, deadline - time.monotonic())))
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


def request(command, payload=b""):
    """Encode an incoming MSPv1 request, including zero-length payloads."""
    packet = bytearray(b"$M<") + bytes([len(payload), command]) + payload
    checksum = 0
    for value in packet[3:]:
        checksum ^= value
    return bytes(packet + bytes([checksum]))


def check_telemetry(packets, inav=False):
    counts = collections.Counter(command for command, _ in packets)
    assert counts[2] and counts[101] and counts[130], counts
    assert not counts[105], "RC stick forwarding must be disabled"
    for command, payload in packets:
        if command == 2:
            assert payload == (b"INAV" if inav else b"BTFL"), payload
    return counts


def complete_frames(packets):
    """Ignore a capture's leading/trailing incomplete DisplayPort frame."""
    frames = []
    current = None
    for command, payload in packets:
        if command != 182:
            continue
        assert payload, "Empty DisplayPort command"
        if payload == b"\x00":
            current = [payload]
        elif current is not None:
            current.append(payload)
            if payload == b"\x04":
                frames.append(current)
                current = None
    return frames


def reconstruct(frame, canvas):
    columns, rows = canvas
    profile = {(60, 22): 3, (50, 18): 1, (30, 16): 0}.get(canvas)
    prefix = [b"\x00"]
    if profile is not None:
        prefix.append(bytes([5, 0, profile]))
    prefix.append(b"\x02")
    assert frame[:len(prefix)] == prefix, "Incorrect heartbeat/canvas/clear sequence"
    assert frame[-1] == b"\x04", "Missing draw"
    grid = [[ord(' ')] * columns for _ in range(rows)]
    writes = []
    for payload in frame[len(prefix):-1]:
        assert 5 <= len(payload) <= 35 and payload[0] == 3, payload
        row, column, attribute = payload[1:4]
        assert attribute <= 1, f"Unexpected glyph page/attributes: {attribute}"
        assert payload[-1] == 0 and 0 not in payload[4:-1], "Unterminated or embedded-NUL text"
        text = payload[4:-1]
        assert 0 <= row < rows and 0 <= column < columns, (canvas, payload)
        assert column + len(text) <= columns, (canvas, payload)
        grid[row][column:column + len(text)] = [value | (attribute << 8) for value in text]
        writes.append((column, row, text))
    return grid, writes


def field(writes, coordinate):
    found = [text for x, y, text in writes if (x, y) == coordinate]
    assert len(found) == 1, (coordinate, found, writes)
    return found[0].decode("ascii")


def region(grid, rectangle):
    x, y, width = rectangle
    return ''.join(chr(value) for value in grid[y][x:x + width]).strip(' ')


def check_fields(grid, writes, canvas, imperial, inav=False):
    positions = POSITIONS
    distance = chr((0x74 if imperial else 0x82) if inav else (0x0f if imperial else 0x0c))
    altitude = chr((0x78 if imperial else 0x76) if inav else (0x0f if imperial else 0x0c))
    speed = chr((0x91 if imperial else 0x8f) if inav else (0x9d if imperial else 0x9f))
    mah = chr(0x99 if inav else 0x07)
    amps = chr(0x6a if inav else 0x9a)
    throttle_icon = chr(0x95 if inav else 0x04)
    speed_icon = chr(0x17 if inav else 0x70)
    airspeed_prefix = chr(0x8c) + r" \*?" if inav else r"AS(?: |\*)"
    battery_icon = f"[{chr(0x63)}-{chr(0x69)}]" if inav else f"[{chr(0x90)}-{chr(0x96)}]"
    scalar = r"(?:--|-?\d+(?:\.\d+)?)"
    patterns = {
        "home": rf"{scalar}{distance}",
        "ground_speed": rf"{speed_icon} \d+\.\d{speed}",
        "airspeed": rf"{airspeed_prefix}\d+\.\d{speed}",
        "altitude": rf"{scalar}{altitude}",
        "throttle": rf"{throttle_icon} {scalar}%",
        "voltage": rf"{scalar}V", "cell_voltage": rf"{scalar}V",
        "compensated_voltage": rf"C {scalar}V", "compensated_cell_voltage": rf"{scalar}V",
        "percentage": rf"{battery_icon} {scalar}%", "capacity": rf"{scalar}(?:{mah}|AH|{chr(0xd3)})",
        "current": rf"{scalar}{amps}",
    }
    def fits(rectangle):
        x, y, width = rectangle
        return y < canvas[1] and x + width <= canvas[0]

    for name, pattern in patterns.items():
        if not fits(positions[name]):
            continue  # Bounds and clipped packets are validated by reconstruct().
        text = region(grid, positions[name])
        assert re.fullmatch(pattern, text), (name, text, pattern)
    for row, separator_column in ((17, 26), (19, 27)):
        if row < canvas[1] and separator_column < canvas[0]:
            assert grid[row][separator_column] == ord('/'), (row, separator_column)
    mode = region(grid, positions["mode"])
    assert mode and re.fullmatch(r"[A-Z0-9 />?_! -]+", mode), mode
    arm = region(grid, positions["arm"])
    assert re.fullmatch(r"(?:ARM|DIS)!?|--", arm), arm
    assert not any('PX4' in ''.join(map(chr, row)) for row in grid)
    if fits(positions["time"]):
        timer = region(grid, positions["time"])
        timer_icon = chr(0x9f if inav else 0x9c)
        assert re.fullmatch(rf"{timer_icon} (?:--:--|\d{{2}}:\d{{2}}(?::\d{{2}})?|>99H)", timer), timer
    arrow_x, arrow_y = positions["home_arrow"]
    arrow = grid[arrow_y][arrow_x]
    assert (0x13c <= arrow <= 0x14b) if inav else (0x60 <= arrow <= 0x6f), arrow
    center_x, center_y = positions["horizon"]
    horizon_text = grid[center_y][center_x - 5:center_x + 6]
    first_bar = 0x14c if inav else 0x80
    bars = [value for row in grid for value in row if first_bar <= value <= first_bar + 8]
    # Crosshairs overwrite the center of the horizon's unavailable indication.
    unavailable = any(text == b"HORIZON --" for _, _, text in writes)
    assert bars or unavailable, (horizon_text, writes)
    assert grid[center_y][center_x - 1:center_x + 2] == ([0x13a, 0x166, 0x13b] if inav else [0x72, 0x73, 0x74]), "Missing reticle"


def check_frames(packets, canvas, imperial, altitude_only=False, inav=False):
    counts = check_telemetry(packets, inav)
    frames = complete_frames(packets)
    assert len(frames) >= 5, f"Only {len(frames)} complete frames"
    metadata_size = sum(max((len(payload) + 6 for command, payload in packets if command == message), default=0)
                        for message in (2, 101, 130))
    largest = 0
    for frame in frames:
        grid, writes = reconstruct(frame, canvas)
        assert all(value == ord(' ') for value in grid[0]), "Default coordinates must leave a top margin"
        # 115200 baud 8N1 at 10Hz permits 1152 bytes/cycle. Reserve >10% margin.
        frame_bytes = sum(len(payload) + 6 for payload in frame) + metadata_size
        largest = max(largest, frame_bytes)
        assert frame_bytes <= 1024, f"Frame too large for UART budget: {frame_bytes}"
        if altitude_only:
            assert len(writes) == 1, writes
            text = region(grid, POSITIONS["altitude"])
            unit = chr((0x78 if imperial else 0x76) if inav else (0x0f if imperial else 0x0c))
            assert re.fullmatch(rf"(?:--|-?\d+){unit}", text), text
        else:
            check_fields(grid, writes, canvas, imperial, inav)
    print(f"PASS: {len(frames)} frames, {canvas[0]}x{canvas[1]}, "
          f"{'imperial' if imperial else 'metric'}, inav={inav}, altitude_only={altitude_only}, "
          f"max {largest} bytes/cycle; MSP counts={dict(counts)}")


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

    def set_parameter(name, value):
        cli("param", "set", name, str(value))

    def settled_capture():
        capture(master, 2, pending)  # Parameter subscription is rate limited to 1Hz.
        return capture(master, 2, pending)

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
                set_parameter("OSD_RC_STICK", 0)
                set_parameter("OSD_FONT", 0)
                set_parameter("OSD_SYMBOLS", FULL_MASK)
                set_parameter("OSD_UNITS", 1)
                set_parameter("OSD_CANVAS", 0)
                set_parameter("OSD_CAM_PITCH", 0)
                cli("msp_osd", "start", "-d", device)
                check_frames(settled_capture(), (53, 20), imperial=True)
                set_parameter("OSD_FONT", 1)
                set_parameter("OSD_CANVAS", 3)
                check_frames(settled_capture(), (60, 22), imperial=True, inav=True)
                set_parameter("OSD_CANVAS", 2)
                check_frames(settled_capture(), (50, 18), imperial=True, inav=True)
                set_parameter("OSD_CANVAS", 1)
                set_parameter("OSD_UNITS", 0)
                check_frames(settled_capture(), (30, 16), imperial=False, inav=True)
                set_parameter("OSD_FONT", 0)
                set_parameter("OSD_CANVAS", 0)

                set_parameter("OSD_UNITS", 0)
                check_frames(settled_capture(), (53, 20), imperial=False)

                # Invalid lengths/indices must not overrun VTX tables or stall the work queue.
                malformed = [request(89), request(89, b"\xff"), request(89, b"\xff\xff"),
                             request(89, b"\xff" * 15), request(89, b"\xff" * 255),
                             request(227, bytes([0, 8]) + b"INVALID " + b"A\x00\x08" + b"\x00" * 16),
                             request(227, bytes([255, 8]) + b"INVALID " + b"A\x00\x08" + b"\x00" * 16),
                             request(228, b"\x00\x00\x00\x00"), request(228, b"\xff\x00\x00\x00"),
                             request(228, b"\x01\x00\x00\x04LONG"),
                             request(188), request(188, b"\x1e"), request(188, b"\xff\xff"),
                             request(188, b"\x00\x00"), request(188, b"\x1e\x10\x00")]
                bad_crc = bytearray(request(188, bytes([30, 16])))
                bad_crc[-1] ^= 1
                os.write(master, b"junk$?" + b"".join(malformed) + bad_crc)
                check_frames(capture(master, 2, pending), (53, 20), imperial=False)
                # Split the header, payload and checksum over separate driver runs.
                packet = request(188, bytes([30, 16]))
                for fragment in (packet[:1], packet[1:4], packet[4:6], packet[6:7], packet[7:]):
                    os.write(master, fragment)
                    time.sleep(0.12)
                check_frames(settled_capture(), (30, 16), imperial=False)

                # Explicit profiles override the air unit's negotiated dimensions.
                set_parameter("OSD_CANVAS", 3)
                set_parameter("OSD_UNITS", 1)
                check_frames(settled_capture(), (60, 22), imperial=True)
                set_parameter("OSD_CANVAS", 1)
                os.write(master, request(188, bytes([60, 22])))
                check_frames(settled_capture(), (30, 16), imperial=True)

                set_parameter("OSD_CANVAS", 4)
                set_parameter("OSD_FONT", 1)
                check_frames(settled_capture(), (53, 20), imperial=True, inav=True)
                set_parameter("OSD_CANVAS", 0)
                os.write(master, request(188, bytes([53, 20])))
                check_frames(settled_capture(), (53, 20), imperial=True, inav=True)

                # Move a normal field, a grouped battery field and a centre-anchored
                # graphic without restarting the driver, then reset their fixed defaults.
                set_parameter("OSD_FONT", 0)
                set_parameter("OSD_SYMBOLS", (1 << 23) | (1 << 9) | (1 << 18))
                for name, value in (("OSD_POS_ASPD_X", 2), ("OSD_POS_ASPD_Y", 4),
                                    ("OSD_POS_CURR_X", 4), ("OSD_POS_CURR_Y", 17),
                                    ("OSD_POS_CROSS_X", 40), ("OSD_POS_CROSS_Y", 10)):
                    set_parameter(name, value)
                frames = complete_frames(settled_capture())
                assert len(frames) >= 5
                for frame in frames:
                    grid, writes = reconstruct(frame, (53, 20))
                    assert re.fullmatch(r"AS(?: |\*)\d+\.\d\x9d", region(grid, (2, 4, 10)))
                    assert re.fullmatch(r"(?:--|\d+\.\d)\x9a", region(grid, (4, 17, 10)))
                    assert grid[10][39:42] == [0x72, 0x73, 0x74]
                    assert all(value == ord(' ') for value in grid[6]), "Old airspeed row was not cleared"
                print("PASS: live X/Y positioning for airspeed, current and crosshair")
                for prefix in ("OSD_POS_ASPD", "OSD_POS_CURR", "OSD_POS_CROSS"):
                    for axis in ("X", "Y"):
                        cli("param", "reset", prefix + "_" + axis)
                set_parameter("OSD_SYMBOLS", FULL_MASK)
                check_frames(settled_capture(), (53, 20), imperial=True)
                # Firmware before fixed defaults allowed saved -1 coordinates.
                # Migration must restore these axes without disturbing a valid axis.
                set_parameter("OSD_POS_ASPD_X", -1)
                set_parameter("OSD_POS_ASPD_Y", 6)
                check_frames(settled_capture(), (53, 20), imperial=True)
                print("PASS: reset and legacy negative coordinates restore fixed defaults")

                os.write(master, request(188, bytes([60, 22])))
                set_parameter("OSD_CANVAS", 0)
                set_parameter("OSD_UNITS", 0)
                set_parameter("OSD_SYMBOLS", ALTITUDE_MASK)
                check_frames(settled_capture(), (60, 22), imperial=False, altitude_only=True)

                set_parameter("OSD_SYMBOLS", 0)
                transition = capture(master, 2, pending)
                commands = [payload for command, payload in transition if command == 182]
                assert commands[-3:] == [b"\x02", b"\x04", b"\x01"], commands
                disabled = capture(master, 2, pending)
                check_telemetry(disabled)
                assert not any(command == 182 for command, _ in disabled), "Disabled OSD must stay released"
                print("PASS: disabling OSD clears/releases once and retains status/battery telemetry")

                set_parameter("OSD_SYMBOLS", FULL_MASK)
                set_parameter("OSD_UNITS", 1)
                check_frames(settled_capture(), (60, 22), imperial=True)
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
