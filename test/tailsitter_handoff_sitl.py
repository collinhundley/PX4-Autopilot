#!/usr/bin/env python3
"""Ordinary local-only SITL transitions; never inject trim or match thrust in the harness.

Start an isolated PX4 instance with 4018_gz_quadtailsitter, 10042_sihsim_xvert,
or 5021 X-Plane Dragonfly.
Example: python test/tailsitter_handoff_sitl.py --bin build/px4_sitl_default/bin \
    --instance 2 --port 14542 --output build/handoff-validation/auto.json
Requires pymavlink. Tuning and simulator setup belong to the airframe, not this test.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin', type=Path, required=True)
    parser.add_argument('--instance', type=int, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--fw-seconds', type=float, default=8)
    parser.add_argument('--airspeed', type=float)
    parser.add_argument('--stick', type=float, help='Manual stabilized throttle, 0..1; omit for automatic Hold')
    parser.add_argument('--rapid-increase', action='store_true')
    parser.add_argument('--abort', action='store_true')
    args = parser.parse_args()
    args.bin = args.bin.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.stick is not None:
        assert 0 <= args.stick <= 1
    events, samples, latest = [], [], {}
    start = time.monotonic()
    stage = 'connect'
    last_hb = last_manual = 0
    throttle = None
    def cli(module, *command):
        result = subprocess.run([str(args.bin / ('px4-' + module)), '--instance', str(args.instance), *map(str, command)],
                                capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        return result.stdout
    def event(name, **extra):
        entry = dict(t=time.monotonic() - start, event=name, **extra)
        events.append(entry)
        print(json.dumps(entry), flush=True)
    # Refuse hardware and arbitrary running aircraft. The port is always bound to loopback.
    autostart = cli('param', 'show', '-q', 'SYS_AUTOSTART').strip()
    assert autostart in ('10042', '4018', '5021'), autostart
    assert 'Disarmed' in cli('commander', 'status')
    link = mavutil.mavlink_connection('udpin:127.0.0.1:' + str(args.port), source_system=250)
    target = None
    def pump(duration=.1):
        nonlocal last_hb, last_manual, target
        until = time.monotonic() + duration
        while time.monotonic() < until:
            now = time.monotonic()
            if now - last_hb >= .5:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
            if throttle is not None and target and now - last_manual >= .025:
                link.mav.manual_control_send(target, 0, 0, int(1000 * throttle), 0, 0)
                last_manual = now
            message = link.recv_match(blocking=True, timeout=.02)
            if message is None:
                continue
            if message.get_type() == 'HEARTBEAT' and message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                target = message.get_srcSystem()
            if target and message.get_srcSystem() == target:
                data = message.to_dict()
                name = message.get_type()
                latest[name] = data
                if name in ('HEARTBEAT', 'ATTITUDE', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE', 'VFR_HUD', 'STATUSTEXT', 'COMMAND_ACK'):
                    samples.append(dict(t=now-start, stage=stage, **data))
                if name == 'STATUSTEXT':
                    event('PX4', text=data['text'])
    def wait_for(predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pump()
            if predicate():
                event(label, altitude=-latest.get('LOCAL_POSITION_NED', {}).get('z', 0))
                return
        raise RuntimeError('Timeout: ' + label)
    def vtol():
        return latest.get('EXTENDED_SYS_STATE', {}).get('vtol_state')
    def transition(state):
        link.mav.command_long_send(target, 1, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, state, 0, 0, 0, 0, 0, 0)
        event('transition requested', state=state)
    passed = False
    reason = None
    try:
        wait_for(lambda: target is not None, 10, 'connected')
        for name in ['ATTITUDE', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE', 'VFR_HUD']:
            mid = getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + name)
            link.mav.command_long_send(target, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 20000, 0, 0, 0, 0, 0)
        # Test-flight setup only. These parameters do not alter the controller tuning.
        cli('param', 'set', 'MIS_TAKEOFF_ALT', 50)
        cli('param', 'set', 'COM_RCL_EXCEPT', 4)
        pump(2)
        cli('commander', 'takeoff')
        stage = 'takeoff'
        wait_for(lambda: -latest.get('LOCAL_POSITION_NED', {}).get('z', 0) > 47, 65, 'hover reached')
        cli('commander', 'mode', 'auto:loiter')
        stage = 'hover'
        pump(5)
        for cycle in range(args.cycles):
            if args.stick is not None:
                throttle = args.stick
                pump(1)
                cli('commander', 'mode', 'stabilized')
                pump(1)
            stage = 'front'
            transition(4)
            if args.abort:
                pump(.5)
                transition(3)
            else:
                wait_for(lambda: vtol() == 4, 20, 'FW entered')
                stage = 'fw'
                fw_start = time.monotonic()
                changed = False
                while time.monotonic() - fw_start < args.fw_seconds:
                    pump()
                    if vtol() != 4:
                        raise RuntimeError('Left FW before requested backtransition')
                    attitude = latest.get('ATTITUDE', {})
                    if any(abs(attitude.get(axis, 0)) > math.radians(300) for axis in ('rollspeed', 'pitchspeed', 'yawspeed')):
                        raise RuntimeError('Angular rate exceeded 300 deg/s')
                    if -latest.get('LOCAL_POSITION_NED', {}).get('z', 50) < 10:
                        raise RuntimeError('Altitude below 10 m')
                    if not changed and time.monotonic() - fw_start > 1:
                        if args.airspeed is not None:
                            link.mav.command_long_send(target, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, args.airspeed, -1, 0, 0, 0, 0)
                            event('airspeed change', speed=args.airspeed)
                        if args.rapid_increase:
                            throttle = 1.0
                            event('rapid throttle increase')
                        changed = True
                transition(3)
            stage = 'back'
            throttle = None
            cli('commander', 'mode', 'auto:loiter')
            wait_for(lambda: vtol() == 3, 20, 'MC returned')
            stage = 'hover'
            pump(8)
            pos = latest.get('LOCAL_POSITION_NED', {})
            speed = math.hypot(pos.get('vx', 100), pos.get('vy', 100))
            event('recovery', cycle=cycle, horizontal_speed=speed, vertical_speed=pos.get('vz'))
            if speed > 3 or abs(pos.get('vz', 100)) > 2:
                raise RuntimeError('MC recovery did not settle')
        passed = True
    except Exception as error:
        reason = str(error)
        event('failed', reason=reason)
    finally:
        cli('commander', 'disarm', '-f')
        pump(.5)
        args.output.write_text(json.dumps(dict(passed=passed, reason=reason, airframe=int(autostart), events=events, samples=samples), indent=2) + '\n')
        event('finished', passed=passed, output=str(args.output))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
