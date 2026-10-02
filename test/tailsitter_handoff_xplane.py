#!/usr/bin/env python3
"""Guarded ordinary Dragonfly handoffs. No trim copying or staged thrust matching.

Reuses the user's reset/new-flight and CFD correction helpers. Cases specify
environment, pilot demand and transition speed; controller gains stay unchanged.
Run with the PX4 .venv Python, --cases JSON and --output an empty result directory.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

from pymavlink import mavutil
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
SITL = Path('/Users/collin/Documents/UAV/Simulation/XPlane/Dragonfly/SITL')
BIN = REPO / 'build/px4_sitl_default_xplane/bin'
sys.path.insert(0, str(REPO / 'build/xplane-setup'))
import xplane_api as xp


def cli(module, *args):
    # The macOS command socket can occasionally leave a read-only client waiting
    # after a response. Retry reads only; never repeat an uncertain flight action.
    attempts = 3 if (module == 'commander' and args == ('status',)) or module in ('listener', 'ver') or (module == 'param' and args[0] == 'show') else 1
    for attempt in range(attempts):
        try:
            result = subprocess.run([str(BIN / ('px4-' + module)), *map(str, args)],
                                    capture_output=True, text=True, timeout=10)
            break
        except subprocess.TimeoutExpired:
            if attempt+1 == attempts:
                raise
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def run_case(case, output):
    case = dict(case)
    case.setdefault('back_throttle_slew', .5)
    # Record the actual firmware relationship, including for later log analysis.
    case['back_throttle_rise'] = case['back_throttle_slew']
    out = output / case['name']
    out.mkdir()
    (out / 'case.json').write_text(json.dumps(case, indent=2))
    label = output.name + '-' + case['name']
    params = {'VT_ARSP_TRANS': case.get('handoff_mph', 33) * .44704,
              'VT_ARSP_BLEND': min(13.4112, case.get('handoff_mph', 33) * .44704 * .9),
              'FW_AIRSPD_TRIM': case.get('cruise_mph', 33) * .44704,
              'VT_TS_THR_SLEW': case.get('throttle_slew', .2),
              'VT_TS_B_THR_SLEW': case['back_throttle_slew'],
              'FW_THR_SLEW_MAX': case.get('tecs_throttle_slew', 0),
              'FW_PSP_OFF': case.get('pitch_target_deg', 10),
              'MIS_TAKEOFF_ALT': 60}
    with (out / 'reset.log').open('w') as stream:
        subprocess.run([sys.executable, str(SITL / 'reset_sim.py'), label,
                        '--parameters', json.dumps(params)], stdout=stream, stderr=subprocess.STDOUT, check=True)
    with (out / 'correction.json').open('w') as stream:
        subprocess.run([sys.executable, str(SITL / 'fuselage-correction.py'), 'on', '--model', 'cfd'],
                       stdout=stream, check=True)
    refs, commands = xp.catalog('datarefs'), xp.catalog('commands')
    def read(name):
        return xp.request('/datarefs/%s/value' % refs[name]['id'])['data']
    def set_ref(name, value):
        xp.request('/datarefs/%s/value' % refs[name]['id'], {'data': value}, 'PATCH')
    def command(name):
        xp.request('/command/%s/activate' % commands[name]['id'], {'duration': 0}, 'POST')
    aircraft = base64.b64decode(read('sim/aircraft/view/acf_relative_path')).decode().rstrip('\0')
    assert aircraft == 'Aircraft/PX4/Dragonfly/Dragonfly.acf', aircraft
    assert 'Disarmed' in cli('commander', 'status')
    assert cli('param', 'show', '-q', 'SYS_AUTOSTART').strip() == '5021'
    assert not read('sim/operation/failures/enable_random_failures')
    assert not read('sim/time/paused')
    assert read('dragonfly/probe/seconds_remaining') == 0
    assert read('sim/operation/override/override_engines') == 0
    assert 'dragonfly/handoff_test/seconds_remaining' in refs, 'Moment test fixture is not loaded'
    set_ref('dragonfly/handoff_test/seconds_remaining', 0)
    set_ref('sim/weather/region/change_mode', 3)
    set_ref('sim/weather/region/update_immediately', 1)
    weather = {}
    for key, value in [('wind_speed_msc', case.get('wind_mps', 0)),
                       ('wind_direction_degt', case.get('wind_from_deg', 30)),
                       ('shear_speed_msc', case.get('gust_mps', 0)),
                       ('shear_direction_degt', case.get('wind_from_deg', 30)),
                       ('turbulence', case.get('turbulence', 0))]:
        name = 'sim/weather/region/' + key
        set_ref(name, [value] * len(read(name)))
        weather[key] = read(name)
    (out / 'weather.json').write_text(json.dumps(weather, indent=2))
    # Validate simulator timing before arming. A stalled/slow renderer also
    # delays the X-Plane sensor bridge, making controller comparisons invalid.
    timing = []
    for _ in range(5):
        time.sleep(1)
        timing.append({name: read(name) for name in (
            'sim/operation/misc/frame_rate_period', 'sim/time/total_flight_time_sec')})
    (out / 'simulator-timing.json').write_text(json.dumps(timing, indent=2))
    assert all(0 < sample['sim/operation/misc/frame_rate_period'] < .025 for sample in timing), 'Simulator timing guard before arming'
    assert timing[-1]['sim/time/total_flight_time_sec'] - timing[0]['sim/time/total_flight_time_sec'] > 3, 'Simulator clock stalled before arming'
    (out / 'parameters.txt').write_text(cli('param', 'show', '-a'))
    for path in (SITL / 'rootfs').glob('*bson'):
        shutil.copy2(path, out / path.name)
    (out / 'firmware.json').write_text(json.dumps({
        'sha256': hashlib.sha256((BIN / 'px4').read_bytes()).hexdigest(),
        'version': cli('ver', 'all')}, indent=2))
    link = mavutil.mavlink_connection('udpin:127.0.0.1:14540', source_system=250)
    latest, last_seen, events = {}, {}, []
    start = time.monotonic()
    stage, stage_at = 'connect', start
    last_send = last_hb = last_print = 0
    stable = None
    moments_on = changed = rapid = aborted = back_aborted = fw_att_stopped = mc_att_stopped = interrupted = False
    fw_stick_applied = False
    fw_count = cycle = 0
    throttle = .5
    recovery_alt = 60.
    passed, reason = False, 'Total test timeout'
    def event(message, **extra):
        item = dict(t=time.monotonic()-start, case=case['name'], stage=stage, event=message, **extra)
        events.append(item)
        print(json.dumps(item), flush=True)
    def transition(state, immediate=False):
        link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, state, int(immediate), 0, 0, 0, 0, 0)
        event('transition', state=state, immediate=immediate)
    def upload_mission():
        dataman = SITL / 'rootfs/dataman'
        if dataman.exists():
            shutil.copy2(dataman, out / 'dataman-before-mission')
        gps, att = latest['GLOBAL_POSITION_INT'], latest['ATTITUDE']
        heading = att['yaw']
        latitude = gps['lat'] / 1e7 + math.degrees(800 * math.cos(heading) / 6371000)
        longitude = gps['lon'] / 1e7 + math.degrees(800 * math.sin(heading) /
                                                  (6371000 * math.cos(math.radians(gps['lat'] / 1e7))))
        mission = dict(latitude=latitude, longitude=longitude, altitude=60, distance=800)
        (out / 'mission.json').write_text(json.dumps(mission, indent=2))
        # A distant landing item satisfies the existing mission-feasibility setting.
        # The guarded test returns to Hold long before reaching either waypoint.
        link.mav.mission_count_send(1, 1, 2)
        sent = False
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            link.mav.manual_control_send(1, 0, 0, 500, 0, 0)
            message = link.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
                                      blocking=True, timeout=.2)
            if message and message.get_type() in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
                assert message.seq in (0, 1)
                navigation = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT if message.seq == 0 else mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND
                link.mav.mission_item_int_send(1, 1, message.seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                               navigation, int(message.seq == 0), 1, 0, 0, 0, float('nan'),
                                               round(latitude*1e7), round(longitude*1e7), 60 if message.seq == 0 else 0)
                sent = True
            elif message and sent:
                assert message.type == mavutil.mavlink.MAV_MISSION_ACCEPTED, message
                event('mission uploaded', **mission)
                return
        raise RuntimeError('Mission upload timeout')
    try:
        with (out / 'telemetry.jsonl').open('w') as stream:
            while time.monotonic() - start < 240:
                now = time.monotonic()
                if now-last_send > .04:
                    link.mav.manual_control_send(1, 0, 0, round(throttle*1000), 0, 0)
                    last_send = now
                if now-last_hb > .5:
                    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_hb = now
                message = link.recv_match(blocking=True, timeout=.02)
                if message and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                    typ = message.get_type()
                    if typ in ('HEARTBEAT', 'ATTITUDE', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE',
                               'VFR_HUD', 'STATUSTEXT', 'COMMAND_ACK', 'ATTITUDE_TARGET', 'GLOBAL_POSITION_INT'):
                        latest[typ] = message.to_dict()
                        last_seen[typ] = now
                        stream.write(json.dumps(dict(t=now-start, stage=stage, **latest[typ]))+'\n')
                    if typ == 'STATUSTEXT':
                        event('PX4', text=message.text)
                if not all(name in latest for name in ('HEARTBEAT', 'ATTITUDE', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE')):
                    if now-start > 10:
                        raise RuntimeError('Required telemetry missing')
                    continue
                hb, att, pos = [latest[k] for k in ('HEARTBEAT', 'ATTITUDE', 'LOCAL_POSITION_NED')]
                assert hb['autopilot'] == 12 and hb['type'] == 20
                vtol = latest['EXTENDED_SYS_STATE']['vtol_state']
                alt, vz = -pos['z'], pos['vz']
                speed = math.hypot(pos['vx'], pos['vy'])
                rate = max(abs(math.degrees(att[k])) for k in ('rollspeed', 'pitchspeed', 'yawspeed'))
                if any(now-last_seen[k] > 2 for k in ('ATTITUDE', 'LOCAL_POSITION_NED', 'HEARTBEAT')):
                    raise RuntimeError('Telemetry-loss guard')
                if alt > 120 or math.hypot(pos['x'], pos['y']) > 2000 or rate > 250:
                    raise RuntimeError('Altitude/distance/angular-rate guard: altitude=%.2fm distance=%.1fm rate=%.2fdeg/s' % (alt, math.hypot(pos['x'], pos['y']), rate))
                if vtol == 4:
                    attitude = Rotation.from_euler('ZYX', [att['yaw'], att['pitch'], att['roll']])
                    fw_angles = (attitude * Rotation.from_euler('Y', 90, degrees=True)).as_euler('ZYX', degrees=True)
                    if abs(fw_angles[1]) > 40 or abs(fw_angles[2]) > 40:
                        raise RuntimeError('FW attitude guard')
                elif stage == 'climb' and max(abs(att['roll']), abs(att['pitch'])) > math.radians(40):
                    raise RuntimeError('Hover attitude guard')
                if stage in ('front', 'fw', 'back', 'recover') and alt < 25:
                    raise RuntimeError('Low-altitude guard')
                if stage != 'connect' and now-stage_at > 3 and not hb['base_mode'] & 128:
                    raise RuntimeError('Unexpected disarm')
                if alt > 30 and not moments_on:
                    for key in ('pitch', 'roll', 'yaw'):
                        set_ref('dragonfly/handoff_test/' + key + '_nm', case.get(key + '_nm', 0))
                    set_ref('dragonfly/handoff_test/seconds_remaining', 180)
                    moments_on = True
                    event('moments enabled', pitch=case.get('pitch_nm', 0), roll=case.get('roll_nm', 0), yaw=case.get('yaw_nm', 0))
                    (out / 'moment-readback.json').write_text(json.dumps({
                        key: read('dragonfly/handoff_test/' + key + '_nm') for key in ('pitch', 'roll', 'yaw')}, indent=2))
                if (not changed and 'speed_change_mph' in case
                        and stage == case.get('speed_change_stage', 'fw')
                        and now-stage_at > case.get('speed_change_after', 3)):
                    link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0,
                                              case['speed_change_mph']*.44704, -1, 0, 0, 0, 0)
                    changed = True
                    event('automatic airspeed change', mph=case['speed_change_mph'])
                if stage == 'connect':
                    assert not hb['base_mode'] & 128 and abs(alt) < 3 and vtol == 3
                    if case.get('mission'):
                        if 'GLOBAL_POSITION_INT' not in latest:
                            continue
                        upload_mission()
                    for name in ('ATTITUDE', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE', 'VFR_HUD'):
                        mid = getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + name)
                        link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 20000, 0, 0, 0, 0, 0)
                    cli('commander', 'takeoff')
                    stage, stage_at = 'climb', now
                    event('takeoff')
                elif stage in ('climb', 'recover'):
                    hover_alt = 60. if stage == 'climb' else recovery_alt
                    good = (vtol == 3 and abs(alt-hover_alt) < 4 and speed < 1.5 and abs(vz) < .6
                            and rate < case.get('settle_rate_dps', 20))
                    stable = (stable or now) if good else None
                    if stable and now-stable > case.get('settle_seconds', 4):
                        if stage == 'recover' and cycle >= case.get('cycles', 1):
                            passed, reason = True, 'Ordinary transition cycle completed'
                            break
                        throttle = case.get('stick', .5)
                        mode = 'auto:mission' if case.get('mission') else 'stabilized' if 'stick' in case else 'altctl'
                        cli('commander', 'mode', mode)
                        if case.get('delay_fw_ms'):
                            cli('fw_att_control', 'stop')
                            fw_att_stopped = True
                            event('FW attitude publications stopped')
                        stage, stage_at, stable = 'mode', now, None
                    elif now-stage_at > (105 if stage == 'climb' else case.get('recovery_timeout', 45)):
                        raise RuntimeError('Hover did not settle')
                elif stage == 'mode':
                    if now-stage_at > .5:
                        transition(4)
                        stage, stage_at = 'front', now
                elif stage == 'front':
                    if case.get('abort') and not aborted and now-stage_at > .8:
                        transition(3)
                        aborted = True
                        throttle = .5
                        recovery_alt = alt
                        cli('commander', 'mode', 'auto:loiter')
                        stage, stage_at = 'back', now
                    elif vtol == 4:
                        fw_count += 1
                        event('FW entered', alt=alt, cas=latest.get('VFR_HUD', {}).get('airspeed'))
                        (out / ('environment-at-handoff-%d.json' % fw_count)).write_text(json.dumps({
                            'wind_mps': read('sim/weather/aircraft/wind_now_speed_msc'),
                            'wind_from_deg': read('sim/weather/aircraft/wind_now_direction_degt'),
                            'moment_seconds_remaining': read('dragonfly/handoff_test/seconds_remaining')}, indent=2))
                        stage, stage_at = 'fw', now
                        rapid = False
                    elif now-stage_at > 15 or (now-stage_at > 3 and vtol == 3):
                        raise RuntimeError('Front transition failed or timed out')
                elif stage == 'fw':
                    if vtol != 4:
                        if case.get('expected_quadchute') and now-stage_at > .8:
                            if fw_att_stopped:
                                cli('fw_att_control', 'start', 'vtol')
                                fw_att_stopped = False
                            throttle, recovery_alt, cycle = .5, alt, 1
                            cli('commander', 'mode', 'auto:loiter')
                            event('expected missing-output quad-chute')
                            stage, stage_at = 'back', now
                            continue
                        raise RuntimeError('Unexpected FW exit')
                    if fw_att_stopped and now-stage_at > case['delay_fw_ms']/1000:
                        cli('fw_att_control', 'start', 'vtol')
                        fw_att_stopped = False
                        event('FW attitude publications resumed')
                    if not rapid and case.get('rapid') and now-stage_at > .2:
                        throttle = 1.
                        rapid = True
                        event('rapid full-throttle request')
                    if 'fw_stick' in case and not fw_stick_applied and now-stage_at > case.get('fw_stick_after', 3):
                        throttle = case['fw_stick']
                        cli('commander', 'mode', 'stabilized')
                        fw_stick_applied = True
                        event('FW manual throttle request', throttle=throttle)
                    if now-stage_at > case.get('fw_seconds', 12):
                        if case.get('delay_mc_ms'):
                            cli('mc_att_control', 'stop')
                            mc_att_stopped = True
                            event('MC attitude publications stopped')
                        transition(3, case.get('immediate_back', False))
                        throttle = .5
                        recovery_alt = alt
                        cli('commander', 'mode', 'auto:loiter')
                        cycle += 1
                        stage, stage_at = 'back', now
                elif stage == 'back':
                    if mc_att_stopped and now-stage_at > case['delay_mc_ms']/1000:
                        cli('mc_att_control', 'start', 'vtol')
                        mc_att_stopped = False
                        event('MC attitude publications resumed')
                    if case.get('abort_back') and not back_aborted and now-stage_at > .8 and vtol == 2:
                        transition(4)
                        back_aborted = True
                        event('back-transition aborted into FW')
                        cli('commander', 'mode', 'altctl')
                        stage, stage_at = 'front', now
                        continue
                    if vtol == 3:
                        stage, stage_at, stable = 'recover', now, None
                    elif now-stage_at > 12:
                        raise RuntimeError('Back transition timeout')
                if now-last_print > 10:
                    frame_period = read('sim/operation/misc/frame_rate_period')
                    event('state', alt=round(alt, 2), cas=latest.get('VFR_HUD', {}).get('airspeed'), rate=round(rate, 2), frame_period=frame_period)
                    if not 0 < frame_period < .04:
                        raise RuntimeError('Simulator timing guard during flight')
                    last_print = now
                    stream.flush()
    except (Exception, KeyboardInterrupt) as error:
        interrupted = isinstance(error, KeyboardInterrupt)
        reason = str(error) or type(error).__name__
        event('failed', reason=reason)
    finally:
        disarmed = False
        try:
            cli('commander', 'disarm', '-f')
            time.sleep(.2)
            disarmed = 'Disarmed' in cli('commander', 'status')
            if fw_att_stopped:
                cli('fw_att_control', 'start', 'vtol')
            if mc_att_stopped:
                cli('mc_att_control', 'start', 'vtol')
        finally:
            try:
                if 'dragonfly/handoff_test/seconds_remaining' in refs:
                    set_ref('dragonfly/handoff_test/seconds_remaining', 0)
            finally:
                command('sim/operation/pause_on')
        event('disarmed and paused', confirmed_disarmed=disarmed)
        result = dict(passed=passed, reason=reason, fw_entries=fw_count, completed_cycles=cycle, events=events)
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        console = SITL / 'tests' / (label + '-px4-console.txt')
        shutil.copy2(console, out / 'console.txt')
        log_paths = re.findall(r'Opened full log file: (.+\.ulg)', console.read_text())
        if log_paths:
            source = SITL / 'rootfs' / log_paths[-1]
            shutil.copy2(source, out / 'flight.ulg')
        (out / 'parameters-after.txt').write_text(cli('param', 'show', '-a'))
        for topic in ('tailsitter_handoff', 'tailsitter_handoff_ack'):
            (out / (topic + '.txt')).write_text(cli('listener', topic, '-n', 1))
        link.close()
    if interrupted:
        raise KeyboardInterrupt
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    results = []
    for case in json.loads(args.cases.read_text()):
        try:
            result = run_case(case, args.output)
        except Exception as error:
            out = args.output / case['name']
            result = dict(passed=False, reason='Harness/setup error: '+str(error), fw_entries=0)
            if not (out / 'result.json').exists():
                (out / 'result.json').write_text(json.dumps(result, indent=2))
            commands = xp.catalog('commands')
            xp.request('/command/%s/activate' % commands['sim/operation/pause_on']['id'], {'duration': 0}, 'POST')
            console = SITL / 'tests' / (args.output.name+'-'+case['name']+'-px4-console.txt')
            if console.exists():
                shutil.copy2(console, out / 'console.txt')
                logs = re.findall(r'Opened full log file: (.+\.ulg)', console.read_text())
                if logs:
                    shutil.copy2(SITL/'rootfs'/logs[-1], out/'flight.ulg')
            raise
        results.append(dict(name=case['name'], passed=result['passed'], reason=result['reason'], fw_entries=result['fw_entries']))
        (args.output / 'summary.json').write_text(json.dumps(results, indent=2))
        if not result['passed'] and (not result['fw_entries'] or 'Simulator timing guard' in result['reason']):
            raise RuntimeError('Batch stopped after failure before handoff or invalid simulator timing: '+result['reason'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
