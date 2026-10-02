#!/usr/bin/env python3
"""Measure logged tailsitter handoff continuity independently of flight stability."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def analyze(path):
    log = ULog(str(path))
    def data(name, instance=0):
        return log.get_dataset(name, instance).data
    try:
        requests = data('tailsitter_handoff')
    except (KeyError, IndexError):
        return dict(log=str(path), handoffs=[], reason='No logged handoff request')
    case_path = path.parent / 'case.json'
    case = json.loads(case_path.read_text()) if case_path.exists() else {}
    results = []
    for request in sorted(set(map(int, requests['handoff_id'])) - {0}):
        rows = np.flatnonzero(requests['handoff_id'] == request)
        row = rows[0]
        active_rows = rows[requests['active'][rows] != 0]
        active = int(requests['timestamp'][active_rows[0]]) if len(active_rows) else None
        cancelled = np.flatnonzero((requests['timestamp'] > request) & (requests['handoff_id'] == 0))
        cancel = int(requests['timestamp'][cancelled[0]]) if len(cancelled) else int(requests['timestamp'][-1])
        fault = bool(case.get('expected_quadchute') and active is None)
        target = [float(requests[f'torque[{i}]'][row]) for i in range(3)]
        thrust = float(requests['thrust'][row])
        result = dict(request_us=request, active_us=active,
                      ready_delay_ms=(active-request)/1000 if active else None,
                      outgoing_torque=target, outgoing_thrust=thrust,
                      learned_bias_mc=[float(requests[f'torque_bias[{i}]'][row]) for i in range(3)])
        airspeed = data('airspeed_validated')
        speed_index = int(np.argmin(np.abs(airspeed['timestamp'].astype(np.int64)-request)))
        result['handoff_cas_mps'] = float(airspeed['calibrated_airspeed_m_s'][speed_index])
        result['handoff_cas_sample_offset_ms'] = (int(airspeed['timestamp'][speed_index])-request)/1000
        passed = active is not None
        if fault:
            status = data('vtol_vehicle_status')
            failures = np.flatnonzero((status['timestamp'] >= request) & status['fixed_wing_system_failure'])
            delay = (int(status['timestamp'][failures[0]])-request)/1e6 if len(failures) else None
            result['missing_output_quadchute_seconds'] = delay
            result['expected_fault'] = True
            passed = delay is not None and .99 <= delay <= 1.2
        for name, cols, expected in [
            ('vehicle_torque_setpoint', [f'xyz[{i}]' for i in range(3)], target),
            ('vehicle_thrust_setpoint', ['xyz[2]'], [-thrust]),
            ('actuator_motors', [f'control[{i}]' for i in range(4)], None),
        ]:
            d = data(name)
            values = np.column_stack([d[c] for c in cols])
            index = np.searchsorted(d['timestamp'], request)
            if index == 0 or index >= len(values):
                passed = False
                result[name] = dict(error='Missing samples across the switch')
                continue
            step = values[index] - values[index-1]
            accepted_index = np.searchsorted(d['timestamp'], active) if active else len(values)
            accepted_step = values[accepted_index] - values[accepted_index-1] if 0 < accepted_index < len(values) else None
            first_after = np.searchsorted(d['timestamp'], active, side='right') if active else len(values)
            first_active_step = values[first_after] - values[first_after-1] if 0 < first_after < len(values) else None
            hold = (d['timestamp'] >= request) & (d['timestamp'] < cancel if fault else d['timestamp'] <= (active or request))
            target_value = np.asarray(expected) if expected is not None else values[index-1]
            held_error = float(np.max(np.abs(values[hold] - target_value))) if np.any(hold) else None
            window = (d['timestamp'] >= request) & (d['timestamp'] <= request + 500000)
            increments = np.diff(values[window], axis=0)
            max_step = float(np.max(np.abs(increments))) if increments.size else None
            finite = bool(np.all(np.isfinite(values[window])))
            match = bool(np.max(np.abs(step)) < 1e-5 and held_error is not None and held_error < 1e-5 and finite
                         and (fault or (accepted_step is not None and np.max(np.abs(accepted_step)) < 1e-5
                         and first_active_step is not None and np.max(np.abs(first_active_step)) < 1e-5)))
            result[name] = dict(switch_step=step.tolist(), max_hold_error=held_error,
                                acceptance_step=accepted_step.tolist() if accepted_step is not None else None,
                                first_active_step=first_active_step.tolist() if first_active_step is not None else None,
                                max_step_first_half_second=max_step, finite=finite, continuity_passed=match)
            passed = passed and match
        result['continuity_passed'] = passed
        try:
            feedback = data('tailsitter_handoff_status')
            f = feedback['handoff_id'] == request
            done = np.flatnonzero(f & (feedback['throttle_slew_active'] == 0))
            end = int(feedback['timestamp'][done[0]]) if len(done) else None
            result['slew_completion_ms'] = (end-request)/1000 if end else None
            result['slew_completed'] = end is not None
            # Check the actual rate-controller output against the sensor dt it uses.
            # Separately report routed/motor steps above: asynchronous routing can repeat
            # one command then skip to the next, so publication-time slopes are not the
            # controller's integration interval.
            d = data('vehicle_thrust_setpoint_virtual_fw')
            selected = (d['timestamp'] >= (active or request)) & (d['timestamp'] <= (end or request+5000000))
            times = d['timestamp_sample'][selected].astype(float)
            values = d['xyz[0]'][selected]
            dt = np.clip(np.diff(times) / 1e6, .002, .04)
            increments = np.diff(values)
            slope = increments / dt
            fall, rise = case.get('throttle_slew', .2), 2*case.get('throttle_slew', .2)
            result['slew_max_fall_per_s'] = float(max(0., -np.min(slope))) if len(slope) else None
            result['slew_max_rise_per_s'] = float(max(0., np.max(slope))) if len(slope) else None
            result['slew_fall_bound_passed'] = bool(len(slope) and np.all(increments >= -fall*dt-1e-5))
            protected = False
            try:
                tecs = data('tecs_status')
                ts = (tecs['timestamp'] >= request) & (tecs['timestamp'] <= (end or request+5000000))
                protected = bool(np.any(tecs['underspeed_ratio'][ts] > 1e-6))
            except (KeyError, IndexError):
                pass
            result['underspeed_during_slew'] = protected
            result['slew_rise_bound_passed'] = protected or bool(len(slope) and np.all(increments <= rise*dt+1e-5))
            result['slew_passed'] = (end is not None and result['slew_fall_bound_passed']
                                     and result['slew_rise_bound_passed'])
        except (KeyError, IndexError):
            result['slew_passed'] = None
        if fault:
            result['slew_passed'] = None  # No FW controller was accepted.
        if case.get('mission') and 'speed_change_mph' in case:
            commands = data('vehicle_command')
            speed_commands = np.flatnonzero(commands['command'] == 178)
            target_speed = case['speed_change_mph']*.44704
            command_time = int(commands['timestamp'][speed_commands[-1]]) if len(speed_commands) else None
            ack = data('vehicle_command_ack')
            accepted = bool(command_time and np.any((ack['command'] == 178) & (ack['timestamp'] >= command_time) & (ack['result'] == 0)))
            tecs = data('tecs_status')
            samples = (tecs['timestamp'] >= request) & (tecs['timestamp'] < cancel)
            speed = tecs['equivalent_airspeed_sp'][samples]
            times = tecs['timestamp'][samples].astype(float)
            slope = np.abs(np.diff(speed) / (np.diff(times)/1e6))
            reached = bool(command_time and np.any((tecs['timestamp'] >= max(request, command_time))
                         & (tecs['timestamp'] < cancel) & (np.abs(tecs['equivalent_airspeed_sp']-target_speed) < .01)))
            tail = (tecs['timestamp'] >= cancel-2000000) & (tecs['timestamp'] < cancel)
            held = bool(np.any(tail) and np.all(np.abs(tecs['equivalent_airspeed_sp'][tail]-target_speed) < .01))
            source = data('fixed_wing_longitudinal_setpoint')
            source_resets = np.flatnonzero(np.isfinite(source['equivalent_airspeed'][:-1])
                                          & ~np.isfinite(source['equivalent_airspeed'][1:]))+1
            source_resets = source_resets[(source['timestamp'][source_resets] >= (command_time or request))
                                         & (source['timestamp'][source_resets] < cancel)]
            result['mission_speed'] = dict(command_accepted=accepted, target_reached=reached,
                target_held_last_two_seconds=held,
                upstream_target_reset_after_handoff_ms=[(int(source['timestamp'][i])-request)/1000 for i in source_resets],
                commanded_mps=target_speed, command_after_handoff_ms=(command_time-request)/1000 if command_time else None,
                max_logged_reference_slew_mps2=float(max(slope)) if len(slope) else None)
        if 'stick' in case and not case.get('rapid'):
            d = data('vehicle_thrust_setpoint_virtual_fw')
            selected = (d['timestamp'] >= (active or request)) & (d['timestamp'] <= request+500000)
            dt = np.clip(np.diff(d['timestamp_sample'][selected].astype(float)) / 1e6, .002, .04)
            slope = np.diff(d['xyz[0]'][selected]) / dt
            result['constant_stick_max_rise_per_s'] = float(max(0, np.max(slope))) if len(slope) else None
            result['constant_stick_max_fall_per_s'] = float(max(0, -np.min(slope))) if len(slope) else None
            rate = case.get('throttle_slew', .2)
            result['constant_stick_ramp_passed'] = bool(len(slope) and np.max(slope) <= 2*rate+1e-4 and np.min(slope) >= -rate-1e-4)
            result['continuity_passed'] = passed and result['constant_stick_ramp_passed']
        if case.get('rapid'):
            pilot = data('manual_control_setpoint')
            full = np.flatnonzero((pilot['timestamp'] >= request) & (pilot['throttle'] >= .999))
            d = data('vehicle_thrust_setpoint')
            reached = np.flatnonzero((d['timestamp'] >= request) & (-d['xyz[2]'] >= .999))
            if len(full) and len(reached):
                result['full_throttle_latency_ms'] = (int(d['timestamp'][reached[0]])-int(pilot['timestamp'][full[0]]))/1000
        allocation = data('control_allocator_status')
        allocated = (allocation['timestamp'] >= request) & (allocation['timestamp'] <= request+500000)
        residual = np.column_stack([allocation[f'unallocated_torque[{i}]'][allocated] for i in range(3)])
        result['max_unallocated_torque_first_half_second'] = float(np.max(np.abs(residual))) if residual.size else None
        angular = data('vehicle_angular_velocity')
        select = (angular['timestamp'] >= request) & (angular['timestamp'] <= request + 1000000)
        result['max_rate_first_second_dps'] = float(np.rad2deg(np.max(np.abs(
            np.column_stack([angular[f'xyz[{i}]'][select] for i in range(3)]))))) if np.any(select) else None
        results.append(result)
    return dict(log=str(path), handoffs=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = [analyze(path) for path in args.logs]
    args.output.write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
    for result in results:
        print(result['log'], [(h['continuity_passed'], h['ready_delay_ms']) for h in result['handoffs']])


if __name__ == '__main__':
    main()
