#!/usr/bin/env python3
"""Measure logged tailsitter handoff continuity independently of flight stability."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def back_transitions(log, case):
    """Report the whole back-transition, including the second change of thrust demand."""
    vtol = log.get_dataset('vtol_vehicle_status').data
    state = vtol['vehicle_vtol_state']
    starts = np.flatnonzero((state[1:] == 2) & (state[:-1] != 2)) + 1
    request = log.get_dataset('tailsitter_handoff').data
    feedback = log.get_dataset('tailsitter_handoff_status').data
    thrust = log.get_dataset('vehicle_thrust_setpoint_virtual_mc').data
    result = []
    for start in starts:
        begin = int(vtol['timestamp'][start])
        ends = np.flatnonzero((vtol['timestamp'] > begin) & (state != 2))
        end = int(vtol['timestamp'][ends[0]]) if len(ends) else None
        ids = np.flatnonzero((request['timestamp'] >= begin-20000) & (request['timestamp'] <= begin+20000)
                            & (request['handoff_id'] != 0) & request.get('to_mc', np.zeros(len(request['timestamp']))))
        identity = int(request['handoff_id'][ids[0]]) if len(ids) else None
        releases = np.flatnonzero((request['timestamp'] > begin) & (request['handoff_id'] == 0))
        normal_release = bool(len(releases) and not request.get('to_mc', np.zeros(len(request['timestamp'])))[releases[0]])
        item = dict(start_us=begin, end_us=end, matching_requested=identity is not None,
                    transition_seconds=(end-begin)/1e6 if end else None,
                    completed=bool(len(ends) and state[ends[0]] == 3), normal_release=normal_release)
        for name, fields in [('vehicle_thrust_setpoint', ['xyz[2]']),
                             ('vehicle_torque_setpoint', [f'xyz[{i}]' for i in range(3)]),
                             ('actuator_motors', [f'control[{i}]' for i in range(4)])]:
            d = log.get_dataset(name).data
            values = np.column_stack([d[k] for k in fields])
            row = {}
            for label, t in [('start', begin), ('completion', end)]:
                index = np.searchsorted(d['timestamp'], t) if t else 0
                row[label+'_step'] = (values[index]-values[index-1]).tolist() if 0 < index < len(values) else None
                selected = (d['timestamp'] >= (t or 0)-10000) & (d['timestamp'] <= (t or 0)+100000)
                increments = np.diff(values[selected], axis=0)
                row[label+'_max_step_100ms'] = float(np.max(np.abs(increments))) if increments.size else None
            item[name] = row
        # Check both one-shot acquisitions, stopping each window at its last logged
        # active sample. Commands after release are deliberately unrestricted.
        f = (feedback['handoff_id'] == identity) if identity else np.zeros(len(feedback['timestamp']), dtype=bool)
        active_indices = np.flatnonzero(f & feedback['throttle_slew_active'])
        segments = np.split(active_indices, np.flatnonzero(np.diff(active_indices) > 1)+1)
        item['slew_phases'] = []
        for segment in segments:
            if not len(segment):
                continue
            lo, hi = int(feedback['timestamp'][segment[0]]), int(feedback['timestamp'][segment[-1]])
            selected = (thrust['timestamp'] >= lo) & (thrust['timestamp'] <= hi)
            dt = np.clip(np.diff(thrust['timestamp_sample'][selected].astype(float))/1e6, .000125, .02)
            delta = np.diff(-thrust['xyz[2]'][selected])
            fall = case.get('back_throttle_slew', 1.)
            # Older recorded runs used a 2:1 rise/fall ratio. New runs record both rates.
            rise = case.get('back_throttle_rise', 2*fall)
            item['slew_phases'].append(dict(start_after_request_s=(lo-begin)/1e6,
                logged_active_duration_s=(hi-lo)/1e6,
                fall_bound_passed=bool(np.all(delta >= -fall*dt-1e-5)),
                rise_bound_passed=bool(np.all(delta <= rise*dt+1e-5))))
        item['normal_handoff_exercised'] = identity is not None and item['completed'] and normal_release
        result.append(item)
    return result


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
        to_mc = bool(requests.get('to_mc', np.zeros(len(requests['timestamp'])))[row])
        active_rows = rows[requests['active'][rows] != 0]
        active = int(requests['timestamp'][active_rows[0]]) if len(active_rows) else None
        cancelled = np.flatnonzero((requests['timestamp'] > request) & (requests['handoff_id'] != request))
        cancel = int(requests['timestamp'][cancelled[0]]) if len(cancelled) else int(requests['timestamp'][-1])
        fault = bool(case.get('expected_quadchute') and active is None)
        target = [float(requests[f'torque[{i}]'][row]) for i in range(3)]
        thrust = float(requests['thrust'][row])
        result = dict(request_us=request, active_us=active, direction='FW→MC' if to_mc else 'MC→FW',
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
            d = data('vehicle_thrust_setpoint_virtual_mc' if to_mc else 'vehicle_thrust_setpoint_virtual_fw')
            selected = (d['timestamp'] >= (active or request)) & (d['timestamp'] <= (end or request+5000000))
            times = d['timestamp_sample'][selected].astype(float)
            values = -d['xyz[2]'][selected] if to_mc else d['xyz[0]'][selected]
            dt = np.clip(np.diff(times) / 1e6, .000125 if to_mc else .002, .02 if to_mc else .04)
            increments = np.diff(values)
            slope = increments / dt
            fall = case.get('back_throttle_slew', 1.) if to_mc else case.get('throttle_slew', .2)
            rise = case.get('back_throttle_rise', 2*fall) if to_mc else 2*fall
            result['slew_max_fall_per_s'] = float(max(0., -np.min(slope))) if len(slope) else None
            result['slew_max_rise_per_s'] = float(max(0., np.max(slope))) if len(slope) else None
            result['slew_fall_bound_passed'] = bool(len(slope) and np.all(increments >= -fall*dt-1e-5))
            protected = False
            try:
                tecs = data('tecs_status')
                ts = (tecs['timestamp'] >= request) & (tecs['timestamp'] <= (end or request+5000000))
                protected = not to_mc and bool(np.any(tecs['underspeed_ratio'][ts] > 1e-6))
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
        if not to_mc and case.get('mission') and 'speed_change_mph' in case:
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
        if not to_mc and 'stick' in case and not case.get('rapid'):
            d = data('vehicle_thrust_setpoint_virtual_fw')
            selected = (d['timestamp'] >= (active or request)) & (d['timestamp'] <= request+500000)
            dt = np.clip(np.diff(d['timestamp_sample'][selected].astype(float)) / 1e6, .002, .04)
            slope = np.diff(d['xyz[0]'][selected]) / dt
            result['constant_stick_max_rise_per_s'] = float(max(0, np.max(slope))) if len(slope) else None
            result['constant_stick_max_fall_per_s'] = float(max(0, -np.min(slope))) if len(slope) else None
            rate = case.get('throttle_slew', .2)
            result['constant_stick_ramp_passed'] = bool(len(slope) and np.max(slope) <= 2*rate+1e-4 and np.min(slope) >= -rate-1e-4)
            result['continuity_passed'] = passed and result['constant_stick_ramp_passed']
        if not to_mc and case.get('rapid'):
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
    return dict(log=str(path), handoffs=results, back_transitions=back_transitions(log, case))


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
