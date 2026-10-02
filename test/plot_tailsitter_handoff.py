#!/usr/bin/env python3
"""Plot actual motor commands around ordinary handoffs from saved ULogs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

from analyze_tailsitter_handoff import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--back', action='store_true', help='Plot the first FW to MC handoff')
    args = parser.parse_args()
    fig, axes = plt.subplots(4, len(args.logs), figsize=(max(8., 4.4*len(args.logs)), 10), sharex='col', squeeze=False)
    colours = ['#2670bb', '#d25b24', '#389357', '#905eb5']
    for column, path in enumerate(args.logs):
        log = ULog(str(path))
        analysis = analyze(path)
        info = next(h for h in analysis['handoffs'] if (h['direction'] == 'FW→MC') == args.back)
        origin = info['request_us']
        ready = info['ready_delay_ms']/1000 if info['ready_delay_ms'] is not None else 1
        done = (info.get('slew_completion_ms') or 0)/1000
        def draw(row, name, fields, labels, multipliers=None, dashed=False):
            try:
                data = log.get_dataset(name).data
            except (KeyError, IndexError):
                return
            t = (data['timestamp'].astype(float)-origin)/1e6
            mask = (t >= -.25) & (t <= args.seconds)
            for index, (field, label) in enumerate(zip(fields, labels)):
                value = data[field] * (multipliers[index] if multipliers else 1)
                if row == 0 and dashed and not args.back:
                    value = np.where(np.isfinite(value), np.where(value > .02, value, 0), np.nan)
                axes[row, column].plot(t[mask], value[mask], label=label, lw=1.1,
                    color='#a13c3c' if dashed else colours[index], ls='--' if dashed else '-')
        case = json.loads((path.parent / 'case.json').read_text())
        if args.back:
            draw(0, 'vehicle_rates_setpoint_virtual_mc', ['thrust_body[2]'], ['MC demand'], [-1], dashed=True)
        else:
            demand_topic, demand_field = ('vehicle_rates_setpoint_virtual_fw', 'thrust_body[0]') if 'stick' in case else ('tecs_status', 'throttle_sp')
            draw(0, demand_topic, [demand_field], ['FW demand¹'], dashed=True)
        draw(0, 'vehicle_thrust_setpoint', ['xyz[2]'], ['Applied collective'], [-1])
        draw(1, 'vehicle_torque_setpoint', [f'xyz[{i}]' for i in range(3)], ['MC roll', 'MC pitch', 'MC yaw'])
        draw(2, 'actuator_motors', [f'control[{i}]' for i in range(4)], ['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'])
        draw(3, 'vehicle_angular_velocity', [f'xyz[{i}]' for i in range(3)], ['MC roll', 'MC pitch', 'MC yaw'], [180/np.pi]*3)
        for row in range(4):
            ax = axes[row, column]
            ax.axvspan(0, ready, color='#f4c34e', alpha=.3)
            if done > ready:
                ax.axvspan(ready, done, color='#8cb9e8', alpha=.18)
            ax.axvline(0, color='#777777', lw=.7)
            if args.back:
                for back in analysis['back_transitions']:
                    if back['end_us'] and abs(back['start_us']-origin) < 20000:
                        ax.axvline((back['end_us']-origin)/1e6, color='#389357', lw=.9, ls=':')
            ax.set_xlim(-.25, args.seconds)
            ax.grid(alpha=.15)
            ax.legend(fontsize=7, loc='best', ncol=2)
        title = path.parent.name.replace('-slew-', '\nslew ')
        if args.back:
            fall = case.get('back_throttle_slew', 1.)
            title += '\nRise/fall slew: %g/%g/s' % (case.get('back_throttle_rise', 2*fall), fall)
        axes[0, column].set_title(title, fontsize=10)
        axes[3, column].set_xlabel('Time from '+ ('FW → MC' if args.back else 'MC → FW') + ' switch (s)')
    for row, label in enumerate(['Collective command', 'Torque command', 'Motor command', 'Body rates (deg/s)']):
        axes[row, 0].set_ylabel(label)
    fig.suptitle('Ordinary Dragonfly handoffs — measured commands and response', fontsize=14)
    note = ('Green dotted line: transition trajectory ends and ordinary MC targets resume.' if args.back else
            '¹ FW demand includes the existing 2% motor-stop deadband; battery scaling is disabled in these runs.')
    fig.text(.5, .01, 'Yellow: waiting for fresh controller outputs (measured duration). Blue: initial throttle acquisition.\n'
             + note, ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .045, 1, .96))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for suffix in ['.png', '.pdf']:
        fig.savefig(args.output.with_suffix(suffix), dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    main()
