#!/usr/bin/env python3
"""Bounded RTL checks; retains logs, floating references, hashes and failures.

Uses only Python stdlib, Icarus Verilog and Verilator. Does not operate a board.
The float reference uses mathematical sin/cos and unquantized PI coefficients,
so agreement checks numerical error rather than repeating RTL integer steps.
"""
import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import random
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
RTL = HERE.parent / 'rtl'
SOURCES = [RTL / name for name in (
    'focmini_math.sv', 'focmini_pi.sv', 'focmini_sincos.sv',
    'focmini_current_scale.sv', 'focmini_control.sv', 'focmini_driver.sv')]


def q(value):
    return round(value * 65536)


def clip(value, limit):
    return max(-limit, min(limit, value))


class PI:
    def __init__(self, kp, ki, period, limit):
        self.kp, self.half, self.limit = kp, ki * period / 2, limit
        self.integral = self.previous = self.output = 0.0

    def step(self, error):
        self.integral = clip(self.integral + self.half * (error + self.previous), self.limit)
        self.previous = error
        self.output = clip(self.kp * error + self.integral, self.limit)
        return self.output


def vectors(path):
    """Exercise phase quadrants, encoder wrap, reverse direction and PI recovery."""
    rng = random.Random(2805)
    count = 0
    with path.open('w') as out:
        for reverse, zero, initial, corners in ((0, 0, 4090, False), (0, 16384, 1020, False),
                                       (1, 55000, 4, False), (1, 0, 3070, False),
                                       (0, 20000, 1000, True), (1, 0, 2000, True)):
            speed_pi, d_pi, q_pi = PI(.5, 10, .001, .5), PI(3, 300, .0001, 1.5), PI(3, 300, .0001, 1.5)
            filtered_d = filtered_q = velocity = 0.0
            angle, previous, accumulated = initial, None, 0
            current_alpha = .0001 / (.0001 + 1 / (2 * math.pi * 300))
            for step in range(128):
                angle = (angle + (1 if step < 48 else -1 if step < 96 else 0)) % 4096
                ia = q(.2 * math.sin(step * .17) + rng.uniform(-.01, .01)) / 65536
                ib = q(.2 * math.cos(step * .13) + rng.uniform(-.01, .01)) / 65536
                if corners:
                    # All six vertices of the valid 2.5A three-phase hexagon.
                    ia, ib = ((2.5, 0), (0, 2.5), (-2.5, 2.5),
                              (-2.5, 0), (0, -2.5), (2.5, -2.5))[(step // 8) % 6]
                target = (1.0 if step < 32 else -1.0 if step < 64 else .05 if step < 96 else 5.0)
                due = step == 0 or step % 10 == 0
                if previous is not None:
                    delta = ((angle - previous + 2048) % 4096) - 2048
                    accumulated += -delta if reverse else delta
                if due and step:
                    raw_speed = accumulated * 2 * math.pi * 1000 / 4096
                    velocity += (raw_speed - velocity) / 6
                    accumulated = 0
                previous = angle
                turns = (((-angle if reverse else angle) * 16 * 7) - zero) % 65536
                theta = turns * 2 * math.pi / 65536
                sn, cs = math.sin(theta), math.cos(theta)
                beta = (ia + 2 * ib) / math.sqrt(3)
                measured_d, measured_q = ia * cs + beta * sn, beta * cs - ia * sn
                filtered_d += current_alpha * (measured_d - filtered_d)
                filtered_q += current_alpha * (measured_q - filtered_q)
                if due:
                    speed_pi.step(target - velocity)
                vd, vq = d_pi.step(-filtered_d), q_pi.step(speed_pi.output - filtered_q)
                ua, ubeta = vd * cs - vq * sn, vd * sn + vq * cs
                ub, uc = -.5 * ua + math.sqrt(3) / 2 * ubeta, -.5 * ua - math.sqrt(3) / 2 * ubeta
                duty = [int(max(0, min(2500, (u + 4.5) * 2500 / 9))) for u in (ua, ub, uc)]
                values = [int(step == 0), angle, zero, reverse, q(ia), q(ib), q(target),
                          q(filtered_d), q(filtered_q), q(velocity), q(speed_pi.output), q(vd), q(vq), *duty]
                out.write(' '.join(map(str, values)) + '\n')
                count += 1
    return count


def check_profile():
    profile = json.loads((HERE.parent / 'profile.json').read_text())
    source = (ROOT / profile['source_file']).read_text()
    constants = {
        'bus_v': 'kBusVoltageV', 'pole_pairs': 'kMotorPolePairs',
        'pwm_hz': 'kPwmFrequencyHz', 'dq_axis_voltage_limit_v': 'kMotorVoltageAmplitudeV',
        'iq_reference_limit_a': 'kMotorCurrentLimitA',
        'velocity_reference_limit_rad_s': 'kVelocityLimitRadPerSec',
        'sensor_sensitivity_mv_a': 'kAcs72405AbSensitivityMilliVPerA',
        'esp32_divider_top_ohm': 'kCurrentDividerTopOhms',
        'esp32_divider_bottom_ohm': 'kCurrentDividerBottomOhms',
        'alignment_voltage_v_from_esp32': 'kAlignmentVoltageV'}
    for key, constant in constants.items():
        match = re.search(r'\b' + constant + r'\s*=\s*([0-9.]+)f?\s*;', source)
        if match is None or float(match[1]) != profile[key]:
            raise ValueError(f'ESP32/profile mismatch: {constant} / {key}')
    return {key: profile[key] for key in constants}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=ROOT / 'results/arty_focmini')
    parser.add_argument('--plant', action='store_true', help='Also run closed-loop PWM/motor fixtures with Verilator and a C++ compiler')
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S-')
    out = Path(tempfile.mkdtemp(prefix=stamp, dir=args.output_root)).resolve()
    report = {'status': 'running', 'commands': [], 'source_sha256': {}, 'tests': {}}

    def command(name, cmd, timeout=120, allow_failure=False):
        process = subprocess.run(list(map(str, cmd)), cwd=ROOT, text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        (out / (name + '.log')).write_text(process.stdout)
        report['commands'].append({'name': name, 'argv': list(map(str, cmd)), 'returncode': process.returncode})
        if process.returncode and not allow_failure:
            raise RuntimeError(f'{name} failed; see {out / (name + ".log")}')
        return process.stdout.strip()

    try:
        report['tests']['source_profile'] = check_profile()
        report['source_head'] = command('git-head', ['git', 'rev-parse', 'HEAD'])
        report['source_status'] = command('git-status', ['git', 'status', '--short'])
        report['submodules'] = command('git-submodules', ['git', 'submodule', 'status'])
        report['tools'] = {name: command(name + '-version', cmd) for name, cmd in (
            ('python', ['python3', '--version']), ('iverilog', ['iverilog', '-V']),
            ('verilator', ['verilator', '--version']))}
        all_files = sorted(p for p in HERE.parent.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
        all_files += [ROOT / 'hw/focmini/src/main.cpp', ROOT / 'hw/focmini/platformio.ini']
        for source in all_files:
            relative = source.relative_to(ROOT)
            report['source_sha256'][str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
            snapshot = out / 'source' / relative
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            snapshot.write_bytes(source.read_bytes())
        report['tests']['float_reference_frames'] = vectors(out / 'vectors.txt')
        # Compile the exact immutable snapshots hashed above.
        source_files = [out / 'source' / p.relative_to(ROOT) for p in SOURCES]
        test_dir = out / 'source' / HERE.relative_to(ROOT)
        # Warn-free lint, independently of Icarus elaboration/simulation.
        command('lint-driver', ['verilator', '--lint-only', '--top-module', 'focmini_driver', '-Wall', *source_files])
        command('lint-scale', ['verilator', '--lint-only', '--top-module', 'focmini_current_scale', '-Wall', *source_files])
        for suite in ('math', 'control', 'driver'):
            top = 'tb_' + suite
            command('compile-' + suite, ['iverilog', '-g2012', '-s', top, '-o', out / (top + '.vvp'),
                                        *source_files, test_dir / (top + '.sv')])
            result = command(suite, ['vvp', out / (top + '.vvp'), '+vectors=' + str(out / 'vectors.txt')])
            if 'PASS ' + suite not in result:
                raise RuntimeError(f'Missing {suite} completion')
            report['tests'][suite] = result
        if args.plant:
            report['plant_gates'] = {'tail_abs_mean_error_rad_s': .15, 'tail_rmse_rad_s': .4,
                                     'peak_phase_current_a': 1.0, 'peak_speed_rad_s': 5.0,
                                     'note': 'Assumed fixtures, not measured motor qualification'}
            command('compile-plant', ['verilator', '--cc', '--exe', '--build', '-j', '1', '-O3', '-Wall',
                    '--top-module', 'focmini_driver', '--Mdir', out / 'obj',
                    *source_files, test_dir / 'plant.cpp', '-CFLAGS', '-O3'], timeout=300)
            report['tests']['plant'] = []
            for fixture in ('nominal', 'heavy_reverse'):
                result = command('plant-' + fixture, [out / 'obj/Vfocmini_driver', fixture,
                                  out / ('plant-' + fixture + '.csv')], timeout=600, allow_failure=True)
                measured = json.loads(result.splitlines()[-1])
                report['tests']['plant'].append(measured)
            if any(item['status'] != 'passed' for item in report['tests']['plant']):
                raise RuntimeError('Closed-loop fixture tracking gate failed; inspect per-fixture metrics')
        command('diff-check', ['git', 'diff', '--check'])
        report['status'] = 'passed'
    except Exception as error:
        report.update(status='failed', error=str(error))
    finally:
        (out / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'status': report['status'], 'evidence': str(out), 'error': report.get('error')}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
