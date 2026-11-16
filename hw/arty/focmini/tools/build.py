#!/usr/bin/env python3
"""Vivado out-of-context verification for Arty A7-35T; never programs hardware.

Uses an immutable RTL snapshot, a 100MHz clock and registered launch/capture
boundaries. Acceptance covers register-to-register timing; external IO reports
use diagnostic zero-delay assumptions and are not accepted as board timing.
Raw logs and failed runs are preserved.
"""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
MODULE_DIR = Path(__file__).resolve().parents[1]


def fixture(path):
    """Registered boundaries keep every DUT signal variable and observable."""
    inputs = {'reset': 1, 'fault_n': 1, 'arm': 1, 'stop': 1, 'target_valid': 1,
              'target_velocity': 32, 'calibration_valid': 1, 'angle_reverse': 1,
              'electrical_zero': 16, 'phase_trip': 32, 'sample_valid': 1,
              'sensor_ok': 1, 'angle_raw': 12, 'phase_a': 32, 'phase_b': 32}
    outputs = {'sample_request': 1, 'pwm_a': 1, 'pwm_b': 1, 'pwm_c': 1, 'enable': 1,
               'sleep_n': 1, 'reset_n': 1, 'ready': 1, 'armed': 1, 'fault_latched': 1,
               'command_rejected': 1, 'fault_code': 4, 'velocity': 32, 'current_d': 32,
               'current_q': 32, 'reference_q': 32, 'voltage_d': 32, 'voltage_q': 32}
    width = lambda n: '' if n == 1 else f'[{n-1}:0] '
    ports = ['input wire clk'] + [f'input wire {width(n)}{s}' for s, n in inputs.items()]
    ports += [f'output reg {width(n)}{s}' for s, n in outputs.items()]
    lines = ['// Timing fixture only; never programmed or used as motor interface.',
             'module focmini_impl_probe(' + ','.join(ports) + ');']
    for s, n in inputs.items():
        lines += [f'(* DONT_TOUCH="TRUE" *) reg {width(n)}q_{s};', f'always @(posedge clk) q_{s}<={s};']
    for s, n in outputs.items():
        lines += [f'wire {width(n)}d_{s};', f'always @(posedge clk) {s}<=d_{s};']
    connections = ['.clk(clk)'] + [f'.{s}(q_{s})' for s in inputs] + [f'.{s}(d_{s})' for s in outputs]
    lines += ['focmini_driver dut(' + ','.join(connections) + ');', 'endmodule']
    path.write_text('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vivado', default=shutil.which('vivado') or str(Path.home() / 'Vivado/2024.2/bin/vivado'))
    ap.add_argument('--stage', choices=('synth', 'route'), default='route')
    ap.add_argument('--output-root', type=Path, default=ROOT / 'results/arty_focmini_build')
    args = ap.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S-'), dir=args.output_root)).resolve()
    report = {'status': 'running', 'part': 'xc7a35ticsg324-1L', 'stage': args.stage,
              'clock_ns': 10.0, 'boundary_delay_ns': 0.0,
              'timing_scope': 'register-to-register including generated fixture boundaries; physical IO unvalidated',
              'source_sha256': {}}
    try:
        for p in sorted(MODULE_DIR.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                rel = p.relative_to(MODULE_DIR)
                dest = out / 'source' / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(p.read_bytes())
                report['source_sha256'][str(rel)] = hashlib.sha256(p.read_bytes()).hexdigest()
        fixture(out / 'fixture.sv')
        report['fixture_sha256'] = hashlib.sha256((out / 'fixture.sv').read_bytes()).hexdigest()
        report['source_head'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        report['source_status'] = subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True)
        cmd = [args.vivado, '-mode', 'batch', '-nojournal', '-log', 'vivado.log',
               '-source', str(out / 'source/tools/build.tcl'), '-tclargs', args.stage]
        report['command'] = cmd
        with (out / 'console.log').open('w') as log:
            proc = subprocess.Popen(cmd, cwd=out, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = proc.wait(timeout=1200)
            except subprocess.TimeoutExpired:
                import os, signal
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=30)
                raise RuntimeError('Vivado exceeded the 20 minute bound')
        report['returncode'] = code
        if (out / 'summary.txt').exists():
            for line in (out / 'summary.txt').read_text().splitlines():
                key, value = line.split('=', 1)
                report[key] = value
        if code or report.get('implementation_status') != 'passed':
            raise RuntimeError('Build failed; inspect retained reports and console.log')
        report['status'] = 'passed'
    except Exception as error:
        report.update(status='failed', error=str(error))
    finally:
        (out / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'status': report['status'], 'evidence': str(out), 'error': report.get('error')}), flush=True)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
