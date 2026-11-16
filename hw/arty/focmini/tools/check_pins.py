#!/usr/bin/env python3
"""Validate planned board constraints against Digilent's source and Vivado.

Elaborates a port-only contract fixture, checks XDC ports, pin uniqueness,
package functions, directions, IO standards and clock. No bitstream or motor.
"""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[4]
MODULE_DIR = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vivado', default=shutil.which('vivado') or str(Path.home() / 'Vivado/2024.2/bin/vivado'))
    ap.add_argument('--master-xdc', type=Path, help='Optional local copy of the pinned official master XDC')
    args = ap.parse_args()
    base = ROOT / 'results/arty_focmini_pins'
    base.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S-'), dir=base))
    report = {'status': 'running', 'scope': 'port/constraint/package validation, not physical implementation'}
    try:
        for source in (MODULE_DIR / 'board/arty_a7_35.xdc', MODULE_DIR / 'board/pinmap.json',
                       MODULE_DIR / 'tests/pin_contract.sv', Path(__file__).resolve()):
            shutil.copyfile(source, out / source.name)
        pinmap = json.loads((out / 'pinmap.json').read_text())
        raw = args.master_xdc.read_bytes() if args.master_xdc else urllib.request.urlopen(pinmap['source_url'], timeout=30).read()
        (out / 'official_master.xdc').write_bytes(raw)
        assert hashlib.sha256(raw).hexdigest() == pinmap['source_sha256'], 'Official source changed; review pin map'
        official = {}
        for line in raw.decode().splitlines():
            match = re.search(r'PACKAGE_PIN\s+(\w+)\s+IOSTANDARD\s+(\w+).*?get_ports\s+\{\s*([^}]+?)\s*\}', line)
            if match:
                official[match[3]] = (match[1], match[2])
        pins = pinmap['pins']
        assert len({p['pin'] for p in pins}) == len(pins), 'Duplicate package pin'
        assert len({p['port'] for p in pins}) == len(pins), 'Duplicate board port'
        for pin in pins:
            assert official[pin['master_port']] == (pin['pin'], 'LVCMOS33'), pin
        # Digilent names TXD/RXD from the USB bridge's perspective.
        # Verify electrical direction as well as the named master-XDC pair.
        by_name = {p['port']: p for p in pins}
        assert (by_name['uart_tx']['pin'], by_name['uart_tx']['direction']) == ('D10', 'output')
        assert (by_name['uart_rx']['pin'], by_name['uart_rx']['direction']) == ('A9', 'input')
        report['uart_direction_contract'] = 'passed: FPGA TX=D10, RX=A9'
        report['official_source_checks'] = len(pins)
        lines = ['read_verilog pin_contract.sv',
                 'synth_design -rtl -top focmini_pin_contract -part ' + pinmap['part'],
                 'read_xdc arty_a7_35.xdc',
                 f'if {{[llength [get_ports]] != {len(pins)}}} {{error "Wrong port count"}}',
                 'set rows [open pins.tsv w]', 'puts $rows "port\tpin\tfunction\tdirection\tiostandard"']
        for pin in pins:
            name, package, direction = pin['port'], pin['pin'], {'input': 'IN', 'output': 'OUT', 'inout': 'INOUT'}[pin['direction']]
            lines += [f'set p [get_ports {{{name}}}]',
                      f'if {{[get_property PACKAGE_PIN $p] ne "{package}"}} {{error "Pin mismatch: {name}"}}',
                      f'if {{[get_property DIRECTION $p] ne "{direction}"}} {{error "Direction mismatch: {name}"}}',
                      'if {[get_property IOSTANDARD $p] ne "LVCMOS33"} {error "IO standard mismatch"}',
                      f'if {{[llength [get_package_pins {package}]] != 1}} {{error "No package pin: {package}"}}',
                      f'set function [get_property PIN_FUNC [get_package_pins {package}]]',
                      f'puts $rows [join [list {{{name}}} {{{package}}} $function {{{direction}}} LVCMOS33] "\\t"]']
            if name.startswith('vaux'):
                function = name.replace('vaux', 'AD').replace('_', '').upper()
                lines += [f'if {{![string match *{function}* $function]}} {{error "Wrong ADC function: {name}"}}']
        lines += ['close $rows',
                  'if {[get_property PERIOD [get_clocks board_clk]] != 10.0} {error "Clock period mismatch"}',
                  'puts "PASS pin_constraints"', 'exit 0']
        (out / 'check.tcl').write_text('\n'.join(lines) + '\n')
        cmd = [args.vivado, '-mode', 'batch', '-nojournal', '-log', 'vivado.log', '-source', 'check.tcl']
        report['command'] = cmd
        with (out / 'console.log').open('w') as log:
            run = subprocess.run(cmd, cwd=out, stdout=log, stderr=subprocess.STDOUT, timeout=120)
        report['returncode'] = run.returncode
        if run.returncode or 'PASS pin_constraints' not in (out / 'console.log').read_text():
            raise RuntimeError('Vivado pin validation failed; inspect console.log')
        report['source_head'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        report['source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir()
                                   if p.is_file() and p.suffix in ('.xdc', '.sv', '.py', '.tcl', '.json')}
        report['status'] = 'passed'
    except Exception as error:
        report.update(status='failed', error=str(error))
    finally:
        (out / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'status': report['status'], 'evidence': str(out), 'error': report.get('error')}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
