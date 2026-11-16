#!/usr/bin/env python3
"""RL inference RTL: export, bit-exact simulation, optional remote Vivado build.

Never programs a device. Creates an immutable run directory under
results/arty_rl_infer/ holding the source snapshot, generated memories,
quantization report, simulator logs, comparison metrics and (with --build)
the bitstream and Vivado reports.

Repro:
  python3 hw/arty/focmini/rl_infer/run.py                       # sim only
  python3 hw/arty/focmini/rl_infer/run.py --build-host lazybanana@10.249.214.176
"""
import argparse, datetime, hashlib, json, pathlib, shutil, subprocess, sys, tempfile

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('--build-host', help='ssh target with Vivado 2024.2 (e.g. user@host)')
p.add_argument('--remote-vivado', default='~/.local/bin/vivado')
p.add_argument('--quick', action='store_true', help='skip the 518-vector engine sim (top-level test only)')
a = p.parse_args()

src = pathlib.Path(__file__).resolve().parent
fm = src.parent
repo = src.parents[3]
root = repo / 'results/arty_rl_infer'
root.mkdir(parents=True, exist_ok=True)
run = pathlib.Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-'), dir=root))
shutil.copytree(src, run / 'source/rl_infer', ignore=shutil.ignore_patterns('__pycache__'))
(run / 'source/position_loop').mkdir(parents=True)
(run / 'source/sensor_diag').mkdir(parents=True)
shutil.copy(fm / 'position_loop/uart_rx.sv', run / 'source/position_loop/')
shutil.copy(fm / 'sensor_diag/uart_tx.sv', run / 'source/sensor_diag/')
meta = {'status': 'running',
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=repo, text=True),
        'sources': {str(f.relative_to(run / 'source')): hashlib.sha256(f.read_bytes()).hexdigest()
                    for f in (run / 'source').rglob('*') if f.is_file()},
        'scope': 'Quantized two-actor inference RTL and P1 UART parity image; motor pins constant LOW; no programming',
        'commands': []}
(root / 'latest_work.txt').write_text(str(run))


def save():
    (run / 'metadata.json').write_text(json.dumps(meta, indent=2))


def command(args, log, timeout=900, cwd=None):
    meta['commands'].append(args); save()
    with (run / log).open('w') as f:
        r = subprocess.run(args, cwd=cwd or run, stdout=f, stderr=subprocess.STDOUT, timeout=timeout)
    if r.returncode:
        raise RuntimeError(f'{log} failed ({r.returncode}); see {run}')


try:
    s = 'source/'
    pair = repo / 'Vroom_2805_Experimental_Controller_Pair'
    meta['checkpoint'] = str(pair / 'selected_checkpoint.npz'); meta['vectors'] = str(pair / 'golden_vectors.npz')
    command([sys.executable, s + 'rl_infer/quant.py', '--out', '.', '--checkpoint', meta['checkpoint'],
             '--vectors', meta['vectors']], 'quant.log')
    meta['quant'] = json.loads((run / 'quant_report.json').read_text()); save()
    command(['iverilog', '-V'], 'iverilog-version.log')
    common = [s + 'rl_infer/rl_mlp.sv']
    if not a.quick:
        command(['iverilog', '-g2012', '-s', 'tb_rl_mlp', '-o', 'tb_rl_mlp'] + common + [s + 'rl_infer/tb_rl_mlp.sv'], 'tb_rl_mlp_compile.log')
        command(['vvp', '-n', 'tb_rl_mlp'], 'tb_rl_mlp.log', 1800)
        cmp = {}
        for n in ('inner', 'outer'):
            rtl = np.loadtxt(run / f'rtl_{n}_y.txt', dtype=np.int64)
            ref = np.loadtxt(run / f'{n}_y.txt', dtype=np.int64)
            cmp[n] = {'vectors': int(len(rtl)), 'mismatched_words': int((rtl != ref).sum()),
                      'max_abs_lsb_diff': int(np.abs(rtl - ref).max())}
        cmp['bit_exact'] = all(v['mismatched_words'] == 0 for v in cmp.values() if isinstance(v, dict))
        (run / 'rtl_vs_integer_model.json').write_text(json.dumps(cmp, indent=2))
        meta['rtl_vs_integer_model'] = cmp; save()
        print(json.dumps(cmp, indent=2), flush=True)
        if not cmp['bit_exact']:
            raise RuntimeError('RTL is not bit-exact with quant.py')
    top = common + [s + 'rl_infer/rl_parity_top.sv', s + 'position_loop/uart_rx.sv', s + 'sensor_diag/uart_tx.sv',
                    s + 'rl_infer/tb_rl_parity_top.sv']
    command(['iverilog', '-g2012', '-s', 'tb_rl_parity_top', '-o', 'tb_rl_parity_top'] + top, 'tb_rl_parity_top_compile.log')
    command(['vvp', '-n', 'tb_rl_parity_top'], 'tb_rl_parity_top.log', 600)
    log = (run / 'tb_rl_parity_top.log').read_text(); print(log, flush=True)
    if 'PASS PARITY_TOP' not in log or 'FAIL' in log:
        raise RuntimeError('parity top test failed')
    meta['status'] = 'simulation_passed'; save()
    if a.build_host:
        rdir = f'vroom-builds/arty_rl_infer/{run.name}'
        command(['ssh', a.build_host, f'mkdir -p {rdir}'], 'remote_mkdir.log', 60)
        command(['rsync', '-a', '--exclude', 'tb_*', '--exclude', 'rtl_*', f'{run}/', f'{a.build_host}:{rdir}/'], 'rsync_up.log', 300)
        command(['ssh', a.build_host, f'cd {rdir} && {a.remote_vivado} -mode batch -nojournal -log vivado.log '
                 f'-source source/rl_infer/build.tcl > console.log 2>&1; rc=$?; tail -40 console.log; exit $rc'],
                'remote_build.log', 3600)
        command(['rsync', '-a', '--exclude', 'routed.dcp', f'{a.build_host}:{rdir}/', f'{run}/'], 'rsync_down.log', 300)
        console = (run / 'console.log').read_text()
        if 'CRITICAL WARNING:' in console:
            raise RuntimeError('Vivado critical warning: review required')
        meta['status'] = 'bitstream_built'
        meta['build_host'] = a.build_host
        meta['bitstream_sha256'] = hashlib.sha256((run / 'rl_parity.bit').read_bytes()).hexdigest()
        print((run / 'summary.txt').read_text(), flush=True)
except Exception as e:
    meta['status'] = 'failed'; meta['failure'] = str(e); raise
finally:
    save(); print(run, flush=True)
