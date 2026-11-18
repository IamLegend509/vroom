#!/usr/bin/env python3
"""Closed-loop C-RL RTL: export, bit-exact controller sim, optional remote Vivado build.

Never programs a device. Creates an immutable run directory under
results/arty_rl_infer/ holding the source snapshot, generated memories,
controller trace comparison, simulator logs, and (with --build-host) the
bitstream (rl_closed_loop.bit) and Vivado reports.

Repro:
  python3 hw/arty/focmini/rl_infer/run_closed_loop.py
  python3 hw/arty/focmini/rl_infer/run_closed_loop.py --build-host lazybanana@10.249.214.176
"""
import argparse, datetime, hashlib, json, pathlib, shutil, subprocess, sys, tempfile

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('--build-host', help='ssh target with Vivado 2024.2 (e.g. user@host)')
p.add_argument('--remote-vivado', default='~/.local/bin/vivado')
p.add_argument('--trace-ticks', type=int, default=250)
a = p.parse_args()

src = pathlib.Path(__file__).resolve().parent
fm = src.parent
repo = src.parents[3]
root = repo / 'results/arty_rl_infer'
root.mkdir(parents=True, exist_ok=True)
run = pathlib.Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-crl-'), dir=root))
shutil.copytree(src, run / 'source/rl_infer', ignore=shutil.ignore_patterns('__pycache__'))
(run / 'source/position_loop').mkdir(parents=True)
(run / 'source/interactive_lab').mkdir(parents=True)
(run / 'source/sensor_diag').mkdir(parents=True)
for fn in ('uart_rx.sv', 'position_loop.xdc'):
    shutil.copy(fm / 'position_loop' / fn, run / 'source/position_loop/')
shutil.copy(fm / 'interactive_lab/command_rx.sv', run / 'source/interactive_lab/')
for fn in ('as5600_reader.sv', 'uart_tx.sv', 'xadc_a0.sv'):
    shutil.copy(fm / 'sensor_diag' / fn, run / 'source/sensor_diag/')

meta = {
    'status': 'running',
    'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
    'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=repo, text=True),
    'sources': {str(f.relative_to(run / 'source')): hashlib.sha256(f.read_bytes()).hexdigest()
                for f in (run / 'source').rglob('*') if f.is_file()},
    'scope': 'Closed-loop two-actor C-RL image (rl_closed_loop_top + rl_ctrl + rl_dual_mlp); no programming',
    'commands': [],
}
(root / 'latest_crl.txt').write_text(str(run))


def save():
    (run / 'metadata.json').write_text(json.dumps(meta, indent=2))


def command(args, log, timeout=1800, cwd=None):
    meta['commands'].append(args); save()
    with (run / log).open('w') as f:
        r = subprocess.run(args, cwd=cwd or run, stdout=f, stderr=subprocess.STDOUT, timeout=timeout)
    if r.returncode:
        raise RuntimeError(f'{log} failed ({r.returncode}); see {run}')


try:
    s = 'source/'
    pair = repo / 'Vroom_2805_Experimental_Controller_Pair'
    ckpt = pair / 'selected_checkpoint.npz'
    vecs = pair / 'golden_vectors.npz'
    meta['checkpoint'] = str(ckpt)
    command([sys.executable, s + 'rl_infer/quant.py', '--out', '.', '--checkpoint', str(ckpt), '--vectors', str(vecs)], 'quant.log')
    command([sys.executable, s + 'rl_infer/ctrl_model.py', '--checkpoint', str(ckpt), '--trace-ticks', str(a.trace_ticks), '--out', '.'], 'ctrl_model.log')
    command([sys.executable, s + 'rl_infer/ctrl_vectors.py', '.'], 'ctrl_vectors.log')
    command(['iverilog', '-V'], 'iverilog-version.log')
    command(['iverilog', '-g2012', '-s', 'tb_rl_ctrl', '-o', 'tb_rl_ctrl',
             s + 'rl_infer/rl_dual_mlp.sv', s + 'rl_infer/rl_ctrl.sv', s + 'rl_infer/tb_rl_ctrl.sv'],
            'tb_rl_ctrl_compile.log')
    command(['vvp', '-n', 'tb_rl_ctrl'], 'tb_rl_ctrl.log', 600)
    log = (run / 'tb_rl_ctrl.log').read_text(); print(log, flush=True)
    if 'overrun=0' not in log:
        raise RuntimeError('tb_rl_ctrl reported overrun or did not finish cleanly')
    cmp = {}
    for rtl_f, ref_f, key in (('rtl_ctrl_out.txt', 'ctrl_exp.txt', 'out'),
                              ('rtl_ctrl_xi.txt', 'ctrl_xi_exp.txt', 'xi'),
                              ('rtl_ctrl_xo.txt', 'ctrl_xo_exp.txt', 'xo')):
        ra = np.loadtxt(run / rtl_f, dtype=np.int64)
        rb = np.loadtxt(run / ref_f, dtype=np.int64)
        cmp[key] = {'rows': int(len(ra)), 'mismatched_words': int((ra != rb).sum())}
    cmp['bit_exact'] = all(v['mismatched_words'] == 0 for v in cmp.values())
    (run / 'rtl_vs_ctrl_model.json').write_text(json.dumps(cmp, indent=2))
    meta['rtl_vs_ctrl_model'] = cmp; save()
    print(json.dumps(cmp, indent=2), flush=True)
    if not cmp['bit_exact']:
        raise RuntimeError('rl_ctrl is not bit-exact with ctrl_model.py')
    meta['status'] = 'simulation_passed'; save()

    if a.build_host:
        rdir = f'vroom-builds/arty_rl_infer/{run.name}'
        command(['ssh', a.build_host, f'mkdir -p {rdir}'], 'remote_mkdir.log', 60)
        command(['rsync', '-a', '--exclude', 'tb_*', '--exclude', 'rtl_*', f'{run}/', f'{a.build_host}:{rdir}/'], 'rsync_up.log', 300)
        command(['ssh', a.build_host, f'cd {rdir} && {a.remote_vivado} -mode batch -nojournal -log vivado.log '
                 f'-source source/rl_infer/build_closed_loop.tcl > console.log 2>&1; rc=$?; tail -40 console.log; exit $rc'],
                'remote_build.log', 3600)
        command(['rsync', '-a', '--exclude', 'routed.dcp', f'{a.build_host}:{rdir}/', f'{run}/'], 'rsync_down.log', 300)
        console = (run / 'console.log').read_text()
        if 'CRITICAL WARNING:' in console:
            raise RuntimeError('Vivado critical warning: review required')
        meta['status'] = 'bitstream_built'
        meta['build_host'] = a.build_host
        meta['bitstream_sha256'] = hashlib.sha256((run / 'rl_closed_loop.bit').read_bytes()).hexdigest()
        print((run / 'summary.txt').read_text(), flush=True)
except Exception as e:
    meta['status'] = 'failed'; meta['failure'] = str(e); raise
finally:
    save(); print(run, flush=True)
