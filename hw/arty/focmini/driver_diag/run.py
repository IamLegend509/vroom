#!/usr/bin/env python3
"""Snapshot/tests/routed driver diagnostic. Never programs hardware."""
import argparse,datetime,hashlib,json,pathlib,shutil,subprocess,sys,tempfile
p=argparse.ArgumentParser();p.add_argument('--build',action='store_true')
p.add_argument('--vivado',default=shutil.which('vivado') or str(pathlib.Path.home()/'Vivado/2024.2/bin/vivado'),help='Vivado executable on the build host')
a=p.parse_args()
src=pathlib.Path(__file__).resolve().parent;repo=src.parents[3]
root=repo/'results/arty_driver_diag';root.mkdir(parents=True,exist_ok=True)
run=pathlib.Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-'),dir=root))
for folder in ('driver_diag','sensor_diag','open_loop'):
 shutil.copytree(src.parent/folder,run/'source'/folder,ignore=shutil.ignore_patterns('__pycache__'))
meta={'status':'running','scope':'wake-only diagnostic, four motor controls tied GND; no physical test or programming in runner',
 'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
 'git_status':subprocess.check_output(['git','status','--short'],cwd=repo,text=True),
 'submodules':subprocess.check_output(['git','submodule','status'],cwd=repo,text=True),
 'sources':{str(f.relative_to(run/'source')):hashlib.sha256(f.read_bytes()).hexdigest() for f in (run/'source').rglob('*') if f.is_file()},
 'seed':'deterministic','commands':[]}
(root/'latest_work.txt').write_text(str(run))
def save():(run/'metadata.json').write_text(json.dumps(meta,indent=2))
def command(args,log,timeout=600):
 meta['commands'].append(args);save()
 with (run/log).open('w') as f:r=subprocess.run(args,cwd=run,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
 if r.returncode:raise RuntimeError(f'{log} failed ({r.returncode}) at {run}')
try:
 command(['iverilog','-V'],'iverilog-version.log')
 sd='source/sensor_diag/';ol='source/open_loop/';dd='source/driver_diag/'
 suites=[('tb_wake',[dd+'wake_control.sv',dd+'tb_wake.sv']),
  ('tb_driver_diag_top',[sd+n for n in ('as5600_reader.sv','uart_tx.sv','xadc_a0.sv','xadc_mock.sv')]+[ol+n for n in ('uart_rx.sv','command_rx.sv')]+[dd+n for n in ('wake_control.sv','driver_diag_top.sv','tb_driver_diag_top.sv')]),
  ('tb_uart',[ol+n for n in ('uart_rx.sv','command_rx.sv','tb_uart.sv')]),
  ('tb_i2c',[sd+'as5600_reader.sv',sd+'tb_i2c.sv'])]
 for top,files in suites:
  command(['iverilog','-g2012','-s',top,'-o',top]+files,top+'_compile.log')
  command(['vvp',top],top+'.log',60);print((run/(top+'.log')).read_text(),flush=True)
 command([sys.executable,dd+'test_bench.py'],'host_tests.log',30);print((run/'host_tests.log').read_text(),flush=True)
 meta['status']='simulation_passed';save()
 if a.build:
  command([a.vivado,'-mode','batch','-nojournal','-log','vivado.log','-source',dd+'build.tcl'],'console.log',600)
  log=(run/'console.log').read_text()
  if any(s in log for s in ('CRITICAL WARNING:','No ports matched','No pins matched')):raise RuntimeError('Vivado warning needs review')
  meta['status']='bitstream_built';meta['bitstream_sha256']=hashlib.sha256((run/'driver_diag.bit').read_bytes()).hexdigest()
  print((run/'summary.txt').read_text(),flush=True)
except Exception as e:meta['status']='failed';meta['failure']=str(e);raise
finally:save();print(run,flush=True)
