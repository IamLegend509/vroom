#!/usr/bin/env python3
"""Snapshot, test, and optionally build the sensor-only image; never programs hardware."""
import argparse, datetime, hashlib, json, pathlib, shutil, subprocess, tempfile
p=argparse.ArgumentParser();p.add_argument('--build',action='store_true')
p.add_argument('--vivado',default=shutil.which('vivado') or str(pathlib.Path.home()/'Vivado/2024.2/bin/vivado'),help='Vivado executable on the build host')
a=p.parse_args()
src=pathlib.Path(__file__).resolve().parent;repo=src.parents[3]
root=repo/'results/arty_sensor_diag';root.mkdir(parents=True,exist_ok=True)
run=pathlib.Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-'),dir=root))
shutil.copytree(src,run/'source',ignore=shutil.ignore_patterns('__pycache__'))
meta={'status':'running','git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
 'git_status':subprocess.check_output(['git','status','--short'],cwd=repo,text=True),
 'submodules':subprocess.check_output(['git','submodule','status'],cwd=repo,text=True),
 'sources':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in src.iterdir() if f.is_file()},
 'scope':'sensor diagnostic only; no current calibration or motor control','commands':[]}
(root/'latest_work.txt').write_text(str(run))
def save(): (run/'metadata.json').write_text(json.dumps(meta,indent=2))
def command(args,log,timeout=600):
 meta['commands'].append(args);save()
 with (run/log).open('w') as f:
  r=subprocess.run(args,cwd=run,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
 if r.returncode: raise RuntimeError(f'{log} failed ({r.returncode})')
try:
 assert 'PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports {uart_tx}]' in (run/'source/sensor_diag.xdc').read_text(), 'FPGA UART TX must be D10'
 for top,names in [('tb_i2c',['as5600_reader.sv','tb_i2c.sv']),('tb_top',['as5600_reader.sv','uart_tx.sv','xadc_a0.sv','xadc_mock.sv','sensor_diag_top.sv','tb_top.sv'])]:
  command(['iverilog','-g2012','-s',top,'-o',top]+['source/'+n for n in names],top+'_compile.log')
  command(['vvp',top],top+'.log',60);print((run/(top+'.log')).read_text(),flush=True)
 meta['status']='simulation_passed';save()
 if a.build:
  command([a.vivado,'-mode','batch','-nojournal','-log','vivado.log','-source','source/build.tcl'],'console.log',600)

  if 'CRITICAL WARNING:' in (run/'console.log').read_text(): raise RuntimeError('Vivado critical warning; review required')
  meta['status']='bitstream_built';meta['bitstream_sha256']=hashlib.sha256((run/'sensor_diag.bit').read_bytes()).hexdigest()
  print((run/'summary.txt').read_text(),flush=True)
except Exception as e:
 meta['status']='failed';meta['failure']=str(e);raise
finally:
 save();print(run,flush=True)
