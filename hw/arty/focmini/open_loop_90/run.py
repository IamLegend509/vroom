#!/usr/bin/env python3
"""Immutable source snapshot, digital tests, optional routed bitstream; never flashes."""
import argparse,datetime,hashlib,json,math,pathlib,shutil,subprocess,tempfile,sys
p=argparse.ArgumentParser();p.add_argument('--build',action='store_true')
p.add_argument('--vivado',default=shutil.which('vivado') or str(pathlib.Path.home()/'Vivado/2024.2/bin/vivado'),help='Vivado executable on the build host')
a=p.parse_args()
src=pathlib.Path(__file__).resolve().parent;repo=src.parents[3]
root=repo/'results/arty_open_loop_90';root.mkdir(parents=True,exist_ok=True)
run=pathlib.Path(tempfile.mkdtemp(prefix=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S-'),dir=root))
for folder in ('open_loop_90','sensor_diag'):
 shutil.copytree(src.parent/folder,run/'source'/folder,ignore=shutil.ignore_patterns('__pycache__'))
meta={'status':'running','git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
 'git_status':subprocess.check_output(['git','status','--short'],cwd=repo,text=True),
 'submodules':subprocess.check_output(['git','submodule','status'],cwd=repo,text=True),
 'sources':{str(f.relative_to(run/'source')):hashlib.sha256(f.read_bytes()).hexdigest() for f in (run/'source').rglob('*') if f.is_file()},
 'scope':'Fixed 0.6V/9V slow nominal 90-degree shaft sweep, 35s, seven pole pairs assumed; no current regulation, physical test or programming',
 'seed':'deterministic directed tests','commands':[]}
(root/'latest_work.txt').write_text(str(run))
def save(): (run/'metadata.json').write_text(json.dumps(meta,indent=2))
def command(args,log,timeout=600):
 meta['commands'].append(args);save()
 with (run/log).open('w') as f:
  r=subprocess.run(args,cwd=run,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
 if r.returncode: raise RuntimeError(f'{log} failed ({r.returncode}); see {run}')
try:
 xdc=(run/'source/open_loop_90/open_loop_90.xdc').read_text()
 for pin,name in [('D10','uart_tx'),('A9','uart_rx')]:
  assert f'PACKAGE_PIN {pin} IOSTANDARD LVCMOS33 }} [get_ports {{{name}}}]' in xdc
 # Independent LUT validation using phase line-line differences (common mode cancels).
 words=[int(s,16) for s in (run/'source/open_loop_90/sweep_duty.mem').read_text().splitlines()]
 assert len(words)==256
 max_error=0
 for n,w in enumerate(words):
  d=[(w>>shift)&8191 for shift in (26,13,0)]
  assert all(2166<=v<=2834 for v in d)
  ideal=[.6*math.sin(2*math.pi*n/256+off) for off in (0,-2*math.pi/3,2*math.pi/3)]
  for k in (1,2):
   max_error=max(max_error,abs((d[0]-d[k])*9/5000-(ideal[0]-ideal[k])))
 assert max_error<=.0018
 (run/'lut_metrics.json').write_text(json.dumps({'entries':256,'max_line_line_rounding_error_V':max_error,'assumed_bus_V':9},indent=2))
 command(['iverilog','-V'],'iverilog-version.log')
 sd='source/sensor_diag/';ol='source/open_loop_90/'
 suites=[('tb_uart',[ol+'uart_rx.sv',ol+'command_rx.sv',ol+'tb_uart.sv']),
  ('tb_controller',[ol+'sweep_controller.sv',ol+'tb_controller.sv']),
  ('tb_open_loop_90_top',[sd+n for n in ('as5600_reader.sv','uart_tx.sv','xadc_a0.sv','xadc_mock.sv')]+[ol+n for n in ('uart_rx.sv','command_rx.sv','sweep_controller.sv','open_loop_90_top.sv','tb_open_loop_90_top.sv')]),
  ('tb_i2c',[sd+'as5600_reader.sv',sd+'tb_i2c.sv']),
  ('tb_top',[sd+n for n in ('as5600_reader.sv','uart_tx.sv','xadc_a0.sv','xadc_mock.sv','sensor_diag_top.sv','tb_top.sv')])]
 for top,files in suites:
  command(['iverilog','-g2012','-s',top,'-o',top]+files,top+'_compile.log')
  command(['vvp',top],top+'.log',120);print((run/(top+'.log')).read_text(),flush=True)
 command([sys.executable,'source/open_loop_90/test_bench.py'],'host_tests.log',30)
 print((run/'host_tests.log').read_text(),flush=True)
 meta['status']='simulation_passed';save()
 if a.build:
  command([a.vivado,'-mode','batch','-nojournal','-log','vivado.log','-source','source/open_loop_90/build.tcl'],'console.log',600)
  console=(run/'console.log').read_text()
  if 'CRITICAL WARNING:' in console or 'No ports matched' in console or 'No pins matched' in console:
   raise RuntimeError('Vivado critical/constraint warning: review required')
  meta['status']='bitstream_built';meta['bitstream_sha256']=hashlib.sha256((run/'open_loop_90.bit').read_bytes()).hexdigest()
  print((run/'summary.txt').read_text(),flush=True)
except Exception as e:
 meta['status']='failed';meta['failure']=str(e);raise
finally:
 save();print(run,flush=True)
