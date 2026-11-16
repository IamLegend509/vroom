#!/usr/bin/env python3
"""Summarize actual RTL traces; assumed nonlinear plant, not measured hardware."""
import csv,json,pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
rows=list(csv.DictReader(open('position_trace.csv')))
metrics={'scope':'RTL feedback against assumed nonlinear damped rotor; no physical validation',
 'plant':{'pole_pairs':7,'drive_acceleration_counts_per_s2':6000,'viscous_damping_per_s':30,
          'coulomb_acceleration_counts_per_s2':300,'integration_s':.001,
          'disturbance_counts_per_s2':600,'disturbance_window_s':[9,10]},'scenarios':{}}
fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
for scenario,label in [(1,'Ordinary angle'),(2,'Across 360° wrap'),(7,'Higher assumed friction')]:
 data=[r for r in rows if int(r['scenario'])==scenario and int(r['state'])==5 and int(r['elapsed_ms'])>2000]
 t=[int(r['elapsed_ms'])/1000 for r in data]
 err=[int(r['error_raw'])*360/4096 for r in data]
 settled=[abs(e) for s,e in zip(t,err) if any(lo<=s<hi for lo,hi in [(14.5,15),(26.5,27),(38.5,39),(50.5,51)])]
 metrics['scenarios'][str(scenario)]={'label':label,'coulomb_acceleration_counts_per_s2':3000 if scenario==7 else 300,'max_abs_settling_error_deg':max(settled),
  'max_abs_phase_deg':max(abs(int(r['phase_offset']))*360/65536 for r in data),
  'return_error_deg':err[-1],'trace_rows':len(data)}
 assert max(settled)<=12*360/4096
 axes[0].plot(t,err,label=label)
 if scenario==1:
  origin=int(data[0]['angle_raw'])
  wrap=lambda n:((n+2048)%4096-2048)*360/4096
  axes[1].plot(t,[wrap(int(r['target_raw'])-origin) for r in data],label='Target')
  axes[1].plot(t,[wrap(int(r['angle_raw'])-origin) for r in data],label='Encoder')
for ax in axes:
 ax.grid(alpha=.25);ax.legend()
axes[0].axhline(12*360/4096,color='grey',ls='--');axes[0].axhline(-12*360/4096,color='grey',ls='--')
axes[0].set_ylabel('Position error (degrees)');axes[1].set_ylabel('Relative position (degrees)');axes[1].set_xlabel('Profile time (s)')
fig.suptitle('Position feedback in RTL simulation — assumed plant, not hardware')
fig.tight_layout();fig.savefig('position_simulation.png',dpi=160)
pathlib.Path('simulation_metrics.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics,indent=2))
