#!/usr/bin/env python3
"""Summarize retained C1/C2 telemetry without asserting physical angle accuracy."""
import argparse,csv,json,pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('trial',type=pathlib.Path);a=p.parse_args()
rows=[json.loads(line) for line in (a.trial/'packets.jsonl').read_text().splitlines()]
running=[r for r in rows if r['m']==5]
closed=[r for r in running if r['k']]
signed=lambda x,b:x-(1<<b) if x>=(1<<(b-1)) else x
if not closed:raise SystemExit('No calibrated closed-loop samples; retain raw records.')
origin=closed[0]['d']
converted=[]
for r in closed:
 converted.append({'seconds':r['t']/1000,'encoder_relative_deg':signed((r['r']-origin)&4095,12)*360/4096,
 'target_relative_deg':signed((r['d']-origin)&4095,12)*360/4096,
 'error_deg':signed(r['z'],12)*360/4096,'field_phase_deg':signed(r['o'],16)*360/65536,
 'adc_raw':r['a'],'weak_field':bool(r['s']&16)})
summary=json.loads((a.trial/'summary.json').read_text())
metrics={'scope':'Encoder-reported relative positions; weak-field sensor accuracy unvalidated. ADC is not amperes.',
 'host_status':summary['status'],'host_error':summary.get('error'),'last_packet':rows[-1],
 'origin_deg':origin*360/4096,'active_packets':len(running),'closed_loop_packets':len(closed),
 'weak_field_packets':sum(bool(r['s']&16) for r in rows),
 'max_reported_relative_deg':max(r['encoder_relative_deg'] for r in converted),
 'min_reported_relative_deg':min(r['encoder_relative_deg'] for r in converted),
 'last_active_error_deg':converted[-1]['error_deg'],
 'max_abs_field_phase_deg':max(abs(r['field_phase_deg']) for r in converted),
 'driver_fault_packets_while_active':sum(not r['f'] for r in running),
 'bad_sensor_packets_while_active':sum(not r['v'] or r['c'] or not r['i'] or r['e'] or not r['s']&32 or r['s']&8 for r in running)}
(a.trial/'analysis.json').write_text(json.dumps(metrics,indent=2))
with (a.trial/'tracking.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(converted[0]));w.writeheader();w.writerows(converted)
fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
t=[r['seconds'] for r in converted]
axes[0].plot(t,[r['encoder_relative_deg'] for r in converted],label='Encoder')
axes[0].plot(t,[r['target_relative_deg'] for r in converted],label='Target')
axes[0].set_ylabel('Relative angle (°)');axes[0].legend()
axes[1].plot(t,[r['error_deg'] for r in converted],label='Error')
axes[1].axhline(12*360/4096,color='grey',ls='--');axes[1].axhline(-12*360/4096,color='grey',ls='--')
axes[1].set_ylabel('Position error (°)')
axes[2].plot(t,[r['field_phase_deg'] for r in converted]);axes[2].set_ylabel('Field phase (°)');axes[2].set_xlabel('Test time (s)')
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('FPGA position trial: '+summary['status']+' — weak-field encoder readings')
fig.tight_layout();fig.savefig(a.trial/'tracking.png',dpi=160)
print(json.dumps(metrics,indent=2))
