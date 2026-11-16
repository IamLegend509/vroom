#!/usr/bin/env python3
"""Turn ctrl_model.py's ctrl_trace.json into replay stimulus/expectations for tb_rl_ctrl.

Writes ctrl_in.txt (th w adc ref per tick), ctrl_exp.txt (e16 ia id iq iqref duty_a duty_b duty_c),
ctrl_xi_exp.txt / ctrl_xo_exp.txt (actor input words in write order) and ctrl_cal.txt.
"""
import json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import ctrl_model as cm

run = pathlib.Path(sys.argv[1])
tr = json.loads((run / 'ctrl_trace.json').read_text())
with (run / 'ctrl_in.txt').open('w') as f:
    for r in tr: f.write(f"{r['th']} {r['w']} {r['adc']} {r['ref']}\n")
with (run / 'ctrl_exp.txt').open('w') as f:
    for r in tr: f.write(f"{r['e16']} {r['ia']} {r['id']} {r['iq']} {r['iq_ref']} {r['duty'][0]} {r['duty'][1]} {r['duty'][2]}\n")
(run / 'ctrl_xi_exp.txt').write_text(''.join(f'{v}\n' for r in tr for v in r['xi']))
(run / 'ctrl_xo_exp.txt').write_text(''.join(f'{v}\n' for r in tr if 'xo' in r for v in r['xo']))
# calibration used by closed_loop() defaults: enc_dir=-1, enc_off=1234, cur_sign=-1, zero_code=416
ph = json.loads((run / 'ctrl_report.json').read_text())['trace']['sensed_phase']
cal = {'dir_neg': 1, 'off': (cm.POLE_PAIRS * 16 * 1234) & 0xFFFF, 'zero': 16 * 416, 'sign_neg': 1, 'gain': cm.ADC_GAIN, 'ph': ph}
(run / 'ctrl_cal.txt').write_text(f"{cal['dir_neg']} {cal['off']} {cal['zero']} {cal['sign_neg']} {cal['gain']} {len(tr)} {ph}\n")
print(json.dumps(cal), len(tr), 'ticks')
