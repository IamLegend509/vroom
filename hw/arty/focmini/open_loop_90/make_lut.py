#!/usr/bin/env python3
"""Fixed 0.6V phase-neutral sinusoidal amplitude, assuming 9V bus, 5000 PWM clocks.
The table is not motor calibration. Three duties are packed A[38:26],B[25:13],C[12:0].
"""
import math,pathlib
p=pathlib.Path(__file__).with_name('sweep_duty.mem')
words=[]
for n in range(256):
 d=[round(5000*(.5+.6/9*math.sin(2*math.pi*n/256+offset))) for offset in (0,-2*math.pi/3,2*math.pi/3)]
 words.append(f'{(d[0]<<26)|(d[1]<<13)|d[2]:010x}')
p.write_text('\n'.join(words)+'\n')
