# Arty motor-control work

[`focmini/`](focmini/README.md) contains the user-authorized FPGA classical FOC
baseline for the 9 V / seven-pole-pair FOCMini setup. RTL simulation, strict
lint and the registered 35T implementation fixture pass at 100MHz. One heavier
assumed motor misses its closed-loop tracking gate. Separate physical board tops
now read the AS5600 and one current ADC and have driven the motor. Closed-loop
position trials failed full tracking acceptance, and the nominal 90-degree
open-loop command did not produce a verified 90-degree shaft sweep.
See [HANDOFF.md](focmini/HANDOFF.md) for the current state and continuation steps.

This branch does not contain the separate FP32 actor inference implementation
from `crisiumnih/vroom:codex/arty-inference-board`. The conventional controller
here is a baseline for motor bring-up before integrating that actor.
