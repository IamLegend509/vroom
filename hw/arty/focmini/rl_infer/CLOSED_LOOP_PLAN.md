# RL closed-loop image (C-RL) — proposed, not implemented

Goal: the two quantized actors (`rl_mlp.sv`) drive the 2805 motor from the Arty, at the
trained rates (inner 10 kHz, outer 1 kHz, ten inner decisions per outer step).

## Declared deviations from the trained contract

| Trained (sim) | C-RL on Arty | Status |
| --- | --- | --- |
| Measured id/iq (two-phase N1 sensing) | **Model observer + one ACS724 (phase A) correction** | Controller change; approximate-plant study: speed RMSE 2.4–2.9 vs 1.9 rad/s, no trips |
| Inverse transform uses true angle | AS5600 angle (≤140 µs old) + speed extrapolation | Unavoidable on hardware |
| Ideal mechanical speed | Filtered AS5600 angle difference | Unavoidable |
| FP32 actors | Quantized (≤0.0056 / 0.009 output error) | Measured |
| Fixed Vdc | 9 V bench supply, 0.10 A CC limit | Supply sag is not modelled |

Study: `results/arty_rl_infer/single_sensor_study_*/` (approximate numpy PMSM, not GEM).

## Data path per 100 µs inner tick

1. Phase-A XADC samples (16× averaged), synchronized to the PWM center. Subtract the zero offset
   calibrated at ARM, then apply the calibrated sign and gain. Nominal gain is only about 15 mA/LSB:
   400 mV/A × 18k/(18k + 18k‖3.32k Arty load) × 1/3.32 ≈ 16 mV/A at the XADC. The real gain must be
   measured with known currents (board README). The study re-run at 15 mA/LSB shows the
   model-dominated observer is insensitive to this: speed RMSE 2.6–3.0 rad/s, no trips.
2. Latest AS5600 angle (400 kHz I2C, continuous polling, status checked). Electrical
   angle = dir·7·θm − offset, plus ω·age compensation.
3. Observer: exact-discretized RL model driven by the previous applied dq voltage
   (the one-step delay is known), then a phase-A innovation correction (gain 0.5).
4. Build the inner observation exactly like `InnerIO` (25 features, Q.14, clipped).
5. Inner actor (25 µs). Action transform: ×0.9, circle projection to 0.9, ×Vdc/2.
6. Inverse Park/Clarke with a sin/cos LUT. Centered sine duty = 0.5 + v/Vdc. Latch at
   the next PWM period boundary (20 kHz, 3 phases).
7. Every 10th tick, the outer observation follows `OuterIO`: lags 5/10/20 history, tanh(err/0.05),
   the IASA adapter, then iq_ref.

## Calibration at ARM (driver awake, bounded voltage)

- Current zero: 50 % duty on all phases, average 4,096 samples.
- Electrical zero and direction: hold a d-axis vector at 0.6 V, then rotate slowly forward one
  electrical turn and back; record the encoder sign and offset; also confirm 7 pole pairs from
  mechanical travel.
- Current sign: a known d-axis voltage gives the expected phase-A sign.

Any failed check latches a refusal; the motor never runs uncalibrated.

## Protection (any trip → PWM low, EN low, latched until reset)

nFAULT asynchronous kill (existing pattern); |i_a| measured > 0.27 A; observer |i| > 0.27 A;
|ω| > 25 rad/s; encoder invalid, weak or stale > 1 ms; actor overrun (result late for its tick);
UART keepalive missing > 250 ms; run-time cap (e.g. 20 s); reset button. The 0.10 A bench CC limit
remains the outer hardware bound.

## Verification before power

- Python reference of the full controller I/O (float). The RTL controller I/O is checked against
  it per tick in closed-loop simulation with a behavioural plant (Verilator).
- Trip tests for every protection path, ARM refusal paths, keepalive loss.
- Vivado timing/DRC clean, then volatile programming only.

## Bench sequence

1. P1 inference parity on silicon (no motor power).
2. S2 sensors: encoder rate and health, current zero/noise, motor power ON but EN low.
3. Calibration only (bounded 0.6 V), with logs reviewed.
4. C-RL at reference 0, then ±3 rad/s, then ±10 rad/s, each for a short bounded run with telemetry.
