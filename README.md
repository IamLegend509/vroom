# vroom — motor RL harness (sim only)

Main repo. `third_party/gym-electric-motor` is a submodule (plant + PI baselines only).
All RL pipeline, motor registry, run viz live here.

## Layout

- `motors/bldc/` — thin wrapper over submodule plant, nominal config registry.
- `rl/inner/` — frozen inner current DDPG (from-scratch, no PI data).
- `rl/outer/` — outer speed RL at 1 kHz over frozen 10 kHz inner.
- `rl/experiments/` — capacity / reward / horizon study adapters.
- `ui/` — clean run viewer: status + key metrics + 3 graphs. No motor animation.
- `configs/motors/` — new motors go here as YAML (assumed sim params, not measured).
- `hw/arty/focmini/` — FPGA FOC baseline, direct sensor diagnostics and bounded
  motor tests. See the [hardware handoff](hw/arty/focmini/HANDOFF.md) for current
  build/bench status; accurate closed-loop motion is not commissioned.
- `hw/focmini/` — ESP32 bring-up firmware for the DFRobot SimpleFOCmini
  (DRI0058), with PlatformIO and native ESP-IDF build paths. Kept separate from
  the simulation harness.

## Scope

The simulation and RL harness remains simulation-only. `hw/focmini/` and
`hw/arty/focmini/` are explicitly authorized motor-control targets; they do not
change the RL pipeline. No model export or quantization is included.
Final learning: from-scratch motor interaction, no PI demos/imitation/shadow PI.
See `AGENTS.md`.

## Quickstart

```bash
python -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
git submodule update --init --recursive
python -m pytest -q
python ui/render.py --run /path/to/results/bldc/<study> --out /tmp/vroom-ui
```

## PR system

- `main` is protected. Work on branches, open PRs, require tests + `git diff --check`.
- Never commit `results/`, `*.zip`, `*.pkl`, replay buffers, `lit/*.pdf`, logs.
- Record submodule SHA per run for reproducibility.
