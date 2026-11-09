# Simulator calibration

Read `scratchpad/report_calib.md` in the task's supplied scratch directory for results and limits. The full calibration gate currently fails; do not rank or deploy controllers from these scores.

- `extract.py`: supplements the immutable EPS caches with carControl body motion, vehicleParameters, torqueState and tuning.
- `identify.py` and `body_fit.py`: scipy-only fitting on training routes; use the supplied wander venv with `PYTHONPATH=$PWD`.
- `plant.py`: schema-v2 plant; state is steering-equivalent acceleration plus calibrated yaw-derived body acceleration. Positive command means positive acceleration, opposite controller output.
- `controller.py`: real V0/NNLC adapter with logged tuning, pitch, delay and quantized angle feedback; optional V2 candidate factory.
- `evaluate.py`: one-command baseline and candidate hook, held-out masks, native command metrics and explicit calibration gate. Reads existing extraction caches. Corrects legacy cached road and offset signs in memory.
- `check.py`: scalar API and independent lane-closure checks.
- `report.py`: renders results and provenance into the requested scratch report.

From repo root:

```sh
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.check
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.report
.venv/bin/ruff check tools/odyssey_sim/calib
```

`evaluate --model /absolute/model.json --output candidate.json` accepts 18-input NNLC models. Other contracts use `--controller module:factory`. The factory receives `(metadata, Config)` and returns an adapter exposing `update(target_accel, measured_accel, state, future_plan)` returning normalized acceleration-positive torque, and `measured_lataccel` containing its actual feedback measurement. Use `--logged-plan-only` for acceleration-loop diagnostics. Never interpret a candidate score as passing the baseline calibration gate.

Schema v2 `steering` vector: gains at 10/20/30 m/s, lag seconds, friction at those speeds in m/s², roll gain m/s²/rad, bias m/s². `body` vector: gains at 10/20/30 m/s, lag seconds, residual roll gain, bias. Body lag at the fit bound is unresolved at the 10 Hz fitting rate.
