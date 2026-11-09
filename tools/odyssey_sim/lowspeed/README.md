# Low-speed steering research

This directory is an isolated proposed controller, not an installed runtime patch.
The fitted plants and tuned controllers **do not establish a road improvement**.
Use the scratch report `report_lowspeed.md` for the decision and evidence.

## Map

- `controller.py`: numpy-only `AngleController`, `Gains`, and `Result`. Disabled by default; no Params writes. Stateful reference rate limiting, PI/PD, smooth friction and angle/rate feedforward, anti-windup, bounded effort, smooth speed blending, and a stall/authority flag.
- `plant.py`: numpy-only exploratory torque → wheel angle/rate model and scoring. Parameter order: three speed gains at 1/4/8 m/s, aggregate torque dead zone, centering load per degree, rate time constant, command delay.
- `extract_plan.py`: current-frame model yaw-rate/velocity horizons from low-speed-containing rlog segments. Uses the existing EPS caches and the original rlogs; never consumes future log targets as preview.
- `research.py`: route-disjoint identification, public segment-disjoint validation, stock replay cache scoring, differential evolution generations, human-input model and observed full-effort rate envelope.
- `diagnostics.py`: speed-stratified plant checks, manual release evidence, approximate installed-angle replay, and synthetic tight-turn stress on multiple plants.
- `candidate_gains.json`: unselected optimization results, retained for reproduction. These are not road-tuned defaults.
- `test_controller.py`, `test_plant.py`: public-interface tests.

## Reproduce

Run from `~/Code/odyssey-pilot`, branch `odyssey-sim`. All data outputs stay in the
session scratch directory embedded in `research.py`. The original brief supplies
that path. Existing `plant_data`, `calib/*_body.npz`, `plant_public.npz`, and
`nnlc_1a7/1aa/1ab.npz` are immutable inputs. The public cache is a deterministic
1-in-10 CSV sample, not all 250 hours.

```sh
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.lowspeed.extract_plan
# With the brief's scipy-capable scratchpad/wander/venv/bin/python:
PYTHONPATH=$PWD /path/to/scipy/python -m tools.odyssey_sim.lowspeed.research
PYTHONPATH=$PWD /path/to/scipy/python -m tools.odyssey_sim.lowspeed.diagnostics
.venv/bin/python tools/test_runner.py tools/odyssey_sim/lowspeed/test_controller.py tools/odyssey_sim/lowspeed/test_plant.py
.venv/bin/ruff check tools/odyssey_sim/lowspeed
```

## Proposed integration contract (lead owns integration)

Do not wire this in based on offline scores. If an experiment is approved:

1. Make an explicit default-off experimental parameter, separate from the installed
   `SteerLowSpeedAngle`, and instantiate one controller per controls process.
   There must be only one low-speed angle override; do not stack the installed one
   with this controller. Keep deadband compensation off for the first A/B.
2. Pass the actual `latActive`, `steeringPressed`, measured steering angle/rate,
   speed, and the upstream lateral controller output each 10 ms. Reset on
   disengagement, driver override and mode changes. Never omit those flags merely
   because `update` provides defaults for offline calls.
3. Convert desired curvature with the real `VehicleModel` and current calibrated
   roll/angle offset, exactly as the installed angle mode does. All input angles,
   rates and output torques have the `carState.steeringAngleDeg` sign. Raw Honda
   CAN torque and openpilot curvature have the opposite sign.
4. For preview, use one current valid model frame: curvature = yaw rate / forward
   speed, with a low-speed floor, then VehicleModel → steering angle for every
   horizon sample. Pass `(horizon_seconds, angles_degrees)`. Reject stale frames
   and account for the age of the model frame; do not feed future logged frames.
   The study uses a 0.4 s sample with model age <0.15 s, kinematic conversion and
   a 0.5 m/s denominator floor. It does not validate roll/transient tyre effects.
5. Preserve stock KP while this experiment is off. While enabled, coordinate the
   blend and KP cap with the upstream controller exactly once. On invalid inputs,
   returning the passed stock command cannot undo a KP cap applied earlier in
   the same frame: fallback policy must account for that upstream state.
6. `Result.limited` says the angle reference is rate limited. `authority_limited`
   says effort >0.8, error >10°, and measured rate <1°/s persisted for >1.5 s.
   Log it first; any user alert/handoff belongs in the existing alert framework.
   The controller does not release torque or silently disengage during a turn.
7. The 0.95 cap prevents literal 100% dwell, but is not a proof of useful effort.
   Score >0.8 effort and angular progress too. A physical capability model and
   authority threshold need road validation before automatic handoff decisions.

At >=8 m/s output is exactly upstream stock. The smoothstep blend is entirely
angle control <=4.5 m/s. Below 0.5 m/s the controller resets and returns upstream
stock; standstill steering and stop/start transitions are not road validated.
A unit test found that slewing the final blend could cause a large jump when
crossing 8 m/s; slewing the angle arm before blending fixes that discontinuity.
