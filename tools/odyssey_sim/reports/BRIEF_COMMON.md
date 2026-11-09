# Shared brief for every Odyssey steering worker

William's 2018 Honda Odyssey (Nidec) runs our sunnypilot fork on a comma 3X: factory ACC for gas and brake, sunnypilot steers. He is asleep; work autonomously without asking questions, and keep going through as many failing generations as it takes. A failed attempt is data. Question every pass/fail criterion, including ones you inherited, and log each generation: what you tried, the metrics, and why it failed or helped.

## His goals, in priority order

1. Extremely straight tracking: solve the slow 7-20 s lane wander (ping-pong).
2. Low-speed steering: neighbourhood streets, intersections, below about 20 mph.
3. Turn pre-entry: start turning before the curve rather than running wide and catching up at 100% torque.
4. Turn exit: unwind promptly to straight. Today the wheel holds the turn too long and returns slowly; he would rather it returned faster, or briefly let go so the steering self-centres.
5. Anything else that makes it drive better.

Leave no stone unturned.

**The ultimate objective is never needing to hold the wheel.** William drives hands-off almost all the time and only grabs the wheel when a curve needs more torque than openpilot gives. Ripple or jitter that reaches the wheel rim does not matter to him, even where it is perceptible.

What matters, in order:

1. The van's actual path: lane position, no wander, making curves without running wide.
2. Avoiding takeovers: in curves, at low speed, on exits.
3. Body motion he would feel in the seat: lateral jerk or sway.

Do not reject a candidate for command reversals or wheel/hand-torque ripple alone. Treat smoothness as a constraint only where it shows up as body motion or worse tracking.

## Read first

- `~/Code/odyssey-pilot/ODYSSEY.md`: sections "What actually runs (code map)", "Measured facts about this van" and "Working on the van (pitfalls)". The controller is sunnypilot's `LatControlTorqueV0` plus the NNLC extension. Our add-ons apply in `controlsd` through `OdysseySteer`.
- The van logs liveParameters as `vehicleParameters`, and there is no `livePose`.
- Sign conventions are in ODYSSEY.md.
- `VEHICLE_DYNAMICS.LAT_ACCEL` is unusable. For body lateral acceleration use vEgo × carControl.angularVelocity[2], which is validated.

## Reports in the scratch directory

Scratch directory: `/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/`

| Report | What it covers |
|---|---|
| `report_plant.md` | Plant fit v1 |
| `report_nnlc.md` | NNLC vs stock open-loop replay. Validated harness: `tools/odyssey_sim/replay/` |
| `report_closedloop.md` | Closed-loop comparison on comma's Controls Challenge harness |
| `report_van_nnlc.md` | Van-specific NNLC V1/V2 |
| `report_calib.md` | Yaw-validated plant v2 (`scratchpad/calib/plant_v2.json`) and the one-command evaluator `python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --model <json>` or `--controller module:factory`. The lane loop does not close yet. |

Finished: `report_perception.md` (code: `tools/odyssey_sim/perception/metric.py`, README, stable API).

- Raw torque-sign reversals are a poor smoothness gate. In 55 matched windows NNLC had more command reversals but lower output roughness after the EPS.
- Judge smoothness on the post-EPS outputs: wheel motion, body motion and hand-torque ripple, scored with `perception.metric`.
- The hand-torque thresholds are provisional, because there is no N·m calibration.
- The driver-torque sensor does carry some command-correlated ripple, so don't assume friction hides everything.
- The metric gives zero excess roughness in 85-93% of 10 s windows on 1a7/1aa/1ab, which is consistent with William's report.

In progress:

| Report | Code | What it covers |
|---|---|---|
| `report_perception.md` | `tools/odyssey_sim/perception/metric.py` | Perceptible-roughness metric. Raw torque reversals may be imperceptible through his lossy, high-friction EPS. |
| `report_nnlc_smooth.md` | `tools/odyssey_sim/nnlc_smooth/` | Making the van NNLC smooth and better |
| `report_calib3.md` | `tools/odyssey_sim/calib3/` | Lane-geometry observer, so the lane loop and wander simulation close |

## Key measured facts

- EPS motor deadband: about 2.8% of command.
- Stick-slip friction: equivalent to 0.14-0.29 command, largest at low speed.
- Command-to-steering lag: about 0.9-1.06 s aggregate.
- Steering gain rises with speed.
- The model's lane-centre estimate jitters about 3 cm per frame.
- Wander: 5-18 cm RMS, 15-20 s period.
- The controller follows the request with a follow ratio of about 0.25.
- Below 15 mph, torque sits at 100% 61-92% of the time, driven by the KP_INTERP boost to 250.
- On 1aa/1ab: drivers took over in about 52% of curves, the van ran 13-29 cm wide mid-curve, and the model's desired lateral accel decays over about 3 s after a curve.
- On straights the van sat 8-18 cm right of centre. CameraOffset +0.12 is now set to counter this.
- Learned torque params: latAccelFactor 1.47, friction 0.145.
- Learned steer delay: about 0.32-0.35 s.

## Data

- rlogs: `~/comma-logs/04b44e4c4fe44232_<route>/<route>--<seg>/rlog.zst`. Read with `LogReader` given the file path.
  - Train: 00000174--f116099ddd, 0000018c--9a64dd12bc, 0000019b--67c64d0ac6, 000001a3--c16cc8db23, 000001a7--a7f62f4b3f.
  - Held out: 0000019c--0d91088e6e, 000001aa--ba236ee054, 000001ab--9191c0d55d.
  - Manual driving only: 000001a4--917ddc3202, 000001a5--555b3a0ea8.
- comma's public Odyssey data: `~/comma-logs/commaSteeringControl/HONDA_ODYSSEY_2018/*.csv`, about 250 hours.
- Reuse extraction caches in the scratch directory where they are valid.

## Rules

- Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`. Never switch branches, never `git add -A`, never commit; the lead reviews and commits.
- Write only in your own `tools/odyssey_sim/<yourdir>/` and the scratch directory. Import from other workers' directories but do not edit them.
- Runtime hooks are owned by the NNLC-smoothing worker: `openpilot/sunnypilot/selfdrive/controls/lib/odyssey_steer.py` and `nnlc/nnlc.py`. Other workers deliver add-ons as standalone modules, with a function or class and tests, plus integration instructions, and the lead wires them in.
- Runtime code must be numpy only, fit within about 2 ms per 100 Hz frame on the 3X CPU, produce bounded output, sit behind a param that defaults off, and fall back to stock on bad input.
- Python: `cd ~/Code/odyssey-pilot && PYTHONPATH=$PWD .venv/bin/python`.
- scipy venv: `scratchpad/wander/venv/bin/python`.
- torch venv: see `report_van_nnlc.md` (nnlc-venv).
- Tests: unittest `OpenpilotTestCase`, run with `.venv/bin/python tools/test_runner.py <paths>`. There is no pytest.
- Lint: `.venv/bin/ruff check`.
- Use `if __name__ == "__main__"` guards for multiprocessing.
- Never touch the comma device. Never push.
- Cite outside sources you use.
- Deliver a report `scratchpad/report_<name>.md` with a plain-language summary for William, then finish with a short summary message.
