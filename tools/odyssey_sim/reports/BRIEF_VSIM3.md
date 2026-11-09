# Brief: video simulator, pass 3. Can it tell controllers apart, and does it predict the road?

Read these first. They are in `~/Code/odyssey-pilot/tools/odyssey_sim/reports/`, now committed: `BRIEF_COMMON.md`, `BRIEF_VSIM.md`, `BRIEF_VSIM2.md`, `report_vsim.md` and `report_vsim2.md`. Then read `tools/odyssey_sim/vsim/` and its README, committed at `b68493d15`. Their rules all apply.

- **onnxruntime** is now installed in the repo's `.venv`, so no scratch dependency path is needed.
- **Evidence files** go in `scratchpad/vsim3/` in this scratch directory.
- **Write scope:** write only in `tools/odyssey_sim/vsim/` and the scratch directory. Never touch the comma device, and never commit or push.

## The critique of pass 2

Pass 2's closure result on 1a3 matched the logged wander within 1.2%. But the simulated pose never moved more than 4.5 cm from the logged path. With a perturbation plant, that means the simulated wander is mostly the logged wander carried over, so a close match was close to guaranteed. The ±25% plant variants all passing is consistent with that: it may show the metric can't discriminate, not that the simulator is robust.

The question a ranking simulator must answer is this: **when the controller changes, does the simulator predict the change the road would show?**

## Tasks, in order

### 1. Discrimination: does changing the controller change the simulated lane behaviour?

Run deliberately different controllers on many windows: every eligible autonomous window of 45 s or more on 1a3 and 1a7, not one hand-picked straight. The 1a7 video is arriving now.

- Stock NNLC (the logged controller)
- No NNLC (V0 torque only)
- V0 with KP ×0.5 and ×2
- Van models V1, S3 and S5 (`nnlc/odyssey_models/`; all candidates are in `~/comma-logs/odyssey-nnlc-models/`)
- `FrictionAssist` (`openpilot/sunnypilot/selfdrive/controls/lib/odyssey_wander.py`)
- A deliberately bad controller: steering delay +0.3 s, or the feedforward halved

For each, report the difference against the logged-controller run, paired by window, with bootstrap confidence intervals:

- 7–20 s lane-offset RMS
- the spectrum
- maximum offset
- how far the pose departs from the logged path

**Expected results:**
- **The bad controller** should produce visibly worse wander. If it doesn't, the simulator can't rank anything; find out why (plant authority, perturbation size, metric).
- **The KP and model variants** should separate when they genuinely differ.

### 2. Natural-fit bias check

Pass 2 measured the planner's response to injected weaves at 0.6–1.2 m/s² per metre. A regression on natural driving predicted 0.18–0.34, at matching phase (about −120° to −130°).

Measurement noise in the regressors attenuates regression coefficients toward zero. The model's lane estimate jitters about 3 cm per frame. That attenuation could explain the 3× gap.

Test it on the simulator itself, where the true response is known:
1. Run the natural-fit regression on free-running simulator output.
2. If it recovers about a third of the injected gain, the injected measurement stands and the natural fit is biased.
3. If it recovers the full gain, the gap is real and needs explaining.

### 3. Held-out closure, with curves

Do this once the held-out video is local.

The copy from the comma runs at about 2 MB/s:

| Route | Arrival |
|---|---|
| 1a7 | Arriving now |
| 1aa, 1ab | Likely around 11:00–11:45 CST today |
| 19f, 1a0, 1a2 | After that |

- **A segment is ready** when both `fcamera.hevc` and `ecamera.hevc` exist and the next segment's folder has started filling.
- **If you finish the training-route work before the held-out video arrives,** wait for it: check every few minutes rather than stopping.
- **No tuning on held-out data.**

Run every eligible window with the logged controller, and add the curve metrics from `tools/odyssey_sim/curves/characterize.py`: how wide the van runs mid-curve, and the exit release timing against the plan.

Report two things per window:
- the simulator against the logged values;
- how far the pose departs from the logged path, so it's clear how much of each match is carried over from the log.

### 4. Pre-registered predictions for the next road drive

The van is now running a road A/B: `NnlcModelAB` rotates stock NNLC, S3 and S5 every 180 s on the monotonic clock. See `odyssey_ab.py` and `nnlc.py update_model_ab`. Friction assistance is off. Scoring uses `tools/odyssey_wander.py <route> --ab nnlc_model`, which skips each block's first 30 s.

**Before any such drive exists,** write `scratchpad/vsim3/predictions.json` and a short table in the report. For S3 vs stock and S5 vs stock, give the predicted change in:
- 7–20 s lane-offset RMS;
- curve run-wide distance;
- body lateral jerk.

Each prediction needs an interval, computed on all video routes and stated in the units `odyssey_wander.py --ab nnlc_model` reports.

These predictions get scored against the real drive. That is the real validation of the simulator, and it can't be gamed afterwards.

### 5. Rankings, if tasks 1 and 3 hold up

Rank all candidates with paired windows and uncertainty, leading with tracking, then curves, then body motion. Use the plant ±25% variants as a robustness check on each ranking. Then propose the next generation of ideas the simulator makes testable, with your best guess at their effect.

## Deliverables

- Update `tools/odyssey_sim/vsim/`: code, README and tests.
- Write `scratchpad/report_vsim3.md`:
  - a plain-language summary for William on top;
  - the generation log;
  - per-task evidence;
  - `predictions.json` written before any new drive exists.
