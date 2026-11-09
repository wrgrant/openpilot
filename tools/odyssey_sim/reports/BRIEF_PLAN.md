# Brief: is the lane wander coming from the plan? Test plan-side fixes and other driving models

Read `BRIEF_COMMON.md` first; its rules apply. Then read, in `~/Code/odyssey-pilot/tools/odyssey_sim/reports/`:
- `report_wander.md`
- `report_vsim.md`, `report_vsim2.md` and `report_vsim3.md`

Then read `tools/odyssey_sim/vsim/` (README and code, commit `e6e9b55bd`).

- **Write scope:** write only in your own `tools/odyssey_sim/plan/` and the scratch directory `scratchpad/plan/`. Import vsim; don't edit it. Never touch the comma device, and never commit or push.
- **Hands off the locked forecasts:** don't change or reinterpret `tools/odyssey_sim/reports/vsim3/predictions.json`. It is for a road test that hasn't happened yet.

## Why

Three findings now point the same way:

1. **The wander worker:** on clean highway stretches, the van's body motion follows 90–98% of the slow (7–20 s) lateral acceleration openpilot requests. So the steering mostly does what it's asked.
2. **The video simulator** runs openpilot's real driving model on the van's recorded video in closed loop. In it, changing the steering controller barely moves the wander:
   - stock, S3, S5, KP ×0.5 or ×2, and friction assistance all stay within ±0.5 cm of about 4–19 cm RMS;
   - only halving the feedforward makes it measurably worse (+1.2 cm).
3. **The driving model's response to a lane offset** lags the offset by about 120–130° at 7–20 s periods (`report_vsim2.md`). In a feedback loop, a lag that large is a recipe for a slow oscillation, and the wander's period is about 13–18 s.

**Hypothesis:** much of the wander originates in the plan, meaning the driving model's request plus the plan→van→camera loop, not in the steering controller. If that's right, the levers are upstream:
- which driving model runs;
- how its request is filtered or compensated;
- a lane-centring correction with phase lead.

Test the hypothesis. Then test the levers.

## Tasks

### 1. Decompose the request (open loop, real model, logged path)

On every eligible autonomous window with video (1a3, 1a7, 1aa, 1ab; train on 1a3/1a7, hold out 1aa/1ab), split the model's 7–20 s band request into three parts:
- the road's own curvature;
- the response to the van's actual offset and heading;
- a residual: request wander that would exist even if the van held a perfect line.

Use the injected-weave transfer function from vsim (gain and phase per period) rather than a regression; pass 3 showed regressions under-recover the gain. Where possible, measure the residual directly: replay the model with the van warped onto a smoothed version of its own path, so the offset seen by the camera stays nearly constant.

Report how many cm of wander each part accounts for, and how much of the 7–20 s wander a perfect controller could remove.

### 2. Other driving models

sunnypilot ships many driving models (Model Manager; the bundle lists are public at github.com/sunnypilot/sunnypilot-models). The van runs the default `driving_supercombo.onnx`.

1. Find which bundles can run on the Mac. They need ONNX or a format you can execute; the device-compiled tinygrad pickles target the comma's GPU. Note each one's input contract.
2. For 2–4 promising candidates, run the same video and measure:
   - the open-loop request residual from task 1;
   - the injected-weave transfer function (gain and phase at 7, 10, 15 and 20 s);
   - curve-entry timing against the logged plan.
3. Run each in the closed-loop simulator with the stock controller.

**Expected:** a model with less residual wander and less phase lag should wander less on the road. Switching models is one setting on the device, so this is a cheap road test if one stands out. Report which models the van could actually run: check the bundle metadata for the minimum version and runner type against this branch, `openpilot/sunnypilot/models/`.

### 3. Plan-side compensation

Prototype candidates in closed loop on the training routes, scored on held-out windows. Each must be numpy-only, run within about 1 ms per frame, sit behind an off-by-default param, and fall back to the unmodified request on bad input.

Candidates:
- **Phase-lead filter:** a lead filter on the 7–20 s band of desired curvature, aimed at the measured lag.
- **Lane-centring term:** a correction from the model's own lane lines and lane-centre estimate, with derivative (heading) action. The wander worker's observer in `tools/odyssey_sim/wander/` is a starting point; it was rejected for drifting when camera updates drop out, so handle dropouts.
- **Residual smoothing:** smoothing of the request's residual component.

Watch for second-order effects:
- **Curves:** don't delay turn-in or slow the exit. Report curve run-wide and exit timing.
- **Body motion:** report lateral jerk.
- **Lane changes and low speed:** the candidate must stay off there.

## Validity

The simulator passed open-loop model fidelity (r=0.9998), and pass 3 showed it responds when the change is large enough. But it failed full held-out closure: 3 of 7 windows pass the spectrum check. So:
- **Lead with what the real model alone can measure:** tasks 1 and 2 open-loop.
- **Treat closed-loop results as indications, not rankings.** Report pose departure from the logged path for each, since larger departures stress the perturbation plant more.
- **Each finding gets a falsifiable road prediction:** state what a road A/B should show if the finding is right.

## Deliverables

- Code in `tools/odyssey_sim/plan/`: README, tests and CLI.
- Report `scratchpad/report_plan.md`, with:
  - a plain-language summary for William on top;
  - the generation log;
  - evidence per task;
  - for each lever that looks real, a recommendation and the road test that would confirm it.
