# Brief: video simulator, pass 2

Read these first: `BRIEF_COMMON.md`, `BRIEF_VSIM.md`, and the pass-1 report `report_vsim.md`, all in this scratch directory. Then read the code in `~/Code/odyssey-pilot/tools/odyssey_sim/vsim/`. Their rules all apply. Continue in `vsim/` and build on pass 1 rather than restarting.

## What pass 1 established

- **The real driving model replays the van's recorded video on the Studio.** Curvature correlation is r=0.99983 against the log, and it runs at 31.7 fps, faster than real time. This is the foundation, and it is solid.
- **Lateral image shifts work:** the measured slope is 0.91–0.95.

Two results blocked pass 1. Both come from gates in `BRIEF_VSIM.md` that were badly designed. Replace those gates as described below. Don't weaken them.

## Correction 1: the zero-injection plant gate was wrong; use a perturbation plant

Pass 1 fed the logged commands open-loop into the calibrated plant. It drifted 160–190 m in 60 s. That result is expected and says little:

- **Integration error:** a constant acceleration error of 0.1 m/s² integrates to 0.5 × 0.1 × 60² = 180 m.
- **Every plant would drift:** any plant, and the real van itself, diverges quadratically when commands are replayed with no camera feedback.
- **The bias is unmodelled road:** at the scale of a 0.1 m/s² error it comes from crown, roll, wind, alignment and road curvature, none of which the plant models.

The simulator should take every one of those disturbances from the log and simulate only the difference our controller makes. Use a perturbation plant around the logged trajectory:

`a_sim(t) = a_logged_body(t) + [P(u_sim)(t) − P(u_logged)(t)]`

- **P** is the calibrated nonlinear plant (`tools/odyssey_sim/calib/`), run on both command histories with the same speed, roll and initial state.
- **What cancels:** bias, crown and road curvature all cancel.
- **What stays:** friction, the deadband and lag still shape the difference, through P's nonlinearity.

Zero injection is then exact by construction, so it is no longer a test. Validate the perturbation response instead:

1. **Characterization pulses.** Drives 1a3 and 1a7 contain the odyssey_suite torque-pulse maneuvers: 0–8% pulses held for 0.5–1 s with lane keeping frozen. See `tools/odyssey_eps_report.py`, item 13 in `ODYSSEY.md`, and `curves/probe_releases.json`. The perturbation plant must predict each pulse's body yaw-rate and lateral-acceleration response relative to the no-pulse holds: amplitude, timing, and the left/right difference.
   - Fit only on 1a3, validate on 1a7.
   - Report per pulse size, including the small sizes inside the deadband.
2. **Native timing check.** Rerun the pass-1 drift diagnostic with native 100 Hz applied commands instead of the 10 Hz cache. It's cheap, and it tells us whether resampling matters.
3. **Sensitivity.** Rerun the closure gate with plant gain, friction and lag each varied by ±25%. The gate decision should not hinge on one parameter.

## Correction 2: the static-yaw test was kinematically inconsistent

A fixed 0.5° yaw warp shows the van heading 0.5° off the lane. But the lane offset in the video never drifts the way it must when a van holds that heading (v·ψ, about 0.2 m/s at 25 m/s). The model reads motion over time through its image and feature history. It sees a heading that produces no drift, so under-reporting the heading is a reasonable reading of contradictory evidence. In the closed loop that contradiction cannot arise, because `dy` evolves from `dpsi`.

Keep the static-yaw result as a diagnostic. The gate becomes **kinematically consistent perturbations**:

- **Signal:** inject `dy(t)` with `dpsi(t) = ḋy(t)/v` (road curvature unchanged).
- **Shapes:** sinusoids at 7, 10, 15 and 20 s periods, 0.1–0.3 m amplitude, plus a few ramps and steps.
- **Warm-up:** confirm the warp covers the warm-up frames and history buffers consistently.
- **Measure, per period:**
  - the gain and phase of the perceived offset and heading against the injected ones;
  - the gain and phase of the plan's desired lateral-acceleration response.

That plan response is the planner's transfer function, the thing earlier passes tried and failed to fit with a formula (R² 0.44–0.64).

Compare it with natural evidence. On the training routes, fit how the logged model's desired accel responds to its own perceived offset and heading in the 7–20 s band. The simulator's injected-perturbation response should agree in gain and phase. Report the agreement or the gap honestly.

## Then: close the loop now

Implement the free-running `module:factory` mode.

- **Loop:** dynamic `dy`/`dpsi` reprojection of both cameras, then the real model, then the real controlsd lateral math at 100 Hz, then the real controller (through the existing replay adapter, with isolated scratch Params), then the perturbation plant, then the pose update.
- **Controller identity:** before comparing, verify which controller each drive actually ran, from initData and carParamsSP. Note that NNLC A/B and model overrides didn't exist on those drives.
- **Gate 4 (closure):** on held-out 1aa/1ab, without tuning on them, free-running with the logged controller must reproduce, within about 25%, per window:
  - the logged 7–20 s lane-offset RMS, its spectrum and period;
  - how wide the van runs mid-curve;
  - the curve-exit timing.
- **Ranking, once it passes:** rank with paired windows and uncertainty, leading with tracking:
  - stock NNLC;
  - the van models V1/S3/S5 (in `nnlc/odyssey_models/`; all candidates are in `~/comma-logs/odyssey-nnlc-models/`);
  - `FrictionAssist` (`odyssey_wander.py`);
  - the curve preview (`curves/transition.py`).

  Then propose the next ideas the simulator makes testable.

## Video arrival

The copy runs at about 2 MB/s.

| Route | Status |
|---|---|
| 1a3 | Arriving now (segments 0–21 at the time of writing) |
| 1a7 | About 1 hour after 1a3 |
| 1aa, 1ab | Roughly 2 to 3 hours out |
| 19f, 1a0, 1a2 | After that |

- **A segment is ready** when both `fcamera.hevc` and `ecamera.hevc` exist and the next segment's folder has started filling.
- **Order of work:** do corrections 1 and 2 and the closed-loop mode first, on 1a3 and 1a7, while the held-out video lands.

## Deliverables

- Update `tools/odyssey_sim/vsim/`: code, README and tests.
- Write `scratchpad/report_vsim2.md`:
  - a plain-language summary for William on top;
  - the generation log;
  - per-gate evidence.

If gate 4 fails, give the measured reason and the next step, as in pass 1.
