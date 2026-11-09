# Curve transitions — research result and handoff

William: the clearest first experiment is to anticipate the model's changing request, especially its exit, with a small preview correction. Simply increasing steer delay does not do this in the van's V0 controller. The wheel usually follows a request that itself unwinds slowly. A bounded preview moved the requested entry about half a second earlier and requested straight about 0.45 seconds earlier in replay. These are target timings, not measured improvements in the van.

The moderate preview improves the offline logged-plan score, but worsens some curves and raises reversals. Full-strength preview and inverse compensation fail. “Let go” is not established: there are too few clean hands-off zero-command intervals, and conservative return detection almost never activates in recorded curves. **Recommendation: hand off preview-only for shadow logging, with both road intervention params off. Do not enable active return or promote this as a validated steering improvement.** No device was accessed or changed.

## What was measured

All five requested routes were analyzed. Continuous autonomous events require >5 m/s, no driver override, |driver torque|<500, no blinkers, no timestamp gap >160 ms, |requested acceleration|>0.4 for >=1 s, peak >=0.5 m/s², three seconds of context before and six after. This excludes takeovers: the table is the surviving autonomous population, not all problematic curves. Entries already above a crossing threshold at the left boundary are censored rather than called zero-lag. “Wheel” below is the logged VehicleModel-equivalent acceleration; body is vEgo × calibrated carControl yaw rate. No VEHICLE_DYNAMICS.LAT_ACCEL is used.

|Route|Detected events|Plan exit 80→20%, median seconds (n)|Wheel reaches 20% after plan, seconds (n)|Body reaches 20% after plan, seconds (n)|Uncensored entry wheel lag, seconds (n)|
|---|---:|---:|---:|---:|---:|
|1a7|8|3.10 (5)|0.80 (5)|0.30 (5)|0.30 (1)|
|1aa|10|1.80 (7)|0.31 (7)|0.30 (7)|0.30 (8)|
|1ab|19|3.00 (13)|0.50 (12)|0.40 (13)|0.40 (7)|
|18c|15|2.99 (15)|0.30 (15)|0.10 (15)|0.30 (14)|
|19c|17|4.75 (14)|0.30 (14)|0.35 (14)|0.30 (13)|

This supports a slow request plus additional steering lag. It does **not** establish that the road is already straight throughout the entire plan decay. Cubic 15 m origin curvature from calib3, smoothed over 0.5 s and masked by lane confidence, reaches the same exit threshold relative to plan at route medians −0.35/+0.30/0.00/−1.75/0.00 s for 1a7/1aa/1ab/18c/19c. It is another model-derived estimate, with poor/variable closure, not surveyed road truth. Subtracting road and plan timings is not a causal decomposition.

During the model's 80→20% exit, median fractions of command samples still pushing into the curve are 40/49/58/38/76% by the same route order. Holding torque is real, but often needed to follow a still-curved plan; it cannot all be called controller error or friction. Wheel and body timing are not additive independent delays. An aggregate first-order 1.06 s time constant is not a 1.06 s transport delay; a 20% threshold lag of 0.3 s is not inconsistent with that fit.

### Natural self-centering and maneuver data

A clean candidate requires |CAN command|<0.028, |wheel|>5°, speed>5 m/s, |raw driver torque|<100, steeringPressed false, continuously >=0.3 s. Routes 1a4/1a5 each provide **zero** such intervals. They have approximately 143/124 s of zero-command large-angle moving samples, but median absolute driver torque is 1320/1710 raw units. Manual driving is not a hands-off coastdown experiment.

Only one eligible interval each appears in 1a7 and 1a3: median inward rates −0.1 and +0.2 deg/s. No eligible interval appears in the other four requested autonomous routes. This is insufficient to estimate passive return dynamics. During eligible engaged inward-motion samples at >5° the median rate is 0.5–0.8 deg/s, but this is a conditionally selected sample statistic, not an event-matched self-centering comparison.

There are 106/131 >=0.15 s nonzero probe bouts in 1a3/1a7. In the first 0.5 s after the probe ends, median absolute command remains 0.0676/0.1099, and median inward rate is −0.1/0.0 deg/s. Ending the offset pulse does not release the held baseline command. The pulse data cannot supply the missing free-return measurement. Detailed bouts are retained in curves/probe_releases.json.

### Does future model acceleration really anticipate the plan?

Using only a frame's acceleration.y at t=0 and future t, compare its predicted change with the subsequently logged desired-acceleration change. No future logged observation enters the candidate. At a 1 s horizon, forecast/persistence RMSE is 0.080/0.098 (18c), 0.111/0.114 (19c), 0.124/0.123 (1a7), 0.096/0.123 (1aa), 0.109/0.115 (1ab) m/s². Correlation is 0.52–0.72. At 0.7 s several routes are worse than persistence; 1.5–2 s changes are more predictable, but carry more risk of turning before the road.

The add-on uses **difference-anchored** preview: current target + 0.5×[a(t+1 s)−a(t)], bounded ±0.3 m/s² and filtered with 0.25 s time constant. A halfway sample must agree in direction and the predicted change exceed 0.05. Difference anchoring avoids replacing action.desiredCurvature's current feedback with the trajectory's biased zero-time acceleration. ModelConstants.T_IDXS is used, not uniform index spacing. Production must supply actual plan age; calib supplies age zero as an explicit approximation.

## Generation log

Each candidate below was a fixed hypothesis. Held-out results were inspected to reject candidates, not used to tune further numerical gains. Training routes and held-out routes remain as defined in BRIEF_COMMON.

|Generation|Attempt|Evidence and decision|
|---|---|---|
|G0|Characterize timing, zero-command intervals, future forecast|Main available lever is anticipation of changing plan. No defensible passive return time. Keep the distinction between first-order lag and pure delay.|
|G1|50% 1 s preview, ±0.3 accel, 0.25 s filter|Logged-plan simulated cost 0.17869 vs baseline 0.26511 (−32.6%); reversals 1.0829 vs 1.0004/s (+8.2%). Candidate for **shadow logging**, not promotion.|
|G2|100% preview, same bounds/filter|Cost 0.38925 (+46.8%). Reject; stronger anticipation overcompensates.|
|G3|Partial release of holding torque, delta cap ±0.20, slew 0.5/s|No aggregate calib improvement. Only one of 58 curve windows changes materially, with tracking-error ratio 3.85× baseline. Reject for first A/B.|
|G4|Target return velocity, proportional rate error, 0.12 return-command cap, same delta/slew limits|Same rare intervention and regression as G3. Current target is usually already near actual wheel position; trigger correctly stays mostly off. No evidence for relaxing it.|
|G5|Preview + active return|Cost 0.17869 in calib, effectively preview alone. Curve mean MSE slightly worse than preview alone, 0.002751 vs 0.002716. Keep return off.|
|G6|First-order inverse lead: 0.5×1.061×[a(0.5)−a(0)]/0.5, bounded/filter as above|Cost 0.43085 (+62.5%). Reject. Derivative noise and inverse mismatch outweigh modeled lag correction.|

![Offline candidate comparison; calibration gate fails](curves/candidate_cost.png)

The calib table uses the same 20 held-out window/band rows, weighted by scored sample count (7637 samples total), real V0/NNLC, plant v2, and logged-plan mode. Saturation is zero for all these selected held-out windows, so this experiment cannot establish relief of the wide-curve saturation problem. baseline is calib/evaluation.json's logged-plan rows; candidate outputs have unique curves_*.json names. Every ranking_authorized flag is false.

A separate matched **58 × 14 s curve-window** check runs all six baseline/preview/full/release/active/combined variants with real LaC at 100 Hz and cache motion/plan at 10 Hz. Four seconds of context precede the detected entry; ten seconds are scored. For moderate preview, median paired MSE ratio is 0.549, but **12/58 worsen**, worst ratio 1.64. Full preview worsens 49/58, worst 4.74×. These windows overlap where event geometry overlaps; they are not 58 independent statistical trials. The short window may end before a long curve's exit; native event replay below covers exit timing separately.

A full Smith predictor or preview MPC was not implemented. Plant v2 identifies a lumped frictional first-order response, not a transport delay plus separately identified motor/column dynamics. The inverse generation already fails; implementing a predictor around an unidentified delay split would add unjustified state, not resolve that failure. A future MPC needs a validated regenerated plan, uncertainty bounds and lane constraints; the current lane loop still fails its calibration gate.

## Replay, calib3 and perception checks

- **Native replay:** apply the standalone return module to the validated harness's cached NNLC V0 output, native wheel/rate and as-of plan. 37 detected events in 1a7/1aa/1ab, three return modes. At uncensored thresholds, preview target reaches 80% on entry **0.500 s earlier median** (n=25, p10–p90 0.141–0.977), and 20% on exit **0.450 s earlier** (n=25, 0.141–0.796). Return's median changed-command fraction and peak delta are zero. This path does not regenerate native LaC commands for the changed preview target; it explicitly reports a target lead. Detailed event rows: curves/native_replay.json.
- **Real-LaC fixed-motion replay:** experiment.py covers all five routes. Moderate preview changes commands >0.02 during a median 86% of event-window frames, median peak delta 0.531 normalized command (p90 1.044). That is a substantial controller response despite bounded target correction, because feedback/PID motion is held fixed. It is not the torque that would persist after the van moved differently. This is a reason to require shadow logging. Native harness cache commands are opposite acceleration sign; conversion is explicit in native_replay.py.
- **Calib3:** existing reconstructed-road lane evaluator with its consistent-future surrogate, preview factory. Logged-plan reproduces cost 0.17869. Lane-loop cost 0.62342 vs baseline 0.65414 (−4.7%); reversals 2.3491 vs 2.3255/s (+1.0%). Ranking still false. The smaller lane-loop difference and failed closure prevent interpreting the logged-plan improvement as tighter actual lane tracking.
- **Perception:** uses perception.metric.perceptible_roughness on wheel-equivalent angle and calibrated body acceleration, never command reversals as roughness. Across curve plant runs, median index is zero; preview max is 0.425 vs baseline max zero. Fixed-motion replay has identical output-based roughness by construction (median 0.463), despite altered torque. Model/friction smoothing can hide perceptible effects; zero simulated score does not certify an imperceptible road change. Slow exit is a response/lag issue below the primary >=0.5 Hz roughness bands, so event timing remains separate.
- **Overshoot:** 30 sign×speed×mode straight-return plant-grid cases at 5/10/20/30/35 m/s show no opposite-sign acceleration. Some do not settle below 10% in ten seconds because friction leaves residual angle. The grid uses a simple proportional controller and approximate bicycle wheel conversion, not production LaC or an inertia-resolving EPS model. Seven tests also check center/stopping guards, signs, stale/invalid/driver bypass, bounds, slew and 1000 randomized cases. These are rejection checks, not a physical no-overshoot guarantee.

## Deliverables, checks and exact next step

`tools/odyssey_sim/curves/transition.py`: NumPy-only, default-off runtime class with preview, partial-release and active-return options. `controller.py`: calib factory adapters. `README.md`: exact pre-LaC preview and post-LaC return wiring, required sign conversions, param registration, timestamp contract, PID freeze/anti-windup, reset and telemetry requirements. Do not put preview only inside OdysseySteer.apply: LaC has already consumed its target there.

Tests: **7 passed**, via tools/test_runner.py. Ruff check: **pass**. Compileall: **pass**. Host microbenchmark: 10,000 calls, p50 9.9 µs, p99 12.6 µs on Mac Studio, not a comma 3X CPU measurement. A target-device timing claim remains unverified. Production curvature/jerk and CAN limits still apply; the standalone API bounds its intervention rather than modifying stock behavior on invalid data.

All artifacts are in curves/ and the requested scratch area. No commits, staging, pushes, device access, runtime hook edits or other-worker edits. The repository has no ARCHITECTURE.md/check onboarding contract; the scoped README supplies this module's map without altering shared files. As the research child, I performed the work and self-review locally; I did not create nested reviewer agents. Self-review found and fixed censored entry timing, malformed-plan fallback, and an analysis performance bug that repeatedly decompressed native caches. No road improvement is claimed.

**Exact next step for the lead:** integrate preview-only behind the off-by-default SteerCurvePreview param, preserving clip_curvature and recording original/adjusted requests in shadow mode. Do not change learned delay or enable return in the same arm. The active/release modules are research artifacts pending stronger evidence.

## Road A/B plan

1. First do a normal supervised daylight drive with preview calculations logged but no command changes. Repeat familiar gentle left/right curves at comparable speed. Exclude driver input, blinkers, poor lane confidence, construction and maneuver mode. Include >=20 mph because the plant has no low-speed validation. Establish whether preview predicts a genuine road transition, rather than only a future policy correction.
2. If shadow behavior is acceptable, compare stock against moderate preview only in alternating route loops, with sufficient separation/reset at each boundary; pair **whole curves**, not 60 s blocks that switch mid-turn. Start with familiar gentle curves and normal driver supervision. Do not deliberately provoke saturation or hands-off release on public roads.
3. Score original-plan and actual wheel/body threshold times, origin lane offset at entry/apex/exit, outside-lane maximum and RMS, saturation fraction, driver overrides per whole curve, opposite-side crossing near straight, perception output metrics and William's exit feel. Keep speed and curve direction strata. Require repeat passes in each arm; report the distribution and worst curve rather than only a pooled average. Reject if the earlier request produces earlier lane cutting, more overrides, extra overshoot, or higher perceived roughness. The offline 33% score is not a required road-effect size.
4. Keep active return off until a controlled, appropriately supervised measurement establishes what near-zero assist actually does at known speed/angle with low driver torque. Record wheel rate and v*yaw for held assist, zero assist, and a small bounded return request. Pulse-offset removal alone does not meet that definition. A later return-only A/B needs its own anti-windup and inertia/overshoot evidence.

## External research

[SAE 2015-01-2729](https://saemobilus.sae.org/papers/active-return-center-control-strategy-steering-wheel-sensor-electric-power-steering-system-2015-01-2729): the public abstract describes return-state recognition from wheel angular velocity and steering torque, then double-loop PID compensation. This motivated an explicit return state and velocity-error term; our module is not a reproduction of the paper's inner motor-current loop.

[SAE 2008-01-0499](https://saemobilus.sae.org/papers/sliding-mode-return-control-electric-power-steering-2008-01-0499): the abstract attributes poor return to friction, uses rotational column dynamics and a switch between assist and return. Sliding-mode discontinuous switching is not copied into this noisy, delayed torque interface.

[US6408234B1](https://patents.google.com/patent/US6408234B1/en): describes estimating hysteresis from torque samples at center crossings in opposite directions, then compensating it. This motivates measuring hysteresis rather than assuming friction is constant. It does not establish this van's friction or passive return rate. SAE full texts were not accessed; only their public abstracts were used.
