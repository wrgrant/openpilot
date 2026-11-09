# Video simulator, pass 2

The real driving model now runs in a free-running lane loop with the real V0/NNLC controller and a differential plant. On a 53-second autonomous **training** straight, the baseline reproduces lane-offset RMS within 1.2%, the dominant period exactly, and the spectrum within 6.7% normalized absolute difference. All six ±25% plant variants complete with RMS error below 5% and spectrum error below 20% on that same training window. This is a useful step forward; it does not establish held-out closure or authorize controller rankings.

Two prerequisites are missing. The local 1aa/1ab camera video has not arrived. Also, 1a3 ran the old short-pulse suite, not the frozen-command/no-pulse holds assumed by the brief; those holds exist on 1a7. I did not fit on 1a7 and relabel that as validation. The unretuned plant shows material pulse amplitude, timing and directional mismatches there.

My recommendation: keep this implementation, finish the held-out video copy, and validate the existing model on those paired windows without tuning. For the pulse gate, recover a separate training drive with frozen holds, or explicitly revise the split before fitting. Meanwhile, treat the injected planner transfer function as measured perception evidence, not a proven controller-ranking tool.

## Scope and checks

- Continued `~/Code/odyssey-pilot`, branch `odyssey-sim`; changes are confined to `tools/odyssey_sim/vsim/` and the assigned scratch folder. No comma access, commit, push, branch switch, or deployment.
- Existing model replay and homographies are retained. Added dual nonlinear plants, consistent dynamic injections, native-command drift audit, pulse analysis, natural/injected transfer measurements, free-running `module:factory`, scratch-only Params, controller identity and tracking spectra.
- 14 unittest tests pass, including cancellation under nonzero roll, deadband response, derivative-consistent heading, existing exact SE(2) and projection tests, recurrent model-state and NV12 tests. See `vsim2/final_tests.log`. Ruff passes: `vsim2/final_lint.log`.
- The initial missing-module test failed before implementation. A first exact-cancellation test caught floating-point subtraction ordering; grouping the plant difference makes matching command histories exactly cancel.
- No extra agents were spawned. As the assigned research worker, I performed the three self-review passes described below.

## Generation log

| Generation | Experiment / fix | Result |
|---|---|---|
| G0 | Read prior implementation, briefs, logs and controlsd | Reused the pass-1 real-model runner; replaced obsolete acceptance gates. |
| G1 | Test then implement differential plant | Matching commands cancel exactly; 2% perturbation remains inside the deadband. This is an invariant, not validation. |
| G2 | Native applied-command drift audit | 155.77 / 159.68 / 185.16m endpoints; resampling does not explain the absolute-plant drift. |
| G3 | Four sinusoidal periods plus ramp and finite-rise step, warped throughout warmup | Offset gain ~0.92–0.94. Heading and planner transfer measured below. |
| G4 | Schedule-aligned pulse extraction | No frozen holds on 1a3; 114 scheduled windows on 1a7, including aborted/driver-contaminated windows. Removed driver overrides from scored means. |
| G5 | First real-controller integration | Caught NumPy scalars rejected by Cap’n Proto and a missing blinker field in the inherited adapter; fixed locally. |
| G6 | First requested 610–670s loop window | Native driver/activity eligibility fails during context; no valid closure claim. Selected a continuous autonomous training window instead. |
| G7 | 897–950s training loop, 15s recorded-state warmup | Completed, maximum relative pose ~4.5cm, close wander match. No reanchoring after release. |
| G8 | Self-review against real controlsd | Found smoothing-delay omission in inherited adapter; local adapter now passes lateralDelay + LAT_SMOOTH_SECONDS to real LaC, raw learned delay to NNLC extension, and actual curvature-limited flag. Superseded sweep stopped; final sweep rerun. |
| G9 | ±25% gain/friction/lag sweep | Final per-run evidence below; no held-out data was used for tuning. |

## Gate 1: retained open-loop fidelity

Pass 1 measured desired-curvature correlation r=0.999832 on moving 1a3. Existing evidence remains in `vsim/g6.json` and its traces. These pass-2 changes retain the model graph, preprocessing, pairing and original logged mode. No custom bundle was substituted.

## Corrected perception gate: consistent motion

1a3, scored 610–670s; comparisons use the common 610–665s portion of the existing zero-warp baseline: 1,100 warmed paired frames. Warps apply during all 15s of warmup and are retained in ONNX image/feature history. Positive phase means output leads the injected signal. Heading input is dy derivative / v. Ramp is triangular; step has a finite tanh rise so heading stays finite.

| Shape / period | Amplitude m | Offset gain / phase ° | Heading gain / phase ° | Plan accel per m gain / phase ° | Natural-fit predicted gain / phase ° |
|---|---:|---:|---:|---:|---:|
| ramp_15 | 0.2 | 0.923 / 0.5 | 0.715 / -12.7 | 0.605 / -125.7 | 0.197 / -131.1 |
| sine_10 | 0.2 | 0.935 / 2.3 | 0.711 / -5.0 | 0.892 / -119.1 | 0.259 / -117.1 |
| sine_15 | 0.3 | 0.923 / 2.0 | 0.668 / -15.5 | 0.678 / -128.5 | 0.195 / -133.9 |
| sine_20 | 0.2 | 0.937 / 0.8 | 0.776 / -16.6 | 0.569 / -128.7 | 0.180 / -138.1 |
| sine_7 | 0.1 | 0.928 / 3.1 | 0.704 / -0.6 | 1.180 / -118.4 | 0.342 / -107.2 |
| step_20 | 0.2 | 0.927 / 1.2 | 0.812 / -15.6 | 0.579 / -126.2 | 0.183 / -135.1 |

Natural training evidence: 8 autonomous 60s windows on 1a3/1a7, no held-out fitting. The 7–20s band-limited instantaneous fit includes perceived offset, heading and lane-curvature acceleration; coefficients are [-0.1051968389594909, -12.72026729579841, 0.7363722192177407], R²=0.801, standardized condition=2.76. Injected and natural perception both use cubic lane-origin fits over 25m. The final table column predicts the injected response from the measured perceived offset/heading using that fit, holding road curvature unchanged.

The transfer functions do **not** agree closely enough to declare this gate passed. The natural fit is observational, instantaneous and confounded by road geometry and feedback; its R² does not identify a causal phase response. The injected model response is directly measured, but only one video training route is available. Static-yaw pass-1 results remain diagnostic; no yaw multiplier was introduced. Heading gain is still ~0.66–0.80 even with consistent motion. The data do not establish that the old static contradiction was the sole cause.

## Corrected plant gate: pulse evidence

`vsim2/pulses.json` contains per-size, width and direction body/yaw RMSE, peak timings and full mean response traces. Responses use v × carControl.angularVelocity[2], never VSA LAT_ACCEL. Pre-pulse 0.2s context initializes both plants identically; applied-CAN histories are compared against the frozen pre-pulse command. Matched no-pulse responses are subtracted from both prediction and observation. Driver-override windows are excluded.

1a3 labels are `torque pulses 0.2s/0.4s`, `slow sweep`, and `nudge`; there are **zero** current `held pulses` runs. Therefore the requested fit-only-on-1a3 frozen-hold experiment cannot be executed from these logs. Existing calibration is retained without tuning. 1a7 contains 0, 3, 4.5, 6 and 8% scheduled levels; 1.0s 8% coverage is absent after schedule/run truncation. There are no nonzero frozen-hold pulses strictly below the ~2.8% motor deadband; 2% belongs to the older 1a3 suite. Do not claim inside-deadband pulse validation.

| Width s | Size % | Direction | Clean n | Observed end accel | Predicted end accel | Trace RMSE m/s² | Observed / predicted peak s |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 0.0 | -1 | 12 | 0.0000 | -0.0000 | 0.0000 | 0.37 / 1.07 |
| 0.5 | 0.0 | +1 | 11 | -0.0000 | 0.0000 | 0.0000 | 0.26 / 1.16 |
| 0.5 | 3.0 | -1 | 11 | 0.0011 | 0.0360 | 0.0219 | 0.85 / 1.19 |
| 0.5 | 3.0 | +1 | 11 | -0.0638 | -0.0158 | 0.0282 | 1.09 / 0.64 |
| 0.5 | 4.5 | -1 | 7 | 0.0133 | -0.0004 | 0.0216 | 0.65 / 0.54 |
| 0.5 | 4.5 | +1 | 9 | -0.0514 | -0.0319 | 0.0231 | 1.13 / 0.75 |
| 0.5 | 6.0 | -1 | 5 | 0.0338 | 0.0237 | 0.0280 | 0.45 / 1.19 |
| 0.5 | 6.0 | +1 | 5 | 0.0509 | 0.0280 | 0.0219 | 1.15 / 1.19 |
| 0.5 | 8.0 | -1 | 2 | 0.0826 | 0.0297 | 0.0595 | 0.14 / 0.93 |
| 0.5 | 8.0 | +1 | 2 | 0.0160 | 0.0184 | 0.0266 | 0.25 / 1.19 |
| 1.0 | 0.0 | -1 | 6 | 0.0000 | 0.0000 | 0.0000 | 1.34 / 1.48 |
| 1.0 | 0.0 | +1 | 6 | -0.0000 | 0.0000 | 0.0000 | 0.14 / 1.45 |
| 1.0 | 3.0 | -1 | 4 | 0.0329 | 0.0073 | 0.0405 | 0.39 / 1.06 |
| 1.0 | 3.0 | +1 | 3 | 0.0050 | -0.0446 | 0.0504 | 0.79 / 1.49 |
| 1.0 | 4.5 | -1 | 4 | -0.0326 | -0.0476 | 0.0291 | 0.97 / 1.49 |
| 1.0 | 4.5 | +1 | 4 | 0.0201 | 0.0004 | 0.0296 | 0.44 / 0.84 |
| 1.0 | 6.0 | -1 | 4 | -0.0292 | -0.0154 | 0.0338 | 1.49 / 1.49 |
| 1.0 | 6.0 | +1 | 4 | 0.0150 | 0.0145 | 0.0343 | 0.04 / 1.49 |

Direction is the suite torque sign, not a sign-normalized body output. End accel is the last 0.2s mean in the scored horizon. The 0.5s pulses score through 1.2s; 1.0s pulses score through 1.5s. Zero rows cancel by matched subtraction and are not independent validation. Peak times in noisy/near-zero rows are descriptive, not reliable latency estimates. Several nonzero rows have wrong sign or materially wrong amplitude; left/right agreement is not established. Means lack sufficient precision to separate all small-pulse effects from road disturbance; this is a failed/incomplete plant validation, not grounds for hand tuning.

Native timing audit replays exactly the same 60s windows as pass 1 with native CAN commands. Endpoints: 155.77, 159.68, 185.16m. Body RMSE: 0.1531, 0.1095, 0.1384m/s². Logged-yaw integration oracle remains zero. See `vsim2/native_drift.json`; this diagnostic no longer gates the perturbation plant.

## Gate 4: closure and parameter sensitivity

All four drives have initData NeuralNetworkLateralControl=1, EnforceTorqueControl=0, LagdToggle=1, CameraOffset=0 and carParamsSP model HONDA_ODYSSEY at the stock 18-input JSON path. NNLC override/A-B keys are absent, rather than inferred from later code. See `vsim2/identity.json`.

The final loop uses both dynamically warped cameras, the recurrent real ONNX model, real action interpolation and upstream clip_curvature, the real V0/NNLC implementation through a local extension of the existing adapter, native applied-CAN baseline, differential plant and exact relative-pose update. The controller consumes logged v/a/roll/pitch, learned tuning and steering-equivalent measured acceleration. Its rounded 0.1° angle matches the prior calibrated adapter. Native CAN cadence is nominal 100Hz, not synthesized exact 10ms; outside 5–25ms gaps abort. Params handles for production extensions and the adapter are rooted in `vsim2/params`.

Controller/model warm for 15s on recorded state before the 897s release; no state/pose recentering occurs afterward. This is a fixed training window chosen for eligibility and available camera pairs, not a held-out pass. No candidate tuning was performed.

| Plant variant | Completed | Lane RMS m | Logged RMS m | RMS error % | Period s | Spectrum L1 error % | Max pose m |
|---|---|---:|---:|---:|---:|---:|---:|
| base_1 | True | 0.1345 | 0.1329 | 1.2 | 17.67 | 6.7 | 0.0451 |
| friction_0.75 | True | 0.1375 | 0.1329 | 3.5 | 17.67 | 9.0 | 0.0434 |
| friction_1.25 | True | 0.1319 | 0.1329 | 0.7 | 17.67 | 19.7 | 0.1502 |
| gain_0.75 | True | 0.1380 | 0.1329 | 3.8 | 17.67 | 14.0 | 0.1391 |
| gain_1.25 | True | 0.1393 | 0.1329 | 4.9 | 17.67 | 11.3 | 0.0449 |
| lag_0.75 | True | 0.1362 | 0.1329 | 2.5 | 17.67 | 8.2 | 0.0440 |
| lag_1.25 | True | 0.1305 | 0.1329 | 1.8 | 17.67 | 12.6 | 0.0791 |

Lag sensitivity scales both steering and body lag; gain scales the steering-stage gain. Final sweep evidence files present: 7/7. Each result includes exact frequency-bin powers, not just summary medians. Spectrum L1 compares absolute band powers normalized by total recorded band power; it does not require each tiny bin to match within 25%. The native perturbation path and unchanged road geometry explain much of the training similarity, so one straight cannot establish general closure.

**Held-out gate 4 is unpassed.** No 1aa/1ab ready camera pair is present. There is no per-held-out-window curve width/exit evidence. The current CLI explicitly leaves these curve metrics unavailable and never authorizes rankings. Candidate ranking with uncertainty (stock NNLC, V1/S3/S5, FrictionAssist, preview) was not run because its prerequisite is unmet.

## Speed

- finalclosure_base_1: 38.6 pipeline fps including measured controller/plant time; 28.5 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7408883018069901, 'decode': 21.11764127697097, 'warp': 3.9321312269894406, 'model': 9.40213400189532, 'total': 47.6986240419792}.
- finalclosure_friction_0.75: 39.1 pipeline fps including measured controller/plant time; 30.2 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7350487492512912, 'decode': 20.95128290442517, 'warp': 3.9117310111178085, 'model': 9.211454620293807, 'total': 45.06564491597237}.
- finalclosure_friction_1.25: 38.9 pipeline fps including measured controller/plant time; 29.9 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7333273386466317, 'decode': 21.053519359324127, 'warp': 3.924468070617877, 'model': 9.279508302279282, 'total': 45.49152979097562}.
- finalclosure_gain_0.75: 39.1 pipeline fps including measured controller/plant time; 30.0 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7318940958939493, 'decode': 20.904936402861495, 'warp': 3.9334190532099456, 'model': 9.242394075670745, 'total': 45.26262612501159}.
- finalclosure_gain_1.25: 39.2 pipeline fps including measured controller/plant time; 30.2 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7302464884123765, 'decode': 20.769858927000314, 'warp': 3.9327037364710122, 'model': 9.217638222617097, 'total': 45.10076208302053}.
- finalclosure_lag_0.75: 39.1 pipeline fps including measured controller/plant time; 30.1 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7377489858772606, 'decode': 20.887659387139138, 'warp': 3.9332216972252354, 'model': 9.255446284019854, 'total': 45.13471195899183}.
- finalclosure_lag_1.25: 39.1 pipeline fps including measured controller/plant time; 30.1 fps including log loading; 1360 model frames. Timers: {'controller_plant': 0.7395449526957236, 'decode': 20.86811110243434, 'warp': 3.915654091455508, 'model': 9.278512427059468, 'total': 45.23122920899186}.

## Review, remaining work and exact next step

Self-review 1 checked differential algebra and geometric signs: fixed subtraction grouping, retained exact SE(2), and verified physical injection derivative. Self-review 2 checked experimental comparisons: subtracted no-pulse controls from prediction as well as observation, added pre-pulse initialization, excluded driver overrides, and refused the invalid 1a3/1a7 split substitution. Self-review 3 checked the production interface: fixed scalar types, blinkers, core smoothing delay, clipping flag, warmup/release semantics and failed-window evidence; reran the final sensitivity generation after the delay correction. No fresh delegated review was run.

Exact next step: when 1aa/1ab have stable paired camera segments (both files exist and the next segment is filling), identify native continuously autonomous windows with ≥15s context plus ≥45–60s score. Run the README factory command on each, unchanged. Add matched curve-event width and exit-release scoring against the existing curves tool before asserting full gate 4. Preserve paired windows and bootstrap uncertainty before ranking candidates. Recover appropriate frozen-hold training data to resolve the pulse gate first; do not fit on validation data.

Ideas the simulator now makes experimentally accessible, once validation passes: hysteresis-aware friction inversion using measured steering velocity; curve-exit release pulses; speed-dependent preview with saturation budgeting; gain schedules that target the measured 7–20s planner/plant phase margin. Judge actual lane position and body motion first. Command reversals alone are not rejection criteria.

Ready-pair file inventory is in `vsim2/video_inventory.json`; authored Python hashes are in `vsim2/source_manifest.json`. No new outside sources were used; inherited model/augmentation sources are cited in `report_vsim.md`. Code and local logs are the primary evidence for this pass. Artifact generation time: 2026-11-09T09:53:48.993037
