# Low-speed steering research — 2026-11-09

## William: the useful result

**Recommendation: road-test the already installed angle mode on gentle streets before replacing it. Do not install these optimized gains yet.** I built a separate PI/PD angle controller with rate-limited reference, smooth friction compensation, model-preview feedforward, anti-windup and an authority flag; eight tests pass. But neither optimized candidate consistently beats the installed angle law on the held-out drives. The low-speed model is useful for rejecting designs, not for claiming the van can make intersections autonomously.

The strongest finding is a coverage limit: strict low-driver-torque training data contain no continuous three-second windows below 10 mph. Relaxing to two seconds and a higher torque-sensor cutoff gives only **two** wholly-below-10-mph training windows. Route 1a3 contributes none. Many apparent low-speed turning samples include substantial torque-sensor readings even when `steeringPressed` is false. Treating those as clean autonomous steering would overstate the EPS's capability.

The installed mode has not been driven. The optimizer's failure is not evidence that it is bad; it means we cannot honestly replace guessed gains with "validated" gains from these logs. Deliverables are complete research artifacts; the goal of a validated walking-speed/tight-turn plant and a demonstrated improved controller remains unmet.

## Scope and reproducibility

Repository `~/Code/odyssey-pilot`, branch `odyssey-sim`; all new code is in `tools/odyssey_sim/lowspeed/`. Installed controller and runtime hooks were not edited. No device access, push or commit. Source map and commands are in that directory's README. Root `ARCHITECTURE.md` and root agent files are absent; ODYSSEY.md is the runtime map. No unrelated onboarding files were added outside the authorized directory.

Immutable inputs: existing `plant_data/*.npz` CAN caches, `calib/*_body.npz` calibrated vehicle parameters, validated `nnlc_1a7/1aa/1ab.npz` controller replay caches, and `plant_public.npz`. Existing caches were reused, not overwritten. The old scratch `lowspeed.npz` was not used: its extractor has the wrong desired-angle sign and listens for `liveParameters` rather than this van's `vehicleParameters`.

New model horizon caches were extracted from original rlogs for low-speed-containing segments: 14,399 / 21,599 / 16,290 model frames on 1a7 / 1aa / 1ab. Preview uses the latest available current model frame, never the future logged target. Model yaw-rate / forward velocity → kinematic curvature → steering angle, with the calibrated steer ratio and the van's negative curvature-to-angle sign. Cache frames older than 0.15 s fall back to the current target. The preview horizon is fixed at 0.4 s, not optimized from nonexistent future information.

All angles below are **steering-wheel degrees**, rates degrees/s, effort normalized to maximum command. Raw CAN command is negated to match wheel-angle / actuators.torque sign. Angle offset and steer ratio use `vehicleParameters` from calibrated caches, aligned with a past-only timestamp lookup. Desired current angle comes from replay's `carControl.actuators.curvature`, not the controller's internally modified acceleration error. Kinematic conversion omits VehicleModel tyre/roll corrections; production integration must use the real VehicleModel.

## Plant and validation

EPS CAN is downsampled by timestamp to 10 Hz. Two-second windows advance one second (overlap is disclosed; counts are not independent experiments). No window crosses an rlog segment or a >0.16 s gap. The autonomous fit uses 1a3 + 1a7, never 1aa/1ab. Eligible samples require 0.5–8 m/s, lateral active, no steeringPressed, and absolute torque sensor <1000 raw units. That last cutoff is a deliberately broad, potentially confounded filter. The first generation used <500 and three seconds, then failed the low-speed coverage check.

The plant directly predicts angle and angular rate:

```
net = delayed_command - centering_load * angle
rate_target = gain(speed) * sign(net) * max(abs(net) - dead_zone, 0)
rate += (rate_target - rate) * (1 - exp(-dt / rate_tau))
angle += rate * dt
```

Gain interpolates at 1/4/8 m/s. Fit is robust output-error least squares with three starts and bounded parameters. State is initialized once per window, never reset to measured angle within the prediction. The dead zone is aggregate motor/friction resistance: this dataset does not separately identify motor deadband, static breakaway and sliding friction. There is no claimed new measurement of the brief's 2.8% motor deadband. Parameters hitting bounds, especially centering load and time constant, signal identifiability trouble. Learned zero transport delay is not evidence that this van has zero actuator delay.

Broad eligible coverage and prediction errors:

| Route | Eligible s | 2 s windows | Angle RMSE ° | Constant-rate baseline ° |
|---|---:|---:|---:|---:|
| 1a3 | 0.0 | 0 | nan | nan |
| 1a7 | 71.7 | 19 | 1.57 | 6.69 |
| 1aa | 60.2 | 12 | 3.74 | 5.54 |
| 1ab | 101.7 | 41 | 4.25 | 7.53 |

Parameters: gain1=2.224, gain4=16.268, gain8=3.7769, friction=0.029831, hold=0.03, tau=0.050001, delay=7.0642e-12.

The overall 3.74° / 4.25° held-out errors beat persistent-rate predictions, but the speed breakdown defeats a broad validity claim: neither wholly-0.5–2 m/s nor wholly-2–4.5 m/s training windows exist. The two low-speed training windows cross the 2 m/s boundary. On 1ab the two wholly-walking-speed test windows have 1.51° model RMSE versus **0.76° for constant rate**. Most available training windows are 4.5–8 m/s. No demonstrated low-speed plant validity follows from overall fit error.

Public Odyssey cache: deterministic 1-in-10 sample of CSVs, not all 250 hours. The split is by segment ID (`id % 20 == 0` train, remainder test); it is not random adjacent frames. Angle sign versus `steerFiltered` was chosen by training fit only; the winning sign is positive. The public cache does not supply a driver torque sensor. Derivative rate is obtained from angle and timestamps, so it is less precise than the van's EPS rate signal. Unknown owner/route identity means adjacent public segments may still correlate across this split; route-level public holdout is unavailable.

Public fit: 222 train / 77 test windows. Test angle RMSE 1.50° versus persistent-rate 2.98°. William-model transfer RMSE 1.39°. These results support an aggregate small-angle plant family, not a tight-turn authority certificate.

## Manual drives: human assistance is a separate operating mode

At 0.5–8 m/s, 1a4 provides 208.8 s and 1a5 116.2 s. Their 90th-percentile absolute wheel rates are 73.1°/s and 96.3°/s. Median motor magnitude in driver-sensor bins 0–200 / 200–500 / 500–1000 / 1000–3000 is 9 / 7 / 8 / 61 on 1a4 and 2 / 7 / 9 / 71 on 1a5. Large human effort recruits a very different response than the autonomous command path. Motor magnitude is unsigned and raw; it is not calibrated motor torque in Nm. Its sign cannot be reconstructed reliably during releases/reversals simply from the driver sensor.

A separate attempted combined human+assist angle model replaces autonomous command with negative driver sensor /1000. Train on 1a4, hold out 1a5: 38.0° / 50.0° angle RMSE, against constant-rate 47.3° / 58.3°. Friction, centering, time constant and delay reach bounds. **Reject this as a usable manual plant.** It omits nonlinear human-assist engagement, and unsigned motor measurements cannot uniquely recover the applied rack effort. Do not use these fast human wheel rates as EPS autonomous rate limits.

Release-like samples with sensor <200 and wheel angle >30° show median motion toward center of 57.2°/s (1a4) and 48.6°/s (1a5), but only 13 and 15 samples qualify. This suggests strong self-centering can exist after a driver-assisted turn. It does not establish that letting go of commanded torque is safe, repeatable or faster for autonomous exits; road grade, prior input and actual hand release are uncontrolled.

Observed full-effort autonomous rate is also highly variable: at 0.5–2 m/s median motion in command direction is 1.3 / 4.2 / 1.6°/s on 1a7 / 1aa / 1ab; at 2–4.5 m/s it is 5.2 / 9.05 / 6.35°/s. The lower quantile is often zero or negative. These are conditional observed responses, not guaranteed achievable rates: driver-sensor confounding, load and transient history remain. No defensible universal EPS angle-rate ceiling was identified.

## Stock and installed angle controller replay

Existing harness executes actual stock V0 and NNLC V0 against recorded motion; the shared brief reports near-perfect logged-command agreement. I reused these immutable outputs. In 0.5–6.7 m/s active/no-steeringPressed frames, stock V0 saturation is 33.7 / 42.9 / 33.3%, versus logged NNLC 66.8 / 83.2 / 67.8% for 1a7 / 1aa / 1ab. This revises the shorthand claim that all observed 100% dwell is stock KP: NNLC materially worsens it on these drives. KP boost remains a mechanism, but stock and NNLC must be distinguished.

Below 4.5 m/s the installed angle arm fully overrides the controller, so its algebra can be replayed without approximating the KP-capped stock blend. Kinematic desired-angle approximation gives:

| Route | Frames | Stock saturation | NNLC saturation | Installed-angle saturation |
|---|---:|---:|---:|---:|
| 1a7 | 3001 | 45.0% | 80.2% | 16.4% |
| 1aa | 4545 | 42.2% | 90.3% | 19.8% |
| 1ab | 3719 | 35.1% | 76.4% | 9.8% |

This is command replay, not a prediction of wheel tracking. Above 4.5 m/s the exact installed mode also caps upstream KP at 8; uncapped stock replay blended after the fact would not reproduce it. This study does not claim that exact blended replay.

## Controller generations and selection

Public interface `AngleController.update(stock, speed, angle, rate, desired, active, driver, future)` returns bounded torque, reference-rate-limited status, and authority-limited status. Features: reference slew limit; PI with conditional integration and bounded integral; measured-rate damping; smooth tanh friction; desired-angle holding and desired-reference-rate feedforward; smoothstep speed blend; invalid-input fallback; reset on inactive/driver override. Preview is causal current-model information. Angle arm effort is capped at 0.95. Near-limit dwell >0.8 is scored too, since reducing a cap from 1.0 to .95 alone does not fix useless effort.

The narrow closed-loop experiment replays only desired angle, not logged wheel motion. It initializes plant angle/rate once per window; new torque changes the plant state thereafter. All tuning uses 1a7 only. 1aa and 1ab are held-out checks. Initial angle-arm output is zero because the stock command is unavailable in this isolated experiment. Experiments are restricted to wholly-below-4.5 m/s windows so no fake highway-controller blend enters the comparison. These two-second local wheel tests do not close the vehicle/path/lane loop, omit tyre/roll dynamics and cannot establish lane clearance or takeovers.

Generation log:

- G−1: strict <500 sensor, 3 s fit. Rejected: no continuous training windows below 4.5 m/s; low-speed gain was free.
- G0: installed angle law. Baseline: training tracking 8.89°, wheel acceleration proxy 8.86°/s², command slew RMS 0.897/s.
- G1: initial PI/PD with friction .22, reference 12°/s and slew .65/s. Training tracking 9.01°, acceleration 10.49°/s². Rejected: worse tracking and wheel roughness despite lower command slew.
- G2: differential evolution, 480 evaluations, objective tracking + .18 × wheel acceleration + .5 × command slew. Tracking 7.47° on training, but >.8 effort 73.8% of the time. Held-out 1aa tracking worsened; acceleration and slew worsened on both held-out routes. Rejected.
- G3: causal 0.4 s model preview, friction ≤.3, slew ≤.8/s, reference ≤15°/s, and explicit near-limit effort penalty. Differential evolution, 280 evaluations; tracking + .7 × acceleration + 2 × slew + 12 × high-effort fraction. Training tracking 8.83°, command slew .449/s, near-limit dwell zero. Mixed held-out result below; not selected for road.

The 0.4 s preview is fixed. The optimizer stops by its normal population-convergence criterion before the maximum iteration bound; evaluation counts above are actual. Objectives are engineering tradeoffs, not a perceptual validation. "Wheel acceleration" is a plant-output roughness proxy; it has not been calibrated to William's sensation or the parallel perception worker's metric.

Held-out comparison, G0 versus G3 (six overlapping 2 s windows per route, all gentle by requested angle ≤40°):

| Route | Tracking G0 → G3 ° | Wheel acceleration G0 → G3 °/s² | Command slew G0 → G3 /s |
|---|---:|---:|---:|
| 1aa | 8.97 → 10.11 | 11.32 → 10.36 | 1.165 → 0.627 |
| 1ab | 2.73 → 2.12 | 7.87 → 9.82 | 0.611 → 0.683 |

G3 loses tracking on 1aa and increases wheel acceleration on 1ab. Both observed differences are comparable to the model's own prediction error, and windows are few and overlapping. No statistically defensible winner. G3's 3.09°/s reference limit may look smooth simply because it refuses fast turns; it must not become a production rate cap.

G3 parameters (unselected research candidate):

```json
{
  "kp": 0.06245283792477635,
  "ki": 0.007303596336086688,
  "kd": 0.021216002610678016,
  "friction": 0.10134507069731219,
  "hold": 0.007902419488028514,
  "rate_ff": 0.07264709590575943,
  "max_rate": 3.0894556111346017,
  "slew": 0.7882379251048954,
  "preview": 0.4
}
```

G1/G2/G3 parameters and metrics are also in `candidate_gains.json`; precise raw metrics in scratch `lowspeed/results.json`. Class defaults retain the initial research gains, are disabled by default, and are not claimed to be tuned. There is deliberately no runtime auto-loading of an unselected fit.

## Intersections and tight turns

Requested-angle >90° has **zero eligible held-out low-speed windows** after filtering. Tight turns exist in the logs, but substantial driver input and/or intervention removes them from a clean autonomous validation set. Do not merge them into gentle-curve scores or treat manual trajectories as autonomous capability.

A separate synthetic 2 m/s demand ramps to 120°, holds, then unwinds. On the William fit, G0 tracking RMSE is 64.5°, literal saturation 63.4%; G3 tracking is worse at 68.8°, despite lower effort/roughness. On the public fit, G0/G3 RMSE is 67.0°/69.6°. Half/double authority stress also fails tracking. This is extrapolation, not a measured intersection result. It does show that smoothing commands cannot create missing steering authority, and that saturation percentage alone rewards a controller that turns too slowly.

The candidate flags sustained inability (error >10°, measured rate <1°/s, effort >.8 for >1.5 s), bounds the integral, and keeps bounded support. It does not silently let go or disengage in an intersection. Reduced effort cannot be chosen solely from this plant: backing off could also increase understeer. Flag timing and human warning/handoff need integration with the existing alert framework and on-road validation.

## Tests and self-review

Eight tests pass under the required OpenpilotTestCase runner: disabled passthrough; malformed/nonfinite horizon fallback; continuous highway blending; reference-rate and torque bounds with driver override; stall indication; invalid input and preview behavior; plant command sign/dead zone; dissipation of unforced motion. The first disable test failed on missing implementation, then passed. Later tests reproduced a malformed-horizon IndexError and an approximately 1.57-torque jump near 8 m/s when final output slew lagged the blend. Both were fixed; slew now applies to the angle arm before blending. Ruff check passes.

Host benchmark, 10,000 measured updates with a 33-point future horizon: median 10.8 µs, p99 14.5 µs, max 105 µs. This is the Studio, not a 3X timing measurement; the <2 ms device budget remains unverified. Runtime uses numpy, dataclasses and math only; scipy is offline only. No root repo-wide `check` target was found/read; verification is scoped to the authorized directory and prescribed runner/lint.

Self-review rounds: (1) semantics/signs/chronology: fixed stale liveParameters assumption by using vehicleParameters and past-only lookups, kept raw CAN sign explicit; (2) identifiability and holdout: rejected unconstrained walking-speed gain, exposed overlaps, driver contamination, manual-model boundary hits and walking-speed baseline failure; (3) adversarial behavior: malformed future input and speed transition tests, near-limit-effort metric, no phantom future targets, synthetic tight-turn rejection. These are local self-reviews, no delegated reviewers.

## Integration and road A/B plan

The lead owns runtime hooks. `tools/odyssey_sim/lowspeed/README.md` gives the exact integration contract: separate default-off experimental parameter, one angle override only, real VehicleModel angle conversion, proper active/driver reset flags, causal/stale-checked model horizons, logging/alert integration, and coordinated upstream KP-cap/fallback behavior. Returning a passed stock torque cannot undo a KP cap already applied upstream. That interaction must be resolved before calling fallback validated.

1. **First rep: installed mode A versus existing mode-off B**, keeping delay, camera offset, NNLC selection and deadband compensation constant. Start on a familiar gentle, low-traffic loop with ample room, daylight and dry pavement. William remains responsible for steering and takes over before a capability limit, not after a warning. Score the planned route in blocks, not changing arms in a live turn. Approximately 15–20 minutes: repeat the same straights and gentle left/right curves at 5–10, 10–15, and 15–20 mph. Include a stop/start and crossing 18 mph to inspect blending. Record subjective jerk/effort immediately after each block.
2. **Log and compare** desired/current/future steering angles, measured angle and rate, command and actual CAN request, speed, active/override, driver sensor, motor magnitude, exact mode/parameters, reference, integral and limit flags. Score lateral path error/takeovers separately from wheel error; for motion roughness use wheel-rate changes and the perception worker's validated metric when available. Score 100% and >80% effort plus progress toward the requested angle. Compare matched stretches, both turn directions and the same speed bin. A lower command-slew number with worse tracking is a failure.
3. **Before candidate B testing**, obtain identifiable low-speed excitation. Adapt the existing maneuver/replay tooling to logged, bounded, driver-supervised trials at 1–2, 3–4 and 5–7 m/s on an appropriate closed course, with modest angles first. Include both torque signs, short holds, release/unwind and repeated attempts. Driver takeovers invalidate the autonomous window but remain labeled evidence of capability limits. Do not infer autonomous authority from human-assisted turns. The exact pulse limits/course protocol need the lead's review; no such experiment was installed or run here.
4. **Plant gate**: collect continuous, independently held-out low-speed windows with low driver effort and measured EPS rate. Require multi-second predictions to beat persistence in each speed/load bin, left and right, with enough independent repetitions to estimate uncertainty; a root aggregate fit score is insufficient. Fit motor deadband, static/sliding friction, self-centering and delay only when the excitation identifies them. Expand the current plant if release behaviour requires an angle/speed-dependent restoring term.
5. **Then candidate A/B**, only if that plant and replay support a particular candidate. Use the installed mode as A, candidate as B; default off outside the controlled run. Reject if either direction adds takeovers, worsens path tracking meaningfully, creates a blend/stop-start jump, or increases perceptible roughness. Do not decide from these six overlapping windows.
6. **Intersections/tight turns are a separate later gate.** First establish measured achievable angle/rate under autonomous effort on a closed course. If the requested turn exceeds it, provide an early clear driver-takeover indication using the existing alert mechanism; do not test unattended intersections and do not make a slow reference look like successful tracking. Automatic torque release is not justified by the sparse manual release samples.

## Exact next step for the lead

Review the first installed-angle road drive and score gentle low-speed tracking and wheel motion. Keep this controller disabled. If that drive is still jerky or weak, use the logged current/future angle and EPS rate to choose bounded closed-course identification maneuvers; collect clean continuous low-speed input/output pairs before optimizing another set of gains.

Artifacts: `lowspeed/results.json`, `plant.json`, `windows.npz`, `diagnostics.json`, `tuned.json` (rejected G2), `smooth_tuned.json` (unselected G3), `benchmark.json`, `plan_*.npz`; logs `lowspeed_run.log`, `lowspeed_plan.log`, `lowspeed_diagnostics.log`. No outside sources were used: claims derive from local primary controller code, rlog caches and the local public Odyssey dataset sample.
