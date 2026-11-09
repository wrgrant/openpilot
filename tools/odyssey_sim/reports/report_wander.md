# Highway wander: findings and add-on handoff

William: the logs do not support treating all slow lane wander as a steering deadband problem. On the clearest highway stretches, body motion follows most of the requested slow acceleration. Road geometry and the planner's correction both matter. Smooth friction assistance is a useful candidate, but I have not proved a road fix or centimetre-level lane holding. I recommend testing that assistance first while logging the new lane observer in shadow mode. Keep the lane correction disabled until its inconsistent heading/road estimates improve.

All authored code is in `tools/odyssey_sim/wander/`; exact integration and commands are in its README. Seven runtime tests pass. Nothing was committed, pushed, deployed, or run on the comma. Checkout: `/Users/williamgrant/Code/odyssey-pilot`, branch `odyssey-sim`. Other workers' files were left alone.

## What was measured on real highway logs

`wander/diagnosis.json` and `addon_replay.npz` contain uninterrupted, driver-free, blinkers-off, lane-confident, >=22 m/s windows from the calibrated caches. Segment boundaries are allowed only when timestamps remain continuous; an initial attempt incorrectly required a 60 s scoring window to fit strictly inside one nominal 60 s log segment and found no windows. That was fixed rather than relaxing sensor eligibility. Shorter 10/30 s windows are retained for replay, explicitly marked unsuitable for a 7–20 s wander estimate.

For six long windows on 18c, median 7–20 s lane-offset RMS is 8.41 cm; for fifteen on held-out 19c it is 9.12 cm using the legacy geometry/filter. The fresh origin-geometry worker reports 6.71/5.74 cm respectively with its own filtering/scored interior. These are different geometry/window definitions, not a measured improvement.

Body acceleration is `vEgo * carControl.angularVelocity[2]`. On these long windows:

| Median diagnostic | 18c | 19c |
|---|---:|---:|
| body/request band amplitude ratio | 0.982 | 0.890 |
| body/request correlation | 0.984 | 0.961 |
| fine-angle/body band correlation | 0.977 | 0.987 |
| road/request band correlation | 0.894 | 0.942 |
| legacy geometric closure correlation | 0.181 | 0.345 |
| geometric acceleration / offset-second-derivative RMS | 1.86 | 1.99 |

The old “follow ratio 0.25” is not reproduced by this amplitude-ratio definition and sample selection. An amplitude ratio includes road-following and is NOT a causal tracking transfer function. After subtracting geometric road acceleration from both signals, per-window ratios range 0.70–1.34 on 18c and 0.65–1.50 on 19c; residual correlations range 0.86–0.97 and 0.50–0.88. Tracking lag/error remains measurable, but blanket 75% attenuation is not supported. Fine wheel angle agrees with body motion, so this is not simply the coarse 0.1° angle hiding all response.

Road, planner and camera are inside a closed feedback loop. The observations do not identify which one initiates the oscillation. Road/request correlation is strong, while independent geometric closure is poor; a camera-generated road-curvature fluctuation can masquerade as real road forcing. Roll/tracking-error correlation varies markedly (median -0.42 on 18c, -0.03 on 19c), so a universal crown explanation also fails. No claim that crown is absent follows from that.

A fresh train-only fit using origin geometry on 18c/19b/1a7, excluding nonzero maneuver injection, gives:

`request - road = 0.280*offset + 0.652*(speed*heading) + 0.006`

R² is 0.44/0.55/0.62 on those training routes and 0.47/0.64/0.61 on 19c/1aa/1ab. This is descriptive, not a planner model suitable for closed-loop ranking. It does not validate the older R² 0.98 assumption. Exact coefficients and sample counts: `wander/native_planner_fit.json`. Fresh geometry is calib3's train-selected quadratic x=0 fit through 25 m. The replay retains all eligible highway stretches rather than claiming suburban portions are highway; 19b and 1ab have little continuous long-window evidence.

## Friction research and sizing

The newer calibrated plant has speed-dependent steering gains 1.113/2.083/2.194 and friction in acceleration units 0.170/0.186/0.071 at 10/20/30 m/s. A first inverse estimate is `motor_deadband + friction/gain`: 0.181/0.117/0.061 command. This differs from the older 0.14–0.29 command-equivalent estimate. Friction, gain, sensor bias and roll are confounded in ordinary closed-loop drives; neither fit uniquely identifies static breakaway torque.

Implemented candidates:

- Smooth inverse assistance: error-conditioned tanh, calibrated speed schedule, 0.22 absolute addition cap, 1.5 command/s addition slew. It adds only the estimated missing friction and does not invert the entire plant. Existing PID/NNLC friction can overlap; double compensation is a real limitation.
- Release pulses: error >0.025 m/s² plus wheel-rate magnitude <0.2°/s for 250 ms; 150 ms pulse, 1.5 s cooldown, abort on error reversal. These are finite test pulses, not a claim of optimal PWM.
- Dither: 5 Hz sine, gated by error and slew-limited. Its large requested amplitudes are clipped by shaping and Honda's rate limit; at 5 Hz a 1.5/s triangular limit permits only about 0.075 amplitude. Thus old 0.14–0.29 static friction cannot be overcome by simply turning this dither knob up. Raising frequency reduces available amplitude further. The first-order fit cannot represent EPS resonances or tactile motor ripple.

Literature supports trying these families, not transplanting their gains onto a van:

- [Canudas de Wit et al., 1995, LuGre](https://lup.lub.lu.se/search/ws/files/6363840/8498924.pdf): internal bristle deflection models presliding, Stribeck effects and hysteresis; friction observers can compensate without simply raising feedback gain. A real LuGre implementation needs identified bristle stiffness/damping, breakaway/static and sliding friction versus velocity, and reference-motion/load information. These logs do not identify them; I did not invent them and call the result fitted LuGre.
- [Wilhelm et al., power-steering friction compensation](https://infoscience.epfl.ch/entities/publication/a1243bfd-08c5-4f30-b4d3-d1710cfb4332): reduced steering model, internal friction estimation and an external reference-model tracking loop, experimentally evaluated on a power column. This motivates separating lane reference estimation from actuator compensation.
- [Pervozvanski and Canudas-de-Wit, 2002](https://www.sciencedirect.com/science/article/abs/pii/S0005109801001662): dither effectiveness depends on amplitude AND frequency under dynamic friction; its reported stick-slip suppression does not establish imperceptibility on this EPS.
- [Karnopp, 1985](https://doi.org/10.1115/1.3140698): stick/slip modeling around zero velocity motivates the stationary-wheel gate; the delivered pulse logic is not a full Karnopp plant model.
- [Adaptive impulsive control experiments](https://code.eng.buffalo.edu/tdf/papers/CCA_06_Fric.pdf) and [pulse-amplitude controller design](https://code.eng.buffalo.edu/tdf/papers/pulse_friction.pdf): discrete releases can reduce stiction positioning errors, but pulse amplitude/width must follow measured displacement and the mechanical plant. No extrapolation of their lab precision to highway lane accuracy is justified.

Best next identification experiment: paired positive/negative command plateaus around the existing holding torque, about 0.04/0.07/0.10/0.14 addition at highway speed, capturing fine wheel motion, signed command, motor magnitude and body yaw. Existing 0.03/0.06 probes may never cross breakaway. This requires a separately reviewed maneuver design; it was not run. Static holding torque is not the same as static friction.

## Lane observer and generation log

The runtime observer predicts `e_dot = v*h`, `h_dot = v*kroad - yaw + bias/v`, anchors offset/heading only on new camera frames, and estimates a bounded acceleration bias from camera innovations. It avoids differentiating raw 3 cm frame noise for the control derivative. It neglects sideslip, yaw bias and camera calibration dynamics; the bias state cannot distinguish those from road error.

The additive acceleration correction is `wn²*(e + 0.7*e_dot) + 1.8*wn*e_dot`, capped at 0.12 m/s² and slewed at 0.08 m/s³. Natural frequency decreases 0.55/0.50/0.42/0.38 rad/s at 18/25/35/40 m/s. Curvature addition is acceleration divided by speed squared. These gains are conservative hypotheses, not an optimum established by the logs.

- G0: reproduced documentation/class/sign/cache constraints. Found the one-segment window-selection bug and fixed it. The first test run also exposed a wrong test-helper import; corrected to `openpilot.common.test.OpenpilotTestCase`. A >0.20 command expectation based on older friction failed against v2's ~0.06 highway compensation; corrected the expectation, not the compensation gain.
- G1: offset/yaw observer with weak camera heading correction and no acceleration-bias state. Legacy-geometry replay was poor, especially 1ab (only about 15% median-window inward direction on small outward drifts). Rejected that configuration; saved runtime snapshot in `wander/runtime_g1.py`. Initial synthetic screen made combined assistance look strong but could not justify the road hypothesis.
- G2: added bounded bias state and stronger offset innovations to prevent road/yaw mismatch integrating indefinitely. Legacy replay still weak: sample-weighted inward fractions about 63/40/69/63/58/40% on 18c/19b/19c/1a7/1aa/1ab. Rejected legacy geometry as the observer's road input. `diagnosis_g2.json` preserves the result.
- G3: fresh 20 Hz origin geometry and camera heading, train-selected quadratic horizon rather than legacy curvature. See `native_replay.json` and per-route NPZ traces. Inward command during small outward drifts improves substantially on the main highways, but not uniformly; retained as a shadow candidate, not a claimed solution.

| Route | Eligible minutes | Small-drift samples | Inward correction, with heading | Without heading |
|---|---:|---:|---:|---:|
| 18c | 20.03 | 1988 | 87.8% | 74.7% |
| 19b | 2.72 | 73 | 57.5% | 61.6% |
| 19c held out | 26.97 | 2774 | 86.2% | 74.0% |
| 1a7 | 3.88 | 509 | 86.2% | 67.2% |
| 1aa held out | 2.96 | 243 | 88.5% | 88.1% |
| 1ab held out | 5.58 | 214 | 65.9% | 65.4% |

“Early drift” uses causal backward-smoothed camera offset magnitude 2–8 cm and growing magnitude. It is not independent ground truth, samples are serially correlated, and sign correctness does not prove sufficient torque, net acceleration, or reduced future deviation. Eligible minutes precede the runtime's tighter curvature/offset gates. Nonzero maneuver-injection samples are excluded; full maneuver-mode state is absent from this cache, so hold intervals can still enter shadow eligibility. With heading the correction reaches its 0.12 cap about 0.09–11% of eligible samples depending on route, another reason not to claim few-cm precision. Shadow replay confirms that the add-on can request corrections before larger drifts on many stretches; it does not prove the EPS would execute them.

Calib3's current independent closure audit still has errors comparable to the signal: held-out 19c acceleration closure RMS 0.0248 m/s² against lane second-derivative RMS 0.0246; velocity closure RMS 0.0207 m/s against lane-velocity RMS 0.0326. Its selected observer's training dropout error is ~0.206 m, already far above a few cm. These fail the intended precision requirement, so I did not treat the lane loop as closed merely because an observer follows its camera input.

## Evaluation results and what they authorize

Ran calib's logged-plan evaluator with the real running V0/NNLC controller plus each friction add-on. This preserves the active controller class, avoiding the previous wrong-class road A/B failure. Its adapter calls the add-on in acceleration-positive command space and retains final Honda slew in the fitted plant.

On 17 >=50 mph scoring rows across held-out routes:

| Candidate | Median simulated tracking cost | Median command reversals/s |
|---|---:|---:|
| stock | 0.246 | 1.20 |
| smooth inverse assist | 0.119 | 1.60 |
| pulse release | 0.235 | 1.11 |
| 5 Hz dither | 0.227 | 2.02 |

Tracking cost is calib's `100*mean((desired-measured)^2)`, using its mask and warmup. These are medians of row scores, not an asserted 51% road improvement. The baseline and every candidate have `ranking_authorized=false`. Its pass rule also requires measured wander even in logged-plan mode, so a false gate is partly structural; independently, the baseline controller tracking/reversal errors remain material. Changing that rule would not repair lane geometry closure.

`logged_plan.py` separately runs the same real-controller family with 100 Hz plant outputs through the new perception metric: 16 highway windows per arm. Every arm's simulated roughness score is zero under the current thresholds. This means the screen cannot distinguish these arms here; it does NOT prove imperceptibility. No hand-torque channel is simulated. The fitted plant was identified at 10 Hz and is first-order: 5 Hz ripple and EPS resonances are outside reliable identification bandwidth.

Fallback lane stress suite: 126 runs, seven arms × three speeds (22/30/35 m/s) × three plant perturbations (nominal, +0.3 s delay, double friction) × two inherited planner gain pairs (0.17/0.57 and 0.34/0.91). Deterministic seed 1; 120 s each with 20 s warmup; camera noise 3 cm at 20 Hz; sinusoidal road demand and roll disturbance. Outputs are in `wander/simulations.json`.

| Arm | Median 7–20 s offset RMS cm | Median total offset RMS cm |
|---|---:|---:|
| stock surrogate | 4.36 | 12.70 |
| old 0.03 kick | 3.95 | 8.04 |
| inverse assist | 2.89 | 11.92 |
| pulse | 4.10 | 12.18 |
| dither | 3.77 | 12.84 |
| observer lane correction | 1.51 | 5.16 |
| lane + inverse | 0.90 | 3.62 |

All have zero simulated perception excess and saturation. The lane simulator uses a simplified PID, exact simulated road curvature, and hypothesized planner feedback. It is a sensitivity/rejection screen, NOT a calibrated closed-loop reproduction. Exact simulated road geometry is an optimistic advantage absent from the actual camera. A median 0.90 cm wander-band number alongside 3.62 cm total offset is not “the van holds within a centimetre.” The old kick's larger simulated gain than earlier published ~0.5% is another warning that these surrogate rankings do not transfer reliably.

## Road A/B plan

Recommended next experiment: **stock versus smooth inverse assistance**, lane loop shadow-only, same model/NNLC/camera-offset/delay settings throughout. Arm A has every wander addition off; arm B uses inverse scale 1, no old deadband kick. Verify the intended class and nonzero pre/post torque difference before scoring any result. No change to Honda safety or stock command/slew limits.

Use balanced randomized pairs of 180 s blocks, alternate pair order AB/BA, exclude the first 30 s after each arm change, and reset controller/observer memory consistently. Score the final 150 s only when both lanes are confident, speed >=22 m/s, no blinkers/driver input/maneuver injection, and the block includes an uninterrupted >=60 s eligible interval. Do not silently count failed eligibility as a good block. Collect at least **10 eligible blocks per arm: 25 scored minutes/arm, 50 scored minutes total, at least 60 minutes driving** plus exclusions. Include both travel directions and repeat another day if wind/traffic differ substantially. Pair by road stretch and speed, not by unrelated route-wide medians.

Primary endpoint: per-block 7–20 s lane-origin RMS, with unfiltered offset RMS, mean bias, 95th-percentile absolute offset, maximum excursion and lane-confidence coverage reported alongside. Target at least 20% lower paired median wander with bootstrap CI by whole block/pair excluding zero; no increased 95th-percentile deviation, takeover rate or saturation. Insufficient pairs/minutes or a wide CI means inconclusive, not pass. Few-cm holding additionally requires total offset RMS <=3 cm and 95th-percentile absolute deviation <=6 cm, measured against a stable lane reference; the camera's present noise floor may make that endpoint unresolvable.

Secondary: desired/body tracking error and fitted phase in 0.05–0.143 Hz; fine-angle movement after small correction requests; body/wheel perception screen; native EPS rate and torque spectra through available bandwidth; driver hand-torque residual with a validated unit conversion; driver-reported wheel buzz. Commands alone are not a comfort metric. Log excluded driving and interventions as well as eligible blocks so apparent gains cannot come from fewer difficult samples.

Abort the experimental arm for new wheel buzz, a growing oscillation, steering alert, an intervention attributable to the arm, or visibly worse lane holding. Keep the driver supervising and use an already approved driving workflow. No autonomous command probes were executed here.

Then, only if heading/offset consistency and shadow early-correction results improve on 19b/1ab, compare the winning friction arm against the same arm plus lane correction with another 25 scored min/arm under the same block design. This isolates the lane loop's effect. A four-arm factorial (stock / friction / lane / combined) can follow, but needs at least 25 scored min/arm rather than subdividing the first hour four ways. Pulse/dither should wait for higher-bandwidth actuator/hand-torque characterization because the current comfort screen cannot distinguish them.

## Verification and exact remaining work

Seven public-API tests cover stale/missing camera fallback, nonfinite input, default off, speed range, takeover/reset, no false correction on a consistent curve, reduction of 3 cm independent camera jitter in a stationary synthetic case, bounded lane correction and pulse duty. `tools/test_runner.py` passes; ruff passes. The initial older-friction assertion failed and was revised to the new identified scale, recorded above. Tests do not prove on-road stability.

Host runtime benchmark, 20,000 paired observer/assist updates: mean 3.77 µs, p99 4.38 µs. This is a Mac host result, NOT a 3X measurement; measure 3X p99 before enabling. Modules contain no I/O, no SciPy and no openpilot imports. Their only external dependency is NumPy. No repository-wide architecture map existed in the examined root; the own-directory README provides the add-on map without editing shared documentation.

Lead's exact next steps: review `tools/odyssey_sim/wander/README.md`, wire default-off inverse assistance and observer shadow logging into the actual V0/NNLC/OdysseySteer path, confirm command/body signs and body-versus-VM error convention from a replay, run the listed tests, and stage the two-arm road experiment for William's approval. Do not enable the lane actuator correction based on this report's optimistic simulation scores. The missing evidence is independent lane-motion closure and a real response to assistance, not another assumed planner refit.

Artifacts: `scratchpad/wander/{diagnosis.json,diagnosis_g2.json,native_replay.json,native_planner_fit.json,simulations.json,logged_plan_perception.json,runtime_benchmark.json}`, per-route replay NPZs, and `scratchpad/calib/wander_{stock,inverse,pulse,dither}.json`. Source entry points reproduce the important tables; `python -m tools.odyssey_sim.wander.planner_fit` reproduces the descriptive regression. Artifacts under scratch use the shared brief's absolute scratch directory.
