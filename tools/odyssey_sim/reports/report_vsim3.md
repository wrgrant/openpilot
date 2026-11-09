# Video simulator pass 3

The simulator can reject a substantially weakened controller, but **it is not validated to rank stock NNLC, S3 and S5 on the road**. All 93 requested controller/window runs completed. Of 13 held-out windows, only seven have enough visible lane data; four of those seven miss the inherited spectrum target even though their total wander RMS is close. Two of six observable curve-width comparisons also miss the 25% target.

Halving feedforward worsens wander by **1.24 cm RMS [0.09, 2.67]**, while adding 0.3s delay has an inconclusive effect. Zero steering leaves the one-metre pose domain in 9.28s, so the plant is not frozen. Normal controller changes are smaller than the motion carried forward from the log. The natural regression under-recovers the directly injected planner response; independent 3cm/frame offset jitter alone does not explain that gap.

**The next decision is the road A/B, using the locked forecasts below.** They predict essentially no measurable wander improvement from S3/S5, with wide conditional uncertainty. No ranking or controller deployment is recommended from this simulator. No new road A/B results were opened, no device was accessed, and nothing was committed or pushed.

## Generation log

| Generation | Experiment and measured result |
|---|---|
| G0 | Read shared/pass-1/pass-2 briefs and reports; retained real ONNX, native controller, dual nonlinear plants and exact SE(2). |
| G1 | Added exhaustive maximal autonomous-window discovery and camera-readiness inventory. Missing arrivals remain pending. |
| G2 | Ran nine controller variants with identical paired-window boundaries and 15s warmup, retaining full failures. |
| G3 | Added near-lane RMS/spectrum/max offset, pose-departure diagnostics and matched logged-plan curve events. |
| G4 | Applied natural regression to free-running simulator output; compared implied transfer with existing injected transfer and true-pose/perception differential fits. |
| G4b | Reproduced and fixed curve-event selection to use native controlsd desired acceleration, as characterize.events requires; preregistration history retains the correction before held-out video arrival. |
| G4c | Found unsupported unmasked RMS on low-confidence held-out lanes. Reproduced visibility/outlier tests; corrected scoring to existing tool semantics, retained original registration and unmasked v1 shadow forecast, and registered v2 before reading any road A/B results. |
| G5 | Waited for held-out video with periodic local-only readiness checks; unchanged held-out controller/plant settings. |
| G6 | Prospective S3/S5 differences and bootstrap intervals computed across completed video-route pairs; numerical registration locks only when all requested routes are scored. |
| G7 | Stronger training stress: halved feedforward on every training window; zero-steering sanity run exits the one-metre domain. Both stay outside the registered forecast population. |
| G8 | Shared canonical lane scores in CLI and batch; 24 tests pass. Exact comparison preserves all 93 locked full-window numerical scores and repeated model/motion traces. |

## Coverage and evidence

Extra unseen route audit: all raw carControl messages on 19f/1a0/1a2 have latActive=false and enabled=false (123284/160942/67931 messages). These manual-only routes were inspected but contribute no eligible controller pairs. See extra_raw_activity.json and extra_driving.json.

Window eligibility: native CAN continuity (no gap >25ms), lateral active, no steeringPressed/blinker, |driver torque|<500, speed>5m/s and finite required context. Maximal stretches >=45s; round endpoints inward 30ms. Ready video is required throughout 15s recurrent warmup and scoring. This includes low speeds below the usual 25mph A/B threshold; see limitations before comparing to a road A/B.

`vsim3/inventory.json` records every eligible window, missing camera pairs and file sizes. `wait_status.json` records arrival state. `protocol.json` retains pre-held-out selection/measurement rules. `0000*.json`, `.traces.json`, `.motion.json` and `.log` preserve per-run evidence and failures.

| Route | Eligible windows | Ready | Completed stock |
|---|---:|---:|---:|
| 000001a3 | 3 | 3 | 3 |
| 000001a7 | 3 | 3 | 3 |
| 000001aa | 6 | 6 | 6 |
| 000001ab | 7 | 7 | 7 |
| 0000019f | 0 | 0 | 0 |
| 000001a0 | 0 | 0 | 0 |
| 000001a2 | 0 | 0 | 0 |

## 1. Controller discrimination

Canonical wander and spectra use the odyssey_wander visibility rule: both logged lane probabilities >0.5 for >80% of the window, with low-confidence holes interpolated. The fixed logged mask is shared by candidates. Unsupported wander is null; unmasked diagnostics remain in analysis.json. Maximum offset uses observed visible samples; it cannot bound hidden lane position. Curve widths also require visible mid-curve support. Candidate-specific visibility is stored for new traces, but the historical common-mask comparison does not pretend to reproduce every A/B scorer selection.

Training-context audit: five of the six native-eligible training stretches contain nonzero maneuver torque offsets (up to 0.08 on 1a3 and 0.18 on 1a7). Production controlsd can use lateralManeuverPlan curvature and adds its torqueOffset. The free-running simulator uses the neural plan and omits external characterization commands; its stock training run is therefore a normal-lane-control counterfactual over maneuver footage, not a reproduction of all logged interventions. All training windows remain included as requested, with this confound explicit. The eligible 1aa/1ab stretches have no nonzero probe offsets. See intervention_audit.json.

![Controller differences and pose departure](vsim3/discrimination.png)

Paired mean candidate-minus-stock differences on **training routes only**. 95% percentile bootstrap resamples windows (10,000 deterministic resamples). Correlated windows/routes limit generalization. A single window has no uncertainty interval. Full band powers and exact frequencies are retained in analysis.json.

| Controller | Paired windows | Wander change cm RMS [95% CI] | Max lane offset change m [95% CI] | Max pose change m [95% CI] | Spectrum L1 change [95% CI] |
|---|---:|---|---|---|---|
| v0 | 6 | -0.332 [-0.653, -0.015] | 0.022 [-0.002, 0.054] | 0.035 [0.018, 0.052] | 0.237 [0.099, 0.419] |
| kp_half | 6 | 0.143 [-0.362, 0.493] | 0.042 [-0.008, 0.118] | 0.042 [0.033, 0.052] | 0.246 [0.136, 0.379] |
| kp_double | 6 | -0.104 [-0.456, 0.196] | 0.032 [0.009, 0.060] | 0.036 [0.021, 0.054] | 0.176 [0.126, 0.226] |
| V1 | 6 | 0.053 [-0.249, 0.335] | -0.009 [-0.020, 0.004] | 0.030 [0.004, 0.059] | 0.320 [0.185, 0.467] |
| S3 | 6 | 0.064 [-0.237, 0.301] | -0.002 [-0.010, 0.006] | 0.031 [0.006, 0.056] | 0.319 [0.173, 0.478] |
| S5 | 6 | -0.097 [-0.487, 0.377] | -0.004 [-0.019, 0.010] | 0.036 [0.004, 0.069] | 0.339 [0.216, 0.461] |
| friction | 6 | 0.115 [-0.215, 0.450] | 0.009 [0.002, 0.016] | 0.011 [0.002, 0.021] | 0.201 [0.119, 0.288] |
| bad_delay | 6 | 0.203 [-0.094, 0.537] | 0.013 [-0.018, 0.052] | 0.047 [0.029, 0.059] | 0.241 [0.144, 0.369] |

Per-window simulated wander (cm RMS) and maximum relative pose (m):

| Window | Arm | Wander | Max offset m | Max pose m |
|---|---|---:|---:|---:|
| 000001a3_00 | S3 | 14.149 | 0.838 | 0.191 |
| 000001a3_00 | S5 | 14.109 | 0.815 | 0.184 |
| 000001a3_00 | V1 | 14.052 | 0.826 | 0.188 |
| 000001a3_00 | bad_delay | 14.184 | 0.888 | 0.209 |
| 000001a3_00 | friction | 14.167 | 0.840 | 0.196 |
| 000001a3_00 | kp_double | 14.211 | 0.898 | 0.219 |
| 000001a3_00 | kp_half | 14.481 | 0.855 | 0.242 |
| 000001a3_00 | stock | 14.052 | 0.845 | 0.204 |
| 000001a3_00 | v0 | 13.983 | 0.883 | 0.226 |
| 000001a3_01 | S3 | 15.104 | 0.754 | 0.115 |
| 000001a3_01 | S5 | 15.581 | 0.768 | 0.151 |
| 000001a3_01 | V1 | 15.207 | 0.758 | 0.114 |
| 000001a3_01 | bad_delay | 14.991 | 0.752 | 0.121 |
| 000001a3_01 | friction | 15.359 | 0.779 | 0.081 |
| 000001a3_01 | kp_double | 14.984 | 0.771 | 0.102 |
| 000001a3_01 | kp_half | 14.981 | 0.812 | 0.089 |
| 000001a3_01 | stock | 14.586 | 0.770 | 0.057 |
| 000001a3_01 | v0 | 14.333 | 0.763 | 0.117 |
| 000001a3_02 | S3 | 4.125 | 0.306 | 0.091 |
| 000001a3_02 | S5 | 3.466 | 0.320 | 0.098 |
| 000001a3_02 | V1 | 4.106 | 0.311 | 0.092 |
| 000001a3_02 | bad_delay | 4.766 | 0.257 | 0.131 |
| 000001a3_02 | friction | 4.319 | 0.318 | 0.091 |
| 000001a3_02 | kp_double | 4.058 | 0.332 | 0.159 |
| 000001a3_02 | kp_half | 4.271 | 0.269 | 0.146 |
| 000001a3_02 | stock | 3.843 | 0.293 | 0.084 |
| 000001a3_02 | v0 | 3.020 | 0.295 | 0.152 |
| 000001a7_00 | S3 | 3.701 | 0.619 | 0.112 |
| 000001a7_00 | S5 | 3.562 | 0.593 | 0.114 |
| 000001a7_00 | V1 | 3.713 | 0.590 | 0.111 |
| 000001a7_00 | bad_delay | 3.943 | 0.715 | 0.152 |
| 000001a7_00 | friction | 3.776 | 0.628 | 0.107 |
| 000001a7_00 | kp_double | 4.023 | 0.707 | 0.122 |
| 000001a7_00 | kp_half | 4.941 | 0.843 | 0.131 |
| 000001a7_00 | stock | 4.304 | 0.617 | 0.105 |
| 000001a7_00 | v0 | 4.568 | 0.711 | 0.117 |
| 000001a7_01 | S3 | 10.981 | 1.204 | 0.121 |
| 000001a7_01 | S5 | 10.571 | 1.216 | 0.103 |
| 000001a7_01 | V1 | 10.855 | 1.195 | 0.115 |
| 000001a7_01 | bad_delay | 11.145 | 1.206 | 0.135 |
| 000001a7_01 | friction | 10.775 | 1.222 | 0.091 |
| 000001a7_01 | kp_double | 10.193 | 1.215 | 0.102 |
| 000001a7_01 | kp_half | 9.971 | 1.206 | 0.121 |
| 000001a7_01 | stock | 10.972 | 1.213 | 0.074 |
| 000001a7_01 | v0 | 10.087 | 1.207 | 0.111 |
| 000001a7_02 | S3 | 7.469 | 1.024 | 0.196 |
| 000001a7_02 | S5 | 7.278 | 1.021 | 0.209 |
| 000001a7_02 | V1 | 7.530 | 1.027 | 0.205 |
| 000001a7_02 | bad_delay | 7.337 | 1.018 | 0.176 |
| 000001a7_02 | friction | 7.444 | 1.023 | 0.145 |
| 000001a7_02 | kp_double | 7.056 | 1.029 | 0.152 |
| 000001a7_02 | kp_half | 7.360 | 1.025 | 0.165 |
| 000001a7_02 | stock | 7.391 | 1.020 | 0.118 |
| 000001a7_02 | v0 | 7.166 | 1.028 | 0.131 |
| 000001aa_00 | S3 | — | 0.253 | 0.178 |
| 000001aa_00 | S5 | — | 0.331 | 0.247 |
| 000001aa_00 | stock | — | 0.159 | 0.197 |
| 000001aa_01 | S3 | 19.174 | 1.773 | 0.133 |
| 000001aa_01 | S5 | 19.492 | 1.769 | 0.128 |
| 000001aa_01 | stock | 19.296 | 1.769 | 0.069 |
| 000001aa_02 | S3 | 8.388 | 0.825 | 0.089 |
| 000001aa_02 | S5 | 8.503 | 0.820 | 0.086 |
| 000001aa_02 | stock | 8.369 | 0.874 | 0.040 |
| 000001aa_03 | S3 | — | 0.700 | 0.120 |
| 000001aa_03 | S5 | — | 0.687 | 0.140 |
| 000001aa_03 | stock | — | 0.683 | 0.122 |
| 000001aa_04 | S3 | 3.940 | 0.390 | 0.221 |
| 000001aa_04 | S5 | 3.946 | 0.384 | 0.246 |
| 000001aa_04 | stock | 4.187 | 0.290 | 0.085 |
| 000001aa_05 | S3 | — | 0.972 | 0.254 |
| 000001aa_05 | S5 | — | 0.991 | 0.277 |
| 000001aa_05 | stock | — | 0.713 | 0.100 |
| 000001ab_00 | S3 | 5.143 | 0.663 | 0.147 |
| 000001ab_00 | S5 | 4.938 | 0.648 | 0.161 |
| 000001ab_00 | stock | 6.296 | 0.716 | 0.110 |
| 000001ab_01 | S3 | 8.311 | 0.609 | 0.169 |
| 000001ab_01 | S5 | 9.314 | 0.608 | 0.196 |
| 000001ab_01 | stock | 5.438 | 0.507 | 0.077 |
| 000001ab_02 | S3 | 5.394 | 0.430 | 0.137 |
| 000001ab_02 | S5 | 5.655 | 0.443 | 0.147 |
| 000001ab_02 | stock | 4.091 | 0.383 | 0.056 |
| 000001ab_03 | S3 | 6.334 | 0.633 | 0.171 |
| 000001ab_03 | S5 | 6.476 | 0.611 | 0.188 |
| 000001ab_03 | stock | 8.082 | 0.620 | 0.089 |
| 000001ab_04 | S3 | — | 0.129 | 0.136 |
| 000001ab_04 | S5 | — | 0.119 | 0.135 |
| 000001ab_04 | stock | — | 0.159 | 0.178 |
| 000001ab_05 | S3 | — | 0.202 | 0.171 |
| 000001ab_05 | S5 | — | 0.201 | 0.175 |
| 000001ab_05 | stock | — | 0.203 | 0.168 |
| 000001ab_06 | S3 | — | 0.589 | 0.116 |
| 000001ab_06 | S5 | — | 0.525 | 0.118 |
| 000001ab_06 | stock | — | 0.584 | 0.059 |

Plant authority and metric carry-through (stock runs): command difference is against the native applied CAN command; body difference uses v×logged yaw as baseline. The pose-band/logged-offset ratio compares amplitudes, not independent variance contributions. A high perception/pose correlation confirms the metric sees motion, not that the plant response is physically validated.

| Window | Command delta RMS | Body delta RMS m/s² | Pose-band / logged wander | Perception delta vs pose r |
|---|---:|---:|---:|---:|
| 000001a3_00 | 0.126 | 0.066 | 0.289 | 0.989 |
| 000001a3_01 | 0.046 | 0.020 | 0.086 | 0.967 |
| 000001a3_02 | 0.061 | 0.026 | 0.137 | 0.975 |
| 000001a7_00 | 0.101 | 0.055 | 0.411 | 0.932 |
| 000001a7_01 | 0.071 | 0.039 | 0.202 | 0.995 |
| 000001a7_02 | 0.079 | 0.048 | 0.261 | 0.991 |
| 000001aa_00 | 0.125 | 0.023 | 0.551 | 0.975 |
| 000001aa_01 | 0.030 | 0.011 | 0.048 | 0.920 |
| 000001aa_02 | 0.027 | 0.011 | 0.093 | 0.913 |
| 000001aa_03 | 0.143 | 0.035 | 0.145 | 0.957 |
| 000001aa_04 | 0.102 | 0.016 | 0.224 | 0.991 |
| 000001aa_05 | 0.081 | 0.020 | 0.173 | 0.940 |
| 000001ab_00 | 0.065 | 0.024 | 0.256 | 0.991 |
| 000001ab_01 | 0.044 | 0.017 | 0.291 | 0.929 |
| 000001ab_02 | 0.038 | 0.015 | 0.136 | 0.989 |
| 000001ab_03 | 0.062 | 0.026 | 0.165 | 0.882 |
| 000001ab_04 | 0.067 | 0.020 | 0.693 | 0.973 |
| 000001ab_05 | 0.074 | 0.025 | 0.388 | 0.921 |
| 000001ab_06 | 0.122 | 0.010 | 0.301 | 0.841 |

Failures are retained separately and exclude a complete ranking. No partial-window metric substitutes for a requested full window:

```json
{}
```

### Deliberately bad controllers and diagnosis

The original +0.3s delay changes wander by +0.203 [-0.094, +0.537] cm RMS over six paired windows; its interval includes zero. This result does not satisfy a claim of consistently worse wander by itself. The alternative required by the brief, **halved feedforward**, changes wander by +1.238 [+0.089, +2.674] cm RMS over all six windows. Its paired maximum-offset change is +0.028 [-0.004, +0.070] m and maximum-pose change is +0.071 [+0.033, +0.104] m.

The zero-command diagnostic exceeds 1m after 9.28s and stops as designed; it is not assigned a 45s wander score. These results distinguish limited sensitivity to modest changes from a disconnected plant or warp. The delayed controller can alter phase/power without reliably increasing total band RMS. Logged disturbances dominate much of the offset, and the nonlinear EPS model limits small command changes. The authority table and full spectra quantify this rather than treating a close stock RMS as predictive validation. The independent pulse-plant validation remains incomplete; no parameter was tuned to make these tests pass.

Supplemental inputs, isolated Params, complete/partial traces and failures are in `vsim3/stress/`; paired evidence is `vsim3/stress_analysis.json`. The single intentional zero-command domain failure is separate from the 93 complete primary runs.

## 2. Natural-fit bias check

54 complete training-route free runs with sufficient logged lane visibility enter the pooled observational regression. R²=0.478, standardized condition=2.992.

| Injected period s | Free-fit / measured injected gain | Phase difference ° |
|---:|---:|---:|
| 10.0 | 0.175 | -11.573 |
| 15.0 | 0.194 | -18.056 |
| 20.0 | 0.221 | -22.051 |
| 7.0 | 0.162 | -0.495 |

The free-output regression also under-recovers the known injected response. This supports the conclusion that the natural regression is unsuitable as a gain-validation gate. It does **not** isolate regressor noise: feedback confounding, correlated road geometry and omitted recurrence also attenuate/distort the fit. The synthetic known-linear clean/noisy calibration in bias.json verifies attenuation algebra, not the real model’s noise distribution. Differential fits use true relative pose and measured perception separately; these remain observational, not an identified dynamic transfer.

Known-response calibration at the actual free-simulation regressor scales adds independent 3cm/frame offset noise **before** the same bandpass. It recovers 0.9983 of the offset coefficient; band noise is 0.296cm RMS against 11.066cm signal. Thus white frame jitter alone cannot explain a threefold gap here. Temporally colored lane/heading/road errors remain unmeasured. See vsim3/noise_check.json for the 200-noise-draw coefficient interval and assumptions.

## 3. Held-out closure and curves

![Held-out wander and spectrum versus pose](vsim3/heldout_closure.png)

Curve width is a signed outside-directed lane offset: negative values mean inward offset. A more negative value is not automatically an improvement; judge lane tracking first. Release values remain null where the logged plan never crosses its 20% threshold within the characterization horizon.

No tuning uses 1aa/1ab or extra unseen data. Log-forwarded similarity is reported with maximum pose departure for each stock window. The inherited ±25% lane/spectrum target remains a necessary check; absence of curve coverage is not a pass.

| Window | Sim wander cm | Logged cm | Relative RMS error | Spectrum L1 relative error | Period sim/log s | Max pose m | Lane confidence % | Matched curves |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 000001aa_00 | — | — | — | — | — / — | 0.197 | 1.971 | 1 |
| 000001aa_01 | 19.296 | 20.069 | 0.038 | 0.075 | 16.717 / 16.717 | 0.069 | 93.519 | 0 |
| 000001aa_02 | 8.369 | 8.506 | 0.016 | 0.118 | 12.108 / 12.108 | 0.040 | 98.073 | 2 |
| 000001aa_03 | — | — | — | — | — / — | 0.122 | 70.164 | 0 |
| 000001aa_04 | 4.187 | 4.154 | 0.008 | 0.576 | 11.838 / 11.838 | 0.085 | 93.347 | 0 |
| 000001aa_05 | — | — | — | — | — / — | 0.100 | 78.563 | 1 |
| 000001ab_00 | 6.296 | 5.922 | 0.063 | 0.533 | 15.433 / 15.433 | 0.110 | 87.041 | 0 |
| 000001ab_01 | 5.438 | 5.415 | 0.004 | 0.404 | 14.975 / 14.975 | 0.077 | 83.890 | 1 |
| 000001ab_02 | 4.091 | 4.161 | 0.017 | 0.348 | 11.980 / 11.980 | 0.056 | 82.053 | 0 |
| 000001ab_03 | 8.082 | 8.306 | 0.027 | 0.130 | 18.300 / 18.300 | 0.089 | 90.301 | 2 |
| 000001ab_04 | — | — | — | — | — / — | 0.178 | 25.329 | 0 |
| 000001ab_05 | — | — | — | — | — / — | 0.168 | 16.861 | 0 |
| 000001ab_06 | — | — | — | — | — / — | 0.059 | 0.353 | 0 |

| Window/event | Sim run-wide m | Logged m | Sim body release minus plan s | Logged s |
|---|---:|---:|---:|---:|
| 000001aa_00/624.3 | — | — | 0.300 | 0.300 |
| 000001aa_02/890.1 | 0.219 | 0.218 | — | — |
| 000001aa_02/897.8 | 0.043 | 0.026 | — | — |
| 000001aa_05/1629.1 | -0.226 | -0.198 | 0.600 | 0.600 |
| 000001ab_01/229.5 | -0.278 | -0.259 | — | — |
| 000001ab_03/644.9 | -0.242 | -0.228 | 0.200 | 0.200 |
| 000001ab_03/696.7 | -0.063 | -0.041 | 0.300 | 0.300 |

### Held-out gate decision

The seven lane-observable stock windows all match total band RMS within **6.4%**, and their dominant periods match. Nevertheless, only **3/7** match RMS, period and spectrum together. The four failing spectrum L1 errors are **34.8–57.6%**, above the inherited 25% target. Six other windows cannot provide the required wander measurement and are reported as null rather than manufactured agreement.

There are 7 matched curve events, 6 observable signed widths, 2 width disagreements above 25%, and 4 finite body-release comparisons. All four finite release differences match at the characterization's 0.1s resolution. The two relative-width failures are centimetre-scale errors on small signed widths; percentage error is unstable near zero, but no post-hoc absolute floor is introduced to pass them. An absolute tolerance needs independently measured lane-geometry precision.

Stock pose departs from the recorded trajectory by **4.0–19.7cm maximum per window**. Small perturbations and the shared logged body motion explain why RMS/threshold timing can agree without proving controller-change prediction. Full held-out closure is **not passed**. Rankings and their ±25% plant robustness sweeps are therefore not licensed. Pass-2 sensitivity results are not reused as a ranking validation.

## 4. Prospective road predictions

These forecasts target the next road A/B without adjusting them after its results. Transport assumptions are explicit: historical video uses CameraOffset=0 while the common brief reports the van now uses +0.12; comparative model effects are assumed to transport across that change. Fully autonomous historical windows also omit takeover-heavy curves. The default road scorer applies 25mph eligibility and 30s block settling, while the simulator uses >5m/s eligibility and 15s recorded-state warmup. These differences remain part of the forecast test, not post-hoc reasons to excuse a wrong prediction.

Candidate minus stock. Wander is cm RMS, curve run-wide is metres, body lateral jerk is m/s³ (20Hz derivative RMS). The existing --ab scorer reports wander only; additional metrics require this protocol. The road protocol skips 30s after each 180s block switch, whereas these independent-window runs use 15s recorded-state warmup. That transport difference must be retained when scoring the forecast.

| Candidate | Wander cm RMS [95% CI] | Curve run-wide m [95% CI] | Body jerk m/s³ [95% CI] |
|---|---|---|---|
| S3 | 0.101 [-0.450, 0.710] | -0.033 [-0.072, 0.006] | 0.015 [0.011, 0.019] |
| S5 | 0.153 [-0.511, 0.931] | -0.042 [-0.087, 0.002] | 0.036 [0.026, 0.047] |

Original formula registration: **2026-11-09T16:24:43.114161+00:00**. Visibility-corrected formula registration: **2026-11-09T18:26:38.289296+00:00**. All-route numerical forecasts locked: **2026-11-09T18:54:18.673186+00:00**. These are distinct timestamps; the early formula timestamp is not presented as an early completed numerical forecast. No new-drive A/B results were read; local evidence cannot establish the absence of an untransferred drive.

Intervals describe resampling uncertainty conditional on these roads; they are not calibrated prediction intervals for a new road drive. No curve coverage yields null, never an invented zero prediction.

### Forecast support and preserved original registration

The primary visibility-corrected forecast uses **13 paired windows for wander, five for curve width, and 19 for body jerk**. All seven named video routes were inspected; the three manual-only routes supply no autonomous pairs. Positive wander/jerk changes are worse. Negative signed curve-width change means movement inward relative to the curve, which is not automatically better lane tracking.

The original unmasked rule is retained in `vsim3/predictions_v1_shadow.json`. Its frozen source hash and original two-window numerical snapshot reproduce exactly; the full historical data were then evaluated with that same rule. This shadow version is not substituted for the primary v2 after observing the road:

| Original unmasked v1 | Wander change cm RMS [95% CI] | Signed width change m [95% CI] |
|---|---|---|
| S3 | +0.067 [-0.341, +0.487] | -0.026 [-0.062, +0.006] |
| S5 | +0.154 [-0.317, +0.669] | -0.044 [-0.080, -0.007] |

The body-jerk predictions are identical in both versions. Version history records the visibility correction, not a controller/plant retune. Primary v2 is the road-scoring forecast; v1 remains an audit trail, not a choice to make after the drive.

The supplied `odyssey_wander --ab nnlc_model` currently prints **wander only**. To score the other two forecasts, extend its report with the already-defined `metrics.curve_scores` signed-width measurement and 20Hz RMS derivative of `v × carControl.angularVelocity[2]`, using the same monotonic arms, 30s block settling and autonomous selection. This worker does not edit that out-of-scope reporter. Definitions and units are fixed in `protocol.json` and the shared VSIM metrics, so that integration must not select a new definition after seeing road outcomes.

## 5. Rankings and next experiments

No validated ranking is issued: held-out spectrum and signed-width closure fail, with additional lane-visibility gaps. The ±25% plant sweep in pass 2 does not validate controller-specific rankings, and was not reused as if it did. Conditional next experiments: stronger plant identification from clean held-pulse data; independent road A/B scoring of the locked S3/S5 forecasts; then friction inverse and curve-entry/exit anticipation trials scored by lane position and body motion. No wheel-ripple rejection rule is introduced.

## Verification and self-review

- Local branch odyssey-sim; no branch switch, commit, push, device access or runtime-hook edit.
- Scope: tools/odyssey_sim/vsim/ and the assigned scratch tree.
- Self-review 1: matched eligibility, gaps, camera readiness and warmup; retained failed/partial runs; one-window bootstrap intervals suppressed.
- Self-review 2: event selection held fixed to logged plan, explicit units and sign convention, true-pose departure reported; no absolute-plant tuning.
- Self-review 3: production model SafeNNTorqueModel/response-shaper path, isolated scratch Params/processes, prospective source inventory and registration guard. No nested agents were used by this assigned research worker.
- Tests: geometry, recurrent model, perturbation invariants and campaign/bootstrap/known-signal checks. Final lint/test receipts and source hashes accompany this report.

Generated 2026-11-09T19:09:15.060696+00:00. Local code/logs are primary sources; no new external source was used. Prior real-model/reprojection sources remain in report_vsim.md.

## End-state checks and exact next step

| Check | Result and evidence |
|---|---|
| Every eligible training window, nine requested controllers | **Done:** 6 × 9 = 54 complete runs, paired bootstrap evidence in analysis.json. |
| Wait for video and test every eligible held-out window | **Done:** 13 × 3 = 39 complete stock/S3/S5 runs; 13 stock closure rows. Arrival cycles and completion are in watch.log/wait_status.json. |
| Inspect extra unseen routes | **Done:** raw logs confirm no enabled lateral control; zero eligible windows. |
| Natural-fit recovery/noise experiment | **Done:** 54 training free runs from six independent windows; injected-response comparison and known-response noise calibration retained. |
| Timestamped forecasts with intervals | **Done:** locked predictions.json plus preserved v1 shadow, source/model hashes, and explicit 13/5/19 support counts. Exact pre-drive timing is independently checkable from the published timestamps. |
| Code, CLI, README and tests | **Done:** 24 tests and Ruff pass. Both CLI model/motion replay checks match exactly; canonical scores match batch output; all 93 full-window numerical scores survive the shared-score refactor exactly. |
| Full closure and validated controller ranking | **Not achieved:** four observable spectrum failures, two signed-width failures and six unsupported wander windows. No ranking or robustness claim is made. |
| Scope/cleanup | **Done:** authored code only in vsim/, evidence only in scratch; owned wait/run processes exit. No comma access, commits or pushes. |

**Next step:** score the next road A/B against primary v2 without changing its numbers or definitions. If it fails, improve independently identified perturbation dynamics and lane observability on training data before optimizing controller parameters. Clean no-pulse/held-pulse training measurements address the existing plant-validation gap; further held-out fitting would invalidate this test. The simulated S3/S5 effect estimates above are the current best guesses, not a replacement for that road result.
