CONTRACT — schema v1, available now

`plant_model.json` in this scratch directory is the authoritative fitted configuration. `tools/odyssey_sim/plant/plant.py` exports `load_params(path=DEFAULT_PATH)`, `new_state(params=None, **initial)`, and `step(state, torque_cmd, v_ego, dt) -> NEW state dict`. Positive normalized `torque_cmd` is CAN STEERING_CONTROL STEER_TORQUE / 4096: positive openpilot curvature/lateral acceleration, negative Honda wheel angle. Public `steerFiltered` must be NEGATED. Commands clip to ±1. Speeds are m/s, dt seconds. Default valid domain is moving driving, approximately 5–35 m/s, not parking.

JSON top-level `speed_mps` anchors `gain` (m/s² per normalized effective command), `tau_s` (seconds), and `friction_accel` (m/s² hysteresis threshold). `deadband` is normalized effective actuator deadband; `motor_deadband`, `motor_gain`, `motor_baseline` describe magnitude-only CAN telemetry. `motor_tau_s` and `vehicle_tau_s` are separate nominal lag priors. `roll_gain` multiplies roll radians; `bias_accel` is constant m/s². `steer_ratio`, `wheelbase_m`, `understeer_coefficient` map wheel response into degrees. `fit_routes`, `heldout_routes`, `parameter_evidence`, `status` preserve limitations. `public_model` contains an alternative parameter overlay, not a second active plant; copy the top-level dictionary then update from this overlay to use it. `simulator` holds the lane-loop/controller settings used below.

State: `steering_accel` is the wheel-derived acceleration proxy; `lat_accel` is the nominal lagged physical output, NOT independently validated body acceleration. `motor_cmd` is signed effective normalized motor torque; `motor_torque` is unsigned CAN units. `angle_deg` and `angle_rate_deg_s` use Honda sign. Optional `roll`, `bias_accel`, and `driver_torque` allow external disturbances; driver_torque is an equivalent normalized input, NOT raw driver sensor units. `_params` is carried per state. Initializing nonzero acceleration should initialize steering_accel, lat_accel, motor_cmd and angle_deg consistently. Defaults are zero. step does not mutate its input. No measurement noise or road/planner/controller feedback is hidden inside step.

Example:
```python
from tools.odyssey_sim.plant.plant import load_params, new_state, step
params = load_params()
s = new_state(params=params)
s = step(s, torque_cmd=0.2, v_ego=29.0, dt=0.1)
a = s['steering_accel']  # preferred comparison to latAccelSteeringAngle
```

# Fitted steering plant and reality check

## Decision for William

The van has a measurable motor threshold around 3% command, but most lane-keeping hysteresis occurs beyond that simple motor threshold. A speed-dependent, sticky first-order plant predicts short steering response reasonably. It does **not** make the lane simulator a faithful reproduction of the van: on held-out suburban route 1aa, closed-loop simulated wander is over three times observed wander. Use this model to reject obviously fragile controllers and explore uncertainty; do not select a road controller by its simulator ranking alone.

No device access, production changes, commits, branch switches, or changes to existing tracked files were made.

## Data and split

Every available rlog segment in all ten requested route directories was decoded: 259.2 minutes, including manual routes 1a4/1a5. CANParser used the requested Odyssey DBC and file-path LogReader. Angle was reconstructed with fine_angle(). Extraction retains gaps and segment resets. Fit uses 10 Hz timestamp-selected rows, whole 10-second windows with five seconds of prior uninterrupted engagement, speed >5 m/s, no steeringPressed, |raw driver torque|<500, |command|<0.95. Windows never bridge timestamp gaps. Consequently not every logged minute is eligible for autonomous identification; manual data enters the EPS-assistance analysis instead.

Training routes: 174, 18c, 19b, 1a3, 1a7. Entire routes 19c, 1aa, 1ab are held out for acceleration validation. 1aa/1ab A/B torque compensation is already inside logged sendcan, with no additional correction. No held-out lane-wander matching was used to retune the plant.

Public Odyssey data: deterministic sorted-file sample [::10], 1,499 of 14,987 CSVs, 898,631 source rows (~25 hours). Whole segment IDs with (ID//10)%5==0 are held out; 871 eligible test windows (145.2 minutes), with other sampled segments for fitting. Fit caps its random seed-42 sample at 1,800 training windows. This is segment-disjoint, **not proven owner-disjoint**: anonymized filenames do not identify owners/routes. Public data does not replace the CAN-level motor analysis. Source/field semantics: https://github.com/commaai/comma-steering-control .

## Model and measured mechanisms

At each speed, deadzoned torque drives a first-order motor state; desired steering acceleration is gain(v)*motor + roll_gain*roll + bias. The difference between desired and current steering acceleration must exceed friction_accel before steering moves. The update approaches the edge of this friction interval exponentially. This reproduces stick/slip and self-centering toward the zero-command equilibrium, while retaining a friction-sized residual. It is an effective hysteresis model, not a direct measurement of rack Coulomb torque.

| Speed m/s | William gain | Public gain | William equivalent command hysteresis |
|---:|---:|---:|---:|
| 10 | 1.659 | 1.029 | 0.290 |
| 20 | 2.451 | 2.090 | 0.204 |
| 30 | 3.727 | 2.265 | 0.143 |

William pooled lag is 0.872 s, effective friction threshold 0.440 m/s², bias 0.0005 m/s². Public pooled lag is 0.642 s, friction 0.377 m/s², roll coefficient 6.245 m/s²/rad, bias -0.044 m/s². Lag is pooled, not separately identified by speed. High-speed gains differ substantially; this may reflect the van, firmware, data selection, input normalization, or closed-loop identification bias. It does not prove a mechanical difference.

Quasi-static CAN motor fit (low driver torque, command stable over 300 ms): baseline + gain*max(|command|-deadband,0), baseline 1.44 CAN units, deadband 0.028, gain 111.9. Training n=3841, held-out n=4336; train/test RMS 3.32/3.58 CAN units. The initial pooled linear fit incorrectly suggested almost no deadband: telemetry has a 1–3 unit floor and assistance confounding. The refined fit supports 2–4%, not a precise universal 2.8% threshold. Motor sign is unavailable in telemetry and inferred from command only during autonomous steering.

Manual assist confirms a gradual threshold around 400–600 raw driver-torque units, rather than a hard universal switch at 500. In route 1a4, median motor torque is 3 units below raw driver torque 500, 4 at 500–600, 6 at 600–800, 11 at 800–1200, and 24 at 1200–2000. In 1a5 the corresponding medians are approximately 1–3, 3, 5, 11, 24. Raw driver torque is not a calibrated wheel torque; autonomous command-only model cannot predict manual driving without that external input.

Wheel mapping includes understeer: angle=-degrees(a*SR*WB*(1+0.0008927*v²)/v²), SR=17.8, WB=3.0 m. This empirical mapping used all eligible routes and is descriptive, not an independent held-out calibration. Fine angle improves resolution, but angle/rate errors below show the aggregate plant is not a fine steering sensor model.

## Validation: acceleration proxy

All errors m/s². One-step horizon is 100 ms, teacher-forced; free run is 10 seconds with only initial acceleration anchored. Persistence is the previous measured acceleration. A small one-step error alone is weak evidence because persistence already scores well. R² pools eligible window samples; it is not a route-level causal identification statistic.

| Route | Role | Windows | 100 ms RMS | Persistence RMS | 10 s RMS | R² |
|---|---|---:|---:|---:|---:|---:|
| 174 | training | 18 | 0.0129 | 0.0141 | 0.101 | 0.917 |
| 18c | training | 97 | 0.0104 | 0.0116 | 0.131 | 0.835 |
| 19b | training | 17 | 0.0108 | 0.0145 | 0.093 | 0.820 |
| 19c | held out | 139 | 0.0166 | 0.0176 | 0.168 | 0.776 |
| 1a3 | training | 44 | 0.0143 | 0.0162 | 0.145 | 0.784 |
| 1a7 | training | 33 | 0.0146 | 0.0165 | 0.162 | 0.718 |
| 1aa | held out | 62 | 0.0118 | 0.0125 | 0.145 | 0.765 |
| 1ab | held out | 74 | 0.0122 | 0.0138 | 0.132 | 0.830 |

Actual shared step() API was exercised separately: held-out 10 s RMS 19c/1aa/1ab = 0.168, 0.146, 0.134 m/s². Small differences from fit objective come from the extra nominal motor lag and numerical substeps.

| Held-out route | Speed m/s | Samples | 100 ms RMS | 10 s RMS |
|---|---|---:|---:|---:|
| 19c | 5-15 | 99 | 0.0085 | 0.041 |
| 19c | 15-25 | 179 | 0.0082 | 0.063 |
| 19c | 25-40 | 13483 | 0.0168 | 0.170 |
| 1aa | 5-15 | 792 | 0.0063 | 0.092 |
| 1aa | 15-25 | 4356 | 0.0131 | 0.159 |
| 1aa | 25-40 | 990 | 0.0086 | 0.109 |
| 1ab | 5-15 | 565 | 0.0076 | 0.154 |
| 1ab | 15-25 | 5110 | 0.0125 | 0.135 |
| 1ab | 25-40 | 1651 | 0.0125 | 0.115 |

public_test: 100 ms RMS 0.0160, persistence 0.0179, 10 s RMS 0.133, R² 0.821. Public-model held-out speed-band free RMS: 5-15: 0.070, 15-25: 0.105, 25-40: 0.138.


william_on_public_test: 100 ms RMS 0.0181, persistence 0.0179, 10 s RMS 0.182, R² 0.667. Public-model held-out speed-band free RMS: 5-15: 0.071, 15-25: 0.139, 25-40: 0.190.

The VSA LAT_ACCEL issue is decisive: across autonomous routes its correlation with controller angle-derived acceleration is only about -0.03 to -0.07. That decoded signal cannot validate body lateral response here. All liveParameters roll samples are zero, so William roll/crown gain is a 9.81 prior, not identified. Separate motor lag 30 ms and vehicle lag 10 ms are nominal priors; the fitted ~0.87 s is aggregate steering dynamics. The claimed ~0.6 s wheel-to-body lag is **not verified**. Road bank, alignment offset, static friction and crown remain confounded.

## Wheel and rate prediction

Free 10-second wheel angle RMS and 500 ms smoothed rate RMS, using the shared plant. These are descriptive because understeer mapping used all routes. No claim of sub-0.1-degree fidelity is justified.

| Route | Angle RMS deg | Rate RMS deg/s |
|---|---:|---:|
| 174 | 1.93 | 0.93 |
| 18c | 1.95 | 0.47 |
| 19b | 1.31 | 0.77 |
| 19c | 1.92 | 0.65 |
| 1a3 | 1.75 | 0.78 |
| 1a7 | 1.99 | 0.70 |
| 1aa | 2.12 | 0.78 |
| 1ab | 2.02 | 0.81 |
## Closed-loop simulator versus logged road

`simulate.py` is a new implementation under the requested directory; original tracked simulator remains untouched. It retains road + 0.336*offset + 0.911*sideways_speed, openpilot-like torque FF/PID, learned FF gain 1.47/friction 0.145, saturation and anti-windup, but uses the fitted plant. Controller kp=0.86, ki=0.15. Camera lane-centre curvature is fitted below 50 m. No artificial lane jitter is added: logged road already includes measurement noise. This controller is approximate: it does not duplicate all openpilot filters, actuator delay, rate limits, changing gains, maneuvers or A/B compensation.

Matched stretches: >=45 seconds, active, no driver override/blinker, >25 mph, >80% valid left/right lane probabilities. Invalid lane samples are interpolated for real wander, matching odyssey_wander. Period is peak desired-acceleration period in 7–20 s, follow is bandpassed desired-actual RMS / desired RMS. Values below are duration-weighted means of per-stretch metrics, not statistics of stitched roads. Route 174 has no eligible matched lane stretches; 1a4/1a5 are manual only.

| Route | Minutes | Wander real/sim cm RMS | Period real/sim s | Follow real/sim | Saturation real/sim % |
|---|---:|---:|---:|---:|---:|
| 18c | 14.0 | 7.03 / 11.29 | 17.72 / 11.86 | 0.26 / 0.51 | 0.00 / 0.13 |
| 19b | 1.0 | 25.12 / 12.69 | 19.12 / 7.17 | 0.26 / 0.39 | 0.96 / 0.00 |
| 19c | 19.9 | 8.19 / 18.81 | 15.29 / 14.49 | 0.30 / 0.58 | 0.00 / 0.71 |
| 1a3 | 4.8 | 11.05 / 13.39 | 13.64 / 12.53 | 0.35 / 0.65 | 0.00 / 0.68 |
| 1a7 | 2.9 | 6.43 / 11.53 | 15.30 / 9.78 | 0.55 / 0.64 | 0.00 / 0.09 |
| 1aa | 5.2 | 10.04 / 34.09 | 11.27 / 12.04 | 0.24 / 0.61 | 0.08 / 3.28 |
| 1ab | 6.5 | 8.80 / 10.69 | 15.51 / 12.71 | 0.26 / 0.45 | 0.00 / 0.00 |

**Reality check fails as a calibrated lane-wander model.** It gets the broad slow-wander frequency range, but usually overstates wander and follow error. 19b is a contrary small single-stretch case. On 1aa simulated wander is 34.1 cm versus 10.0 real and saturation is 3.28% versus 0.08%. The camera road estimate and simplified controller are major confounders, so this mismatch is not proof that the steering fit alone is wrong. A diagnostic attempt to regress lane gains using frame-differenced camera offset produced implausibly little damping; it was rejected because frame-difference offset includes perception jitter and is not measured lateral heading. Established lane-model gains were retained.

## Frozen-plan and frozen-torque replay

| Route | Real plan error RMS | Frozen-plan sim error RMS | Torque-replay acceleration RMS versus real | Frozen-plan follow / real follow |
|---|---:|---:|---:|---:|
| 18c | 0.052 | 0.089 | 0.179 | 0.42 / 0.26 |
| 19b | 0.089 | 0.125 | 0.240 | 0.37 / 0.26 |
| 19c | 0.062 | 0.113 | 0.235 | 0.49 / 0.30 |
| 1a3 | 0.058 | 0.089 | 0.155 | 0.63 / 0.35 |
| 1a7 | 0.074 | 0.075 | 0.217 | 0.50 / 0.55 |
| 1aa | 0.062 | 0.113 | 0.219 | 0.40 / 0.24 |
| 1ab | 0.062 | 0.098 | 0.170 | 0.40 / 0.26 |

Frozen plan replay tests approximate controller tracking; frozen torque replay tests the plant with actual logged input. Their integrated lane positions are unusable: frozen plans cannot adapt to a different simulated path, and tiny acceleration/road biases integrate into hundreds of metres over long stretches. Their 7–20 s offset estimates are contaminated by drift/finite-record FFT leakage (several metres RMS), not credible wander predictions. Full raw results are preserved in plant_sim_scores.json rather than silently discarded. Logged-road closed feedback is the meaningful lane comparison above.

## Route-wide curve tracking and saturation

These real-data statistics use all autonomous samples >5 m/s without steeringPressed, independent of lane visibility. A curve has |desired acceleration|>0.5 m/s². Saturation can mean command magnitude >=0.99 or controller logged saturated; these are different statistics. Fit windows excluded |command|>=0.95, so steering-fit scores do not validate the saturation regime.

| Route | Curve seconds | Curve plan error RMS | Cmd>=.99 % | Logged controller saturated % |
|---|---:|---:|---:|---:|
| 174 | 195.2 | 0.145 | 7.089 | 1.369 |
| 18c | 246.9 | 0.079 | 0.949 | 0.061 |
| 19b | 56.7 | 0.180 | 0.893 | 0.244 |
| 19c | 248.3 | 0.187 | 1.370 | 0.413 |
| 1a3 | 80.8 | 0.074 | 0.000 | 0.000 |
| 1a7 | 130.4 | 0.221 | 6.945 | 1.749 |
| 1aa | 216.7 | 0.136 | 2.488 | 0.180 |
| 1ab | 210.9 | 0.137 | 4.081 | 0.542 |
## What to trust and next exact step

Trust the motor threshold to a few percentage points, the gradual manual-assist threshold, speed-dependent aggregate response, and the held-out prediction errors in this report. Treat fitted friction/gain/lag as coupled phenomenological parameters. Do not trust this model for absolute lane centering, centimeter-scale changes from a kick, precise wheel-rate prediction, body acceleration, road crown, owner generalization, or safety limits.

Next step before using controller rankings for the van: recover a physically coherent independent lateral acceleration/yaw channel, and replay the actual logged openpilot controller/planner stack with observed lane heading, rate limits and delay. Keep the public-model overlay and William model as separate robustness conditions. Do not tune a controller merely until this simulator reports less wander.

## Reproduction and verification

From ~/Code/odyssey-pilot with PYTHONPATH=$PWD: use .venv/bin/python for extract.py, public_data.py, diagnostics.py, verify.py, finish.py and simulate.py; fit.py uses scratchpad/wander/venv/bin/python for scipy. Order: extract, public_data, fit, diagnostics, verify, finish, simulate. Extraction checks its scratch cache first. All ten raw routes were read; public sample selection is deterministic. JSON scores and cached windows support independent reanalysis. Scripts use main guards for multiprocessing.

Shared API exercised on all held-out windows and on 20 long constant-command/speed combinations: finite/bounded responses, expected signs, immutable inputs. Ruff check passes. Original tracked files were not edited; artifacts are untracked. Analysis is offline only.

