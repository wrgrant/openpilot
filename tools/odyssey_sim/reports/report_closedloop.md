# Odyssey closed-loop steering comparison

Offline research only. No vehicle access, parameter deployment, tracked-file edits, commits or pushes.

## Plain-language answer

At ≥20 mph, NNLC 0.32 s has 8.4% lower cost than stock V0 in the William plant (447.85 versus 489.18), but is essentially tied in the public overlay (1554.62 versus 1554.99). NNLC 0.45 s raises William-plant reversals by 39% (0.67 to 0.93/s) without a robust tracking benefit. The fitted Odyssey plant and the generic comma plant disagree about what wins. This experiment identifies candidates for further validation; it does not establish the best controller for the real van. Low-speed angle steering cannot be judged reliably because neither available learned plant validates that regime.

## Overall results

Challenge cost is 50 × lateral-acceleration cost + jerk cost; lower is better. Figures below weight each route/window by eligible sample count. These are curve-heavy selected windows, not full-drive averages.

|Configuration|Provisional total|William total|Public-overlay total|Generic tinyphysics total|
|---|---:|---:|---:|---:|
|Stock + 0.32s|693.37|477.11|1580.59|3424.99|
|Stock + 0.45s|693.37|477.11|1580.59|3424.99|
|Stock + 0.32s + angle|699.50|483.83|1585.14|3458.92|
|Stock + 0.32s + friction×0.5|693.69|487.20|1590.22|3205.20|
|Stock + 0.32s + threshold0.6|693.69|484.64|1590.19|3300.72|
|Stock + 0.32s + kick0.03|692.31|478.06|1577.53|3527.81|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|696.66|492.33|1589.64|3345.52|
|NNLC + 0.32s|828.44|535.88|1628.83|3951.09|
|NNLC + 0.45s|842.64|537.94|1628.40|3899.80|
|NNLC + 0.32s + angle|727.42|450.13|1584.52|3885.43|
|NNLC + 0.32s + friction×0.5|827.86|535.84|1628.31|3975.31|
|NNLC + 0.32s + threshold0.6|827.86|535.84|1628.19|3975.74|
|NNLC + 0.32s + kick0.03|825.38|534.34|1629.46|3983.83|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|722.64|446.63|1576.65|3954.39|

![Cost comparison](closedloop_chart.svg)

## Ranked candidates and expected effect

**Primary ranking uses ≥20 mph only** (1,450 scored samples = 145 seconds per arm); overall totals above include invalid low-speed extrapolation. The fitted-model train routes include 1a7/18c/19b; 19c/1aa/1ab are route-held-out. Neither plant is calibrated for lane position.

1. **NNLC + 0.32s + angle + friction×0.5 + kick0.03**: William ≥20 mph total 445.99, 8.8% below stock. Public-overlay 1546.95; provisional 806.05; generic 3993.06.
2. **NNLC + 0.32s + kick0.03**: William ≥20 mph total 446.02, 8.8% below stock. Public-overlay 1547.60; provisional 806.69; generic 4047.84.
3. **NNLC + 0.32s + friction×0.5**: William ≥20 mph total 447.80, 8.5% below stock. Public-overlay 1553.98; provisional 809.17; generic 3921.03.
4. **NNLC + 0.32s + threshold0.6**: William ≥20 mph total 447.80, 8.5% below stock. Public-overlay 1553.83; provisional 809.17; generic 3920.57.

|≥20 mph configuration|William|Public-overlay|Provisional|Generic|
|---|---:|---:|---:|---:|
|Stock + 0.32s|489.18|1554.99|790.96|3376.74|
|Stock + 0.45s|489.18|1554.99|790.96|3376.74|
|Stock + 0.32s + angle|489.18|1554.99|790.96|3376.74|
|Stock + 0.32s + friction×0.5|501.63|1566.83|791.36|3165.68|
|Stock + 0.32s + threshold0.6|498.45|1566.80|791.36|3266.06|
|Stock + 0.32s + kick0.03|490.39|1551.32|789.70|3522.30|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|502.14|1562.50|789.96|3274.54|
|NNLC + 0.32s|447.85|1554.62|809.85|3915.71|
|NNLC + 0.45s|450.25|1554.35|817.51|3876.79|
|NNLC + 0.32s + angle|447.85|1554.62|809.85|3909.72|
|NNLC + 0.32s + friction×0.5|447.80|1553.98|809.17|3921.03|
|NNLC + 0.32s + threshold0.6|447.80|1553.83|809.17|3920.57|
|NNLC + 0.32s + kick0.03|446.02|1547.60|806.69|4047.84|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|445.99|1546.95|806.05|3993.06|

Route-held-out ≥20 mph robustness:

|Configuration|William|Public overlay|
|---|---:|---:|
|Stock + 0.32s|278.88|933.25|
|Stock + 0.45s|278.88|933.25|
|Stock + 0.32s + angle|278.88|933.25|
|Stock + 0.32s + friction×0.5|296.16|949.43|
|Stock + 0.32s + threshold0.6|290.67|949.38|
|Stock + 0.32s + kick0.03|281.03|927.93|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|297.33|943.34|
|NNLC + 0.32s|216.18|935.09|
|NNLC + 0.45s|217.49|932.91|
|NNLC + 0.32s + angle|216.17|935.09|
|NNLC + 0.32s + friction×0.5|216.24|934.38|
|NNLC + 0.32s + threshold0.6|216.24|933.99|
|NNLC + 0.32s + kick0.03|214.77|925.56|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|214.82|924.97|

Recommendation: retain the current road configuration until the fitted model passes independent trajectory validation; keep 0.32 s as the preferred delay, and prioritize unmodified NNLC versus stock as the next robustness comparison. Avoid deploying the combined arm based on this ranking alone: interactions obscure what helped. The van’s driver takeovers, lateral path error and safety cannot be predicted by these acceleration-only costs. At ≥20 mph, the 0.03 kick saves only about 0.4–0.5% versus NNLC alone on the William/public plants while raising command reversals; it is not a compelling change from this evidence. Stock V0 is delay-invariant in these runs; NNLC 0.45 s increases reversals and has no robust tracking advantage.

Delay changes: 0.45 s changes both LaC history alignment and NNLC future interpolation. The logged upstream plan remains fixed, so this is not a complete replay of changing steerActuatorDelay in the running planner.

Friction softening can reduce oscillation but may delay breaking static friction; a 0.03 kick can improve deadband response while increasing chatter or near-zero bias. NNLC has friction learned into its model: halving the explicit torque-parameter friction does not halve learned NN friction. Angle mode uses the lead’s real controller-independent OdysseySteer.apply after V0/NNLC, plus the V0 odyssey_kp_interp wiring (gain cap8 when angle enabled). The deadband kick also executes in this shared post-processing path. The prior V1-only wiring did not execute on the recorded drives; these arms predict the newly corrected wiring, rather than historical feature performance.

## Harness, scope and validation

- Uses the real V0 LatControlTorque, sunnypilot extension, Odyssey NNLC JSON, VehicleModel and real logged modelV2 arrays. See tools/odyssey_sim/closedloop/README.md for exact execution commands and conventions.
- All six requested drives are decoded from local rlog FILE paths. At 10 Hz the CSV preserves speed, longitudinal acceleration, roll and planned lateral acceleration; NPZ adds future acceleration/orientation arrays. The actual controller curvature input is carControl.actuators.curvature. The controlsState desiredCurvature is retained separately; their eligible route RMS acceleration difference is 0.0031–0.0096 m/s².
- 25 selected 30-second windows (750 seconds of context) per plant/configuration, ten seconds discarded, then band/latActive/!steeringPressed masking. Each window has at least three eligible seconds. The same window can be selected for adjacent bands; scores never count out-of-band samples. This deliberately stresses curves and overweights difficult episodes.
- Native plants execute actual controller and dynamics at 100 Hz with 10 Hz logged exogenous inputs held. Tinyphysics imports comma’s actual TinyPhysicsSimulator/Model and uses 10 Hz physics with ten native controller calls per observation. It has no interpolated 100 Hz motion feedback, so its jerk/history response is a baseline approximation.
- Tinyphysics uses its original tokenizer, stochastic sampling, history, warmup and ±0.5 m/s²-per-step cap. Seed is fixed per route/window and shared across arms. One seed per window is insufficient for confidence intervals.
- The provisional plant assumes gain1.47, deadband0.03, Coulomb-like friction0.145, offset−0.055 and lag0.4s. These are priors, not fitted dynamics. Gain, friction, hysteresis and lag uncertainty can reverse rankings.
- Corrected field names: live roll/ratio/offset from vehicleParameters (roll std 0.0177–0.0264 rad across routes); torque parameters are lateralTorqueParameters and delay lateralDelay in validation. The controller comparison deliberately fixes learned torque params at 1.47/0.145/−0.055 and delays at 0.32/0.45. calibrated_pose is None because livePose is unavailable, so NNLC pitch correction is not reconstructed. Worker 2’s William roll gain is a prior because its fit extractor missed this field; applying that fit with live roll is a material model uncertainty.
- Worker 2 schema-v1 step() plant is used with a frozen parameter snapshot. Feedback/cost uses steering_accel (wheel-derived acceleration, matching torqueState), rather than separately lagged inertial lat_accel. Real VehicleModel provides the inverse wheel-angle measurement. Worker plant’s own bicycle angle output is not fed directly into a different vehicle model.
- No CAN torque rate limiter, Honda safety/driver torque limiter, driver input, sensor quantization, lateral planner response or lane geometry feedback. Existing inactive or driver-pressed context is still simulated with control active; those samples are excluded from costs.
- All fourteen construction settings are verified by assertions and check.py; positive torque increases plant acceleration; real VehicleModel inverse feedback error <1e-9 m/s²; cost identity checks pass. Params writes are blocking and every run has an isolated simclosed-prefixed namespace.

Logged-motion validation: {"frames": 33727, "eligible": 3000, "logged_version": [0], "nnlc_v0_logged": {"command_mae": 2.592773238276481e-05, "command_rmse": 0.00020350165285428775, "correlation": 0.9999993207303991}, "nnlc_v1_logged": {"command_mae": 0.01685508938447835, "command_rmse": 0.020632901370697135, "correlation": 0.9979307732597522}}

The matching v0 NNLC replay closely reproduces logged commands. The separate v1 diagnostic differs from V0 in the logs (RMSE ~0.0206 normalized torque); all final sweeps use V0, which matches logged v0 with RMSE ~0.00020. This validates data/sign/instantiation, not closed-loop plant fidelity.

Stock-controller simulated versus recorded tracking on identical selected samples:

|Speed mph|Recorded accel cost|Stock provisional|Stock William|Stock public|Stock tinyphysics|
|---|---:|---:|---:|---:|---:|
|0-10|1.462|0.192|0.802|8.124|52.365|
|10-20|4.556|9.692|14.343|53.362|53.977|
|20-35|4.851|53.843|35.648|121.714|39.942|
|35-50|0.358|0.108|0.885|0.577|44.309|
|50+|9.091|5.590|1.868|3.521|49.309|

**The stock simulated tracking costs do not reproduce recorded tracking costs consistently.** These are fixed learned-parameter/V0 runs, whereas the logs contain dynamic parameters, NNLC/v0 and driver/system transitions. Even after accounting for those differences, this is a failed criterion for claiming validated van prediction. Plant fit validation and held-out results in report_plant.md bound all interpretation.

## Per-plant and speed-band metrics

L = mean squared accel error ×100; J = mean squared 10 Hz acceleration derivative ×100; T=50L+J. Sat is |torque|≥0.99 command percentage. Physical Sat/flips use 100 Hz commands; generic uses applied 10 Hz commands. Flips require >0.01 torque change; adjacent samples must both be eligible. Entry/exit are sample-count-weighted means of route medians in seconds, with matched event counts in parentheses. NA means no matched event, not zero lag. Positive is late, negative early. Crossing threshold |accel|=0.3 m/s²; nearest crossing within ±2 seconds is descriptive and may mismatch oscillatory events. Aggregated J is eligible-sample-weighted rather than pooled-adjacent-pair weighted.

### provisional

#### 0-10 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|0.192|5.72|15.30|16.09|13.05|NA|NA|151|
|Stock + 0.45s|0.192|5.72|15.30|16.09|13.05|NA|NA|151|
|Stock + 0.32s + angle|0.458|5.26|28.15|6.29|1.59|NA|NA|151|
|Stock + 0.32s + friction×0.5|0.192|5.79|15.38|15.76|13.25|NA|NA|151|
|Stock + 0.32s + threshold0.6|0.192|5.79|15.38|15.76|13.25|NA|NA|151|
|Stock + 0.32s + kick0.03|0.191|5.76|15.31|16.69|13.84|NA|NA|151|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|0.425|5.39|26.64|6.36|2.45|NA|NA|151|
|NNLC + 0.32s|25.176|10.35|1269.13|62.32|50.79|NA|NA|151|
|NNLC + 0.45s|25.316|11.21|1277.01|63.91|51.46|NA|NA|151|
|NNLC + 0.32s + angle|0.458|5.26|28.15|6.29|1.59|NA|NA|151|
|NNLC + 0.32s + friction×0.5|25.180|10.24|1269.26|62.58|50.99|NA|NA|151|
|NNLC + 0.32s + threshold0.6|25.180|10.24|1269.26|62.58|50.99|NA|NA|151|
|NNLC + 0.32s + kick0.03|24.996|10.02|1259.81|62.58|51.79|NA|NA|151|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|0.425|5.39|26.64|6.36|2.45|NA|NA|151|

#### 10-20 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|9.692|10.24|494.86|42.34|0.61|0.50 (1)|-0.40 (1)|197|
|Stock + 0.45s|9.692|10.24|494.86|42.34|0.61|0.50 (1)|-0.40 (1)|197|
|Stock + 0.32s + angle|10.713|5.22|540.86|25.38|0.00|0.80 (1)|-0.50 (1)|197|
|Stock + 0.32s + friction×0.5|9.693|10.13|494.78|42.34|0.61|0.50 (1)|-0.40 (1)|197|
|Stock + 0.32s + threshold0.6|9.693|10.13|494.78|42.34|0.61|0.50 (1)|-0.40 (1)|197|
|Stock + 0.32s + kick0.03|9.684|10.16|494.38|42.34|0.71|0.00 (1)|0.00 (1)|197|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|10.366|5.10|523.42|27.56|0.00|0.70 (1)|-0.40 (1)|197|
|NNLC + 0.32s|11.809|37.04|627.47|53.65|13.20|0.60 (1)|-0.40 (1)|197|
|NNLC + 0.45s|12.959|46.71|694.65|55.28|16.14|0.60 (1)|-0.40 (1)|197|
|NNLC + 0.32s + angle|12.640|24.77|656.75|31.47|10.76|0.80 (1)|-0.50 (1)|197|
|NNLC + 0.32s + friction×0.5|11.802|37.00|627.09|53.65|13.20|0.60 (1)|-0.40 (1)|197|
|NNLC + 0.32s + threshold0.6|11.802|37.00|627.09|53.65|13.20|0.60 (1)|-0.40 (1)|197|
|NNLC + 0.32s + kick0.03|11.846|37.63|629.94|53.91|13.20|0.60 (1)|-0.40 (1)|197|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|12.334|25.49|642.19|33.71|10.76|0.70 (1)|-0.40 (1)|197|

#### 20-35 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|53.843|0.79|2692.93|73.20|0.14|NA|NA|347|
|Stock + 0.45s|53.843|0.79|2692.93|73.20|0.14|NA|NA|347|
|Stock + 0.32s + angle|53.843|0.79|2692.93|73.20|0.14|NA|NA|347|
|Stock + 0.32s + friction×0.5|53.845|0.75|2692.99|73.20|0.14|NA|NA|347|
|Stock + 0.32s + threshold0.6|53.845|0.75|2692.99|73.20|0.14|NA|NA|347|
|Stock + 0.32s + kick0.03|53.836|0.80|2692.61|73.49|0.26|NA|NA|347|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|53.837|0.76|2692.64|73.46|0.20|NA|NA|347|
|NNLC + 0.32s|54.013|2.26|2702.92|76.22|4.09|NA|NA|347|
|NNLC + 0.45s|54.238|2.35|2714.25|76.83|5.88|NA|NA|347|
|NNLC + 0.32s + angle|54.013|2.26|2702.92|76.22|4.09|NA|NA|347|
|NNLC + 0.32s + friction×0.5|54.009|2.26|2702.72|76.25|4.09|NA|NA|347|
|NNLC + 0.32s + threshold0.6|54.009|2.26|2702.72|76.25|4.09|NA|NA|347|
|NNLC + 0.32s + kick0.03|54.011|2.48|2703.05|76.34|4.27|NA|NA|347|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|54.009|2.48|2702.93|76.34|4.27|NA|NA|347|

#### 35-50 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|0.108|3.70|9.10|3.21|0.30|NA|NA|368|
|Stock + 0.45s|0.108|3.70|9.10|3.21|0.30|NA|NA|368|
|Stock + 0.32s + angle|0.108|3.70|9.10|3.21|0.30|NA|NA|368|
|Stock + 0.32s + friction×0.5|0.119|3.31|9.24|3.15|0.33|NA|NA|368|
|Stock + 0.32s + threshold0.6|0.119|3.31|9.24|3.15|0.33|NA|NA|368|
|Stock + 0.32s + kick0.03|0.074|3.84|7.55|3.26|0.33|NA|NA|368|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|0.082|3.44|7.53|3.18|0.38|NA|NA|368|
|NNLC + 0.32s|0.697|4.86|39.71|4.38|6.82|NA|NA|368|
|NNLC + 0.45s|0.821|4.90|45.97|5.27|11.28|NA|NA|368|
|NNLC + 0.32s + angle|0.697|4.86|39.72|4.38|6.82|NA|NA|368|
|NNLC + 0.32s + friction×0.5|0.681|4.86|38.91|4.38|6.82|NA|NA|368|
|NNLC + 0.32s + threshold0.6|0.681|4.86|38.91|4.38|6.82|NA|NA|368|
|NNLC + 0.32s + kick0.03|0.603|5.31|35.43|4.51|7.07|NA|NA|368|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|0.587|5.31|34.67|4.54|7.07|NA|NA|368|

#### 50+ mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|5.590|5.00|284.48|6.97|0.01|0.20 (1)|0.00 (1)|735|
|Stock + 0.45s|5.590|5.00|284.48|6.97|0.01|0.20 (1)|0.00 (1)|735|
|Stock + 0.32s + angle|5.590|5.00|284.48|6.97|0.01|0.20 (1)|0.00 (1)|735|
|Stock + 0.32s + friction×0.5|5.611|4.61|285.17|6.86|0.03|0.20 (1)|0.00 (1)|735|
|Stock + 0.32s + threshold0.6|5.611|4.61|285.17|6.86|0.03|0.20 (1)|0.00 (1)|735|
|Stock + 0.32s + kick0.03|5.558|5.03|282.93|7.12|0.04|0.10 (1)|0.00 (1)|735|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|5.576|4.64|283.45|7.03|0.01|0.10 (1)|0.00 (1)|735|
|NNLC + 0.32s|5.947|4.33|301.70|6.97|3.29|0.20 (1)|0.20 (1)|735|
|NNLC + 0.45s|6.078|4.41|308.33|8.00|6.64|0.30 (1)|0.10 (1)|735|
|NNLC + 0.32s + angle|5.947|4.33|301.70|6.97|3.29|0.20 (1)|0.20 (1)|735|
|NNLC + 0.32s + friction×0.5|5.930|4.33|300.85|6.98|3.29|0.20 (1)|0.20 (1)|735|
|NNLC + 0.32s + threshold0.6|5.930|4.33|300.85|6.98|3.29|0.20 (1)|0.20 (1)|735|
|NNLC + 0.32s + kick0.03|5.863|4.40|297.56|6.98|3.39|0.10 (1)|0.20 (1)|735|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|5.847|4.40|296.74|7.01|3.39|0.10 (1)|0.20 (1)|735|

### odyssey

**0–10 mph is outside the plant’s intended range; speed is clamped at 5 m/s. Those rows are extrapolation diagnostics only. 10–20 mph includes some samples below that limit.**

#### 0-10 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|0.802|3.71|43.79|16.42|4.17|NA|NA|151|
|Stock + 0.45s|0.802|3.71|43.79|16.42|4.17|NA|NA|151|
|Stock + 0.32s + angle|1.031|2.68|54.25|9.67|2.19|NA|NA|151|
|Stock + 0.32s + friction×0.5|0.802|3.70|43.80|16.36|4.17|NA|NA|151|
|Stock + 0.32s + threshold0.6|0.802|3.70|43.80|16.36|4.17|NA|NA|151|
|Stock + 0.32s + kick0.03|0.801|3.78|43.80|16.62|4.30|NA|NA|151|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|1.017|2.72|53.58|9.80|2.72|NA|NA|151|
|NNLC + 0.32s|22.790|2.39|1141.87|35.03|11.52|NA|NA|151|
|NNLC + 0.45s|22.796|2.52|1142.34|37.15|13.25|NA|NA|151|
|NNLC + 0.32s + angle|1.031|2.68|54.25|9.67|2.19|NA|NA|151|
|NNLC + 0.32s + friction×0.5|22.790|2.39|1141.88|35.03|11.52|NA|NA|151|
|NNLC + 0.32s + threshold0.6|22.790|2.39|1141.88|35.03|11.52|NA|NA|151|
|NNLC + 0.32s + kick0.03|22.775|2.42|1141.19|36.16|12.12|NA|NA|151|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|1.017|2.72|53.58|9.80|2.72|NA|NA|151|

#### 10-20 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|14.343|3.30|720.47|44.37|0.56|0.50 (1)|0.10 (1)|197|
|Stock + 0.45s|14.343|3.30|720.47|44.37|0.56|0.50 (1)|0.10 (1)|197|
|Stock + 0.32s + angle|15.446|1.48|773.79|31.47|0.25|NA|NA|197|
|Stock + 0.32s + friction×0.5|14.354|3.21|720.89|44.26|0.56|0.50 (1)|0.10 (1)|197|
|Stock + 0.32s + threshold0.6|14.354|3.21|720.89|44.26|0.56|0.50 (1)|0.10 (1)|197|
|Stock + 0.32s + kick0.03|14.333|3.48|720.12|44.72|0.56|0.50 (1)|0.10 (1)|197|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|15.099|1.54|756.47|35.53|0.36|NA|NA|197|
|NNLC + 0.32s|14.298|4.41|719.30|52.34|8.32|0.50 (1)|-0.40 (1)|197|
|NNLC + 0.45s|14.306|4.79|720.09|53.76|9.95|0.50 (1)|-0.20 (1)|197|
|NNLC + 0.32s + angle|15.363|2.18|770.31|34.47|6.14|NA|NA|197|
|NNLC + 0.32s + friction×0.5|14.298|4.41|719.31|52.34|8.32|0.50 (1)|-0.40 (1)|197|
|NNLC + 0.32s + threshold0.6|14.298|4.41|719.31|52.34|8.32|0.50 (1)|-0.40 (1)|197|
|NNLC + 0.32s + kick0.03|14.296|4.50|719.28|52.59|8.22|0.50 (1)|-0.40 (1)|197|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|15.007|2.27|752.63|38.78|6.19|NA|NA|197|

#### 20-35 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|35.648|0.53|1782.93|72.05|0.00|NA|NA|347|
|Stock + 0.45s|35.648|0.53|1782.93|72.05|0.00|NA|NA|347|
|Stock + 0.32s + angle|35.648|0.53|1782.93|72.05|0.00|NA|NA|347|
|Stock + 0.32s + friction×0.5|35.710|0.53|1786.02|72.05|0.00|NA|NA|347|
|Stock + 0.32s + threshold0.6|35.710|0.53|1786.02|72.05|0.00|NA|NA|347|
|Stock + 0.32s + kick0.03|35.630|0.53|1782.02|72.05|0.00|NA|NA|347|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|35.688|0.53|1784.94|72.05|0.00|NA|NA|347|
|NNLC + 0.32s|35.469|0.54|1774.01|72.16|0.29|NA|NA|347|
|NNLC + 0.45s|35.562|1.02|1779.12|72.13|0.63|NA|NA|347|
|NNLC + 0.32s + angle|35.469|0.54|1774.01|72.16|0.29|NA|NA|347|
|NNLC + 0.32s + friction×0.5|35.469|0.54|1774.00|72.16|0.29|NA|NA|347|
|NNLC + 0.32s + threshold0.6|35.469|0.54|1773.98|72.16|0.29|NA|NA|347|
|NNLC + 0.32s + kick0.03|35.452|0.54|1773.15|72.16|0.40|NA|NA|347|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|35.452|0.54|1773.14|72.16|0.40|NA|NA|347|

#### 35-50 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|0.885|1.30|45.58|0.00|0.19|NA|NA|368|
|Stock + 0.45s|0.885|1.30|45.58|0.00|0.19|NA|NA|368|
|Stock + 0.32s + angle|0.885|1.30|45.58|0.00|0.19|NA|NA|368|
|Stock + 0.32s + friction×0.5|1.082|1.13|55.25|0.00|0.27|NA|NA|368|
|Stock + 0.32s + threshold0.6|1.077|1.13|54.99|0.00|0.27|NA|NA|368|
|Stock + 0.32s + kick0.03|0.902|1.40|46.50|0.00|0.33|NA|NA|368|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|1.092|1.21|55.81|0.00|0.22|NA|NA|368|
|NNLC + 0.32s|0.427|1.26|22.63|0.05|0.76|NA|NA|368|
|NNLC + 0.45s|0.429|1.26|22.70|0.11|0.98|NA|NA|368|
|NNLC + 0.32s + angle|0.427|1.26|22.63|0.05|0.76|NA|NA|368|
|NNLC + 0.32s + friction×0.5|0.428|1.26|22.64|0.05|0.76|NA|NA|368|
|NNLC + 0.32s + threshold0.6|0.428|1.26|22.64|0.05|0.76|NA|NA|368|
|NNLC + 0.32s + kick0.03|0.390|1.37|20.86|0.05|1.30|NA|NA|368|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|0.390|1.38|20.87|0.05|1.28|NA|NA|368|

#### 50+ mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|1.868|7.09|100.48|3.61|0.54|0.10 (1)|-0.10 (1)|735|
|Stock + 0.45s|1.868|7.09|100.48|3.61|0.54|0.10 (1)|-0.10 (1)|735|
|Stock + 0.32s + angle|1.868|7.09|100.48|3.61|0.54|0.10 (1)|-0.10 (1)|735|
|Stock + 0.32s + friction×0.5|2.235|7.02|118.75|3.97|0.41|0.10 (1)|-0.10 (1)|735|
|Stock + 0.32s + threshold0.6|2.111|7.04|112.62|3.84|0.39|0.10 (1)|-0.10 (1)|735|
|Stock + 0.32s + kick0.03|1.910|7.38|102.86|3.70|0.76|0.10 (1)|-0.20 (1)|735|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|2.254|7.27|119.98|4.04|0.67|0.10 (1)|-0.20 (1)|735|
|NNLC + 0.32s|0.576|5.85|34.66|0.93|0.80|0.10 (1)|0.30 (1)|735|
|NNLC + 0.45s|0.625|5.68|36.95|0.91|1.05|0.20 (1)|0.30 (1)|735|
|NNLC + 0.32s + angle|0.576|5.85|34.66|0.93|0.80|0.10 (1)|0.30 (1)|735|
|NNLC + 0.32s + friction×0.5|0.574|5.84|34.56|0.94|0.79|0.10 (1)|0.30 (1)|735|
|NNLC + 0.32s + threshold0.6|0.575|5.85|34.57|0.93|0.79|0.10 (1)|0.30 (1)|735|
|NNLC + 0.32s + kick0.03|0.526|6.05|32.35|0.97|1.25|0.10 (1)|0.20 (1)|735|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|0.525|6.05|32.29|1.02|1.25|0.10 (1)|0.20 (1)|735|

### public

**0–10 mph is outside the plant’s intended range; speed is clamped at 5 m/s. Those rows are extrapolation diagnostics only. 10–20 mph includes some samples below that limit.**

#### 0-10 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|8.124|1.22|407.41|46.82|2.32|NA|NA|151|
|Stock + 0.45s|8.124|1.22|407.41|46.82|2.32|NA|NA|151|
|Stock + 0.32s + angle|8.376|1.01|419.81|39.54|0.93|NA|NA|151|
|Stock + 0.32s + friction×0.5|8.124|1.22|407.43|46.82|2.32|NA|NA|151|
|Stock + 0.32s + threshold0.6|8.124|1.22|407.43|46.82|2.32|NA|NA|151|
|Stock + 0.32s + kick0.03|8.122|1.22|407.33|46.89|2.32|NA|NA|151|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|8.351|1.04|418.60|39.67|0.99|NA|NA|151|
|NNLC + 0.32s|19.719|1.11|987.07|42.98|9.27|NA|NA|151|
|NNLC + 0.45s|19.672|1.24|984.86|44.30|10.20|NA|NA|151|
|NNLC + 0.32s + angle|8.376|1.01|419.80|39.54|0.93|NA|NA|151|
|NNLC + 0.32s + friction×0.5|19.719|1.11|987.07|42.98|9.27|NA|NA|151|
|NNLC + 0.32s + threshold0.6|19.719|1.11|987.07|42.98|9.27|NA|NA|151|
|NNLC + 0.32s + kick0.03|21.225|1.13|1062.38|41.19|10.99|NA|NA|151|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|8.351|1.04|418.59|39.67|0.99|NA|NA|151|

#### 10-20 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|53.362|0.24|2668.32|75.79|0.10|0.80 (1)|-0.20 (1)|197|
|Stock + 0.45s|53.362|0.24|2668.32|75.79|0.10|0.80 (1)|-0.20 (1)|197|
|Stock + 0.32s + angle|54.001|0.25|2700.30|54.82|0.00|NA|NA|197|
|Stock + 0.32s + friction×0.5|53.374|0.24|2668.95|75.58|0.10|0.80 (1)|0.00 (1)|197|
|Stock + 0.32s + threshold0.6|53.374|0.24|2668.95|75.58|0.10|0.80 (1)|0.00 (1)|197|
|Stock + 0.32s + kick0.03|53.344|0.25|2667.44|76.04|0.10|0.80 (1)|0.10 (1)|197|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|53.735|0.26|2687.03|58.98|0.00|NA|NA|197|
|NNLC + 0.32s|53.331|0.37|2666.93|78.53|1.32|0.90 (1)|-0.40 (1)|197|
|NNLC + 0.45s|53.327|0.37|2666.73|77.61|2.13|0.80 (1)|-0.20 (1)|197|
|NNLC + 0.32s + angle|53.941|0.32|2697.36|53.60|0.76|1.50 (1)|0.10 (1)|197|
|NNLC + 0.32s + friction×0.5|53.331|0.37|2666.91|79.14|1.32|0.90 (1)|-0.40 (1)|197|
|NNLC + 0.32s + threshold0.6|53.331|0.37|2666.91|79.14|1.32|0.90 (1)|-0.40 (1)|197|
|NNLC + 0.32s + kick0.03|53.326|0.37|2666.69|78.98|1.22|0.90 (1)|-0.40 (1)|197|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|53.651|0.33|2682.87|57.87|1.17|1.30 (1)|0.20 (1)|197|

#### 20-35 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|121.714|0.42|6086.14|74.93|0.00|NA|NA|347|
|Stock + 0.45s|121.714|0.42|6086.14|74.93|0.00|NA|NA|347|
|Stock + 0.32s + angle|121.714|0.42|6086.14|74.93|0.00|NA|NA|347|
|Stock + 0.32s + friction×0.5|122.068|0.41|6103.81|74.84|0.00|NA|NA|347|
|Stock + 0.32s + threshold0.6|122.067|0.41|6103.76|74.87|0.00|NA|NA|347|
|Stock + 0.32s + kick0.03|121.410|0.42|6070.91|74.93|0.00|NA|NA|347|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|121.718|0.40|6086.29|74.93|0.00|NA|NA|347|
|NNLC + 0.32s|122.174|0.47|6109.17|74.81|0.14|NA|NA|347|
|NNLC + 0.45s|122.007|0.67|6101.01|74.70|0.46|NA|NA|347|
|NNLC + 0.32s + angle|122.174|0.47|6109.17|74.81|0.14|NA|NA|347|
|NNLC + 0.32s + friction×0.5|122.140|0.47|6107.48|74.81|0.14|NA|NA|347|
|NNLC + 0.32s + threshold0.6|122.121|0.47|6106.51|74.81|0.14|NA|NA|347|
|NNLC + 0.32s + kick0.03|121.732|0.47|6087.04|74.93|0.14|NA|NA|347|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|121.708|0.47|6085.86|75.01|0.14|NA|NA|347|

#### 35-50 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|0.577|1.23|30.07|1.09|0.19|NA|NA|368|
|Stock + 0.45s|0.577|1.23|30.07|1.09|0.19|NA|NA|368|
|Stock + 0.32s + angle|0.577|1.23|30.07|1.09|0.19|NA|NA|368|
|Stock + 0.32s + friction×0.5|0.717|1.06|36.91|1.09|0.14|NA|NA|368|
|Stock + 0.32s + threshold0.6|0.717|1.06|36.91|1.09|0.14|NA|NA|368|
|Stock + 0.32s + kick0.03|0.568|1.30|29.68|1.11|0.19|NA|NA|368|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|0.704|1.13|36.31|1.09|0.19|NA|NA|368|
|NNLC + 0.32s|0.389|1.20|20.64|0.65|0.27|NA|NA|368|
|NNLC + 0.45s|0.393|1.20|20.88|0.68|0.49|NA|NA|368|
|NNLC + 0.32s + angle|0.389|1.20|20.64|0.65|0.27|NA|NA|368|
|NNLC + 0.32s + friction×0.5|0.389|1.20|20.64|0.65|0.27|NA|NA|368|
|NNLC + 0.32s + threshold0.6|0.389|1.20|20.64|0.65|0.27|NA|NA|368|
|NNLC + 0.32s + kick0.03|0.355|1.27|19.03|0.68|0.38|NA|NA|368|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|0.355|1.27|19.03|0.68|0.38|NA|NA|368|

#### 50+ mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|3.521|3.24|179.28|6.34|0.24|0.20 (1)|0.10 (1)|735|
|Stock + 0.45s|3.521|3.24|179.28|6.34|0.24|0.20 (1)|0.10 (1)|735|
|Stock + 0.32s + angle|3.521|3.24|179.28|6.34|0.24|0.20 (1)|0.10 (1)|735|
|Stock + 0.32s + friction×0.5|3.757|3.06|190.89|6.30|0.15|0.20 (1)|0.10 (1)|735|
|Stock + 0.32s + threshold0.6|3.756|3.06|190.85|6.30|0.15|0.20 (1)|0.10 (1)|735|
|Stock + 0.32s + kick0.03|3.522|3.31|179.43|6.34|0.42|0.20 (1)|0.00 (1)|735|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|3.756|3.12|190.91|6.31|0.33|0.20 (1)|0.00 (1)|735|
|NNLC + 0.32s|3.393|2.78|172.41|6.12|0.46|0.20 (1)|0.40 (1)|735|
|NNLC + 0.45s|3.459|2.66|175.60|5.99|0.64|0.30 (1)|0.50 (1)|735|
|NNLC + 0.32s + angle|3.393|2.78|172.41|6.12|0.46|0.20 (1)|0.40 (1)|735|
|NNLC + 0.32s + friction×0.5|3.383|2.78|171.94|6.15|0.41|0.20 (1)|0.40 (1)|735|
|NNLC + 0.32s + threshold0.6|3.387|2.78|172.12|6.15|0.41|0.20 (1)|0.40 (1)|735|
|NNLC + 0.32s + kick0.03|3.340|2.81|169.81|6.18|0.63|0.20 (1)|0.30 (1)|735|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|3.326|2.80|169.10|6.23|0.63|0.20 (1)|0.30 (1)|735|

### tiny

#### 0-10 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|52.365|1010.16|3628.41|88.74|1.26|NA|NA|151|
|Stock + 0.45s|52.365|1010.16|3628.41|88.74|1.26|NA|NA|151|
|Stock + 0.32s + angle|72.441|1098.93|4720.98|82.12|1.19|NA|NA|151|
|Stock + 0.32s + friction×0.5|52.379|1002.13|3621.10|87.42|1.26|NA|NA|151|
|Stock + 0.32s + threshold0.6|52.379|1002.13|3621.10|87.42|1.26|NA|NA|151|
|Stock + 0.32s + kick0.03|62.615|1129.05|4259.79|90.73|1.19|NA|NA|151|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|68.730|1138.33|4574.83|81.46|1.26|NA|NA|151|
|NNLC + 0.32s|77.637|1370.96|5252.81|94.04|1.85|NA|NA|151|
|NNLC + 0.45s|73.606|1154.63|4834.93|92.72|1.52|NA|NA|151|
|NNLC + 0.32s + angle|75.944|1075.62|4872.81|82.78|1.19|NA|NA|151|
|NNLC + 0.32s + friction×0.5|77.637|1370.96|5252.81|94.04|1.85|NA|NA|151|
|NNLC + 0.32s + threshold0.6|77.637|1370.96|5252.81|94.04|1.85|NA|NA|151|
|NNLC + 0.32s + kick0.03|70.322|1115.44|4631.53|92.72|1.59|NA|NA|151|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|71.050|1131.86|4684.36|81.46|1.26|NA|NA|151|

#### 10-20 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|53.977|925.41|3624.27|90.36|0.66|0.30 (1)|0.10 (1)|197|
|Stock + 0.45s|53.977|925.41|3624.27|90.36|0.66|0.30 (1)|0.10 (1)|197|
|Stock + 0.32s + angle|45.457|823.56|3096.43|49.75|0.81|-0.30 (1)|0.30 (1)|197|
|Stock + 0.32s + friction×0.5|47.711|791.74|3177.29|86.80|0.86|0.00 (1)|-0.20 (1)|197|
|Stock + 0.32s + threshold0.6|50.310|794.72|3310.20|87.82|0.81|0.00 (1)|-0.20 (1)|197|
|Stock + 0.32s + kick0.03|44.957|759.48|3007.32|89.34|0.81|0.00 (1)|-0.20 (1)|197|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|42.568|797.40|2925.77|56.35|0.76|-0.40 (1)|0.20 (1)|197|
|NNLC + 0.32s|48.505|788.49|3213.71|90.36|0.96|-0.30 (1)|0.40 (1)|197|
|NNLC + 0.45s|50.706|817.10|3352.38|92.39|0.91|-0.20 (1)|-0.40 (1)|197|
|NNLC + 0.32s + angle|43.030|798.33|2949.85|53.30|0.81|-0.40 (1)|0.20 (1)|197|
|NNLC + 0.32s + friction×0.5|50.796|855.77|3395.58|91.37|1.02|-0.40 (1)|0.20 (1)|197|
|NNLC + 0.32s + threshold0.6|50.985|853.74|3402.99|90.86|0.96|-0.40 (1)|0.20 (1)|197|
|NNLC + 0.32s + kick0.03|45.754|728.47|3016.18|92.39|0.96|-0.20 (1)|-0.40 (1)|197|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|45.884|815.98|3110.20|60.91|0.81|-0.40 (1)|0.20 (1)|197|

#### 20-35 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|39.942|499.68|2496.80|76.66|0.40|NA|NA|347|
|Stock + 0.45s|39.942|499.68|2496.80|76.66|0.40|NA|NA|347|
|Stock + 0.32s + angle|39.942|499.68|2496.80|76.66|0.40|NA|NA|347|
|Stock + 0.32s + friction×0.5|37.912|501.91|2397.50|75.79|0.43|NA|NA|347|
|Stock + 0.32s + threshold0.6|38.873|511.84|2455.50|77.81|0.43|NA|NA|347|
|Stock + 0.32s + kick0.03|42.714|527.14|2662.83|79.25|0.40|NA|NA|347|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|40.595|515.01|2544.77|77.52|0.43|NA|NA|347|
|NNLC + 0.32s|47.579|565.04|2943.99|81.56|0.46|NA|NA|347|
|NNLC + 0.45s|46.229|576.43|2887.86|80.12|0.58|NA|NA|347|
|NNLC + 0.32s + angle|47.036|567.15|2918.94|81.56|0.46|NA|NA|347|
|NNLC + 0.32s + friction×0.5|46.814|600.92|2941.60|81.27|0.43|NA|NA|347|
|NNLC + 0.32s + threshold0.6|47.568|594.21|2972.61|80.98|0.43|NA|NA|347|
|NNLC + 0.32s + kick0.03|54.287|581.60|3295.96|80.98|0.58|NA|NA|347|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|47.580|563.77|2942.78|81.27|0.58|NA|NA|347|

#### 35-50 mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|44.309|1319.40|3534.83|36.96|1.60|NA|NA|368|
|Stock + 0.45s|44.309|1319.40|3534.83|36.96|1.60|NA|NA|368|
|Stock + 0.32s + angle|44.309|1319.40|3534.83|36.96|1.60|NA|NA|368|
|Stock + 0.32s + friction×0.5|40.322|1319.13|3335.22|31.25|1.79|NA|NA|368|
|Stock + 0.32s + threshold0.6|41.581|1287.36|3366.39|34.78|1.68|NA|NA|368|
|Stock + 0.32s + kick0.03|47.915|1309.71|3705.47|38.32|1.55|NA|NA|368|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|43.379|1316.04|3485.00|34.51|1.60|NA|NA|368|
|NNLC + 0.32s|51.346|1244.01|3811.31|36.68|1.66|NA|NA|368|
|NNLC + 0.45s|50.938|1257.05|3803.93|35.87|1.66|NA|NA|368|
|NNLC + 0.32s + angle|51.346|1244.01|3811.31|36.68|1.66|NA|NA|368|
|NNLC + 0.32s + friction×0.5|51.467|1249.30|3822.64|36.68|1.60|NA|NA|368|
|NNLC + 0.32s + threshold0.6|51.347|1256.99|3824.33|37.77|1.66|NA|NA|368|
|NNLC + 0.32s + kick0.03|48.494|1370.15|3794.87|36.41|1.79|NA|NA|368|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|50.424|1418.73|3939.90|36.68|1.85|NA|NA|368|

#### 50+ mph

|Configuration|L|J|T|Sat %|Flips/s|Entry s (n)|Exit s (n)|Samples|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|Stock + 0.32s|49.309|1247.54|3713.00|37.55|1.47|0.00 (1)|0.00 (1)|735|
|Stock + 0.45s|49.309|1247.54|3713.00|37.55|1.47|0.00 (1)|0.00 (1)|735|
|Stock + 0.32s + angle|49.309|1247.54|3713.00|37.55|1.47|0.00 (1)|0.00 (1)|735|
|Stock + 0.32s + friction×0.5|44.572|1214.87|3443.46|31.56|1.39|0.00 (1)|-0.10 (1)|735|
|Stock + 0.32s + threshold0.6|47.615|1217.78|3598.51|36.33|1.37|0.00 (1)|-0.10 (1)|735|
|Stock + 0.32s + kick0.03|50.632|1304.77|3836.36|38.64|1.40|0.00 (1)|0.10 (1)|735|
|Stock + 0.32s + angle + friction×0.5 + kick0.03|45.846|1221.37|3513.69|34.69|1.29|-0.30 (1)|0.20 (1)|735|
|NNLC + 0.32s|62.686|1292.43|4426.74|20.14|1.51|0.10 (1)|0.10 (1)|735|
|NNLC + 0.45s|61.378|1311.25|4380.15|20.00|1.50|0.10 (1)|0.00 (1)|735|
|NNLC + 0.32s + angle|62.686|1292.43|4426.74|20.14|1.51|0.10 (1)|0.10 (1)|735|
|NNLC + 0.32s + friction×0.5|62.806|1292.41|4432.70|20.54|1.56|0.10 (1)|0.10 (1)|735|
|NNLC + 0.32s + threshold0.6|62.498|1291.37|4416.29|20.54|1.56|0.10 (1)|0.10 (1)|735|
|NNLC + 0.32s + kick0.03|63.509|1354.03|4529.47|22.86|1.47|0.10 (1)|0.00 (1)|735|
|NNLC + 0.32s + angle + friction×0.5 + kick0.03|63.328|1349.14|4515.52|23.13|1.47|0.10 (1)|0.10 (1)|735|

## Evidence and handoff

- Production provenance: closedloop_provenance.json records source hashes and branch/head. This worker only writes new closedloop files and scratch artifacts; concurrent tracked changes/commits belong to other workers. Stock V0 delay invariance is exact in the William sweep (maximum cost difference 0).
- Raw per-window results: closedloop_{provisional,odyssey,public,tiny}.json; commands/prediction traces in matching *_traces.npz. frozen worker plant: closedloop_{odyssey,public}_plant_snapshot.json.
- Full converted logs: closedloop_data/*.csv and *.npz. Controller/metric checks: closedloop_checks.json. Replay validation: closedloop_validation.json.
- Reproduction scripts: tools/odyssey_sim/closedloop/{data,controller,rollout,worker_plant,check,validate,report}.py and README.md. No existing tracked files modified by this worker; no commit or push.
- Self-review corrected asynchronous Params initialization and checked the V0 source baseline friction threshold (0.3). All results shown use blocking writes and source-correct V0 baseline, with final production OdysseySteer/KP-cap wiring. No additional subagents were spawned because this task explicitly assigned a single research worker.
- Exact next validation: run matched logged command sequences through the frozen plant on independent segments; quantify 1/3/5-second wheel-angle-derived acceleration error per speed band, then rerun this harness over multiple windows/seeds and bounded plant parameter perturbations before selecting a road-test setting.
