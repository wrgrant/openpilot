# Odyssey simulator calibration pass 3

## Plain-language result

The lane loop is **not calibrated well enough to rank controllers**. Reconstructing the lane at the vehicle origin and combining its heading change with yaw improves the road estimate, but the remaining error is comparable to the wander William wants to remove. A camera-corrected observer looks accurate while it receives camera updates and drifts badly when those updates are withheld. Using that apparent accuracy to approve a controller would hide the error.

The torque-reversal discrepancy has a concrete partial explanation: keeping the real controller inputs at 100 Hz and preserving its recorded slow integral reproduces the observed reversals closely. Holding all inputs at 10 Hz loses substantial reversal activity. Delay jitter and torque-parameter updates do not explain it in the sampled windows. Arbitrary sensor-noise injection is not justified.

No stock/NNLC/van-model/smoothing ranking or road-setting recommendation is issued. No device, commit, stage, push, branch switch or older worker source/cache was touched.

## Data and validation contract

Train: 00000174, 0000018c, 0000019b, 000001a3, 000001a7. Held out: 0000019c, 000001aa, 000001ab. All eight routes were extracted from file-path LogReader inputs. Full time-indexed model position, velocity, orientation, orientationRate, acceleration; road edges; lane confidence/uncertainty; six origin fits; extrinsics; frame timing; native control/sensor/tuning channels are cached in `/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/calib3`.

Origin variants are quadratic/cubic polynomials over 15/25/50 m. Curvature is 2*c2/(1+slope²)^(3/2), heading atan(slope), offset intercept at x=0. The model source applies extrinsics in its image warp before producing calibrated-frame geometry (`openpilot/selfdrive/modeld/modeld.py`, `fill_model_msg.py`). A second pitch/yaw transform would duplicate that calibration. Camera height is not independently identifiable from these already-inferred metric outputs; no ad hoc height correction was fitted.

The spectral audit matches pass 2: detrend each channel, retain 1/20–1/7 Hz Fourier bins, differentiate spectrally, compare offset second derivative with road minus body acceleration, and convert residual acceleration back to implied wander displacement. Windows and fitted origin offsets differ from pass 2, so differences are not a pure causal estimate of improvement. The separate padded Butterworth/interior-derivative audit is a boundary-method sensitivity check, not the headline result.

Strict geometry audit requires every sample autonomous, no override/blinker, >5 m/s and both inner-lane probabilities >0.5. It is stricter than the behavior evaluator’s >80% lane-visibility window rule. For 1aa/1ab, 30 s geometry windows are reported as diagnostics, not sufficient 7–20 s wander validation. No confident continuous 60 s windows exist there under this mask.

## Generation log

|Generation|Experiment and result|Disposition|
|---|---|---|
|G0|Read pass-2 measured facts/code and 0/40 behavior gate; retained route split, plant v2 and real V0/NNLC.|Baseline reproduced from saved artifacts.|
|G1|Six origin geometries × five temporal shifts. Training selects quadratic 25 m, −0.3 s spatial-curvature shift. Native frame EOF-to-message median is 31–42 ms across routes.|Origin curvature alone still fails closure. Fitted −0.3 s is not a measured camera latency; it can absorb preview/geometry/model effects.|
|G2|Causal [offset, offset velocity, curvature bias] observer. Twelve train-only settings; hide offset/heading for final 10 s.|Reject as independent simulator; good posterior agreement depends on assimilation.|
|G3|World-road turning from temporal lane heading plus recorded yaw, optionally subtract model lateral velocity. Butterworth sweep selects cubic 15 m, +0.3 s and model-vy subtraction; spectral sweep selects quadratic 25 m, zero shift, no vy subtraction.|Method-sensitive parameters; heading transport helps but does not close.|
|G4a|Native-input real-controller replay initialized only at window start; 10 s warmup.|Slow integral mismatch can shift zero crossings despite high command correlation. Preserved in reversals_free_integrator.json; not evidence that sensors are wrong.|
|G4b|Forensic replay clamps integral to the recorded value every frame, then ablates bandwidth/futures/tuning/delay on identical recorded motion.|Explains a substantial bandwidth component. This intervention is not a free-running controller simulation.|
|G5|Heading-transport road, real controllers, local NNLC future perturbation; same metric/tolerance contract as pass 2.|0/40 behavior rows pass.|
|G6|Train-only linear fusion of spatial road-minus-body, d(v*lane-heading)/dt, d(model-vy)/dt. Coefficients [0.37809254969012607, 0.2982681063672973, 0.030119912967580476]. Fits training 30 s windows only.|Reduces error, still fails independent closure. No held-out tuning.|
|G7|Fusion road plus recorded initial controller integral, real controller loop, simulated steering/body feedback, local consistent NNLC future correction.|0/40 behavior rows pass; rankings remain unauthorized.|

The NNLC future correction adds the simulated request change to model acceleration.y with a 1.5 s exponential horizon fade; recorded future roll/pitch remain exogenous. This removes the exact unchanged-future mismatch from pass 2 but is an explicit approximate planner surrogate, not a validated regeneration of modelV2. Full position/velocity/orientation arrays were preserved for stronger future reconstruction; they were not silently treated as independent road ground truth.

## Closure: matched spectral audit, 60 s windows

|Route / split|Windows|Spatial residual m/s²|Heading-transport residual m/s²|Lane second-derivative RMS m/s²|Transport implied wander error cm|Observed wander cm|
|---|---:|---:|---:|---:|---:|---:|
|00000174 / train|0|unavailable|unavailable|unavailable|unavailable|unavailable|
|0000018c / train|6|0.0339|0.0331|0.0317|11.1502|7.8343|
|0000019b / train|0|unavailable|unavailable|unavailable|unavailable|unavailable|
|000001a3 / train|1|0.0446|0.0181|0.0433|4.8580|10.9102|
|000001a7 / train|1|0.0331|0.0189|0.0367|4.0231|7.8436|
|0000019c / heldout|15|0.0350|0.0210|0.0271|7.8418|8.9020|
|000001aa / heldout|0|unavailable|unavailable|unavailable|unavailable|unavailable|
|000001ab / heldout|0|unavailable|unavailable|unavailable|unavailable|unavailable|

## Fusion closure, 30 s diagnostics

|Route|Windows|Acceleration residual m/s²|Lane second-derivative RMS m/s²|Implied wander error cm|Observed wander cm|
|---|---:|---:|---:|---:|---:|
|00000174|0|unavailable|unavailable|unavailable|unavailable|
|0000018c|26|0.0144|0.0260|4.9096|6.2185|
|0000019b|0|unavailable|unavailable|unavailable|unavailable|
|000001a3|7|0.0175|0.0363|5.4285|8.2720|
|000001a7|5|0.0227|0.0424|6.1069|9.6913|
|0000019c|42|0.0192|0.0304|5.9650|6.9163|
|000001aa|4|0.0104|0.0221|3.8302|6.0291|
|000001ab|3|0.0183|0.0281|4.2825|5.5541|

Fusion reduces error without using offset as a forcing input during held-out prediction. Its residual is still material relative to observed wander. Spatial and heading/model-motion observations are correlated model outputs, not independent sensors; fitting a fusion does not establish road ground truth. The variable-speed correction a/v × offset_rate was included in the interior audit; spectral tables retain the pass-2 road-minus-body definition for comparability.

## Observer dropout validation

|Route|60 s windows|Median camera-assisted posterior offset RMSE m|Median final-10-second prediction RMSE m|
|---|---:|---:|---:|
|00000174|0|unavailable|unavailable|
|0000018c|6|0.0241|0.4173|
|0000019b|0|unavailable|unavailable|
|000001a3|1|0.0590|0.2057|
|000001a7|1|0.0253|0.4039|
|0000019c|15|0.0331|0.5976|
|000001aa|0|unavailable|unavailable|
|000001ab|0|unavailable|unavailable|

Training-selected settings: `{'camera_sigma': 0.3, 'heading_sigma': 0.3, 'bias_walk': 0.05, 'bias_retention': 0.99}`. No offset or heading update is allowed in the scored final 10 seconds; spatial camera curvature continues as road input. Constant bias prediction is not reliable enough to support free-running wander simulation. This dropout test is deliberately stricter than posterior consistency and reveals why small observer residuals alone are insufficient.

## Behavior calibration gate (G7)

Same 30% per-window tolerances, metric floors, native 100 Hz reversal definition (>0.01 adjacent crossing), saturation definition, warmup and wander requirements as pass 2. A missing wander metric fails. Values below are duration-weighted recorded / simulated aggregates; a group fails if any constituent row fails.

|Route|mph|Mode|Rows|Cost|Reversals/s|Saturation %|Wander cm|Pass rows|
|---|---|---|---:|---:|---:|---:|---:|---:|
|0000019c|50-200|lane_loop|11|0.444 / 0.430|1.455 / 1.553|0.000 / 0.000|7.077 / 48.469|0|
|0000019c|50-200|logged_plan|11|0.444 / 0.266|1.455 / 0.955|0.000 / 0.000|7.077 / 24.324|0|
|000001aa|35-50|lane_loop|1|0.212 / 0.174|1.499 / 1.405|0.000 / 0.000|unavailable|0|
|000001aa|35-50|logged_plan|1|0.212 / 0.196|1.499 / 0.496|0.000 / 0.000|unavailable|0|
|000001aa|50-200|lane_loop|2|0.477 / 0.333|1.856 / 1.627|0.000 / 0.000|11.552 / 122.263|0|
|000001aa|50-200|logged_plan|2|0.477 / 0.258|1.856 / 0.944|0.000 / 0.000|11.552 / 31.970|0|
|000001ab|35-50|lane_loop|2|0.421 / 0.369|1.940 / 0.992|0.000 / 0.000|unavailable|0|
|000001ab|35-50|logged_plan|2|0.421 / 0.334|1.940 / 1.043|0.000 / 0.000|unavailable|0|
|000001ab|50-200|lane_loop|4|0.351 / 0.264|1.363 / 2.016|0.000 / 0.000|unavailable|0|
|000001ab|50-200|logged_plan|4|0.351 / 0.236|1.363 / 1.478|0.000 / 0.000|unavailable|0|

Full G7 gate: **0/40 rows pass. ranking_authorized=false.** No candidate rankings are justified. Cost/saturation agreement in some rows does not cure closure, reversal or missing-wander failures.

## Reversal-gap explanation

Eight representative windows (three 19c, three 1ab, two 1aa), identical recorded motion for all ablations, scored after 10 s warmup. The native replay uses per-frame recorded integral as an explicit forensic control. Means below weight each window equally; exact window rows are in reversals.json.

|Input mode|Mean reversals/s|Median command RMSE vs recorded|
|---|---:|---:|
|Recorded|1.6011|—|
|native|1.5747|0.00262|
|sensors_10hz|1.4982|0.01292|
|target_10hz|1.4395|0.03242|
|all_10hz|1.1283|0.03404|
|tuning_constant|1.5747|0.00262|
|delay_constant|1.5747|0.00262|
|future_constant|1.3339|0.01955|

Interpretation: holding target and sensed vehicle motion at 10 Hz removes high-frequency crossings; holding future arrays constant also changes them. Logged tuning is essentially constant on these short windows and freezing delay changes neither mean reversal rate nor the conclusion. Real steering angle already has 0.1° quantization; pass 2 quantizes synthetic feedback, so the mere existence of quantization is not the missing mechanism. The plant produces smooth feedback and loses real sub-frame disturbances; their independent contribution is not yet identified. No arbitrary sensor-noise scale was tuned to force agreement. Raw crossing count is also sensitive to slow integral bias and is not a perceptibility metric; use the separate perception worker for that question.

## Low-speed coverage

Counts below require the entire 10 s fragment inside the band and strict continuous high-confidence autonomy. Short fragments can calibrate fast steering behavior, not a 7–20 s wander oscillation. Values under ~11.2 mph remain excluded by the inherited >5 m/s plant gate.

|Held-out route|10–20 mph 10 s fragments|20–35 mph 10 s fragments|10–35 mph 60 s fragments|
|---|---:|---:|---:|
|0000019c|0|0|0|
|000001aa|0|3|0|
|000001ab|0|0|0|

No 10–35 mph closed-loop/wander qualification follows from these fragments. Coverage for all training/held-out routes and 10/20/30/60 s durations is retained in spectral.json.

## Reuse, checks and exact next step

The wander worker can import `tools.odyssey_sim.calib3.observer.LaneObserver`, `geometry.origin`, and `lane.records`. Reuse the APIs and artifacts, but do not assume the observer is validated. One-command evaluator: `PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib3.evaluate --output evaluation_g7.json`; the original `--model` and `--controller module:factory` contracts remain. See calib3/README.md for full extraction, fit and validation commands. scipy is required only by offline fitting/auditing, using the supplied wander venv; step APIs and evaluator use the repo Python.

Validation: five analytic/observer/future tests pass; ruff check passes; compileall passes; all eight extraction artifacts exist; both G5 and G7 full held-out evaluations completed. Geometry and bias-state observer have physical analytic checks, including a blind prediction test that prevents hiding drift behind camera assimilation.

**Recommended next execution slice:** fit a causal joint offset/heading observation model with explicit slowly varying camera heading bias on the training routes, and validate offset prediction in withheld 10 s blocks on 1aa/1ab as well as 19c. Compare against the present fusion and bias observer with identical windows. Require an independent displacement residual below 30% of observed 7–20 s wander (with a 1 cm floor) before using any lane ranking. Preserve the actual 100 Hz controller-input bandwidth and initialize the slow integral from the recorded common starting state. A posterior camera fit, an offset-derived road acceleration, or a tuned stochastic noise injection cannot substitute for that withheld-observation check. The current fusion residual means further controller sweeps would mainly optimize the simulator error.

## Workspace receipt

Checkout `/Users/williamgrant/Code/odyssey-pilot`, branch `odyssey-sim`. Source writes: only `tools/odyssey_sim/calib3/`. Research writes: requested scratch directory. Existing concurrent worker changes were not staged, edited or reverted. No external actions or device access. All delivered code and intermediate numerical artifacts remain uncommitted for the lead. Source hashes are in calib3/provenance.json.
