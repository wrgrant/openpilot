# Odyssey simulator calibration, pass 2

**Result: the body channel and a better plant are validated; the full simulator is not calibrated and cannot rank controllers.** Keep the current van configuration. No controller or add-on is promoted by this analysis.

The van provides a usable motion signal after all: speed × calibrated yaw rate. The new plant predicts ten seconds of steering/body motion to roughly 0.08–0.10 m/s² RMS on unseen routes. That accuracy does not make it a trustworthy lane simulator: small acceleration/road errors accumulate into large lane-position errors, and the real controller still produces different reversal rates in simulation. A lower simulated cost is not evidence of better road steering.

## Body channel and frames

`controlsd.py` publishes calibrated angular velocity and orientation into carControl. `carControl.angularVelocity[2] × carState.vEgo` is positive with openpilot lateral acceleration and negative with Honda steering angle and torqueState.output. Roll/pitch angular-velocity axes do not show that relationship. This is yaw/curvature acceleration, not a gravity-bearing accelerometer reading; rapid sideslip transients and longitudinal-speed approximations remain limitations. SensorEvents fallback was unnecessary.

|Held-out route|Steady yaw/steering correlation|Roll std (rad)|Body samples finite|Raw-wheel steady-fit RMSE (m/s²)|
|---|---:|---:|---:|---:|
|0000019c|0.9960|0.0264|99.84%|0.0735|
|000001aa|0.9983|0.0219|99.76%|0.0528|
|000001ab|0.9973|0.0237|99.85%|0.0548|

Steady means active, no driver override, |driver torque| <500, speed >5 m/s, no blinkers, |steering rate| <1 deg/s, and |logged acceleration| >0.15 m/s². Supplemental channels are timestamp-aligned as-of with 150 ms maximum age (500 ms for vehicleParameters); no interpolation across gaps. The new extractor orders segments numerically; alignment asserts sorted timestamps. Original extraction caches are read-only.

A training-route-only raw wheel fit uses `wheel = -radians(angle-offset) × v²/(steerRatio × 2.9)` and `body = wheel/(1 + K×v²) + R×roll + bias`: K=0.0006441 s²/m², R=2.081 m/s²/rad, bias=-0.0107 m/s². This captures speed-dependent understeer, with decreasing wheel-to-body gain at speed. Roll and road curvature are correlated, so R is an empirical correction rather than a causal crown measurement. Also, vehicleParameters is learned from motion: agreement with torqueState actual is not independent validation of every VehicleModel parameter. The raw wheel/yaw fit provides the additional check.

## Plant v2

Files: `scratchpad/calib/plant_v2.json`, `identification.json`, `body_fit.json`, and `checks.json`; API `tools/odyssey_sim/calib/plant.py`.

Command → steering-equivalent acceleration uses deadband 0.028, speed-interpolated gain/friction, first-order lag, measured roll, and bias. Steering-equivalent acceleration is the VehicleModel quantity observed by V0, rather than raw wheel angle. The yaw-body stage is identified separately from logged wheel-equivalent acceleration and then validated in combined free run. The API adds the Honda command slew limit (3 normalized units/s). No observations reset it during a prediction.

Gain at 10/20/30 m/s: [1.1127961718471011, 2.083082915437098, 2.194089452395769]; friction in acceleration units: [0.16977866300563133, 0.1857813043664438, 0.07140966657351758]; lag 1.061 s; roll gain 7.802 m/s²/rad; bias -0.0087 m/s². Speed-dependent friction is retained. These ordinary closed-loop logs do not identify separate static and kinetic friction reliably; no extra breakaway parameter is claimed.

Body-stage gain at 10/20/30 m/s: [1.0179062720182948, 0.9901541506727294, 0.9880435758918191]. Its lag hits the 5 ms lower bound. At 10 Hz this means no additional lag is resolved; it does not establish a physical 5 ms body lag. The body-stage roll term is only a residual correction; measured roll already enters the steering-equivalent state.

Training: 174, 18c, 19b, 1a3, 1a7. Held out throughout plant/planner fitting: 19c, 1aa, 1ab. Nonoverlapping 10 s predictions, 2 s continuously eligible context, one segment, no gaps >160 ms; initial motion and applied command observed once. Predictions use logged commands/speed/roll, with no subsequent motion feedback. Below 5 m/s remains outside the fit.

|Held-out route|10 s windows|Steering RMSE, fit / scalar API|Body RMSE, fit / scalar API|Body given logged steering RMSE|
|---|---:|---:|---:|---:|
|0000019c|96|0.0910 / 0.0912|0.0980 / 0.0970|0.0624|
|000001aa|45|0.0772 / 0.0778|0.0841 / 0.0832|0.0372|
|000001ab|52|0.0772 / 0.0772|0.0878 / 0.0873|0.0437|

All RMSEs are m/s², pooled over every forecast sample after initialization, not just the endpoint. Scalar API is independently exercised at 100 Hz and includes slew; the optimizer uses a vectorized 10 Hz approximation. The differences are small on held-out data. The finite/bounded/sign grid passes at 5/10/20/30/35 m/s and commands −1/−.04/0/.04/1.

## End-to-end calibration gate

The real sunnypilot V0 class and its logged NNLC mode run at 100 Hz. All three held-out route metadata records have NNLC enabled and no NnlcAB setting. The updated adapter uses per-frame logged torque tuning, learned lateralDelay, vehicleParameters, and calibrated pitch, plus the applied-command slew feedback. This replaces the previous fixed factor/friction/offset/delay and missing pitch. Feedback includes the Honda 0.1-degree wheel-angle quantizer; angle-rate feedback remains continuous. Simulated tracking cost uses the actual VehicleModel measurement from that quantized feedback, matching torqueState semantics. Logged delay medians are approximately 0.349/0.330/0.322 s for 19c/1aa/1ab; torque factor also differs materially across routes.

The logged-plan mode replays the recorded request/future model trajectory and closes the steering feedback loop. The lane-loop mode instead computes request = road + KP×offset + KD×sideways speed and integrates road minus body acceleration. Both use real V0/NNLC. NNLC future arrays remain recorded in lane-loop mode: a newly simulated lane request does not have a corresponding regenerated modelV2 future. That mismatch is an explicit failure of the planner surrogate, not a validated simulation of modeld.

Refitting the lane law on training data gives KP/KD/bias = [0.17408258091480472, 0.571826615568362, 0.010920143069607653], R²=0.384. The calibration table uses this training-only refit, including its bias. We did not tune against held-out wander to force a passing answer.

Tolerance: every metric within 30% on each scored window. To avoid division by zero, relative-error denominators have floors of 0.01 cost units, 0.1 reversals/s, 1 percentage point saturation, and 1 cm wander. Passing requires lane-wander coverage as well as cost/reversals/saturation. Cost is 100×mean((torqueState desired − torqueState actual)²); no jerk penalty is mixed into this tracking cost. Reversals use native 100 Hz torqueState output versus native simulated controller output, >0.01 crossing amplitude, and adjacent eligible samples. Saturation is |output|≥0.99 rather than the delayed torqueState saturated flag.

Windows are continuously autonomous, no driver override or blinker, >5 m/s, reliable lane lines >80%, 60 s with 10 s warmup. Route 1ab has no eligible 60 s window, so it uses 30 s windows (20 s scored). Wander requires at least 40 uninterrupted scored seconds in the speed band and uses the wander meter’s detrending/7–20 s FFT band at 20 Hz. A missing wander score fails the gate. There is no below-5-m/s claim; bands with no eligible window are unvalidated.

Each cell is **recorded / simulated**, duration-weighted across windows. Individual failures are retained in evaluation.json. “Fail” means one or more constituent windows misses the full gate, even when an aggregate metric is close.

|Route|mph band|Mode|Windows|Tracking cost|Reversals/s|Saturation %|Wander cm|Gate|
|---|---|---|---:|---:|---:|---:|---:|---|
|0000019c|50-200|lane_loop|11|0.444 / 0.325|1.455 / 0.742|0.000 / 0.000|7.042 / 11.555|FAIL|
|0000019c|50-200|logged_plan|11|0.444 / 0.267|1.455 / 0.956|0.000 / 0.000|7.042 / 42.891|FAIL|
|000001aa|35-50|lane_loop|1|0.212 / 0.276|1.499 / 0.248|0.000 / 0.000|unavailable|FAIL|
|000001aa|35-50|logged_plan|1|0.212 / 0.195|1.499 / 0.331|0.000 / 0.000|unavailable|FAIL|
|000001aa|50-200|lane_loop|2|0.477 / 0.300|1.856 / 0.728|0.000 / 0.000|11.461 / 10.384|FAIL|
|000001aa|50-200|logged_plan|2|0.477 / 0.258|1.856 / 0.944|0.000 / 0.000|11.461 / 46.037|FAIL|
|000001ab|35-50|lane_loop|2|0.421 / 0.368|1.940 / 0.687|0.000 / 0.000|unavailable|FAIL|
|000001ab|35-50|logged_plan|2|0.421 / 0.336|1.940 / 1.018|0.000 / 0.000|unavailable|FAIL|
|000001ab|50-200|lane_loop|4|0.351 / 0.248|1.363 / 1.022|0.000 / 0.000|unavailable|FAIL|
|000001ab|50-200|logged_plan|4|0.351 / 0.237|1.363 / 1.492|0.000 / 0.000|unavailable|FAIL|

Full gate: 0/40 route/window/band/mode rows pass. Ranking authorized: False.

## Lane sign correction and remaining calibration failure

The legacy plant cache negates both lane-centre curvature and lane-centre offset. Its cached road acceleration correlates −0.94 to −0.95 with the independently validated yaw channel. Negating cached road and offset makes road curvature positive with body acceleration and offset positive for the lane error used by request = road + KP×offset + KD×sideways speed. This correction lives in calib.evaluate.records; the original caches and old simulators remain untouched. It removes a major false road forcing, improves the planner fit, and sharply reduces the closure residual, but does not make all metrics agree.

A direct closure audit checks `offset_second_derivative = road_acceleration − body_acceleration` in the 7–20 s band. This must hold before any controller is added. It does not. The camera road estimate is a quadratic lane fit over <50 m, not independently surveyed road curvature. Camera lane estimates, horizon/lookahead and offset changes need a measurement/geometry model; double integration treats their mismatch as physical lane drift. Roll/crown residuals and sideslip are additional possible contributors. The audit cannot separate these causes, but it rules out treating this road/offset pair as a consistent external forcing.

|Held-out route|Closure windows|Median acceleration closure residual m/s²|Median observed lane second-derivative RMS m/s²|Equivalent wander error cm|
|---|---:|---:|---:|---:|
|0000019c|11|0.0581|0.0382|33.4|
|000001aa|2|0.0653|0.0436|37.3|
|000001ab|0|unavailable|unavailable|unavailable|

The closure audit uses complete 60 s windows and spectral differentiation; boundary/periodicity effects remain. Even in the logged-plan mode, integrating the imperfect road with simulated body motion produces large offset errors. It is a diagnostic of the lane reconstruction, not a meaningful prediction of leaving the lane.

Coverage: no 10–20 or 20–35 mph band has a qualifying ≥30 s lane-visible autonomous window in any held-out route. 19c also has no 35–50 mph window. These cells are **UNVALIDATED / FAIL**, rather than silently passed. Low-speed tracking could be tested separately with shorter fragments, but that cannot meet the requested wander gate.

## Candidate diagnostics and uncertainty

**No stock-vs-NNLC/add-on ranking is issued.** The requested prerequisite fails. Both van-model artifacts exist and were evaluated on the same plant and logged-plan windows. The following are simulator diagnostics, with no ordering or deployment recommendation. 19c was part of the candidate author’s training set, so candidate generalization is assessed only on 1aa/1ab here. V1 uses the runtime bounded 18-input evaluator; V2 uses its author’s 26-input history adapter. These runs satisfy evaluation of available artifacts, not qualification for the van.

|Route / band|Artifact|Sim cost, window mean [bootstrap 95% interval]|Sim reversals/s, mean [95% interval]|
|---|---|---:|---:|
|000001aa / 35-50|original NNLC|0.195 [one window]|0.331 [one window]|
|000001aa / 50-200|original NNLC|0.267 [0.200, 0.334]|0.891 [0.501, 1.280]|
|000001ab / 35-50|original NNLC|0.335 [0.306, 0.364]|1.011 [0.622, 1.400]|
|000001ab / 50-200|original NNLC|0.237 [0.157, 0.301]|1.494 [1.306, 1.637]|
|000001aa / 35-50|William V1|0.054 [one window]|2.975 [one window]|
|000001aa / 50-200|William V1|0.063 [0.055, 0.071]|2.271 [1.662, 2.880]|
|000001ab / 35-50|William V1|0.089 [0.067, 0.111]|2.338 [2.176, 2.500]|
|000001ab / 50-200|William V1|0.064 [0.051, 0.081]|2.514 [1.775, 3.192]|
|000001aa / 35-50|William V2|0.049 [one window]|3.471 [one window]|
|000001aa / 50-200|William V2|0.077 [0.064, 0.091]|2.307 [1.794, 2.820]|
|000001ab / 35-50|William V2|0.080 [0.058, 0.101]|2.570 [2.550, 2.591]|
|000001ab / 50-200|William V2|0.066 [0.054, 0.082]|2.961 [2.294, 3.629]|

Intervals resample whole nonoverlapping windows, 1,000 bootstrap draws, fixed seed 20261109. Few windows make these intervals weak; “one window” means no defensible interval. The plant/controller run is deterministic, so repeated identical seeds add no uncertainty information. These intervals exclude plant identification uncertainty, candidate training uncertainty and planner mismatch; the latter dominates and prevents rankings. Add-ons were not swept after the calibration gate failed.

## One-command hook and reproduction

From `~/Code/odyssey-pilot`:

```sh
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate

PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --model "$PWD/openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY_WILLIAM_V1.json" --output evaluation_v1.json

PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --controller tools.odyssey_sim.calib.controller:candidate_v2 --output evaluation_v2.json
```

An 18-input candidate uses --model. A different controller/input contract uses --controller module:factory; the factory receives (metadata, Config), returns an object with update(target_accel, measured_accel, state, future_plan), and returns normalized acceleration-positive requested torque. The adapter also exposes measured_lataccel for scoring its actual feedback. Every candidate retains the same plant, routes, masks and metrics. Use a distinct --output to preserve the baseline. The JSON always includes ranking_authorized; candidate improvement never overrides the calibration gate.

To rebuild: run calib.extract with repo Python; calib.identify and calib.body_fit with the supplied wander/venv scipy Python and PYTHONPATH=$PWD; then calib.check, calib.evaluate, candidate commands, and calib.report with repo Python. Ruff and compileall pass. All entry points have main guards; no multiprocessing is needed.

## Limits and exact next step

The command-to-steering plant is an aggregate model fitted from feedback-controlled driving, not a persistently excited system-identification experiment. Parameter trade-offs remain, especially gain/friction/lag/roll. The public Odyssey data is not used to override this van’s fitted parameters. Manual routes a4/a5 are supplemented but excluded from autonomous fitting. Driver takeovers, Honda driver-torque safety limits, realistic EPS angle-rate sensing, camera geometry and a regenerated future model/planner remain incomplete. The simulations use selected continuous lane-visible windows, not the full route population; low-speed parking is unvalidated.

**Recommended next step:** reconstruct lane geometry at the vehicle origin (offset, heading, curvature and confidence) from modelV2, fit its observation dynamics and test the body/road/offset closure on training routes before touching controller rankings. Require the same closure on 19c/1aa/1ab and enough ≥45–60 s autonomous coverage per speed band. Then regenerate NNLC future trajectories consistently with simulated lane feedback and pass the current calibration gate. Until then use replay to compare output behavior and this plant to reject fragile candidates, not choose road settings.

## Workspace receipt

Checkout `/Users/williamgrant/Code/odyssey-pilot`, branch `odyssey-sim`. Only new files under tools/odyssey_sim/calib/ and requested scratch artifacts were written. No commits, staging, pushes, branch switches or device access. Concurrent worker changes were left alone. Provenance/source hashes are in scratchpad/calib/provenance.json.
