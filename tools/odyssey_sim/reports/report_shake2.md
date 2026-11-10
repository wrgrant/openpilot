# Shake research, pass 2: the simulator still fails

**No: it still does not reproduce the US-169 shake well enough to tune your steering.** Separating the measured links removes the fictitious quarter-second motor delay, but exposes a larger problem: the fitted column and tyre model does not transfer reliably between routes or controllers. Adding measured noise estimates and a stock-road torque residual does produce frequent stock-command crossings; it also makes the S5 simulation much too large. This is a rejected model, not a quiet-bump design tool.

On development windows S5 body RMS is **18.39 times the road**, and its command crosses zero **0.44 times as often**. The closed-loop bundle passes for **0 of 3 controllers**. Wheel-to-yaw alone passes 100 of 298 coherent held-out/check frequency bins; it already fails before the EPS is involved. More search cannot turn that into a validated physical explanation.

Your reported US-169 motion remains the reference: ±0.25 m/s² (±20 N on 80 kg) typical and ±0.50–0.625 m/s² (±40–50 N), about 1.3 oscillations per second and 2.7 direction changes per second. The new tables below use RMS, which is a different statistic. Your 75 kg force is 0.9375 times the quoted 80 kg force.

**Recommendation:** keep S5 off. Use the existing held-pulse suite at steady 40 and 55–60 mph to identify small-signal breakaway and reversal response independently of lane keeping, then check the frozen result on a later drive. The minimum two-speed suite is about **4 minutes 36 seconds**, excluding acceleration, traffic interruptions and aborts. No extra sensor or closed course is required by this specification. It is a data-collection proposal; nothing was enabled or sent to the van.

## Scope, split and reproducibility

Checkout `odyssey-sim`, HEAD `ee0215e6b611b9d67197f2db8f013369384a891d`. No branch switch, worktree, commit, push or device access. All authored analysis and results stay in `tools/odyssey_sim/shake/`; caches and Params stay in the assigned `scratchpad/shake/`. This explicitly requested report is the only folder-rule exception. Pass-1 artifacts are retained. Runtime start/end hashes and full dirty status are in `loop_manifest_start2.json` / `loop_manifest_end2.json`. Concurrently changed during the campaign: **openpilot/sunnypilot/selfdrive/controls/lib/odyssey_ab.py**. Imported Python objects stay loaded within the process; this does not recreate the historical on-van controller. Exact imported source bytes were not archived before import, so the boundary hashes are a provenance limitation, not proof of a frozen source checkout.

Tyre fitting trains on routes through 1ac excluding entire 1a5, 19c and 1ad routes, which serve as link checks; 1a4 hand driving enters training and 1a5 hand driving enters checking. 1ae is only an exposed check. EPS trains on 1a7/1aa/1ab/1ac, checks on 1ad/1ae. Column parameter fitting uses the same training pool, selects friction family on 1ad, and checks 1ae. Thus 1ad is development/model-selection data for the assembled plant, not an untouched whole-chain holdout. **Only the next highway drive is untouched proof.** No logs after 1ae were opened.

Continuous, nonoverlapping 20 s records are resampled at 100 samples per second; gaps, segments, speed bands and eligibility split records. Tyre fitting uses all eligible steering motion, including the two manual routes; automated records require lateral active. Driver-press flags on straights are retained. Curves with true hands can still contaminate the fits. Whole-chain windows retain pass 1’s 180 s model/lane arms and skip their first 30 s. Score after 5 s warmup, trim the final 1 s, and apply the same 0.3–3 oscillations-per-second bandpass to road and simulation. The real V0/NNLC/response shaper/OdysseySteer runs, with scratch-only Params, learned calibration and logged future plans. The LaC now receives learned delay plus `LAT_SMOOTH_SECONDS`; NNLC receives learned delay. This also differs from pass 1, so noise alone is not the only changed element.

## Link 1: measured wheel angle to measured yaw

The four states are lateral velocity, yaw rate, front tyre force and rear tyre force. Front/rear forces relax toward linear slip-angle forces with time constant relaxation-length/speed. Fit front and rear axle stiffness, yaw inertia and common relaxation length separately in 25–45 and >45 mph bands, with fixed inherited mass 2163.35 kg, wheelbase 3.00 m, CG distances 1.35/1.65 m and steering ratio 14.35. Mass includes the inherited nominal 80 kg load; actual occupants, fuel and tyre conditions remain unknown. Roll has no feedback path into yaw. No fitted scale is added to the measured angle or yaw.

Each band uses differential evolution (40-member population, up to 90 generations), then bounded least squares. Forty route-cluster bootstrap fits give conditional percentile 95% intervals. Bounds: stiffness 20–600 kN/rad, yaw inertia 1000–16000 kg·m², relaxation length 0.02–6 m. Fits use only 0.3–3 oscillations-per-second bins with measured angle/yaw coherence ≥0.7. Five-second Hann records with half overlap give 0.2 oscillations-per-second spacing: the first scored bin is 0.4, not an invented exact 0.3 bin. Overlapping spectral records are not independent route repetitions.

| mph | Parameter | Estimate | Conditional 95% interval |
| --- | --- | --- | --- |
| 25–45 | Cf_N_rad | 1.98e+05 | 1.42e+05–2.6e+05 |
| 25–45 | Cr_N_rad | 5.51e+05 | 2.84e+05–6e+05 |
| 25–45 | Iz_kgm2 | 6.29e+03 | 4.42e+03–7.15e+03 |
| 25–45 | relaxation_m | 1.81 | 1.65–1.99 |
| >45 | Cf_N_rad | 1.32e+05 | 8.24e+04–1.64e+05 |
| >45 | Cr_N_rad | 1.65e+05 | 8.63e+04–2.37e+05 |
| >45 | Iz_kgm2 | 3.22e+03 | 1.81e+03–4.02e+03 |
| >45 | relaxation_m | 1.88 | 1.51–2.69 |


| mph | Route | Coherent bins | Pass ±15%, ±15° | Uncovered |
| --- | --- | --- | --- | --- |
| 25–45 | 0000019c | 5 | 3 | 23 |
| 25–45 | 000001a5 | 11 | 3 | 31 |
| 25–45 | 000001ad | 0 | 0 | 0 |
| 25–45 | 000001ae | 73 | 27 | 39 |
| >45 | 0000019c | 82 | 14 | 744 |
| >45 | 000001a5 | 33 | 3 | 51 |
| >45 | 000001ad | 94 | 50 | 102 |
| >45 | 000001ae | 0 | 0 | 0 |


Low-band rear stiffness has a bootstrap interval reaching the 600 kN/rad search ceiling: it is weakly identified/bound-limited, not a precise stiffness measurement. The implied understeer coefficient m/L × (b/Cf − a/Cr) is about 0.00424 versus 0.00313 s²/m in the two bands. The across-speed estimates are not physically consistent enough: rear stiffness changes by more than a factor of three and yaw inertia nearly halves. Route bootstrap intervals are conditional on this inadequate linear model, not a physical confidence certificate. Model discrepancy, calibration/timing and tyre nonlinearity remain mixed. The measured yaw is the validated carControl z rate, not VSA lateral acceleration or torqueState’s steering-derived acceleration. Route window counts, every scored/uncovered bin, gains and phases are retained in `yaw2.json`.

## Link 2: CAN command to motor magnitude

A saturating deadband map with first-order magnitude lag and linear low-speed fade is fitted independently of wheel/yaw. Use the actual CAN command when present; earlier CAN magnitude caches are aligned by timestamp. It includes low-speed active windows down to 1 m/s. Differential evolution then robust least squares and 40 window-bootstrap refits are used. Command sign assigns motor direction only in hands-off-eligible records; omit ±0.10 s around command reversals from fitting because magnitude cannot determine reversal direction. At reversals in forward prediction the EPS follows its signed target with a lag; that is an unverified modeling assumption. Driver torque <300 units or a straight above 25 mph defines EPS eligibility; this is not a perfect hand detector.

| Parameter | Estimate | Conditional 95% interval |
| --- | --- | --- |
| deadband | 0.0176 | 0.016–0.0192 |
| raw_per_command | 112 | 110–115 |
| cap_raw | 61.4 | 55–65 |
| tau_s | 0.016 | 0.0145–0.0195 |
| fade_full_m_s | 7.37 | 5.57–9.58 |


Motor magnitude RMS residual is **6.99 raw units training / 11.74 checking**, 109 / 25 windows. The cap’s bootstrap reaches its 55-unit lower bound; cap is **unidentified**, not a new measurement replacing the known roughly 75-unit firmware limit. The simple piecewise map trades off curvature of the assist curve, deadband, saturation and driver assist. The fitted deadband differs from the earlier approximately 0.028 measurement; treat it as an effective map parameter, not a revised hardware fact. Low-speed fade is conditional and driver-confounded. Lag is now in hundredths of a second rather than the fictitious 0.25 s aggregate lag, but the held-out residual rejects a precise electrical interpretation.

## Link 3: measured motor torque to wheel motion

Use signed measured motor magnitude with the same reversal exclusion, and front aligning force from link 1 driven by measured wheel angle. Fit only column inertia, viscous friction, breakaway, effective trail and a constant load bias. Angle derivatives use a 0.31 s cubic Savitzky–Golay filter. Inverse force balance is a convenient identification method, but its torque residual is not wheel validation. The independent forward checks below drive the column with measured motor and never reinsert logged angle after initialization.

Coulomb uses a narrow tanh regularization; Dahl uses a bristle state; the reduced LuGre candidate adds a fixed 2× static/dynamic ratio, a Stribeck velocity and a fixed 0.02 s bristle-damping coefficient. This is a restricted comparison, not exhaustive full-LuGre identification. Population search over bristle displacement/Stribeck speed nests bounded linear column fits. Forty window bootstraps give column intervals conditional on the selected friction state settings. Analytic bristle integration prevents small-displacement numerical chatter.

| Family | Training torque RMSE | 1ad RMSE | 1ae RMSE |
| --- | --- | --- | --- |
| Coulomb | 0.651 N·m (assumed gauge) | 0.546 N·m (assumed gauge) | 0.641 N·m (assumed gauge) |
| Dahl | 0.633 N·m (assumed gauge) | 0.523 N·m (assumed gauge) | 0.625 N·m (assumed gauge) |
| LuGre | 0.637 N·m (assumed gauge) | 0.527 N·m (assumed gauge) | 0.628 N·m (assumed gauge) |


| Selected Dahl parameter | Estimate | Conditional 95% interval |
| --- | --- | --- |
| J_gauge | 0.469 | 0.368–0.621 |
| B_gauge | 3.77 | 2.85–4.77 |
| Fc_gauge | 0.468 | 0.408–0.525 |
| trail_gauge | 0.0203 | 0.0181–0.0225 |
| offset_Nm_gauge | 0.262 | 0.168–0.342 |


All N·m column quantities use the inherited arbitrary 0.05 N·m/raw gauge: absolute inertia, friction and trail are not identified in calibrated physical units. The selected Dahl bristle displacement reaches its lower search bound (0.000335 rad); **unidentified**. Dahl’s Stribeck-speed parameter does not enter its equation and is flat/unidentified. LuGre’s speed reaches its lower bound too. The small friction-family residual differences do not establish the van’s friction law.

| Family | Route | Windows | Median wheel correlation | Median wheel RMS ratio |
| --- | --- | --- | --- | --- |
| Coulomb | 000001ad | 14 | 0.748 | 1.21 |
| Coulomb | 000001ae | 8 | 0.733 | 1.11 |
| Dahl | 000001ad | 14 | 0.744 | 1.11 |
| Dahl | 000001ae | 8 | 0.715 | 1.15 |
| LuGre | 000001ad | 14 | 0.75 | 1.2 |
| LuGre | 000001ae | 8 | 0.727 | 1.14 |


These free-run failures expose the weakness of derivative-based inverse fitting. Reversal sign, road load and model force errors remain substantial; the column has not passed a motion gate. A forward-trajectory global refit with independently injected command as an instrument is the next model change, rather than tuning S5 against this rejected plant.

## Sensors, rack residual and plan feedback

Noise levels are median standard deviations of logged signal minus an 11-sample quadratic smoothing fit, measured on development stock windows. Add independent Gaussian samples at those levels, then quantize the controller angle to 0.1°. This high-frequency residual is an **estimated noise floor**, not identified noise across the whole shake band; it can include real fast motion and interpolation suppresses some quantization. It excludes unknown low-frequency bias and cross-sensor dependence. Yaw noise is attached to carState for completeness; V0’s feedback is computed from steering angle, so that yaw field does not drive this controller.

| mph | Angle ° | Wheel rate °/s | Yaw rad/s | Speed m/s | Rack residual SD N·m gauge | AR coefficient | Windows |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 25–45 | 0.0143 | 1.4 | 0.000406 | 0.00179 | 0.615 | 0.955 | 41 |
| >45 | 0.0095 | 1.07 | 0.000419 | 0.00239 | 0.44 | 0.954 | 42 |


Rack residual = fitted column required torque minus measured signed motor torque, demeaned per window. Fit one AR(1) coefficient per speed band without pairing samples across window edges; innovation SD gives the measured stationary residual variance. Inject newly seeded residual samples, never the actual held-out road residual. This is **not independently measured road torque**: it also contains derivative error, human input, alignment error and failed tyre/column dynamics. Treating all of it as independent exogenous rack forcing can over-excite the loop and double-count measurement error. The factorial therefore diagnoses this chosen stochastic model, not a proven road/wind cause.

`report_vsim2.md` measured planner responses only at periods 7–20 s, about 0.05–0.143 times per second. `report_plan.md` does not establish a causal planner transfer in 0.3–3 times per second either. Thus fixed logged plan is a conditional experiment, not evidence that the planner can be neglected in this band. A fast injected-video transfer measurement remains needed before claiming full perception closure. No unmeasured 0.3–3 transfer has been extrapolated from those reports.

For lane experiments, modelV2 lane geometry is reread from 1ac/1ad/1ae rlogs into scratch-only `_plan2.npz` caches. Reconstruct the cubic 25 m offset/heading correction, capped at ±0.15 m/s² (±12 N on 80 kg); retain logged desired curvature and replace its reconstructed historical lane component. The factorial toggles the **fast raw-minus-0.5 s-smoothed component**, retaining slow correction. Forecasts compare correction absent versus 0.5 s smoothed. Other clip_curvature/model-history interactions are not replayed exactly; future model acceleration arrays remain logged. These are conditional lane-noise interventions, not a new live planner.

Roll: the raw pass-1 IMU cache does not preserve enough sensor identity/extrinsic/stationary-gravity verification to establish a vehicle frame. No roll claim is made from S5 blocks, and no roll state is added to yaw or labeled seat/head acceleration. Body metric remains speed × measured/simulated yaw, as requested.

## Gate 1: whole chain, coherent bins only — FAIL

| Controller | mph | Output | Pass / coherent bins | Uncovered bins | Median correlation |
| --- | --- | --- | --- | --- | --- |
| stock | 25–45 | wheel | 0/6 | 2 | 0.504 |
| stock | 25–45 | yaw_acceleration | 0/4 | 4 | 0.581 |
| stock | >45 | wheel | 0/8 | 4 | 0.762 |
| stock | >45 | yaw_acceleration | 0/7 | 5 | 0.62 |
| S3 | 25–45 | wheel | 0/7 | 1 | 0.883 |
| S3 | 25–45 | yaw_acceleration | 3/6 | 2 | 0.791 |
| S3 | >45 | wheel | 0/14 | 2 | 0.758 |
| S3 | >45 | yaw_acceleration | 0/15 | 1 | 0.783 |
| S5 | 25–45 | wheel | 0/9 | 3 | 0.842 |
| S5 | 25–45 | yaw_acceleration | 2/10 | 2 | 0.821 |
| S5 | >45 | wheel | 0/11 | 1 | 0.735 |
| S5 | >45 | yaw_acceleration | 0/6 | 6 | 0.764 |


**0 of 12 cells pass; 5/103 coherent bins pass; 33 bins are uncovered.** This table checks 1ad and exposed 1ae; 1ac rows are also retained in `gate12.json` as development diagnostics. Frequency points are 0.5, 1.0, 1.5 and 2.0 times per second, with at least 0.7 measured command/output coherence. Gain tolerance remains ±20%, phase ±20°, and the original per-window correlation ≥0.8 is retained. Uncovered bins never pass by omission. A cell passes only if every covered bin and every window correlation passes; there is no optimistic median-bin gate.

## Gate 2: real controllers with sensor noise and rack disturbance — FAIL

Development:

| Controller | Windows | Crossings ratio | +0.1 s size ratio | Command RMS ratio | Wheel RMS ratio | Body RMS ratio | Peak error times/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| stock | 11 | 1.43 | 0.901 | 1.21 | 1.74 | 1.29 | 0.195 |
| S3 | 7 | 1.07 | 2.41 | 1.81 | 4.16 | 4.28 | 0.0977 |
| S5 | 11 | 0.438 | 5.61 | 4.16 | 19.8 | 18.4 | 0.488 |


Exposed check:

| Controller | Windows | Crossings ratio | +0.1 s size ratio | Command RMS ratio | Wheel RMS ratio | Body RMS ratio | Peak error times/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| stock | 2 | 1.84 | 1.3 | 1.4 | 2.11 | 1.5 | 0.879 |
| S3 | 2 | 1.39 | 1.24 | 1.13 | 1.56 | 1.14 | 0.293 |
| S5 | 3 | 0.386 | 5.39 | 4.94 | 12.4 | 9.63 | 0.293 |


Ratios are median paired-window simulation/road ratios; each simulated window is the median of three seeds. A ±25% bundle requires every ratio 0.75–1.25 and S5 median peak error ≤0.3 times per second. Even this aggregate test fails for all development controllers; stricter per-window checking cannot rescue it. The S5 development peak subcheck also fails. Seed-wise values and every road/simulation statistic are in `gate22.json`.

| Controller | Road body RMS | Simulated body RMS | Road crossings/s | Simulated crossings/s |
| --- | --- | --- | --- | --- |
| stock | 0.0581 m/s² (4.64 N on 80 kg) | 0.0699 m/s² (5.59 N on 80 kg) | 2.79 | 4.64 |
| S3 | 0.0944 m/s² (7.55 N on 80 kg) | 0.5282 m/s² (42.26 N on 80 kg) | 3.21 | 3.21 |
| S5 | 0.0994 m/s² (7.95 N on 80 kg) | 2.0472 m/s² (163.77 N on 80 kg) | 3.43 | 1.5 |


## Closed-loop factorial: what this model amplifies

All **32 cells × 2 high-speed 1ac windows × 3 paired seeds = 192 runs** completed. Factors: stock/S5 model, shaper error gain 1/2, sensor noise off/on, rack disturbance off/on, fast lane correction off/on. Retain each model’s other shaper constants, so the model contrast includes architecture/weights plus its non-gain shaping. Same window/seed comparisons hold the exogenous plan fixed. This is not a factor sweep over the invalid pass-1 plant.

| Factor high / low | Median paired body RMS multiplier | Command RMS multiplier | Crossings multiplier |
| --- | --- | --- | --- |
| model | 22.8 | 8.8 | 0.368 |
| gain | 1.6 | 1.53 | 0.815 |
| noise | 1 | 1 | 1.05 |
| disturbance | 1.12 | 1.06 | 1 |
| lane | 1.19 | 1.19 | 1.05 |


| Model, all forcing on | Error gain | Body RMS | Crossings/s |
| --- | --- | --- | --- |
| stock | 1 | 0.0874 m/s² (7.00 N on 80 kg) | 5.21 |
| stock | 2 | 0.1619 m/s² (12.95 N on 80 kg) | 5.11 |
| S5 | 1 | 1.2860 m/s² (102.88 N on 80 kg) | 2 |
| S5 | 2 | 2.2007 m/s² (176.06 N on 80 kg) | 1.54 |


With all forcing on, the paired gain-doubling body multiplier is **1.07 times larger for S5 than stock**. These effects depend strongly on the window and saturation; a single product of marginal medians is not a causal decomposition. Raw cells remain in `factorial2.json`. The plant already over-amplifies S5, so none of these simulated multipliers establishes how much of the real shake was motor stiction, sensors, rack disturbance, shaper gain or lane noise. **The tenfold earlier forecast miss is still not causally resolved.** What is established is that the earlier clean-loop fit omitted excitation sources, while this attempted correction over-excites a poorly validated chain. The road’s compressor observation remains direct evidence; this factorial is a falsified explanatory model of it.

![Rejected-model gates and factorial](../shake/gates2.png)

## Locked forecast for the next highway drive

`forecast_locked2.json` is locked before accessing any next-drive data; SHA-256 is in `forecast_lock2.json`. It predicts **stock NNLC only**, lane correction off and 0.5 s smoothed, above 45 mph, under the same eligibility/filter/scoring rules. Five seeds over 20 development highway windows produce 100 conditional window outcomes per arm. It is a falsifiable forecast from a rejected model, **not a validated prediction interval**; its 2.5–97.5 percentile ranges omit parameter uncertainty, road-distribution change and model discrepancy. No S3/S5 road use is proposed. No lane-ranking confidence follows from these overlapping ranges.

| Lane correction | Metric | Forecast median | Conditional 95% range |
| --- | --- | --- | --- |
| off | crossings_s | 4.07 | 0 – 7.07 |
| off | after_crossing | 0.0445 | 0 – 0.363 |
| off | command_rms | 0.0632 | 0.0453 – 0.117 |
| off | wheel_rms_deg | 0.291 | 0.196 – 0.557 |
| off | body_rms | 0.0594 m/s² (4.75 N on 80 kg) | 0.0335 m/s² (2.68 N on 80 kg) – 0.1229 m/s² (9.83 N on 80 kg) |
| smoothed | crossings_s | 4.54 | 0 – 8.12 |
| smoothed | after_crossing | 0.0539 | 0 – 0.339 |
| smoothed | command_rms | 0.0672 | 0.0497 – 0.121 |
| smoothed | wheel_rms_deg | 0.318 | 0.221 – 0.603 |
| smoothed | body_rms | 0.0651 m/s² (5.21 N on 80 kg) | 0.0380 m/s² (3.04 N on 80 kg) – 0.1378 m/s² (11.02 N on 80 kg) |


Crossings are raw command sign crossings per second; +0.1 s size is normalized command. Command RMS is normalized full scale, wheel RMS is degrees; all RMS values use 0.3–3 times per second. Following pass 1, a window with no qualifying crossing is assigned zero for +0.1 s size; zero at the interval floor is this missing-event convention, not a measured zero-size crossing. Score window medians per arm on the untouched next highway drive, report its sample count and achieved speeds, and reject the model if any five-statistic ratio is outside ±25%. Do not retune this JSON after seeing that drive.

## No quiet-bump design yet; shortest useful excitation

The required closed-loop gate fails, so no quiet-dead-zone module, optimized pulse height/time base or candidate road forecast is justified. The algebraic duty correction can still be stated without tuning: for requested linear-equivalent command q and pulse height H above deadband d, duty = |q|/(H−d), capped at 1, if q means torque on the zero-deadband linear map. With H=0.06 and d=0.028, old duty |q|/0.06 supplies only 0.032/0.06 = **53%** of that intended torque. If q instead denotes an existing deadband-bearing actuator request, duty = max(|q|−d,0)/(H−d). These are different request semantics. EPS lag, slew, stiction and finite pulse duration make either static formula insufficient for selecting a felt bump. Whether to low-pass first remains an experiment, not a deployed answer.

**Minimum informative run: one complete existing Odyssey suite at steady 40 mph and one at 55–60 mph**, each on a quiet straight stretch with normal driver supervision and per-suite arming. The suite already provides frozen baseline torque holds with bounded hold correction, 0 / 3 / 4.5 / 6 / 8% added command, both signs, 0.5 and 1.0 s widths, with left-first and right-first repetitions. Do not replace this with S5 oscillation. The code remains unchanged; actual applied CAN command, not nominal injected amplitude, is the identification input.

| Per speed | Pulses/holds | Duration | What it identifies |
| --- | --- | --- | --- |
| 0.5 s, left-first and right-first | 20 total: each of five levels twice each sign | 60 s action time | EPS onset/lag, reversal/breakaway, fast impulse response |
| 1.0 s, left-first and right-first | 20 total: each of five levels twice each sign | 70 s action time | sustained wheel response vs short pulse, aligning load/viscous effects |
| Four steady-speed readiness periods | 2 s each | 8 s | reject acceleration/transient starts |
| Both speed plateaus | 80 total holds, including 16 zero-input shams | 276 s = 4 min 36 s minimum | speed dependence and direction asymmetry |


The holds last 1.5 s for short pulses and 2.0 s for long pulses; 1.5 s normal lane keeping between holds recentres the van. Existing `MAX_HOLD_CORRECTION=0.1` can clip a nominal freeze: retain the actual correction and mark clipped/aborted holds rather than treating them as independent excitation. Identify EPS magnitude lag from native CAN timestamps; estimate wheel motion with the rate signal plus quantized angle; compare matched zero-pulse holds. Fit measured wheel-to-yaw jointly with all normal turns between holds. Low-speed fade and full-cap behavior are not pinned down by this 40–60 mph run and are not needed to optimize a small highway bump.

This is the shortest **existing-suite** two-speed design with repeats and shams, not a guarantee of closing all 0.3–3-times-per-second bins. A single 1.5–2 s hold cannot support a settled 0.3-times-per-second sinusoid, so no long frozen-lane-control sine is specified for a public road. Pulses have broadband energy; pool randomized-sign/repeat impulse responses and use injected pulse as an instrument to reduce feedback bias. Preserve timing and score achieved coherence, directional repeatability and held-out wheel trajectories. If 8% pulses remain below the wheel-response noise, do not escalate automatically: report that those parameters remain unidentified. After the two-speed training run, require a separate stock highway drive as validation; an additional 138 s suite at a previously tested speed is the shortest same-protocol holdout if ordinary driving remains under-excited. Occupants/seats/cargo, tyre pressure/model, road surface and wind should be recorded for each plateau.

## Verification and exact continuation

Eight checks pass under the repository unittest runner: four retained pass-1 mechanics checks plus independent tyre time/frequency agreement, stable/sign-symmetric bristles, plant zero equilibrium/signed response, and EPS deadband/cap/fade. Ruff passes over the entire shake directory; the static plot was visually checked for legible labels and paired acceleration/force axes. No fresh reviewer agents were spawned because this is a research-worker assignment; three self-review passes checked equations/units, data split/reversal/coverage, and runtime/forecast provenance. Fixes included the NumPy batched-solve shape, stable analytic bristle integration and production smoothing-delay handling. Tests prove code invariants; they do not override failed road gates.

Reproduce with `PYTHONPATH=$PWD:<assigned scratch>/shake/site .venv/bin/python -m tools.odyssey_sim.shake.identify2`, then `...loop2`, `...column_check2`, and `...report2`. Full fitting uses seeded global searches and 40 cluster/window bootstrap fits; running it replaces pass-2 fit files. **Do not rerun the forecast writer after opening the next drive.** Keep its current hash as the pre-drive lock. Large plan caches and logs stay in scratch. `summary2.json` consolidates the computed gate counts/effects, while raw per-bin/window/seed evidence remains in the individual JSONs.

Exact continuation: first investigate why measured-motor forward column trajectories fail despite moderate inverse torque residuals; fit forward wheel trajectories on independent held-pulse training records with correctly handled reversal sign and nuisance road load, retaining the separate tyre-link check. Then rerun these gates before selecting a PWM bump. No seat/head or roll sensor claim is needed to use validated yaw-body motion. Confirm sensor-frame/timing only before making an additional IMU-specific roll claim.

Sources for the inherited physical structure: [MathWorks bicycle model equations](https://www.mathworks.com/help/driving/ref/bicyclemodel.html) for two-axle force/yaw balance; [Canudas de Wit et al., 1995](https://lup.lub.lu.se/search/ws/files/6363840/8498924.pdf) for dynamic friction with bristle state. Neither source validates this van’s fitted parameters. Plan response evidence is local `report_vsim2.md` and `report_plan.md`; published vehicle specifications and their model-year/load caveats remain in `report_shake.md`.
