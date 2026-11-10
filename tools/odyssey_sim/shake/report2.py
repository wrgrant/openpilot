# ruff: noqa: E501
"""Render the pass-2 evidence report; never changes fitted/locked artifacts."""
import hashlib
import json
import numpy as np
from tools.odyssey_sim.shake.identify2 import save
from tools.odyssey_sim.shake.analyze import OUT


def read(name):
  return json.loads((OUT/name).read_text())


def mdtable(headers, rows):
  return '\n'.join(['| '+' | '.join(headers)+' |', '| '+' | '.join(['---']*len(headers))+' |']+
                   ['| '+' | '.join(map(str, row))+' |' for row in rows])+'\n'


def fmt(x):
  return f'{x:.3g}'


def force(x):
  return f'{x:.4f} m/s² ({80*x:.2f} N on 80 kg)'


def main():
  yaw, eps, col = read('yaw2.json'), read('eps2.json'), read('column2.json')
  opens, closed = read('gate12.json')['rows'], read('gate22.json')['rows']
  fact, forecast = read('factorial2.json')['rows'], read('forecast_locked2.json')
  text = []
  def add(s):
    text.append(s+'\n')
  add('# Shake research, pass 2: the simulator still fails')
  add('**No: it still does not reproduce the US-169 shake well enough to tune your steering.** Separating the measured links removes the fictitious quarter-second motor delay, but exposes a larger problem: the fitted column and tyre model does not transfer reliably between routes or controllers. Adding measured noise estimates and a stock-road torque residual does produce frequent stock-command crossings; it also makes the S5 simulation much too large. This is a rejected model, not a quiet-bump design tool.')
  dev = [r for r in closed if r['name'] != '000001ae']
  s5 = [r for r in dev if r['model'] == 2]
  add(f"On development windows S5 body RMS is **{np.median([r['ratios']['body_rms'] for r in s5]):.2f} times the road**, and its command crosses zero **{np.median([r['ratios']['crossings_s'] for r in s5]):.2f} times as often**. The closed-loop bundle passes for **0 of 3 controllers**. Wheel-to-yaw alone passes 100 of 298 coherent held-out/check frequency bins; it already fails before the EPS is involved. More search cannot turn that into a validated physical explanation.")
  add('Your reported US-169 motion remains the reference: ±0.25 m/s² (±20 N on 80 kg) typical and ±0.50–0.625 m/s² (±40–50 N), about 1.3 oscillations per second and 2.7 direction changes per second. The new tables below use RMS, which is a different statistic. Your 75 kg force is 0.9375 times the quoted 80 kg force.')
  add('**Recommendation:** keep S5 off. Use the existing held-pulse suite at steady 40 and 55–60 mph to identify small-signal breakaway and reversal response independently of lane keeping, then check the frozen result on a later drive. The minimum two-speed suite is about **4 minutes 36 seconds**, excluding acceleration, traffic interruptions and aborts. No extra sensor or closed course is required by this specification. It is a data-collection proposal; nothing was enabled or sent to the van.')
  add('## Scope, split and reproducibility')
  a, b = read('loop_manifest_start2.json'), read('loop_manifest_end2.json')
  changed = [p for p, h in a['sha256'].items() if b['sha256'].get(p) != h]
  add(f"Checkout `odyssey-sim`, HEAD `{a['head']}`. No branch switch, worktree, commit, push or device access. All authored analysis and results stay in `tools/odyssey_sim/shake/`; caches and Params stay in the assigned `scratchpad/shake/`. This explicitly requested report is the only folder-rule exception. Pass-1 artifacts are retained. Runtime start/end hashes and full dirty status are in `loop_manifest_start2.json` / `loop_manifest_end2.json`. Concurrently changed during the campaign: **{', '.join(changed) or 'none'}**. Imported Python objects stay loaded within the process; this does not recreate the historical on-van controller. Exact imported source bytes were not archived before import, so the boundary hashes are a provenance limitation, not proof of a frozen source checkout.")
  add('Tyre fitting trains on routes through 1ac excluding entire 1a5, 19c and 1ad routes, which serve as link checks; 1a4 hand driving enters training and 1a5 hand driving enters checking. 1ae is only an exposed check. EPS trains on 1a7/1aa/1ab/1ac, checks on 1ad/1ae. Column parameter fitting uses the same training pool, selects friction family on 1ad, and checks 1ae. Thus 1ad is development/model-selection data for the assembled plant, not an untouched whole-chain holdout. **Only the next highway drive is untouched proof.** No logs after 1ae were opened.')
  add('Continuous, nonoverlapping 20 s records are resampled at 100 samples per second; gaps, segments, speed bands and eligibility split records. Tyre fitting uses all eligible steering motion, including the two manual routes; automated records require lateral active. Driver-press flags on straights are retained. Curves with true hands can still contaminate the fits. Whole-chain windows retain pass 1’s 180 s model/lane arms and skip their first 30 s. Score after 5 s warmup, trim the final 1 s, and apply the same 0.3–3 oscillations-per-second bandpass to road and simulation. The real V0/NNLC/response shaper/OdysseySteer runs, with scratch-only Params, learned calibration and logged future plans. The LaC now receives learned delay plus `LAT_SMOOTH_SECONDS`; NNLC receives learned delay. This also differs from pass 1, so noise alone is not the only changed element.')
  add('## Link 1: measured wheel angle to measured yaw')
  add('The four states are lateral velocity, yaw rate, front tyre force and rear tyre force. Front/rear forces relax toward linear slip-angle forces with time constant relaxation-length/speed. Fit front and rear axle stiffness, yaw inertia and common relaxation length separately in 25–45 and >45 mph bands, with fixed inherited mass 2163.35 kg, wheelbase 3.00 m, CG distances 1.35/1.65 m and steering ratio 14.35. Mass includes the inherited nominal 80 kg load; actual occupants, fuel and tyre conditions remain unknown. Roll has no feedback path into yaw. No fitted scale is added to the measured angle or yaw.')
  add('Each band uses differential evolution (40-member population, up to 90 generations), then bounded least squares. Forty route-cluster bootstrap fits give conditional percentile 95% intervals. Bounds: stiffness 20–600 kN/rad, yaw inertia 1000–16000 kg·m², relaxation length 0.02–6 m. Fits use only 0.3–3 oscillations-per-second bins with measured angle/yaw coherence ≥0.7. Five-second Hann records with half overlap give 0.2 oscillations-per-second spacing: the first scored bin is 0.4, not an invented exact 0.3 bin. Overlapping spectral records are not independent route repetitions.')
  rows = []
  for f in yaw['fits']:
    for j, name in enumerate(yaw['parameter_names']):
      rows.append(['25–45' if f['speed'] == 0 else '>45', name, fmt(f['parameters'][j]),
                   f"{fmt(f['interval95'][0][j])}–{fmt(f['interval95'][1][j])}"])
  add(mdtable(['mph', 'Parameter', 'Estimate', 'Conditional 95% interval'], rows))
  rows = []
  for f in yaw['fits']:
    for route in yaw['heldout_routes']+['000001ae']:
      z = [r for r in f['test'] if r['route'] == route]
      rows.append(['25–45' if f['speed'] == 0 else '>45', route, sum(r['covered'] for r in z), sum(r['passed'] for r in z), sum(not r['covered'] for r in z)])
  add(mdtable(['mph', 'Route', 'Coherent bins', 'Pass ±15%, ±15°', 'Uncovered'], rows))
  add('Low-band rear stiffness has a bootstrap interval reaching the 600 kN/rad search ceiling: it is weakly identified/bound-limited, not a precise stiffness measurement. The implied understeer coefficient m/L × (b/Cf − a/Cr) is about 0.00424 versus 0.00313 s²/m in the two bands. The across-speed estimates are not physically consistent enough: rear stiffness changes by more than a factor of three and yaw inertia nearly halves. Route bootstrap intervals are conditional on this inadequate linear model, not a physical confidence certificate. Model discrepancy, calibration/timing and tyre nonlinearity remain mixed. The measured yaw is the validated carControl z rate, not VSA lateral acceleration or torqueState’s steering-derived acceleration. Route window counts, every scored/uncovered bin, gains and phases are retained in `yaw2.json`.')
  add('## Link 2: CAN command to motor magnitude')
  add('A saturating deadband map with first-order magnitude lag and linear low-speed fade is fitted independently of wheel/yaw. Use the actual CAN command when present; earlier CAN magnitude caches are aligned by timestamp. It includes low-speed active windows down to 1 m/s. Differential evolution then robust least squares and 40 window-bootstrap refits are used. Command sign assigns motor direction only in hands-off-eligible records; omit ±0.10 s around command reversals from fitting because magnitude cannot determine reversal direction. At reversals in forward prediction the EPS follows its signed target with a lag; that is an unverified modeling assumption. Driver torque <300 units or a straight above 25 mph defines EPS eligibility; this is not a perfect hand detector.')
  add(mdtable(['Parameter', 'Estimate', 'Conditional 95% interval'], [[name, fmt(eps['parameters'][j]), f"{fmt(eps['interval95'][0][j])}–{fmt(eps['interval95'][1][j])}"] for j, name in enumerate(eps['names'])]))
  add(f"Motor magnitude RMS residual is **{eps['train_rmse']:.2f} raw units training / {eps['test_rmse']:.2f} checking**, {eps['train_windows']} / {eps['test_windows']} windows. The cap’s bootstrap reaches its 55-unit lower bound; cap is **unidentified**, not a new measurement replacing the known roughly 75-unit firmware limit. The simple piecewise map trades off curvature of the assist curve, deadband, saturation and driver assist. The fitted deadband differs from the earlier approximately 0.028 measurement; treat it as an effective map parameter, not a revised hardware fact. Low-speed fade is conditional and driver-confounded. Lag is now in hundredths of a second rather than the fictitious 0.25 s aggregate lag, but the held-out residual rejects a precise electrical interpretation.")
  add('## Link 3: measured motor torque to wheel motion')
  add('Use signed measured motor magnitude with the same reversal exclusion, and front aligning force from link 1 driven by measured wheel angle. Fit only column inertia, viscous friction, breakaway, effective trail and a constant load bias. Angle derivatives use a 0.31 s cubic Savitzky–Golay filter. Inverse force balance is a convenient identification method, but its torque residual is not wheel validation. The independent forward checks below drive the column with measured motor and never reinsert logged angle after initialization.')
  add('Coulomb uses a narrow tanh regularization; Dahl uses a bristle state; the reduced LuGre candidate adds a fixed 2× static/dynamic ratio, a Stribeck velocity and a fixed 0.02 s bristle-damping coefficient. This is a restricted comparison, not exhaustive full-LuGre identification. Population search over bristle displacement/Stribeck speed nests bounded linear column fits. Forty window bootstraps give column intervals conditional on the selected friction state settings. Analytic bristle integration prevents small-displacement numerical chatter.')
  add(mdtable(['Family', 'Training torque RMSE', '1ad RMSE', '1ae RMSE'], [[f['label']]+[fmt(f['torque_residual_rmse'][k])+' N·m (assumed gauge)' for k in ('train', 'test', 'check')] for f in col['fits']]))
  add(mdtable(['Selected '+col['selected']['label']+' parameter', 'Estimate', 'Conditional 95% interval'], [[name, fmt(col['selected']['parameters'][j]), f"{fmt(col['selected']['interval95'][0][j])}–{fmt(col['selected']['interval95'][1][j])}"] for j, name in enumerate(col['names'])]))
  add('All N·m column quantities use the inherited arbitrary 0.05 N·m/raw gauge: absolute inertia, friction and trail are not identified in calibrated physical units. The selected Dahl bristle displacement reaches its lower search bound (0.000335 rad); **unidentified**. Dahl’s Stribeck-speed parameter does not enter its equation and is flat/unidentified. LuGre’s speed reaches its lower bound too. The small friction-family residual differences do not establish the van’s friction law.')
  check = read('column_check2.json')['rows']
  add(mdtable(['Family', 'Route', 'Windows', 'Median wheel correlation', 'Median wheel RMS ratio'], [[label, route, len(z), fmt(np.median([r['wheel_corr'] for r in z])), fmt(np.median([r['rms_ratio'] for r in z]))] for label in ('Coulomb', 'Dahl', 'LuGre') for route in ('000001ad', '000001ae') if (z := [r for r in check if r['friction'] == label and r['route'] == route])]))
  add('These free-run failures expose the weakness of derivative-based inverse fitting. Reversal sign, road load and model force errors remain substantial; the column has not passed a motion gate. A forward-trajectory global refit with independently injected command as an instrument is the next model change, rather than tuning S5 against this rejected plant.')
  add('## Sensors, rack residual and plan feedback')
  add('Noise levels are median standard deviations of logged signal minus an 11-sample quadratic smoothing fit, measured on development stock windows. Add independent Gaussian samples at those levels, then quantize the controller angle to 0.1°. This high-frequency residual is an **estimated noise floor**, not identified noise across the whole shake band; it can include real fast motion and interpolation suppresses some quantization. It excludes unknown low-frequency bias and cross-sensor dependence. Yaw noise is attached to carState for completeness; V0’s feedback is computed from steering angle, so that yaw field does not drive this controller.')
  add(mdtable(['mph', 'Angle °', 'Wheel rate °/s', 'Yaw rad/s', 'Speed m/s', 'Rack residual SD N·m gauge', 'AR coefficient', 'Windows'], [[('25–45' if n['speed'] == 0 else '>45')]+[fmt(x) for x in n['sensor_std']]+[fmt(n['rack_std_Nm_gauge']), fmt(n['rack_rho']), n['windows']] for n in col['noise']]))
  add('Rack residual = fitted column required torque minus measured signed motor torque, demeaned per window. Fit one AR(1) coefficient per speed band without pairing samples across window edges; innovation SD gives the measured stationary residual variance. Inject newly seeded residual samples, never the actual held-out road residual. This is **not independently measured road torque**: it also contains derivative error, human input, alignment error and failed tyre/column dynamics. Treating all of it as independent exogenous rack forcing can over-excite the loop and double-count measurement error. The factorial therefore diagnoses this chosen stochastic model, not a proven road/wind cause.')
  add('`report_vsim2.md` measured planner responses only at periods 7–20 s, about 0.05–0.143 times per second. `report_plan.md` does not establish a causal planner transfer in 0.3–3 times per second either. Thus fixed logged plan is a conditional experiment, not evidence that the planner can be neglected in this band. A fast injected-video transfer measurement remains needed before claiming full perception closure. No unmeasured 0.3–3 transfer has been extrapolated from those reports.')
  add('For lane experiments, modelV2 lane geometry is reread from 1ac/1ad/1ae rlogs into scratch-only `_plan2.npz` caches. Reconstruct the cubic 25 m offset/heading correction, capped at ±0.15 m/s² (±12 N on 80 kg); retain logged desired curvature and replace its reconstructed historical lane component. The factorial toggles the **fast raw-minus-0.5 s-smoothed component**, retaining slow correction. Forecasts compare correction absent versus 0.5 s smoothed. Other clip_curvature/model-history interactions are not replayed exactly; future model acceleration arrays remain logged. These are conditional lane-noise interventions, not a new live planner.')
  add('Roll: the raw pass-1 IMU cache does not preserve enough sensor identity/extrinsic/stationary-gravity verification to establish a vehicle frame. No roll claim is made from S5 blocks, and no roll state is added to yaw or labeled seat/head acceleration. Body metric remains speed × measured/simulated yaw, as requested.')
  add('## Gate 1: whole chain, coherent bins only — FAIL')
  open_summary = []
  for model in range(3):
    for speed in range(2):
      for output in ('wheel', 'yaw_acceleration'):
        group = [r for r in opens if r['name'] != '000001ac' and r['model'] == model and r['speed'] == speed]
        ms = [next(m for m in r['metrics'] if m['output'] == output) for r in group]
        bins = [b for m in ms for b in m['bins']]
        covered = [b for b in bins if b['coherence'] >= .7]
        good = [b for b in covered if .8 <= b['gain_ratio'] <= 1.2 and abs(b['phase_error_deg']) <= 20]
        corr = float(np.median([m['corr'] for m in ms])) if ms else None
        passed = bool(covered and len(good) == len(covered) and ms and all(m['corr'] >= .8 for m in ms))
        open_summary.append({'model': model, 'speed': speed, 'output': output, 'covered': len(covered), 'pass_bins': len(good), 'uncovered': len(bins)-len(covered), 'corr': corr, 'passed': passed})
  add(mdtable(['Controller', 'mph', 'Output', 'Pass / coherent bins', 'Uncovered bins', 'Median correlation'], [[('stock', 'S3', 'S5')[r['model']], ('25–45' if r['speed'] == 0 else '>45'), r['output'], f"{r['pass_bins']}/{r['covered']}", r['uncovered'], fmt(r['corr']) if r['corr'] is not None else 'absent'] for r in open_summary]))
  total = sum(r['covered'] for r in open_summary)
  good = sum(r['pass_bins'] for r in open_summary)
  uncovered = sum(r['uncovered'] for r in open_summary)
  add(f"**{sum(r['passed'] for r in open_summary)} of 12 cells pass; {good}/{total} coherent bins pass; {uncovered} bins are uncovered.** This table checks 1ad and exposed 1ae; 1ac rows are also retained in `gate12.json` as development diagnostics. Frequency points are 0.5, 1.0, 1.5 and 2.0 times per second, with at least 0.7 measured command/output coherence. Gain tolerance remains ±20%, phase ±20°, and the original per-window correlation ≥0.8 is retained. Uncovered bins never pass by omission. A cell passes only if every covered bin and every window correlation passes; there is no optimistic median-bin gate.")
  add('## Gate 2: real controllers with sensor noise and rack disturbance — FAIL')
  summaries = []
  for scope, names in (('development', ('000001ac', '000001ad')), ('exposed check', ('000001ae',))):
    rows = []
    for model in range(3):
      group = [r for r in closed if r['name'] in names and r['model'] == model]
      ratios = {k: float(np.median([r['ratios'][k] for r in group])) for k in group[0]['ratios']}
      peak = float(np.median([r['peak_error'] for r in group]))
      passed = all(.75 <= x <= 1.25 for x in ratios.values()) and (model != 2 or peak <= .3)
      summaries.append({'scope': scope, 'model': model, 'windows': len(group), 'ratios': ratios, 'peak_error': peak, 'passed': passed})
      rows.append([('stock', 'S3', 'S5')[model], len(group)]+[fmt(x) for x in ratios.values()]+[fmt(peak)])
    add(scope.capitalize()+':')
    add(mdtable(['Controller', 'Windows', 'Crossings ratio', '+0.1 s size ratio', 'Command RMS ratio', 'Wheel RMS ratio', 'Body RMS ratio', 'Peak error times/s'], rows))
  add('Ratios are median paired-window simulation/road ratios; each simulated window is the median of three seeds. A ±25% bundle requires every ratio 0.75–1.25 and S5 median peak error ≤0.3 times per second. Even this aggregate test fails for all development controllers; stricter per-window checking cannot rescue it. The S5 development peak subcheck also fails. Seed-wise values and every road/simulation statistic are in `gate22.json`.')
  rows = []
  for m in range(3):
    z = [r for r in dev if r['model'] == m]
    rows.append([('stock', 'S3', 'S5')[m], force(np.median([r['road']['body_rms'] for r in z])), force(np.median([r['sim']['body_rms'] for r in z])), fmt(np.median([r['road']['crossings_s'] for r in z])), fmt(np.median([r['sim']['crossings_s'] for r in z]))])
  add(mdtable(['Controller', 'Road body RMS', 'Simulated body RMS', 'Road crossings/s', 'Simulated crossings/s'], rows))
  add('## Closed-loop factorial: what this model amplifies')
  add('All **32 cells × 2 high-speed 1ac windows × 3 paired seeds = 192 runs** completed. Factors: stock/S5 model, shaper error gain 1/2, sensor noise off/on, rack disturbance off/on, fast lane correction off/on. Retain each model’s other shaper constants, so the model contrast includes architecture/weights plus its non-gain shaping. Same window/seed comparisons hold the exogenous plan fixed. This is not a factor sweep over the invalid pass-1 plant.')
  rows = []
  effects = {}
  for factor in ('model', 'gain', 'noise', 'disturbance', 'lane'):
    levels = sorted({r[factor] for r in fact})
    pairs = []
    for r in fact:
      if r[factor] != levels[0]:
        continue
      other = next(x for x in fact if x[factor] == levels[1] and all(x[k] == r[k] for k in ('model', 'gain', 'noise', 'disturbance', 'lane', 'window', 'seed') if k != factor))
      pairs.append({k: other['stats'][k]/r['stats'][k] if r['stats'][k] else None for k in ('body_rms', 'command_rms', 'crossings_s')})
    vals = {k: float(np.median([r[k] for r in pairs if r[k] is not None])) for k in pairs[0]}
    effects[factor] = vals
    rows.append([factor]+[fmt(x) for x in vals.values()])
  add(mdtable(['Factor high / low', 'Median paired body RMS multiplier', 'Command RMS multiplier', 'Crossings multiplier'], rows))
  interaction = []
  for model in (0, 2):
    for gain in (1, 2):
      z = [r for r in fact if r['model'] == model and r['gain'] == gain and r['noise'] and r['disturbance'] and r['lane']]
      interaction.append([('stock', 'S5')[model//2], gain, force(np.median([r['stats']['body_rms'] for r in z])), fmt(np.median([r['stats']['crossings_s'] for r in z]))])
  add(mdtable(['Model, all forcing on', 'Error gain', 'Body RMS', 'Crossings/s'], interaction))
  # True paired difference-in-differences as a multiplicative interaction.
  inter = []
  for window in (0, 1):
    for seed in range(3):
      def val(model, gain, window=window, seed=seed):
        return next(r['stats']['body_rms'] for r in fact if r['model'] == model and r['gain'] == gain and r['noise'] and r['disturbance'] and r['lane'] and r['window'] == window and r['seed'] == seed)
      inter.append((val(2, 2)/val(2, 1))/(val(0, 2)/val(0, 1)))
  add(f"With all forcing on, the paired gain-doubling body multiplier is **{np.median(inter):.3g} times larger for S5 than stock**. These effects depend strongly on the window and saturation; a single product of marginal medians is not a causal decomposition. Raw cells remain in `factorial2.json`. The plant already over-amplifies S5, so none of these simulated multipliers establishes how much of the real shake was motor stiction, sensors, rack disturbance, shaper gain or lane noise. **The tenfold earlier forecast miss is still not causally resolved.** What is established is that the earlier clean-loop fit omitted excitation sources, while this attempted correction over-excites a poorly validated chain. The road’s compressor observation remains direct evidence; this factorial is a falsified explanatory model of it.")
  add('![Rejected-model gates and factorial](../shake/gates2.png)')
  add('## Locked forecast for the next highway drive')
  add('`forecast_locked2.json` is locked before accessing any next-drive data; SHA-256 is in `forecast_lock2.json`. It predicts **stock NNLC only**, lane correction off and 0.5 s smoothed, above 45 mph, under the same eligibility/filter/scoring rules. Five seeds over 20 development highway windows produce 100 conditional window outcomes per arm. It is a falsifiable forecast from a rejected model, **not a validated prediction interval**; its 2.5–97.5 percentile ranges omit parameter uncertainty, road-distribution change and model discrepancy. No S3/S5 road use is proposed. No lane-ranking confidence follows from these overlapping ranges.')
  rows = []
  for arm in forecast['forecasts']:
    for key, m in arm['metrics'].items():
      def unit(x, key=key):
        return force(x) if key == 'body_rms' else fmt(x)
      rows.append(['smoothed' if arm['smoothed_lane_correction'] else 'off', key, unit(m['median']), unit(m['interval95'][0])+' – '+unit(m['interval95'][1])])
  add(mdtable(['Lane correction', 'Metric', 'Forecast median', 'Conditional 95% range'], rows))
  add('Crossings are raw command sign crossings per second; +0.1 s size is normalized command. Command RMS is normalized full scale, wheel RMS is degrees; all RMS values use 0.3–3 times per second. Following pass 1, a window with no qualifying crossing is assigned zero for +0.1 s size; zero at the interval floor is this missing-event convention, not a measured zero-size crossing. Score window medians per arm on the untouched next highway drive, report its sample count and achieved speeds, and reject the model if any five-statistic ratio is outside ±25%. Do not retune this JSON after seeing that drive.')
  add('## No quiet-bump design yet; shortest useful excitation')
  add('The required closed-loop gate fails, so no quiet-dead-zone module, optimized pulse height/time base or candidate road forecast is justified. The algebraic duty correction can still be stated without tuning: for requested linear-equivalent command q and pulse height H above deadband d, duty = |q|/(H−d), capped at 1, if q means torque on the zero-deadband linear map. With H=0.06 and d=0.028, old duty |q|/0.06 supplies only 0.032/0.06 = **53%** of that intended torque. If q instead denotes an existing deadband-bearing actuator request, duty = max(|q|−d,0)/(H−d). These are different request semantics. EPS lag, slew, stiction and finite pulse duration make either static formula insufficient for selecting a felt bump. Whether to low-pass first remains an experiment, not a deployed answer.')
  add('**Minimum informative run: one complete existing Odyssey suite at steady 40 mph and one at 55–60 mph**, each on a quiet straight stretch with normal driver supervision and per-suite arming. The suite already provides frozen baseline torque holds with bounded hold correction, 0 / 3 / 4.5 / 6 / 8% added command, both signs, 0.5 and 1.0 s widths, with left-first and right-first repetitions. Do not replace this with S5 oscillation. The code remains unchanged; actual applied CAN command, not nominal injected amplitude, is the identification input.')
  add(mdtable(['Per speed', 'Pulses/holds', 'Duration', 'What it identifies'], [
    ['0.5 s, left-first and right-first', '20 total: each of five levels twice each sign', '60 s action time', 'EPS onset/lag, reversal/breakaway, fast impulse response'],
    ['1.0 s, left-first and right-first', '20 total: each of five levels twice each sign', '70 s action time', 'sustained wheel response vs short pulse, aligning load/viscous effects'],
    ['Four steady-speed readiness periods', '2 s each', '8 s', 'reject acceleration/transient starts'],
    ['Both speed plateaus', '80 total holds, including 16 zero-input shams', '276 s = 4 min 36 s minimum', 'speed dependence and direction asymmetry']]))
  add('The holds last 1.5 s for short pulses and 2.0 s for long pulses; 1.5 s normal lane keeping between holds recentres the van. Existing `MAX_HOLD_CORRECTION=0.1` can clip a nominal freeze: retain the actual correction and mark clipped/aborted holds rather than treating them as independent excitation. Identify EPS magnitude lag from native CAN timestamps; estimate wheel motion with the rate signal plus quantized angle; compare matched zero-pulse holds. Fit measured wheel-to-yaw jointly with all normal turns between holds. Low-speed fade and full-cap behavior are not pinned down by this 40–60 mph run and are not needed to optimize a small highway bump.')
  add('This is the shortest **existing-suite** two-speed design with repeats and shams, not a guarantee of closing all 0.3–3-times-per-second bins. A single 1.5–2 s hold cannot support a settled 0.3-times-per-second sinusoid, so no long frozen-lane-control sine is specified for a public road. Pulses have broadband energy; pool randomized-sign/repeat impulse responses and use injected pulse as an instrument to reduce feedback bias. Preserve timing and score achieved coherence, directional repeatability and held-out wheel trajectories. If 8% pulses remain below the wheel-response noise, do not escalate automatically: report that those parameters remain unidentified. After the two-speed training run, require a separate stock highway drive as validation; an additional 138 s suite at a previously tested speed is the shortest same-protocol holdout if ordinary driving remains under-excited. Occupants/seats/cargo, tyre pressure/model, road surface and wind should be recorded for each plateau.')
  add('## Verification and exact continuation')
  add('Eight checks pass under the repository unittest runner: four retained pass-1 mechanics checks plus independent tyre time/frequency agreement, stable/sign-symmetric bristles, plant zero equilibrium/signed response, and EPS deadband/cap/fade. Ruff passes over the entire shake directory; the static plot was visually checked for legible labels and paired acceleration/force axes. No fresh reviewer agents were spawned because this is a research-worker assignment; three self-review passes checked equations/units, data split/reversal/coverage, and runtime/forecast provenance. Fixes included the NumPy batched-solve shape, stable analytic bristle integration and production smoothing-delay handling. Tests prove code invariants; they do not override failed road gates.')
  add('Reproduce with `PYTHONPATH=$PWD:<assigned scratch>/shake/site .venv/bin/python -m tools.odyssey_sim.shake.identify2`, then `...loop2`, `...column_check2`, and `...report2`. Full fitting uses seeded global searches and 40 cluster/window bootstrap fits; running it replaces pass-2 fit files. **Do not rerun the forecast writer after opening the next drive.** Keep its current hash as the pre-drive lock. Large plan caches and logs stay in scratch. `summary2.json` consolidates the computed gate counts/effects, while raw per-bin/window/seed evidence remains in the individual JSONs.')
  add('Exact continuation: first investigate why measured-motor forward column trajectories fail despite moderate inverse torque residuals; fit forward wheel trajectories on independent held-pulse training records with correctly handled reversal sign and nuisance road load, retaining the separate tyre-link check. Then rerun these gates before selecting a PWM bump. No seat/head or roll sensor claim is needed to use validated yaw-body motion. Confirm sensor-frame/timing only before making an additional IMU-specific roll claim.')
  add('Sources for the inherited physical structure: [MathWorks bicycle model equations](https://www.mathworks.com/help/driving/ref/bicyclemodel.html) for two-axle force/yaw balance; [Canudas de Wit et al., 1995](https://lup.lub.lu.se/search/ws/files/6363840/8498924.pdf) for dynamic friction with bristle state. Neither source validates this van’s fitted parameters. Plan response evidence is local `report_vsim2.md` and `report_plan.md`; published vehicle specifications and their model-year/load caveats remain in `report_shake.md`.')
  dest = OUT.parent/'reports/report_shake2.md'
  dest.write_text('\n'.join(text))
  save('summary2.json', {'open_loop_cells': open_summary, 'open_loop_pass_bins': good, 'open_loop_covered_bins': total,
                        'open_loop_uncovered_bins': uncovered, 'closed_loop': summaries, 'factorial_effects': effects,
                        'factorial_interaction_gain_model': float(np.median(inter)), 'report_sha256': hashlib.sha256(dest.read_bytes()).hexdigest()})
  print(dest, 'open', good, total, uncovered, 'closed', summaries, flush=True)


if __name__ == '__main__':
  main()
