"""Render measured results and failed calibration gates without ranking candidates."""
# Report prose is wrapped for editing; adjacent literals form one paragraph.
# ruff: noqa: ISC002, ISC004, E501
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from tools.odyssey_sim.calib.identify import S, TEST


def load(name):
  return json.loads((S/'calib'/name).read_text())


def grouped(rows):
  out = {}
  for row in rows:
    out.setdefault((row['route'][:8],row['band'],row['mode']),[]).append(row)
  return out


def average(rows, source, metric):
  z = [r for r in rows if metric in r[source]]
  if not z:
    return None
  return float(np.average([r[source][metric] for r in z],weights=[r['n'] for r in z]))


def pair(rows,k):
  a,b = average(rows,'recorded',k),average(rows,'simulated',k)
  return 'unavailable' if a is None else f'{a:.3f} / {b:.3f}'


def ci(rows, metric):
  if len(rows)<2:
    return f'{rows[0]["simulated"][metric]:.3f} [one window]'
  values = np.array([r['simulated'][metric] for r in rows])
  boot = np.random.default_rng(20261109).choice(values,(1000,len(values)),replace=True).mean(axis=1)
  a,b = np.percentile(boot,[2.5,97.5])
  return f'{values.mean():.3f} [{a:.3f}, {b:.3f}]'


def main():
  fit,body,checks,evaluation = map(load,['identification.json','body_fit.json','checks.json','evaluation.json'])
  p = fit['steering_parameters']
  rows = evaluation['rows']
  lines = ['# Odyssey simulator calibration, pass 2', '',
    '**Result: the body channel and a better plant are validated; the full simulator is not calibrated and cannot rank controllers.** '
    'Keep the current van configuration. No controller or add-on is promoted by this analysis.', '',
    'The van provides a usable motion signal after all: speed × calibrated yaw rate. '
    'The new plant predicts ten seconds of steering/body motion to roughly 0.08–0.10 m/s² RMS on unseen routes. '
    'That accuracy does not make it a trustworthy lane simulator: small acceleration/road errors accumulate into large lane-position errors, '
    'and the real controller still produces different reversal rates in simulation. A lower simulated cost is not evidence of better road steering.', '',
    '## Body channel and frames', '',
    '`controlsd.py` publishes calibrated angular velocity and orientation into carControl. '
    '`carControl.angularVelocity[2] × carState.vEgo` is positive with openpilot lateral acceleration and negative with Honda steering angle and torqueState.output. '
    'Roll/pitch angular-velocity axes do not show that relationship. This is yaw/curvature acceleration, not a gravity-bearing accelerometer reading; '
    'rapid sideslip transients and longitudinal-speed approximations remain limitations. SensorEvents fallback was unnecessary.', '',
    '|Held-out route|Steady yaw/steering correlation|Roll std (rad)|Body samples finite|Raw-wheel steady-fit RMSE (m/s²)|',
    '|---|---:|---:|---:|---:|']
  for k in TEST:
    d=fit['routes'][k]
    lines.append(f"|{k}|{body['axis_correlations_roll_pitch_yaw'][k][2]:.4f}|{d['roll_std']:.4f}|{d['body_finite_pct']:.2f}%|{body['heldout_rmse'][k]:.4f}|")
  lines += ['', 'Steady means active, no driver override, |driver torque| <500, speed >5 m/s, no blinkers, '
    '|steering rate| <1 deg/s, and |logged acceleration| >0.15 m/s². '
    'Supplemental channels are timestamp-aligned as-of with 150 ms maximum age (500 ms for vehicleParameters); no interpolation across gaps. '
    'The new extractor orders segments numerically; alignment asserts sorted timestamps. Original extraction caches are read-only.', '',
    f"A training-route-only raw wheel fit uses `wheel = -radians(angle-offset) × v²/(steerRatio × 2.9)` and "
    f"`body = wheel/(1 + K×v²) + R×roll + bias`: K={body['understeer_coefficient_per_mps2']:.7f} s²/m², "
    f"R={body['roll_gain']:.3f} m/s²/rad, bias={body['bias']:.4f} m/s². "
    'This captures speed-dependent understeer, with decreasing wheel-to-body gain at speed. '
    'Roll and road curvature are correlated, so R is an empirical correction rather than a causal crown measurement. '
    'Also, vehicleParameters is learned from motion: agreement with torqueState actual is not independent validation of every VehicleModel parameter. '
    'The raw wheel/yaw fit provides the additional check.', '',
    '## Plant v2', '',
    'Files: `scratchpad/calib/plant_v2.json`, `identification.json`, `body_fit.json`, and `checks.json`; API `tools/odyssey_sim/calib/plant.py`.', '',
    'Command → steering-equivalent acceleration uses deadband 0.028, speed-interpolated gain/friction, first-order lag, measured roll, and bias. '
    'Steering-equivalent acceleration is the VehicleModel quantity observed by V0, rather than raw wheel angle. '
    'The yaw-body stage is identified separately from logged wheel-equivalent acceleration and then validated in combined free run. '
    'The API adds the Honda command slew limit (3 normalized units/s). No observations reset it during a prediction.', '',
    f'Gain at 10/20/30 m/s: {p[:3]}; friction in acceleration units: {p[4:7]}; lag {p[3]:.3f} s; '
    f'roll gain {p[7]:.3f} m/s²/rad; bias {p[8]:.4f} m/s². '
    'Speed-dependent friction is retained. These ordinary closed-loop logs do not identify separate static and kinetic friction reliably; '
    'no extra breakaway parameter is claimed.', '',
    f"Body-stage gain at 10/20/30 m/s: {fit['body_parameters'][:3]}. "
    'Its lag hits the 5 ms lower bound. At 10 Hz this means no additional lag is resolved; it does not establish a physical 5 ms body lag. '
    'The body-stage roll term is only a residual correction; measured roll already enters the steering-equivalent state.', '',
    'Training: 174, 18c, 19b, 1a3, 1a7. Held out throughout plant/planner fitting: 19c, 1aa, 1ab. '
    'Nonoverlapping 10 s predictions, 2 s continuously eligible context, one segment, no gaps >160 ms; initial motion and applied command observed once. '
    'Predictions use logged commands/speed/roll, with no subsequent motion feedback. Below 5 m/s remains outside the fit.', '',
    '|Held-out route|10 s windows|Steering RMSE, fit / scalar API|Body RMSE, fit / scalar API|Body given logged steering RMSE|',
    '|---|---:|---:|---:|---:|']
  for k in TEST:
    d,a=fit['routes'][k],checks['api'][k]
    lines.append(f"|{k}|{d['windows']}|{d['steering_10s_rmse']:.4f} / {a['steering_10s_rmse']:.4f}|"
                 f"{d['body_10s_rmse']:.4f} / {a['body_10s_rmse']:.4f}|{d['body_given_logged_steer_rmse']:.4f}|")
  lines += ['', 'All RMSEs are m/s², pooled over every forecast sample after initialization, not just the endpoint. '
    'Scalar API is independently exercised at 100 Hz and includes slew; the optimizer uses a vectorized 10 Hz approximation. '
    'The differences are small on held-out data. The finite/bounded/sign grid passes at 5/10/20/30/35 m/s and commands −1/−.04/0/.04/1.', '',
    '## End-to-end calibration gate', '',
    'The real sunnypilot V0 class and its logged NNLC mode run at 100 Hz. All three held-out route metadata records have NNLC enabled and no NnlcAB setting. '
    'The updated adapter uses per-frame logged torque tuning, learned lateralDelay, vehicleParameters, and calibrated pitch, plus the applied-command slew feedback. '
    'This replaces the previous fixed factor/friction/offset/delay and missing pitch. '
    'Feedback includes the Honda 0.1-degree wheel-angle quantizer; angle-rate feedback remains continuous. '
    'Simulated tracking cost uses the actual VehicleModel measurement from that quantized feedback, matching torqueState semantics. '
    'Logged delay medians are approximately 0.349/0.330/0.322 s for 19c/1aa/1ab; torque factor also differs materially across routes.', '',
    'The logged-plan mode replays the recorded request/future model trajectory and closes the steering feedback loop. '
    'The lane-loop mode instead computes request = road + KP×offset + KD×sideways speed and integrates road minus body acceleration. '
    'Both use real V0/NNLC. NNLC future arrays remain recorded in lane-loop mode: a newly simulated lane request does not have a corresponding regenerated modelV2 future. '
    'That mismatch is an explicit failure of the planner surrogate, not a validated simulation of modeld.', '',
    f"Refitting the lane law on training data gives KP/KD/bias = {evaluation['planner_fit']}, R²={evaluation['planner_train_r2']:.3f}. "
    'The calibration table uses this training-only refit, including its bias. '
    'We did not tune against held-out wander to force a passing answer.', '',
    'Tolerance: every metric within 30% on each scored window. To avoid division by zero, relative-error denominators have floors '
    'of 0.01 cost units, 0.1 reversals/s, 1 percentage point saturation, and 1 cm wander. '
    'Passing requires lane-wander coverage as well as cost/reversals/saturation. '
    'Cost is 100×mean((torqueState desired − torqueState actual)²); no jerk penalty is mixed into this tracking cost. '
    'Reversals use native 100 Hz torqueState output versus native simulated controller output, >0.01 crossing amplitude, and adjacent eligible samples. '
    'Saturation is |output|≥0.99 rather than the delayed torqueState saturated flag.', '',
    'Windows are continuously autonomous, no driver override or blinker, >5 m/s, reliable lane lines >80%, '
    '60 s with 10 s warmup. Route 1ab has no eligible 60 s window, so it uses 30 s windows (20 s scored). '
    'Wander requires at least 40 uninterrupted scored seconds in the speed band and uses the wander meter’s detrending/7–20 s FFT band at 20 Hz. '
    'A missing wander score fails the gate. There is no below-5-m/s claim; bands with no eligible window are unvalidated.', '',
    'Each cell is **recorded / simulated**, duration-weighted across windows. Individual failures are retained in evaluation.json. '
    '“Fail” means one or more constituent windows misses the full gate, even when an aggregate metric is close.', '',
    '|Route|mph band|Mode|Windows|Tracking cost|Reversals/s|Saturation %|Wander cm|Gate|',
    '|---|---|---|---:|---:|---:|---:|---:|---|']
  for (route,band,mode),rs in sorted(grouped(rows).items()):
    lines.append(f'|{route}|{band}|{mode}|{len(rs)}|'+ '|'.join(pair(rs,k) for k in ['cost','reversals_s','saturation_pct','wander_cm'])+'|'+('PASS' if all(r['pass'] for r in rs) else 'FAIL')+'|')
  lines += ['', f"Full gate: {sum(r['pass'] for r in rows)}/{len(rows)} route/window/band/mode rows pass. "
    f"Ranking authorized: {evaluation['ranking_authorized']}.", '',
    '## Lane sign correction and remaining calibration failure', '',
    'The legacy plant cache negates both lane-centre curvature and lane-centre offset. '
    'Its cached road acceleration correlates −0.94 to −0.95 with the independently validated yaw channel. '
    'Negating cached road and offset makes road curvature positive with body acceleration and offset positive for the lane error '
    'used by request = road + KP×offset + KD×sideways speed. '
    'This correction lives in calib.evaluate.records; the original caches and old simulators remain untouched. '
    'It removes a major false road forcing, improves the planner fit, and sharply reduces the closure residual, '
    'but does not make all metrics agree.', '',
    'A direct closure audit checks `offset_second_derivative = road_acceleration − body_acceleration` in the 7–20 s band. '
    'This must hold before any controller is added. It does not. The camera road estimate is a quadratic lane fit over <50 m, '
    'not independently surveyed road curvature. Camera lane estimates, horizon/lookahead and offset changes need a measurement/geometry model; '
    'double integration treats their mismatch as physical lane drift. Roll/crown residuals and sideslip are additional possible contributors. '
    'The audit cannot separate these causes, but it rules out treating this road/offset pair as a consistent external forcing.', '',
    '|Held-out route|Closure windows|Median acceleration closure residual m/s²|Median observed lane second-derivative RMS m/s²|Equivalent wander error cm|',
    '|---|---:|---:|---:|---:|']
  for key in TEST:
    rs=[r for r in checks['lane_closure'] if r['route'].startswith(key)]
    if rs:
      vals=[np.median([r[k] for r in rs]) for k in ['road_minus_body_vs_lane_dd_rmse','lane_dd_rms','accel_residual_equiv_wander_cm']]
      lines.append(f'|{key}|{len(rs)}|{vals[0]:.4f}|{vals[1]:.4f}|{vals[2]:.1f}|')
    else:
      lines.append(f'|{key}|0|unavailable|unavailable|unavailable|')
  lines += ['', 'The closure audit uses complete 60 s windows and spectral differentiation; boundary/periodicity effects remain. '
    'Even in the logged-plan mode, integrating the imperfect road with simulated body motion produces large offset errors. '
    'It is a diagnostic of the lane reconstruction, not a meaningful prediction of leaving the lane.', '',
    'Coverage: no 10–20 or 20–35 mph band has a qualifying ≥30 s lane-visible autonomous window in any held-out route. '
    '19c also has no 35–50 mph window. These cells are **UNVALIDATED / FAIL**, rather than silently passed. '
    'Low-speed tracking could be tested separately with shorter fragments, but that cannot meet the requested wander gate.', '',
    '## Candidate diagnostics and uncertainty', '',
    '**No stock-vs-NNLC/add-on ranking is issued.** The requested prerequisite fails. '
    'Both van-model artifacts exist and were evaluated on the same plant and logged-plan windows. '
    'The following are simulator diagnostics, with no ordering or deployment recommendation. '
    '19c was part of the candidate author’s training set, so candidate generalization is assessed only on 1aa/1ab here. '
    'V1 uses the runtime bounded 18-input evaluator; V2 uses its author’s 26-input history adapter. '
    'These runs satisfy evaluation of available artifacts, not qualification for the van.', '',
    '|Route / band|Artifact|Sim cost, window mean [bootstrap 95% interval]|Sim reversals/s, mean [95% interval]|',
    '|---|---|---:|---:|']
  for name,file in [('original NNLC','evaluation.json'),('William V1','evaluation_v1.json'),('William V2','evaluation_v2.json')]:
    e=load(file)
    for (route,band,mode),rs in sorted(grouped(e['rows']).items()):
      if route not in ['000001aa','000001ab'] or mode!='logged_plan':
        continue
      lines.append(f'|{route} / {band}|{name}|{ci(rs,"cost")}|{ci(rs,"reversals_s")}|')
  lines += ['', 'Intervals resample whole nonoverlapping windows, 1,000 bootstrap draws, fixed seed 20261109. '
    'Few windows make these intervals weak; “one window” means no defensible interval. '
    'The plant/controller run is deterministic, so repeated identical seeds add no uncertainty information. '
    'These intervals exclude plant identification uncertainty, candidate training uncertainty and planner mismatch; '
    'the latter dominates and prevents rankings. Add-ons were not swept after the calibration gate failed.', '',
    '## One-command hook and reproduction', '',
    'From `~/Code/odyssey-pilot`:', '', '```sh',
    'PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate', '',
    'PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --model "$PWD/openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY_WILLIAM_V1.json" --output evaluation_v1.json', '',
    'PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --controller tools.odyssey_sim.calib.controller:candidate_v2 --output evaluation_v2.json',
    '```', '',
    'An 18-input candidate uses --model. A different controller/input contract uses --controller module:factory; '
    'the factory receives (metadata, Config), returns an object with update(target_accel, measured_accel, state, future_plan), '
    'and returns normalized acceleration-positive requested torque. The adapter also exposes measured_lataccel for scoring its actual feedback. '
    'Every candidate retains the same plant, routes, masks and metrics. '
    'Use a distinct --output to preserve the baseline. The JSON always includes ranking_authorized; candidate improvement never overrides the calibration gate.', '',
    'To rebuild: run calib.extract with repo Python; calib.identify and calib.body_fit with the supplied wander/venv scipy Python and PYTHONPATH=$PWD; '
    'then calib.check, calib.evaluate, candidate commands, and calib.report with repo Python. '
    'Ruff and compileall pass. All entry points have main guards; no multiprocessing is needed.', '',
    '## Limits and exact next step', '',
    'The command-to-steering plant is an aggregate model fitted from feedback-controlled driving, not a persistently excited system-identification experiment. '
    'Parameter trade-offs remain, especially gain/friction/lag/roll. The public Odyssey data is not used to override this van’s fitted parameters. '
    'Manual routes a4/a5 are supplemented but excluded from autonomous fitting. '
    'Driver takeovers, Honda driver-torque safety limits, realistic EPS angle-rate sensing, camera geometry and a regenerated future model/planner remain incomplete. '
    'The simulations use selected continuous lane-visible windows, not the full route population; low-speed parking is unvalidated.', '',
    '**Recommended next step:** reconstruct lane geometry at the vehicle origin (offset, heading, curvature and confidence) from modelV2, '
    'fit its observation dynamics and test the body/road/offset closure on training routes before touching controller rankings. '
    'Require the same closure on 19c/1aa/1ab and enough ≥45–60 s autonomous coverage per speed band. '
    'Then regenerate NNLC future trajectories consistently with simulated lane feedback and pass the current calibration gate. '
    'Until then use replay to compare output behavior and this plant to reject fragile candidates, not choose road settings.', '',
    '## Workspace receipt', '',
    'Checkout `/Users/williamgrant/Code/odyssey-pilot`, branch `odyssey-sim`. '
    'Only new files under tools/odyssey_sim/calib/ and requested scratch artifacts were written. '
    'No commits, staging, pushes, branch switches or device access. Concurrent worker changes were left alone. '
    'Provenance/source hashes are in scratchpad/calib/provenance.json.']
  (S/'report_calib.md').write_text('\n'.join(lines)+'\n')
  paths=list(Path('tools/odyssey_sim/calib').glob('*.py'))
  paths+=list(Path('openpilot/sunnypilot/neural_network_data/neural_network_lateral_control').glob('*WILLIAM*.json'))
  provenance={'branch':subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
              'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
  (S/'calib'/'provenance.json').write_text(json.dumps(provenance,indent=2))
  print(S/'report_calib.md')


if __name__ == '__main__':
  main()
