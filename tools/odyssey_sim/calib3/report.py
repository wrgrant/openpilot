"""Render the calibration receipt from saved numerical artifacts."""

import hashlib
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
from tools.odyssey_sim.calib.identify import S, TRAIN, TEST
from tools.odyssey_sim.calib3.extract import OUT


def median(rows, key):
  return float(np.median([r[key] for r in rows])) if rows else None


def fmt(x):
  return 'unavailable' if x is None else f'{x:.4f}'


def main():
  spectral = json.loads((OUT / 'spectral.json').read_text())
  fusion = json.loads((OUT / 'fusion.json').read_text())
  observer = json.loads((OUT / 'observer.json').read_text())
  rev = json.loads((OUT / 'reversals.json').read_text())
  gate = json.loads((OUT / 'evaluation_g7.json').read_text())
  g5 = json.loads((OUT / 'evaluation_g5.json').read_text())
  lines = [
    '# Odyssey simulator calibration pass 3',
    '',
    '## Plain-language result',
    '',
    (
      'The lane loop is **not calibrated well enough to rank controllers**. Reconstructing the lane at the vehicle origin '
      + 'and combining its heading change with yaw improves the road estimate, but the remaining error is comparable to the '
      + 'wander William wants to remove. A camera-corrected observer looks accurate while it receives camera updates and '
      + 'drifts badly when those updates are withheld. Using that apparent accuracy to approve a controller would hide the '
      + 'error.'
    ),
    '',
    (
      'The torque-reversal discrepancy has a concrete partial explanation: keeping the real controller inputs at 100 Hz '
      + 'and preserving its recorded slow integral reproduces the observed reversals closely. Holding all inputs at 10 Hz '
      + 'loses substantial reversal activity. Delay jitter and torque-parameter updates do not explain it in the sampled '
      + 'windows. Arbitrary sensor-noise injection is not justified.'
    ),
    '',
    (
      'No stock/NNLC/van-model/smoothing ranking or road-setting recommendation is issued. No device, commit, stage, push, '
      + 'branch switch or older worker source/cache was touched.'
    ),
    '',
    '## Data and validation contract',
    '',
    (
      f'Train: {", ".join(TRAIN)}. Held out: {", ".join(TEST)}. All eight routes were extracted from file-path LogReader '
      + 'inputs. Full time-indexed model position, velocity, orientation, orientationRate, acceleration; road edges; lane '
      + 'confidence/uncertainty; six origin fits; extrinsics; frame timing; native control/sensor/tuning channels are cached '
      + f'in `{OUT}`.'
    ),
    '',
    (
      'Origin variants are quadratic/cubic polynomials over 15/25/50 m. Curvature is 2*c2/(1+slope²)^(3/2), heading '
      + 'atan(slope), offset intercept at x=0. The model source applies extrinsics in its image warp before producing '
      + 'calibrated-frame geometry (`openpilot/selfdrive/modeld/modeld.py`, `fill_model_msg.py`). A second pitch/yaw '
      + 'transform would duplicate that calibration. Camera height is not independently identifiable from these '
      + 'already-inferred metric outputs; no ad hoc height correction was fitted.'
    ),
    '',
    (
      'The spectral audit matches pass 2: detrend each channel, retain 1/20–1/7 Hz Fourier bins, differentiate spectrally, '
      + 'compare offset second derivative with road minus body acceleration, and convert residual acceleration back to '
      + 'implied wander displacement. Windows and fitted origin offsets differ from pass 2, so differences are not a pure '
      + 'causal estimate of improvement. The separate padded Butterworth/interior-derivative audit is a boundary-method '
      + 'sensitivity check, not the headline result.'
    ),
    '',
    (
      'Strict geometry audit requires every sample autonomous, no override/blinker, >5 m/s and both inner-lane '
      + 'probabilities >0.5. It is stricter than the behavior evaluator’s >80% lane-visibility window rule. For 1aa/1ab, 30 '
      + 's geometry windows are reported as diagnostics, not sufficient 7–20 s wander validation. No confident continuous 60 '
      + 's windows exist there under this mask.'
    ),
    '',
    '## Generation log',
    '',
    '|Generation|Experiment and result|Disposition|',
    '|---|---|---|',
    ('|G0|Read pass-2 measured facts/code and 0/40 behavior gate; retained route split, plant v2 and real V0/NNLC.|Baseline reproduced from saved artifacts.|'),
    (
      '|G1|Six origin geometries × five temporal shifts. Training selects quadratic 25 m, −0.3 s spatial-curvature shift. '
      + 'Native frame EOF-to-message median is 31–42 ms across routes.|Origin curvature alone still fails closure. Fitted '
      + '−0.3 s is not a measured camera latency; it can absorb preview/geometry/model effects.|'
    ),
    (
      '|G2|Causal [offset, offset velocity, curvature bias] observer. Twelve train-only settings; hide offset/heading for '
      + 'final 10 s.|Reject as independent simulator; good posterior agreement depends on assimilation.|'
    ),
    (
      '|G3|World-road turning from temporal lane heading plus recorded yaw, optionally subtract model lateral velocity. '
      + 'Butterworth sweep selects cubic 15 m, +0.3 s and model-vy subtraction; spectral sweep selects quadratic 25 m, zero '
      + 'shift, no vy subtraction.|Method-sensitive parameters; heading transport helps but does not close.|'
    ),
    (
      '|G4a|Native-input real-controller replay initialized only at window start; 10 s warmup.|Slow integral mismatch can '
      + 'shift zero crossings despite high command correlation. Preserved in reversals_free_integrator.json; not evidence '
      + 'that sensors are wrong.|'
    ),
    (
      '|G4b|Forensic replay clamps integral to the recorded value every frame, then ablates bandwidth/futures/tuning/delay '
      + 'on identical recorded motion.|Explains a substantial bandwidth component. This intervention is not a free-running '
      + 'controller simulation.|'
    ),
    (
      '|G5|Heading-transport road, real controllers, local NNLC future perturbation; same metric/tolerance contract as '
      + f'pass 2.|{sum(x["pass"] for x in g5["rows"])}/{len(g5["rows"])} behavior rows pass.|'
    ),
    (
      '|G6|Train-only linear fusion of spatial road-minus-body, d(v*lane-heading)/dt, d(model-vy)/dt. Coefficients '
      + f'{fusion["coefficients_spatial_transport_model_vy"]}. Fits training 30 s windows only.|Reduces error, still fails '
      + 'independent closure. No held-out tuning.|'
    ),
    (
      '|G7|Fusion road plus recorded initial controller integral, real controller loop, simulated steering/body feedback, '
      + f'local consistent NNLC future correction.|{sum(x["pass"] for x in gate["rows"])}/{len(gate["rows"])} behavior rows '
      + 'pass; rankings remain unauthorized.|'
    ),
    '',
    (
      'The NNLC future correction adds the simulated request change to model acceleration.y with a 1.5 s exponential '
      + 'horizon fade; recorded future roll/pitch remain exogenous. This removes the exact unchanged-future mismatch from '
      + 'pass 2 but is an explicit approximate planner surrogate, not a validated regeneration of modelV2. Full '
      + 'position/velocity/orientation arrays were preserved for stronger future reconstruction; they were not silently '
      + 'treated as independent road ground truth.'
    ),
    '',
    '## Closure: matched spectral audit, 60 s windows',
    '',
    (
      '|Route / split|Windows|Spatial residual m/s²|Heading-transport residual m/s²|Lane second-derivative RMS '
      + 'm/s²|Transport implied wander error cm|Observed wander cm|'
    ),
    '|---|---:|---:|---:|---:|---:|---:|',
  ]
  for key in TRAIN + TEST:
    a = [x for x in spectral['rows'] if x['route'] == key and x['seconds'] == 60 and x['mode'] == 'spatial']
    b = [x for x in spectral['rows'] if x['route'] == key and x['seconds'] == 60 and x['mode'] == 'transport']
    split = 'train' if key in TRAIN else 'heldout'
    values = [f'{key} / {split}', str(len(a)), fmt(median(a, 'accel_rmse'))]
    values += [fmt(median(b, c)) for c in ('accel_rmse', 'lane_dd_rms', 'equivalent_wander_error_cm', 'observed_wander_cm')]
    lines.append('|' + '|'.join(values) + '|')
  lines += [
    '',
    '## Fusion closure, 30 s diagnostics',
    '',
    '|Route|Windows|Acceleration residual m/s²|Lane second-derivative RMS m/s²|Implied wander error cm|Observed wander cm|',
    '|---|---:|---:|---:|---:|---:|',
  ]
  for key in TRAIN + TEST:
    rows = [x for x in fusion['rows'] if x['route'] == key and x['seconds'] == 30]
    columns = ('accel_rmse', 'lane_dd_rms', 'equivalent_wander_error_cm', 'observed_wander_cm')
    values = [key, str(len(rows))] + [fmt(median(rows, c)) for c in columns]
    lines.append('|' + '|'.join(values) + '|')
  lines += [
    '',
    (
      'Fusion reduces error without using offset as a forcing input during held-out prediction. Its residual is still '
      + 'material relative to observed wander. Spatial and heading/model-motion observations are correlated model outputs, '
      + 'not independent sensors; fitting a fusion does not establish road ground truth. The variable-speed correction a/v × '
      + 'offset_rate was included in the interior audit; spectral tables retain the pass-2 road-minus-body definition for '
      + 'comparability.'
    ),
    '',
    '## Observer dropout validation',
    '',
    '|Route|60 s windows|Median camera-assisted posterior offset RMSE m|Median final-10-second prediction RMSE m|',
    '|---|---:|---:|---:|',
  ]
  for key in TRAIN + TEST:
    rows = [x for x in observer['rows'] if x['route'] == key]
    lines.append(f'|{key}|{len(rows)}|{fmt(median(rows, "posterior_rmse_m"))}|{fmt(median(rows, "dropout_rmse_m"))}|')
  lines += [
    '',
    (
      f'Training-selected settings: `{observer["selection"]["params"]}`. No offset or heading update is allowed in the '
      + 'scored final 10 seconds; spatial camera curvature continues as road input. Constant bias prediction is not reliable '
      + 'enough to support free-running wander simulation. This dropout test is deliberately stricter than posterior '
      + 'consistency and reveals why small observer residuals alone are insufficient.'
    ),
    '',
    '## Behavior calibration gate (G7)',
    '',
    (
      'Same 30% per-window tolerances, metric floors, native 100 Hz reversal definition (>0.01 adjacent crossing), '
      + 'saturation definition, warmup and wander requirements as pass 2. A missing wander metric fails. Values below are '
      + 'duration-weighted recorded / simulated aggregates; a group fails if any constituent row fails.'
    ),
    '',
    '|Route|mph|Mode|Rows|Cost|Reversals/s|Saturation %|Wander cm|Pass rows|',
    '|---|---|---|---:|---:|---:|---:|---:|---:|',
  ]
  groups = defaultdict(list)
  for row in gate['rows']:
    groups[(row['route'][:8], row['band'], row['mode'])].append(row)
  for group, rows in sorted(groups.items()):
    cells = []
    for key in ('cost', 'reversals_s', 'saturation_pct', 'wander_cm'):
      a = [x for x in rows if key in x['recorded']]
      if not a:
        cells.append('unavailable')
        continue
      weight = np.array([x['n'] for x in a])
      values = [float(np.average([x[side][key] for x in a], weights=weight)) for side in ('recorded', 'simulated')]
      cells.append(f'{values[0]:.3f} / {values[1]:.3f}')
    lines.append('|' + '|'.join(map(str, [*group, len(rows), *cells, sum(x['pass'] for x in rows)])) + '|')
  lines += [
    '',
    (
      f'Full G7 gate: **{sum(x["pass"] for x in gate["rows"])}/{len(gate["rows"])} rows pass. ranking_authorized=false.** '
      + 'No candidate rankings are justified. Cost/saturation agreement in some rows does not cure closure, reversal or '
      + 'missing-wander failures.'
    ),
    '',
    '## Reversal-gap explanation',
    '',
    (
      'Eight representative windows (three 19c, three 1ab, two 1aa), identical recorded motion for all ablations, scored '
      + 'after 10 s warmup. The native replay uses per-frame recorded integral as an explicit forensic control. Means below '
      + 'weight each window equally; exact window rows are in reversals.json.'
    ),
    '',
    '|Input mode|Mean reversals/s|Median command RMSE vs recorded|',
    '|---|---:|---:|',
    f'|Recorded|{np.mean([x["recorded"] for x in rev]):.4f}|—|',
  ]
  for mode in rev[0]['modes']:
    lines.append(f'|{mode}|{np.mean([x["modes"][mode]["reversals_s"] for x in rev]):.4f}|{np.median([x["modes"][mode]["rmse"] for x in rev]):.5f}|')
  lines += [
    '',
    (
      'Interpretation: holding target and sensed vehicle motion at 10 Hz removes high-frequency crossings; holding future '
      + 'arrays constant also changes them. Logged tuning is essentially constant on these short windows and freezing delay '
      + 'changes neither mean reversal rate nor the conclusion. Real steering angle already has 0.1° quantization; pass 2 '
      + 'quantizes synthetic feedback, so the mere existence of quantization is not the missing mechanism. The plant '
      + 'produces smooth feedback and loses real sub-frame disturbances; their independent contribution is not yet '
      + 'identified. No arbitrary sensor-noise scale was tuned to force agreement. Raw crossing count is also sensitive to '
      + 'slow integral bias and is not a perceptibility metric; use the separate perception worker for that question.'
    ),
    '',
    '## Low-speed coverage',
    '',
    (
      'Counts below require the entire 10 s fragment inside the band and strict continuous high-confidence autonomy. Short '
      + 'fragments can calibrate fast steering behavior, not a 7–20 s wander oscillation. Values under ~11.2 mph remain '
      + 'excluded by the inherited >5 m/s plant gate.'
    ),
    '',
    '|Held-out route|10–20 mph 10 s fragments|20–35 mph 10 s fragments|10–35 mph 60 s fragments|',
    '|---|---:|---:|---:|',
  ]
  for key in TEST:

    def count(seconds, band, key=key):
      return sum(x['windows'] for x in spectral['coverage'] if x['route'] == key and x['seconds'] == seconds and x['band'] == band)

    lines.append(f'|{key}|{count(10, "10-20")}|{count(10, "20-35")}|{count(60, "10-20") + count(60, "20-35")}|')
  lines += [
    '',
    (
      'No 10–35 mph closed-loop/wander qualification follows from these fragments. Coverage for all training/held-out '
      + 'routes and 10/20/30/60 s durations is retained in spectral.json.'
    ),
    '',
    '## Reuse, checks and exact next step',
    '',
    (
      'The wander worker can import `tools.odyssey_sim.calib3.observer.LaneObserver`, `geometry.origin`, and '
      + '`lane.records`. Reuse the APIs and artifacts, but do not assume the observer is validated. One-command evaluator: '
      + '`PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib3.evaluate --output evaluation_g7.json`; the original '
      + '`--model` and `--controller module:factory` contracts remain. See calib3/README.md for full extraction, fit and '
      + 'validation commands. scipy is required only by offline fitting/auditing, using the supplied wander venv; step APIs '
      + 'and evaluator use the repo Python.'
    ),
    '',
    (
      'Validation: five analytic/observer/future tests pass; ruff check passes; compileall passes; all eight extraction '
      + 'artifacts exist; both G5 and G7 full held-out evaluations completed. Geometry and bias-state observer have physical '
      + 'analytic checks, including a blind prediction test that prevents hiding drift behind camera assimilation.'
    ),
    '',
    (
      '**Recommended next execution slice:** fit a causal joint offset/heading observation model with explicit slowly '
      + 'varying camera heading bias on the training routes, and validate offset prediction in withheld 10 s blocks on '
      + '1aa/1ab as well as 19c. Compare against the present fusion and bias observer with identical windows. Require an '
      + 'independent displacement residual below 30% of observed 7–20 s wander (with a 1 cm floor) before using any lane '
      + 'ranking. Preserve the actual 100 Hz controller-input bandwidth and initialize the slow integral from the recorded '
      + 'common starting state. A posterior camera fit, an offset-derived road acceleration, or a tuned stochastic noise '
      + 'injection cannot substitute for that withheld-observation check. The current fusion residual means further '
      + 'controller sweeps would mainly optimize the simulator error.'
    ),
    '',
    '## Workspace receipt',
    '',
    (
      'Checkout `/Users/williamgrant/Code/odyssey-pilot`, branch `odyssey-sim`. Source writes: only '
      + '`tools/odyssey_sim/calib3/`. Research writes: requested scratch directory. Existing concurrent worker changes were '
      + 'not staged, edited or reverted. No external actions or device access. All delivered code and intermediate numerical '
      + 'artifacts remain uncommitted for the lead. Source hashes are in calib3/provenance.json.'
    ),
  ]
  (S / 'report_calib3.md').write_text('\n'.join(lines) + '\n')
  paths = list(Path('tools/odyssey_sim/calib3').glob('*.py')) + [Path('ODYSSEY.md'), Path('tools/odyssey_sim/calib/evaluate.py'), S / 'calib' / 'plant_v2.json']
  (OUT / 'provenance.json').write_text(json.dumps({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2))
  print('Wrote', S / 'report_calib3.md')


if __name__ == '__main__':
  main()
