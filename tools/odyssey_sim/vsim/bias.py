"""Apply the pass-2 natural regression to free-running simulator perception.

Compare its prediction of the measured injected transfer, without asserting that
an instantaneous observational regression identifies a recurrent nonlinear model.
"""

import argparse
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.vsim.transfer import perceive, harmonic, ratio
from tools.odyssey_wander import bandpass


def design(rows):
  offset, heading, road = perceive(rows, horizon=25)
  v = np.array([r['speed'] for r in rows])
  x = np.column_stack([bandpass(z) for z in (offset, heading, road * v * v)])
  y = bandpass(np.array([r['curvature'] for r in rows]) * v * v)
  return x, y


def fit(x, y):
  coefficients = np.linalg.lstsq(x, y, rcond=None)[0]
  return {
    'coefficients': coefficients.tolist(),
    'r2': float(1 - np.var(y - x @ coefficients) / np.var(y)),
    'condition': float(np.linalg.cond(x / x.std(axis=0))),
  }


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch', type=Path, required=True)
  args = ap.parse_args()
  root = args.scratch
  evidence = json.loads((root / 'analysis.json').read_text())
  paths = [
    root / f'{window}_{arm}.traces.json'
    for window, arms in evidence['windows'].items()
    if window[:8] in ('000001a3', '000001a7')
    for arm in arms
    if arms[arm]['simulated']['wander_available']
  ]
  arrays = [design(json.loads(p.read_text())) for p in paths]
  if not arrays:
    raise ValueError('no complete free-running windows')
  x, y = [np.concatenate(z) for z in zip(*arrays, strict=True)]
  free_fit = fit(x, y)
  co = free_fit['coefficients']
  differential = []
  for window, arms in evidence['windows'].items():
    if 'stock' not in arms:
      continue
    baseline_rows = {r['frame']: r for r in json.loads((root / f'{window}_stock.traces.json').read_text())}
    for arm in arms:
      if arm == 'stock':
        continue
      rs = json.loads((root / f'{window}_{arm}.traces.json').read_text())
      pairs = [(r, baseline_rows[r['frame']]) for r in rs if r['frame'] in baseline_rows]
      rs, bs = zip(*pairs, strict=True)
      dx, by = design(rs)
      bx, yy = design(bs)
      true = np.column_stack([bandpass(np.array([r[k] - b[k] for r, b in pairs])) for k in ('dy', 'dpsi')])
      differential.append({'window': window, 'arm': arm, 'true_pose_fit': fit(true, by - yy), 'measured_perception_fit': fit(dx - bx, by - yy)})
  baseline = {r['frame']: r for r in json.loads((root.parent / 'vsim/g6.traces.json').read_text())}
  comparisons = []
  for path in sorted((root.parent / 'vsim2').glob('sine_*.traces.json')):
    meta = json.loads(path.with_name(path.name.replace('.traces', '')).read_text())
    pairs = [(r, baseline[r['frame']]) for r in json.loads(path.read_text()) if r['frame'] in baseline and r['warm_frames'] >= 200]
    rows, base = zip(*pairs, strict=True)
    off, head, _ = perceive(rows, horizon=25)
    boff, bhead, _ = perceive(base, horizon=25)
    t = np.array([r['t'] for r in rows])
    period = meta['injection']['period']
    actual = np.array([(r['curvature'] - b['curvature']) * r['speed'] ** 2 for r, b in pairs])
    predicted = co[0] * (off - boff) + co[1] * (head - bhead)
    hactual = harmonic(t, actual, period)
    comparisons.append({'period_s': period, 'regression_over_injected': ratio(harmonic(t, predicted, period), hactual)})
  # Known linear-response calibration: noise added to the measured regressors,
  # never to their known generating signal. This isolates attenuation from feedback.
  rng = np.random.default_rng(903)
  synthetic_x = rng.normal(size=(100000, 3))
  truth = np.array([0.7, -10, 0.8])
  clean = fit(synthetic_x, synthetic_x @ truth)
  noisy = fit(synthetic_x + rng.normal(size=synthetic_x.shape) * np.sqrt(2), synthetic_x @ truth)
  result = {
    'free_fit': free_fit,
    'complete_free_windows': len(paths),
    'fit_scope': 'Complete training-route free runs with sufficient logged lane visibility; no held-out fit used.',
    'differential_true_vs_perceived': differential,
    'injection_comparison': comparisons,
    'known_linear_clean': clean,
    'known_linear_noise_variance_twice_signal': noisy,
    'caution': (
      'Free video output has no known instantaneous linear response. Failure to recover injected gain also admits feedback confounding, '
      + 'omitted temporal dynamics and correlated road geometry. This experiment alone cannot identify measurement-noise attenuation.'
    ),
  }
  (root / 'bias.json').write_text(json.dumps(result, indent=2))
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
