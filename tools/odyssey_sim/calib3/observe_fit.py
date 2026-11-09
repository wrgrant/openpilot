"""Fit only training camera-observer settings; score blind final-10-second prediction."""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import TRAIN, TEST
from tools.odyssey_sim.calib3.audit import OUT, records, pieces
from tools.odyssey_sim.calib3.observer import LaneObserver


def trial(d, r, variant, params):
  e, h, k = d['g'][r, variant].T
  road = d['v'][r] ** 2 * k
  obs = LaneObserver(e[0], d['v'][r[0]] * np.tan(h[0]), **params)
  prediction = []
  for i in range(len(r)):
    visible = i < len(r) - 100
    x = obs.step(road[i], d['body'][r[i]], e[i] if visible else None, d['v'][r[i]] * np.tan(h[i]) if visible else None)
    prediction.append(x)
  prediction = np.array(prediction)
  error = prediction[-100:, 0] - e[-100:]
  return {
    'dropout_rmse_m': float(np.sqrt(np.mean(error**2))),
    'dropout_end_error_m': float(error[-1]),
    'posterior_rmse_m': float(np.sqrt(np.mean((prediction[100:-100, 0] - e[100:-100]) ** 2))),
    'bias_rms': float(np.sqrt(np.mean(prediction[:, 2] ** 2))),
  }


def main():
  audit = json.loads((OUT / 'closure.json').read_text())
  variant = audit['selection']['variant']
  ds = {k: records(k) for k in TRAIN + TEST}
  train = []
  for k in TRAIN:
    rr = list(pieces(ds[k]))
    train.extend((k, rr[j]) for j in np.unique(np.linspace(0, len(rr) - 1, min(3, len(rr))).astype(int)))
  candidates = []
  for sigma in (0.03, 0.1, 0.3):
    for q in (0.005, 0.05):
      for retention in (0.99, 1.0):
        p = {'camera_sigma': sigma, 'heading_sigma': 0.3, 'bias_walk': q, 'bias_retention': retention}
        scores = [trial(ds[k], r, variant, p)['dropout_rmse_m'] for k, r in train]
        candidates.append({'params': p, 'train_median_dropout_rmse_m': float(np.median(scores))})
  best = min(candidates, key=lambda c: c['train_median_dropout_rmse_m'])
  rows = []
  for k, d in ds.items():
    for j, r in enumerate(pieces(d)):
      rows.append(dict(route=k, window=j, split='train' if k in TRAIN else 'heldout', **trial(d, r, variant, best['params'])))
  result = {'selection': best, 'candidates': candidates, 'rows': rows, 'ranking_authorized': False}
  (OUT / 'observer.json').write_text(json.dumps(result, indent=2))
  print(json.dumps(best), flush=True)
  for key in TRAIN + TEST:
    z = [x for x in rows if x['route'] == key]
    print(key, len(z), {c: float(np.median([x[c] for x in z])) for c in ('dropout_rmse_m', 'posterior_rmse_m', 'bias_rms')})


if __name__ == '__main__':
  main()
