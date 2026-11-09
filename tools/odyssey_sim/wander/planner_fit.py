"""Descriptive train-only planner regression on calib3 origin geometry."""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import S, align
from tools.odyssey_sim.wander.research import ROUTES


def main():
  xs, ys = {}, {}
  for route in ROUTES:
    d = np.load(S / 'calib3' / f'{route}.npz')
    n, t = d['native'], d['t']
    v, des = align(t, n, 1, 0.15), align(t, n, 8, 0.15)
    g = d['geometry'][:, 1]
    m = (
      (v > 22) & (align(t, n, 7, 0.15) > 0.5) & (align(t, n, 5, 0.15) == 0) & (align(t, n, 6, 0.15) == 0) & (d['probs'][:, 1] > 0.6) & (d['probs'][:, 2] > 0.6)
    )
    m &= np.isfinite(np.c_[g, v, des]).all(axis=1)
    cached = np.load(S / 'calib' / f'{route}_aligned.npz')
    probe = align(t, np.c_[cached['t'], cached['probe']], 1, 0.15)
    m &= np.isfinite(probe) & (abs(probe) < 0.001)
    xs[route] = np.c_[g[m, 0], v[m] * g[m, 1], np.ones(m.sum())]
    ys[route] = des[m] - g[m, 2] * v[m] ** 2
  train = ['0000018c', '0000019b', '000001a7']
  coef = np.linalg.lstsq(np.concatenate([xs[k] for k in train]), np.concatenate([ys[k] for k in train]), rcond=None)[0]
  result = {'train': train, 'features': ['offset', 'speed*heading', 'intercept'], 'coefficients': coef.tolist(), 'routes': {}}
  for route in ROUTES:
    err = xs[route] @ coef - ys[route]
    result['routes'][route] = {'n': len(err), 'r2': float(1 - np.var(err) / np.var(ys[route])), 'rmse': float(np.sqrt(np.mean(err**2)))}
  (S / 'wander/native_planner_fit.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
