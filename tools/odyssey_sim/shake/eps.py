"""Identify unsigned CAN motor response separately from wheel/body fitting."""
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter
from tools.odyssey_sim.shake.extract import SCRATCH
from tools.odyssey_sim.shake.analyze import OUT


def fit_eps():
  d = np.load(SCRATCH/'000001ac.npz')
  t = np.arange(d['t'][0]+1, d['t'][-1]-1, .01)
  u = np.interp(t, d['can_cmd_raw'][:, 0], abs(d['can_cmd_raw'][:, 1])/4096)
  y = np.interp(t, d['motor_raw'][:, 0], d['motor_raw'][:, 1])
  v = np.interp(t, d['t'], d['v'])
  active = np.interp(t, d['t'], d['lat']) > .5
  mask = active & (v > 11.17)
  x = np.array([0., .028, .1, .2, .3, .4, .5, .6, .7, .8, 1.])
  knots = np.array([0., 0., 6., 14., 25., 39., 50., 57., 63., 67., 75.])

  def pred(p):
    alpha = -np.expm1(-.01/p[0])
    return lfilter([alpha], [1, -(1-alpha)], np.interp(u, x, knots)*p[1])

  result = least_squares(lambda p: (pred(p)-y)[mask][::5], [.03, 1.], bounds=([.001, .5], [.3, 1.5]), loss='soft_l1')
  stats = {'training_route': '1ac', 'tau_seconds': float(result.x[0]), 'map_scale': float(result.x[1]),
           'raw_motor_rmse': float(np.sqrt(np.mean((pred(result.x)[mask]-y[mask])**2))),
           'knots_command': x.tolist(), 'knots_motor': (knots*result.x[1]).tolist(), 'bin_medians': []}
  for lo, hi in zip(x[:-1], x[1:], strict=True):
    m = mask & (u >= lo) & (u < hi)
    stats['bin_medians'].append({'lo': float(lo), 'hi': float(hi), 'n': int(m.sum()),
                               'motor_median': float(np.median(y[m])) if m.any() else None})
  (OUT/'eps.json').write_text(json.dumps(stats, indent=2))
  print(stats)


if __name__ == '__main__':
  fit_eps()
