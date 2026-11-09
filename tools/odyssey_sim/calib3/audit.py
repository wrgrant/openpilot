"""Train-only origin/latency selection and held-out independent Frenet closure."""

import json
import numpy as np
from scipy.signal import butter, sosfiltfilt, savgol_filter
from tools.odyssey_sim.calib.identify import S, TRAIN, TEST
from tools.odyssey_sim.calib3.geometry import VARIANTS

OUT = S / 'calib3'
DT = 0.1


def band(x):
  return sosfiltfilt(butter(3, [1 / 20, 1 / 7], fs=10, btype='bandpass', output='sos'), x, padlen=150)


def derivative(x, order=1):
  return savgol_filter(x, 21, 3, deriv=order, delta=DT)


def records(key):
  z = np.load(OUT / f'{key}.npz')
  b = np.load(S / 'calib' / f'{key}_aligned.npz')
  t = b['t']
  ix = np.maximum(np.searchsorted(z['t'], t, side='right') - 1, 0)
  g = z['geometry'][ix]
  ok = b['ok'] & (t - z['t'][ix] < 0.15) & (z['probs'][ix, 1] > 0.5) & (z['probs'][ix, 2] > 0.5)
  return {
    't': t,
    'v': b['v'],
    'body': b['body'],
    'g': g,
    'ok': ok,
    'action': z['action'][ix] * b['v'] ** 2,
    'vel': z['velocity'][ix, :, 0],
    'yaw': z['orientationRate'][ix, 2, 0],
    'age': z['age'][ix],
    'edges': z['edges'][ix],
    'calibration': z['calibration'][ix],
  }


def pieces(d, seconds=60):
  ix = np.where(d['ok'])[0]
  for r in np.split(ix, np.where((np.diff(ix) > 1) | (np.diff(d['t'][ix]) > 0.16))[0] + 1):
    for i in range(0, len(r) - seconds * 10 + 1, seconds * 10):
      yield r[i : i + seconds * 10]


def closure(d, r, variant, shift=0):
  e, h, k = d['g'][r, variant].T
  v, body, t = (d[c][r] for c in ('v', 'body', 't'))
  road = v * v * k
  road = np.interp(t + shift, t, road)
  ed = derivative(e)
  # Variable speed Frenet term a/v*e_dot is missing from pass 2.
  rhs = road - body + derivative(v) / np.maximum(v, 1) * ed
  sl = slice(100, -100)
  residual = band(rhs - derivative(e, 2))[sl]
  ed_residual = band(v * np.tan(h) - ed)[sl]
  return {
    'accel_rmse': float(np.sqrt(np.mean(residual**2))),
    'lane_dd_rms': float(np.sqrt(np.mean(band(derivative(e, 2))[sl] ** 2))),
    'velocity_rmse': float(np.sqrt(np.mean(ed_residual**2))),
    'lane_ed_rms': float(np.sqrt(np.mean(band(ed)[sl] ** 2))),
    'observed_wander_cm': float(100 * np.std(band(e)[sl])),
    'speed_mph': float(np.mean(v) * 2.23694),
  }


def main():
  ds = {k: records(k) for k in TRAIN + TEST}
  selections = {k: list(pieces(d)) for k, d in ds.items()}
  candidates = []
  for variant in range(len(VARIANTS)):
    for shift in (-0.6, -0.3, 0, 0.3, 0.6):
      rows = [closure(ds[k], r, variant, shift) for k in TRAIN for r in selections[k]]
      candidates.append(
        {'variant': variant, 'degree_horizon': VARIANTS[variant], 'shift': shift, 'train_rmse': float(np.median([x['accel_rmse'] for x in rows]))}
      )
  best = min(candidates, key=lambda x: x['train_rmse'])
  result = {'selection': best, 'candidates': candidates, 'routes': {}, 'rows': []}
  for k, d in ds.items():
    rows = []
    for j, r in enumerate(selections[k]):
      row = dict(route=k, window=j, split='train' if k in TRAIN else 'heldout', **closure(d, r, best['variant'], best['shift']))
      rows.append(row)
      result['rows'].append(row)
    result['routes'][k] = dict(
      windows=len(rows),
      **{
        c: float(np.median([x[c] for x in rows])) if rows else None for c in ('accel_rmse', 'lane_dd_rms', 'velocity_rmse', 'lane_ed_rms', 'observed_wander_cm')
      },
      shorter_10s_windows=len(list(pieces(d, 10))),
      model_latency_median_s=float(np.median(d['age'][:, 1])),
    )
  (OUT / 'closure.json').write_text(json.dumps(result, indent=2))
  print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'candidates')}, indent=2))


if __name__ == '__main__':
  main()
