"""Same spectral closure definition as pass 2; matched windows and short coverage."""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import TRAIN, TEST
from tools.odyssey_sim.calib3.audit import OUT, records, pieces


def features(d, r, variant):
  e, h, k = d['g'][r, variant].T
  v = d['v'][r]
  w = 2 * np.pi * np.fft.rfftfreq(len(r), 0.1)
  keep = (w >= 2 * np.pi / 20) & (w <= 2 * np.pi / 7)

  def spec(x):
    t = np.arange(len(x))
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    return np.fft.rfft(x) * keep

  return w, spec(e), spec(v * v * k - d['body'][r]), spec(v * np.tan(h)), spec(d['vel'][r, 1]), spec(d['body'][r]), spec(v * v * k)


def score(f, mode, shift=0, vy_gain=0, gain=1):
  w, e, _, vh, vy, body, road = f
  phase = np.exp(1j * w * shift)
  rhs = (road * phase - body) if mode == 'spatial' else 1j * w * (gain * vh - vy_gain * vy) * phase
  residual = rhs + w * w * e

  def rms(z):
    return float(np.std(np.fft.irfft(z)))

  disp = np.divide(residual, -w * w, out=np.zeros_like(residual), where=w > 0)
  return {'accel_rmse': rms(residual), 'lane_dd_rms': rms(-w * w * e), 'equivalent_wander_error_cm': 100 * rms(disp), 'observed_wander_cm': 100 * rms(e)}


def main():
  ds = {k: records(k) for k in TRAIN + TEST}
  fs = {k: [[features(d, r, i) for i in range(6)] for r in pieces(d)] for k, d in ds.items()}
  candidates = []
  for mode in ('spatial', 'transport'):
    for variant in range(6):
      for shift in (-0.6, -0.3, 0, 0.3, 0.6):
        for vy in (0,) if mode == 'spatial' else (0, 1):
          p = {'mode': mode, 'variant': variant, 'shift': shift, 'vy_gain': vy}
          rows = [score(f[variant], mode, shift, vy) for k in TRAIN for f in fs[k]]
          candidates.append({**p, 'train_rmse': float(np.median([x['accel_rmse'] for x in rows]))})
  best = {mode: min((c for c in candidates if c['mode'] == mode), key=lambda c: c['train_rmse']) for mode in ('spatial', 'transport')}
  rows = []
  for key in TRAIN + TEST:
    for seconds in (60, 30):
      for j, r in enumerate(pieces(ds[key], seconds)):
        for mode, p in best.items():
          rows.append(
            {'route': key, 'seconds': seconds, 'window': j, 'mode': mode, **score(features(ds[key], r, p['variant']), mode, p['shift'], p['vy_gain'])}
          )
  coverage = []
  for key, d in ds.items():
    for seconds in (10, 20, 30, 60):
      for lo, hi in ((10, 20), (20, 35), (35, 50), (50, 200)):
        count = sum(bool(((d['v'][r] * 2.23694 >= lo) & (d['v'][r] * 2.23694 < hi)).all()) for r in pieces(d, seconds))
        coverage.append({'route': key, 'seconds': seconds, 'band': f'{lo}-{hi}', 'windows': count})
  result = {'selection': best, 'rows': rows, 'coverage': coverage, 'candidates': candidates}
  (OUT / 'spectral.json').write_text(json.dumps(result, indent=2))
  print(json.dumps(best), flush=True)
  for key in TRAIN + TEST:
    for mode in best:
      z = [x for x in rows if x['route'] == key and x['seconds'] == 60 and x['mode'] == mode]
      if z:
        print(
          key,
          mode,
          len(z),
          {c: float(np.median([x[c] for x in z])) for c in ('accel_rmse', 'lane_dd_rms', 'equivalent_wander_error_cm', 'observed_wander_cm')},
          flush=True,
        )


if __name__ == '__main__':
  main()
