"""Reconstruct road turning from lane-heading transport, independently of offset.

World-road acceleration = recorded body acceleration + v*d(lane_heading)/dt.
Unlike offset double differentiation, this uses a separate geometric observation.
It is still a camera-conditioned reconstruction, not an independently surveyed road.
"""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import TRAIN, TEST
from tools.odyssey_sim.calib3.audit import OUT, records, pieces, derivative, band


def sample(d, r, variant, shift, gain, vy_gain):
  e, h, k = d['g'][r, variant].T
  v, t = d['v'][r], d['t'][r]
  velocity = v * np.tan(h) * gain - vy_gain * d['vel'][r, 1]
  velocity = np.interp(t + shift, t, velocity)
  sl = slice(100, -100)
  residual = band(derivative(velocity) - derivative(e, 2))[sl]
  lane = band(derivative(e, 2))[sl]
  # Also compare velocity-integrated displacement in the wander band.
  freq = np.fft.rfftfreq(len(r), 0.1)
  vh = np.fft.rfft(velocity - np.mean(velocity))
  denom = 2j * np.pi * freq
  integ = np.divide(vh, denom, out=np.zeros_like(vh), where=freq > 0)
  integ[(freq < 0.05) | (freq > 1 / 7)] = 0
  ef = np.fft.rfft(e - np.mean(e))
  ef[(freq < 0.05) | (freq > 1 / 7)] = 0
  return {
    'accel_rmse': float(np.sqrt(np.mean(residual**2))),
    'lane_dd_rms': float(np.sqrt(np.mean(lane**2))),
    'velocity_rmse': float(np.sqrt(np.mean(band(velocity - derivative(e))[sl] ** 2))),
    'equivalent_wander_error_cm': float(100 * np.std(np.fft.irfft(integ - ef, len(r)))),
  }


def main():
  ds = {k: records(k) for k in TRAIN + TEST}
  rr = {k: list(pieces(d)) for k, d in ds.items()}
  candidates = []
  for variant in range(6):
    for shift in (-0.3, 0, 0.3):
      for gain in (0.5, 1.0, 1.5):
        for vy_gain in (0, 1):
          rows = [sample(ds[k], r, variant, shift, gain, vy_gain) for k in TRAIN for r in rr[k]]
          candidates.append(
            {'variant': variant, 'shift': shift, 'gain': gain, 'vy_gain': vy_gain, 'train_rmse': float(np.median([x['accel_rmse'] for x in rows]))}
          )
  best = min(candidates, key=lambda x: x['train_rmse'])
  rows = []
  for k in TRAIN + TEST:
    for j, r in enumerate(rr[k]):
      rows.append({'route': k, 'window': j, **sample(ds[k], r, **{c: best[c] for c in ('variant', 'shift', 'gain', 'vy_gain')})})
  result = {'selection': best, 'rows': rows, 'candidates': candidates, 'ranking_authorized': False}
  (OUT / 'transport.json').write_text(json.dumps(result, indent=2))
  print(json.dumps(best), flush=True)
  for k in TRAIN + TEST:
    z = [x for x in rows if x['route'] == k]
    if z:
      print(k, {c: float(np.median([x[c] for x in z])) for c in ('accel_rmse', 'lane_dd_rms', 'equivalent_wander_error_cm')}, flush=True)


if __name__ == '__main__':
  main()
