"""Train-only spectral fusion of independent lane heading/curvature/motion channels."""

import json
import numpy as np
from tools.odyssey_sim.calib.identify import TRAIN, TEST
from tools.odyssey_sim.calib3.audit import OUT, records, pieces
from tools.odyssey_sim.calib3.spectral import features


def xy(d, r):
  w, e, _, vh, vy, body, road = features(d, r, 1)
  keep = (w >= 2 * np.pi / 20) & (w <= 2 * np.pi / 7)
  # Model yaw is an independent learned orientation-rate channel at time zero.
  x = np.c_[road - body, 1j * w * vh, 1j * w * vy]
  return w, e, x, keep


def main():
  ds = {k: records(k) for k in TRAIN + TEST}
  xx, yy = [], []
  for key in TRAIN:
    for r in pieces(ds[key], 30):
      w, e, x, keep = xy(ds[key], r)
      xx.extend(x[keep])
      yy.extend((-w * w * e)[keep])
  x, y = np.array(xx), np.array(yy)
  a = np.r_[x.real, x.imag]
  b = np.r_[y.real, y.imag]
  coef = np.linalg.lstsq(a, b, rcond=None)[0]
  rows = []
  for key in TRAIN + TEST:
    for seconds in (30, 60):
      for j, r in enumerate(pieces(ds[key], seconds)):
        w, e, x, keep = xy(ds[key], r)
        residual = (x @ coef + w * w * e) * keep
        disp = np.divide(residual, -w * w, out=np.zeros_like(residual), where=w > 0)
        rows.append(
          {
            'route': key,
            'seconds': seconds,
            'window': j,
            'accel_rmse': float(np.std(np.fft.irfft(residual))),
            'lane_dd_rms': float(np.std(np.fft.irfft(-w * w * e))),
            'equivalent_wander_error_cm': float(100 * np.std(np.fft.irfft(disp))),
            'observed_wander_cm': float(100 * np.std(np.fft.irfft(e))),
          }
        )
  result = {'coefficients_spatial_transport_model_vy': coef.tolist(), 'train_30s_windows': len(yy) // 3, 'rows': rows, 'ranking_authorized': False}
  (OUT / 'fusion.json').write_text(json.dumps(result, indent=2))
  print(coef, flush=True)
  for key in TRAIN + TEST:
    z = [x for x in rows if x['route'] == key and x['seconds'] == 30]
    if z:
      print(key, len(z), {c: float(np.median([x[c] for x in z])) for c in ('accel_rmse', 'lane_dd_rms', 'equivalent_wander_error_cm')}, flush=True)


if __name__ == '__main__':
  main()
