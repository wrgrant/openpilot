"""Exercise actual shared API against held-out windows and held-out CAN telemetry."""

import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.plant.plant import new_state, step

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')


def main():
  params = json.loads((S / 'plant_model.json').read_text())
  ww = np.load(S / 'plant_windows.npz')
  scores = {}
  for key in params['heldout_routes']:
    w = ww[key]
    errors = []
    one = []
    for z in w:
      u, v, roll, y = z
      s = new_state(params=params, lat_accel=y[0], steering_accel=y[0], motor_cmd=np.sign(u[0]) * max(abs(u[0]) - 0.025, 0))
      for i in range(1, len(u)):
        s['roll'] = roll[i - 1]
        s = step(s, float(u[i - 1]), float(v[i - 1]), 0.1)
        errors.append(s['steering_accel'] - y[i])
        p = new_state(params=params, lat_accel=y[i - 1], steering_accel=y[i - 1], motor_cmd=np.sign(u[i - 1]) * max(abs(u[i - 1]) - 0.025, 0), roll=roll[i - 1])
        p = step(p, float(u[i - 1]), float(v[i - 1]), 0.1)
        one.append(p['steering_accel'] - y[i])
    scores[key] = {'api_free_10s_rmse': float(np.sqrt(np.mean(np.array(errors) ** 2))), 'api_one_100ms_rmse': float(np.sqrt(np.mean(np.array(one) ** 2)))}
  train = []
  test = []
  for p in sorted((S / 'plant_data').glob('*.npz')):
    d = dict(np.load(p))
    ok = (d['active'] > 0) & (d['pressed'] == 0) & (abs(d['drv']) < 300) & (d['v'] > 10) & (abs(d['cmd']) < 0.8)
    # Limit assistance confounding and fit quasi-static transfer only.
    stable = abs(d['cmd'] - np.roll(d['cmd'], 30)) < 0.01
    ix = np.where(ok & stable)[0][::10]
    vals = np.c_[abs(d['cmd'][ix]), d['motor'][ix]]
    (test if p.stem in params['heldout_routes'] else train).extend(vals)
  a = np.array(train)
  b = np.array(test)
  best = None
  for db in np.arange(0, 0.051, 0.001):
    x = np.c_[np.maximum(a[:, 0] - db, 0), np.ones(len(a))]
    coef = np.linalg.lstsq(x, a[:, 1], rcond=None)[0]
    if not 0 <= coef[1] <= 4:
      continue
    mse = np.mean((x @ coef - a[:, 1]) ** 2)
    if best is None or mse < best[0]:
      best = (mse, db, coef)
  mse, db, c = best
  pred = np.maximum(b[:, 0] - db, 0) * c[0] + c[1]
  scores['motor'] = {
    'train_n': len(a),
    'test_n': len(b),
    'deadband': float(db),
    'gain': float(c[0]),
    'baseline': float(c[1]),
    'train_rmse': float(np.sqrt(mse)),
    'test_rmse': float(np.sqrt(np.mean((pred - b[:, 1]) ** 2))),
  }
  # Verify finite, bounded, sign-consistent response and immutable state.
  for v in [5, 15, 25, 35]:
    for u in [-1.0, -0.04, 0.0, 0.04, 1.0]:
      s0 = new_state(params=params)
      s = s0
      for _ in range(1000):
        s = step(s, u, v, 0.01)
      assert s0['lat_accel'] == 0
      assert np.isfinite(s['lat_accel']) and abs(s['lat_accel']) < 10
      assert s['lat_accel'] * u >= 0 and s['angle_deg'] * u <= 0
  (S / 'plant_api_validation.json').write_text(json.dumps(scores, indent=2) + '\n')
  print(json.dumps(scores, indent=2))


if __name__ == '__main__':
  main()
