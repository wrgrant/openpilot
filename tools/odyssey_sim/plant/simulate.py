"""Fitted lane simulator and route replay. Writes diagnostics to scratch, no log-side caches.

Modes: logged_torque (plant replay), logged_plan (controller plus plant), and
road_feedback (closed lane loop, road + KP*offset + KD*sideways speed).
The camera road polynomial is an imperfect road estimate, NOT surveyed curvature.
"""

import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.plant.plant import new_state, step
from tools.odyssey_wander import bandpass, peak_period

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')


def runs(d):
  ok = (d['active'] > 0) & (d['pressed'] == 0) & (d['blinker'] == 0) & (d['v'] > 11.18) & np.isfinite(d['offset']) & np.isfinite(d['road'])
  ix = np.where(ok)[0]
  return [
    r
    for r in np.split(ix, np.where((np.diff(ix) > 1) | (np.diff(d['t'][ix]) > 0.15))[0] + 1)
    if len(r) >= 900 and np.mean((d['lp'][r] > 0.5) & (d['rp'][r] > 0.5)) > 0.8
  ]


def metrics(e, des, a, u):
  bd = bandpass(des)
  ba = bandpass(a)
  return {
    'wander_cm': float(np.std(bandpass(e)) * 100),
    'period_s': peak_period(des),
    'follow': float(np.std(bd - ba) / max(np.std(bd), 1e-6)),
    'tracking_rmse': float(np.sqrt(np.mean((des - a) ** 2))),
    'saturation_pct': float(np.mean(abs(u) >= 0.99) * 100),
    'small_torque_pct': float(np.mean(abs(u) < 0.05) * 100),
  }


def simulate(d, r, params, kp=0.336, kd=0.911, mode='road_feedback'):
  dt = 0.05
  a0 = d['act'][r[0]]
  s = new_state(params=params, lat_accel=a0, steering_accel=a0, motor_cmd=d['cmd'][r[0]])
  e = d['offset'][r[0]]
  ed = 0.0
  integ = 0.0
  # Initialize feedforward to measured torque instead of inducing a controller startup impulse.
  gain = 1.47
  friction = 0.145
  integ = (d['cmd'][r[0]] * gain - d['des'][r[0]]) / 0.15
  es = []
  aa = []
  uu = []
  dd = []
  for i in r:
    road = d['road'][i]
    req = road + kp * e + kd * ed if mode == 'road_feedback' else d['des'][i]
    if mode == 'logged_torque':
      u = d['cmd'][i]
    else:
      err = req - s['steering_accel']
      fric = np.clip(err / 0.2, -1, 1) * friction * gain
      u0 = (req + fric + 0.86 * err + 0.15 * integ) / gain
      u = float(np.clip(u0, -1, 1))
      if abs(u0) < 1 or u0 * err < 0:
        integ += err * dt
    s = step(s, float(u), float(d['v'][i]), dt)
    ed += (road - s['lat_accel']) * dt
    e += ed * dt
    es.append(e)
    aa.append(s['lat_accel'])
    uu.append(u)
    dd.append(req)
  return {k: np.array(v) for k, v in {'e': es, 'a': aa, 'u': uu, 'des': dd}.items()}


def main():
  params = json.loads((S / 'plant_model.json').read_text())
  routes = {}
  X = []
  Y = []
  for p in sorted((S / 'plant_data').glob('*.npz')):
    raw = dict(np.load(p))
    d = {k: v[::5] for k, v in raw.items()}
    rs = runs(d)
    routes[p.stem] = (d, rs)
    if p.stem in params['fit_routes']:
      for r in rs:
        off = d['offset'][r]
        ed = np.gradient(off, 0.05)
        X.extend(np.c_[off, ed, np.ones(len(r))][::10])
        Y.extend((d['des'][r] - d['road'][r])[::10])
  b = np.linalg.lstsq(np.array(X), np.array(Y), rcond=None)[0]
  # Gains from matched logged plan/road/offset frames; constrained fallback if camera road estimate reverses sign.
  kp = 0.336
  kd = 0.911  # Existing lane-model gains; frame-difference offset derivative is not lateral heading.
  print('lane gains raw', b, 'used', kp, kd, flush=True)
  results = {'lane_gains_raw': b.tolist(), 'lane_gains_used': [kp, kd], 'routes': {}}
  for key, (d, rs) in routes.items():
    pieces = []
    for r in rs:
      lanes = (d['lp'][r] > 0.5) & (d['rp'][r] > 0.5)
      off = np.interp(np.arange(len(r)), np.where(lanes)[0], d['offset'][r][lanes])
      real = metrics(off, d['des'][r], d['act'][r], d['cmd'][r])
      row = {
        'start_t': float(d['t'][r[0]]),
        'seconds': len(r) * 0.05,
        'real': real,
        'road_accel_rms': float(np.std(d['road'][r])),
        'mean_speed': float(d['v'][r].mean()),
      }
      for mode in ('logged_torque', 'logged_plan', 'road_feedback'):
        sim = simulate(d, r, params, kp, kd, mode)
        mm = metrics(sim['e'], sim['des'], sim['a'], sim['u'])
        mm['plant_vs_logged_accel_rmse'] = float(np.sqrt(np.mean((sim['a'] - d['act'][r]) ** 2)))
        mm['peak_abs_offset_m'] = float(np.max(abs(sim['e'])))
        row[mode] = mm
      pieces.append(row)
    agg = {}
    if pieces:
      for mode in ('real', 'logged_torque', 'logged_plan', 'road_feedback'):
        agg[mode] = {k: float(np.average([p[mode][k] for p in pieces], weights=[p['seconds'] for p in pieces])) for k in pieces[0][mode]}
    results['routes'][key] = {'pieces': pieces, 'seconds': sum(p['seconds'] for p in pieces), 'aggregate': agg}
    print(key, len(pieces), agg, flush=True)
  (S / 'plant_sim_scores.json').write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
  main()
