"""Low-speed output-error identification and train-only controller optimization."""

import json
from dataclasses import asdict
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares, differential_evolution
from tools.odyssey_sim.lowspeed.controller import AngleController, Gains
from tools.odyssey_sim.lowspeed.plant import simulate, metrics

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')
O = S / 'lowspeed'
DT = 0.1


def own(route):
  raw = dict(np.load(S / f'plant_data/00000{route}.npz'))
  ix = np.r_[0, np.where(np.diff(np.floor(raw['t'] * 10)) > 0)[0] + 1]
  d = {k: x[ix] for k, x in raw.items()}
  body = np.load(S / f'calib/00000{route}_body.npz')
  vp = body['vp']
  vi = np.clip(np.searchsorted(vp[:, 0], d['t'], side='right') - 1, 0, len(vp) - 1)
  d['angle'] -= vp[vi, 2]
  d['u'] = -d['cmd']  # CAN command is opposite actuators.torque
  tr = body['torque']
  ti = np.clip(np.searchsorted(tr[:, 0], d['t'], side='right') - 1, 0, len(tr) - 1) if len(tr) else None
  des = tr[ti, 1] if ti is not None else d['des']
  d['target'] = -np.degrees(np.arctan(des / np.maximum(d['v'] ** 2, 0.25) * 3.0)) * vp[vi, 3]
  replay = S / f'nnlc_{route}.npz'
  if replay.exists():
    q = np.load(replay)
    qi = np.clip(np.searchsorted(q['t'], d['t'], side='right') - 1, 0, len(q['t']) - 1)
    d['target'] = -np.degrees(np.arctan(q['desired'][qi] / np.maximum(d['v'] ** 2, 0.25) * 3.0)) * vp[vi, 3]
  d['preview'] = d['target'].copy()
  plan_path = O / f'plan_{route}.npz'
  if plan_path.exists():
    plan = np.load(plan_path)
    pi = np.clip(np.searchsorted(plan['t'], d['t'], side='right') - 1, 0, len(plan['t']) - 1)
    future_angle = -np.degrees(np.arctan(plan['yaw'] / np.maximum(plan['speed'], 0.5) * 3.0))
    preview = np.array([np.interp(0.4, plan['horizon'], a) for a in future_angle])
    valid = (d['t'] - plan['t'][pi] >= 0) & (d['t'] - plan['t'][pi] < 0.15)
    d['preview'][valid] = preview[pi[valid]] * vp[vi[valid], 3]
  d['ok'] = (d['v'] >= 0.5) & (d['v'] < 8) & (d['active'] > 0.5) & (d['pressed'] == 0) & (abs(d['drv']) < 1000)
  return d


def windows(d, n=21):
  rows = []
  for i in range(10, len(d['t']) - n, 10):
    sl = slice(i, i + n)
    if d['ok'][sl].all() and np.max(np.diff(d['t'][i - 10 : i + n])) < 0.16 and len(set(d['seg'][i - 10 : i + n])) == 1:
      rows.append(np.array([d[k][sl] for k in ['u', 'v', 'angle', 'rate', 'target', 't', 'preview']]))
  return np.asarray(rows) if rows else np.empty((0, 7, n))


def fit(w):
  def residual(p):
    a, r = simulate(p, w)
    return np.r_[(a[:, 1:] - w[:, 2, 1:]).ravel() / 5, (r[:, 1:] - w[:, 3, 1:]).ravel() / 10]

  results = []
  for seed in [0.15, 0.4, 0.65]:
    res = least_squares(
      residual,
      [25, 35, 45, seed, 0.002, 0.4, 0.25],
      bounds=([1, 1, 1, 0, 0, 0.05, 0], [180, 180, 180, 0.85, 0.03, 2.0, 1.0]),
      loss='soft_l1',
      max_nfev=100,
      diff_step=0.01,
    )
    results.append((np.mean(residual(res.x) ** 2), res.x))
  return min(results, key=lambda x: x[0])[1]


def closed(p, w, gains=None, installed=False, preview=False):
  if not len(w):
    return {}
  a = w[:, 2, 0].copy()
  r = w[:, 3, 0].copy()
  commands = np.zeros((len(w), w.shape[2]))
  controllers = [AngleController(True, gains, DT) for _ in w]
  aa = []
  rr = []
  for j in range(w.shape[2]):
    for i, c in enumerate(controllers):
      des = float(w[i, 4, j])
      v = float(w[i, 1, j])
      if installed:
        commands[i, j] = np.clip(np.clip(0.02 * des, -0.35, 0.35) + 0.02 * (des - a[i]), -1, 1)
      else:
        future = ([0.0, 0.4], [des, float(w[i, 6, j])]) if preview else None
        commands[i, j] = c.update(0.0, v, float(a[i]), float(r[i]), des, future=future).torque
    if j:
      f = max(0.0, j - 1 - p[6] / DT)
      lo = int(f)
      hi = min(lo + 1, j - 1)
      net = commands[:, lo] * (1 - (f - lo)) + commands[:, hi] * (f - lo) - p[4] * a
      goal = np.interp(w[:, 1, j], [1, 4, 8], p[:3]) * np.sign(net) * np.maximum(abs(net) - p[3], 0)
      r += (goal - r) * (-np.expm1(-DT / p[5]))
      a += r * DT
    aa.append(a.copy())
    rr.append(r.copy())
  a = np.asarray(aa).T
  r = np.asarray(rr).T
  return {
    'tracking_deg': float(np.sqrt(np.mean((a - w[:, 4]) ** 2))),
    'rate_change_deg_s2': float(np.sqrt(np.mean((np.diff(r) / DT) ** 2))),
    'command_slew_rms': float(np.sqrt(np.mean((np.diff(commands) / DT) ** 2))),
    'saturation': float(np.mean(abs(commands) > 0.99)),
    'high_effort': float(np.mean(abs(commands) > 0.8)),
  }


def objective(g, p, w):
  q = closed(p, w, Gains(*g))
  return q['tracking_deg'] + 0.18 * q['rate_change_deg_s2'] + 0.5 * q['command_slew_rms']


def main():
  O.mkdir(exist_ok=True)
  data = {r: own(r) for r in ['1a3', '1a7', '1aa', '1ab', '1a4', '1a5']}
  ws = {r: windows(d) for r, d in data.items()}
  report = {
    'coverage': {},
    'generations': [
      {
        'generation': -1,
        'name': 'strict_3_second_fit_rejected',
        'reason': 'No train windows below 4.5 m/s; gain1 unidentifiable. Broad fit uses 2s windows and sensor <1000; confounded.',
      }
    ],
  }
  for route, d in data.items():
    report['coverage'][route] = {
      'broad_sensor_seconds': float(d['ok'].sum() / 10),
      'windows': len(ws[route]),
      'speed_bins': {f'{lo}-{hi}': float(np.sum(d['ok'] & (d['v'] >= lo) & (d['v'] < hi)) / 10) for lo, hi in [(0.5, 2), (2, 4.5), (4.5, 8)]},
    }
  train = np.concatenate([ws['1a3'], ws['1a7']])
  p = fit(train)
  report['plant'] = {
    'parameters': p.tolist(),
    'names': ['gain1', 'gain4', 'gain8', 'friction', 'hold', 'tau', 'delay'],
    'scores': {r: metrics(p, w) for r, w in ws.items() if r not in ['1a4', '1a5']},
  }
  np.savez_compressed(O / 'windows.npz', **ws)
  (O / 'plant.json').write_text(json.dumps(report['plant'], indent=2))
  print('PLANT', report['plant'], flush=True)
  manual = {}
  for route in ['1a4', '1a5']:
    d = data[route]
    m = (d['v'] >= 0.5) & (d['v'] < 8)
    manual[route] = {
      'seconds': float(m.sum() / 10),
      'rate_abs_quantiles': np.quantile(abs(d['rate'][m]), [0.5, 0.9, 0.99]).tolist(),
      'motor_quantiles': np.quantile(d['motor'][m], [0.5, 0.9, 0.99]).tolist(),
      'torque_bins': {
        f'{lo}-{hi}': {
          'n': int(np.sum(m & (abs(d['drv']) >= lo) & (abs(d['drv']) < hi))),
          'median_motor': float(np.median(d['motor'][m & (abs(d['drv']) >= lo) & (abs(d['drv']) < hi)])),
        }
        for lo, hi in [(0, 200), (200, 500), (500, 1000), (1000, 3000)]
      },
    }
  report['manual'] = manual
  q = dict(np.load(S / 'plant_public.npz'))
  pub = []
  for seg in np.unique(q['segment']):
    ix = np.flatnonzero(q['segment'] == seg)
    t = q['t'][ix]
    a = q['steeringAngleDeg'][ix]
    v = q['vEgo'][ix]
    if len(t) < 22 or np.any(np.diff(t) <= 0):
      continue
    ok = (v >= 0.5) & (v < 8) & (q['latActive'][ix] > 0.5) & (q['steeringPressed'][ix] == 0)
    d = {'t': t, 'angle': a, 'rate': np.gradient(a, t), 'u': -q['steerFiltered'][ix], 'v': v, 'target': a, 'preview': a, 'ok': ok, 'seg': q['segment'][ix]}
    w = windows(d)
    if len(w):
      pub.append((int(seg), w))
  ptr = np.concatenate([w for seg, w in pub if seg % 20 == 0])
  pte = np.concatenate([w for seg, w in pub if seg % 20 != 0])
  pp = fit(ptr)
  flipped = ptr.copy()
  flipped[:, 0] *= -1
  pf = fit(flipped)
  if metrics(pf, flipped)['angle_rmse'] < metrics(pp, ptr)['angle_rmse']:
    pp = pf
    ptr = flipped
    pte = pte.copy()
    pte[:, 0] *= -1
    public_sign = 1
  else:
    public_sign = -1
  report['public'] = {
    'train': len(ptr),
    'test': len(pte),
    'sign': public_sign,
    'parameters': pp.tolist(),
    'train_score': metrics(pp, ptr),
    'test_score': metrics(pp, pte),
    'william_transfer': metrics(p, pte),
  }
  print('PUBLIC', report['public'], flush=True)
  low = {r: w[(np.max(w[:, 1], axis=1) < 4.5) & (np.max(abs(w[:, 4]), axis=1) < 600)] for r, w in ws.items()}
  tw = low['1a7']
  report['low_windows'] = {r: len(w) for r, w in low.items()}
  default = Gains()
  report['generations'].append({'generation': 0, 'name': 'installed', 'train': closed(p, tw, installed=True)})
  report['generations'].append({'generation': 1, 'name': 'initial_pi_pd', 'params': asdict(default), 'train': closed(p, tw, default)})
  bounds = [(0.002, 0.07), (0, 0.015), (0, 0.08), (0, 0.65), (0, 0.012), (0, 0.08), (3, 35), (0.2, 2.0), (0, 0.8)]
  bounds[-1] = (0.4, 0.4)  # Preview cannot be tuned without a recorded model horizon.
  opt = differential_evolution(lambda g: objective(g, p, tw), bounds, seed=20261109, maxiter=18, popsize=5, polish=False, workers=1)
  tuned = Gains(*opt.x)
  report['generations'].append(
    {'generation': 2, 'name': 'DE_training_only', 'evaluations': int(opt.nfev), 'params': asdict(tuned), 'train': closed(p, tw, tuned)}
  )

  # Second optimization penalizes near-limit dwell explicitly and caps slew/friction.
  def smooth_cost(g):
    q = closed(p, tw, Gains(*g), preview=True)
    return q['tracking_deg'] + 0.7 * q['rate_change_deg_s2'] + 2 * q['command_slew_rms'] + 12 * q['high_effort']

  bounds[3] = (0.0, 0.3)
  bounds[6] = (2.0, 15.0)
  bounds[7] = (0.2, 0.8)
  smooth = differential_evolution(smooth_cost, bounds, seed=19, maxiter=35, popsize=5, polish=False)
  smooth_gains = Gains(*smooth.x)
  report['generations'].append(
    {
      'generation': 3,
      'name': 'constrained_preview_DE',
      'params': asdict(smooth_gains),
      'evaluations': int(smooth.nfev),
      'train': closed(p, tw, smooth_gains, preview=True),
    }
  )
  report['smooth_validation'] = {r: closed(p, low[r], smooth_gains, preview=True) for r in ['1aa', '1ab']}
  # Human input model is distinct: fit 1a4, test 1a5. Motor magnitude is unsigned,
  # so this describes combined human+assist response, not autonomous motor authority.
  manual_windows = {}
  for r in ['1a4', '1a5']:
    d = data[r].copy()
    d['ok'] = (d['v'] >= 0.5) & (d['v'] < 8)
    d['u'] = -d['drv'] / 1000.0
    manual_windows[r] = windows(d)
  mp = fit(manual_windows['1a4'])
  report['manual_model'] = {'parameters': mp.tolist(), 'scores': {r: metrics(mp, w) for r, w in manual_windows.items()}}
  report['empirical_envelope'] = {}
  for r, d in data.items():
    by_speed = {}
    for lo, hi in [(0.5, 2), (2, 4.5), (4.5, 8)]:
      m = (d['v'] >= lo) & (d['v'] < hi) & (d['active'] > 0) & (d['pressed'] == 0) & (abs(d['u']) > 0.95) & (abs(d['drv']) < 1000)
      toward = d['rate'][m] * np.sign(d['u'][m])
      by_speed[f'{lo}-{hi}'] = {
        'n': int(m.sum()),
        'toward_rate_quantiles': np.quantile(toward, [0.1, 0.5, 0.9]).tolist() if m.any() else [],
        'angle_abs_quantiles': np.quantile(abs(d['angle'][m]), [0.1, 0.5, 0.9]).tolist() if m.any() else [],
      }
    report['empirical_envelope'][r] = by_speed
  (O / 'smooth_tuned.json').write_text(json.dumps(asdict(smooth_gains), indent=2))
  report['validation'] = {}
  for r in ['1aa', '1ab']:
    w = low[r]
    for label, m in [('all', np.ones(len(w), dtype=bool)), ('gentle', np.max(abs(w[:, 4]), axis=1) <= 40), ('tight', np.max(abs(w[:, 4]), axis=1) > 90)]:
      report['validation'][r + '_' + label] = {'n': int(m.sum()), 'installed': closed(p, w[m], installed=True), 'candidate': closed(p, w[m], tuned)}
  report['sensitivity'] = {
    str(mult): {'installed': closed(np.r_[p[:3] * mult, p[3:]], low['1aa'], installed=True), 'candidate': closed(np.r_[p[:3] * mult, p[3:]], low['1aa'], tuned)}
    for mult in [0.5, 1.5]
  }
  report['replay'] = {}
  for r in ['1a7', '1aa', '1ab']:
    q = np.load(S / f'nnlc_{r}.npz')
    m = (q['active'] > 0) & (q['pressed'] == 0) & (q['speed'] >= 0.5) & (q['speed'] < 6.7)
    report['replay'][r] = {
      name: {'n': int(m.sum()), 'saturation': float(np.mean(abs(q[name][m]) > 0.99)), 'mean_abs': float(np.mean(abs(q[name][m])))}
      for name in ['logged', 'stock_v0_logged', 'nnlc_v0_logged']
    }
  (O / 'results.json').write_text(json.dumps(report, indent=2))
  (O / 'tuned.json').write_text(json.dumps(asdict(tuned), indent=2))
  print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
  main()
