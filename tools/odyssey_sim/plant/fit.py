"""Blocked, output-error identification; run with scratch scipy venv Python."""

import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

S = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')


def own(p):
  d = dict(np.load(p))
  # Resample by timestamp, retaining gaps. 100 Hz EPS timestamps have jitter.
  ix = np.r_[0, np.where(np.diff(np.floor(d['t'] * 10)) > 0)[0] + 1]
  d = {k: v[ix] for k, v in d.items()}
  d['ok'] = (d['active'] > 0) & (d['pressed'] == 0) & (abs(d['drv']) < 500) & (d['v'] > 5) & (abs(d['cmd']) < 0.95)
  d['u'] = d['cmd']
  d['y'] = d['act']
  return d


def windows(d, public=False):
  n = 100
  out = []
  # Five-second washout on every eligible interval; never stitch discontinuities.
  ok = d['ok'].copy()
  for j in range(1, 51):
    ok[j:] &= d['ok'][:-j]
  for i in range(50, len(ok) - n, n):
    if ok[i : i + n].all() and np.max(np.diff(d['t'][i : i + n])) < 0.16:
      if public and len(set(d['segment'][i : i + n])) != 1:
        continue
      out.append(np.array([d[k][i : i + n] for k in ('u', 'v', 'roll', 'y')]))
  return np.array(out)


def predict(p, w, one=False):
  gains = p[:3]
  tau = p[3]
  fric = p[4]
  rollgain = p[5]
  bias = p[6]
  u, v, r, y = w.transpose(1, 0, 2)
  # Hardware deadband fixed: effective aggregate Coulomb friction is fitted separately.
  eff = np.sign(u) * np.maximum(abs(u) - 0.025, 0)
  g = np.interp(v, [10, 20, 30], gains)
  a = y[:, 0].copy()
  pred = np.empty_like(y)
  pred[:, 0] = a
  for j in range(1, y.shape[1]):
    if one:
      a = y[:, j - 1].copy()
    target = g[:, j - 1] * eff[:, j - 1] + rollgain * r[:, j - 1] + bias
    push = target - a
    a = a + np.sign(push) * np.maximum(abs(push) - fric, 0) * (-np.expm1(-0.1 / tau))
    pred[:, j] = a
  return pred


def fit(w):
  rng = np.random.default_rng(42)
  z = w[rng.choice(len(w), min(len(w), 1800), replace=False)]

  def residual(p):
    return (predict(p, z)[:, 1:] - z[:, 3, 1:]).ravel()

  res = least_squares(
    residual,
    [1.4, 1.6, 1.8, 0.35, 0.12, 9.81, 0],
    bounds=([0.2, 0.2, 0.2, 0.04, 0, -30, -0.6], [8, 8, 8, 3, 0.6, 30, 0.6]),
    max_nfev=65,
    loss='soft_l1',
    f_scale=0.2,
    diff_step=0.001,
  )
  print('FIT', res.x, 'cost', res.cost, 'optimality', res.optimality, flush=True)
  return res.x


def score(p, w):
  if not len(w):
    return {}
  y = w[:, 3, 1:]
  pred = predict(p, w)[:, 1:]
  one = predict(p, w, True)[:, 1:]
  e = pred - y
  out = {
    'windows': len(w),
    'free_10s_rmse': float(np.sqrt(np.mean(e * e))),
    'one_100ms_rmse': float(np.sqrt(np.mean((one - y) ** 2))),
    'persistence_100ms_rmse': float(np.sqrt(np.mean(np.diff(w[:, 3], axis=1) ** 2))),
    'r2': float(1 - np.mean(e * e) / np.var(y)),
    'bands': {},
  }
  for lo, hi in [(5, 15), (15, 25), (25, 40)]:
    ok = (w[:, 1, 1:] >= lo) & (w[:, 1, 1:] < hi)
    if ok.any():
      out['bands'][f'{lo}-{hi}'] = {
        'n': int(ok.sum()),
        'free_rmse': float(np.sqrt(np.mean(e[ok] ** 2))),
        'one_rmse': float(np.sqrt(np.mean((one - y)[ok] ** 2))),
      }
  return out


def main():
  routes = {p.stem: own(p) for p in sorted((S / 'plant_data').glob('*.npz'))}
  ws = {k: windows(d) for k, d in routes.items()}
  for k, w in ws.items():
    print(k, w.shape, flush=True)
  trainkeys = ['00000174', '0000018c', '0000019b', '000001a3', '000001a7']
  wt = np.concatenate([ws[k] for k in trainkeys if len(ws[k])])
  p = fit(wt)
  params = json.loads((S / 'plant_model.json').read_text())
  params.update(
    status='William blocked-route output-error fit',
    speed_mps=[10, 20, 30],
    gain=p[:3].tolist(),
    tau_s=[float(p[3])] * 3,
    friction_accel=[float(p[4])] * 3,
    roll_gain=float(p[5]),
    bias_accel=float(p[6]),
    vehicle_tau_s=0.01,
    fit_routes=trainkeys,
    heldout_routes=['0000019c', '000001aa', '000001ab'],
  )
  (S / 'plant_model.json').write_text(json.dumps(params, indent=2) + '\n')
  scores = {
    'william_parameters': p.tolist(),
    'william': {k: score(p, w) for k, w in ws.items()},
    'cross_validation': '19c/1aa/1ab held out by route; other engaged routes training; manual routes no autonomous fit windows',
  }
  np.savez_compressed(S / 'plant_windows.npz', **ws)
  (S / 'plant_scores.json').write_text(json.dumps(scores, indent=2) + '\n')
  pub = dict(np.load(S / 'plant_public.npz'))
  pub.update(
    u=-pub['steerFiltered'],
    y=pub['latAccelSteeringAngle'],
    v=pub['vEgo'],
    ok=(pub['latActive'] > 0) & (pub['steeringPressed'] == 0) & (pub['vEgo'] > 5) & (abs(pub['steerFiltered']) < 0.95),
  )
  print('PUBLIC corr raw steer vs y', np.corrcoef(pub['steerFiltered'][pub['ok']], pub['y'][pub['ok']])[0, 1], flush=True)
  # Public steering output is opposite the normalized CAN command.
  ww = windows(pub, True)
  # Split whole sampled source segments deterministically, preserving disjoint validation segments.
  # windows occur on their own segment; derive segment from first matching timestamp is ambiguous,
  # so reconstruct separately by segment membership.
  parts = {}
  for arm in ('train', 'test'):
    dd = {k: v.copy() for k, v in pub.items()}
    held = (pub['segment'].astype(int) // 10) % 5 == 0
    dd['ok'] &= held if arm == 'test' else ~held
    parts[arm] = windows(dd, True)
  print('PUBLIC windows', ww.shape, {k: len(v) for k, v in parts.items()}, flush=True)
  pp = fit(parts['train'])
  scores.update(
    public_parameters=pp.tolist(),
    public_train=score(pp, parts['train']),
    public_test=score(pp, parts['test']),
    william_on_public_test=score(p, parts['test']),
    public_on_william={k: score(pp, w) for k, w in ws.items()},
  )
  (S / 'plant_scores.json').write_text(json.dumps(scores, indent=2) + '\n')
  params['public_model'] = {
    'speed_mps': [10, 20, 30],
    'gain': pp[:3].tolist(),
    'tau_s': [float(pp[3])] * 3,
    'friction_accel': [float(pp[4])] * 3,
    'roll_gain': float(pp[5]),
    'bias_accel': float(pp[6]),
    'source_segments_sampled': 1500,
    'sample_stride': 10,
  }
  (S / 'plant_model.json').write_text(json.dumps(params, indent=2) + '\n')
  print('DONE', json.dumps(scores, indent=2), flush=True)


if __name__ == '__main__':
  main()
