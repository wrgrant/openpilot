"""Supplemental checks: uncertainty, tight-turn stress and manual release behaviour."""

import json
from dataclasses import asdict
import numpy as np
from tools.odyssey_sim.lowspeed.research import O, own, closed
from tools.odyssey_sim.lowspeed.controller import Gains


def main():
  result = json.loads((O / 'results.json').read_text())
  gains = Gains(**json.loads((O / 'smooth_tuned.json').read_text()))
  p = np.asarray(result['plant']['parameters'])
  public = np.asarray(result['public']['parameters'])
  # Synthetic planned turn is known at t=0: no future logged frames used.
  t = np.arange(101) * 0.1
  target = np.interp(t, [0, 1, 5, 7, 10], [0, 0, 120, 120, 0])
  preview = np.interp(t + 0.4, t, target)
  w = np.zeros((1, 7, len(t)))
  w[0, 1] = 2.0
  w[0, 4] = target
  w[0, 5] = t
  w[0, 6] = preview
  out = {'synthetic_tight': {}, 'manual_release': {}, 'gains': asdict(gains)}
  for name, plant in [('william', p), ('public', public), ('half_gain', np.r_[p[:3] * 0.5, p[3:]]), ('double_gain', np.r_[p[:3] * 2, p[3:]])]:
    out['synthetic_tight'][name] = {'installed': closed(plant, w, installed=True), 'candidate': closed(plant, w, gains, preview=True)}
  for route in ['1a4', '1a5']:
    d = own(route)
    m = (d['v'] > 0.5) & (d['v'] < 8) & (abs(d['drv']) < 200) & (abs(d['angle']) > 30)
    toward = -d['rate'][m] * np.sign(d['angle'][m])
    out['manual_release'][route] = {
      'n': int(m.sum()),
      'toward_center_rate_quantiles': np.quantile(toward, [0.1, 0.5, 0.9]).tolist(),
      'stationary_fraction': float(np.mean(abs(toward) < 1)),
    }
  # Separate scored bins and counts: don't let >4.5m/s dominate low-speed validation.
  cache = dict(np.load(O / 'windows.npz'))
  from tools.odyssey_sim.lowspeed.research import metrics

  out['plant_bins'] = {}
  for route in ['1a7', '1aa', '1ab']:
    z = cache[route]
    out['plant_bins'][route] = {
      f'{lo}-{hi}': metrics(p, z[(np.min(z[:, 1], axis=1) >= lo) & (np.max(z[:, 1], axis=1) < hi)]) for lo, hi in [(0.5, 2), (2, 4.5), (4.5, 8)]
    }
  out['angle_only_replay'] = {}
  for route in ['1a7', '1aa', '1ab']:
    from tools.odyssey_sim.lowspeed.research import S

    q = np.load(S / f'nnlc_{route}.npz')
    raw = np.load(S / f'plant_data/00000{route}.npz')
    vp = np.load(S / f'calib/00000{route}_body.npz')['vp']
    ri = np.clip(np.searchsorted(raw['t'], q['t'], side='right') - 1, 0, len(raw['t']) - 1)
    vi = np.clip(np.searchsorted(vp[:, 0], q['t'], side='right') - 1, 0, len(vp) - 1)
    desired = -np.degrees(np.arctan(q['desired'] / np.maximum(q['speed'] ** 2, 0.25) * 3.0)) * vp[vi, 3]
    measured = raw['angle'][ri] - vp[vi, 2]
    installed = np.clip(np.clip(0.02 * desired, -0.35, 0.35) + 0.02 * (desired - measured), -1, 1)
    m = (q['speed'] >= 0.5) & (q['speed'] < 4.5) & (q['active'] > 0) & (q['pressed'] == 0)
    out['angle_only_replay'][route] = {
      name: {'n': int(m.sum()), 'saturation': float(np.mean(abs(u[m]) > 0.99)), 'mean_abs': float(np.mean(abs(u[m])))}
      for name, u in [('installed_kinematic', installed), ('stock', q['stock_v0_logged']), ('nnlc', q['nnlc_v0_logged'])]
    }
  (O / 'diagnostics.json').write_text(json.dumps(out, indent=2))
  print(json.dumps(out, indent=2))


if __name__ == '__main__':
  main()
