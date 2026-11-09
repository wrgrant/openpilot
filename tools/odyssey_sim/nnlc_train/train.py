"""Deterministic CPU torch training; numpy-only exported models and route holdouts."""

import copy
import json
import numpy as np
import torch
from torch import nn

from tools.odyssey_sim.nnlc_train import SCRATCH as S, MODEL_DIR


def predict_json(p, x):
  z = (x - np.array(p['input_mean']).ravel()) / np.array(p['input_std']).ravel()
  for layer in p['layers']:
    w = next(v for k, v in layer.items() if k.endswith('_W'))
    b = next(v for k, v in layer.items() if k.endswith('_b'))
    z = z @ np.array(w).T + np.array(b).ravel()
    if layer['activation'] in ('sigmoid', 'σ'):
      z = 1 / (1 + np.exp(-np.clip(z, -80, 80)))
  return z.ravel()


def network(n):
  return nn.Sequential(nn.Linear(n, 64), nn.Sigmoid(), nn.Linear(64, 32), nn.Sigmoid(), nn.Linear(32, 1))


def export(net, mean, std, name):
  layers = []
  for i, l in enumerate(net):
    if isinstance(l, nn.Linear):
      layers.append(
        {
          f'dense_{i}_W': l.weight.detach().numpy().tolist(),
          f'dense_{i}_b': l.bias.detach().numpy()[:, None].tolist(),
          'activation': 'identity' if i == len(net) - 1 else 'sigmoid',
        }
      )
  p = {
    'input_size': len(mean),
    'output_size': 1,
    'input_mean': mean[:, None].tolist(),
    'input_std': std[:, None].tolist(),
    'layers': layers,
    'training': 'Inverse achieved-angle acceleration aligned +0.32s; route holdouts aa/ab; validation a7; seed 20261109',
    'experimental': True,
  }
  p['input_vars'] = json.loads((MODEL_DIR / 'HONDA_ODYSSEY.json').read_text())['input_vars'] + (
    ['steering_angle_deg', 'steering_rate_deg_s', 'actual_accel', 'actual_accel_m01', 'actual_accel_m03', 'actual_accel_m05', 'command_m03', 'command_m05']
    if len(mean) > 18
    else []
  )
  destination = S / name if '_PUBLIC' in name else MODEL_DIR / name
  destination.write_text(json.dumps(p, separators=(',', ':')) + '\n')
  return p


def fit(name, extended, pub, own, val):
  torch.manual_seed(20261109)
  rng = np.random.default_rng(20261109)

  def inp(d):
    return np.c_[d['x'], d['extra']] if extended else d['x']

  px, ox, vx = map(inp, (pub, own, val))
  mean = px.mean(0)
  std = np.maximum(px.std(0), 0.02)
  net = network(px.shape[1])
  opt = torch.optim.Adam(net.parameters(), lr=0.002)
  vt = torch.tensor((vx - mean) / std, dtype=torch.float32)
  vy = torch.tensor(val['y'][:, None], dtype=torch.float32)
  best = None
  score = float('inf')
  for phase, d, x, epochs in [('public', pub, px, 35), ('van', own, ox, 60)]:
    tx = torch.tensor((x - mean) / std, dtype=torch.float32)
    ty = torch.tensor(d['y'][:, None], dtype=torch.float32)
    weights = d['weight'] * (1 + 2 * (abs(d['y']) < 0.1))
    weights = weights / weights.sum()
    for ep in range(epochs):
      ix = rng.choice(len(x), min(len(x), 180000), replace=True, p=weights)
      net.train()
      for idx in np.array_split(ix, max(1, len(ix) // 4096)):
        xx = tx[idx]
        yy = ty[idx]
        out = net(xx)
        # Preserve a useful restoring torque gradient when all acceleration inputs rise.
        shift = torch.zeros_like(xx)
        shift[:, [1, 4, 5, 6, 7, 8, 9, 10]] = torch.tensor(0.05 / std[[1, 4, 5, 6, 7, 8, 9, 10]], dtype=torch.float32)
        slope = net(xx + shift) - out
        loss = ((out - yy) ** 2).mean() + 2 * torch.relu(0.006 - slope).square().mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
      with torch.no_grad():
        err = net(vt) - vy
        metric = float((err.square() * (1 + 2 * (abs(vy) < 0.1))).mean())
      if metric < score:
        score = metric
        best = copy.deepcopy(net.state_dict())
      if ep % 10 == 0:
        print(name, phase, ep, metric, flush=True)
    net.load_state_dict(best)
    if phase == 'public':
      export(net, mean, std, name.replace('.json', '_PUBLIC.json'))
  net.load_state_dict(best)
  return export(net, mean, std, name)


def combine(ds):
  return {k: np.concatenate([d[k] for d in ds]) for k in ds[0]}


def main():
  torch.set_num_threads(4)
  ds = {p.stem: dict(np.load(p)) for p in (S / 'nnlc_data').glob('*.npz')}
  pub = ds.pop('public')
  test = {k: v for k, v in ds.items() if k.startswith(('000001aa', '000001ab'))}
  val = next(v for k, v in ds.items() if k.startswith('000001a7'))
  own = combine([v for k, v in ds.items() if not k.startswith(('000001aa', '000001ab', '000001a7'))])
  print('counts', len(pub['y']), len(own['y']), len(val['y']), {k: len(v['y']) for k, v in test.items()}, flush=True)
  models = {'stock': json.loads((MODEL_DIR / 'HONDA_ODYSSEY.json').read_text())}
  for name, ext in [('v1', False), ('v2', True)]:
    models[name] = fit(f'HONDA_ODYSSEY_WILLIAM_{name.upper()}.json', ext, pub, own, val)
  scores = []
  for route, d in test.items():
    for name, p in models.items():
      x = np.c_[d['x'], d['extra']] if name == 'v2' else d['x']
      pred = predict_json(p, x)
      err = pred - d['y']
      mph = d['v'] * 2.23694
      masks = {'all': np.ones(len(err), bool), 'small_torque': abs(d['y']) < 0.1, 'large_torque': abs(d['y']) >= 0.1}
      masks.update({f'{lo}-{hi}mph': (mph >= lo) & (mph < hi) for lo, hi in [(0, 10), (10, 20), (20, 35), (35, 50), (50, 999)]})
      for band, m in masks.items():
        scores.append(
          {
            'route': route,
            'model': name,
            'band': band,
            'n': int(sum(m)),
            'mae': float(np.mean(abs(err[m]))) if m.any() else None,
            'rmse': float(np.sqrt(np.mean(err[m] ** 2))) if m.any() else None,
          }
        )
  (S / 'nnlc_scores.json').write_text(json.dumps(scores, indent=2))
  print('DONE', flush=True)


if __name__ == '__main__':
  main()
