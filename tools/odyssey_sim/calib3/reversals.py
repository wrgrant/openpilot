"""Paired recorded-motion ablations; isolates input bandwidth, not plant behavior."""

import json
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib3.extract import OUT
from tools.odyssey_sim.calib.identify import TEST
from tools.odyssey_sim.calib.evaluate import records, pieces
from tools.odyssey_sim.calib.controller import Controller
from tools.odyssey_sim.closedloop.controller import Config, model_from_array
from tools.odyssey_sim.replay.harness import metadata


def rate(u):
  return float(np.sum((u[1:] * u[:-1] < 0) & (abs(np.diff(u)) > 0.01)) / (len(u) * 0.01))


def run(meta, z, n, mode):
  c = Controller(meta, Config(nnlc=meta['params'].get('NeuralNetworkLateralControl') == b'1'))
  c.lac.pid.i = float(n[0, 12])
  c.lac.previous_measurement = float(n[0, 9])
  outputs = []
  times = z['t']
  plans = np.stack([z['acceleration'][:, 1], z['orientation'][:, 0], z['orientation'][:, 1]], axis=1)
  for _i, row in enumerate(n):
    j = max(0, np.searchsorted(n[:, 0], row[0] - (row[0] - n[0, 0]) % 0.1, side='right') - 1)
    # Forensic intervention: retain recorded slow integral, isolate fast inputs.
    c.lac.pid.i = float(row[12])
    s = row.tolist()
    if mode in ('sensors_10hz', 'all_10hz'):
      s[1:8] = n[j, 1:8]
      s[19:24] = n[j, 19:24]
    if mode in ('target_10hz', 'all_10hz'):
      s[8] = n[j, 8]
    if mode in ('tuning_constant', 'all_10hz'):
      s[16:19] = n[0, 16:19] if mode == 'tuning_constant' else n[j, 16:19]
    if mode == 'delay_constant':
      s[15] = n[0, 15]
    mi = max(0, np.searchsorted(times, (n[j, 0] if mode == 'all_10hz' else row[0]), side='right') - 1)
    if mode == 'future_constant':
      mi = max(0, np.searchsorted(times, n[0, 0], side='right') - 1)
    s = list(map(float, s))
    model = model_from_array(plans[mi])
    cs = NS(vEgo=s[1], aEgo=s[2], steeringAngleDeg=s[3], steeringRateDeg=s[4], steeringPressed=False, steeringRateLimited=False)
    lp = NS(roll=s[19], angleOffsetDeg=s[20])
    c.vm.update_params(max(s[22], 0.1), max(s[21], 0.1))
    ext = c.lac.extension
    ext.update_model_v2(model)
    ext.update_lateral_lag(s[15])
    c.lac.update_torque_parameters(s[16], s[18], s[17])
    ext.update_limits()
    u, _, _ = c.lac.update(True, cs, c.vm, lp, False, s[8] / max(s[1], 0.1) ** 2, NS(orientation=NS(pitch=s[23])), False, s[15])
    outputs.append(u)
  return np.array(outputs)


def main():
  result = []
  for route, d in records():
    key = route[:8]
    if key not in TEST:
      continue
    z = dict(np.load(OUT / f'{key}.npz'))
    meta = metadata(route)
    selections = list(pieces(d)) or list(pieces(d, 30))
    # Three representative windows per route; identical motion for every ablation.
    for wi in np.unique(np.linspace(0, len(selections) - 1, min(3, len(selections))).astype(int)):
      r = selections[wi]
      n = z['native'][(z['native'][:, 0] >= d['t'][r[0]]) & (z['native'][:, 0] <= d['t'][r[-1]])]
      row = {'route': key, 'window': int(wi), 'recorded': rate(n[1000:, 10]), 'modes': {}}
      for mode in ('native', 'sensors_10hz', 'target_10hz', 'all_10hz', 'tuning_constant', 'delay_constant', 'future_constant'):
        u = run(meta, z, n, mode)
        row['modes'][mode] = {
          'reversals_s': rate(u[1000:]),
          'correlation': float(np.corrcoef(u[1000:], n[1000:, 10])[0, 1]),
          'rmse': float(np.sqrt(np.mean((u[1000:] - n[1000:, 10]) ** 2))),
        }
      result.append(row)
      (OUT / 'reversals.json').write_text(json.dumps(result, indent=2))
      print(json.dumps(row), flush=True)


if __name__ == '__main__':
  main()
