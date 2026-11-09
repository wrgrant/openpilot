"""Build segment-safe inverse training examples from other workers' read-only caches."""

import csv
import json
from collections import Counter
from pathlib import Path
import numpy as np
from opendbc.car.vehicle_model import VehicleModel
from tools.odyssey_sim.replay.harness import metadata

from tools.odyssey_sim.nnlc_train import SCRATCH as S

OFFSETS = np.array([-0.3, -0.2, -0.1, 0.3, 0.6, 1.0, 1.5])


def examples(t, v, y, roll, u, angle, rate, active, pressed, group, fw_weight=1.0):
  # Desired acceleration at command time is represented by achieved acceleration
  # after a fixed 320 ms actuation delay. No sample crosses a segment or dropout.
  ok = active.astype(bool) & ~pressed.astype(bool)
  ix = np.arange(10, len(t) - 22, 2)
  good = np.array([ok[i - 10 : i + 22].all() and np.max(np.diff(t[i - 10 : i + 22])) < 0.16 for i in ix], dtype=bool)
  ix = ix[good]
  ti = t[ix]
  target = np.interp(ti + 0.32, t, y)
  jerk = (np.interp(ti + 0.42, t, y) - np.interp(ti + 0.22, t, y)) / 0.2
  x = np.column_stack([v[ix], target, jerk, roll[ix], *[np.interp(ti + 0.32 + o, t, y) for o in OFFSETS], *[np.interp(ti + o, t, roll) for o in OFFSETS]])
  extra = np.column_stack([angle[ix], rate[ix], y[ix], *[np.interp(ti - o, t, y) for o in (0.1, 0.3, 0.5)], *[np.interp(ti - o, t, u) for o in (0.3, 0.5)]])
  keep = np.isfinite(x).all(1) & np.isfinite(extra).all(1) & (v[ix] > 1) & (abs(u[ix]) < 0.98) & (abs(target) < 4)
  return {'x': x[keep], 'extra': extra[keep], 'y': u[ix][keep], 'weight': np.full(sum(keep), fw_weight), 'group': np.full(sum(keep), group), 'v': v[ix][keep]}


def main():
  dest = S / 'nnlc_data'
  dest.mkdir(exist_ok=True)
  d = dict(np.load(S / 'plant_public.npz'))
  chunks = []
  fws = Counter()
  files = sorted(Path.home().joinpath('comma-logs/commaSteeringControl/HONDA_ODYSSEY_2018').glob('*.csv'))[::10]
  firmware = {}
  for p in files:
    with p.open() as f:
      fw = next(csv.DictReader(f))['epsFwVersion']
    fws[fw] += 1
    firmware[int(p.stem)] = fw
  for seg in np.unique(d['segment']):
    q = {k: v[d['segment'] == seg] for k, v in d.items()}
    t = q['t']
    rate = np.gradient(q['steeringAngleDeg'], t)
    chunks.append(
      examples(
        t,
        q['vEgo'],
        q['latAccelSteeringAngle'],
        q['roll'],
        -q['steerFiltered'],
        q['steeringAngleDeg'],
        rate,
        q['latActive'],
        q['steeringPressed'],
        int(seg),
        3.0 if 'A020' in firmware[int(seg)] else 1.0,
      )
    )
  np.savez_compressed(dest / 'public.npz', **{k: np.concatenate([c[k] for c in chunks]) for k in chunks[0]})
  own = {}
  for p in sorted((S / 'closedloop_data').glob('*.npz')):
    if p.stem.startswith('window'):
      continue
    q = dict(np.load(p))
    meta = metadata(p.stem)
    vm = VehicleModel(meta['carParams'])
    y = []
    for i in range(len(q['t'])):
      vm.update_params(q['stiffness'][i], q['steerRatio'][i])
      y.append(-vm.calc_curvature(np.radians(q['angle'][i] - q['angleOffset'][i]), q['vEgo'][i], q['roll'][i]) * q['vEgo'][i] ** 2)
    plant = np.load(S / 'plant_data' / (p.stem[:8] + '.npz'))
    # Applied CAN torque includes rate limiting and experimental probe offsets.
    q['steerCommand'] = np.interp(q['t'], plant['t'], plant['cmd'])
    # Treat each original minute as a segment, removing discontinuities as well.
    chunks = []
    minute = np.floor((q['t'] - q['t'][0]) / 60)
    for seg in np.unique(minute):
      m = minute == seg
      a = {k: v[m] for k, v in q.items()}
      chunks.append(examples(a['t'], a['vEgo'], np.asarray(y)[m], a['roll'], a['steerCommand'], a['angle'], a['rate'], a['active'], a['pressed'], int(seg)))
    data = {k: np.concatenate([c[k] for c in chunks]) for k in chunks[0]}
    np.savez_compressed(dest / (p.stem + '.npz'), **data)
    own[p.stem] = {
      'n': len(data['y']),
      'corr': float(np.corrcoef(data['x'][:, 1], data['y'])[0, 1]),
      'fw': [(str(f.ecu), str(bytes(f.fwVersion))) for f in meta['carParams'].carFw if str(f.ecu) == 'eps'],
    }
  (S / 'nnlc_data_manifest.json').write_text(json.dumps({'public_segments': len(files), 'firmware_segments': dict(fws), 'own': own}, indent=2))
  print('DONE', own, flush=True)


if __name__ == '__main__':
  main()
