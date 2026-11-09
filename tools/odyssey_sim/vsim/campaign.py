"""Inventory, maximal autonomous windows and isolated real-model batch experiments."""

import argparse
from datetime import datetime, UTC
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np

ROUTES = ('000001a3', '000001a7', '000001aa', '000001ab', '0000019f', '000001a0', '000001a2')


def paired_interval(candidate, baseline):
  a, b = np.asarray(candidate), np.asarray(baseline)
  if a.shape != b.shape or not len(a) or not np.isfinite(a - b).all():
    raise ValueError('finite matched windows required')
  delta = a - b
  if len(delta) < 2:
    return {'n': len(delta), 'mean': float(delta.mean()), 'ci95': None, 'reason': 'only one independent window'}
  boot = np.random.default_rng(903).choice(delta, (10000, len(delta)), replace=True).mean(axis=1)
  return {'n': len(delta), 'mean': float(delta.mean()), 'ci95': np.percentile(boot, [2.5, 97.5]).tolist()}


def eligible_runs(t, ok):
  ix = np.flatnonzero(ok)
  return [r for r in np.split(ix, np.where((np.diff(ix) > 1) | (np.diff(t[ix]) > 0.025))[0] + 1) if len(r) > 1 and t[r[-1]] - t[r[0]] >= 45]


def inventory(route):
  segs = sorted(route.glob('*--*'), key=lambda p: int(p.name.rsplit('--', 1)[1]))
  result = []
  for i, seg in enumerate(segs):
    sizes = {n: (seg / n).stat().st_size if (seg / n).exists() else 0 for n in ('fcamera.hevc', 'ecamera.hevc')}
    next_filling = i + 1 < len(segs) and any('camera.hevc' in p.name and p.stat().st_size > 0 for p in segs[i + 1].iterdir())
    result.append({'segment': int(seg.name.rsplit('--', 1)[1]), 'sizes': sizes, 'ready': all(sizes.values()) and next_filling})
  return result


def manifest(route):
  from tools.odyssey_sim.vsim.closed_loop import native_data
  from tools.odyssey_sim.vsim.run import records

  key = route.name.split('_')[1].split('--')[0]
  rows, indexes, _, _ = records(route)
  d = native_data(key)
  t = d['t']
  fields = ['v', 'roll', 'angle_offset', 'sr', 'stiffness', 'yaw', 'body', 'act', 'pitch', 'delay', 'factor', 'friction', 'torque_offset', 'a_ego']
  ok = (d['active'] > 0) & (d['pressed'] == 0) & (abs(d['drv']) < 500) & (d['blinker'] == 0) & (d['v'] > 5)
  ok &= np.isfinite([d[k] for k in fields]).all(axis=0)
  ready = {r['segment'] for r in inventory(route) if r['ready']}
  windows = []
  for j, r in enumerate(eligible_runs(t, ok)):
    # Round inward to leave the native eligibility endpoints outside scoring.
    lo, hi = t[r[0]] + 0.03 - rows[0][0], t[r[-1]] - 0.03 - rows[0][0]
    context = [md for ts, md, _ in rows if lo - 15 <= ts - rows[0][0] < hi]
    pairs = [indexes[c].get(fid) for md in context for c, fid in [('narrow', md.frameId), ('wide', md.frameIdExtra)]]
    complete = bool(pairs) and all(p is not None and int(p[0].name.rsplit('--', 1)[1]) in ready for p in pairs)
    windows.append({'id': f'{key}_{j:02}', 'lo': float(lo), 'hi': float(hi), 'ready': complete, 'native_start': float(t[r[0]]), 'native_end': float(t[r[-1]])})
  return windows


def completion_status(root, inventory):
  from tools.odyssey_sim.vsim.candidates import ARMS

  summary = {}
  for key in ROUTES:
    item = inventory.get(key, {})
    windows = item.get('windows')
    ready = windows is not None and any(v['ready'] for v in item.get('video', []))
    arms = ARMS if key in ROUTES[:2] else ('stock', 'S3', 'S5')
    if ready:
      ready = all(w['ready'] and all((root / f'{w["id"]}_{arm}.json').exists() for arm in arms) for w in windows)
    summary[key] = {'complete': ready, 'windows': len(windows) if windows is not None else None}
  return {'at': datetime.now(UTC).isoformat(), 'routes': summary, 'complete': all(v['complete'] for v in summary.values())}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch', type=Path, required=True)
  ap.add_argument('--routes', nargs='+', default=ROUTES[:4])
  ap.add_argument('--arms', nargs='+', default=['stock'])
  args = ap.parse_args()
  args.scratch.mkdir(parents=True, exist_ok=True)
  inventory_path = args.scratch / 'inventory.json'
  status = json.loads(inventory_path.read_text()) if inventory_path.exists() else {}
  for key in args.routes:
    route = next((Path.home() / 'comma-logs').glob('*_' + key + '--*'), None)
    if route is None:
      status[key] = {'unavailable_route': True}
      continue
    try:
      windows = manifest(route)
    except FileNotFoundError as e:
      status[key] = {'unavailable_cache': str(e), 'video': inventory(route)}
      continue
    status[key] = {'windows': windows, 'video': inventory(route)}
    (args.scratch / 'inventory.json').write_text(json.dumps(status, indent=2))
    for w in windows:
      if not w['ready']:
        continue
      for arm in args.arms:
        stem = args.scratch / f'{w["id"]}_{arm}'
        if stem.with_suffix('.json').exists():
          continue
        env = dict(os.environ, VSIM_ARM=arm, PYTHONDONTWRITEBYTECODE='1')
        command = [
          sys.executable,
          '-m',
          'tools.odyssey_sim.vsim.run',
          '--route',
          str(route),
          '--window',
          f'{w["lo"]}:{w["hi"]}',
          '--controller',
          'tools.odyssey_sim.vsim.candidates:candidate',
          '--scratch',
          str(args.scratch / f'params_{w["id"]}_{arm}'),
          '--output',
          str(stem.with_suffix('.json')),
        ]
        print(datetime.now(UTC).isoformat(), w['id'], arm, flush=True)
        with stem.with_suffix('.log').open('w') as out:
          completed = subprocess.run(command, env=env, stdout=out, stderr=subprocess.STDOUT, check=False)
        if completed.returncode:
          stem.with_suffix('.json').write_text(json.dumps({'process_failure': completed.returncode}))
  (args.scratch / 'inventory.json').write_text(json.dumps(status, indent=2))
  (args.scratch / 'wait_status.json').write_text(json.dumps(completion_status(args.scratch, status), indent=2))


if __name__ == '__main__':
  main()
