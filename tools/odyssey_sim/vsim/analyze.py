"""Score complete paired windows and expose failures without survivor-only ranking."""

import argparse
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.vsim.transfer import perceive
from tools.odyssey_sim.vsim.campaign import paired_interval
from tools.odyssey_wander import bandpass
from tools.odyssey_sim.vsim.metrics import curve_scores, logged_lane_mask, lane_scores as score


def regression(rows):
  v = np.array([r['speed'] for r in rows])
  off, heading, road = perceive(rows, horizon=25)
  x = np.column_stack([bandpass(a) for a in (off, heading, road * v * v)])
  y = bandpass(np.array([r['curvature'] for r in rows]) * v * v)
  coef = np.linalg.lstsq(x, y, rcond=None)[0]
  return {'coefficients': coef.tolist(), 'r2': float(1 - np.var(y - x @ coef) / np.var(y)), 'condition': float(np.linalg.cond(x / x.std(axis=0)))}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch', type=Path, required=True)
  args = ap.parse_args()
  root = args.scratch
  inv = json.loads((root / 'inventory.json').read_text())
  windows = {w['id']: w for r in inv.values() for w in r.get('windows', [])}
  results = {}
  failures = {}
  fits = {}
  native = {}
  from tools.odyssey_sim.vsim.closed_loop import native_data

  for p in sorted(root.glob('0000*.json')):
    if p.stem.endswith(('.traces', '.motion')):
      continue
    ident = p.stem[:11]
    arm = p.stem[12:]
    w = windows.get(ident)
    if w is None:
      continue
    meta = json.loads(p.read_text())
    if 'process_failure' in meta or meta.get('closure_failure') or not p.with_suffix('.traces.json').exists():
      failures[p.stem] = meta.get('closure_failure', meta.get('failure', meta.get('process_failure')))
      continue
    rows = json.loads(p.with_suffix('.traces.json').read_text())
    motion = json.loads(p.with_suffix('.motion.json').read_text())
    if not rows or rows[-1]['t'] - rows[0]['t'] < w['hi'] - w['lo'] - 0.2:
      failures[p.stem] = 'incomplete requested window'
      continue
    key = ident[:8]
    origin = w['native_start'] + 0.03 - w['lo']
    if key not in native:
      native[key] = native_data(key)
    confidence = logged_lane_mask(rows, key, origin)
    result = score(rows, motion, confidence)
    result['curves'] = curve_scores(rows, motion, native[key], origin, confidence)
    results.setdefault(ident, {})[arm] = result
    fits[p.stem] = regression(rows)
  comparisons = {}
  for arm in ('v0', 'kp_half', 'kp_double', 'V1', 'S3', 'S5', 'friction', 'bad_delay'):
    pairs = [
      (r[arm], r['stock'])
      for key, r in results.items()
      if key[:8] in ('000001a3', '000001a7')
      and arm in r
      and 'stock' in r
      and r[arm]['simulated']['wander_available']
      and r['stock']['simulated']['wander_available']
    ]
    if not pairs:
      continue
    comparisons[arm] = {k: paired_interval([a['simulated'][k] for a, b in pairs], [b['simulated'][k] for a, b in pairs]) for k in ('wander_cm', 'max_offset_m')}
    comparisons[arm]['max_pose_m'] = paired_interval([a['max_pose_m'] for a, b in pairs], [b['max_pose_m'] for a, b in pairs])
    comparisons[arm]['spectrum_l1_change'] = [
      float(np.sum(abs(np.array(a['simulated']['power_m2']) - b['simulated']['power_m2'])) / max(np.sum(b['simulated']['power_m2']), 1e-12)) for a, b in pairs
    ]
    comparisons[arm]['spectrum_l1_interval'] = paired_interval(comparisons[arm]['spectrum_l1_change'], np.zeros(len(pairs)))
    comparisons[arm]['bandpower_m2'] = paired_interval([sum(a['simulated']['power_m2']) for a, b in pairs], [sum(b['simulated']['power_m2']) for a, b in pairs])
  (root / 'analysis.json').write_text(json.dumps({'windows': results, 'failures': failures, 'paired': comparisons, 'natural_free_fits': fits}, indent=2))
  print(json.dumps({'completed_windows': len(results), 'failures': failures, 'paired': comparisons}, indent=2))


if __name__ == '__main__':
  main()
