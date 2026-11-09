"""Immutable prospective calculation, candidate minus stock across historical roads."""

import argparse
from datetime import datetime, UTC
import hashlib
import json
from pathlib import Path
import numpy as np
from tools.odyssey_sim.vsim.campaign import paired_interval, completion_status

METRICS = ('wander_cm', 'curve_run_wide_m', 'body_jerk_rms_mps3')


def prediction(windows, arm, metric):
  candidate, baseline = [], []
  for variants in windows.values():
    if arm not in variants or 'stock' not in variants:
      continue
    x, y = variants[arm], variants['stock']
    if metric == 'wander_cm':
      a, b = x['simulated']['wander_cm'], y['simulated']['wander_cm']
      if a is None or b is None:
        continue
    elif metric == 'curve_run_wide_m':
      xe, ye = x['curves']['events'], y['curves']['events']
      if not xe or len(xe) != len(ye):
        continue
      pairs = [(a, b) for a, b in zip(xe, ye, strict=True) if a['simulated_run_wide_m'] is not None and b['simulated_run_wide_m'] is not None]
      if not pairs:
        continue
      a = float(np.median([a['simulated_run_wide_m'] for a, b in pairs]))
      b = float(np.median([b['simulated_run_wide_m'] for a, b in pairs]))
    elif metric == 'body_jerk_rms_mps3':
      a, b = x['curves']['simulated_body_jerk_rms_mps3'], y['curves']['simulated_body_jerk_rms_mps3']
    else:
      raise ValueError(metric)
    candidate.append(a)
    baseline.append(b)
  return paired_interval(candidate, baseline) if candidate else {'n': 0, 'mean': None, 'ci95': None, 'reason': 'no complete paired events'}


def calculate(windows):
  return {arm: {metric: prediction(windows, arm, metric) for metric in METRICS} for arm in ('S3', 'S5')}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--scratch', type=Path, required=True)
  ap.add_argument('--finalize', action='store_true')
  args = ap.parse_args()
  root = args.scratch
  path = root / 'predictions.json'
  digest = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
  now = datetime.now(UTC).isoformat()
  analysis = json.loads((root / 'analysis.json').read_text())
  if path.exists():
    record = json.loads(path.read_text())
    if record['formula_sha256'] != digest:
      raise ValueError('registered prediction formula changed; preserve the original registration')
  else:
    record = {
      'version': 2,
      'registered_at_utc': now,
      'formula_sha256': digest,
      'locked': False,
      'status': 'formula preregistered; all-route numerical forecast awaiting historical video arrival',
      'initial_training_only_snapshot': calculate(analysis['windows']),
      'named_historical_routes': ['000001a3', '000001a7', '000001aa', '000001ab', '0000019f', '000001a0', '000001a2'],
      'units': {'wander_cm': 'cm RMS', 'curve_run_wide_m': 'm', 'body_jerk_rms_mps3': 'm/s3'},
      'estimator': 'Paired means; 10000 seeded window bootstraps. Visibility-gated wander/curve widths; body jerk uses all complete pairs.',
      'interval_caution': 'Conditional 95% confidence intervals, not calibrated prediction intervals for a new road drive.',
      'road_ab_settle_s': 30,
      'simulation_warmup_s': 15,
      'ab_scoring': 'Existing odyssey_wander --ab nnlc_model reports wander only; curve and jerk use protocol.json additions.',
      'new_drive_access': 'No device access or new-drive A/B results read. Local inventory cannot establish whether an untransferred drive exists.',
      'local_routes_at_registration': [p.name for p in (Path.home() / 'comma-logs').glob('04b*')],
      'predictions': None,
    }
  if args.finalize and not record['locked']:
    inventory = json.loads((root / 'inventory.json').read_text())
    wait = completion_status(root, inventory)
    if not wait['complete']:
      raise ValueError('cannot finalize before every historical video route is scored')
    from tools.odyssey_sim.vsim.candidates import ARMS

    expected = {
      f'{w["id"]}_{arm}'
      for key, item in inventory.items()
      for w in item.get('windows', [])
      for arm in (ARMS if key in ('000001a3', '000001a7') else ('stock', 'S3', 'S5'))
    }
    observed = {f'{w}_{arm}' for w, variants in analysis['windows'].items() for arm in variants} | set(analysis['failures'])
    if expected - observed:
      raise ValueError('analysis is stale: rerun analyze before finalizing predictions')
    record.update(
      locked=True,
      computed_at_utc=now,
      predictions=calculate(analysis['windows']),
      analysis_sha256=hashlib.sha256((root / 'analysis.json').read_bytes()).hexdigest(),
      status='numerical forecast locked from preregistered formula; no road-ranking validation claimed',
    )
  if not path.exists() or args.finalize:
    path.write_text(json.dumps(record, indent=2))
  print('Prediction formula registered:', record['registered_at_utc'], 'numerical forecast locked:', record['locked'])


if __name__ == '__main__':
  main()
