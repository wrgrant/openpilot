#!/usr/bin/env python3
"""Analyze odyssey_pulse records and CAN in route folders. Outputs JSON for shake fitting.

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_eps_report.py <route> [...] -o pulse.json
Motor torque is magnitude only; no signed motor measurement is claimed.
"""
import argparse
from collections import defaultdict
from pathlib import Path
import json

import numpy as np

STEER_MAX = 4096
DBC = 'honda_odyssey_exl_2018_generated'
MIN_SHOTS, MIN_SHAMS = 10, 10  # per setting, direction and speed; three repeats are a screening pass only


def fine_angle(angle, rate, dt=.01):
  out = np.empty_like(angle)
  if not len(angle):
    return out
  x = angle[0]
  for i in range(len(angle)):
    x = min(max(x + rate[i] * (dt if np.isscalar(dt) else dt[i]), angle[i] - .05), angle[i] + .05)
    out[i] = x
  return out


def pulse_record(raw):
  try:
    d = json.loads(raw)
    d = d.get('msg', d)
    return d if isinstance(d, dict) and d.get('event') == 'odyssey_pulse' else None
  except (ValueError, TypeError, AttributeError):
    return None


def load(routes):
  from opendbc.can import CANParser
  from openpilot.tools.lib.logreader import LogReader
  records, rows = [], []
  for route in routes:
    parser = CANParser(DBC, [('STEERING_SENSORS', 0), ('STEER_STATUS', 0), ('STEER_MOTOR_TORQUE', 0)], 0)
    speed, yaw, output = 0., 0., 0.
    files = sorted(Path(route).glob('**/rlog.zst'))
    if Path(route).is_file():
      files = [Path(route)]
    for path in files:
      for m in LogReader(str(path)):
        w = m.which()
        if w == 'logMessage':
          r = pulse_record(m.logMessage)
          if r:
            records.append(r)
        elif w == 'carState':
          speed = m.carState.vEgo
        elif w == 'carControl' and len(m.carControl.angularVelocity) == 3:
          yaw = m.carControl.angularVelocity[2]
        elif w == 'carOutput':
          output = m.carOutput.actuatorsOutput.torque
        elif w == 'can':
          up = parser.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.can])])
          if 342 in up:
            v = parser.vl
            rows.append((m.logMonoTime/1e9, v['STEERING_SENSORS']['STEER_ANGLE'], v['STEER_STATUS']['STEER_ANGLE_RATE'],
                         v['STEER_MOTOR_TORQUE']['MOTOR_TORQUE'], speed*yaw, output))
  return records, np.asarray(sorted(rows), dtype=float).reshape(-1, 6)


def measurements(records, samples):
  """samples: monotonic time, CAN angle, CAN rate, motor magnitude, speed*yaw, carOutput torque."""
  out = []
  # Route id/start, not condition id alone: resets and repeated drives must remain independent observations.
  seen = set()
  for r in records:
    if not r.get('completed') or (r['start'], r['id']) in seen:
      continue
    seen.add((r['start'], r['id']))
    s = samples[(samples[:, 0] >= r['start']) & (samples[:, 0] < r['end'])]
    if len(s) < 100 or np.max(np.diff(s[:, 0])) > .05 or not np.isfinite(s).all():
      continue
    if s[0, 0]-r['start'] > .05 or r['end']-s[-1, 0] > .05:
      continue
    pre = s[:, 0] < r['start']+.5
    response = ~pre
    a = fine_angle(s[:, 1], s[:, 2], np.r_[.01, np.diff(s[:, 0])])
    delta = s[:, 4]-np.mean(s[pre, 4])
    sign = r['condition']['direction']
    duration = s[-1, 0]-s[0, 0]
    reversals = np.sum(np.diff(np.sign(delta[np.abs(delta) > .01])) != 0) / duration
    out.append({**r, 'wheel': sign*(np.mean(a[-10:])-np.mean(a[pre])), 'wheel_rate': float(np.max(np.abs(s[response, 2]))),
                'motor': float(np.mean(s[response, 3])-np.mean(s[pre, 3])),
                'average_command': float(sign*np.mean(s[(s[:, 0] >= r['start']+.5) &
                                                                    (s[:, 0] < r['start']+.5+r['condition']['duration']), 5])),
                'accel': float(np.max(np.abs(delta))), 'push_N': float(75*np.max(np.abs(delta))), 'reversals_hz': float(reversals)})
  return out


def interval(values, controls):
  """Deterministic independent-shot bootstrap; descriptive, not causal road matching."""
  rng = np.random.default_rng(301)
  a, b = np.asarray(values), np.asarray(controls)
  draws = np.mean(rng.choice(a, (2000, len(a))), axis=1)-np.mean(rng.choice(b, (2000, len(b))), axis=1)
  return np.quantile(draws, [.025, .975]).tolist()


def summarize(rows):
  groups, shams = defaultdict(list), defaultdict(list)
  for r in rows:
    c = r['condition']
    k = (r['band'], c['direction'])
    if c['kind'] == 'sham':
      shams[k].append(r)
    else:
      groups[(*k, c['kind'], c['height'], c['duration'], c.get('period', 0), c.get('duty', 0))].append(r)
  results = []
  for key, g in sorted(groups.items()):
    b, direction, kind, height, duration, period, duty = key
    sham = shams[(b, direction)]
    ci = interval([r['wheel'] for r in g], [r['wheel'] for r in sham]) if sham else None
    noise = max(.02, float(np.quantile(np.abs([r['wheel'] for r in sham]), .95))) if sham else None
    reliable = len(g) >= MIN_SHOTS and len(sham) >= MIN_SHAMS
    moved = bool(reliable and ci is not None and ci[0] > noise)
    result = {'band': b, 'direction': direction, 'kind': kind, 'height': height, 'duration': duration, 'period': period,
              'duty': duty, 'n': len(g), 'n_sham': len(sham), 'sufficient': reliable, 'wheel_interval_deg': ci,
              'reliably_moves': moved, 'below_7N': bool(moved and max(r['push_N'] for r in g) < 7),
              'last_direction_counts': {str(d): sum(r.get('last_direction') == d for r in g) for d in (-1, 0, 1)}}
    for metric in ('wheel', 'wheel_rate', 'motor', 'average_command', 'accel', 'push_N', 'reversals_hz'):
      result[metric] = float(np.mean([r[metric] for r in g]))
      if sham:
        result[metric+'_minus_sham'] = result[metric]-float(np.mean([r[metric] for r in sham]))
    if kind == 'train':
      result['nominal_average'] = height*duty
      result['delivered_command_ratio'] = result['average_command']/(height*duty)
      # Command average cannot establish equivalent steady torque through nonlinear EPS friction.
      ratio = result['delivered_command_ratio']
      result['rate_limit_corrected_on_time_s'] = min(period, period*duty/ratio) if ratio > 0 else None
      result['corrected_on_time'] = None
      result['correction_reason'] = 'Needs matched steady-torque calibration; pulse motor magnitude alone cannot identify it.'
    results.append(result)
  thresholds = []
  for b, d, duration in sorted({(r['band'], r['direction'], r['duration']) for r in results if r['kind'] == 'single'}):
    g = sorted([r for r in results if (r['band'], r['direction'], r['duration'], r['kind']) == (b, d, duration, 'single')], key=lambda r: r['height'])
    responding = [r for r in g if r['reliably_moves']]
    first = responding[0] if responding else None
    below = [r['height'] for r in g if first and r['height'] < first['height'] and r['sufficient'] and not r['reliably_moves']]
    thresholds.append({'band': b, 'direction': d, 'duration': duration, 'smallest_tested': first['height'] if first else None,
                       'grid_bracket': [max(below) if below else None, first['height']] if first else None})
  return {'schema': 1, 'mass_kg': 75, 'minimum_shots': MIN_SHOTS, 'minimum_shams': MIN_SHAMS,
          'thresholds': thresholds, 'conditions': results, 'completed_with_coverage': len(rows),
          'limits': 'Bootstrap resamples shots, not drives. Road crown confounds zero-baseline response; three repeats are screening only. Motor is unsigned.'}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('routes', nargs='+')
  ap.add_argument('-o', '--output', default='pulse.json')
  args = ap.parse_args()
  rows = []
  for route in args.routes:
    records, samples = load([route])  # monotonic clocks can overlap across boots; never align different routes together
    rows.extend({**r, 'route': route} for r in measurements(records, samples))
  result = summarize(rows)
  Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
  print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
  main()
