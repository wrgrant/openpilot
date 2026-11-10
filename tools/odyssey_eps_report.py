#!/usr/bin/env python3
"""Pulse-shot analysis from closed rlogs; JSON plus a human report, with no device access.

Samples: time, angle_deg, rate_deg_s, unsigned_motor, speed*yaw, output,
         driver_sensor, lane_offset_m, requested_command (optional).
Yaw is carControl.angularVelocity: pose filtered, 75 ms later than raw gyro.
"""
import argparse
from collections import defaultdict
from pathlib import Path
import json

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import (
  LEAD_S, TAIL_S, HARD_SHOT_S, HARD_DRIVER_TORQUE, DRIVER_TORQUE, DRIVER_SECONDS, RAMP_STEP_S, session,
)

DBC = 'honda_odyssey_exl_2018_generated'
MIN_SHOTS, MIN_SHAMS = 10, 10


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


def segment_key(path):
  """Numeric order matters at segment 10, and shots can span a segment boundary."""
  p = Path(path)
  name = p.parent.name
  try:
    return name.rsplit('--', 1)[0], int(name.rsplit('--', 1)[1])
  except ValueError:
    return name, 0


def load(routes):
  from opendbc.can import CANParser
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_lane import lane_pose
  records, rows = [], []
  for route in routes:
    parser = CANParser(DBC, [('STEERING_SENSORS', 0), ('STEER_STATUS', 0), ('STEER_MOTOR_TORQUE', 0)], 0)
    yaw, output, command, offset = np.nan, np.nan, np.nan, np.nan
    updated = defaultdict(lambda: -np.inf)
    files = sorted(Path(route).glob('**/rlog.zst'), key=segment_key)
    if Path(route).is_file():
      files = [Path(route)]
    if not files:
      raise FileNotFoundError(f'No rlog.zst files in {route}')
    for path in files:
      for m in LogReader(str(path)):
        w, now = m.which(), m.logMonoTime/1e9
        if w == 'logMessage':
          r = pulse_record(m.logMessage)
          if r:
            records.append(r)
        elif w == 'carControl':
          command = m.carControl.actuators.torque
          if len(m.carControl.angularVelocity) == 3:
            yaw = m.carControl.angularVelocity[2]
            updated['yaw'] = now
          updated['command'] = now
        elif w == 'carOutput':
          output = m.carOutput.actuatorsOutput.torque
          updated['output'] = now
        elif w == 'modelV2':
          pose = lane_pose(m.modelV2)
          offset = pose[0] if pose else np.nan
          updated['lane'] = now
        elif w == 'can':
          up = parser.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.can])])
          if 427 in up or 'STEER_MOTOR_TORQUE' in up:
            updated['motor'] = now
          # Parser message addresses vary by DBC; updated names are available in vl_all.
          if parser.vl_all['STEER_MOTOR_TORQUE']['MOTOR_TORQUE']:
            updated['motor'] = now
        elif w == 'carState':
          cs = m.carState
          motor = parser.vl['STEER_MOTOR_TORQUE']['MOTOR_TORQUE'] if now-updated['motor'] < .05 else np.nan
          rows.append((now, cs.steeringAngleDeg, cs.steeringRateDeg, motor,
                       cs.vEgo*yaw if now-updated['yaw'] < .05 else np.nan,
                       output if now-updated['output'] < .05 else np.nan, cs.steeringTorque,
                       offset if now-updated['lane'] < .15 else np.nan,
                       command if now-updated['command'] < .05 else np.nan))
  return records, np.asarray(sorted(rows), dtype=float).reshape(-1, 9)


def bandpass(x, dt):
  """FFT 0.3-3 Hz, zero padded on both sides; short shots cannot resolve 0.3 Hz well."""
  x = x-np.mean(x)
  n = len(x)
  padded = np.pad(x, (n, n))
  f = np.fft.rfftfreq(len(padded), dt)
  return np.fft.irfft(np.fft.rfft(padded)*((f >= .3) & (f <= 3)), len(padded))[n:2*n]


def steps(t, u):
  edges = np.r_[0, np.flatnonzero(np.abs(np.diff(u)) > 1e-6)+1, len(u)]
  return [(i, j) for i, j in zip(edges[:-1], edges[1:], strict=True) if j-i >= 3 and abs(u[i]) > 1e-6]


def measurements(records, samples):
  out, seen = [], set()
  samples = np.asarray(samples)
  if not samples.size:
    samples = np.empty((0, 9))
  for r in records:
    if 'completed' not in r or (r.get('start'), r['id']) in seen:
      continue
    seen.add((r.get('start'), r['id']))
    s = samples[(samples[:, 0] >= r['start']) & (samples[:, 0] < r['end'])]
    problems = [] if r['completed'] else ['aborted: '+r.get('abort', 'unknown')]
    coverage = (len(s) >= 100 and s[0, 0]-r['start'] <= .05 and r['end']-s[-1, 0] <= .05
                and np.max(np.diff(s[:, 0])) <= .05 and np.isfinite(s[:, :4]).all())
    if not coverage:
      problems.append('insufficient samples')
    sensor = float(np.max(np.abs(s[:, 6]))) if len(s) and s.shape[1] > 6 and np.isfinite(s[:, 6]).all() else None
    if sensor is None:
      problems.append('wheel sensor unavailable')
    elif sensor >= HARD_DRIVER_TORQUE:
      problems.append('wheel sensor limit')
    if len(s) > 1 and s.shape[1] > 6:
      high = np.abs(s[:, 6]) >= DRIVER_TORQUE
      edges = np.r_[0, np.flatnonzero(np.diff(high))+1, len(s)]
      if any(high[i] and s[j-1, 0]-s[i, 0] >= DRIVER_SECONDS for i, j in zip(edges[:-1], edges[1:], strict=True)):
        problems.append('sustained wheel sensor limit')
    # Legacy six-column analysis callers have no sensor; retain their screening use only.
    if s.shape[1] == 6:
      problems = [p for p in problems if p != 'wheel sensor unavailable']
    row = {**r, 'clean': not problems, 'problems': problems, 'samples': len(s), 'wheel_sensor_peak': sensor}
    out.append(row)
    if not coverage:
      continue
    t = s[:, 0]-r['start']
    dt = float(np.median(np.diff(t)))
    pre = t < LEAD_S
    if not np.any(pre):
      row['clean'] = False
      row['problems'].append('no lead-in')
      continue
    c = r['condition']
    active = (t >= LEAD_S-1e-6) & (t < LEAD_S+c['duration']-1e-6)
    a = fine_angle(s[:, 1], s[:, 2], np.r_[dt, np.diff(t)])
    u = s[:, 8].copy() if s.shape[1] > 8 else s[:, 5].copy()
    frames = [f for f in records if f.get('phase') == 'frame' and f.get('id') == r['id'] and f.get('start') == r['start']]
    if frames:
      frames.sort(key=lambda f: f['mono_s'])
      u = np.interp(s[:, 0], [f['mono_s'] for f in frames], [f['command'] for f in frames])
    if not np.isfinite(u).all() or not np.isfinite(s[:, 5]).all() or not np.any(active):
      row['clean'] = False
      row['problems'].append('command/output unavailable')
      continue
    motor = s[:, 3]-np.mean(s[pre, 3])
    row.update(wheel=float(c['direction']*(np.mean(a[-10:])-np.mean(a[pre]))),
               wheel_rate=float(np.max(np.abs(s[:, 2]))), motor=float(np.mean(motor[active])),
               average_command=float(c['direction']*np.mean(s[active, 5])),
               lane_movement_m=float(np.nanmax(np.abs(s[:, 7]-s[0, 7])))
               if s.shape[1] > 7 and np.isfinite(s[:, 7]).all() else None)
    if np.isfinite(s[:, 4]).all():
      accel = bandpass(s[:, 4], dt)
      significant = accel[np.abs(accel) > max(.001, np.max(np.abs(accel))*.1)]
      row.update(push_N=float(75*np.max(np.abs(accel))),
                 reversals_hz=float(np.count_nonzero(np.diff(np.sign(significant)))/ (t[-1]-t[0])))
    else:
      row.update(push_N=None, reversals_hz=None)
    row['levels'] = []
    row['pulses'] = []
    for i, j in steps(t, u):
      if not active[i] or c['kind'] == 'chirp':
        continue
      settled = slice(i+(j-i)//2, j)
      level = {'command_pct': float(100*u[i]), 'motor': float(np.mean(motor[settled])),
               'wheel_delta_deg': float(c['direction']*(a[j-1]-a[i])), 'duration_s': float((j-i)*dt)}
      row['levels'].append(level)
      if c['kind'] in ('pair', 'single'):
        next_start = next((ii for ii, _ in steps(t, u) if ii >= j), len(t))
        stop = min(next_start, int(np.searchsorted(t, t[j-1]+TAIL_S)))
        response = slice(i, stop)
        def delay(values, threshold, begin=i, end=stop, times=t):
          idx = np.flatnonzero(np.abs(values[begin:end]) > threshold)
          return float(times[begin+idx[0]]-times[begin]) if len(idx) else None
        row['pulses'].append({'role': 'rest' if not row['pulses'] else 'reversal',
                              'wheel_delta_deg': float(np.sign(u[i])*(a[stop-1]-a[i])),
                              'peak_rate_deg_s': float(np.max(np.abs(s[response, 2]))),
                              'peak_motor': float(np.max(motor[response])),
                              'motor_delay_s': delay(motor, max(.2, 3*np.std(motor[pre]))),
                              'wheel_delay_s': delay(s[:, 2]-np.mean(s[pre, 2]), max(.15, 3*np.std(s[pre, 2])))})
    if c['kind'] == 'chirp':
      row['signals'] = {'dt': dt, 'command': u.tolist(), 'rate': s[:, 2].tolist(),
                        'angle': a.tolist(), 'accel': s[:, 4].tolist()}
  return out


def interval(values, controls=(0.,)):
  rng = np.random.default_rng(301)
  a, b = np.asarray(values), np.asarray(controls)
  draws = np.mean(rng.choice(a, (500, len(a))), axis=1)-np.mean(rng.choice(b, (500, len(b))), axis=1)
  return np.quantile(draws, [.025, .975]).tolist()


def transfer(x, y, dt):
  """ARX(2,2) with a searched transport delay. Short sweeps require a model assumption.

  Fit on first 80%, score the held-out tail; spectral coherence is computed separately
  across independent repeats, never from one FFT (which would always return one).
  """
  x, y = np.asarray(x), np.asarray(y)
  if not np.isfinite(x).all() or not np.isfinite(y).all() or np.std(x) < 1e-7:
    return None
  best = None
  for delay in range(21):
    k = np.arange(23, len(x))
    X = np.column_stack((y[k-1], y[k-2], x[k-delay], x[k-delay-1], x[k-delay-2], np.ones(len(k))))
    split = int(.8*len(k))
    coef = np.linalg.lstsq(X[:split], y[k[:split]], rcond=None)[0]
    error = np.mean((X[split:]@coef-y[k[split:]])**2)
    if best is None or error < best[0]:
      best = error, delay, coef, float(1-error/max(np.var(y[k[split:]]), 1e-20))
  _, delay, coef, score = best
  a1, a2, b0, b1, b2, _ = coef
  result = []
  for hz in (.5, 1., 1.5, 2., 2.5, 3.):
    z = np.exp(-2j*np.pi*hz*dt)
    h = z**delay*(b0+b1*z+b2*z*z)/(1-a1*z-a2*z*z)
    result.append({'hz': hz, 'gain': float(abs(h)), 'phase_deg': float(np.angle(h, deg=True))})
  return {'frequencies': result, 'transport_delay_s': delay*dt, 'held_out_prediction_r2': score,
          'method': 'ARX(2,2); conditional on second-order linear plant', 'coherence': None}


def summarize(rows):
  good = [r for r in rows if r.get('clean', True) and 'wheel' in r]
  shams = defaultdict(list)
  for r in good:
    if r['condition']['kind'] == 'sham':
      shams[(r['band'], r['condition']['direction'])].append(abs(r['wheel']))
  def noise(r):
    v = shams[(r['band'], r['condition']['direction'])]
    return max(.02, float(np.quantile(v, .95))) if v else None
  maps, ramps, pairs, trains, chirps, thresholds = [], [], [], [], [], []
  grouped = defaultdict(list)
  for r in good:
    grouped[(r['band'], r['condition']['direction'], r['condition']['kind'])].append(r)
  for (band, direction, kind), group in sorted(grouped.items()):
    if kind == 'square':
      for sign in (-1, 1):
        levels = defaultdict(list)
        for r in group:
          for v in r['levels']:
            if np.sign(v['command_pct']) == sign:
              levels[round(abs(v['command_pct']), 4)].append(v['motor'])
        if len(levels) < 3:
          continue
        x = np.array(sorted(levels))
        y = np.array([np.mean(levels[v]) for v in x])
        candidates = []
        for dead in np.linspace(0, min(6, max(x)-.01), 601):
          xx = np.maximum(x-dead, 0)
          slope = float(xx@y/(xx@xx))
          candidates.append((float(np.sum((y-slope*xx)**2)), dead, slope))
        error, dead, slope = min(candidates)
        plausible = [v for v in candidates if v[0] <= error+max(.01, float(np.var(y-slope*np.maximum(x-dead, 0))))]
        maps.append({'band': band, 'first_direction': direction, 'sign': sign, 'deadzone_pct': float(dead),
                     'slope_per_pct': slope, 'deadzone_interval_pct': [float(min(v[1] for v in plausible)), float(max(v[1] for v in plausible))],
                     'slope_interval_per_pct': [min(v[2] for v in plausible), max(v[2] for v in plausible)],
                     'points': [{'command_pct': float(sign*v), 'motor': float(np.mean(levels[v])),
                                 'motor_interval': interval(levels[v]), 'n': len(levels[v])} for v in x]})
    elif kind == 'ramp':
      for r in group:
        floor = noise(r)
        levels = r['levels']
        moved = [v for v in levels if floor is not None and v['wheel_delta_deg'] > floor]
        first = moved[0] if moved else None
        below = [abs(v['command_pct']) for v in levels if first and abs(v['command_pct']) < abs(first['command_pct'])]
        ramps.append({'id': r['id'], 'band': band, 'direction': direction, 'noise_deg': floor, 'levels': levels,
                      'bracket_pct': [max(below, default=0.), abs(first['command_pct'])] if first else None})
    elif kind in ('pair', 'single'):
      for r in group:
        pairs.append({'id': r['id'], 'band': band, **r['condition'], 'pulses': r['pulses'], 'noise_deg': noise(r)})
    elif kind == 'train':
      for r in group:
        trains.append({'id': r['id'], 'band': band, **r['condition'], 'wheel_delta_deg': r['wheel'],
                       'mean_motor': r['motor'], 'average_command_pct': 100*r['average_command']})
    elif kind == 'chirp':
      for r in group:
        sig = r['signals']
        chirps.append({'id': r['id'], 'band': band, **r['condition'],
                       'command_to_wheel_rate': transfer(sig['command'], sig['rate'], sig['dt']),
                       # accel / speed = yaw; use logged entry speed, constrained within 0.5 m/s during shot.
                       'wheel_angle_to_yaw': transfer(sig['angle'], np.asarray(sig['accel'])/r.get('speed', 25.), sig['dt'])})
  for train in trains:
    ref = [(abs(v['command_pct']), v['motor']) for r in good
           if r['band'] == train['band'] and r['condition']['kind'] in ('square', 'ramp') for v in r['levels']
           if np.sign(v['command_pct']) == train['direction']]
    avg = train['height']*train['duty']*100
    if ref:
      ref.append((0., 0.))
    if ref and min(v[0] for v in ref) <= avg <= max(v[0] for v in ref):
      xx = sorted({v[0] for v in ref})
      yy = [np.mean([m for x, m in ref if x == h]) for h in xx]
      steady = float(np.interp(avg, xx, yy))
      train.update(steady_reference_motor=steady, motor_delivery_ratio=train['mean_motor']/steady if steady > .1 else None,
                   reference='interpolated square/ramp settled motor magnitude')
    else:
      train.update(steady_reference_motor=None, motor_delivery_ratio=None, reference='no steady reference bracketing average')
  # Queued single-level probes tighten the ramp bracket on later segments.
  for ramp in ramps:
    b = ramp['bracket_pct']
    if not b:
      continue
    for p in pairs:
      if (p['kind'] != 'single' or p['band'] != ramp['band'] or p['direction'] != ramp['direction'] or
          p['noise_deg'] is None or not p['pulses'] or not b[0] < p['height']*100 < b[1]):
        continue
      index = 1 if p['pulses'][0]['wheel_delta_deg'] > p['noise_deg'] else 0
      b[index] = p['height']*100
  asymmetry = []
  for m in maps:
    mirror = next((q for q in maps if q['band'] == m['band'] and q['first_direction'] == m['first_direction']
                   and q['sign'] == -m['sign']), None)
    if m['sign'] == 1 and mirror:
      asymmetry.append({'band': m['band'], 'first_direction': m['first_direction'],
                        'deadzone_difference_pct': m['deadzone_pct']-mirror['deadzone_pct'],
                        'slope_ratio_positive_negative': m['slope_per_pct']/mirror['slope_per_pct'] if mirror['slope_per_pct'] > 0 else None})
  # Preserve the old single-pulse screening API for older logs.
  for (band, direction, kind), group in sorted(grouped.items()):
    if kind != 'single':
      continue
    for duration in sorted({r['condition']['duration'] for r in group}):
      g = [r for r in group if r['condition']['duration'] == duration]
      responding = sorted({r['condition']['height'] for r in g if noise(r) is not None and r['wheel'] > noise(r)})
      first = responding[0] if responding else None
      below = [r['condition']['height'] for r in g if first and r['condition']['height'] < first]
      thresholds.append({'band': band, 'direction': direction, 'duration': duration, 'smallest_tested': first,
                         'grid_bracket': [max(below, default=0), first] if first else None})
  coverage = []
  for band in sorted({r['band'] for r in rows}) or ['45-70']:
    passes = sorted({r['id'].split(':')[0] for r in rows if r['band'] == band and ':' in r['id']}) or ['0']
    for repeat in passes:
      ids = {r['id'].split(':', 1)[-1] for r in good if r['band'] == band and r['id'].startswith(repeat+':')}
      missing = [name for name, _ in session() if name not in ids]
      coverage.append({'band': band, 'pass': repeat, 'good': 26-len(missing), 'total': 26, 'missing': missing})
  missing = []
  if not rows:
    missing.append('No pulse shots in these logs.')
  if not shams:
    missing.append('Need shams to judge wheel motion against noise.')
  for r in ramps:
    b = r['bracket_pct']
    missing.append(f"{r['band']} ramp {r['direction']:+}: breakaway {b[0]:g}-{b[1]:g}% (screening only)." if b else
                   f"{r['band']} ramp {r['direction']:+}: breakaway unbracketed.")
  if not maps:
    missing.append('Need square waves for EPS dead zone, slope and left/right comparison.')
  if not ramps:
    missing.append('Need ramps in both directions for breakaway.')
  if not pairs:
    missing.append('Need pulse pairs for rest/reversal response.')
  if not trains:
    missing.append('Need trains for PWM delivery.')
  if not chirps:
    missing.append('Need sweeps for frequency response.')
  elif any(c['command_to_wheel_rate'] is None or c['wheel_angle_to_yaw'] is None for c in chirps):
    missing.append('Some sweep transfer fits lack input excitation or yaw samples.')
  if any(r.get('push_N') is None for r in good):
    missing.append('Some shots lack fresh yaw samples for push in newtons.')
  if any(r.get('lane_movement_m') is None for r in good):
    missing.append('Some shots lack lane pose; lane displacement is unavailable.')
  if any(t['motor_delivery_ratio'] is None for t in trains):
    missing.append('PWM needs a steady reference at the same average command.')
  # Spectral coherence requires independent shots; pooled same-condition Fourier estimates.
  for c in chirps:
    peers = [r for r in good if r['condition']['kind'] == 'chirp' and r['band'] == c['band']
             and r['condition']['height'] == c['height']]
    for key, xx, yy in (('command_to_wheel_rate', 'command', 'rate'), ('wheel_angle_to_yaw', 'angle', 'accel')):
      if c[key] is None or len(peers) < 2:
        continue
      coherence = []
      for hz in (.5, 1., 1.5, 2., 2.5, 3.):
        spectra = []
        for p in peers:
          sig = p['signals']
          n = len(sig[xx])
          w = np.hanning(n)*np.exp(-2j*np.pi*hz*np.arange(n)*sig['dt'])
          spectra.append((np.dot(sig[xx], w), np.dot(sig[yy], w)))
        X, Y = np.array(spectra).T
        den = float(np.sum(abs(X)**2)*np.sum(abs(Y)**2))
        coherence.append({'hz': hz, 'value': float(np.clip(abs(np.sum(X.conj()*Y))**2/den, 0, 1)) if den > 1e-20 else None})
      c[key]['coherence'] = coherence
  public = [{k: v for k, v in r.items() if k != 'signals'} for r in rows]
  return {'schema': 2, 'mass_kg': 75, 'shots': public, 'clean': len(good), 'completed_with_coverage': len(good),
          'coverage': coverage, 'eps_map': maps, 'eps_asymmetry': asymmetry, 'ramps': ramps, 'pairs': pairs, 'trains': trains, 'chirps': chirps,
          'thresholds': thresholds, 'missing': missing,
          'signal_sources': {'command': 'odyssey_pulse frame.command or carControl.actuators.torque',
                             'motor': 'CAN unsigned MOTOR_TORQUE; sign labels describe command only',
                             'wheel': 'carState angle constrained rate integration, +/-0.05 degree',
                             'yaw': 'carControl.angularVelocity[2], pose filter; not raw gyro',
                             'yaw_extra_delay_s': .075, 'delay_compensated': False},
          'limits': ['One pass is screening, not a causal or statistically settled road test.',
                     'Intervals are descriptive grid residual bounds / independent-shot bootstrap, not road uncertainty.',
                     'Ramps have history and crown; motion is incremental per level, not a static friction measurement.',
                     'Short shots poorly resolve 0.3 Hz; push is 75 x band-passed speed*yaw (0.3-3 Hz).',
                     'Chirp gain/phase assume second-order linear dynamics; coherence needs two independent repeats.',
                     'Lane movement is model offset change, not ground-truth displacement.']}


def note(summary):
  if str(summary.get('exit_reason', '')).startswith('fault pattern:'):
    return 'Paused: check test data\n'+summary['exit_reason'].removeprefix('fault pattern: ')[:34]
  best = max(summary['coverage'], key=lambda c: c['good'])
  first = f"{best['good']} of 26 good"
  bracket = next((r['bracket_pct'] for r in summary['ramps'] if r['bracket_pct']), None)
  if bracket:
    first += f". Moves {bracket[0]:g}-{bracket[1]:g}%"
  second = 'Pass complete; review results' if not best['missing'] else needs(best['missing'])
  return '\n'.join(line[:34] for line in (first, second))


def needs(missing, width=34):
  """What is still missing in the driver's words, most numerous first, as much as fits a panel line."""
  words = {'sham': 'blank', 'square': 'square', 'ramp': 'ramp', 'pair': 'pair', 'train': 'train', 'chirp': 'sweep'}
  counts = {}
  for name in missing:
    kind = words.get(name.split('-')[0].rstrip('+-abci'), name)
    counts[kind] = counts.get(kind, 0)+1
  line = 'Need:'
  for kind, n in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])):
    part = f" {n} {kind}{'s' if n > 1 else ''},"
    if len(line)+len(part)-1 > width:
      break
    line += part
  return line.rstrip(',')


def suggest(summary, limit=6):
  """Each item has a Condition dict and an explanation; queue only item['condition']."""
  out = []
  def add(c, reason):
    if (0 <= c['height'] <= .12 and c['duration']+LEAD_S+TAIL_S < HARD_SHOT_S and
        c['direction'] in (-1, 1) and c not in [v['condition'] for v in out]):
      out.append({'condition': c, 'reason': reason})
  for r in summary['ramps']:
    b = r['bracket_pct']
    if b and b[1]-b[0] > .5:
      add({'kind': 'single', 'height': round((b[0]+b[1])/200, 5), 'duration': RAMP_STEP_S,
               'direction': r['direction'], 'period': 0., 'duty': 0.}, f"Halve {r['band']} breakaway bracket {b[0]:g}-{b[1]:g}%.")
  pairs = summary['pairs']
  for p in pairs:
    for q in pairs:
      if p['band'] != q['band'] or p['height'] != q['height'] or not p['pulses'] or not q['pulses']:
        continue
      if p['noise_deg'] is None or q['noise_deg'] is None:
        continue
      pm = p['pulses'][0]['wheel_delta_deg'] > p['noise_deg']
      qm = q['pulses'][0]['wheel_delta_deg'] > q['noise_deg']
      if pm == qm:
        continue
      if p['direction'] == -q['direction'] and p['period'] == q['period']:
        c = {k: p[k] for k in ('kind', 'height', 'duration', 'direction', 'period', 'duty')}
        add(c, 'Repeat disagreeing mirror pulse.')
      elif p['direction'] == q['direction'] and abs(p['period']-q['period']) > .02:
        width = round((p['period']+q['period'])/2, 3)
        add({'kind': 'pair', 'height': p['height'], 'duration': 2*width+.5, 'direction': p['direction'], 'period': width, 'duty': 0.},
            'Test pulse width between moving and still results.')
  return out[:max(0, min(6, limit))]


def report(summary):
  lines = [note(summary), f"{summary['clean']} clean shots; {len(summary['shots'])} recorded shots."]
  if summary.get('exit_reason'):
    lines.insert(0, 'Exit: '+summary['exit_reason'])
    lines.append(summary.get('recommendation', 'Lead: review the evidence and missing shots.'))
  for m in summary['eps_map']:
    lines.append(f"{m['band']} command sign {m['sign']:+}: motor dead zone {m['deadzone_pct']:.2f}%, slope {m['slope_per_pct']:.3f} units/%.")
  for t in summary['trains']:
    ratio = t['motor_delivery_ratio']
    lines.append(f"{t['id']}: {t['height']*100:g}% on {t['duty']*100:g}%; motor {t['mean_motor']:.2f}, " +
                 (f"{ratio*100:.0f}% of steady reference." if ratio is not None else 'no matched steady reference.'))
  for p in summary['pairs']:
    for v in p['pulses']:
      lines.append(f"{p['id']} {v['role']}: wheel {v['wheel_delta_deg']:.3f} deg, peak {v['peak_rate_deg_s']:.2f} deg/s, " +
                   f"motor {v['peak_motor']:.2f}, delays motor/wheel {v['motor_delay_s']}/{v['wheel_delay_s']} s.")
  for c in summary['chirps']:
    for key in ('command_to_wheel_rate', 'wheel_angle_to_yaw'):
      result = c[key]
      if result:
        for f in result['frequencies']:
          lines.append(f"{c['id']} {key} {f['hz']:g}/s: gain {f['gain']:.3g}, phase {f['phase_deg']:.1f} deg.")
  lines.append('Yaw: pose-filter carControl.angularVelocity; 75 ms extra delay, uncompensated.')
  for r in summary['shots']:
    lines.append(f"{r['id']}: {'clean' if r['clean'] else ', '.join(r['problems'])}; push {r.get('push_N')} N, " +
                 f"{r.get('reversals_hz')} reversals/s, sensor {r['wheel_sensor_peak']}, lane {r.get('lane_movement_m')} m.")
  lines.extend(summary['missing'])
  for c in summary['coverage']:
    lines.append(f"{c['band']} pass {c['pass']} missing: {', '.join(c['missing']) or 'none'}")
  lines.extend('Suggest: '+v['reason'] for v in suggest(summary))
  lines.extend(summary['limits'])
  return '\n'.join(lines)+'\n'


def analyze(routes):
  rows = []
  for route in routes:
    records, samples = load([route])
    rows.extend({**r, 'route': str(route)} for r in measurements(records, samples))
  return summarize(rows)


def write_summary(summary, output):
  output = Path(output)
  output.parent.mkdir(parents=True, exist_ok=True)
  output.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
  output.with_suffix('.txt').write_text(report(summary))


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('routes', nargs='+')
  ap.add_argument('-o', '--output', default='pulse.json')
  args = ap.parse_args()
  result = analyze(args.routes)
  write_summary(result, args.output)
  print(report(result), end='')


if __name__ == '__main__':
  main()
