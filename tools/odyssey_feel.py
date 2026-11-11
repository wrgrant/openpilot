#!/usr/bin/env python3
"""Feel test analysis: what the driver rated against what the van measurably did.

Usage: tools/odyssey_feel.py <route_dir> [<route_dir> ...]

Reads the `odyssey_feel` records (openpilot/sunnypilot/selfdrive/controls/lib/odyssey_feel.py) from each route's
rlogs and, for every trial, measures the stimulus the driver actually got over its STIM_S window:
  lane_p2p   peak-to-peak lane position, m: (laneLines[1].y[0] + laneLines[2].y[0]) / 2 from modelV2, 1 s moving
             average. The wander family's measure: it runs open loop, so the command says little about the sway.
  acc_peak   body lateral accel, m/s^2: vEgo x carControl.angularVelocity[2] (pose-filtered yaw rate), 0.5-3 Hz.
  jerk_peak  its derivative, m/s^3, same band. acc_peak and jerk_peak are the bump family's measures.
Then per family it prints each trial's rating against those measures, rating rates in bins of the measure, and the
size at which a logistic fit crosses 50% for "noticed" (rating >= 1), "mildly annoying" (>= 2) and "annoying" (>= 3).
Shams are in the fits: they put the noise floor at the bottom of the scale. Aborted trials are listed apart, by
stimulus and reason; a press that ends a big trial is a rating of its own.
"""
import argparse
from collections import defaultdict
import glob
import json
import os

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_feel import RATINGS, STIM_S, REST_S

BAND = (.5, 3.)  # Hz
LANE_SMOOTH_S = 1.
THRESHOLDS = ((1, 'noticed'), (2, 'mildly annoying'), (3, 'annoying'))
MEASURES = {'wander': ('lane_p2p',), 'bump': ('acc_peak', 'jerk_peak')}
UNITS = {'lane_p2p': 'm', 'acc_peak': 'm/s²', 'jerk_peak': 'm/s³'}


def feel_record(raw):
  try:
    d = json.loads(raw)
    d = d.get('msg', d)
    return d if isinstance(d, dict) and d.get('event') == 'odyssey_feel' else None
  except (ValueError, TypeError, AttributeError):
    return None


def rlogs(route):
  if os.path.isfile(route):
    return [route]
  segs = sorted(glob.glob(os.path.join(route, '*--*', 'rlog.zst')), key=lambda p: int(os.path.basename(os.path.dirname(p)).rsplit('--', 1)[1]))
  if not segs:
    raise FileNotFoundError(f'No */rlog.zst segments in {route}')
  return segs


def load(route):
  """The route's feel records and the three signals the measures need, each as (time, value) arrays."""
  from openpilot.tools.lib.logreader import LogReader
  records, lane, body = [], [], []
  v = np.nan
  for path in rlogs(route):
    for m in LogReader(path):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w == 'logMessage':
        r = feel_record(m.logMessage)
        if r is not None:
          records.append(r)
      elif w == 'carState':
        v = m.carState.vEgo
      elif w == 'carControl':
        av = m.carControl.angularVelocity
        if len(av) == 3 and np.isfinite(v):
          body.append((t, v * av[2]))
      elif w == 'modelV2':
        lines = m.modelV2.laneLines
        if len(lines) == 4 and len(lines[1].y) and len(lines[2].y):
          lane.append((t, (lines[1].y[0] + lines[2].y[0]) / 2))
  lane, body = (np.array(sorted(x), dtype=float).reshape(-1, 2) for x in (lane, body))
  return {'records': records, 'lane': lane, 'body': body}


def bandpass(x, dt, band=BAND):
  """FFT band-pass with zero padding at both ends, so a short window does not wrap around."""
  x = x - np.mean(x)
  n = len(x)
  padded = np.pad(x, (n, n))
  f = np.fft.rfftfreq(len(padded), dt)
  spec = np.fft.rfft(padded)
  spec[(f < band[0]) | (f > band[1])] = 0
  return np.fft.irfft(spec, len(padded))[n:2 * n]


def measure(d, t0, duration=STIM_S):
  """What the van did over [t0, t0 + duration]; NaN where a signal is missing."""
  out = {'lane_p2p': np.nan, 'acc_peak': np.nan, 'acc_rms': np.nan, 'jerk_peak': np.nan}
  lane = d['lane']
  inside = lane[(lane[:, 0] >= t0) & (lane[:, 0] <= t0 + duration)] if len(lane) else lane
  if len(inside) >= 10 * duration:  # at least half the model's 20 Hz frames
    dt = .05
    grid = np.arange(t0, t0 + duration, dt)
    y = np.interp(grid, inside[:, 0], inside[:, 1])
    k = round(LANE_SMOOTH_S / dt)
    smooth = np.convolve(y, np.ones(k) / k, mode='valid')
    out['lane_p2p'] = float(np.ptp(smooth))
  body = d['body']
  pad = 2.  # filter context on each side, cropped off before measuring
  inside = body[(body[:, 0] >= t0 - pad) & (body[:, 0] <= t0 + duration + pad)] if len(body) else body
  if len(inside) >= 50 * (duration + 2 * pad):
    dt = .01
    grid = np.arange(t0 - pad, t0 + duration + pad, dt)
    a = bandpass(np.interp(grid, inside[:, 0], inside[:, 1]), dt)
    jerk = np.gradient(a, dt)
    keep = (grid >= t0) & (grid <= t0 + duration)
    out.update(acc_peak=float(np.max(np.abs(a[keep]))), acc_rms=float(np.sqrt(np.mean(a[keep] ** 2))),
               jerk_peak=float(np.max(np.abs(jerk[keep]))))
  return out


def trials(d, route=''):
  """One row per started trial: its stimulus, how it ended, its last rating, felt-that taps, and the measures."""
  rows, by_trial = [], {}
  felt = [r for r in d['records'] if r.get('phase') == 'felt']
  for r in d['records']:
    phase, n = r.get('phase'), r.get('trial')
    if phase == 'start':
      row = {'route': route, 'trial': n, 'id': r['id'], 'start': r['mono_s'], 'stimulus': r['stimulus'],
             'family': r['stimulus']['family'], 'level': r['stimulus']['amplitude' if r['stimulus']['family'] == 'wander' else 'size'],
             'scale': r.get('scale', 1.), 'outcome': 'running', 'rating': None, 'abort': '', 'felt': 0}
      rows.append(row)
      by_trial[n] = row
    elif n in by_trial:
      row = by_trial[n]
      if phase == 'end':
        row['outcome'] = 'completed'
      elif phase == 'abort':
        row.update(outcome='aborted', abort=r.get('reason', ''), elapsed=r.get('elapsed'))
      elif phase == 'rating':
        row['rating'] = int(r['value'])  # the last tap wins: a later one corrects an earlier one
      elif phase == 'unrated':
        row['outcome'] = 'completed'
  for row in rows:
    t0 = row['start']
    row['felt'] = sum(t0 <= f.get('tap_s', f['mono_s']) <= t0 + STIM_S + REST_S for f in felt)
    row.update(measure(d, t0))
  return rows


def fit_logistic(x, y, ridge=1e-3):
  """P(y) = 1 / (1 + exp(-(b0 + b1 x))) by Newton's method, a little ridge so a clean split still converges.
  Returns the x where P = 0.5, or None when the data cannot place it (one class only, or a falling fit)."""
  x, y = np.asarray(x, float), np.asarray(y, float)
  ok = np.isfinite(x)
  x, y = x[ok], y[ok]
  if len(x) < 4 or y.min() == y.max() or np.ptp(x) == 0:
    return None
  mu, sd = x.mean(), x.std()
  X = np.column_stack([np.ones_like(x), (x - mu) / sd])
  b = np.zeros(2)
  for _ in range(100):
    p = 1 / (1 + np.exp(-X @ b))
    g = X.T @ (y - p) - ridge * b
    H = X.T @ (X * (p * (1 - p))[:, None]) + ridge * np.eye(2)
    step = np.linalg.solve(H, g)
    b += step
    if np.max(np.abs(step)) < 1e-8:
      break
  if b[1] <= 0:
    return None
  return float(mu - b[0] / b[1] * sd)


def bins(x, ratings, n_bins=4):
  """Rating rates in quantile bins of the measure, at least three trials a bin."""
  order = np.argsort(x)
  n_bins = max(1, min(n_bins, len(x) // 3))
  out = []
  for part in np.array_split(order, n_bins):
    if len(part):
      r = ratings[part]
      out.append((float(np.min(x[part])), float(np.max(x[part])), len(part),
                  *(float(np.mean(r >= k)) for k, _ in THRESHOLDS), float(np.mean(r))))
  return out


def report(rows):
  rated = [r for r in rows if r['rating'] is not None]
  print(f"{len(rows)} trials started, {sum(r['outcome'] == 'completed' for r in rows)} completed, {len(rated)} rated, "
        + f"{sum(r['outcome'] == 'aborted' for r in rows)} aborted, {sum(r['felt'] for r in rows)} felt-that taps in trials")
  results = {}
  for family, measures in MEASURES.items():
    fam = [r for r in rated if r['family'] == family]
    print(f"\n== {family}: {len(fam)} rated trials ==")
    if not fam:
      continue
    level = 'A m, period s' if family == 'wander' else 'size %, width s'
    print(f"{'trial':>5}  {level:<16} {'lane p2p m':>10} {'acc m/s²':>9} {'jerk m/s³':>9}  felt  rating")
    for r in sorted(fam, key=lambda r: (r['level'], r['trial'])):
      s = r['stimulus']
      label = (f"{s['amplitude'] * r['scale']:.2f} @ {s['period']:g}" if family == 'wander' else f"{100 * s['size']:.0f} @ {s['width']:g}")
      print(f"{r['trial']:>5}  {label:<16} {r['lane_p2p']:>10.3f} {r['acc_peak']:>9.3f} {r['jerk_peak']:>9.2f}  {r['felt']:>4}  "
            + f"{r['rating']} {RATINGS[r['rating']]}")
    ratings = np.array([r['rating'] for r in fam])
    for m in measures:
      x = np.array([r[m] for r in fam], dtype=float)
      ok = np.isfinite(x)
      print(f"\n  {m} ({UNITS[m]}), {ok.sum()} trials measured")
      if ok.sum() == 0:
        continue
      print(f"  {'range':>17} {'n':>3} {'noticed':>8} {'mild+':>6} {'annoy+':>7} {'mean':>5}")
      for lo, hi, n, p1, p2, p3, mean in bins(x[ok], ratings[ok]):
        print(f"  {lo:>7.3f} - {hi:<7.3f} {n:>3} {p1:>8.0%} {p2:>6.0%} {p3:>7.0%} {mean:>5.1f}")
      for k, name in THRESHOLDS:
        t = fit_logistic(x[ok], ratings[ok] >= k)
        results[(family, m, name)] = t
        print(f"  50% {name:<16} " + (f"{t:.3f} {UNITS[m]}" if t is not None else "n/a (needs both answers and a rising trend)"))
  aborted = [r for r in rows if r['outcome'] == 'aborted']
  if aborted:
    print('\n== aborted ==')
    counts = defaultdict(list)
    for r in aborted:
      counts[(r['family'], r['level'], r['abort'])].append(r['trial'])
    for (family, lvl, reason), ns in sorted(counts.items()):
      print(f"  {family} {lvl:g}: {reason} x{len(ns)} (trials {', '.join(map(str, ns))})")
  return results


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('routes', nargs='+', help='route folders of segment folders with rlog.zst, or single rlogs')
  a = ap.parse_args()
  rows = []
  for route in a.routes:
    d = load(route)
    found = trials(d, route)
    print(f"{route}: {len(found)} feel trials, {len(d['lane'])} lane frames, {len(d['body'])} body frames")
    rows += found
  if not rows:
    print('No odyssey_feel trials: was the feel test selected (OdysseyABExperiment = feel) and started on the panel?')
    return
  report(rows)


if __name__ == '__main__':
  main()
