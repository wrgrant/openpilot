"""Feel analysis on a synthetic route written as real rlogs: a simulated driver notices sway above 0.2 m and bumps above
0.1 m/s^2, and the script must measure each trial's stimulus and place the thresholds between the levels that bracket them."""
import contextlib
import io
import json
import math
import os
import tempfile
import unittest

import numpy as np
import zstandard as zstd

from openpilot.cereal import log, messaging
from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_feel as of
from tools import odyssey_feel as feel

V = 27.
TRIAL_S = of.STIM_S + of.REST_S


def log_message(t, **record):
  m = log.Event.new_message()  # logMessage is text, not a struct: new_message cannot init it
  m.logMonoTime = int(t * 1e9)
  m.logMessage = json.dumps({'msg': {'event': 'odyssey_feel', **record}})
  return m


def synthetic_route(path, trials):
  """trials: (stimulus, rating or None, abort reason or '') in order, one every TRIAL_S. The van sways 0.6 x the
  commanded wander (the model's centring fights it), and each bump gives a 0.3 s bump of body accel of 4 x its size."""
  rng = np.random.default_rng(0)
  msgs = []
  t_end = 5 + TRIAL_S * len(trials) + 5
  lane = np.zeros(round(t_end * 100))
  body = rng.normal(0, .005, len(lane))
  t_grid = np.arange(len(lane)) / 100
  for k, (stim, rating, abort) in enumerate(trials):
    t0 = 5 + k * TRIAL_S
    n = k + 1
    msgs.append(log_message(t0, phase='start', trial=n, id=f'0:{k}', mono_s=t0, stimulus=of.asdict(stim), scale=stim.scale, stim_s=of.STIM_S))
    window = (t_grid >= t0) & (t_grid < t0 + of.STIM_S)
    if stim.family == 'wander':
      # the sway itself: integrate the command twice and keep 0.6 of it
      a = np.array([stim.accel(t - t0) for t in t_grid[window]])
      lane[window] = .6 * np.cumsum(np.cumsum(a) * .01) * .01
    else:
      for j, b in enumerate(stim.blips):
        sign = stim.direction * (1 if j % 2 == 0 else -1)
        body += sign * 4 * stim.size * np.exp(-.5 * ((t_grid - t0 - b - stim.width / 2) / .1) ** 2)
    if abort:
      msgs.append(log_message(t0 + 5, phase='abort', trial=n, id=f'0:{k}', reason=abort, elapsed=5., mono_s=t0 + 5, stimulus=of.asdict(stim)))
      continue
    msgs.append(log_message(t0 + of.STIM_S, phase='end', trial=n, id=f'0:{k}', mono_s=t0 + of.STIM_S))
    if rating is not None:
      msgs.append(log_message(t0 + of.STIM_S + 2, phase='rating', trial=n, id=f'0:{k}', value=rating, label=of.RATINGS[rating],
                              tap_s=t0 + of.STIM_S + 2, mono_s=t0 + of.STIM_S + 2))
      if rating >= 1:
        msgs.append(log_message(t0 + 8, phase='felt', tap_s=t0 + 8, mono_s=t0 + 8, state='stimulus', trial=n))
  lane += rng.normal(0, .01, len(lane))
  for i, t in enumerate(t_grid):
    if i % 10 == 0:
      cs = messaging.new_message('carState')
      cs.logMonoTime = int(t * 1e9)
      cs.carState.vEgo = V
      msgs.append(cs)
    cc = messaging.new_message('carControl')
    cc.logMonoTime = int(t * 1e9) + 1000
    cc.carControl.angularVelocity = [0., 0., float(body[i] / V)]
    msgs.append(cc)
    if i % 5 == 0:
      md = messaging.new_message('modelV2')
      md.logMonoTime = int(t * 1e9) + 2000
      lines = md.modelV2.init('laneLines', 4)
      for j, y in enumerate((-5.4, -1.8 - lane[i], 1.8 - lane[i], 5.4)):
        lines[j].y = [float(y)]
      md.modelV2.laneLineProbs = [.9] * 4
      msgs.append(md)
  msgs.sort(key=lambda m: m.logMonoTime)
  seg = os.path.join(path, 'synthetic--0')
  os.makedirs(seg)
  with open(os.path.join(seg, 'rlog.zst'), 'wb') as f:
    f.write(zstd.compress(b''.join(m.to_bytes() for m in msgs), 3))


def rate(noticed, annoying, x):
  return 0 if x < noticed else 3 if x >= annoying else 1


class TestFeelAnalysis(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    stims = []
    for a in of.AMPLITUDES:
      stims.append((of.Stimulus('wander', amplitude=a, period=12., direction=1), 'wander'))
      stims.append((of.Stimulus('wander', amplitude=a, period=6., direction=-1), 'wander'))
    for s in of.SIZES:
      stims.append((of.Stimulus('bump', size=s, width=.1, direction=1, blips=(3., 9., 15.)), 'bump'))
      stims.append((of.Stimulus('bump', size=s, width=.3, direction=-1, blips=(2.5, 8., 14.)), 'bump'))
    trials = []
    for stim, family in stims:  # the driver's (hidden) sense: commanded sway 0.6 x 2A, bump accel 4 x size
      x = .6 * 2 * stim.amplitude * stim.scale if family == 'wander' else 4 * stim.size
      trials.append((stim, rate(.2, .4, x) if family == 'wander' else rate(.1, .3, x), ''))
    trials.append((of.Stimulus('bump', size=.08, width=.3, direction=1, blips=(2., 8., 14.)), None, "you're steering"))
    cls.dir = tempfile.TemporaryDirectory()
    synthetic_route(cls.dir.name, trials)
    cls.trials = trials

  @classmethod
  def tearDownClass(cls):
    cls.dir.cleanup()

  def test_measures_and_thresholds(self):
    d = feel.load(self.dir.name)
    rows = feel.trials(d, 'synthetic')
    self.assertEqual(len(rows), len(self.trials))
    self.assertEqual(sum(r['outcome'] == 'aborted' for r in rows), 1)
    by = {(r['family'], r['level'], r['stimulus']['period'] or r['stimulus']['width']): r for r in rows if r['outcome'] == 'completed'}
    big = by[('wander', .25, 12.)]
    self.assertAlmostEqual(big['lane_p2p'], .6 * 2 * .25, delta=.06, msg='measured sway, after the 1 s average')
    self.assertLess(by[('wander', 0., 12.)]['lane_p2p'], .05, 'a sham sways only by the noise')
    accel = [by[('bump', s, .1)]['acc_peak'] for s in of.SIZES]
    self.assertEqual(accel, sorted(accel), 'bigger bumps measure bigger')
    self.assertLess(accel[0], .03)
    self.assertGreater(by[('bump', .08, .1)]['jerk_peak'], by[('bump', .02, .1)]['jerk_peak'])
    self.assertEqual(by[('bump', .04, .1)]['felt'], 1)
    with contextlib.redirect_stdout(io.StringIO()) as out:
      results = feel.report(rows)
    text = out.getvalue()
    self.assertIn('== aborted ==', text)
    self.assertIn("bump 0.08: you're steering x1", text)
    noticed = results[('wander', 'lane_p2p', 'noticed')]
    self.assertTrue(.6 * 2 * .15 - .05 < noticed < .6 * 2 * .25 + .05, noticed)  # the driver's 0.2 m lies between these
    bump = results[('bump', 'acc_peak', 'noticed')]
    lo, hi = by[('bump', .02, .3)]['acc_peak'], by[('bump', .04, .3)]['acc_peak']
    self.assertTrue(min(lo, hi) - .03 < bump < max(lo, hi) + .03, (bump, lo, hi))
    self.assertIsNotNone(results[('bump', 'acc_peak', 'annoying')])

  def test_cli(self):
    import sys
    from unittest.mock import patch
    with patch.object(sys, 'argv', ['odyssey_feel.py', self.dir.name]), contextlib.redirect_stdout(io.StringIO()) as out:
      feel.main()
    self.assertIn('50% noticed', out.getvalue())

  def test_fit(self):
    x = np.linspace(0, 1, 40)
    p = 1 / (1 + np.exp(-(x - .4) * 20))
    y = np.random.default_rng(1).random(40) < p
    self.assertAlmostEqual(feel.fit_logistic(x, y), .4, delta=.1)
    self.assertIsNone(feel.fit_logistic(x, np.zeros(40, bool)))
    self.assertIsNone(feel.fit_logistic(x, x < .5), 'a falling trend has no threshold')
    self.assertTrue(math.isnan(feel.measure({'lane': np.empty((0, 2)), 'body': np.empty((0, 2))}, 0.)['lane_p2p']))


if __name__ == '__main__':
  unittest.main()
