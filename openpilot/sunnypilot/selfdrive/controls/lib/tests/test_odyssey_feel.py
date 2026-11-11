"""Feel test: the plan is balanced and blind, the stimuli have the promised shape and bounds, every gate pauses or
aborts a trial, and a rating tapped on the panel reaches the controller, the log and the progress file."""
from collections import Counter
from dataclasses import replace
import json
import os
import tempfile
import time
import unittest
from types import SimpleNamespace as S

from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_feel as of
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_feel import Feel, Inputs, Stimulus

DT = .01


CLEAR = Inputs()


def run(feel, t0, t1, x=CLEAR, selected=True, paused=False, taps_at=None):
  """Step from t0 to t1; taps_at maps a frame time (rounded to the frame) to a list of taps. Returns outputs."""
  out = []
  n0, n1 = round(t0 / DT), round(t1 / DT)
  for n in range(n0, n1):
    t = n * DT
    taps = (taps_at or {}).get(n, ())
    out.append((t, *feel.step(t, x, selected, paused, taps)))
  return out


def first_start(feel):
  out = run(feel, 0, of.SETTLE_S + .05)
  assert feel.state == 'stimulus', feel.reason
  return out[-1][0] + DT  # the next frame


class TestPlan(unittest.TestCase):
  def test_blocks_are_balanced_and_shuffled(self):
    p = of.plan(7)
    names = [i.split(':', 1)[1] for i, _ in p]
    per_block = len(of.conditions())
    self.assertEqual(per_block, 20)
    self.assertEqual(len(p), of.BLOCKS * per_block)
    for b in range(of.BLOCKS):
      self.assertEqual(Counter(names[b * per_block:(b + 1) * per_block]), Counter(n for n, *_ in of.conditions()))
    self.assertNotEqual(names[:per_block], names[per_block:2 * per_block])
    self.assertEqual(p, of.plan(7), 'seeded: a restart rebuilds the same plan')
    self.assertNotEqual(p, of.plan(8))
    shams = [s for _, s in p[:per_block] if s.sham]
    self.assertEqual(Counter(s.family for s in shams), {'wander': 2, 'bump': 2})
    self.assertEqual({s.direction for _, s in p}, {1, -1})

  def test_bump_schedule(self):
    for _, s in of.plan(3):
      if s.family != 'bump':
        continue
      self.assertEqual(len(s.blips), 3)
      self.assertTrue(of.FIRST_BLIP[0] <= s.blips[0] <= of.FIRST_BLIP[1])
      for a, b in zip(s.blips, s.blips[1:], strict=False):
        self.assertTrue(of.BLIP_GAP[0] - .01 <= b - a <= of.BLIP_GAP[1] + .01)
      self.assertLess(s.blips[-1] + s.width, of.STIM_S)
      torque = [s.torque(n * DT) for n in range(round(of.STIM_S / DT))]
      signs = [v for i, v in enumerate(torque) if v and (i == 0 or not torque[i - 1])]
      if s.sham:
        self.assertFalse(any(torque))
        continue
      self.assertEqual(len(signs), 3, 'three blips')
      self.assertEqual([v > 0 for v in signs], [s.direction > 0, s.direction < 0, s.direction > 0], 'alternating')
      self.assertAlmostEqual(max(abs(v) for v in torque), s.size)
      self.assertAlmostEqual(sum(1 for v in torque if v) * DT, 3 * s.width, delta=3 * DT)

  def test_wander_shape(self):
    for a in of.AMPLITUDES:
      for period in of.PERIODS:
        s = Stimulus('wander', amplitude=a, period=period, direction=-1)
        accel = [s.accel(n * DT) for n in range(round(of.STIM_S / DT))]
        self.assertLessEqual(max(abs(v) for v in accel), of.MAX_ACCEL + 1e-12)
        self.assertEqual(accel[0], 0.)
        self.assertEqual(s.accel(of.STIM_S), 0.)
        v = y = 0.
        ys = []
        for acc in accel:  # integrate the open-loop command: it must sway and come back, not drift
          v += acc * DT
          y += v * DT
          ys.append(y)
        if a == 0:
          self.assertFalse(any(accel))
          continue
        self.assertLess(abs(v), .005, 'ends with no sideways speed')
        self.assertLess(abs(y), .1 * a + .02, 'ends where it started')
        self.assertGreater(max(ys) - min(ys), 1.9 * a * s.scale, 'sways about +-A')
        self.assertLess(max(ys) - min(ys), 2.1 * a * s.scale)
        mid = 10.  # in the plateau the command is exactly -A w^2 sin(wt)
        import math
        w = 2 * math.pi / period
        self.assertAlmostEqual(s.accel(mid), s.scale * a * w * w * math.sin(w * mid), places=9)


class TestGate(unittest.TestCase):
  def test_reasons(self):
    f = Feel(seed=1)
    good = Inputs()
    self.assertEqual(f.gate(0., good, False), '')
    for change, reason in [({'active': False}, 'steering inactive'), ({'speed': float('nan')}, 'invalid data'),
                           ({'valid': False}, 'invalid data'), ({'maneuver': True}, 'maneuver mode'),
                           ({'model_age': .5}, 'stale model'), ({'blinker': True}, 'blinker'),
                           ({'lane_change': True}, 'lane change'), ({'brake': True}, 'brake'),
                           ({'speed': 24.9 * .44704}, 'under 25 mph'), ({'lane_prob': .49}, 'lane lines lost'),
                           ({'pressed': True}, "you're steering")]:
      f.pressed_since = 0.
      self.assertEqual(f.gate(0., replace(good, **change), False), reason, change)
    f.pressed_since = 0.
    self.assertEqual(f.gate(of.PRESS_S - .01, replace(good, pressed=True), True), '', 'a short press does not abort a trial')
    self.assertEqual(f.gate(of.PRESS_S, replace(good, pressed=True), True), "you're steering")


class TestTrials(unittest.TestCase):
  def test_waits_for_clear_road_then_runs_and_asks(self):
    f = Feel(seed=2)
    run(f, 0, 5, replace(Inputs(), speed=10.))
    self.assertEqual((f.state, f.reason), ('wait', 'under 25 mph'))
    run(f, 5, 5 + of.SETTLE_S - .05)
    self.assertEqual((f.state, f.reason), ('wait', 'settling'))
    run(f, 5 + of.SETTLE_S - .05, 5 + of.SETTLE_S + .05)
    self.assertEqual(f.state, 'stimulus')
    start = f.events[-1]
    self.assertEqual(start['phase'], 'start')
    self.assertEqual(start['id'], f.plan[0][0])
    self.assertEqual(start['stimulus'], of.asdict(f.plan[0][1]))
    status = json.dumps(f.status())
    for secret in ('wander', 'bump', 'amplitude', 'size', 'sham', f.plan[0][0]):
      self.assertNotIn(secret, status, 'the panel status is blind')
    t = f.start
    run(f, t, t + of.STIM_S + .05)
    self.assertEqual(f.state, 'rate')
    self.assertEqual(f.events[-1]['phase'], 'end')

  def test_rating_marks_done_and_next_waits_for_rest(self):
    f = Feel(seed=3)
    t = first_start(f)
    first = f.trial
    early = round((t + 5) / DT)
    run(f, t, t + of.STIM_S + 1, taps_at={early: [('rate', 4, t + 5)]})
    self.assertEqual([e['phase'] for e in f.events if 'rating' in e['phase']], ['stray_rating'], 'no rating mid-stimulus')
    self.assertEqual(f.done, set())
    end = f.end
    run(f, t + of.STIM_S + 1, end + 2, taps_at={round((end + 1.5) / DT): [('rate', 1, end + 1.4)]})
    self.assertEqual(f.state, 'rest')
    self.assertEqual(f.done, {first[1]})
    self.assertTrue(f.dirty)
    rating = [e for e in f.events if e['phase'] == 'rating'][-1]
    self.assertEqual((rating['trial'], rating['id'], rating['value'], rating['label']), (first[0], first[1], 1, 'noticed'))
    run(f, end + 2, end + 3, taps_at={round((end + 2.5) / DT): [('rate', 2, end + 2.5)]})
    self.assertEqual(f.rating, 2, 'a later tap corrects the rating')
    self.assertTrue([e for e in f.events if e['phase'] == 'rating'][-1]['correction'])
    run(f, end + 3, end + of.REST_S + .05)
    self.assertEqual(f.state, 'stimulus')
    self.assertGreaterEqual(f.start - end, of.REST_S - DT, 'at least the rest between trials')
    self.assertNotEqual(f.trial[1], first[1])
    self.assertEqual(f.trial[0], first[0] + 1)

  def test_felt_that_is_logged_any_time(self):
    f = Feel(seed=4)
    run(f, 0, 1, replace(Inputs(), active=False), taps_at={50: [('felt', None, .5)]})
    f2 = Feel(seed=4)
    t = first_start(f2)
    run(f2, t, t + 2, taps_at={round((t + 1) / DT): [('felt', None, t + 1)]})
    felt = [e for e in f.events + f2.events if e['phase'] == 'felt']
    self.assertEqual(len(felt), 2)
    self.assertNotIn('trial', felt[0])
    self.assertEqual(felt[1]['trial'], f2.trial[0])
    self.assertAlmostEqual(felt[1]['elapsed'], round(t + 1 - f2.start, 2), places=6)
    self.assertEqual(f2.felt, 1)

  def test_every_gate_aborts_a_running_trial(self):
    for change in ({'pressed': True}, {'blinker': True}, {'active': False}, {'speed': 10.}, {'lane_prob': .2}):
      f = Feel(seed=5)
      t = first_start(f)
      trial = f.trial
      run(f, t, t + 3)
      out = run(f, t + 3, t + 3 + of.PRESS_S + .02, replace(Inputs(), **change))
      self.assertEqual(f.state, 'wait', change)
      self.assertEqual(out[-1][1:], (0., 0.), 'an abort hands the road straight back')
      abort = [e for e in f.events if e['phase'] == 'abort'][-1]
      self.assertEqual((abort['trial'], abort['id']), trial[:2])
      self.assertEqual(f.misses[trial[1]], 1)
      self.assertNotIn(trial[1], f.done)
    f = Feel(seed=5)
    t = first_start(f)
    run(f, t, t + 1)
    run(f, t + 1, t + 1 + of.PRESS_S - .05, replace(Inputs(), pressed=True))
    self.assertEqual(f.state, 'stimulus', 'a torque-sensor blip under PRESS_S does not abort')

  def test_pause_and_deselect_abort(self):
    for kw in ({'paused': True}, {'selected': False}):
      f = Feel(seed=6)
      t = first_start(f)
      run(f, t, t + 1, **kw)
      self.assertEqual(f.state, 'wait')
      self.assertEqual(f.events[-1]['reason'], 'paused' if 'paused' in kw else 'not selected')

  def test_unrated_and_repeated_aborts(self):
    f = Feel(seed=7)
    t = first_start(f)
    trial = f.trial
    run(f, t, t + of.STIM_S + of.RATE_TIMEOUT_S + .1)
    self.assertIn('unrated', [e['phase'] for e in f.events])
    self.assertEqual(f.misses[trial[1]], 1)
    run(f, t + of.STIM_S + of.RATE_TIMEOUT_S + .1, t + of.STIM_S + of.RATE_TIMEOUT_S + of.SETTLE_S + .2)
    self.assertNotEqual(f.trial[1], trial[1], 'a missed trial goes to the back, behind ones not yet tried')
    f = Feel(seed=7)
    f.misses = {f.plan[0][0]: of.MAX_MISSES - 1}
    t = first_start(f)
    self.assertNotEqual(f.trial[1], f.plan[0][0])
    f2 = Feel(seed=7)
    f2.misses = {i: of.MAX_MISSES - 1 for i, _ in f2.plan}
    t = first_start(f2)
    victim = f2.trial[1]
    run(f2, t, t + .1, replace(Inputs(), blinker=True))
    self.assertIn(victim, f2.skipped)
    self.assertEqual(f2.events[-1]['phase'], 'skip')

  def test_stimulus_output(self):
    f = Feel(seed=8)
    f.plan = (('0:w', Stimulus('wander', amplitude=.25, period=6., direction=1)),
              ('0:b', Stimulus('bump', size=.06, width=.1, direction=-1, blips=(2., 8., 14.))))
    t = first_start(f)
    out = run(f, t, t + of.STIM_S)
    self.assertTrue(any(a for _, a, _ in out))
    self.assertFalse(any(q for *_, q in out))
    peak = max(abs(a) for _, a, _ in out)
    self.assertGreaterEqual(peak, .25 * (2 * 3.14159265 / 6) ** 2 - 1e-6, 'reaches A w^2 in the plateau')
    self.assertLessEqual(peak, of.MAX_ACCEL)
    run(f, t + of.STIM_S, t + of.STIM_S + .1, taps_at={round((t + of.STIM_S + .05) / DT): [('rate', 0, 0.)]})
    end = f.end
    out = run(f, t + of.STIM_S + .1, end + of.REST_S + of.SETTLE_S + of.STIM_S + .2)
    torque = [q for *_, q in out if q]
    self.assertEqual(round(len(torque) * DT, 2), .3)
    self.assertEqual(sorted(set(torque)), [-.06, .06])

  def test_progress_survives_restart(self):
    f = Feel(seed=9)
    t = first_start(f)
    run(f, t, t + of.STIM_S + .1, taps_at={round((t + of.STIM_S + .05) / DT): [('rate', 3, 0.)]})
    saved = json.loads(json.dumps(f.progress()))
    g = Feel(saved)
    self.assertEqual((g.seed, g.done, g.trials), (9, f.done, 1))
    self.assertEqual(g.plan, f.plan)
    run(g, 0, of.SETTLE_S + .05)
    self.assertEqual(g.trial[0], 2)
    self.assertNotIn(g.trial[1], f.done)
    for bad in (None, {}, {'version': 0}, {'version': of.VERSION, 'seed': 'x', 'done': 'y', 'trials': -1}, []):
      Feel(bad)  # never raises
    self.assertEqual(Feel({'version': of.VERSION, 'seed': 1, 'done': ['nope', '0:bump-0-0.1']}).done, {'0:bump-0-0.1'})

  def test_done(self):
    f = Feel(seed=10)
    f.done = {i for i, _ in f.plan}
    run(f, 0, 5)
    self.assertEqual(f.state, 'done')


class TestInputChannel(unittest.TestCase):
  def test_post_and_read(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, 'input')
      of.post_input('rate', 9, now=1., path=path)  # before the reader started: ignored
      reader = of.InputReader(path)
      self.assertEqual(reader.read(), [])
      of.post_input('rate', 2, now=5., path=path)
      of.post_input('felt', now=6., path=path)
      with open(path, 'a') as fh:
        fh.write('garbage\n{"kind": "rate", "value": 7, "tap_s": 1}\n{"kind": "rate", "value": true, "tap_s": 1}\n{"kind": "rate", "va')
      self.assertEqual(reader.read(), [('rate', 2, 5.), ('felt', None, 6.)])
      with open(path, 'a') as fh:
        fh.write('lue": 4, "tap_s": 7}\n')
      self.assertEqual(reader.read(), [('rate', 4, 7.)], 'a line split across reads is joined')
      os.remove(path)
      of.post_input('rate', 0, now=8., path=path)
      self.assertEqual(reader.read(), [('rate', 0, 8.)], 'a recreated file starts over')


class TestRuntime(unittest.TestCase):
  def test_tap_reaches_log_and_progress(self):
    from openpilot.common.params import Params
    with tempfile.TemporaryDirectory() as d:
      params = Params(os.path.join(d, 'params'))
      params.put('OdysseyABExperiment', 'feel', block=True)
      params.put_bool('OdysseyPulsePaused', False, block=True)
      inp, prog = os.path.join(d, 'input'), os.path.join(d, 'progress.json')
      logged = []
      from openpilot.common import swaglog
      orig_event = swaglog.cloudlog.event
      swaglog.cloudlog.event = lambda name, **kw: logged.append((name, kw))
      os.environ['REPLAY'] = '1'  # do not force a pause on start, as the controlsd replay check runs it
      rt = of.FeelRuntime(params, input_path=inp, progress_path=prog)
      try:
        deadline = time.monotonic() + 2
        while (rt.controller is None or not rt.config[0]) and time.monotonic() < deadline:
          time.sleep(.01)
        self.assertTrue(rt.config[0])
        first = rt.controller.plan[0][0]  # the largest, scaled wander, under its real id so the restart keeps it
        rt.controller.plan = ((first, Stimulus('wander', amplitude=.35, period=6., direction=1)),) + rt.controller.plan[1:]
        cs = S(vEgo=27., steeringPressed=False, leftBlinker=False, rightBlinker=False, brakePressed=False)
        md = S(laneLineProbs=[.9] * 4, meta=S(laneChangeState=0))
        curv = []
        n = 0
        for n in range(round((of.SETTLE_S + of.STIM_S + .5) / DT)):
          curv.append(rt.update(n * DT, cs, md, True, True, .05, False))
        self.assertTrue(any(c for c, _ in curv))
        self.assertLessEqual(max(abs(c) for c, _ in curv), of.MAX_ACCEL / 27. ** 2 + 1e-12)
        self.assertEqual(rt.controller.state, 'rate')
        of.post_input('rate', 2, path=inp)
        deadline = time.monotonic() + 2
        while first not in rt.controller.done and time.monotonic() < deadline:
          n += 1
          rt.update(n * DT, cs, md, True, True, .05, False)
          time.sleep(.005)
        self.assertIn(first, rt.controller.done)
        for _ in range(20):  # the status goes out every 0.1 s of message time
          n += 1
          rt.update(n * DT, cs, md, True, True, .05, False)
        deadline = time.monotonic() + 2
        while not os.path.exists(prog) and time.monotonic() < deadline:
          time.sleep(.01)
        with open(prog) as fh:
          self.assertIn(first, json.load(fh)['done'])
        time.sleep(.2)
        phases = [kw['phase'] for name, kw in logged if name == 'odyssey_feel']
        self.assertEqual(Counter(p for p in phases if p != 'frame'), {'start': 1, 'end': 1, 'rating': 1})
        self.assertGreater(phases.count('frame'), 150)
        from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import mem_params
        status = json.loads(mem_params(params).get('OdysseyABStatus'))
        self.assertEqual((status['experiment'], status['rating']), ('feel', 2))
        params.put('OdysseyABExperiment', 'lane', block=True)
        deadline = time.monotonic() + 2
        while rt.config[0] and time.monotonic() < deadline:
          time.sleep(.01)
        self.assertEqual(rt.update(n * DT, cs, md, True, True, .05, False), (0., 0.))
        g = of.FeelRuntime(params, input_path=inp, progress_path=prog)
        deadline = time.monotonic() + 2
        while g.controller is None and time.monotonic() < deadline:
          time.sleep(.01)
        self.assertEqual(g.controller.done, {first}, 'a restarted process resumes the same plan')
        g.close()
      finally:
        rt.close()
        swaglog.cloudlog.event = orig_event
        os.environ.pop('REPLAY', None)

  def test_fault_zeroes_output(self):
    rt = of.FeelRuntime.__new__(of.FeelRuntime)
    rt.controller, rt.broken, rt.config, rt.heartbeat = Feel(seed=1), False, (True, False), time.monotonic()
    import collections
    rt.inbox = collections.deque()
    self.assertEqual(rt.update(0., None, None, True, True, 0., False), (0., 0.))
    self.assertTrue(rt.broken)


if __name__ == '__main__':
  unittest.main()
