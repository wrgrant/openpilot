import json
import unittest
from unittest.mock import patch

from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_ab as ab


class TestOdysseyAB(unittest.TestCase):
  def test_selection(self):
    for e, arms in ab.EXPERIMENTS.items():
      for arm in arms:
        self.assertEqual(ab.selection(e, arm, True, 999), arm)
      self.assertEqual(ab.selection(e, 'bad', True, 99), arms[0])
      self.assertEqual(ab.selection(e, '', False, 0), arms[0])
    self.assertEqual(ab.selection('bad', 'bad', True, 0), 'off')
    self.assertEqual(ab.selection('bump', '', False, 60), 'kick')
    with patch.object(ab, '_config', ('bump', True, 'kick')):
      self.assertEqual(ab.runtime_arm('bump', 0, ab.DEADBAND_ARMS), 'kick')
      self.assertFalse(ab.clock_enabled('lane', True))

  def test_reasons(self):
    good = {'active': True, 'speed': 20, 'since_switch': 6, 'lane_probs': (1, 1), 'desired_accel': 0}
    self.assertEqual(ab.good_data('lane', **good), (True, 'counting', 'straight'))
    for change, reason in [({'active': False}, 'steering inactive'), ({'speed': float('nan')}, 'invalid data'),
                           ({'blinker': True}, 'blinker'), ({'lane_change': True}, 'lane change'),
                           ({'speed': 10}, 'under 25 mph'), ({'since_switch': 4.99}, '5 s after switch'),
                           ({'pressed_s': 1}, "you're steering"), ({'lane_probs': (.5, 1)}, 'lane lines lost')]:
      self.assertEqual(ab.good_data('lane', **(good | change))[1], reason)
    self.assertTrue(ab.good_data('lane', **(good | {'pressed_s': .99}))[0])
    self.assertEqual(ab.good_data('lowspeed', **good)[1], 'outside 0–18 mph')
    self.assertTrue(ab.good_data('lowspeed', **(good | {'speed': 4}))[0])
    self.assertEqual(ab.good_data('', **good)[1], 'no experiment')
    self.assertEqual(ab.good_data('lane', **(good | {'desired_accel': .5}))[2], 'curve')

  def test_totals(self):
    zero = ab.validate_totals('{}')
    for raw in ('null', '[]', 'bad', '{"lane": []}', '{"lane":{"off":null}}'):
      self.assertEqual(ab.validate_totals(raw), zero)
    totals = ab.validate_totals('{"lane":{"off":{"straight":12,"curve":-1}}}')
    self.assertEqual(totals['lane']['off'], {'straight': 12., 'curve': 0.})
    self.assertEqual(ab.validate_totals(json.dumps(totals)), totals)
    self.assertEqual(ab.validate_totals('{}'), zero)

  def test_log_roundtrip(self):
    from tools.odyssey_wander import logged_arm
    record = ab.parse_record(json.dumps({'event': 'odyssey_ab', 'experiment': 'lane', 'arm': 'on', 'good': True, 'reason': 'counting', 'kind': 'straight'}))
    self.assertIsNotNone(record)
    import logging
    from openpilot.common.logging_extra import SwagFormatter
    from openpilot.common.swaglog import cloudlog
    formatter = SwagFormatter(cloudlog)
    wire = formatter.format(logging.LogRecord("test", 20, __file__, 1, {"event": "odyssey_ab", **record}, (), None))
    self.assertEqual(ab.parse_record(wire), record)
    self.assertEqual(logged_arm({'records': [(10, record)]}, 11, ab.LANE_ARMS, 180), 'on')
    self.assertEqual(logged_arm({'records': [(10, record)]}, 9, ab.LANE_ARMS, 180), 'off')
    self.assertIsNone(ab.parse_record('{}'))


class TestLivePersistence(unittest.TestCase):
  def test_worker_persists_and_resets(self):
    import tempfile
    import time
    from types import SimpleNamespace as S
    from openpilot.common.params import Params
    with tempfile.TemporaryDirectory() as path:
      params = Params(path)
      params.put("OdysseyABExperiment", "bump", block=True)
      params.put_bool("OdysseyABManual", True, block=True)
      params.put("OdysseyABArm", "kick", block=True)
      worker = ab.LiveAB(params)
      try:
        def wait_for(predicate):
          deadline = time.monotonic() + .5
          while not predicate() and time.monotonic() < deadline:
            time.sleep(.005)
          self.assertTrue(predicate())
        wait_for(lambda: ab.manual_experiment("bump"))
        cs = S(vEgo=25., steeringPressed=False, leftBlinker=False, rightBlinker=False)
        md = S(laneLineProbs=[1.]*4, meta=S(laneChangeState=0))
        for t in (0., 5., 5.01, 5.01 + ab.TOTALS_SAVE_S):  # the last frame is only there to trigger the periodic save
          worker.update(t, cs, md, True, 0.)
        wait_for(lambda: ab.validate_totals(params.get("OdysseyABTotals"))["bump"]["kick"]["straight"] > 0)
        stored = Params(path).get("OdysseyABTotals")
        self.assertAlmostEqual(ab.validate_totals(stored)["bump"]["kick"]["straight"], .01)
        params.put_bool("OdysseyABReset", True, block=True)
        wait_for(lambda: params.get_bool("OdysseyABReset") is False)
        wait_for(lambda: ab.validate_totals(params.get("OdysseyABTotals")) == ab.validate_totals("{}"))
      finally:
        worker.close()
