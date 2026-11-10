import unittest
import numpy as np
from tools.odyssey_eps_report import fine_angle, measurements, summarize, pulse_record


class TestPulseReport(unittest.TestCase):
  def test_fine_motion(self):
    self.assertGreater(fine_angle(np.zeros(20), np.ones(20)*.1)[-1], .01)

  def test_known_threshold_from_log_records_and_samples(self):
    records, samples = [], []
    for direction in (-1, 1):
      for height in (0, .02, .03, .04, .05):
        for _repeat in range(12):
          start = 4*len(records)
          records.append({'id': str(len(records)), 'start': start, 'end': start+2.7, 'completed': True,
                          'band': '45-70', 'condition': {'kind': 'sham' if height == 0 else 'single', 'height': height,
                                                       'duration': .2, 'direction': direction}})
          for n in range(270):
            # Independent synthetic EPS: only requests >=4% produce 0.2 degrees of wheel movement.
            angle = direction*.2*min(1, max(0, (n-50)/20)) if height >= .04 else 0.
            rate = direction if 50 < n <= 70 and height >= .04 else 0.
            samples.append((start+n*.01, round(angle, 1), rate, 3 if height >= .04 and n > 50 else 0, .02*angle, height))
    decoded = [pulse_record(__import__('json').dumps({'msg': {'event': 'odyssey_pulse', **r}})) for r in records]
    result = summarize(measurements(decoded, np.asarray(samples)))
    self.assertEqual(len(result['thresholds']), 2)
    for threshold in result['thresholds']:
      self.assertEqual(threshold['smallest_tested'], .04)
      self.assertEqual(threshold['grid_bracket'], [.03, .04])

  def test_no_evidence_no_threshold(self):
    self.assertEqual(summarize([])['thresholds'], [])

  def test_square_map(self):
    from dataclasses import asdict
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import session
    c = dict(session())['square+']
    t = np.arange(190)*.01
    u = np.array([c.command(x) for x in t])
    motor = 112*np.maximum(np.abs(u)-.028, 0)
    s = np.column_stack((t, t*0, t*0, motor, t*0, u, t*0, t*0))
    r = {'id': '0:square+', 'start': 0., 'end': 1.9, 'completed': True, 'band': '45-70', 'condition': asdict(c)}
    result = summarize(measurements([r], s))
    self.assertAlmostEqual(result['eps_map'][0]['deadzone_pct'], 2.8, delta=.15)
    self.assertAlmostEqual(result['eps_map'][0]['slope_per_pct'], 1.12, delta=.05)

  def test_all_shot_kinds_and_suggestions(self):
    from tools.tests.test_odyssey_drive_companion import synthetic_drive
    from tools.odyssey_eps_report import note, suggest
    records, samples = synthetic_drive()
    s = summarize(measurements(records, samples))
    self.assertEqual(s['clean'], 26)
    self.assertEqual(s['coverage'][0]['missing'], [])
    self.assertEqual(s['ramps'][0]['bracket_pct'], [4., 5.])
    p = next(p for p in s['pairs'] if p['id'] == '0:pair-6-0.1')
    self.assertGreater(p['pulses'][0]['wheel_delta_deg'], 1.8*p['pulses'][1]['wheel_delta_deg'])
    train = next(t for t in s['trains'] if t['id'] == '0:train-6-60+')
    self.assertAlmostEqual(train['motor_delivery_ratio'], .5, delta=.06)
    self.assertLessEqual(len(note(s).splitlines()), 2)
    self.assertTrue(all(len(line) <= 34 for line in note(s).splitlines()))
    proposed = suggest(s)
    self.assertEqual(proposed, suggest(s))
    self.assertTrue(proposed)
    from tools.odyssey_drive_companion import validate_write
    import json
    validate_write('OdysseyPulseQueue', json.dumps([p['condition'] for p in proposed]))
    self.assertEqual(suggest(summarize([])), [])
    for r in s['ramps']:
      r['bracket_pct'] = [4.9, 5.]
    self.assertEqual(suggest(s), [])

  def test_chirp_known_second_order(self):
    from tools.tests.test_odyssey_drive_companion import synthetic_drive
    records, samples = synthetic_drive()
    s = summarize(measurements(records, samples))
    result = s['chirps'][0]['command_to_wheel_rate']
    for v in result['frequencies']:
      # Independent discrete plant used by the fixture: poles radius .90, angle .10.
      z = np.exp(-2j*np.pi*v['hz']*.01)
      expected = .02/(1-1.7910074975*z+.81*z*z)
      self.assertAlmostEqual(v['gain']/abs(expected), 1, delta=.15)
      phase_error = (v['phase_deg']-np.angle(expected, deg=True)+180)%360-180
      self.assertLess(abs(phase_error), 15)
    self.assertIsNotNone(result['coherence'])
    yaw = s['chirps'][0]['wheel_angle_to_yaw']
    for v in yaw['frequencies']:
      self.assertAlmostEqual(v['gain']/.0008, 1, delta=.15)
      self.assertLess(abs(v['phase_deg']), 15)

  def test_missing_coverage_and_faults_retained(self):
    from tools.tests.test_odyssey_drive_companion import synthetic_drive
    records, samples = synthetic_drive()
    summary = summarize(measurements(records, samples[:0]))
    self.assertEqual(summary['clean'], 0)
    self.assertEqual(summary['shots'][0]['samples'], 0)
    self.assertEqual(len(summary['coverage'][0]['missing']), 26)

  def test_suggest_mirror_and_width(self):
    from tools.odyssey_eps_report import suggest
    s = summarize([])
    def pair(direction, width, motion):
      return {'kind': 'pair', 'height': .06, 'duration': .5+2*width, 'direction': direction,
              'period': width, 'duty': 0., 'band': '45-70', 'noise_deg': .02,
              'pulses': [{'wheel_delta_deg': motion}]}
    s['pairs'] = [pair(1, .1, .1), pair(-1, .1, 0), pair(1, .05, 0)]
    suggestions = suggest(s)
    self.assertTrue(any('mirror' in v['reason'] for v in suggestions))
    self.assertTrue(any(v['condition']['period'] == .075 for v in suggestions))
    self.assertEqual(suggestions, suggest(s))
    self.assertEqual(len(suggest(s, limit=1)), 1)
