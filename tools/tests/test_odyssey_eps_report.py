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
