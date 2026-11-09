import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.vsim.campaign import paired_interval, eligible_runs


class TestCampaign(OpenpilotTestCase):
  def test_pairing_preserves_window_effect(self):
    self.assertEqual(paired_interval([2, 4, 6], [1, 3, 5])['ci95'], [1.0, 1.0])

  def test_gap_breaks_autonomous_run(self):
    t = np.r_[np.arange(5000) * 0.01, 60 + np.arange(5000) * 0.01]
    runs = eligible_runs(t, np.ones(len(t), bool))
    self.assertEqual(len(runs), 2)
    self.assertAlmostEqual(t[runs[1][0]], 60.0)

  def test_one_window_does_not_claim_uncertainty(self):
    self.assertIsNone(paired_interval([2], [1])['ci95'])

  def test_wander_metric_detects_known_ten_second_signal(self):
    from tools.odyssey_sim.vsim.analyze import score

    t = np.arange(1200) * 0.05
    offset = 0.1 * np.cos(2 * np.pi * t / 10)
    rows = [{'t': float(ts), 'lane': [[float(-y)] * 33] * 2, 'logged_lane': [[float(-y)] * 33] * 2} for ts, y in zip(t, offset, strict=True)]
    result = score(rows, [{'dy': 0.0}])['simulated']
    self.assertAlmostEqual(result['wander_cm'], 7.071, delta=0.1)
    self.assertAlmostEqual(result['max_offset_m'], 0.1, places=6)
    peak = np.argmax(result['power_m2'])
    self.assertAlmostEqual(result['frequencies_hz'][peak], 0.1, delta=0.001)

  def test_curve_events_use_logged_controlsd_plan(self):
    from tools.odyssey_sim.vsim.analyze import curve_scores

    t = np.arange(1200) * 0.05
    request = np.maximum(0.0, 1.0 - abs(t - 30.0) / 8.0)
    rows = [{'t': float(ts), 'speed': 20.0, 'logged_curvature': 0.0, 'lane': [[-1.8] * 33, [1.8] * 33], 'logged_lane': [[-1.8] * 33, [1.8] * 33]} for ts in t]
    motion = [{'t': float(ts), 'body': float(a)} for ts, a in zip(t, request, strict=True)]
    native = {'t': t, 'act': request, 'body': request, 'cmd': request * 0.2, 'des': request}
    result = curve_scores(rows, motion, native, 0.0)
    self.assertEqual(len(result['events']), 1)
    self.assertAlmostEqual(result['events'][0]['logged']['peak'], 1.0, places=2)

  def test_partial_curve_trace_reports_unavailable(self):
    from tools.odyssey_sim.vsim.metrics import curve_scores

    result = curve_scores([], [], {}, 0.0)
    self.assertEqual(result['events'], [])
    self.assertIsNone(result['simulated_body_jerk_rms_mps3'])

  def test_wander_unavailable_when_lanes_visible_under_eighty_percent(self):
    from tools.odyssey_sim.vsim.analyze import score

    rows = [{'t': i * 0.05, 'lane': [[-1.8] * 33, [1.8] * 33], 'logged_lane': [[-1.8] * 33, [1.8] * 33]} for i in range(1200)]
    confidence = np.arange(1200) >= 300
    result = score(rows, [{'dy': 0.0}], confidence)
    self.assertIsNone(result['simulated']['wander_cm'])

  def test_low_confidence_outlier_is_interpolated(self):
    from tools.odyssey_sim.vsim.analyze import score

    rows = [{'t': i * 0.05, 'lane': [[-1.8] * 33, [1.8] * 33], 'logged_lane': [[-1.8] * 33, [1.8] * 33]} for i in range(1200)]
    rows[600]['lane'] = [[-1.0] * 33, [2.6] * 33]
    confidence = np.ones(1200, bool)
    confidence[600] = False
    result = score(rows, [{'dy': 0.0}], confidence)['simulated']
    self.assertEqual(result['wander_cm'], 0.0)
    self.assertGreater(result['unmasked_wander_cm'], 0.0)

  def test_curve_visibility_does_not_hide_body_release(self):
    from tools.odyssey_sim.vsim.metrics import curve_scores

    t = np.arange(1200) * 0.05
    request = np.maximum(0.0, 1.0 - abs(t - 30.0) / 8.0)
    rows = [{'t': float(ts), 'speed': 20.0, 'lane': [[-1.6] * 33, [2.0] * 33], 'logged_lane': [[-1.6] * 33, [2.0] * 33]} for ts in t]
    motion = [{'t': float(ts), 'body': float(a)} for ts, a in zip(t, request, strict=True)]
    native = {'t': t, 'act': request, 'body': request, 'cmd': request * 0.2, 'des': request}
    event = curve_scores(rows, motion, native, 0.0, t > 42)['events'][0]
    self.assertIsNone(event['simulated_run_wide_m'])
    self.assertAlmostEqual(event['unmasked_simulated_run_wide_m'], 0.2, places=6)
    self.assertEqual(event['simulated_exit_body_minus_plan_s'], 0.0)

  def test_short_trace_cannot_claim_wander(self):
    from tools.odyssey_sim.vsim.analyze import score

    rows = [{'t': i * 0.05, 'lane': [[-1.8] * 33, [1.8] * 33], 'logged_lane': [[-1.8] * 33, [1.8] * 33]} for i in range(20)]
    self.assertIsNone(score(rows, [{'dy': 0.0}])['simulated']['wander_cm'])
