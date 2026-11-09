from openpilot.common.test import OpenpilotTestCase
import numpy as np
from tools.odyssey_sim.plan.compensation import Compensation


class TestCompensation(OpenpilotTestCase):
  def test_disabled_and_invalid_are_stock(self):
    c = Compensation('lane')
    self.assertEqual(c.update(.2, 25, .1, 0, 1, .05), .2)
    self.assertEqual(c.update(.2, 25, np.nan, 0, 1, .05, enabled=True), .2)

  def test_lane_sign_bound_dropout_and_gates(self):
    c = Compensation('lane')
    # Offset is positive right; requested acceleration must correct left.
    out = c.update(0, 25, .2, 0, 1, .05, enabled=True)
    self.assertLess(out, 0)
    self.assertLessEqual(abs(out), .15)
    self.assertEqual(c.update(.2, 25, .2, 0, 0, .05, enabled=True), .2)
    self.assertEqual(c.update(.2, 8, .2, 0, 1, .05, enabled=True), .2)
    self.assertEqual(c.update(.2, 25, .2, 0, 1, .05, enabled=True, lane_change=True), .2)

  def test_reentry_has_no_stale_filter_kick(self):
    c = Compensation('lead')
    for _ in range(100):
      c.update(1, 25, 0, 0, 1, .05, enabled=True)
    c.update(0, 25, 0, 0, 0, .05, enabled=True)
    self.assertEqual(c.update(.1, 25, 0, 0, 1, .05, enabled=True), .1)

  def test_heading_corrects_rightward_motion_and_large_inputs_are_bounded(self):
    c = Compensation('lane')
    self.assertLess(c.update(0, 25, 0, .01, 1, .05, enabled=True), 0)
    self.assertAlmostEqual(c.update(0, 25, 1.4, .09, 1, .05, enabled=True), -.15)

  def test_lead_has_zero_dc_and_advances_ten_second_request(self):
    c = Compensation('lead')
    t = np.arange(0, 100, .05)
    y = np.array([c.update(.1*np.sin(2*np.pi*x/10), 25, 0, 0, 1, .05, enabled=True) for x in t])
    from tools.odyssey_sim.vsim.transfer import harmonic
    phase = np.angle(harmonic(t[800:], y[800:], 10)/harmonic(t[800:], .1*np.sin(2*np.pi*t[800:]/10), 10), deg=True)
    self.assertGreater(phase, 0)
    self.assertLess(phase, 30)
    for _ in range(2000):
      value = c.update(.2, 25, 0, 0, 1, .05, enabled=True)
    self.assertAlmostEqual(value, .2, places=8)

  def test_old_camera_frame_bypasses_and_clears_history(self):
    c = Compensation('lane')
    self.assertEqual(c.update(.2, 25, .2, 0, 1, .05, enabled=True, camera_age=.11), .2)

  def test_missing_camera_measurement_falls_back(self):
    c = Compensation('lane')
    self.assertEqual(c.update(.2, 25, None, 0, 1, .05, enabled=True), .2)
    self.assertEqual(c.update(.2, 25, .1, 0, 1, .11, enabled=True), .2)

  def test_outer_lane_confidence_does_not_disable_inner_lane_correction(self):
    from tools.odyssey_sim.plan.compensation import inner_lane_confidence
    self.assertEqual(inner_lane_confidence([0, .1, 0, .9, 0, .8, 0, .1]), .8)
    self.assertEqual(inner_lane_confidence([0, .1]), 0)
