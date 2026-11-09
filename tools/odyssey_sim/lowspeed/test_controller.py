import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.lowspeed.controller import AngleController


class TestLowSpeedAngle(OpenpilotTestCase):
  def test_disabled_is_stock(self):
    c = AngleController()
    self.assertEqual(c.update(0.42, 2.0, 0.0, 0.0, 10.0, active=True).torque, 0.42)

  def test_invalid_plan_falls_back(self):
    c = AngleController(True)
    for future in [([], []), ([0, 0], [1, 2]), ([0, 1], [1, np.nan]), (), ('x', 'y')]:
      self.assertEqual(c.update(0.2, 2.0, 0.0, 0.0, 20.0, future=future).torque, 0.2)

  def test_blend_reaches_highway_command_continuously(self):
    c = AngleController(True)
    for _ in range(500):
      c.update(-0.7, 2.0, 0.0, 0.0, 20.0)
    result = c.update(-0.7, 7.999, 0.0, 0.0, 20.0)
    self.assertAlmostEqual(result.torque, -0.7, places=5)

  def test_reference_slew_and_driver_override(self):
    c = AngleController(True)
    previous = 0.0
    for _ in range(200):
      result = c.update(0.0, 2.0, 0.0, 0.0, 500.0)
      self.assertLessEqual(abs(result.reference - previous), 0.120001)
      self.assertLessEqual(abs(result.torque), 0.950001)
      previous = result.reference
    self.assertEqual(c.update(0.13, 2.0, 0.0, 0.0, 500.0, driver=True).torque, 0.13)

  def test_stall_reports_authority_without_windup(self):
    c = AngleController(True)
    for _ in range(3000):
      result = c.update(0.0, 2.0, 0.0, 0.0, 500.0)
    self.assertTrue(result.authority_limited)
    # Reverse request remains bounded; driver override takes effect immediately.
    self.assertEqual(c.update(0.0, 2.0, 0.0, 0.0, -500.0, driver=True).torque, 0.0)

  def test_bad_input_and_future_preview(self):
    c = AngleController(True)
    self.assertEqual(c.update(0.3, 2.0, np.nan, 0.0, 2.0).torque, 0.3)
    self.assertEqual(c.update(np.nan, 2.0, 0.0, 0.0, 2.0).torque, 0.0)
    for _ in range(50):
      result = c.update(0.0, 2.0, 0.0, 0.0, 0.0, future=([0.0, 1.0], [0.0, 20.0]))
    self.assertGreater(result.reference, 0.0)
