import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.lowspeed.plant import simulate


class TestAnglePlant(OpenpilotTestCase):
  def test_command_sign_and_dead_zone(self):
    # Independent physical expectations: a sub-threshold command cannot move
    # a stationary wheel; positive and negative commands move it correspondingly.
    w = np.zeros((3, 7, 21))
    w[:, 1] = 2.0
    w[0, 0] = 0.02
    w[1, 0] = 0.7
    w[2, 0] = -0.7
    angle, rate = simulate([20, 20, 20, 0.1, 0.002, 0.3, 0.2], w)
    self.assertEqual(float(abs(angle[0]).max()), 0.0)
    self.assertGreater(angle[1, -1], 0.0)
    self.assertLess(angle[2, -1], 0.0)
    self.assertTrue(np.isfinite(rate).all())

  def test_zero_input_dissipates_motion(self):
    w = np.zeros((1, 7, 41))
    w[:, 1] = 2.0
    w[:, 3, 0] = 10.0
    angle, rate = simulate([20, 20, 20, 0.1, 0.0, 0.3, 0.0], w)
    self.assertLess(abs(rate[0, -1]), 0.01)
    self.assertLess(angle[0, -1], 4.0)
