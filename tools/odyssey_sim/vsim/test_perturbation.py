import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.vsim.perturbation import PerturbationPlant, injection


class TestPerturbation(OpenpilotTestCase):
  def test_matching_commands_cancel_with_nonzero_roll(self):
    p = PerturbationPlant()
    p.reset(.2, .1, -.1)
    for u in np.sin(np.arange(1000) / 40) * .3:
      y, body = p.step(u, u, 20, .03, .4, .5)
      self.assertEqual(body, .5)
      self.assertEqual(y, .4)

  def test_differential_response_preserves_deadband(self):
    p = PerturbationPlant()
    p.reset(0, 0, 0)
    for _ in range(100):
      _, body = p.step(.02, 0, 20, 0, 0, 0)
    self.assertEqual(body, 0)

  def test_consistent_injected_heading(self):
    t, v = 2., 20.
    y, psi = injection(t, v, .2, 10, 'sine')
    yp = injection(t + .0001, v, .2, 10, 'sine')[0]
    ym = injection(t - .0001, v, .2, 10, 'sine')[0]
    self.assertAlmostEqual((yp - ym) / .0002 / v, psi, places=8)
    self.assertLess(abs(y), .2)
