"""Physical limiting cases, independent of the road fit."""
import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.shake.physics import simulate, DEFAULT


class TestPhysics(OpenpilotTestCase):
  def test_unforced_equilibrium(self):
    y = simulate(np.zeros(2000), np.full(2000, 25.), DEFAULT, np.zeros(8))
    np.testing.assert_array_equal(y, np.zeros_like(y))

  def test_sub_breakaway_command_stays_still(self):
    y = simulate(np.full(2000, .01), np.full(2000, 25.), DEFAULT, np.zeros(8))
    np.testing.assert_array_equal(y, np.zeros_like(y))

  def test_left_right_symmetry_and_finite_response(self):
    u = np.sin(np.arange(2000)*.01*2*np.pi)*.2
    a = simulate(u, np.full(2000, 25.), DEFAULT, np.zeros(8))
    b = simulate(-u, np.full(2000, 25.), DEFAULT, np.zeros(8))
    self.assertTrue(np.isfinite(a).all())
    self.assertGreater(np.std(a[:, 0]), .001)
    np.testing.assert_allclose(a, -b, atol=1e-10)

  def test_closed_loop_stepping_matches_open_loop(self):
    from tools.odyssey_sim.shake.physics import advance
    u = .2*np.sin(np.arange(500)*.03)
    v = np.full(len(u), 25.)
    expected = simulate(u, v, DEFAULT, np.zeros(8))
    state, pieces = np.zeros(8), []
    for i in range(len(u)):
      y, state = advance(u[i:i+1], v[i:i+1], DEFAULT, state)
      pieces.append(y[0])
    np.testing.assert_allclose(pieces, expected, atol=1e-12)
