"""Independent synthetic checks for offline EPS identification."""
import unittest
import numpy as np
from tools.odyssey_eps.eps_law import filter_rate, fit_linear, predict, lagged, score


class TestEpsLaw(unittest.TestCase):
  def test_known_damping_recovered_with_unsigned_measurements(self):
    rng = np.random.default_rng(4)
    u = rng.uniform(-.8, .8, 4000)
    rate = rng.uniform(-4, 4, len(u))
    y = abs(90*np.sign(u)*np.maximum(abs(u)-.02, 0)-1.5*rate)
    fit = fit_linear(u, rate, y, dead=.02, cap=1, damping=True)
    np.testing.assert_allclose(fit, [90, 1.5], atol=.01)
    self.assertLess(np.mean((predict(u, rate, fit, .02, 1)-y)**2), 1e-6)

  def test_lag_never_bridges_missing_segment(self):
    d = {'t': np.array([0., .01, 10., 10.01]), 'u': np.array([.2, .3, .4, .5]),
         'seg': np.array([0, 0, 2, 2])}
    _, valid = lagged(d, .02)
    self.assertFalse(valid[2])

  def test_missing_speed_coverage_is_not_a_score(self):
    self.assertEqual(score(np.array([]), np.array([])), {'n': 0, 'r2': None, 'rmse': None})

  def test_filter_does_not_carry_state_across_log_gaps(self):
    out = filter_rate(np.array([0., .01, 10., 10.01]), np.array([0., 10., 2., 2.]), .1)
    self.assertAlmostEqual(out[2], 2)
    self.assertGreater(out[1], 0)
    self.assertLess(out[1], 10)


if __name__ == '__main__':
  unittest.main()
