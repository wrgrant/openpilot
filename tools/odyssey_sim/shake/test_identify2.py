"""Check independent frequency/time formulations and noisy-plant mechanics."""
import importlib.util
import unittest

import numpy as np

HAVE_SCIPY = importlib.util.find_spec("scipy") is not None  # the repo venv has none: put the scratch site on PYTHONPATH
if HAVE_SCIPY:
  from tools.odyssey_sim.shake.identify2 import yaw_tf, tyre, friction_basis, eps_predict
  from tools.odyssey_sim.shake.loop2 import step


@unittest.skipUnless(HAVE_SCIPY, "needs scipy (see report_shake2.md, Reproduce)")
class TestIdentification(unittest.TestCase):
  def test_yaw_frequency_matches_time_integration(self):
    p = np.array([130000., 165000., 3200., 1.8])
    t = np.arange(8000)*.01
    for f in (.5, 1., 2.):
      theta = .01*np.sin(2*np.pi*f*t)
      y = tyre(theta, np.full(len(t), 27.), p)[:, 1]
      s = slice(4000, None)
      h = np.sum(y[s]*np.exp(-2j*np.pi*f*t[s]))/np.sum(theta[s]*np.exp(-2j*np.pi*f*t[s]))
      expected = yaw_tf(p, 27., np.array([f]))[0]
      self.assertLess(abs(h/expected-1), .1)

  def test_bristles_stable_at_high_speed_and_sign_symmetric(self):
    w = np.full(2000, 2.)
    for kind in (1, 2):
      f = friction_basis(w, kind, .00034, .1)
      self.assertTrue(np.isfinite(f).all())
      self.assertLess(abs(f[-1]), 1.01)
      np.testing.assert_allclose(f, -friction_basis(-w, kind, .00034, .1))

  def test_zero_equilibrium_and_signed_motion(self):
    yp = np.array([130000., 165000., 3200., 1.8])
    cp = np.array([.46, 3.7, .46, .02, 0.])
    ep = np.array([.028, 112., 75., .024, 8.])
    z = np.zeros(9)
    np.testing.assert_equal(step(0., 27., 0., z, yp, cp, ep, 1, .00034, .1), z)
    for _ in range(100):
      z = step(.2, 27., 0., z, yp, cp, ep, 1, .00034, .1)
    self.assertGreater(z[0], 0)
    self.assertGreater(z[3], 0)

  def test_eps_deadband_cap_and_fade(self):
    p = np.array([.028, 112., 75., .024, 8.])
    np.testing.assert_equal(eps_predict(np.full(100, .02), np.full(100, 27.), p), 0.)
    hi = eps_predict(np.ones(100), np.full(100, 27.), p)
    lo = eps_predict(np.ones(100), np.full(100, 4.), p)
    self.assertAlmostEqual(hi[-1], 75.)
    self.assertAlmostEqual(lo[-1]/hi[-1], .5)


if __name__ == '__main__':
  unittest.main()
