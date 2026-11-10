"""Independent timing/frame checks; skip without scipy like pass 2."""

import importlib.util
import unittest
import numpy as np

HAVE_SCIPY = importlib.util.find_spec('scipy') is not None
if HAVE_SCIPY:
  from tools.odyssey_sim.shake.timing3 import vehicle_vectors, lag_seconds, band


@unittest.skipUnless(HAVE_SCIPY, 'needs scipy (scratch shake/site)')
class TestTiming(unittest.TestCase):
  def test_gravity_and_right_turn_in_device_sensor_axes(self):
    # Native -x is device-down, native -z device-forward. Calib yaw +90deg
    # turns device-forward into vehicle-left; gyro down retains right-turn sign.
    np.testing.assert_allclose(vehicle_vectors([[0, 0, -1]], [[0, 0, np.pi / 2]]), [[0, -1, 0]], atol=1e-12)
    np.testing.assert_allclose(vehicle_vectors([[-9.81, 0, 0]], [[0, 0, 0]]), [[0, 0, 9.81]])

  def test_positive_lag_is_later_output(self):
    rng = np.random.default_rng(3)
    x = band(rng.normal(size=5000))
    y = np.r_[np.zeros(9), x[:-9]]
    lag, corr = lag_seconds(x[100:-100], y[100:-100])
    self.assertAlmostEqual(lag, 0.09)
    self.assertGreater(corr, 0.999)


if __name__ == '__main__':
  unittest.main()
