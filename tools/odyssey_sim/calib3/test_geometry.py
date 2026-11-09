import unittest
import numpy as np
from tools.odyssey_sim.calib3.geometry import origin


class GeometryTest(unittest.TestCase):
  def test_origin_polynomial_and_curvature(self):
    x = np.linspace(0.2, 60, 33)
    e, h, k = origin(x, 0.2 + 0.03 * x + 0.001 * x * x)
    np.testing.assert_allclose([e, h, k], [0.2, np.arctan(0.03), 0.002 / (1 + 0.03**2) ** 1.5], atol=1e-10)

  def test_missing_is_not_a_straight_road(self):
    self.assertTrue(np.isnan(origin([0], [0])).all())


if __name__ == '__main__':
  unittest.main()
