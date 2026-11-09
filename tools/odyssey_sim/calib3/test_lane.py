import unittest
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib3.lane import consistent_future


class FutureTest(unittest.TestCase):
  def test_zero_change_preserves_future_and_nonzero_is_bounded(self):
    model = NS(acceleration=NS(y=np.ones(33)), orientation=NS(x=np.zeros(33), y=np.zeros(33)))
    np.testing.assert_array_equal(consistent_future(model, 0).acceleration.y, model.acceleration.y)
    plan = consistent_future(model, 0.2)
    self.assertAlmostEqual(plan.acceleration.y[0], 1.2)
    self.assertTrue(np.all((plan.acceleration.y >= 1) & (plan.acceleration.y <= 1.2)))
    self.assertIs(plan.orientation, model.orientation)


if __name__ == '__main__':
  unittest.main()
