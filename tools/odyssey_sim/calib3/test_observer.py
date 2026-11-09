import unittest
import numpy as np
from tools.odyssey_sim.calib3.observer import LaneObserver


class ObserverTest(unittest.TestCase):
  def test_blind_constant_acceleration_matches_kinematics(self):
    observer = LaneObserver()
    for _ in range(100):
      x = observer.step(0.02, 0)
    np.testing.assert_allclose(x[:2], [1, 0.2], atol=1e-12)

  def test_camera_updates_correct_without_hiding_blind_drift(self):
    observer = LaneObserver()
    for _ in range(500):
      x = observer.step(0.03, 0, 0, 0)
    self.assertLess(abs(x[0]), 0.03)
    self.assertTrue(np.linalg.eigvalsh(observer.p).min() > 0)
    for _ in range(100):
      x = observer.step(0.1, 0)
    self.assertGreater(abs(x[0]), 1)


if __name__ == '__main__':
  unittest.main()
