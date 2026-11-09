import unittest
import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.curves.transition import CurveTransitions


class TestTransitions(OpenpilotTestCase):
  def inputs(self, **kw):
    args = {
      'command': 0.4,
      'target_accel': 0.3,
      'speed': 20.0,
      'angle_deg': -8.0,
      'rate_deg_s': 0.0,
      'planned_angle_deg': -1.0,
      'plan_t': np.linspace(0, 3, 33),
      'plan_accel': np.linspace(0.3, 0.9, 33),
      'active': True,
      'driver_pressed': False,
      'driver_torque': 0.0,
      'plan_age': 0.03,
      'dt': 0.01,
    }
    args.update(kw)
    return args

  def test_default_off_and_invalid_plan_fall_back(self):
    self.assertEqual(CurveTransitions().update(**self.inputs()), (0.3, 0.4))
    c = CurveTransitions(enabled=True)
    self.assertEqual(c.update(**self.inputs(plan_age=0.5)), (0.3, 0.4))
    self.assertEqual(c.update(**self.inputs(plan_accel=np.full(33, np.nan))), (0.3, 0.4))

  def test_preview_anticipates_and_is_bounded(self):
    c = CurveTransitions(enabled=True, preview=True)
    for _ in range(100):
      target, command = c.update(**self.inputs())
    self.assertGreater(target, 0.35)
    self.assertLessEqual(target, 0.6)
    self.assertEqual(command, 0.4)

  def test_release_reduces_holding_and_active_returns_with_correct_sign(self):
    for mode in ['release', 'active']:
      c = CurveTransitions(enabled=True, return_mode=mode)
      for _ in range(100):
        _, command = c.update(**self.inputs(command=0.1))
      self.assertLess(command, 0.1)
      if mode == 'release':
        self.assertAlmostEqual(command, 0.0)
      else:
        self.assertLess(command, 0.0)
      _, at_center = c.update(**self.inputs(command=0.0, angle_deg=-0.2, planned_angle_deg=0.0))
      self.assertEqual(at_center, 0.0)

  def test_driver_stale_and_stopping_guard_disarm(self):
    c = CurveTransitions(enabled=True, return_mode='active')
    for _ in range(100):
      c.update(**self.inputs(command=0.1))
    for extra in [{'driver_pressed': True}, {'driver_torque': 500.0}, {'plan_age': 0.4}, {'angle_deg': -3.0, 'rate_deg_s': 10.0}]:
      self.assertEqual(c.update(**self.inputs(command=0.1, **extra)), (0.3, 0.1))

  def test_sign_symmetry_and_slew(self):
    a = CurveTransitions(enabled=True, preview=True, return_mode='active')
    b = CurveTransitions(enabled=True, preview=True, return_mode='active')
    last = 0.1
    for _ in range(100):
      x = a.update(**self.inputs(command=0.1))
      y = b.update(**self.inputs(command=-0.1, target_accel=-0.3, angle_deg=8.0, planned_angle_deg=1.0, plan_accel=-np.linspace(0.3, 0.9, 33)))
      np.testing.assert_allclose(x, -np.array(y), atol=1e-12)
      self.assertLessEqual(abs(x[1] - last), 0.0050001)
      last = x[1]

  def test_malformed_plan_is_stock(self):
    c = CurveTransitions(enabled=True, preview=True)
    self.assertEqual(c.update(**self.inputs(plan_accel=['bad'])), (0.3, 0.4))

  def test_inverse_and_overshoot_guard_grid(self):
    c = CurveTransitions(enabled=True, preview=True, predictor='inverse', return_mode='active')
    rng = np.random.default_rng(20261109)
    for _ in range(1000):
      angle = float(rng.uniform(-30, 30))
      rate = float(rng.uniform(-30, 30))
      command = float(rng.uniform(-1, 1))
      target, u = c.update(**self.inputs(command=command, angle_deg=angle, rate_deg_s=rate, planned_angle_deg=0.0))
      self.assertTrue(np.isfinite([target, u]).all())
      self.assertLessEqual(abs(u), 1.0)
      self.assertLessEqual(abs(target - 0.3), 0.3)
      if abs(angle) <= 2 or angle * (angle + 0.4 * rate) <= 0:
        self.assertEqual(u, command)


if __name__ == '__main__':
  unittest.main()
