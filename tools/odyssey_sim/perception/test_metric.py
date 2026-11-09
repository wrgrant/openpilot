import numpy as np

from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.perception.metric import from_simulator, perceptible_roughness, sluggishness


class TestPerception(OpenpilotTestCase):
  def setUp(self):
    self.t = np.arange(0, 30, .01)

  def test_command_chatter_without_output_is_not_roughness(self):
    zero = np.zeros_like(self.t)
    r = perceptible_roughness(dt=.01, angle_deg=zero,
      command=.5 * np.sin(2 * np.pi * 8 * self.t))
    self.assertEqual(r['roughness'], 0)
    self.assertGreater(r['windows'][0]['command_reversals_hz'], 10)
    self.assertNotIn('hand', r['coverage_channels'])

  def test_visible_wheel_oscillation_exceeds_small_motion(self):
    wave = np.sin(2 * np.pi * self.t)
    small = perceptible_roughness(dt=.01, angle_deg=.01 * wave)
    large = perceptible_roughness(dt=.01, angle_deg=.4 * wave)
    self.assertEqual(small['roughness'], 0)
    self.assertGreater(large['roughness'], 2)

  def test_slow_curve_is_diagnostic_not_roughness(self):
    r = perceptible_roughness(dt=.01, angle_deg=np.sin(2 * np.pi * .2 * self.t))
    self.assertLess(r['roughness'], .01)
    self.assertGreater(r['windows'][0]['bands'][0]['channels']['wheel']['threshold_ratio'], 1)

  def test_tiny_high_frequency_body_jerk_is_not_enough(self):
    r = perceptible_roughness(dt=.01, angle_deg=np.zeros_like(self.t),
      lat_accel=.01 * np.sin(2 * np.pi * 8 * self.t))
    self.assertEqual(r['roughness'], 0)

  def test_no_eligible_windows_is_unknown(self):
    r = perceptible_roughness(dt=.01, angle_deg=self.t, valid=np.zeros(len(self.t), dtype=bool))
    self.assertIsNone(r['roughness'])

  def test_simulator_proxy_is_not_angular_acceleration(self):
    states = [{'angle_deg': 0, 'angle_rate_deg_s': 0, 'lat_accel': 0, 'steering_accel': 100} for _ in self.t]
    self.assertEqual(from_simulator(states, dt=.01)['roughness'], 0)

  def test_slow_unwind_delay_is_measured_separately(self):
    ref = np.where(self.t < 10, 1., np.exp(-(self.t - 10)))
    actual = np.interp(self.t - .8, self.t, ref)
    r = sluggishness(dt=.01, actual=actual, reference=ref, threshold=.1)
    self.assertAlmostEqual(r['lag_s'], .8, places=2)
    self.assertGreater(r['excess_error_integral'], 0)

  def test_bad_data_rejected(self):
    with self.assertRaises(ValueError):
      perceptible_roughness(dt=.01, angle_deg=[0, np.nan])
