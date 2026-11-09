import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.wander.runtime import LaneCentering, FrictionAssist


class TestWander(OpenpilotTestCase):
  def test_lane_offset_generates_bounded_correction_and_disables(self):
    lane = LaneCentering()
    for i in range(300):
      a = lane.update(dt=0.01, speed=30.0, yaw=0.0, road_curvature=0.0, camera_offset=0.1, camera_heading=0.0, camera_new=i % 5 == 0, valid=True, enabled=True)
    self.assertGreater(a, 0.01)
    self.assertLessEqual(abs(a), 0.12)
    self.assertEqual(lane.update(dt=0.01, speed=30.0, yaw=0.0, road_curvature=0.0, camera_offset=np.nan, camera_new=True, valid=True, enabled=True), 0.0)

  def test_no_phantom_correction_on_consistent_curved_road(self):
    lane = LaneCentering()
    for i in range(1000):
      a = lane.update(
        dt=0.01, speed=30.0, yaw=0.03, road_curvature=0.001, camera_offset=0.0, camera_heading=0.0, camera_new=i % 5 == 0, valid=True, enabled=True
      )
    self.assertAlmostEqual(a, 0.0, places=8)

  def test_friction_is_off_by_default_and_releases_on_takeover(self):
    f = FrictionAssist()
    self.assertEqual(f.update(0.1, target=0.1, actual=0.0, speed=30.0, rate=0.0, dt=0.01), 0.1)
    for _ in range(100):
      u = f.update(0.1, target=0.1, actual=0.0, speed=30.0, rate=0.0, dt=0.01, enabled=True)
    self.assertGreater(u, 0.15)
    self.assertEqual(f.update(0.1, target=0.1, actual=0.0, speed=30.0, rate=0.0, dt=0.01, enabled=True, valid=False), 0.1)

  def test_stale_camera_and_bad_timing_fall_back(self):
    lane = LaneCentering()
    for i in range(100):
      a = lane.update(dt=0.01, speed=30.0, yaw=0.0, road_curvature=0.0, camera_offset=0.1, camera_new=i == 0, valid=True, enabled=True)
    self.assertEqual(a, 0.0)

  def test_camera_yaw_fusion_reduces_frame_jitter(self):
    lane = LaneCentering()
    rng = np.random.default_rng(22)
    estimates = []
    camera = 0.1
    for i in range(4000):
      if i % 5 == 0:
        camera = 0.1 + rng.normal(0.0, 0.03)
      lane.update(dt=0.01, speed=30.0, yaw=0.0, road_curvature=0.0, camera_offset=camera, camera_heading=0.0, camera_new=i % 5 == 0, valid=True, enabled=True)
      if i > 1000:
        estimates.append(lane.e)
    self.assertLess(np.sqrt(np.mean((np.array(estimates) - 0.1) ** 2)), 0.015)

  def test_pulses_are_bounded_and_not_continuous(self):
    f = FrictionAssist('pulse', scale=2.0)
    commands = [f.update(0.0, target=0.1, actual=0.0, speed=25.0, rate=0.0, dt=0.01, enabled=True) for _ in range(1000)]
    self.assertLessEqual(max(commands), 0.22)
    self.assertGreater(max(commands), 0.1)
    self.assertLess(np.mean(np.array(commands) > 0.01), 0.3)

  def test_highway_only_and_quiet_at_zero_error(self):
    f = FrictionAssist('inverse')
    for speed in [0.0, 10.0, 17.0, 41.0, np.nan]:
      self.assertEqual(f.update(0.2, target=0.1, actual=0.0, speed=speed, rate=0.0, dt=0.01, enabled=True), 0.2)
    for _ in range(100):
      self.assertEqual(f.update(0.0, target=0.0, actual=0.0, speed=30.0, rate=0.0, dt=0.01, enabled=True), 0.0)
