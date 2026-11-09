import numpy as np
from openpilot.common.test import OpenpilotTestCase
from tools.odyssey_sim.vsim.geometry import homography, project, RelativePose


class TestGeometry(OpenpilotTestCase):
  def test_ground_shift_has_physical_sign(self):
    # Camera x right/y down/z forward, road 1.5m below camera.
    k = np.array([[100., 0, 50], [0, 100, 50], [0, 0, 1]])
    h = homography(k, np.eye(3), 1.5, dy=.3)
    np.testing.assert_allclose(project(h, [[50, 65]]), [[47, 65]], atol=1e-10)

  def test_zero_pose_preserves_pixels(self):
    np.testing.assert_allclose(homography(np.eye(3), np.eye(3), 1.5), np.eye(3))

  def test_rotation_matches_off_plane_point(self):
    h = homography(np.eye(3), np.eye(3), 1.5, dpsi=np.pi / 2)
    np.testing.assert_allclose(project(h, [[1, .2]]), [[-1, .2]], atol=1e-10)


class TestPose(OpenpilotTestCase):
  def test_logged_yaw_does_not_drift_on_curved_path(self):
    p = RelativePose()
    for _ in range(6000):
      p.step(20, .05, .05, .01)
    self.assertEqual(p.dy, 0)
    self.assertEqual(p.dpsi, 0)

  def test_straight_road_constant_yaw_has_exact_arc(self):
    p = RelativePose()
    for _ in range(100):
      p.step(10, .1, 0, .01)
    self.assertAlmostEqual(p.dy, 100 * (1 - np.cos(.1)), places=10)
    self.assertAlmostEqual(p.dpsi, .1)


class TestPhysicalProjection(OpenpilotTestCase):
  def test_tilted_camera_road_points_match_3d_projection(self):
    from openpilot.common.transformations.orientation import rot_from_euler
    k = np.array([[910., 0, 256], [0, 910, 128], [0, 0, 1]])
    a = rot_from_euler([.03, -.02, .01])
    points = np.array([[-1., 1.5, 10], [2., 1.5, 30], [4., 1.5, 80]])
    def pixels(p):
      q = (p @ a.T) @ k.T
      return q[:, :2] / q[:, 2:]
    # Right shift .3m followed by yaw .02rad, computed directly in 3D.
    q = points - [.3, 0, 0]
    yaw = .02
    virtual = np.c_[np.cos(yaw) * q[:, 0] - np.sin(yaw) * q[:, 2], q[:, 1],
                    np.sin(yaw) * q[:, 0] + np.cos(yaw) * q[:, 2]]
    h = homography(k, a, 1.5, .3, yaw)
    np.testing.assert_allclose(project(h, pixels(points)), pixels(virtual), atol=1e-10)

  def test_pure_rotation_does_not_need_ground_depth(self):
    h = homography(np.eye(3), np.eye(3), 1.5, dpsi=np.pi / 2)
    # Same optical ray at unrelated depths must map to the same pixel.
    np.testing.assert_allclose(project(h, [[.1, -.3], [.1, -.3]]),
                               [[-10., -3.], [-10., -3.]], atol=1e-8)

  def test_bad_height_is_rejected(self):
    with self.assertRaises(ValueError):
      homography(np.eye(3), np.eye(3), 0)
