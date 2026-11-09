"""Source-to-virtual pixel homography and logged-path-relative integration.

Pose uses calibrated device coordinates: x forward, y right, z down.
Camera coordinates are x right, y down, z forward. Positive dy and yaw
move/right-turn the virtual vehicle. The source ground plane is z=height
in calibrated device coordinates. Rotation is exact for all depths;
translation assumes the ground plane. Sky/vertical objects are not exact.
"""
from dataclasses import dataclass
import math
import numpy as np


def homography(k, camera_from_calib, height, dy=0., dpsi=0.):
  if not np.isfinite([height, dy, dpsi]).all() or height <= 0:
    raise ValueError('finite pose and positive camera height required')
  # camera_from_calib is a rotation in VIEW coordinates; identity is also
  # useful for a level pinhole camera whose ground normal is +camera-y.
  c, s = math.cos(dpsi), math.sin(dpsi)
  r = np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])
  a = np.asarray(camera_from_calib)
  t = np.array([dy, 0., 0.])
  n = np.array([0., 1., 0.])
  transform = a @ r @ (np.eye(3) - np.outer(t, n) / height) @ a.T
  return k @ transform @ np.linalg.inv(k)


def project(h, pixels):
  p = np.c_[pixels, np.ones(len(pixels))] @ h.T
  return p[:, :2] / p[:, 2:]


@dataclass
class RelativePose:
  dy: float = 0.
  dpsi: float = 0.
  dx: float = 0.

  def step(self, speed, yaw, logged_yaw, dt):
    """Exact SE(2) arc step then express displacement in new logged frame.

    Tracks dx too: omitting this state creates false lateral displacement
    in sustained curves. Assumes matched speed and no sideslip.
    """
    if not np.isfinite([speed, yaw, logged_yaw, dt]).all() or dt <= 0:
      raise ValueError('finite inputs and positive dt required')
    def arc(rate):
      theta = rate * dt
      return speed * dt * np.array([np.sinc(theta / np.pi), .5 * theta * np.sinc(theta / (2 * np.pi))**2])
    c, s = math.cos(self.dpsi), math.sin(self.dpsi)
    displacement = np.array([self.dx, self.dy]) + np.array([[c, -s], [s, c]]) @ arc(yaw) - arc(logged_yaw)
    a = logged_yaw * dt
    self.dx, self.dy = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]) @ displacement
    self.dpsi += (yaw - logged_yaw) * dt
    return self
