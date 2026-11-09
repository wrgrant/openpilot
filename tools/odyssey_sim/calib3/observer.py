"""Causal camera/yaw observer, with explicit curvature-bias state.

State [lane offset m, offset rate m/s, road-acceleration observation bias m/s²].
Camera updates may be disabled: validation must score prediction, not assimilation.
No scipy is needed by this importable stepping API.
"""

import numpy as np


class LaneObserver:
  def __init__(self, offset=0.0, velocity=0.0, camera_sigma=0.1, heading_sigma=0.2, bias_walk=0.02, bias_retention=0.99):
    self.x = np.array([offset, velocity, 0.0], dtype=float)
    self.p = np.diag([camera_sigma**2, heading_sigma**2, 0.1**2])
    self.r = np.diag([camera_sigma**2, heading_sigma**2])
    self.q = bias_walk
    self.retention = bias_retention

  def step(self, road_accel, body_accel, offset=None, offset_velocity=None, dt=0.1):
    f = np.array([[1, dt, -0.5 * dt * dt], [0, 1, -dt], [0, 0, self.retention ** (dt / 0.1)]])
    self.x = f @ self.x + np.array([0.5 * dt * dt, dt, 0]) * (road_accel - body_accel)
    self.p = f @ self.p @ f.T + np.diag([1e-8, 1e-6, self.q**2 * dt])
    if offset is not None and offset_velocity is not None and np.isfinite([offset, offset_velocity]).all():
      k = np.linalg.solve(self.p[:2, :2] + self.r, self.p[:2, :]).T
      self.x += k @ (np.array([offset, offset_velocity]) - self.x[:2])
      # Joseph form keeps covariance positive under repeated camera updates.
      a = np.eye(3) - k @ np.eye(3)[:2]
      self.p = a @ self.p @ a.T + k @ self.r @ k.T
    return self.x.copy()
