"""Exploratory angle-domain plant. Parameters are fitted, not road validated.

State: wheel angle and rate; speed-dependent torque authority, centering load,
aggregate dead zone, first-order rate response and transport delay. This simple
model cannot separately identify motor deadband, static friction and slip friction.
"""

import numpy as np

DT = 0.1


def simulate(p, w):
  u, v, angle, rate = w[:, :4].transpose(1, 0, 2)
  a = angle[:, 0].copy()
  r = rate[:, 0].copy()
  aa = [a.copy()]
  rr = [r.copy()]
  for j in range(1, w.shape[2]):
    f = max(0.0, j - 1 - p[6] / DT)
    lo = int(f)
    hi = min(lo + 1, j - 1)
    net = u[:, lo] * (1 - (f - lo)) + u[:, hi] * (f - lo) - p[4] * a
    target = np.interp(v[:, j - 1], [1, 4, 8], p[:3]) * np.sign(net) * np.maximum(abs(net) - p[3], 0.0)
    r += (target - r) * (-np.expm1(-DT / p[5]))
    a += r * DT
    aa.append(a.copy())
    rr.append(r.copy())
  return np.asarray(aa).T, np.asarray(rr).T


def metrics(p, w):
  if not len(w):
    return {'n': 0}
  a, r = simulate(p, w)
  baseline = w[:, 2, 0, None] + w[:, 3, 0, None] * np.arange(w.shape[2]) * DT
  return {
    'n': len(w),
    'angle_rmse': float(np.sqrt(np.mean((a[:, 1:] - w[:, 2, 1:]) ** 2))),
    'endpoint_mae': float(np.mean(abs(a[:, -1] - w[:, 2, -1]))),
    'rate_rmse': float(np.sqrt(np.mean((r[:, 1:] - w[:, 3, 1:]) ** 2))),
    'constant_rate_angle_rmse': float(np.sqrt(np.mean((baseline[:, 1:] - w[:, 2, 1:]) ** 2))),
  }
