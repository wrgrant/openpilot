"""Origin Frenet geometry. Inputs are already calibrated model car-frame metres."""

import numpy as np

VARIANTS = [(d, h) for d in (2, 3) for h in (15, 25, 50)]


def origin(x, y, degree=3, horizon=25):
  x, y = np.asarray(x), np.asarray(y)
  keep = np.isfinite(x + y) & (x >= -5) & (x <= horizon)
  if keep.sum() <= degree:
    return np.full(3, np.nan)
  p = np.polynomial.polynomial.polyfit(x[keep], y[keep], degree)
  return np.array([p[0], np.arctan(p[1]), 2 * p[2] / (1 + p[1] ** 2) ** 1.5])
