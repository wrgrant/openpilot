"""Research-only desired-acceleration compensation. No production params are written.

Caller supplies OdysseyPlanCompensation (default false), causal lane origin/heading,
lane confidence and lane-change state. Positive offset/heading means right of lane.
"""
import math
import numpy as np


def inner_lane_confidence(probabilities):
  """Parser emits [not-exists, exists] pairs for four lanes; use inner two."""
  p = np.asarray(probabilities)
  if p.shape != (8,) or not np.isfinite(p).all():
    return 0.
  return float(np.min(p[3:6:2]))


class Compensation:
  def __init__(self, mode='stock', strength=1.):
    if not np.isfinite(strength) or strength < 0:
      raise ValueError('finite nonnegative strength required')
    if mode not in ('stock', 'lead', 'lane', 'smooth'):
      raise ValueError(mode)
    self.mode, self.strength = mode, strength
    self.reset()

  def reset(self):
    self.slow = self.fast = self.residual = None

  def update(self, request, speed, offset, heading, confidence, dt, *, enabled=False, lane_change=False, camera_age=0.):
    try:
      values = np.asarray([request, speed, offset, heading, confidence, dt, camera_age], dtype=float)
      valid = values.shape == (7,) and np.isfinite(values).all()
      valid = valid and all(isinstance(v, (int, float, np.number)) for v in (request, speed, offset, heading, confidence, dt, camera_age))
    except (TypeError, ValueError):
      valid = False
    if (not enabled or lane_change or not valid
        or not 0 <= camera_age <= .1 or not .005 <= dt <= .1 or speed < 11.2 or confidence <= .5 or abs(offset) > 1.5 or abs(heading) > .1):
      self.reset()
      return request
    if self.slow is None:
      self.slow = self.fast = request
      self.residual = 0.
    # Band-selective lead: a fast-minus-slow filter has zero DC gain.
    self.slow += (1-math.exp(-dt/3.))* (request-self.slow)
    self.fast += (1-math.exp(-dt/.45))* (request-self.fast)
    if self.mode == 'lead':
      delta = .5*(self.fast-self.slow)
    elif self.mode == 'lane':
      delta = -.3*offset-1.2*speed*heading
    elif self.mode == 'smooth':
      # Smooth the high-passed request; this is NOT an identified causal residual.
      high = request-self.slow
      self.residual += (1-math.exp(-dt/.7))*(high-self.residual)
      delta = .5*(self.residual-high)
    else:
      delta = 0.
    return float(request+np.clip(self.strength*delta, -.15, .15))
