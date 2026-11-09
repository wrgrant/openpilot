"""Research-only angle controller. Caller explicitly enables; no device/Params access.

All angles and torques use carState angle / actuators.torque sign. Future samples
must be from ONE currently available model prediction, never future log frames.
"""

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Gains:
  kp: float = 0.02
  ki: float = 0.003
  kd: float = 0.012
  friction: float = 0.22
  hold: float = 0.003
  rate_ff: float = 0.025
  max_rate: float = 12.0
  slew: float = 0.65
  preview: float = 0.4


@dataclass(frozen=True)
class Result:
  torque: float
  limited: bool = False
  authority_limited: bool = False
  reference: float = 0.0


class AngleController:
  def __init__(self, enabled=False, gains=None, dt=0.01):
    if not math.isfinite(dt) or not 0 < dt <= 0.1:
      raise ValueError('dt must be in (0, .1]')
    self.enabled, self.g, self.dt = enabled, gains or Gains(), dt
    self.reset()

  def reset(self):
    self.i = self.stall = 0.0
    self.ref = self.previous = None

  def update(self, stock, speed, angle, rate, desired, active=True, driver=False, future=None):
    """Return authority flag for caller's established alert path; never disengage silently.

    At a standstill return stock; the integrator resets. Above 8 m/s pass stock
    through exactly. On sustained stall, retain bounded effort and report inability;
    reducing effort or releasing on a tight turn needs on-road evidence first.
    """
    good = all(math.isfinite(x) for x in (stock, speed, angle, rate, desired))
    if not good or not self.enabled or not active or driver or speed < 0.5 or speed >= 8:
      self.reset()
      return Result(float(np.clip(stock, -1, 1)) if math.isfinite(stock) else 0.0)
    g, dt = self.g, self.dt
    target = desired
    if future is not None:
      try:
        t, a = (np.asarray(x, dtype=float) for x in future)
      except (ValueError, TypeError):
        self.reset()
        return Result(float(np.clip(stock, -1, 1)))
      if t.ndim != 1 or a.shape != t.shape or len(t) < 2 or not np.all(np.isfinite(t)) or not np.all(np.isfinite(a)) or np.any(np.diff(t) <= 0) or t[0] < 0:
        self.reset()
        return Result(float(np.clip(stock, -1, 1)))
      target = float(np.interp(g.preview, t, a))
    if self.ref is None:
      self.ref, self.previous = angle, stock
    step = float(np.clip(target - self.ref, -g.max_rate * dt, g.max_rate * dt))
    self.ref += step
    wanted_rate = step / dt
    error = self.ref - angle
    motion = wanted_rate + 0.8 * error
    friction = g.friction * math.tanh(motion / 3.0)
    raw = g.kp * error + self.i + g.kd * (wanted_rate - rate) + friction
    raw += g.hold * self.ref + g.rate_ff * wanted_rate
    bounded = float(np.clip(raw, -0.95, 0.95))
    if abs(raw) < 0.95 or raw * error < 0:
      self.i = float(np.clip(self.i + g.ki * error * dt, -0.2, 0.2))
    self.stall = self.stall + dt if abs(error) > 10 and abs(rate) < 1 and abs(bounded) > 0.8 else max(0.0, self.stall - dt)
    w = float(np.clip((8 - speed) / 3.5, 0, 1))
    w = w * w * (3 - 2 * w)
    low = float(np.clip(bounded, self.previous - g.slew * dt, self.previous + g.slew * dt))
    self.previous = low
    torque = (1 - w) * stock + w * low
    return Result(float(np.clip(torque, -1, 1)), abs(step) >= g.max_rate * dt - 1e-9, self.stall > 1.5, self.ref)
