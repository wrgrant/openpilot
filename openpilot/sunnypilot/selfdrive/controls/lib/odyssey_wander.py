"""Odyssey wander: friction assistance for small highway corrections (OdysseyWanderFriction, default off).

From tools/odyssey_sim/wander/runtime.py (report: wander worker, 2026-11-09). Adds the estimated friction the EPS
swallows (motor deadband + speed-scheduled stick-slip from the fitted plant v2) in the direction of the lateral-accel
error, smoothed by tanh(error/0.06), capped at 0.22 and slewed at 1.5 command/s; 18-40 m/s only. Works in
acceleration-positive units: openpilot's torque command (actuators.torque) is the opposite sign, so OdysseySteer
negates on the way in and out.
"""
import math

import numpy as np


def clip(x, lo, hi):
  return max(lo, min(hi, x))


class FrictionAssist:
  """Bounded error-conditioned friction experiment, not an identified LuGre model.

  Modes: smooth inverse/Karnopp-style gate, release pulse, or sinusoidal dither.
  Uses calibrated v2 friction/gain + motor deadband. Pulse and dither are bench
  candidates only. Rate is Honda degrees/s (magnitude used). No online learning.
  """

  def __init__(self, mode='inverse', scale=1.0):
    if mode not in ('inverse', 'pulse', 'dither') or not math.isfinite(scale) or not 0 <= scale <= 2:
      raise ValueError('invalid friction experiment')
    self.mode, self.scale = mode, scale
    self.reset()

  def reset(self):
    self.time = self.add = self.dwell = self.cooldown = self.remaining = self.direction = 0.0

  def update(self, command, *, target, actual, speed, rate, dt, enabled=False, valid=True):
    if not math.isfinite(command):
      self.reset()
      return 0.0
    if (
      not enabled
      or not valid
      or not all(math.isfinite(x) for x in (target, actual, speed, rate, dt))
      or not 0.001 <= dt <= 0.12
      or not 18 <= speed <= 40
      or abs(target) > 1.2
    ):
      self.reset()
      return command
    self.time += dt
    error = target - actual
    gain = float(np.interp(speed, [10, 20, 30], [1.1128, 2.0831, 2.1941]))
    friction = float(np.interp(speed, [10, 20, 30], [0.1698, 0.1858, 0.0714]))
    amplitude = min(0.22, self.scale * (0.028 + friction / gain))
    desired = 0.0
    if self.mode == 'inverse':
      desired = amplitude * math.tanh(error / 0.06)
    elif self.mode == 'dither':
      desired = amplitude * math.sin(2 * math.pi * 5 * self.time) * min(abs(error) / 0.06, 1.0)
    else:
      self.cooldown = max(0.0, self.cooldown - dt)
      self.dwell = self.dwell + dt if abs(error) > 0.025 and abs(rate) < 0.2 else 0.0
      if self.dwell > 0.25 and self.cooldown == 0:
        self.direction = math.copysign(1.0, error)
        self.remaining, self.cooldown, self.dwell = 0.15, 1.5, 0.0
      if self.remaining > 0 and error * self.direction > 0:
        desired = amplitude * self.direction
        self.remaining -= dt
      else:
        self.remaining = 0.0
    self.add += clip(desired - self.add, -1.5 * dt, 1.5 * dt)
    return clip(command + self.add, -1.0, 1.0)
