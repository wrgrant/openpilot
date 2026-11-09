"""Experimental default-off lane and friction add-ons. No I/O or openpilot imports.

Acceleration-positive conventions throughout, including command. Honda runtime
command is opposite: lead must negate on both sides of FrictionAssist.update.
Camera offset = lane centre relative to car, metres, positive toward +curvature.
Camera heading and road curvature must use the same convention as body yaw.
"""

import math
import numpy as np


def clip(x, lo, hi):
  return max(lo, min(hi, x))


class LaneCentering:
  """Complementary lane observer; 100 Hz prediction, new-camera-only innovation.

  e_dot = v*h; h_dot = v*k_road - yaw. This approximation neglects sideslip.
  Camera anchors integration; it does not provide ground truth road geometry.
  Output is additive desired lateral acceleration; divide by v² for curvature.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.e = self.h = self.correction = self.bias = 0.0
    self.age = 1e9
    self.initialized = False

  def update(self, *, dt, speed, yaw, road_curvature, camera_offset, camera_heading=None, camera_new=False, valid=False, enabled=False):
    values = [dt, speed, yaw, road_curvature, camera_offset]
    if camera_heading is not None:
      values.append(camera_heading)
    if (
      not enabled
      or not valid
      or not all(math.isfinite(x) for x in values)
      or not 0.001 <= dt <= 0.12
      or not 18 <= speed <= 40
      or abs(road_curvature) * speed**2 > 1.2
      or abs(camera_offset) > 0.6
    ):
      self.reset()
      return 0.0
    self.age += dt
    if camera_new:
      if not self.initialized:
        self.e = camera_offset
        self.h = 0.0 if camera_heading is None else camera_heading
        self.initialized = True
      # Model-frame interval, NOT dt: avoid applying one frame 5 times.
      frame_dt = clip(self.age, 0.01, 0.1)
      self.age = 0.0
    else:
      frame_dt = 0.0
    if not self.initialized or self.age > 0.25:
      self.reset()
      return 0.0
    self.h += (speed * road_curvature - yaw + self.bias / speed) * dt
    self.e += speed * self.h * dt
    if camera_new:
      innovation = clip(camera_offset - self.e, -0.12, 0.12)
      self.e += (1 - math.exp(-2.4 * frame_dt)) * innovation
      self.h += 1.92 * frame_dt * innovation / speed
      self.bias = clip(self.bias + 0.512 * frame_dt * innovation, -0.2, 0.2)
      if camera_heading is not None:
        self.h += (1 - math.exp(-frame_dt / 0.7)) * clip(camera_heading - self.h, -0.02, 0.02)
    self.e = clip(self.e, -0.6, 0.6)
    self.h = clip(self.h, -0.03, 0.03)
    # Lower natural frequency at higher speed; compensate aggregate actuator lag.
    wn = float(np.interp(speed, [18.0, 25.0, 35.0, 40.0], [0.55, 0.50, 0.42, 0.38]))
    ed = clip(speed * self.h, -0.4, 0.4)
    target = clip(wn**2 * (self.e + 0.7 * ed) + 1.8 * wn * ed, -0.12, 0.12)
    self.correction += clip(target - self.correction, -0.08 * dt, 0.08 * dt)
    return self.correction


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
