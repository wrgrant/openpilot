"""Counterfactual dynamics around a log; no recentering or fitted planner."""
import copy
import numpy as np
from tools.odyssey_sim.calib.plant import Plant


class PerturbationPlant:
  def __init__(self, params=None, sensitivity=None):
    params = copy.deepcopy(params or Plant().params)
    if sensitivity:
      name, scale = sensitivity
      indexes = {'gain': [0, 1, 2], 'friction': [4, 5, 6], 'lag': [3]}[name]
      for i in indexes:
        params['steering'][i] *= scale
      if name == 'lag':
        params['body'][3] *= scale
    self.sim, self.logged = Plant(params), Plant(params)

  def reset(self, steering, body, command):
    for p in (self.sim, self.logged):
      p.reset(steering, body, command)

  def step(self, command, logged_command, speed, roll, logged_steering, logged_body, dt=.01):
    self.sim.step(command, speed, roll, dt)
    self.logged.step(logged_command, speed, roll, dt)
    return logged_steering + (self.sim.y - self.logged.y), logged_body + (self.sim.body - self.logged.body)


def injection(t, speed, amplitude, period, shape):
  """y right, psi right; evaluate during warmup as well as scoring."""
  w = 2 * np.pi / period
  if shape == 'sine':
    y, yd = amplitude * np.sin(w * t), amplitude * w * np.cos(w * t)
  elif shape == 'ramp':
    # Triangular ramps; exact derivative except at the turning points.
    q = (t / period) % 1
    y = amplitude * (1 - 4 * abs(q - .5))
    yd = amplitude * (4 / period) * (1 if q < .5 else -1)
  elif shape == 'step':
    # A finite-rise step avoids an unphysical infinite heading at a discontinuity.
    q = np.tanh(4 * np.sin(w * t))
    y, yd = amplitude * q, amplitude * (1 - q*q) * 4*w*np.cos(w*t)
  else:
    raise ValueError(shape)
  return float(y), float(yd / max(speed, .1))
