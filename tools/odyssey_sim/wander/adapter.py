"""calib.evaluate real LatControlTorqueV0/NNLC friction candidate factories.

The adapter evaluates logged-plan tracking only; it has no lane observation.
"""

from tools.odyssey_sim.calib.controller import Controller
from tools.odyssey_sim.wander.runtime import FrictionAssist


class AssistedController(Controller):
  def __init__(self, meta, cfg, mode, scale=1.0):
    super().__init__(meta, cfg)
    self.assist = FrictionAssist(mode, scale)
    self.previous = None

  def update(self, target_lataccel, current_lataccel, state, future_plan=None):
    command = super().update(target_lataccel, current_lataccel, state, future_plan)
    rate = 0.0 if self.previous is None else (current_lataccel - self.previous) / 0.01
    self.previous = current_lataccel
    # Proxy rate to degrees/s: for pulse stationarity screening only.
    rate *= 57.2958 * state.ratio * 2.9 / max(state.v_ego**2, 1.0)
    return self.assist.update(command, target=target_lataccel, actual=current_lataccel, speed=state.v_ego, rate=rate, dt=0.01, enabled=True)


def inverse(meta, cfg):
  return AssistedController(meta, cfg, 'inverse')


def pulse(meta, cfg):
  return AssistedController(meta, cfg, 'pulse')


def dither(meta, cfg):
  return AssistedController(meta, cfg, 'dither')
