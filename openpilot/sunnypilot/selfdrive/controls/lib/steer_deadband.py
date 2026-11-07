"""Kick small steering torque requests through the steering motor's deadband.

On the Odyssey the EPS motor gives essentially no torque for commands below about 150 of 4096 (3.7%), and openpilot's
own friction term only fires on a measured lateral-accel error, which the 0.1 degree angle sensor can't see for the
small corrections that hold the lane. SteerDeadbandComp (normalized torque, 0 = off) adds that much torque in the
direction of any request, blended in over SMOOTH_WIDTH near zero so it doesn't chatter back and forth.
"""
import math

from openpilot.common.params import Params

SMOOTH_WIDTH = 0.01  # normalized torque over which the kick blends in around zero
PARAMS_PERIOD_FRAMES = 500  # re-read the setting every 5 s at 100 Hz


def deadband_comp(torque: float, amount: float, width: float = SMOOTH_WIDTH) -> float:
  if amount <= 0.0:
    return torque
  return torque + amount * math.tanh(torque / width)


class SteerDeadbandComp:
  def __init__(self):
    self._params = Params()
    self._frame = 0
    self.amount = 0.0

  def update(self, torque: float, steer_max: float) -> float:
    if self._frame % PARAMS_PERIOD_FRAMES == 0:
      try:
        self.amount = max(0.0, float(self._params.get("SteerDeadbandComp", return_default=True) or 0.0))
      except Exception:
        self.amount = 0.0
    self._frame += 1
    return max(-steer_max, min(steer_max, deadband_comp(torque, self.amount)))
