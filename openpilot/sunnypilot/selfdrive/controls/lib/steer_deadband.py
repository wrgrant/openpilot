"""Kick small steering torque requests through the steering motor's deadband.

On the Odyssey the EPS motor barely answers commands below about 2-4% of max torque, and openpilot's own friction term
only fires on a measured lateral-accel error, which the 0.1 degree angle sensor can't see for the small corrections
that hold the lane. SteerDeadbandComp (normalized torque, 0 = off) adds that much torque in the direction of any
request, blended in over SMOOTH_WIDTH near zero so it doesn't chatter back and forth.

A negative SteerDeadbandComp selects bump mode instead (preliminary): requests smaller than |value| are sent as
pulses of |value|, on for a share of each BUMP_PERIOD_FRAMES equal to the request's share of |value|, so the average
torque matches the request but each pulse is big enough to break the wheel free. Larger requests pass through.

SteerDeadbandCompAB turns the kick on only in every other AB_BLOCK_S block of the monotonic clock (the clock behind
logMonoTime), so one drive compares kick and no kick on the same roads: tools/odyssey_wander.py --ab scores it.
"""
import math
import time

from openpilot.common.params import Params

SMOOTH_WIDTH = 0.01  # normalized torque over which the kick blends in around zero
PARAMS_PERIOD_FRAMES = 500  # re-read the setting every 5 s at 100 Hz
AB_BLOCK_S = 180.0
BUMP_PERIOD_FRAMES = 25  # 0.25 s at 100 Hz
BUMP_MIN = 0.003  # below this a request is treated as zero, so noise around zero doesn't pulse


def ab_kick_on(mono_s: float) -> bool:
  """In A/B mode, whether the kick is on at this monotonic time (s): odd-numbered blocks."""
  return int(mono_s // AB_BLOCK_S) % 2 == 1


def deadband_comp(torque: float, amount: float, width: float = SMOOTH_WIDTH) -> float:
  if amount <= 0.0:
    return torque
  return torque + amount * math.tanh(torque / width)


def bump_comp(torque: float, amp: float, phase: int, period: int = BUMP_PERIOD_FRAMES) -> float:
  """Pulse-width version of a small request: amp for a share of each period equal to |torque| / amp."""
  if amp <= 0.0 or abs(torque) >= amp:
    return torque
  if abs(torque) < BUMP_MIN:
    return 0.0
  on_frames = round(abs(torque) / amp * period)
  return math.copysign(amp, torque) if (phase % period) < on_frames else 0.0


class SteerDeadbandComp:
  def __init__(self):
    self._params = Params()
    self._frame = 0
    self.amount = 0.0
    self.ab = False

  def update(self, torque: float, steer_max: float, mono_s: float | None = None) -> float:
    if self._frame % PARAMS_PERIOD_FRAMES == 0:
      try:
        self.amount = float(self._params.get("SteerDeadbandComp", return_default=True) or 0.0)
        self.ab = bool(self._params.get_bool("SteerDeadbandCompAB"))
      except Exception:
        self.amount, self.ab = 0.0, False
    self._frame += 1
    amount = self.amount
    if self.ab and not ab_kick_on(time.monotonic() if mono_s is None else mono_s):
      amount = 0.0
    out = bump_comp(torque, -amount, self._frame) if amount < 0 else deadband_comp(torque, amount)
    return max(-steer_max, min(steer_max, out))
