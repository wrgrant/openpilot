"""Kick small steering torque requests through the steering motor's deadband.

On the Odyssey the EPS motor barely answers commands below about 2-4% of max torque, and openpilot's own friction term
only fires on a measured lateral-accel error, which the 0.1 degree angle sensor can't see for the small corrections
that hold the lane. SteerDeadbandComp (normalized torque, 0 = off) adds that much torque in the direction of any
request, blended in over SMOOTH_WIDTH near zero so it doesn't chatter back and forth.

A negative SteerDeadbandComp selects bump mode instead (preliminary): requests smaller than |value| are sent as
pulses of |value|, on for a share of each BUMP_PERIOD_FRAMES equal to the request's share of |value|, so the average
torque matches the request but each pulse is big enough to break the wheel free. Larger requests pass through.

SteerDeadbandCompAB rotates through AB_ARMS in AB_BLOCK_S blocks of the monotonic clock (the clock behind
logMonoTime): no compensation, the kick at AB_KICK, bump mode at AB_BUMP, ignoring SteerDeadbandComp. One drive then
compares all three on the same roads; tools/odyssey_wander.py --ab scores them.
"""
import math
import time

from openpilot.common.params import Params

SMOOTH_WIDTH = 0.01  # normalized torque over which the kick blends in around zero
PARAMS_PERIOD_FRAMES = 500  # re-read the setting every 5 s at 100 Hz
AB_BLOCK_S = 80.0  # one off -> kick -> bump cycle every 4 min, so a straight stretch sees all three
BUMP_PERIOD_FRAMES = 25  # 0.25 s at 100 Hz
BUMP_MIN = 0.003  # below this a request is treated as zero, so noise around zero doesn't pulse


AB_ARMS = ("off", "kick", "bump")
AB_KICK, AB_BUMP = 0.03, 0.06


def ab_arm(mono_s: float) -> str:
  """In A/B mode, which compensation runs at this monotonic time (s)."""
  return AB_ARMS[int(mono_s // AB_BLOCK_S) % len(AB_ARMS)]


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
    if self.ab:
      amount = {"off": 0.0, "kick": AB_KICK, "bump": -AB_BUMP}[ab_arm(time.monotonic() if mono_s is None else mono_s)]
    out = bump_comp(torque, -amount, self._frame) if amount < 0 else deadband_comp(torque, amount)
    return max(-steer_max, min(steer_max, out))
