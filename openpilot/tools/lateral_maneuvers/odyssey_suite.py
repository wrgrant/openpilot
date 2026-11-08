"""Steering characterization maneuvers for the Odyssey: how big a steering torque pulse has to be before the wheel
and the van respond, left vs right, which sets the size of the SteerDeadbandComp kick.

Each maneuver holds lane keeping's own torque still for a moment and adds one torque pulse to it, so the wheel's
response to the pulse alone is visible; holds with no pulse measure the noise. Between holds openpilot's normal lane
keeping steers, so the van stays in its lane and any highway works, curves included. Lateral-accel requests on top of
lane keeping don't work for this: the model cancels them within about a second. Each run takes 30-35 s and starts at
whatever steady speed the van is cruising between MIN_MPH and MAX_MPH. Finished maneuvers are remembered across drives
(LateralManeuverDone), so the suite can be done in pieces. It is the default suite on this branch
(LateralManeuverSuite = "odyssey"; set it to "" for comma's stock maneuvers). Turn on Settings > Developer > Lateral
Maneuver Mode while parked. Report: tools/odyssey_eps_report.py.
"""
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.tools.longitudinal_maneuvers.maneuversd import Action
from openpilot.tools.lateral_maneuvers.lateral_maneuversd import MAX_SPEED_DEV, Maneuver

MIN_MPH, MAX_MPH = 38.0, 80.0


PULSE_LEVELS = (0.0, 0.03, 0.045, 0.06, 0.08)  # fraction of max torque, 0-328 of 4096; 0 is a hold with no pulse
PULSE_WIDTHS = (0.5, 1.0)  # s
HOLD_PRE_S, HOLD_POST_S = 0.3, 0.7  # lane keeping's torque is held from before each pulse until after it
PULSE_GAP_S = 1.5  # normal lane keeping between holds, so the van recentres
MAX_HOLD_CORRECTION = 0.1  # the hold never moves the command more than this from what lane keeping asks for


def pulse_train(levels: tuple[float, ...], width_s: float, first: float) -> tuple[list[Action], list[tuple[float, float]]]:
  """Steering torque pulses, each level once each way (first, then -first), width_s long, and the windows (start, end
  in s) during which lane keeping's torque is held still around each pulse. Net torque is zero."""
  # breakpoints sit on half frames so each pulse is on for exactly width_s / DT_MDL plan frames
  pre, width, post, gap = (round(x / DT_MDL) for x in (HOLD_PRE_S, width_s, HOLD_POST_S, PULSE_GAP_S))
  t, v, holds, k = [0.0], [0.0], [], 0
  for level in levels:
    for sign in (first, -first):
      on = k + pre
      t += [(on + 0.5) * DT_MDL, (on + 1) * DT_MDL, (on + width) * DT_MDL, (on + width + 0.5) * DT_MDL]
      v += [0.0, sign * level, sign * level, 0.0]
      holds.append(((k + 0.5) * DT_MDL, (on + width + post + 0.5) * DT_MDL))
      k = on + width + post + gap
  t.append(k * DT_MDL)
  v.append(0.0)
  return [Action(v, t)], holds


@dataclass
class HighwayManeuver(Maneuver):
  """Starts at any steady speed in [MIN_MPH, MAX_MPH] and rides on top of lane keeping."""
  follow_lane: bool = True
  holds: tuple[tuple[float, float], ...] = ()  # windows of the (single) action where lane keeping's torque is held
  _held: float | None = None

  def hold_correction(self, steer_now: float) -> float:
    """Inside a hold window, what to add so the command stays where lane keeping's torque was when the hold began."""
    t = max(self._action_frames - 1, 0) * DT_MDL  # _step has already advanced past the value just returned
    if self._action_index != 0 or not any(a <= t < b for a, b in self.holds):
      self._held = None
      return 0.0
    if self._held is None:
      self._held = steer_now
    return min(max(self._held - steer_now, -MAX_HOLD_CORRECTION), MAX_HOLD_CORRECTION)

  def reset(self):
    super().reset()
    self._held = None

  def get_accel(self, v_ego: float, lat_active: bool, curvature: float, roll: float) -> float:
    in_range = MIN_MPH * CV.MPH_TO_MS <= v_ego <= MAX_MPH * CV.MPH_TO_MS
    if not self._active and in_range and abs(v_ego - self.initial_speed) >= MAX_SPEED_DEV:
      self.initial_speed = v_ego  # re-anchor to the current speed and restart the timer: it has to hold there
      self._ready_cnt = 0
    return super().get_accel(v_ego, lat_active, curvature, roll)


def maneuvers() -> list[Maneuver]:
  out: list[Maneuver] = []
  for width in PULSE_WIDTHS:
    for first, sign in (("left", 1.0), ("right", -1.0)):  # + torque is left in openpilot
      actions, holds = pulse_train(PULSE_LEVELS, width, sign)
      out.append(HighwayManeuver(f"held pulses {width}s {first} first", actions, repeat=1, initial_speed=65 * CV.MPH_TO_MS,
                                 torque=True, holds=tuple(holds)))
  return out
