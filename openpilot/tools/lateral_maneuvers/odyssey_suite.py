"""Steering characterization maneuvers for the Odyssey: small, slow lateral-accel requests that show the steering
motor's deadband, the torque it takes to get the wheel moving, and whether turning away from straight behaves
differently from returning to it.

Each maneuver is a push-pull-push (+a, -a for twice as long, +a) added on top of openpilot's normal lane keeping, so
the van stays in its lane and any highway works, curves included. Each run takes 6-10 s and starts at whatever steady
speed the van is cruising between MIN_MPH and MAX_MPH. Finished maneuvers are remembered across drives
(LateralManeuverDone), so the suite can be done in pieces. It is the default suite on this branch
(LateralManeuverSuite = "odyssey"; set it to "" for comma's stock maneuvers). Turn on Settings > Developer > Lateral
Maneuver Mode while parked. Report: tools/odyssey_eps_report.py.
"""
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.tools.longitudinal_maneuvers.maneuversd import Action
from openpilot.tools.lateral_maneuvers.lateral_maneuversd import MAX_SPEED_DEV, Maneuver

SETTLE_S = 3.0  # lane keeping only between runs, so each run starts from rest
MIN_MPH, MAX_MPH = 45.0, 80.0


def push_pull(accel: float, lobe_s: float, ramp_s: float) -> list[Action]:
  """+accel for lobe_s, -accel for 2 * lobe_s, +accel for lobe_s. The two direction changes ramp over ramp_s (slow
  ramps there show how the wheel crosses through straight); the start and end are steps so the areas cancel and the
  request alone would leave the van on its line."""
  a, T, r, e = accel, lobe_s, ramp_s, DT_MDL
  t = [0, e, T - r, T + r, 3 * T - r, 3 * T + r, 4 * T - e, 4 * T]
  v = [0, a, a, -a, -a, a, a, 0]
  return [Action(v, t), Action([0.0], [SETTLE_S])]


@dataclass
class HighwayManeuver(Maneuver):
  """Starts at any steady speed in [MIN_MPH, MAX_MPH] and rides on top of lane keeping."""
  follow_lane: bool = True

  def get_accel(self, v_ego: float, lat_active: bool, curvature: float, roll: float) -> float:
    in_range = MIN_MPH * CV.MPH_TO_MS <= v_ego <= MAX_MPH * CV.MPH_TO_MS
    if not self._active and in_range and abs(v_ego - self.initial_speed) >= MAX_SPEED_DEV:
      self.initial_speed = v_ego  # re-anchor to the current speed and restart the timer: it has to hold there
      self._ready_cnt = 0
    return super().get_accel(v_ego, lat_active, curvature, roll)


def maneuvers() -> list[Maneuver]:
  out: list[Maneuver] = []
  for accel, lobe, ramp, name in ((0.05, 2.0, 0.3, "nudge 0.05"), (0.1, 1.5, 0.3, "nudge 0.10"), (0.04, 2.6, 1.2, "slow sweep 0.04")):
    for first, sign in (("left", 1.0), ("right", -1.0)):  # + curvature is left in openpilot
      out.append(HighwayManeuver(f"{name} {first} first", push_pull(sign * accel, lobe, ramp), repeat=2,
                                 initial_speed=65 * CV.MPH_TO_MS))
  return out
