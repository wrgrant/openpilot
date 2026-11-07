"""Steering characterization maneuvers for the Odyssey: small, slow lateral-accel requests on a straight, flat road
that show the steering motor's deadband, the torque it takes to get the wheel moving, and whether turning away from
straight behaves differently from returning to it.

Each maneuver is a push-pull-push (+a, -a for twice as long, +a) so the van ends up back on its line at the same
heading; the largest sideways excursion is about 20-25 cm. It is the default suite on this branch
(LateralManeuverSuite = "odyssey"; set it to "" for comma's stock maneuvers). Turn on Settings > Developer > Lateral
Maneuver Mode while parked. Report: tools/odyssey_eps_report.py.
"""
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.tools.longitudinal_maneuvers.maneuversd import Action

SETTLE_S = 3.0  # straight driving between runs, so each run starts from rest


def push_pull(accel: float, lobe_s: float, ramp_s: float) -> list[Action]:
  """+accel for lobe_s, -accel for 2 * lobe_s, +accel for lobe_s. The two direction changes ramp over ramp_s (slow
  ramps there show how the wheel crosses through straight); the start and end are steps so the areas cancel and the
  van finishes on its line."""
  a, T, r, e = accel, lobe_s, ramp_s, DT_MDL
  t = [0, e, T - r, T + r, 3 * T - r, 3 * T + r, 4 * T - e, 4 * T]
  v = [0, a, a, -a, -a, a, a, 0]
  return [Action(v, t), Action([0.0], [SETTLE_S])]


def _suite():
  from openpilot.tools.lateral_maneuvers.lateral_maneuversd import Maneuver
  out = []
  for mph in (50, 65):
    for accel, lobe, ramp, name in ((0.05, 2.0, 0.3, "nudge 0.05"), (0.1, 1.5, 0.3, "nudge 0.10"), (0.04, 2.6, 1.2, "slow sweep 0.04")):
      for first, sign in (("left", 1.0), ("right", -1.0)):  # + curvature is left in openpilot
        out.append(Maneuver(f"{name} {first} first {mph}mph", push_pull(sign * accel, lobe, ramp), repeat=2,
                            initial_speed=mph * CV.MPH_TO_MS))
  return out


def maneuvers():
  return _suite()
