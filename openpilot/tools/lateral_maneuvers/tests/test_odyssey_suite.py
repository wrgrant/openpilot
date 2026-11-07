"""The Odyssey characterization maneuvers stay small and bring the van back to its line."""
import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.tools.lateral_maneuvers.odyssey_suite import maneuvers

MAX_EXCURSION_M = 0.35
MAX_ACCEL = 0.1  # m/s^2


def run_once(m) -> np.ndarray:
  """Step one run of a maneuver the way lateral_maneuversd does, after it has become active."""
  m._active = True
  out = []
  while True:
    out.append(m._step())
    if m._run_completed:
      return np.array(out)


@pytest.mark.parametrize("m", maneuvers(), ids=lambda m: m.description)
def test_small_and_returns_to_line(m):
  accel = run_once(m)
  vel = np.cumsum(accel) * DT_MDL
  pos = np.cumsum(vel) * DT_MDL
  assert np.abs(accel).max() <= MAX_ACCEL + 1e-9
  assert np.abs(pos).max() <= MAX_EXCURSION_M, f"strays {np.abs(pos).max():.2f} m"
  assert abs(vel[-1]) < 0.01 and abs(pos[-1]) < 0.05, "should end on its line at its starting heading"


def test_both_directions_and_speeds():
  names = [m.description for m in maneuvers()]
  assert any("left first" in n for n in names) and any("right first" in n for n in names)
  assert {n.split()[-1] for n in names} == {"50mph", "65mph"}
