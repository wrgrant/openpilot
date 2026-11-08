"""The Odyssey characterization maneuvers stay small and bring the van back to its line."""
import numpy as np
import pytest

from openpilot.common.constants import CV
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


@pytest.mark.parametrize("m", [m for m in maneuvers() if not m.torque], ids=lambda m: m.description)
def test_small_and_returns_to_line(m):
  accel = run_once(m)
  vel = np.cumsum(accel) * DT_MDL
  pos = np.cumsum(vel) * DT_MDL
  assert np.abs(accel).max() <= MAX_ACCEL + 1e-9
  assert np.abs(pos).max() <= MAX_EXCURSION_M, f"strays {np.abs(pos).max():.2f} m"
  assert abs(vel[-1]) < 0.01 and abs(pos[-1]) < 0.05, "should end on its line at its starting heading"


def test_both_directions():
  names = [m.description for m in maneuvers()]
  assert any("left first" in n for n in names) and any("right first" in n for n in names)
  assert len(set(names)) == len(names), "names key the progress saved across drives"
  assert all(m.follow_lane for m in maneuvers()), "rides on lane keeping, so the road needn't be straight"


def frames_to_start(m, speeds_mph) -> int | None:
  for i, mph in enumerate(speeds_mph):
    m.get_accel(mph * CV.MPH_TO_MS, True, 0.0, 0.0)
    if m.active:
      return i
  return None


@pytest.mark.parametrize("mph", [47, 58, 72])
def test_starts_at_any_steady_highway_speed(mph):
  i = frames_to_start(maneuvers()[0], [mph] * 100)
  assert i is not None and i * DT_MDL < 2.5


def test_waits_while_speed_changes_or_too_slow():
  assert frames_to_start(maneuvers()[0], np.linspace(50, 70, 200)) is None  # accelerating ~1 mph/s
  assert frames_to_start(maneuvers()[0], [35] * 200) is None


@pytest.mark.parametrize("m", [m for m in maneuvers() if m.torque], ids=lambda m: m.description)
def test_torque_pulses_small_balanced_and_sharp(m):
  tq = run_once(m)
  assert np.abs(tq).max() <= 0.08 + 1e-9, "stays well below what the driver would notice as a tug"
  assert abs(tq.sum() * DT_MDL) < 1e-6, "net torque zero, so the van isn't pushed off its line"
  assert tq[-1] == 0.0
  vals, counts = np.unique(np.round(np.abs(tq), 4), return_counts=True)
  levels = vals[(vals > 0) & (counts >= 4)]  # pulse plateaus, not the one-frame edges
  assert len(levels) == 6 and levels.min() < 0.037 < levels.max(), "brackets the measured deadband"


@pytest.mark.parametrize("m", [m for m in maneuvers() if m.torque], ids=lambda m: m.description)
def test_torque_pulses_last_their_width(m):
  on = np.abs(run_once(m)) > 0
  edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
  widths = (edges[1::2] - edges[::2]) * DT_MDL
  assert np.allclose(widths, float(m.description.split()[2].rstrip("s")))
