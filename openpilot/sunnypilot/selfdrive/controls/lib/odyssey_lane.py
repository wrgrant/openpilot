"""Odyssey lane-centring correction (OdysseyLaneCenter / OdysseyLaneCenterAB, both off by default).

The driving model already pulls the van back toward the lane centre: in the video simulator its request behaves like
about 0.44 m/s² per metre off centre plus 1.2 m/s² per m/s of sideways drift (tools/odyssey_sim/reports/report_plan.md).
This adds a small, bounded correction of the same kind to the model's desired curvature: stiffer centring and more
damping of the slow 7-20 s wander. In the simulator it cut held-out wander by about 1.7 cm RMS; it has not been
driven yet.

Lane pose comes from the model's two inner lane lines, fitted over the first 25 m. Model frame: y is positive to the
van's right (verified on drives 1a7/1aa/1ab: the lane centre moves toward +y with the left blinker on), and openpilot
curvature is positive for a right turn, so a van right of centre gets a negative (leftward) correction.
It stays off below 25 mph, during lane changes and with blinkers on, when either inner line is unsure, and when the
model output is stale; then the model's curvature passes through unchanged.
"""
import math
import time

import numpy as np

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import LANE_ARMS, WANDER_BLOCK_S, ab_arm

K_OFFSET = 0.3  # m/s² of lateral accel per metre off the lane centre
K_HEADING = 1.2  # m/s² per m/s of sideways drift (speed × heading)
MAX_ACCEL = 0.15  # m/s²: bound on the correction
V_MIN = 11.2  # m/s (25 mph)
FIT_X = 25.0  # m of lane line used for the centre fit
MIN_PROB = 0.5  # both inner lane lines must be at least this likely
MAX_MODEL_AGE = 0.2  # s: the model runs at 20 Hz
MAX_OFFSET, MAX_HEADING = 1.5, 0.1  # m, rad: beyond these the fit is not a lane-keeping situation


def lane_pose(md) -> tuple[float, float] | None:
  """(offset m, heading rad) of the van from the centre of the two inner lane lines; + = right of centre / pointing
  right. None if the lines are missing, unsure or out of range."""
  if len(md.laneLines) != 4 or len(md.laneLineProbs) != 4 or min(md.laneLineProbs[1], md.laneLineProbs[2]) <= MIN_PROB:
    return None
  x = np.asarray(md.laneLines[1].x, dtype=float)
  yc = (np.asarray(md.laneLines[1].y, dtype=float) + np.asarray(md.laneLines[2].y, dtype=float)) / 2
  if x.shape != yc.shape:
    return None
  keep = x <= FIT_X
  if keep.sum() < 4 or not np.isfinite(x[keep]).all() or not np.isfinite(yc[keep]).all():
    return None
  c = np.polynomial.polynomial.polyfit(x[keep], yc[keep], 3)
  offset, heading = -float(c[0]), -math.atan(float(c[1]))
  if not (abs(offset) <= MAX_OFFSET and abs(heading) <= MAX_HEADING):
    return None
  return offset, heading


def correction(offset: float, heading: float, v_ego: float) -> float:
  """Lateral accel to add, in openpilot's curvature sign (+ = rightward)."""
  return float(np.clip(-K_OFFSET * offset - K_HEADING * v_ego * heading, -MAX_ACCEL, MAX_ACCEL))


class LaneCentering:
  def __init__(self, params: Params | None = None):
    p = params or Params()
    self.on = p.get_bool("OdysseyLaneCenter")
    self.ab = p.get_bool("OdysseyLaneCenterAB")
    self.delta = 0.0  # m/s², from the latest model frame
    self.model_mono = -math.inf
    if self.on or self.ab:
      # numpy's first polyfit takes about 60 ms on the comma; pay that here, not in the 100 Hz loop
      np.polynomial.polynomial.polyfit(np.arange(8.0), np.zeros(8), 3)

  def active_arm(self, mono_s: float) -> bool:
    return ab_arm(mono_s, LANE_ARMS, WANDER_BLOCK_S) == "on" if self.ab else self.on

  def update_model(self, md, v_ego: float, mono_s: float | None = None) -> None:
    """Call once per new modelV2 message. Does nothing while both params are off."""
    if not (self.on or self.ab):
      return
    pose = lane_pose(md)
    self.delta = 0.0 if pose is None else correction(pose[0], pose[1], v_ego)
    self.model_mono = time.monotonic() if mono_s is None else mono_s

  def apply(self, curvature: float, v_ego: float, lat_active: bool, lane_change: bool, mono_s: float | None = None) -> float:
    """The model's desired curvature plus the correction, or unchanged whenever the correction should not run."""
    mono_s = time.monotonic() if mono_s is None else mono_s
    if (not (self.on or self.ab) or not lat_active or lane_change or not v_ego >= V_MIN
        or mono_s - self.model_mono > MAX_MODEL_AGE or not math.isfinite(curvature) or not self.active_arm(mono_s)):
      return curvature
    return curvature + self.delta / (v_ego * v_ego)
