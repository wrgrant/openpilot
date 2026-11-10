"""Lane-centring correction: pushes toward the lane centre in openpilot's sign, bounded, and off whenever it should be."""
from types import SimpleNamespace as NS

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib import odyssey_lane as ol
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import WANDER_BLOCK_S

X = np.asarray(ModelConstants.X_IDXS, dtype=float)
V = 29.0


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def model(centre=0.0, slope=0.0, probs=(0.9, 0.9, 0.9, 0.9), width=3.6):
  """modelV2 stand-in: lane centre y = centre + slope * x (model frame, + = right of the van)."""
  yc = centre + slope * X
  lines = [NS(x=list(X), y=list(yc + off)) for off in (-1.5 * width, -width / 2, width / 2, 1.5 * width)]
  return NS(laneLines=lines, laneLineProbs=list(probs))


def lane(on=True, ab=False):
  return ol.LaneCentering(FakeParams(OdysseyLaneCenter=on, OdysseyLaneCenterAB=ab))


class TestOdysseyLane(OpenpilotTestCase):
  def test_right_of_centre_steers_left(self):
    # lane centre 0.2 m to the van's left (y = -0.2): the van is 0.2 m right of centre
    offset, heading = ol.lane_pose(model(centre=-0.2))
    self.assertAlmostEqual(offset, 0.2, places=3)
    self.assertAlmostEqual(heading, 0.0, places=4)
    self.assertLess(ol.correction(offset, heading, V), 0.0)  # openpilot curvature: negative = left

  def test_pointing_right_steers_left(self):
    # lane centre drifts to the left ahead (y decreasing with x): the van points right of the lane
    offset, heading = ol.lane_pose(model(slope=-0.002))
    self.assertGreater(heading, 0.0)
    self.assertLess(ol.correction(offset, heading, V), 0.0)
    _, heading = ol.lane_pose(model(slope=0.002))
    self.assertGreater(ol.correction(0.0, heading, V), 0.0)

  def test_bounded(self):
    self.assertEqual(ol.correction(1.4, 0.09, V), -ol.MAX_ACCEL)
    self.assertEqual(ol.correction(-1.4, -0.09, V), ol.MAX_ACCEL)

  def test_apply_adds_accel_as_curvature(self):
    lc = lane()
    lc.update_model(model(centre=-0.2), V, mono_s=10.0)
    expected = ol.correction(0.2, 0.0, V) / V**2
    self.assertAlmostEqual(lc.apply(0.001, V, True, False, mono_s=10.05), 0.001 + expected, places=9)

  def test_off_cases_pass_curvature_through(self):
    lc = lane()
    lc.update_model(model(centre=-0.2), V, mono_s=10.0)
    k = 0.001
    self.assertEqual(lane(on=False).apply(k, V, True, False, mono_s=10.05), k)  # param off
    self.assertEqual(lc.apply(k, V, False, False, mono_s=10.05), k)  # not steering
    self.assertEqual(lc.apply(k, V, True, True, mono_s=10.05), k)  # lane change / blinker
    self.assertEqual(lc.apply(k, ol.V_MIN - 0.1, True, False, mono_s=10.05), k)  # below 25 mph
    self.assertEqual(lc.apply(k, V, True, False, mono_s=10.0 + ol.MAX_MODEL_AGE + 0.01), k)  # stale model

  def test_correction_is_smoothed(self):
    lc = lane()
    lc.update_model(model(), V, mono_s=10.0)
    target = ol.correction(0.2, 0.0, V)
    for i in range(1, 11):  # the van is suddenly 0.2 m right of centre, for 0.5 s of model frames
      lc.update_model(model(centre=-0.2), V, mono_s=10.0 + 0.05 * i)
    self.assertAlmostEqual(lc.delta / target, 1 - (ol.SMOOTH_S / (ol.SMOOTH_S + 0.05))**10, places=6)
    self.assertTrue(0.55 < lc.delta / target < 0.7)
    lc.update_model(model(centre=-0.2, probs=(0.9, 0.4, 0.9, 0.9)), V, mono_s=10.55)  # lines lost: fades, no step
    self.assertTrue(0.5 < lc.delta / target < 0.65)
    lc.update_model(model(centre=-0.2), V, mono_s=20.0)  # after a gap it starts from the new value
    self.assertAlmostEqual(lc.delta, target)

  def test_off_does_no_work(self):
    lc = lane(on=False)
    lc.update_model(None, V, mono_s=10.0)  # would raise if it touched the model message
    self.assertEqual(lc.model_mono, -float('inf'))

  def test_unsure_or_bad_lines_give_no_correction(self):
    self.assertIsNone(ol.lane_pose(model(probs=(0.9, 0.4, 0.9, 0.9))))
    self.assertIsNone(ol.lane_pose(model(centre=-2.0)))
    bad = model(centre=-0.2)
    bad.laneLines[1].y[0] = float('nan')
    self.assertIsNone(ol.lane_pose(bad))
    self.assertIsNone(ol.lane_pose(NS(laneLines=[], laneLineProbs=[])))
    lc = lane()
    lc.update_model(model(probs=(0.9, 0.4, 0.9, 0.9), centre=-0.2), V, mono_s=10.0)
    self.assertEqual(lc.apply(0.001, V, True, False, mono_s=10.05), 0.001)

  def test_ab_blocks_alternate(self):
    lc = lane(on=False, ab=True)
    lc.update_model(model(centre=-0.2), V, mono_s=0.0)
    arms = []
    for block in range(4):
      t = (block + 0.5) * WANDER_BLOCK_S
      lc.model_mono = t
      arms.append(lc.apply(0.001, V, True, False, mono_s=t) != 0.001)
    self.assertEqual(arms, [False, True, False, True])
