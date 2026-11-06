import math

from openpilot.selfdrive.ui.sunnypilot.onroad.steering_position import (
  DWELL_FRAMES, FALLBACK_EXTENT_DEG, PERSIST_MIN_INTERVAL_FRAMES, SteerExtentLearner, position_fraction,
)


def feed(learner, angle, frames):
  persist = False
  for _ in range(frames):
    persist |= learner.update(angle)
  return persist


class TestSteerExtentLearner:
  def test_unlearned_uses_fallback(self):
    learner = SteerExtentLearner()
    assert learner.extents() == (FALLBACK_EXTENT_DEG, FALLBACK_EXTENT_DEG)
    assert learner.learned() == (False, False)

  def test_learns_each_side_after_dwell(self):
    learner = SteerExtentLearner()
    feed(learner, 512.0, DWELL_FRAMES - 1)
    assert learner.left == 0.0  # not held long enough yet
    feed(learner, 512.0, 1)
    assert learner.left == 512.0
    feed(learner, -498.0, DWELL_FRAMES)
    assert learner.right == 498.0
    assert learner.learned() == (True, True)

  def test_side_switch_does_not_carry_peak(self):
    # straight from left lock to right lock with no centre sample in between
    learner = SteerExtentLearner()
    feed(learner, 560.0, DWELL_FRAMES)
    feed(learner, -470.0, DWELL_FRAMES)
    assert learner.extents() == (560.0, 470.0)

  def test_spike_is_ignored(self):
    learner = SteerExtentLearner(500.0, 500.0)
    feed(learner, 900.0, DWELL_FRAMES - 1)
    feed(learner, 10.0, 1)
    assert learner.left == 500.0

  def test_out_of_range_and_nan_ignored(self):
    learner = SteerExtentLearner()
    feed(learner, 45.0, 50)       # a normal turn is not a lock
    feed(learner, 5000.0, 50)     # glitch
    feed(learner, math.nan, 50)
    assert learner.learned() == (False, False)

  def test_only_grows(self):
    learner = SteerExtentLearner(520.0, 520.0)
    feed(learner, 300.0, 50)
    feed(learner, -300.0, 50)
    assert learner.extents() == (520.0, 520.0)

  def test_invalid_stored_values_rejected(self):
    assert SteerExtentLearner(5.0, 5000.0).learned() == (False, False)

  def test_persist_is_throttled(self):
    learner = SteerExtentLearner()
    assert feed(learner, 400.0, DWELL_FRAMES)  # first growth persists immediately
    learner.mark_persisted()
    assert not feed(learner, 410.0, DWELL_FRAMES)  # grew, but too soon after the last write
    assert feed(learner, 410.0, PERSIST_MIN_INTERVAL_FRAMES)
    learner.mark_persisted()
    assert not feed(learner, 410.5, PERSIST_MIN_INTERVAL_FRAMES)  # below the step size


class TestPositionFraction:
  def test_centre_and_ends(self):
    assert position_fraction(0.0, 500.0, 500.0) == 0.0
    assert position_fraction(500.0, 500.0, 500.0) == -1.0   # left lock -> left end of the bar
    assert position_fraction(-500.0, 500.0, 500.0) == 1.0
    assert position_fraction(800.0, 500.0, 500.0) == -1.0   # clamped

  def test_small_angles_visible(self):
    # sqrt scale: 5 degrees of a 500 degree lock moves the marker 10% of the half bar
    assert abs(position_fraction(-5.0, 500.0, 500.0) - 0.1) < 1e-9

  def test_asymmetric_extents(self):
    assert position_fraction(-450.0, 500.0, 450.0) == 1.0
    assert position_fraction(450.0, 500.0, 450.0) > -1.0
