"""The Odyssey add-ons apply to the controller output regardless of which torque controller produced it."""
from types import SimpleNamespace as NS

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_steer import OdysseySteer


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    return self.values.get(key, 0.0)


class FakeVM:
  def get_steer_from_curvature(self, curv, u, roll):
    return curv * 3.0 * 17.0


def make(**params):
  o = OdysseySteer(FakeParams(**params))
  o.deadband_comp._params = FakeParams(**params)
  return o


class TestOdysseySteer(OpenpilotTestCase):
  CS_SLOW = NS(vEgo=2.0, steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=False, leftBlinker=False, rightBlinker=False)
  CS_FAST = NS(vEgo=25.0, steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=False, leftBlinker=False, rightBlinker=False)
  LIVE = NS(roll=0.0, angleOffsetDeg=0.0)

  def test_all_off_is_identity(self):
    o = make()
    for steer in (-0.5, 0.0, 0.02, 0.7):
      assert o.apply(steer, True, self.CS_SLOW, FakeVM(), self.LIVE, 0.01, 1.0) == steer

  def test_inactive_is_identity(self):
    o = make(SteerLowSpeedAngle=True, SteerDeadbandComp=0.03)
    assert o.apply(0.2, False, self.CS_SLOW, FakeVM(), self.LIVE, 0.05, 1.0) == 0.2

  def test_low_speed_angle_only_at_low_speed(self):
    o = make(SteerLowSpeedAngle=True)
    assert o.apply(0.9, True, self.CS_SLOW, FakeVM(), self.LIVE, 0.0, 1.0) != 0.9
    assert o.apply(0.9, True, self.CS_FAST, FakeVM(), self.LIVE, 0.0, 1.0) == 0.9

  def test_kick_applies_and_respects_steer_max(self):
    o = make(SteerDeadbandComp=0.03)
    assert o.apply(0.05, True, self.CS_FAST, FakeVM(), self.LIVE, 0.0, 1.0) > 0.05 + 0.029
    assert o.apply(0.99, True, self.CS_FAST, FakeVM(), self.LIVE, 0.0, 1.0) <= 1.0
