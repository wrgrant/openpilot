"""Wander friction assistance through OdysseySteer: off is identity, signs match openpilot, A/B arms and gates hold."""
from types import SimpleNamespace as NS

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import WANDER_BLOCK_S
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_steer import OdysseySteer


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    return self.values.get(key, 0.0)


def cs(v=29.0, pressed=False, blinker=False):
  return NS(vEgo=v, steeringAngleDeg=0.0, steeringRateDeg=0.0, steeringPressed=pressed, leftBlinker=blinker, rightBlinker=False)


def run(o, steer, des, act, frames=100, mono=0.0, **cs_kw):
  out = steer
  for k in range(frames):
    out = o.apply(steer, True, cs(**cs_kw), None, NS(roll=0.0, angleOffsetDeg=0.0), 0.0, 1.0, des, act, mono + k * 0.01)
  return out


def make(**params):
  o = OdysseySteer(FakeParams(**params))
  o.deadband_comp._params = FakeParams(**params)
  return o


class TestOdysseyWander(OpenpilotTestCase):
  def test_off_is_identity(self):
    assert run(make(), 0.05, 0.3, 0.0) == 0.05

  def test_pushes_toward_more_lateral_accel_with_openpilot_sign(self):
    # wanting more left (positive) lateral accel = more negative actuators.torque on this car
    out = run(make(OdysseyWanderFriction=True), 0.0, 0.3, 0.0)
    assert -0.22 <= out < -0.01

  def test_gates(self):
    assert run(make(OdysseyWanderFriction=True), 0.0, 0.3, 0.0, v=10.0) == 0.0, "highway speeds only"
    assert run(make(OdysseyWanderFriction=True), 0.0, 0.3, 0.0, pressed=True) == 0.0, "driver input"
    assert run(make(OdysseyWanderFriction=True), 0.0, 0.3, 0.0, blinker=True) == 0.0, "lane change"

  def test_ab_blocks(self):
    off_block = run(make(OdysseyWanderAB=True), 0.0, 0.3, 0.0, mono=0.5 * WANDER_BLOCK_S)
    on_block = run(make(OdysseyWanderAB=True), 0.0, 0.3, 0.0, mono=1.5 * WANDER_BLOCK_S)
    assert off_block == 0.0 and on_block < -0.01
