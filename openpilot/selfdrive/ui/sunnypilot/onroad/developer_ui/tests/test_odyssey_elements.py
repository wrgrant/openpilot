import unittest
from types import SimpleNamespace as NS

from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.elements import CurveSpeedElement, SteeringTorquePercentElement


def sm_for(v_ego=25.0, rate=None, vel=None, torque=0.0, lat_active=True, steering_pressed=False):
  n = 33
  rate = [0.0] * n if rate is None else rate
  vel = [v_ego] * n if vel is None else vel
  return {
    'carState': NS(vEgo=v_ego, steeringPressed=steering_pressed),
    'modelV2': NS(orientationRate=NS(z=rate), velocity=NS(x=vel)),
    'carOutput': NS(actuatorsOutput=NS(torque=torque)),
    'carControl': NS(latActive=lat_active),
  }


class TestCurveSpeed(unittest.TestCase):
  def test_straight_road_shows_nothing(self):
    assert CurveSpeedElement().update(sm_for(), False).value == "-"

  def test_tight_curve_ahead(self):
    # 25 m/s (56 mph) into a curve with 3 m/s^2 predicted lateral accel -> sqrt(2/3)*25 = 20.4 m/s (46 mph)
    v = 25.0
    rate = [0.0] * 20 + [3.0 / v] * 13
    el = CurveSpeedElement().update(sm_for(v, rate), False)
    assert el.value == "46" and el.unit == "mph"
    assert el.color.r == 255 and el.color.g == 0  # more than 5 mph over

  def test_gentle_curve_below_threshold(self):
    v = 25.0
    assert CurveSpeedElement().update(sm_for(v, [1.0 / v] * 33), False).value == "-"

  def test_slow_or_bad_input(self):
    assert CurveSpeedElement.curve_speed([0.5] * 5, [3.0] * 5, 3.0) is None
    assert CurveSpeedElement.curve_speed([], [], 20.0) is None
    assert CurveSpeedElement.curve_speed([0.1] * 3, [20.0] * 2, 20.0) is None

  def test_metric(self):
    v = 25.0
    el = CurveSpeedElement().update(sm_for(v, [3.0 / v] * 33), True)
    assert el.unit == "km/h" and el.value == "73"


class TestSteeringTorquePercent(unittest.TestCase):
  def test_percent_and_colours(self):
    e = SteeringTorquePercentElement()
    assert e.update(sm_for(torque=-0.42), False).value == "42"
    assert e.update(sm_for(torque=0.42), False).color.g == 255
    assert e.update(sm_for(torque=0.7), False).color.g == 188
    assert e.update(sm_for(torque=0.95), False).color.g == 0

  def test_inactive(self):
    assert SteeringTorquePercentElement().update(sm_for(torque=0.5, lat_active=False), False).value == "-"

  def test_blank_while_driver_overrides(self):
    assert SteeringTorquePercentElement().update(sm_for(torque=1.0, steering_pressed=True), False).value == "-"
