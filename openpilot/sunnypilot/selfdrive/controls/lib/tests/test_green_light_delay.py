"""Green light chime delay: after the light turns green, the chime waits GreenLightAlertDelay and is
skipped if the driver lifts off the brake (or drives away) in that time."""
from types import SimpleNamespace as NS

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import e2e_alerts_helper as h


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    return self.values.get(key)


class FakeEvents:
  def __init__(self):
    self.chimes = 0

  def add(self, name):
    self.chimes += 1


def sm_for(go: bool, brake: bool, moving: bool = False):
  return {
    'carState': NS(standstill=not moving, vEgo=2.0 if moving else 0.0, gasPressed=False, brakePressed=brake),
    'carControl': NS(enabled=False),
    'modelV2': NS(position=NS(x=[0.0, 50.0 if go else 5.0])),
    'radarState': NS(leadOne=NS(present=False, dRel=0.0)),
  }


def make_helper(monkeypatch, delay_index):
  monkeypatch.setattr(h, "Params", lambda: FakeParams({"GreenLightAlert": True, "GreenLightAlertDelay": delay_index}))
  return h.E2EAlertsHelper()


def run(helper, events, seconds, **sm_kwargs):
  """Step the helper; return the time (s from the start of this call) of the first chime, or None."""
  first = None
  start = events.chimes
  for i in range(round(seconds / DT_MDL)):
    helper.update(sm_for(**sm_kwargs), events)
    if first is None and events.chimes > start:
      first = i * DT_MDL
  return first


def stopped_at_red(helper, events, brake=True):
  # drive up to the light, then wait long enough that "recently moving" clears and the alert arms
  assert run(helper, events, 1.0, go=False, brake=False, moving=True) is None
  assert run(helper, events, 3.0, go=False, brake=brake) is None


@pytest.mark.parametrize("delay_index", range(len(h.GREEN_LIGHT_DELAYS)))
def test_chimes_after_the_delay_when_driver_stays_on_the_brake(monkeypatch, delay_index):
  helper, events = make_helper(monkeypatch, delay_index), FakeEvents()
  stopped_at_red(helper, events)
  t = run(helper, events, 3.0, go=True, brake=True)
  expected = h.TRIGGER_TIMER_THRESHOLD + h.GREEN_LIGHT_DELAYS[delay_index]
  assert t is not None and expected - 0.11 <= t <= expected + 0.11


def test_no_chime_when_driver_lifts_off_the_brake_in_time(monkeypatch):
  helper, events = make_helper(monkeypatch, 1), FakeEvents()  # 0.5 s
  stopped_at_red(helper, events)
  assert run(helper, events, 0.5, go=True, brake=True) is None  # light just went green, still on the brake
  assert run(helper, events, 3.0, go=True, brake=False) is None  # foot off the brake within the delay


def test_brake_hold_still_chimes_after_the_delay(monkeypatch):
  # Brake Hold: the foot is already off the pedal, so there is no lift to see; chime after the delay
  helper, events = make_helper(monkeypatch, 1), FakeEvents()
  stopped_at_red(helper, events, brake=False)
  t = run(helper, events, 3.0, go=True, brake=False)
  assert t is not None and t == pytest.approx(h.TRIGGER_TIMER_THRESHOLD + 0.5, abs=0.11)


def test_off_keeps_the_original_timing(monkeypatch):
  helper, events = make_helper(monkeypatch, 0), FakeEvents()
  stopped_at_red(helper, events)
  t = run(helper, events, 3.0, go=True, brake=True)
  assert t == pytest.approx(h.TRIGGER_TIMER_THRESHOLD, abs=0.11)
