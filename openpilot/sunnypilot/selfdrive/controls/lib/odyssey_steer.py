"""Odyssey steering add-ons, applied in controlsd to the torque command after the lateral controller.

sunnypilot picks the torque controller class at runtime (latcontrol_torque_v0.LatControlTorque unless
EnforceTorqueControl is set, else openpilot's latcontrol_torque.LatControlTorque), and NNLC plugs into either.
Applying our add-ons here, to the controller's output in actuators.torque sign, makes them work whichever runs:
  - low-speed angle steering and its gain A/B (SteerLowSpeedAngle / SteerLowSpeedAngleAB, low_speed_angle.py)
  - the deadband kick / bump and their A/B (SteerDeadbandComp / SteerDeadbandCompAB, steer_deadband.py)
  - wander friction assistance and its A/B (OdysseyWanderFriction / OdysseyWanderAB, odyssey_wander.py)
"""
import time

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import WANDER_ARMS, WANDER_BLOCK_S, ab_arm, clock_enabled
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_wander import FrictionAssist
from openpilot.sunnypilot.selfdrive.controls.lib.low_speed_angle import LowSpeedAngleSteer
from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import SteerDeadbandComp


class OdysseySteer:
  def __init__(self, params=None):
    params = params or Params()
    self.low_speed_angle = LowSpeedAngleSteer(params)
    self.deadband_comp = SteerDeadbandComp()
    self.friction = FrictionAssist('inverse', scale=1.0)
    self.friction_on = bool(params.get_bool("OdysseyWanderFriction"))
    self.friction_ab = bool(params.get_bool("OdysseyWanderAB"))
    self.friction_arm = "off"

  def friction_enabled(self, mono_s: float) -> bool:
    if clock_enabled("wander", self.friction_ab):
      arm = ab_arm(mono_s, WANDER_ARMS, WANDER_BLOCK_S)
      if arm != self.friction_arm:
        self.friction.reset()  # each block starts from the same state
      self.friction_arm = arm
      return arm == "inverse"
    return self.friction_on

  def apply(self, steer: float, lat_active: bool, CS, VM, live_params, desired_curvature: float, steer_max: float,
            desired_lat_accel: float = float("nan"), actual_lat_accel: float = float("nan"), mono_s: float | None = None) -> float:
    if not lat_active:
      self.friction.reset()
      return steer
    mono_s = time.monotonic() if mono_s is None else mono_s
    steer = self.low_speed_angle.blend(steer, CS.vEgo, CS.steeringAngleDeg, VM, live_params, desired_curvature, mono_s)
    steer = self.deadband_comp.update(steer, steer_max)
    enabled = self.friction_enabled(mono_s)
    valid = not (CS.steeringPressed or CS.leftBlinker or CS.rightBlinker)
    # FrictionAssist is acceleration-positive; actuators.torque is the opposite sign
    steer = -self.friction.update(-steer, target=desired_lat_accel, actual=actual_lat_accel, speed=CS.vEgo,
                                  rate=CS.steeringRateDeg, dt=0.01, enabled=enabled, valid=valid)
    return max(-steer_max, min(steer_max, steer))
