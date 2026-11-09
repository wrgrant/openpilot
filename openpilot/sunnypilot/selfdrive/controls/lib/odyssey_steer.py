"""Odyssey steering add-ons, applied in controlsd to the torque command after the lateral controller.

sunnypilot picks the torque controller class at runtime (latcontrol_torque_v0.LatControlTorque unless
EnforceTorqueControl is set, else openpilot's latcontrol_torque.LatControlTorque), and NNLC plugs into either.
Applying our add-ons here, to the controller's output in actuators.torque sign, makes them work whichever runs:
  - low-speed angle steering (SteerLowSpeedAngle, low_speed_angle.py)
  - the deadband kick / bump and their A/B (SteerDeadbandComp / SteerDeadbandCompAB, steer_deadband.py)
"""
from openpilot.sunnypilot.selfdrive.controls.lib.low_speed_angle import LowSpeedAngleSteer
from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import SteerDeadbandComp


class OdysseySteer:
  def __init__(self, params=None):
    self.low_speed_angle = LowSpeedAngleSteer(params)
    self.deadband_comp = SteerDeadbandComp()

  def apply(self, steer: float, lat_active: bool, CS, VM, live_params, desired_curvature: float, steer_max: float) -> float:
    if not lat_active:
      return steer
    steer = self.low_speed_angle.blend(steer, CS.vEgo, CS.steeringAngleDeg, VM, live_params, desired_curvature)
    return self.deadband_comp.update(steer, steer_max)
