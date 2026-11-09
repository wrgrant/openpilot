"""Challenge-style controller interface backed by the unmodified production LaC."""
import os
import math
from dataclasses import dataclass
from types import SimpleNamespace as NS
from pathlib import Path
from tools.odyssey_sim.replay import harness as harness  # Import before restoring our isolated Params namespace.
os.environ['OPENPILOT_PREFIX'] = 'simclosed'
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_steer import OdysseySteer
from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque
import openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 as core
import openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc as nn


@dataclass(frozen=True)
class Config:
  nnlc: bool = False
  delay: float = .32
  angle: bool = False
  friction: float = 1.
  threshold: float = .3
  kick: float = 0.

  @property
  def name(self):
    return f'{"nnlc" if self.nnlc else "stock"}_d{self.delay}_angle{int(self.angle)}_f{self.friction}_th{self.threshold}_kick{self.kick}'


def configs():
  for nnlc in (False,True):
    for delay in (.32,.45):
      yield Config(nnlc=nnlc,delay=delay)
    for kw in ({'angle':True},{'friction':.5},{'threshold':.6},{'kick':.03}, {'angle':True,'friction':.5,'kick':.03}):
      yield Config(nnlc=nnlc,**kw)


class Controller:
  def __init__(self, meta, cfg):
    self.cfg=cfg
    p=Params()
    for key in ('LateralJerkTorqueController','NnlcAB','EnforceTorqueControl','TorqueParamsOverrideEnabled','SteerDeadbandCompAB'):
      p.put_bool(key,False,block=True)
    p.put_bool('NeuralNetworkLateralControl',cfg.nnlc,block=True)
    p.put_bool('SteerLowSpeedAngle',cfg.angle,block=True)
    p.put('SteerDeadbandComp',cfg.kick,block=True)
    p.put('SteerLowSpeedKpCap',0.,block=True)
    cp,sp=meta['carParams'].as_reader(),meta['carParamsSP']
    sp.neuralNetworkLateralControl.model.path=str(Path.cwd()/'openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json')
    self.vm=VehicleModel(cp)
    ci=interfaces[cp.carFingerprint](cp,sp.as_reader())
    self.lac=LatControlTorque(cp,sp.as_reader(),ci,.01)
    self.lac.update_torque_parameters(1.47,-.055,.145*cfg.friction)
    core.FRICTION_THRESHOLD=cfg.threshold
    nn.FRICTION_THRESHOLD=.2 if cfg.threshold==.3 else cfg.threshold
    assert self.lac.extension.enabled == cfg.nnlc
    self.odyssey=OdysseySteer()
    self.low_speed_angle=self.odyssey.low_speed_angle
    self.deadband_comp=self.odyssey.deadband_comp
    assert self.low_speed_angle.enabled == cfg.angle
    assert abs(float(p.get('SteerDeadbandComp'))-cfg.kick)<1e-6
    self.last_angle=None
    self.nn_active=0

  def update(self, target_lataccel, current_lataccel, state, future_plan=None):
    """state includes logged live params and modelV2; torque returned in accel-positive sign."""
    v=max(float(state.v_ego),.1)
    lp=NS(roll=float(state.roll),angleOffsetDeg=float(state.offset))
    self.vm.update_params(max(float(state.stiffness),.1),max(float(state.ratio),.1))
    angle=math.degrees(self.vm.get_steer_from_curvature(-current_lataccel/v**2,v,lp.roll))+lp.angleOffsetDeg
    rate=0. if self.last_angle is None else (angle-self.last_angle)/.01
    self.last_angle=angle
    cs=NS(vEgo=v,aEgo=float(state.a_ego),steeringAngleDeg=angle,steeringRateDeg=rate,steeringPressed=False,steeringRateLimited=False)
    ext=self.lac.extension
    ext.update_model_v2(future_plan)
    ext.update_lateral_lag(self.cfg.delay)
    ext.update_limits()
    self.nn_active+=int(ext._nnlc_enabled)
    torque,_,log=self.lac.update(True,cs,self.vm,lp,False,target_lataccel/v**2,None,False,self.cfg.delay)
    # Post-processing consumes actuators.torque (opposite internal LaC sign).
    torque=self.odyssey.apply(float(torque),True,cs,self.vm,lp,target_lataccel/v**2,1.0)
    return -float(torque)


def model_from_array(a):
  return NS(acceleration=NS(y=a[0]),orientation=NS(x=a[1],y=a[2]))
