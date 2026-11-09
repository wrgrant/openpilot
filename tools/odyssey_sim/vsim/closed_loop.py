"""Native applied-command timeline and scratch-only real-controller adapter."""
import importlib
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib.identify import S, align
from tools.odyssey_sim.vsim.geometry import RelativePose
from tools.odyssey_sim.vsim.perturbation import PerturbationPlant


def native_data(key):
  root = S if (S / 'plant_data' / f'{key}.npz').exists() else S / 'vsim3' / 'cache'
  d = dict(np.load(root / 'plant_data' / f'{key}.npz'))
  b = np.load(root / 'calib' / f'{key}_body.npz')
  for name, col in [('roll', 1), ('angle_offset', 2), ('sr', 3), ('stiffness', 4)]:
    d[name] = align(d['t'], b['vp'], col, .5)
  for name, col in [('yaw', 3), ('pitch', 5)]:
    d[name] = align(d['t'], b['cc'], col, .15)
  for name, col in [('delay', 1), ('factor', 2), ('friction', 3), ('torque_offset', 4)]:
    d[name] = align(d['t'], b['tuning'], col, .15)
  d['body'] = d['v'] * d['yaw']
  controls = np.load(root / 'calib3' / f'{key}.npz')['native']
  d['a_ego'] = align(d['t'], controls, 2, .04)
  return d


def recorded(meta, cfg):
  import math
  from tools.odyssey_sim.calib.controller import Controller
  from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS

  class NativeController(Controller):
    def update(self, target_lataccel, current_lataccel, state, future_plan=None):
      v = max(state.v_ego, .1)
      lp = NS(roll=state.roll, angleOffsetDeg=state.offset)
      self.vm.update_params(max(state.stiffness,.1), max(state.ratio,.1))
      angle = math.degrees(self.vm.get_steer_from_curvature(-current_lataccel/v**2,v,lp.roll))+lp.angleOffsetDeg
      rate = 0. if self.last_angle is None else (angle-self.last_angle)/.01
      self.last_angle = angle
      cs = NS(vEgo=v,aEgo=state.a_ego,steeringAngleDeg=round(angle*10)/10,steeringRateDeg=rate,
              steeringPressed=False,steeringRateLimited=False,leftBlinker=False,rightBlinker=False)
      self.measured_lataccel = -self.vm.calc_curvature(math.radians(cs.steeringAngleDeg-lp.angleOffsetDeg),v,lp.roll)*v**2
      ext = self.lac.extension
      ext.update_model_v2(future_plan)
      ext.update_lateral_lag(state.delay)
      self.lac.update_torque_parameters(state.factor,state.torque_offset,state.friction*self.cfg.friction)
      ext.update_limits()
      pose = NS(orientation=NS(pitch=state.pitch))
      torque,_,_ = self.lac.update(True,cs,self.vm,lp,state.limited,target_lataccel/v**2,
                                  pose,state.curvature_limited,state.delay+LAT_SMOOTH_SECONDS)
      return -float(self.odyssey.apply(float(torque),True,cs,self.vm,lp,target_lataccel/v**2,1.0))

  return NativeController(meta,cfg)


class ClosedLoop:
  def __init__(self, route, factory, scratch, sensitivity=None, free_after=0.):
    # Install before importing any controller modules: every default Params
    # constructor in the adapter and production extensions stays under scratch.
    import openpilot.common.params as params
    original = params.Params
    root = str(Path(scratch).resolve() / 'params')

    class ScratchParams(original):
      def __init__(self, d=''):
        super().__init__(root)

    params.Params = ScratchParams
    from tools.odyssey_sim.replay.harness import metadata
    from tools.odyssey_sim.closedloop.controller import Config
    self.meta = metadata(str(route))
    self.identity = {'params': {k: v.decode(errors='replace') for k, v in self.meta['params'].items()
                               if k in ('NeuralNetworkLateralControl', 'EnforceTorqueControl', 'LagdToggle', 'LagdValueCache')},
                     'carParamsSP': self.meta['carParamsSP'].to_dict()}
    cfg = Config(nnlc=self.meta['params'].get('NeuralNetworkLateralControl') == b'1')
    module, name = factory.split(':')
    self.controller = getattr(importlib.import_module(module), name)(self.meta, cfg)
    import os
    self.identity['simulation_arm'] = os.environ.get('VSIM_ARM', 'recorded')
    self.d = native_data(route.name.split('_')[1].split('--')[0])
    self.plant = PerturbationPlant(sensitivity=sensitivity)
    self.pose = RelativePose()
    self.index = None
    self.current = self.body = self.command = self.clipped = 0.
    self.history = []
    self.free_after = free_after
    self.plan = None
    self.target = None

  def advance(self, timestamp):
    from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
    d = self.d
    end = np.searchsorted(d['t'], timestamp, side='right')
    if self.index is None:
      self.index = max(0, end - 1)
      i = self.index
      self.current, self.body, self.command = d['act'][i], d['body'][i], d['cmd'][i]
      self.plant.reset(self.current, self.body, self.command)
      self.clipped = d['des'][i] / max(d['v'][i]**2, 1)
    while self.index < end - 1:
      i = self.index
      dt = d['t'][i+1] - d['t'][i]
      if not .005 <= dt <= .025:
        raise ValueError(f'native command gap {dt}s')
      required = ['v', 'roll', 'angle_offset', 'sr', 'stiffness', 'yaw', 'body', 'act', 'pitch', 'delay', 'factor', 'friction', 'torque_offset']
      if not np.isfinite([d[k][i] for k in required]).all():
        raise ValueError('nonfinite native context')
      if d['t'][i] >= self.free_after and (not d['active'][i] or d['pressed'][i] or abs(d['drv'][i]) >= 500 or d['blinker'][i]):
        raise ValueError(f'window left autonomous eligibility at {d["t"][i]}')
      state = NS(v_ego=d['v'][i], a_ego=d['a_ego'][i], roll=d['roll'][i], offset=d['angle_offset'][i],
                 ratio=d['sr'][i], stiffness=d['stiffness'][i], pitch=d['pitch'][i], delay=d['delay'][i],
                 factor=d['factor'][i], friction=d['friction'][i], torque_offset=d['torque_offset'][i],
                 limited=abs(self.command - self.plant.sim.u) > .01)
      state = NS(**{k: (bool(v) if k == 'limited' else float(v)) for k, v in vars(state).items()})
      target = self.target if self.target is not None else self.clipped
      self.clipped, curvature_limited = clip_curvature(state.v_ego, self.clipped, target, state.roll)
      state.curvature_limited = bool(curvature_limited)
      self.command = self.controller.update(self.clipped * state.v_ego**2, self.current, state, self.plan)
      if d['t'][i] < self.free_after:
        self.current, self.body = d['act'][i], d['body'][i]
        self.plant.reset(self.current, self.body, d['cmd'][i])
        self.index += 1
        continue
      self.current, self.body = self.plant.step(self.command, d['cmd'][i], state.v_ego, state.roll,
                                                d['act'][i], d['body'][i], dt)
      self.pose.step(state.v_ego, self.body / max(state.v_ego, .1), d['yaw'][i], dt)
      self.history.append({'t': float(d['t'][i]), 'dy': float(self.pose.dy), 'dpsi': float(self.pose.dpsi),
                           'body': float(self.body), 'command': float(self.command)})
      if abs(self.pose.dy) > 1 or abs(self.pose.dpsi) > .1:
        raise ValueError('pose exceeded validated road-plane domain (1m / 0.1rad)')
      self.index += 1
