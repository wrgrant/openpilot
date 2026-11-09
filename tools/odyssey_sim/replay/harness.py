"""Real controller execution, with recorded vehicle motion held fixed."""
import os
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

# Set before Params constructs any handle; never use the device/default namespace.
os.environ['OPENPILOT_PREFIX'] = 'simreplay'

from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


@dataclass(frozen=True)
class ControllerConfig:
  nnlc: bool = True
  delay: float | None = None
  version: int = 0

  @property
  def name(self):
    return f'{"nnlc" if self.nnlc else "stock"}_v{self.version}_{self.delay or "logged"}'


def route_files(route):
  p = Path(route).expanduser()
  if p.is_file():
    return [p]
  if not p.is_dir():
    matches = list((Path.home() / 'comma-logs').glob(f'*{route}*'))
    if len(matches) != 1:
      raise ValueError(f'Expected one local route for {route}, found {matches}')
    p = matches[0]
  files = sorted(p.glob('*/rlog.zst'), key=lambda f: int(f.parent.name.rsplit('--', 1)[1]))
  if not files:
    raise FileNotFoundError(p)
  return files


def metadata(route):
  result = {}
  for m in LogReader(str(route_files(route)[0])):
    k = m.which()
    if k in ('carParams', 'carParamsSP', 'initData'):
      result[k] = getattr(m, k).as_builder()
    if len(result) == 3:
      break
  result['params'] = {e.key: bytes(e.value) for e in result['initData'].params.entries}
  return result


def replay_configs(route, configs, max_frames=None):
  """Yield dict rows (one per control frame), with commands keyed by config.name.

  Replays all frames to retain state; callers filter active and !steeringPressed.
  Config delay overrides both core lat_delay and extension plan lookahead.
  Controllers share recorded CS, plan, calibration and safety-limit history.
  """
  meta = metadata(route)
  cp, sp = meta['carParams'], meta['carParamsSP']
  sp.neuralNetworkLateralControl.model.path = str(Path(__file__).resolve().parents[3] /
    'openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json')
  p = Params()
  for key in ('LateralJerkTorqueController', 'NnlcAB', 'EnforceTorqueControl', 'TorqueParamsOverrideEnabled',
              'SteerLowSpeedAngle', 'SteerDeadbandCompAB'):
    p.put_bool(key, False, block=True)
  p.put('SteerDeadbandComp', 0.0, block=True)
  p.put('SteerLowSpeedKpCap', 0.0, block=True)
  cp = cp.as_reader()
  sp = sp.as_reader()
  ci = interfaces[cp.carFingerprint](cp, sp)
  vm = VehicleModel(cp)
  controllers = []
  for cfg in configs:
    p.put_bool('NeuralNetworkLateralControl', cfg.nnlc, block=True)
    controller = (LatControlTorqueV0 if cfg.version == 0 else LatControlTorque)(cp, sp, ci, .01)
    assert controller.extension.enabled == cfg.nnlc, "Params toggle did not reach controller"
    controllers.append((cfg, controller))
  latest = {}
  limited = False
  frames = 0
  sampled = None
  relevant = {'carState', 'carControl', 'carOutput', 'vehicleParameters', 'liveParameters',
              'lateralTorqueParameters', 'liveTorqueParameters', 'lateralDelay', 'modelV2', 'controlsState', 'selfdriveState'}
  for file in route_files(route):
    events = sorted((m for m in LogReader(str(file)) if m.which() in relevant), key=lambda m: m.logMonoTime)
    for m in events:
      k = m.which()
      if k not in relevant:
        continue
      latest[k] = getattr(m, k)
      if k == 'selfdriveState':
        sampled = latest.copy()
      if k != 'carControl' or not all(x in latest for x in ('controlsState', 'carState', 'modelV2')):
        continue
      inputs = sampled or latest
      lp = inputs.get('vehicleParameters', inputs.get('liveParameters'))
      if lp is None:
        continue
      cs, cc, state = inputs['carState'], latest['carControl'], latest['controlsState']
      if state.lateralControlState.which() != 'torqueState':
        continue
      logged = state.lateralControlState.torqueState
      vm.update_params(max(lp.stiffnessFactor, .1), max(lp.steerRatio, .1))
      tp = inputs.get('lateralTorqueParameters', inputs.get('liveTorqueParameters'))
      delay = inputs['lateralDelay'].lateralDelay if 'lateralDelay' in inputs else .32
      ext_delay = delay if meta['params'].get('LagdToggle') == b'1' else float(meta['params'].get('LagdValueCache', b'.32'))
      pose = SimpleNamespace(orientation=SimpleNamespace(pitch=cc.orientationNED[1])) if len(cc.orientationNED) == 3 else None
      row = {'t': m.logMonoTime * 1e-9, 'speed': cs.vEgo, 'active': cc.latActive, 'pressed': cs.steeringPressed,
                 'desired': cc.actuators.curvature * cs.vEgo**2, 'actual': logged.actualLateralAccel,
                 'logged': logged.output, 'logged_error': logged.error, 'logged_p': logged.p, 'logged_i': logged.i, 'logged_f': logged.f,
                 'version': logged.version, 'delay': delay, 'factor': tp.latAccelFactorFiltered if tp else 0,
                 'friction': tp.frictionCoefficientFiltered if tp else 0, 'offset': tp.latAccelOffsetFiltered if tp else 0}
      for cfg, controller in controllers:
        if tp is not None and tp.useParams:
          controller.update_torque_parameters(tp.latAccelFactorFiltered, tp.latAccelOffsetFiltered, tp.frictionCoefficientFiltered)
        controller.extension.update_limits()
        controller.extension.update_model_v2(inputs['modelV2'])
        controller.extension.update_lateral_lag(ext_delay if cfg.delay is None else cfg.delay)
        if not cc.latActive:
          controller.reset()
        command, _, plog = controller.update(cc.latActive, cs, vm, lp, limited, cc.actuators.curvature,
                                             pose, False, delay if cfg.delay is None else cfg.delay)
        row[cfg.name] = float(command)
        row[cfg.name + '_error'] = plog.error
        row[cfg.name + '_p'] = plog.p
        row[cfg.name + '_i'] = plog.i
        row[cfg.name + '_f'] = plog.f
      if cc.latActive and 'carOutput' in latest:
        limited = abs(cc.actuators.torque - inputs['carOutput'].actuatorsOutput.torque) > .01
      yield row
      frames += 1
      if max_frames is not None and frames >= max_frames:
        return


def replay_route(route, config=None, max_frames=None):
  """Yield per-frame dictionaries with `command` in logged torqueState.output sign."""
  config = config or ControllerConfig()
  for row in replay_configs(route, [config], max_frames):
    row['command'] = row[config.name]
    yield row
