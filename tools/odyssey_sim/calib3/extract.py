"""Preserve model time series and native controls for independent closure diagnostics."""

from pathlib import Path
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from tools.odyssey_sim.calib.identify import S, TRAIN, TEST
from tools.odyssey_sim.calib3.geometry import origin, VARIANTS

OUT = S / 'calib3'


def extract(key):
  dest = OUT / f'{key}.npz'
  if dest.exists():
    return
  (route,) = Path.home().joinpath('comma-logs').glob(f'*_{key}--*')
  data = {
    k: []
    for k in (
      't',
      'geometry',
      'edges',
      'position',
      'orientation',
      'velocity',
      'orientationRate',
      'acceleration',
      'probs',
      'stds',
      'action',
      'age',
      'calibration',
      'native',
    )
  }
  cs = cc = vp = None
  delay, factor, friction, offset = 0.32, 1.47, 0.145, -0.055
  calib = [np.nan] * 3
  for path in sorted(route.glob('*/rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    for m in LogReader(str(path)):
      k, t = m.which(), m.logMonoTime / 1e9
      if k == 'carState':
        cs = m.carState
      elif k == 'carControl':
        cc = m.carControl
      elif k == 'vehicleParameters':
        vp = m.vehicleParameters
      elif k == 'lateralDelay':
        delay = m.lateralDelay.lateralDelay
      elif k == 'extrinsicsCalibration':
        calib = list(m.extrinsicsCalibration.rpyCalib)
      elif k == 'lateralTorqueParameters' and m.lateralTorqueParameters.useParams:
        p = m.lateralTorqueParameters
        factor, friction, offset = p.latAccelFactorFiltered, p.frictionCoefficientFiltered, p.latAccelOffsetFiltered
      elif k == 'controlsState' and cs is not None and cc is not None and vp is not None and m.controlsState.lateralControlState.which() == 'torqueState':
        q = m.controlsState.lateralControlState.torqueState
        data['native'].append(
          [
            t,
            cs.vEgo,
            cs.aEgo,
            cs.steeringAngleDeg,
            cs.steeringRateDeg,
            cs.steeringPressed,
            cs.leftBlinker or cs.rightBlinker,
            cc.latActive,
            q.desiredLateralAccel,
            q.actualLateralAccel,
            q.output,
            q.p,
            q.i,
            q.f,
            q.error,
            delay,
            factor,
            friction,
            offset,
            vp.roll,
            vp.angleOffsetDeg,
            vp.steerRatio,
            vp.stiffnessFactor,
            cc.orientationNED[1] if len(cc.orientationNED) == 3 else 0,
          ]
        )
      elif k == 'modelV2' and len(m.modelV2.laneLines) == 4:
        q = m.modelV2
        l, r = q.laneLines[1], q.laneLines[2]
        x, y = np.array(l.x), (np.array(l.y) + np.array(r.y)) / 2
        data['t'].append(t)
        data['geometry'].append([origin(x, y, d, h) for d, h in VARIANTS])
        data['edges'].append([origin(z.x, z.y) for z in q.roadEdges])
        for field in ('position', 'orientation', 'velocity', 'orientationRate', 'acceleration'):
          z = getattr(q, field)
          data[field].append([list(z.x), list(z.y), list(z.z), list(z.t)])
        data['probs'].append(list(q.laneLineProbs))
        data['stds'].append(list(q.laneLineStds))
        data['action'].append(q.action.desiredCurvature)
        data['age'].append([q.frameAge, (m.logMonoTime - q.timestampEof) / 1e9])
        data['calibration'].append(calib)
    print(key, path.parent.name, len(data['t']), flush=True)
  np.savez_compressed(dest, **{k: np.array(v) for k, v in data.items()})


def main():
  OUT.mkdir(exist_ok=True)
  for key in TRAIN + TEST:
    extract(key)


if __name__ == '__main__':
  main()
