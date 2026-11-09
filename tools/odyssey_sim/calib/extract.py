"""Supplement immutable EPS caches with timestamped calibrated body motion and roll."""
from pathlib import Path
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from tools.odyssey_sim.plant.extract import SCRATCH


def extract(route):
  dest = SCRATCH / 'calib' / f'{route.name.split("_")[1].split("--")[0]}_body.npz'
  if dest.exists() and 'tuning' in np.load(dest).files:
    return
  cc, vp, torque, tuning = [], [], [], []
  delay, factor, friction, offset = .32, 1.47, .145, -.055
  for path in sorted(route.glob('*/rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    for m in LogReader(str(path)):
      k = m.which()
      t = m.logMonoTime / 1e9
      if k == 'lateralDelay':
        delay = m.lateralDelay.lateralDelay
      elif k == 'lateralTorqueParameters' and m.lateralTorqueParameters.useParams:
        q = m.lateralTorqueParameters
        factor, friction, offset = q.latAccelFactorFiltered, q.frictionCoefficientFiltered, q.latAccelOffsetFiltered
      elif k == 'carControl':
        c = m.carControl
        w = list(c.angularVelocity)
        o = list(c.orientationNED)
        tuning.append([t, delay, factor, friction, offset])
        cc.append([t, * (w if len(w) == 3 else [np.nan]*3), *(o if len(o) == 3 else [np.nan]*3), c.actuators.torque, c.latActive])
      elif k == 'vehicleParameters':
        p = m.vehicleParameters
        vp.append([t, p.roll, p.angleOffsetDeg, p.steerRatio, p.stiffnessFactor])
      elif k == 'controlsState' and m.controlsState.lateralControlState.which() == 'torqueState':
        s = m.controlsState.lateralControlState.torqueState
        torque.append([t, s.desiredLateralAccel, s.actualLateralAccel, s.output, s.saturated])
    print(route.name, path.parent.name, len(cc), flush=True)
  np.savez_compressed(dest, cc=np.array(cc), vp=np.array(vp), torque=np.array(torque), tuning=np.array(tuning))


def main():
  (SCRATCH/'calib').mkdir(exist_ok=True)
  for route in sorted(Path.home().joinpath('comma-logs').glob('04b*')):
    extract(route)


if __name__ == '__main__':
  main()
