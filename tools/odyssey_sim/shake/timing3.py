"""Native sensor/CAN timing extraction for pass 3; no device or runtime writes.

Sensor-to-device mapping and calibration transpose follow locationd.py/helpers.py.
Caches retain source, sensor time, publication time and segment; no blended IMUs.
"""

import json
from pathlib import Path
import numpy as np
from scipy import signal
from opendbc.can import CANParser
from openpilot.tools.lib.logreader import LogReader
from openpilot.common.transformations.orientation import rot_from_euler
from tools.odyssey_sim.shake.extract import SCRATCH

ROOT = Path.home() / 'comma-logs'


def vehicle_vectors(raw, rpy):
  device = -np.asarray(raw)[..., ::-1]
  return np.einsum('...ji,...j->...i', rot_from_euler(rpy), device)


def lag_seconds(x, y, fs=100, limit=0.25):
  """Positive lag means y follows x; normalized overlap, band limited upstream."""
  if np.std(x) < 1e-12 or np.std(y) < 1e-12:
    return 0., 0.
  n = int(limit * fs)
  lags = np.arange(-n, n + 1)
  c = []
  for k in lags:
    a, b = (x[:-k], y[k:]) if k > 0 else (x[-k:], y[:k]) if k < 0 else (x, y)
    c.append(np.corrcoef(a, b)[0, 1])
  j = int(np.nanargmax(c))
  return float(lags[j] / fs), float(c[j])


def extract3(name):
  path = SCRATCH / f'{name}_native3.npz'
  if path.exists():
    return dict(np.load(path))
  (route,) = ROOT.glob(f'04b*_{name}--*')
  rows = {k: [] for k in ('gyro', 'accel', 'calib', 'angle', 'rate', 'motor', 'command', 'pose')}
  sources = {}
  for f in sorted(route.glob('*/rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    seg = int(f.parent.name.rsplit('--', 1)[1])
    parser = CANParser(
      'honda_odyssey_exl_2018_generated', [('STEERING_SENSORS', 0), ('STEER_STATUS', 0), ('STEER_MOTOR_TORQUE', 0), ('STEERING_CONTROL', 0)], 0
    )
    for m in LogReader(str(f)):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w in ('gyroscope', 'accelerometer'):
        s = getattr(m, w)
        if s.which() not in ('gyroUncalibrated', 'acceleration'):
          continue
        source = str(s.source)
        if source not in sources:
          sources[source] = len(sources)
        rows['gyro' if w == 'gyroscope' else 'accel'].append([s.timestamp * 1e-9, t, sources[source], seg, *getattr(s, s.which()).v])
      elif w in ('liveCalibration', 'extrinsicsCalibration'):
        c = getattr(m, w)
        if len(c.rpyCalib) == 3:
          rows['calib'].append([t, seg, int(c.calStatus.raw), *c.rpyCalib])
      elif w == 'carControl' and len(m.carControl.angularVelocity) == 3:
        rows['pose'].append([t, seg, *m.carControl.angularVelocity])
      elif w in ('can', 'sendcan'):
        updated = parser.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in getattr(m, w)])])
        for addr, key, msg, field in (
          (0x156, 'angle', 'STEERING_SENSORS', 'STEER_ANGLE'),
          (0x18F, 'rate', 'STEER_STATUS', 'STEER_ANGLE_RATE'),
          (0x1AB, 'motor', 'STEER_MOTOR_TORQUE', 'MOTOR_TORQUE'),
          (0xE4, 'command', 'STEERING_CONTROL', 'STEER_TORQUE'),
        ):
          if addr in updated:
            rows[key].append([t, seg, parser.vl[msg][field]])
    print(name, seg, {k: len(v) for k, v in rows.items()}, flush=True)
  result = {k: np.array(v) for k, v in rows.items()}
  result['sources_json'] = np.array(json.dumps(sources))
  np.savez_compressed(path, **result)
  return result


def band(x, lo=0.3, hi=3):
  return signal.sosfiltfilt(signal.butter(3, [lo, hi], fs=100, btype='bandpass', output='sos'), x, axis=0)


if __name__ == '__main__':
  import sys

  for name in sys.argv[1:]:
    extract3(name)
