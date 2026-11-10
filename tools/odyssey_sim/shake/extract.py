"""Supplement existing 100 Hz tables; all new caches stay in shake scratch."""
from pathlib import Path
import numpy as np
from opendbc.can import CANParser
from openpilot.tools.lib.logreader import LogReader

SCRATCH = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/shake')
DATA = Path.home() / 'comma-logs/odyssey-evidence/drive_1109'


def extract(name):
  out = SCRATCH / f'{name}.npz'
  if out.exists():
    return
  d = dict(np.load(DATA / f'{name}.npz'))
  series = {k: [] for k in ('motor', 'can_cmd', 'gyro', 'accel', 'orientation', 'vehicle', 'tuning', 'delay', 'plans')}
  route, = (Path.home() / 'comma-logs').glob(f'04b*_{name}--*')
  for f in sorted(route.glob('*/rlog.zst'), key=lambda p: int(p.parent.name.rsplit('--', 1)[1])):
    parser = CANParser('honda_odyssey_exl_2018_generated', [('STEER_MOTOR_TORQUE', 0), ('STEERING_CONTROL', 0)], 0)
    for m in LogReader(str(f)):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w in ('can', 'sendcan'):
        packets = getattr(m, w)
        updated = parser.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in packets])])
        for address, key, message, field in ((0x1ab, 'motor', 'STEER_MOTOR_TORQUE', 'MOTOR_TORQUE'),
                                             (0xe4, 'can_cmd', 'STEERING_CONTROL', 'STEER_TORQUE')):
          if address in updated:
            series[key].append([t, parser.vl[message][field]])
      elif w in ('accelerometer', 'gyroscope'):
        event = getattr(m, w)
        values = getattr(event, event.which()).v
        if len(values) == 3:
          series['accel' if w == 'accelerometer' else 'gyro'].append([t, *values])
      elif w == 'carControl' and len(m.carControl.orientationNED) == 3:
        series['orientation'].append([t, *m.carControl.orientationNED])
      elif w == 'vehicleParameters':
        p = m.vehicleParameters
        series['vehicle'].append([t, p.roll, p.angleOffsetDeg, p.steerRatio, p.stiffnessFactor])
      elif w == 'lateralTorqueParameters':
        p = m.lateralTorqueParameters
        series['tuning'].append([t, p.latAccelFactorFiltered, p.latAccelOffsetFiltered, p.frictionCoefficientFiltered])
      elif w == 'lateralDelay':
        series['delay'].append([t, m.lateralDelay.lateralDelay])
      elif w == 'modelV2':
        p = m.modelV2
        series['plans'].append([t, *p.acceleration.y, *p.orientation.x, *p.orientation.y])
    print(name, f.parent.name, flush=True)
  for key, rows in series.items():
    a = np.asarray(rows)
    if len(a):
      a = a[np.argsort(a[:, 0])]
      d[key + '_raw'] = a
      d[key] = np.column_stack([np.interp(d['t'], a[:, 0], a[:, j]) for j in range(1, a.shape[1])])
  np.savez_compressed(out, **d)
  print('DONE', name, {k: len(v) for k, v in series.items()}, flush=True)


if __name__ == '__main__':
  import sys
  for name in sys.argv[1:]:
    extract(name)
