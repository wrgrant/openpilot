"""Decode all requested rlogs, retaining chronology and segment boundaries; cache only in scratch."""

import os
from pathlib import Path
import numpy as np
from opendbc.can import CANParser
from openpilot.tools.lib.logreader import LogReader
from tools.odyssey_eps_report import fine_angle

SCRATCH = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')
COLS = 't seg v active pressed blinker angle rate drv motor cmd req des act roll offset lp rp road probe angle_offset sr sat'.split()


def extract(route):
  out = SCRATCH / 'plant_data' / (route.name.split('_')[1].split('--')[0] + '.npz')
  if out.exists():
    return str(out)
  rows = []
  for seg in sorted(route.glob('*--*'), key=lambda p: int(p.name.rsplit('--', 1)[1])):
    f = seg / 'rlog.zst'
    if not f.exists():
      continue
    car = CANParser('honda_odyssey_exl_2018_generated', [(s, 0) for s in ('STEERING_SENSORS', 'STEER_STATUS', 'STEER_MOTOR_TORQUE', 'VEHICLE_DYNAMICS')], 0)
    cmd = CANParser('honda_odyssey_exl_2018_generated', [('STEERING_CONTROL', 0)], 0)
    s = dict.fromkeys(COLS, 0.0)
    s.update(seg=int(seg.name.rsplit('--', 1)[1]), offset=np.nan, road=np.nan, sr=17.8)
    for m in sorted(LogReader(str(f)), key=lambda m: m.logMonoTime):
      w = m.which()
      if w == 'can':
        up = car.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.can])])
        if 342 in up:
          vl = car.vl
          s.update(
            t=m.logMonoTime / 1e9,
            angle=vl['STEERING_SENSORS']['STEER_ANGLE'],
            rate=vl['STEER_STATUS']['STEER_ANGLE_RATE'],
            drv=vl['STEER_STATUS']['STEER_TORQUE_SENSOR'],
            motor=vl['STEER_MOTOR_TORQUE']['MOTOR_TORQUE'],
            act_can=vl['VEHICLE_DYNAMICS']['LAT_ACCEL'],
          )
          rows.append([s[c] for c in COLS] + [s['act_can']])
      elif w == 'sendcan':
        cmd.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.sendcan])])
        s.update(cmd=cmd.vl['STEERING_CONTROL']['STEER_TORQUE'] / 4096, req=cmd.vl['STEERING_CONTROL']['STEER_TORQUE_REQUEST'])
      elif w == 'carState':
        c = m.carState
        s.update(v=c.vEgo, pressed=float(c.steeringPressed), blinker=float(c.leftBlinker or c.rightBlinker))
      elif w == 'carControl':
        s['active'] = float(m.carControl.latActive)
      elif w == 'controlsState':
        lc = m.controlsState.lateralControlState
        if lc.which() == 'torqueState':
          s.update(des=lc.torqueState.desiredLateralAccel, act=lc.torqueState.actualLateralAccel, sat=float(lc.torqueState.saturated))
      elif w == 'liveParameters':
        s.update(roll=m.liveParameters.roll, angle_offset=m.liveParameters.angleOffsetDeg, sr=m.liveParameters.steerRatio)
      elif w == 'lateralManeuverPlan':
        s['probe'] = m.lateralManeuverPlan.torqueOffset if m.valid else 0
      elif w == 'modelV2' and len(m.modelV2.laneLines) == 4:
        l, r = m.modelV2.laneLines[1], m.modelV2.laneLines[2]
        x = np.asarray(l.x)
        y = (np.asarray(l.y) + np.asarray(r.y)) / 2
        k = x < 50
        s.update(offset=-float(y[0]), lp=m.modelV2.laneLineProbs[1], rp=m.modelV2.laneLineProbs[2], road=-(s['v'] ** 2) * 2 * np.polyfit(x[k], y[k], 2)[0])
    print(route.name, seg.name, len(rows), flush=True)
  a = np.asarray(rows)
  d = {k: a[:, i] for i, k in enumerate(COLS + ['vsa'])}
  d['fine'] = fine_angle(d['angle'], d['rate'])
  np.savez_compressed(out, **d)
  return str(out)


if __name__ == '__main__':
  from concurrent.futures import ProcessPoolExecutor

  (SCRATCH / 'plant_data').mkdir(exist_ok=True)
  routes = sorted(Path(os.path.expanduser('~/comma-logs')).glob('04b*'))
  with ProcessPoolExecutor(max_workers=2) as pool:
    for p in pool.map(extract, routes):
      print('DONE', p, flush=True)
