"""Real rlog -> 10 Hz challenge CSV plus NNLC future-plan arrays; no synthetic plans."""
from pathlib import Path
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from tools.odyssey_sim.replay.harness import route_files

SCRATCH = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')
ROUTES = ['000001a7--a7f62f4b3f', '000001aa--ba236ee054', '000001ab--9191c0d55d', '0000018c--9a64dd12bc', '0000019b--67c64d0ac6', '0000019c--0d91088e6e']
COLS = ('t vEgo aEgo roll targetLateralAcceleration steerCommand actual active pressed angle rate angleOffset ' +
        'steerRatio stiffness curvature rawCurvature saturated').split()


def extract(route):
  dest = SCRATCH / 'closedloop_data' / f'{route}.npz'
  if dest.exists():
    return str(dest)
  dest.parent.mkdir(exist_ok=True)
  rows, plans = [], []
  latest = {}
  next_t = 0
  for file in route_files(route):
    for msg in LogReader(str(file)):
      k = msg.which()
      if k not in {'carState','carControl','controlsState','liveParameters','vehicleParameters','modelV2'}:
        continue
      latest[k] = getattr(msg,k)
      if k != 'controlsState' or msg.logMonoTime*1e-9 < next_t:
        continue
      if not all(x in latest for x in ('carState','carControl','modelV2')):
        continue
      lp = latest.get('vehicleParameters',latest.get('liveParameters'))
      if lp is None or msg.controlsState.lateralControlState.which() != 'torqueState':
        continue
      cs,cc,m = latest['carState'], latest['carControl'], latest['modelV2']
      if len(m.acceleration.y)!=33 or len(m.orientation.x)!=33:
        continue
      log = msg.controlsState.lateralControlState.torqueState
      t = msg.logMonoTime*1e-9
      next_t = t + .095
      # carControl curvature is the actual input of LaC; retain controlsState plan too.
      curv = cc.actuators.curvature
      rows.append([t,cs.vEgo,cs.aEgo,lp.roll,curv*cs.vEgo**2,log.output,log.actualLateralAccel,
                   cc.latActive,cs.steeringPressed,cs.steeringAngleDeg,cs.steeringRateDeg,
                   lp.angleOffsetDeg,lp.steerRatio,lp.stiffnessFactor,curv,msg.controlsState.desiredCurvature,log.saturated])
      plans.append([list(m.acceleration.y),list(m.orientation.x),list(m.orientation.y)])
    print(route,file.parent.name,len(rows),flush=True)
  a=np.asarray(rows)
  np.savez_compressed(dest, **{k:a[:,i] for i,k in enumerate(COLS)},plans=np.asarray(plans))
  np.savetxt(dest.with_suffix('.csv'),a,delimiter=',',header=','.join(COLS),comments='')
  return str(dest)


if __name__ == '__main__':
  from concurrent.futures import ProcessPoolExecutor
  with ProcessPoolExecutor(max_workers=2) as pool:
    print(list(pool.map(extract,ROUTES)))
